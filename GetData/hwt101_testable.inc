/* ================= [1] 帧解析层（纯逻辑） ================= */

/* 本文件被 include 到 hwt101.c 与 PC 单测两处, 故自带 hwt101.h:
 * 单测只 include 本文件, 不会先 include 头文件。hwt101.h 有 include guard,
 * 重复 include 无副作用。**本文件禁止 include 任何 HAL 头** —— 那会让 PC 单测编不过 */
#include "hwt101.h"

/* 解析状态。默认 static: 帧解析层不外泄任何可写全局，
 * 外部只能经 Hwt101_* 读接口与 Health 取值。
 * HWT101_TEST 下不 static —— PC 单测要能直接 inspect/清零它。
 * 生产固件里它仍然是文件内私有, 可见面不变。 */
#ifdef HWT101_TEST
#define HWT101_LOCAL
#else
#define HWT101_LOCAL static
#endif

HWT101_LOCAL struct
{
	uint8_t  buf[HWT101_FRAME_LEN];  /* 正在攒的帧 */
	uint8_t  cnt;                    /* 已攒字节数 0..11 */
	/* 交叉校验用的帧快照。必须在解析层内保存而不是让 Car_Sense 去比:
	 * 两路数据只有在这里才是同一时刻采到的, 跨层比会拿到"新 Wz + 旧 Yaw"。
	 * 存**已解码的工程量**(deg / dps), 不存 raw —— 差分要的是物理量的增量。
	 * yaw_deg 是**已 wrap 到 ±180** 的当前值, prev_yaw_deg 是上一帧的副本。 */
	float last_wz_dps;
	float yaw_deg;
	float prev_yaw_deg;
	uint8_t has_wz;          /* 已采纳过一个通过校验的 Wz */
	uint8_t has_prev_yaw;    /* 已收到过至少两帧 0x53(有了可差分的前值) */
} ps;

/* 诊断计数。ISR 写、主循环读。
 * ⚠ **刻意不加 volatile**: 那样 Hwt101_Health() 就无法返回 const 指针(丢掉
 * 限定符会告警), 而观测层只该**读**这些量、不该有写路径。
 * 正确性不依赖 volatile: 单字/双字的读写在 Cortex-M4 上是单条 LDR/STR, 天然原子;
 * 多字快照(xchk_ok/xchk_discard 与帧计数一起读)只用于人看的统计量, 撕裂一两次
 * 无害。真正需要严格一致性的"新鲜度"判据走 Hwt101_YawSeq()/Hwt101_WzSeq() 单读。
 * 判 volatile 加不加都不会有正确性差别, 除非将来有第二写者或用 mapped register。 */
HWT101_LOCAL hwt101_health_t health;

/* sum8 = 前 10 字节之和取低 8 位（官方 SUMCRC 定义） */
HWT101_LOCAL uint8_t Hwt101_Sum8(const uint8_t *b)
{
	uint8_t i;
	uint8_t s = 0u;

	for (i = 0u; i < (HWT101_FRAME_LEN - 1u); i++)
	{
		s = (uint8_t)(s + b[i]);
	}
	return s;
}

/* int16 由低/高字节合成。注意官方要求高字节先转有符号再左移，
 * 直接按 uint16 合成后强转在边界值上等价（C 语言实现定义行为由编译器定），
 * 这里显式按官方写法走，避免依赖实现定义 */
HWT101_LOCAL int16_t Hwt101_GetInt16(uint8_t lo, uint8_t hi)
{
	uint16_t u = (uint16_t)(((uint16_t)hi << 8) | (uint16_t)lo);

	return (int16_t)u;
}

/* 原始角速度码 → deg/s（官方: raw/32768*2000） */
HWT101_LOCAL float Hwt101_WzToDps(int16_t raw)
{
	return (float)raw / 32768.0f * HWT101_WZ_FULLSCALE;
}

/* 原始航向码 → deg，归一到 ±180（官方: raw/32768*180，值域本就是 ±180） */
HWT101_LOCAL float Hwt101_YawToDeg(int16_t raw)
{
	float yaw = (float)raw / 32768.0f * HWT101_YAW_FULLSCALE;

	/* 协议值域是 [-180,180]；越界说明取整边界，按官方 wrap 归一。
	 * 必须在写入前归一: 差分时若吃到 360 度跨度就是一次假跳变 */
	if (yaw > 180.0f)
	{
		yaw = HWT101_YAW_FULLSCALE - yaw;
	}
	else if (yaw < -180.0f)
	{
		yaw = -HWT101_YAW_FULLSCALE - yaw;
	}
	return yaw;
}

/* 本库自带的 fabs: 不引 mf_math.h —— 采样层不该依赖运动库(依赖方向反了),
 * 且这里只需要单精度取绝对值, 联调时也不值得为它拉一个头文件 */
HWT101_LOCAL float Hwt101_AbsF(float x)
{
	return (x < 0.0f) ? -x : x;
}

void Hwt101_FeedBytes(const uint8_t *buf, uint16_t len)
{
	uint16_t i;

	if (buf == 0)
	{
		return;
	}
	for (i = 0u; i < len; i++)
	{
		uint8_t b = buf[i];
		uint8_t type;

		if (ps.cnt == 0u)
		{
			/* 帧头前导: 噪声直接丢，不进状态机 */
			if (b != HWT101_HEAD)
			{
				continue;
			}
		}
		ps.buf[ps.cnt++] = b;
		if (ps.cnt < HWT101_FRAME_LEN)
		{
			continue;
		}
		/* 攒满一帧 */
		ps.cnt = 0u;
		if (Hwt101_Sum8(ps.buf) != ps.buf[HWT101_FRAME_LEN - 1u])
		{
			/* 校验失败。**不能把攒下的 11 字节全丢** —— 真实帧头可能就落在
			 * 这 11 字节的中间(丢字节/粘包时最常见), 全丢等于放弃重新对齐的
			 * 机会, 后面每帧都错位 ⇒ 永远收不到有效帧(这个 bug 实测踩过:
			 * "噪声+真帧" wz_seq=0)。正确做法是**回扫**: 重新在这 11 字节里
			 * 找下一个 0x55 作为新起点, 找不到才真丢。
			 * 回扫最多 10 字节, 仍是 O(1) 常数上界, 不破坏 ISR 的最坏时间。 */
			health.bad_sum++;
			{
				uint8_t k;
				uint8_t found = 0u;
				for (k = 1u; k < HWT101_FRAME_LEN; k++)
				{
				if (ps.buf[k] == HWT101_HEAD)
				{
					/* 把 buf[k..10] 这 11-k 个字节前移到 buf[0..],
					 * 保留的 0x55 落在新 buf[0], cnt 置成已持有的字节数。
					 * ⚠ 搬移只前移、**不清零尾部**: 清零会把已收到的真实后续
					 *   字节抹成 0x00, 下一轮凑满必然 sum8 失败, 白丢一帧
					 *   (实测: 噪声+3 帧只救回 1 帧)。
					 * ⚠ cnt 必须是"已有字节数"= 11-k。写成"还差几个"会让 cnt
					 *   越过帧长, 下一拍越界写 ps.buf, 状态彻底乱(实测 bad_sum 13)。 */
					uint8_t keep = (uint8_t)(HWT101_FRAME_LEN - k);
					uint8_t m;

					for (m = 0u; m < keep; m++)
					{
						ps.buf[m] = ps.buf[k + m];
					}
					ps.cnt = keep;
					found = 1u;
					break;
				}
				}
				if (found == 0u)
				{
					ps.cnt = 0u;   /* 整帧无 0x55: 真丢, 等下一个前导 */
				}
			}
			continue;
		}

		type = ps.buf[1];
		if (type == HWT101_TYPE_YAW)
		{
			/* [55][53][00][00][00][00][YawL][YawH][VL][VH][SUM] */
			int16_t yaw_raw = Hwt101_GetInt16(ps.buf[6], ps.buf[7]);
			float yaw = Hwt101_YawToDeg(yaw_raw);

			/* 差分前值留档: 下一帧 0x53 到来时它就是"上一帧" */
			ps.prev_yaw_deg = ps.yaw_deg;
			ps.yaw_deg = yaw;
			/* 有"前值"当且仅当本帧之前已收过至少一帧 0x53:
			 * health.yaw_seq 在下面 ++, 所以此刻读到的就是已收帧数 */
			ps.has_prev_yaw = (uint8_t)((health.yaw_seq != 0u) ? 1u : 0u);
			health.yaw_seq++;
		}
		else if (type == HWT101_TYPE_WZ)
		{
			/* [55][52][00][00][RWzL][RWzH][WzL][WzH][00][00][SUM]
			 * 取 6/7 字节的 Wz（**已校准**），不是 4/5 的原始 RWz */
			int16_t wz_raw = Hwt101_GetInt16(ps.buf[6], ps.buf[7]);
			float wz = Hwt101_WzToDps(wz_raw);

#if HWT101_XCHK_EN
			/* 交叉校验: 0x52 直给 与 0x53 差分 是同一个 ψ̇ 的两次独立测量,
			 * 误差机制几乎不相关 ⇒ 差值过阈即判该 Wz 不可信。
			 * 差分路是 0x53 的导数, 符号定义上不会错 ⇒ 顺带当 0x52 的符号裁判。
			 * 量程闸(MF_WZ_RATE_MEAS_MAX_DPS)挡大错(航向阶跃/假同步),
			 * 本交叉校验挡小错(错字节伪装成合理小值) —— 互补, 不重复丢帧。
			 *
			 * 判据: |Wz − Δψ/Δt| >  tol。Δψ 取**最近两帧 0x53** 之差, Δt 取
			 * 两帧的模块输出间隔(按 RRATE 折算, 200Hz ⇒ 5ms)。两路是模块同刻
			 * 发出的成对帧, 所以这个比值与 Wz 应当一致。
			 * ⚠ 幅值低于 HWT101_XCHK_MIN_DPS 时不判: 静止/过零时 Δψ 的量化噪声
			 *   与 Wz 的零偏同量级, 比值无意义, 判了全是误报。 */
			if ((ps.has_prev_yaw != 0u) && (ps.has_wz != 0u)
			    && (Hwt101_AbsF(wz) > HWT101_XCHK_MIN_DPS)
			    && (Hwt101_AbsF(ps.last_wz_dps) > HWT101_XCHK_MIN_DPS))
			{
				float dyaw = ps.yaw_deg - ps.prev_yaw_deg;   /* 已 wrap 到 ±180 */
				float yaw_rate = dyaw / HWT101_FRAME_DT_S;
				float err = Hwt101_AbsF(wz - yaw_rate);

				if (err > HWT101_XCHK_TOL_DPS)
				{
					/* 两路对不上: 记丢弃, 且**不采纳**本帧 Wz。
					 * 采纳会让一个错字节变成持续虚假偏置(比一次跳变更难发现),
					 * 这正是交叉校验要治的形态。航向侧照常更新(它是裁判基准)。 */
					health.xchk_discard++;
					ps.has_wz = 0u;   /* 清有效位 ⇒ 下一帧重新积累再判 */
				}
				else
				{
					health.xchk_ok++;
				}
			}
			else
			{
				health.xchk_ok++;   /* 不判(静止/首帧), 不是丢弃 */
			}
#endif
			health.wz_seq++;
			ps.last_wz_dps = wz;
			ps.has_wz = 1u;
		}
		else if (type == HWT101_TYPE_REG)
		{
			/* 回读应答 [55 5F d0 d1 d2 d3 d4 d5 d6 d7 SUM] (11B)。
			 * VERSION(0x2E) 占 d0..d3, 按官方:
			 *   VERSION = (VHH<<24)|(VHL<<16)|(VLH<<8)|VLL
			 * 示例 55 5F 01 41 EF 89 → 0x89EF4101 = 10173.1.1
			 * 本库只留原始 32 位, 拆位显示交给观测层(格式会变, 别在驱动里定型) */
			uint32_t ver = ((uint32_t)ps.buf[2])
			             | ((uint32_t)ps.buf[3] << 8)
			             | ((uint32_t)ps.buf[4] << 16)
			             | ((uint32_t)ps.buf[5] << 24);
			health.version = (uint16_t)ver;
			health.has_version = 1u;
		}
		else
		{
			/* 未知 TYPE: 校验过了但不是本库关心的帧, 静默丢弃 */
		}
	}
}

float Hwt101_GetYawDeg(void)
{
	return ps.yaw_deg;
}

float Hwt101_GetWzDps(void)
{
	/* has_wz=0 = 还没收到帧, 或上一帧被交叉校验判为不可信 ⇒ 此时**不返回**。
	 * 返回一个"最后已知值"会让调用方以为反馈是新鲜的 —— 新鲜度必须由
	 * Hwt101_WzSeq() 的帧计数增量判, 不能靠这个值本身 */
	return ps.has_wz ? ps.last_wz_dps : 0.0f;
}

uint32_t Hwt101_YawSeq(void)
{
	return health.yaw_seq;
}

uint32_t Hwt101_WzSeq(void)
{
	return health.wz_seq;
}

const hwt101_health_t *Hwt101_Health(void)
{
	return &health;
}

/* ---- 寄存器层: 写帧编解码（纯函数） ---- */

/* 组写帧: [FF AA ADDR DATAL DATAH] */
HWT101_LOCAL void Hwt101_MakeWrite(uint8_t reg, uint16_t val, uint8_t *out)
{
	out[0] = HWT101_WRITE_HEAD0;
	out[1] = HWT101_WRITE_HEAD1;
	out[2] = reg;
	out[3] = (uint8_t)(val & 0xFFu);
	out[4] = (uint8_t)((val >> 8) & 0xFFu);
}

