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
	uint8_t reg_pending;     /* 等待解释的 0x5F 应答对应哪个寄存器; 0=无 */
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
	/* raw 是 int16_t, raw/32768*180 的值域是 [-180.0, +179.994] ⇒ 天然就在 ±180 内,
	 * **不需要**再 clamp(早期版本这里有一段越界归一, 属死代码, 已删)。
	 * 真正需要归一的是**两个航向相减**: 见 Hwt101_WrapDeg。 */
	return (float)raw / 32768.0f * HWT101_YAW_FULLSCALE;
}

/* 角度差归一到 ±180。**这是跨 ±180 穿越时唯一正确的做法**。
 * 两个都在 [-180,180] 的航向直接相减, 可达范围是 ±360: 车每转过 180° 就必然穿越一次
 * (+179.9 → -179.9 给出 dyaw=-359.8, 除以 5ms 得 -71960 dps, 任何容差都会被击穿)。
 * 官方未定义角度差的表示法, 这里按最短弧取, 与 mf_math.h 的 mf_wrap_deg 同义;
 * 刻意自备而不 include mf_math.h —— 采样层不该反向依赖运动库。 */
HWT101_LOCAL float Hwt101_WrapDeg(float d)
{
	while (d > 180.0f)
	{
		d -= 360.0f;
	}
	while (d < -180.0f)
	{
		d += 360.0f;
	}
	return d;
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
			 * 这 11 字节的中间(丢字节/粘包时最常见), 全丢等于放弃重新对齐的机会,
			 * 后面每帧都错位 ⇒ 永远收不到有效帧(实测"噪声+真帧" wz_seq=0)。
			 * 正确做法是**回扫**: 重新在这 11 字节里找下一个 0x55 作为新起点。 */
			health.bad_sum++;
			{
				uint8_t k;
				uint8_t found = 0u;
				for (k = 1u; k < HWT101_FRAME_LEN; k++)
				{
				if (ps.buf[k] == HWT101_HEAD)
				{
					/* 把 buf[k..10] 前移到 buf[0..], cnt 置成"已持有字节数"= 11-k。
					 * ⚠ 搬移只前移、**不清零尾部**: 清零会把已收到的真实后续字节抹成
					 *   0x00, 下一轮凑满必然 sum8 失败, 白丢一帧(实测只救回 1/3)。
					 * ⚠ cnt 写成"还差几个"会让它越过帧长, 下一拍越界写 ps.buf(实测 bad_sum 13)。 */
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
				 * 差分路符号定义上不会错 ⇒ 顺带当 0x52 的符号裁判。
				 * 量程闸(MF_WZ_RATE_MEAS_MAX_DPS)挡大错(航向阶跃/假同步),
				 * 本交叉校验挡小错(错字节伪装成合理小值) —— 互补, 不重复丢帧。
				 * Δψ取最近两帧 0x53 之差并 wrap 到 ±180; 不 wrap 则每转过 180° 必然
				 * 误丢一帧(现象"车每 8 秒丢一次反馈"), 曾是 P0 级缺陷(评审 P0-2)。 */
			if ((ps.has_prev_yaw != 0u) && (ps.has_wz != 0u)
			    && (Hwt101_AbsF(wz) > HWT101_XCHK_MIN_DPS)
			    && (Hwt101_AbsF(ps.last_wz_dps) > HWT101_XCHK_MIN_DPS))
			{
				float dyaw = Hwt101_WrapDeg(ps.yaw_deg - ps.prev_yaw_deg);
				float yaw_rate = dyaw / HWT101_FRAME_DT_S;
				float err = Hwt101_AbsF(wz - yaw_rate);

				if (err > HWT101_XCHK_TOL_DPS)
				{
					/* 两路对不上: 记丢弃, 且**真正不采纳**本帧 Wz。
					 * ⚠ 这里必须 continue —— 早先版本只清 has_wz, 而紧随其后的
					 *   "wz_seq++ / last_wz_dps=wz / has_wz=1" 无条件执行、把它盖回去,
					 *   校验对输出零作用(评审 P0-1)。帧计数也必须一起跳:
					 *   Car_Sense 靠 WzSeq() 变化判"新帧"。航向侧照常更新。 */
					health.xchk_discard++;
					continue;
				}
				health.xchk_ok++;
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
			 * ⚠ 早期版本把结果截成 uint16 存(0x4101), 那个数字与官方版本号语义
			 *   无关、无法用于判版本, 现存完整 32 位(评审 P1-6)。
			 * ⚠ 0x5F 是**所有**寄存器回读的通用应答头。0x2E 恰�� d0..d3, 所以本分支
			 *   只有在"当前回读的就是 0x2E"时才解释; 否则忽略(否则将来加 RRATE
			 *   回读, 版本号会被 RRATE 数据静默覆盖)。 */
			if (ps.reg_pending == (uint8_t)HWT101_REG_VERSION)
			{
				uint32_t ver = ((uint32_t)ps.buf[2])
				             | ((uint32_t)ps.buf[3] << 8)
				             | ((uint32_t)ps.buf[4] << 16)
				             | ((uint32_t)ps.buf[5] << 24);
				health.version = ver;
				health.has_version = 1u;
				ps.reg_pending = 0u;   /* 消费掉, 避免重复应答被当成新的一次 */
			}
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

void Hwt101_ExpectReg(uint8_t reg)
{
	ps.reg_pending = reg;
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

