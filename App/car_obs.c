#include "car_obs.h"
#include "car_link.h"
#include "main.h"
#include "Emm_V5.h"
#include "mf_path.h"
#include "mf_math.h"          /* mf_absf: 弧几何观测行(RAD:) */
#include "motion_app.h"
#include "nav_task.h"
#include "nav_turntable.h"
#include "nav_tracking.h"
#include "hwt101.h"           /* 叶子头(只含 stdint.h): 取 hwt101_health_t 诊断字段 */
#include <stdio.h>

/* RAD: 行"算半径"的最小角速度(dps): 低于它 v/ω 只是"接近静止的随机比值", 打了反而误导 */
#define CAR_OBS_RAD_MIN_DPS   20.0f

/* ---- 段内瞬时跟踪('t' 命令) -------------------------------------------------
 * 为什么需要它: ARC:/LN: 两行是**出口统计量**(只给段尾的 EY/YE), 段内的振荡/卡顿在它们身上
 * 完全看不见。判"转向一卡一卡"卡在哪一环只能看时序, 一行同时给出三层:
 *   SP/W/O/I  = 外环设定 / IMU 实测(航向差分) / 内环输出 / 积分贡献  -> 速率内环
 *   CL/SL/CR/SR = 左右轮速环的指令/实测                  -> 轮速环与履带接触
 *   RS        = 段内剩余里程(mm)                          -> 定位在段的哪个位置
 *   GZ        = 0x52 陀螺帧直接给出的角速度(dps)           -> **判 W 的真伪**
 * 读法: ① GZ 平滑而 W 以 2 个采样为周期反相摆动 = W 的摆动是采样伪影, 不是车体真在抖
 *         (机理: 250Hz 航向帧 ÷ 100Hz 控制拍 = 每拍 2/3 帧交替, 见 mf_pid.h);
 *      ② GZ 与 W 同幅同相 = 振荡是真的, 再往下列找;
 *      ③ W≈SP 而 SL/SR 一阵一阵掉到 0 = 轮速环 stick-slip(机械/摩擦), 控制环无罪;
 *      ④ SP 自己就在摆 = 外环航向环欠阻尼(kp/kd 未按内环重校, 见 docs/坑-编码器与里程.md §5)
 * 触发: 't' 武装, 首次出现 |SP| >= CAR_OBS_TRACE_TRIG_DPS(只有弧/转向段才会到)即开窗 4s */
#define CAR_OBS_TRACE_MS          4000u
#define CAR_OBS_TRACE_PERIOD_MS   50u     /* 20Hz: 能分辨 5Hz 级的振荡 */
#define CAR_OBS_TRACE_TRIG_DPS    30.0f

static uint8_t  tr_state;                  /* 0=空闲 1=武装(等弧/转向段) 2=采集 */
static uint32_t tr_end_ms;
static uint32_t tr_last_ms;

void Car_Obs_TraceArm(void)
{
	tr_state = 1u;
	printf("TRC:ARM (等 |SP|>=%d dps)\r\n", (int)CAR_OBS_TRACE_TRIG_DPS);
}

void Car_Obs_TraceTask(void)
{
	mf_ctx_t *m = mf_app();
	mf_rate_info_t ri;
	uint32_t now = HAL_GetTick();

	if (tr_state == 0u)                                     /* 空闲: 只剩一次判断 */
	{
		return;
	}
	if ((uint32_t)(now - tr_last_ms) < CAR_OBS_TRACE_PERIOD_MS)
	{
		return;                                             /* 武装态与采集态统一 20Hz 节流 */
	}
	tr_last_ms = now;

	__disable_irq();
	mf_rate_get_info(m, &ri);   /* 快照必须在**这里**取: 首版只在武装分支取, 采集分支打的是未初始化
	                             * 的局部 —— 现场表现为 SP/W/O/I 恒 0、偶发 INT_MAX(2026-09-30 实拍) */
	__enable_irq();

	if (tr_state == 1u)                                     /* 武装: 等弧/转向段的 |SP| */
	{
		if (mf_absf(ri.sp_dps) < CAR_OBS_TRACE_TRIG_DPS)
		{
			return;
		}
		tr_state  = 2u;
		tr_end_ms = now + CAR_OBS_TRACE_MS;
		printf("TRC:START %ums\r\n", (unsigned)CAR_OBS_TRACE_MS);
	}

	{
		float cl, sl, cr, sr, rs, gz;

		__disable_irq();
		cl = m->wheel[0].cmd_mps;
		sl = m->wheel[0].speed_mps;
		cr = m->wheel[1].cmd_mps;
		sr = m->wheel[1].speed_mps;
		rs = mf_path_remain_m(&m->path);
		gz = Car_GetGyroWzDps();
		__enable_irq();
		printf("TRC:SP=%d W=%d GZ=%d O=%d I=%d CL=%d SL=%d CR=%d SR=%d RS=%d\r\n",
		       (int)ri.sp_dps, (int)ri.meas_dps, (int)gz, (int)ri.out_dps, (int)ri.i_term_dps,
		       (int)(cl * 1000.0f), (int)(sl * 1000.0f),
		       (int)(cr * 1000.0f), (int)(sr * 1000.0f), (int)(rs * 1000.0f));
	}
	if ((uint32_t)(tr_end_ms - now) >= 0x80000000u)          /* 模块化比较: now 已过 tr_end */
	{
		tr_state = 0u;
		printf("TRC:END\r\n");
	}
}

/* "这一拍到底谁在写 vx" —— 段表出没出速度 / 跟瞄有没有压零。
 * 生产者 TIM4 ISR(Car_Obs_SetVx), 消费者主循环 Car_Debug_Stat_Print。
 * 现场判"车为什么不动": VXP>0 而 VXF=0 = 跟瞄在压零(看 TRK 字母); VXP=0 = 段表没出速度
 * (看 NAV/RS); VXP/VXF 都>0 而 WHEEL 的 SL/SR≈0 = 指令到了电机侧没跟(驱动/使能/编码器) */
static volatile int16_t dbg_vx_path_mm;    /* mf_tick 后(段表含咬靶限速)的 vx */
static volatile int16_t dbg_vx_final_mm;   /* 跟瞄叠加后的 vx(真正进轮速环的值) */
static volatile uint8_t dbg_ovl;           /* 跟瞄本拍是否改写了 cmd(1=介入) */

void Car_Obs_SetVx(int16_t path_mm, int16_t final_mm, uint8_t overlay)
{
	dbg_vx_path_mm  = path_mm;
	dbg_vx_final_mm = final_mm;
	dbg_ovl         = overlay;
}

/* USART2 调试口状态块/打印('?'/p 周期): main 上下文, 原 9 参 Car_Debug_Snapshot
 * 一并内聚于此(printf 本身仍在 main 上下文)。仅调试显示, 不进控制路径。
 * 【已知偏离 D-1, 已裁决记账不修】本函数分四次取样(2 个 GetInfo + nav 2 次裸读 +
 * 下方 wheel 窗口), 不像基线那样共用一个关中断窗口 → 各字段可能跨 TIM4 拍,
 * 偶发"新 NAV 配旧 SEG"式自相矛盾读数。量级与修复方案: 施工说明 §8 D-1 */
void Car_Debug_Stat_Print(void)
{
	extern volatile uint32_t Dbg2_TxFail_Cnt;   /* stm32f4xx_it.c: 调试口链路健康计数 */
	extern volatile uint32_t Dbg2_RxErr_Cnt;
	/* HWT101 全部诊断字段已收进驱动库的 hwt101_health_t(GetData/hwt101.c),
	 * 经 Hwt101_Health() 取。原先在这里 extern 的 Hwt101_RateHz/RateBoot/YawSeq
	 * 三个散量已删除 —— 它们现在是 health 的成员, 少三个跨层 extern。 */
	const hwt101_health_t *hw = Hwt101_Health();
	const mf_status_t *st;
	const mf_pose_t *pose;
	mf_ctx_t *m;

	uint8_t  nav_state, nav_id, seg_done, trk_state, trk_found, tt_armed, tt_code;
	uint8_t  trk_reason;
	int8_t   trk_zx;
	uint16_t seg_idx, trk_frame_age, trk_hold, tt_ms, tt_settle;
	uint32_t trk_remain_mm;
	uint32_t tt_tick;
	int16_t  err_f10k;
	int16_t  vxp_mm, vxf_mm;
	uint8_t  ovl_flag;
	float    x_m, y_m, yaw_deg, wl_cmd, wl_spd, wr_cmd, wr_spd, tt_deg;
	float    reff_m;
	uint8_t  reff_valid;
	uint8_t  arc_src;
	Tt_Info_t tti;
	Trk_Info_t tri;
	mf_rate_info_t ri;
	mf_path_obs_t obs;
	float    frame_base, hdg_route;
	char     hdg_kind;

	/* 快照四次取样的前三段(见函数头 D-1): ①② 的 GetInfo 自带临界区, 塞进下方关中断块
	 * 会被它内部的 __enable_irq() 提前放行 TIM4; ③ 两次裸读单字节原子但无窗口保护。
	 * 修复(D-1): 改调 Turntable_GetInfoRaw/Tracking_GetInfoRaw, 与下方窗口合为一个 */
	Turntable_GetInfo(&tti);
	Tracking_GetInfo(&tri);
	nav_state = Nav_GetState();
	nav_id    = Nav_GetId();

	__disable_irq();
	m = mf_app();
	st = mf_get_status(m);
	pose = mf_get_pose(m);
	seg_idx   = st->seg_index;
	seg_done  = st->seg_done;
	trk_state = tri.state;
	trk_found = tri.found;
	err_f10k  = tri.err_f10k;
	trk_frame_age = tri.frame_age_ms;
	trk_hold  = tri.hold_ms;
	trk_zx    = tri.zero_sign;
	trk_reason = tri.reason;
	trk_remain_mm = (uint32_t)(mf_path_remain_m(&m->path) * 1000.0f);
	x_m    = pose->x_m;
	y_m    = pose->y_m;
	yaw_deg = pose->yaw_deg;
	wl_cmd = m->wheel[0].cmd_mps;   /* 框架轮序 0=左 1=右(CarSense_Output 同序) */
	wl_spd = m->wheel[0].speed_mps;
	wr_cmd = m->wheel[1].cmd_mps;
	wr_spd = m->wheel[1].speed_mps;
	vxp_mm = dbg_vx_path_mm;
	vxf_mm = dbg_vx_final_mm;
	ovl_flag = dbg_ovl;
	tt_deg = tti.angle_deg;
	tt_armed = tti.armed;
	tt_ms     = tti.ms;        /* 台架标定用: 本次 DONE 已计时 / 目标 settle */
	tt_settle = tti.settle_ms;
	tt_code   = tti.code;
	tt_tick   = tti.last_tick; /* 台架标定用: 最近一次下发的脉冲数(解位置当量) */
	mf_rate_get_info(m, &ri);  /* 速率内环快照: 与本行以上同处一个关中断窗口 */
	reff_m = mf_get_reff(m, &reff_valid);   /* 实测弧半径: 同一窗口内取, 与 pose 同拍 */
	arc_src = mf_get_odom_arc_src(m);       /* 里程计形状来源: "跑在哪个分支"必须看得见 */
	mf_path_get_obs(&m->path, &obs);        /* 按拍航向观测(HDG 行): 与 pose 同窗口 */
	frame_base = mf_path_get_frame_base(&m->path);
	hdg_route = mf_wrap_deg(yaw_deg - frame_base);   /* 路线系航向: 置零那一刻 = 0 */
	hdg_kind = (obs.kind <= 4u) ? "ALTRG"[obs.kind] : '?';
	__enable_irq();

	printf("CHASSIS:NAV=%c ID=%02u SEG=%u/%u TRK=%c E=%d F=%u AGE=%u HOLD=%u\r\n",
	       "IRD"[nav_state & 0x03u], (unsigned)nav_id, (unsigned)seg_idx, (unsigned)seg_done,
	       Tracking_StateChar((TrackingState_t)trk_state), err_f10k, (unsigned)trk_found,
	       (unsigned)trk_frame_age, (unsigned)trk_hold);
	/* ST 四态 O/A/S/F; ZX=已确认误差符号(+1/-1/0); RS=段表剩余里程(mm); FR=FAULT reason */
	printf("TRK2:ST=%c ZX=%d HOLD=%u RS=%u FR=%u E=%d F=%u AGE=%u\r\n",
	       Tracking_StateChar((TrackingState_t)trk_state), (int)trk_zx,
	       (unsigned)trk_hold, (unsigned)trk_remain_mm, (unsigned)trk_reason,
	       (int)err_f10k, (unsigned)trk_found, (unsigned)trk_frame_age);
	/* VXP=段表(含咬靶限速)当拍 vx mm/s; VXF=跟瞄叠加后 vx; OVL=跟瞄本拍是否改写了 cmd。
	 * 判"车为什么不动": VXP>0 && VXF=0 = 跟瞄压零(看 TRK); VXP=0 = 段表没出速度(看 NAV/RS);
	 * VXP/VXF>0 但 SL/SR≈0 = 指令到电机侧没跟(驱动/使能/编码器, 与跟瞄无关) */
	printf("WHEEL:CL=%d SL=%d CR=%d SR=%d VXP=%d VXF=%d OVL=%u\r\n",
	       (int)(wl_cmd * 1000.0f), (int)(wl_spd * 1000.0f),
	       (int)(wr_cmd * 1000.0f), (int)(wr_spd * 1000.0f),
	       (int)vxp_mm, (int)vxf_mm, (unsigned)ovl_flag);
	printf("POSE:%d %d %d\r\n",
	       (int)(x_m * 1000.0f), (int)(y_m * 1000.0f), (int)(yaw_deg * 10.0f));
	/* 航向观测(路线系, 由 mf_path 的按拍观测给出): K=段类型 A弧/L直线/T绝对转/R相对转/G到点;
	 * TGT=本段航向目标(路线系); YAW=本拍路线系航向(= POSE 的 yaw − BASE); ERR=环内误差;
	 * EY=横向偏差(直线: 本段名义线左侧正 / ARC: 弧外正); BASE=本表重锚基点(= 置零那拍读数)。
	 * 单位 0.1°, EY 是 mm。判读(长直线):
	 *   ① ERR 稳在 0 而实物仍偏 = 基准/基点的问题(表里的角解释错了), 不是控制环;
	 *   ② EY 沿行程基本不变 = 整段平行挪出去(进段偏移/弧半径); EY 线性增长 = 越走越斜 */
	printf("HDG:ID=%02u SEG=%u K=%c TGT=%d YAW=%d ERR=%d EY=%d BASE=%d\r\n",
	       (unsigned)nav_id, (unsigned)seg_idx, hdg_kind,
	       (int)(obs.target_deg * 10.0f), (int)(hdg_route * 10.0f),
	       (int)(obs.yaw_err_deg * 10.0f), (int)(obs.ey_m * 1000.0f),
	       (int)(frame_base * 10.0f));
	printf("WTBL:%d ARM=%u CODE=%u SET=%u/%u TICK=%u\r\n",   /* SET=已计时/本次目标 ms; TICK=最近下发脉冲数(标定当量) */
	       (int)(tt_deg * 10.0f), (unsigned)tt_armed,
	       (unsigned)tt_code, (unsigned)tt_ms, (unsigned)tt_settle,
	       (unsigned)tt_tick);
	printf("EMM:TF=%u RX=%u OK=%u\r\n",   /* 转台台架: UART6 DMA 失败/RX 应答字节/0x11 应答帧 */
	       (unsigned)Emm_tx_fail_cnt,
	       (unsigned)Emm_rx_cnt, (unsigned)Emm_reply_ok_cnt);
	/* 调试口链路健康: MS=MCU uptime(空档后跳过整段=丢在 PC/线缆侧, 归小=MCU 复位过),
	 * TF=发送失败次数, RXE=接收错误次数 */
	printf("LINK:MS=%u TF=%u RXE=%u\r\n",
	       (unsigned)HAL_GetTick(),
	       (unsigned)Dbg2_TxFail_Cnt, (unsigned)Dbg2_RxErr_Cnt);
	/* HWT101 链路健康(诊断字段现由驱动库 hwt101_health_t 统一持有):
	 * RRATE=当前滚动 0x53 帧率 / BOOT=上电那次采样 / SEQ=0x53 帧计数(不涨即断流)
	 * BAD=sum8 校验失败帧数 / XOK/XBAD=交叉校验通过/丢弃 / TXF=配置帧发送失败
	 * VP=版本号产品段(17bit) / VH=硬件段(6bit) / VS=软件段(8bit)
	 * 判"速率配置生效没有"看 RRATE: ≈200 已配到 200Hz / ≈10 仍出厂默认 / 0 模块没通。
	 * TXF 非 0 = 有配置帧没发出去(写寄存器单向无 ACK, 这是唯一线索)。
	 * RRATE 是真实帧率, 但旧固件日志是突发下界 ⇒ 跨版本比较前先对齐口径(docs §8.11)。 */
	printf("HWT:RRATE=%u BOOT=%u SEQ=%u BAD=%u XOK=%u XBAD=%u TXF=%u VER=%u.%u.%u\r\n",
	       (unsigned)hw->rate_hz, (unsigned)hw->rate_boot, (unsigned)hw->yaw_seq,
	       (unsigned)hw->bad_sum, (unsigned)hw->xchk_ok, (unsigned)hw->xchk_discard,
	       (unsigned)hw->tx_fail,
	       (unsigned)((hw->version >> 15) & 0x1FFFFu),   /* 产品版本 17bit */
	       (unsigned)((hw->version >> 9) & 0x3Fu),       /* 硬件版本 6bit */
	       (unsigned)(hw->version & 0xFFu));             /* 软件版本 8bit */
	/* 交叉校验: 0x52 直给 与 0x53 差分 是同一个 ψ̇ 的两次独立测量, 差值超阈即丢该 Wz。
	 * XBAD 常年 0 = 链路干净(这机制只是保险); 持续增长 = 真有错帧, 且形态是
	 * "错字节伪装成合理小值" —— 那种错 sum8 查不出来, 只能靠这层冗余 */
	/* 断流硬告警: 最近一个窗口一帧 0x53 都没收到 ⇒ 航向冻结(弧永远走不完)、速率内环退化成
	 * "开环基线+冻结积分"。最常见原因是模块波特率不是 115200 —— 官方 BAUD(0x04) 出厂默认
	 * 0x0002=9600, 而本工程 USART3 固定 115200 且固件不写也不验(§7.11), 换模块/恢复出厂即中招 */
	if (hw->rate_hz == 0u)
	{
		printf("HWT:WARN NOFRAME BOOT=%u SEQ=%u -> check BAUD=115200(reg 0x04)/wiring\r\n",
		       (unsigned)hw->rate_boot, (unsigned)hw->yaw_seq);
	}
	/* 角速度内环健康度: SP=外环设定(它"想要"的 deg/s) W=IMU 实测 O=实际下发(含开环基线
	 * +PI), B=开环基线贡献(=SP×基线增益), I=积分贡献(ki·∫e; O−B−I 就是 P 项)。
	 * 基线与积分是**同一份补偿的两半**(基线越准 I 越接近 0), 不是两套补偿。
	 * OK/ST/SN = 闭环/反馈停更/反馈反号 的累计拍数。
	 * 读法: ① ST 或 SN 在涨 = 反馈链断了, 补偿只剩开环基线 ⇒ 先查 HWT: SEQ 涨不涨;
	 *      ② OK 在涨且 W≈SP = 内环真在闭环(半径由它保证, 可把基线往 1.0 收);
	 *      ③ OK 在涨但 W 只有 SP 的 0.6 倍 = 环在、对象跟不上(轮速环饱和/堵转/基线偏小),
	 *         看 O 是否已顶到 150 上限。W/O 就是对象增益 k, 直接用来复标 MF_ARC_SLIP_GAIN */
	printf("RATE:SP=%d W=%d O=%d B=%d I=%d OK=%u ST=%u SN=%u\r\n",
	       (int)ri.sp_dps, (int)ri.meas_dps, (int)ri.out_dps,
	       (int)ri.base_dps, (int)ri.i_term_dps,
	       (unsigned)ri.n_ok, (unsigned)ri.n_stale, (unsigned)ri.n_sign);
	/* 弧的实时几何(纯观测): 三条独立尺子量同一条弧 —— RSP=设定半径 v/|wz_sp|(段表想要的);
	 * RM=实测半径 v/|ω_meas|(IMU 说的); REF=库内同间隔实测半径(轮位移÷IMU 转角, mf_odom.c)。
	 * RV=REF 是否新鲜(不转弯就没有新估计); KH=对象增益 ω_meas/ω_out(%);
	 * YF=**帧数/拍数** 原始比值(格式 dq/dtk, 如 200/200 = 1:1), 不是百分比。
	 * ⚠ 2026-10 拍周期改200Hz 后帧拍比=1:1 ⇒ 健康的 YF 恒为 "N/N"; 曾用百分比
	 * (dq*100/dtk) 时健康值恰好 100, **与"YF<100 = 零阶保持/退化"的判据边界重合**,
	 * 丢一帧就变成 99 而被读成故障, 且与历史日志(200)不可比。改打印原始比值后
	 * 丢帧表现为 "199/200", 一眼可辨。dq/dtk 的比值才是判据(帧率÷拍率), % 是它的有损压缩。
	 * YF 是 1s 窗口量, 同一行里其余字段是当拍量(可差 1s)。OSRC=里程计形状来源。
	 * ⚠ 判半径只认 RM/REF/R=(或卷尺): ARC:EY 与 odom 形状同源, 结构性看不见打滑(§8)。
	 * 速率闸取 20dps(弧工作点的一半): 再小就是"接近静止的随机比值", 打出来只会误导 */
	{
		static uint32_t yf_seq, yf_tick, yf_ms, yf_dq, yf_dtk;
		uint32_t tick_now = ri.n_ok + ri.n_stale + ri.n_sign;
		uint32_t now = HAL_GetTick();
		float v = (mf_absf(wl_spd) + mf_absf(wr_spd)) * 0.5f;
		int rsp = (mf_absf(ri.sp_dps) >= CAR_OBS_RAD_MIN_DPS)
		          ? (int)(v / (mf_absf(ri.sp_dps) * (MF_PI_F / 180.0f)) * 1000.0f) : 0;
		int rmeas = (mf_absf(ri.meas_dps) >= CAR_OBS_RAD_MIN_DPS)
		            ? (int)(v / (mf_absf(ri.meas_dps) * (MF_PI_F / 180.0f)) * 1000.0f) : 0;
		int kh = (mf_absf(ri.out_dps) >= CAR_OBS_RAD_MIN_DPS)
		         ? (int)(ri.meas_dps / ri.out_dps * 100.0f) : 0;

		if ((uint32_t)(now - yf_ms) >= 1000u)   /* 1s 窗口; 首查给出上电以来均值 */
		{
			/* 存原始 dq/dtk 而非百分比: 百分比会把 1:1 的健康值压到 100, 正好落在
			 * "丢帧/<100" 的判据边界上, 丢一帧读成 99=误判故障。原始比值里丢帧
			 * 表现为 199/200, 一眼可辨(见上面注释) */
			yf_dq   = hw->yaw_seq - yf_seq;
			yf_dtk  = tick_now - yf_tick;
			yf_seq  = hw->yaw_seq;
			yf_tick = tick_now;
			yf_ms   = now;
		}
		printf("RAD:RSP=%d RM=%d REF=%d RV=%u KH=%d YF=%lu/%lu OSRC=%u TR=%u\r\n",
		       rsp, rmeas, (int)(reff_m * 1000.0f), (unsigned)reff_valid, kh,
		       (unsigned long)yf_dq, (unsigned long)yf_dtk, (unsigned)arc_src,
		       (unsigned)(MF_ARC_WZ_TRIM * 1000.0f));   /* TR=弧径向 trim ×1000 */
	}
}

/* 跟瞄状态迁移打点(主循环): Tracking_Goto 在 ISR 只登记, 这里打一行。
 * 同 D-1: 迁移记录/跟瞄快照/剩余里程分三次取样(基线共用一个窗口), 可能跨拍 */
void Car_TrkChange_Log(void)
{
	uint8_t from, to;
	uint32_t rs_mm;
	Trk_Info_t tri;

	if (Tracking_TakeChangeLog(&from, &to) == 0u)
	{
		return;
	}
	Tracking_GetInfo(&tri);
	__disable_irq();
	rs_mm = (uint32_t)(mf_path_remain_m(&mf_app()->path) * 1000.0f);
	__enable_irq();

	printf("TRK:%c>%c E=%d F=%u AGE=%u RS=%u\r\n",
	       Tracking_StateChar((TrackingState_t)from), Tracking_StateChar((TrackingState_t)to),
	       (int)tri.err_f10k, (unsigned)tri.found,
	       (unsigned)tri.frame_age_ms, (unsigned)rs_mm);
}

/* ARMED/STOPPED 期高频打点(主循环 10Hz, 规格 §10.6): 定性"误差卡在哪一环" ——
 * 光看 TRK: 迁移行只能知道换了态, 不知道车在不在动、轮子跟没跟、判据卡在哪。
 * 读法: E 在收敛 = 车真在逼近靶心; E 不动而 SL/SR>0 = 车在走但视觉没更新;
 * CL/CR>0 而 SL/SR≈0 = 指令到电机侧没跟(轮速环/驱动/编码器); BRK=1 = 已触发制动。
 * 同 D-1: tri 与 wheel 分两次取样(基线共用一个窗口), 可能跨拍 → 上述读法会被误导 */
void Car_TrkDetail_Log(void)
{
	static uint32_t trace_tick = 0u;
	uint32_t now = HAL_GetTick();
	int32_t  cl, sl, cr, sr;
	Trk_Info_t tri;

	Tracking_GetInfo(&tri);
	if ((tri.state != (uint8_t)TRACKING_ARMED) && (tri.state != (uint8_t)TRACKING_STOPPED))
	{
		return;   /* 只在跟瞄介入期打点 */
	}
	if ((uint32_t)(now - trace_tick) < 100u)
	{
		return;
	}
	trace_tick = now;

	__disable_irq();
	cl = (int32_t)(mf_app()->wheel[0].cmd_mps * 1000.0f);
	sl = (int32_t)(mf_app()->wheel[0].speed_mps * 1000.0f);
	cr = (int32_t)(mf_app()->wheel[1].cmd_mps * 1000.0f);
	sr = (int32_t)(mf_app()->wheel[1].speed_mps * 1000.0f);
	__enable_irq();

	printf("TRKD:ST=%c E=%d ZX=%d HOLD=%u BRK=%u CL=%d SL=%d CR=%d SR=%d\r\n",
	       Tracking_StateChar((TrackingState_t)tri.state), (int)tri.err_f10k, (int)tri.zero_sign,
	       (unsigned)tri.hold_ms, (unsigned)tri.braking,
	       (int)cl, (int)sl, (int)cr, (int)sr);
}

/* 段切换观测(替代原 mf_path.c 库层内的 printf): 库层打印 + 经 mf_tick 在
 * TIM4 ISR 内阻塞发送(每字符 HAL_UART_Transmit 带 10ms 超时)是双违规。
 * 主循环远快于 10ms 控制拍, seg_index 一变即打出, 时效等价; 若要精确落点
 * 用 'p' 周期状态块的 POSE 行。ST: 0=IDLE 1=RUN 2=DONE */
void Car_SegChange_Log(void)
{
	static uint16_t seg_last = 0xFFFFu;
	static uint8_t  state_last = 0xFFu;
	mf_ctx_t *m;
	const mf_status_t *st;
	const mf_pose_t *pose;
	uint16_t seg_now;
	uint8_t  state_now;
	int32_t  x_mm, y_mm, yaw_d10;

	m = mf_app();
	__disable_irq();
	st    = mf_get_status(m);
	pose  = mf_get_pose(m);
	seg_now   = st->seg_index;
	state_now = (uint8_t)st->state;
	x_mm   = (int32_t)(pose->x_m * 1000.0f);
	y_mm   = (int32_t)(pose->y_m * 1000.0f);
	yaw_d10 = (int32_t)(pose->yaw_deg * 10.0f);
	__enable_irq();

	if ((seg_now == seg_last) && (state_now == state_last))
	{
		return;
	}
	seg_last   = seg_now;
	state_last = state_now;
	printf("SEG:%u ST=%u X=%d Y=%d YAW=%d\r\n",
	       (unsigned)seg_now, (unsigned)state_now,
	       (int)x_mm, (int)y_mm, (int)yaw_d10);
}

/* 段出口质量(每跑完一条 ARC / 直线 MOVE 打一行):
 *   ARC:<段> EY=<mm> YE=<0.1°> R=<mm> DY=<0.1°>
 *       EY=出口横向偏差(正=弧外侧); YE=出口航向偏差; DY=**本段实际转角**(名义 900)。
 *       R 与 EY 都是"位姿 vs 名义圆"的**自证量**(内环让 R≡R_cmd), 少转 0.7° 它们照样全 0;
 *       **DY 才是"弧转够了没有"的读数** —— 实测 891~896(名义 900), 每条弧少转 0.7°
 *       ⇒ 出口沿切线少走 11mm(到位余量已改为按实测步长运行期算, 见 mf_config.h)
 *   LN :<段> IE=<0.1°> YE=<0.1°> EY=<mm> IE=进段航向误差(带进来多少); YE=出口残差(纠完剩多少);
 *       EY=出口横向偏差(相对"进段点 + 本段目标航向"这条名义线, 正=左侧)。它把"直线走偏"
 *       分成两种: EY 与进段时差不多 = 整段平行挪出去; EY 明显变大 = 本段越走越斜
 *       ⚠ **IE 非零 = 上一条弧没转够**(MOVE 在替它补角), 不是直线没走直 —— 这是弧到位
 *       余量错配的现场指纹, 与 ARC:DY 成对看
 * 判据: |EY| ≤ ±10mm; DY 收到 900±3; 直线 YE≈0 且 IE 不随路程增长(单基准航向架构
 * 的验收点: 旧行为下 IE 恒 0, 上游误差被锁死看不见) */
void Car_SegExit_Log(void)
{
	uint16_t seg;
	uint8_t  kind;
	float    ey, ie, oe;
	uint8_t  got;

	__disable_irq();
	got = mf_pop_seg_exit(mf_app(), &seg, &kind, &ey, &ie, &oe);
	__enable_irq();

	if (got == 0u)
	{
		return;
	}
	if (kind == 0u)
	{
		printf("ARC:%u EY=%d YE=%d R=%d DY=%d\r\n", (unsigned)seg,
		       (int)(ey * 1000.0f), (int)(oe * 10.0f),
		       (int)(mf_exit_radius_m(mf_app()) * 1000.0f),
		       (int)(mf_exit_dyaw_deg(mf_app()) * 10.0f));
	}
	else
	{
		printf("LN:%u IE=%d YE=%d EY=%d\r\n", (unsigned)seg,
		       (int)(ie * 10.0f), (int)(oe * 10.0f), (int)(ey * 1000.0f));
	}
}

/* 整表重锚事件(每趟 GYRO_ZERO 打一行):
 *   YAWZERO:N=<次数> BASE=<0.1°> FEN=<0/1> LOCK=<0/1>
 *       BASE = 置零那一刻的模块读数 = 本趟的整表重锚基点。表里写死的绝对角(重锚角/返程角)
 *       从此解释在"置零时刻车头 = 0"的路线系里, 所以 BASE 非 0 不再是问题;
 *       它仍是"这趟比上趟转了多少"的读数 —— 两趟 BASE 差多少, 旧实现下地图就差多少。
 *       FEN=MF_HEADING_FRAME_EN(1=整表重锚 / 0=A/B 旧行为: 绝对角钉在上电那一帧)
 * 触发: mf_path 的 ref_seq 变化(置零才涨), 变化才打, 主循环上下文 */
void Car_YawZero_Log(void)
{
	static uint32_t seq_last = 0u;
	mf_path_ctx_t *p = &mf_app()->path;
	uint32_t seq;
	float base;
	uint8_t locked;

	__disable_irq();
	seq    = mf_path_get_ref_seq(p);
	base   = mf_path_get_frame_base(p);
	locked = p->heading_ref_locked;
	__enable_irq();

	if ((seq == seq_last) || (seq == 0u))
	{
		return;
	}
	seq_last = seq;
	printf("YAWZERO:N=%u BASE=%d FEN=%u LOCK=%u\r\n",
	       (unsigned)seq, (int)(base * 10.0f),
	       (unsigned)MF_HEADING_FRAME_EN, (unsigned)locked);
}
