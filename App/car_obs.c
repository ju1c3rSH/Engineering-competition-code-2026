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

/* 't' 命令: 段内瞬时跟踪(20Hz 开窗 4s), 判"转向一卡一卡"卡在哪一环。
 * 列: SP/W/O/I=速率内环(设定/IMU实测/下发/积分); CL/SL/CR/SR=轮速环(指令/实测);
 *     RS=段内剩余里程; GZ=0x52 陀螺直给角速度(**判 W 真伪**)。
 * 读法: GZ 平滑而 W 反相摆=采样伪影(250Hz帧÷拍率交替, 见 mf_pid.h); GZ≡W=真振荡;
 *     W≈SP 而 SL/SR 阵掉 0=轮速环 stick-slip; SP 自摆=外环欠阻尼(见 docs/坑-编码器与里程.md §5)。 */
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

/* USART2 调试口状态块('?'/p 周期, main 上下文, 仅调试不进控制路径): 原 9 参 Snapshot 内聚于此。
 * 【已知偏离 D-1, 已裁决不修】本函数分四次取样(2 GetInfo + nav 2 裸读 + wheel 窗口),
 * 不共用一个关中断窗口 → 各字段可能跨 TIM4 拍, 偶发"新 NAV 配旧 SEG"式矛盾读数。见 施工说明 §8 D-1。 */
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
	/* 航向观测(HDG, 路线系): K=段型 A弧/L直/T绝对转/R相对转/G到点; TGT=航向目标; YAW=本拍路线系航向;
	 * ERR=环内误差; EY=横向偏差(直线: 名义线左侧正 / ARC: 弧外正); BASE=本表重锚基点。单位 0.1°, EY 为 mm。
	 * 判读: ERR 稳 0 仍偏=基准/基点问题; EY 不变=整段平移; EY 线性增=越走越斜。 */
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
	/* HWT101 链路健康(hwt101_health_t): RRATE=当前 0x53 帧率 / BOOT=上电采样 / SEQ=帧计数(不涨即断流)
	 * / BAD=校验失败 / XOK/XBAD=交叉校验通过/丢弃 / TXF=配置帧发送失败 / VER=版本。
	 * 判"速率配置生效": RRATE≈200 已配 / ≈10 出厂默认 / 0 没通; TXF≠0=配置帧没发出(写寄存器单向无 ACK)。
	 * 跨版本比较先对齐口径(RRATE 真实帧率 vs 旧日志突发下界): docs §8.11。 */
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
	/* 速率内环健康(RATE): SP=外环设定; W=IMU 实测; O=下发(基线+PI); B=开环基线; I=积分(=ki·∫e)。
	 * 基线与积分是**同一补偿的两半**(基线越准 I 越≈0)。OK/ST/SN=闭环/停更/反号累计拍。
	 * 读法: ST/SN 涨=反馈断(先查 HWT SEQ); W≈SP=真闭环(可把基线往 1.0 收);
	 *     W 仅 SP 的 0.6 倍=环在但对象跟不上(看 O 是否顶 150 上限), W/O 即对象增益 k, 复标 MF_ARC_SLIP_GAIN 用。 */
	printf("RATE:SP=%d W=%d O=%d B=%d I=%d OK=%u ST=%u SN=%u\r\n",
	       (int)ri.sp_dps, (int)ri.meas_dps, (int)ri.out_dps,
	       (int)ri.base_dps, (int)ri.i_term_dps,
	       (unsigned)ri.n_ok, (unsigned)ri.n_stale, (unsigned)ri.n_sign);
	/* 弧几何观测(RAD): RSP=设定半径 v/|wz_sp|; RM=实测 v/|ω_meas|; REF=库内同间隔实测半径(mf_odom.c);
	 * RV=REF 新鲜; KH=对象增益 ω_meas/ω_out; YF=**帧数/拍数原始比值 dq/dtk**(如 200/200), 不是百分比。
	 * 务必打原始比值: 百分比把 1:1 健康值压成 100, 正落在"丢帧/<100=故障"边界, 丢帧变 99 误判;
	 * 原始比值里丢帧显为 199/200 一眼可辨(200Hz 改拍后 YF 恒 N/N)。判半径只认 RM/REF/R 或卷尺(§8)。 */
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

/* 段出口质量(每跑完一段打一行): ARC: EY=出口横向偏差(正=弧外); YE=出口航向残差;
 *   R/EY 是"位姿vs名义圆"自证量(内环让 R≡R_cmd, 少转 0.7° 仍全 0); **DY=本段实际转角**才是"转够了没"
 *   (实测 891~896, 名义 900 ⇒ 出口少走 11mm, 到位余量已改按实测步长运行期算, 见 mf_config.h)。
 * LN : IE=进段航向误差(非零=上一条弧没转够, MOVE 在替补); YE=出口残差; EY=相对名义线横向偏差。
 * 判据: |EY|≤±10mm; DY≈900±3; 直线 YE≈0 且 IE 不随路程增长。 */
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

/* ---- 'i' 陀螺仪观测窗口(3s, 5Hz) ----
 * 起因: 固件对 HWT101 零偏零动作(无 CALIYAW、无 0x4C 读写), 零偏 100% 由模块
 * 出厂自动校准负责。现有观测够不到它: TRC: 的 GZ 是**转速触发**(30dps), 静止时
 * 一个数都打不出来 ⇒ 只能加一条静止可打的观测线。
 * 纯只读: 官方明确读寄存器不需要解锁、不写 Flash、不进校准模式, 零副作用。
 * 字段判读见文件末尾注释与 docs/坑-IMU零偏与长期漂移.md。 */
/* IMU: 行怎么读 —— 车放平完全不动, 敲 i 看这四条组合:
 *   BIAS≈0  且 GZ 明显非零 => 自动校准从未成功补偿 ⇒ "跑久了偏移"是纯零偏积累
 *   BIAS≠0  但 GZ 仍非零    => 补偿与实际脱节(官方描述的失效形态) ⇒ 该重校零偏
 *   BIAS≠0  且 GZ≈0         => 补偿与实际一致, 链路健康
 * 字段为 "-" = 该槽还没回读成功(窗口开头几拍正常)。GZ 是 ×1000 定点(0.1°/s 精度)。
 * ★BIAS 是模块存下来的**账面值**, 不等于此刻的物理零偏(官方明说二者会脱节);
 *   真要拿"此刻物理零偏"只能看 GZ。WSEQ 不涨 = 0x52 断流 ⇒ GZ 是陈旧值, 先查 BAUD。
 *   TP(0x4A 静止峰峰值)偏大 = 履带振动被模块当成转动; MODE≠0 = 不在普通态。 */
#define CAR_OBS_IMU_WIN_MS    3000u   /* 敲一次连打 3s: 够看出静止时 GZ 的斜率 */
#define CAR_OBS_IMU_PERIOD_MS 200u     /* 5Hz。四个槽位轮读一圈 800ms, 窗口内每槽 3~4 次 */

static uint32_t imu_end_ms;
static uint32_t imu_last_ms;
static uint8_t  imu_slot;      /* 本拍要发读命令的槽位 */
static uint8_t  imu_win;       /* 0=空闲 1=窗口开 */

/* 把回读槽位格式化成定点串: 未回读过给 "-"(与"值真的是 0"必须能区分)。
 * BIAS/TP 按官方 /1000 定点(0x4C 的单位是 °/s×1000); 开关/模式类原值直出。
 * **手写而非 sprintf**: 本工程 printf 是逐字符轮询忙等, 再引一个 stdio 格式化
 * 会拖慢主循环; 且 buf 是栈上数组, sprintf 的变参调用在 microlib 下有踩栈风险。
 * 小数部分必须**左补零到三位**(0.005 印成 "0.005" 而不是 "0.5"), 否则同一个
 * 零偏值会随大小印出不同位数、读数对不上 —— 见 build_verify/hwt101_test 单测。 */
static const char *Imu_FmtRaw(char *buf, uint8_t fresh, int16_t raw, uint8_t fixed3)
{
	uint32_t mag, ip, fp;
	int8_t  n = 0, i, p = 0;
	char     tmp[8];

	if (fresh == 0u)
	{
		buf[0] = '-';
		buf[1] = '\0';
		return buf;
	}
	mag = (raw < 0) ? (uint32_t)(-(int32_t)raw) : (uint32_t)raw;
	if (raw < 0)
	{
		buf[p++] = '-';
	}
	if (fixed3 != 0u)
	{
		ip = mag / 1000u;
		fp = mag % 1000u;
		/* 整数部分**按需吐位**(值域 ±32.767 ⇒ 最多两位)。固定只吐一位会把
		 * 32.767 印成 2.767 —— 正好落在"大零偏"这个最需要看清的区间上 */
		if (ip >= 10u)
		{
			buf[p++] = (char)('0' + ((ip / 10u) % 10u));
		}
		buf[p++] = (char)('0' + (ip % 10u));
		buf[p++] = '.';
		buf[p++] = (char)('0' + ((fp / 100u) % 10u));
		buf[p++] = (char)('0' + ((fp / 10u) % 10u));
		buf[p++] = (char)('0' + (fp % 10u));
		buf[p] = '\0';
		return buf;
	}
	do   /* 0 也要出 "0", 故用 do-while */
	{
		tmp[n++] = (char)('0' + (mag % 10u));
		mag /= 10u;
	} while (mag != 0u);
	for (i = (int8_t)(n - 1); i >= 0; i--)
	{
		buf[p++] = tmp[i];
	}
	buf[p] = '\0';
	return buf;
}

void Car_ImuProbe_Arm(void)
{
	imu_win  = 1u;
	imu_slot = (uint8_t)HWT101_RB_BIAS;
	/* 末拍阈值比窗口短一个周期: 让 imu_last_ms 天然落在"一个周期前" ⇒ 首拍立即发第一条,
	 * 3s 窗口打满 15 行而不是 14 行 */
	imu_last_ms = HAL_GetTick() - CAR_OBS_IMU_PERIOD_MS;
	imu_end_ms  = HAL_GetTick() + CAR_OBS_IMU_WIN_MS;
	/* 清掉有效位: 窗口内若某槽一直没回读成功, 显示 "-" 而不是上一次的陈旧值。
	 * 代价是刚敲下键的前几拍可能全是 "-", 那正是"还没读到"的诚实表达 */
	Hwt101_ClearRegFresh();
	printf("IMU:WIN %ums (BIAS=%s NOCAL=%s TP=%s MODE=%s)\r\n",
	       (unsigned)CAR_OBS_IMU_WIN_MS, "0x4C", "0xA7", "0x4A", "0x48");
}

void Car_ImuProbe_Task(void)
{
	const hwt101_health_t *hw = Hwt101_Health();
	char b_bias[16], b_nocal[16], b_tp[16], b_mode[16];
	float   yaw, gz;
	uint32_t wseq, tick;
	int32_t  ims;

	if (imu_win == 0u)
	{
		return;
	}
	tick = HAL_GetTick();
	if ((uint32_t)(tick - imu_last_ms) < CAR_OBS_IMU_PERIOD_MS)
	{
		return;   /* 节流到 5Hz */
	}
	imu_last_ms = tick;

	/* 轮读: 本拍只发一条, 打的是**上一轮**回填的值(Hwt101_ReadReg 不等应答)。
	 * 快照与发命令之间刻意不关中断 —— 打印的量都是单字/双字, 撕裂一两次无害 */
	switch (imu_slot)
	{
	case HWT101_RB_BIAS:  Hwt101_ReadReg(HWT101_REG_GYROBAIS);   break;
	case HWT101_RB_NOCAL: Hwt101_ReadReg(HWT101_REG_NOAUTOCALI); break;
	case HWT101_RB_OPTPE: Hwt101_ReadReg(HWT101_REG_OPTPE);      break;
	default:              Hwt101_ReadReg(HWT101_REG_WORKMODE);   break;
	}
	imu_slot = (uint8_t)((imu_slot + 1u) % HWT101_RB_COUNT);

	yaw  = Hwt101_GetYawDeg();
	wseq = Hwt101_WzSeq();
	gz   = Hwt101_GetWzDps();

	Imu_FmtRaw(b_bias,  Hwt101_RegFresh(HWT101_RB_BIAS),  Hwt101_RegRaw(HWT101_RB_BIAS),  1u);
	Imu_FmtRaw(b_nocal, Hwt101_RegFresh(HWT101_RB_NOCAL), Hwt101_RegRaw(HWT101_RB_NOCAL), 0u);
	Imu_FmtRaw(b_tp,    Hwt101_RegFresh(HWT101_RB_OPTPE), Hwt101_RegRaw(HWT101_RB_OPTPE), 1u);
	Imu_FmtRaw(b_mode,  Hwt101_RegFresh(HWT101_RB_MODE),  Hwt101_RegRaw(HWT101_RB_MODE),  0u);

	/* GZ 乘 1000 输出(定点, 不依赖 printf 的浮点支持): 它是静止零偏的直接观测点。
	 * WSEQ 不涨 = 0x52 断流, 此时 GZ 是陈旧值, 判读前先看它 */
	ims = (int32_t)(gz * 1000.0f);
	printf("IMU:YAW=%d GZ=%d WSEQ=%lu BIAS=%s NOCAL=%s TP=%s MODE=%s TXF=%lu\r\n",
	       (int)(yaw * 10.0f), (int)ims, (unsigned long)wseq,
	       b_bias, b_nocal, b_tp, b_mode, (unsigned long)hw->tx_fail);

	if ((uint32_t)(tick - imu_end_ms) >= 0x80000000u)   /* 模块化比较: tick 已过窗口末 */
	{
		imu_win = 0u;
		printf("IMU:END\r\n");
	}
}

/* 整表重锚事件(YAWZERO, 每趟 GYRO_ZERO 打一行): BASE=置零时刻模块读数=本趟重锚基点;
 * 表里绝对角从此解释在"置零时刻车头=0"路线系, BASE 非 0 不再是问题(仍是"两趟差多少"读数)。
 * FEN=MF_HEADING_FRAME_EN(1=整表重锚 / 0=A/B 旧行为)。触发: mf_path 的 ref_seq 变化才打。 */
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
