#include "car_obs.h"
#include "main.h"
#include "Emm_V5.h"
#include "mf_path.h"
#include "motion_app.h"
#include "nav_task.h"
#include "nav_turntable.h"
#include "nav_tracking.h"
#include <stdio.h>

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
	Tt_Info_t tti;
	Trk_Info_t tri;

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
 *   ARC:<段> EY=<mm> YE=<0.1°>   EY=出口横向偏差(正=弧外侧, 负=往里切); YE=出口航向偏差
 *   LN :<段> IE=<0.1°> YE=<0.1°> IE=进段航向误差(带进来多少); YE=出口残差(纠完剩多少)
 * 判据: 单弧 |EY| 压到 ±10mm 内; 直线 YE≈0 且 IE 不随路程增长 —— 后者是单基准航向
 * 架构(MF_HEADING_FRAME_EN=1)的现场验收点: 旧行为下 IE 恒 0(上游误差被锁死看不见) */
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
		printf("ARC:%u EY=%d YE=%d\r\n", (unsigned)seg,
		       (int)(ey * 1000.0f), (int)(oe * 10.0f));
	}
	else
	{
		printf("LN:%u IE=%d YE=%d\r\n", (unsigned)seg,
		       (int)(ie * 10.0f), (int)(oe * 10.0f));
	}
}
