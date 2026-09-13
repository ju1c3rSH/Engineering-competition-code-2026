#include "Control_Car.h"
#include "Car_Sense.h"
#include "mf_math.h"
#include "mf_kinematics.h"

Parameter_TypeDef_Car Car_data;
Yaw_TypeDef Yaw_struct;

void Yaw_pid_Init()
{
	Yaw_struct.Yaw_pid.Kp                  = 0.15f;
	Yaw_struct.Yaw_pid.Ki                  = 0.0f;
	Yaw_struct.Yaw_pid.Kd                  = 0.0f;
	Yaw_struct.Yaw_pid.I_Max               = 0.0f;
	Yaw_struct.Yaw_pid.Out_Max             = 0.8f;
	Yaw_struct.Yaw_pid.Out                 = 0.0f;
}

/* nav_id 段表: 原 flags 路线逐段复刻(基线 git 4f3ceb2 true_BOOM_Finish) */
#define SEG_MOVE(v, d)  { .type = MF_SEG_MOVE_DIST, .reset_pid = 1, \
                          .p.move_dist = { (v), (d), MF_KEEP, TURN_ACC_MPS2 } }
#define SEG_ARC(v, r)   { .type = MF_SEG_ARC, .reset_pid = 1, \
                          .p.arc = { (v), (r), ((r) > 0.0f ? 90.0f : -90.0f), TURN_ACC_MPS2 } }
#define SEG_TURN_TO(y)  { .type = MF_SEG_TURN_TO, .reset_pid = 1, \
                          .p.turn = { (y), 0.0f } }
#define SEG_END         { .type = MF_SEG_END }

static const mf_seg_t tbl_qr[] =          /* NAV_TO_QR: flag0 出发首段 43(台架单段用) */
{
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(43.0f)),
	SEG_END
};
static const mf_seg_t tbl_bomb[] =        /* NAV_TO_BOMB: flag0~10 排爆区 */
{
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(43.0f)),
	SEG_ARC(MF_M_SPEED(Middle_speed),  TURN_RADIUS_M),   /* 左 90 (flag1) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(12.0f)),
	SEG_ARC(MF_M_SPEED(Middle_speed), -TURN_RADIUS_M),   /* 右 90 (flag2) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(46.0f)),
	SEG_ARC(MF_M_SPEED(Middle_speed),  TURN_RADIUS_M),   /* 左 90 (flag4) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(35.0f)),
	SEG_ARC(MF_M_SPEED(Middle_speed), -TURN_RADIUS_M),   /* 右 90 (flag6) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(48.5f)),
	SEG_ARC(MF_M_SPEED(Middle_speed), -TURN_RADIUS_M),   /* 右 90 (flag8) */
	SEG_MOVE(MF_M_SPEED(Low_speed),    MF_M_DIST(45.5f)),
	SEG_END
};
static const mf_seg_t tbl_bucket[] =      /* NAV_TO_BUCKET: 桶与排爆作业点在车道两侧同一点位,
                                           * 到位后仅转台转向桶侧, 无底盘位移 */
{
	SEG_END
};
static const mf_seg_t tbl_target[] =      /* NAV_TO_TARGET: flag11~12 打靶区 */
{
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(175.0f)),
	SEG_END
};
static const mf_seg_t tbl_hostage[] =     /* NAV_TO_HOSTAGE: flag13~16 人质区 */
{
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(37.5f)),
	SEG_ARC(MF_M_SPEED(Middle_speed), -TURN_RADIUS_M),   /* 右 90 (flag14) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(60.0f)),
	SEG_END
};
static const mf_seg_t tbl_return[] =      /* NAV_TO_RETURN: flag17~18 返回区 */
{
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(170.0f)),
	SEG_END
};
static const mf_seg_t tbl_rebase_bomb[] =   /* NAV_REBASE_BOMB: flag10 排爆区重锚, 对坑前原地转绝对角 */
{
	SEG_TURN_TO(-90.3f),
	SEG_END
};
static const mf_seg_t tbl_rebase_return[] = /* NAV_REBASE_RETURN: flag17 携人质返程前重锚 */
{
	SEG_TURN_TO(-179.8f),
	SEG_END
};

static const mf_seg_t *const nav_routes[CAR_NAV_ID_COUNT] =
{
	tbl_qr,             /* 0 NAV_TO_QR         */
	tbl_bomb,           /* 1 NAV_TO_BOMB       */
	tbl_bucket,         /* 2 NAV_TO_BUCKET     */
	tbl_target,         /* 3 NAV_TO_TARGET     */
	tbl_hostage,        /* 4 NAV_TO_HOSTAGE    */
	tbl_return,         /* 5 NAV_TO_RETURN     */
	tbl_rebase_bomb,    /* 6 NAV_REBASE_BOMB   */
	tbl_rebase_return,  /* 7 NAV_REBASE_RETURN */
};
typedef char nav_route_count_check[(sizeof(nav_routes) / sizeof(nav_routes[0])) == CAR_NAV_ID_COUNT ? 1 : -1];

#define NAV_GEAR_BUF_LEN 12u      /* 最长表 tbl_bomb: 11 段 + END */

static float Nav_GearSpeed(uint8_t speed_gear)
{
	switch (speed_gear)
	{
	case 1u:  return MF_M_SPEED(Low_speed);
	case 2u:  return MF_M_SPEED(Middle_speed);
	case 3u:  return MF_M_SPEED(High_speed);
	default:  return 0.0f;
	}
}

/* speed_gear 覆盖需改写段速度，拷入 RAM 缓冲后启动 */
static const mf_seg_t *Nav_GearTable(uint8_t navigation_target_id, uint8_t speed_gear)
{
	static mf_seg_t buf[NAV_GEAR_BUF_LEN];
	const mf_seg_t *src = nav_routes[navigation_target_id];
	uint8_t i;
	float v;

	if (speed_gear == 0u)
	{
		return src;
	}
	v = Nav_GearSpeed(speed_gear);
	for (i = 0u; i < NAV_GEAR_BUF_LEN; i++)
	{
		buf[i] = src[i];
		if (buf[i].type == MF_SEG_END)
		{
			break;
		}
		if (buf[i].type == MF_SEG_MOVE_DIST)
		{
			buf[i].p.move_dist.speed_mps = v;
		}
		else if (buf[i].type == MF_SEG_ARC)
		{
			buf[i].p.arc.speed_mps = v;
		}
	}
	return buf;
}

/* ================== 调度层状态（panzer_links_protocol.md 4.5） ================== */
#define CAR_TASK_MS            10u
#define NAV_STALL_MS           3000u
#define NAV_STALL_DELTA_M      0.001f
#define TURNTABLE_SETTLE_MS    3000u

typedef enum
{
	NAV_IDLE = 0,   /* 上电/REQ_STOP/FAULT 后，任意 REQ_NAV 均从段 0 开始 */
	NAV_RUN,        /* 路径执行中（mf path RUN） */
	NAV_DONE,       /* 已完成，同 id 重发只补发 NAV_DONE */
} NavState_t;

static struct
{
	NavState_t state;
	uint8_t    nav_id;
	uint16_t   stall_ms;
} nav;

/* ---- 打靶段内跟瞄 v2: 段表不断, 段内叠加视觉航向修正 ----
 * 靶标前一段由中控发 TRACKING_START 开启, 底盘按指定速度边走边修航向;
 * horizontal_error_normalized 归零即段内刹停等发射, 发射后 TRACKING_STOP
 * (mode=1) 继续走完当前段。履带动, 转盘不动 */
typedef enum
{
	TRACKING_IDLE = 0,    /* 未跟瞄 */
	TRACKING_RUN,         /* 跟瞄中: 段内叠加视觉航向修正 */
	TRACKING_LOCKED,      /* 已收敛刹停: 等发射 */
} TrackingState_t;

static struct
{
	TrackingState_t state;
	float horizontal_error_normalized;  /* 最新归一化横向误差, 约 -1~+1 */
	uint8_t target_found_flag;          /* 最新有效标志 */
	uint16_t frame_age_ms;              /* 距上次误差帧时长 */
	uint16_t hold_ms;                   /* |e|小于阈值维持时长 */
	uint8_t tracking_speed_level;       /* 0=段表速度/1=低速跟瞄 */
} tracking;

#define TRACKING_ERROR_FIXED_SCALE  10000.0f

/* cmd mailbox：UART4 中断单生产者投递，本定时任务消费 */
static volatile uint8_t mail_head, mail_tail;
static volatile uint8_t mail_cmd[8], mail_a0[8], mail_a1[8];

void Car_Nav_MailPush(uint8_t cmd, uint8_t a0, uint8_t a1)
{ 
	uint8_t next = (uint8_t)(mail_head + 1u) & 7u;

	if (next == mail_tail)
	{
		return;
	}
	mail_cmd[mail_head] = cmd;
	mail_a0[mail_head] = a0;
	mail_a1[mail_head] = a1;
	mail_head = next;
}

/* 跟瞄误差帧独立投递: error_fixed = 横向误差归一化*10000(小端 i16) */
static volatile int16_t tracking_error_mail;
static volatile uint8_t tracking_found_mail;
static volatile uint8_t tracking_error_pending;

void Car_Tracking_ErrorPush(int16_t error_fixed, uint8_t target_found_flag)
{
	__disable_irq();
	tracking_error_mail = error_fixed;
	tracking_found_mail = target_found_flag;
	tracking_error_pending = 1u;
	__enable_irq();
}

/* 转台码->定点角: 对坑 1->123 2->90 3->57, 对桶 4/5->270 6->90, 7 回正->90, 0 不动作 */
static struct
{
	uint8_t  busy;
	uint8_t  code;      /* 本次请求码, TURN_DONE 原样回显 */
	uint16_t ms;
} tt;
static uint8_t CW_OR_CCW = 1;

/* 转台定点角: 排爆/人质摆放距离一致, 惯导保证车位, 按码打固定角。
 * 对坑 1~3->123/90/57; 对桶 4~6->270/270/90(入桶摆位沿用实车验证值,
 * 坑3 反向 CW 到 90, 其余 CCW); 7 回正 90 */
static void Nav_Turntable(uint8_t turn_code)
{
	CW_OR_CCW = 1;
	switch (turn_code)
	{
	case 1u: Car_data.Motor_Angle = 123.0f; break;
	case 2u: Car_data.Motor_Angle =  90.0f; break;
	case 3u: Car_data.Motor_Angle =  57.0f; break;
	case 4u:
	case 5u: Car_data.Motor_Angle = 270.0f; break;
	case 6u: Car_data.Motor_Angle =  90.0f; CW_OR_CCW = 0u; break;
	case 7u: Car_data.Motor_Angle =  90.0f; break;
	default: return;
	}
	tt.busy = 1u;
	tt.code = turn_code;
	tt.ms = 0u;
}

/* REQ_NAV 幂等: 同 id 进行中忽略 / 已完成补发 DONE / 其余停旧起新 */
static void Nav_DoRequest(uint8_t navigation_target_id, uint8_t speed_gear)
{
	if (nav.state == NAV_RUN && nav.nav_id == navigation_target_id)
	{
		return;
	}
	if (nav.state == NAV_DONE && nav.nav_id == navigation_target_id)
	{
		Car_Event_Send(CAR_EVT_NAV_DONE, navigation_target_id, 0x00u);
		return;
	}
	mf_start_path(mf_app(), Nav_GearTable(navigation_target_id, speed_gear));
	nav.state = NAV_RUN;
	nav.nav_id = navigation_target_id;
	nav.stall_ms = 0u;
	/* 起新段清跟瞄: 防 EXE_LASER 重试/FAULT 泄漏的 LOCKED 带到新路段 */
	tracking.state = TRACKING_IDLE;
	tracking.hold_ms = 0u;
}

static void Nav_DoStop(void)
{
	mf_stop(mf_app());
	nav.state = NAV_IDLE;
	tracking.state = TRACKING_IDLE;
}

/* ---- 段内跟瞄: 段表不断, 段内叠加视觉航向修正 ---- */
static void Tracking_HoldStill(mf_ctx_t *motion)
{
	motion->cmd.vx_mps = 0.0f;
	motion->cmd.vy_mps = 0.0f;
	motion->cmd.wz_dps = 0.0f;
	/* 同步泄放段内斜坡状态: mf_path_tick 下拍会从 0 重新起坡,
	 * 保证 TRACKING_STOP(1) 释放时是低速缓起而不是 0->巡航 的速度阶跃 */
	motion->path.cur_speed = 0.0f;
	motion->path.cur_wz = 0.0f;
	motion->path.prev_wz = 0.0f;
}

static void Tracking_Start(uint8_t tracking_speed_level)
{
	if (tracking.state == TRACKING_LOCKED)
	{
		/* 已收敛后重收 START = Brain 补发, 只补发 DONE 不复位 */
		Car_Event_Send(CAR_EVT_TRACKING_DONE, 0u, 0x00u);
		return;
	}
	tracking.state = TRACKING_RUN;
	tracking.tracking_speed_level = tracking_speed_level;
	tracking.frame_age_ms = 0u;
	tracking.hold_ms = 0u;
	tracking.target_found_flag = 0u;
	tracking.horizontal_error_normalized = 0.0f;
	tracking_error_pending = 0u;   /* 丢弃上轮残帧, 首拍不等旧误差 */
}

/* 原子消费误差帧: 有新帧更新误差并喂狗, 无新帧不碰 frame_age_ms */
static void Tracking_ConsumeErrorFrame(void)
{
	uint8_t has_new_frame;

	__disable_irq();
	has_new_frame = tracking_error_pending;
	if (has_new_frame != 0u)
	{
		tracking.horizontal_error_normalized =
		    (float)tracking_error_mail / TRACKING_ERROR_FIXED_SCALE;
		if (tracking.horizontal_error_normalized > 1.0f)  { tracking.horizontal_error_normalized = 1.0f; }
		if (tracking.horizontal_error_normalized < -1.0f) { tracking.horizontal_error_normalized = -1.0f; }
		tracking.target_found_flag = (uint8_t)(tracking_found_mail & 0x01u);
		tracking.frame_age_ms = 0u;
		tracking_error_pending = 0u;
	}
	__enable_irq();
}

static void Tracking_Stop(uint8_t tracking_stop_mode)
{
	/* mode=0 就地刹停(锚点保留) / mode=1 解除压零, 段表续跑 */
	if (tracking_stop_mode == 0u)
	{
		Tracking_HoldStill(mf_app());
		tracking.state = TRACKING_LOCKED;
	}
	else
	{
		tracking.state = TRACKING_IDLE;
	}
}

/* 跟瞄收敛判定: |e|小于阈值累计 hold_ms, 达标即刹停上报 DONE。返回 1=本拍已收敛 */
static uint8_t Tracking_CheckConverged(mf_ctx_t *motion)
{
	float absolute_error = tracking.horizontal_error_normalized;
	if (absolute_error < 0.0f)
	{
		absolute_error = -absolute_error;
	}
	if (absolute_error >= TRACKING_ERROR_THRESHOLD)
	{
		tracking.hold_ms = 0u;
		return 0u;
	}
	tracking.hold_ms = (uint16_t)(tracking.hold_ms + CAR_TASK_MS);
	if (tracking.hold_ms < TRACKING_HOLD_TIME_MS)
	{
		return 0u;
	}
	Tracking_HoldStill(motion);
	tracking.state = TRACKING_LOCKED;
	Car_Event_Send(CAR_EVT_TRACKING_DONE, 0u, 0x00u);
	return 1u;
}

/* 视觉 P 前馈: vx 保留, wz += -Kp*e (e>0 目标偏右→顺时针; 镜像摄像头翻增益符号) */
static void Tracking_ApplyYawCorrection(mf_ctx_t *motion)
{
	float yaw_correction_dps = -TRACKING_PROPORTIONAL_GAIN * tracking.horizontal_error_normalized;
	if (yaw_correction_dps > TRACKING_MAX_YAW_RATE_DPS)
	{
		yaw_correction_dps = TRACKING_MAX_YAW_RATE_DPS;
	}
	else if (yaw_correction_dps < -TRACKING_MAX_YAW_RATE_DPS)
	{
		yaw_correction_dps = -TRACKING_MAX_YAW_RATE_DPS;
	}
	motion->cmd.wz_dps += yaw_correction_dps;
	if (tracking.tracking_speed_level == 1u && motion->cmd.vx_mps > MF_M_SPEED(Low_speed))
	{
		motion->cmd.vx_mps = MF_M_SPEED(Low_speed);
	}
}

/* 每拍在 mf_tick 之后调用, RUN 态恒返 1(修正须当拍重算轮速环) */
static uint8_t Tracking_ApplyOverlay(void)
{
	mf_ctx_t *motion = mf_app();
	float absolute_error;
	uint8_t servo_allowed;

	if (tracking.state == TRACKING_LOCKED)
	{
		Tracking_HoldStill(motion);
		return 1u;
	}
	if (tracking.state != TRACKING_RUN)
	{
		return 0u;
	}
	Tracking_ConsumeErrorFrame();
	tracking.frame_age_ms = (uint16_t)(tracking.frame_age_ms + CAR_TASK_MS);
	if (tracking.target_found_flag == 0u || tracking.frame_age_ms > TRACKING_FRAME_TIMEOUT_MS)
	{
		tracking.hold_ms = 0u;
		Tracking_HoldStill(motion);
		return 1u;
	}
	absolute_error = (tracking.horizontal_error_normalized < 0.0f)
	                 ? -tracking.horizontal_error_normalized
	                 : tracking.horizontal_error_normalized;
	/* 静瞄(段已走完)仅限小误差, 防大角旋转带歪后续段表; 段内不限 */
	servo_allowed = (uint8_t)((mf_is_done(motion) == 0u)
	                          || (absolute_error <= TRACKING_STATIC_ERROR_MAX));
	if (Tracking_CheckConverged(motion) != 0u)
	{
		return 1u;
	}
	if (servo_allowed != 0u)
	{
		Tracking_ApplyYawCorrection(motion);
	}
	else
	{
		Tracking_HoldStill(motion);
	}
	return 1u;
}

static void Nav_Fault(void)
{
	mf_stop(mf_app());
	nav.state = NAV_IDLE;
	Car_Event_Send(CAR_EVT_NAV_FAULT, nav.nav_id, CAR_FAULT_WATCHDOG);
}

void Car_Nav_Task(void)
{
	static uint8_t inited = 0;
	mf_ctx_t *m;

	if (inited == 0)
	{
		mf_app_init();
		inited = 1;
	}

	while (mail_tail != mail_head)
	{
		uint8_t cmd = mail_cmd[mail_tail];
		uint8_t a0 = mail_a0[mail_tail];
		uint8_t a1 = mail_a1[mail_tail];

		mail_tail = (uint8_t)(mail_tail + 1u) & 7u;
		switch (cmd)
		{
		case CAR_SUB_REQ_NAVIGATION:    Nav_DoRequest(a0, a1); break;
		case CAR_SUB_REQ_STOP:          Nav_DoStop(); break;
		case CAR_SUB_REQ_TURNTABLE:     Nav_Turntable(a0); break;
		case CAR_SUB_TRACKING_START:    Tracking_Start(a0); break;
		case CAR_SUB_TRACKING_STOP:     Tracking_Stop(a0); break;
		case CAR_SUB_GYRO_ZERO:
			/* 车静止时置零，此刻必在原点(上电流程：扫码->置零->起步) */
			if (nav.state != NAV_RUN)
			{
				ResetHWT101();
				mf_set_pose(mf_app(), 0.0f, 0.0f, 0.0f);
			}
			tracking.state = TRACKING_IDLE;
			break;
		default: break;
		}
	}
	/* 误差帧每拍消费一次 */
	Tracking_ConsumeErrorFrame();

	if (tt.busy != 0u)
	{
		tt.ms = (uint16_t)(tt.ms + CAR_TASK_MS);
		if (tt.ms >= TURNTABLE_SETTLE_MS)
		{
			tt.busy = 0u;
			Car_Event_Send(CAR_EVT_TURN_DONE, tt.code, 0x00u);
		}
	}

	if (nav.state != NAV_RUN)
	{
		return;
	}

	m = mf_app();
	if (mf_is_done(m))
	{
		nav.state = NAV_DONE;
		Car_Event_Send(CAR_EVT_NAV_DONE, nav.nav_id, 0x00u);
		return;
	}

	{
		/* 堵转判据=两轮位移绝对值之和: 原地转(重锚 TURN_TO)左右轮反向等大,
		 * 取代数和恒 0 会误报, 逐轮取绝对值对直行/弧线/原地转统一成立 */
		float d0 = m->wheel[0].delta_m;
		float d1 = m->wheel[1].delta_m;
		float d = (d0 < 0.0f ? -d0 : d0) + (d1 < 0.0f ? -d1 : d1);

		if (tracking.state != TRACKING_IDLE)
		{
			/* 跟瞄期间的停车是有意的(收敛刹停/丢帧悬停), 不算堵转 */
			nav.stall_ms = 0u;
		}
		else if (d > NAV_STALL_DELTA_M)
		{
			nav.stall_ms = 0u;
		}
		else
		{
			nav.stall_ms = (uint16_t)(nav.stall_ms + CAR_TASK_MS);
			if (nav.stall_ms >= NAV_STALL_MS)
			{
				Nav_Fault();
			}
		}
	}
}

/* ================== 运动框架实例 ==================
 * 增益为旧环等效换算值，台架上按 mf 库语义重调：
 * 轮速环 P: 旧 Kp4000(计数误差->PWM 满幅 1500) 折归一化(暂留 1.0 台架重调)
 * 轮速环 I: 按旧环 Ki=50/拍(误差 rev/s, 满幅 1500, I_Max 1000) 等效折算:
 *   贡献速率 = 误差×50÷0.01s÷1500 = 3.33 duty/s per rev/s
 *            = 7.07 duty/s per m/s(1rev/s=π×0.15m)          -> ki = 7.0
 *   I 上限   = 1000÷1500 = 0.667 duty -> i_max = 0.667÷7.07 = 0.095
 * 航向环: 旧 Kp0.15/OutMax0.8(rev/s 差速) × π×WHEEL_DIA_M / (track/2) -> deg/s */
static mf_ctx_t g_mf;

mf_ctx_t *mf_app(void)
{
	return &g_mf;
}

void mf_app_init(void)
{
	static const mf_cfg_t cfg =
	{
		.track_width_m = 0.25f,          /* 履带中心距 250mm 实测(motion.h) */
		.half_length_m = 0.0f,
		.half_width_m  = 0.0f,
		.max_vx_mps    = 1.5f,           /* 覆盖 High_speed 1.41m/s */
		.max_vy_mps    = 0.0f,
		.max_wz_dps    = 150.0f,         /* Middle 弧线稳态 57dps + 修正余量 */
		.yaw_kp        = 32.4f,
		.yaw_ki        = 0.0f,
		.yaw_kd        = 0.0f,
		.yaw_i_max     = 0.0f,
		.yaw_out_max   = 172.8f,
		.pv_kp         = 2.0f,
		.wheel_kp      = 1.0f,
		.wheel_ki      = 7.0f,     /* 旧环 I 等效(见上方折算), 恢复静摩擦穿透能力 */
		.wheel_i_max   = 0.095f,   /* 7.0×0.095≈0.665 满占空比, 等效旧 I_Max1000/1500 */
	};
	mf_init(&g_mf, &cfg);
}

/* 按改写后 cmd 重解轮速环(轮速环积分每拍多累一次, 复调 Ki 注意) */
static void Tracking_RecomputeWheels(void)
{
	mf_ctx_t *motion = mf_app();
	float wheel_command[MF_WHEEL_MAX];
	uint8_t wheel_index;

	motion->cmd.vx_mps = mf_clampf(motion->cmd.vx_mps, motion->cfg.max_vx_mps);
	motion->cmd.wz_dps = mf_clampf(motion->cmd.wz_dps, motion->cfg.max_wz_dps);
	motion->cmd.vy_mps = 0.0f;
	mf_inverse_kinematics(&motion->cmd, motion->cfg.track_width_m,
	                      motion->cfg.half_length_m, motion->cfg.half_width_m,
	                      wheel_command);
	for (wheel_index = 0u; wheel_index < MF_WHEEL_COUNT; wheel_index++)
	{
		motion->wheel[wheel_index].cmd_mps = wheel_command[wheel_index];
		motion->wheel[wheel_index].out = mf_pid_calc(&motion->pid_wheel[wheel_index],
		    wheel_command[wheel_index] - motion->wheel[wheel_index].speed_mps, MF_DT_S);
		if (motion->cmd.vx_mps == 0.0f && motion->cmd.wz_dps == 0.0f)
		{
			motion->wheel[wheel_index].out = 0.0f;
			mf_pid_reset(&motion->pid_wheel[wheel_index]);
		}
	}
}

//坦克主控函数：TIM4 10ms 中断调用
void Car_Control()
{
	CarSense_Sample();                   /* 采样层: 编码器+航向 -> 快照(seq+1) */
	Car_Nav_Task();                      /* 调度层: 信箱/启停/事件/看门狗 */
	Car_TxQueue_Pump();                  /* 上行发送队列: ACK/EVT 单点泵出 */
	mf_tick(mf_app(), CarSense_Get());   /* 运动库: 纯消费快照, 无新样自动跳过 */
	if (Tracking_ApplyOverlay() != 0u)   /* 段内跟瞄改写了 cmd */
	{
		Tracking_RecomputeWheels();
	}
	CarSense_Output(mf_app());           /* 执行层: wheel[].out -> DIR+PWM */
}

//HWT101角度置零
void ResetHWT101(void)
{
	uint8_t temp[]={0xff,0xAA,0X76,0x00,0x00};
	HAL_UART_Transmit_DMA(&huart3, temp, 5);
}
void Emm_Pos_Control(float Angle)
{
	static uint32_t Angle_Tick = 0;
	Angle_Tick = (uint32_t)(1208.888 * Angle);
	Emm_V5_Pos_Control(1,CW_OR_CCW,180,0,Angle_Tick,true,false);
}
