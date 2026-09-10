#include "Control_Car.h"
#include "Car_Sense.h"

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

/* ================== nav_id 段表：原 if-else 路线完整复刻 ==================
 * 逐段对应原 Car_Drive_route 的 flags 路线(git true_BOOM_Finish 基线)：
 * 数值即原常量(旧 Car_Pos 单位，1 单位 = 1/15 驱动轮转)，速度档同旧
 * Middle/Low，转弯 = 原 Car_turn_corner(90° 弧，中心线走 π 轮转)。
 * NAV_TO_QR 仅出发首段；NAV_TO_BOMB 自原点连续复刻 flag0~10(原点长焦
 * 扫码，途中不停)。转台编排(flag9/100/11)不进段表，由主控 REQ_TURNTABLE
 * 按节点调度；flag17 的 5s 兜底转向同属主控逻辑。
 * ======================================================================== */
#define SEG_MOVE(v, d)  { .type = MF_SEG_MOVE_DIST, .reset_pid = 1, \
                          .p.move_dist = { (v), (d), MF_KEEP, TURN_ACC_MPS2 } }
#define SEG_ARC(v, r)   { .type = MF_SEG_ARC, .reset_pid = 1, \
                          .p.arc = { (v), (r), ((r) > 0.0f ? 90.0f : -90.0f), TURN_ACC_MPS2 } }
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

static const mf_seg_t *const nav_routes[CAR_NAV_ID_COUNT] =
{
	tbl_qr,        /* 0 NAV_TO_QR     */
	tbl_bomb,      /* 1 NAV_TO_BOMB   */
	tbl_bucket,    /* 2 NAV_TO_BUCKET */
	tbl_target,    /* 3 NAV_TO_TARGET */
	tbl_hostage,   /* 4 NAV_TO_HOSTAGE*/
	tbl_return,    /* 5 NAV_TO_RETURN */
};
typedef char nav_route_count_check[(sizeof(nav_routes) / sizeof(nav_routes[0])) == CAR_NAV_ID_COUNT ? 1 : -1];

#define NAV_GEAR_BUF_LEN 12u      /* 最长表 tbl_bomb: 11 段 + END */

static float Nav_GearSpeed(uint8_t gear)
{
	switch (gear)
	{
	case 1u:  return MF_M_SPEED(Low_speed);
	case 2u:  return MF_M_SPEED(Middle_speed);
	case 3u:  return MF_M_SPEED(High_speed);
	default:  return 0.0f;
	}
}

/* gear 覆盖需改写段速度，拷入 RAM 缓冲后启动 */
static const mf_seg_t *Nav_GearTable(uint8_t nav_id, uint8_t gear)
{
	static mf_seg_t buf[NAV_GEAR_BUF_LEN];
	const mf_seg_t *src = nav_routes[nav_id];
	uint8_t i;
	float v;

	if (gear == 0u)
	{
		return src;
	}
	v = Nav_GearSpeed(gear);
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

/* 转台对坑：1->123  2->90  3->57，pit=0 不动作 */
static struct
{
	uint8_t  busy;
	uint8_t  pit;
	uint16_t ms;
} tt;
static uint8_t CW_OR_CCW = 1;

static void Nav_Turntable(uint8_t pit)
{
	switch (pit)
	{
	case 1u: Car_data.Motor_Angle = 123.0f; break;
	case 2u: Car_data.Motor_Angle =  90.0f; break;
	case 3u: Car_data.Motor_Angle =  57.0f; break;
	default: return;
	}
	CW_OR_CCW = 1;
	tt.busy = 1u;
	tt.pit = pit;
	tt.ms = 0u;
}

/* REQ_NAV: 若路径索引与当前正在进行的相同，则不做处理；同一索引若已完成，则补发事件；
 * 其余(新 id/STOP 后/FAULT 后)=停旧起新，从段 0 开始 */
static void Nav_DoRequest(uint8_t nav_id, uint8_t gear)
{
	if (nav.state == NAV_RUN && nav.nav_id == nav_id)
	{
		return;
	}
	if (nav.state == NAV_DONE && nav.nav_id == nav_id)
	{
		Car_Event_Send(CAR_EVT_NAV_DONE, nav_id, 0x00u);
		return;
	}
	mf_start_path(mf_app(), Nav_GearTable(nav_id, gear));
	nav.state = NAV_RUN;
	nav.nav_id = nav_id;
	nav.stall_ms = 0u;
}

static void Nav_DoStop(void)
{
	mf_stop(mf_app());
	nav.state = NAV_IDLE;
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
		case CAR_SUB_REQ_NAV:       Nav_DoRequest(a0, a1); break;
		case CAR_SUB_REQ_STOP:      Nav_DoStop(); break;
		case CAR_SUB_REQ_TURNTABLE: Nav_Turntable(a0); break;
		case CAR_SUB_GYRO_ZERO:
			/* 车静止时置零，此刻必在原点(上电流程：扫码->置零->起步) */
			if (nav.state != NAV_RUN)
			{
				ResetHWT101();
				mf_set_pose(mf_app(), 0.0f, 0.0f, 0.0f);
			}
			break;
		default: break;
		}
	}

	if (tt.busy != 0u)
	{
		tt.ms = (uint16_t)(tt.ms + CAR_TASK_MS);
		if (tt.ms >= TURNTABLE_SETTLE_MS)
		{
			tt.busy = 0u;
			Car_Event_Send(CAR_EVT_TURN_DONE, tt.pit, 0x00u);
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
		float d = m->wheel[0].delta_m + m->wheel[1].delta_m;

		if (d < 0.0f)
		{
			d = -d;
		}
		if (d > NAV_STALL_DELTA_M)
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
 * 轮速环: 旧 Kp4000(计数误差->PWM 满幅 1500) 折归一化
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
		.wheel_ki      = 0.02f,
		.wheel_i_max   = 0.75f,
	};
	mf_init(&g_mf, &cfg);
}

//坦克主控函数：TIM4 10ms 中断调用
void Car_Control()
{
	CarSense_Sample();                   /* 采样层: 编码器+航向 -> 快照(seq+1) */
	Car_Nav_Task();                      /* 调度层: 信箱/启停/事件/看门狗 */
	mf_tick(mf_app(), CarSense_Get());   /* 运动库: 纯消费快照, 无新样自动跳过 */
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
