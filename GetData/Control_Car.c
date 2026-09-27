#include "Control_Car.h"
#include "Car_Sense.h"
#include "selftest.h"
#include "mf_math.h"
#include "mf_kinematics.h"

Parameter_TypeDef_Car Car_data;
Yaw_TypeDef Yaw_struct;

/* 张大头驱动器命令地址
   配置错误会导致驱动起执行静默失效*/
#define EMM_MOTOR_ADDR   2u

void Yaw_pid_Init()
{
	Yaw_struct.Yaw_pid.Kp                  = 0.15f;
	Yaw_struct.Yaw_pid.Ki                  = 0.0f;
	Yaw_struct.Yaw_pid.Kd                  = 0.0f;
	Yaw_struct.Yaw_pid.I_Max               = 0.0f;
	Yaw_struct.Yaw_pid.Out_Max             = 0.8f;
	Yaw_struct.Yaw_pid.Out                 = 0.0f;
}

/* 运动调优 (test/motion-tuning): 旧 TURN_ACC(45转/s²≈21.2m/s²) 等效阶跃,
 * 满目标速度 0.94m/s 仅 45ms —— 起步电流浪涌触发电源 BOR 复位(实测 RST=BOR)。
 * 直行/弧线统一降到 5转/s²(≈2.36m/s², 0.4s 拉满 0.94m/s), 削起步浪涌 */
#define SEG_ACC_MPS2  MF_M_ACC(5.0f)

/* nav_id 段表: 原 flags 路线逐段复刻(基线 git 4f3ceb2 true_BOOM_Finish) */
#define SEG_MOVE(v, d)  { .type = MF_SEG_MOVE_DIST, .reset_pid = 1, \
                          .p.move_dist = { (v), (d), MF_KEEP, SEG_ACC_MPS2 } }
/* MOVE 段、但航向目标写死成**绝对角**(不用 MF_KEEP 的推算值) */
#define SEG_MOVE_HD(v, d, hdg) { .type = MF_SEG_MOVE_DIST, .reset_pid = 1, \
                          .p.move_dist = { (v), (d), (hdg), SEG_ACC_MPS2 } }
#define SEG_ARC(v, r)   { .type = MF_SEG_ARC, .reset_pid = 1, \
                          .p.arc = { (v), (r), ((r) > 0.0f ? 90.0f : -90.0f), SEG_ACC_MPS2 } }
/* 重锚转向的到位容差: 0.5°(段默认 1.0° 太松 —— 重锚角是后续各段的航向基准) */
#define SEG_TURN_TO(y)  { .type = MF_SEG_TURN_TO, .reset_pid = 1, \
                          .tol_yaw_deg = 0.5f, \
                          .p.turn = { (y), 0.0f } }
#define SEG_END         { .type = MF_SEG_END }

static const mf_seg_t tbl_qr[] =          /* NAV_TO_QR: flag0 出发首段 43(台架单段用) */
{
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(47.0f)),
	SEG_END
};
static const mf_seg_t tbl_bomb[] =        /* NAV_TO_BOMB: 排爆区去程(末段转弯+末段直线已移入重锚表) */
{
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(47.0f)),
	SEG_ARC(MF_M_SPEED(Low_speed),  TURN_RADIUS_M),   /* 左 90 (flag1) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(17.0f)),
	SEG_ARC(MF_M_SPEED(Low_speed), -TURN_RADIUS_M),   /* 右 90 (flag2) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(49.0f)),
	SEG_ARC(MF_M_SPEED(Low_speed),  TURN_RADIUS_M),   /* 左 90 (flag4) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(39.0f)),
	SEG_ARC(MF_M_SPEED(Low_speed), -TURN_RADIUS_M),   /* 右 90 (flag6) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(49.0f)),
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
static const mf_seg_t tbl_hostage[] =     /* NAV_TO_HOSTAGE: flag13~14 人质区(末段直线+重锚移入 rebase 表) */
{
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(48.5f)),
	SEG_ARC(MF_M_SPEED(Low_speed), -TURN_RADIUS_M),   /* 右 90 (flag14) */
	SEG_END
};
/* 人质对坑前重锚 = 弧线出口先转绝对角、再按绝对航向走末段直线(标定项, 同 NAV_REBASE_BOMB_YAW_DEG) */
#define NAV_REBASE_HOSTAGE_YAW_DEG  (-180.0f)
static const mf_seg_t tbl_rebase_hostage[] = /* NAV_REBASE_HOSTAGE: 弧线出口重锚+末段直线进停点 */
{
	SEG_TURN_TO(NAV_REBASE_HOSTAGE_YAW_DEG),
	SEG_MOVE_HD(MF_M_SPEED(Middle_speed), MF_M_DIST(80.0f), NAV_REBASE_HOSTAGE_YAW_DEG),
	SEG_END
};
/* ---- 返程段 / 终点的绝对航向: 任务终点 YAW 严格 -180° ----
 * -180° 是几何真值: 扫码 GYRO_ZERO 定义出发朝向 0°、首段沿它走 ⇒ -180° = 严格反向。
 * NAV_RETURN_TRIM_DEG 标定: trim = ΔY / 93.2
 *   ΔY = 终点 POSE 的 Y − 标称终点的 Y(两个都是机上 odom 值, 直接相减)
 *   93.2 = 5341mm × π/180 = 这条腿上 1° 航向偏差对应的横向位移
 * 填完跑一趟看实物: 残余偏差变小则对, 变大就翻符号重跑(一次即定)。
 * 推导、±180° 卷绕、完赛区要求: Motion/README.md「返程与终点航向」 */
#define NAV_RETURN_TRIM_DEG        (0.2f)
#define NAV_RETURN_LEG_YAW_DEG     (-180.0f + NAV_RETURN_TRIM_DEG)
#define NAV_RETURN_FINAL_REBASE_EN 1u

static const mf_seg_t tbl_return[] =      /* NAV_TO_RETURN: flag17~18 返回区 */
{
	SEG_MOVE_HD(MF_M_SPEED(Middle_speed), MF_M_DIST(171.0f), NAV_RETURN_LEG_YAW_DEG),
#if NAV_RETURN_FINAL_REBASE_EN
	SEG_TURN_TO(NAV_RETURN_LEG_YAW_DEG),   /* 终点姿态收尾: 注入 -180° 绝对真值 */
#endif
	SEG_END
};
/* 重锚角 = 几何标称 + 台架标定补偿。标定: 车跑到该点、臂对准目标时读 '?' 的 POSE: YAW, 直接填进来;
 * 改了任何影响耗时的行为(ARC 几何 / 标称航向 / 减速 / 停等)后需重标 */
#define NAV_REBASE_BOMB_YAW_DEG     (-90.3f)
/* 返程重锚角 = 几何标称 -180.0 */
#define NAV_REBASE_RETURN_YAW_DEG   (-179.5f) //如果最后有偏，可以改这个

static const mf_seg_t tbl_rebase_bomb[] =   /* NAV_REBASE_BOMB: 排爆区末段(原 tbl_bomb 表尾)+绝对航向重锚。
                                             * 末段转弯与其后的低速直线挪进本表, 与转盘 90° 扫描位并行:
                                             * 本段入口由底盘自行摆码 7(见 nav_entry_turn), 转台在最后一个
                                             * 转弯期间就开始转(原工程 flag9 时点)。此刻臂为收起位,
                                             * 扫掠虽与末段转弯叠加但无干涉; 若目视碍事, 把
                                             * nav_entry_turn[6] 置 0 由主控在 NAV_DONE 之后再发(=串行) */
{
	SEG_ARC(MF_M_SPEED(Low_speed), -TURN_RADIUS_M),      /* 右 90: 原 tbl_bomb 表尾转弯 */
	SEG_MOVE(MF_M_SPEED(Low_speed),    MF_M_DIST(51.5f)),/* 末段低速进作业点 */
	SEG_TURN_TO(NAV_REBASE_BOMB_YAW_DEG),                /* 对坑前重锚: 五个闭环 90° 弧的终点。
	                                                      * 标定项, 含 IMU 累积漂移补偿 —— 见上面
	                                                      * NAV_REBASE_BOMB_YAW_DEG 的说明与重新标定方法 */
	SEG_END
};
static const mf_seg_t tbl_rebase_return[] = /* NAV_REBASE_RETURN: flag17 携人质返程前重锚 */
{
	SEG_TURN_TO(NAV_REBASE_RETURN_YAW_DEG),
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
	tbl_rebase_hostage, /* 8 NAV_REBASE_HOSTAGE */
};
typedef char nav_route_count_check[(sizeof(nav_routes) / sizeof(nav_routes[0])) == CAR_NAV_ID_COUNT ? 1 : -1];

/* NAV_GEAR_BUF_LEN: car_config.h 按 SELFTEST_EN 切换(比赛 12 / 自测 24) */

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

/* speed_gear 覆盖需改写段速度, 拷入 RAM 缓冲后启动; gear=0 直接用 flash 原表。
 * gear=0 一律不拷贝, 让自测与比赛走同一条路径 —— 否则自测结论不可外推到比赛。
 * 代价: 自测插停等段(SEG_PAUSE)必须走拷贝路径, 故自测 gear 不能为 0
 * (见 selftest.c 的 SELFTEST_GEAR, 配错会 #error) */
static const mf_seg_t *Nav_GearTable(uint8_t navigation_target_id, uint8_t speed_gear)
{
	static mf_seg_t buf[NAV_GEAR_BUF_LEN];
	const mf_seg_t *src = nav_routes[navigation_target_id];
	uint8_t i, o;
	float v;

	if (speed_gear == 0u)
	{
		return src;
	}

	v = Nav_GearSpeed(speed_gear);
	o = 0u;
	for (i = 0u; (i < NAV_GEAR_BUF_LEN) && (o < NAV_GEAR_BUF_LEN); i++)
	{
		buf[o] = src[i];
		if (buf[o].type == MF_SEG_END)
		{
			break;
		}
		if (buf[o].type == MF_SEG_MOVE_DIST)
		{
			buf[o].p.move_dist.speed_mps = v;
		}
		else if (buf[o].type == MF_SEG_ARC)
		{
			buf[o].p.arc.speed_mps = v;
		}
		o++;
		SELFTEST_TABLE_POST(src, i, buf, &o, NAV_GEAR_BUF_LEN);
	}
	/* 兜底: 缓冲耗尽时截断, 必须以 END 结束(否则段表会越过缓冲读飞) */
	if (o >= NAV_GEAR_BUF_LEN)
	{
		o = (uint8_t)(NAV_GEAR_BUF_LEN - 1u);
	}
	buf[o].type = MF_SEG_END;
	return buf;
}

/* ================== 调度层状态（panzer_links_protocol.md 4.5） ================== */
#define CAR_TASK_MS            10u
#define NAV_STALL_MS           3000u
#define NAV_STALL_DELTA_M      0.001f
/* 转台 DONE 计时: 开环静置, 按本次行程 |本次目标−上次目标| × TURNTABLE_MS_PER_DEG 算, 只增不减。
 * TURNTABLE_MS_PER_DEG 标定: 上机敲桶位码 4/5(270°), 录像量"发令→实际停稳" t, 置 t/270 */
#define TURNTABLE_MS_PER_DEG   1.9f    /* 额定 180RPM ⇒ 0.93ms/°; 旧值 ×2 计"100ms 重发重启
                                        * 梯形规划"欠速——2026-09 目标去重后单发不再重启, 1.9 纯作
                                        * 偏长裕量(报早=伸臂干涉), 可按下行标定法回收时间 */
#define TURNTABLE_RAMP_MARGIN  300u    /* 加减速 + 机械/传动裕量 */
#define TURNTABLE_SETTLE_MIN   1000u   /* 只增不减: 旧固定值, 相对旧行为零回退 */
#define TURNTABLE_SETTLE_MAX   3000u   /* 上限: 旧基线值, 防异常行程把流程拖死 */

typedef enum
{
	NAV_IDLE = 0,   /* 上电/REQ_STOP/FAULT 后，任意 REQ_NAV 均从段 0 开始 */
	NAV_RUN,        /* 路径执行中（mf path RUN） */
	NAV_DONE,       /* 已完成，同 id 重发只补发 NAV_DONE */
} NavState_t;

/* nav_id 的"无"哨兵。必须 != 任何合法 nav_id(0~8): nav 是 static 零初始化,
 * 用 0 当哨兵会让上电后第一条 REQ_NAV(0) 被误判成"已跑完" */
#define NAV_ID_NONE  0xFFu

static struct
{
	NavState_t state;
	uint8_t    nav_id;
	uint8_t    chain_to;    /* 本表跑完将自动续跑的 nav_id; NAV_ID_NONE=不续 */
	uint8_t    last_done;   /* 已跑完但因链续未进 NAV_DONE 态的 nav_id; NAV_ID_NONE=无 */
	uint16_t   stall_ms;
} nav;

static void Nav_Fault(void);   /* 前置声明: Nav_DoRequest 段表校验失败即调(定义在后) */
static void Tracking_RaiseFault(uint8_t reason);   /* 前置声明: Tracking_Start 无段表时调 */

/* ---- 打靶段内跟瞄 v3.0: 刹停 / 续跑 ----
 * 段表全程是**唯一**位移来源(跟瞄态一个字节的 vx/wz 都不写); 视觉只回答"是否已到靶位"。
 * 中控发 TRACKING_START 武装(ARMED), 底盘每拍只**评估**刹停判据; 判据成立则按固定减速
 * 度把段表当拍目标速度压到 0(不是瞬间压零 -> 滑行更短), 进 STOPPED 暂停本段并报 DONE;
 * 中控发射后发 TRACKING_STOP(1) 回 OFF, 段表从 cur_speed=0 起坡补完剩余里程, 终点仍
 * 落在"进段位置 + 段表里程"上, 故零标定、零污染。规格见 docs/打靶段架构-刹停与续跑.md */
typedef enum
{
	TRACKING_OFF = 0,   /* 不干预: 段表全权 */
	TRACKING_ARMED,     /* 已武装: 段表全权驱动, 只评估刹停判据, 不改 cmd */
	TRACKING_STOPPED,   /* 已刹停(暂停本段): cmd 全零, 剩余里程冻结, 等 0x07 STOP(1) */
	TRACKING_FAULT      /* 本地判定瞄不上: 全零 + 上报 0x06, 等 START 复位 */
} TrackingState_t;

static struct
{
	TrackingState_t state;
	float    horizontal_error_normalized;  /* 最新归一化横向误差, 约 -1~+1 */
	uint8_t  target_found_flag;            /* 最新有效标志 */
	uint16_t frame_age_ms;                 /* 距上次有效误差帧时长 */
	uint16_t hold_ms;                      /* J1 兜底: |e|≤阈 的连续累计 */
	int8_t   zero_sign;                    /* 已确认误差符号(+1/-1/0=未确认), 过零判据的基准 */
	int8_t   sign_cand;                    /* 去抖候选符号 */
	uint8_t  confirm_cnt;                  /* 去抖: 候选符号连续同号帧数 */
	uint8_t  braking;                      /* 已触发制动的锁存: 不再重判, 只走制动斜坡 */
	float    brake_cap;                    /* 制动斜坡: 当拍允许的段表目标速度上限 */
	uint16_t noconv_ms;                    /* 段表已完 + |e|>阈 的连续累计(FAULT 01) */
	uint8_t  noconv_latch;                 /* FAULT(01) 闩锁: 防重复上报, START/STOP 清 */
	uint16_t lost_ms;                      /* 段表已完 + 无测量的连续累计(FAULT 02) */
	uint8_t  stop_done;                    /* STOPPED 已报 TRACKING_DONE 的闩锁(START 清, 补发用) */
	uint8_t  fault_reason;                 /* TRACKING_FAULT 的 reason, 见 Control_Car.h TRK_FAULT_* */
} tracking;

/* 状态迁移唯一入口: 复位判据计数器 + 登记一次迁移打点(主循环 Car_TrkChange_Log 消费)。
 * 只走这里是为了让"哪一拍切的态、为什么切"有据可查, 也防止某条路径漏清计数 */
static volatile uint8_t trk_chg_from, trk_chg_to, trk_chg_fresh;

static void Tracking_Goto(TrackingState_t next)
{
	TrackingState_t from = tracking.state;

	if (from == next)
	{
		return;
	}
	tracking.state = next;
	tracking.hold_ms = 0u;
	tracking.zero_sign = 0;
	tracking.sign_cand = 0;
	tracking.confirm_cnt = 0u;
	tracking.braking = 0u;
	tracking.brake_cap = 0.0f;
	tracking.stop_done = 0u;
	tracking.noconv_ms = 0u;
	tracking.lost_ms = 0u;
	/* 注意**不清** noconv_latch —— 它由 START/STOP 专门清(见 Tracking_Start/Stop),
	 * 好让 Brain 的容错计数能在每次重开后重新累计 */
	__disable_irq();
	trk_chg_from = (uint8_t)from;
	trk_chg_to = (uint8_t)next;
	trk_chg_fresh = 1u;
	__enable_irq();
}

static char Tracking_StateChar(TrackingState_t s)
{
	switch (s)
	{
	case TRACKING_ARMED:   return 'A';
	case TRACKING_STOPPED: return 'S';
	case TRACKING_FAULT:   return 'F';
	default:                return 'O';
	}
}

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

/* 转台下发门控: 上电后首条 TURNTABLE 命令才允许 Emm 下发, 急停后重新冻结。
 * 防两类失控: 1) 仅 MCU 复位(驱动器带电)时自动甩回旧目标; 2) 急停后 TIM5
 * 继续周期下发驱动转台 */
static uint8_t tt_arm = 0u;

/* 目标去重: Emm_V5 每收一条 0xFD 位置帧就重启梯形规划, 周期重发→"起步-重启"锯齿、
 * 转台抖动到不了巡航速度; 改为目标 key(方向+脉冲数)变化才单发, 到位后停发。
 * 根因细节/丢帧鲁棒性/重新武装需失效去重: 见 docs/坑-机械臂与转台.md §1 */
#define TT_KEY_UNSENT  0xFFFFFFFFu   /* 哨兵: 合法 key ≤ 0x8004FAC0(dir位|tick), 不会撞上 */
static uint32_t tt_sent_key = TT_KEY_UNSENT;

/* 重新武装(tt_arm 0->1)必须同时失效去重: 急停('x'/REQ_STOP)后驱动器停在半路,
 * 同码再摆(如再敲 '7' 或段入口回正)时 key 虽未变, 也需要一条全新的位置帧把
 * 电机带回目标——不能因"目标没变"而跳过 */
static void Tt_Arm(void)
{
	tt_arm = 1u;
	tt_sent_key = TT_KEY_UNSENT;
}

/* 转台码->定点角: 对坑 1->123 2->90 3->57, 对桶 4/5->270 6->90, 7 回正->90, 0 不动作 */
/* TURN_DONE 在置 busy 后按行程动态计时上报(Nav_TurnSettle), 不校验 Emm 应答
 * 与到位状态; 若现场出现"未到位即动作", 进一步改法: 轮询 Emm 状态字
 * (S_FLAG 到位位)确认到位后再报 TURN_DONE */
static struct
{
	uint8_t  busy;
	uint8_t  code;      /* 本次请求码, TURN_DONE 原样回显 */
	uint16_t ms;
	uint16_t settle_ms; /* 本次 DONE 计时目标: 按行程动态算(见 Nav_TurnSettle/常量注释) */
} tt;
static uint8_t CW_OR_CCW = 1;
/* 最近一次**实际下发**的脉冲数(仅台架标定位置当量用): '?' 的 WTBL 行 TICK= 回显。
 * 有了它就不用从"角度×当量"去反推下发量, 直接在台架上用
 * 实测转角 θ 解出真当量: WTBL_TICK_PER_DEG_true = TICK / θ */
static uint32_t tt_last_tick = 0u;

/* 位置当量标定: 冷机上电敲 '7'(目标 90°)量实际转角 θ, 必须 ≈90°。
 * θ≈4 圈以上 = 驱动器 MStep 没存成 256(仍是 16), 回查驱动器菜单重设, 不要改这里。
 * 落角偏差 >2° 时按 新系数 = 1208.888 × 90 / θ 微调。驱动器侧参数不动 */
#define WTBL_TICK_PER_DEG   1208.888f   /* = 51200 × 8.5 / 360 (MStep=256 + 转盘减速比 8.5,
                                         * 与 2025 原工程实车值一致; 转台驱动器曾为 16 细分, 那档是 67.16) */

/* 转台上电零点对齐: 上电发 0x0A 清零驱动器计数器(电机不动), MCU 记账角置 TURNTABLE_BOOT_DEG。
 * 标定: 上电敲 '7'(目标 90°)量实际转角 θ —— θ≠90° 说明上电姿态不是 0°, 该机械摆正, 不要改这个宏 */
#define TURNTABLE_BOOT_DEG  0.0f

/* 本次转台 DONE 计时(ms): 按行程动态算, 只增不减(下限=旧固定值 1000ms)。
 * 行程 = |本次目标角 − 上次目标角| —— 绝对定位下, 电机上次就该停在旧目标上。
 * 方向约定"宁可偏长": 报早了 = 主控在转台未停稳时就伸臂/放腕(机构干涉);
 * 报晚了只多花几百毫秒。常量与标定方法见上方 TURNTABLE_MS_PER_DEG 注释 */
static uint16_t Nav_TurnSettle(float prev_deg, float new_deg)
{
	float    delta = new_deg - prev_deg;
	uint32_t ms;

	if (delta < 0.0f)
	{
		delta = -delta;
	}
	ms = (uint32_t)(delta * TURNTABLE_MS_PER_DEG) + TURNTABLE_RAMP_MARGIN;
	if (ms < TURNTABLE_SETTLE_MIN)
	{
		ms = TURNTABLE_SETTLE_MIN;
	}
	if (ms > TURNTABLE_SETTLE_MAX)
	{
		ms = TURNTABLE_SETTLE_MAX;
	}
	return (uint16_t)ms;
}

/* 转台定点角: 排爆/人质摆放距离一致, 惯导保证车位, 按码打固定角。
 * 对坑 1~3->123/90/57; 对桶 4~6->270/270/90(入桶摆位沿用实车验证值,
 * 坑3 反向 CW 到 90, 其余 CCW); 7 回正 90。
 * 先写方向后写角度 + tt_arm 先关后开: TIM5 被抢占恢复后读到的一定是
 * 同一对(方向,角度), 不会出现"新角度+旧方向"挥向另一侧 */

static void Nav_Turntable(uint8_t turn_code)
{
#if !TURNTABLE_ENABLE
	/* 转台冻结(TURNTABLE_ENABLE=0, 由 car_config.h 随 SELFTEST_EN 切换):
	 * tt_arm 恒 0, TIM5 不下发任何位置帧 */
	(void)turn_code;
	return;
#endif
	uint8_t dir = 1u;
	float   angle;
	float   prev_deg = Car_data.Motor_Angle;   /* 绝对定位: 上次目标=电机应已在的位置 */

	tt_arm = 0u;                     /* 更新期间冻结 TIM5 重发 */
	switch (turn_code)
	{
	case 1u: angle = 123.0f; break;
	case 2u: angle =  90.0f; break;
	case 3u: angle =  57.0f; break;
	case 4u:
	case 5u: angle = 270.0f; break;
	case 6u: angle =  90.0f; dir = 0u; break;
	case 7u: angle =  90.0f; break;
	default: return;
	}
	CW_OR_CCW = dir;
	Car_data.Motor_Angle = angle;
	Tt_Arm();                        /* 置 tt_arm + 失效目标去重(见 Tt_Arm 注释) */
	tt.busy = 1u;
	tt.code = turn_code;
	tt.ms = 0u;
	tt.settle_ms = Nav_TurnSettle(prev_deg, angle);   /* 按本次行程定 DONE 计时 */
	Car_Dbg2_TtRecord(1u, turn_code, angle);
}

/* 段入口自动转台: 启动该 nav_id 段时底盘自行摆到指定码位(0=不动); 现仅 NAV_REBASE_BOMB(6) -> 码 7 */
static const uint8_t nav_entry_turn[CAR_NAV_ID_COUNT] =
{
	0u, 0u, 0u, 0u, 0u, 0u, 7u, 0u, 0u,   /* 仅 NAV_REBASE_BOMB(6) -> 回正 90° */
};

/* 段表自动链续: 该 nav 跑完由底盘自行续跑下一条(0=不续, 等主控下发)。
 * 自测形态必须置 0: SELFTEST_NEXT_NAV 与它同在"跑完一条"处接管, 同开会双触发。
 * 续表一律用 flash 原表 nav_routes[], 不走 RAM 缓冲(会把在跑 path 指针弄悬空)。
 * 拆表代价(表尾硬减速/跨表空窗)与链续收益: 见 docs/坑-运动与导航.md §5 */
static const uint8_t nav_chain_next[CAR_NAV_ID_COUNT] =
{
	0u, NAV_CHAIN_1_TO, 0u, 0u, NAV_CHAIN_4_TO, 0u, 0u, 0u, 0u,
};

/* REQ_NAV 幂等: 同 id 进行中忽略 / 已完成补发 DONE / 其余停旧起新 */
static void Nav_DoRequest(uint8_t navigation_target_id, uint8_t speed_gear)
{
	uint8_t chain_to;

	if (nav.state == NAV_RUN && nav.nav_id == navigation_target_id)
	{
		/* 同段进行中: 幂等忽略。跟瞄的 ARMED/STOPPED 都**不许**被主控的幂等重发打断
		 * (重发 REQ_NAV 若把 STOPPED 放走, 段表立刻把车从靶心拖走, 激光就打偏了) */
		return;
	}
	if (nav.state == NAV_DONE && nav.nav_id == navigation_target_id)
	{
		Car_Event_Send(CAR_EVT_NAV_DONE, navigation_target_id, 0x00u);
		return;
	}
	/* 链续后旧 id 的重发: 该表确实已跑完, 只是 DONE 帧可能丢了 —— 补发而不重跑,
	 * 否则"主控超时重发 REQ_NAV(1)"会走到下面的停旧起新、把 tbl_bomb 从头再跑一遍 */
	if (nav.last_done == navigation_target_id)
	{
		Car_Event_Send(CAR_EVT_NAV_DONE, navigation_target_id, 0x00u);
		return;
	}

	chain_to = ((navigation_target_id < CAR_NAV_ID_COUNT)
	            && (nav_chain_next[navigation_target_id] != 0u))
	         ? nav_chain_next[navigation_target_id] : NAV_ID_NONE;
	mf_start_path_chain(mf_app(), Nav_GearTable(navigation_target_id, speed_gear),
	                    (chain_to != NAV_ID_NONE) ? nav_routes[chain_to] : 0);
	nav.state = NAV_RUN;
	nav.nav_id = navigation_target_id;
	nav.chain_to = chain_to;
	nav.last_done = NAV_ID_NONE;
	nav.stall_ms = 0u;
	/* 起新段清跟瞄: 防上一段残留的 STOPPED/FAULT 压零带到新路段 */
	Tracking_Goto(TRACKING_OFF);
	if (mf_path_get_state(&mf_app()->path) != MF_PATH_RUN)
	{
		/* 段表启动校验未过(如 ARC 半径/符号非法): 立即按进度看门狗原因
		 * 上报 NAV_FAULT, 不静默空转等 3s 里程看门狗兜底 */
		Nav_Fault();
		return;   /* 本段没起来: 段入口的转台动作也不做 */
	}

	/* 段入口自动转台(见表注释): 与段表同拍下发, 与行驶并行 */
	if ((navigation_target_id < CAR_NAV_ID_COUNT) && (nav_entry_turn[navigation_target_id] != 0u))
	{
		Nav_Turntable(nav_entry_turn[navigation_target_id]);
	}
}

/* 急停冻结转台: 弃用在途位置帧(DMA 中途可被共享 rxb 的 memset 撕裂, 且
 * Emm_V5 内部 HAL_UART_Transmit_DMA 遇 BUSY 会静默吞掉停止帧), 改用局部
 * 缓冲阻塞直发(5B@115200≈0.43ms, 仅 STOP 时一次)。残窗: TIM5 恰在
 * "tt_arm 检查后、发送前"被抢占时, 恢复后会多发一帧旧位置——之后无重发,
 * 电机走完该目标即停, 与旧行为相当 */
static void Turntable_EmergencyStop(void)
{
	uint8_t stop_cmd[5] = {EMM_MOTOR_ADDR, 0xFEu, 0x98u, 0x00u, 0x6Bu};
	uint8_t try_n;

	/* 在途 TX 被优先级更高的 TIM5 位置帧占用时首停必 HAL_BUSY: Abort+阻塞发送
	 * 最多试 3 次(每次 abort 清 TX 锁), 仍失败只能放弃——电机会走到旧目标 */
	for (try_n = 0u; try_n < 3u; try_n++)
	{
		HAL_UART_AbortTransmit(&huart6);
		if (HAL_UART_Transmit(&huart6, stop_cmd, sizeof(stop_cmd), 10u) == HAL_OK)
		{
			break;
		}
	}
}

static void Nav_DoStop(void)
{
	mf_stop(mf_app());
	nav.state = NAV_IDLE;
	nav.last_done = NAV_ID_NONE;
	nav.chain_to = NAV_ID_NONE;
	Tracking_Goto(TRACKING_OFF);
	/* 急停覆盖转台: 停电机 + 冻结 TIM5 重发 + 作废未决 TURN_DONE
	 * (协议头 REQ_STOP 语义: 急停安全停车, 清执行状态) */
	tt_arm = 0u;
	tt.busy = 0u;
	Turntable_EmergencyStop();
	Car_Dbg2_TtRecord(0u, tt.code, Car_data.Motor_Angle);
}

/* ---- 台架直转调试(USART2 'w'/'e'/'1'~'7'/'x', 经 Car_DebugTask 主循环执行):
 * 绕过 UART4 协议与状态机, 单独验证 TIM5 重发 -> UART6 DMA -> 驱动器链路。
 * 微调不置 tt.busy —— 台架期间不发 TURN_DONE 事件, 不会误喂 Brain 状态机 */
void Car_Turntable_BenchNudge(float delta_deg)
{
#if !TURNTABLE_ENABLE
	(void)delta_deg;   /* 转台冻结: 台架微调亦不开电机 */
	return;
#endif
	__disable_irq();
	tt_arm = 0u;
	Car_data.Motor_Angle += delta_deg;
	Tt_Arm();                        /* 置 tt_arm + 失效目标去重 */
	__enable_irq();
	Car_Dbg2_TtRecord(2u, 0u, Car_data.Motor_Angle);
}

void Car_Turntable_BenchCode(uint8_t turn_code)
{
	Nav_Turntable(turn_code);   /* 走协议同款码表, 含 TT 事件登记 */
	tt.busy = 0u;               /* 台架不播出 TURN_DONE: 挂着链路时不误推进 Brain */
}

void Car_Turntable_BenchStop(void)
{
	tt_arm = 0u;
	tt.busy = 0u;
	Turntable_EmergencyStop();
	Car_Dbg2_TtRecord(0u, tt.code, Car_data.Motor_Angle);
}

/* ---- 段内跟瞄(v3.0): 段表不断; 视觉只当"该刹停"的判据, 不写 vx/wz ---- */
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

/* 收到 TRACKING_START: 武装跟瞄(段表全权, 底盘只评估刹停判据)。
 * v3.0 下 START 是**无状态标志** —— 重发不改变段表状态、不清测量、不换态, 天然幂等;
 * 从 FAULT 重收 START = 主控要求重开一轮(清未收敛闩锁, 好让容错计数能再累计)。
 * a0 旧 speed_level / a1 aim_axis 都不再参与逻辑; aim_axis 仅做入参范围校验 */
static void Tracking_Start(uint8_t tracking_speed_level, uint8_t aim_axis)
{
	mf_ctx_t *motion = mf_app();
	mf_path_state_t path_state = mf_path_get_state(&motion->path);

	(void)tracking_speed_level;   /* v3.0: 进近低速由咬靶区自动决定, 该档位不再生效 */

	if ((aim_axis != TRK_AXIS_ROTATE) && (aim_axis != TRK_AXIS_TRANSLATE))
	{
		return;   /* 越界: 与协议层(interurp.c 的 NACK 0x03)同口径, 不武装 */
	}
	if (path_state == MF_PATH_IDLE)
	{
		Tracking_RaiseFault(TRK_FAULT_NO_PATH);   /* 无活动段表: 无从驱动, 报故障 */
		return;
	}

	/* STOPPED 内重发 START: **必须保持刹停** —— 回 OFF 会让段表立刻把车从靶心拖走。
	 * 只清 DONE 闩锁: 主控的 done 标志被这次 START 清零了, 下拍 STOPPED 分支补发一次 */
	if (tracking.state == TRACKING_STOPPED)
	{
		tracking.stop_done = 0u;
		return;
	}
	/* 已 ARMED 重发(主控 ALIGN 入口/align_ms 超时补发都会发): 幂等忽略, 不打断判据 */
	if (tracking.state == TRACKING_ARMED)
	{
		return;
	}

	tracking.frame_age_ms = 0u;
	tracking.target_found_flag = 0u;
	tracking.horizontal_error_normalized = 0.0f;
	tracking_error_pending = 0u;    /* 丢弃上轮残帧, 首拍不等旧误差 */
	tracking.noconv_latch = 0u;     /* START/STOP 清未收敛闩锁, 允许 Brain 的容错计数继续累计 */
	Tracking_Goto(TRACKING_ARMED);
}

/* 立即暂停本段并上报"已刹停可发射": cmd 全零 + 泄放段内斜坡 -> STOPPED, 发一次 DONE */
static void Tracking_EnterStopped(mf_ctx_t *motion)
{
	Tracking_HoldStill(motion);
	Tracking_Goto(TRACKING_STOPPED);   /* Goto 含清 stop_done, 故闩锁在 Goto 之后置 */
	tracking.stop_done = 1u;
	Car_Event_Send(CAR_EVT_TRACKING_DONE, 0u, 0x00u);
}

/* 触发制动(锁存): 记下当拍速度作为制动斜坡起点。制动不是瞬间压零 —— 段表按固定减速
 * 度把目标速度压到 0, 滑行更短、停点可复现(规格 §5) */
static void Tracking_BrakeStart(mf_ctx_t *motion)
{
	tracking.braking = 1u;
	tracking.brake_cap = motion->path.cur_speed;
}

/* 制动斜坡一拍: 把 brake_cap 沿固定减速度往 0 收(带符号, 倒车段也适用), 并只**往下**压
 * 段表当拍目标速度。返回 1 = 已到零, 该进 STOPPED。必须在 mf_tick **之后**调用:
 * 本拍压的是"下一拍 mf_tick 起坡的起点", 故本拍不改 cmd(轮速环不重算) */
static uint8_t Tracking_BrakeRamp(mf_ctx_t *motion)
{
	const float step = TRACKING_BRAKE_MPS2 * ((float)CAR_TASK_MS * 0.001f);

	if (tracking.brake_cap > 0.0f)
	{
		tracking.brake_cap = (tracking.brake_cap > step) ? (tracking.brake_cap - step) : 0.0f;
	}
	else if (tracking.brake_cap < 0.0f)
	{
		tracking.brake_cap = (tracking.brake_cap < -step) ? (tracking.brake_cap + step) : 0.0f;
	}
	/* 只往下限: 段表当拍可能已把这值往目标抬了 acc·dt, 这里再压回去 */
	if (((tracking.brake_cap >= 0.0f) && (motion->path.cur_speed > tracking.brake_cap)) ||
	    ((tracking.brake_cap <= 0.0f) && (motion->path.cur_speed < tracking.brake_cap)))
	{
		motion->path.cur_speed = tracking.brake_cap;
	}
	return (uint8_t)((tracking.brake_cap <= 0.02f) && (tracking.brake_cap >= -0.02f));
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

/* 是否已进入"咬靶区" = 本段最后 TRACKING_SLOW_ZONE_M 米(按剩余里程判, 与行进方向无关)。
 * 降速与刹停判据**共用**这一条门槛: 段起点或离靶尚远处的 found/e 可能是误检/锁错目标,
 * 不拦就会"起步即刹停"(2026-09-23 现场事故 1d99420 的同款症状), 且会把车从起步压在低速上 */
static uint8_t Tracking_InSlowZone(const mf_ctx_t *motion)
{
	return (uint8_t)(mf_path_remain_m(&motion->path) <= TRACKING_SLOW_ZONE_M);
}

/* 进近固定低速(零系数): 在 mf_tick **之前**调用, 只压 path.cur_speed —— 这样 mf_tick
 * 里的轮速环每拍只算一次(不像 Tracking_RecomputeWheels 会把轮速 PI 的 I 项一拍积两次)。
 * 为什么必须降速: 停点精度 ≈ 帧间隔×车速 + 制动距离 —— 0.94m/s 时制动距离 0.22m(直接冲
 * 过靶位), 0.20m/s 时仅 10mm。进咬靶区后**不看测量**(视觉挂了也保持低速走完), 分支更少 */
static void Tracking_LimitApproachSpeed(void)
{
	mf_path_ctx_t *path = &mf_app()->path;

	if (tracking.state != TRACKING_ARMED)
	{
		return;   /* 只武装期限速 */
	}
	if (Tracking_InSlowZone(mf_app()) == 0u)
	{
		return;   /* 还没进咬靶区: 段表全速 */
	}
	if (path->cur_speed > TRACKING_SLOW_MPS)
	{
		path->cur_speed = TRACKING_SLOW_MPS;   /* 只往下限: 倒车段不会被误抬速 */
	}
}

/* TRACKING_STOP: 0=就地刹停(立即暂停本段) / 1=放行段表续跑完本段。
 * 1 是"开激光后恢复行进"的放行口: STOPPED 下段表只被压零、里程没走完, 回 OFF 后从
 * cur_speed=0 起坡把剩余里程补完, 终点仍落在名义终点(不变量 I3: 暂停可逆) */
static void Tracking_Stop(uint8_t tracking_stop_mode)
{
	tracking.noconv_latch = 0u;   /* 清未收敛闩锁, 与 START 同口径 */
	if (tracking_stop_mode == 0u)
	{
		if (tracking.state != TRACKING_STOPPED)   /* 已在 STOPPED 就别重复广播 DONE */
		{
			Tracking_EnterStopped(mf_app());
		}
	}
	else
	{
		Tracking_Goto(TRACKING_OFF);
	}
}

/* 本地判定瞄不上: 锁停 + 上报 0x06 FAULT, 等主控决策(重试/容错) */
static void Tracking_RaiseFault(uint8_t reason)
{
	tracking.fault_reason = reason;
	Tracking_Goto(TRACKING_FAULT);
	Tracking_HoldStill(mf_app());
	Car_Event_Send(CAR_EVT_TRACKING_FAULT, reason, 0x00u);
}

/* 每拍在 mf_tick 之后调用。返回 1 = 本拍改写了 cmd(须重算轮速环); 0 = 不干预(段表全权)。
 * 跟瞄态**只评估判据**, 不改 vx/wz(不变量 I1: 位移唯一来源是段表) */
static uint8_t Tracking_ApplyOverlay(void)
{
	mf_ctx_t *motion = mf_app();
	float e, abs_e;
	uint8_t fresh, done;

	if (tracking.state == TRACKING_OFF)
	{
		return 0u;                     /* 段表全权(不变量 I1) */
	}
	/* 已刹停/故障: 全零驻停, 等主控(0x07/0x05)。STOPPED 只报一次 DONE; START 重发会
	 * 清 stop_done 闩锁 => 下拍补发一次(主控的 done 标志被那次 START 清掉了) */
	if ((tracking.state == TRACKING_STOPPED) || (tracking.state == TRACKING_FAULT))
	{
		Tracking_HoldStill(motion);
		if ((tracking.state == TRACKING_STOPPED) && (tracking.stop_done == 0u))
		{
			tracking.stop_done = 1u;
			Car_Event_Send(CAR_EVT_TRACKING_DONE, 0u, 0x00u);   /* 已刹停可发射 */
		}
		return 1u;
	}

	/* ==== ARMED: 段表全权驱动, 这里只评估刹停判据/上报故障, 不改 cmd ==== */
	Tracking_ConsumeErrorFrame();
	if (tracking.frame_age_ms < (uint16_t)(0xFFFFu - CAR_TASK_MS))
	{
		tracking.frame_age_ms = (uint16_t)(tracking.frame_age_ms + CAR_TASK_MS);
	}
	fresh = (uint8_t)((tracking.target_found_flag != 0u) &&
	                  (tracking.frame_age_ms <= TRACKING_FRAME_TIMEOUT_MS));
	done = mf_is_done(motion);
	e = tracking.horizontal_error_normalized;
	abs_e = (e < 0.0f) ? -e : e;

	/* ---- 制动中(已触发, 锁存): 只把段表当拍目标速度沿固定减速度压到 0 ----
	 * 不再重判判据 —— 减速期间车还在动, 否则会反复触发/失效。到零即进 STOPPED */
	if (tracking.braking != 0u)
	{
		if (Tracking_BrakeRamp(motion) != 0u)
		{
			Tracking_EnterStopped(motion);
			return 1u;
		}
		return 0u;   /* 本拍只压段表 target, 制动下一拍 mf_tick 起坡生效, cmd 未改 */
	}

	/* ---- 无测量: 判据休眠(清计数, 不动作); 段表已完且持续无测量 -> FAULT(02) ---- */
	if (fresh == 0u)
	{
		tracking.hold_ms = 0u;
		if (tracking.frame_age_ms > TRACKING_FRAME_TIMEOUT_MS)
		{
			/* 只有真的测量中断才把过零基准清零; 单帧丢检(found=0 但帧不老)保留基准 ——
			 * 否则过零恰好落在那一帧就会被永久漏掉(基准被清后新符号只能当基准, 不再触发) */
			tracking.zero_sign = 0;
			tracking.sign_cand = 0;
			tracking.confirm_cnt = 0u;
		}
		if (done != 0u)
		{
			tracking.lost_ms = (uint16_t)(tracking.lost_ms + CAR_TASK_MS);
			if (tracking.lost_ms >= TRACKING_LOST_DONE_MS)
			{
				Tracking_RaiseFault(TRK_FAULT_NO_TARGET);   /* 02: 段表已完仍丢目标 */
				return 1u;
			}
		}
		else
		{
			tracking.lost_ms = 0u;
		}
		return 0u;   /* 段表未完不停车: 停了就再也看不到靶(死锁) */
	}
	tracking.lost_ms = 0u;

	/* ---- J2 过零(主判据): 符号翻转那一拍 = 车刚从靶心旁经过 ----
	 * 去抖: 连续 TRACKING_ZERO_CONFIRM_N 帧同号才认; 死区挡小噪声 */
	{
		int8_t s = (e > TRACKING_ZERO_DEADBAND) ? 1 : ((e < -TRACKING_ZERO_DEADBAND) ? -1 : 0);

		if (s != 0)
		{
			if (s == tracking.sign_cand)
			{
				if (tracking.confirm_cnt < 0xFFu) { tracking.confirm_cnt++; }
			}
			else
			{
				tracking.sign_cand = s;
				tracking.confirm_cnt = 1u;
			}
			if (tracking.confirm_cnt >= TRACKING_ZERO_CONFIRM_N)
			{
				if ((tracking.zero_sign != 0) && (s != tracking.zero_sign) &&
				    (Tracking_InSlowZone(motion) != 0u))
				{
					Tracking_BrakeStart(motion);   /* 刚经过靶心: 立即刹停 */
				}
				/* 过零后**立刻**更新已确认符号: 让"过零"严格等于"符号翻转那一拍"。
				 * 不更新则旧符号一直留着, "s != zero_sign" 会在随后每一拍都成立,
				 * 过零从一次性事件退化成持续电平(详见交付报告 §3) */
				tracking.zero_sign = s;
			}
		}
	}

	/* ---- J1 兜底: 阈值 + 驻留(整段 e 都不换号时兜住) ----
	 * 与 J2 同门槛: 只在咬靶区内计数 —— 否则段起点的误检(锁错目标 ⇒ e≈0)会把车从起步刹死 */
	if ((abs_e <= TRACKING_ERROR_THRESHOLD) && (Tracking_InSlowZone(motion) != 0u))
	{
		tracking.hold_ms = (uint16_t)(tracking.hold_ms + CAR_TASK_MS);
		if (tracking.hold_ms >= TRACKING_HOLD_TIME_MS)
		{
			Tracking_BrakeStart(motion);
		}
	}
	else
	{
		tracking.hold_ms = 0u;
	}

	/* ---- 未收敛 FAULT(01): 段表已完 + |e| 持续超阈; 闩锁防重复上报 ---- */
	if ((done != 0u) && (abs_e > TRACKING_ERROR_THRESHOLD))
	{
		tracking.noconv_ms = (uint16_t)(tracking.noconv_ms + CAR_TASK_MS);
		if ((tracking.noconv_latch == 0u) && (tracking.noconv_ms >= TRACKING_NOCONV_MS))
		{
			tracking.noconv_latch = 1u;
			Tracking_RaiseFault(TRK_FAULT_NOT_CONVERGED);   /* 01: 到位后长时间未收敛 */
			return 1u;
		}
	}
	else
	{
		tracking.noconv_ms = 0u;
	}

	return 0u;   /* ARMED 不写 vx/wz: 段表全权 */
}

static void Nav_Fault(void)
{
	mf_stop(mf_app());
	nav.state = NAV_IDLE;
	nav.last_done = NAV_ID_NONE;
	nav.chain_to = NAV_ID_NONE;
	Car_Event_Send(CAR_EVT_NAV_FAULT, nav.nav_id, CAR_FAULT_WATCHDOG);
}

void Car_Nav_Task(void)
{
	static uint8_t inited = 0;
	mf_ctx_t *m;

	if (inited == 0)
	{
		mf_app_init();
		nav.chain_to = NAV_ID_NONE;     /* static 零初始化的 0 是合法 nav_id, 必须显式置无 */
		nav.last_done = NAV_ID_NONE;
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
		case CAR_SUB_TRACKING_START:    Tracking_Start(a0, a1); break;
		case CAR_SUB_TRACKING_STOP:     Tracking_Stop(a0); break;
		case CAR_SUB_GYRO_ZERO:
			/* 车静止时置零，此刻必在原点(上电流程：扫码->置零->起步) */
			if (nav.state != NAV_RUN)
			{
				ResetHWT101();
				mf_set_pose(mf_app(), 0.0f, 0.0f, 0.0f);
			}
			Tracking_Goto(TRACKING_OFF);
			break;
		default: break;
		}
	}
	/* 误差帧每拍消费一次 */
	Tracking_ConsumeErrorFrame();

	if (tt.busy != 0u)
	{
		tt.ms = (uint16_t)(tt.ms + CAR_TASK_MS);
		if (tt.ms >= tt.settle_ms)   /* 本次按行程算出的 DONE 计时, 见 Nav_TurnSettle */
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

	/* 段表链续: 底盘已自行跑进续表, 但对外仍要按旧 id 上报 NAV_DONE, 并把 nav_id 切到
	 * 续表 —— 于是主控随后发来的 REQ_NAV(续表) 只会被幂等忽略, 不会打断行驶。
	 * 转台码 7 在此触发, 时点与"主控下发 nav 6 才触发"一致(都在末段转弯的入口) */
	if ((nav.chain_to != NAV_ID_NONE) && (mf_pop_chain(m) != 0u))
	{
		uint8_t prev_id = nav.nav_id;

		nav.last_done = prev_id;
		nav.nav_id = nav.chain_to;
		nav.chain_to = NAV_ID_NONE;
		Car_Event_Send(CAR_EVT_NAV_DONE, prev_id, 0x00u);
		if (nav_entry_turn[nav.nav_id] != 0u)
		{
			Nav_Turntable(nav_entry_turn[nav.nav_id]);
		}
	}

	/* 必须在 mf_is_done 分支**之前**: 最后一段的 WAIT 出沿与 path 置 DONE 同拍发生,
	 * 放后面会因提前 return 而漏采 */
	SELFTEST_DRIFT_SAMPLE(m);
	if (mf_is_done(m))
	{
		nav.state = NAV_DONE;
		Car_Event_Send(CAR_EVT_NAV_DONE, nav.nav_id, 0x00u);
		SELFTEST_NEXT_NAV(nav.nav_id);
		return;
	}

	{
		/* 堵转判据=两轮位移绝对值之和: 原地转(重锚 TURN_TO)左右轮反向等大,
		 * 取代数和恒 0 会误报, 逐轮取绝对值对直行/弧线/原地转统一成立 */
		float d0 = m->wheel[0].delta_m;
		float d1 = m->wheel[1].delta_m;
		float d = (d0 < 0.0f ? -d0 : d0) + (d1 < 0.0f ? -d1 : d1);

		if ((tracking.state == TRACKING_STOPPED) || (tracking.state == TRACKING_FAULT))
		{
			/* 有意停才豁免堵转: STOPPED/FAULT 全零但段表仍在 RUN(本段没走完),
			 * 不豁免会在 3s 后误报 NAV_FAULT 把整段打断 */
			nav.stall_ms = 0u;
		}
		else if (mf_is_waiting(m) != 0u)
		{
			/* WAIT 段本就要求车体静止: 停等时长与 NAV_STALL_MS 同宽时不豁免必误报 */
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
 * 增益为旧环等效换算值, 台架已按蛇形修复重调(2026-09);
 * 折算过程与调参史(别再升过 0.7 等): 见 docs/坑-编码器与里程.md §5 */
static mf_ctx_t g_mf;

mf_ctx_t *mf_app(void)
{
	return &g_mf;
}

void mf_app_init(void)
{
	static const mf_cfg_t cfg =
	{
		/* 等效轨距标定: 原地转 90°, 调到编码器差分航向与 IMU 一致
		 * (步骤见 Motion/README.md「接入 panzer 的步骤」第 3 步;
		 *  本值同时是转向链增益、不可当半径旋钮 —— 见「track_width_m 的四处作用链与禁忌」) */
		.track_width_m = 0.25f,
		.half_length_m = 0.0f,
		.half_width_m  = 0.0f,
		.max_vx_mps    = 1.5f,           /* 覆盖 High_speed 1.41m/s */
		.max_vy_mps    = 0.0f,
		.max_wz_dps    = 150.0f,         /* Middle 弧线稳态 57dps + 修正余量 */
		.yaw_kp        = 16.0f,    /* 走直线蛇形修复: 外环降带宽匹配轮速内环(原32.4欠阻尼~1s极限环) */
		.yaw_ki        = 0.0f,     /* 2026-09-21: 2.0 未做 A/B 且实车反而更差, 回退到 09-20 可跑值 */
		.yaw_kd        = 0.8f,     /* 航向环加阻尼项, 抑制匀速直行左右摆 */
		.yaw_i_max     = 0.0f,     /* 与 yaw_ki 同步回退(ki=0 时 I 项本就不累积) */
		.yaw_out_max   = 172.8f,
		.pv_kp         = 2.0f,
		.wheel_kp      = 0.6f,     /* 蛇形修复: 0.3 内环响应过慢(全靠 I 慢爬);
					            * 旧1.0 近 bang-bang 起步浪涌, 0.6 折中, 别再升过 0.7 */
		.wheel_ki      = 7.0f,     /* 旧环 I 等效(见上方折算), 恢复静摩擦穿透能力 */
		.wheel_i_max   = 0.14f,    /* 7×0.14≈0.98 duty: 巡航需≈0.67, 原钳位0.095(0.665)恰卡死
					            * 工作点 → 差动时单侧积分顶满响应不对称(蛇形成因之一) */
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
		if ((motion->cmd.vx_mps == 0.0f) && (motion->cmd.wz_dps == 0.0f))
		{
			motion->wheel[wheel_index].out = 0.0f;
			mf_pid_reset(&motion->pid_wheel[wheel_index]);
		}
	}
}

/* 调试口观测: "这一拍到底谁在写 vx" —— 段表出没出速度 / 跟瞄有没有压零。
 * 生产者 TIM4 ISR, 消费者主循环 Car_Debug_Stat_Print(与其他 dbg 字段同一套路)。
 * 现场判"车为什么不动": VXP>0 而 VXF=0 = 跟瞄在压零(看 TRK 字母); VXP=0 = 段表没出速度
 * (看 NAV/RS); VXP/VXF 都>0 而 WHEEL 的 SL/SR≈0 = 指令到了电机侧没跟(驱动/使能/编码器) */
static volatile int16_t dbg_vx_path_mm;    /* mf_tick 后(段表含咬靶限速)的 vx */
static volatile int16_t dbg_vx_final_mm;   /* 跟瞄叠加后的 vx(真正进轮速环的值) */
static volatile uint8_t dbg_ovl;           /* 跟瞄本拍是否改写了 cmd(1=介入) */

//坦克主控函数：TIM4 10ms 中断调用
void Car_Control()
{
	extern void Uart_RxRearm_Task(void);   /* stm32f4xx_it.c: RX DMA 重装失败的重试点 */
	CarSense_Sample();                   /* 采样层: 编码器+航向 -> 快照(seq+1) */
	Uart_RxRearm_Task();                 /* 上拍 IDLE/Error 路径 HAL_BUSY 未装回的 RX DMA, 此处堵死口 */
	Car_Nav_Task();                      /* 调度层: 信箱/启停/事件/看门狗 */
	Car_TxQueue_Pump();                  /* 上行发送队列: ACK/EVT 单点泵出 */
	Tracking_LimitApproachSpeed();       /* 咬靶限速: 必须早于 mf_tick(轮速环只算一次) */
	mf_tick(mf_app(), CarSense_Get());   /* 运动库: 纯消费快照, 无新样自动跳过 */
	dbg_vx_path_mm = (int16_t)(mf_app()->cmd.vx_mps * 1000.0f);
	{
		uint8_t ovl = Tracking_ApplyOverlay();
		dbg_ovl = ovl;
		if (ovl != 0u)                   /* 段内跟瞄改写了 cmd */
		{
			Tracking_RecomputeWheels();
		}
		dbg_vx_final_mm = (int16_t)(mf_app()->cmd.vx_mps * 1000.0f);
	}
	CarSense_Output(mf_app());           /* 执行层: wheel[].out -> DIR+PWM */
	IWDG->KR = 0xAAAAU;                  /* 喂独立看门狗(198ms): 本函数停摆即复位清 PWM */
}

//HWT101角度置零: 缓冲必须 static —— Transmit_DMA 异步搬运, 栈数组函数返回即失效会发坏字节
void ResetHWT101(void)
{
	static uint8_t temp[] = {0xff, 0xAA, 0X76, 0x00, 0x00};
	if (huart3.gState != HAL_UART_STATE_READY)
	{
		return;   /* 上帧置零还在途: 跳过(命令幂等, 免 HAL_BUSY 静默失败) */
	}
	HAL_UART_Transmit_DMA(&huart3, temp, 5);
}

/* USART2 调试口状态块/打印('?'/p 周期): main 上下文, 关中断抓 nav/tracking/wheel
 * 快照防 TIM4 写入中途读到混合值。仅调试显示, 单字级撕裂容忍, 不进控制路径。
 * 原 9 参 Car_Debug_Snapshot 一并内聚于此(printf 本身仍在 main 上下文) */
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

	__disable_irq();
	m = mf_app();
	st = mf_get_status(m);
	pose = mf_get_pose(m);
	nav_state = nav.state;
	nav_id    = nav.nav_id;
	seg_idx   = st->seg_index;
	seg_done  = st->seg_done;
	trk_state = tracking.state;
	trk_found = tracking.target_found_flag;
	err_f10k  = (int16_t)(tracking.horizontal_error_normalized * TRACKING_ERROR_FIXED_SCALE);
	trk_frame_age = tracking.frame_age_ms;
	trk_hold  = tracking.hold_ms;
	trk_zx    = tracking.zero_sign;
	trk_reason = tracking.fault_reason;
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
	tt_deg = Car_data.Motor_Angle;
	tt_armed = tt_arm;
	tt_ms     = tt.ms;        /* 台架标定用: 本次 DONE 已计时 / 目标 settle */
	tt_settle = tt.settle_ms;
	tt_code   = tt.code;
	tt_tick   = tt_last_tick; /* 台架标定用: 最近一次下发的脉冲数(解位置当量) */
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

/* 跟瞄状态迁移打点(主循环): Tracking_Goto 在 ISR 只登记, 这里打一行 */
void Car_TrkChange_Log(void)
{
	uint8_t from, to, found;
	uint16_t age;
	int16_t  e10k;
	uint32_t rs_mm;

	__disable_irq();
	if (trk_chg_fresh == 0u)
	{
		__enable_irq();
		return;
	}
	from = trk_chg_from;
	to   = trk_chg_to;
	trk_chg_fresh = 0u;
	e10k = (int16_t)(tracking.horizontal_error_normalized * TRACKING_ERROR_FIXED_SCALE);
	found = tracking.target_found_flag;
	age = tracking.frame_age_ms;
	rs_mm = (uint32_t)(mf_path_remain_m(&mf_app()->path) * 1000.0f);
	__enable_irq();

	printf("TRK:%c>%c E=%d F=%u AGE=%u RS=%u\r\n",
	       Tracking_StateChar((TrackingState_t)from), Tracking_StateChar((TrackingState_t)to),
	       e10k, (unsigned)found, (unsigned)age, (unsigned)rs_mm);
}

/* ARMED/STOPPED 期高频打点(主循环 10Hz, 规格 §10.6): 定性"误差卡在哪一环" ——
 * 光看 TRK: 迁移行只能知道换了态, 不知道车在不在动、轮子跟没跟、判据卡在哪。
 * 读法: E 在收敛 = 车真在逼近靶心; E 不动而 SL/SR>0 = 车在走但视觉没更新;
 * CL/CR>0 而 SL/SR≈0 = 指令到电机侧没跟(轮速环/驱动/编码器); BRK=1 = 已触发制动 */
void Car_TrkDetail_Log(void)
{
	static uint32_t trace_tick = 0u;
	uint32_t now = HAL_GetTick();
	uint8_t  st_trk, brk;
	int8_t   zx;
	uint16_t hold;
	int32_t  e10k, cl, sl, cr, sr;

	if ((tracking.state != TRACKING_ARMED) && (tracking.state != TRACKING_STOPPED))
	{
		return;   /* 只在跟瞄介入期打点 */
	}
	if ((uint32_t)(now - trace_tick) < 100u)
	{
		return;
	}
	trace_tick = now;

	__disable_irq();
	st_trk = (uint8_t)tracking.state;
	e10k   = (int32_t)(tracking.horizontal_error_normalized * TRACKING_ERROR_FIXED_SCALE);
	zx     = tracking.zero_sign;
	hold   = tracking.hold_ms;
	brk    = tracking.braking;
	cl = (int32_t)(mf_app()->wheel[0].cmd_mps * 1000.0f);
	sl = (int32_t)(mf_app()->wheel[0].speed_mps * 1000.0f);
	cr = (int32_t)(mf_app()->wheel[1].cmd_mps * 1000.0f);
	sr = (int32_t)(mf_app()->wheel[1].speed_mps * 1000.0f);
	__enable_irq();

	printf("TRKD:ST=%c E=%d ZX=%d HOLD=%u BRK=%u CL=%d SL=%d CR=%d SR=%d\r\n",
	       Tracking_StateChar((TrackingState_t)st_trk), (int)e10k, (int)zx,
	       (unsigned)hold, (unsigned)brk,
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

/* 弧线出口质量(每跑完一条 ARC 打一行): EY = 出口横向偏差 mm(正=弧外侧, 负=往里切);
 * YE = 出口航向偏差 0.1°(正=车头偏向外侧切线)。判据: 单弧 |EY| 应压到 ±10mm 内 */
void Car_ArcExit_Log(void)
{
	uint16_t seg;
	float    ey, yerr;
	uint8_t  got;

	__disable_irq();
	got = mf_pop_arc_report(mf_app(), &seg, &ey, &yerr);
	__enable_irq();

	if (got != 0u)
	{
		printf("ARC:%u EY=%d YE=%d\r\n", (unsigned)seg,
		       (int)(ey * 1000.0f), (int)(yerr * 10.0f));
	}
}
/* TIM5 100ms 转台下发(经 interurp.c 调用)。
 * tt_arm 门控: 上电后首条 TURNTABLE 命令前/急停后不下发——旧版上电即向
 * 绝对 0°(=驱动器上电零点)发位置, 仅 MCU 复位时转台会自行大行程甩动。
 * 方向+角度关中断成对快照, 防 TIM4 更新中途读到混合对挥向另一侧。
 * 目标去重(见 tt_sent_key 注释): (方向,目标)没变就不发——重发会重启 Emm
 * 梯形规划, 10Hz 锯齿起步 = 转台剧烈抖动; 发送失败不记账, 下拍重试。
 * acc=20: 旧值 0 为直接启动, 带载 180RPM 零斜坡起转冲击/失步 */
void Emm_Pos_Control(void)
{
	uint8_t  dir;
	float    deg;
	uint32_t tick, key;

	if (tt_arm == 0u)
	{
		return;
	}
	__disable_irq();
	dir = CW_OR_CCW;
	deg = Car_data.Motor_Angle - TURNTABLE_BOOT_DEG;   /* 角度域 -> 驱动器计数域(上电清零后 0 脉冲 = BOOT_DEG) */
	__enable_irq();
	if (deg < 0.0f)
	{
		/* 防御: 负角转 uint32 是 ~4.29e9 脉冲, 一条命令就能让电机转到天荒地老。
		 * 正常码表(57~270°)且 BOOT_DEG=0 时不会走到这里 */
		deg = 0.0f;
	}
	tick = (uint32_t)(WTBL_TICK_PER_DEG * deg);
	key = (dir != 0u) ? tick : (tick | 0x80000000u);   /* 方向并入 key: 码6(CW 90)->码7(CCW 90) tick 相同 */
	if (key == tt_sent_key)
	{
		return;   /* 目标未变且已成功下发: 不再重发, 让驱动器独立跑完梯形规划 */
	}
	/* vel=200/ acc=120 都是**电机轴 RPM 口径**, 与 MStep 无关:
	 * 改驱动器细分时只改 WTBL_TICK_PER_DEG, 这两个不用动(行程耗时只取决于 RPM 与角度) */
	if (Emm_V5_Pos_Control(EMM_MOTOR_ADDR, dir, 180, 120, tick, true, false) != HAL_OK)
	{
		return;   /* 帧没进 DMA(如 TX 在途 HAL_BUSY): 不记账, 下个 100ms 拍重试 */
	}
	tt_sent_key = key;
	tt_last_tick = tick;   /* 台架标定读数('?' 的 WTBL 行 TICK=): 最近一次实际下发量 */
}

/* 上电转台零点对齐: 发 0x0A 清零驱动器计数器(电机不动), MCU 记账角置 TURNTABLE_BOOT_DEG。
 * 调用时点: TIM5 启动之前 + 驱动器上电自检完成之后(约 1s) */
void Car_Turntable_BootZero(void)
{
	tt_arm = 0u;
	tt_sent_key = TT_KEY_UNSENT;   /* 驱动器计数器已清零, 旧"已下发"记录全部作废 */
	tt.busy = 0u;
	tt.code = 0u;
	tt.ms = 0u;
	tt.settle_ms = 0u;
	Car_data.Motor_Angle = TURNTABLE_BOOT_DEG;
	Emm_V5_Reset_CurPos_To_Zero(EMM_MOTOR_ADDR);
	Car_Dbg2_TtRecord(3u, 0u, TURNTABLE_BOOT_DEG);   /* kind=3: 上电清零 */
}

/**
  * @brief 配置 HWT101 输出速率为 200Hz
 * @note  配置会保存到 IMU 内部 Flash，后续上电无需重复配置。
 *        建议仅在首次使用或 IMU 恢复出厂设置后调用一次。
 */
void ConfigHWT101(void)
{
    /* 1. 解锁寄存器 (进入配置模式) */
    static uint8_t unlock_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    if (huart3.gState != HAL_UART_STATE_READY) {
        HAL_Delay(10);
    }
    HAL_UART_Transmit_DMA(&huart3, unlock_cmd, 5);
    HAL_Delay(100);  /* 等待 IMU 进入配置模式 */

    /* 2. 设置输出速率为 200Hz (寄存器 RRATE, 值 0x05) */
    static uint8_t set_200hz_cmd[] = {0xFF, 0xAA, 0x69, 0x05, 0x00};
    if (huart3.gState != HAL_UART_STATE_READY) {
        HAL_Delay(10);
    }
    HAL_UART_Transmit_DMA(&huart3, set_200hz_cmd, 5);
    HAL_Delay(100);  /* 等待寄存器写入完成 */

    /* 3. 保存配置并重启 IMU */
    static uint8_t save_cmd[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    if (huart3.gState != HAL_UART_STATE_READY) {
        HAL_Delay(10);
    }
    HAL_UART_Transmit_DMA(&huart3, save_cmd, 5);
    HAL_Delay(500);  /* 等待 IMU 保存并重启 */
}