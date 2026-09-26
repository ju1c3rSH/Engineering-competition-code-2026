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
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(50.0f)),
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
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(54.5f)),
	SEG_ARC(MF_M_SPEED(Low_speed), -TURN_RADIUS_M),   /* 右 90 (flag14) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(80.0f)),
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
	SEG_MOVE(MF_M_SPEED(Low_speed),    MF_M_DIST(48.5f)),/* 末段低速进作业点 */
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

/* nav_id 的"无"哨兵。必须 != 任何合法 nav_id(0~7): nav 是 static 零初始化,
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

/* ---- 打靶段内跟瞄 v2: 段表不断, 段内叠加视觉航向修正 ----
 * 靶标前一段由中控发 TRACKING_START 开启, 底盘按指定速度边走边修航向;
 * horizontal_error_normalized 归零即段内刹停等发射, 发射后 TRACKING_STOP
 * (mode=1) 继续走完当前段。履带动, 转盘不动 */
/* 段内跟瞄六态机: APPROACH 不碰 vx 只写 wz(覆盖, 非叠加); 段表 done 才进 AIM,
 * DONE = 到位且收敛; APPROACH 丢目标不停车、AIM 才压零; LOCKED/FAULT 可被解除 */
typedef enum
{
	TRACKING_OFF = 0,     /* 不干预: 段表全权(原 TRACKING_IDLE) */
	TRACKING_APPROACH,    /* 进近: 段表给 vx, 跟瞄独占 wz 修正 */
	TRACKING_AIM,         /* 静瞄: 段表已到位(vx=0), 跟瞄独占 wz, 等收敛 */
	TRACKING_HOLD,        /* 测量失效: APPROACH 下放行段表 / AIM 下压零等恢复(有上限) */
	TRACKING_LOCKED,      /* 已到位且收敛(或主控要求锁停): 全零; 收敛型会发 DONE */
	TRACKING_FAULT        /* 本地判定瞄不上(见 fault_reason): 全零 + 上报 0x06, 等 START 复位 */
} TrackingState_t;

static struct
{
	TrackingState_t state;
	float    horizontal_error_normalized;  /* 最新归一化横向误差, 约 -1~+1 */
	uint8_t  target_found_flag;            /* 最新有效标志 */
	uint16_t frame_age_ms;                 /* 距上次有效误差帧时长 */
	uint16_t hold_ms;                      /* AIM 下 |e| 达标累计(收敛判据) */
	uint16_t hover_ms;                     /* 测量失效累计(APPROACH 记观察, AIM 计时上限) */
	uint16_t aim_bad_ms;                   /* AIM 下 |e| 超阈累计(FAULT 判据) */
	uint8_t  tracking_speed_level;         /* 0=段表速度/1=低速跟瞄 */
	uint8_t  fault_reason;                 /* TRACKING_FAULT 的 reason, 见 Control_Car.h TRK_FAULT_* */
} tracking;

/* 状态迁移唯一入口: 复位三个计时器 + 登记一次迁移打点(主循环 Car_TrkChange_Log 消费)。
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
	tracking.hover_ms = 0u;
	tracking.aim_bad_ms = 0u;
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
	case TRACKING_APPROACH: return 'A';
	case TRACKING_AIM:      return 'M';
	case TRACKING_HOLD:     return 'H';
	case TRACKING_LOCKED:   return 'L';
	case TRACKING_FAULT:    return 'F';
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

/* 目标去重(2026-09 改, 替代旧基线"100ms 周期重发同一绝对位置"):
 * Emm_V5 驱动器每收到一条 0xFD 位置帧都会**重启电机梯形规划**——周期重发把
 * 行程切成 10Hz 的"起步-重启-起步"锯齿, 永远到不了巡航速度, 带载表现为转台
 * 剧烈抖动(台架 'w'/'e' 微调与实车摆坑/桶位走的是同一条 TIM5 下发链路, 症状
 * 相同)。改为: 目标(方向+脉冲数打包 key)变化才下发一次, 到位后停发。
 * 丢帧鲁棒性: 发送失败(HAL_BUSY 等)不记账, 下个 100ms 拍自动重试; 驱动器
 * 是否真收到靠 '?' 状态块 EMM 行的应答计数(Emm_reply_ok_cnt/Emm_rx_cnt)观测 */
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
	0u, 0u, 0u, 0u, 0u, 0u, 7u, 0u,   /* 仅 NAV_REBASE_BOMB(6) -> 回正 90° */
};

/* 段表自动链续: 该 nav 跑完由底盘自行续跑下一条(0=不续, 等主控下发)。
 * 现仅 NAV_TO_BOMB(1) -> NAV_REBASE_BOMB(6): 两表在几何上首尾相接, 中间本无停顿理由
 * —— 末段转弯只是为了让转台 90° 与行驶并行才被拆进 nav 6。拆表带来的代价是表尾被
 * 当成"路线终点"(END ⇒ v_next=0) 而硬性减速, 加上跨芯片握手的零输出空窗。
 * 链续后: ① 末段按续表首段速度收尾(见 mf_path seg_next_entry_speed);
 *        ② 跨表同拍切换、输出不归零, 无空窗、cur_speed 连续、轮速环不复位。
 * 自测形态必须置 0: SELFTEST_NEXT_NAV 与它同在"跑完一条"处接管, 同开会双触发。
 * 续表一律用 flash 原表 nav_routes[], 不走 Nav_GearTable 的 RAM 缓冲 —— 那份 static
 * 缓冲此刻仍装着当前表的拷贝, 往里拷新表会让正在跑的 path 指针悬空 */
static const uint8_t nav_chain_next[CAR_NAV_ID_COUNT] =
{
	0u, NAV_CHAIN_1_TO, 0u, 0u, 0u, 0u, 0u, 0u,
};

/* REQ_NAV 幂等: 同 id 进行中忽略 / 已完成补发 DONE / 其余停旧起新 */
static void Nav_DoRequest(uint8_t navigation_target_id, uint8_t speed_gear)
{
	uint8_t chain_to;

	if (nav.state == NAV_RUN && nav.nav_id == navigation_target_id)
	{
		/* 同段进行中: 幂等忽略。只解除"卡住"的 LOCKED/FAULT(否则重发对跟瞄无效);
		 * APPROACH/AIM/HOLD 保持原状 —— 无条件回 OFF 会被中控的幂等重发
		 * (超时重试/EXE_LASER 恢复段)打断正在收敛的跟瞄 */
		if ((tracking.state == TRACKING_LOCKED) || (tracking.state == TRACKING_FAULT))
		{
			Tracking_Goto(TRACKING_OFF);
		}
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
	/* 起新段清跟瞄: 防 EXE_LASER 重试/FAULT 泄漏的 LOCKED/FAULT 带到新路段 */
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

/* 收到 TRACKING_START: 段表已到位进 AIM, 否则进 APPROACH; 无段表直接 FAULT */
static void Tracking_Start(uint8_t tracking_speed_level)
{
	mf_ctx_t *motion = mf_app();

	if (mf_path_get_state(&motion->path) != MF_PATH_RUN)
	{
		tracking.fault_reason = TRK_FAULT_NO_PATH;
		Tracking_Goto(TRACKING_FAULT);
		Tracking_HoldStill(motion);
		Car_Event_Send(CAR_EVT_TRACKING_FAULT, TRK_FAULT_NO_PATH, 0x00u);
		return;
	}
	tracking.tracking_speed_level = tracking_speed_level;
	tracking.frame_age_ms = 0u;
	tracking.target_found_flag = 0u;
	tracking.horizontal_error_normalized = 0.0f;
	tracking_error_pending = 0u;   /* 丢弃上轮残帧, 首拍不等旧误差 */
	/* LOCKED/FAULT 下重收 START = 主控要求重新开始跟瞄 ⇒ 回到 APPROACH/AIM 重瞄(解除黏性),
	 * 若确实仍收敛, 500ms 后会再发一次 DONE, 对主控的幂等重发友好 */
	Tracking_Goto((mf_is_done(motion) != 0u) ? TRACKING_AIM : TRACKING_APPROACH);
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

/* TRACKING_STOP: 0=就地刹停(mode=0 不发 DONE, 那是主控要求而非本地收敛) / 1=让段表续跑 */
static void Tracking_Stop(uint8_t tracking_stop_mode)
{
	if (tracking_stop_mode == 0u)
	{
		Tracking_HoldStill(mf_app());
		Tracking_Goto(TRACKING_LOCKED);
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

/* 视觉 P 控制: **覆盖** wz(跟瞄态下段表航向环让出); vx 只封顶不清零 */
static void Tracking_WriteYaw(mf_ctx_t *motion)
{
	float wz = -TRACKING_PROPORTIONAL_GAIN * tracking.horizontal_error_normalized;

	if (wz >  TRACKING_MAX_YAW_RATE_DPS) { wz =  TRACKING_MAX_YAW_RATE_DPS; }
	if (wz < -TRACKING_MAX_YAW_RATE_DPS) { wz = -TRACKING_MAX_YAW_RATE_DPS; }
#if (TRACKING_PATH_WZ_KEEP_PCT > 0u)
	/* 可选前馈: 保留段表当拍 wz 的一部分, 分担跟瞄单独扛全部航向修正的压力 */
	wz += motion->cmd.wz_dps * ((float)TRACKING_PATH_WZ_KEEP_PCT / 100.0f);
#endif
	motion->cmd.wz_dps = mf_clampf(wz, motion->cfg.max_wz_dps);
	if ((tracking.tracking_speed_level == 1u) && (motion->cmd.vx_mps > MF_M_SPEED(Low_speed)))
	{
		motion->cmd.vx_mps = MF_M_SPEED(Low_speed);
	}
}

/* 每拍在 mf_tick 之后调用。返回 1 = 本拍改写了 cmd(须重算轮速环); 0 = 不干预(段表全权)。
 * 这是跟瞄的唯一执行入口: 状态决定"谁写 vx/wz", 同一拍不存在两个写入者 */
static uint8_t Tracking_ApplyOverlay(void)
{
	mf_ctx_t *motion = mf_app();
	float absolute_error;
	uint8_t fresh;

	if (tracking.state == TRACKING_OFF)
	{
		return 0u;                     /* 段表全权 */
	}
	if ((tracking.state == TRACKING_LOCKED) || (tracking.state == TRACKING_FAULT))
	{
		Tracking_HoldStill(motion);    /* 锁停/故障: 全零, 等主控解除 */
		return 1u;
	}

	Tracking_ConsumeErrorFrame();
	if (tracking.frame_age_ms < (uint16_t)(0xFFFFu - CAR_TASK_MS))
	{
		tracking.frame_age_ms = (uint16_t)(tracking.frame_age_ms + CAR_TASK_MS);
	}
	fresh = (uint8_t)((tracking.target_found_flag != 0u) &&
	                  (tracking.frame_age_ms <= TRACKING_FRAME_TIMEOUT_MS));

	if (fresh == 0u)
	{
		/* 测量失效: 未到位不停车(否则"车不动⇒靶不进视野"死锁), 已到位才压零 */
		tracking.aim_bad_ms = 0u;
		if (tracking.state != TRACKING_HOLD)
		{
			Tracking_Goto(TRACKING_HOLD);   /* 进入即清零 hover, 从 0 起计 */
		}
		tracking.hover_ms = (uint16_t)(tracking.hover_ms + CAR_TASK_MS);
		if (mf_is_done(motion) == 0u)
		{
			/* 进近中丢目标: **不停车**(停了就再也看不到靶 ⇒ 死锁), 只停 wz 修正、段表照常
			 * 推进; 若到靶前仍无测量, 由下面的 AIM 分支按上限收尾 */
			motion->cmd.wz_dps = 0.0f;
			return 1u;
		}
		Tracking_HoldStill(motion);
		if ((TRACKING_AIM_LOST_MAX_MS > 0u) && (tracking.hover_ms >= TRACKING_AIM_LOST_MAX_MS))
		{
			Tracking_RaiseFault(TRK_FAULT_NO_TARGET);
		}
		return 1u;
	}

	tracking.hover_ms = 0u;
	if (tracking.state == TRACKING_HOLD)
	{
		Tracking_Goto((mf_is_done(motion) != 0u) ? TRACKING_AIM : TRACKING_APPROACH);
	}
	absolute_error = (tracking.horizontal_error_normalized < 0.0f)
	                 ? -tracking.horizontal_error_normalized
	                 : tracking.horizontal_error_normalized;

	/* 到位才升 AIM: "段表跑完 = 几何到位(事实)"与"误差小 = 收敛(测量)"解耦 ——
	 * 进近途中的小误差只说明车头正对着靶, 不构成"该停车" */
	if ((tracking.state == TRACKING_APPROACH) && (mf_is_done(motion) != 0u))
	{
		Tracking_Goto(TRACKING_AIM);
	}

	if (tracking.state == TRACKING_AIM)
	{
		Tracking_HoldStill(motion);    /* vx=0: 已到位, 原地精瞄 */
		Tracking_WriteYaw(motion);     /* wz 归跟瞄 */
		if (absolute_error >= TRACKING_ERROR_THRESHOLD)
		{
			tracking.hold_ms = 0u;
			tracking.aim_bad_ms = (uint16_t)(tracking.aim_bad_ms + CAR_TASK_MS);
			if ((TRACKING_AIM_FAULT_MS > 0u) && (tracking.aim_bad_ms >= TRACKING_AIM_FAULT_MS))
			{
				Tracking_RaiseFault(TRK_FAULT_NOT_CONVERGED);
			}
			return 1u;
		}
		tracking.aim_bad_ms = 0u;
		tracking.hold_ms = (uint16_t)(tracking.hold_ms + CAR_TASK_MS);
		if (tracking.hold_ms >= TRACKING_HOLD_TIME_MS)
		{
			Tracking_HoldStill(motion);
			Tracking_Goto(TRACKING_LOCKED);
			Car_Event_Send(CAR_EVT_TRACKING_DONE, 0u, 0x00u);   /* 到位且收敛 = 可发射 */
		}
		return 1u;
	}

	/* APPROACH: 速度权归段表(vx 一律不碰), 跟瞄只写 wz */
	tracking.hold_ms = 0u;
	Tracking_WriteYaw(motion);
	return 1u;
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
		case CAR_SUB_TRACKING_START:    Tracking_Start(a0); break;
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

		if ((tracking.state == TRACKING_AIM) || (tracking.state == TRACKING_LOCKED) ||
		    (tracking.state == TRACKING_FAULT))
		{
			/* 有意停才豁免堵转(AIM/LOCKED/FAULT); APPROACH 由段表给速度, 不该静止 */
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
 * 增益为旧环等效换算值, 台架已按蛇形修复重调(2026-09, 见各成员注释):
 * 轮速环 P: 0.6(旧1.0近bang-bang→0.3欠响应→0.6 折中)
 * 轮速环 I: 按旧环 Ki=50/拍(误差 rev/s, 满幅 1500, I_Max 1000) 等效折算:
 *   贡献速率 = 误差×50÷0.01s÷1500 = 3.33 duty/s per rev/s
 *            = 7.07 duty/s per m/s(1rev/s=π×0.15m)          -> ki = 7.0
 *   I 上限   = 巡航占空比≈0.67 需余量 -> i_max = 0.14(≈0.98 duty, 原钳位 0.095 恰卡巡航点)
 * 航向环: kp=16(旧环等效折算原 32.4, 内外环带宽失配欠阻尼蛇形, 降半); kd=0.8 加阻尼 */
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
	extern void Uart_RxRearm_Task(void);   /* stm32f4xx_it.c: RX DMA 重装失败的重试点 */
	CarSense_Sample();                   /* 采样层: 编码器+航向 -> 快照(seq+1) */
	Uart_RxRearm_Task();                 /* 上拍 IDLE/Error 路径 HAL_BUSY 未装回的 RX DMA, 此处堵死口 */
	Car_Nav_Task();                      /* 调度层: 信箱/启停/事件/看门狗 */
	Car_TxQueue_Pump();                  /* 上行发送队列: ACK/EVT 单点泵出 */
	mf_tick(mf_app(), CarSense_Get());   /* 运动库: 纯消费快照, 无新样自动跳过 */
	if (Tracking_ApplyOverlay() != 0u)   /* 段内跟瞄改写了 cmd */
	{
		Tracking_RecomputeWheels();
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
	uint8_t  trk_speed, trk_reason;
	uint16_t seg_idx, trk_frame_age, trk_hold, trk_hover, trk_aim_bad, tt_ms, tt_settle;
	uint32_t trk_remain_mm;
	uint32_t tt_tick;
	int16_t  err_f10k;
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
	trk_hover = tracking.hover_ms;
	trk_aim_bad = tracking.aim_bad_ms;
	trk_speed = tracking.tracking_speed_level;
	trk_reason = tracking.fault_reason;
	trk_remain_mm = (uint32_t)(mf_path_remain_m(&m->path) * 1000.0f);
	x_m    = pose->x_m;
	y_m    = pose->y_m;
	yaw_deg = pose->yaw_deg;
	wl_cmd = m->wheel[0].cmd_mps;   /* 框架轮序 0=左 1=右(CarSense_Output 同序) */
	wl_spd = m->wheel[0].speed_mps;
	wr_cmd = m->wheel[1].cmd_mps;
	wr_spd = m->wheel[1].speed_mps;
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
	/* ST 六态 O/A/M/H/L/F; RS=段表剩余里程(mm); FR=FAULT reason */
	printf("TRK2:ST=%c SPD=%u HOVER=%u AIMBAD=%u RS=%u FR=%u\r\n",
	       Tracking_StateChar((TrackingState_t)trk_state), (unsigned)trk_speed,
	       (unsigned)trk_hover, (unsigned)trk_aim_bad,
	       (unsigned)trk_remain_mm, (unsigned)trk_reason);
	printf("WHEEL:CL=%d SL=%d CR=%d SR=%d\r\n",
	       (int)(wl_cmd * 1000.0f), (int)(wl_spd * 1000.0f),
	       (int)(wr_cmd * 1000.0f), (int)(wr_spd * 1000.0f));
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
	/* vel=180 / acc=20 都是**电机轴 RPM 口径**, 与 MStep 无关:
	 * 改驱动器细分时只改 WTBL_TICK_PER_DEG, 这两个不用动(行程耗时只取决于 RPM 与角度) */
	if (Emm_V5_Pos_Control(EMM_MOTOR_ADDR, dir, 180, 20, tick, true, false) != HAL_OK)
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