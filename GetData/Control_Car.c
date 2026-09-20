#include "Control_Car.h"
#include "Car_Sense.h"
#include "mf_math.h"
#include "mf_kinematics.h"

Parameter_TypeDef_Car Car_data;
Yaw_TypeDef Yaw_struct;

/* 张大头驱动器命令地址: 实车电机设为 2(2026-09 台架核对), 改电机地址就翻这。
 * 位置帧与急停帧都取此宏——场上曾因写死 1 而地址为 2 臂到驱动器全程静默 */
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
#define SEG_ARC(v, r)   { .type = MF_SEG_ARC, .reset_pid = 1, \
                          .p.arc = { (v), (r), ((r) > 0.0f ? 90.0f : -90.0f), SEG_ACC_MPS2 } }
#define SEG_TURN_TO(y)  { .type = MF_SEG_TURN_TO, .reset_pid = 1, \
                          .p.turn = { (y), 0.0f } }
#define SEG_END         { .type = MF_SEG_END }

/* 测试路线停等段(test/route): 每个运动段之后插一次, 停稳 3s 供现场读 POSE 查落点。
 * 段表侧 WAIT 的前一段会自动按抛物线减速到 0(seg_next_needs_stop 含 WAIT), 所以
 * "跑完一段→停三秒→继续"是段表原生语义, 无需上层干预。
 *
 * 【置 0 = 关闭停等】此时**完全不插 WAIT 段**(见 Nav_GearTable), 段表回到原版形态:
 * 段间速度连续、每条 nav 只首末起步停车。用于 A/B 对比定位误差来源 ——
 * 有/无停等各跑一遍, 终点位姿之差就是"每段停 3s"的净贡献。
 *
 * 【为何停等会改变里程累积】不是停等本身"算错", 而是它把整条路线的运动学剖面改了:
 *   1) 起停次数 8 段×1 → 18 段×18, 每次起步的履带静摩擦打滑都不可复现
 *   2) 段完成判据是 remain<=8mm(MF_TOL_ARRIVE_M), 此时车速还有 ~0.19m/s,
 *      进 WAIT 段后靠斜坡减到 0 的这段滑行(~12mm)只进 pose、不进段进度 → 白走
 *   3) WAIT 段 300 拍的位移全部不计入任何段的 dist_accum(见 mf_path_tick),
 *      下一段从 0 重新计, 不会扣回去
 *   现场用串口 PD: 行(SEG/PAUSE/TOT)可直接读出白走里程
 *
 * 【配套约束】WAIT 段内车体静止, 里程 3s 无变化会撞上 NAV_STALL_MS(3000ms)
 * 堵转看门狗 —— 已由 Car_Nav_Task 里的 mf_is_waiting() 分支豁免。
 * 改这个时长或删掉豁免, 车会在每次停等结束时报 NAV_FAULT 并中途停车 */
#define ROUTE_TEST_PAUSE_MS  3000u
#define SEG_WAIT_MS(ms) { .type = MF_SEG_WAIT, .reset_pid = 0, .p.wait = { (ms) } }
#define SEG_PAUSE       SEG_WAIT_MS(ROUTE_TEST_PAUSE_MS)

static const mf_seg_t tbl_qr[] =          /* NAV_TO_QR: flag0 出发首段 43(台架单段用) */
{
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(43.0f)),
	SEG_END
};
static const mf_seg_t tbl_bomb[] =        /* NAV_TO_BOMB: 排爆区去程(末段转弯+末段直线已移入重锚表) */
{
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(43.0f)),
	SEG_ARC(MF_M_SPEED(Low_speed),  TURN_RADIUS_M),   /* 左 90 (flag1) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(12.0f)),
	SEG_ARC(MF_M_SPEED(Low_speed), -TURN_RADIUS_M),   /* 右 90 (flag2) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(46.0f)),
	SEG_ARC(MF_M_SPEED(Low_speed),  TURN_RADIUS_M),   /* 左 90 (flag4) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(35.0f)),
	SEG_ARC(MF_M_SPEED(Low_speed), -TURN_RADIUS_M),   /* 右 90 (flag6) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(48.5f)),
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
	SEG_ARC(MF_M_SPEED(Low_speed), -TURN_RADIUS_M),   /* 右 90 (flag14) */
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(60.0f)),
	SEG_END
};
static const mf_seg_t tbl_return[] =      /* NAV_TO_RETURN: flag17~18 返回区 */
{
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(170.0f)),
	SEG_END
};
static const mf_seg_t tbl_rebase_bomb[] =   /* NAV_REBASE_BOMB: 排爆区末段(原 tbl_bomb 表尾)+绝对航向重锚。
                                             * 末段转弯与其后的低速直线挪进本表, 与转盘 90° 扫描位并行:
                                             * 本段入口由底盘自行摆码 7(见 nav_entry_turn), 转台在最后一个
                                             * 转弯期间就开始转(原工程 flag9 时点)。此刻臂为收起位,
                                             * 扫掠虽与末段转弯叠加但无干涉; 若目视碍事, 把
                                             * nav_entry_turn[6] 置 0 由主控在 NAV_DONE 之后再发(=串行) */
{
	SEG_ARC(MF_M_SPEED(Low_speed), -TURN_RADIUS_M),      /* 右 90: 原 tbl_bomb 表尾转弯 */
	SEG_MOVE(MF_M_SPEED(Low_speed),    MF_M_DIST(45.5f)),/* 末段低速进作业点 */
	SEG_TURN_TO(-90.0f),                                 /* 对坑前重锚: 五个闭环 90° 弧的标称终点。
	                                                      * 旧值 -90.3 是旧车 flag10 实测搬运; 0.3° 在
	                                                      * 0.5~0.7m 臂展上仅 3mm 级, 够不着横向偏移问题。
	                                                      * 要定这个角: 用底盘 '?' 的 POSE 行读到达 YAW
	                                                      * + 看三坑在相机里的位置, 别猜 */
	SEG_END
};
static const mf_seg_t tbl_rebase_return[] = /* NAV_REBASE_RETURN: flag17 携人质返程前重锚 */
{
	SEG_TURN_TO(-179.8f),
	SEG_END
};

#if ROUTE_TEST_EN
/* ================== test/route: 自测命令序列(底盘自己扮演 Brain) ==================
 * 推进方式: 不接 Brain, 底盘按**原版命令状态机**的 nav 顺序逐条自己下发 REQ_NAV,
 * 跑完一条(NAV_DONE)自动发下一条, 直到返回区(完赛区)停车, 不循环。
 *   序列 = 1→6→2→3→4→7→5     (去掉 NAV_TO_QR: 出发首段不走)
 * 与 Brain Engine/mission_sm.c 的对应:
 *   ST_NAV_BOMB(1) → ST_NAV_REBASE_BOMB(6) → ST_NAV_BUCKET(2) → ST_NAV_TARGET(3)
 *   → ST_NAV_HOSTAGE(4) → ST_NAV_REBASE_RETURN(7) → ST_NAV_RETURN(5)
 * 注: 2 NAV_TO_BUCKET 是空表(桶与排爆作业点同点位, 无底盘位移), 会即时 DONE 并自动
 *     推进到 3, 现场表现为"在排爆作业点不做停留"; 3 打靶区的段内跟瞄需 Brain 下发
 *     TRACKING_START, 自测不跟瞄, 只走段表直线。
 *
 * 【停 3s 在哪加的】不是这里 —— 由 Nav_GearTable() 在把段表拷进 RAM 时自动在每个
 * 运动段之后插一个 SEG_PAUSE(3s)。所以原版每张表一个字不改, 只在自测时展开成
 * "跑一段→停 3s" 的节奏; 空表(2)没有运动段, 自然不停。
 * 【位姿连续性】跨 nav 只换段表指针, 里程计 odom 不重置, 故重锚段 6/7 的 TURN_TO
 * 绝对角仍依赖前面所有段的累积航向, 与真机一次跑完的基准一致 */
static const uint8_t route_test_seq[] =
{
	1u,   /* NAV_TO_BOMB         排爆区去程 */
	6u,   /* NAV_REBASE_BOMB     末段转弯+低速进作业点+航向重锚 */
	2u,   /* NAV_TO_BUCKET       空表(同点位, 无底盘位移) */
	3u,   /* NAV_TO_TARGET       打靶区 */
	4u,   /* NAV_TO_HOSTAGE      人质区 */
	7u,   /* NAV_REBASE_RETURN   返程前航向重锚 */
	5u,   /* NAV_TO_RETURN       返回区=完赛区 */
};
#define ROUTE_TEST_SEQ_LEN  ((uint8_t)(sizeof(route_test_seq) / sizeof(route_test_seq[0])))
static uint8_t rt_idx;        /* 当前正在跑的序列下标 */
static uint8_t rt_finished;   /* 1=序列已跑完(停在返回区), 主循环打印一次 */
#endif


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

#if ROUTE_TEST_EN
#define NAV_GEAR_BUF_LEN 24u      /* 自测展开后最长: tbl_bomb 9 运动段 + 9 停等 + END = 19 */
#else
#define NAV_GEAR_BUF_LEN 12u      /* 最长表 tbl_bomb: 9 段 + END */
#endif

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

/* speed_gear 覆盖需改写段速度，拷入 RAM 缓冲后启动。
 * test/route: 拷的同时在每个运动段之后插一个 SEG_PAUSE(3s) —— 原版每张表不动,
 * 只在自测时展开成"跑一段→停 3s"。停等是段表原生语义(WAIT 前一段自动抛物线减速
 * 到 0), 所以"跑完一段停三秒再继续"不需要上层再造状态机 */
static const mf_seg_t *Nav_GearTable(uint8_t navigation_target_id, uint8_t speed_gear)
{
	static mf_seg_t buf[NAV_GEAR_BUF_LEN];
	const mf_seg_t *src = nav_routes[navigation_target_id];
	uint8_t i, o;
	float v;
#if ROUTE_TEST_EN
  #if (ROUTE_TEST_PAUSE_MS > 0u)
	const mf_seg_t pause_seg = SEG_PAUSE;
  #endif
#else
	if (speed_gear == 0u)
	{
		return src;
	}
#endif

	v = (speed_gear == 0u) ? 0.0f : Nav_GearSpeed(speed_gear);
	o = 0u;
	for (i = 0u; (i < NAV_GEAR_BUF_LEN) && (o < NAV_GEAR_BUF_LEN); i++)
	{
		buf[o] = src[i];
		if (buf[o].type == MF_SEG_END)
		{
			break;
		}
		if (speed_gear != 0u)
		{
			if (buf[o].type == MF_SEG_MOVE_DIST)
			{
				buf[o].p.move_dist.speed_mps = v;
			}
			else if (buf[o].type == MF_SEG_ARC)
			{
				buf[o].p.arc.speed_mps = v;
			}
		}
		o++;
#if ROUTE_TEST_EN
		/* 运动段之后插停等; WAIT 段本身不再插(防表里已有 WAIT 时连停两次)。
		 * ROUTE_TEST_PAUSE_MS=0 时**完全不插** —— 段表回到原版形态(段间速度连续,
		 * 只在本条 nav 的首末起步/停车), 用于 A/B 对比: 有停等 vs 无停等各跑一遍,
		 * 终点偏差之差就是"每段停 3s"对路程累积的净贡献 */
  #if (ROUTE_TEST_PAUSE_MS > 0u)
		if ((src[i].type != MF_SEG_WAIT) && (o < NAV_GEAR_BUF_LEN))
		{
			buf[o++] = pause_seg;
		}
  #endif
#endif
	}
	/* 缓冲耗尽时兜底收尾: 宁可截断也必须以 END 结束, 否则段表会越过缓冲读飞 */
	if (o >= NAV_GEAR_BUF_LEN)
	{
		o = (uint8_t)(NAV_GEAR_BUF_LEN - 1u);
		buf[o].type = MF_SEG_END;
	}
	return buf;
}

/* ================== 调度层状态（panzer_links_protocol.md 4.5） ================== */
#define CAR_TASK_MS            10u
#define NAV_STALL_MS           3000u
#define NAV_STALL_DELTA_M      0.001f
/* 转台 DONE 计时(开环静置, 不校验 Emm 到位)。**按本次行程动态算, 且只增不减**:
 *   行程(°) = |本次目标角 − 上次目标角|   (绝对定位: 上次目标即电机应已在的位置)
 *   settle  = clamp(行程 × TURNTABLE_MS_PER_DEG + TURNTABLE_RAMP_MARGIN, MIN, MAX)
 * 下限取旧固定值 1000ms —— 旧值已覆盖 180RPM/67.16ticks每度 下 270°(桶位码, 最大行程,
 * 约 0.25s 巡航) 的物理时间, 所以本改动**不缩短任何现有码的等待**, 只在更大行程时加长。
 * 方向约定: 宁可偏长。报早了 = 主控在转台未停稳时就伸臂/放腕(机构干涉); 报晚了只多花
 * 几百毫秒。第一条命令(tt_arm==0, 上电后)行程不可知时也由下限 1000ms 兜住。
 * 【待台架标定】TIM5 每 100ms 重发会重启驱动器梯形规划, 电机未必跑到巡航速度, 故
 * TURNTABLE_MS_PER_DEG 现按额定值的一半保守取。上机敲桶位码 4/5(270°) 录像量
 * "发令→实际停稳"时长 t, 置 TURNTABLE_MS_PER_DEG = t/270 即可(看 '?' 块的 WTBL 行) */
#define TURNTABLE_MS_PER_DEG   1.9f    /* 额定 180RPM ⇒ 0.93ms/°; ×2 计梯形重启欠速 */
#define TURNTABLE_RAMP_MARGIN  300u    /* 加减速 + 机械/传动裕量 */
#define TURNTABLE_SETTLE_MIN   1000u   /* 只增不减: 旧固定值, 相对旧行为零回退 */
#define TURNTABLE_SETTLE_MAX   3000u   /* 上限: 旧基线值, 防异常行程把流程拖死 */

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

static void Nav_Fault(void);   /* 前置声明: Nav_DoRequest 段表校验失败即调(定义在后) */

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

/* 转台重发门控: 上电后首条 TURNTABLE 命令才允许 Emm 下发, 急停后重新冻结。
 * 防两类失控: 1) 仅 MCU 复位(驱动器带电)时自动甩回旧目标; 2) 急停后 TIM5
 * 100ms 重发继续驱动转台 */
static uint8_t tt_arm = 0u;

/* 转台码->定点角: 对坑 1->123 2->90 3->57, 对桶 4/5->270 6->90, 7 回正->90, 0 不动作 */
/* 注意(旧基线遗留行为, 暂不改动): 转台为开环时序——TIM5 100ms 周期重发
 * 同一绝对位置(Emm_V5 绝对模式, 重复下发还会重启电机梯形规划), TURN_DONE
 * 在置 busy 后固定 3s 计时上报, 不校验 Emm 应答与到位状态。沿用实车验证
 * 行为; 若现场出现"未到位即动作", 改法: 目标变化才下发一次 + 轮询 Emm
 * 状态字确认到位后再报 TURN_DONE */
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

/* 位置当量: ticks/° = (每电机转脉冲数 × 减速比 N) / 360
 *   - 每电机转脉冲数 = 200 步/转 × MStep:  MStep=16 → 3200;  MStep=256 → 51200。
 *     **改 MStep 时当量同比变化**(16→256 即 ×16), 这是唯一必须跟着改的量。
 *   - 减速比 N = 电机转 N 圈、转盘转 1 圈, 由机械决定, **改 MStep 不变**。
 *   - vel/acc 的单位是 RPM(电机轴), 与 MStep 无关 —— 改细分不用动速度参数;
 *     行程耗时只取决于 RPM 和角度, 所以 TURNTABLE_MS_PER_DEG / settle 也不用动。
 *
 * 现值 1208.888 = 51200 × 8.5 / 360 —— **2025 原工程(4f3ceb2)实车验证值**,
 *   隐含 MStep=256 + 减速比 8.5。
 * 旧值 67.16  = 3200 × 7.5556 / 360 —— 2026-09-17 台架在 **MStep=16** 下实测
 *   (`w`(+5°=6044 ticks) 实转 90°、`7`(108800 ticks) 实转约 4.5 圈
 *    → 实测计数域 ≈ 24178 ticks/转盘转)。
 *   **两个值各自都成立, 只是对应不同 MStep**; 18× = 16(细分比) × 1.125
 *   (减速比假设 8.5 与实测 7.5556 之差)。2026-09-20 把 MStep 由 16 改回 256
 *   (项目初始配置), 当量随之回到 1208.888。
 *
 * ⚠ **一次实测就能判定取值对不对** —— 冷机上电敲 `7`(目标 90°), 量实际转角 θ:
 *     θ ≈ 90°  → 减速比确为 8.5, 现值正确(当初那两次目测各偏约 12.5%)
 *     θ ≈ 101° → 减速比其实是 7.5556, 应改回 **1074.56**(= 67.16 × 16)
 *   万一当初 MStep 本就是 256(则当量该回到 67.16), 同样由 θ 暴露: θ ≈ 5.6°。
 *   通用反算: 新系数 = 1208.888 × 90 / θ。落角偏差 >2° 时改这里重烧,
 *   **驱动器侧参数不改**。 */
#define WTBL_TICK_PER_DEG   1208.888f   /* MStep=256 + 减速比 8.5(2025 原工程实车值) */

/* 上电零点对齐(转台"一条命令转好几圈"的根治)。
 * 现状问题: 转台是绝对定位(raF=true), 下发的是"驱动器脉冲计数域里的绝对目标",
 * 而 MCU 侧的 Car_data.Motor_Angle 只是"上次目标角"的开环记账, 从不读驱动器实际
 * 位置, 也从不回零(全工程零调用 Emm_V5_Reset_CurPos_To_Zero / Emm_V5_Origin_*)。
 * 于是 MCU 的 0° 与驱动器的 0 脉冲是否指向同一个物理姿态完全靠"上电时碰巧对上":
 * 一旦脱节(上次跑到哪 / MCU 单独复位而驱动器带电), 第一条命令的实际行程就是
 * |驱动器计数 − 目标| 而不是 |上次目标 − 本次目标|, 可以是好几圈 —— 而终点仍
 * 是那个绝对目标(90°=6044 脉冲), 于是表现为"转好几圈最后停在 90°"。
 *
 * 修法: 上电发一次 0x0A(清零当前位置, 只把驱动器计数器归零, **电机不动**),
 * 同时把 MCU 记账角置为 TURNTABLE_BOOT_DEG。两侧零点就此恒等, 此后任何命令的
 * 行程恒 = |本次目标 − 上次目标| ≤ 270°, 结构上不可能再多圈。
 *
 * 约定: TURNTABLE_BOOT_DEG 必须等于"上电那一刻转台的真实姿态"在码表坐标系里的
 * 角度。现取 0.0 = 沿用原设计的隐含假设(上电姿态即码表 0°), 行为零变化风险。
 * 若收车后转台实际停在回正位(90°), 别把这宏改成 90 —— 码 3 是 57°, 会出现
 * "目标 < 零点"的负脉冲, 绝对模式表达不了, 那种情况必须改用相对模式下发。
 * 要改就先在台架量: 上电后敲 '7', 实际转角 θ≠90° 说明上电姿态不是 0°, 该把
 * 转盘机械摆正(推荐)而不是改这个宏。 */
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
#if !ROUTE_TEST_TURNTABLE_EN
	/* test/route: 转台整体冻结 —— 排爆末段的自动摆码(码 7)、Brain 下发的
	 * TURNTABLE 命令都不驱动电机。本分支只验证底盘路线, 转盘挂在车上会碍事。
	 * tt_arm 因此恒 0, TIM5 的 Emm_Pos_Control 不会下发任何位置帧 */
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
	tt_arm = 1u;
	tt.busy = 1u;
	tt.code = turn_code;
	tt.ms = 0u;
	tt.settle_ms = Nav_TurnSettle(prev_deg, angle);   /* 按本次行程定 DONE 计时 */
	Car_Dbg2_TtRecord(1u, turn_code, angle);
}

/* 段入口自动转台动作: 启动该 nav_id 段时, 底盘自行把转盘摆到指定码位(0=本段不动转台)。
 * 排爆末段(6=NAV_REBASE_BOMB)入口摆 90° 扫描位(码 7) —— 与末段行驶并行, 复刻原工程
 * flag9 的时点。**触发权放底盘**的缘由: 本段含"最后一个转弯 + 末段直线 + 原地重锚",
 * 只有段表拥有者知道它何时开始; 转台扫掠与末段转弯的联合时序因此只在这一处维护,
 * 不必跨控制器对账(旧版主控在重锚状态入口发码 7、底盘注释却写"车停稳后才开转盘",
 * 两边说法相反)。主控侧只等 CAR_EVT_TURN_DONE(echo=7), 不再主动发码。
 * 只在这条"停旧起新"路径触发: 同 id 进行中/已完成补发两条幂等分支不重复开转盘 */
static const uint8_t nav_entry_turn[CAR_NAV_ID_COUNT] =
{
	0u, 0u, 0u, 0u, 0u, 0u, 7u, 0u,   /* 仅 NAV_REBASE_BOMB(6) -> 回正 90° */
};

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

#if ROUTE_TEST_EN
/* ---- test/route: 自测推进器(底盘自己扮演 Brain 的命令状态机) ----
 * 当前 nav 跑完(NAV_DONE)后按 route_test_seq 自动下发下一条 REQ_NAV。
 * 走的是 Brain 下发命令的**同一个入口 Nav_DoRequest**, 区别只是命令不来自串口信箱,
 * 所以段表装载/转台入口/幂等语义与原版完全一致, 核对的就是原版每张表。
 * 调用点在 Car_Nav_Task(10ms 定时)里, 与 nav 状态机同上下文, 无访问竞争 */
static void RouteTest_OnNavDone(void)
{
	if (rt_idx >= ROUTE_TEST_SEQ_LEN)            return;   /* 序列已跑完: 停在返回区 */
	if (nav.nav_id != route_test_seq[rt_idx])    return;   /* 不是本序列的 nav(手动下发): 不接管 */
	rt_idx++;
	if (rt_idx >= ROUTE_TEST_SEQ_LEN)
	{
		rt_finished = 1u;   /* 到返回区了, 主循环打一次 RT:FINISH */
		return;
	}
	Nav_DoRequest(route_test_seq[rt_idx], ROUTE_TEST_GEAR);
}

/* ---- 停等漂移诊断(只测不改, 回答"停 3s 是不是在偷里程") ----
 * 机制: odom 每拍无条件积分(mf_tick -> mf_odom_update, 与段型无关), 但段进度
 * dist_accum 只在 MOVE/ARC 段累加 ds、且**进段那一拍刻意丢弃**。于是 WAIT 段
 * 3 秒(300 拍)里车体任何挪动——减速尾巴、惯性滑行、坡道溜车、履带回弹、编码器
 * 抖动——全部进了 pose.x/y, 一分都进不了 dist_accum, 成为"白走里程"。
 * 下一段 MOVE 从 dist_accum=0 重新开始, 不会把它扣回去, 所以每段白走一点、
 * 18 段线性叠加, 表现为终点"路程累积或多或少"。
 * 本函数在 WAIT 段进/出沿各抓一次位姿:
 *   SEG   = 本段起点(上次停等结束) -> 本次停等开始, 即该运动段实际走的位移
 *   PAUSE = 停等期间车体自己挪了多少(白走里程, 期望≈0)
 *   TOT   = 全程白走里程累加 */
static uint8_t  pd_in_wait;                     /* 1=当前在 WAIT 段内(entry 已抓) */
static uint8_t  pd_valid;                       /* 1=有一组新数据待主循环打印 */
static uint16_t pd_seq;                         /* 停等序号 */
static float    pd_entry_x, pd_entry_y;         /* WAIT 进段位姿 */
static float    pd_seg_x,  pd_seg_y;            /* 本段起点(=上次停等结束位姿) */
static float    pd_seg_dx, pd_seg_dy;           /* 本段实际位移 m */
static float    pd_pau_dx, pd_pau_dy;           /* 停等期间位移 m */
static float    pd_total_x, pd_total_y;         /* 白走里程累计(有符号分量 m):
                                                 * 单向漂移会线性累加, 来回抖动会互相抵消,
                                                 * 据此可区分"系统性溜车"与"随机抖动" */

static void RouteTest_DriftSample(const mf_ctx_t *m)
{
	const mf_pose_t *p = mf_get_pose(m);

	if (mf_is_waiting(m) != 0u)
	{
		if (pd_in_wait == 0u)               /* WAIT 进沿 */
		{
			pd_in_wait = 1u;
			pd_entry_x = p->x_m;
			pd_entry_y = p->y_m;
			pd_seg_dx  = p->x_m - pd_seg_x; /* 本段实际位移 */
			pd_seg_dy  = p->y_m - pd_seg_y;
		}
	}
	else if (pd_in_wait != 0u)              /* WAIT 出沿 */
	{
		pd_in_wait = 0u;
		pd_pau_dx  = p->x_m - pd_entry_x;   /* 停等白走 */
		pd_pau_dy  = p->y_m - pd_entry_y;
		pd_total_x += pd_pau_dx;            /* 有符号累加: 看漂移方向是否一致 */
		pd_total_y += pd_pau_dy;
		pd_seq++;
		pd_valid   = 1u;
		pd_seg_x   = p->x_m;                /* 下一段起点 */
		pd_seg_y   = p->y_m;
	}
}

/* 供调试口读取: 返回 1 表示有新数据; 读走后 pd_valid 自动清零 */
uint8_t Car_RouteTest_Drift(uint16_t *seq, float *seg_dx, float *seg_dy,
                            float *pau_dx, float *pau_dy,
                            float *tot_x, float *tot_y)
{
	if (pd_valid == 0u) { return 0u; }
	pd_valid = 0u;
	if (seq != 0)    { *seq    = pd_seq; }
	if (seg_dx != 0) { *seg_dx = pd_seg_dx; }
	if (seg_dy != 0) { *seg_dy = pd_seg_dy; }
	if (pau_dx != 0) { *pau_dx = pd_pau_dx; }
	if (pau_dy != 0) { *pau_dy = pd_pau_dy; }
	if (tot_x != 0)  { *tot_x  = pd_total_x; }
	if (tot_y != 0)  { *tot_y  = pd_total_y; }
	return 1u;
}

/* 上电起跑: main.c 在看门狗初始化之后调用一次, 相当于 Brain 发的第一条 REQ_NAV。
 *
 * ⚠️ 必须**投递到命令信箱**, 不能直接调 Nav_DoRequest():
 * 本函数在 while(1) 之前跑, 那时第一个 TIM4 中断(10ms)还没到, g_mf 尚未 mf_init;
 * 同步调用 Nav_DoRequest 会把段表挂到未初始化的 mf 上, 随后 Car_Nav_Task 第一拍的
 * mf_app_init() -> mf_init() -> mf_path_init() 把它连同 state 一起抹掉, 而 nav.state
 * 已被置成 NAV_RUN —— 结果是 mf_is_done() 永假(IDLE≠DONE)、mf_tick 无输出、车一动
 * 不动, 3s 后堵转看门狗报 NAV_FAULT 收场(连 NAV_DONE 都不发)。
 * 走信箱则消费发生在 mf_app_init() 之后(见 Car_Nav_Task: 先 init 后 while(mail)),
 * 顺序天然正确, 且与 Brain 下发命令的路径完全一致。
 * 信箱深度 8, 这里单生产者只投一次; ISR 只改 mail_tail 不改 mail_head, 无竞争 */
void Car_RouteTest_BootStart(void)
{
	rt_idx = 0u;
	rt_finished = 0u;
	Car_Nav_MailPush(CAR_SUB_REQ_NAV, route_test_seq[0], ROUTE_TEST_GEAR);
}

/* 供调试口显示进度: idx=当前序列下标, total=序列长度, finished=是否已到返回区 */
void Car_RouteTest_Progress(uint8_t *idx, uint8_t *total, uint8_t *finished)
{
	if (idx != 0)      { *idx = rt_idx; }
	if (total != 0)    { *total = ROUTE_TEST_SEQ_LEN; }
	if (finished != 0) { *finished = rt_finished; }
}
#endif

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
	tracking.state = TRACKING_IDLE;
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
#if !ROUTE_TEST_TURNTABLE_EN
	(void)delta_deg;   /* test/route: 转台冻结, 台架微调亦不开电机 */
	return;
#endif
	__disable_irq();
	tt_arm = 0u;
	Car_data.Motor_Angle += delta_deg;
	tt_arm = 1u;
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
#if ROUTE_TEST_EN
	/* 停等漂移采样(只测不改)。必须放在 mf_is_done 分支**之前**: 最后一段的
	 * WAIT 出沿与 path 置 DONE 同拍发生, 放后面会因提前 return 而漏采 */
	RouteTest_DriftSample(m);
#endif
	if (mf_is_done(m))
	{
		nav.state = NAV_DONE;
		Car_Event_Send(CAR_EVT_NAV_DONE, nav.nav_id, 0x00u);
#if ROUTE_TEST_EN
		RouteTest_OnNavDone();   /* 自测: 按原版命令序列自动推进到下一条 nav */
#endif
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
		else if (mf_is_waiting(m) != 0u)
		{
			/* 段表 WAIT 段(测试路线每段后的 3s 停等)本就要求车体静止。
			 * 不豁免的话: ROUTE_TEST_PAUSE_MS=3000 与 NAV_STALL_MS=3000 正好
			 * 同宽, 每次停等到点必判堵转 -> NAV_FAULT 中途停车, 路线跑不完 */
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
		/* 等效轨距: 须按 Motion/README.md 原地转 90° 标定(编码器差分航向对齐 IMU)。
		 * 旧注 250mm 是几何中心距, 与标定值不等价; 本值同时是弧长反解的基准,
		 * 偏大 → 转弯里程等比偏大, 偏小 → 转弯里程等比偏小(mf_odom.c) */
		.track_width_m = 0.25f,
		.half_length_m = 0.0f,
		.half_width_m  = 0.0f,
		.max_vx_mps    = 1.5f,           /* 覆盖 High_speed 1.41m/s */
		.max_vy_mps    = 0.0f,
		.max_wz_dps    = 150.0f,         /* Middle 弧线稳态 57dps + 修正余量 */
		.yaw_kp        = 16.0f,    /* 走直线蛇形修复: 外环降带宽匹配轮速内环(原32.4欠阻尼~1s极限环) */
		.yaw_ki        = 0.0f,
		.yaw_kd        = 0.8f,     /* 航向环加阻尼项, 抑制匀速直行左右摆 */
		.yaw_i_max     = 0.0f,
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
	const mf_status_t *st;
	const mf_pose_t *pose;
	mf_ctx_t *m;

	uint8_t  nav_state, nav_id, seg_done, trk_state, trk_found, tt_armed, tt_code;
	uint16_t seg_idx, trk_frame_age, trk_hold, tt_ms, tt_settle;
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
	       "IRL"[trk_state & 0x03u], err_f10k, (unsigned)trk_found,
	       (unsigned)trk_frame_age, (unsigned)trk_hold);
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
/* TIM5 100ms 转台重发(经 interurp.c 调用)。
 * tt_arm 门控: 上电后首条 TURNTABLE 命令前/急停后不下发——旧版上电即向
 * 绝对 0°(=驱动器上电零点)发位置, 仅 MCU 复位时转台会自行大行程甩动。
 * 方向+角度关中断成对快照, 防 TIM4 更新中途读到混合对挥向另一侧。
 * acc=20: 旧值 0 为直接启动, 带载 180RPM 零斜坡起转冲击/失步 */
void Emm_Pos_Control(void)
{
	uint8_t  dir;
	float    deg;
	uint32_t tick;

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
	tt_last_tick = tick;   /* 台架标定读数('?' 的 WTBL 行 TICK=) */
	/* vel=180 / acc=20 都是**电机轴 RPM 口径**, 与 MStep 无关:
	 * 改驱动器细分时只改 WTBL_TICK_PER_DEG, 这两个不用动(行程耗时只取决于 RPM 与角度) */
	Emm_V5_Pos_Control(EMM_MOTOR_ADDR, dir, 180, 20, tick, true, false);
}

/* 上电转台零点对齐 —— 见 TURNTABLE_BOOT_DEG 处的长注释。
 * 发 0x0A「清零当前位置」: 只把驱动器计数器归零, **电机不动**; 随后把 MCU 记账角
 * 置成同一个物理姿态对应的角度。两侧零点恒等后, 第一条命令的行程就是
 * |本次目标 − 上电姿态|, 不再是 |驱动器残留计数 − 目标|。
 * 调用时点必须是 TIM5 启动之前(此时 tt_arm 恒 0, 无重发竞争, huart6 空闲),
 * 且要在驱动器上电自检完成之后(张大头上电到可收命令约 1s 级)。 */
void Car_Turntable_BootZero(void)
{
	tt_arm = 0u;
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