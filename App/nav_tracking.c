#include "nav_tracking.h"
#include "car_units.h"
#include "stm32f4xx.h"      /* __disable_irq/__enable_irq(CMSIS core) */
#include "motion_app.h"
#include "mf_path.h"
#include "mf_math.h"
#include "mf_kinematics.h"
#include "car_link.h"
#include <math.h>           /* sqrtf: trim 滑行预算二次方程(同 mf_path.c) */

static struct
{
	TrackingState_t state;
	float    horizontal_error_normalized;  /* 最新归一化横向误差, 约 -1~+1 */
	uint8_t  target_found_flag;            /* 最新有效标志 */
	uint8_t  error_fresh;                  /* 本拍消费到一帧(无论 found); 斜率窗口据此喂样 */
	uint16_t frame_age_ms;                 /* 距上次有效误差帧时长 */
	uint16_t hold_ms;                      /* 进阈即刹: |e|≤阈 的连续累计 */
	int8_t   zero_sign;                    /* 已确认误差符号(+1/-1/0=未确认), 过零判据的基准 */
	int8_t   sign_cand;                    /* 去抖候选符号 */
	uint8_t  confirm_cnt;                  /* 去抖: 候选符号连续同号帧数 */
	uint8_t  braking;                      /* 已触发制动的锁存: 不再重判, 只走制动斜坡 */
	float    brake_thr;                    /* 当拍实际生效的点火阈(观测用; 触发后不再刷新=保留点火值) */
	float    brake_cap;                    /* 制动斜坡: 当拍允许的段表目标速度上限 */
	int16_t  fire_e_f10k;                   /* 触发制动那一拍的 E(定点, 观测用): C = E_stop - 本值;
	                                        * 0=本轮尚未点火(指令刹停/FAULT 不刷它) */
	uint16_t noconv_ms;                    /* 段表已完 + |e|>阈 的连续累计(FAULT 01) */
	uint8_t  noconv_latch;                 /* FAULT(01) 闩锁: 防重复上报, START/STOP 清 */
	uint16_t lost_ms;                      /* 段表已完 + 无测量的连续累计(FAULT 02) */
	uint8_t  stop_done;                    /* STOPPED 已报 TRACKING_DONE 的闩锁(START 清, 补发用) */
	uint8_t  fault_reason;                 /* TRACKING_FAULT 的 reason, 见本头 TRK_FAULT_* */
	uint8_t  near_cap;                     /* 近靶二级限速当拍是否生效(纯观测, TRKD 的 N=) */
	uint16_t settle_ms;                    /* 物理停稳驻留: 轮速实测≤阈的连续累计(SETTLE_MS) */
	uint16_t settle_to_ms;                 /* 停稳确认总等待: 超过 GIVEUP 强制放行(测速异常兜底) */
	uint8_t  settling;                     /* 目标已到零但还在等物理停稳(纯观测, TRKD 的 W=) */
} tracking;

/* 状态迁移唯一入口: 复位判据计数器 + 登记一次迁移打点(主循环 Car_TrkChange_Log 消费)。
 * 只走这里是为了让"哪一拍切的态、为什么切"有据可查, 也防止某条路径漏清计数 */
static volatile uint8_t trk_chg_from, trk_chg_to, trk_chg_fresh;

/* ---- 误差序列斜率估计(预测式提前制动的唯一输入) ----
 * 帧率在 18~23Hz 之间飘, 差分必须按**实际间隔**归一, 不能写死帧周期(与航向差分同一个坑,
 * 见 docs/坑-编码器与里程.md §7)。时间戳用本模块自己的拍计数, 不借 frame_age_ms ——
 * 后者会被重复帧重置, 拿它当时间基准等于让重复帧同时污染帧龄与斜率。 */
static struct
{
	float    e[TRACKING_ERR_WIN];
	uint32_t ms[TRACKING_ERR_WIN];
	uint8_t  head;    /* 下一个写入位 */
	uint8_t  count;
} trk_rate;

static uint32_t trk_ms;   /* 每拍 +CAR_TICK_MS; 只用差值, 不拍回绕 */

/* ---- 刹停后微调(trim)状态: 见 TRACKING_TRIM_* (nav_tracking.h) 与 §10.18 ----
 * 律与 build_verify/trim_loop_sim.py 一一对应(该脚本全参数扫 100% 收敛后才落的固件)。
 * 里程与滞后全部在线学习, 不写死任何 px↔mm 换算(旧注释两套口径矛盾已删) */
static struct
{
	uint8_t  phase;        /* TRIM_PHASE_* */
	uint8_t  iter;         /* 已执行的移动次数 */
	uint8_t  outcome;      /* 终态成因: TRIM_OUTCOME_*(0=收敛, 1~5=护栏细分) */
	int16_t  resid_f10k;   /* 终态那拍的 E(定点), 进 EVT echo 折算 + TRKD 的 R= */
	float    gain_cpmm;    /* E->里程增益(计数/mm), 在线学习 */
	float    lag_s;        /* 停令后滑行滞后(s), 在线学习 = over/v */
	float    v_mps;        /* 当前移动的爬行速度(带符号) */
	float    dcmd_mm;      /* 当前移动的指令里程(恒正幅值, 方向在 v_mps) */
	float    dcut_mm;      /* 停令那拍的累计轮程幅值(-1=未切割): 滑行学习的分界 */
	float    travel0_mm;   /* 当前移动起点的累计轮程(带符号) */
	float    travel_mm;    /* STOPPED 期累计轮程(带符号, wheel delta 累加) */
	float    budget_mm;    /* 前向预算 = 段表剩余里程 - margin(不许吃穿名义端点) */
	int16_t  estart_f10k;  /* 当前移动起点的 E(学习用) */
	uint32_t t0_ms;        /* trim 总计时起点(GIVEUP_MS) */
	uint32_t eval_t0_ms;   /* 本轮 EVAL 等帧计时起点(FIRST_WAIT_MS) */
	uint32_t move_t0_ms;   /* 当前移动起点(单步超时守卫) */
	uint32_t frame_seen_ms;/* 最近一次真误差帧的 trk_ms。STOPPED 态 frame_age_ms 无人
	                        * 递增(递增在 ARMED 分支), 断流时旧值恒"新鲜" —— trim 的
	                        * 新鲜判据只能用这个独立时戳, 不归零(跨态有效) */
} trim;

static void Trim_ResetAll(void)
{
	trim.phase = TRIM_PHASE_IDLE;
	trim.iter = 0u;
	trim.outcome = TRIM_OUTCOME_CONV;
	trim.resid_f10k = 0;
	trim.gain_cpmm = 0.0f;
	trim.lag_s = 0.0f;
	trim.v_mps = 0.0f;
	trim.dcmd_mm = 0.0f;
	trim.dcut_mm = -1.0f;
	trim.travel0_mm = 0.0f;
	trim.travel_mm = 0.0f;
	trim.budget_mm = 0.0f;
	trim.estart_f10k = 0;
	trim.t0_ms = 0u;
	trim.eval_t0_ms = 0u;
	trim.move_t0_ms = 0u;
	/* frame_seen_ms 不清: 测量时戳归零会伪造"刚见过帧" */
}

/* ---- trim 事件环形缓冲: 归因黑匣子(TRKT 行, §10.18) ----
 * ISR 生产(TrimTick), 主循环读后清(Car_TrimLog)。一轮 trim ≤6 条, 环 12 条够两轮
 * 完整历史; 满则覆盖最旧 —— 终态(END)记录不可丢, 丢早期 MOVE 记录可以接受 */
#define TRIM_LOG_DEPTH  12u
static TrkTrimLog_t trim_log[TRIM_LOG_DEPTH];
static volatile uint8_t trim_log_head, trim_log_count;

static void Trim_LogPush(uint8_t type, float e_counts, int16_t f1, int16_t f2, uint8_t aux)
{
	TrkTrimLog_t *r;

	__disable_irq();
	if (trim_log_count >= TRIM_LOG_DEPTH)
	{
		trim_log_head = (uint8_t)((trim_log_head + 1u) % TRIM_LOG_DEPTH);   /* 覆盖最旧 */
		trim_log_count--;
	}
	r = &trim_log[(uint8_t)((trim_log_head + trim_log_count) % TRIM_LOG_DEPTH)];
	r->ms = trk_ms;
	r->e_f10k = (int16_t)e_counts;
	r->f1 = f1;
	r->f2 = f2;
	r->type = type;
	r->aux = aux;
	trim_log_count++;
	__enable_irq();
}

/* 读后清(主循环 Car_TrimLog 消费): 返回 0 = 无记录 */
uint8_t Tracking_TakeTrimLog(TrkTrimLog_t *out)
{
	uint8_t has;

	__disable_irq();
	has = (uint8_t)((trim_log_count != 0u) ? 1u : 0u);
	if (has != 0u)
	{
		*out = trim_log[trim_log_head];
		trim_log_head = (uint8_t)((trim_log_head + 1u) % TRIM_LOG_DEPTH);
		trim_log_count--;
	}
	__enable_irq();
	return has;
}

static void Trk_RateClear(void)
{
	trk_rate.head = 0u;
	trk_rate.count = 0u;
}

static void Trk_RateFeed(float e)
{
	uint8_t prev, slot;

	if (trk_rate.count != 0u)
	{
		prev = (uint8_t)((trk_rate.head + TRACKING_ERR_WIN - 1u) % TRACKING_ERR_WIN);
		if ((uint32_t)(trk_ms - trk_rate.ms[prev]) < TRACKING_ERR_MIN_DT_MS)
		{
			/* 同一帧被重复投递: 相邻 |de/dt| 恒 0, 会把斜率直接拖平 ⇒ 静默失效 */
			return;
		}
	}
	slot = trk_rate.head;
	trk_rate.e[slot] = e;
	trk_rate.ms[slot] = trk_ms;
	trk_rate.head = (uint8_t)((trk_rate.head + 1u) % TRACKING_ERR_WIN);
	if (trk_rate.count < (uint8_t)TRACKING_ERR_WIN)
	{
		trk_rate.count++;
	}
}

/* 最小二乘斜率(e/s)。返回 0 = 窗口不足/跨度太短/退化 ⇒ 调用方退回固定阈 */
static float Trk_RateSlope(void)
{
	uint8_t n = trk_rate.count;
	uint8_t oldest, i;
	float   t0, st = 0.0f, se = 0.0f, stt = 0.0f, ste = 0.0f, den;

	if (n < 3u)
	{
		return 0.0f;
	}
	oldest = (uint8_t)((trk_rate.head + TRACKING_ERR_WIN - n) % TRACKING_ERR_WIN);
	t0 = trk_rate.ms[oldest];
	if ((uint32_t)(trk_rate.ms[(uint8_t)((trk_rate.head + TRACKING_ERR_WIN - 1u) % TRACKING_ERR_WIN)] - t0)
	    < TRACKING_ERR_MIN_SPAN_MS)
	{
		return 0.0f;
	}
	for (i = 0u; i < n; i++)
	{
		uint8_t idx = (uint8_t)((oldest + i) % TRACKING_ERR_WIN);
		float   t   = (float)(trk_rate.ms[idx] - t0) * 0.001f;

		st  += t;
		se  += trk_rate.e[idx];
		stt += t * t;
		ste += t * trk_rate.e[idx];
	}
	den = ((float)n * stt) - (st * st);
	if (den <= 0.0f)
	{
		return 0.0f;
	}
	return (((float)n * ste) - (st * se)) / den;
}

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
	tracking.near_cap = 0u;
	tracking.settle_ms = 0u;
	tracking.settle_to_ms = 0u;
	tracking.settling = 0u;
	tracking.noconv_ms = 0u;
	tracking.lost_ms = 0u;
	tracking.error_fresh = 0u;
	Trk_RateClear();
	Trim_ResetAll();   /* trim 字段一并复位; TrimBegin/Goto(STOPPED) 之后会重填初值 */
	/* 注意**不清** noconv_latch —— 它由 START/STOP 专门清(见 Tracking_Start/Stop),
	 * 好让 Brain 的容错计数能在每次重开后重新累计 */
	__disable_irq();
	trk_chg_from = (uint8_t)from;
	trk_chg_to = (uint8_t)next;
	trk_chg_fresh = 1u;
	__enable_irq();
}

char Tracking_StateChar(TrackingState_t s)
{
	switch (s)
	{
	case TRACKING_ARMED:   return 'A';
	case TRACKING_STOPPED: return 'S';
	case TRACKING_FAULT:   return 'F';
	default:                return 'O';
	}
}

/* trim 子相位字母(TRKD 行的 T= 字段) */
char Tracking_TrimChar(uint8_t trim_phase)
{
	switch (trim_phase)
	{
	case TRIM_PHASE_EVAL:     return 'E';
	case TRIM_PHASE_CREEP:    return 'C';
	case TRIM_PHASE_SETTLE:   return 'S';
	case TRIM_PHASE_FINISHED: return 'F';
	default:                  return '-';
	}
}

#define TRACKING_ERROR_FIXED_SCALE  10000.0f

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

/* ---- 段内跟瞄: 段表不断; 视觉只当"该刹停"的判据, 不写 vx/wz ---- */
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
	/* 同上, 硬置零期间速率内环的输出被丢弃, 积分必须一起泄放(否则放行即甩转向) */
	mf_clear_rate(motion);
}

static void Tracking_RaiseFault(uint8_t reason);   /* 前置声明: Tracking_Start 无段表时调 */

/* 收到 TRACKING_START: 武装跟瞄(段表全权, 底盘只评估刹停判据)。
 * START 是**无状态标志** —— 重发不改变段表状态、不清测量、不换态, 天然幂等;
 * 从 FAULT 重收 START = 主控要求重开一轮(清未收敛闩锁, 好让容错计数能再累计)。
 * speed_level 不参与逻辑(进近低速由咬靶区自动决定); aim_axis 仅做入参范围校验 */
void Tracking_Start(uint8_t tracking_speed_level, uint8_t aim_axis)
{
	mf_ctx_t *motion = mf_app();
	mf_path_state_t path_state = mf_path_get_state(&motion->path);

	(void)tracking_speed_level;   /* 进近低速由咬靶区自动决定, 该档位不再生效 */

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
	tracking.fire_e_f10k = 0;       /* 新一轮跟瞄: EF=0 直到本拍真点火 */
	tracking_error_pending = 0u;    /* 丢弃上轮残帧, 首拍不等旧误差 */
	tracking.noconv_latch = 0u;     /* START/STOP 清未收敛闩锁, 允许 Brain 的容错计数继续累计 */
	Tracking_Goto(TRACKING_ARMED);
}

/* 指令刹停(0x07 mode0)的 STOPPED 入口: 全零驻停 + 登记终态。
 * DONE 由 STOPPED 分支统一发出(≤1 拍), echo/reason=0 与旧版逐字节一致。
 * 与自动刹停路径(Tracking_TrimBegin)的分工同 SettleCheck 先例: 主控点名要停的
 * 不做微调 —— DONE 即时语义不变 */
static void Tracking_EnterStopped(mf_ctx_t *motion)
{
	Tracking_HoldStill(motion);
	Tracking_Goto(TRACKING_STOPPED);   /* Goto 含清 stop_done, 闩锁由 STOPPED 分支发送时置 */
	trim.phase = TRIM_PHASE_FINISHED;  /* 指令路径无微调: 直接终态, 残余/护栏均为 0 */
}

/* 触发制动(锁存): 记下当拍速度作为制动斜坡起点。制动不是瞬间压零 —— 段表按固定减速
 * 度把目标速度压到 0, 滑行更短、停点可复现(规格 §5) */
static void Tracking_BrakeStart(mf_ctx_t *motion)
{
	tracking.braking = 1u;
	tracking.fire_e_f10k = (int16_t)(tracking.horizontal_error_normalized
	                                * TRACKING_ERROR_FIXED_SCALE);
	tracking.brake_cap = motion->path.cur_speed;
	tracking.settle_ms = 0u;       /* 停稳计时从制动起点锚定: 制动段 ≤0.45/BRAKE≈112ms,
	                                * 计入 GIVEUP 预算仍余 ~890ms, 不必另设"目标到零"迁移检测 */
	tracking.settle_to_ms = 0u;
	tracking.settling = 0u;
}

/* 制动斜坡一拍: 把 brake_cap 沿固定减速度往 0 收(带符号, 倒车段也适用), 并只**往下**压
 * 段表当拍目标速度。返回 1 = 已到零, 该进 STOPPED。必须在 mf_tick **之后**调用:
 * 本拍压的是"下一拍 mf_tick 起坡的起点", 故本拍不改 cmd(轮速环不重算) */
static uint8_t Tracking_BrakeRamp(mf_ctx_t *motion)
{
	const float step = TRACKING_BRAKE_MPS2 * ((float)CAR_TICK_MS * 0.001f);

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

/* 物理停稳确认: 制动**目标**到零 ≠ 车停了 —— 低速轮速环制动偏软(0.15m/s 下实测等效
 * ~0.6m/s², 远低于 BRAKE 4), 目标零后还要滑 ~10mm, 此刻报 DONE 会让 Brain 开火时
 * 枪口还在滑, 弹着点系统性早于停点(现场"擦边"根因之一)。轮速实测 ≤ 阈驻留
 * SETTLE_MS 才放行; GIVEUP 兜底防测速异常导致 DONE 永不来。
 * 主刹停路径按 TRACKING_SETTLE_EN 开关走这里; trim 的 SETTLE 相位无条件走
 * (滑行没停完就学习 L/gain 会把估计污染, 且 trim 自有总时长护栏兜底) */
static uint8_t Tracking_SettleCheck(mf_ctx_t *motion)
{
	float sp = mf_absf(motion->wheel[0].speed_mps);

	if (mf_absf(motion->wheel[1].speed_mps) > sp)
	{
		sp = mf_absf(motion->wheel[1].speed_mps);
	}
	if (sp > TRACKING_SETTLE_MPS)
	{
		tracking.settle_ms = 0u;               /* 还在滑/回弹: 驻留重新计 */
	}
	if (tracking.settle_to_ms < (uint16_t)(0xFFFFu - CAR_TICK_MS))
	{
		tracking.settle_to_ms = (uint16_t)(tracking.settle_to_ms + CAR_TICK_MS);
	}
	if (sp <= TRACKING_SETTLE_MPS)
	{
		if (tracking.settle_ms < (uint16_t)(0xFFFFu - CAR_TICK_MS))
		{
			tracking.settle_ms = (uint16_t)(tracking.settle_ms + CAR_TICK_MS);
		}
	}
	return (uint8_t)((tracking.settle_ms >= TRACKING_SETTLE_MS) ||
	                 (tracking.settle_to_ms >= TRACKING_SETTLE_GIVEUP_MS));
}

/* ---- 微调(trim): 终态登记 / 入口 / 每拍相位机 ----
 * 只在自动刹停(点火制动+停稳)路径启用; 指令刹停(Tracking_Stop(0))即时终态不进微调。
 * 里程货币 = wheel[].delta_m 自累计(与 mf_path 的 dist_accum 同源同号, 段表按实际轮程
 * 自动入账, STOP(1) 放行补完的剩余里程自动扣除微调量 —— 无需任何新接口) */

/* 微调终态登记(不动 stop_done 闩锁 —— 发送统一在 STOPPED 分支)。
 * outcome 见 TRIM_OUTCOME_*: 0=收敛, 1~5=护栏细分(归因用, EVT reason 透传) */
static void Tracking_TrimEnd(mf_ctx_t *motion, uint8_t outcome, float e_counts)
{
	Tracking_HoldStill(motion);
	trim.phase = TRIM_PHASE_FINISHED;
	trim.outcome = outcome;
	trim.resid_f10k = (int16_t)e_counts;
	Trim_LogPush(TRKT_END, e_counts, (int16_t)outcome,
	             (int16_t)(trim.gain_cpmm * 10.0f), trim.iter);
}

/* 自动刹停路径的 STOPPED 入口: 进微调评估。前进预算 = 段表剩余里程 - margin,
 * 微调的全部前进量都从段表账面走(轮程自动入账), 不许把累计器推过名义端点 ——
 * 否则段表在微调中途自完成(path DONE 与 trim 交错) */
static void Tracking_TrimBegin(mf_ctx_t *motion)
{
	float budget;

	Tracking_HoldStill(motion);
	Tracking_Goto(TRACKING_STOPPED);       /* 复位全部判据与 trim 字段 */
	/* 必须用本段剩余: 全路径剩余会被后续位移段稀释; 非 MOVE 段返回 0 = 拒绝微调
	 * (TURN 期间 dist_accum 不入账, 在那爬行会把账记丢, 见 mf_path_seg_remain_m) */
	budget = mf_path_seg_remain_m(&motion->path) * 1000.0f
	         - (TRACKING_TRIM_BUDGET_MARGIN_M * 1000.0f);
	trim.gain_cpmm = TRACKING_TRIM_GAIN_DEF;
	trim.lag_s = TRACKING_TRIM_L_INIT_S;
	trim.budget_mm = (budget > 0.0f) ? budget : 0.0f;
	trim.t0_ms = trk_ms;
	trim.eval_t0_ms = trk_ms;
	trim.phase = TRIM_PHASE_EVAL;
	{
		int32_t b10 = (int32_t)(trim.budget_mm * 10.0f);

		if (b10 > 30000) { b10 = 30000; }   /* int16 限幅(防御; 本段剩余不会到 3m) */
		Trim_LogPush(TRKT_BEGIN,
		             tracking.horizontal_error_normalized * TRACKING_ERROR_FIXED_SCALE,
		             (int16_t)b10, 0, 0u);
	}
}

/* STOPPED 态每拍的微调相位机。分支头部已 HoldStill(全零驻停), 只有 CREEP 覆写 cmd。
 * 前置: Tracking_ConsumeErrorFrame 本拍已跑(nav_task/ApplyOverlay 头部), 误差新鲜 */
static void Tracking_TrimTick(mf_ctx_t *motion)
{
	float e, abs_e;
	uint8_t fresh;

	trim.travel_mm += (motion->wheel[0].delta_m + motion->wheel[1].delta_m) * 500.0f;
	fresh = (uint8_t)((tracking.target_found_flag != 0u) &&
	                  ((trk_ms - trim.frame_seen_ms) <= TRACKING_FRAME_TIMEOUT_MS));

	switch (trim.phase)
	{
	case TRIM_PHASE_EVAL:
	{
		uint8_t usable = fresh;

		e = tracking.horizontal_error_normalized * TRACKING_ERROR_FIXED_SCALE;
		abs_e = (e < 0.0f) ? -e : e;
		if ((usable != 0u) && (abs_e >= TRACKING_ERR_SAT_LEVEL * TRACKING_ERROR_FIXED_SCALE))
		{
			usable = 0u;   /* found=1 但钉在量程沿(现场实测连续 2.5s): 与 ARMED 同判不可用,
			                * 否则按它规划会走出 budget 级的巨步 */
		}
		if (usable == 0u)
		{
			if ((trk_ms - trim.eval_t0_ms) >= TRACKING_TRIM_FIRST_WAIT_MS)
			{
				/* 视觉死/断流/饱和: 车已物理停稳, 护栏放行(残余 = 最后一次测量, 观测用) */
				Tracking_TrimEnd(motion, TRIM_OUTCOME_NOFRAME, e);
			}
			break;
		}
		if (abs_e <= (float)TRACKING_TRIM_TOL_E)
		{
			Tracking_TrimEnd(motion, TRIM_OUTCOME_CONV, e);   /* 已在窗内: 收敛 */
			break;
		}
		if ((trim.iter >= TRACKING_TRIM_MAX_ITER) ||
		    ((trk_ms - trim.t0_ms) >= TRACKING_TRIM_GIVEUP_MS))
		{
			Tracking_TrimEnd(motion, TRIM_OUTCOME_CAP, e);    /* 护栏放行, 残余照报 */
			break;
		}
		/* 规划(与 trim_loop_sim.py 逐式对应): d_need=|E|/gain;
		 * 滑行预算=min(d_need/2, 3mm); v 解 "L·v+v²/2a=预算"(小修正自动慢爬);
		 * 再约束步时长 >= 1 名义帧(否则整步在两帧之间完成, E 背停无采样机会);
		 * d_cmd = d_need - slide: 瞄短一个滑行量, 停稳正好落在靶心 */
		{
			float d_need      = abs_e / trim.gain_cpmm;
			float slide_budget = ((d_need * 0.5f) < 3.0f) ? (d_need * 0.5f) : 3.0f;
			float al          = TRACKING_BRAKE_MPS2 * trim.lag_s;
			float v           = sqrtf(al * al + 2.0f * TRACKING_BRAKE_MPS2
			                                 * slide_budget * 0.001f) - al;
			float d_cmd       = d_need;
			uint8_t i;

			v = (v < TRACKING_TRIM_V_MIN_MPS) ? TRACKING_TRIM_V_MIN_MPS : v;
			v = (v > TRACKING_TRIM_V_MAX_MPS) ? TRACKING_TRIM_V_MAX_MPS : v;
			for (i = 0u; i < 2u; i++)
			{
				float slide = (v * trim.lag_s + v * v / (2.0f * TRACKING_BRAKE_MPS2)) * 1000.0f;
				float v_cap;

				d_cmd = d_need - slide;
				if (d_cmd < TRACKING_TRIM_MIN_STEP_MM) { d_cmd = TRACKING_TRIM_MIN_STEP_MM; }
				v_cap = d_cmd * 0.001f / TRACKING_TRIM_FRAME_S;
				if (v > v_cap) { v = v_cap; }   /* 单向钳位(步时长>=1帧), 与仿真逐式一致 */
				v = (v < TRACKING_TRIM_V_MIN_MPS) ? TRACKING_TRIM_V_MIN_MPS : v;
				v = (v > TRACKING_TRIM_V_MAX_MPS) ? TRACKING_TRIM_V_MAX_MPS : v;
			}
			if (e > 0.0f)
			{
				/* E>0 = 靶心仍在车前方: 前进, 受段表剩余预算钳位(防吃穿名义端点) */
				if (d_cmd > trim.budget_mm) { d_cmd = trim.budget_mm; }
			}
			else
			{
				float cap = TRACKING_TRIM_MAX_MOVE_M * 1000.0f;

				if (d_cmd > cap) { d_cmd = cap; }    /* 后退: 单步上限(remain 只增不减, 安全) */
			}
			if (d_cmd < TRACKING_TRIM_MIN_STEP_MM)
			{
				Tracking_TrimEnd(motion, TRIM_OUTCOME_BUDGET, e); /* 预算/上限吃光 */
				break;
			}
			trim.v_mps       = (e > 0.0f) ? v : -v;
			trim.dcmd_mm     = d_cmd;
			trim.estart_f10k = (int16_t)e;
			trim.travel0_mm  = trim.travel_mm;
			trim.move_t0_ms  = trk_ms;
			trim.dcut_mm     = -1.0f;
			trim.phase       = TRIM_PHASE_CREEP;
		}
		break;
	}

	case TRIM_PHASE_CREEP:
	{
		float moved     = trim.travel_mm - trim.travel0_mm;
		float moved_abs = (moved < 0.0f) ? -moved : moved;
		float cmd_abs   = (trim.dcmd_mm < 0.0f) ? -trim.dcmd_mm : trim.dcmd_mm;
		float v_abs     = (trim.v_mps < 0.0f) ? -trim.v_mps : trim.v_mps;
		uint32_t timeout_ms = (uint32_t)(cmd_abs / v_abs) + 500u;   /* mm/(m/s)=ms; 堵转守卫 */
		uint8_t cut = 0u;      /* 0=未停 1=走满指令量 2=E窗背停 3=单步超时 4=总时长到 */
		uint8_t cut_reason = 0u;

		motion->cmd.vx_mps = trim.v_mps;
		motion->cmd.vy_mps = 0.0f;
		motion->cmd.wz_dps = 0.0f;
		/* 钉住执行器速度状态保持账实一致(收尾 HoldStill 归零, 放行从 0 起坡) */
		motion->path.cur_speed = trim.v_mps;

		e = tracking.horizontal_error_normalized * TRACKING_ERROR_FIXED_SCALE;
		abs_e = (e < 0.0f) ? -e : e;
		if (moved_abs >= cmd_abs)
		{
			cut = 1u; cut_reason = 0u;                   /* 走满指令量: 滑行自此开始 */
		}
		else if ((fresh != 0u) && (abs_e <= (float)TRACKING_TRIM_TOL_E))
		{
			cut = 1u; cut_reason = 1u;                   /* E 窗背停: 提前收令 */
		}
		else if ((trk_ms - trim.move_t0_ms) >= timeout_ms)
		{
			cut = 1u; cut_reason = 2u;                   /* 打滑/堵转: 别在一小步上吊死 */
		}
		if (timeout_ms < 2000u) { timeout_ms = 2000u; }   /* 下限: 低速步给轮速环爬静摩擦的时间 */
		if ((trk_ms - trim.t0_ms) >= TRACKING_TRIM_GIVEUP_MS)
		{
			cut = 1u; cut_reason = 3u;                   /* 总时长到: 停稳后 EVAL 必然 CAP 终态 */
		}
		if (cut != 0u)
		{
			trim.dcut_mm = moved_abs;
			tracking.settle_ms = 0u;
			tracking.settle_to_ms = 0u;
			trim.phase = TRIM_PHASE_SETTLE;
			/* 归因记录: 指令量 vs 实际轮程差 = 本步真实过冲, X=2(超时)说明轮子没跟上 */
			Trim_LogPush(TRKT_MOVED, e, (int16_t)(trim.dcmd_mm * 10.0f),
			             (int16_t)(moved_abs * 10.0f), cut_reason);
		}
		break;
	}

	case TRIM_PHASE_SETTLE:
	{
		if (Tracking_SettleCheck(motion) == 0u)
		{
			break;                                       /* 还在滑: 学习窗口没关 */
		}
		{
			float moved     = trim.travel_mm - trim.travel0_mm;
			float moved_abs = (moved < 0.0f) ? -moved : moved;
			float dcut      = (trim.dcut_mm < 0.0f) ? moved_abs : trim.dcut_mm;
			float e_prev    = (float)trim.estart_f10k;
			float de;

			e = tracking.horizontal_error_normalized * TRACKING_ERROR_FIXED_SCALE;
			/* 增益带符号: 朝 sign(E) 方向走而 E 反向显著增大 = 几何假设不成立, 别跟物理打架 */
			de = (trim.v_mps >= 0.0f) ? (e_prev - e) : (e - e_prev);
			if ((moved_abs >= 0.8f) && (de <= -8.0f))
			{
				Tracking_TrimEnd(motion, TRIM_OUTCOME_SIGN, e);
				break;
			}
			if ((moved_abs >= 0.8f) && (de >= 8.0f))
			{
				float g = de / moved_abs;

				trim.gain_cpmm = (g < 2.0f) ? 2.0f : ((g > 500.0f) ? 500.0f : g);
			}
			/* 学滞后必须轮程 >=0.8mm(与 gain 同门): 堵转步 over≈0 会把 lag 伪学到下限,
			 * 后续滑行补偿少算 -> 系统性偏短。超时切(reason 2/3)但轮子真动了仍照学 */
			if ((moved_abs >= 0.8f) && (moved_abs >= dcut))
			{
				float v_abs = (trim.v_mps < 0.0f) ? -trim.v_mps : trim.v_mps;
				float over  = moved_abs - dcut;
				float l;

				if (over >= 0.0f)
				{
					l = over / (v_abs * 1000.0f);
					trim.lag_s = (l < 0.02f) ? 0.02f : ((l > 0.30f) ? 0.30f : l);
				}
				if (trim.v_mps > 0.0f)
				{
					trim.budget_mm -= moved_abs;         /* 前向步计入预算(后退不返还, 保守) */
				}
			}
			trim.iter++;
			trim.eval_t0_ms = trk_ms;
			trim.phase = TRIM_PHASE_EVAL;
		}
		break;
	}

	default:   /* IDLE/FINISHED: 无事可做 */
		break;
	}
}

/* 原子消费误差帧: 有新帧更新误差并喂狗, 无新帧不碰 frame_age_ms */
void Tracking_ConsumeErrorFrame(void)
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
		tracking.error_fresh = 1u;
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
void Tracking_LimitApproachSpeed(void)
{
	mf_path_ctx_t *path = &mf_app()->path;

	if (tracking.state != TRACKING_ARMED)
	{
		return;   /* 只武装期限速 */
	}
	tracking.near_cap = 0u;
	if (Tracking_InSlowZone(mf_app()) == 0u)
	{
		return;   /* 还没进咬靶区: 段表全速 */
	}
	if (path->cur_speed > TRACKING_SLOW_MPS)
	{
		path->cur_speed = TRACKING_SLOW_MPS;   /* 只往下限: 倒车段不会被误抬速 */
	}
#if (TRACKING_NEAR_EN != 0u)
	/* 近靶二级限速: 按**帧龄**持有, 不按 found —— found=0 的窄窗闪断帧会让限速释放、
	 * 车速在点火前爬回高速, 实测停车消耗 C 在 60~241 计数双峰(2026-10-10 两趟),
	 * 固定点火阈无法两头兼顾; 闪断帧沿用最后 e 爬行, 盲爬上限=TIMEOUT×NEAR_MPS=7.5cm,
	 * 比闪断即加速回 0.45 安全。本函数用上一拍已消费的帧(nav_task 每拍先消费), 差一拍无害 */
	if ((tracking.braking == 0u) &&
	    (tracking.frame_age_ms <= TRACKING_FRAME_TIMEOUT_MS) &&
	    (mf_absf(tracking.horizontal_error_normalized) < TRACKING_ERR_SAT_LEVEL) &&
	    (mf_absf(tracking.horizontal_error_normalized) <=
	         ((float)TRACKING_NEAR_E_THR / TRACKING_ERROR_FIXED_SCALE)))
	{
		if (path->cur_speed > TRACKING_NEAR_MPS)
		{
			path->cur_speed = TRACKING_NEAR_MPS;   /* 只往下限, 与 SLOW_MPS 同契约 */
		}
		tracking.near_cap = 1u;
	}
#endif
}

/* TRACKING_STOP: 0=就地刹停(立即暂停本段) / 1=放行段表续跑完本段。
 * 1 是"开激光后恢复行进"的放行口: STOPPED 下段表只被压零、里程没走完, 回 OFF 后从
 * cur_speed=0 起坡把剩余里程补完, 终点仍落在名义终点(不变量 I3: 暂停可逆) */
void Tracking_Stop(uint8_t tracking_stop_mode)
{
	tracking.noconv_latch = 0u;   /* 清未收敛闩锁, 与 START 同口径 */
	if (tracking_stop_mode == 0u)
	{
		if (tracking.state != TRACKING_STOPPED)   /* 已在 STOPPED 就别重复广播 DONE */
		{
			Tracking_EnterStopped(mf_app());
		}
#if (TRACKING_TRIM_EN != 0u)
		else if (trim.phase != TRIM_PHASE_FINISHED)
		{
			/* 微调中途主控点名刹停: 放弃微调立即终态(同 SettleCheck 的即时响应先例),
			 * 残余照报 —— Brain 要的是"停", 不是"最优停" */
			Tracking_TrimEnd(mf_app(), TRIM_OUTCOME_ABORT,
			                 tracking.horizontal_error_normalized * TRACKING_ERROR_FIXED_SCALE);
		}
#endif
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
uint8_t Tracking_ApplyOverlay(void)
{
	mf_ctx_t *motion = mf_app();
	float e, abs_e;
	uint8_t fresh, done;

	trk_ms = trk_ms + CAR_TICK_MS;   /* 本模块时基: 必须在任何早返回之前推进 */
	if (tracking.state == TRACKING_OFF)
	{
		return 0u;                     /* 段表全权(不变量 I1) */
	}
	/* 误差帧消费提到状态分支之前: trim 在 STOPPED 态也读 E, 且新鲜判据用的是
	 * frame_seen_ms 时戳(STOPPED 无人递增 frame_age_ms, 断流时旧值恒"新鲜")。
	 * 与 nav_task 每拍的那次调用重复, 幂等(帧已消费, 这里 no-op) */
	Tracking_ConsumeErrorFrame();
	if (tracking.error_fresh != 0u)
	{
		tracking.error_fresh = 0u;
		trim.frame_seen_ms = trk_ms;
		if (tracking.state == TRACKING_ARMED)
		{
			if ((tracking.target_found_flag != 0u) &&
			    (mf_absf(tracking.horizontal_error_normalized) < TRACKING_ERR_SAT_LEVEL))
			{
				Trk_RateFeed(tracking.horizontal_error_normalized);
			}
			else
			{
				/* 两种"有帧但不可用": 丢检(found=0, e 被置 0) 与 量程饱和(found=1 但 e 钉在 ±0.96)。
				 * 前者让 |de/dt| 恒 0, 后者让重捕那一帧的跳变(实测 -9594 -> +9588)进窗 —— 都得清 */
				Trk_RateClear();
			}
		}
	}

	/* 已刹停: STOPPED 分支。终态(FINISHED)且闩锁未置 → 发 DONE: echo=|残余E|/40,
	 * reason=护栏标志(Brain 对 DONE 只置标志不读这两字节, 纯前向兼容的观测通道)。
	 * STOPPED 只报一次; START 重发会清 stop_done 闩锁 => 下拍补发一次(主控的 done
	 * 标志被那次 START 清掉了)。微调中途(phase 未到 FINISHED)不发 —— DONE 只在
	 * 收敛/护栏后出, trim 的总时长护栏(GIVEUP_MS)保证它总会到 */
	if (tracking.state == TRACKING_STOPPED)
	{
		Tracking_HoldStill(motion);
#if (TRACKING_TRIM_EN != 0u)
		Tracking_TrimTick(motion);            /* 可能覆写 cmd 为爬行速度 */
#endif
		if ((tracking.stop_done == 0u) && (trim.phase == TRIM_PHASE_FINISHED))
		{
			int32_t echo = trim.resid_f10k;

			if (echo < 0) { echo = -echo; }
			echo /= 40;                       /* 1 单位 ≈ 1px(1px=31.25 计数), 饱和 255 */
			if (echo > 255) { echo = 255; }
			tracking.stop_done = 1u;
			Car_Event_Send(CAR_EVT_TRACKING_DONE, (uint8_t)echo, trim.outcome);
		}
		return 1u;
	}
	/* 故障: 全零驻停, 等主控(START 重开 / 容错) */
	if (tracking.state == TRACKING_FAULT)
	{
		Tracking_HoldStill(motion);
		return 1u;
	}
	/* ==== ARMED: 段表全权驱动, 这里只评估刹停判据/上报故障, 不改 cmd ====
	 * (I1 的例外只有 STOPPED 的 trim 爬行: overlay 持有 cmd, 位移由段表账本按
	 *  实际轮程入账, 记账论证见 §10.18 —— "跟瞄一个字节不写"的旧说法已过期) */
	if (tracking.frame_age_ms < (uint16_t)(0xFFFFu - CAR_TICK_MS))
	{
		tracking.frame_age_ms = (uint16_t)(tracking.frame_age_ms + CAR_TICK_MS);
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
#if (TRACKING_SETTLE_EN != 0u)
			if (Tracking_SettleCheck(motion) == 0u)
			{
				tracking.settling = 1u;
				return 0u;   /* 目标已到零但车还在滑: 等物理停稳再报 DONE, 否则开火早于停点 */
			}
			tracking.settling = 0u;
#endif
#if (TRACKING_TRIM_EN != 0u)
			Tracking_TrimBegin(motion);     /* 自动刹停: 进微调评估, DONE 推迟到收敛/护栏 */
#else
			Tracking_EnterStopped(motion);  /* TRIM_EN=0: 回旧行为(停稳即发 DONE) */
#endif
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
			tracking.lost_ms = (uint16_t)(tracking.lost_ms + CAR_TICK_MS);
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

	/* ---- 过零兜底: 符号翻转那一拍 = 车刚从靶心旁经过 ----
	 * 正常时下面"进阈即刹"已先触发, 走不到这里; 留着是为了"整帧漏过阈值窗"
	 * (如某帧 |e| 恰好跳过去)时仍能刹住。去抖: 连续 TRACKING_ZERO_CONFIRM_N 帧同号才认 */
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

	/* ---- 进阈即刹(主判据): |e| 一进阈就刹, 不等符号翻转 ----
	 * 阈 = max(固定阈, 预测阈)。预测阈把停止距离 v·L + v²/(2a) 用当前斜率折算到 e 域,
	 * 只加不减 ⇒ 斜率不可用/没在接近时与固定阈逐拍等价(不会更晚刹)。
	 * 去抖按帧读(见 TRACKING_HOLD_TIME_MS 注释)。
	 * 只在咬靶区内计数 —— 否则段起点的误检(锁错目标 ⇒ e≈0)会把车从起步刹死 */
	{
		float approach = -Trk_RateSlope();   /* >0 = |e| 在缩小 */
		float thr = TRACKING_ERROR_THRESHOLD;
		uint8_t in_win;

#if (TRACKING_FIRE_PAST_EN != 0u)
		if (TRACKING_FIRE_PAST_E > 0u)
		{
			/* 定点开火: E ≤ +FIRE_PAST_E 就点火 —— E>0=靶心仍在车前方, 天然欠冲,
			 * 故停点 = 点火点 - C(C~110~115 计数, 近靶速下恒定), 微调因此永远只需
			 * “前进”一个方向。视觉侧已做弹着纠偏 ⇒ 目标就是 E→0,
			 * 本宏不再背安装偏置, 只需“够大”保证欠冲。
			 * 标定 1:1: 跑一趟读 TRK:A>S 行的 E_stop —— E_stop<0(过冲) ⇒ 宏 += |E_stop|+40,
			 *        E_stop>+80(欠冲) ⇒ 宏 -= (E_stop-30); 两趟收敛后永不重标。
			 * 符号从负过点反转的代价: 旧写法“丢检帧 e 保持旧值即不满足负过点”的抗丢检
			 * 特性随之消失(旧值已 ≤+150 时 hold_ms 会继续累计) —— 由 hold_ms>=40ms 与
			 * frame_age>500ms 清 hold 两道闸限制, 最多提前 150 计数点火 */
			thr = +(float)TRACKING_FIRE_PAST_E / TRACKING_ERROR_FIXED_SCALE;
			in_win = (uint8_t)((e <= thr) && (Tracking_InSlowZone(motion) != 0u));
			tracking.brake_thr = thr;   /* 观测: THR 行显示点火点(带符号, +150=靶心前方 150 计数) */
		}
		else
#endif
		{
			if ((TRACKING_PRED_EN != 0u) && (approach > TRACKING_ERR_MIN_SLOPE))
			{
				float lead;

				if (approach > TRACKING_ERR_MAX_SLOPE)
				{
					approach = TRACKING_ERR_MAX_SLOPE;
				}
				lead = ((float)TRACKING_LAG_MS * 0.001f)
				     + (mf_absf(motion->path.cur_speed) / (2.0f * TRACKING_BRAKE_MPS2));
				if ((approach * lead) > thr)
				{
					thr = approach * lead;
				}
			}
			tracking.brake_thr = thr;
			in_win = (uint8_t)((abs_e <= thr) && (Tracking_InSlowZone(motion) != 0u));
		}
		if (in_win)
		{
			tracking.hold_ms = (uint16_t)(tracking.hold_ms + CAR_TICK_MS);
			if (tracking.hold_ms >= TRACKING_HOLD_TIME_MS)
			{
				Tracking_BrakeStart(motion);
			}
		}
		else
		{
			tracking.hold_ms = 0u;
		}
	}

	/* ---- 未收敛 FAULT(01): 段表已完 + |e| 持续超阈; 闩锁防重复上报 ---- */
	if ((done != 0u) && (abs_e > TRACKING_ERROR_THRESHOLD))
	{
		tracking.noconv_ms = (uint16_t)(tracking.noconv_ms + CAR_TICK_MS);
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

/* 结束跟瞄(新段/急停/GYRO_ZERO 都隐含"结束跟瞄"): 回 OFF, 段表重获全权 */
void Tracking_Abort(void)
{
	Tracking_Goto(TRACKING_OFF);
}

/* STOPPED/FAULT 都是"有意停": 段表仍在 RUN 但车不动, 堵转看门狗必须豁免 */
uint8_t Tracking_IsHolding(void)
{
	return (uint8_t)((tracking.state == TRACKING_STOPPED) || (tracking.state == TRACKING_FAULT));
}

/* 读一次迁移打点并清 fresh(读后清): 返回 0 = 本次无迁移 */
uint8_t Tracking_TakeChangeLog(uint8_t *from, uint8_t *to)
{
	uint8_t fresh;

	__disable_irq();
	fresh = trk_chg_fresh;
	if (fresh != 0u)
	{
		*from = trk_chg_from;
		*to   = trk_chg_to;
		trk_chg_fresh = 0u;
	}
	__enable_irq();
	return fresh;
}

/* 观测快照: 关中断一次拷完(仅调试显示, 单字级撕裂容忍, 不进控制路径) */
void Tracking_GetInfo(Trk_Info_t *out)
{
	__disable_irq();
	out->state        = (uint8_t)tracking.state;
	out->found        = tracking.target_found_flag;
	out->braking      = tracking.braking;
	out->near         = tracking.near_cap;
	out->settling     = tracking.settling;
	out->reason       = tracking.fault_reason;
	out->zero_sign    = tracking.zero_sign;
	out->err_f10k     = (int16_t)(tracking.horizontal_error_normalized * TRACKING_ERROR_FIXED_SCALE);
	out->thr_f10k     = (int16_t)(tracking.brake_thr * TRACKING_ERROR_FIXED_SCALE);
	out->fire_e_f10k  = tracking.fire_e_f10k;
	out->frame_age_ms = tracking.frame_age_ms;
	out->hold_ms      = tracking.hold_ms;
	out->trim_phase   = trim.phase;
	out->trim_iter    = trim.iter;
	out->trim_gaveup  = trim.outcome;   /* 终态成因码(TRIM_OUTCOME_*), 0=收敛 */
	out->trim_resid_f10k = trim.resid_f10k;
	out->trim_gain_x10   = (int16_t)(trim.gain_cpmm * 10.0f);
	__enable_irq();
}

/* 按改写后 cmd 重解轮速环(轮速环积分每拍多累一次, 复调 Ki 注意) */
void Tracking_RecomputeWheels(void)
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
