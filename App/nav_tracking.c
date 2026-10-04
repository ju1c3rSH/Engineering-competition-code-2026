#include "nav_tracking.h"
#include "car_units.h"
#include "stm32f4xx.h"      /* __disable_irq/__enable_irq(CMSIS core) */
#include "motion_app.h"
#include "mf_path.h"
#include "mf_math.h"
#include "mf_kinematics.h"
#include "car_link.h"

static struct
{
	TrackingState_t state;
	float    horizontal_error_normalized;  /* 最新归一化横向误差, 约 -1~+1 */
	uint8_t  target_found_flag;            /* 最新有效标志 */
	uint16_t frame_age_ms;                 /* 距上次有效误差帧时长 */
	uint16_t hold_ms;                      /* 进阈即刹: |e|≤阈 的连续累计 */
	int8_t   zero_sign;                    /* 已确认误差符号(+1/-1/0=未确认), 过零判据的基准 */
	int8_t   sign_cand;                    /* 去抖候选符号 */
	uint8_t  confirm_cnt;                  /* 去抖: 候选符号连续同号帧数 */
	uint8_t  braking;                      /* 已触发制动的锁存: 不再重判, 只走制动斜坡 */
	float    brake_cap;                    /* 制动斜坡: 当拍允许的段表目标速度上限 */
	uint16_t noconv_ms;                    /* 段表已完 + |e|>阈 的连续累计(FAULT 01) */
	uint8_t  noconv_latch;                 /* FAULT(01) 闩锁: 防重复上报, START/STOP 清 */
	uint16_t lost_ms;                      /* 段表已完 + 无测量的连续累计(FAULT 02) */
	uint8_t  stop_done;                    /* STOPPED 已报 TRACKING_DONE 的闩锁(START 清, 补发用) */
	uint8_t  fault_reason;                 /* TRACKING_FAULT 的 reason, 见本头 TRK_FAULT_* */
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
void Tracking_Stop(uint8_t tracking_stop_mode)
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
uint8_t Tracking_ApplyOverlay(void)
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

	/* ---- 进阈即刹(主判据): |e| 一进 ±阈 就刹, 不等符号翻转 ----
	 * 提前量 = 停止距离(v·链路滞后 + v²/(2·BRAKE))折算到 e 域; 用 |e| 直接比, 不引入
	 * 第二个几何标定。去抖按帧读(见 TRACKING_HOLD_TIME_MS 注释)。
	 * 只在咬靶区内计数 —— 否则段起点的误检(锁错目标 ⇒ e≈0)会把车从起步刹死 */
	if ((abs_e <= TRACKING_ERROR_THRESHOLD) && (Tracking_InSlowZone(motion) != 0u))
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
	out->reason       = tracking.fault_reason;
	out->zero_sign    = tracking.zero_sign;
	out->err_f10k     = (int16_t)(tracking.horizontal_error_normalized * TRACKING_ERROR_FIXED_SCALE);
	out->frame_age_ms = tracking.frame_age_ms;
	out->hold_ms      = tracking.hold_ms;
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
