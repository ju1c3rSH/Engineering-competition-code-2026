#include "nav_turntable.h"
#include "car_units.h"
#include "car_config.h"
#include "Emm_V5.h"

extern void Car_Event_Send(uint8_t event, uint8_t echo, uint8_t reason);   /* 实现: GetData/interurp.c */

/* 转台记账角: 原 Control_Car.c 的全局, 外部已无引用, 收为本模块私有 */
typedef struct
{
	float Motor_Angle;   /* 转台目标角(°)，TIM5 周期下发 Emm_V5 */

} Parameter_TypeDef_Car;
static Parameter_TypeDef_Car Car_data;

/* 张大头驱动器命令地址
   配置错误会导致驱动起执行静默失效*/
#define EMM_MOTOR_ADDR   2u

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

/* 转台 DONE 计时: 开环静置, 按本次行程 |本次目标−上次目标| × TURNTABLE_MS_PER_DEG 算, 只增不减。
 * TURNTABLE_MS_PER_DEG 标定: 上机敲桶位码 4/5(270°), 录像量"发令→实际停稳" t, 置 t/270 */
#define TURNTABLE_MS_PER_DEG   1.9f    /* 额定 180RPM ⇒ 0.93ms/°; 旧值 ×2 计"100ms 重发重启
                                        * 梯形规划"欠速——2026-09 目标去重后单发不再重启, 1.9 纯作
                                        * 偏长裕量(报早=伸臂干涉), 可按下行标定法回收时间 */
#define TURNTABLE_RAMP_MARGIN  300u    /* 加减速 + 机械/传动裕量 */
#define TURNTABLE_SETTLE_MIN   1000u   /* 只增不减: 旧固定值, 相对旧行为零回退 */
#define TURNTABLE_SETTLE_MAX   3000u   /* 上限: 旧基线值, 防异常行程把流程拖死 */

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

void Turntable_Request(uint8_t turn_code)
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

/* 急停: 冻结 TIM5 重发 + 作废未决 TURN_DONE + 停电机 + 登记(原 Nav_DoStop 的转台段 /
 * 原 Car_Turntable_BenchStop 共用此序列, 逐字保留)。
 * 弃用在途位置帧(DMA 中途可被共享 rxb 的 memset 撕裂, 且 Emm_V5 内部
 * HAL_UART_Transmit_DMA 遇 BUSY 会静默吞掉停止帧), 改用局部缓冲阻塞直发
 * (5B@115200≈0.43ms, 仅 STOP 时一次)。残窗: TIM5 恰在"tt_arm 检查后、发送前"
 * 被抢占时, 恢复后会多发一帧旧位置——之后无重发, 电机走完该目标即停, 与旧行为相当 */
void Turntable_Stop(void)
{
	uint8_t stop_cmd[5] = {EMM_MOTOR_ADDR, 0xFEu, 0x98u, 0x00u, 0x6Bu};
	uint8_t try_n;

	tt_arm = 0u;
	tt.busy = 0u;
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
	Car_Dbg2_TtRecord(0u, tt.code, Car_data.Motor_Angle);
}

/* 转台 DONE 计时(每拍一次): tt.busy 置起后按行程计时, 到点报 TURN_DONE。
 * 必须留在 Car_Nav_Task 的 "nav.state != NAV_RUN 即 return" **之前** —— 非运行态照样每拍跑 */
void Turntable_Tick(void)
{
	if (tt.busy != 0u)
	{
		tt.ms = (uint16_t)(tt.ms + CAR_TICK_MS);
		if (tt.ms >= tt.settle_ms)
		{
			tt.busy = 0u;
			Car_Event_Send(CAR_EVT_TURN_DONE, tt.code, 0x00u);
		}
	}
}

/* 观测快照: 关中断一次拷完(仅调试显示, 单字级撕裂容忍, 不进控制路径) */
void Turntable_GetInfo(Tt_Info_t *out)
{
	__disable_irq();
	out->busy      = tt.busy;
	out->code      = tt.code;
	out->armed     = tt_arm;
	out->cw        = CW_OR_CCW;
	out->ms        = tt.ms;
	out->settle_ms = tt.settle_ms;
	out->last_tick = tt_last_tick;
	out->angle_deg = Car_data.Motor_Angle;
	__enable_irq();
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
	Turntable_Request(turn_code);   /* 走协议同款码表, 含 TT 事件登记 */
	tt.busy = 0u;                   /* 台架不播出 TURN_DONE: 挂着链路时不误推进 Brain */
}

void Car_Turntable_BenchStop(void)
{
	Turntable_Stop();
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
