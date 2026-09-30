/**
 * @file    Control_Car.c
 * @brief   底盘运行入口与总装层
 *
 * 它唯一拥有 TIM4 10ms 拍的调用顺序; 它是底盘对外(协议层 / 自测层 / main)唯一可见的
 * 接口面; 它把 Motion 库、IMU、转台、NAV 各子模块装配成一个可运行系统。
 * **除装配与拍序之外, 本文件不包含任何业务逻辑。**
 *
 * 新函数该放哪 —— 三条判据, 全不满足就属于某个子模块:
 *   1. 它是拍序本身吗?          -> 留(只有 Car_Control 是)
 *   2. 它是底盘外部要调的吗?     -> 原型进 Control_Car.h 契约面, 实现放对应子模块
 *   3. 它在跨 >=3 个子模块装配?  -> 留
 *
 * 已知杂质: HWT101 三函数(Yaw_pid_Init/ResetHWT101/ConfigHWT101)既非拍序也非装配,
 * 属采样层, 应归 Car_Sense.c; 因 interurp.c 直接写 Yaw_struct.Yaw, 搬动会牵动协议层,
 * 留给下一轮单独做。
 */
#include "Control_Car.h"
#include "Car_Sense.h"

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

/* 10ms 控制拍(TIM4 ISR)的唯一编排点: 下面这个调用顺序**本身就是语义** ——
 * 哪一步必须在 mf_tick 之前/之后、哪一步必须无条件跑, 全系统只在此处定义。
 * 改顺序前先看 App/nav_tracking.h 上 LimitApproachSpeed/ApplyOverlay 的两条契约 */
void Car_Control()
{
	extern void Uart_RxRearm_Task(void);   /* stm32f4xx_it.c: RX DMA 重装失败的重试点 */
	CarSense_Sample();                   /* 采样层: 编码器+航向 -> 快照(seq+1) */
	Uart_RxRearm_Task();                 /* 上拍 IDLE/Error 路径 HAL_BUSY 未装回的 RX DMA, 此处堵死口 */
	Car_Nav_Task();                      /* 调度层: 信箱/启停/事件/看门狗 */
	Car_TxQueue_Pump();                  /* 上行发送队列: ACK/EVT 单点泵出 */
	Tracking_LimitApproachSpeed();       /* 咬靶限速: 必须早于 mf_tick(轮速环只算一次) */
	mf_tick(mf_app(), CarSense_Get());   /* 运动库: 纯消费快照, 无新样自动跳过 */
	{
		int16_t vxp_mm = (int16_t)(mf_app()->cmd.vx_mps * 1000.0f);
		uint8_t ovl = Tracking_ApplyOverlay();

		if (ovl != 0u)                   /* 段内跟瞄改写了 cmd */
		{
			Tracking_RecomputeWheels();
		}
		Car_Obs_SetVx(vxp_mm, (int16_t)(mf_app()->cmd.vx_mps * 1000.0f), ovl);
	}
	CarSense_Output(mf_app());           /* 执行层: wheel[].out -> DIR+PWM */
	IWDG->KR = 0xAAAAU;                  /* 喂独立看门狗(198ms): 本函数停摆即复位清 PWM */
}

/* HWT101 配置/命令帧发送: 上一帧 DMA 在途时发会 HAL_BUSY 静默失败 —— 等就绪再发, 带上限
 * 免上电阶段死等。**内部含 HAL_Delay, 只许上电初始化(main.c)调用**, 中断内调会堵控制拍 */
static void Hwt101_CfgSend(uint8_t *cmd)
{
	uint16_t wait_ms = 0u;

	while ((huart3.gState != HAL_UART_STATE_READY) && (wait_ms < 50u))
	{
		HAL_Delay(1u);
		wait_ms++;
	}
	(void)HAL_UART_Transmit_DMA(&huart3, cmd, 5u);
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

/* HWT101 上电配置: 输出速率由出厂默认 10Hz 提到 200Hz(RRATE 寄存器, 写模块内部 Flash)。
 * 速率是速率内环能不能用差分的**前提**(见 docs/坑-编码器与里程.md §7.6)。配置命令单向
 * 无 ACK, 所以用"单位时间收到的 0x53 帧数"验; 已达标就不写模块 Flash(它有擦写寿命) */
#define HWT101_CFG_RRATE_EN     1      /* 置 0 = 完全不碰模块配置 */
#define HWT101_RATE_TARGET_HZ   50u    /* 判定阈值: 远高于出厂 10Hz 即认为配置已生效 */
#define HWT101_RATE_WIN_MS      500u   /* 帧率统计窗口(ms) */

/* 0x53 帧率(Hz)两个口径, 必须分开看 —— 只打一个会误判(量化见 docs/坑-编码器与里程.md §7.6):
 * Hwt101_RateHz   = 最近一个窗口的滚动值(主循环刷新) -> 判"配置生效没有"看它;
 * Hwt101_RateBoot = 上电那一次采样 -> "要不要写模块 Flash"的判据, 留作事后审计。
 * 上电采样必须在模块出流之后做: 采早了把启动期低读成速率, 会白写一遍模块 Flash */
uint16_t Hwt101_RateHz   = 0u;
uint16_t Hwt101_RateBoot = 0u;

#if HWT101_CFG_RRATE_EN
static uint16_t Hwt101_MeasureYawHz(void)
{
	uint32_t f0 = YawSeq;
	uint16_t t_ms = 0u;

	while (t_ms < HWT101_RATE_WIN_MS)
	{
		HAL_Delay(10u);
		t_ms += 10u;
	}
	return (uint16_t)(((YawSeq - f0) * 1000u) / HWT101_RATE_WIN_MS);
}
#endif

/* 帧率滚动统计: 主循环(Car_DebugTask)每圈调用, 每满一个窗口用帧计数增量算一次。
 * 与上电那次一次性采样互补 —— 它反映"现在", 任何时候查都作数 */
void Hwt101_RateTask(void)
{
	static uint32_t prev_seq = 0u;
	static uint32_t prev_ms  = 0u;
	uint32_t now_ms  = HAL_GetTick();
	uint32_t now_seq = YawSeq;
	uint32_t span    = now_ms - prev_ms;

	if (span >= HWT101_RATE_WIN_MS)
	{
		Hwt101_RateHz = (uint16_t)(((now_seq - prev_seq) * 1000u) / span);
		prev_seq = now_seq;
		prev_ms  = now_ms;
	}
}

/**
  * @brief 把 HWT101 输出速率配到 200Hz(出厂 10Hz 对速率内环不够用), 并记下实测帧率
  * @note  寄存器取自维特官方协议: RRATE=0x03(值 0x0B=200Hz), KEY=0x69(写 0xB588 解锁)。
  *        旧码写 {FF AA 69 05 00}: 打的是 KEY 寄存器(只认 0xB588, 其他值无效)且 0x05 是
  *        5Hz ⇒ 被模块静默丢弃, 速率一直停在出厂 10Hz。见 docs/坑-编码器与里程.md §7.6
  *        **调用时机有约束**: 必须等模块出流(main.c 在配置前的那个 delay), 否则首测低读
  *        被判成"没达标", 每次上电都白写一遍模块 Flash
  */
void ConfigHWT101(void)
{
#if HWT101_CFG_RRATE_EN
	static uint8_t unlock_cmd[]    = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
	static uint8_t set_200hz_cmd[] = {0xFF, 0xAA, 0x03, 0x0B, 0x00};
	static uint8_t save_cmd[]      = {0xFF, 0xAA, 0x00, 0x00, 0x00};

	Hwt101_RateBoot = Hwt101_MeasureYawHz();   /* 先看现状: 已达标就别再写模块 Flash */
	Hwt101_RateHz   = Hwt101_RateBoot;
	if (Hwt101_RateBoot >= HWT101_RATE_TARGET_HZ)
	{
		return;
	}

	Hwt101_CfgSend(unlock_cmd);
	HAL_Delay(100u);
	Hwt101_CfgSend(set_200hz_cmd);
	HAL_Delay(100u);
	Hwt101_CfgSend(save_cmd);
	HAL_Delay(500u);

	Hwt101_RateBoot = Hwt101_MeasureYawHz();   /* 复验: 仍不达标会经 HWT: 行暴露出来 */
	Hwt101_RateHz   = Hwt101_RateBoot;
#endif
}