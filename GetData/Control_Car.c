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