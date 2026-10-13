/**
 * @file    Control_Car.c
 * @brief   底盘运行入口与总装层
 *
 * 它唯一拥有 TIM4 10ms 拍的调用顺序, 是底盘对外(协议层/自测层/main)唯一可见的接口面,
 * 并把 Motion 库、IMU、转台、NAV 装配成可运行系统。**除装配与拍序外不含业务逻辑。**
 *
 * 新函数该放哪 —— 三条判据, 全不满足就属于某个子模块:
 *   1. 它是拍序本身吗? -> 留(只有 Car_Control 是)2. 底盘外部要调吗? -> 原型进
 *      Control_Car.h 契约面, 实现放对应子模块 3. 跨 >=3 个子模块装配? -> 留
 *
 * 已知杂质 HWT101 的配置/归零已收敛到 GetData/hwt101.c(既非拍序也非装配, 属采样层);
 * 本文件只留供 App 调的转发壳 Car_GetYawDeg/Car_GetGyroWzDps, 原型见 App/car_link.h。
 */
#include "Control_Car.h"
#include "Car_Sense.h"
#include "hwt101.h"

/* 当前 HWT101 航向读数(deg, ±180): 跨层接缝给 App 层用(car_link.h)。
 * 转发到采样层驱动, 真实读数与帧计数都由 hwt101 持有 */
float Car_GetYawDeg(void)
{
	return Hwt101_GetYawDeg();
}

/* 当前 HWT101 Z 轴角速度(deg/s, 0x52 帧直给, 非差分)。用途: ① 观测口径 —— 与"航向差分"并排打, 判后者零阶保持采样伪影(§8.6/§8.11);
 * ② 备选速率反馈源(CARSENSE_WZ_SRC_GYRO_EN, car_config.h), 但**必须先台架确认符号**(官方没给方向, 反了即正反馈, 反号自检不足替代)。 */
float Car_GetGyroWzDps(void)
{
	return Hwt101_GetWzDps();
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
