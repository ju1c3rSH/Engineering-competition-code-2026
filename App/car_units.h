/**
 * @file    car_units.h
 * @brief   底盘单位换算、车辆几何与节拍常量（panzer 专用口径）
 *
 * 路线里程沿用旧口径: 1 单位 = 1/15 驱动轮转, 速度/加速度单位为"驱动轮转/秒"。
 * MF_M_* 负责换算到框架口径(m / m/s / m/s^2)。
 * 段表、挡位与 Car_Sense 必须共用这一份 —— 两处各写一遍必然漂移。
 */
#ifndef _CAR_UNITS_H_
#define _CAR_UNITS_H_

#define M_PI_F 3.141592653589793f

/* 控制拍周期: TIM4 10ms。段表斜坡、跟瞄制动、堵转看门狗共用同一值 */
#define CAR_TICK_MS   10u
#define CAR_TICK_S    0.01f

/* 旧速度挡位(驱动轮转/秒) */
#define Low_speed     1.5f
#define Middle_speed  2.0f
#define High_speed    3.0f

/* 驱动轮直径(m)。TODO: 台架 5m 直线复标定 */
#define WHEEL_DIA_M   0.150f

/* 原路线里程约定: 旧 Car_Pos 1 单位 = 1/15 驱动轮转 */
#define MF_M_DIST(u)    (((float)(u) / 15.0f) * (M_PI_F * WHEEL_DIA_M))
#define MF_M_SPEED(rps) ((float)(rps) * (M_PI_F * WHEEL_DIA_M))
#define MF_M_ACC(rps2)  ((float)(rps2) * (M_PI_F * WHEEL_DIA_M))

/* 原转弯机制: 90° 弧中心线走 π 轮转, 反解 R=2π×轮径(基线 Diameter=15 单位=1 轮转) */
#define TURN_RADIUS_M (2.0f * M_PI_F * WHEEL_DIA_M)

/* 旧 speed_control() 的 Acc=45 轮转/s^2。speed_control 本体仍在 GetData/Encode.c,
 * 属待清死代码; 保留此宏作为下一轮清理时的换算锚点 */
#define TURN_ACC_MPS2 MF_M_ACC(45.0f)

#endif /* _CAR_UNITS_H_ */
