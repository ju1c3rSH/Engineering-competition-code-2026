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

/* 控制拍周期: TIM4 5ms。段表斜坡、跟瞄制动、堵转看门狗共用同一值 */
#define CAR_TICK_MS   5u

/* 旧速度挡位(驱动轮转/秒) */
#define Low_speed     1.5f
#define Middle_speed  2.0f
#define High_speed    3.0f

/* 驱动轮直径(m)。TODO: 台架 5m 直线复标定 */
#define WHEEL_DIA_M   0.150f

/* 原路线里程约定: 旧 Car_Pos 1 单位 = 1/15 驱动轮转 = π·Ø150/15 = **31.42 mm 轮上行程**。
 * ⚠ Ø150 的轮面周长恰好是 15π cm ⇒ 1 单位 = π cm ≈ 3.14 cm —— 它不是 1 cm, 差一个 π。
 * 段表按这条读: 217 单位 ≈ 6.8 m 轮上(不是 2.17 m); 真值另乘 真实轮径/0.150 ≈ 1.04 */
#define MF_M_DIST(u)    (((float)(u) / 15.0f) * (M_PI_F * WHEEL_DIA_M))
#define MF_M_SPEED(rps) ((float)(rps) * (M_PI_F * WHEEL_DIA_M))
#define MF_M_ACC(rps2)  ((float)(rps2) * (M_PI_F * WHEEL_DIA_M))

/* 原转弯机制: 90° 弧中心线走 π 轮转, 反解 R=2π×轮径(基线 Diameter=15 单位=1 轮转) */
#define TURN_RADIUS_M (2.0f * M_PI_F * WHEEL_DIA_M)

#endif /* _CAR_UNITS_H_ */
