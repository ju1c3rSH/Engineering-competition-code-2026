/**
 * @file    car_units.h
 * @brief   底盘单位换算、车辆几何与节拍常量（panzer 专用口径）
 *
 * 口径(2026-09-29 更正): **1 单位 = 1 cm** —— 一次"驱动轮转"的行进距离 =
 * WHEEL_TRAVEL_PER_REV_M, 15 单位 = 1 轮转; 速度/加速度单位仍是"驱动轮转/秒"。
 * 旧式 MF_M_* 多乘了一个 π: 命令侧与反馈侧同乘、比值恒定 ⇒ 车一直走对, 只有读数
 * 放大 π 倍。段表/挡位/Car_Sense 必须共用这一份 —— 两处各写一遍必然漂移。
 * 量化依据与验收(RS 应 6691→2130): docs/坑-编码器与里程.md
 */
#ifndef _CAR_UNITS_H_
#define _CAR_UNITS_H_

/* 控制拍周期: TIM4 10ms。段表斜坡、跟瞄制动、堵转看门狗共用同一值 */
#define CAR_TICK_MS   10u

/* 旧速度挡位(驱动轮转/秒) */
#define Low_speed     1.5f
#define Middle_speed  2.0f
#define High_speed    3.0f

/* 一个编码器轮转(= 主动轮一转)对应的行进距离(m)。**不是轮子直径** —— 旧名 WHEEL_DIA_M
 * 误导了 π 的乘入。1 单位 = 本值/15 = 10mm。台架 5m 直线复标就是标这个值(自带 TODO):
 * CAD/场地/弧半径三条证据指向真实值 ≈0.133~0.138, 即读数仍会偏大 8~13% */
#define WHEEL_TRAVEL_PER_REV_M   0.150f

/* 路线里程约定: 旧 Car_Pos 1 单位 = 1/15 轮转 = 1cm */
#define MF_M_DIST(u)    (((float)(u) / 15.0f) * WHEEL_TRAVEL_PER_REV_M)
#define MF_M_SPEED(rps) ((float)(rps) * WHEEL_TRAVEL_PER_REV_M)
#define MF_M_ACC(rps2)  ((float)(rps2) * WHEEL_TRAVEL_PER_REV_M)

/* 原转弯机制: 90° 弧中心线走 π 轮转 ⇒ 弧长 π×轮转行进 ⇒ R = 2×轮转行进 = 300mm */
#define TURN_RADIUS_M (2.0f * WHEEL_TRAVEL_PER_REV_M)

/* 旧 speed_control() 的 Acc=45 轮转/s^2。speed_control 本体仍在 GetData/Encode.c,
 * 属待清死代码; 保留此宏作为下一轮清理时的换算锚点 */
#define TURN_ACC_MPS2 MF_M_ACC(45.0f)

#endif /* _CAR_UNITS_H_ */
