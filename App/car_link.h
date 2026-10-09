/**
 * @file    car_link.h
 * @brief   跨层接缝: 实现在 GetData/(协议层 / 底盘入口), 被 App/ 各模块调用
 *
 * App 模块不得 include GetData/Control_Car.h —— 它聚合了全部子模块头, 反向引用即成环。
 * 于是"实现在别处"的这几个函数在此集中声明一次; 否则同一原型要在每个用到的 .c 里
 * 各 extern 一遍, 改签名时必漏。
 */
#ifndef _CAR_LINK_H_
#define _CAR_LINK_H_

#include <stdint.h>

/* 上行事件帧入队(主循环 Car_TxQueue_Pump 泵出): 实现 GetData/interurp.c */
void Car_Event_Send(uint8_t event, uint8_t echo, uint8_t reason);
/* 转台动作登记到调试口回显区: 实现 GetData/interurp.c */
void Car_Dbg2_TtRecord(uint8_t kind, uint8_t code, float angle_deg);
/* 作废上电静置期攒下的台架转台命令(bench_fresh): 实现 GetData/interurp.c */
void Car_Dbg2_BenchDrop(void);
/* Nav_Fault 观测登记(调试口 NAVFLT: 行): 实现 GetData/interurp.c */
void Car_Dbg2_NavFltRecord(uint8_t nav_id);
/* TIM4 回调最大耗时(DWT 原始周期数; 观测口读后清零): 实现 GetData/interurp.c, car_obs.c CPU: 行消费 */
extern volatile uint32_t Car_Isr_Cyc_Max;

/* ---- HWT101(采样层驱动 GetData/hwt101.c)----
 * 这一层只留"壳函数"而非让 App 直接调驱动, 使依赖方向单一(换驱动/协议不动 App 调用点)。
 * HWT101_Cfg/Hwt101_ZeroYaw 等配置函数**不上这里**: 它们阻塞(HAL_Delay), 只许 main() 上电阶段调用。
 * 诊断字段走 hwt101.h 的 Hwt101_Health(): 该头是叶子头(只含 stdint.h), 观测层可直接 include 取类型, 不构环。 */
float Car_GetYawDeg(void);      /* 航向角(deg, ±180), GYRO_ZERO 钉基准用 */
float Car_GetGyroWzDps(void);   /* 0x52 直给角速度(deg/s), 调试口 TRC 的 GZ= 列 */

#endif /* _CAR_LINK_H_ */
