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
/* HWT101 航向置零: 实现 GetData/Control_Car.c(属采样层, 待迁 Car_Sense.c) */
void ResetHWT101(void);

#endif /* _CAR_LINK_H_ */
