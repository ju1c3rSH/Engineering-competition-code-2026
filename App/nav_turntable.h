/**
 * @file    nav_turntable.h
 * @brief   转台: 码表定点 / 目标去重 / TIM5 重发 / 急停 / 上电零点 / 台架直转
 *
 * 本模块不 include Control_Car.h(会形成头循环), 协议码统一取 App/car_protocol.h,
 * 跨层接缝在此声明并注明实现位置。
 * Car_data / tt / tt_arm / tt_sent_key / CW_OR_CCW 均为 nav_turntable.c 内部私有。
 */
#ifndef _NAV_TURNTABLE_H_
#define _NAV_TURNTABLE_H_

#include <stdint.h>
#include "car_protocol.h"
#include "car_link.h"

/* 转台观测快照(仅调试显示用) */
typedef struct { uint8_t busy, code, armed, cw;
                 uint16_t ms, settle_ms;
                 uint32_t last_tick;
                 float angle_deg; } Tt_Info_t;

void Turntable_Request(uint8_t turn_code);   /* 原 static Nav_Turntable: 码表定点 + 起 DONE 计时 */
void Turntable_Stop(void);                   /* 原急停序列: 冻结重发 + 停电机 + 作废未决 TURN_DONE */
/* 每拍**无条件**调用: 必须留在 Car_Nav_Task 的 "nav.state != NAV_RUN 即 return" 之前,
 * 否则非运行态的 TURN_DONE 不再上报 */
void Turntable_Tick(void);
void Turntable_GetInfo(Tt_Info_t *out);      /* 内部关中断一次拷完 */

void Emm_Pos_Control(void);                  /* TIM5 回调, 名字不变 */
void Car_Turntable_BootZero(void);           /* main.c 调, 名字不变 */
void Car_Turntable_BenchNudge(float delta_deg);   /* 名字不变 */
void Car_Turntable_BenchCode(uint8_t turn_code);  /* 名字不变 */
void Car_Turntable_BenchStop(void);               /* 名字不变 */

#endif /* _NAV_TURNTABLE_H_ */
