/**
 * @file    nav_task.h
 * @brief   NAV 调度层: 命令信箱 / 请求分发 / 重发幂等 / 链续 / 堵转看门狗 / 急停故障
 *
 * 唯一拍点入口 Car_Nav_Task() 由 Car_Control 每 10ms 调一次; 命令由 UART4 ISR 经
 * Car_Nav_MailPush 单生产者投递(协议: panzer_links_protocol.md 4.5)。
 * 段执行本身由 Motion 库 mf_tick 承担 —— 本层只做分发/幂等/事件上报, 不碰轮速。
 */
#ifndef _NAV_TASK_H_
#define _NAV_TASK_H_

#include <stdint.h>

/* 调度三态(观测: 打印取 "IRD"[state]) */
typedef enum
{
	NAV_IDLE = 0,   /* 上电/REQ_STOP/FAULT 后，任意 REQ_NAV 均从段 0 开始 */
	NAV_RUN,        /* 路径执行中（mf path RUN） */
	NAV_DONE,       /* 已完成，同 id 重发只补发 NAV_DONE */
} NavState_t;

void Car_Nav_Task(void);
void Car_Nav_MailPush(uint8_t cmd, uint8_t a0, uint8_t a1);

/* 观测用: 单字节读, 主循环无需关中断 */
uint8_t Nav_GetState(void);
uint8_t Nav_GetId(void);
uint8_t Nav_GetGear(void);   /* 本次起跑生效的档位: ≠0 会覆盖段表里的逐段速度 */

#endif /* _NAV_TASK_H_ */
