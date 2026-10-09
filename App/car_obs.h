/**
 * @file    car_obs.h
 * @brief   底盘观测层: USART2 调试口的全部 printf 出口
 *
 * 全工程唯一的 printf 出口在 car_obs.c。仓库硬规则(AGENTS.md 中断分层): ISR 内禁止
 * printf/阻塞发送 —— 本模块每个函数都只能在 main 上下文(Car_DebugTask)调用。
 *
 * 数据单向: 控制拍经 Car_Obs_SetVx() 把观测值推入本层, 本层只读不写控制状态;
 * 其余状态经各子模块的 *_GetInfo() 快照拿, 不直接碰它们的 static 结构。
 */
#ifndef _CAR_OBS_H_
#define _CAR_OBS_H_

#include <stdint.h>

void Car_Debug_Stat_Print(void);   /* '?'/p 周期: 状态块(CHASSIS/TRK2/WHEEL/POSE/HDG/WTBL/EMM/LINK/HWT) */
void Car_SegChange_Log(void);      /* 主循环: seg_index/state 变化时打一行 SEG(原库层 ISR printf) */
void Car_SegExit_Log(void);        /* 主循环: 段跑完打一行 —— ARC 出 EY/YE, 直线出 LN:IE/YE/EY */
void Car_YawZero_Log(void);        /* 主循环: GYRO_ZERO 那一刻打一行 YAWZERO(本表重锚基点) */
void Car_TrkChange_Log(void);      /* 主循环: 跟瞄状态迁移(四态机)打一行 TRK:旧>新 */
void Car_TrkDetail_Log(void);      /* 主循环: ARMED/STOPPED 期 10Hz 打点 TRKD(判误差卡哪) */
void Car_Obs_TraceArm(void);       /* 't': 武装段内瞬时跟踪(命中弧/转向段自动开窗) */
void Car_Obs_TraceTask(void);      /* 主循环每圈调: 武装态/采集态才有输出, 其余立即返回 */
void Car_ImuProbe_Arm(void);       /* 'i': 启动 3s 陀螺仪观测窗口(轮读零偏/校准开关/峰峰/模式) */
void Car_ImuProbe_Task(void);      /* 主循环每圈调: 窗口期内 5Hz 打一行 IMU */
void Car_ImuBootCalib_Log(void);   /* main 上电: Hwt101_BootCalib() 之后打一行 BIA: 快照校准结果 */

/* 控制拍 -> 观测层单向发布(TIM4 ISR 写, main 上下文读) */
void Car_Obs_SetVx(int16_t path_mm, int16_t final_mm, uint8_t overlay);

#endif /* _CAR_OBS_H_ */
