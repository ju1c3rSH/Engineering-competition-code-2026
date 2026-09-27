#ifndef _CONTROL_CAR_H_
#define _CONTROL_CAR_H_

//welcome to my github
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "PID.h"
#include "math.h"
#include "usart.h"
#include "stdio.h"
#include "Encode.h"
#include "Emm_V5.h"
#include "interurp.h"
#include "mf_motion.h"
#include "car_units.h"
#include "car_protocol.h"
#include "motion_app.h"
#include "nav_routes.h"
#include "nav_turntable.h"
#include "nav_tracking.h"
#include "nav_task.h"

/* 协议码表(FRAME_TYPE_* / CAR_SUB_* / CAR_EVT_* / CAR_FAULT_* / TRK_AXIS_* /
 * TRK_FAULT_*)已下沉到 App/car_protocol.h: 那是零依赖契约头, 否则 nav_* 子模块
 * 要用码表就得反向 include 本文件(成环)。经上面 include 仍对本文件及外部可见。

 * 固件形态(自测总闸 / 转台使能 / 调试口默认值 / 段表缓冲尺寸)见 GetData/car_config.h,
 * 自测覆盖见 Selftest/selftest.h —— 本文件不再持有任何测试宏;
 * CAR_NAV_ID_COUNT 契约已迁到 App/nav_routes.h(经上面 include 仍对本文件可见) */

typedef struct
{
	PID_TypeDef Yaw_pid;
	float Yaw;
	float Tar_Yaw;
	float Yaw_Gz;

}Yaw_TypeDef;

void Yaw_pid_Init(void);
void Car_Control(void);
void ResetHWT101(void);
/* 转台(Emm_Pos_Control / Car_Turntable_BootZero / 台架三函数 / 急停 / 码表)已迁到
 * App/nav_turntable.h(经上面 include 仍对本文件可见) */

/* 调度层(Car_Nav_Task / Car_Nav_MailPush)已迁到 App/nav_task.h(经上面 include 仍可见) */

extern Yaw_TypeDef Yaw_struct;

void Car_Event_Send(uint8_t event, uint8_t echo, uint8_t reason);  /* interurp.c */

void Car_Debug_Stat_Print(void);   /*'. '?'/p 周期: 四行状态块(CHASSIS/WHEEL/POSE/WTBL) */
void Car_SegChange_Log(void);      /* 主循环: seg_index/state 变化时打一行 SEG(原库层 ISR printf) */
void Car_ArcExit_Log(void);        /* 主循环: 每条 ARC 段跑完打一行 ARC(出口横向/航向偏差) */
void Car_TrkChange_Log(void);      /* 主循环: 跟瞄状态迁移(四态机)打一行 TRK:旧>新 */
void Car_TrkDetail_Log(void);      /* 主循环: ARMED/STOPPED 期 10Hz 打点 TRKD(判误差卡哪) */

void ConfigHWT101(void);


#endif
