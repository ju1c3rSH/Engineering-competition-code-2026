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
#include "car_obs.h"
#include "car_link.h"

/* 协议码表(FRAME_TYPE_* / CAR_SUB_* / CAR_EVT_* / CAR_FAULT_* / TRK_AXIS_* /
 * TRK_FAULT_*)已下沉到 App/car_protocol.h: 那是零依赖契约头, 否则 nav_* 子模块
 * 要用码表就得反向 include 本文件(成环)。经上面 include 仍对本文件及外部可见。

 * 固件形态(自测总闸 / 转台使能 / 调试口默认值 / 段表缓冲尺寸)见 GetData/car_config.h,
 * 自测覆盖见 Selftest/selftest.h —— 本文件不再持有任何测试宏;
 * CAR_NAV_ID_COUNT 契约已迁到 App/nav_routes.h(经上面 include 仍对本文件可见) */

void Car_Control(void);

/* HWT101 全部迁到独立驱动库 GetData/hwt101.h(采样层): 帧解析(0x52/0x53/0x5F)、
 * 0x52/0x53 交叉校验、寄存器读写、上电配置(测帧率/RRATE)、观测字段(帧率/帧计数/
 * 坏帧/版本号)。原先本文件里的 ConfigHWT101/ResetHWT101/Hwt101_RateTask/
 * Hwt101_RateHz/Hwt101_RateBoot 与 Yaw_struct 全局量已删除。
 * 外部需要帧计数或诊断字段时经 hwt101.h 的 Hwt101_Health() 取, 不再 extern 变量。 */

/* 转台(Emm_Pos_Control / Car_Turntable_BootZero / 台架三函数 / 急停 / 码表)已迁到
 * App/nav_turntable.h(经上面 include 仍对本文件可见) */

/* 调度层(Car_Nav_Task / Car_Nav_MailPush)已迁到 App/nav_task.h(经上面 include 仍可见) */

/* 观测层(Car_Debug_Stat_Print / Car_SegChange_Log / Car_SegExit_Log / Car_YawZero_Log /
 * Car_TrkChange_Log / Car_TrkDetail_Log)已迁到 App/car_obs.h —— 那是全工程唯一的
 * printf 出口, 经上面 include 仍对本文件及外部可见 */

/* 跨层接缝(Car_Event_Send / Car_GetYawDeg / Car_GetGyroWzDps)单点声明在 App/car_link.h
 * —— App 子模块不能反向 include 本头(成环), 故接缝声明在那儿 */


#endif

