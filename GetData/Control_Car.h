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

#define M_PI_F 3.141592653589793f

//定义小车速度（旧编码器速度单位：驱动轮转每秒）
#define	Low_speed     1.5f
#define	Middle_speed  2.0f
#define High_speed    3.0f

/* 驱动轮直径(m)
 * TODO: 台架 5m 直线复标定 */
#define WHEEL_DIA_M   0.150f
/* 原路线里程约定：旧 Car_Pos 1 单位 = 1/15 驱动轮转；速度/加速度单位为轮转每秒 */
#define MF_M_DIST(u)    (((float)(u) / 15.0f) * (M_PI_F * WHEEL_DIA_M))
#define MF_M_SPEED(rps) ((float)(rps) * (M_PI_F * WHEEL_DIA_M))
#define MF_M_ACC(rps2)  ((float)(rps2) * (M_PI_F * WHEEL_DIA_M))
/* 原转弯机制：90° 内中心线走过 Diameter×π 单位 = π 轮转的弧长，
 * 反解 R = 弧长/(π/2) = 2π×轮径（原 Diameter=15 单位即 1 轮转） */
#define TURN_RADIUS_M (2.0f * M_PI_F * WHEEL_DIA_M)
#define TURN_ACC_MPS2 MF_M_ACC(45.0f)      /* 旧 speed_control Acc=45 轮转/s2 */

/* ================== UART4 导航协议常量 (panzer_links_protocol.md 4.x) ================== */
#define CAR_DATA_ID_CTRL        0x21u   /* CMD_CAR_CTRL: 运动请求(下行 sub_cmd 式) */
#define CAR_DATA_ID_EVT         0x22u   /* EVT_CAR_EVENT: 底盘事件上报(上行) */

#define CAR_SUB_REQ_NAV         0x01u   /* REQ_NAV:  arg0=nav_id arg1=gear */
#define CAR_SUB_REQ_STOP        0x02u   /* REQ_STOP: 急停清状态 */
#define CAR_SUB_REQ_TURNTABLE   0x03u   /* REQ_TURNTABLE: arg0=pit 0~3 */
#define CAR_SUB_GYRO_ZERO       0x04u   /* GYRO_ZERO: 陀螺仪置零(须车静止) */

#define CAR_EVT_NAV_DONE        0x02u   /* 段序列全部完成 */
#define CAR_EVT_NAV_FAULT       0x03u   /* 段执行异常 */
#define CAR_EVT_TURN_DONE       0x04u   /* 转台到位 */

#define CAR_FAULT_WATCHDOG      0x04u   /* 进度看门狗: 里程 3s 无变化 */

#define CAR_NAV_ID_COUNT        6u      /* NavTarget_t 契约(与 Brain mission_sm.h 一致) */

typedef struct
{
	PID_TypeDef Yaw_pid;
	float Yaw;
	float Tar_Yaw;
	float Yaw_Gz;

}Yaw_TypeDef;

typedef struct
{
	float Motor_Angle;   /* 转台目标角(°)，TIM5 周期下发 Emm_V5 */

}Parameter_TypeDef_Car;


void Yaw_pid_Init(void);
void Car_Control(void);
void ResetHWT101(void);
void Emm_Pos_Control(float Angle);

/* ---- 段表导航调度层: 段执行由 Motion 库 mf_tick 承担，本层只做
 *      REQ_NAV/REQ_STOP/REQ_TURNTABLE/GYRO_ZERO 的调度、重发幂等与事件上报 ---- */
void Car_Nav_Task(void);       /* 消费cmd mailbox + 启停 + 看门狗 */
void Car_Nav_MailPush(uint8_t cmd, uint8_t a0, uint8_t a1);  /* TIM4 10ms 中断投递mailbox */

mf_ctx_t *mf_app(void);
void mf_app_init(void);


extern Yaw_TypeDef Yaw_struct;
extern Parameter_TypeDef_Car Car_data;

void Car_Event_Send(uint8_t event, uint8_t echo, uint8_t reason);  /* interurp.c */



#endif
