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
/* 原转弯机制: 90° 弧中心线走 π 轮转, 反解 R=2π×轮径(基线 Diameter=15 单位=1 轮转) */
#define TURN_RADIUS_M (2.0f * M_PI_F * WHEEL_DIA_M)
#define TURN_ACC_MPS2 MF_M_ACC(45.0f)      /* 旧 speed_control Acc=45 轮转/s2 */

/* ================== UART4 导航协议常量 (panzer_links_protocol.md 4.x) ================== */
#define FRAME_TYPE_REQUEST        0x20u   /* CMD_REQUEST */
#define FRAME_TYPE_ACK            0x21u   /* CMD_ACK */
#define FRAME_TYPE_NACK           0x22u   /* CMD_NACK */
#define FRAME_TYPE_STOP           0x23u   /* CMD_STOP(通用) */
#define FRAME_TYPE_DATA           0x24u   /* DATA_STREAM */

#define CAR_DATA_ID_CTRL        0x21u   /* CMD_CAR_CTRL: 运动请求(下行 sub_cmd 式) */
#define CAR_DATA_ID_EVT         0x22u   /* EVT_CAR_EVENT: 底盘事件上报(上行) */

#define CAR_SUB_REQ_NAVIGATION  0x01u   /* arg0=navigation_target_id 0~5, arg1=speed_gear 0~3 */
#define CAR_SUB_REQ_STOP          0x02u   /* 急停安全停车, 清执行状态 */
#define CAR_SUB_REQ_TURNTABLE     0x03u   /* arg0=转台码 0~7: 1~3对坑(123/90/57°) 4~6对桶(270/270/90°CW反向) 7回正90°, 0=不动作; arg1=保留恒0 */
#define CAR_SUB_GYRO_ZERO         0x04u   /* 陀螺仪置零(须车静止) */
/* 打靶段内跟瞄: 段表不断, 边走边修航向, 收敛刹停, 发射后续跑 */
#define CAR_SUB_TRACKING_START    0x05u   /* arg0=tracking_speed_level(0=段表速度/1=低速跟瞄), arg1=0保留 */
#define CAR_SUB_TRACKING_ERROR    0x06u   /* [error_low_byte,error_high_byte,target_found_flag], 定点=误差归一化*10000 小端 */
#define CAR_SUB_TRACKING_STOP     0x07u   /* arg0=tracking_stop_mode(0=就地刹停/1=继续走完当前段), arg1=0保留 */

#define CAR_EVT_NAV_DONE          0x02u   /* 段序列全部完成 */
#define CAR_EVT_NAV_FAULT         0x03u   /* 段执行异常 */
#define CAR_EVT_TURN_DONE         0x04u   /* 转台到位 */
#define CAR_EVT_TRACKING_DONE     0x05u   /* 段内跟瞄收敛已刹停, 可发射 */
#define CAR_EVT_TRACKING_FAULT    0x06u   /* 预留未实现: 当前丢帧/未收敛为悬停等待, 不上报 */

/* 跟瞄看门狗/收敛参数(可在台架复标) */
#define TRACKING_ERROR_THRESHOLD        0.05f   /* |归一化误差|小于此值视为瞄准 */
#define TRACKING_HOLD_TIME_MS           500u    /* 维持此时长判收敛 */
#define TRACKING_FRAME_TIMEOUT_MS       500u    /* 误差流超此时长无更新: 悬停等待 */
#define TRACKING_PROPORTIONAL_GAIN      60.0f   /* P 增益 dps/单位误差, 内部取负(e>0→顺时针); 镜像摄像头翻符号 */
#define TRACKING_MAX_YAW_RATE_DPS       60.0f   /* 跟瞄角速度限幅 */
#define TRACKING_STATIC_ERROR_MAX       0.30f   /* 段走完后的原地静瞄仅当|e|不超过此值(护住后续段表航向基准) */

/* 旧短名兼容别名: interurp.c 收发沿用, 值与全称一致, 新代码用全称 */
#define CAR_SUB_REQ_NAV         CAR_SUB_REQ_NAVIGATION

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

/* ---- 调度层: 段执行由 Motion 库 mf_tick 承担, 本层只做
 *      请求分发/重发幂等/事件上报, 跟瞄叠加见 Tracking_ApplyOverlay ---- */
void Car_Nav_Task(void);
void Car_Nav_MailPush(uint8_t cmd, uint8_t a0, uint8_t a1);
void Car_Tracking_ErrorPush(int16_t error_fixed, uint8_t target_found_flag);

mf_ctx_t *mf_app(void);
void mf_app_init(void);


extern Yaw_TypeDef Yaw_struct;
extern Parameter_TypeDef_Car Car_data;

void Car_Event_Send(uint8_t event, uint8_t echo, uint8_t reason);  /* interurp.c */



#endif
