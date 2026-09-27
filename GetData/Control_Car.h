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
#include "motion_app.h"
#include "nav_routes.h"
#include "nav_turntable.h"
#include "nav_tracking.h"

/* ================== UART4 导航协议常量 (panzer_links_protocol.md 4.x) ================== */
#define FRAME_TYPE_REQUEST        0x20u   /* CMD_REQUEST */
#define FRAME_TYPE_ACK            0x21u   /* CMD_ACK */
#define FRAME_TYPE_NACK           0x22u   /* CMD_NACK */
#define FRAME_TYPE_STOP           0x23u   /* CMD_STOP(通用) */
#define FRAME_TYPE_DATA           0x24u   /* DATA_STREAM */

/* NACK 原因码枚举 (panzer_links_protocol.md v2.2 §1.4, 三链路统一) */
#define CAR_NACK_FORMAT       0x01u   /* 长度不足/非REQUEST/data_id 不支持 */
#define CAR_NACK_BUSY         0x02u   /* 忙/未就绪(预留) */
#define CAR_NACK_PARAM        0x03u   /* 子命令存在但参数越界 */
#define CAR_NACK_UNKNOWNSUB   0x04u   /* data_id 合法但 sub_cmd 不在定义表 */

#define CAR_DATA_ID_CTRL        0x21u   /* CMD_CAR_CTRL: 运动请求(下行 sub_cmd 式) */
#define CAR_DATA_ID_EVT         0x22u   /* EVT_CAR_EVENT: 底盘事件上报(上行) */

#define CAR_SUB_REQ_NAVIGATION  0x01u   /* arg0=navigation_target_id 0~8(6/7/8=重锚伪路线), arg1=speed_gear 0~3 */
#define CAR_SUB_REQ_STOP          0x02u   /* 急停安全停车, 清执行状态 */
#define CAR_SUB_REQ_TURNTABLE     0x03u   /* arg0=转台码 0~7: 1~3对坑(123/90/57°) 4~6对桶(270/270/90°CW反向) 7回正90°, 0=不动作; arg1=保留恒0 */
#define CAR_SUB_GYRO_ZERO         0x04u   /* 陀螺仪置零(须车静止) */
/* 打靶段内跟瞄 v3.0(刹停/续跑): 位移唯一来源是段表, 视觉只回答"是否已到靶位"。
 * 不变量: 跟瞄态一个字节的 vx/wz 都不写(零标定); 恢复后终点仍是"进段位置 + 段表里程" */
#define CAR_SUB_TRACKING_START    0x05u   /* arg0=保留(旧 speed_level, v3.0 不再生效),
                                             arg1=aim_axis(仅范围校验, v3.0 不再分轴) */
#define CAR_SUB_TRACKING_ERROR    0x06u   /* [error_low_byte,error_high_byte,target_found_flag], 定点=误差归一化*10000 小端 */
#define CAR_SUB_TRACKING_STOP     0x07u   /* arg0=tracking_stop_mode(0=就地刹停/1=放行段表走完当前段), arg1=0保留 */

/* 跟瞄相关宏(TRK_AXIS_* / CAR_EVT_TRACKING_* / TRACKING_* / TRK_FAULT_*)已随跟瞄模块
 * 下沉到 App/nav_tracking.h(经上面 include 仍对本文件可见) */

#define CAR_EVT_NAV_DONE          0x02u   /* 段序列全部完成 */
#define CAR_EVT_NAV_FAULT         0x03u   /* 段执行异常 */
/* CAR_EVT_TURN_DONE(0x04, 转台到位)已随转台模块下沉到 App/nav_turntable.h */

/* 旧短名兼容别名: interurp.c 收发沿用, 值与全称一致, 新代码用全称 */
#define CAR_SUB_REQ_NAV         CAR_SUB_REQ_NAVIGATION

#define CAR_FAULT_WATCHDOG      0x04u   /* 进度看门狗: 里程 3s 无变化 */

/* 固件形态(自测总闸 / 转台使能 / 调试口默认值 / 段表缓冲尺寸)见 GetData/car_config.h,
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

/* ---- 调度层: 段执行由 Motion 库 mf_tick 承担, 本层只做
 *      请求分发/重发幂等/事件上报, 跟瞄叠加见 Tracking_ApplyOverlay ---- */
void Car_Nav_Task(void);
void Car_Nav_MailPush(uint8_t cmd, uint8_t a0, uint8_t a1);

extern Yaw_TypeDef Yaw_struct;

void Car_Event_Send(uint8_t event, uint8_t echo, uint8_t reason);  /* interurp.c */

void Car_Debug_Stat_Print(void);   /*'. '?'/p 周期: 四行状态块(CHASSIS/WHEEL/POSE/WTBL) */
void Car_SegChange_Log(void);      /* 主循环: seg_index/state 变化时打一行 SEG(原库层 ISR printf) */
void Car_ArcExit_Log(void);        /* 主循环: 每条 ARC 段跑完打一行 ARC(出口横向/航向偏差) */
void Car_TrkChange_Log(void);      /* 主循环: 跟瞄状态迁移(四态机)打一行 TRK:旧>新 */
void Car_TrkDetail_Log(void);      /* 主循环: ARMED/STOPPED 期 10Hz 打点 TRKD(判误差卡哪) */

void ConfigHWT101(void);


#endif
