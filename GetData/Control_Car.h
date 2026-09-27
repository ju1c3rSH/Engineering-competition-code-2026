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

/* 收敛轴码表: v3.0 不再分轴(一律不写 vx/wz), 两个宏仅用于入参范围校验 */
#define TRK_AXIS_ROTATE           0x00u
#define TRK_AXIS_TRANSLATE        0x01u

#define CAR_EVT_NAV_DONE          0x02u   /* 段序列全部完成 */
#define CAR_EVT_NAV_FAULT         0x03u   /* 段执行异常 */
#define CAR_EVT_TURN_DONE         0x04u   /* 转台到位 */
#define CAR_EVT_TRACKING_DONE     0x05u   /* 已刹停(暂停本段), 可发射; 等 0x07 STOP(1) 放行续跑 */
#define CAR_EVT_TRACKING_FAULT    0x06u   /* 本地判定瞄不上(reason 见 TRK_FAULT_*), Brain 容错定夺 */

/* ---- v3.0 打靶段: 刹停/续跑 (零标定; 下列系数均为经验值, 不需测任何物理量) ----
 * 判据主用 J2 过零(车从靶心旁经过那一拍), J1 阈值+驻留兜底(整段符号不换号时);
 * 进近在咬靶区用固定低速, 保证制动距离可控。完整规格见
 * docs/打靶段架构-刹停与续跑.md §10 */
#define TRACKING_ERROR_THRESHOLD        0.03f   /* ±300 定点: J1 兜底阈 + "未收敛"判定阈 */
#define TRACKING_HOLD_TIME_MS           500u    /* J1: |e|≤阈 连续这么久即触发刹停 */
#define TRACKING_FRAME_TIMEOUT_MS       500u    /* 误差帧超时: 超时视为无测量 */
#define TRACKING_ZERO_DEADBAND          0.002f  /* 过零死区: |e| 小于它不计符号(挡噪声) */
#define TRACKING_ZERO_CONFIRM_N         2u      /* 符号去抖: 连续 N 帧同号才更新"已确认符号" */
#define TRACKING_SLOW_ZONE_M            2.5f    /* 咬靶区 = 本段**最后**这么多米(按剩余里程判):
                                                 * 固定低速与刹停判据都只在这段内生效。
                                                 * 2026-09-27 由 1.5 → 2.5: 延长 tbl_target 后,
                                                 * 靶心落在"段尾往前 0~1.2m"都有可能, 窗口留 2.5m
                                                 * 才能让"过零点落进咬靶区"这条前提对 d 的跨趟波动
                                                 * (0.5~1.2m)免疫。代价: 低速段 2.5m/0.2m/s=12.5s,
                                                 * 段时 ≈17s, nav_ms=30s 仍有余量 */
#define TRACKING_SLOW_MPS               0.20f   /* 咬靶区固定低速(替代按 |e| 成比例的限速) */
#define TRACKING_BRAKE_MPS2             2.0f    /* 触发后的制动减速度(压 cur_speed 用) */
#define TRACKING_NOCONV_MS              500u    /* 段表已完 + |e|>阈 持续这么久 → FAULT(01) */
#define TRACKING_LOST_DONE_MS           1500u   /* 段表已完 + 无测量 持续这么久 → FAULT(02) */

/* TRACKING_FAULT(事件 0x06)的 reason 码: 上报给主控做决策(重试/容错发射) */
#define TRK_FAULT_NOT_CONVERGED         0x01u   /* 到位后长时间未收敛 */
#define TRK_FAULT_NO_TARGET             0x02u   /* 到位后丢失目标(遮挡/阈值/相机) */
#define TRK_FAULT_NO_PATH               0x03u   /* 收到 START 时无活动段表(未发 REQ_NAV/段启动失败) */

/* 旧短名兼容别名: interurp.c 收发沿用, 值与全称一致, 新代码用全称 */
#define CAR_SUB_REQ_NAV         CAR_SUB_REQ_NAVIGATION

#define CAR_FAULT_WATCHDOG      0x04u   /* 进度看门狗: 里程 3s 无变化 */

/* 固件形态(自测总闸 / 转台使能 / 调试口默认值 / 段表缓冲尺寸)见 GetData/car_config.h,
 * 自测覆盖见 Selftest/selftest.h —— 本文件不再持有任何测试宏 */
#define CAR_NAV_ID_COUNT        9u      /* NavTarget_t 契约(与 Brain mission_sm.h 一致): 0~5 任务路线, 6~8 重锚伪路线 */

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
void Emm_Pos_Control(void);   /* TIM5 转台重发: 内部带 tt_arm 门控+方向/角度成对快照 */
void Car_Turntable_BootZero(void);   /* 上电零点对齐: 清零驱动器计数 + Motor_Angle=TURNTABLE_BOOT_DEG。
                                      * 必须在 TIM5 启动前调用一次(见 Control_Car.c 内注释) */

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

void Car_Debug_Stat_Print(void);   /*'. '?'/p 周期: 四行状态块(CHASSIS/WHEEL/POSE/WTBL) */
void Car_SegChange_Log(void);      /* 主循环: seg_index/state 变化时打一行 SEG(原库层 ISR printf) */
void Car_ArcExit_Log(void);        /* 主循环: 每条 ARC 段跑完打一行 ARC(出口横向/航向偏差) */
void Car_TrkChange_Log(void);      /* 主循环: 跟瞄状态迁移(四态机)打一行 TRK:旧>新 */
void Car_TrkDetail_Log(void);      /* 主循环: ARMED/STOPPED 期 10Hz 打点 TRKD(判误差卡哪) */

/* 台架直转(USART2 'w'/'e'/'1'~'7'/'x', Car_DebugTask 主循环执行): 微调不报
 * TURN_DONE, 打码走协议码表 */
void Car_Turntable_BenchNudge(float delta_deg);
void Car_Turntable_BenchCode(uint8_t turn_code);
void Car_Turntable_BenchStop(void);
void Car_Dbg2_TtRecord(uint8_t kind, uint8_t code, float angle_deg);  /* interurp.c */

void ConfigHWT101(void);


#endif
