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

/* NACK 原因码枚举 (panzer_links_protocol.md v2.2 §1.4, 三链路统一) */
#define CAR_NACK_FORMAT       0x01u   /* 长度不足/非REQUEST/data_id 不支持 */
#define CAR_NACK_BUSY         0x02u   /* 忙/未就绪(预留) */
#define CAR_NACK_PARAM        0x03u   /* 子命令存在但参数越界 */
#define CAR_NACK_UNKNOWNSUB   0x04u   /* data_id 合法但 sub_cmd 不在定义表 */

#define CAR_DATA_ID_CTRL        0x21u   /* CMD_CAR_CTRL: 运动请求(下行 sub_cmd 式) */
#define CAR_DATA_ID_EVT         0x22u   /* EVT_CAR_EVENT: 底盘事件上报(上行) */

#define CAR_SUB_REQ_NAVIGATION  0x01u   /* arg0=navigation_target_id 0~7(6/7=重锚伪路线), arg1=speed_gear 0~3 */
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
/* =========================================================================
 * 段内跟瞄参数(状态机实现见 Control_Car.c 的 Tracking_ApplyOverlay / Tracking_Goto)
 *
 * 重构说明(2026-09-23): 旧实现是"IDLE/RUN/LOCKED 三态 + overlay 覆盖 cmd", 把"到位"与
 * "收敛"混在一起, 现场表现为打靶段"有误差但车不往前走"。现改为六态机
 * (OFF / APPROACH / AIM / HOLD / LOCKED / FAULT), 四条不变量:
 *   · APPROACH 不碰 vx(速度权归段表), 只写 wz;  wz 由跟瞄**覆盖**而非叠加;
 *   · 只有"段表到位后的 AIM"才允许判收敛, DONE 的含义 = 到位且收敛;
 *   · APPROACH 丢目标不停车(停了就看不到靶), AIM 丢目标才压零且有上限;
 *   · LOCKED/FAULT 可被 START / 同 id REQ_NAV / STOP 解除, 不留黏性态。
 * 各量的物理意义与现场调法写在下面; 调参只改这一段, 不要再散落到别处。
 * ========================================================================= */

#define TRACKING_ERROR_THRESHOLD        0.05f   /* 收敛阈值: 归一化横向误差 = 横向像素/半幅宽。
                                                 * 物理上 ≈ tan(半视场角)·e, 应按"激光光斑允许
                                                 * 半径对应的角度"反推。现场若因抖动收不敛,
                                                 * 先查视觉去抖与帧率, 不要一味放大它 */
#define TRACKING_HOLD_TIME_MS           500u    /* 收敛需连续达标时长: 取车体摆动周期的 1~2 倍 */
#define TRACKING_FRAME_TIMEOUT_MS       500u    /* 测量失效阈值: 应 > 主控透传周期(30ms)×N +
                                                 * 视觉帧周期(50ms)×N; 太小会被偶发丢帧打断 */
#define TRACKING_PROPORTIONAL_GAIN      60.0f   /* wz = -Kp·e (dps/单位误差)。**符号即镜像方向**:
                                                 * 台架给固定 e, 车头应朝靶转; 反向就取负 */
#define TRACKING_MAX_YAW_RATE_DPS       60.0f   /* 跟瞄角速度限幅(仍受底盘 max_wz_dps 二次钳位) */
#define TRACKING_PATH_WZ_KEEP_PCT       0u      /* 跟瞄态保留段表 wz 的比例(%): 0 = 跟瞄独占 wz
                                                 * (段表航向环让出, 避免两个控制器对抗); 若视觉
                                                 * 帧率低、修正带宽不够, 可给 20~40 做前馈 */

/* ---- 有界收尾: 三种"瞄不上"都必须有终点, 否则主控只能等自己的超时 ---- */
#define TRACKING_AIM_LOST_MAX_MS        3000u   /* AIM(已到位)丢目标上限 -> FAULT(丢目标); 0=关 */
#define TRACKING_AIM_FAULT_MS           5000u   /* AIM 下 |e| 超阈上限 -> FAULT(未收敛); 0=关 */

/* TRACKING_FAULT(事件 0x06)的 reason 码: 上报给主控做决策(重试/容错发射) */
#define TRK_FAULT_NOT_CONVERGED         0x01u   /* 到位后长时间未收敛 */
#define TRK_FAULT_NO_TARGET             0x02u   /* 到位后丢失目标(遮挡/阈值/相机) */
#define TRK_FAULT_NO_PATH               0x03u   /* 收到 START 时无活动段表(未发 REQ_NAV/段启动失败) */

/* 旧短名兼容别名: interurp.c 收发沿用, 值与全称一致, 新代码用全称 */
#define CAR_SUB_REQ_NAV         CAR_SUB_REQ_NAVIGATION

#define CAR_FAULT_WATCHDOG      0x04u   /* 进度看门狗: 里程 3s 无变化 */

/* 固件形态(自测总闸 / 转台使能 / 调试口默认值 / 段表缓冲尺寸)见 GetData/car_config.h,
 * 自测覆盖见 Selftest/selftest.h —— 本文件不再持有任何测试宏 */
#define CAR_NAV_ID_COUNT        8u      /* NavTarget_t 契约(与 Brain mission_sm.h 一致): 0~5 任务路线, 6~7 重锚伪路线 */

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
void Car_TrkChange_Log(void);      /* 主循环: 跟瞄状态迁移(六态机)打一行 TRK:旧>新 */

/* 台架直转(USART2 'w'/'e'/'1'~'7'/'x', Car_DebugTask 主循环执行): 微调不报
 * TURN_DONE, 打码走协议码表 */
void Car_Turntable_BenchNudge(float delta_deg);
void Car_Turntable_BenchCode(uint8_t turn_code);
void Car_Turntable_BenchStop(void);
void Car_Dbg2_TtRecord(uint8_t kind, uint8_t code, float angle_deg);  /* interurp.c */

void ConfigHWT101(void);


#endif
