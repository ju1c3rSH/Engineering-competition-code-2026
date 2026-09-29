/**
 * @file    nav_tracking.h
 * @brief   打靶段内跟瞄 v3.0: 刹停 / 续跑 四态机
 *
 * 段表全程是**唯一**位移来源(跟瞄态一个字节的 vx/wz 都不写); 视觉只回答"是否已到靶位"。
 * 中控 START 武装(ARMED), 底盘每拍只**评估**刹停判据, 成立则按固定减速度把段表当拍目标速度
 * 压到 0 -> STOPPED 并报 DONE; 发射后 STOP(1) 回 OFF, 段表从 cur_speed=0 起坡补完剩余里程,
 * 终点仍 = 进段位置 + 段表里程, 故零标定、零污染。规格: docs/打靶段架构-刹停与续跑.md
 *
 * 本模块不 include Control_Car.h(会形成头循环); 协议码统一取 App/car_protocol.h,
 * 跨层接缝在下方声明并注明实现位置。
 */
#ifndef _NAV_TRACKING_H_
#define _NAV_TRACKING_H_

#include <stdint.h>
#include "car_protocol.h"

/* 跟瞄四态机(替代 v2.x 七态) */
typedef enum
{
	TRACKING_OFF = 0,   /* 不干预: 段表全权 */
	TRACKING_ARMED,     /* 已武装: 段表全权驱动, 只评估刹停判据, 不改 cmd */
	TRACKING_STOPPED,   /* 已刹停(暂停本段): cmd 全零, 剩余里程冻结, 等 0x07 STOP(1) */
	TRACKING_FAULT      /* 本地判定瞄不上: 全零 + 上报 0x06, 等 START 复位 */
} TrackingState_t;

/* ---- v3.0 打靶段: 刹停/续跑 (零标定; 下列系数均为经验值, 不需测任何物理量) ----
 * 2026-09-29 起判据主用 **J1 提前点火**: |e| 一进 ±阈 就刹, 不等符号翻转 —— 过零是
 * "已经冲过靶心才知道", 会把整个停止距离留在靶心之后; 提前一个停止距离点火才停在靶心。
 * J2 过零退为兜底(万一整帧漏过阈值窗)。进近在咬靶区用固定低速, 制动距离可控。
 * 完整规格与实测: docs/打靶段架构-刹停与续跑.md §10/§11 */
#define TRACKING_ERROR_THRESHOLD        0.03f   /* ±300 定点: J1 提前点火阈 + "未收敛"判定阈 */
/* 提前点火去抖: 连续这么久 |e|≤阈 才刹。按"帧数"读比按"时长"读好: 帧周期 30ms 时
 * 40ms≈2 帧、17ms 时≈3 帧 —— 帧率越高反而越抗噪。旧的 500/300 是当"收敛驻留"用的,
 * 结果是车一路冲过靶心才刹(现场 TRK:A>S 几乎不出现)。实测 300→40 使过冲 78→26mm */
#define TRACKING_HOLD_TIME_MS           40u
#define TRACKING_FRAME_TIMEOUT_MS       500u    /* 误差帧超时: 超时视为无测量 */
#define TRACKING_ZERO_DEADBAND          0.002f  /* 过零死区: |e| 小于它不计符号(挡噪声) */
#define TRACKING_ZERO_CONFIRM_N         2u      /* 符号去抖: 连续 N 帧同号才更新"已确认符号" */
#define TRACKING_SLOW_ZONE_M   0.7958f  /* 咬靶区 = 本段**最后**这么多米(按剩余里程判): 固定低速与刹停
                                      * 判据都只在这段内生效。2026-09-27 由 1.5→2.5; 2026-09-29 口径
                                      * 去 π 后本群三个"口径 m/m/s"同步 ÷π, 物理含义不变。
                                      * 推导与时间账: docs/打靶段架构-刹停与续跑.md §10.10 */
/* 咬靶区固定低速。**实跑速度 = 本值 + acc·dt**(段表每拍在限速之后再沿加速度抬头一次,
 * mf_path.c 的 seg_speed_ramp)。停点超调 ≈ 滞后(链路+判据)×v + v²/(2·BRAKE);
 * 0.45 与"低滞后"是同一个旋钮的两端, 打破耦合要改成提前判(预测式)。
 * 账(数值均为去 π 前口径): build_verify/tracking_v3_approach_sim.py + tracking_v3_lag_budget.py */
#define TRACKING_SLOW_MPS               0.1432f
/* 触发后的制动减速度(压 cur_speed 用)。2026-09-29 由 2.0 提到 4.0: 制动距离 v²/(2a) 减半。
 * 再往上(6.0)可再省约 1/3, 但要先确认履带不打滑、母线不塌 */
#define TRACKING_BRAKE_MPS2             1.2732f
#define TRACKING_NOCONV_MS              500u    /* 段表已完 + |e|>阈 持续这么久 → FAULT(01) */
#define TRACKING_LOST_DONE_MS           1500u   /* 段表已完 + 无测量 持续这么久 → FAULT(02) */

/* 跟瞄观测快照(仅调试显示用) */
typedef struct { uint8_t state, found, braking, reason;
                 int8_t zero_sign;
                 int16_t err_f10k;
                 uint16_t frame_age_ms, hold_ms; } Trk_Info_t;

void    Tracking_Start(uint8_t speed_level, uint8_t aim_axis);
void    Tracking_Stop(uint8_t mode);
void    Tracking_Abort(void);                 /* 原 Tracking_Goto(TRACKING_OFF) */
uint8_t Tracking_IsHolding(void);             /* STOPPED/FAULT: 有意停(NAV 堵转看门狗豁免用) */
void    Car_Tracking_ErrorPush(int16_t error_fixed, uint8_t found);
void    Tracking_ConsumeErrorFrame(void);     /* Car_Nav_Task 每拍消费一次误差帧 */
char    Tracking_StateChar(TrackingState_t s);
/* 契约: 必须早于 mf_tick —— 只压 path.cur_speed, 轮速环每拍只算一次 */
void    Tracking_LimitApproachSpeed(void);
/* 契约: 必须晚于 mf_tick —— 压的是下一拍起坡起点; 返 1 需重解轮速环 */
uint8_t Tracking_ApplyOverlay(void);
void    Tracking_RecomputeWheels(void);
uint8_t Tracking_TakeChangeLog(uint8_t *from, uint8_t *to);   /* 读后清, 内部关中断 */
void    Tracking_GetInfo(Trk_Info_t *out);                    /* 内部关中断一次拷完 */

#endif /* _NAV_TRACKING_H_ */
