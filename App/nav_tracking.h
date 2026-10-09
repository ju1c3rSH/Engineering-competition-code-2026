/**
 * @file    nav_tracking.h
 * @brief   打靶段内跟瞄：刹停 / 续跑 四态机
 *
 * 不变量: 段表全程是**唯一**位移来源(跟瞄态一个字节的 vx/wz 都不写), 视觉只回答"是否已到
 * 靶位"。START 武装(ARMED)后底盘每拍只**评估**刹停判据, 成立则按固定减速度把段表当拍目标
 * 速度压到 0 -> STOPPED 报 DONE; 发射后 STOP(1) 回 OFF, 段表从 cur_speed=0 起坡补完剩余里程,
 * 终点仍 = 进段位置 + 段表里程 ⇒ 零标定零污染。规格: docs/打靶段架构-刹停与续跑.md
 * 本模块不 include Control_Car.h(会成环); 协议码统一取 App/car_protocol.h。
 */
#ifndef _NAV_TRACKING_H_
#define _NAV_TRACKING_H_

#include <stdint.h>
#include "car_protocol.h"

/* 跟瞄四态机 */
typedef enum
{
	TRACKING_OFF = 0,   /* 不干预: 段表全权 */
	TRACKING_ARMED,     /* 已武装: 段表全权驱动, 只评估刹停判据, 不改 cmd */
	TRACKING_STOPPED,   /* 已刹停(暂停本段): cmd 全零, 剩余里程冻结, 等 0x07 STOP(1) */
	TRACKING_FAULT      /* 本地判定瞄不上: 全零 + 上报 0x06, 等 START 复位 */
} TrackingState_t;

/* ---- 打靶段: 刹停/续跑 (零标定; 下列系数均为经验值, 不需测任何物理量) ----
 * 刹停判据主用**进阈即刹**: |e| 一进 ±阈就刹, 不等符号翻转 —— 过零是"已经冲过靶心才知道",
 * 会把整个停止距离留在靶心之后; 提前一个停止距离点火才停在靶心。
 * 符号翻转(过零)退为兜底(万一整帧漏过阈值窗)。进近在咬靶区用固定低速, 制动距离可控。
 * 完整规格与实测: docs/打靶段架构-刹停与续跑.md §10/§11 */
#define TRACKING_ERROR_THRESHOLD        0.03f   /* ±300 定点: 进阈即刹的阈 + "未收敛"判定阈 */
/* 去抖: 连续这么久 |e|≤阈 才刹。按"帧数"读比按"时长"读好: 帧周期 30ms 时
 * 40ms≈2 帧、17ms 时≈3 帧 —— 帧率越高反而越抗噪。旧的 500/300 是当"收敛驻留"用的,
 * 结果是车一路冲过靶心才刹(现场 TRK:A>S 几乎不出现)。实测 300→40 使过冲 78→26mm */
#define TRACKING_HOLD_TIME_MS           40u
#define TRACKING_FRAME_TIMEOUT_MS       500u    /* 误差帧超时: 超时视为无测量 */
#define TRACKING_ZERO_DEADBAND          0.002f  /* 过零死区: |e| 小于它不计符号(挡噪声) */
#define TRACKING_ZERO_CONFIRM_N         2u      /* 符号去抖: 连续 N 帧同号才更新"已确认符号" */
#define TRACKING_SLOW_ZONE_M   2.5f  /* 咬靶区 = 本段**最后**这么多米(按剩余里程判): 固定低速与刹停判据
                                      * 都只在这段内生效。2026-09-27 由 1.5→2.5(靶心落在"段尾往前
                                      * 0~1.2m", 窗口 2.5m 才对 d 的跨趟波动 0.5~1.2m 免疫)。
                                      * 推导与时间账: docs/打靶段架构-刹停与续跑.md §10.10 */
/* 咬靶区固定低速。**实跑速度 = 本值 + acc·dt**(段表每拍在限速之后再沿加速度抬头一次,
 * mf_path.c 的 seg_speed_ramp) = 0.474m/s。停点超调 ≈ 滞后(链路+判据)×v + v²/(2·BRAKE):
 * 只算判据滞后是 82~108mm, 按三级链路时序算进去约 **+134mm**(0.20 时 +51mm) —— 2026-09-28 提速时
 * 主动接受的取舍; 0.45 与"低滞后"是同一个旋钮的两端, 打破耦合要改成提前判(预测式)。
 * 账: build_verify/tracking_v3_approach_sim.py(无链路) + tracking_v3_lag_budget.py(含链路) */
#define TRACKING_SLOW_MPS               0.45f
/* 触发后的制动减速度(压 cur_speed 用)。2026-09-29 由 2.0 提到 4.0: 制动距离 v²/(2a)
 * 从 56mm 降到 28mm。再往上(6.0)可再省约 13mm, 但要先确认履带不打滑、母线不塌 */
#define TRACKING_BRAKE_MPS2             4.0f
#define TRACKING_NOCONV_MS              500u    /* 段表已完 + |e|>阈 持续这么久 → FAULT(01) */
#define TRACKING_LOST_DONE_MS           1500u   /* 段表已完 + 无测量 持续这么久 → FAULT(02) */

/* ---- 预测式提前制动(斜率外推) ----
 * 车还要走 v·L + v²/(2a) 才停; 用误差帧的 |de/dt| 把这距离折算到 e 域(不必标相机几何 S,
 * v 与 S 同时消掉): 触发阈 = |de/dt|·(L + v/(2a))。该阈与固定阈**取大者** ⇒ 斜率不可用、
 * 没在接近、或没进咬靶区时, 行为与固定阈完全一致(只会更早刹, 不会更晚, 回退即现状)。
 * 标定 L: 跑一趟读停车那拍的 |E|(定点), 停点冲过靶心就加大、没到靶心就减小, 一次即定。
 * 实测口径与反解见 docs/打靶段架构-刹停与续跑.md §10.15 */
#define TRACKING_PRED_EN                1u      /* 0 = 关(回退现状) */
#define TRACKING_LAG_MS                 80u     /* L: 视觉取像推理+转发+拍对齐(不含制动段); build_verify/calibrate_lag_ms.py 标定 */
#define TRACKING_ERR_WIN                6u      /* 斜率窗口(帧)。两点差分噪声达均值 ±40%, 必须最小二乘 */
#define TRACKING_ERR_MIN_DT_MS          12u     /* 相邻样本最小间隔: 更密的重复帧不喂进窗口 */
#define TRACKING_ERR_MIN_SPAN_MS        40u     /* 窗口最小跨度: 不足则判据不可用, 走固定阈 */
#define TRACKING_ERR_MIN_SLOPE          0.02f   /* e/s: 接近速度低于它不预测(没在收敛, 提前量无意义) */
#define TRACKING_ERR_MAX_SLOPE          0.80f   /* e/s: 斜率限幅, 挡噪声把提前量放大成欠冲 */
#define TRACKING_ERR_SAT_LEVEL          0.95f   /* |e| 到此=量程饱和。实测 found=1 也会长时间给 ±0.96 的
                                                 * 常值(2026-10-09 现场 2.5s), 它既不是"接近速度"、也不是
                                                 * 重捕后的可信起点 ⇒ 与 found=0 同处理: 清窗 */
#if (TRACKING_ERR_WIN < 3u)
#error "TRACKING_ERR_WIN 必须 >= 3"
#endif

/* 跟瞄观测快照(仅调试显示用) */
typedef struct { uint8_t state, found, braking, reason;
                 int8_t zero_sign;
                 int16_t err_f10k;
                 int16_t thr_f10k;    /* 当拍实际生效的点火阈(定点): 标定 LAG 与判"预测是否生效"都读它 */
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
