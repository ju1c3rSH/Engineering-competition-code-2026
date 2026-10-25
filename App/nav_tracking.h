/**
 * @file    nav_tracking.h
 * @brief   打靶段内跟瞄：刹停 / 续跑 四态机
 *
 * 不变量: 段表全程是**唯一**位移来源(跟瞄态一个字节的 vx/wz 都不写), 视觉只回答"是否已到
 * 靶位"。START 武装(ARMED)后底盘每拍只**评估**刹停判据, 成立则按固定减速度把段表当拍目标
 * 速度压到 0 -> STOPPED; 自动刹停路径再走 trim 子流程(E 闭环低速爬行微调, 收敛才发 DONE,
 * 见 TRACKING_TRIM_* 与 docs/打靶段架构-刹停与续跑.md §10.18), 指令刹停(0x07 mode0)即时发
 * DONE。"段表是唯一位移来源"指 ARMED 态一个字节不写; trim 爬行(STOPPED 态)是**唯一例外**
 * —— overlay 持有 cmd, 位移由段表账本按实际轮程入账, 终点不变量由 §10.18 的预算护栏保住。
 * 发射后 STOP(1) 回 OFF, 段表从 cur_speed=0 起坡补完剩余里程,
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
#define TRACKING_ERROR_THRESHOLD        0.008f  /* 地板只挡噪声(须>死区20与帧量化33计数), 不再背停车距离:
                                                 * 实测停点=点火帧−C, C 随点火时车速强变化(0.15m/s→C≈60,
                                                 * 0.45→C≈454), 固定大地板两头不讨好(300 停 +59 / 250 停
                                                 * +190, 同一辆车两趟)。速度自适应交给预测阈(∝v, 天然跟随),
                                                 * 见 docs/打靶段架构-刹停与续跑.md §10.16 */
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
/* 近靶二级限速: 咬靶区内 |e|≤NEAR_E_THR(定点计数) 时把限速从 SLOW_MPS 再压到 NEAR_MPS
 * —— 剩余停点误差三项(帧量化/触发抖动/斜率噪声)都 ∝ v, 全部缩到 1/3。限速按**帧龄**持有
 * (found=0 闪断帧不释放, 见 .c 内注释), 饱和/断流>FRAME_TIMEOUT 才释放回 SLOW_MPS;
 * **故意不加欠冲护栏**: 本限速永不到 0, 车不会停在段表中途(上 §11 到点律时需重评)。
 * 实测 S≈1.5m 时 ±1000 ≈ 0.15m 窗; 窗太紧(E 在减速完成前过零)就加大, 与 LAG 同款一次标定 */
#define TRACKING_NEAR_EN                1u      /* 0 = 关(回退现状) */
#define TRACKING_NEAR_E_THR             1000u   /* |e|≤此值(×1e-4)进二级限速 */
#define TRACKING_NEAR_MPS               0.15f   /* 近靶限速; 制动距离 0.15²/(2·BRAKE)≈5.6mm */
/* 停稳判据: 制动**目标**到零 ≠ 物理停稳 —— 低速轮速环制动偏软(0.15m/s 下实测等效
 * ~0.6m/s², 远低于 BRAKE 4), 目标零后还要滑 ~10mm, 此刻报 DONE 会让开火早于停点
 * (2026-10-10 现场"开火早"第二层根因)。轮速实测 ≤ 阈驻留 SETTLE_MS 才进 STOPPED;
 * 兜底上限防测速异常导致 DONE 永不来(超时=带滑行量报, Brain 超时容错仍在) */
#define TRACKING_SETTLE_EN              1u      /* 0 = 回旧行为(制动目标到零即报 DONE) */
#define TRACKING_SETTLE_MPS             0.02f
#define TRACKING_SETTLE_MS              150u
#define TRACKING_SETTLE_GIVEUP_MS       1000u
#define TRACKING_NOCONV_MS              500u    /* 段表已完 + |e|>阈 持续这么久 → FAULT(01) */
#define TRACKING_LOST_DONE_MS           1500u   /* 段表已完 + 无测量 持续这么久 → FAULT(02) */

/* ---- 刹停后微调(trim): STOPPED 内 E 闭环低速爬行, 收敛才发 DONE ----
 * 动机: FIRE_PAST 标定只消系统性停点偏差, 趟间随机分量(帧量化/E噪声/地面变化)与标定
 * 漂移治不了。STOPPED 进 EVAL: |E|<=TOL 直接 DONE; 窗外按在线估计的增益/滞后做带滑行
 * 补偿的比例步(爬行速度由滑行预算二次方程解出, 保证小修正也跨 >=1 帧有采样机会),
 * 收敛/超限才发 DONE。全程不依赖 px↔mm 换算(旧注释两套口径矛盾已删); 微调里程由段表
 * dist_accum 按实际轮程自动入账, STOP(1) 放行补完的剩余里程自动扣除, 下游成对常数不受
 * 影响。收敛性: build_verify/trim_loop_sim.py(gain 5~80 计数/mm × L 0.04~0.15s 全扫 100%)。
 * 规格: docs/打靶段架构-刹停与续跑.md §10.18 */
#define TRACKING_TRIM_EN                1u      /* 0 = 回旧行为(停稳即发 DONE) */
#define TRACKING_TRIM_TOL_E             40      /* DONE 窗: |E|<=此值(计数x1e-4) ≈1.3px */
#define TRACKING_TRIM_V_MAX_MPS         0.030f  /* 爬行速度上限; 下限见 trim_loop_sim.py 推导 */
#define TRACKING_TRIM_V_MIN_MPS         0.004f  /* 速度下限: 小修正也保证 >=1 帧采样机会 */
#define TRACKING_TRIM_GAIN_DEF          30.0f   /* 增益初值(计数/mm), 仅第一迭代用, 之后实测 */
#define TRACKING_TRIM_L_INIT_S          (TRACKING_LAG_MS * 0.001f)  /* 滞后初值, 之后实测 */
#define TRACKING_TRIM_MAX_ITER          4u      /* 迭代上限, 超限 GIVEUP */
#define TRACKING_TRIM_GIVEUP_MS         5000u   /* 总时长上限(EVAL/CREEP 都查); 最坏 ≈5s+末步停稳
                                                 * ≈6s; Brain nav_ms=30s, TARGET_PROBE=8s 兼容 */
#define TRACKING_TRIM_FIRST_WAIT_MS     300u    /* 每次 EVAL 等新帧上限(视觉死 -> GIVEUP) */
#define TRACKING_TRIM_FRAME_S           0.060f  /* 名义帧周期(规划用, 保守取 16.7Hz) */
#define TRACKING_TRIM_MIN_STEP_MM       0.2f    /* 最小指令步(防御: 理论上规划不会给出更小值) */
#define TRACKING_TRIM_MAX_MOVE_M        0.060f  /* 单步上限(后退方向; 前进由段表剩余里程管) */
#define TRACKING_TRIM_BUDGET_MARGIN_M   0.005f  /* 前进预算余量: 微调不许吃穿段表名义端点 */

/* ---- 预测式提前制动(斜率外推) ----
 * 车还要走 v·L + v²/(2a) 才停; 用误差帧的 |de/dt| 把这距离折算到 e 域(不必标相机几何 S,
 * v 与 S 同时消掉): 触发阈 = |de/dt|·(L + v/(2a))。该阈与固定阈**取大者** ⇒ 斜率不可用、
 * 没在接近、或没进咬靶区时, 行为与固定阈完全一致(只会更早刹, 不会更晚, 回退即现状)。
 * 标定 L: 跑一趟读停车那拍的 |E|(定点), 停点冲过靶心就加大、没到靶心就减小, 一次即定。
 * 实测口径与反解见 docs/打靶段架构-刹停与续跑.md §10.15 */
#define TRACKING_PRED_EN                1u      /* 0 = 关(回退现状) */
#define TRACKING_LAG_MS                 69u     /* L: 视觉取像推理+转发+拍对齐(不含制动段)。
                                                 * 标定=跑一趟读停点E: 停点冲过靶心就加大、没到就减小,
                                                 * 一次即定; 标定步长别小于帧量化噪声(约±5ms)。
                                                 * (旧注释的"计数↔ms↔mm"换算与 FIRE_PAST_E 口径矛盾且
                                                 * 无法推导, 2026-10-25 删 —— E↔里程换算一律现场实测) */
#define TRACKING_ERR_WIN                6u      /* 斜率窗口(帧)。两点差分噪声达均值 ±40%, 必须最小二乘 */
#define TRACKING_ERR_MIN_DT_MS          12u     /* 相邻样本最小间隔: 更密的重复帧不喂进窗口 */
#define TRACKING_ERR_MIN_SPAN_MS        40u     /* 窗口最小跨度: 不足则判据不可用, 走固定阈 */
#define TRACKING_ERR_MIN_SLOPE          0.02f   /* e/s: 接近速度低于它不预测(没在收敛, 提前量无意义) */
#define TRACKING_ERR_MAX_SLOPE          0.80f   /* e/s: 斜率限幅, 挡噪声把提前量放大成欠冲 */
#define TRACKING_ERR_SAT_LEVEL          0.95f   /* |e| 到此=量程饱和。实测 found=1 也会长时间给 ±0.96 的
                                                 * 常值(2026-10-09 现场 2.5s), 它既不是"接近速度"、也不是
                                                 * 重捕后的可信起点 ⇒ 与 found=0 同处理: 清窗 */
/* 定点开火(带符号, >0 启用): E ≤ +FIRE_PAST_E 就点火 —— E>0=靶心仍在车前方,
 * 天然欠冲。停点 = 点火点 - C(C ≈ 110~115 计数, 近靶速下恒定), 故微调
 * 永远只需“前进”一个方向。
 * 视觉侧已做弹着纠偏 ⇒ 目标就是 E→0, 本宏不再背目标偏置,
 * 只需“够大”保证欠冲。
 * 标定 1:1: 跑一趟读 TRK:A>S 行的 E_stop —— E_stop<0(过冲) ⇒ 宏 += |E_stop|+40,
 *        E_stop>+80(欠冲) ⇒ 宏 -= (E_stop-30); 两趟收敛后永不重标。
 * 启用后 J1 进阀窗与预测式同时退位(TRKD 的 THR 行改显点火点,
 * 带符号); J2 过零兜底保留 */
#define TRACKING_FIRE_PAST_EN           1u      /* 0 = 回退进阀窗点火 */
#define TRACKING_FIRE_PAST_E            150u    /* 点火点(计数, >0 启用)。停点 = 点火点 - C, C 现场标:
                                                 * 跑一趟读 TRK:A>S 行的 EF 与 E_stop, C = EF - E_stop。
                                                 * (旧注释"150计数=4.8px≈22mm"的 px↔mm 换算无法推导, 已删) */
#if (TRACKING_ERR_WIN < 3u)
#error "TRACKING_ERR_WIN 必须 >= 3"
#endif

/* trim 子相位(TRKD 行 T= 字段, Tracking_TrimChar 给字母) */
#define TRIM_PHASE_IDLE     0u  /* 未启用(TRIM_EN=0 / 未进 STOPPED); 指令刹停直接置 FINISHED */
#define TRIM_PHASE_EVAL     1u  /* 读 E 判收敛/规划下一步 */
#define TRIM_PHASE_CREEP    2u  /* 低速爬行中 */
#define TRIM_PHASE_SETTLE   3u  /* 移动后等物理停稳 */
#define TRIM_PHASE_FINISHED 4u  /* 终态: DONE 已发(或待补发) */

/* trim 终态成因(EVT reason / TRKD 的 GV= / TRKT 终态记录): 0=收敛, 非 0=护栏放行。
 * 细分成因只为归因 —— Brain 对 DONE 的 reason 字节不解析, 逐字节前向兼容 */
#define TRIM_OUTCOME_CONV      0u  /* |E| 进窗收敛 */
#define TRIM_OUTCOME_CAP       1u  /* 迭代(4)或总时长(5s)超限 */
#define TRIM_OUTCOME_NOFRAME   2u  /* 等新帧超时(视觉死/断流) */
#define TRIM_OUTCOME_SIGN      3u  /* 几何反转: 朝 sign(E) 方向走而 E 反向显著增大 */
#define TRIM_OUTCOME_BUDGET    4u  /* 前向预算/单步上限吃光, 无法再进 */
#define TRIM_OUTCOME_ABORT     5u  /* 微调中途被主控 0x07 mode0 点名刹停 */

/* trim 事件记录(TRKT 行, 归因黑匣子): ISR 生产(TrimTick), 主循环读后清(Car_TrimLog)。
 * 一轮 trim ≤ BEGIN + 4×MOVED + END = 6 条, 环满覆盖最旧(终态记录不可丢) */
typedef struct
{
	uint32_t ms;        /* trk_ms 时基(4ms/拍, 只用差值) */
	int16_t  e_f10k;    /* 事件拍的 E(定点) */
	int16_t  f1, f2;    /* 类型相关, 见 TRKT_* 注释 */
	uint8_t  type;      /* TRKT_* */
	uint8_t  aux;       /* 类型相关 */
} TrkTrimLog_t;
#define TRKT_BEGIN  1u  /* 进微调: f1=前向预算x10(mm) */
#define TRKT_MOVED  2u  /* 一步完成: f1=指令里程x10(mm), f2=实际轮程x10(mm),
                         * aux=停止原因(0=走满指令量 1=E窗背停提前收令 2=单步超时/堵转
                         *            3=总时长到, 停稳后 EVAL 必然 CAP 终态) */
#define TRKT_END    3u  /* 终态: f1=终态成因(TRIM_OUTCOME_*), f2=在线增益x10(计数/mm),
                         * aux=已移动次数; e_f10k=残余 E */

/* 跟瞄观测快照(仅调试显示用) */
typedef struct { uint8_t state, found, braking, reason, near, settling;
                 int8_t zero_sign;
                 int16_t err_f10k;
                 int16_t thr_f10k;    /* 当拍点火点(定点, 带符号): 标定 FIRE_PAST_E 读它, 正常恒 +150 */
                 int16_t fire_e_f10k; /* 触发制动那一拍的 E(定点): C = E_stop - 本值,
                                       * 本值只在点火那一拍刷新(见 Tracking_BrakeStart);
                                       * 0=本轮尚未点火(指令刹停/FAULT 不刷它) */
                 uint16_t frame_age_ms, hold_ms;
                 uint8_t trim_phase, trim_iter, trim_gaveup;
                 int16_t trim_resid_f10k;  /* 发 DONE 那拍的 E(定点) */
                 int16_t trim_gain_x10;    /* 在线增益×10(计数/mm), 观测换算率用 */ } Trk_Info_t;
/* trim_gaveup 字段实为终态成因码(0=收敛, 1~5=护栏细分, 见 TRIM_OUTCOME_*) */

void    Tracking_Start(uint8_t speed_level, uint8_t aim_axis);
void    Tracking_Stop(uint8_t mode);
void    Tracking_Abort(void);                 /* 原 Tracking_Goto(TRACKING_OFF) */
uint8_t Tracking_IsHolding(void);             /* STOPPED/FAULT: 有意停(NAV 堵转看门狗豁免用) */
void    Car_Tracking_ErrorPush(int16_t error_fixed, uint8_t found);
void    Tracking_ConsumeErrorFrame(void);     /* Car_Nav_Task 每拍消费一次误差帧 */
char    Tracking_StateChar(TrackingState_t s);
char    Tracking_TrimChar(uint8_t trim_phase);                       /* TRKD 行 T= 字段 */
/* 契约: 必须早于 mf_tick —— 只压 path.cur_speed, 轮速环每拍只算一次 */
void    Tracking_LimitApproachSpeed(void);
/* 契约: 必须晚于 mf_tick —— 压的是下一拍起坡起点; 返 1 需重解轮速环 */
uint8_t Tracking_ApplyOverlay(void);
void    Tracking_RecomputeWheels(void);
uint8_t Tracking_TakeChangeLog(uint8_t *from, uint8_t *to);   /* 读后清, 内部关中断 */
uint8_t Tracking_TakeTrimLog(TrkTrimLog_t *out);              /* 读后清, 返回 0=无; 内部关中断 */
void    Tracking_GetInfo(Trk_Info_t *out);                    /* 内部关中断一次拷完 */

#endif /* _NAV_TRACKING_H_ */
