/**
 * @file    mf_path.h
 * @brief   声明式路径段执行器：一条路径 = 以 MF_SEG_END 结尾的 mf_seg_t 常量数组(放 Flash)，
 *          每个控制周期 tick 一次 —— 输入当前位姿, 输出车体速度指令。
 */
#ifndef MF_PATH_H
#define MF_PATH_H

#include <stddef.h>
#include "mf_types.h"
#include "mf_pid.h"

typedef struct
{
    const mf_seg_t *path;   /* 当前路径数组 */
    const mf_seg_t *next_path;  /* 本表跑完将自动续跑的下一条(0=不续)。两处用途:
                                 * ① 末段减速取它的首段速度, 免表尾硬性减到 0;
                                 * ② 跨表时同拍切过去, 消除段表之间的零输出空窗 */
    uint16_t idx;           /* 当前段下标 */
    mf_path_state_t state;
    uint8_t chain_new;      /* 1=刚跨过表边界(上层 pop 后清零), 见 mf_path_pop_chain */

    /* 段进入时刻快照（增量段的基准） */
    mf_pose_t entry;
    mf_pose_t target;       /* GOTO 段解算出的绝对目标点 */

    /* 段内累加量（进段清零） */
    float dist_accum;       /* 已走弧长/位移，m（带符号）；
                             * MF_SEG_CARRY_EN=1 时为**全程连续里程**（进段不清零） */
    uint32_t time_ms;       /* 段内已用时，ms */

    /* 段间累积（仅 MF_SEG_CARRY_EN=1 生效；见 mf_config.h 的说明）：
     * MOVE 段的完成判据改用"累计名义目标 - 当前里程"，把上一段欠走/多走的残差结转过来 */
    float dist_target_m;    /* 累计名义目标里程，m（残差结转的基准） */
    float dist_base_m;      /* 本段起点处的 dist_accum：段内进度 = dist_accum - dist_base_m */
    uint8_t carry_link;     /* 1=上一段是位移段，本段可承接残差；ARC/GOTO 后归 0 */
    uint16_t tol_hold_ms;   /* 到位驻留累计：|err|<tol 连续保持时长 */
    float cur_speed;        /* 斜坡后的当前前向速度，m/s（跨段保持，衔接用） */
    float cur_wz;           /* 斜坡后的当前角速度指令，deg/s（TURN/GOTO 入段缓起） */
    float prev_wz;          /* 上一拍最终输出角速度，deg/s（进段时作 cur_wz 种子） */
    uint8_t subphase;       /* GOTO 段：0=接近点，1=收尾转向 */
    uint8_t started;

    /* ARC 段几何解算（进段时由 entry 位姿 + R·左法线解出，段内只读）：
     * 半径带符号，左转圆心在车体左侧、右转在右侧，用同一个式子即可两向通用 */
    float arc_cx_m;         /* 名义圆心 X（世界系） */
    float arc_cy_m;         /* 名义圆心 Y（世界系） */
    float arc_r_abs;        /* |radius_m| */
    float arc_sgn;          /* +1=左转 / -1=右转（d_yaw 符号；与 radius 同号） */
    float arc_ey_m;         /* 本拍带符号横向偏差 m，正=弧外（诊断与横向环共用） */

    /* ARC 段出口质量记录（段完成那一拍落一次，等上层取走） */
    float arc_exit_ey_m;    /* 出口横向偏差 m，正=弧外 */
    float arc_exit_yawerr_deg;  /* 出口航向偏差 deg，正=车头偏向弧外切线外侧 */
    uint16_t arc_exit_seg;  /* 出口所在的段下标 */
    uint8_t  arc_exit_new;  /* 1=有新记录待取 */

    /* 路径标称航向（仅 MF_SEG_NOM_HEADING_EN=1 生效，见 mf_config.h）：
     * 由路径几何累积 —— 起点用实测航向初始化，之后每过一条 ARC 加 d_yaw、
     * 每过一个 TURN 赋其目标角。MOVE 段的目标航向取它，于是上游残留的航向误差
     * 会被直线段主动纠回来，而不是被 MF_KEEP 锁定后按"航向误差×直线长度"放大 */
    float nom_yaw_deg;      /* 当前段起点处的标称航向 deg */
    uint8_t nom_yaw_init;   /* 0=还没用实测航向定过基准（本段进段时补上） */

    /* 由运动层注入的控制器（所有权在 mf_motion） */
    mf_pid_t *pid_yaw;      /* 航向保持 / 原地转向 / 方位角 P */
    mf_pid_t *pid_point_v;  /* 点到点前向速度 P */
} mf_path_ctx_t;

void mf_path_init(mf_path_ctx_t *p, mf_pid_t *pid_yaw, mf_pid_t *pid_point_v);

/** @brief 开始执行一条路径（须以 MF_SEG_END 结尾）
 *  @param next  本表跑完自动续跑的下一条(须以 MF_SEG_END 结尾); 0=跑完即 DONE。
 *               非空时末段按 next 首段速度收尾, 且跨表同拍切换、输出不归零 */
void mf_path_start(mf_path_ctx_t *p, const mf_seg_t *path, const mf_seg_t *next);

/** @brief 停止并回到 IDLE（同时解除待续表） */
void mf_path_stop(mf_path_ctx_t *p);

/**
 * @brief 取走"刚跨过表边界"标志（读后清）
 *
 * 段表链续时底盘已自行跑进下一条表，但上层仍要按旧 nav_id 上报 NAV_DONE、
 * 并把 nav_id 切到续表。用它把这两件事与几何边界对齐。
 * @return 1=本次取到新边界；0=无
 */
uint8_t mf_path_pop_chain(mf_path_ctx_t *p);

/**
 * @brief 段执行器 tick，每个控制周期调用一次
 * @param pose   当前位姿（里程计输出）
 * @param ds     本周期车体前向位移（m，带符号）
 * @param dt     控制周期 s
 * @param out    输出车体速度指令（执行完毕输出全零）
 * @return 1=当前段刚完成（供上层在段间插入动作），0=未完成/未运行
 */
uint8_t mf_path_tick(mf_path_ctx_t *p, const mf_pose_t *pose,
                     float ds, float dt, mf_body_cmd_t *out);

mf_path_state_t mf_path_get_state(const mf_path_ctx_t *p);
uint16_t mf_path_get_seg_index(const mf_path_ctx_t *p);

/**
 * @brief 当前是否正处在 MF_SEG_WAIT（定时停等）段
 *
 * 上层（如底盘堵转看门狗）用它区分"段表主动要求的原地停等"与"真堵转":
 * WAIT 段内车体本就应静止，里程 3s 无变化不能判成故障。
 * @return 1=正在 WAIT 段；0=非运行态/其它段
 */
uint8_t mf_path_is_waiting(const mf_path_ctx_t *p);

/**
 * @brief 取走最近一次 ARC 段出口的质量记录（读后清标志）
 *
 * 出口横向偏差反映本段跑没跑偏; 出口航向偏差决定其后直线段的方向(MF_KEEP 直线会把它当基准锁住)。
 * 上层在主循环调用并自行加中断保护即可打点, 用于现场逐段验收弧线精度。
 *
 * @param seg_index   出口所在段下标（可传 0）
 * @param ey_m        出口横向偏差 m，正=落在名义弧外侧（可传 0）
 * @param yaw_err_deg 出口航向偏差 deg，正=车头偏向外侧切线（可传 0）
 * @return 1=本次取到新记录；0=无新记录（出参不变）
 */
uint8_t mf_path_pop_arc_report(mf_path_ctx_t *p, uint16_t *seg_index,
                               float *ey_m, float *yaw_err_deg);

float mf_path_remain_m(const mf_path_ctx_t *p);   /* 剩余里程估算(m): 只算位移段, ARC 全额计入 */

#endif /* MF_PATH_H */
