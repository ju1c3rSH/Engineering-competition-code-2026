/**
 * @file    mf_path.h
 * @brief   声明式路径段执行器，支持增量式、绝对式
 *
 * 一条路径 = mf_seg_t 常量数组（放 Flash），以 MF_SEG_END 结尾。
 * 执行器每个控制周期被 tick 一次：输入当前位姿，输出车体速度指令。
 *
 * 与 stmtest/mc_path 的关系：吸收其“进段快照 + 累计量完成判据”的增量执行框架，
 * 新增了绝对坐标段（GOTO_XY）与声明式收尾朝向，并把转向统一成段类型。
 */
#ifndef MF_PATH_H
#define MF_PATH_H

#include <stddef.h>
#include "mf_types.h"
#include "mf_pid.h"

typedef struct
{
    const mf_seg_t *path;   /* 当前路径数组 */
    uint16_t idx;           /* 当前段下标 */
    mf_path_state_t state;

    /* 段进入时刻快照（增量段的基准） */
    mf_pose_t entry;
    mf_pose_t target;       /* GOTO 段解算出的绝对目标点 */

    /* 段内累加量（进段清零） */
    float dist_accum;       /* 已走弧长/位移，m（带符号） */
    uint32_t time_ms;       /* 段内已用时，ms */
    float cur_speed;        /* 斜坡后的当前前向速度，m/s */
    uint8_t subphase;       /* GOTO 段：0=接近点，1=收尾转向 */
    uint8_t started;

    /* 由运动层注入的控制器（所有权在 mf_motion） */
    mf_pid_t *pid_yaw;      /* 航向保持 / 原地转向 / 方位角 P */
    mf_pid_t *pid_point_v;  /* 点到点前向速度 P */
} mf_path_ctx_t;

void mf_path_init(mf_path_ctx_t *p, mf_pid_t *pid_yaw, mf_pid_t *pid_point_v);

/** @brief 开始执行一条路径（须以 MF_SEG_END 结尾） */
void mf_path_start(mf_path_ctx_t *p, const mf_seg_t *path);

/** @brief 停止并回到 IDLE */
void mf_path_stop(mf_path_ctx_t *p);

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

#endif /* MF_PATH_H */
