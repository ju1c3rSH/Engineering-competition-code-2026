/**
 * @file    mf_kinematics.h
 * @brief   逆运动学：底盘车体速度指令 (vx, vy, wz) -> 各轮线速度目标 (m/s)
 *
 *   差速/履带：vL = vx - wz*R/2, vR = vx + wz*R/2（R = 等效轨距）
 *   麦轮     ：标准 X-构型解算
 */
#ifndef MF_KINEMATICS_H
#define MF_KINEMATICS_H

#include "mf_types.h"

/**
 * @param cmd       车体速度指令
 * @param track_w   等效轨距 m（差速/履带用；麦轮填 half_len+half_wid 的 2 倍以下不用）
 * @param half_len  麦轮半车长（差速/履带不用）
 * @param half_wid  麦轮半车宽
 * @param wheel_out 输出：各轮线速度目标 m/s，数组长度 MF_WHEEL_COUNT
 */
void mf_inverse_kinematics(const mf_body_cmd_t *cmd, float track_w,
                           float half_len, float half_wid,
                           float *wheel_out);

#endif /* MF_KINEMATICS_H */
