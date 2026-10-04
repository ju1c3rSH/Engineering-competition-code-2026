/**
 * @file    mf_pid.h
 * @brief   位置式 PID（带积分限幅/输出限幅），供航向保持、点到点、轮速环共用
 */
#ifndef MF_PID_H
#define MF_PID_H

#include <stdint.h>
#include "mf_config.h"      /* MF_YAW_DIFF_WIN_MAX: 差分窗口上限, 与采样层的实测窗口同源 */

/* 微分差分窗口(拍)是运行期变量, 但当前无人下发 ⇒ mf_pid_init 置 1、实际恒为逐拍差分; 现场 200Hz÷200Hz=1:1 逐拍整帧正确。
 * 仅帧率非拍率整数倍时(一拍整帧数周期交替)才需跨窗消除伪影, 代价半个窗相位。
 * kd **不随 dt 缩放**: 否则改拍周期误降 kd 会腰斩航向环阻尼(唯一作用是抑直线蛇形)。§8.13
 * 启用前确认真帧率: HWT:RRATE 数的是突发非帧(§8.11)。§8.6/§8.11 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float i_max;            /* 积分限幅，0=不限 */
    float out_max;          /* 输出限幅，0=不限 */
    float e_hist[MF_YAW_DIFF_WIN_MAX + 1];  /* err 历史: [0]=上一拍, [k]=k 拍前 */
    uint8_t d_win;          /* 微分差分跨几拍(运行期下发); 1=逐拍差分 */
    uint8_t hist_fill;      /* 历史已积累样本数; < d_win 时微分给 0(不做起跳踢) */
} mf_pid_t;

void mf_pid_init(mf_pid_t *pid, float kp, float ki, float kd,
                 float i_max, float out_max);
void mf_pid_reset(mf_pid_t *pid);
float mf_pid_calc(mf_pid_t *pid, float err, float dt);

#endif /* MF_PID_H */
