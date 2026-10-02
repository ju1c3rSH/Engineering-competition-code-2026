/**
 * @file    mf_pid.h
 * @brief   位置式 PID（带积分限幅/输出限幅），供航向保持、点到点、轮速环共用
 */
#ifndef MF_PID_H
#define MF_PID_H

#include <stdint.h>
#include "mf_config.h"      /* MF_YAW_DIFF_WIN_MAX: 差分窗口上限, 与采样层的实测窗口同源 */

/* 微分项的差分窗口(拍)是**运行期变量**, 但**当前没有任何人下发** —— mf_pid_init 置 1,
 * 采样层没写 snap.yaw_diff_win, mf_tick 也没拷进来 ⇒ 实际恒为逐拍差分。
 * 这在现场是对的: 模块 200Hz ÷ 控制拍 200Hz = 1/1, 每拍整 1 帧、无交替 ⇒ 逐拍差分即真值。
 *
 * 只有帧率不是拍率整数倍时才需要跨窗: 那时一拍的整帧数会按周期交替(250÷100 = 5:2 ⇒ 2/3 帧),
 * 把交替当转速变化注入伪影, 而微分增益 kd/dt 会把它放大;
 * 跨满一个交替周期即在该谱线上放零点(窗口 = 每拍帧数的分母), 代价是半个窗的相位。
 * ⚠ kd **不随 dt 缩放**: 误差恒斜率变化时 Δerr 与 dt 同比例缩, 与分母相消 ⇒ 稳态阻尼不变。
 *   改拍周期时误降 kd 会白白腰斩航向环阻尼(而它的唯一作用正是抑制直线蛇形)。§8.13
 * ⚠ 启用前先确认真帧率: `HWT:RRATE` 数的是突发不是帧(见 §8.11)。§8.6 / §8.11 */
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
