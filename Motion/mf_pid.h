/**
 * @file    mf_pid.h
 * @brief   位置式 PID（带积分限幅/输出限幅），供航向保持、点到点、轮速环共用
 */
#ifndef MF_PID_H
#define MF_PID_H

#include <stdint.h>
#include "mf_config.h"      /* MF_YAW_DIFF_WIN_MAX: 差分窗口上限, 与采样层的实测窗口同源 */

/* 微分项的差分窗口(拍)是**运行期变量**, 不再有编译期常数。
 *
 * 正确窗口 = "每拍航向帧数"(化最简分数)的**分母**: HWT101 的航向是零阶保持值, 一拍的
 * 整数帧数只能按周期交替(250Hz÷100Hz = 5:2 ⇒ 每拍 2、3 帧交替), 交替周期 = 分母,
 * 差分跨满一个周期才在该根谱线上有零点 —— 这正是要的: 杀伪影、不伤慢变真值。
 * 微分增益是 kd/dt(航向环 kd=0.8 ⇒ 80 dps/每拍误差变化 1°), 伪影会被放大 80 倍, 必须杀。
 *
 * 为什么必须运行期: 帧率存在**模块自己的 Flash**、可被重配(实测 250, 固件写的 200)。写死
 * "2" 只在"恰好 5:2"那一刻成立, 换成 2:1 它就变成一个无意义还带滞后的滤波器。
 * 窗口由采样层(Car_Sense.c)每 1s 实测一次、随快照下发; 未下发时用 1 拍(逐拍差分)。
 * 实测与推导见 docs/坑-编码器与里程.md §8.6 */
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
