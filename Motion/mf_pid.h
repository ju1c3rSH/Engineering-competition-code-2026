/**
 * @file    mf_pid.h
 * @brief   位置式 PID（带积分限幅/输出限幅），供航向保持、点到点、轮速环共用
 */
#ifndef MF_PID_H
#define MF_PID_H

#include <stdint.h>

/* 微分项的差分步长(拍)。1 = 逐位等于旧行为(单拍差分, 供 A/B); 2 = 两拍滑动平均差分。
 *
 * 为什么必须是 2(2026-09-30 实测定量): 微分增益是 kd/dt —— 航向环 kd=0.8、dt=10ms
 * ⇒ **80 dps / 每拍误差变化 1°**。而 HWT101 的航向是 **250Hz 的零阶保持值**, 控制拍只有
 * 100Hz ⇒ 250:100 = 5:2, 每拍恰好摊到 2.5 帧, 帧数按 2、3 交替。于是"每拍航向增量"在
 * 0.344° 与 0.516° 之间交替 —— 真值 43dps 时, 微分项被注入 ±14dps 的 **50Hz** 噪声,
 * 占指令的 1/3。两趟独立日志实拍: SP/W/O 三列都以**恰好 2 个采样**为周期反相摆动
 * (SP 均值 42 摆 ±8、O 均值 109 摆 ±24), 而**实测轮速 SL/SR 是平的**(±2.5%)——
 * 轮速环 1.5Hz 根本跟不上 50Hz, 所以摆动是控制器注入的, 不是车体真在抖。
 *
 * 取 2 的理由是它**精确**而不是近似: (e[k]-e[k-2])/(2dt) 的传递函数在 1/(2·dt)=50Hz
 * 处有**零点**, 交替的 2/3 帧正好被平均掉; 而真值(慢变)完全透过, 相位代价只有约半拍
 * (5ms, 2Hz 处 4°)。一阶低通做不到"只杀这一根谱线不伤相位"。
 * ⚠ 前提是 0x53 帧率 ≈ 控制拍率的整数倍半(2.5×/1.5×…); 若以后把模块配成 200Hz
 * (2.0×, 每拍整 2 帧)则噪声本就消失, 本项退回 1 也等价 */
#ifndef MF_PID_D_AVG_TICKS
#define MF_PID_D_AVG_TICKS  2
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float i_max;            /* 积分限幅，0=不限 */
    float out_max;          /* 输出限幅，0=不限 */
    float last_err;
    float e1, e2;           /* 上两拍误差(滑窗差分的分子端) */
    uint8_t first;
    uint8_t filled;         /* 滑窗已积累样本数(0..2); <2 时微分给 0 */
} mf_pid_t;

void mf_pid_init(mf_pid_t *pid, float kp, float ki, float kd,
                 float i_max, float out_max);
void mf_pid_reset(mf_pid_t *pid);
float mf_pid_calc(mf_pid_t *pid, float err, float dt);

#endif /* MF_PID_H */
