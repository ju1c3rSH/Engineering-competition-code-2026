/**
 * @file    mf_rate.h
 * @brief   角速度内环：外环算出的 wz 只当设定值，对 IMU 实测转速闭环
 *
 * panzer 履带转弯的实测角速度只有指令的 k 倍（k≈0.6，差速指令要乘 1/k 才跟得上），
 * 外环（航向/横向）看不见这个亏损，以前只由 MF_DIFF_BOOST 用静态系数硬猜。本环把实测
 * 角速度接进来做成**级联内环**：
 *
 *     wz_sp(外环) --(+)--> 开环基线 gain0·wz_sp + PI --> 逆解 --| k |--> ω_meas --+
 *                   ^(-)                                                         |
 *                   +------------------------------------------------------------+
 *
 * gain0 = 本段的开环滑移基线（mf_path 的 slip_gain，只有弧线 ≠1，见 MF_ARC_SLIP_GAIN）。
 * 它**不是**"与 PI 二选一"的另一套补偿：基线越准，PI 的积分越接近 0，两者相加恒等于
 * 所需的那一份补偿 —— 这一点是 2026-09-30 改出来的，旧版把基线写成
 * `#if (MF_DIFF_BOOST_EN && !MF_WZ_RATE_EN)` 的编译期互斥，于是反馈一失效补偿就**归零**，
 * 实测弧半径 0.94m→1.63m（×1.73 = 1/k）、弧耗时 2.1s→3.6s。
 */
#ifndef MF_RATE_H
#define MF_RATE_H

#include "mf_config.h"
#include "mf_math.h"
#include "mf_pid.h"

typedef struct
{
    mf_pid_t pid;           /* 作用在(wz_sp - ω_meas)上的 PI；积分吃掉基线的残差 */
    float    lpf_alpha;     /* 实测角速度一阶低通系数 */
    float    meas_f;        /* 低通后的实测角速度 deg/s */
    float    out_dps;       /* 上一拍下发值 deg/s（反号自检的参照 + 诊断） */
    float    sp_dps;        /* 上一拍设定值 deg/s（诊断） */
    float    base_dps;      /* 上一拍开环基线 = sp_dps×gain0（诊断：O = B + P + I） */
    float    meas_dps;      /* 上一拍原始实测值 deg/s（诊断） */
    uint32_t n_ok;          /* 闭环拍数；与下面两个互斥，三者之和 = 上电以来走过拍数 */
    uint32_t n_stale;       /* 反馈停更（上层判）-> 退基线 */
    uint32_t n_sign;        /* 反馈与上拍下发反号 -> 退基线 */
    uint8_t  primed;        /* 首拍用原始实测值播种，免滤波器从 0 爬升 */
    uint8_t  armed;         /* 已跑过至少一拍：out_dps 才有资格参与反号自检 */
} mf_rate_t;

/* 内环快照（诊断：上层主循环读一次就够，见 App/car_obs.c 的 RATE: 行） */
typedef struct
{
    float    sp_dps;
    float    meas_dps;
    float    out_dps;
    float    base_dps;           /* 开环基线贡献 = sp×gain0, deg/s */
    float    i_term_dps;         /* 积分项贡献 = ki·∫e, deg/s（基线与积分是两半, 不是两套） */
    uint32_t n_ok, n_stale, n_sign;
} mf_rate_info_t;

static inline void mf_rate_init(mf_rate_t *r, float kp, float ki,
                                float i_max, float out_max, float lpf_hz)
{
    float tau = (lpf_hz > 0.0f) ? (1.0f / (2.0f * MF_PI_F * lpf_hz)) : 0.0f;

    mf_pid_init(&r->pid, kp, ki, 0.0f, i_max, out_max);
    r->lpf_alpha = (tau > 0.0f) ? (MF_DT_S / (tau + MF_DT_S)) : 1.0f;
    r->meas_f    = 0.0f;
    r->out_dps   = 0.0f;
    r->sp_dps    = 0.0f;
    r->meas_dps  = 0.0f;
    r->base_dps  = 0.0f;
    r->n_ok      = 0u;
    r->n_stale   = 0u;
    r->n_sign    = 0u;
    r->primed    = 0u;
    r->armed     = 0u;
}

/** @brief 单拍闭环：wz_sp=外环设定(deg/s)，wz_meas=IMU 实测(deg/s)，fb_valid=反馈是否新鲜，
 *         gain0=本段开环滑移基线（弧线≠1，其余 1.0） -> 下发角速度 */
static inline float mf_rate_apply(mf_rate_t *r, float wz_sp, float wz_meas,
                                  uint8_t fb_valid, float gain0)
{
    float base = wz_sp * gain0;     /* 前馈：该弯道"指令亏损 k"的标定反补 */
    float out;

    r->sp_dps   = wz_sp;
    r->meas_dps = wz_meas;
    r->base_dps = base;

    if (fb_valid == 0u)
    {
        /* 反馈停更：冻结 PI（保留已学到的基线残差）、丢掉 kp 项。绝不可退成纯 wz_sp ——
         * 那是补偿归零，弧半径 ×1/k。也不可按 wz_meas=0 闭环：误差恒等于设定值，
         * kp·wz_sp + ki·I 足以把输出顶到 out_max（实算 57dps 命令撞 150dps） */
        r->n_stale++;
        r->primed = 0u;
        out = base + r->pid.ki * r->pid.integral;
    }
    else if ((r->armed != 0u) &&
             (mf_absf(r->out_dps) >= MF_WZ_RATE_SIGN_MIN_DPS) &&
             (mf_absf(wz_meas) >= MF_WZ_RATE_SIGN_MIN_DPS) &&
             ((r->out_dps * wz_meas) < 0.0f))
    {
        /* 打滑只改变转速幅值、不改变转向 ⇒ 实测与上一拍下发反号只可能来自反馈源异常
         * (角速度帧偏移/符号错)或单侧空转。此时丢掉反馈按基线走, 否则 PI 会把
         * 反向误差当亏损一路补成正反馈。armed: 刚 init/reset 过时 out_dps=0, 不参判。
         * ⚠ 两侧都要过 MF_WZ_RATE_SIGN_MIN_DPS 死区: 判号用的是**未滤波原始值**, 近零
         * 时它就是量化噪声, 无死区会每拍误判一次(实测 SN=1399, 占 32%), 而每次误判都
         * 清 primed + 跳过 mf_pid_calc ⇒ 低通不滤波、积分不累积, PI 形同虚设 */
        r->n_sign++;
        r->primed = 0u;
        out = base;
    }
    else
    {
        if (r->primed == 0u)
        {
            r->meas_f = wz_meas;
            r->primed = 1u;
        }
        else
        {
            r->meas_f += (wz_meas - r->meas_f) * r->lpf_alpha;
        }
        r->n_ok++;
        /* 设定值已顶到 out_max 时本环没有补偿余量, 这是物理上限, 不再向上放开 */
        out = base + mf_pid_calc(&r->pid, wz_sp - r->meas_f, MF_DT_S);
    }

    out = mf_clampf(out, r->pid.out_max);
    r->out_dps = out;
    r->armed = 1u;
    return out;
}

/* 停机/上层硬置零时清积分与滤波器，免下次起步用陈旧转速。
 * 刻意**不清 n_***：计数是"上电以来"的，每次停车都清就看不出反馈是否长期不新鲜 */
static inline void mf_rate_reset(mf_rate_t *r)
{
    mf_pid_reset(&r->pid);
    r->meas_f  = 0.0f;
    r->out_dps = 0.0f;
    r->primed  = 0u;
    r->armed   = 0u;
}

#endif /* MF_RATE_H */
