#include "mf_pid.h"

void mf_pid_init(mf_pid_t *pid, float kp, float ki, float kd,
                 float i_max, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_max = i_max;
    pid->out_max = out_max;
    pid->d_win = 1u;    /* 采样层未下发 ⇒ 恒逐拍差分, 见 mf_pid.h */
    mf_pid_reset(pid);
}

void mf_pid_reset(mf_pid_t *pid)
{
    uint8_t i;

    pid->integral = 0.0f;
    for (i = 0u; i <= MF_YAW_DIFF_WIN_MAX; ++i)
    {
        pid->e_hist[i] = 0.0f;
    }
    pid->hist_fill = 0u;
    /* d_win 不复位: 它是配置(和 kp/kd 同类), 不是状态 */
}

static float mf_clamp(float v, float lim)
{
    if (lim > 0.0f)
    {
        if (v > lim)  return lim;
        if (v < -lim) return -lim;
    }
    return v;
}

float mf_pid_calc(mf_pid_t *pid, float err, float dt)
{
    float d_raw;
    float unsat, out;
    uint8_t w, i;

    /* 微分项: 跨 w 拍差分 (err − e_hist[w−1])/(w·dt) —— 跨满一个整数帧交替周期即杀该根谱线,
     * 慢变真值几乎原样透过(相位代价约半个窗 = w·dt/2); hist_fill<w 时给 0, 兼作首拍不起跳踢 */
    w = pid->d_win;
    if (w < 1u)
    {
        w = 1u;
    }
    if (w > MF_YAW_DIFF_WIN_MAX)
    {
        w = MF_YAW_DIFF_WIN_MAX;
    }

    if (pid->hist_fill >= w)
    {
        d_raw = (err - pid->e_hist[w - 1u]) / ((float)w * dt);
    }
    else
    {
        d_raw = 0.0f;
        pid->hist_fill++;
    }
    for (i = MF_YAW_DIFF_WIN_MAX; i > 0u; --i)
    {
        pid->e_hist[i] = pid->e_hist[i - 1u];
    }
    pid->e_hist[0] = err;

    unsat = pid->kp * err
          + pid->ki * pid->integral
          + pid->kd * d_raw;

    /* 条件积分抗饱和: 输出饱和时只在误差往带内拉的方向进积分, 同向(继续顶饱和)冻结
     * —— 巡航钳位卡死时"只降不升"的不对称响应即源于此 */
    if (unsat > pid->out_max)
    {
        out = pid->out_max;
        if (err < 0.0f)
        {
            pid->integral += err * dt;
        }
    }
    else if (unsat < -pid->out_max)
    {
        out = -pid->out_max;
        if (err > 0.0f)
        {
            pid->integral += err * dt;
        }
    }
    else
    {
        pid->integral += err * dt;
        out = unsat;
    }
    pid->integral = mf_clamp(pid->integral, pid->i_max);

    return out;
}
