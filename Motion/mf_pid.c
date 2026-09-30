#include "mf_pid.h"

void mf_pid_init(mf_pid_t *pid, float kp, float ki, float kd,
                 float i_max, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_max = i_max;
    pid->out_max = out_max;
    pid->d_win = 1u;    /* 采样层下发前用逐拍差分: 不假设任何帧率 */
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

    /* 微分项: 跨 w 拍差分, w 由采样层按**实测帧率**下发(见 mf_pid.h)。
     * (err − e_hist[w−1])/(w·dt): 跨满一个"整数帧数交替周期"就把该根谱线杀掉;
     * 慢变真值几乎原样透过(相位代价约半个窗 = w·dt/2)。
     * hist_fill < w 时给 0 —— 这同时充当"首拍不起跳踢"(旧 first 标志的职责) */
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

    /* 条件积分抗饱和: 输出已饱和时只在误差把输出往带内拉的方向退/进积分;
     * 误差同向(继续顶饱和)则冻结 —— 巡航钳位卡死时单侧差动
     * "只降不升"的不对称响应即源于此 */
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
