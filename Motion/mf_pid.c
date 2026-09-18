#include "mf_pid.h"

void mf_pid_init(mf_pid_t *pid, float kp, float ki, float kd,
                 float i_max, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_max = i_max;
    pid->out_max = out_max;
    mf_pid_reset(pid);
}

void mf_pid_reset(mf_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->last_err = 0.0f;
    pid->first = 1;
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
    float unsat, out, sat_hi, sat_lo;

    if (pid->first)
    {
        pid->last_err = err;    /* 首拍不做微分踢 */
        pid->first = 0;
    }

    unsat = pid->kp * err
          + pid->ki * pid->integral
          + pid->kd * (err - pid->last_err) / dt;

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
    pid->last_err = err;

    return out;
}
