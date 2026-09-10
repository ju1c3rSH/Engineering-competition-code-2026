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
    float out;

    if (pid->first)
    {
        pid->last_err = err;    /* 首拍不做微分踢 */
        pid->first = 0;
    }

    pid->integral += err * dt;
    pid->integral = mf_clamp(pid->integral, pid->i_max);

    out = pid->kp * err
        + pid->ki * pid->integral
        + pid->kd * (err - pid->last_err) / dt;
    pid->last_err = err;

    return mf_clamp(out, pid->out_max);
}
