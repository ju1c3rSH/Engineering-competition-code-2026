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
    pid->e1 = 0.0f;
    pid->e2 = 0.0f;
    pid->first = 1;
    pid->filled = 0u;
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

    if (pid->first)
    {
        pid->last_err = err;    /* 首拍不做微分踢 */
        pid->first = 0;
    }

    /* 微分项: 见 mf_pid.h 的 MF_PID_D_AVG_TICKS —— 默认 2 拍滑窗差分, 在 50Hz 有零点,
     * 精确抵消"250Hz 航向帧 ÷ 100Hz 控制拍"造成的每拍 2/3 帧交替。1 = 旧行为(逐位相同) */
#if (MF_PID_D_AVG_TICKS >= 2)
    if (pid->filled >= 2u)
    {
        d_raw = (err - pid->e2) / (2.0f * dt);
    }
    else
    {
        d_raw = 0.0f;
        pid->filled++;
    }
    pid->e2 = pid->e1;
    pid->e1 = err;
#else
    d_raw = (err - pid->last_err) / dt;   /* 滑窗字段在本模式下只被 reset 写过一次 */
#endif

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
    pid->last_err = err;

    return out;
}
