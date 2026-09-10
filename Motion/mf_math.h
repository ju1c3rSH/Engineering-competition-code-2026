/**
 * @file    mf_math.h
 * @brief   utils：角度规整、限幅、斜坡
 */
#ifndef MF_MATH_H
#define MF_MATH_H

#define MF_PI_F 3.14159265358979323846f

static inline float mf_wrap_deg(float a)
{
    while (a >= 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static inline float mf_clampf(float v, float lim)
{
    if (v > lim)  return lim;
    if (v < -lim) return -lim;
    return v;
}

/* 斜坡逼近：从 cur 向 target 以 rate/s 变化（rate<=0 直达） */
static inline float mf_rampf(float cur, float target, float rate, float dt)
{
    float step;
    if (rate <= 0.0f) return target;
    step = rate * dt;
    if (target > cur + step) return cur + step;
    if (target < cur - step) return cur - step;
    return target;
}

#endif /* MF_MATH_H */
