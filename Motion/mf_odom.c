#include "mf_odom.h"
#include "mf_math.h"
#include <math.h>

void mf_odom_init(mf_odom_t *o, float track_width_m,
                  float half_length_m, float half_width_m)
{
    o->pose.x_m = 0.0f;
    o->pose.y_m = 0.0f;
    o->pose.yaw_deg = 0.0f;
    o->track_width_m = track_width_m;
    o->half_length_m = half_length_m;
    o->half_width_m = half_width_m;
    o->last_ds_for_xy = 0.0f;
}

void mf_odom_set_pose(mf_odom_t *o, float x, float y, float yaw)
{
    o->pose.x_m = x;
    o->pose.y_m = y;
    o->pose.yaw_deg = yaw;
}

#if (MF_CHASSIS_TYPE == MF_CHASSIS_DIFF || MF_CHASSIS_TYPE == MF_CHASSIS_TRACK)

void mf_odom_update(mf_odom_t *o, const mf_wheel_t *wheel, float imu_yaw)
{
    float dL, dR, ds_enc, ds_for_xy;
    float dtheta_enc_rad, dyaw_deg, dyaw_rad;
    float yaw_mid, c, s;

    dL = wheel[0].delta_m;
    dR = wheel[1].delta_m;
    ds_enc = (dL + dR) * 0.5f;

    /* 编码器几何转角（rad）：对称打滑下它与真实转角只差一个公共系数 k，
     * 而 ds_enc 同样被 k 放大，所以比值 R = ds_enc / dtheta_enc 不受 k 影响 */
    if (o->track_width_m > 0.0f)
    {
        dtheta_enc_rad = (dR - dL) / o->track_width_m;
    }
    else
    {
        dtheta_enc_rad = 0.0f;
    }

#if (MF_YAW_SOURCE == MF_YAW_SRC_IMU)
    dyaw_deg = mf_wrap_deg(imu_yaw - o->pose.yaw_deg);
#else
    (void)imu_yaw;
    dyaw_deg = dtheta_enc_rad * (180.0f / MF_PI_F);
#endif
    dyaw_rad = dyaw_deg * (MF_PI_F / 180.0f);

    /* 弧长反解：
     *   ds_true = R_enc × |dyaw_imu|
     *           = ds_enc × (dyaw_rad / dtheta_enc_rad)
     * 只在**真转弯**时可信 —— 见 MF_ODOM_RATIO_MIN_*。直线段航向纠偏的 wz 仅几 dps:
     * dyaw 小到与 IMU 读数分辨率同级, 而 dtheta_enc 是编码器小量 ⇒ 比值退化成
     * "噪声除小量", 撞 2.0 限幅那一拍里程凭空翻倍（2026-09-29 实测, 长腿 sd 104mm）。
     * 不可退回纯 ds_enc：履带转弯滑移使 (dL+dR)/2 系统性偏大于真实车体位移,
     * 弧线越多 x,y 累计漂移越大——本段是修 x/y 累计里程的唯一手段。
     * 本值只喂 pose.x/y; 路径段进度仍用 mf_motion.c 的轮速均值(要求平稳不抖),
     * 两者不可互换。基准 track_width_m 偏大会等比放大转弯里程(见 mf_app_init) */
    if (fabsf(dyaw_deg) < MF_ODOM_RATIO_MIN_DYAW_DEG)
    {
        ds_for_xy = ds_enc;                              /* 直线, 或纠偏量太小不足以反解 */
    }
    else if (fabsf(dtheta_enc_rad) >= MF_ODOM_RATIO_MIN_DTHETA_RAD)
    {
        float ratio = dyaw_rad / dtheta_enc_rad;
        /* 限幅：单侧空转/单边堵转时 ratio 会异常，别让 x/y 单拍跳变 */
        if (ratio < 0.5f) ratio = 0.5f;
        if (ratio > 2.0f) ratio = 2.0f;
        ds_for_xy = ds_enc * ratio;
    }
    else
    {
        ds_for_xy = ds_enc;                              /* 转角太小/编码器看不见 → 不用反解 */
    }

    /* 中点积分：用真实航向的中间值旋转本拍位移 */
    yaw_mid = o->pose.yaw_deg + dyaw_deg * 0.5f;
    c = cosf(yaw_mid * (MF_PI_F / 180.0f));
    s = sinf(yaw_mid * (MF_PI_F / 180.0f));

    o->pose.x_m += ds_for_xy * c;
    o->pose.y_m += ds_for_xy * s;
    o->pose.yaw_deg += dyaw_deg;
    o->last_ds_for_xy = ds_for_xy;
}

#elif (MF_CHASSIS_TYPE == MF_CHASSIS_MECANUM)

void mf_odom_update(mf_odom_t *o, const mf_wheel_t *wheel, float imu_yaw)
{
    float ds_fl, ds_fr, ds_rl, ds_rr;
    float dx_b, dy_b, dyaw, yaw_mid, c, s, k;

    /* 轮序与 HAL 约定一致：0=FL 1=FR 2=RL 3=RR（均沿各自前进方向为正） */
    ds_fl = wheel[0].delta_m;
    ds_fr = wheel[1].delta_m;
    ds_rl = wheel[2].delta_m;
    ds_rr = wheel[3].delta_m;

    dx_b = ( ds_fl + ds_fr + ds_rl + ds_rr) * 0.25f;
    dy_b = (-ds_fl + ds_fr - ds_rl + ds_rr) * 0.25f;

#if (MF_YAW_SOURCE == MF_YAW_SRC_IMU)
    (void)k;
    dyaw = mf_wrap_deg(imu_yaw - o->pose.yaw_deg);
#else
    k = o->half_length_m + o->half_width_m;
    if (k > 0.0f)
    {
        dyaw = (-ds_fl + ds_fr + ds_rl - ds_rr) / (2.0f * k)
             * (180.0f / MF_PI_F);
    }
    else
    {
        dyaw = 0.0f;
    }
#endif

    yaw_mid = o->pose.yaw_deg + dyaw * 0.5f;
    c = cosf(yaw_mid * (MF_PI_F / 180.0f));
    s = sinf(yaw_mid * (MF_PI_F / 180.0f));

    o->pose.x_m += dx_b * c - dy_b * s;
    o->pose.y_m += dx_b * s + dy_b * c;
    o->pose.yaw_deg += dyaw;
    o->last_ds_for_xy = dx_b;
}

#endif /* MF_CHASSIS_TYPE */
