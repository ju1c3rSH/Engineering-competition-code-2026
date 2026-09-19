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

    /* 回退纯编码器 */
    ds_for_xy = ds_enc;

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
