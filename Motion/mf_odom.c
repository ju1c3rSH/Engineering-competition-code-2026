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
    o->arc_src = (uint8_t)MF_ODOM_ARC_SRC;   /* mf_init 可被 mf_cfg_t 覆盖 */
    o->reff.r_m = 0.0f;
    o->reff.ds_acc_m = 0.0f;
    o->reff.dyaw_acc_deg = 0.0f;
    o->reff.last_yaw_deg = 0.0f;
    o->reff.last_seq = 0u;
    o->reff.stale = (uint16_t)MF_REFF_STALE_TICKS;
    o->reff.seeded = 0u;
    o->reff.valid = 0u;
}

/* 弧实测半径(观测口径): 见 mf_odom.h。三道保护都是"丢窗重来", 绝不相除不同性质的量:
 * 航向阶跃(单拍闸)、窗口转角越界、攒够 MIN_DYAW 前比值已不可能落在弧半径量程(直线/跨段残窗) */
static void mf_odom_reff_update(mf_odom_t *o, float ds_enc,
                                float imu_yaw, float dtheta_enc_rad,
                                uint32_t yaw_seq)
{
    mf_reff_t *r = &o->reff;
    float est;
    uint8_t drop = 0u;
    uint8_t settled = 0u;

    if (r->seeded == 0u)                 /* 首拍没有基线: 丢弃本拍位移, 只钉起点 */
    {
        r->seeded = 1u;
        r->last_yaw_deg = imu_yaw;
        r->last_seq = yaw_seq;
        r->stale = (uint16_t)MF_REFF_STALE_TICKS;
        return;
    }

    r->ds_acc_m += ds_enc;

#if (MF_YAW_SOURCE == MF_YAW_SRC_IMU)
    if (yaw_seq != r->last_seq)          /* 只在新航向样上推进"分母"的终点 */
    {
        float dyaw = mf_wrap_deg(imu_yaw - r->last_yaw_deg);

        r->last_seq = yaw_seq;
        r->last_yaw_deg = imu_yaw;
        if ((dyaw <= -MF_REFF_STEP_MAX_DEG) || (dyaw >= MF_REFF_STEP_MAX_DEG))
        {
            drop = 1u;                   /* 航向阶跃不是转速(归零/假同步) */
        }
        else
        {
            r->dyaw_acc_deg += dyaw;
        }
    }
#else
    (void)yaw_seq;
    r->dyaw_acc_deg += dtheta_enc_rad * (180.0f / MF_PI_F);   /* 无 IMU: 只有编码器几何转角, 信息量为零 */
#endif

    if ((drop != 0u) || (mf_absf(r->dyaw_acc_deg) >= MF_REFF_MAX_DYAW_DEG))
    {
        r->ds_acc_m = 0.0f;
        r->dyaw_acc_deg = 0.0f;
    }
    else if (mf_absf(r->dyaw_acc_deg) >= MF_REFF_MIN_DYAW_DEG)
    {
        est = r->ds_acc_m / (mf_absf(r->dyaw_acc_deg) * (MF_PI_F / 180.0f));
        if ((est >= MF_REFF_MIN_M) && (est <= MF_REFF_MAX_M))
        {
            r->r_m = (r->r_m == 0.0f) ? est
                                      : (r->r_m + (est - r->r_m) * MF_REFF_LPF_ALPHA);
            settled = 1u;
        }
        r->ds_acc_m = 0.0f;              /* 跨阈值即换窗(不论这个值收不收) */
        r->dyaw_acc_deg = 0.0f;
    }
    else if (mf_absf(r->ds_acc_m)
             > (mf_absf(r->dyaw_acc_deg) * (MF_PI_F / 180.0f) * MF_REFF_IMPLAUSIBLE_M))
    {
        r->ds_acc_m = 0.0f;              /* 直线/跨段残窗: 现在就丢, 不留到跨阈值 */
        r->dyaw_acc_deg = 0.0f;
    }

    if (settled != 0u)
    {
        r->stale = 0u;
    }
    else if (r->stale < 0xFFFFu)
    {
        r->stale++;                      /* 语义 = 距上次**成功结算**的拍数, 失败也要涨 */
    }

    r->valid = (uint8_t)(((r->r_m > 0.0f)
                          && (r->stale < (uint16_t)MF_REFF_STALE_TICKS)) ? 1u : 0u);
}

float mf_odom_get_reff(const mf_odom_t *o, uint8_t *valid)
{
    if (o == 0)
    {
        return 0.0f;
    }
    if (valid != 0)
    {
        *valid = o->reff.valid;
    }
    return o->reff.r_m;
}

uint8_t mf_odom_get_arc_src(const mf_odom_t *o)
{
    return (o != 0) ? o->arc_src : (uint8_t)MF_ODOM_ARC_SRC;
}

void mf_odom_set_pose(mf_odom_t *o, float x, float y, float yaw)
{
    o->pose.x_m = x;
    o->pose.y_m = y;
    o->pose.yaw_deg = yaw;
}

#if (MF_CHASSIS_TYPE == MF_CHASSIS_DIFF || MF_CHASSIS_TYPE == MF_CHASSIS_TRACK)

void mf_odom_update(mf_odom_t *o, const mf_wheel_t *wheel, float imu_yaw,
                    uint32_t yaw_seq)
{
    float dL, dR, ds_enc, ds_for_xy;
    float dtheta_enc_rad, dyaw_deg, dyaw_rad;
    float yaw_mid, c, s;

    dL = wheel[0].delta_m;
    dR = wheel[1].delta_m;
    ds_enc = (dL + dR) * 0.5f;

    /* 编码器几何转角(rad): 对称打滑下与真实转角只差公共系数 k, 而 ds_enc 同被 k 放大
     * ⇒ 比值 R = ds_enc / dtheta_enc 不受 k 影响 */
    if (o->track_width_m > 0.0f)
    {
        dtheta_enc_rad = (dR - dL) / o->track_width_m;
    }
    else
    {
        dtheta_enc_rad = 0.0f;
    }

    /* 实测半径: 独立观测量, 与下面用哪条尺子算 x/y 无关(见 mf_odom.h) */
    mf_odom_reff_update(o, ds_enc, imu_yaw, dtheta_enc_rad, yaw_seq);

#if (MF_YAW_SOURCE == MF_YAW_SRC_IMU)
    dyaw_deg = mf_wrap_deg(imu_yaw - o->pose.yaw_deg);
#else
    (void)imu_yaw;
    dyaw_deg = dtheta_enc_rad * (180.0f / MF_PI_F);
#endif
    dyaw_rad = dyaw_deg * (MF_PI_F / 180.0f);

    /* 本拍用于 x/y 的位移：两种尺子的差别只在"转不转"上（见 mf_config.h 的 MF_ODOM_ARC_SRC） */
    ds_for_xy = ds_enc;
    if (o->arc_src == 0u)
    {
        /* 旧律弧长反解 ds_enc×(dyaw_imu/dtheta_enc): 只在真转弯时可信(门限见 MF_ODOM_RATIO_MIN_*);
         * 它使 R_odom ≡ ds_enc/dtheta_enc 只含轮子量 ⇒ 弧半径恒等于指令半径, ARC 的 e_y/e_psi 结构性失效(§8) */
        if ((fabsf(dyaw_deg) >= MF_ODOM_RATIO_MIN_DYAW_DEG)
            && (fabsf(dtheta_enc_rad) >= MF_ODOM_RATIO_MIN_DTHETA_RAD))
        {
            float ratio = dyaw_rad / dtheta_enc_rad;

            if (ratio < MF_ODOM_RATIO_CLAMP_LO) { ratio = MF_ODOM_RATIO_CLAMP_LO; }
            if (ratio > MF_ODOM_RATIO_CLAMP_HI) { ratio = MF_ODOM_RATIO_CLAMP_HI; }
            ds_for_xy = ds_enc * ratio;
        }
    }
    /* 本值只喂 pose.x/y; 路径段进度用 mf_motion.c 的轮速均值（要求平稳不抖），两者不可互换 */

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

void mf_odom_update(mf_odom_t *o, const mf_wheel_t *wheel, float imu_yaw,
                    uint32_t yaw_seq)
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

    k = o->half_length_m + o->half_width_m;
    mf_odom_reff_update(o, dx_b, imu_yaw,
                        (k > 0.0f) ? ((-ds_fl + ds_fr + ds_rl - ds_rr) / (2.0f * k)) : 0.0f,
                        yaw_seq);

#if (MF_YAW_SOURCE == MF_YAW_SRC_IMU)
    (void)k;
    dyaw = mf_wrap_deg(imu_yaw - o->pose.yaw_deg);
#else
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
