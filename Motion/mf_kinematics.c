#include "mf_kinematics.h"
#include "mf_math.h"
#include <math.h>

#if (MF_CHASSIS_TYPE == MF_CHASSIS_DIFF || MF_CHASSIS_TYPE == MF_CHASSIS_TRACK)

void mf_inverse_kinematics(const mf_body_cmd_t *cmd, float track_w,
                           float half_len, float half_wid,
                           float *wheel_out)
{
    float wz_rad = cmd->wz_dps * (MF_PI_F / 180.0f);

    (void)half_len;
    (void)half_wid;

    wheel_out[0] = cmd->vx_mps - wz_rad * track_w * 0.5f;   /* 左轮 */
    wheel_out[1] = cmd->vx_mps + wz_rad * track_w * 0.5f;   /* 右轮 */
}

#elif (MF_CHASSIS_TYPE == MF_CHASSIS_MECANUM)

void mf_inverse_kinematics(const mf_body_cmd_t *cmd, float track_w,
                           float half_len, float half_wid,
                           float *wheel_out)
{
    float wz_rad = cmd->wz_dps * (MF_PI_F / 180.0f);
    float k = half_len + half_wid;

    (void)track_w;

    wheel_out[0] = cmd->vx_mps - cmd->vy_mps - wz_rad * k;  /* FL */
    wheel_out[1] = cmd->vx_mps + cmd->vy_mps + wz_rad * k;  /* FR */
    wheel_out[2] = cmd->vx_mps + cmd->vy_mps - wz_rad * k;  /* RL */
    wheel_out[3] = cmd->vx_mps - cmd->vy_mps + wz_rad * k;  /* RR */
}

#endif /* MF_CHASSIS_TYPE */
