#include "mf_kinematics.h"
#include "mf_math.h"
#include "mf_config.h"
#include <math.h>

#if (MF_CHASSIS_TYPE == MF_CHASSIS_DIFF || MF_CHASSIS_TYPE == MF_CHASSIS_TRACK)

void mf_inverse_kinematics(const mf_body_cmd_t *cmd, float track_w,
                           float half_len, float half_wid,
                           float *wheel_out)
{
    float wz_rad = cmd->wz_dps * (MF_PI_F / 180.0f);

    (void)half_len;
    (void)half_wid;

#if MF_DIFF_BOOST_EN
    /* 差速前馈补偿: 履带转弯时内侧刹不住/外侧跟不上, 实际差速只有指令的
     * 0.74~0.89(右转更差), 导致实际半径大于名义 -> 弧线往外飘(ARC 出口 EY 恒正)。
     * 按转向方向放大差速指令, 让实际差速回到目标。系数是实车标定项,
     * 现场按 ARC:<段> EY=<mm> 微调, 详见 mf_config.h 的说明。 */
    wz_rad *= (cmd->wz_dps >= 0.0f) ? MF_DIFF_BOOST_POS : MF_DIFF_BOOST_NEG;
#endif

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
