/**
 * @file    mf_odom.h
 * @brief   里程计：把各轮位移增量融合成底盘位姿（世界系 x,y,yaw）
 *
 * 底盘差异由编译期宏选择：
 *   差速/履带：ds=(dL+dR)/2，dyaw 用 IMU 或 (dR-dL)/W_eff（履带 W_eff 需打滑标定）
 *   麦轮     ：标准 X-构型四轮正运动学，可估计横向位移
 * 所有底盘均做中点积分（以本周期中间航向旋转）。
 */
#ifndef MF_ODOM_H
#define MF_ODOM_H

#include "mf_types.h"

typedef struct
{
    mf_pose_t pose;         /* 融合输出位姿 */
    float track_width_m;    /* 差速/履带：等效轨距（左右轮距）；麦轮不用 */
    float half_length_m;    /* 麦轮几何参数：半车长 */
    float half_width_m;     /* 麦轮几何参数：半车宽 */
} mf_odom_t;

void mf_odom_init(mf_odom_t *o, float track_width_m,
                  float half_length_m, float half_width_m);

/**
 * @brief 里程计更新
 * @param o        里程计对象
 * @param wheel    各轮状态（本函数读取 delta_m 增量）
 * @param imu_yaw  外部航向（deg，多圈）；MF_YAW_SOURCE 为 ENCODER 时忽略
 */
void mf_odom_update(mf_odom_t *o, const mf_wheel_t *wheel, float imu_yaw);

/** @brief 重定位（比赛起点标定 / 到达地标后校准漂移） */
void mf_odom_set_pose(mf_odom_t *o, float x, float y, float yaw);

#endif /* MF_ODOM_H */
