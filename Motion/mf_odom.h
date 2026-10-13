/**
 * @file    mf_odom.h
 * @brief   里程计：把各轮位移增量融合成底盘位姿（世界系 x,y,yaw）
 *
 * 底盘差异由编译期宏选择: 差速/履带 ds=(dL+dR)/2, dyaw 用 IMU 或 (dR-dL)/W_eff(履带需打滑标定);
 * 麦轮标准 X-构型四轮正运动学。所有底盘均中点积分(以本周期中间航向旋转)。
 */
#ifndef MF_ODOM_H
#define MF_ODOM_H

#include "mf_types.h"

/* 弧实测半径估计器（观测口径）：R_eff = Σ轮位移 / |ΣΔψ|。分子分母必须覆盖同一段时间
 * (航向是零阶保持量, 单拍相除会在帧率≠拍率时给出 0 与数倍跳变); 本值是弧几何唯一的
 * 非循环观测量 —— odom 形状里打滑因子被约掉, ARC 的 e_y/e_psi 看不见半径错(§8) */
typedef struct
{
    float    r_m;             /* 上次结算的实测半径(m, 恒正)；0 = 尚无 */
    float    ds_acc_m;        /* 当前结算窗口内累计的轮位移(带符号) */
    float    dyaw_acc_deg;    /* 同窗口内累计的航向转角(带符号) */
    float    last_yaw_deg;    /* 窗口起点航向 */
    uint32_t last_seq;        /* 窗口起点的航向帧计数 */
    uint16_t stale;           /* 距上次成功结算的拍数 */
    uint8_t  seeded;
    uint8_t  valid;           /* 当前是否有新鲜半径(stale 未超时) */
} mf_reff_t;

typedef struct
{
    mf_pose_t pose;         /* 融合输出位姿 */
    float track_width_m;    /* 差速/履带：等效轨距（左右轮距）；麦轮不用 */
    float half_length_m;    /* 麦轮几何参数：半车长 */
    float half_width_m;     /* 麦轮几何参数：半车宽 */
    float last_ds_for_xy;   /* 本拍反解后实际用于 x/y 的位移 */
    uint8_t arc_src;        /* 形状来源（MF_ODOM_ARC_SRC，可由 mf_cfg_t 覆盖）：0=弧长反解 1=轮速均值 */
    mf_reff_t reff;         /* 弧实测半径估计（独立观测量） */
} mf_odom_t;

void mf_odom_init(mf_odom_t *o, float track_width_m,
                  float half_length_m, float half_width_m);

/**
 * @brief 里程计更新
 * @param o        里程计对象
 * @param wheel    各轮状态（读 delta_m 增量）
 * @param imu_yaw  外部航向（deg，多圈）；MF_YAW_SOURCE 为 ENCODER 时忽略
 * @param yaw_seq  航向帧计数：判"本拍航向是否新样"，供 R_eff 同间隔配对；编码器源填 0
 */
void mf_odom_update(mf_odom_t *o, const mf_wheel_t *wheel, float imu_yaw,
                    uint32_t yaw_seq);

/** @brief 实测弧半径(m) 与新鲜标志；*valid 可为 NULL。恒正，0 = 尚无估计 */
float mf_odom_get_reff(const mf_odom_t *o, uint8_t *valid);

/** @brief 当前生效的形状来源（0=弧长反解 / 1=轮速均值），供调试口回报 */
uint8_t mf_odom_get_arc_src(const mf_odom_t *o);

/** @brief 重定位（比赛起点标定 / 到达地标后校准漂移） */
void mf_odom_set_pose(mf_odom_t *o, float x, float y, float yaw);

#endif /* MF_ODOM_H */
