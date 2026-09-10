/**
 * @file    mf_motion.h
 * @brief   运动框架顶层 API
 *
 * 结构：应用层 sense(采样/存储) -> mf_tick(里程计->段执行器/手动->逆解->轮速环) -> 应用层执行
 * 输入使用采样snapshot mf_state_in_t，输出留在 mf_ctx_t.wheel[].out
 *
 * 用法（panzer底盘）：
 *   mf_cfg_t cfg = { ... };
 *   mf_init(&mf, &cfg);
 *   mf_start_path(&mf, panzer_route);    // 或 mf_set_velocity() 手动模式
 *
 *   // 10ms 定时中断里：先采样新快照，再喂给库，最后把 wheel[].out 写到电机
 *   sense_sample();                      // 应用层: 更新快照(seq+1)
 *   mf_tick(&mf, sense_get());           // 无新样时本拍自动跳过
 *   drive_apply(&mf);                    // 应用层: DIR+PWM
 */
#ifndef MF_MOTION_H
#define MF_MOTION_H

#include "mf_types.h"
#include "mf_pid.h"
#include "mf_odom.h"
#include "mf_path.h"

/* 手动指令 / 声明式路径 */
typedef enum
{
    MF_MODE_MANUAL = 0,
    MF_MODE_PATH   = 1
} mf_mode_t;

typedef struct
{
    /* 车辆几何参数 */
    float track_width_m;    /* 差速/履带：等效轨距 m（需打滑标定）；麦轮则缺省此项 */
    float half_length_m;    /* 麦轮：半车长 m */
    float half_width_m;     /* 麦轮：半车宽 m */

    /* 限幅 */
    float max_vx_mps;
    float max_vy_mps;       /* 仅麦轮 */
    float max_wz_dps;

    /* 航向环（巡航保持 / 原地转向 / 方位角共用） */
    float yaw_kp, yaw_ki, yaw_kd;
    float yaw_i_max, yaw_out_max;

    /* 点到点前向速度环（P 即可） */
    float pv_kp;

#if (MF_MOTOR_MODE == MF_MOTOR_PWM_DIR)
    /* 轮速环（m/s 误差 -> 归一化 PWM），panzer 现状即此模式 */
    float wheel_kp, wheel_ki, wheel_i_max;
#endif
} mf_cfg_t;

typedef struct
{
    mf_cfg_t cfg;
    mf_mode_t mode;

    mf_odom_t odom;
    mf_wheel_t wheel[MF_WHEEL_MAX];
    mf_pose_t pose;

    mf_pid_t pid_yaw;
    mf_pid_t pid_point_v;
#if (MF_MOTOR_MODE == MF_MOTOR_PWM_DIR)
    mf_pid_t pid_wheel[MF_WHEEL_MAX];
#endif

    mf_path_ctx_t path;
    mf_body_cmd_t manual_cmd;   /* 手动模式指令 */
    mf_body_cmd_t cmd;          /* 本周期最终车体指令（调试用） */
    mf_status_t status;

    uint32_t in_seq;            /* 上次已消费的采样序号（去重：快照未更新则跳过） */
} mf_ctx_t;

/** @brief 初始化 */
void mf_init(mf_ctx_t *m, const mf_cfg_t *cfg);

/**
 * @brief 主 tick：在固定周期中断里调用（如 panzer TIM4 10ms）
 * @param in  应用层最新采样快照；为 NULL 或 seq 未变化时本拍不做任何事
 */
void mf_tick(mf_ctx_t *m, const mf_state_in_t *in);

/* ---------- 路径（声明式）模式 ---------- */
void mf_start_path(mf_ctx_t *m, const mf_seg_t *path);
void mf_stop(mf_ctx_t *m);
uint8_t mf_is_done(const mf_ctx_t *m);
const mf_status_t *mf_get_status(mf_ctx_t *m);

/* 单步便捷指令：内部各自打包成单段路径，即“一条段的路径” */
void mf_move_rel(mf_ctx_t *m, float dist_m, float speed_mps, float heading_deg);
void mf_turn_to(mf_ctx_t *m, float yaw_deg, float w_max_dps);
void mf_turn_rel(mf_ctx_t *m, float d_yaw_deg, float w_max_dps);
void mf_goto_xy(mf_ctx_t *m, float x, float y, float yaw_deg, float v_max);
void mf_goto_rel(mf_ctx_t *m, float dx, float dy, float yaw_deg, float v_max);

/* ---------- 手动模式 ---------- */
void mf_set_mode(mf_ctx_t *m, mf_mode_t mode);
void mf_set_velocity(mf_ctx_t *m, float vx, float vy, float wz);

/* ---------- 位姿 ---------- */
void mf_set_pose(mf_ctx_t *m, float x, float y, float yaw);     /* 重定位/标定 */
const mf_pose_t *mf_get_pose(const mf_ctx_t *m);

#endif /* MF_MOTION_H */
