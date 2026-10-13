/**
 * @file    mf_motion.h
 * @brief   运动框架顶层 API
 *
 * 结构: 应用层 sense(采样) -> mf_tick(里程计->段执行/手动->逆解->轮速环) -> 应用层执行。
 * 固定周期中断里 sense_sample() -> mf_tick() -> drive_apply()
 */
#ifndef MF_MOTION_H
#define MF_MOTION_H

#include "mf_types.h"
#include "mf_pid.h"
#include "mf_odom.h"
#include "mf_path.h"
#include "mf_rate.h"

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

    /* 里程计形状来源（见 mf_config.h 的 MF_ODOM_ARC_SRC）：0=旧弧长反解 1=轮速均值（实测半径） */
    uint8_t odom_arc_src;

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
    mf_rate_t rate;             /* 角速度内环（MF_WZ_RATE_EN，见 mf_rate.h） */
    mf_body_cmd_t manual_cmd;   /* 手动模式指令 */
    mf_body_cmd_t cmd;          /* 本周期最终车体指令（调试用）；wz_dps 已是速率内环输出 */
    mf_status_t status;

    uint32_t in_seq;            /* 上次已消费的采样序号（去重：快照未更新则跳过） */
} mf_ctx_t;

/** @brief 初始化 */
void mf_init(mf_ctx_t *m, const mf_cfg_t *cfg);

/**
 * @brief 主 tick：固定周期中断里调用
 * @param in  应用层最新采样快照；为 NULL 或 seq 未变化时本拍不做任何事
 */
void mf_tick(mf_ctx_t *m, const mf_state_in_t *in);

/* ---------- 路径（声明式）模式 ---------- */
void mf_start_path(mf_ctx_t *m, const mf_seg_t *path);
/* 同 mf_start_path, 但本表跑完自动续跑 next(0=不续, 等价 mf_start_path)。
 * 跨表不停车: 末段按 next 首段速度收尾, 边界无零输出空窗 */
void mf_start_path_chain(mf_ctx_t *m, const mf_seg_t *path, const mf_seg_t *next);
/* 取走"刚跨过表边界"标志(读后清), 见 mf_path_pop_chain */
uint8_t mf_pop_chain(mf_ctx_t *m);
void mf_stop(mf_ctx_t *m);
/* 清速率内环状态（积分/滤波）。上层把 cmd 硬置 0（跟瞄驻停/故障硬停）时必须调用，
 * 否则内环积分会在"输出未被采纳"的驻留期里空转，放行瞬间甩出未指令的转向 */
void mf_clear_rate(mf_ctx_t *m);
/* 取速率内环快照(诊断只读): sp=外环设定 / meas=IMU 实测 / out=下发(含基线+PI) /
 * n_ok,n_stale,n_sign=三种分支的累计拍数(上电起, 停车不清)。见 car_obs.c 的 RATE: 行 */
void mf_rate_get_info(const mf_ctx_t *m, mf_rate_info_t *info);
uint8_t mf_is_done(const mf_ctx_t *m);
uint8_t mf_is_waiting(const mf_ctx_t *m);   /* 当前段是否为 WAIT(主动停等), 见 mf_path_is_waiting */
/**
 * @brief 取走最近一次"段出口质量记录"（读后清标志），见 mf_path_pop_seg_exit
 * @return 1=取到新记录
 */
uint8_t mf_pop_seg_exit(mf_ctx_t *m, uint16_t *seg_index, uint8_t *kind,
                        float *ey_m, float *in_yawerr_deg, float *out_yawerr_deg);
/* 最近一次 ARC 出口的 R(m, 自证量 R≡R_cmd, 判物理半径要用卷尺), 见 mf_path.h */
float mf_exit_radius_m(const mf_ctx_t *m);
/* 最近一次 ARC 的实际转角(deg, 恒正; 名义 90), 见 mf_path.h:mf_path_exit_dyaw_deg */
float mf_exit_dyaw_deg(const mf_ctx_t *m);
/* 实测弧半径(m, 恒正, *valid 可 NULL): 只供观测(RAD: 行); 判据认它或卷尺, 不认 ARC:EY */
float mf_get_reff(const mf_ctx_t *m, uint8_t *valid);
/* 当前生效的里程计形状来源(0/1): 调试口回报, 免得"跑在哪个分支"只能靠猜 */
uint8_t mf_get_odom_arc_src(const mf_ctx_t *m);
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

/* ---------- 航向基准（单基准架构，见 mf_config.h 的 MF_HEADING_FRAME_EN） ---------- */
/* GYRO_ZERO 那一刻调一次: 把出发朝向钉成全程唯一航向基准 */
void mf_set_heading_ref(mf_ctx_t *m, float yaw_deg);

#endif /* MF_MOTION_H */
