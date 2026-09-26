#include "mf_motion.h"
#include "mf_kinematics.h"
#include "mf_math.h"
#include <stddef.h>

void mf_init(mf_ctx_t *m, const mf_cfg_t *cfg)
{
    uint8_t i;

    m->cfg = *cfg;
    m->mode = MF_MODE_MANUAL;
    m->manual_cmd.vx_mps = 0.0f;
    m->manual_cmd.vy_mps = 0.0f;
    m->manual_cmd.wz_dps = 0.0f;

    mf_odom_init(&m->odom, cfg->track_width_m,
                 cfg->half_length_m, cfg->half_width_m);
    m->pose.x_m = 0.0f;
    m->pose.y_m = 0.0f;
    m->pose.yaw_deg = 0.0f;
    
    for (i = 0; i < MF_WHEEL_MAX; i++)
    {
        m->wheel[i].dist_m = 0.0f;
        m->wheel[i].last_dist_m = 0.0f;
        m->wheel[i].delta_m = 0.0f;
        m->wheel[i].speed_mps = 0.0f;
        m->wheel[i].cmd_mps = 0.0f;
        m->wheel[i].out = 0.0f;
    }

    mf_pid_init(&m->pid_yaw, cfg->yaw_kp, cfg->yaw_ki, cfg->yaw_kd,
                cfg->yaw_i_max, cfg->yaw_out_max);
    mf_pid_init(&m->pid_point_v, cfg->pv_kp, 0.0f, 0.0f,
                0.0f, cfg->max_vx_mps);

#if (MF_MOTOR_MODE == MF_MOTOR_PWM_DIR)
    for (i = 0; i < MF_WHEEL_MAX; i++)
    {
        mf_pid_init(&m->pid_wheel[i], cfg->wheel_kp, cfg->wheel_ki, 0.0f,
                    cfg->wheel_i_max, 1.0f);
    }
#endif

    mf_path_init(&m->path, &m->pid_yaw, &m->pid_point_v);
    m->status.state = MF_PATH_IDLE;
    m->status.seg_index = 0;
    m->status.seg_done = 0;
    m->in_seq = 0;
}

void mf_set_mode(mf_ctx_t *m, mf_mode_t mode)
{
    if (mode == MF_MODE_MANUAL)
    {
        mf_path_stop(&m->path);
    }
    m->mode = mode;
}

void mf_set_velocity(mf_ctx_t *m, float vx, float vy, float wz)
{
    m->mode = MF_MODE_MANUAL;
    m->manual_cmd.vx_mps = vx;
    m->manual_cmd.vy_mps = vy;
    m->manual_cmd.wz_dps = wz;
}

void mf_start_path(mf_ctx_t *m, const mf_seg_t *path)
{
    m->mode = MF_MODE_PATH;
    mf_path_start(&m->path, path, 0);
}

void mf_start_path_chain(mf_ctx_t *m, const mf_seg_t *path, const mf_seg_t *next)
{
    m->mode = MF_MODE_PATH;
    mf_path_start(&m->path, path, next);
}

uint8_t mf_pop_chain(mf_ctx_t *m)
{
    return mf_path_pop_chain(&m->path);
}

void mf_stop(mf_ctx_t *m)
{
    mf_path_stop(&m->path);
    m->mode = MF_MODE_MANUAL;
    m->manual_cmd.vx_mps = 0.0f;
    m->manual_cmd.vy_mps = 0.0f;
    m->manual_cmd.wz_dps = 0.0f;
}

uint8_t mf_is_done(const mf_ctx_t *m)
{
    return (mf_path_get_state(&m->path) == MF_PATH_DONE);
}

uint8_t mf_is_waiting(const mf_ctx_t *m)
{
    return mf_path_is_waiting(&m->path);
}

uint8_t mf_pop_arc_report(mf_ctx_t *m, uint16_t *seg_index,
                          float *ey_m, float *yaw_err_deg)
{
    return mf_path_pop_arc_report(&m->path, seg_index, ey_m, yaw_err_deg);
}

const mf_status_t *mf_get_status(mf_ctx_t *m)
{
    m->status.state = mf_path_get_state(&m->path);
    m->status.seg_index = mf_path_get_seg_index(&m->path);
    return &m->status;
}

void mf_set_pose(mf_ctx_t *m, float x, float y, float yaw)
{
    mf_odom_set_pose(&m->odom, x, y, yaw);
    m->pose = m->odom.pose;
}

const mf_pose_t *mf_get_pose(const mf_ctx_t *m)
{
    return &m->pose;
}

#include "mf_motion.h"
#include "mf_kinematics.h"
#include "mf_math.h"
#include <stddef.h>

/* ---------- 单步便捷指令：单段路径（段 + 哨兵） ---------- */

void mf_move_rel(mf_ctx_t *m, float dist_m, float speed_mps, float heading_deg)
{
    static mf_seg_t seg[2];
    seg[0].type = MF_SEG_MOVE_DIST;
    seg[0].reset_pid = 1;
    seg[0].tol_dist_m = 0.0f;
    seg[0].tol_yaw_deg = 0.0f;
    seg[0].p.move_dist.speed_mps = speed_mps;
    seg[0].p.move_dist.dist_m = dist_m;
    seg[0].p.move_dist.heading_deg = heading_deg;
    seg[0].p.move_dist.acc_mps2 = MF_DEFAULT_ACC;
    seg[1].type = MF_SEG_END;
    mf_start_path(m, seg);
}

void mf_turn_to(mf_ctx_t *m, float yaw_deg, float w_max_dps)
{
    static mf_seg_t seg[2];
    seg[0].type = MF_SEG_TURN_TO;
    seg[0].reset_pid = 1;
    seg[0].tol_dist_m = 0.0f;
    seg[0].tol_yaw_deg = 0.0f;
    seg[0].p.turn.yaw_deg = yaw_deg;
    seg[0].p.turn.w_max_dps = w_max_dps;
    seg[1].type = MF_SEG_END;
    mf_start_path(m, seg);
}

void mf_turn_rel(mf_ctx_t *m, float d_yaw_deg, float w_max_dps)
{
    static mf_seg_t seg[2];
    seg[0].type = MF_SEG_TURN_REL;
    seg[0].reset_pid = 1;
    seg[0].tol_dist_m = 0.0f;
    seg[0].tol_yaw_deg = 0.0f;
    seg[0].p.turn.yaw_deg = d_yaw_deg;
    seg[0].p.turn.w_max_dps = w_max_dps;
    seg[1].type = MF_SEG_END;
    mf_start_path(m, seg);
}

void mf_goto_xy(mf_ctx_t *m, float x, float y, float yaw_deg, float v_max)
{
    static mf_seg_t seg[2];
    seg[0].type = MF_SEG_GOTO_XY;
    seg[0].reset_pid = 1;
    seg[0].tol_dist_m = 0.0f;
    seg[0].tol_yaw_deg = 0.0f;
    seg[0].p.goto_xy.x_m = x;
    seg[0].p.goto_xy.y_m = y;
    seg[0].p.goto_xy.yaw_deg = yaw_deg;
    seg[0].p.goto_xy.v_max_mps = v_max;
    seg[0].p.goto_xy.w_max_dps = m->cfg.max_wz_dps;
    seg[0].p.goto_xy.acc_mps2 = 0.0f;
    seg[1].type = MF_SEG_END;
    mf_start_path(m, seg);
}

void mf_goto_rel(mf_ctx_t *m, float dx, float dy, float yaw_deg, float v_max)
{
    static mf_seg_t seg[2];
    seg[0].type = MF_SEG_GOTO_REL;
    seg[0].reset_pid = 1;
    seg[0].tol_dist_m = 0.0f;
    seg[0].tol_yaw_deg = 0.0f;
    seg[0].p.goto_xy.x_m = dx;
    seg[0].p.goto_xy.y_m = dy;
    seg[0].p.goto_xy.yaw_deg = yaw_deg;
    seg[0].p.goto_xy.v_max_mps = v_max;
    seg[0].p.goto_xy.w_max_dps = m->cfg.max_wz_dps;
    seg[0].p.goto_xy.acc_mps2 = 0.0f;
    seg[1].type = MF_SEG_END;
    mf_start_path(m, seg);
}

/* ---------- 主 tick ---------- */

void mf_tick(mf_ctx_t *m, const mf_state_in_t *in)
{
    float dt = MF_DT_S;
    float yaw_imu;
    float wheel_cmd[MF_WHEEL_MAX];
    float ds_body = 0.0f;
    uint8_t i;

    /* 若无新的采样快照，则不做处理 */
    if ((in == NULL) || (in->seq == m->in_seq))
    {
        return;
    }
    m->in_seq = in->seq;

    /* 更新轮子状态 */
    for (i = 0; i < MF_WHEEL_COUNT; i++)
    {
        m->wheel[i].last_dist_m = m->wheel[i].dist_m;
        m->wheel[i].dist_m = in->wheel_dist_m[i];
        m->wheel[i].delta_m = m->wheel[i].dist_m - m->wheel[i].last_dist_m;
        m->wheel[i].speed_mps = m->wheel[i].delta_m / dt;
    }

    /* 2. 更新 航向 和 里程计 */
#if (MF_YAW_SOURCE == MF_YAW_SRC_IMU)
    yaw_imu = in->yaw_deg;
#else
    yaw_imu = m->pose.yaw_deg;
#endif
    mf_odom_update(&m->odom, m->wheel, yaw_imu);
    m->pose = m->odom.pose;

    /* 3. 车体指令：声明式路径 或 手动 */
    if (m->mode == MF_MODE_PATH)
    {
        ds_body = (m->wheel[0].delta_m + m->wheel[1].delta_m) * 0.5f;
        // ds_body = m->odom.last_ds_for_xy; 这个不稳定
        mf_path_tick(&m->path, &m->pose, ds_body, dt, &m->cmd);
    }
    else
    {
        m->cmd = m->manual_cmd;
    }

    /* 速度限幅 */
    m->cmd.vx_mps = mf_clampf(m->cmd.vx_mps, m->cfg.max_vx_mps);
    m->cmd.wz_dps = mf_clampf(m->cmd.wz_dps, m->cfg.max_wz_dps);
#if (MF_CHASSIS_TYPE == MF_CHASSIS_MECANUM)
    m->cmd.vy_mps = mf_clampf(m->cmd.vy_mps, m->cfg.max_vy_mps);
#else
    m->cmd.vy_mps = 0.0f;
#endif

    /* 4. 逆解 -> 轮速目标 */
    mf_inverse_kinematics(&m->cmd, m->cfg.track_width_m,
                          m->cfg.half_length_m, m->cfg.half_width_m,
                          wheel_cmd);

    /* 5. 轮速环计算得到理论输出值 */
    for (i = 0; i < MF_WHEEL_COUNT; i++)
    {
        m->wheel[i].cmd_mps = wheel_cmd[i];

#if (MF_MOTOR_MODE == MF_MOTOR_PWM_DIR)
        m->wheel[i].out = mf_pid_calc(&m->pid_wheel[i],
                                      wheel_cmd[i] - m->wheel[i].speed_mps,
                                      dt);
        if ((m->cmd.vx_mps == 0.0f) && (m->cmd.wz_dps == 0.0f) &&
            (m->cmd_ext_hold == 0u))
        {
            /* 停止指令：直接清输出，避免速度环静差爬行。
             * cmd_ext_hold=1 例外: 段表跑完(cmd 恒 0)但应用层(跟瞄)正要接管本拍输出,
             * 清积分会让低速指令只剩 P 项而卡在履带起步门槛下 */
            m->wheel[i].out = 0.0f;
            mf_pid_reset(&m->pid_wheel[i]);
        }
#else
        m->wheel[i].out = wheel_cmd[i];
#endif
    }
}
