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
    m->odom.arc_src = cfg->odom_arc_src;    /* 形状来源可由 cfg 覆盖宏默认值 */
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
#if MF_WZ_RATE_EN
    mf_rate_init(&m->rate, MF_WZ_RATE_KP, MF_WZ_RATE_KI, MF_WZ_RATE_I_MAX,
                 cfg->max_wz_dps * MF_WZ_RATE_OUT_HEADROOM,
                 MF_WZ_RATE_LPF_HZ);
#endif
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
        mf_clear_rate(m);
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

/* 清速率内环状态。除停机外, 上层把 cmd 硬置 0(跟瞄驻停/故障硬停)也必须调一次:
 * 内环在 mf_tick 内先于上层 overlay 执行, 上层把输出丢掉却不告诉内环, 积分就会在
 * 驻留期里对着"实测≈0、设定≠0"空转, 放行那一拍甩出一次未指令的转向 */
void mf_clear_rate(mf_ctx_t *m)
{
    mf_rate_reset(&m->rate);
}

/* 速率内环快照(诊断): 上层主循环在临界区里读一次就够。判法见 App/car_obs.c 的 RATE: 行 */
void mf_rate_get_info(const mf_ctx_t *m, mf_rate_info_t *info)
{
    if (info == 0)
    {
        return;
    }
    info->sp_dps   = m->rate.sp_dps;
    info->meas_dps = m->rate.meas_dps;
    info->out_dps  = m->rate.out_dps;
    info->base_dps = m->rate.base_dps;
    info->i_term_dps = m->rate.pid.ki * m->rate.pid.integral;
    info->n_ok     = m->rate.n_ok;
    info->n_stale  = m->rate.n_stale;
    info->n_sign   = m->rate.n_sign;
}

void mf_stop(mf_ctx_t *m)
{
    mf_path_stop(&m->path);
    mf_clear_rate(m);
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

/* 最近一次 ARC 出口的 R(m) = 本段轮程 / |IMU 转角|, 见 mf_path.c。
 * ⚠ 它不是物理半径: 内环让 ω=v_wheel/R_cmd ⇒ 本值 ≡ R_cmd(自证), 判真半径只能用卷尺 */
float mf_exit_radius_m(const mf_ctx_t *m)
{
    return mf_path_exit_radius_m(&m->path);
}

/* 最近一次 ARC 的实际转角(deg) —— 判"弧转够了没有"的直接读数(见 mf_path.h) */
float mf_exit_dyaw_deg(const mf_ctx_t *m)
{
    return mf_path_exit_dyaw_deg(&m->path);
}

/* 实测弧半径(m) + 新鲜标志: 同间隔配对的独立观测量(见 mf_odom.h / mf_config.h 的 MF_REFF_*)。
 * 判"弧此时在哪条半径上"用它, 不要用 ARC:EY —— odom 的形状与指令同源, EY 结构性看不见打滑 */
float mf_get_reff(const mf_ctx_t *m, uint8_t *valid)
{
    return mf_odom_get_reff(&m->odom, valid);
}

/* 当前生效的里程计形状来源: 调试口 RAD: 行要打它 —— "跑在哪个分支"必须能从日志上看出来 */
uint8_t mf_get_odom_arc_src(const mf_ctx_t *m)
{
    return mf_odom_get_arc_src(&m->odom);
}

uint8_t mf_pop_seg_exit(mf_ctx_t *m, uint16_t *seg_index, uint8_t *kind,
                        float *ey_m, float *in_yawerr_deg, float *out_yawerr_deg)
{
    return mf_path_pop_seg_exit(&m->path, seg_index, kind,
                               ey_m, in_yawerr_deg, out_yawerr_deg);
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

/* 全程唯一航向基准: 由上层在 GYRO_ZERO 那一刻调用一次(见 mf_config.h 的 MF_HEADING_FRAME_EN)。
 * 注意它**不动 pose** —— pose.yaw 1:1 跟随 IMU(下一拍就会被覆盖), 能持久的是这个基准 */
void mf_set_heading_ref(mf_ctx_t *m, float yaw_deg)
{
    mf_path_set_heading_ref(&m->path, yaw_deg);
}

const mf_pose_t *mf_get_pose(const mf_ctx_t *m)
{
    return &m->pose;
}

/* ---------- 单步便捷指令：单段路径（段 + 哨兵） ----------
 * ⚠ 每个函数用**函数内 static** 存段表, 而 mf_start_path 保存的是它的指针 ⇒ 正在跑某条单段路径时
 * 再调任何一个单步指令, 会把在跑的段表**就地覆盖**(静默)。所以单步指令只用于台架/调试,
 * 正式路线一律走 nav_routes 的 flash 段表; 要支持嵌套得把 seg 提到调用方(下一轮) */

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
    uint8_t cmd_idle;       /* 本拍外环是否要求停（内环之前取，见第 4 步注释） */

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
    mf_odom_update(&m->odom, m->wheel, yaw_imu, in->yaw_seq);
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

    /* 4. 角速度内环: 上一步算出的 wz 只是**设定值**, 再对 IMU 实测转速闭环 ——
     * 履带转弯的实际角速度只有指令的 k 倍(k<1), 这个亏损由本环的积分补掉,
     * 逆解不再需要静态 boost 去猜 k(见 mf_rate.h / mf_config.h)。
     * cmd_idle 必须在内环**之前**取: 第 6 步的"直接清输出"判的是外环要不要停,
     * 若拿内环输出判, 车还在转时 wz!=0 -> 清不掉输出, 偏离急停语义 */
    cmd_idle = (uint8_t)((m->cmd.vx_mps == 0.0f) && (m->cmd.wz_dps == 0.0f));

#if MF_WZ_RATE_EN
    /* 滑移基线只对**段表弧线**生效: 手动模式的 wz 是遥控量, 弧线的几何前提(半径/
     * 切向)都不存在, 拿 1/0.57 去放大它等于改动遥控手感 */
    {
        float slip0 = (m->mode == MF_MODE_PATH) ? m->path.slip_gain : 1.0f;

        m->cmd.wz_dps = mf_rate_apply(&m->rate, m->cmd.wz_dps, in->wz_dps,
                                      in->wz_valid, slip0);
    }
#endif

    /* 5. 逆解 -> 轮速目标 */
    mf_inverse_kinematics(&m->cmd, m->cfg.track_width_m,
                          m->cfg.half_length_m, m->cfg.half_width_m,
                          wheel_cmd);

    /* 6. 轮速环计算得到理论输出值 */
#if MF_WZ_RATE_EN
    if (cmd_idle != 0u)
    {
        /* cmd 被硬置 0(表跑完/IDLE)时内环也必须停: 否则 wz_sp=0 而 ω≠0, 误差照积,
         * 放行首拍甩出未指令的转向 —— 同 mf_motion.h:mf_clear_rate 的约定 */
        mf_rate_reset(&m->rate);
    }
#endif
    for (i = 0; i < MF_WHEEL_COUNT; i++)
    {
        m->wheel[i].cmd_mps = wheel_cmd[i];

#if (MF_MOTOR_MODE == MF_MOTOR_PWM_DIR)
        m->wheel[i].out = mf_pid_calc(&m->pid_wheel[i],
                                      wheel_cmd[i] - m->wheel[i].speed_mps,
                                      dt);
        if (cmd_idle != 0u)
        {
            /* 停止指令：直接清输出，避免速度环静差爬行 */
            m->wheel[i].out = 0.0f;
            mf_pid_reset(&m->pid_wheel[i]);
        }
#else
        m->wheel[i].out = wheel_cmd[i];
#endif
    }
}
