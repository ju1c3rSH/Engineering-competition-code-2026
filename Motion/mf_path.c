#include "mf_path.h"
#include "mf_math.h"
#include <math.h>

void mf_path_init(mf_path_ctx_t *p, mf_pid_t *pid_yaw, mf_pid_t *pid_point_v)
{
    p->path = 0;
    p->idx = 0;
    p->state = MF_PATH_IDLE;
    p->pid_yaw = pid_yaw;
    p->pid_point_v = pid_point_v;
    p->started = 0;
}

void mf_path_start(mf_path_ctx_t *p, const mf_seg_t *path)
{
    p->path = path;
    p->idx = 0;
    p->state = MF_PATH_RUN;
    p->started = 0;     /* 第一拍做进段快照 */
}

void mf_path_stop(mf_path_ctx_t *p)
{
    p->state = MF_PATH_IDLE;
    p->path = 0;
}

static float seg_tol_dist(const mf_seg_t *s)
{
    return (s->tol_dist_m > 0.0f) ? s->tol_dist_m : MF_TOL_DIST_M;
}

static float seg_tol_yaw(const mf_seg_t *s)
{
    return (s->tol_yaw_deg > 0.0f) ? s->tol_yaw_deg : MF_TOL_YAW_DEG;
}

/* ---------- 各段类型执行（返回 1=完成） ---------- */

/* 增量直线：航向保持 + 里程累计判完成 */
static uint8_t seg_move_dist(mf_path_ctx_t *p, const mf_seg_t *s,
                             const mf_pose_t *pose, float dt,
                             mf_body_cmd_t *out)
{
    const mf_move_dist_t *m = &s->p.move_dist;
    float sp, err, acc;

    acc = (m->acc_mps2 > 0.0f) ? m->acc_mps2 : MF_DEFAULT_ACC;
    p->cur_speed = mf_rampf(p->cur_speed, m->speed_mps, acc, dt);
    sp = (m->dist_m >= 0.0f) ? p->cur_speed : -p->cur_speed;

    out->vx_mps = sp;
    out->vy_mps = 0.0f;

    err = mf_wrap_deg(((m->heading_deg == MF_KEEP) ? p->entry.yaw_deg
                                                   : m->heading_deg)
                      - pose->yaw_deg);
    out->wz_dps = mf_pid_calc(p->pid_yaw, err, dt);

    return (p->dist_accum * m->dist_m >= m->dist_m * m->dist_m);
}

/* 定半径弧线：航向目标按弧长推进，走够 d_yaw 完成（panzer 转弯的声明式版） */
static uint8_t seg_arc(mf_path_ctx_t *p, const mf_seg_t *s,
                       const mf_pose_t *pose, float dt,
                       mf_body_cmd_t *out)
{
    const mf_arc_t *a = &s->p.arc;
    float err, acc, yaw_sp;

    acc = (a->acc_mps2 > 0.0f) ? a->acc_mps2 : MF_DEFAULT_ACC;
    p->cur_speed = mf_rampf(p->cur_speed, a->speed_mps, acc, dt);

    out->vx_mps = p->cur_speed;
    out->vy_mps = 0.0f;

    yaw_sp = p->entry.yaw_deg
           + (p->dist_accum / a->radius_m) * (180.0f / MF_PI_F);
    err = mf_wrap_deg(yaw_sp - pose->yaw_deg);
    out->wz_dps = mf_pid_calc(p->pid_yaw, err, dt);

    return ((pose->yaw_deg - p->entry.yaw_deg) * a->d_yaw_deg
            >= a->d_yaw_deg * a->d_yaw_deg);
}

/* 原地转向（绝对/相对统一） */
static uint8_t seg_turn(mf_path_ctx_t *p, const mf_seg_t *s,
                        const mf_pose_t *pose, float dt,
                        mf_body_cmd_t *out)
{
    const mf_turn_t *t = &s->p.turn;
    float target = (s->type == MF_SEG_TURN_TO) ? t->yaw_deg
                   : p->entry.yaw_deg + t->yaw_deg;
    float err = mf_wrap_deg(target - pose->yaw_deg);

    out->vx_mps = 0.0f;
    out->vy_mps = 0.0f;
    out->wz_dps = mf_pid_calc(p->pid_yaw, err, dt);
    out->wz_dps = mf_clampf(out->wz_dps, (t->w_max_dps > 0.0f)
                                         ? t->w_max_dps : 1e9f);

    return (err < seg_tol_yaw(s)) && (err > -seg_tol_yaw(s));
}

/* 点到点：方位角 P 转向 + 距离 P 前进，到位后按声明收尾转向 */
static uint8_t seg_goto(mf_path_ctx_t *p, const mf_seg_t *s,
                        const mf_pose_t *pose, float dt,
                        mf_body_cmd_t *out)
{
    const mf_goto_t *g = &s->p.goto_xy;
    float tol_d = seg_tol_dist(s);
    float tol_y = seg_tol_yaw(s);
    float err_x_w, err_y_w, err_x_b, err_y_b;
    float dist_err, bearing, c, s_yaw, vx;
    float dyaw;

    c = cosf(pose->yaw_deg * (MF_PI_F / 180.0f));
    s_yaw = sinf(pose->yaw_deg * (MF_PI_F / 180.0f));

    if (p->subphase == 0)
    {
        err_x_w = p->target.x_m - pose->x_m;
        err_y_w = p->target.y_m - pose->y_m;
        err_x_b =  c * err_x_w + s_yaw * err_y_w;   /* 旋转到车体系 */
        err_y_b = -s_yaw * err_x_w + c * err_y_w;
        dist_err = sqrtf(err_x_b * err_x_b + err_y_b * err_y_b);
        bearing = atan2f(err_y_b, err_x_b) * (180.0f / MF_PI_F);

        out->wz_dps = mf_clampf(mf_pid_calc(p->pid_yaw, bearing, dt),
                                (g->w_max_dps > 0.0f) ? g->w_max_dps : 1e9f);

        vx = mf_pid_calc(p->pid_point_v, dist_err, dt);
        vx = mf_clampf(vx, (g->v_max_mps > 0.0f) ? g->v_max_mps : 1e9f);
        if (bearing > 90.0f || bearing < -90.0f)    /* 目标在身后：先原地转 */
        {
            vx = 0.0f;
        }
        out->vx_mps = vx;
        out->vy_mps = 0.0f;

        if (dist_err < tol_d)
        {
            if (g->yaw_deg == MF_KEEP)
            {
                return 1;
            }
            p->subphase = 1;                        /* 转入收尾转向 */
            mf_pid_reset(p->pid_yaw);
        }
        return 0;
    }
    else
    {
        dyaw = mf_wrap_deg(g->yaw_deg - pose->yaw_deg);
        out->vx_mps = 0.0f;
        out->vy_mps = 0.0f;
        out->wz_dps = mf_clampf(mf_pid_calc(p->pid_yaw, dyaw, dt),
                                (g->w_max_dps > 0.0f) ? g->w_max_dps : 1e9f);
        return (dyaw < tol_y) && (dyaw > -tol_y);
    }
}

/* 进段快照：把进入时刻位姿记下，并解算 GOTO 段的绝对目标 */
static void seg_enter(mf_path_ctx_t *p, const mf_seg_t *s,
                      const mf_pose_t *pose)
{
    const mf_goto_t *g;
    float c, s_yaw, dx, dy;

    p->entry = *pose;
    p->dist_accum = 0.0f;
    p->time_ms = 0;
    p->cur_speed = 0.0f;
    p->subphase = 0;

    switch (s->type)
    {
    case MF_SEG_GOTO_XY:
        p->target.x_m = s->p.goto_xy.x_m;
        p->target.y_m = s->p.goto_xy.y_m;
        break;
    case MF_SEG_GOTO_REL:       /* 车体系相对位移 -> 绝对目标 */
        g = &s->p.goto_xy;
        c = cosf(pose->yaw_deg * (MF_PI_F / 180.0f));
        s_yaw = sinf(pose->yaw_deg * (MF_PI_F / 180.0f));
        dx = g->x_m;
        dy = g->y_m;
        p->target.x_m = pose->x_m + dx * c - dy * s_yaw;
        p->target.y_m = pose->y_m + dx * s_yaw + dy * c;
        break;
    default:
        break;
    }
}

uint8_t mf_path_tick(mf_path_ctx_t *p, const mf_pose_t *pose,
                     float ds, float dt, mf_body_cmd_t *out)
{
    const mf_seg_t *s;
    uint8_t done = 0;

    out->vx_mps = 0.0f;
    out->vy_mps = 0.0f;
    out->wz_dps = 0.0f;

    if (p->state != MF_PATH_RUN || p->path == 0)
    {
        return 0;
    }

    s = &p->path[p->idx];
    if (s->type == MF_SEG_END)
    {
        p->state = MF_PATH_DONE;
        return 0;
    }

    if (!p->started)        /* 进段：快照 + 可选复位 PID */
    {
        p->started = 1;
        seg_enter(p, s, pose);
        if (s->reset_pid)
        {
            mf_pid_reset(p->pid_yaw);
            mf_pid_reset(p->pid_point_v);
        }
    }

    p->time_ms += (uint32_t)(dt * 1000.0f);

    switch (s->type)
    {
    case MF_SEG_MOVE_DIST:
    case MF_SEG_ARC:
        p->dist_accum += ds;    /* 弧长/里程统一由位移累计 */
        break;
    default:
        break;
    }

    switch (s->type)
    {
    case MF_SEG_MOVE_DIST:  done = seg_move_dist(p, s, pose, dt, out); break;
    case MF_SEG_ARC:        done = seg_arc(p, s, pose, dt, out);       break;
    case MF_SEG_TURN_TO:
    case MF_SEG_TURN_REL:   done = seg_turn(p, s, pose, dt, out);      break;
    case MF_SEG_GOTO_XY:
    case MF_SEG_GOTO_REL:   done = seg_goto(p, s, pose, dt, out);      break;
    case MF_SEG_WAIT:       done = (p->time_ms >= s->p.wait.duration_ms); break;
    default:                done = 1; break;
    }

    if (done)
    {
        p->idx++;
        p->started = 0;     /* 下一段下拍重新进段 */
        if (p->path[p->idx].type == MF_SEG_END)
        {
            p->state = MF_PATH_DONE;
            out->vx_mps = 0.0f;
            out->vy_mps = 0.0f;
            out->wz_dps = 0.0f;
        }
    }
    return done;
}

mf_path_state_t mf_path_get_state(const mf_path_ctx_t *p)
{
    return p->state;
}

uint16_t mf_path_get_seg_index(const mf_path_ctx_t *p)
{
    return p->idx;
}
