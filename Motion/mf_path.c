#include "mf_path.h"
#include "mf_math.h"
#include <math.h>
static float seg_next_entry_speed(const mf_path_ctx_t *p);

void mf_path_init(mf_path_ctx_t *p, mf_pid_t *pid_yaw, mf_pid_t *pid_point_v)
{
    p->path = 0;
    p->next_path = 0;
    p->chain_new = 0u;
    p->idx = 0;
    p->state = MF_PATH_IDLE;
    p->pid_yaw = pid_yaw;
    p->pid_point_v = pid_point_v;
    p->started = 0;
    p->dist_accum = 0.0f;
    p->dist_target_m = 0.0f;
    p->dist_base_m = 0.0f;
    p->carry_link = 0u;
    p->arc_cx_m = 0.0f;
    p->arc_cy_m = 0.0f;
    p->arc_r_abs = 0.0f;
    p->arc_sgn = 1.0f;
    p->arc_ey_m = 0.0f;
    p->slip_gain = 1.0f;
    p->seg_exit_ey_m = 0.0f;
    p->seg_exit_yawerr_deg = 0.0f;
    p->seg_exit_in_yawerr_deg = 0.0f;
    p->seg_exit_radius_m = 0.0f;
    p->seg_exit_dyaw_deg = 0.0f;
    p->seg_exit_seg = 0u;
    p->seg_exit_kind = 0u;
    p->seg_exit_new = 0u;
    p->nom_yaw_deg = 0.0f;
    p->nom_yaw_init = 0u;
    p->heading_ref_locked = 0u;
    p->line_in_yawerr_deg = 0.0f;
}

/* 显式钉下全程唯一航向基准(单基准架构, MF_HEADING_FRAME_EN=1): 见 mf_path.h 的说明。
 * 与"旧行为按表重定锚"的唯一差别就是这个 locked 标志 —— mf_path_start() 只在未锁定时
 * 才清 nom_yaw_init */
void mf_path_set_heading_ref(mf_path_ctx_t *p, float yaw_deg)
{
    p->nom_yaw_deg = mf_wrap_deg(yaw_deg);
    p->nom_yaw_init = 1u;
    p->heading_ref_locked = 1u;
}

void mf_path_start(mf_path_ctx_t *p, const mf_seg_t *path, const mf_seg_t *next)
{
    uint16_t i;
    const float r_min = 1.0e-3f;

    /* 启动前校验段表: ARC 半径过小或与 d_yaw 反号会除零/原地兜圈, 非法则拒绝启动(保持 IDLE) */
    for (i = 0u; (i < 255u) && (path[i].type != MF_SEG_END); i++)
    {
        if (path[i].type == MF_SEG_ARC)
        {
            const mf_arc_t *a = &path[i].p.arc;
            if ((a->radius_m < r_min) && (a->radius_m > -r_min))
            {
                return;
            }
            if ((a->d_yaw_deg != 0.0f) && ((a->d_yaw_deg * a->radius_m) < 0.0f))
            {
                return;
            }
        }
    }

    p->path = path;
    p->next_path = next;
    p->chain_new = 0u;
    p->idx = 0;
    p->state = MF_PATH_RUN;
    p->started = 0;     /* 第一拍做进段快照 */
    /* 段间累积量：新路径从零起算（MF_SEG_CARRY_EN 下 dist_accum 是全程里程） */
    p->dist_accum = 0.0f;
    p->dist_target_m = 0.0f;
    p->dist_base_m = 0.0f;
    p->carry_link = 0u;
#if MF_SEG_NOM_HEADING_EN
  #if MF_HEADING_FRAME_EN
    /* 单基准: 基准由 mf_path_set_heading_ref() 在 GYRO_ZERO 那一刻显式钉下, 此后跨表**不重定锚**。
     * 未钉下时(heading_ref_locked=0)退回"首段用实测航向定基准", 与旧行为等价 ——
     * 自测形态没有 GYRO_ZERO 命令, 走的就是这条兜底 */
    if (p->heading_ref_locked == 0u)
    {
        p->nom_yaw_init = 0u;
    }
  #elif MF_NOM_HEADING_PERSIST_EN
    /* 旧行为 + 跨表延续(不清初始化标志): 首段实测航向当全程基准, 隐式 */
  #else
    /* 旧行为: 标称航向重定基准 —— 首段进段时用当时的实测航向初始化, 之后纯几何累积 */
    p->nom_yaw_init = 0u;
  #endif
#endif
}

void mf_path_stop(mf_path_ctx_t *p)
{
    p->state = MF_PATH_IDLE;
    p->path = 0;
    p->next_path = 0;
    p->chain_new = 0u;
    p->cur_speed = 0.0f;    /* 停路径即视为停稳, 新路径从 0 起坡 */
    p->cur_wz = 0.0f;
    p->prev_wz = 0.0f;
    p->slip_gain = 1.0f;    /* 手动/停稳段没有弧线几何, 滑移基线必须复位 */
}

uint8_t mf_path_pop_chain(mf_path_ctx_t *p)
{
    if ((p == 0) || (p->chain_new == 0u))
    {
        return 0u;
    }
    p->chain_new = 0u;
    return 1u;
}

/* 跨表链续: 当前表已到 END 且有非空续表时切过去, 由下一拍照常进段执行。
 * 单独成函数是为了让"该不该切"只有一处判断, tick 里两处 END 分支共用 */
static uint8_t mf_path_chain(mf_path_ctx_t *p)
{
    const mf_seg_t *n = p->next_path;

    if ((n == 0) || (n[0].type == MF_SEG_END))
    {
        return 0u;
    }
    p->path = n;
    p->next_path = 0;
    p->idx = 0u;
    p->started = 0;     /* 下一拍重新进段(若本拍在 tick 顶部, 则本拍即进段) */
    p->chain_new = 1u;
    p->dist_accum = 0.0f;
    p->dist_base_m = 0.0f;
    p->carry_link = 0u;
    return 1u;
}

static float seg_tol_dist(const mf_seg_t *s)
{
    return (s->tol_dist_m > 0.0f) ? s->tol_dist_m : MF_TOL_DIST_M;
}

static float seg_tol_yaw(const mf_seg_t *s)
{
    return (s->tol_yaw_deg > 0.0f) ? s->tol_yaw_deg : MF_TOL_YAW_DEG;
}

/* ---------- 段间速度衔接与到位判定 ---------- */

/* 下一段是否要求车体先停稳(原地转向/等待/结束):
 * 是则本段按剩余行程抛物线减速, 段末速度≈0, 杜绝段边界速度阶跃 */
static uint8_t seg_next_needs_stop(const mf_path_ctx_t *p)
{
    mf_seg_type_t next = p->path[p->idx + 1u].type;

    return (uint8_t)((next == MF_SEG_END) || (next == MF_SEG_TURN_TO)
                  || (next == MF_SEG_TURN_REL) || (next == MF_SEG_WAIT));
}

/* 斜坡到段期望速度并输出: cur_speed 跨段保持, 相邻同速段无缝续跑 */
static float seg_speed_ramp(mf_path_ctx_t *p, float target, float acc, float dt)
{
    p->cur_speed = mf_rampf(p->cur_speed, target,
                            (acc > 0.0f) ? acc : MF_DEFAULT_ACC, dt);
    return p->cur_speed;
}

/* 角速度斜坡(幅值不对称): 只限制幅值上升, 消除 TURN/GOTO 入段的差速阶跃;
 * 幅值下降(误差自然衰减)直接跟随, 不给航向环引入额外相位滞后 */
static float seg_wz_ramp(mf_path_ctx_t *p, float target, float dt)
{
    float step = MF_DEFAULT_ANG_ACC * dt;
    float mag_t = (target < 0.0f) ? -target : target;
    float mag_c = (p->cur_wz < 0.0f) ? -p->cur_wz : p->cur_wz;
    float mag;

    if (mag_t > mag_c + step)
    {
        mag = mag_c + step;
    }
    else
    {
        mag = mag_t;
    }
    p->cur_wz = (target < 0.0f) ? -mag : mag;
    return p->cur_wz;
}

/* 带驻留的到位判定: |err|<tol 连续保持 MF_TOL_HOLD_MS 才算到位,
 * 防止高速穿越容差窗被单拍误判, 也抑制噪声抖动 */
static uint8_t seg_dwell_reached(mf_path_ctx_t *p, float err, float tol, float dt)
{
    if ((err > tol) || (err < -tol))
    {
        p->tol_hold_ms = 0u;
        return 0u;
    }
    p->tol_hold_ms = (uint16_t)(p->tol_hold_ms + (uint16_t)(dt * 1000.0f));
    return (uint8_t)(p->tol_hold_ms >= MF_TOL_HOLD_MS);
}

/* ---------- 各段类型执行（返回 1=完成） ---------- */

/* 期望角(基准系几何) -> 实际下发角: 只差一个固定的基准偏差 MF_HEADING_BIAS_DEG。
 * 全工程只在这一处加, 保证"段表里全是几何量、补偿只有一个数"(见 mf_config.h)。
 * 只作用于**绝对角目标**; 相对角(TURN_REL/GOTO_REL)与 ARC 自解算不加 —— 它们从当前
 * 位姿续起, 天然继承整条路线的刚性旋转 */
static float seg_cmd_yaw(float geom_deg)
{
    return geom_deg + MF_HEADING_BIAS_DEG;
}

/* MOVE 段的航向保持目标: 段表给了绝对航向就优先用; MF_KEEP 时按开关取标称航向或进段实测航向 */
static float seg_move_hold_yaw(const mf_path_ctx_t *p, const mf_move_dist_t *m)
{
    if (m->heading_deg != MF_KEEP)
    {
        return m->heading_deg;
    }
#if MF_SEG_NOM_HEADING_EN
    return p->nom_yaw_deg;
#else
    return p->entry.yaw_deg;
#endif
}

/* 增量直线：航向保持 + 剩余行程判完成
 * 下一段要求停稳时按 v=sqrt(2·a·s) 抛物线减速, 段末速度≈0 平滑交棒 */
static uint8_t seg_move_dist(mf_path_ctx_t *p, const mf_seg_t *s,
                             const mf_pose_t *pose, float dt,
                             mf_body_cmd_t *out)
{
    const mf_move_dist_t *m = &s->p.move_dist;
    float dir, remain, acc, cruise, target, v_stop,v_next, err;

    v_next = seg_next_entry_speed(p);
    dir = (m->dist_m >= 0.0f) ? 1.0f : -1.0f;
#if MF_SEG_CARRY_EN
    remain = (p->dist_target_m - p->dist_accum) * dir;  /* 累计名义目标 - 当前里程 */
#else
    remain = (m->dist_m - p->dist_accum) * dir;     /* 沿行进方向剩余行程 m */
#endif
    acc = (m->acc_mps2 > 0.0f) ? m->acc_mps2 : MF_DEFAULT_ACC;
    cruise = (m->speed_mps < 0.0f) ? -m->speed_mps : m->speed_mps;
    target = cruise;

    if (v_next >= 0.0f)
    {
        if (remain > 0.0f)
        {
            v_stop = sqrtf(2.0f * acc * MF_STOP_DECEL_SCALE * remain
                           + v_next * v_next);
        }
        else
        {
            v_stop = v_next; // 已经到位
        }
        if (v_stop < target)
        {
            target = v_stop;
        }
    }

    out->vx_mps = dir * seg_speed_ramp(p, target, acc, dt);
    out->vy_mps = 0.0f;

    err = mf_wrap_deg(seg_cmd_yaw(seg_move_hold_yaw(p, m)) - pose->yaw_deg);
    out->wz_dps = mf_pid_calc(p->pid_yaw, err, dt);

    /* 失控自保 (test/motion-tuning): 编码器/驱动符号矛盾时反馈为负, remain 沿
     * 正向滚出 3 倍目标距离 —— 强制结束本段, 防正反馈无限狂转(实测 BOR 死机工况) */
    if (remain > (3.0f * ((m->dist_m < 0.0f) ? -m->dist_m : m->dist_m) + 0.5f))
    {
        return 1u;
    }
    if (remain <= MF_TOL_ARRIVE_M)
    {
        /* 直线段出口记录: IE=进段航向误差(带进来多少) / YE=出口残差(纠完剩多少)。
         * 单基准架构下 IE 才有非零值 —— 旧行为(MF_KEEP 取进段实测)IE 恒 0, 上游误差
         * 被锁死、现场看不见, 所以这一对数是"新旧架构有没有生效"的直接判据 */
        p->seg_exit_ey_m = 0.0f;    /* 直线无名义线, 横向偏差不在此列(见 mf_path.h) */
        p->seg_exit_in_yawerr_deg = p->line_in_yawerr_deg;
        p->seg_exit_yawerr_deg = err;
        p->seg_exit_seg = p->idx;
        p->seg_exit_kind = 1u;
        p->seg_exit_new = 1u;
        return 1u;
    }
    return 0u;
}

/* 弧线几何前馈 wz_ff = v/R × 径向 trim。trim 见 mf_config.h —— 它是**物理半径的唯一旋钮**
 * (基线/内环/横向项全被速率内环吸收, 改不动稳态半径)。三处用同一个式子, 不许各写一份 */
static float seg_arc_wz_ff(const mf_path_ctx_t *p, const mf_arc_t *a)
{
    return (p->cur_speed / a->radius_m) * (180.0f / MF_PI_F) * MF_ARC_WZ_TRIM;
}

/* 定半径弧线: 走够 d_yaw 即完成(判据只用 IMU 转角); 设定值取"名义切线航向"
 * = 圆心→车体方位角 + sgn·90°(由里程计位置推出), 再加横向偏差比例项。旧律对比见 Motion/README.md */
static uint8_t seg_arc(mf_path_ctx_t *p, const mf_seg_t *s,
                       const mf_pose_t *pose, float dt,
                       mf_body_cmd_t *out)
{
    const mf_arc_t *a = &s->p.arc;
    float acc, target, v_stop, v_next;
    float yaw_done, remain_yaw, remain_m, radius_abs;

    v_next = seg_next_entry_speed(p);
    acc = (a->acc_mps2 > 0.0f) ? a->acc_mps2 : MF_DEFAULT_ACC;
    yaw_done = pose->yaw_deg - p->entry.yaw_deg;
    remain_yaw = (a->d_yaw_deg - yaw_done)
               * ((a->d_yaw_deg >= 0.0f) ? 1.0f : -1.0f);  /* 剩余转角, 正=未走完 */
    target = (a->speed_mps < 0.0f) ? -a->speed_mps : a->speed_mps;

    if (v_next >= 0.0f)
    {
        radius_abs = fabsf(a->radius_m);
        remain_m = ((remain_yaw > 0.0f) ? remain_yaw : 0.0f)
                 * (MF_PI_F / 180.0f) * radius_abs;
        v_stop = sqrtf(2.0f * acc * MF_STOP_DECEL_SCALE * remain_m
                       + v_next * v_next);
        if (v_stop < target)
        {
            target = v_stop;
        }
    }

    /* 没走完就不许停: 末段弧的下一段要求停稳时 v_stop 会把目标减到 0, 而 remain_yaw
     * 可能还大于到位余量 ⇒ 车停在段里永远走不完。见 MF_ARC_CREEP_MPS
     * (用运行期比较而不是 #if: 预处理器不认浮点字面量。宏为 0 时编译器会把整个分支折掉) */
    if ((MF_ARC_CREEP_MPS > 0.0f) && (remain_yaw > MF_TOL_ARRIVE_DEG)
        && (target < MF_ARC_CREEP_MPS))
    {
        target = MF_ARC_CREEP_MPS;
    }

    out->vx_mps = seg_speed_ramp(p, target, acc, dt);
    out->vy_mps = 0.0f;

    /* 几何量: 半径方向偏差 e_y(正=弧外) 与名义切线航向偏差 e_psi。
     * 两者都在世界系解算, 与 dist_accum / entry.yaw 的累积无关 */
    {
        float rx    = pose->x_m - p->arc_cx_m;
        float ry    = pose->y_m - p->arc_cy_m;
        float rho   = sqrtf(rx * rx + ry * ry);
        float phi   = atan2f(ry, rx) * (180.0f / MF_PI_F);
        float e_psi = mf_wrap_deg((phi + p->arc_sgn * 90.0f) - pose->yaw_deg);

        p->arc_ey_m = rho - p->arc_r_abs;       /* 正=弧外 */
        p->seg_exit_yawerr_deg = e_psi;         /* 出口记录取段完成那一拍的实测值 */

        /* 曲率前馈 wz_ff = v/R(带符号 radius, 左转+/右转-): 稳态角速度开环直给,
         * PID 不再靠误差积累(纯P需滞后~3.6°才出稳态差速=进弯晚的根因), 只修打滑
         * 残差; 速度源用 cur_speed(斜坡后实际值), 起步/减速前馈自动缩放。
         * 注意: 勿套 seg_wz_ramp —— 进弯缓起 = 进弯前固定多走直线(实测 10~30cm) */
#if MF_ARC_XTRACK_EN
        /* 横向项**必须乘 arc_sgn**(左+1/右-1): e_y 恒正=弧外、不分左右, 但"回弧内往哪边打"分左右
         * —— 漏乘会把右转弧推向弧外(实测右转 EY +103 vs 左转 +77)。
         * 注意 arc_xtrack_sim.py 只有左转用例, 这个符号错在仿真里不会暴露; 改仿真务必补右转用例。
         * 另: 本项只当**微调** —— 半径的主责归速率内环(v/R + 内环闭环), 故自带 MF_ARC_XTRACK_MAX_FRAC
         * 限幅, 免得它顶到段内限幅、与内环抢同一个误差(见 mf_config.h 的回路增益估算) */
        {
            float wz_ff = seg_arc_wz_ff(p, a);
            float xt    = MF_ARC_XTRACK_K * p->arc_ey_m * p->arc_sgn;
            float xt_lim = mf_absf(wz_ff) * MF_ARC_XTRACK_MAX_FRAC;

            out->wz_dps = mf_pid_calc(p->pid_yaw, e_psi, dt) + wz_ff + mf_clampf(xt, xt_lim);
        }
#else
        {
            float yaw_sp = p->entry.yaw_deg
                         + ((p->dist_accum - p->dist_base_m) / a->radius_m)
                           * (180.0f / MF_PI_F);
            float err = mf_wrap_deg(yaw_sp - pose->yaw_deg);
            out->wz_dps = mf_pid_calc(p->pid_yaw, err, dt) + seg_arc_wz_ff(p, a);
        }
#endif

        /* 弧线自限幅: 上面三项叠加(航向P + v/R + 横向K·e_y)可以顶到 cfg.max_wz_dps,
         * 那是 R≈0.27m 的原地上转 —— 履带横向刮擦物理上做不到, 只会把车钉住、SP 钉在
         * 限幅上十几秒而弧走不完(2026-09-30 实测)。上限定在几何前馈的
         * MF_ARC_WZ_MAX_MULT 倍: 半径最紧只允许做到 R/该倍数, 慢一点也不会自锁 */
        {
            float wz_ff  = seg_arc_wz_ff(p, a);
            float wz_lim = mf_absf(wz_ff) * MF_ARC_WZ_MAX_MULT;

            if (wz_lim > 1.0f)
            {
                out->wz_dps = mf_clampf(out->wz_dps, wz_lim);
            }
        }
    }

    return (uint8_t)(remain_yaw <= MF_TOL_ARRIVE_DEG);
}

/* 原地转向（绝对/相对统一）: 前向速度斜坡归零再转, 角速度缓起, 驻留判到位 */
static uint8_t seg_turn(mf_path_ctx_t *p, const mf_seg_t *s,
                        const mf_pose_t *pose, float dt,
                        mf_body_cmd_t *out)
{
    const mf_turn_t *t = &s->p.turn;
    /* TURN_TO 是绝对角(基准系几何) -> 加基准偏差; TURN_REL 从当前位姿续起, 不加 */
    float target = (s->type == MF_SEG_TURN_TO) ? seg_cmd_yaw(t->yaw_deg)
                   : p->entry.yaw_deg + t->yaw_deg;
    float err = mf_wrap_deg(target - pose->yaw_deg);

    out->vx_mps = seg_speed_ramp(p, 0.0f, MF_DEFAULT_ACC, dt);
    out->vy_mps = 0.0f;
    out->wz_dps = mf_pid_calc(p->pid_yaw, err, dt);
    out->wz_dps = mf_clampf(out->wz_dps, (t->w_max_dps > 0.0f)
                                         ? t->w_max_dps : 1e9f);
    out->wz_dps = seg_wz_ramp(p, out->wz_dps, dt);

    return seg_dwell_reached(p, err, seg_tol_yaw(s), dt);
}

/* 点到点：方位角 P 转向 + 距离 P 前进，到位后按声明收尾转向
 * vx 经 cur_speed 斜坡衔接并按方位角投影(偏角越大前向分量越小, 防绕圈),
 * 角速度缓起, 收尾转向驻留判到位 */
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
        out->wz_dps = seg_wz_ramp(p, out->wz_dps, dt);

        vx = mf_pid_calc(p->pid_point_v, dist_err, dt);
        vx = mf_clampf(vx, (g->v_max_mps > 0.0f) ? g->v_max_mps : 1e9f);
        if (bearing > 90.0f || bearing < -90.0f)    /* 目标在身后：先原地转 */
        {
            vx = 0.0f;
        }
        else
        {
            vx *= cosf(bearing * (MF_PI_F / 180.0f));   /* 投影: 只留朝向目标分量 */
        }
        out->vx_mps = seg_speed_ramp(p, vx, g->acc_mps2, dt);
        out->vy_mps = 0.0f;

        if (dist_err < tol_d)
        {
            if (g->yaw_deg == MF_KEEP)
            {
                return 1;
            }
            p->subphase = 1;                        /* 转入收尾转向 */
            mf_pid_reset(p->pid_yaw);
            p->tol_hold_ms = 0u;
        }
        return 0;
    }
    else
    {
        dyaw = mf_wrap_deg(seg_cmd_yaw(g->yaw_deg) - pose->yaw_deg);   /* 收尾朝向: 绝对角 */
        out->vx_mps = seg_speed_ramp(p, 0.0f, g->acc_mps2, dt);
        out->vy_mps = 0.0f;
        out->wz_dps = mf_clampf(mf_pid_calc(p->pid_yaw, dyaw, dt),
                                (g->w_max_dps > 0.0f) ? g->w_max_dps : 1e9f);
        out->wz_dps = seg_wz_ramp(p, out->wz_dps, dt);
        return seg_dwell_reached(p, dyaw, tol_y, dt);
    }
}

/* 进段快照：把进入时刻位姿记下，并解算 GOTO 段的绝对目标 */
static void seg_enter(mf_path_ctx_t *p, const mf_seg_t *s,
                      const mf_pose_t *pose)
{
    const mf_goto_t *g;
    float c, s_yaw, dx, dy;

    p->entry = *pose;
#if MF_SEG_NOM_HEADING_EN
    if (p->nom_yaw_init == 0u)
    {
        /* 首段: 用实测航向给"路径标称航向"定基准, 之后纯几何累积 */
        p->nom_yaw_deg = pose->yaw_deg;
        p->nom_yaw_init = 1u;
    }
#endif
#if MF_SEG_CARRY_EN
    /* 连续累计：不清零，只记本段起点（旧律算段内进度、残差结转都要用） */
    p->dist_base_m = p->dist_accum;
#else
    p->dist_accum = 0.0f;
    p->dist_base_m = 0.0f;
#endif
    p->time_ms = 0;
    p->tol_hold_ms = 0u;
    p->subphase = 0;
    p->slip_gain = 1.0f;    /* 默认不补滑移; 只有 ARC 段改它(见 mf_config.h) */
    p->cur_wz = p->prev_wz;     /* 角速度斜坡从上拍实际输出续起, 不回零 */
    /* cur_speed 刻意不清零: 跨段保持, 段间速度衔接由各段的
     * seg_speed_ramp/抛物线减速完成 */

    switch (s->type)
    {
    case MF_SEG_MOVE_DIST:
#if MF_SEG_CARRY_EN
        /* 残差结转：目标 = 上一段累计目标 + 本段名义长度，把上一段欠走/多走的量带过来。
         * 上一段不是位移段（ARC/GOTO 之后）则重新对齐到当前里程。
         * 注意 remain 的算法也跟着换成"目标 - 当前里程"，见 seg_move_dist */
        if (p->carry_link)
        {
            p->dist_target_m += s->p.move_dist.dist_m;
        }
        else
        {
            p->dist_target_m  = p->dist_accum + s->p.move_dist.dist_m;
        }
        p->carry_link = 1u;
#endif
        /* 进段航向误差(带进来多少): 与出口残差同口径(都是在"下发角 - 实测角"上取的),
         * 于是"纠完剩多少"可直接与它对比 —— 单基准架构的现场验收判据 */
        p->line_in_yawerr_deg =
            mf_wrap_deg(seg_cmd_yaw(seg_move_hold_yaw(p, &s->p.move_dist)) - pose->yaw_deg);
        break;
    case MF_SEG_ARC:
#if MF_SEG_CARRY_EN
        p->carry_link = 0u;     /* 转弯后方向变了, 标量残差没有意义 -> 重新对齐 */
#endif
        /* 名义圆弧解算到世界系: 圆心 = 进段位姿 + R·左法线。
         * radius 带符号, 左转圆心落在车体左侧、右转落在右侧, 一条式子两向通用。
         * 用进段位姿定圆心, 等价于"车此刻正站在名义弧上且切线对齐" —— 与后续
         * 位姿误差无关, 是本段自己的参考系 */
        {
            float r = s->p.arc.radius_m;
            float yaw_rad = pose->yaw_deg * (MF_PI_F / 180.0f);

            p->arc_sgn = (s->p.arc.d_yaw_deg >= 0.0f) ? 1.0f : -1.0f;
            p->arc_r_abs = fabsf(r);
            p->arc_cx_m = pose->x_m - r * sinf(yaw_rad);
            p->arc_cy_m = pose->y_m + r * cosf(yaw_rad);
            p->arc_ey_m = 0.0f;     /* 进段必然在名义弧上 */
        }
        /* 弧线是唯一"全程都在转弯"的段, 滑移亏损也全程都在 —— 给它前馈基线 */
        p->slip_gain = MF_ARC_SLIP_GAIN;
        break;
    case MF_SEG_TURN_TO:
    case MF_SEG_TURN_REL:
#if MF_SEG_CARRY_EN
        /* 原地转向后航向变了, 旧的纵向残差在新航向下没有意义 -> 重新对齐。
         * WAIT 段不在列(它只是停等、方向不变), 所以 MOVE-WAIT-MOVE 仍会结转 */
        p->carry_link = 0u;
#endif
        break;
    case MF_SEG_GOTO_XY:
#if MF_SEG_CARRY_EN
        p->carry_link = 0u;     /* GOTO 用绝对位置自纠, 不需要标量残差 */
#endif
        p->target.x_m = s->p.goto_xy.x_m;
        p->target.y_m = s->p.goto_xy.y_m;
        break;
    case MF_SEG_GOTO_REL:       /* 车体系相对位移 -> 绝对目标 */
#if MF_SEG_CARRY_EN
        p->carry_link = 0u;
#endif
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
    uint8_t entered = 0;

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
        if (mf_path_chain(p) == 0u)
        {
            p->state = MF_PATH_DONE;
            return 0;
        }
        s = &p->path[p->idx];   /* 切表成功: 本拍即进段执行新表首段 */
    }

    if (!p->started)        /* 进段：快照 + 可选复位 PID */
    {
        p->started = 1;
        entered = 1;
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
#if MF_SEG_CARRY_EN
        /* 全程连续里程: 每拍都记, 含进段拍。进段拍的 ds 虽然由上一拍指令产生，但它是
         * 真实走过的路, 绝对里程必须记; 上一段欠走/多走的量改由 MOVE 段的"累计名义
         * 目标"(残差结转)去补, 不再用"丢一拍"做归属 */
        (void)entered;              /* 连续累计下该标记不再参与累加 */
        p->dist_accum += ds;
#else
        if (entered == 0)   /* 进段拍不计入: 此时的 ds 是上一段尾巴的位移 */
        {
            p->dist_accum += ds;    /* 弧长/里程统一由位移累计 */
        }
#endif
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
    case MF_SEG_WAIT:
        out->vx_mps = seg_speed_ramp(p, 0.0f, MF_DEFAULT_ACC, dt);
        done = (p->time_ms >= s->p.wait.duration_ms);
        break;
    default:                done = 1; break;
    }

    if (done)
    {
#if MF_SEG_NOM_HEADING_EN
        /* 标称航向随路径几何推进: ARC 累加其转角, TURN 取其目标角, 其余不变 */
        if (s->type == MF_SEG_ARC)
        {
            p->nom_yaw_deg = mf_wrap_deg(p->nom_yaw_deg + s->p.arc.d_yaw_deg);
        }
        else if (s->type == MF_SEG_TURN_TO)
        {
            p->nom_yaw_deg = mf_wrap_deg(s->p.turn.yaw_deg);
        }
        else if (s->type == MF_SEG_TURN_REL)
        {
            p->nom_yaw_deg = mf_wrap_deg(p->nom_yaw_deg + s->p.turn.yaw_deg);
        }
        else
        {
            /* MOVE / GOTO / WAIT: 标称航向不变 */
        }
#endif
        if (s->type == MF_SEG_ARC)
        {
            /* 弧线出口质量记录: 横向偏差 + 航向偏差(都是段完成这一拍的实测值)，
             * 由上层主循环取走打 ARC: 行。旧律/新律都记, 指标同源可直接 A/B。
             * 直线段(MOVE)的记录在 seg_move_dist 里落, 两条路共用同一套字段 */
            p->seg_exit_ey_m = p->arc_ey_m;
            /* 实际转角与轮程: 判"转够了没有"与"内环关到位没有"(
             * 两者的口径与陷阱见 mf_path.h 的字段注释) */
            {
                float hdg_d = mf_absf(mf_wrap_deg(pose->yaw_deg - p->entry.yaw_deg));
                float dyaw_rad = hdg_d * (MF_PI_F / 180.0f);

                p->seg_exit_dyaw_deg = hdg_d;
                p->seg_exit_radius_m = (dyaw_rad > 1.0e-3f)
                                     ? (p->dist_accum / dyaw_rad) : 0.0f;
            }
            p->seg_exit_in_yawerr_deg = 0.0f;   /* ARC 无"带进来"一说, 看出口 YE */
            /* seg_exit_yawerr_deg 由 seg_arc 每拍写, 完成那一拍的值即出口偏差 */
            p->seg_exit_seg = p->idx;
            p->seg_exit_kind = 0u;
            p->seg_exit_new = 1u;
        }
        p->idx++;
        p->started = 0;     /* 下一段下拍重新进段 */
        if (p->path[p->idx].type == MF_SEG_END)
        {
            /* 有待续表: 不清 out、不置 DONE —— 本拍沿用刚完成那段的输出过渡一拍,
             * 下拍在 tick 顶部切表并**本拍**执行新表首段。于是跨表没有零输出空窗:
             * cur_speed 连续、轮速环不被复位(mf_motion 只在 vx==wz==0 时清积分),
             * 也就没有"刹到 0 再阶跃拉回"的起步浪涌(实测 BOR 工况, 见 mf_config.h) */
            if (p->next_path == 0)
            {
                p->state = MF_PATH_DONE;
                out->vx_mps = 0.0f;
                out->vy_mps = 0.0f;
                out->wz_dps = 0.0f;
            }
        }
    }
    p->prev_wz = out->wz_dps;   /* 记录本拍最终角速度, 供下段进段做斜坡种子 */
    return done;
}

float mf_path_exit_radius_m(const mf_path_ctx_t *p)
{
    return p->seg_exit_radius_m;
}

float mf_path_exit_dyaw_deg(const mf_path_ctx_t *p)
{
    return p->seg_exit_dyaw_deg;
}

mf_path_state_t mf_path_get_state(const mf_path_ctx_t *p)
{
    return p->state;
}

uint16_t mf_path_get_seg_index(const mf_path_ctx_t *p)
{
    return p->idx;
}

uint8_t mf_path_is_waiting(const mf_path_ctx_t *p)
{
    if ((p == 0) || (p->path == 0) || (p->state != MF_PATH_RUN))
    {
        return 0u;
    }
    return (uint8_t)(p->path[p->idx].type == MF_SEG_WAIT);
}

uint8_t mf_path_pop_seg_exit(mf_path_ctx_t *p, uint16_t *seg_index, uint8_t *kind,
                             float *ey_m, float *in_yawerr_deg, float *out_yawerr_deg)
{
    if ((p == 0) || (p->seg_exit_new == 0u))
    {
        return 0u;
    }
    p->seg_exit_new = 0u;
    if (seg_index != 0)       { *seg_index       = p->seg_exit_seg; }
    if (kind != 0)            { *kind            = p->seg_exit_kind; }
    if (ey_m != 0)            { *ey_m            = p->seg_exit_ey_m; }
    if (in_yawerr_deg != 0)   { *in_yawerr_deg   = p->seg_exit_in_yawerr_deg; }
    if (out_yawerr_deg != 0)  { *out_yawerr_deg  = p->seg_exit_yawerr_deg; }
    return 1u;
}
static float seg_next_entry_speed(const mf_path_ctx_t *p)
{
    const mf_seg_t *next;

    if(p == NULL || p->path == NULL)
    {
        return -1.0f;

    }
    next = &p->path[p->idx + 1u];
    if ((next->type == MF_SEG_END) && (p->next_path != NULL))
    {
        /* 本表跑完会续跑: 末段按续表首段的速度收尾, 不按"路线终点"硬减到 0。
         * 只认续表首段自身声明的速度(不与段表脱钩), TURN_TO/WAIT/END 仍返回 0 = 真要求停 */
        next = &p->next_path[0];
    }
    switch(next->type)
    {
        case MF_SEG_MOVE_DIST: return fabsf(next->p.move_dist.speed_mps);
        case MF_SEG_ARC:       return fabsf(next->p.arc.speed_mps);

        case MF_SEG_TURN_TO:
        case MF_SEG_TURN_REL:
        case MF_SEG_WAIT:
        case MF_SEG_END:
            return 0.0f;

        case MF_SEG_GOTO_XY:
        case MF_SEG_GOTO_REL:
        default:
            /* GOTO 自带接近曲线(PID + seg_speed_ramp)，前一段不预设入口速度，
            * 避免和 GOTO 内部叠加成"双段减速" */
            return -1.0f;
    }
}

/* 剩余里程估算(见 mf_path.h) */
float mf_path_remain_m(const mf_path_ctx_t *p)
{
    uint16_t i;
    float remain = 0.0f;

    if (p == NULL || p->path == NULL || p->state != MF_PATH_RUN)
    {
        return 0.0f;
    }

    if (p->path[p->idx].type == MF_SEG_MOVE_DIST)
    {
        const mf_move_dist_t *m = &p->path[p->idx].p.move_dist;
        float done = fabsf(p->dist_accum - p->dist_base_m);   /* 本段已走量(倒车也取正) */
        float total = fabsf(m->dist_m);
        if (total > done)
        {
            remain += (total - done);
        }
    }
    else if (p->path[p->idx].type == MF_SEG_ARC)
    {
        const mf_arc_t *a = &p->path[p->idx].p.arc;
        remain += fabsf(a->d_yaw_deg) * (MF_PI_F / 180.0f) * fabsf(a->radius_m);
    }

    for (i = (uint16_t)(p->idx + 1u); p->path[i].type != MF_SEG_END; i++)
    {
        if (p->path[i].type == MF_SEG_MOVE_DIST)
        {
            remain += fabsf(p->path[i].p.move_dist.dist_m);
        }
        else if (p->path[i].type == MF_SEG_ARC)
        {
            remain += fabsf(p->path[i].p.arc.d_yaw_deg) * (MF_PI_F / 180.0f)
                      * fabsf(p->path[i].p.arc.radius_m);
        }
    }
    return remain;
}
