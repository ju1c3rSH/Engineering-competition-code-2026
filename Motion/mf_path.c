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
    p->arc_remain_prev_deg = 0.0f;
    p->arc_wheel_deg    = 0.0f;
    p->arc_dead_deg     = 0.0f;
    p->arc_dead_ticks   = 0u;
    p->arc_wzsp_peak    = 0.0f;
    p->arc_soft_min     = 1.0f;
    p->arc_prev_yaw_deg = 0.0f;
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
    p->frame_base_deg = 0.0f;
    p->ref_seq = 0u;
    p->line_hdg_deg = 0.0f;
    p->obs_target_deg = 0.0f;
    p->obs_yawerr_deg = 0.0f;
    p->obs_ey_m = 0.0f;
    p->obs_eyg_m = 0.0f;
    p->obs_xt_corr_deg = 0.0f;
    p->obs_kind = 0u;
    p->v_meas_f = 0.0f;     /* 实测速度源: 从 0 起滤, 弧首拍由下限钳位兜住 */
    /* 名义位置链 + 直线横向环（MF_LINE_XTRACK_EN） */
    p->nom_x_m = 0.0f;
    p->nom_y_m = 0.0f;
    p->nom_seg_x_m = 0.0f;
    p->nom_seg_y_m = 0.0f;
    p->nom_seg_hdg_deg = 0.0f;
    p->nom_pos_init = 0u;
    p->xt_ey_f = 0.0f;
    p->xt_corr_deg = 0.0f;
    p->seg_exit_eyg_m = 0.0f;
    p->seg_exit_xt_deg = 0.0f;
    p->seg_ms = 0u;
    p->seg_exit_ms = 0u;
    p->xt_peak_deg = 0.0f;
    p->eyg_peak_m = 0.0f;
    p->eyg_at_p75_m = 0.0f;
    p->eyg_at_p50_m = 0.0f;
    p->eyg_at_p25_m = 0.0f;
    p->xt_over_ticks = 0u;
    p->eyg_prog_mask = 0u;
}

/* 整表重锚基点(不变量 P5, 见 mf_path.h): 表里绝对角与 nom_yaw 在路线系(置零车头=0),
 * pose.yaw 是读数域, 换算只许走这里。MF_HEADING_FRAME_EN=0 时不平移(=旧行为) */
static float seg_pose_yaw(const mf_path_ctx_t *p, const mf_pose_t *pose)
{
#if MF_HEADING_FRAME_EN
    return mf_wrap_deg(pose->yaw_deg - p->frame_base_deg);
#else
    (void)p;
    return pose->yaw_deg;
#endif
}

/* 按拍观测: 各段分支把"环里真正用的那几个值"写进来, 调试口按拍取走。
 * 纯只读量, 不参与任何控制判断 */
static void seg_obs(mf_path_ctx_t *p, float target_deg, float yawerr_deg,
                    float ey_m, uint8_t kind)
{
    p->obs_target_deg = mf_wrap_deg(target_deg);
    p->obs_yawerr_deg = yawerr_deg;
    p->obs_ey_m = ey_m;
    p->obs_kind = kind;
}

/* 整表重锚: 基点是"置零那一刻的读数", 表里的绝对角从此解释在路线系(置零车头 = 0)。
 * 缺这一步时表里的绝对角会钉在上电那一帧, yaw0≠0 就让首个绝对段之后全偏 */
void mf_path_set_heading_ref(mf_path_ctx_t *p, float yaw_deg)
{
#if MF_HEADING_FRAME_EN
    p->frame_base_deg = mf_wrap_deg(yaw_deg);
    p->nom_yaw_deg = 0.0f;          /* 路线系零点 = 此刻车头朝向 */
#else
    p->frame_base_deg = 0.0f;       /* A/B: 无基点, 表里的绝对角 = 读数域 */
    p->nom_yaw_deg = mf_wrap_deg(yaw_deg);
#endif
    p->nom_yaw_init = 1u;
    p->heading_ref_locked = 1u;
    p->ref_seq++;
    /* 名义位置链同样重定锚: 下一次进段用当时的实测位姿当基准(置零那拍位姿=原点) */
    p->nom_pos_init = 0u;
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
    /* 单基准: 基准由 mf_path_set_heading_ref() 在 GYRO_ZERO 时钉下, 跨表不重定锚;
     * 未钉下(自测无 GYRO_ZERO)退回"首段用实测航向定基准", 与旧行为等价 */
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
#if MF_LINE_XTRACK_EN
    /* 与 nom_yaw 同规则: 基准已由 GYRO_ZERO 显式钉下时, 跨表**不**重定锚 ——
     * 否则每换一条 nav 表名义链都从当前位姿重新起算, 段间累积的横向误差又看不见了 */
  #if MF_HEADING_FRAME_EN
    if (p->heading_ref_locked == 0u)
    {
        p->nom_pos_init = 0u;
    }
  #else
    p->nom_pos_init = 0u;
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
    p->v_meas_f = 0.0f;     /* 停稳即视为 v̂=0, 新路径重新起滤 */
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

/* 期望角(基准系几何) -> 实际下发角: 只差固定基准偏差 MF_HEADING_BIAS_DEG, 全工程只在这一处加
 * (段表全是几何量、补偿只有一个数, 见 mf_config.h)。只作用于**绝对角目标**; 相对角与 ARC
 * 自解算不加 —— 它们从当前位姿续起, 天然继承整条路线的刚性旋转 */
static float seg_cmd_yaw(float geom_deg)
{
    return geom_deg + MF_HEADING_BIAS_DEG;
}

/* MOVE 段的航向保持目标（路线系）: 段表给了绝对角就优先用它; MF_KEEP 时按开关取标称航向
 * 或进段实测航向(后者是读数域, 要换到路线系) */
static float seg_move_hold_yaw(const mf_path_ctx_t *p, const mf_move_dist_t *m)
{
    if (m->heading_deg != MF_KEEP)
    {
        return m->heading_deg;
    }
#if MF_SEG_NOM_HEADING_EN
    return p->nom_yaw_deg;
#else
    return seg_pose_yaw(p, &p->entry);
#endif
}

/* 直线段横向偏差: 名义线 = 进段点 + 本段目标航向(读数域), 正 = 名义线左侧。
 * 用途是把"长直线走偏"分成两种: 沿行程基本不变 = 整段平行挪出去; 线性增长 = 越走越斜 */
static float seg_line_ey(const mf_path_ctx_t *p, const mf_pose_t *pose)
{
    float h = p->line_hdg_deg * (MF_PI_F / 180.0f);
    float dx = pose->x_m - p->entry.x_m;
    float dy = pose->y_m - p->entry.y_m;

    return (dy * cosf(h)) - (dx * sinf(h));
}

#if MF_LINE_XTRACK_EN
/* 直线段横向偏差(名义路径基准): 基准线过**本段名义起点**, 方向 = 本段名义航向(x/y 域)。
 * 与 seg_line_ey 的唯一区别就是基准点 —— entry 是实测位姿(每段归零), nom_seg 是几何点(跨段连续) */
static float seg_line_ey_nom(const mf_path_ctx_t *p, const mf_pose_t *pose)
{
    float h = p->nom_seg_hdg_deg * (MF_PI_F / 180.0f);
    float dx = pose->x_m - p->nom_seg_x_m;
    float dy = pose->y_m - p->nom_seg_y_m;

    return (dy * cosf(h)) - (dx * sinf(h));     /* 正 = 名义线左侧 */
}

/* 直线横向环单拍: 输入相对名义线的 e_y(m, 正=左) 与本段剩余里程, 返回本拍航向偏置(deg)。
 * 输出叠进**航向环的误差**, 不是 wz —— 见 mf_config.h 的形式说明 */
static float seg_line_xtrack_step(mf_path_ctx_t *p, float ey, float v, float dt, float remain)
{
    float tgt, e, d;

    /* τ=50ms 一阶低通: 挡履带过齿/单拍打滑尖峰; 对 λ≤1.2 只加 ~3° 相位 */
    p->xt_ey_f += (ey - p->xt_ey_f) * (dt / (MF_LINE_XTRACK_LPF_TAU_S + dt));

    /* 死区: 编码器量化 0.1~0.3mm/拍, 小抖动不出力(平滑处理, 不在阈值上开关) */
    e = p->xt_ey_f;
    if (e > MF_LINE_XTRACK_DEAD_M)
    {
        e -= MF_LINE_XTRACK_DEAD_M;
    }
    else if (e < -MF_LINE_XTRACK_DEAD_M)
    {
        e += MF_LINE_XTRACK_DEAD_M;
    }
    else
    {
        e = 0.0f;
    }

    if ((v >= MF_LINE_XTRACK_V_MIN) && (mf_absf(p->xt_ey_f) <= MF_LINE_XTRACK_EY_WIN_M))
    {
        /* e_y>0 = 偏左 ⇒ 偏置取负 ⇒ 右打。小偏差下 ė_y = -v·k·e/(k_soft+v) */
        tgt = -atanf(MF_LINE_XTRACK_K * e / (MF_LINE_XTRACK_K_SOFT + v))
            * (180.0f / MF_PI_F);
    }
    else
    {
        /* 窗外/低速/倒车: 平滑退出, 不阶跃。但"有速度却超窗"是**环整段不工作**的工况
         * (换图打滑后弧出口一次带进 150~200mm 就是这样), 必须留痕迹, 否则静默失效 */
        if ((v >= MF_LINE_XTRACK_V_MIN) && (mf_absf(p->xt_ey_f) > MF_LINE_XTRACK_EY_WIN_M))
        {
            p->xt_over_ticks++;
        }
        tgt = 0.0f;
    }
    if (tgt > MF_LINE_XTRACK_CORR_MAX_DEG)
    {
        tgt = MF_LINE_XTRACK_CORR_MAX_DEG;
    }
    if (tgt < -MF_LINE_XTRACK_CORR_MAX_DEG)
    {
        tgt = -MF_LINE_XTRACK_CORR_MAX_DEG;
    }

    /* 斜率限: 段首带大偏差时也不许一拍拧到位(D 项会看到阶跃)。
     * 正常工况偏置本身只有 0.5° 量级, 变化率 ~0.3°/s, 本限只是护栏 */
    d = tgt - p->xt_corr_deg;
    if (d > (MF_LINE_XTRACK_SLEW_DPS * dt))
    {
        d = MF_LINE_XTRACK_SLEW_DPS * dt;
    }
    if (d < -(MF_LINE_XTRACK_SLEW_DPS * dt))
    {
        d = -(MF_LINE_XTRACK_SLEW_DPS * dt);
    }
    p->xt_corr_deg += d;

    /* 段末收偏置: 段末残留的偏置就是"带进下一段的航向误差", 而下一段若是弧, 按 R·Δθ 放大成
     * **位置**误差(1°@942mm = 16mm)。所以这里用**乘法**衰减(保证段末严格为 0) ——
     * 不能交给斜率限去收: corr 稍大就来不及, 段末会留半度 */
    if (remain < MF_LINE_XTRACK_RAMP_M)
    {
        p->xt_corr_deg *= (remain > 0.0f) ? (remain / MF_LINE_XTRACK_RAMP_M) : 0.0f;
    }

    return p->xt_corr_deg;
}

/* 名义位置链推进(段完成那一拍调用, 必须在 nom_yaw_deg 更新**之前**):
 * 纯段表几何, 不含任何测量。方向换算 = 几何 + MF_HEADING_BIAS_DEG + frame_base ——
 * 与 line_hdg_deg 同口径: 基准偏差是**下发航向的整体旋转**, 名义线必须跟着转, 否则
 * 环会把"故意加的补偿"当成真实横偏去纠(等于自废 bias), 这就是"别用 POSE 的 Y 反解"那条 */
static float seg_nom_hdg_xy(const mf_path_ctx_t *p, float route_deg)
{
    return (route_deg + MF_HEADING_BIAS_DEG + p->frame_base_deg) * (MF_PI_F / 180.0f);
}

static void seg_nom_advance(mf_path_ctx_t *p, const mf_seg_t *s)
{
    float h, d, ch, a;

    switch (s->type)
    {
    case MF_SEG_MOVE_DIST:
        h = (s->p.move_dist.heading_deg != MF_KEEP) ? s->p.move_dist.heading_deg
                                                    : p->nom_yaw_deg;
        h = seg_nom_hdg_xy(p, h);
        p->nom_x_m += s->p.move_dist.dist_m * cosf(h);
        p->nom_y_m += s->p.move_dist.dist_m * sinf(h);
        break;
    case MF_SEG_ARC:
        /* 弦长 2|R|sin(|Δ|/2), 方向 = 起点名义航向 + Δ/2 (左右通用) */
        d  = s->p.arc.d_yaw_deg * (MF_PI_F / 180.0f);
        ch = 2.0f * mf_absf(s->p.arc.radius_m) * sinf(0.5f * mf_absf(d));
        a  = seg_nom_hdg_xy(p, p->nom_yaw_deg + 0.5f * s->p.arc.d_yaw_deg);
        p->nom_x_m += ch * cosf(a);
        p->nom_y_m += ch * sinf(a);
        break;
    case MF_SEG_GOTO_XY:
    case MF_SEG_GOTO_REL:
        /* GOTO 用绝对位置自纠(误差本来就归零) ⇒ 名义链直接跟到目标, 不参与横向环 */
        p->nom_x_m = p->target.x_m;
        p->nom_y_m = p->target.y_m;
        break;
    case MF_SEG_TURN_TO:
    case MF_SEG_TURN_REL:
        /* 原地转不推进位置(只改 nom_yaw, 由 mf_path_tick 那段负责)。
         * 物理上原地转会有几毫米的轴心偏移 —— 那正是我们**想看见**的误差, 留给下一段的环 */
        break;
    default:
        break;
    }
}
#endif /* MF_LINE_XTRACK_EN */

/* 段内进度采样(剩余 75/50/25% 各一次): 段末只有一个数, 分不开"段首台阶"与"全程漂移"。
 * 倒车段 remain 按行进方向算, 与 len=|dist| 同号, 门限通用 */
#if MF_LINE_XTRACK_EN
static void seg_line_prog(mf_path_ctx_t *p, float eyg, float remain, float len)
{
    if ((p->eyg_prog_mask & 0x01u) == 0u && (remain <= (0.75f * len)))
    {
        p->eyg_at_p75_m = eyg;
        p->eyg_prog_mask |= 0x01u;
    }
    if ((p->eyg_prog_mask & 0x02u) == 0u && (remain <= (0.50f * len)))
    {
        p->eyg_at_p50_m = eyg;
        p->eyg_prog_mask |= 0x02u;
    }
    if ((p->eyg_prog_mask & 0x04u) == 0u && (remain <= (0.25f * len)))
    {
        p->eyg_at_p25_m = eyg;
        p->eyg_prog_mask |= 0x04u;
    }
}
#endif /* MF_LINE_XTRACK_EN */

/* 增量直线：航向保持 + 剩余行程判完成
 * 下一段要求停稳时按 v=sqrt(2·a·s) 抛物线减速, 段末速度≈0 平滑交棒 */
static uint8_t seg_move_dist(mf_path_ctx_t *p, const mf_seg_t *s,
                             const mf_pose_t *pose, float dt,
                             mf_body_cmd_t *out)
{
    const mf_move_dist_t *m = &s->p.move_dist;
    float dir, remain, acc, cruise, target, v_stop,v_next, err;
    float ey;

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

    err = mf_wrap_deg(seg_cmd_yaw(seg_move_hold_yaw(p, m)) - seg_pose_yaw(p, pose));
    ey = seg_line_ey(p, pose);
#if MF_LINE_XTRACK_EN
    /* 名义线横向闭环: 偏置叠进**航向误差**, 不进 wz(形式与理由见 mf_config.h)。
     * 用带符号的 out->vx_mps: 倒车段自动被 v<V_MIN 挡掉(Stanley 的收敛性只对正向成立) */
    {
        float eyg = seg_line_ey_nom(p, pose);
        float xt  = seg_line_xtrack_step(p, eyg, out->vx_mps, dt, remain);

        err = mf_wrap_deg(err + xt);
        p->obs_eyg_m = eyg;
        p->obs_xt_corr_deg = xt;
        /* 段内统计: 段末 XT 结构性为 0, 只有峰值能说明"环干过多少活/顶没顶限幅" */
        if (mf_absf(xt) > p->xt_peak_deg)
        {
            p->xt_peak_deg = mf_absf(xt);
        }
        if (mf_absf(eyg) > p->eyg_peak_m)
        {
            p->eyg_peak_m = mf_absf(eyg);
        }
        seg_line_prog(p, eyg, remain, mf_absf(m->dist_m));
    }
#endif
    out->wz_dps = mf_pid_calc(p->pid_yaw, err, dt);
    seg_obs(p, seg_cmd_yaw(seg_move_hold_yaw(p, m)), err, ey, 1u);

    /* 失控自保 (test/motion-tuning): 编码器/驱动符号矛盾时反馈为负, remain 沿
     * 正向滚出 3 倍目标距离 —— 强制结束本段, 防正反馈无限狂转(实测 BOR 死机工况) */
    if (remain > (3.0f * ((m->dist_m < 0.0f) ? -m->dist_m : m->dist_m) + 0.5f))
    {
        return 1u;
    }
    if (remain <= (0.5f * mf_absf(out->vx_mps) * dt))
    {
        /* 到位余量同上: = 本拍步长的一半(运行期), 不是常数 —— 见 mf_config.h 的推导 */
        /* 直线段出口记录: IE=进段航向误差 / YE=出口残差, 单基准架构下 IE 才非零
         * (旧行为 IE 恒 0) —— 这一对数是"新旧架构有没有生效"的直接判据 */
        p->seg_exit_ey_m = ey;      /* 直线: 相对本段名义线的横向偏差(本段自己的漂移量) */
        p->seg_exit_in_yawerr_deg = p->line_in_yawerr_deg;
        p->seg_exit_yawerr_deg = err;
        p->seg_exit_seg = p->idx;
        p->seg_exit_kind = 1u;
        p->seg_exit_new = 1u;
        p->seg_exit_ms = p->seg_ms;
#if MF_LINE_XTRACK_EN
        p->seg_exit_eyg_m = p->obs_eyg_m;       /* 相对名义路径(跨段连续) */
        p->seg_exit_xt_deg = p->xt_corr_deg;    /* 段末收偏置后应≈0 */
#endif
        return 1u;
    }
    return 0u;
}

/* 弧线 ω 前馈取哪个速度。取实测(v̂)时 ω_sp = v̂/R, 于是 R_true = v̂·η/(v̂/R·TRIM) 里车速约掉
 * ⇒ 半径与电池/饱和/地面无关, 代价只是弧走慢。钳位理由见 mf_config.h 的 MF_ARC_VSRC_* */
static float seg_arc_vsrc(const mf_path_ctx_t *p)
{
#if MF_ARC_VSRC_MEAS
    {
        float ref = mf_absf(p->cur_speed);
        float mag = mf_absf(p->v_meas_f);

        if (ref > 0.05f)
        {
            float lo = ref * MF_ARC_VSRC_MIN_MULT;
            float hi = ref * MF_ARC_VSRC_MAX_MULT;

            if (mag < lo) { mag = lo; }
            if (mag > hi) { mag = hi; }
        }
        return (p->cur_speed >= 0.0f) ? mag : -mag;
    }
#else
    return p->cur_speed;
#endif
}

/* 弧线几何前馈 wz_ff = v/R × 径向 trim。trim 见 mf_config.h —— 它是**物理半径的唯一旋钮**
 * (基线/内环/横向项全被速率内环吸收, 改不动稳态半径)。三处用同一个式子, 不许各写一份 */
static float seg_arc_wz_ff(const mf_path_ctx_t *p, const mf_arc_t *a)
{
    return (seg_arc_vsrc(p) / a->radius_m) * (180.0f / MF_PI_F) * MF_ARC_WZ_TRIM;
}

/* 定半径弧线: 走够 d_yaw 即完成(判据只用 IMU 转角); 设定值取"名义切线航向"
 * = 圆心→车体方位角 + sgn·90°(由里程计位置推出), 再加横向偏差比例项。旧律对比见 Motion/README.md */
static uint8_t seg_arc(mf_path_ctx_t *p, const mf_seg_t *s,
                       const mf_pose_t *pose, float ds, float dt,
                       mf_body_cmd_t *out)
{
    const mf_arc_t *a = &s->p.arc;
    float acc, target, v_stop, v_next;
    float yaw_done, remain_yaw, remain_m, radius_abs;
    float step_deg, tol_deg;

    v_next = seg_next_entry_speed(p);
    acc = (a->acc_mps2 > 0.0f) ? a->acc_mps2 : MF_DEFAULT_ACC;
    yaw_done = pose->yaw_deg - p->entry.yaw_deg;
    remain_yaw = (a->d_yaw_deg - yaw_done)
               * ((a->d_yaw_deg >= 0.0f) ? 1.0f : -1.0f);  /* 剩余转角, 正=未走完 */
    target = (a->speed_mps < 0.0f) ? -a->speed_mps : a->speed_mps;

    /* 到位余量 = 本段**抽样步长**的一半, 运行期量出(推导见 mf_config.h): 优先实测(上拍到本拍
     * remain 减少量), 首拍/无新样退名义 (v/R)·dt。都不许出现常数 —— 换档/改拍自动跟上 */
    {
        float nom = mf_absf(seg_arc_wz_ff(p, a)) * dt;

        step_deg = p->arc_remain_prev_deg - remain_yaw;
        if (step_deg < (0.5f * nom))
        {
            step_deg = nom;
        }
    }
    p->arc_remain_prev_deg = remain_yaw;
    tol_deg = 0.5f * step_deg;

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

#if MF_ARC_ENTRY_SOFT_EN
    /* 弧入口软启动: 本段前 LEN 米把**速度**乘 MULT→1 线性放开。wz_ff=v/R 跟着 v 一起缩
     * ⇒ R 与几何都不变, 只是"破静摩擦那一刻的差速需求"变小 ⇒ 空转窗口 DD 变小(见 mf_config.h)。
     * 用本段已走轮程判窗口: dist_accum/dist_base_m 在 CARRY 开关的两种口径下都成立。
     * 放在 creep 下限**之前**: 近完成时窗口早已过去(mult 已回 1), 不会与下限打架 */
    {
        float s_in = mf_absf(p->dist_accum - p->dist_base_m);
        float mult = 1.0f;

        if (s_in < MF_ARC_ENTRY_SOFT_LEN_M)
        {
            mult = MF_ARC_ENTRY_SOFT_MULT
                 + (1.0f - MF_ARC_ENTRY_SOFT_MULT) * (s_in / MF_ARC_ENTRY_SOFT_LEN_M);
        }
        target *= mult;
        if (mult < p->arc_soft_min)     /* 观测: 本弧降到过多少(1.0=没动作), ARC: 的 SE 位 */
        {
            p->arc_soft_min = mult;
        }
    }
#endif
    /* 没走完就不许停: v_stop 会把目标减到 0 而 remain_yaw 还大于到位余量 ⇒ 永远走不完。
     * 见 MF_ARC_CREEP_MPS(用运行期比较而非 #if: 预处理器不认浮点字面量) */
    if ((MF_ARC_CREEP_MPS > 0.0f) && (remain_yaw > tol_deg)
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

        /* ---- delta 的机上直读: 轮子折算转角 vs IMU 实测转角 ----
         * 空转窗口内轮子走 ds 而车体不转 ⇒ want>0 且 got~0, want-got 累计就是 delta/R。
         * ds 与 dist_accum 同源(本段轮程); 进段拍的上一段尾巴已由调用方剔成 0。 */
        {
            float want = (mf_absf(ds) / p->arc_r_abs) * (180.0f / MF_PI_F);
            float got  = mf_absf(mf_wrap_deg(pose->yaw_deg - p->arc_prev_yaw_deg));

            p->arc_wheel_deg += want;
            if ((want > 0.02f) && ((want - got) > (0.5f * want)))
            {
                p->arc_dead_deg += (want - got);
                p->arc_dead_ticks++;
            }
            p->arc_prev_yaw_deg = pose->yaw_deg;
        }
        p->seg_exit_yawerr_deg = e_psi;         /* 出口记录取段完成那一拍的实测值 */
        /* 按拍观测(路线系): 弧的"目标航向"就是本拍的名义切线 */
        seg_obs(p, seg_pose_yaw(p, pose) + e_psi, e_psi, p->arc_ey_m, 0u);

        /* 曲率前馈 wz_ff = v/R(带符号 radius, 左+/右-): 稳态角速度开环直给, PID 只修打滑
         * 残差(纯P需滞后~3.6°才出稳态差速=进弯晚的根因); 速度源用 cur_speed, 起步/减速自动
         * 缩放。勿套 seg_wz_ramp —— 进弯缓起=进弯前固定多走直线(实测 10~30cm) */
#if MF_ARC_XTRACK_EN
        /* 横向项**必须乘 arc_sgn**(左+1/右-1): e_y 恒正=弧外不分左右, 但"回弧内往哪边打"分
         * 左右 —— 漏乘把右转弧推向弧外(实测右转 EY +103 vs 左转 +77); arc_xtrack_sim.py 只
         * 有左转用例, 符号错仿真不暴露, 改仿真补右转。本项只当微调, 半径主责归速率内环 */
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

        /* 弧线自限幅: 三项叠加(航向P + v/R + 横向K·e_y)可顶到 cfg.max_wz_dps, 那是
         * R≈0.27m 的原地上转 —— 履带横向刮擦做不到, 只会把车钉住、SP 钉限幅十几秒而弧走不完
         * (2026-09-30 实测)。上限定在几何前馈的 MF_ARC_WZ_MAX_MULT 倍, 最紧只做到 R/该倍数 */
        {
            float wz_ff  = seg_arc_wz_ff(p, a);
            float wz_lim = mf_absf(wz_ff) * MF_ARC_WZ_MAX_MULT;

            if (wz_lim > 1.0f)
            {
                out->wz_dps = mf_clampf(out->wz_dps, wz_lim);
            }
        }

        /* 本弧 |wz_sp| 峰值: 顶到 |wz_ff|*MF_ARC_WZ_MAX_MULT 说明段内自限幅在参与 */
        if (mf_absf(out->wz_dps) > p->arc_wzsp_peak)
        {
            p->arc_wzsp_peak = mf_absf(out->wz_dps);
        }
    }

    return (uint8_t)(remain_yaw <= tol_deg);
}

/* 原地转向（绝对/相对统一）: 前向速度斜坡归零再转, 角速度缓起, 驻留判到位 */
static uint8_t seg_turn(mf_path_ctx_t *p, const mf_seg_t *s,
                        const mf_pose_t *pose, float dt,
                        mf_body_cmd_t *out)
{
    const mf_turn_t *t = &s->p.turn;
    /* TURN_TO 是路线系的绝对角(要加基准偏差); TURN_REL 从进段时刻的**路线系**航向续起,
     * 与基点无关(它天然继承整表重锚) */
    float target = (s->type == MF_SEG_TURN_TO) ? seg_cmd_yaw(t->yaw_deg)
                   : mf_wrap_deg(seg_pose_yaw(p, &p->entry) + t->yaw_deg);
    float err = mf_wrap_deg(target - seg_pose_yaw(p, pose));

    out->vx_mps = seg_speed_ramp(p, 0.0f, MF_DEFAULT_ACC, dt);
    out->vy_mps = 0.0f;
    out->wz_dps = mf_pid_calc(p->pid_yaw, err, dt);
    out->wz_dps = mf_clampf(out->wz_dps, (t->w_max_dps > 0.0f)
                                         ? t->w_max_dps : 1e9f);
    out->wz_dps = seg_wz_ramp(p, out->wz_dps, dt);
    seg_obs(p, target, err, 0.0f, (uint8_t)((s->type == MF_SEG_TURN_TO) ? 2u : 3u));

    return seg_dwell_reached(p, err, seg_tol_yaw(s), dt);
}

/* 点到点: 方位角 P 转向 + 距离 P 前进, 到位后按声明收尾转向;
 * vx 按方位角投影(偏角越大前向分量越小, 防绕圈), 角速度缓起, 收尾转向驻留判到位 */
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
        seg_obs(p, seg_pose_yaw(p, pose) + bearing, bearing, 0.0f, 4u);

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
        dyaw = mf_wrap_deg(seg_cmd_yaw(g->yaw_deg) - seg_pose_yaw(p, pose));  /* 收尾朝向: 路线系绝对角 */
        out->vx_mps = seg_speed_ramp(p, 0.0f, g->acc_mps2, dt);
        out->vy_mps = 0.0f;
        out->wz_dps = mf_clampf(mf_pid_calc(p->pid_yaw, dyaw, dt),
                                (g->w_max_dps > 0.0f) ? g->w_max_dps : 1e9f);
        out->wz_dps = seg_wz_ramp(p, out->wz_dps, dt);
        seg_obs(p, seg_cmd_yaw(g->yaw_deg), dyaw, 0.0f, 4u);
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
#if MF_LINE_XTRACK_EN
    /* 名义链: 首次进段(或刚 YAWZERO 之后)拿实测位姿定基准, 之后只在段完成时按几何推进 */
    if (p->nom_pos_init == 0u)
    {
        p->nom_x_m = pose->x_m;
        p->nom_y_m = pose->y_m;
        p->nom_pos_init = 1u;
    }
    p->nom_seg_x_m = p->nom_x_m;
    p->nom_seg_y_m = p->nom_y_m;
    p->seg_ms = 0u;         /* 段时长从进段起算(段均速 = 段长/本值) */
    p->xt_ey_f = 0.0f;      /* 进段从 0 起滤: 偏置自然软起(上一段末已收到 0) */
    p->xt_corr_deg = 0.0f;
    p->obs_eyg_m = 0.0f;    /* 非直线段没有这条量(ARC 用 arc_ey_m) */
    p->obs_xt_corr_deg = 0.0f;
    p->xt_peak_deg = 0.0f;  /* 段内统计(见 mf_path_get_line_diag) */
    p->eyg_peak_m = 0.0f;
    p->eyg_at_p75_m = 0.0f;
    p->eyg_at_p50_m = 0.0f;
    p->eyg_at_p25_m = 0.0f;
    p->xt_over_ticks = 0u;
    p->eyg_prog_mask = 0u;
#endif
#if MF_SEG_NOM_HEADING_EN
    if (p->nom_yaw_init == 0u)
    {
        /* 首段: 用实测航向给"路径标称航向"定基准(路线系), 之后纯几何累积 */
        p->nom_yaw_deg = seg_pose_yaw(p, pose);
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
        /* 残差结转: 目标 = 上一段累计目标 + 本段名义长度, 把上一段欠走/多走的量带过来;
         * 上一段不是位移段则重新对齐到当前里程。remain 算法跟着换成"目标-当前里程"(见 seg_move_dist) */
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
        /* 进段航向误差(带进来多少): 与出口残差同口径(都是"下发角-实测角"), "纠完剩多少"
         * 可直接对比 —— 单基准架构的现场验收读数。同时存下本段名义线方向(换回读数域) */
        {
            float h = seg_cmd_yaw(seg_move_hold_yaw(p, &s->p.move_dist));

            p->line_in_yawerr_deg = mf_wrap_deg(h - seg_pose_yaw(p, pose));
            p->line_hdg_deg = mf_wrap_deg(h + p->frame_base_deg);
#if MF_LINE_XTRACK_EN
            /* 名义线方向与 line_hdg_deg 完全同口径(几何 + bias + 基点): 两条线只差基准点 */
            p->nom_seg_hdg_deg = p->line_hdg_deg;
#endif
        }
        break;
    case MF_SEG_ARC:
#if MF_SEG_CARRY_EN
        p->carry_link = 0u;     /* 转弯后方向变了, 标量残差没有意义 -> 重新对齐 */
#endif
        /* 名义圆弧解算到世界系: 圆心 = 进段位姿 + R·左法线。radius 带符号, 左转圆心在车体
         * 左侧、右转在右侧, 一条式子两向通用。用进段位姿定圆心 = "车此刻正站在名义弧上且
         * 切线对齐", 是本段自己的参考系, 与后续位姿误差无关 */
        {
            float r = s->p.arc.radius_m;
            float yaw_rad = pose->yaw_deg * (MF_PI_F / 180.0f);

            p->arc_sgn = (s->p.arc.d_yaw_deg >= 0.0f) ? 1.0f : -1.0f;
            p->arc_r_abs = fabsf(r);
            p->arc_cx_m = pose->x_m - r * sinf(yaw_rad);
            p->arc_cy_m = pose->y_m + r * cosf(yaw_rad);
            p->arc_ey_m = 0.0f;     /* 进段必然在名义弧上 */
            /* 首拍的"上一拍 remain" = 全程剩余 ⇒ 第一拍的步长 = 本拍实际转角 */
            p->arc_remain_prev_deg = s->p.arc.d_yaw_deg;
            /* 空转窗口观测清零(见 mf_path.h 结构体注释) */
            p->arc_wheel_deg    = 0.0f;
            p->arc_dead_deg     = 0.0f;
            p->arc_dead_ticks   = 0u;
            p->arc_wzsp_peak    = 0.0f;
            p->arc_soft_min     = 1.0f;
            p->arc_prev_yaw_deg = pose->yaw_deg;
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
    p->seg_ms   = (uint16_t)(p->seg_ms + (uint16_t)(dt * 1000.0f + 0.5f));

    /* 实测前向速度(弧线 ω 前馈的速度源, MF_ARC_VSRC_MEAS): ds/dt 逐拍差分噪声大, 一阶低通。
     * 必须在段分支**之前**更新 —— 段内读到的才是本拍值。dt 恒为 MF_DT_S(>0), 无需防零 */
    {
        float tau = 1.0f / (6.2831853f * MF_ARC_VSRC_LPF_HZ);
        float alpha = dt / (tau + dt);

        p->v_meas_f += ((ds / dt) - p->v_meas_f) * alpha;
    }

    switch (s->type)
    {
    case MF_SEG_MOVE_DIST:
    case MF_SEG_ARC:
#if MF_SEG_CARRY_EN
        /* 全程连续里程: 每拍都记, 含进段拍 —— 进段拍的 ds 是真实走过的路, 绝对里程必须记;
         * 上一段欠走/多走改由 MOVE 段的"累计名义目标"(残差结转)补, 不用"丢一拍"做归属 */
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
    case MF_SEG_ARC:        done = seg_arc(p, s, pose,
                                           (entered != 0u) ? 0.0f : ds, dt, out); break;
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
#if MF_LINE_XTRACK_EN
        /* 名义位置链推进: 必须在下面 nom_yaw 更新**之前**(MOVE/ARC 的推进方向取的是
         * 本段**起点**名义航向, 更新后取就晚了一段)。纯几何, 不含任何测量 */
        seg_nom_advance(p, s);
#endif
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
            /* 弧线出口质量记录: 横向偏差 + 航向偏差(段完成这一拍的实测值), 由上层取走打
             * ARC: 行; 旧律/新律都记, 指标同源可直接 A/B。直线段记录在 seg_move_dist 里落 */
            p->seg_exit_ey_m = p->arc_ey_m;
            /* 实际转角与轮程: 判"转够了没有"与"内环关到位没有"(口径与陷阱见 mf_path.h) */
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
            p->seg_exit_ms = p->seg_ms;
        }
        p->idx++;
        p->started = 0;     /* 下一段下拍重新进段 */
        if (p->path[p->idx].type == MF_SEG_END)
        {
            /* 有待续表: 不清 out、不置 DONE —— 本拍沿用刚完成那段的输出过渡一拍, 下拍切表
             * 并**本拍**执行新表首段 ⇒ 跨表没有零输出空窗(cur_speed 连续、轮速环不被复位,
             * 无"刹到0再阶跃拉回"的起步浪涌, 实测 BOR 工况, 见 mf_config.h) */
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

float mf_path_exit_eyg_m(const mf_path_ctx_t *p)
{
    return p->seg_exit_eyg_m;
}

float mf_path_exit_xt_deg(const mf_path_ctx_t *p)
{
    return p->seg_exit_xt_deg;
}

uint16_t mf_path_exit_ms(const mf_path_ctx_t *p)
{
    return p->seg_exit_ms;
}

void mf_path_get_line_diag(const mf_path_ctx_t *p, float *peak_deg, float *exc_m,
                           uint16_t *over_ticks)
{
    if (peak_deg != 0)   { *peak_deg   = p->xt_peak_deg; }
    if (exc_m != 0)      { *exc_m      = p->eyg_peak_m; }
    if (over_ticks != 0) { *over_ticks = p->xt_over_ticks; }
}

void mf_path_get_line_prog(const mf_path_ctx_t *p, float *p75, float *p50, float *p25)
{
    if (p75 != 0) { *p75 = ((p->eyg_prog_mask & 0x01u) != 0u) ? p->eyg_at_p75_m : 0.0f; }
    if (p50 != 0) { *p50 = ((p->eyg_prog_mask & 0x02u) != 0u) ? p->eyg_at_p50_m : 0.0f; }
    if (p25 != 0) { *p25 = ((p->eyg_prog_mask & 0x04u) != 0u) ? p->eyg_at_p25_m : 0.0f; }
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
#if MF_ARC_ENTRY_SOFT_EN
        case MF_SEG_ARC:
            /* 弧入口软启动的**前半**: 让**前一段**就按 MULT×弧速收尾。
             * 只缩弧段自己来不及 —— 减速度被 acc 限着, 破静摩擦那 80ms 里速度根本没降下来
             * (弧段内部只是把前 LEN 米线性放开到全速, 见 seg_arc)。
             * 代价: 弧前的那一段要提前减速, 32U→ARC10 这种"高速接弧"约 +0.25s */
            return fabsf(next->p.arc.speed_mps) * MF_ARC_ENTRY_SOFT_MULT;
#else
        case MF_SEG_ARC:       return fabsf(next->p.arc.speed_mps);
#endif

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

/* 当前段自身的剩余行程(m, 非 MOVE 段恒 0)。与 mf_path_remain_m 的区别: 不含后续段。
 * 微调(trim)的预算必须用本段剩余 —— 全路径剩余会被后续位移段稀释, 爬行可越过本段
 * 端点; 且非 MOVE 段(如 TURN)期间 dist_accum 不累计(mf_path_tick 只记 MOVE/ARC),
 * 在这些段上微调会把账记丢 —— 返回 0 即调用方的拒绝信号(打靶段架构-刹停与续跑.md §10.18) */
float mf_path_seg_remain_m(const mf_path_ctx_t *p)
{
    const mf_move_dist_t *m;
    float done, total;

    if (p == NULL || p->path == NULL || p->state != MF_PATH_RUN)
    {
        return 0.0f;
    }
    if (p->path[p->idx].type != MF_SEG_MOVE_DIST)
    {
        return 0.0f;
    }
    m = &p->path[p->idx].p.move_dist;
    done = fabsf(p->dist_accum - p->dist_base_m);   /* 本段已走量(倒车也取正) */
    total = fabsf(m->dist_m);
    return (total > done) ? (total - done) : 0.0f;
}

void mf_path_get_arc_diag(const mf_path_ctx_t *p, float *wheel_deg, float *dead_deg,
                          uint16_t *dead_ticks, float *wzsp_peak, float *soft_min)
{
    if (p == 0)
    {
        return;
    }
    if (wheel_deg != 0)  { *wheel_deg  = p->arc_wheel_deg; }
    if (dead_deg != 0)   { *dead_deg   = p->arc_dead_deg; }
    if (dead_ticks != 0) { *dead_ticks = p->arc_dead_ticks; }
    if (wzsp_peak != 0)  { *wzsp_peak  = p->arc_wzsp_peak; }
    if (soft_min != 0)   { *soft_min   = p->arc_soft_min; }
}

float mf_path_get_vsrc(const mf_path_ctx_t *p)
{
    return (p == 0) ? 0.0f : seg_arc_vsrc(p);
}

/* ---------- 调试口用的按拍观测（只读，不参与控制） ---------- */
void mf_path_get_obs(const mf_path_ctx_t *p, mf_path_obs_t *obs)
{
    if ((p == 0) || (obs == 0))
    {
        return;
    }
    obs->target_deg  = p->obs_target_deg;
    obs->yaw_err_deg = p->obs_yawerr_deg;
    obs->ey_m        = p->obs_ey_m;
    obs->eyg_m       = p->obs_eyg_m;
    obs->xt_corr_deg = p->obs_xt_corr_deg;
    obs->kind        = p->obs_kind;
}

float mf_path_get_frame_base(const mf_path_ctx_t *p)
{
    return (p != 0) ? p->frame_base_deg : 0.0f;
}

uint32_t mf_path_get_ref_seq(const mf_path_ctx_t *p)
{
    return (p != 0) ? p->ref_seq : 0u;
}
