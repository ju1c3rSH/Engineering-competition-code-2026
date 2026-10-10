/**
 * @file    mf_path.h
 * @brief   声明式路径段执行器：一条路径 = 以 MF_SEG_END 结尾的 mf_seg_t 常量数组(放 Flash)，
 *          每个控制周期 tick 一次 —— 输入当前位姿, 输出车体速度指令。
 */
#ifndef MF_PATH_H
#define MF_PATH_H

#include <stddef.h>
#include "mf_types.h"
#include "mf_pid.h"

typedef struct
{
    const mf_seg_t *path;   /* 当前路径数组 */
    const mf_seg_t *next_path;  /* 本表跑完将自动续跑的下一条(0=不续)。两处用途:
                                 * ① 末段减速取它的首段速度, 免表尾硬性减到 0;
                                 * ② 跨表时同拍切过去, 消除段表之间的零输出空窗 */
    uint16_t idx;           /* 当前段下标 */
    mf_path_state_t state;
    uint8_t chain_new;      /* 1=刚跨过表边界(上层 pop 后清零), 见 mf_path_pop_chain */

    /* 段进入时刻快照（增量段的基准） */
    mf_pose_t entry;
    mf_pose_t target;       /* GOTO 段解算出的绝对目标点 */

    /* 段内累加量（进段清零） */
    float dist_accum;       /* 已走弧长/位移，m（带符号）；
                             * MF_SEG_CARRY_EN=1 时为**全程连续里程**（进段不清零） */
    uint32_t time_ms;       /* 段内已用时，ms */

    /* 段间累积（仅 MF_SEG_CARRY_EN=1 生效；见 mf_config.h 的说明）：
     * MOVE 段的完成判据改用"累计名义目标 - 当前里程"，把上一段欠走/多走的残差结转过来 */
    float dist_target_m;    /* 累计名义目标里程，m（残差结转的基准） */
    float dist_base_m;      /* 本段起点处的 dist_accum：段内进度 = dist_accum - dist_base_m */
    uint8_t carry_link;     /* 1=上一段是位移段，本段可承接残差；ARC/GOTO 后归 0 */
    uint16_t tol_hold_ms;   /* 到位驻留累计：|err|<tol 连续保持时长 */
    float cur_speed;        /* 斜坡后的当前前向速度，m/s（跨段保持，衔接用） */
    float v_meas_f;         /* 实测前向速度(低通后, m/s): ds/dt 滤波值, 与 cur_speed 同口径。
                             * MF_ARC_VSRC_MEAS=1 时弧线 ω 前馈取它 ⇒ 车速在 R=v/ω 里约掉,
                             * 半径不再随电池/饱和/地面变化(见 mf_config.h)。调试口 VF= 打它 */
    float cur_wz;           /* 斜坡后的当前角速度指令，deg/s（TURN/GOTO 入段缓起） */
    float prev_wz;          /* 上一拍最终输出角速度，deg/s（进段时作 cur_wz 种子） */
    uint8_t subphase;       /* GOTO 段：0=接近点，1=收尾转向 */
    uint8_t started;

    /* ARC 段几何解算（进段时由 entry 位姿 + R·左法线解出，段内只读）：
     * 半径带符号，左转圆心在车体左侧、右转在右侧，用同一个式子即可两向通用 */
    float arc_cx_m;         /* 名义圆心 X（世界系） */
    float arc_cy_m;         /* 名义圆心 Y（世界系） */
    float arc_r_abs;        /* |radius_m| */
    float arc_sgn;          /* +1=左转 / -1=右转（d_yaw 符号；与 radius 同号） */
    float arc_ey_m;         /* 本拍带符号横向偏差 m，正=弧外（诊断与横向环共用） */
    float arc_remain_prev_deg;  /* 上一拍的 remain_yaw：两者的差就是**本段实测抽样步长**，
                                 * 到位余量取它的一半才有零系统偏差（见 mf_config.h）。
                                 * 进段时置为 d_yaw（首拍的步长即第一拍的转角） */
    float slip_gain;        /* 本段的开环滑移基线（弧线 = MF_ARC_SLIP_GAIN，其余 1.0）。
                             * 速率内环(mf_rate.h)拿它当输出侧前馈；见 mf_config.h */

    /* ARC 段出口质量记录（段完成那一拍落一次，等上层取走）。
     * ARC 与直线 MOVE 共用这一套，kind 区分；直线段无名义线，ey 恒 0（见 mf_path.c） */
    float seg_exit_ey_m;        /* ARC: 出口横向偏差 m，正=弧外 */
    float seg_exit_yawerr_deg;  /* 出口航向偏差 deg（纠完剩多少） */
    float seg_exit_in_yawerr_deg; /* 进段航向偏差 deg（带进来多少），仅直线段填 */
    float seg_exit_radius_m;    /* ARC 出口的 R = 本段轮程 / |IMU 转角| (m)。
                                 * 自证量: 速率内环令 ω_actual=ω_sp=v_wheel/R_cmd,
                                 * 故本值 ≡ R_cmd, **不是物理半径**(打滑机上不可观测,
                                 * 真半径只能卷尺量, 见 mf_config.h 的 MF_ARC_WZ_TRIM)。
                                 * 偏离 R_cmd ⇒ 反解/限幅出问题, 是内环健康判据 */
    float seg_exit_dyaw_deg;    /* ARC 出口**实际转角** |Δyaw_imu|(deg, 恒正),
                                 * "转够了没"的直接读数。到位余量已改为按实测步长
                                 * 运行期计算(mf_config.h 的到位余量说明), 本条即验收读数 */
    uint16_t seg_exit_seg;      /* 出口所在的段下标 */
    uint8_t  seg_exit_kind;     /* 0=ARC / 1=直线 MOVE */
    uint8_t  seg_exit_new;      /* 1=有新记录待取 */

    /* 路径标称航向（MF_SEG_NOM_HEADING_EN=1 生效, 见 mf_config.h）：
     * 由路径几何累积(ARC 加 d_yaw、TURN 赋目标角), MOVE 段目标航向取它,
     * 上游残留航向误差被直线段主动纠回, 而非被 MF_KEEP 锁死按"误差×长度"放大 */
    float nom_yaw_deg;      /* 当前段起点处的标称航向 deg（**路线系**，见 frame_base_deg） */
    uint8_t nom_yaw_init;   /* 0=还没定过基准（本段进段时补上） */
    uint8_t heading_ref_locked; /* 1=基准已由 mf_path_set_heading_ref() 显式钉下,
                                 * 全程只读(跨表也不再重定锚); 见 MF_HEADING_FRAME_EN */
    float line_in_yawerr_deg;   /* 直线段进段航向误差(带进来多少), 供出口记录对比 */

    /* ---- 整表重锚基点（不变量 P5, 见 Motion/README.md「单基准航向架构」）----
     * 表里绝对角(重锚/返程)与 nom_yaw 一律在**路线系**(置零那一刻车头 = 0);
     * pose.yaw 是模块读数域, 两者只差此基点, **换算只许走 seg_pose_yaw() 一处**。
     * 缺它时: 上电到置零间车被转过 yaw0, 首绝对段后所有段整体偏 yaw0(走斜) */
    float frame_base_deg;   /* = GYRO_ZERO 那一刻的模块读数 deg */
    uint32_t ref_seq;       /* 基点被钉下的次数(1 起): 调试口靠它打 YAWZERO 事件行 */
    float line_hdg_deg;     /* 直线段名义线方向(读数域 deg): 进段由目标航向 + 基点解出,
                             * 只用来算本段的横向偏差 */

    /* 按拍观测（各段分支每拍写，供调试口按拍打印；三个量同拍读才自洽） */
    float obs_target_deg;   /* 本段航向目标(路线系) */
    float obs_yawerr_deg;   /* 本拍航向误差(环内实际用的那个值) */
    float obs_ey_m;         /* 横向偏差: 直线=名义线左侧正 / ARC=弧外正 */
    uint8_t obs_kind;       /* 0=ARC 1=直线MOVE 2=TURN_TO 3=TURN_REL 4=GOTO */

    /* 由运动层注入的控制器（所有权在 mf_motion） */
    mf_pid_t *pid_yaw;      /* 航向保持 / 原地转向 / 方位角 P */
    mf_pid_t *pid_point_v;  /* 点到点前向速度 P */
} mf_path_ctx_t;

void mf_path_init(mf_path_ctx_t *p, mf_pid_t *pid_yaw, mf_pid_t *pid_point_v);

/** @brief 显式钉下整表航向基准（MF_HEADING_FRAME_EN=1）
 *
 * 上层在 GYRO_ZERO 那一刻调用一次（车静止, 此刻车头朝向 = 路线几何 0°）。调用后
 * heading_ref_locked=1: 之后 mf_path_start()/跨表**不再重定锚**, 表里绝对角一律解释在
 * "置零时刻车头=0"的路线系(见 frame_base_deg)。
 * @param yaw_deg 传**当前实测航向**(Car_GetYawDeg()); 无需模块物理归零, 固定偏差走 MF_HEADING_BIAS_DEG
 */
void mf_path_set_heading_ref(mf_path_ctx_t *p, float yaw_deg);

/** @brief 开始执行一条路径（须以 MF_SEG_END 结尾）
 *  @param next  本表跑完自动续跑的下一条(须以 MF_SEG_END 结尾); 0=跑完即 DONE。
 *               非空时末段按 next 首段速度收尾, 且跨表同拍切换、输出不归零 */
void mf_path_start(mf_path_ctx_t *p, const mf_seg_t *path, const mf_seg_t *next);

/** @brief 停止并回到 IDLE（同时解除待续表） */
void mf_path_stop(mf_path_ctx_t *p);

/**
 * @brief 取走"刚跨过表边界"标志（读后清）
 *
 * 段表链续时底盘已自行跑进下一条表，但上层仍要按旧 nav_id 上报 NAV_DONE 并切 nav_id，
 * 用它把这两件事与几何边界对齐。
 * @return 1=本次取到新边界；0=无
 */
uint8_t mf_path_pop_chain(mf_path_ctx_t *p);

/**
 * @brief 段执行器 tick，每个控制周期调用一次
 * @param pose   当前位姿（里程计输出）
 * @param ds     本周期车体前向位移（m，带符号）
 * @param dt     控制周期 s
 * @param out    输出车体速度指令（执行完毕输出全零）
 * @return 1=当前段刚完成（供上层在段间插入动作），0=未完成/未运行
 */
uint8_t mf_path_tick(mf_path_ctx_t *p, const mf_pose_t *pose,
                     float ds, float dt, mf_body_cmd_t *out);

mf_path_state_t mf_path_get_state(const mf_path_ctx_t *p);
uint16_t mf_path_get_seg_index(const mf_path_ctx_t *p);

/**
 * @brief 当前是否正处在 MF_SEG_WAIT（定时停等）段
 *
 * 上层（如底盘堵转看门狗）用它区分"段表主动要求的原地停等"与"真堵转": WAIT 段内车体
 * 本就应静止，里程 3s 无变化不能判成故障。
 * @return 1=正在 WAIT 段；0=非运行态/其它段
 */
uint8_t mf_path_is_waiting(const mf_path_ctx_t *p);

/**
 * @brief 取走最近一次"段出口质量记录"（读后清标志）
 *
 * ARC: ey_m=出口横向偏差(跑没跑偏); 直线: ey_m 恒 0(无名义线, 见 Motion/README.md)。
 * 两者填 out_yawerr_deg=出口航向残差; 直线另填 in_yawerr_deg, "带进来多少/纠完剩多少"
 * 可现场对比(单基准架构验收判据)。上层主循环调用加中断保护。
 * @param seg_index       出口所在段下标（可传 0）
 * @param kind            0=ARC / 1=直线 MOVE（可传 0）
 * @param ey_m            ARC: 出口横向偏差 m，正=落在名义弧外侧; 直线: 相对"进段点+本段目标
 *                        航向"这条名义线的横向偏差, 正=左侧（可传 0）
 * @param in_yawerr_deg   进段航向误差 deg，仅直线段有效（可传 0）
 * @param out_yawerr_deg  出口航向残差 deg（可传 0）
 * @return 1=本次取到新记录；0=无新记录（出参不变）
 */
uint8_t mf_path_pop_seg_exit(mf_path_ctx_t *p, uint16_t *seg_index, uint8_t *kind,
                             float *ey_m, float *in_yawerr_deg, float *out_yawerr_deg);

/**
 * @brief 最近一次 ARC 出口的 R = 本段轮程 / |本段 IMU 转角|（m，恒正）
 *
 * 名义 TURN_RADIUS_M = 0.9425。**非物理半径**: 速率内环令本值 ≡ R_cmd(自证), 物理半径另乘
 * (1−纵向打滑)且打滑机上不可观测 ⇒ 真半径只能卷尺量(mf_config.h: MF_ARC_WZ_TRIM)。
 * 本值偏离名义 = 内环没关到位/限幅, 是健康判据非几何。
 * @return 名义上 >0; 尚无 ARC 出口记录时返回 0
 */
float mf_path_exit_radius_m(const mf_path_ctx_t *p);

/**
 * @brief 最近一次 ARC 的**实际转角** |Δyaw_imu|（deg，恒正；名义 90）
 *
 * "弧转够了没"的直接读数。EY/YE/R 都是"位姿 vs 名义圆"自证量看不出少转, 唯一旁证是下一段
 * MOVE 的 LN:IE(替弧补角)。到位余量按实测步长运行期算(见 mf_config.h), 本条即验收读数
 */
float mf_path_exit_dyaw_deg(const mf_path_ctx_t *p);

float mf_path_remain_m(const mf_path_ctx_t *p);   /* 剩余里程估算(m): 只算位移段, ARC 全额计入 */

/**
 * @brief 弧线 ω 前馈当前实际使用的速度源(m/s)：MF_ARC_VSRC_MEAS=1 时是实测值，否则是指令值
 *
 * 与段表指令速度对比就是**车速亏损比** v̂/v_cmd —— 它同时就是半径亏损比(取指令源时)。
 * 取实测源后两者相等才对; 持续偏低即"车吃不消这个弧速", 弧会自动走慢但不改半径。
 */
float mf_path_get_vsrc(const mf_path_ctx_t *p);

/* ---- 按拍观测（调试口用）：与 mf_path_pop_seg_exit 的区别是"每拍都更新" ---- */
typedef struct
{
    float   target_deg;     /* 本段航向目标（路线系 deg） */
    float   yaw_err_deg;    /* 本拍航向误差（环内实际用的那个值 deg） */
    float   ey_m;           /* 横向偏差: 直线=本段名义线左侧正 / ARC=弧外正 */
    uint8_t kind;           /* 0=ARC 1=直线MOVE 2=TURN_TO 3=TURN_REL 4=GOTO */
} mf_path_obs_t;

/** @brief 本拍的目标航向/误差/横向偏差（只读, 不参与控制）
 *
 * 直线横向偏差由"进段点+本段目标航向"名义线算出, 正=左侧。
 * 区分两种"长直线走偏": 沿程不变=整段平行挪出(弧半径/进段偏移); 随程线性增长=航向基准被转(越走越斜)
 */
void mf_path_get_obs(const mf_path_ctx_t *p, mf_path_obs_t *obs);

/** @brief 整表重锚基点 = GYRO_ZERO 那一刻的模块读数（deg）；未钉下时为 0
 *
 * 调试口把它和"本拍路线系航向"一起打出来: pose.yaw − 本值 = 路线系航向。 */
float mf_path_get_frame_base(const mf_path_ctx_t *p);

/** @brief 基点被钉下的次数（从 1 起）：调试口靠它判"这一趟置零了没有" */
uint32_t mf_path_get_ref_seq(const mf_path_ctx_t *p);

#endif /* MF_PATH_H */
