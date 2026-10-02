/**
 * @file    mf_types.h
 * @brief   运动框架公共类型：位姿、速度指令、声明式路径段
 *
 * 单位约定（与既有两套库对齐）：
 *   距离/坐标：米(m)     速度：m/s      加速度：m/s^2
 *   角度：度(deg)        角速度：deg/s  角加速度：deg/s^2
 *   航向 yaw：HWT101 语义的多圈连续角，逆时针为正。
 */
#ifndef MF_TYPES_H
#define MF_TYPES_H

#include <stdint.h>
#include "mf_config.h"

/* ================= 基础量 ================= */

typedef struct
{
    float x_m;          /* 世界系 X 位置，m */
    float y_m;          /* 世界系 Y 位置，m */
    float yaw_deg;      /* 航向角，deg，多圈连续 */
} mf_pose_t;

typedef struct
{
    float vx_mps;       /* 车体前向线速度，m/s */
    float vy_mps;       /* 车体横向线速度，m/s（仅麦轮有效，其余底盘恒为 0） */
    float wz_dps;       /* 车体角速度，deg/s，逆时针为正 */
} mf_body_cmd_t;

/* 轮子运行状态（需框架内部维护） */
typedef struct
{
    float dist_m;       /* 带符号的累计轮上位移，m */
    float last_dist_m;  /* 上一周期位移快照 */
    float delta_m;      /* 本周期轮上位移增量，m */
    float speed_mps;    /* 本周期轮速，m/s */
    float cmd_mps;      /* 解算出的轮速目标，m/s */
    float out;          /* 最终输出：归一化 PWM 或轮速指令 */
} mf_wheel_t;

/* 采样快照：运动库的唯一输入端。
 * 采集与存储归应用层 sense 模块（固定周期写快照、seq 单调递增），
 * mf_tick 只消费快照做解算，自身不做任何硬件读取 */
typedef struct
{
    float    wheel_dist_m[MF_WHEEL_MAX]; /* 各轮累计位移 m（带符号，前进为正） */
    float    yaw_deg;                    /* 外部航向 deg（±180；编码器航向源时忽略） */
    float    wz_dps;                     /* IMU 实测车体角速度 deg/s（逆时针为正；无源填 0） */
    uint8_t  wz_valid;                   /* wz_dps 是否为本拍新鲜反馈；0=停更，内环退回纯前馈 */
    uint32_t seq;                        /* 采样序号：与上次相同=未出新样，本拍跳过 */
    uint32_t yaw_seq;                    /* 航向帧计数(0x53)：yaw_deg 是零阶保持值，"这一拍是不是
                                          * 新样"只能靠它判 —— 凡要用"航向增量除位移"(弧半径/形状)
                                          * 的地方，分子分母必须覆盖同一段时间间隔。无 IMU 填 0 */
    uint8_t  yaw_diff_win;               /* 航向差分该跨几拍(1..MF_YAW_DIFF_WIN_MAX)。**当前采样层
                                          * 不下发(恒 0)** ⇒ 库内用 1 拍即逐拍差分: 现场模块
                                          * 200Hz = 2 帧/拍, 无交替。仅当帧率不是拍率整数倍时
                                          * 才需要它(见 mf_pid.h / Car_Sense.c) */
} mf_state_in_t;

/* 特殊值：段参数里的“不指定/保持进入时状态” */
#define MF_KEEP         10000.0f

/* ================= 声明式路径段 ================= */

typedef enum
{
    MF_SEG_MOVE_DIST = 0,   /* 增量直线：沿进入时方向走 dist_m，航向保持 heading_deg */
    MF_SEG_ARC,             /* 定半径弧线（履带/差速转弯的标准做法），走完 d_yaw_deg */
    MF_SEG_TURN_TO,         /* 原地转到绝对航向 yaw_deg */
    MF_SEG_TURN_REL,        /* 原地相对转 d_yaw_deg */
    MF_SEG_GOTO_XY,         /* 绝对坐标点到点，到位后可声明一个绝对朝向 yaw_deg */
    MF_SEG_GOTO_REL,        /* 车体系相对坐标点到点（进入时刻为基准） */
    MF_SEG_WAIT,            /* 定时等待 */
    MF_SEG_END              /* 路径结束哨兵 */
} mf_seg_type_t;

/* MF_SEG_MOVE_DIST：增量直线 */
typedef struct
{
    float speed_mps;        /* 巡航速度，m/s */
    float dist_m;           /* 行程，带符号，负=倒车，m */
    float heading_deg;      /* 航向保持目标（绝对角），MF_KEEP=保持进入时刻航向 */
    float acc_mps2;         /* 加速度，0=用 MF_DEFAULT_ACC */
} mf_move_dist_t;

/* MF_SEG_ARC：定半径弧线，半径带符号（正=左转，负=右转） */
typedef struct
{
    float speed_mps;        /* 弧线切向速度，m/s */
    float radius_m;         /* 转弯半径，m，正=左转，负=右转，不允许 0 */
    float d_yaw_deg;        /* 航向总变化量，deg（符号须与 radius 同向） */
    float acc_mps2;         /* 加速度，0=默认 */
} mf_arc_t;

/* MF_SEG_TURN_TO / MF_SEG_TURN_REL：原地转向 */
typedef struct
{
    float yaw_deg;          /* TURN_TO=绝对目标航向；TURN_REL=相对转角 */
    float w_max_dps;        /* 最大角速度，deg/s，0=不限制（用航向环输出） */
} mf_turn_t;

/* MF_SEG_GOTO_XY / MF_SEG_GOTO_REL：点到点（绝对/相对） */
typedef struct
{
    float x_m;              /* GOTO_XY=绝对 X；GOTO_REL=车体前向 Δx（进入时刻系） */
    float y_m;              /* GOTO_XY=绝对 Y；GOTO_REL=车体左向 Δy（进入时刻系） */
    float yaw_deg;          /* 到位后的收尾朝向，MF_KEEP=不调整 */
    float v_max_mps;        /* 接近速度上限，m/s */
    float w_max_dps;        /* 转向角速度上限，deg/s */
    float acc_mps2;         /* 加速度，0=默认 */
} mf_goto_t;

/* MF_SEG_WAIT */
typedef struct
{
    uint32_t duration_ms;
} mf_wait_t;

/* 统一段描述符：const 数组放 Flash，段尾必须放 MF_SEG_END */
typedef struct
{
    mf_seg_type_t type;
    uint8_t reset_pid;      /* 进入本段时复位 PID 积分（默认建议 1） */
    float tol_dist_m;       /* 到位距离阈值，0=用 MF_TOL_DIST_M */
    float tol_yaw_deg;      /* 到位航向阈值，0=用 MF_TOL_YAW_DEG */
    union
    {
        mf_move_dist_t move_dist;
        mf_arc_t       arc;
        mf_turn_t      turn;
        mf_goto_t      goto_xy;
        mf_wait_t      wait;
    } p;
} mf_seg_t;

/* ================= 执行状态 ================= */

typedef enum
{
    MF_PATH_IDLE = 0,       /* 未在执行 */
    MF_PATH_RUN,            /* 正在执行某段 */
    MF_PATH_DONE            /* 全部段完成 */
} mf_path_state_t;

typedef struct
{
    mf_path_state_t state;  /* 路径执行状态 */
    uint16_t seg_index;     /* 当前段下标 */
    uint8_t seg_done;       /* 当前段是否已到位 */
} mf_status_t;

#endif /* MF_TYPES_H */
