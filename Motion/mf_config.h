/**
 * @file    mf_config.h
 * @brief   运动框架编译期宏配置
 *
 * 三处宏决定整个框架的形态：
 *   MF_CHASSIS_TYPE  底盘类型（差速 / 履带 / 麦轮）
 *   MF_MOTOR_MODE    电机指令模式（框架闭环 PWM / 驱动器闭环速度指令）
 *   MF_YAW_SOURCE    航向来源（IMU / 编码器差分）
 *
 * 使用方式：在 Keil 工程 Options -> C/C++ -> Define 里定义，或在本文件改默认值。
 */
#ifndef MF_CONFIG_H
#define MF_CONFIG_H

/* ================= 底盘类型 ================= */
#define MF_CHASSIS_DIFF     0   /* 双轮差速底盘 */
#define MF_CHASSIS_TRACK    1   /* 履带底盘（运动学与差速同构，轨距/打滑参数独立标定） */
#define MF_CHASSIS_MECANUM  2   /* 麦克纳姆轮底盘 */

#ifndef MF_CHASSIS_TYPE
#define MF_CHASSIS_TYPE     MF_CHASSIS_TRACK    
#endif

/* ================= 整车安装方向（前后颠倒）================= */
/* 物理前进与框架正向相反时置为 -1.0f。前后颠倒 = 车体坐标系绕 z 轴旋转 180°，
 * 等价于：vx 取反 + wz 保持 + 物理左右履带身份对调。 */
#ifndef MF_CHASSIS_DIR_SIGN
#define MF_CHASSIS_DIR_SIGN   (-1.0f)
#endif

/* ================= 电机指令模式 ================= */
#define MF_MOTOR_PWM_DIR    0   /* 框架内做轮速闭环，输出 [-1,+1] 归一化 PWM（panzer 现状） */
#define MF_MOTOR_SPEED_CMD  1   /* 驱动器自带闭环（如 Emm_V5 串口舵机电机），框架输出轮速 m/s */

#ifndef MF_MOTOR_MODE
#define MF_MOTOR_MODE       MF_MOTOR_PWM_DIR
#endif

/* ================= 航向来源 ================= */
#define MF_YAW_SRC_IMU      0   /* 外置 IMU（HWT101 等），履带强烈推荐 */
#define MF_YAW_SRC_ENCODER  1   /* 编码器差分推算（履带打滑大，仅作兜底） */

#ifndef MF_YAW_SOURCE
#define MF_YAW_SOURCE       MF_YAW_SRC_IMU
#endif

/* ================= 通用参数 ================= */
#ifndef MF_WHEEL_MAX
#define MF_WHEEL_MAX        4   /* 最大轮数：差速/履带 2，麦轮 4 */
#endif

#ifndef MF_DT_S
#define MF_DT_S             0.01f   /* 控制周期，panzer TIM4 为 10ms */
#endif

/* 速度斜坡默认加速度（m/s^2）与角加速度（deg/s^2） */
#ifndef MF_DEFAULT_ACC
#define MF_DEFAULT_ACC      1.5f
#endif
#ifndef MF_DEFAULT_ANG_ACC
#define MF_DEFAULT_ANG_ACC  180.0f
#endif

/* 到点判据默认阈值 */
#ifndef MF_TOL_DIST_M
#define MF_TOL_DIST_M       0.02f    /* 2cm */
#endif
#ifndef MF_TOL_YAW_DEG
#define MF_TOL_YAW_DEG      1.0f
#endif

/* 到位余量: 行程/航向类判据带小余量收口, 避免减速末端里程量化导致永远差一点 */
#ifndef MF_TOL_ARRIVE_M
#define MF_TOL_ARRIVE_M     0.008f   /* 8mm */
#endif
#ifndef MF_TOL_ARRIVE_DEG
#define MF_TOL_ARRIVE_DEG   1.0f
#endif

/* 转向/收尾到位驻留: |err|<tol 需连续保持此时长才算到位(防高速穿越误判) */
#ifndef MF_TOL_HOLD_MS
#define MF_TOL_HOLD_MS      100u
#endif

/* ================= ARC 段几何跟踪（置 0 = 回旧律，供 A/B） ================= */
/* 进段把名义圆弧解到世界系(圆心/半径), 设定值取名义切线航向 + 横向偏差比例项 */
#ifndef MF_ARC_XTRACK_EN
#define MF_ARC_XTRACK_EN     1
#endif

/* 横向偏差增益 dps/m。调法: 0.1m 偏差 -> 25dps, 与弧线前馈 43dps 同量级, 宁小勿大 */
#ifndef MF_ARC_XTRACK_K
#define MF_ARC_XTRACK_K      250.0f
#endif

/* ================= 段间残差结转（置 0 = 原行为；本项目不推荐开） ================= */
/* 开后 MOVE 完成判据改用"累计名义目标"(残差结转), 需"相邻 MOVE 段"拓扑才有效 */
#ifndef MF_SEG_CARRY_EN
#define MF_SEG_CARRY_EN     0
#endif

/* 减速 profile 的加速度比例: 0.6 = 实测最优(置 1.0 = 原行为) */
#ifndef MF_STOP_DECEL_SCALE
#define MF_STOP_DECEL_SCALE 0.6f
#endif

/* ============ 差速前馈补偿 = 转弯半径旋钮（实车标定项） ============ */
/* 标定: 跑一趟看串口 ARC:<段> EY=<mm> —— 左转弧 EY 仍 >0 就按 1+EY/942.5 调大 POS,
 * EY 变负(往内切)即过头, 调回; 右转同理用 NEG。目标 |EY| < 20mm。置 EN=0 = A/B */
#ifndef MF_DIFF_BOOST_EN
#define MF_DIFF_BOOST_EN    1
#endif
#ifndef MF_DIFF_BOOST_POS
#define MF_DIFF_BOOST_POS   1.12f    /* 左转（wz_dps >= 0）差速放大 */
#endif
#ifndef MF_DIFF_BOOST_NEG
#define MF_DIFF_BOOST_NEG   1.16f    /* 右转（wz_dps <  0）差速放大 */
#endif

/* ============ 直线段航向基准（置 1 = 走路径标称航向，置 0 = 保持进段实测航向） ============ */
#ifndef MF_SEG_NOM_HEADING_EN
#define MF_SEG_NOM_HEADING_EN   1
#endif

/* ============ 全程单基准航向（MF_HEADING_FRAME_EN）============ */
/* 1 = 全程只有一个航向基准: 由 mf_set_heading_ref() 在 GYRO_ZERO 那一刻显式钉下(值 0°),
 *     此后 mf_path_start() 永不重定锚, nom_yaw 沿路线几何一路推进 —— 直线段于是会
 *     **主动把上游残留的航向误差纠回来**。
 *     旧行为是"每张 nav 表首段用当时的实测航向重新定基准" —— 那等于把上一腿的终点
 *     残差写成下一腿的基准, 沿腿链累加(注释记 ~0.25°/腿 ⇒ 终点 ~1°)。
 *     未调用 set_heading_ref 时 heading_ref_locked=0, 退回旧的按表重定锚:
 *     自测形态没有 GYRO_ZERO 命令, 靠这条兜底(此时基准等价于 main 里那次 ResetHWT101)。
 * 0 = 回旧行为(再受 MF_NOM_HEADING_PERSIST_EN 控制), 供 A/B */
#ifndef MF_HEADING_FRAME_EN
#define MF_HEADING_FRAME_EN   1
#endif

/* 旧的"跨表延续"开关: 仅在 MF_HEADING_FRAME_EN=0 时生效。它与单基准的差别只在**锚点**——
 * 它拿首段的实测航向当全程基准(隐式), 单基准拿 GYRO_ZERO 那一刻的 0° 当基准(显式) */
#ifndef MF_NOM_HEADING_PERSIST_EN
#define MF_NOM_HEADING_PERSIST_EN   0
#endif

/* ============ 航向基准偏差（唯一的静态补偿旋钮, 实车标定项）============ */
/* 基准系(见上)与真实几何之间的固定偏差。只在"期望角 → 误差"这一步加(seg_cmd_yaw),
 * 于是框架内所有角度(段表绝对角 / ARC 名义切线 / pose.yaw 读数口径)一律不动,
 * 只有下发出去的航向整体旋转这么多 —— 整条路线随之刚性旋转, 相对角(TURN_REL/GOTO_REL)
 * 与 ARC 自解算不受影响(它们本来就从当前位姿续起, 天然继承这个旋转)。
 *
 * 为什么必须收成**一个数**: 旧实现把补偿散在 NAV_REBASE_BOMB_YAW_DEG(-89.5)、
 * NAV_REBASE_RETURN_YAW_DEG(-179.5)、NAV_RETURN_TRIM_DEG(1.8) 三处, 与几何真值混在
 * 同一个宏里 ⇒ 分不出"几何错"还是"漂移补偿"; 而且实测证明那两个重锚角符号相反,
 * 不可能同是补偿(Motion/README.md §11.5 的 K5 结论)。
 *
 * 标定(一次即定): 卷尺/场地线量**终点实物**横向偏差 ΔY → 先填 ΔY / L(L=该腿长度,
 * 纯定符号与量级) → 跑一趟看实物残余是否变小, 变大就翻符号重跑。
 * ⚠ 别用 POSE 的 Y 反解: 机上测到的那部分横移几乎 100% 就是这个值自己造成的,
 *   拿它反解必然收敛到 0(2026-09-23 查证)。
 * 0.0f = 不补偿, 即 A/B 基准。 */
#ifndef MF_HEADING_BIAS_DEG
#define MF_HEADING_BIAS_DEG   0.0f
#endif

#if (MF_CHASSIS_TYPE == MF_CHASSIS_DIFF || MF_CHASSIS_TYPE == MF_CHASSIS_TRACK)
#define MF_WHEEL_COUNT 2
#elif (MF_CHASSIS_TYPE == MF_CHASSIS_MECANUM)
#define MF_WHEEL_COUNT 4
#else
#error "mf_config.h: MF_CHASSIS_TYPE is invalid"
#endif

#if (MF_WHEEL_COUNT > MF_WHEEL_MAX)
#error "mf_config.h: MF_WHEEL_MAX is minor than MF_WHEEL_COUNT"
#endif

#endif /* MF_CONFIG_H */
