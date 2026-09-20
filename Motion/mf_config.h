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

/* ================= ARC 段几何跟踪（修"弧线越跑越偏"） ================= */
/* 旧律把弧线的航向设定值按 dist_accum 推进, 而 dist_accum 来自轮上位移均值;
 * 履带转弯时该均值系统性大于真实车体位移(见 mf_odom.c 注释), 于是设定的偏航速率
 * 比车体真能达到的快 —— 航向环被迫多转, 实际弧半径被压成 R/k, 出口位置沿半径
 * 向内偏, 误差随打滑放大且每段累积。改用:
 *   1) 进段把名义圆弧解算到世界系(圆心/半径), 段内设定值取"名义切线航向",
 *      它由里程计位置(x/y, 已用 ds_for_xy 修正过打滑)推出 —— 设定值的偏航速率
 *      天然等于"真实车体位移/R", 不再逼航向环多转;
 *   2) 再加横向偏差比例项, 让"离名义弧多远"本身也进环(旧律完全没有这个维度:
 *      与弧等半径的纯横向平移对它是不可见的)。
 * 量化与归因见 build_verify/mf_path_test/arc_xtrack_sim.py:
 *   打滑 15% 时出口横向偏差 -117mm(旧) -> -0.2mm(新); 出口位置误差 178mm -> 2mm
 *
 * 置 0 = 回旧律, 供 A/B 对比。出口诊断(ARC: 行)两种模式都算, 用的是同一个几何
 * 指标, 所以现场可以直接拿两版的 ARC 行对比 */
#ifndef MF_ARC_XTRACK_EN
#define MF_ARC_XTRACK_EN     1
#endif

/* 横向偏差增益, dps per m（正偏差=弧外时向弧内补转向）。
 * 只治"轮子没跟上指令"这类真实扰动（单侧打滑/外力推/地面塌陷）——对履带刮擦
 * 造成的半径偏差无能为力, 那个只能靠把设定值的里程源用对（上面第1条）来治。
 * 宁小勿大: 0.1m 偏差 -> 25dps, 与弧线前馈 43dps 同量级; 再大就会和航向环抢带宽、引发摆动 */
#ifndef MF_ARC_XTRACK_K
#define MF_ARC_XTRACK_K      250.0f
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
