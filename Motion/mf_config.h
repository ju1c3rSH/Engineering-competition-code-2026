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

/* ================= 段间累积与停车质量（修"转段/停车误差"） ================= */
/* 量化与选型见 build_verify/mf_path_test/seg_handoff_sim.py（直线链 10 段 x 2.112m）。
 *
 * 现状（MF_SEG_CARRY_EN=0）沿路径方向有两处系统偏差，而且**互相抵消一部分**：
 *   1) 到位容差短装：remain <= MF_TOL_ARRIVE_M(8mm) 就宣布完成 → 每段少走约 8mm；
 *   2) 进段丢一拍：mf_path_tick 的 'entered == 0' 门控让转段那一拍的 ds 两段都不记
 *      （本拍 ds 是上一拍指令产生的，旧段已结算、新段整拍跳过）→ 里程表少记 = 车多走，
 *      巡航时每转段丢约 9mm（v*dt），全链累计 +85mm。
 * 所以**单独把容差收紧（8mm→1mm）会更差**（停稳工况 -18mm → +48mm），而且 2mm 以下无效
 * （到点那一拍的位移量子已有约 4mm）。
 *
 * MF_SEG_CARRY_EN=1 改为：dist_accum 全程连续累计（含进段拍），MOVE 段完成判据用
 * **累计名义目标**（残差结转）——一个改动同时消掉 1 和 2。遇 ARC / GOTO 段重新对齐
 * （方向变了，标量残差没有意义），跨 WAIT / TURN 继续结转（这两段不产生位移）。
 * 实测（10 段直线链，tol=8mm）：停稳 -18 → +4mm，巡航 +80 → +5mm；
 * 再叠 MF_STOP_DECEL_SCALE=0.6 → -3.9mm / -1.1mm。
 * 注意：只去掉"进段丢一拍"而不做结转会**更差**（停稳 -18 → -46mm），两者必须同时改。
 *
 * 【但它依赖"相邻位移段"这个拓扑】本项目段表里 MOVE 之间基本都夹着 ARC，而 ARC 段
 * 会重新对齐（方向变了，标量残差没意义），于是残差几乎没有机会被结转。真实路线实测
 * （route_fix_sim.py，1->6->2->3->4->7->5）：只开本宏 130.4 → 124.9mm（几乎无效），
 * 且与 MF_STOP_DECEL_SCALE=0.6 同开反而 61.0 → 93.5mm（更差）。
 * 所以**本项目默认 0、不推荐开**；只有整条路线是长串相邻 MOVE 段时才值得考虑。
 * 置 0 = 完全维持原行为，供 A/B。 */
#ifndef MF_SEG_CARRY_EN
#define MF_SEG_CARRY_EN     0
#endif

/* 抛物线减速 profile 的加速度比例（速度斜坡 seg_speed_ramp 的限幅仍用段自身 acc）。
 * sqrt(2*acc*remain) 的减速率恰好等于斜坡限幅速率，离散拍下这个跟踪是**临界稳定**的：
 * 一旦因离散化/轮速环滞后超前，就再也回不到 profile 上。实测到点那一刻车速仍有 0.44
 * m/s（profile 预期 0.19），随后滑停约 40mm —— 而 8mm 容差提前收口恰好把这块抵消掉，
 * 这就是"容差看着好用、收紧反而变差"的来源。
 * 取 <1 让 profile 更缓（更早开始减速），斜坡才有裕度把速度真正收到 ~0。
 * 推荐 0.6：真实路线实测（route_fix_sim.py）位置偏差 130.4 → 103.9mm、航向 4.6 → 2.7
 * (0.1deg)；再配"去掉进段丢一拍"降到 61.0mm / 1.6。直线链也一致改善。
 * 代价：减速起点从段末约 188mm 提前到约 314mm，短段(<0.4m)跑不满巡航速 —— 那是正确
 * 行为（本来就得停），但整条路线会慢一点。置 1.0 = 原行为（不改变任何运动）。 */
#ifndef MF_STOP_DECEL_SCALE
#define MF_STOP_DECEL_SCALE 1.0f
#endif

/* ============ 直线段航向基准：进段实测 vs 路径标称（修"航向误差被长直线放大"） ============ */
/* 量化与归因见 build_verify/mf_path_test/speed_unify_sim.py。
 *
 * 现状（MF_KEEP = 保持**进段实测航向**）会把上游 ARC/TURN 段残留的航向误差**锁死**
 * 并一路带下去，而终点**位置**误差 ≈ 该航向误差 × 其后直线长度。真实路线上最后两段
 * 长直线（5.5m / 5.3m）各自贡献 +67mm / +56mm，是终点偏差的最大单项 —— 远大于弧线
 * 横向误差（每条弧 0~2mm）和段间里程记账误差。
 *
 * 置 1：MOVE 段的目标航向改取「路径标称航向」——由路径几何累积得到：
 *   起点用实测航向初始化，之后每过一条 ARC 加 d_yaw、每过一个 TURN 赋其目标角。
 *   于是直线段会**主动把上游误差纠回来**，而不再是把它锁定并放大。
 *
 * 实测（同一个仿真，段表 1->6->2->3->4->7->5，10 MOVE + 6 ARC + 2 TURN）：
 *   比赛模式（连续跑）  188.5mm -> 15.0mm (k=1.00) / 172.2 -> 30.5mm (k=1.15)
 *   自测模式（每段停）  190.5mm -> 64.1mm (k=1.00) / 130.3 -> 51.3mm (k=1.15)
 *   终点航向偏差 0.94deg -> 0.00deg
 * 再叠 MF_STOP_DECEL_SCALE=0.6（两模式都最优）：10.7 / 22.2mm 与 7.8 / 5.2mm。
 *
 * 前提：路径起点的实测航向要准（标称航向由它初始化，之后纯几何累积，不再依赖 IMU
 * 持续读数）。若出发点航向本身有系统偏差，车会一路朝"错了同样多"的标称方向跑。
 * 现场做法：先空跑一圈，逐段核对串口 ARC: 行的 YE 是否收敛。
 * 置 0 = 完全维持原行为，供 A/B。 */
#ifndef MF_SEG_NOM_HEADING_EN
#define MF_SEG_NOM_HEADING_EN   1
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
