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
 *
 * 实测（build_verify/mf_path_test/speed_unify_sim.py，全链 1->6->2->3->4->7->5）：
 * 与 MF_SEG_NOM_HEADING_EN=1 配套时三种停等粒度一致改善 ——
 *   连续跑(A)  标称航向 15.0/30.5mm -> +0.6 后 10.7/22.2mm
 *   每段停(B)  标称航向 64.1/51.3mm -> +0.6 后  7.8/ 5.2mm
 *   大段末停(C) 标称航向 62.5/76.0mm -> +0.6 后  4.9/18.4mm   (k=1.00/1.15)
 * 白走里程(WAIT 段只进 pose 不进段进度) 545.6->158.9mm (模式B)、153.6->32.1mm (模式C)。
 * 耗时代价可忽略（模式C 37.3->37.5s）。
 *
 * 注意配套关系：**单独开（不配标称航向）会变差**（模式C 159.1->170.4mm）——它治的是
 * 段末速度收不住，而段末车速高留下的航向漂移会被后面的长直线放大；只有把 MOVE 段航向
 * 换成标称航向后，这个漂移才不再被锁死，减速裕度的收益才体现出来。
 * 代价：减速起点从段末约 188mm 提前到约 314mm，短段(<0.4m)跑不满巡航速 —— 那是正确
 * 行为（本来就得停），整条路线只慢约 0.2s。置 1.0 = 原行为（不改变任何运动）。 */
#ifndef MF_STOP_DECEL_SCALE
#define MF_STOP_DECEL_SCALE 0.6f
#endif

/* ============ 差速前馈补偿：治"转弯半径系统性偏大"（实车标定项） ============ */
/*
 * 根因（2026-09-20 实车串口日志实测，见 build_verify/field_log_20260920.txt
 *      与 analyze_field_log.py）：履带转弯时**内侧履带刹不住、外侧履带跟不上**，
 *      两侧一叠加，实际差速只有指令差速的 0.74~0.89，于是
 *          实际转弯半径 = 名义半径 / 跟踪比  >  名义半径
 *      表现为 ARC 出口横向偏差 EY 恒为正（往外飘），且左右不对称：
 *
 *          转向   跟踪比(实际/指令)   ARC出口 EY    ARC出口 YE    内侧轮超调
 *          左转      0.889            110~114 mm      ~  0 deg     +30~+64
 *          右转      0.742            150~159 mm      ~ -4.1 deg   +30~+83
 *
 *      右转明显更差：右侧履带做内侧时超调(+83)是左侧做内侧(+30)的 2 倍多,
 *      是硬件/机械不对称（张紧、轮径、电机或制动能力），不是控制增益问题 ——
 *      轮速环已是闭环(wheel_kp=0.6 wheel_ki=7.0)，稳态误差仍达 21% 说明到了物理极限。
 *
 * 补偿：把差速指令按转向方向放大，让*实际*差速回到目标值。
 *   系数来源有两个口径，取 EY 反推的保守值（WHEEL 是瞬时快照且在瞬态，
 *   EY 是整弧积分结果，更可信；且过补偿会反向，宁可保守）：
 *     WHEEL 瞬时: 左 1/0.889=1.125   右 1/0.742=1.348
 *     EY   反推: 左 (0.9425+0.112)/0.9425=1.119   右 (0.9425+0.155)/0.9425=1.164
 *
 * 【现场标定】跑一遍看串口 ARC:<段> EY=<mm> 行：
 *   若左转弧 EY 仍 >0，按 missing/0.9425 的比例增大 MF_DIFF_BOOST_POS；
 *   若变成负的（往内切），说明补过头，调回。右转同理用 MF_DIFF_BOOST_NEG。
 *   目标：各弧 EY 落在 ±20mm 内。
 *
 * 【副作用】TURN 原地转向也会按同样比例加快转速。TURN_TO 用 IMU 闭环判完成，
 *   只影响转速不影响终角，可接受；若嫌转得太猛可单独给 TURN 段限速。
 *
 * 置 MF_DIFF_BOOST_EN=0 恢复原行为，用于 A/B。 */
#ifndef MF_DIFF_BOOST_EN
#define MF_DIFF_BOOST_EN    1
#endif
#ifndef MF_DIFF_BOOST_POS
#define MF_DIFF_BOOST_POS   1.12f    /* 左转（wz_dps >= 0）差速放大 */
#endif
#ifndef MF_DIFF_BOOST_NEG
#define MF_DIFF_BOOST_NEG   1.16f    /* 右转（wz_dps <  0）差速放大 */
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

/* ============ 标称航向是否跨 nav 表延续（修"航向误差逐腿累加"） ============ */
/*
 * 根因(2026-09-20 全程自测日志):
 *   mf_path_start() 原本无条件清 nom_yaw_init -> 每条 nav 表首段进段时都用**当时
 *   实测航向**重新定锚(见 mf_path.c seg_enter)。于是"上一腿的终点残差"被写进下一腿
 *   的标称基准 = 合法化, 后续段永远不会去修它 —— 残差沿腿链累加。
 *
 *   实测增量(终点 YAW vs 全局标称):
 *       腿 ID=06 -> +0.2°   ID=03 -> +0.5°   ID=04 -> +0.7°   终点 ID=05 -> +1.0°
 *   增量是**每条腿固定 ~0.25°**, 与腿的长度(1.5~5.7m)、时长(5~9s)都不成比例 ——
 *   不是"传感器按时间漂移"(那样长腿应漂更多), 而是每腿边界的固定残差被定锚累加。
 *   推论: 调航向环增益/加积分只能压小单腿残差, 压不掉"累加"这个结构性动作。
 *   (同期实测: 加 yaw_ki=2.0 后航向仍有问题 —— 正是这个原因, 不是积分无效于噪声。)
 *
 * 本开关: 置 1 时 nom_yaw 不再随新 nav 表重置, 而是沿路线几何延续, 只在
 *   ARC(累加转角) / TURN_TO(取绝对目标角) 推进 —— 后者就是外部真值的注入点
 *   (NAV_REBASE_BOMB/RETURN 的标定角)。于是:
 *     - 每条腿的目标是"几何标称", 不再继承上一腿的残差 -> 残差不再累加;
 *     - 直线段会把上一腿的残差**主动修回来**(这正是旧行为做不到的事);
 *     - 重锚 TURN_TO 完成后, 其欠账也会被后续直线继续修, 而不是被定锚锁死。
 *
 * 【适用前提】两腿之间车不能被人为搬动/外部重定向 —— 搬动后标称航向不再代表真实
 *   几何, 需重新下发含 TURN_TO 的重锚表或复位里程计。比赛流程(Brain 逐条 REQ_NAV,
 *   车在任务点静止等待)满足该前提。
 *
 * 置 0 恢复"每条 nav 表重新定锚"的旧行为, 供 A/B。 */
#ifndef MF_NOM_HEADING_PERSIST_EN
/* 2026-09-21 实车复盘: 置 1 后其它腿反而更差, 暂关, 回退到 09-20 23:00 的可跑状态。
 * 注意: 终点的绝对航向修复(返程腿 SEG_MOVE_HD + 表尾收尾 TURN_TO)不经过 nom_yaw,
 * 与本项无关, 关掉不影响终点航向。 */
#define MF_NOM_HEADING_PERSIST_EN   0
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
