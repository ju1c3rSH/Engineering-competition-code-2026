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

/* 航向差分窗口的**上限**(拍)。实际用几拍由采样层按**实测每拍帧数**决定, 见 Car_Sense.c:
 * 窗口 = 每拍帧数(化最简分数)的**分母** —— 零阶保持阶梯的"整数帧数"交替周期就是这个分母,
 * 差分必须跨满一个周期才在该根谱线上放零点(等价于一个精确的梳状零点, 不伤慢变真值)。
 *   250Hz÷100Hz = 5/2 → 2 拍    200Hz÷100Hz = 2/1 → 1 拍(无交替)    125÷100 = 5/4 → 4 拍
 * 不许写死窗口: 帧率存在**模块自己的 Flash** 里、可被重配(实测报 250, 而固件配的是 200),
 * 写死 = 把帧率焊死在标定那一刻。见 docs/坑-编码器与里程.md §8.6 */
#ifndef MF_YAW_DIFF_WIN_MAX
#define MF_YAW_DIFF_WIN_MAX   4
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

/* 到位余量**没有宏了**: 行程与转角两类判据都是"从上方穿过", 停车那拍的残留必然落在
 * 一个宽度 = **一拍步长 step** 的窗口 [tol−step, tol) 里 ⇒ 只有 tol = step/2 才能把窗口
 * 中心对齐到 0(系统偏差消失, 只剩 ±step/2 散差)。step 是**时序量**, 随档位/半径/拍周期变:
 *   直线  step = |v|·dt        (mf_path.c: seg_move_dist)
 *   弧线  step = |v/R|·dt      (mf_path.c: seg_arc, 实测值优先、名义值兜底)
 * 所以它必须运行期算 —— 历史教训: 弧线曾经写死 1.0°(均值 +0.79°/弧 ⇒ 四弧 +65mm),
 * 改成写死 0.2° 只是"按 43dps、10ms 拍"这一个工作点对了, 换档/降速/改拍周期立刻回到
 * 系统偏差。不是余量的问题, 是"拿常数量一个时序量"的问题。推导与实测见
 * docs/坑-编码器与里程.md §8.8 / §8.10 */
/* 弧线没走完时的最低前向速度(m/s)。末段弧的下一段要求停稳时(v_next=0), 抛物线减速会把
 * vx 一路减到 0 —— 若此时 remain_yaw 还大于到位余量, 车就停在那儿永远走不完(段卡死)。
 * 只在前向目标会被减到本值以下时抬到本值(不是额外加速)。0 = 不要兜底 */
#ifndef MF_ARC_CREEP_MPS
#define MF_ARC_CREEP_MPS    0.15f
#endif
/* 弧线角速度指令的**径向 trim**(物理半径旋钮)。ω_sp_eff = (v/R_cmd)·本值,
 * 于是 物理半径 R_true = R_cmd·(1−纵向打滑)/本值 —— 本值 <1 把弧走**大**, >1 走小(内切)。
 *
 * ⚠ 这是能改变稳态真半径的**唯一**旋钮。下列全都改不动它, 因为全被速率内环吸收了
 *   (内环让 ω_actual = ω_sp, 而 R_true = v_ground/ω_actual):
 *   `MF_ARC_SLIP_GAIN`、`MF_DIFF_BOOST`(开内环后已屏蔽)、`MF_WZ_RATE_KP/KI`、航向环增益、微分项。
 * ⚠ 也别指望 odom 看得见: `R=` = 轮程÷IMU 转角 = v_wheel/ω = R_cmd **恒等**(自证),
 *   `EY` 同理 ⇒ 它只能**用卷尺标**:
 *     ① 沿弧的进段方向拉一根线(或用场地线), 走一条 90° 弧;
 *     ② 量出口车体参考点相对这根线的**垂距 = 物理半径** R_tape(90° 弧出口在 (R,R));
 *     ③ 迭代: `本值_新 = 本值_旧 × R_tape / 942.48`, 再跑一趟看 R_tape 是否收向 942。
 *   2026-09-30 实测: 卷尺量出口垂距 982mm vs 标称 942.5 ⇒ 本值 1.042。
 *   它与同一次测量的另一个推论是同一件事: **真实有效轮径 ≈ 156.3mm**(而固件假设 150)——
 *   因为物理半径恒等于 2π×真实轮径(与固件里的 WHEEL_DIA_M 无关, 它在 MF_M_SPEED 与
 *   TURN_RADIUS_M 之间约掉了), 2π×0.1563 = 982 ✓ 与卷尺一致。
 *   同一比例也作用在直线上: 直线物理长度 = (单位/15)×π×真实轮径 ⇒ 段表 MOVE 距离同样
 *   按 0.96 缩(tbl_bomb 已改)。两处一起改, 终点才回到理想点。
 *   1.0 = 不修。左右弧若不对称, 拆成两个宏各标一次(旧 MF_DIFF_BOOST 就是这么分的) */
#ifndef MF_ARC_WZ_TRIM
#define MF_ARC_WZ_TRIM      1.042f
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

/* 横向偏差增益 dps/m。调法: 0.1m 偏差 -> 25dps, 与弧线前馈 43dps 同量级, 宁小勿大。
 * ⚠ MF_ODOM_ARC_SRC=1 之后本项的**输入含义变了**: e_y 从"轮速环跟踪误差"变成真实横向偏差
 *   (量程从 ±0.1m 变成 ±0.7m)。回路增益 ≈ K×(dR/dω) = 250×0.0219 ≈ 5.5 ⇒ 欠阻尼(PM 只有 ~10°)。
 *   首次上线先置 0 做隔离，跑通后从 60~70dps/m 起复标，不要直接回到 250。 */
#ifndef MF_ARC_XTRACK_K
#define MF_ARC_XTRACK_K      0.0f
#endif
/* 横向项自己的限幅: 只许占几何前馈(v/R)的这个比例。半径的主责归速率内环, 横向项只当微调 ——
 * 否则它会顶到段内限幅 MF_ARC_WZ_MAX_MULT, 两个环抢同一个误差(见 mf_path.h 的职责划分) */
#ifndef MF_ARC_XTRACK_MAX_FRAC
#define MF_ARC_XTRACK_MAX_FRAC  0.5f
#endif

/* ============ 弧线滑移基线（角速度内环的开环前馈，实车标定项）============ */
/* 履带转弯的实测角速度只有差速指令的 k 倍(k<1), 弧线**全程**都吃这个亏损 —— 属确定性
 * 前馈, 不能只靠反馈慢补(反馈要等误差攒出来才出力)。本值由 mf_path 的 slip_gain 传给
 * 速率内环当输出侧基线, **只作用于弧线**; 直线段为 1.0 —— 直线纠偏只有几 dps, 其环
 * 增益(kp=16)是按"未补偿的 k"标定的, 一起乘会把它推向极限环。
 *
 * 标定(两把尺子, 都不许看 odom 的 EY: odom 在弧上的半径**结构性等于 R_cmd**, 见
 * docs/坑-编码器与里程.md §7.9): ① 弧耗时 = 90° / (k·v/R); ② 卷尺量实物弦长。
 * 2026-09-30 二次标定(TRC 段内跟踪直接读轮差): 弧上轮速差 (CR−CL) ≈ 520mm/s ⇒ 轮系几何转速
 * Δv/W = 119dps, 同期车体真实转速(段 Δyaw/Δt) = 43.5dps ⇒ k = 0.37~0.40 ⇒ 1/k ≈ 2.5。
 * 即旧的 1.70 只兑现 68%, 余下 32% 全靠 PI 的积分补 —— 而 PI 正是扰动源(见 MF_WZ_RATE_KP)。
 * 取 2.40(留一点给 PI 微调); |I| 长期明显非 0 时按 RATE: 行的 W/O 复标 */
#ifndef MF_ARC_SLIP_GAIN
#define MF_ARC_SLIP_GAIN     2.40f
#endif

/* ARC 段角速度指令相对"几何前馈 v/R"的允许倍数（弧线自己的限幅）。
 * 弧线的 wz = 航向P + v/R + 横向K·e_y 三项叠加, e_y/e_psi 一被扰动就能叠到
 * cfg.max_wz_dps —— 那是 R≈0.27m 的原地上转, 履带横向刮擦物理上做不到, 只会把车
 * "钉"在原地: 实测 SP 钉在 -150 十几秒、弧再也走不完、SEG 卡在 0(2026-09-30)。
 * 弧线自限幅 => 指令半径最紧也只到 R/倍数 */
#ifndef MF_ARC_WZ_MAX_MULT
#define MF_ARC_WZ_MAX_MULT   1.8f
#endif

/* ============ 里程计形状（弧半径由谁决定，实车标定项）============ */
/* 0 = 旧行为：弧长反解 ds_for_xy = ds_enc×(Δψ_imu/Δθ_enc)。
 *     ⚠ R_odom ≡ ds_enc/Δθ_enc 只含**轮子**量（打滑在分子分母里被约掉）⇒ 弧半径恒等于指令半径，
 *     ARC 的 e_y/e_psi 结构性看不见半径错；且一旦速率内环的补偿真的落地（ω_out = wz_sp/k），
 *     R_odom = v/ω_out = k·R_cmd ≈ 555mm 会让横向项反向推 −97dps（左弧被命令成右转）。
 *     **只在"补偿坏掉"时自洽**，留作 A/B。推导与实测：docs/坑-编码器与里程.md §8
 * 1 = 距离直接取轮速均值 ds_enc（直线段本来就取它）⇒ R_odom = ds_enc/Δψ_imu = **实测半径**，
 *     e_y/e_psi 复活。⚠ 切到 1 必须同时重标横向项（它从"轮速环跟踪误差"变成真横向环） */
#ifndef MF_ODOM_ARC_SRC
#define MF_ODOM_ARC_SRC     1u
#endif
/* 反解比值的钳位（仅 ARC_SRC=0 时用）：单侧空转/单边堵转会让比值异常 */
#ifndef MF_ODOM_RATIO_CLAMP_LO
#define MF_ODOM_RATIO_CLAMP_LO  0.5f
#endif
#ifndef MF_ODOM_RATIO_CLAMP_HI
#define MF_ODOM_RATIO_CLAMP_HI  2.0f
#endif

/* ============ 里程计弧长反解的生效门限（实车标定项，仅 ARC_SRC=0）============ */
/* ds_for_xy = ds_enc × (dyaw_imu / dtheta_enc) 只在**真转弯**时可信：直线段航向纠偏的 wz 仅几 dps，
 * dyaw 小到与 IMU 读数分辨率同级而 dtheta_enc 是编码器小量 ⇒ 退化成"噪声除小量"，撞钳位那一拍
 * 里程凭空翻倍（2026-09-29 实测长腿 sd 104mm）。机理与量化：docs/坑-编码器与里程.md §3.2 */
#ifndef MF_ODOM_RATIO_MIN_DYAW_DEG
#define MF_ODOM_RATIO_MIN_DYAW_DEG    0.17f    /* 每拍最小 IMU 转角 deg */
#endif
#ifndef MF_ODOM_RATIO_MIN_DTHETA_RAD
#define MF_ODOM_RATIO_MIN_DTHETA_RAD  3.0e-3f  /* 每拍最小编码器几何转角 rad */
#endif

/* ============ 弧实测半径 R_eff = Σ轮位移 / |ΣΔψ|（观测口径, 实车标定项） ============
 * 为什么必须"同间隔": 航向是 0x53 帧到达才步进的零阶保持值, 控制拍却是 100Hz(TIM4=84M/840/1000)。
 * 位移每拍累加、航向只在收到新帧时推进, 再累积到足够转角才结算一次 —— 与帧率无关。
 * 结算阈值 3° ⇒ 弧上(43dps)约 70ms 一次、整条 90° 弧约 30 次; 0.0055° 的航向量化误差
 * 摊到 3° 上是 0.18%。它的用途: ① 观测(调试口 RAD: 行) ② 供里程计形状与速率内环前馈 1/k̂ */
#ifndef MF_REFF_MIN_DYAW_DEG
#define MF_REFF_MIN_DYAW_DEG   3.0f
#endif
/* 结算值一阶滤波: 弧上结算率 ≈14Hz ⇒ τ ≈ 结算间隔/α = 70ms/0.30 ≈ 0.23s。
 * α 不能太小: 一条 2.1s 的 90° 弧内 REF 必须走到真值附近才有观测价值(旧值 0.05 ⇒ τ=1.4s ⇒ 只到 78%) */
#ifndef MF_REFF_LPF_ALPHA
#define MF_REFF_LPF_ALPHA      0.30f
#endif
#ifndef MF_REFF_MIN_M
#define MF_REFF_MIN_M          0.20f
#endif
#ifndef MF_REFF_MAX_M
#define MF_REFF_MAX_M          4.00f    /* 合法弧半径窗口, 窗外丢弃(原地转/单侧空转/倒车) */
#endif
/* 阶跃闸(与 mf_rate 的 MEAS_MAX 同目的): 归零(0x53 假同步/GYRO_ZERO)注入的航向跳变不是转速。
 * 单拍闸取 8° = 实车转速上限 400dps 的一拍转角(4°)再留 2 倍余量给帧抖动(一拍拍 2~3 帧)。
 * 窗口闸 40°: 一个结算窗跨过 40° 只可能是阶跃 —— 两道闸都丢窗重来, 免得阶跃两侧的量相除 */
#ifndef MF_REFF_STEP_MAX_DEG
#define MF_REFF_STEP_MAX_DEG   8.0f
#endif
#ifndef MF_REFF_MAX_DYAW_DEG
#define MF_REFF_MAX_DYAW_DEG   40.0f
#endif
/* 窗口"已不可能"阈值: 攒够 3° 之前, 若 |位移|/|转角| 已超过 1.5×MAX_M ⇒ 就地丢窗。
 * 作用是让**直线段永远攒不出半个窗口**(否则下一段转弯会拿它的位移当分子, 见 docs §8) */
#ifndef MF_REFF_IMPLAUSIBLE_M
#define MF_REFF_IMPLAUSIBLE_M  (MF_REFF_MAX_M * 1.5f)
#endif
#ifndef MF_REFF_STALE_TICKS
#define MF_REFF_STALE_TICKS    100u     /* 这么久没**成功结算**即判"当前没有新鲜半径"(≈1.0s) */
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
/* ⚠ MF_WZ_RATE_EN=1 时本项被屏蔽(见下方速率内环): 打滑改由实测转速反馈补,
 *   再乘 boost 属双重补偿。保留此旋钮只为关内环做 A/B。 */
/* 标定: 跑一趟看串口 ARC:<段> EY=<mm> —— EY 仍 >0(弧外) 就按 boost×(1+EY/942.5) 调大,
 * EY 变负(切内)即过头调回。目标 |EY| < 20mm。置 EN=0 = A/B。该式是**相对修正**、
 * 无量纲，故不受 EY 的 pose 比例误差影响; 但"绝对半径"仍须卷尺/arc_radius_probe 复核
 * 2026-09-29 复标(18 弧, 3 次运行): 左 +74.2 / 右 +68.8 mm, sd 2.8, 18/18 同号且已左右对称
 * ⇒ POS 1.12→1.21 / NEG 1.16→1.25。推导与旧值来历: Motion/README.md §5 */
#ifndef MF_DIFF_BOOST_EN
#define MF_DIFF_BOOST_EN    1
#endif
#ifndef MF_DIFF_BOOST_POS
#define MF_DIFF_BOOST_POS   1.21f    /* 左转（wz_dps >= 0）差速放大 */
#endif
#ifndef MF_DIFF_BOOST_NEG
#define MF_DIFF_BOOST_NEG   1.25f    /* 右转（wz_dps <  0）差速放大 */
#endif

/* ============ 角速度内环（IMU 实测转速反馈, mf_rate.h）============ */
/* 1 = 外环输出的 wz 降级为**设定值**, 再对 IMU 实测角速度闭环:
 *       wz_out = wz_sp + PI(wz_sp − ω_meas)
 *     打滑亏损(实际转角只有指令的 k 倍)由积分项补 ⇒ 稳态 ω_meas → wz_sp, 与 k 无关。
 *     开环猜法 MF_DIFF_BOOST 随之失效(mf_kinematics.c 里已按本开关屏蔽), 留作 A/B。
 *     反馈是**实测转速**, 不参与里程/path 进度, 与"两把尺子"无关。0 = A/B 基准。
 *   ⚠ 反馈不可用时(mf_rate.h 的 fb_valid=0 / 反号)本环出"开环基线 + 冻结积分",
 *     绝**不**退成纯 wz_sp —— 旧版编译期把基线与内环做成二选一, 反馈一失效补偿就归零,
 *     弧半径 ×1/k(2026-09-30 实测 0.94m→1.63m)。现场判据看串口的 RATE: 行 */
/* ⚠ 置 1 前必须先重标外环: 内环把"wz 指令 → 实际角速度"的等效增益从 k·boost≈0.69 抬到
 * 1.0(×1.45)并附加约 0.23s 的闭环滞后 ⇒ 航向环 kp/kd 与 MF_ARC_XTRACK_K 的作用点全变。
 * 台架 A/B(EN=0 vs 1)没做完之前置 1, 等于上了一个未定点的环。见 docs/坑-编码器与里程.md §7.5 */
/* ⚠ 2026-09-30 晚改默认为 1: 现场 A/B 已完成 —— 开内环那几趟(18:06/18:18/18:30)
 * 实测 R=943~947(名义 942.5)、EY≤2mm、终点 36~74mm; 关内环走开环 boost 的趟
 * (19:08/19:19) 实测 R=1043~1126、EY=+150~+214mm、终点 826mm。**一个数量级**。
 * 且开环的 boost 必须按地面重量(今晚 k=0.695/0.706 ⇒ 要 1.44/1.42, 而历史提交里
 * 是 1.12~1.25) —— 内环把这件事从"每晚猜"变成"实时测"。 */
#ifndef MF_WZ_RATE_EN
#define MF_WZ_RATE_EN     1
#endif
/* 速率环 PI(dps 误差 -> dps 输出)。kp 管瞬态, ki 管打滑亏损的稳态收敛——
 * 闭环时间常数 ≈ (1+kp·k)/(ki·k), 只有 ki 能把稳态拉到 100%。
 * ⚠ 2026-09-30 实拍（TRC: 20Hz 段内跟踪）：kp=1.0/ki=12 时内环输出在 ~4Hz 上以 ±110dps 摆动、
 *   轮速环跟不上（实测轮速滞后约半个周期）⇒ "转向一卡一卡" + 履带横刮。根因是**执行器带宽**
 *   （轮速环闭环 ~1.5Hz）低于本环想用的带宽：本环积分拐点 ki·k = 4.8rad/s(0.76Hz) 已逼近它。
 *   降到 kp=0.4/ki=5 ⇒ 拐点 2rad/s(0.32Hz)、相位裕度 ≈60°。前馈（gain0=1/k̂）扛大头、本环只当
 *   慢速微调，所以"慢"不损失半径精度 —— 别按"越快越好"往回加 */
#ifndef MF_WZ_RATE_KP
#define MF_WZ_RATE_KP     0.4f
#endif
#ifndef MF_WZ_RATE_KI
#define MF_WZ_RATE_KI     5.0f
#endif
/* 积分上限(单位 deg, 即 ∫误差dt): 稳态所需 I≈(1/k−1)·wz_sp/ki ≈ 3.6(k=0.57,57dps),
 * 取 20 留打滑加重/低速段的余量; 再大只会在堵转时攒出更难退的积分 */
#ifndef MF_WZ_RATE_I_MAX
#define MF_WZ_RATE_I_MAX  20.0f
#endif
/* 实测角速度来源: 0 = 航向差分(默认; 同一路 IMU 航向的导数, 符号定义上不会错);
 * 1 = HWT101 0x52 角速度帧(近零延迟、无差分量化噪声)。
 * 0x52 的字节偏移/量纲/校验已对维特官方协议逐项核实(interurp.c 解析与文档一致),
 * 剩下**只有符号方向**文档没给 ⇒ 仍需台架确认: 手动把车匀速转 ~90dps, 同时打印
 * Yaw_struct.Wz_dps 与航向差分, 同号同量级才置 1。内环的反号自检(mf_rate.h)是兜底,
 * 不足以替代这一步 */
#ifndef MF_WZ_RATE_GYRO_FRAME_EN
#define MF_WZ_RATE_GYRO_FRAME_EN  0
#endif
/* 航向差分速率(deg/s)的物理上限: 超过即判为航向阶跃(GYRO_ZERO 置零/协议假同步),
 * 本拍不给反馈并就地重播种。取 400 = 实车转速上限(不是模块量程: HWT101 满量程 ±2000dps) */
#ifndef MF_WZ_RATE_MEAS_MAX_DPS
#define MF_WZ_RATE_MEAS_MAX_DPS   400.0f
#endif
/* 反馈新鲜度超时(ms): 连续这么久没收到 0x53 帧即判传感器停更, 内环退回纯前馈。
 * 必须容得下"最慢帧率 + 连续丢 2 帧": 出厂 10Hz(100ms) → 丢两帧 = 300ms ⇒ 取 350 */
#ifndef MF_WZ_RATE_FB_TIMEOUT_MS
#define MF_WZ_RATE_FB_TIMEOUT_MS  350u
#endif
/* 换算成控制拍数(10 = MF_DT_S 的毫秒数) */
#ifndef MF_WZ_RATE_FB_TICKS
#define MF_WZ_RATE_FB_TICKS       (MF_WZ_RATE_FB_TIMEOUT_MS / 10u)
#endif
#if (MF_WZ_RATE_FB_TIMEOUT_MS < 10u)
#error "mf_config.h: MF_WZ_RATE_FB_TIMEOUT_MS 不能小于一个控制拍(MF_DT_S = 10ms)"
#endif
/* 一阶低通(Hz): 太狠会吃掉内环相位裕度。15Hz 对应差分噪声 0.5dps -> 约 0.2dps */
#ifndef MF_WZ_RATE_LPF_HZ
#define MF_WZ_RATE_LPF_HZ 15.0f
#endif

/* 内环输出上限的倍数(相对 cfg.max_wz_dps): 内环存在的意义就是把设定值放大 1/k
 * (k≈0.57~0.63, 即 MF_ARC_SLIP_GAIN)。若把输出上限也设成**外环自己的限幅值**,
 * 则 |wz_sp| ≥ max_wz_dps/倍数 之后输出直接撞上限, 放大倍数被削回 1.0 —— 补偿
 * 算出来了却发不出去。2026-09-30 实车 RATE:SP=-150 O=-150 B=-255 即此形态。
 * 输出侧必须留出补偿余量, 上限要显式高于外环限幅 */
#ifndef MF_WZ_RATE_OUT_HEADROOM
#define MF_WZ_RATE_OUT_HEADROOM   1.8f
#endif
/* 反号自检的死区(deg/s): 自检拿**原始实测值**判号, 车静止/慢转时航向差分只剩量化
 * 噪声(±0.5~1.4dps), 输出侧却被开环基线顶到几十 dps => 噪声每换一次号就误判一次。
 * 实测一趟 SN=1399/OK=3082: 32% 的拍走"退基线"分支, 每次还清 primed => 低通永远
 * 在重启、PI 从不累积。低于本死区的实测值一律不参判 */
#ifndef MF_WZ_RATE_SIGN_MIN_DPS
#define MF_WZ_RATE_SIGN_MIN_DPS   8.0f
#endif

#if (MF_WZ_RATE_EN != 0) && (MF_YAW_SOURCE != MF_YAW_SRC_IMU)
#error "mf_config.h: MF_WZ_RATE_EN 需要 MF_YAW_SOURCE=IMU —— 速率内环的反馈量就是 IMU 角速度"
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
