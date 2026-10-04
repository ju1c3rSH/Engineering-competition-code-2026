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

#include <stdint.h>   /* MF_WZ_RATE_FB_TICKS 用 uint32_t 做浮点→整数转换。本文件历史上
                      * 全文无 include, 靠"包含者恰好先include 了 stdint"才编过 —— 宿主端
                      * 回归(独立编译单元)换个包含顺序就会报unknown type name */

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
/* 控制周期(panzer TIM4=5ms)。改周期必须同步核对 4 处(清单见 §8.13): ① panzer.ioc TIM4.Period
 * (CubeMX 重生成源头, 漏改静默回退) ② MF_WZ_RATE_FB_TICKS ③ MF_REFF_STALE_TICKS(保 1.0s 语义)
 * ④ App/car_units.h 的 CAR_TICK_MS。CARSENSE_WZ_DIFF_TICKS **不随本宏缩放**(受 Car_Sense.c
 * 固定两样本实现约束, 见该文件注释)。 */
#define MF_DT_S             0.005f
#endif

/* 航向差分窗口**上限**(拍)= 每拍帧数最简分数的分母; 现 200Hz÷200Hz=1:1 零交替, 故 mf_pid 的
 * d_win 恒 1(采样层从未下发 yaw_diff_win), 航向微分项现配置下实际为逐拍差分, 正确。
 * 本宏只是**上限**: 实际窗口由 Car_Sense.c 的 CARSENSE_WZ_DIFF_TICKS 决定(改成 1 会让反馈翻倍 2 倍)。§8.6/§8.11/§8.13 */
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

/* 到位余量无宏: 判据"从上方穿过", 残留落窗宽=一拍步长 step=[tol−step,tol), 须 tol=step/2 居中;
 * step 是时序量(直线 |v|·dt / 弧 |v/R|·dt, 见 mf_path.c), 必须运行期算。推导实测: docs/坑-编码器与里程.md §8.8/§8.10 */
/* 弧没走完时的最低前向速度(m/s)。末段弧下一段要停稳(v_next=0)时抛物线减速把 vx 减到 0,
 * 若 remain_yaw 仍 > 到位余量就段卡死; 仅当前向目标被减到本值以下时抬到本值, 非额外加速。0=不要兜底 */
#ifndef MF_ARC_CREEP_MPS
#define MF_ARC_CREEP_MPS    0.15f
#endif
/* 弧线角速度指令的**径向 trim**(物理半径旋钮): ω_sp_eff=(v/R_cmd)·本值 ⇒ R_true=R_cmd·(1−纵向打滑)/本值。
 * 本值是能改变稳态真半径的**唯一**旋钮(其余增益全被速率内环吸收, odom 也看不见: R≡R_cmd 自证),
 * 只能**用卷尺标**: 走 90° 弧量出口参考点垂距 R_tape, 迭代 本值_新=本值_旧×R_tape/942.48。
 * 同一测量推得真实有效轮径≈156.3mm(固件假设 150), 且段表 MOVE 距离按同比例 ×0.96 缩, 两处一起改终点才回理想点。
 * 1.0 = 不修; 左右弧不对称时拆成两宏各标一次(旧 MF_DIFF_BOOST 的分工)。 */
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
 * MF_ODOM_ARC_SRC=1 之后本项的**输入含义变了**: e_y 从"轮速环跟踪误差"变成真实横向偏差
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
/* 履带转弯实测角速度只有差速指令的 k 倍(k<1), 弧线**全程**吃这亏损, 属确定性前馈, 不能只靠反馈慢补。
 * 本值=速率内环输出侧基线, **只作用于弧线**(直线段为 1.0: 直线纠偏仅几 dps, 其环 kp=16 按未补偿 k 标定,
 * 一起乘会推向极限环)。标定(不许看 odom 的 EY, 见 §7.9): ① 弧耗时=90°/(k·v/R) ② 卷尺量弦长。
 * 2026-09-30 复标: 弧上轮差≈520mm/s⇒几何转速119dps, 车体真实43.5dps ⇒ k=0.37~0.40, 1/k≈2.5(旧的1.70只兑68%,
 * 余32%靠PI积分补, 而PI正是扰动源, 见 MF_WZ_RATE_KP)。取 2.40 留量给 PI 微调; |I| 长期非0时按 RATE: W/O 复标。 */
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
/* 0=旧行为: ds_for_xy=ds_enc×(Δψ_imu/Δθ_enc), R_odom≡ds_enc/Δθ_enc 只含轮子量(打滑约掉)⇒弧半径恒=指令半径,
 * ARC 的 e_y/e_psi 结构性看不见半径错; 且一旦速率内环补偿落地(ω_out=wz_sp/k), R_odom=k·R_cmd≈555mm 会让横向项反向推−97dps。
 * 只在"补偿坏掉"时自洽, 留作 A/B(推导与实测: docs/坑-编码器与里程.md §8)。
 * 1=距离直接取轮速均值 ds_enc ⇒ R_odom=ds_enc/Δψ_imu=**实测半径**, e_y/e_psi 复活; 切到 1 必须同时重标横向项(它变真横向环)。 */
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
 * 必须"同间隔": 航向是 0x53 帧到达才步进的零阶保持值, 控制拍 200Hz(5ms); 位移每拍累加、航向只在收新帧时推进。
 * 结算阈值 3° ⇒ 弧上(43dps)约 70ms 一次、整条 90° 弧约 30 次; 0.0055° 航向量化误差摊到 3° 仅 0.18%。
 * 用途: ① 观测(调试口 RAD: 行) ② 供里程计形状与速率内环前馈 1/k̂。 */
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
#define MF_REFF_STALE_TICKS    200u    /* 这么久没**成功结算**即判"当前没有新鲜半径"(≈1.0s @200Hz) */
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
/* MF_WZ_RATE_EN=1 时本项被屏蔽(见速率内环): 打滑改由实测转速反馈补, 再乘 boost 属双重补偿; 保留此旋钮只为关内环做 A/B。 */
/* 标定: 看串口 ARC:<段> EY=<mm> —— EY>0(弧外) 按 boost×(1+EY/942.5) 调大, 变负(切内)即调回, 目标 |EY|<20mm。
 * 置 EN=0 = A/B。该式是相对修正、无量纲, 不受 EY 的 pose 比例误差影响; "绝对半径"仍须卷尺/arc_radius_probe 复核。推导与旧值: Motion/README.md §5 */
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
/* 1=外环 wz 降级为设定值, 再对 IMU 实测角速度闭环: wz_out=wz_sp+PI(wz_sp−ω_meas); 打滑亏损由积分补⇒稳态ω_meas→wz_sp, 与k无关。
 * 开环猜法 MF_DIFF_BOOST 随之失效(mf_kinematics.c 已屏蔽), 留作 A/B; 反馈是实测转速, 不参与里程/path 进度。0=A/B 基准。
 * 反馈不可用时(mf_rate.h fb_valid=0/反号)本环出"开环基线+冻结积分", 绝**不**退成纯 wz_sp(旧版编译期二选一 ⇒ 反馈失效补偿归零, 弧半径×1/k)。现场判据看串口 RATE: 行。 */
/* 置 1 前必须先重标外环: 内环把"wz指令→实际角速度"等效增益从 k·boost≈0.69 抬到 1.0(×1.45)并附加约0.23s闭环滞后 ⇒ 航向环 kp/kd 与 MF_ARC_XTRACK_K 作用点全变。台架 A/B(EN=0 vs 1)没做完前置 1 = 上了未定点环。见 docs/坑-编码器与里程.md §7.5 */
/* 默认已置 1(现场 A/B 完成: 开内环半径误差/终点显著优于开环 boost)。复标结论见 Motion/README.md。 */
#ifndef MF_WZ_RATE_EN
#define MF_WZ_RATE_EN     1
#endif
/* 速率环 PI(dps误差→dps输出): kp 管瞬态, ki 把稳态拉满(闭环 τ≈(1+kp·k)/(ki·k), 只有 ki 能到 100%)。
 * 2026-09-30 实拍: kp=1.0/ki=12 时内环 ~4Hz ±110dps 摆动、轮速环跟不上(滞后半周期)⇒"转向一卡一卡"+履带横刮,
 * 根因是执行器带宽(轮速环 ~1.5Hz)低于本环想用带宽(拐点 ki·k=4.8rad/s 已逼近)。降到 kp=0.4/ki=5⇒拐点 2rad/s、PM≈60°。
 * 前馈(gain0=1/k̂)扛大头, 本环只当慢速微调 —— "慢"不损半径精度, 别按"越快越好"往回加。 */
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
/* 实测角速度来源: 0=航向差分(默认, 同路 IMU 航向导数, 符号定义上不会错); 1=HWT101 0x52 角速度帧(近零延迟、无差分量化噪声)。
 * 0x52 的字节偏移/量纲/校验已对维特官方协议逐项核实; 只剩**符号方向**文档没给 ⇒ 仍需台架确认:
 * 匀速转 ~90dps, 同时打印 Hwt101_GetWzDps()(0x52) 与航向差分, 同号同量级才置 1(mf_rate.h 反号自检只是兜底)。 */
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
/* 换算成控制拍数。分母由 MF_DT_S 导出而非写死 —— 写死过(10u), 改拍周期而漏改它
 * 会让反馈超时窗口静默缩一半(350ms→175ms), 且不报错 */
#ifndef MF_WZ_RATE_FB_TICKS
#define MF_WZ_RATE_FB_TICKS       (MF_WZ_RATE_FB_TIMEOUT_MS / (uint32_t)(MF_DT_S * 1000.0f))
#endif
#if (MF_WZ_RATE_FB_TIMEOUT_MS < 10u)
#error "mf_config.h: MF_WZ_RATE_FB_TIMEOUT_MS 不能小于一个控制拍"
#endif
/* 浮点除数恒 ≥1 才安全(否则除零)。#if 里无法对浮点做 (uint32_t) 转换, 只能转成运行时断言;
 * 当前 MF_DT_S = 0.005f 精确舍入为 5.0, 已由反汇编(CMP #0x45 = 70 拍)确认 */
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
/* 1=全程单航向基准: mf_set_heading_ref() 在 GYRO_ZERO 钉下(0°), 之后 mf_path_start() 永不重定锚, nom_yaw 沿几何推进,
 * 直线段会主动把上游残留航向误差纠回。旧行为是每 nav 表首段用实测航向重定基准, 把上腿残差写成下腿基准, 沿腿链累加(~0.25°/腿)。
 * 未调用 set_heading_ref(heading_ref_locked=0)退回按表重定锚(自测无 GYRO_ZERO, 靠此兜底, 基准等价于 main 的 ResetHWT101)。0=A/B 旧行为。 */
#ifndef MF_HEADING_FRAME_EN
#define MF_HEADING_FRAME_EN   1
#endif

/* 旧的"跨表延续"开关: 仅在 MF_HEADING_FRAME_EN=0 时生效。它与单基准的差别只在**锚点**——
 * 它拿首段的实测航向当全程基准(隐式), 单基准拿 GYRO_ZERO 那一刻的 0° 当基准(显式) */
#ifndef MF_NOM_HEADING_PERSIST_EN
#define MF_NOM_HEADING_PERSIST_EN   0
#endif

/* ============ 航向基准偏差（唯一的静态补偿旋钮, 实车标定项）============ */
/* 基准系与真实几何的固定偏差 —— 唯一的静态补偿旋钮, 只在 seg_cmd_yaw("期望角→误差")这一步加;
 * 框架内所有角度不动, 仅下发航向整体旋转, 相对角与 ARC 自解算天然继承此旋转不受影响。
 * 必须收成**一个数**: 旧实现散在 NAV_REBASE_* 三处, 与几何真值混一起分不出"几何错"还是"补偿"(Motion/README.md §11.5 K5)。
 * 标定: 卷尺量终点横偏 ΔY → ΔY/L(L=腿长) → 跑一趟看残余, 变大翻符号重跑; 别用 POSE 的 Y 反解(必收敛到 0)。0.0f=A/B 不补偿 */
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
