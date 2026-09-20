# Motion 运动执行框架

把双轮差速库（`F:\stmtest\Lib\motion_ctrl`）和麦轮库
（`F:\engineering-training-competition\USER\Hardware\Mecanum.c`）的逻辑互补，
整合成一套运动执行框架，最终被 panzer 履带底盘使用。

## 架构

```
HAL (电机/编码器/IMU)
  -> 里程计 mf_odom        (位姿 x,y,yaw；两套运动库里程计逻辑的并集)
  -> 段执行器 mf_path      (声明式路径：增量 + 绝对 + 转向)
  -> 逆解 mf_kinematics    (vx,vy,wz -> 各轮线速度 m/s)
  -> 轮速环 mf_pid         (PWM+DIR 模式；驱动器闭环模式跳过)
  -> HAL
```

## 文件

| 文件 | 说明 |
|---|---|
| `mf_config.h` | 编译期宏：底盘类型 / 电机模式 / 航向源，切换底盘只改这里 |
| `mf_types.h` | 位姿、车体指令、声明式段类型（MOVE_DIST/ARC/TURN_TO/TURN_REL/GOTO_XY/GOTO_REL/WAIT/END） |
| `mf_hal.h` | 硬件抽象：motor_cmd / encoder_dist_m / imu_yaw_deg / tick_ms |
| `mf_pid.h/.c` | 位置式 PID（航向环、点到点、轮速环共用） |
| `mf_odom.h/.c` | 里程计（差速/履带/麦轮分支编译，IMU 优先） |
| `mf_kinematics.h/.c` | 逆运动学 |
| `mf_path.h/.c` | 段执行器 |
| `mf_motion.h/.c` | 顶层 API：mf_init / mf_tick / 路径与手动模式 / 单步便捷指令 |
| `mf_hal_panzer.c` | panzer HAL 适配 + 示例路线（模板，含台架核对清单） |

## 底盘切换

```c
/* mf_config.h（或 Keil Define 里定义） */
#define MF_CHASSIS_TYPE   MF_CHASSIS_TRACK    /* 差速: DIFF / 履带: TRACK / 麦轮: MECANUM */
#define MF_MOTOR_MODE     MF_MOTOR_PWM_DIR    /* 框架闭环 PWM / 驱动器闭环速度指令 */
#define MF_YAW_SOURCE     MF_YAW_SRC_IMU      /* IMU / 编码器差分 */
```

轮序约定：`0=左(前左) 1=右(前右) 2=左后 3=右后`。

## 声明式路径示例（履带）

```c
const mf_seg_t route[] =
{
    { .type = MF_SEG_MOVE_DIST, .reset_pid = 1,
      .p.move_dist = { .speed_mps = 2.0f, .dist_m = 6.5f, .heading_deg = 0.0f } },
    { .type = MF_SEG_ARC, .reset_pid = 1,
      .p.arc = { .speed_mps = 1.5f, .radius_m = -0.30f, .d_yaw_deg = -90.0f } },
    { .type = MF_SEG_GOTO_XY, .reset_pid = 1,
      .p.goto_xy = { .x_m = 3.0f, .y_m = -2.0f,
                     .yaw_deg = -90.0f, .v_max_mps = 1.0f, .w_max_dps = 60.0f } },
    { .type = MF_SEG_END }
};

mf_start_path(&mf, route);   /* 10ms 中断里跑 mf_tick() */
if (mf_is_done(&mf)) { ... } /* 段间插入动作（转盘/抓取/等主控） */
```

## ARC 段：为什么航向设定值的来源是关键

弧线段只有**航向**进环（没有横向位置进环），所以"设定值的偏航速率"就等于实际弧半径：

```
实际半径 ≈ 车体真实前进速度 / 设定值偏航速率
```

旧实现取 `yaw_sp = entry.yaw + dist_accum/R`，而 `dist_accum` 累的是**轮上位移均值**
（`mf_motion.c` 里 `(wheel[0].delta + wheel[1].delta)/2`）。履带转弯时履带面比车体多走
（见 `mf_odom.c` 对横向刮擦的说明），这个均值系统性偏大，于是设定值偏航速率偏快、
航向环被迫多转，**实际半径被压成 R/k 且每条弧都往里切**，误差随打滑线性放大。里程计
x/y 走的是打滑已修正的 `ds_for_xy`，但段进度走的是没修正的均值——同一件事两把尺子。

现在（`MF_ARC_XTRACK_EN=1`）：进段时把名义圆弧解算到世界系（圆心 = 进段位姿 + R·左法线，
半径带符号，左/右转一条式子通用），段内设定值取**名义切线航向** = 圆心→车体方位角 +
sgn·90°。它由里程计位置推出，位置积分用的是 `ds_for_xy`，所以设定值的偏航速率天然等于
"真实车体位移/R"。另加一项横向偏差比例（`MF_ARC_XTRACK_K`，dps per m），让"离弧多远"
本身也进环——旧律对与弧等半径的纯横向平移是完全不可见的（误差恒为 0，永不纠正）。
完成判据仍用 IMU 实测航向差，抗打滑且与"转 d_yaw 度"语义严格一致。

量化（`build_verify/mf_path_test/arc_xtrack_sim.py`、`arc_route_sim.py`）：

| 履带打滑 | 旧律每条弧出口横向偏差 | 新律 |
|---|---|---|
| 5%  | −48 mm  | 0~1 mm |
| 15% | −111 mm | 0~1 mm |
| 20% | −136 mm | 0~1 mm |

全链 6 条弧累积：打滑 15% 时终点位置偏差 586 mm → 77 mm。

现场验收看调试口的 `ARC:<段号> EY=<mm> YE=<0.1°>`：`EY` = 出口横向偏差（正=弧外），
`YE` = 出口航向偏差——后者会被紧随的 `MF_KEEP` 直线当基准锁住，所以那条直线会不会
整段走偏就看它。**注意**：`EY` 是机上里程计给出的估计，纯"里程刻度"类误差（比如等效
轨距标定偏差导致的转弯里程刻度）它会跟着一起偏，看不到——那类误差要靠外部基准（场地线/
相机）一次性标定。

## 接入 panzer 的步骤

1. Keil 工程加上 `Motion/*.c`（`mf_hal_panzer.c` 里的工程头文件取消注释后加入）。
2. `mf_hal_panzer.c` 顶部 TODO 三件事必须在台架上核对：
   符号（前进位移为正）、方向电平（value>0 前进）、驱动轮直径 WHEEL_DIA_M。
3. 标定等效轨距 `track_width_m`：原地转 90°，调到编码器差分航向与 IMU 一致。
4. 把 `Car_Drive_route()` 的 flag 状态机逐段翻译成段表
   （原 Car_Pos 是“圈数×15”，需按驱动轮周长换算成米）。
5. 主控握手点（等 Boss_State / 打靶 / 人质区）保留在上层状态机，
   用 `mf_is_done()` + `mf_stop()` / 下一条 `mf_start_path()` 接力。

## 单位约定

距离 m，速度 m/s，角度 deg（多圈连续，逆时针为正），
角速度 deg/s。航向语义与 HWT101 一致。
