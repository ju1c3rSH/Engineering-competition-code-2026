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
