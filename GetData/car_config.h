/* 固件形态配置: 这里的每个值都是比赛形态默认值。
 * 自测(Selftest/selftest.h)在 SELFTEST_EN=1 时按需覆盖其中几项 —— 覆盖集中在那一个
 * 文件里, 正式代码(Control_Car/main/interurp)不出现任何 #if, 只调 selftest.h 的接缝宏。 */
#ifndef _CAR_CONFIG_H_
#define _CAR_CONFIG_H_

#include "mf_config.h"

/* 路线自测总闸: 1 = 编入上电自跑全程路线 + 停等插桩 + 漂移诊断; 0 = 比赛形态 */
#define SELFTEST_EN              0u

/* 下面三项随形态切换。写在这里而不是 selftest.h 里 #undef 覆盖, 是为了让任何
 * 编译单元拿到的值都一致(只在 selftest.h 覆盖的话, include 链不同的文件会取到
 * 不同值, 且不报错) */
#if SELFTEST_EN
#define TURNTABLE_ENABLE         0u   /* 自测只验证底盘路线: 转台挂在车上碍事 */
#define DBG_PERIODIC_DEFAULT_EN  1u   /* 录下每段停等时的 X/Y/YAW */
#define NAV_GEAR_BUF_LEN         24u  /* 9 运动段 + 逐段停等 9 + END */
#define NAV_CHAIN_1_TO           0u   /* 自测不链续: 序列由 SELFTEST_NEXT_NAV 接管 */
#define NAV_CHAIN_4_TO           0u
#else
#define TURNTABLE_ENABLE         1u   /* 转台正常驱动电机 */
#define DBG_PERIODIC_DEFAULT_EN  0u   /* 调试口不主动刷屏('p' 仍可随时开) */
#define NAV_GEAR_BUF_LEN         12u  /* 最长表 tbl_bomb: 9 段 + END */
#define NAV_CHAIN_1_TO           6u   /* NAV_TO_BOMB(1) 跑完自动续跑 NAV_REBASE_BOMB(6) */
#define NAV_CHAIN_4_TO           8u   /* NAV_TO_HOSTAGE(4) 跑完自动续跑 NAV_REBASE_HOSTAGE(8) */
#endif

/* 转台调试口禁令: 0 = 屏蔽 USART2 台架直转('x''w''e''1'~'7'), 1 = 台架标定时打开。
 * 必须默认 0: 台架链路是裸单字节、无帧无校验, 仅 panzer 上电时 USART2 RX 悬空/受扰,
 * 噪声字节被 Uart2_task 逐字节扫描, 恰为 '1'~'7'/'w'/'e' 即驱动转盘 —— 上电阻塞期
 * (BootZero 前)攒下的会被 BenchDrop 作废, BootZero(≈12s)之后到达的立即执行,
 * 即"只上 panzer 过十几秒转盘自动旋转"。禁令同时挡掉噪声 'x' 在正式任务中
 * 误触 Turntable_Stop 掐断协议驱动的转台动作。开关点: nav_turntable.c 三个 Bench 入口 */
#define TURNTABLE_BENCH_EN       0u

/* 调试口(USART2 与它的 RX DMA)的抢占优先级。必须低于控制拍 TIM4(1) 与转台 TIM5(2):
 * 调试口是纯观测通道, 不该抢占 10ms 控制环。UART4(底盘链路)/USART3(HWT101)/USART6(转台)
 * 是控制链路, 保持 0。usart.c 与 dma.c 两处都取这一个宏 —— 单点改 */
#define DEBUG_IRQ_PRIO           3u

/* 实测角速度来源: 0=航向差分 1=HWT101 0x52 帧。置 1 前必须台架确认符号(悬案: 接手核实报告 §8) */
#define CARSENSE_WZ_SRC_GYRO_EN  0

/* 实测角速度上限(dps), 超限=阶跃/伪帧, 丢弃。400=实车转速上限, 非模块量程 */
#define CARSENSE_WZ_MEAS_MAX_DPS  400.0f

/* 反馈新鲜度(ms), 超时判停更、内环退纯前馈。350=10Hz 下容连丢 2 帧(坑-编码器与里程.md §7) */
#define CARSENSE_WZ_FB_TIMEOUT_MS 350u
/* 换算成拍; 分母必须从 MF_DT_S 导出, 写死会在改拍时静默缩半 */
#define CARSENSE_WZ_FB_TICKS \
    (CARSENSE_WZ_FB_TIMEOUT_MS / (uint32_t)(MF_DT_S * 1000.0f))
#if (CARSENSE_WZ_FB_TIMEOUT_MS < 10u)
#error "car_config.h: CARSENSE_WZ_FB_TIMEOUT_MS 不能小于一个控制拍"
#endif

#endif
