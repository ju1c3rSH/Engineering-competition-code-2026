/* 固件形态配置: 这里的每个值都是比赛形态默认值。
 * 自测(Selftest/selftest.h)在 SELFTEST_EN=1 时按需覆盖其中几项 —— 覆盖集中在那一个
 * 文件里, 正式代码(Control_Car/main/interurp)不出现任何 #if, 只调 selftest.h 的接缝宏。 */
#ifndef _CAR_CONFIG_H_
#define _CAR_CONFIG_H_

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

/* 调试口(USART2 与它的 RX DMA)的抢占优先级。必须低于控制拍 TIM4(1) 与转台 TIM5(2):
 * 调试口是纯观测通道, 不该抢占 10ms 控制环。UART4(底盘链路)/USART3(HWT101)/USART6(转台)
 * 是控制链路, 保持 0。usart.c 与 dma.c 两处都取这一个宏 —— 单点改 */
#define DEBUG_IRQ_PRIO           3u

#endif
