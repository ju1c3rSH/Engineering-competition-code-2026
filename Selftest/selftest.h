/* 自测模块接缝: 路线自测的全部代码收在本目录, 正式代码只通过下面 5 个宏调用它。
 * 关门控(SELFTEST_EN=0)时宏全展开为空, 正式代码因此不需要任何 #if。
 *
 *   1 SELFTEST_BOOT_START()              main.c          上电起跑
 *   2 SELFTEST_NEXT_NAV(done_id)         Car_Nav_Task    nav 跑完推进下一条
 *   3 SELFTEST_DRIFT_SAMPLE(m)           Car_Nav_Task    停等漂移采样(mf_is_done 之前)
 *   4 SELFTEST_TABLE_POST(src,i,buf,o,c) Nav_GearTable   段表拷贝后插停等
 *   5 SELFTEST_DEBUG_POLL()              Car_DebugTask   RT:/PD: 打印
 *
 * 接缝 2 展开后调用 Nav_DoRequest(与 Brain 下发同一入口、同上下文), 只能在 Control_Car.c 用。 */
#ifndef _SELFTEST_H_
#define _SELFTEST_H_

#include "car_config.h"
#include "mf_motion.h"

#if SELFTEST_EN

/* 自测形态下 TURNTABLE_ENABLE / DBG_PERIODIC_DEFAULT_EN / NAV_GEAR_BUF_LEN 由
 * car_config.h 按 SELFTEST_EN 切换(单点, 避免各编译单元取值不一致) */

/* 返回 1 = 该起新段, next_id 与 gear 为下发参数; 0 = 不接管(跑完或非本序列的 nav) */
uint8_t Selftest_NextNav(uint8_t done_nav_id, uint8_t *next_id, uint8_t *gear);
void    Selftest_BootStart(void);
void    Selftest_DriftSample(const mf_ctx_t *m);
void    Selftest_DebugPoll(void);
void    Selftest_TablePost(const mf_seg_t *src, uint8_t i,
                           mf_seg_t *buf, uint8_t *o, uint8_t cap);

#define SELFTEST_BOOT_START()                 Selftest_BootStart()
#define SELFTEST_NEXT_NAV(done_id)                                   \
	do {                                                             \
		uint8_t _st_n, _st_g;                                        \
		if (Selftest_NextNav((done_id), &_st_n, &_st_g) != 0u)       \
		{                                                            \
			Nav_DoRequest(_st_n, _st_g);                             \
		}                                                            \
	} while (0)
#define SELFTEST_DRIFT_SAMPLE(m)              Selftest_DriftSample(m)
#define SELFTEST_DEBUG_POLL()                 Selftest_DebugPoll()
#define SELFTEST_TABLE_POST(src,i,buf,o,cap)  Selftest_TablePost((src),(i),(buf),(o),(cap))

#else

/* 比赛形态 */
#define SELFTEST_BOOT_START()                 ((void)0)
#define SELFTEST_NEXT_NAV(done_id)            ((void)(done_id))
#define SELFTEST_DRIFT_SAMPLE(m)              ((void)(m))
#define SELFTEST_DEBUG_POLL()                 ((void)0)
#define SELFTEST_TABLE_POST(src,i,buf,o,cap)  ((void)0)

#endif

#endif
