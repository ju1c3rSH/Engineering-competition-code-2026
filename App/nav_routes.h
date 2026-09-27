/**
 * @file    nav_routes.h
 * @brief   8 张段表 + 挡位变换 + 段入口转台表 + 段表自动链续表
 */
#ifndef _NAV_ROUTES_H_
#define _NAV_ROUTES_H_

#include "mf_motion.h"

/* NavTarget_t 契约(与 Brain mission_sm.h 一致): 0~5 任务路线, 6~8 重锚伪路线 */
#define CAR_NAV_ID_COUNT        9u

extern const mf_seg_t *const nav_routes[CAR_NAV_ID_COUNT];   /* 原 static */
extern const uint8_t nav_entry_turn[CAR_NAV_ID_COUNT];       /* 段入口自动转台码(0=不动); 原 static const */
extern const uint8_t nav_chain_next[CAR_NAV_ID_COUNT];       /* 段表自动链续(0=不续); 原 static const */
const mf_seg_t *Nav_GearTable(uint8_t nav_id, uint8_t gear);

#endif /* _NAV_ROUTES_H_ */
