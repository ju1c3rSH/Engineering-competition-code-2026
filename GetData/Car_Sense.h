/**
 * @file    Car_Sense.h
 * @brief   运动库的采样/存储/执行层
 *
 * 采集与存储集中在本层：固定周期把编码器与航向写成带序号戳的快照，
 * Motion 库 mf_tick 只读快照做解算（库内不碰硬件），轮输出再由本层写硬件。
 */
#ifndef _CAR_SENSE_H_
#define _CAR_SENSE_H_

#include <stdint.h>
#include "mf_motion.h"

/* 
 * 轮序: [0]左=Motor_1 [1]右=Motor_0, 前进为正;
 *
 *  */
typedef mf_state_in_t CarSense_Snap_t;

void CaSense_Sample(void);              /* 编码器+航向 -> 快照(seq+1) */
const CarSense_Snap_t *CarSense_Get(void);
void CarSense_Output(const mf_ctx_t *m); /* 框架轮输出 wheel[].out -> DIR+PWM */

#endif
