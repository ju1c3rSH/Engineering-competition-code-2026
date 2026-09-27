/**
 * @file    motion_app.h
 * @brief   运动框架实例(mf_ctx 单例) + 台架标定后的 mf_cfg 配置块
 */
#ifndef _MOTION_APP_H_
#define _MOTION_APP_H_

#include "mf_motion.h"

mf_ctx_t *mf_app(void);
void      mf_app_init(void);

#endif /* _MOTION_APP_H_ */
