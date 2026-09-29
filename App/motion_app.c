#include "motion_app.h"

/* ================== 运动框架实例 ==================
 * 增益为旧环等效换算值, 台架已按蛇形修复重调(2026-09);
 * 折算过程与调参史(别再升过 0.7 等): 见 docs/坑-编码器与里程.md §5 */
static mf_ctx_t g_mf;

mf_ctx_t *mf_app(void)
{
	return &g_mf;
}

void mf_app_init(void)
{
	static const mf_cfg_t cfg =
	{
		/* 等效轨距标定: 原地转 90°, 调到编码器差分航向与 IMU 一致
		 * (步骤见 Motion/README.md「接入 panzer 的步骤」第 3 步;
		 *  本值同时是转向链增益、不可当半径旋钮 —— 见「track_width_m 的四处作用链与禁忌」) */
		.track_width_m = 0.25f,
		.half_length_m = 0.0f,
		.half_width_m  = 0.0f,
		.max_vx_mps    = 1.5f,           /* 覆盖 High_speed 1.41m/s */
		.max_vy_mps    = 0.0f,
		.max_wz_dps    = 150.0f,         /* Middle 弧线稳态 57dps + 修正余量 */
		.yaw_kp        = 16.0f,    /* 走直线蛇形修复: 外环降带宽匹配轮速内环(原32.4欠阻尼~1s极限环) */
		.yaw_ki        = 0.0f,     /* 2026-09-21: 2.0 未做 A/B 且实车反而更差, 回退到 09-20 可跑值 */
		.yaw_kd        = 0.8f,     /* 航向环加阻尼项, 抑制匀速直行左右摆 */
		.yaw_i_max     = 0.0f,     /* 与 yaw_ki 同步回退(ki=0 时 I 项本就不累积) */
		.yaw_out_max   = 172.8f,
		.pv_kp         = 2.0f,
		/* 2026-09-29 去 π(口径 1 单位 = 1cm): 增益输入是口径 m/s, 同一物理误差在口径域里小
		 * π 倍, 故 kp/ki 必须 ×π、i_max(钳在**原始**积分上)÷π 才逐点等效。只删 π 不改这
		 * 三个值 = 轮速环增益暗降 3.14 倍。折算与调参史: docs/坑-编码器与里程.md §5 */
		.wheel_kp      = 1.885f,   /* 0.6×π: 蛇形修复取 0.6(去 π 前口径), 别再升过 0.7 */
		.wheel_ki      = 21.99f,   /* 7.0×π: 恢复静摩擦穿透能力 */
		.wheel_i_max   = 0.0446f,  /* 0.14÷π: 与 ki 成对, ki×i_max≈0.98 duty(巡航需≈0.67) */
	};
	mf_init(&g_mf, &cfg);
}
