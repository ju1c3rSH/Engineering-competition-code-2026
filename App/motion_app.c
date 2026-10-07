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
		/* 里程计形状来源: 1 = 距离取轮速均值 ⇒ 弧半径变成**实测值**(e_y/e_psi 才活)。
		 * 0 = 旧弧长反解(A/B 用, 只在补偿坏掉时自洽)。见 mf_config.h 的推导 */
		.odom_arc_src  = (uint8_t)MF_ODOM_ARC_SRC,
		.max_vx_mps    = 1.5f,           /* 覆盖 High_speed 1.41m/s */
		.max_vy_mps    = 0.0f,
		.max_wz_dps    = 150.0f,         /* 航向环(外环)自己的物理边界, 与速率内环的输出余量
					                       * (MF_WZ_RATE_OUT_HEADROOM)是两件事, 别拿它当内环上限。
					                       * 内环稳态需求见 docs/全程高速实现指引.md §2 */
		.yaw_kp        = 16.0f,    /* 走直线蛇形修复: 外环降带宽匹配轮速内环(原32.4欠阻尼~1s极限环) */
		.yaw_ki        = 0.0f,     /* 2026-09-21: 2.0 未做 A/B 且实车反而更差, 回退到 09-20 可跑值 */
		.yaw_kd        = 0.8f,     /* 航向环加阻尼项, 抑制匀速直行左右摆。
				                * kd **不随MF_DT_S 缩放**: 微分项是 Δerr/(w·dt), 误差恒斜率
				                * 变化时分子随 dt 一起减半, 与分母相消(推导见 §8.13) */
		.yaw_i_max     = 0.0f,     /* 与 yaw_ki 同步回退(ki=0 时 I 项本就不累积) */
		.yaw_out_max   = 172.8f,
		.pv_kp         = 2.0f,
		.wheel_kp      = 0.6f,     /* 蛇形修复: 0.3 内环响应过慢(全靠 I 慢爬);
					            * 旧1.0 近 bang-bang 起步浪涌, 0.6 折中, 别再升过 0.7 */
		.wheel_ki      = 7.0f,     /* 旧环 I 等效(见上方折算), 恢复静摩擦穿透能力 */
		.wheel_i_max   = 0.14f,    /* 7×0.14≈0.98 duty: 巡航需≈0.67, 原钳位0.095(0.665)恰卡死
					            * 工作点 → 差动时单侧积分顶满响应不对称(蛇形成因之一) */
	};
	mf_init(&g_mf, &cfg);
}
