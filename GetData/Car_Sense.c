#include "Car_Sense.h"
#include "mf_config.h"
#include "mf_math.h"
#include "Encode.h"
#include "Control_Car.h"

static CarSense_Snap_t snap;

/* 里程符号校准: +1 = 取反后即成"前进为正"(见 CarSense_Sample)。反了轮速环变正反馈:
 * 满占空比狂转、段永不完成、POSE 反向滚(实测踩过, docs/坑-编码器与里程.md §1)。
 * 换电机/换线束先翻它, 不要改别处 */
#define CAR_ENCODER_FWD_MULT (+1.0f)

/* 车体角速度(deg/s) 的差分步长(拍)。2 = 两拍滑窗。⚠ **不许改成 1**。
 *
 * 本宏必须与下面 CarSense_YawRate 的**固定两样本**实现同进退: y1/y2 的移位发生在差分
 * **之后**(d = yaw[k]-y2 时 y2 恒为 yaw[k-2]) ⇒ 分子永远跨 2 拍, 只有分母跟着本宏走。
 * 改 1 ⇒ 分母减半而分子不变 ⇒ **反馈恒为真值的 2 倍**: 速率内环把弧半径推到~1.9m
 * (标称 0.94m), 且反号自检/400dps 量程闸/帧计数有效性判定**三者全都不触发** —— 错得极安静。
 * 2026-10 曾据"1:1 下无交替"误改为 1, 由代码审查推翻并回退(§8.13)。本值的原理是跨满
 * "帧率÷拍率非整数"时的交替周期; 现配置 200÷200=1:1 跨 2 拍在数学上仍正确, 代价仅 2·dt
 * 相位(内环带宽被执行器滞后卡在 ~1.5Hz, 无影响)。完整推导见 §8.6/§7.6/§8.11/§8.13。
 * valid 必须由**帧计数**判, 不能拿"航向是否变化"当判据: 车静止时航向本就不变, 会把静止
 * 误判成传感器掉线。差分若按拍数以外的口径归一(或忘掉归一), 100ms 的转角会算成 10 倍
 * 尖峰、再被量程上限剔掉 ⇒ 反馈恒 0 */
#define CARSENSE_WZ_DIFF_TICKS  2

static float CarSense_YawRate(float yaw, uint8_t *valid)
{
	static float    y1, y2;                      /* 上两拍航向 */
	static uint32_t last_frame;                  /* 上次收到的 0x53 帧序号 */
	static uint16_t stale = MF_WZ_RATE_FB_TICKS; /* 距上次收到帧的拍数; 初值=尚无反馈 */
	static uint8_t  filled;                      /* 滑窗已积累样本数 0..CARSENSE_WZ_DIFF_TICKS */
	uint32_t frame = YawSeq;                     /* 单次读: 读-比-写中途被 USART3 插入会吞帧 */
	float d;
	float rate;

	*valid = 0u;

	if (frame != last_frame)
	{
		last_frame = frame;
		stale = 0u;
	}
	else if (stale < MF_WZ_RATE_FB_TICKS)
	{
		stale++;
	}

	if (filled < CARSENSE_WZ_DIFF_TICKS)
	{
		y2 = y1;
		y1 = yaw;
		filled++;
		*valid = (uint8_t)((stale < MF_WZ_RATE_FB_TICKS) ? 1u : 0u);
		return 0.0f;   /* 窗口未填满: 没有差分基线, 也不给反馈 */
	}

	d = mf_wrap_deg(yaw - y2);
	y2 = y1;
	y1 = yaw;

	rate = d / ((float)CARSENSE_WZ_DIFF_TICKS * MF_DT_S);

	if ((rate > MF_WZ_RATE_MEAS_MAX_DPS) || (rate < -MF_WZ_RATE_MEAS_MAX_DPS))
	{
		/* 航向阶跃(置零/协议假同步)不是真实转速: 丢窗重来, 免得阶跃前后两拍相除 */
		rate = 0.0f;
		filled = 0u;
	}

#if MF_WZ_RATE_GYRO_FRAME_EN
	{
		static uint32_t last_seq;
		float w = Yaw_struct.Wz_dps;

		/* 0x52 帧: sum8 只有 1/256 效力, 假同步会注入任意值 ⇒ 用差分的量程上限拦一道 */
		if ((GyroWz_Seq != last_seq) &&
		    (w <= MF_WZ_RATE_MEAS_MAX_DPS) && (w >= -MF_WZ_RATE_MEAS_MAX_DPS))
		{
			last_seq = GyroWz_Seq;
			rate = w;
			stale = 0u;
		}
	}
#endif

	*valid = (uint8_t)((stale < MF_WZ_RATE_FB_TICKS) ? 1u : 0u);
	return rate;
}

void CarSense_Sample(void)
{
	Gat_Motor_data(Motor_0);
	Gat_Motor_data(Motor_1);

	/* 航向帧计数先读: yaw_deg 是零阶保持值, 库内凡做"航向增量 ÷ 位移"(弧实测半径/形状)
	 * 都要靠它判"本拍是不是新样", 才能让分子分母覆盖同一段时间。
	 * 顺序不能反: 中断在两次读之间插进来会得到"旧航向 + 新序号" ⇒ 门控把那帧增量永久吞掉;
	 * 先读序号则最坏只是把这帧的增量推到下一拍(那时序号最新, 差额照样算进去) */
	snap.yaw_seq = YawSeq;
	snap.yaw_deg = Yaw_struct.Yaw;
	snap.wz_dps = CarSense_YawRate(snap.yaw_deg, &snap.wz_valid);

	/* 180° 旋转：框架左 = 物理右 = Motor_0；框架右 = 物理左 = Motor_1
	 * 框架正向 = 物理后退，所以里程取反（符号约定只在这里 + CarSense_Output 的 DIR 取反，
	 * 库层没有安装方向宏）。CAR_ENCODER_FWD_MULT 是这条链的标定旋钮：换电机/换线束就翻它 */
	snap.wheel_dist_m[0] = CAR_ENCODER_FWD_MULT * (-motors[Motor_0].Parameter.Pos)
	                     * (M_PI_F * WHEEL_DIA_M);
	snap.wheel_dist_m[1] = CAR_ENCODER_FWD_MULT * (-motors[Motor_1].Parameter.Pos)
	                     * (M_PI_F * WHEEL_DIA_M);
	snap.seq++;
}
const CarSense_Snap_t *CarSense_Get(void)
{
	return &snap;
}

/* 框架轮序 0=左 1=右 -> Motor_1/Motor_0。
 * 极性: 本函数在**引脚上把逻辑方向取反**(v>=0 写 DIR=0, 见下方 `DIR 翻转`)。它与编码器符号、
 * HWT101 航向符号一起构成整车的方向约定, 是台架按四方向(前进/后退/左右转)实测定下来的 ——
 * 单改这里任何一处等于把车倒过来, 改前必须重做四方向核对, 别照旧注释(曾写"pwm>=0 映射 DIR=1") */
void CarSense_Output(const mf_ctx_t *m)
{
	uint8_t idx;

	for (idx = 0u; idx < 2u; idx++)
	{
		uint8_t mot = (idx == 0u) ? Motor_0 : Motor_1;
		float v = m->wheel[idx].out;
		float mag = (v < 0.0f) ? -v : v;
		uint32_t ccr;

		if (mag > 1.0f)
			mag = 1.0f;
		ccr = (uint32_t)(mag * PWM_ARR);
		if (ccr > PWM_ARR)
			ccr = PWM_ARR;

		HAL_GPIO_WritePin(motors[mot].Motor_Dir_GPIO_Port, motors[mot].Motor_Dir_Pin,
						  (v >= 0.0f) ? 0u : 1u); /* DIR 翻转 */
		*motors[mot].ccr = ccr;
	}
}