#include "Car_Sense.h"
#include "Encode.h"
#include "Control_Car.h"

static CarSense_Snap_t snap;

void CarSense_Sample(void)
{
	Gat_Motor_data(Motor_0);
	Gat_Motor_data(Motor_1);
	/* Encode.c 的 Pos 为驱动轮转数(已除减速比)，前进为负，取负纠正 */
	snap.wheel_dist_m[0] = -motors[Motor_1].Parameter.Pos * (M_PI_F * WHEEL_DIA_M);
	snap.wheel_dist_m[1] = -motors[Motor_0].Parameter.Pos * (M_PI_F * WHEEL_DIA_M);
	snap.yaw_deg = Yaw_struct.Yaw;
	snap.seq++;
}

const CarSense_Snap_t *CarSense_Get(void)
{
	return &snap;
}

/* 框架轮序 0=左 1=右 -> Motor_1/Motor_0。
 * 旧代码前进时编码器读数为负、DIR=1 且 PID 输出为负，
 * 框架内前进统一为正，故 pwm>=0 映射 DIR=1。
 * 记得核对极性 */
void CarSense_Output(const mf_ctx_t *m)
{
	uint8_t idx;

	for (idx = 0u; idx < 2u; idx++)
	{
		uint8_t mot = (idx == 0u) ? Motor_1 : Motor_0;
		float v = m->wheel[idx].out;
		float mag = (v < 0.0f) ? -v : v;
		uint32_t ccr;

		if (mag > 1.0f)
		{
			mag = 1.0f;
		}
		ccr = (uint32_t)(mag * PWM_ARR);
		if (ccr > PWM_ARR)
		{
			ccr = PWM_ARR;
		}

		HAL_GPIO_WritePin(motors[mot].Motor_Dir_GPIO_Port, motors[mot].Motor_Dir_Pin,
		                  (v >= 0.0f) ? 1u : 0u);
		*motors[mot].ccr = ccr;
	}
}
