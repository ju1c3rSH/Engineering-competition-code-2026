#include "Car_Sense.h"
#include "mf_config.h"
#include "Encode.h"
#include "Control_Car.h"

static CarSense_Snap_t snap;

/* 编码器符号校准 (test/motion-tuning): 台架"前进而 POSE.x 为负、段永不完成、占空比
 * 钉满 100%(正反馈狂转)"实锤 旧注释"前进编码器读数为负, 取负纠正"已过时——
 * 现在 DIR=1 前进时 Pos 为正。+1 = 直接采用; 若换线束/换电机再翻 -1 */
#define CAR_ENCODER_FWD_MULT (+1.0f)

void CarSense_Sample(void)
{
	Gat_Motor_data(Motor_0);
	Gat_Motor_data(Motor_1);

	/* 180° 旋转：框架左 = 物理右 = Motor_0；框架右 = 物理左 = Motor_1
	 * 框架正向 = 物理后退，所以编码器取反 */
	snap.wheel_dist_m[0] = -motors[Motor_0].Parameter.Pos * (M_PI_F * WHEEL_DIA_M);
	snap.wheel_dist_m[1] = -motors[Motor_1].Parameter.Pos * (M_PI_F * WHEEL_DIA_M);
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