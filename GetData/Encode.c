#include "Encode.h"

/****************全局电机配置数组****************/
Motor_TypeDef motors[Motor_Count] = {
	{
		.ID = Motor_0,								  // 电机编号
		.tim = TIM1,								  // 编码器的定时器用于寄存器配置，不用在初始化上；
		.ccr = &TIM3->CCR1,							  // PWM通道
		.Motor_Dir_GPIO_Port = Motor_Dir_1_GPIO_Port, // 控制方向端口
		.Motor_Dir_Pin = Motor_Dir_1_Pin,			  // 控制方向引脚号
		.cnt = 0,									  // 初始化0号电机的脉冲数
		.Parameter.Speed = 0,						  // 电机速度(转每秒)
		.Parameter.Pos = 0,							  // 电机位置

		.Reduction_ratio = 30.0f,	  // 电机减速比
		.Encoder_line_count = 500.0f, // 电机编码器线数
		.Encoder_multiple = 4.0f,
		/*
		编码器会输出两路方波信号，如果只在通道A的上升沿计数，那就是1倍频；
		通道A的上升、下降沿计数，那就是2倍频；
		如果在通道A、B的上升、下降沿计数，那就是4倍频。
		使用倍频可以最大程度地利用两路信号，提高测速的灵敏度。
		*/
	},
	{
		.ID = Motor_1,								  // 电机编号
		.tim = TIM2,								  // 编码器的定时器用于寄存器配置，不用在初始化上
		.ccr = &TIM3->CCR2,							  // PWM通道
		.Motor_Dir_GPIO_Port = Motor_Dir_2_GPIO_Port, // 控制方向端口
		.Motor_Dir_Pin = Motor_Dir_2_Pin,			  // 控制方向引脚号
		.cnt = 0,									  // 初始化1号电机的脉冲数
		.Parameter.Speed = 0,						  // 电机速度(转每秒)
		.Parameter.Pos = 0,							  // 电机位置

		.Reduction_ratio = 30.0f,	  // 电机减速比
		.Encoder_line_count = 500.0f, // 电机编码器线数
		.Encoder_multiple = 4.0f,
		/*
		编码器会输出两路方波信号，如果只在通道A的上升沿计数，那就是1倍频；
		通道A的上升、下降沿计数，那就是2倍频；
		如果在通道A、B的上升、下降沿计数，那就是4倍频。
		使用倍频可以最大程度地利用两路信号，提高测速的灵敏度。
		*/
	}};

void Motor_Init(void) // 初始化电机
{
	HAL_TIM_Encoder_Start(&ENCODER_TIM_1, TIM_CHANNEL_ALL); // 启用编码器
	HAL_TIM_Encoder_Start(&ENCODER_TIM_2, TIM_CHANNEL_ALL); // 启用编码器
	HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_1);				// 启用PWM
	HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_2);				// 启用PWM

	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;

	motors[Motor_0].tim->CNT = ENCODER_CNT_MID;
	motors[Motor_1].tim->CNT = ENCODER_CNT_MID;
}

/* 读取编码器增量：返回 (中点 − 当前 CNT)，随后把 CNT 回写中点。
 * 符号是"通道计数取反"的约定（前进时返回负 ⇒ 里程侧再取反一次，见 Car_Sense.c 的符号说明） */
float Read_Encoder_Value(uint8_t motor_id)
{
	motors[motor_id].cnt = ENCODER_CNT_MID - motors[motor_id].tim->CNT;
	motors[motor_id].tim->CNT = ENCODER_CNT_MID;

	return motors[motor_id].cnt;
}
void Gat_Motor_data(uint8_t motor_id)
{
	motors[motor_id].Parameter.encoder = Read_Encoder_Value(motor_id); // 获取编码器值
	motors[motor_id].Parameter.Speed = motors[motor_id].Parameter.encoder * 100.0f / (motors[motor_id].Encoder_line_count * motors[motor_id].Reduction_ratio * motors[motor_id].Encoder_multiple);
	motors[motor_id].Parameter.Pos += motors[motor_id].Parameter.encoder / (motors[motor_id].Encoder_line_count * motors[motor_id].Reduction_ratio * motors[motor_id].Encoder_multiple);
}
