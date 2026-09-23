#include "Encode.h"

int Encode_count = 0;
float speed_Record[SPEED_RECORD_NUM] = {0};

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
		.Parameter.Tar_Speed = 0,					  // 电机目标速度
		.Parameter.Acc = 45.0f,						  // 加速度大小
		.Parameter.dt = 0.01f,						  // 匀加速时的间隔

		.Reduction_ratio = 30.0f,	  // 电机减速比
		.Encoder_line_count = 500.0f, // 电机编码器线数
		.Encoder_multiple = 4.0f,
		/*
		编码器会输出两路方波信号，如果只在通道A的上升沿计数，那就是1倍频；
		通道A的上升、下降沿计数，那就是2倍频；
		如果在通道A、B的上升、下降沿计数，那就是4倍频。
		使用倍频可以最大程度地利用两路信号，提高测速的灵敏度。
		*/
		.speed_pid = {.Kp = 4000.0f, .Ki = 50, .Kd = 0, .I_Max = 1000, .Out_Max = PWM_ARR}, // 电机PID
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
		.Parameter.Tar_Speed = 0,					  // 电机目标速度
		.Parameter.Acc = 45.0f,						  // 加速度大小
		.Parameter.dt = 0.01f,						  // 匀加速时的间隔

		.Reduction_ratio = 30.0f,	  // 电机减速比
		.Encoder_line_count = 500.0f, // 电机编码器线数
		.Encoder_multiple = 4.0f,
		/*
		编码器会输出两路方波信号，如果只在通道A的上升沿计数，那就是1倍频；
		通道A的上升、下降沿计数，那就是2倍频；
		如果在通道A、B的上升、下降沿计数，那就是4倍频。
		使用倍频可以最大程度地利用两路信号，提高测速的灵敏度。
		*/
		.speed_pid = {.Kp = 4000.0f, .Ki = 50, .Kd = 0, .I_Max = 1000, .Out_Max = PWM_ARR}, // 电机PID
	}};

void Motor_Init(void) // 初始化电机
{
	HAL_TIM_Encoder_Start(&ENCODER_TIM_1, TIM_CHANNEL_ALL); // 启用编码器
	HAL_TIM_Encoder_Start(&ENCODER_TIM_2, TIM_CHANNEL_ALL); // 启用编码器
	HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_1);				// 启用PWM
	HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_2);				// 启用PWM

	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;

	motors[Motor_0].tim->CNT = 20000;
	motors[Motor_1].tim->CNT = 20000;
}

// 读取编码器值函数
float Read_Encoder_Value(uint8_t motor_id)
{
	if (motors[motor_id].tim->CNT == 20000)
	{
		return 0;
	}
	if (motors[motor_id].tim->CNT > 20000)
	{
		motors[motor_id].cnt = 20000 - motors[motor_id].tim->CNT;
	}
	else
	{
		motors[motor_id].cnt = 20000 - motors[motor_id].tim->CNT;
	}
	motors[motor_id].tim->CNT = 20000;

	return motors[motor_id].cnt;
}
void Gat_Motor_data(uint8_t motor_id)
{
	motors[motor_id].Parameter.encoder = Read_Encoder_Value(motor_id); // 获取编码器值
	motors[motor_id].Parameter.Speed = motors[motor_id].Parameter.encoder * 100.0f / (motors[motor_id].Encoder_line_count * motors[motor_id].Reduction_ratio * motors[motor_id].Encoder_multiple);
	motors[motor_id].Parameter.Pos += motors[motor_id].Parameter.encoder / (motors[motor_id].Encoder_line_count * motors[motor_id].Reduction_ratio * motors[motor_id].Encoder_multiple);
	motors[motor_id].Parameter.Filtered_speed = Speed_Low_Filter(motors[motor_id].Parameter.Speed, speed_Record);
}

/**
 * @brief 匀加速/减速控制函数
 * @param accel 加速度大小（单位需与速度单位匹配，如m/s2）
 * @param current_vel 当前速度（带符号）
 * @param target_vel 目标速度（带符号）
 * @return 计算后的新速度（带符号）
 */
float speed_control(float accel, float current_vel, float target_vel, float DT)
{
	const float dt = DT; // 时间间隔例如10ms（0.01秒）
	float diff = target_vel - current_vel;

	// 如果当前速度已等于目标速度，直接返回目标速度
	if (diff == 0.0f)
	{
		return target_vel;
	}

	// 计算每个周期最大速度变化量
	float delta_v = accel * dt;

	// 如果速度差小于或等于最大变化量，直接返回目标速度
	if (fabsf(diff) <= delta_v)
	{
		return target_vel;
	}
	// 否则，根据方向加速或减速
	else
	{
		if (diff > 0.0f)
		{
			return current_vel + delta_v;
		}
		else
		{
			return current_vel - delta_v;
		}
	}
}
int abs_int(int num)
{
	return (num < 0) ? -num : num;
}

/*
 * 进行速度的平均滤波
 * 输入新采样到的速度，存放速度的数组，
 * 返回滤波后的速度
 */
float Speed_Low_Filter(float new_Spe, float *speed_Record)
{
	float sum = 0.0f;
	for (uint16_t i = SPEED_RECORD_NUM - 1; i > 0; i--) // 将现有数据后移一位
	{
		speed_Record[i] = speed_Record[i - 1];
		sum += speed_Record[i - 1];
	}
	speed_Record[0] = new_Spe; // 第一位是新的数据
	sum += new_Spe;
	return sum / SPEED_RECORD_NUM; // 返回均值
}
