#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "PID.h"
#include "math.h"
#include "usart.h"
#include "stdio.h"
#include "Control_Car.h"
#include "stdint.h"

//编码器ARR最大值
#define Encode_ARR 		65535

// PWM定时器ARR值宏定义(与 CubeMX ARR 一致: tim.c Period=1680-1, 否则满占空比封顶89.3%)
#define PWM_ARR 1679  // 84MHz/2 = 42MHz / 1680 = 25kHz PWM


//定时器号
#define ENCODER_TIM_1 htim1
#define ENCODER_TIM_2 htim2
#define PWM_TIM       htim3
#define GAP_TIM       htim4

#define SPEED_RECORD_NUM 100 //求平均值数组

// 电机数量
enum Motor_name{

	Motor_0,
	Motor_1,
	Motor_Count,
};

//电机工作时的各项数据
typedef struct{

	float encoder;
	float Speed;
	float Pos;
	float Tar_Speed;
	float Acc;
	float dt;
	float Filtered_speed;
	
}Parameter_TypeDef;   

// 电机控制结构体
typedef struct {
		TIM_TypeDef *tim;
		volatile uint32_t *ccr;
		GPIO_TypeDef* Motor_Dir_GPIO_Port;
		uint16_t Motor_Dir_Pin;
		uint8_t ID;
		int cnt;
		float Reduction_ratio;
		float Encoder_line_count;
		float Encoder_multiple;
		Parameter_TypeDef Parameter;
    PID_TypeDef speed_pid;     // 速度PID
} Motor_TypeDef;

void Motor_Init(void);         //电机初始化
int abs_int(int num);          //求其绝对值
float Read_Encoder_Value(uint8_t motor_id);//读取编码器脉冲 
void Gat_Motor_data(uint8_t motor_id);
float speed_control(float accel, float current_vel, float target_vel, float DT);
float Speed_Low_Filter(float new_Spe,float *speed_Record);



extern Motor_TypeDef motors[Motor_Count];



#endif


