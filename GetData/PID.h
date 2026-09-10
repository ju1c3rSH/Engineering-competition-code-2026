#ifndef __PID_H
#define __PID_H

#include "main.h"
#include "tim.h"
#include "gpio.h"


typedef struct {
  float Kp;
  float Ki;
  float Kd;
	float Target;
	float Last_Error;				
	float Error;			    
	float I_Max;
	float I_Out;
	float Out;  
	float Out_Max;
	float Current;
	uint8_t Clear;
	
	/*用户自加*/
	
	
	
}PID_TypeDef;


float Position_PID(PID_TypeDef *PID,float Target);
float Angle_PID(PID_TypeDef *PID,float Target,float groy);
void Set_PID(PID_TypeDef *PID,float Kp,float Ki,float Kd);
//float Angle_error(float Target, float Now);
#endif
