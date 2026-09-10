#include "PID.h"

float Position_PID(PID_TypeDef *PID,float Target)
{
	float k=0.9;
	PID->Target = Target;
	/*计算偏差*/
	PID->Error = PID->Current - PID->Target;
	
	while(PID->Error<-180)
	{
		PID->Error = (360+PID->Error);
	}
	while(PID->Error>180)
	{
		PID->Error = (PID->Error-360);
	}
	
	
	/*低通滤波*/
	PID->Error = PID->Error*(1-k)+k * PID->Last_Error;
	PID->I_Out+=PID->Error*PID->Ki;
//	/*积分限幅*/
	if(PID->I_Out > PID->I_Max)
	{
		PID->I_Out = PID->I_Max;
	}
	else if(PID->I_Out < (0 - PID->I_Max))
	{
		PID->I_Out = 0 - PID->I_Max;
	}
	/*输出计算*/
	PID->Out = PID->Kp*PID->Error+PID->I_Out+PID->Kd*(PID->Error-PID->Last_Error);
	/*记录数据*/
	PID->Last_Error = PID->Error;
	/*输出限幅*/
	if(PID->Out > PID->Out_Max)
	{
		PID->Out = PID->Out_Max;
	}
	else if(PID->Out < (0 - PID->Out_Max))
	{
		PID->Out = 0 - PID->Out_Max;
	}
	
	return PID->Out;
}

float Angle_PID(PID_TypeDef *PID,float Target,float groy)
{
	PID->Target = Target;
	/*计算偏差*/
	PID->Error = PID->Current - PID->Target;
	
	while(PID->Error<-180)
	{
		PID->Error = (360+PID->Error);
	}
	while(PID->Error>180)
	{
		PID->Error = (PID->Error-360);
	}
	
	/*输出计算*/
	PID->Out = PID->Kp*PID->Error+PID->Kd*groy;
	/*记录数据*/
	PID->Last_Error = PID->Error;
	/*输出限幅*/
	if(PID->Out > PID->Out_Max)
	{
		PID->Out = PID->Out_Max;
	}
	else if(PID->Out < (0 - PID->Out_Max))
	{
		PID->Out = 0 - PID->Out_Max;
	}
	
	return PID->Out;
}


