#include "pid.h"

void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float Limit_value)
{

	pid->Kp= Kp;
	pid->Ki= Ki;
	pid->Kd= Kd;
	
	pid->PrevError =pid->LastError = pid->Error =pid->SumError= pid->output =  0; 
	pid->Integralmax = pid->outputmax  = Limit_value;
}

PID_HandleTypeDef pid_a;
PID_HandleTypeDef pid_b;

float PID_Calc(PID_HandleTypeDef *pid, float Target_val ,float Actual_val)
{
//	pid->Error = Target_val- Actual_val;                        

////	pid->output  +=  pid->Kp* ( pid->Error - pid->LastError )+   
////					 pid->Ki* pid->Error +   
////					 pid->Kd* ( pid->Error +  pid->PrevError - 2*pid->LastError);  
//	
//	pid->SumError += pid->Error;
//	pid->output += pid->Kp * (pid->Error - pid->LastError)
//             + pid->Ki * pid->SumError
//             + pid->Kd * (pid->Error + pid->PrevError - 2 * pid->LastError);
//								 
//	pid->PrevError = pid->LastError;  
//	pid->LastError = pid->Error;

//	if (pid->SumError > pid->Integralmax) pid->SumError = pid->Integralmax;
//	if (pid->SumError < -pid->Integralmax) pid->SumError = -pid->Integralmax;
//	
//	if(pid->output > pid->outputmax )    pid->output = pid->outputmax;
//	if(pid->output < - pid->outputmax )  pid->output = -pid->outputmax;
	pid->Error = Target_val- Actual_val;                        

	pid->output  +=  pid->Kp* ( pid->Error - pid->LastError )+   
					 pid->Ki* pid->Error +   
					 pid->Kd* ( pid->Error +  pid->PrevError - 2*pid->LastError);  
								 
	pid->PrevError = pid->LastError;  
	pid->LastError = pid->Error;

	if(pid->output > pid->outputmax )    pid->output = pid->outputmax;
	if(pid->output < - pid->outputmax )  pid->output = -pid->outputmax;
	
	return pid->output ;   //Êä³öÎªpwmÖµ

}


void PID_Reset(PID_HandleTypeDef *pid)
{
    pid->Error = 0;
    pid->LastError = 0;
    pid->PrevError = 0;
    pid->SumError = 0;
    pid->output = 0;
}
