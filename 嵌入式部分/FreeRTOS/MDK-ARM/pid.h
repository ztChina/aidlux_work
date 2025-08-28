#ifndef __PID_H__
#define __PID_H__

typedef struct {
//    float kp;
//    float ki;
//    float kd;

//    float target;
//    float current;
//    float error;
//    float prev_error;
//    float integral;
//    float output;
		float  Kp;         //  Proportional Const  Pϵ��
		float  Ki;           //  Integral Const      Iϵ��
		float  Kd;         //  Derivative Const    Dϵ��
		
		float  PrevError ;          //  Error[-2]  
		float  LastError;          //  Error[-1]  
		float  Error;              //  Error[0 ]  
		float  DError;            //pid->Error - pid->LastError	
		float  SumError;           //  Sums of Errors  
		
		float  output;
		
		float  Integralmax;      //����������ֵ
		float  outputmax;        //���������ֵ
} PID_HandleTypeDef;

extern PID_HandleTypeDef pid_a;
extern PID_HandleTypeDef pid_b;

void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float Limit_value);
float PID_Calc(PID_HandleTypeDef *pid, float Target_val ,float Actual_val);
void PID_Reset(PID_HandleTypeDef *pid);


#endif
