#ifndef __PID_H__
#define __PID_H__

typedef struct {
    float kp;
    float ki;
    float kd;

    float target;
    float current;
    float error;
    float prev_error;
    float integral;
    float output;
} PID_HandleTypeDef;

void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd);
float PID_Calc(PID_HandleTypeDef *pid, float current_value);
float PID_Compute(PID_HandleTypeDef *pid);


#endif
