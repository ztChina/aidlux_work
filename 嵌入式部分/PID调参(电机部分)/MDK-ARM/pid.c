#include "pid.h"

void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = 0;
    pid->current = 0;
    pid->error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->output = 0;
}

float PID_Calc(PID_HandleTypeDef *pid, float current_value)
{
    pid->current = current_value;
    pid->error = pid->target - pid->current;

    // 积分限幅，防止积分风暴
    pid->integral += pid->error;
    if (pid->integral > 1000.0f) pid->integral = 1000.0f;
    else if (pid->integral < -1000.0f) pid->integral = -1000.0f;

    float derivative = pid->error - pid->prev_error;

    pid->output = pid->kp * pid->error +
                  pid->ki * pid->integral +
                  pid->kd * derivative;

    pid->prev_error = pid->error;
    return pid->output;
}
