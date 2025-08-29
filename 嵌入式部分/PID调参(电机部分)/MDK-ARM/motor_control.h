#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "pid.h"

// º¯ÊýÉùÃ÷
void Motor_Init(void);
void Motor_UpdateSpeed(void);
void Motor_Set(int16_t pwm_a, int16_t pwm_b, uint8_t dir_a, uint8_t dir_b);
float Motor_GetFilteredSpeedA(void);
float Motor_GetFilteredSpeedB(void);
int SpeedToPWM(float speed_mps);
void Motor_PIDUpdate(void);
void Motor_SetTargetSpeed(float left_mps, float right_mps);
void Motor_SetPWM(float pwm_left, float pwm_right);

extern int16_t speed_a;
extern int16_t speed_b;

extern float filtered_speed_a;
extern float filtered_speed_b;

extern PID_HandleTypeDef pid_a;
extern PID_HandleTypeDef pid_b;

#endif // MOTOR_CONTROL_H
