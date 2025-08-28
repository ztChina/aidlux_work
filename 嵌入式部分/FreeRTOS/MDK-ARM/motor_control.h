#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"

// 函数声明
void Motor_Init(void);
void Motor_Set(int16_t pwm_a, int16_t pwm_b, uint8_t dir_a, uint8_t dir_b);
void Motor_UpdateSpeed(void);
void Motor_Control_SetSpeed(float v_left, float v_right);
void Motor_Set_PWM(int16_t pwm_a, int16_t pwm_b);
void Motor_pid_Set(void);

float Motor_GetLinearSpeedLeft(void);
float Motor_GetLinearSpeedRight(void);
float Motor_GetFilteredSpeedA(void);
float Motor_GetFilteredSpeedB(void);

typedef struct {
    float v_left;
    float v_right;
		
} MotorSpeed_t;

extern MotorSpeed_t current_motor_speed; //用于反馈
extern MotorSpeed_t target_motor_speed ;  //用于控制
extern int16_t speed_a;
extern int16_t speed_b;

extern float filtered_speed_a;
extern float filtered_speed_b;




#endif // MOTOR_CONTROL_H
