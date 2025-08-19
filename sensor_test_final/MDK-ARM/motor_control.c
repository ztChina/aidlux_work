#include "motor_control.h"
#include "tim.h"
//#include "usart.h"
#include "gpio.h"
#include <stdio.h>

// 上一次编码器计数值
static int16_t last_count_a = 0;
static int16_t last_count_b = 0;

// 当前速度(脉冲数/100ms)
int16_t speed_a = 0;
int16_t speed_b = 0;

// 滤波后的速度
float filtered_speed_a = 0;
float filtered_speed_b = 0;

// 滤波系数
static float alpha = 0.3f;

static int16_t GetDeltaCount(int16_t current, int16_t last)
{
    int16_t delta = current - last;
    if(delta > 32767) delta -= 65536;
    else if(delta < -32768) delta += 65536;
    return delta;
}

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  // Motor A
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // Motor B
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);    // 编码器A
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);    // 编码器B
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);  // STBY = 1

    last_count_a = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    last_count_b = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

void Motor_UpdateSpeed(void)
{
    int16_t count_a = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t count_b = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    speed_a = GetDeltaCount(count_a, last_count_a);
    speed_b = GetDeltaCount(count_b, last_count_b);

    last_count_a = count_a;
    last_count_b = count_b;

    // 限幅防止跳变过大
    if(speed_a > 300) speed_a = 300;
    else if(speed_a < -300) speed_a = -300;

    if(speed_b > 300) speed_b = 300;
    else if(speed_b < -300) speed_b = -300;

    // 低通滤波
    filtered_speed_a = alpha * speed_a + (1 - alpha) * filtered_speed_a;
    filtered_speed_b = alpha * speed_b + (1 - alpha) * filtered_speed_b;

    static int print_cnt = 0;
    if(++print_cnt >= 10)
    {
        // printf("Raw Speed A=%d, B=%d | Filtered Speed A=%.2f, B=%.2f\r\n", speed_a, speed_b, filtered_speed_a, filtered_speed_b);
        print_cnt = 0;
    }
}


void Motor_Set(int16_t pwm_a, int16_t pwm_b, uint8_t dir_a, uint8_t dir_b)
{
	// 设置方向
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, dir_a ? GPIO_PIN_SET : GPIO_PIN_RESET); // AIN1
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, dir_a ? GPIO_PIN_RESET : GPIO_PIN_SET); // AIN2
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, dir_b ? GPIO_PIN_SET : GPIO_PIN_RESET); // BIN1
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, dir_b ? GPIO_PIN_RESET : GPIO_PIN_SET); // BIN2
	
	// 设置PWM占空比
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_a); // 电机A
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_b); // 电机B
}

//  

float Motor_GetFilteredSpeedA(void)
{
	return filtered_speed_a;
}

float Motor_GetFilteredSpeedB(void)
{
	return filtered_speed_b;
}
