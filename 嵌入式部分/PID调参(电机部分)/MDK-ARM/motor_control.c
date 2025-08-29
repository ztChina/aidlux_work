#include "motor_control.h"
#include "tim.h"
//#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <math.h>
#include "pid.h"

PID_HandleTypeDef pid_a;
PID_HandleTypeDef pid_b;

// 上一次编码器计数值
static int16_t last_count_a = 0;
static int16_t last_count_b = 0;

// 当前速度(脉冲数/100ms)
int16_t speed_a = 0;
int16_t speed_b = 0;

// 滤波后的速度
float filtered_speed_a = 0;
float filtered_speed_b = 0;

#define WHEEL_DIAMETER       0.065f
#define WHEEL_CIRCUMFERENCE  (3.1416f * WHEEL_DIAMETER) // ≈ 0.2042
#define GEAR_RATIO           30.0f
#define ENCODER_RESOLUTION   13.0f
#define QUADRATURE_MULTIPLIER 4.0f
#define INTERVAL_SEC         0.1f


// 滤波系数
static float alpha = 0.5f;

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
	
	PID_Init(&pid_a, 800.0f, 120.0f, 100.0f);
    PID_Init(&pid_b, 800.0f, 120.0f, 100.0f);
}

void Motor_UpdateSpeed(void)
{
    int16_t count_a = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t count_b = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    speed_a = GetDeltaCount(count_a, last_count_a);
    speed_b = GetDeltaCount(count_b, last_count_b);

    last_count_a = count_a;
    last_count_b = count_b;

	speed_a = -speed_a;
	
//    // 限幅防止跳变过大
//    if(speed_a > 300) speed_a = 300;
//    else if(speed_a < -300) speed_a = -300;

//    if(speed_b > 300) speed_b = 300;
//    else if(speed_b < -300) speed_b = -300;

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

float Motor_ConvertToSpeed(float pulses)
{
    float pulses_per_rev = ENCODER_RESOLUTION * GEAR_RATIO * QUADRATURE_MULTIPLIER;
    float revolutions = pulses / pulses_per_rev;
    float distance = revolutions * WHEEL_CIRCUMFERENCE;
    float velocity = distance / INTERVAL_SEC;

    // 打印中间变量，调试用
//    printf("[DEBUG] pulses=%.2f, rev=%.5f, dist=%.5f, v=%.5f m/s\r\n",
//           pulses, revolutions, distance, velocity);

    return velocity;
}

float Motor_GetFilteredSpeedA(void)
{
    return Motor_ConvertToSpeed(filtered_speed_a);
}

float Motor_GetFilteredSpeedB(void)
{
    return Motor_ConvertToSpeed(filtered_speed_b);
}

// 速度的开环控制
int SpeedToPWM(float speed_mps)
{
    const float K = 750.0f;  // 估计值，调整试试
    float abs_speed = fabs(speed_mps);
    int pwm = (int)(abs_speed * K);

    if (pwm > 1000) pwm = 1000;
    if (pwm < 0) pwm = 0;

//    printf("[DEBUG] speed=%.5f, abs_speed=%.5f, pwm=%d\n", speed_mps, abs_speed, pwm);

    return pwm;
}

void Motor_SetTargetSpeed(float left_mps, float right_mps)
{
    pid_a.target = left_mps;
    pid_b.target = right_mps;

    // 当目标速度接近0时，强制清零积分，防止积分风暴导致停不下来
    if (fabs(left_mps) < 0.01f) {
        pid_a.integral = 0.0f;
        pid_a.prev_error = 0.0f;  // 也建议清理前次误差，避免积分抖动
    }
    if (fabs(right_mps) < 0.01f) {
        pid_b.integral = 0.0f;
        pid_b.prev_error = 0.0f;
    }
}

void Motor_PIDUpdate(void)
{
    float currentL = Motor_GetFilteredSpeedA();
    float currentR = Motor_GetFilteredSpeedB();

    float outputL_speed = PID_Calc(&pid_a, currentL);  // 目标速度 (±)
    float outputR_speed = PID_Calc(&pid_b, currentR);  // 目标速度 (±)

    int dirL = outputL_speed >= 0 ? 1 : 0;
    int dirR = outputR_speed >= 0 ? 1 : 0;

    // ⛳️ 速度转 PWM，使用绝对值
    int pwmL = SpeedToPWM(fabs(outputL_speed));
    int pwmR = SpeedToPWM(fabs(outputR_speed));

    // 限幅，防炸机
    if (pwmL > 1000) pwmL = 1000;
    if (pwmR > 1000) pwmR = 1000;

    Motor_Set(pwmL, pwmR, dirL, dirR);

    // 打印调试信息
    printf("Target L=%.2f R=%.2f | Real L=%.2f R=%.2f | PWM=%d %d DIR=%d %d\r\n",
           pid_a.target, pid_b.target, currentL, currentR, pwmL, pwmR, dirL, dirR);
}



void Motor_SetPWM(float pwm_left, float pwm_right)
{
    // 目标速度为0时，强制输出0
    if (fabsf(pwm_left) < 0.01f) pwm_left = 0.0f;
    if (fabsf(pwm_right) < 0.01f) pwm_right = 0.0f;

    int dirL = pwm_left >= 0 ? 1 : 0;
    int dirR = pwm_right >= 0 ? 1 : 0;

    int pwmL = (int)fabsf(pwm_left);
    int pwmR = (int)fabsf(pwm_right);

    if (pwmL > 1000) pwmL = 1000;
    if (pwmR > 1000) pwmR = 1000;

    // 如果都为0，考虑断电或STBY低电平
    if (pwmL == 0 && pwmR == 0) {
        // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // 休眠电机，视具体硬件
    } else {
        // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    }

    Motor_Set(pwmL, pwmR, dirL, dirR);
}

