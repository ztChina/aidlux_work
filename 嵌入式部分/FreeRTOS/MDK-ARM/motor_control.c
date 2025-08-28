#include "motor_control.h"
#include "tim.h"
#include "gpio.h"
#include <stdio.h>
#include <math.h>  // 可选：用于更精确的 PI 值和 pow/sqrt 等函数
#include <pid.h>

//----------------------------------//
//       编码器与电机参数定义       //
//----------------------------------//
#define ENCODER_PPR     1560.0f     // 编码器每转脉冲数（四倍频后）
//#define ENCODER_PPR     390.0f     // 编码器每转脉冲数
#define GEAR_RATIO      30.0f       // 电机减速比
#define WHEEL_RADIUS    0.0325f     // 轮子半径（单位：米）
#define SAMPLE_TIME     0.05f        // 采样周期（单位：秒）
#define PI              3.1416f     // 圆周率
#define SAFE_SPEED  0.4f     // 最大线速度 (单位：m/s)
#define MAX_SPEED   1.25f			// 最大线速度 (单位：m/s)
#define PWM_MAX    1000      // PWM最大值（根据定时器设置）
//----------------------------------//
//           全局变量定义           //

int zero_count_a = 0;
int zero_count_b = 0;
volatile float pwm_left = 0;
volatile float pwm_right = 0;
float last_valid_speed_a = 0;
float last_valid_speed_b = 0;
MotorSpeed_t current_motor_speed = {0}; //用于反馈
MotorSpeed_t target_motor_speed = {0};  //用于控制
extern PID_HandleTypeDef pid_a;
extern PID_HandleTypeDef pid_b;
#define ZERO_DROP_THRESHOLD 3  // 连续为0超过N帧才认定为真零

//----------------------------------//

// 上一次编码器计数值
static int16_t last_count_a = 0;
static int16_t last_count_b = 0;

// 当前速度(单位：脉冲/采样周期)
int16_t speed_a = 0;
int16_t speed_b = 0;

// 滤波后的速度
float filtered_speed_a = 0;
float filtered_speed_b = 0;

// 滤波系数（低通滤波）
static float alpha = 0.3f;

// 线速度（单位：米/秒）
float linear_speed_left = 0.0f;
float linear_speed_right = 0.0f;



//----------------------------------//
//        内部函数声明/实现         //
//----------------------------------//

static int16_t GetDeltaCount(int16_t current, int16_t last)
{
    int16_t delta = current - last;
    if (delta > 32767) delta -= 65536;
    else if (delta < -32768) delta += 65536;
    return delta;
}

//----------------------------------//
//           电机初始化             //
//----------------------------------//

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);     // 电机A
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);     // 电机B
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 编码器A
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 编码器B
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);  // STBY = 1（使能驱动）

    last_count_a = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    last_count_b = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
		PID_Init(&pid_a, 0.7, 0, 1,1000);
		PID_Init(&pid_b, 0.7, 0, 1,1000);
		//测试
//		target_motor_speed.v_left = 0.45;
//		target_motor_speed.v_right = 0.3;
//		Motor_Set(200, 200, 1, 1);
}


void Motor_UpdateSpeed(void)
{
    int16_t count_a = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t count_b = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    speed_a = -GetDeltaCount(count_a, last_count_a);
    speed_b = GetDeltaCount(count_b, last_count_b);

    last_count_a = count_a;
    last_count_b = count_b;

    // 限幅
    if (speed_a > 300) speed_a = 300;
    else if (speed_a < -300) speed_a = -300;

    if (speed_b > 300) speed_b = 300;
    else if (speed_b < -300) speed_b = -300;

    // 一阶低通滤波
    filtered_speed_a = alpha * speed_a + (1 - alpha) * filtered_speed_a;
    filtered_speed_b = alpha * speed_b + (1 - alpha) * filtered_speed_b;

    // 编码器脉冲 → 线速度（单位：米/秒）
    linear_speed_left = (filtered_speed_a / ENCODER_PPR) * (2.0f * PI * WHEEL_RADIUS)  / SAMPLE_TIME ;
    linear_speed_right = (filtered_speed_b / ENCODER_PPR) * (2.0f * PI * WHEEL_RADIUS)  / SAMPLE_TIME ;

    // 容错处理：滤除短时间“跳 0”
    if (fabs(linear_speed_left) < 1e-3f) {
        zero_count_a++;
        if (zero_count_a < ZERO_DROP_THRESHOLD) {
            linear_speed_left = last_valid_speed_a;
        } else {
            last_valid_speed_a = 0;  // 真正为 0
        }
    } else {
        zero_count_a = 0;
        last_valid_speed_a = linear_speed_left;
    }

    if (fabs(linear_speed_right) < 1e-3f) {
        zero_count_b++;
        if (zero_count_b < ZERO_DROP_THRESHOLD) {
            linear_speed_right = last_valid_speed_b;
        } else {
            last_valid_speed_b = 0;
        }
    } else {
        zero_count_b = 0;
        last_valid_speed_b = linear_speed_right;
    }

    // 可选：调试打印
    /*
    static int print_cnt = 0;
    if (++print_cnt >= 10)
    {
        printf("Speed L: %.3f m/s, Speed R: %.3f m/s\r\n", linear_speed_left, linear_speed_right);
        print_cnt = 0;
    }
    */
}


//----------------------------------//
//            设置PWM与方向          //
//----------------------------------//

void Motor_Set(int16_t pwm_a, int16_t pwm_b, uint8_t dir_a, uint8_t dir_b)
{
    // 设置方向控制引脚
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, dir_a ? GPIO_PIN_SET : GPIO_PIN_RESET); // AIN1
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, dir_a ? GPIO_PIN_RESET : GPIO_PIN_SET); // AIN2
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, dir_b ? GPIO_PIN_SET : GPIO_PIN_RESET); // BIN1
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, dir_b ? GPIO_PIN_RESET : GPIO_PIN_SET); // BIN2

    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_a); // 电机A
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_b); // 电机B
}


void Motor_Set_PWM(int16_t pwm_a, int16_t pwm_b)
{
//    // 限制 PWM 范围，避免超出定时器配置
//    if (pwm_a > PWM_MAX) pwm_a = PWM_MAX;
//    if (pwm_a < -PWM_MAX) pwm_a = -PWM_MAX;
//    if (pwm_b > PWM_MAX) pwm_b = PWM_MAX;
//    if (pwm_b < -PWM_MAX) pwm_b = -PWM_MAX;
    // 设置方向 A
    if (pwm_a >= 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);   // AIN1
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // AIN2
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
        pwm_a = -pwm_a; // 取绝对值用于 PWM 设置
    }

    // 设置方向 B
    if (pwm_b >= 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);   // BIN1
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); // BIN2
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
        pwm_b = -pwm_b;
    }

    // 设置 PWM 占空比（确保为正）
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_a); // 电机A
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_b); // 电机B
}


//----------------------------------//
//         可选的对外访问接口        //
//----------------------------------//

float Motor_GetLinearSpeedLeft(void)
{
    return linear_speed_left;
}

float Motor_GetLinearSpeedRight(void)
{
    return linear_speed_right;
}

float Motor_GetFilteredSpeedA(void)
{
    return filtered_speed_a;
}

float Motor_GetFilteredSpeedB(void)
{
    return filtered_speed_b;
}

void Motor_Control_SetSpeed(float v_left, float v_right)
{
    // 限幅保护
//    if (v_left > SAFE_SPEED) v_left = SAFE_SPEED;
//    if (v_left < -SAFE_SPEED) v_left = -SAFE_SPEED;
//    if (v_right > SAFE_SPEED) v_right = SAFE_SPEED;
//    if (v_right < -SAFE_SPEED) v_right = -SAFE_SPEED;
//		target_motor_speed.v_left = v_left;
//		target_motor_speed.v_right = v_right;
//    // 计算PWM占空比
//    int pwm_l = (int)(fabs(v_left) / MAX_SPEED * PWM_MAX);
//    int pwm_r = (int)(fabs(v_right) / MAX_SPEED * PWM_MAX);
//    // 方向
//    uint8_t dir_l = (v_left >= 0) ? 1 : 0;
//    uint8_t dir_r = (v_right >= 0) ? 1 : 0;
//    // 执行底层电机控制
//    Motor_Set(pwm_l, pwm_r, dir_l, dir_r);
	
	
		if (fabs(v_left) < 0.01f) {
			PID_Reset(&pid_a);  // 速度趋近为零
		}
		if (fabs(v_right) < 0.01f) {
			PID_Reset(&pid_b);  // 速度趋近为零
		}
	//交给pid调速,只负责更新目标值
		target_motor_speed.v_left = v_left;
		target_motor_speed.v_right = v_right;
}


void Motor_pid_Set(void)
{
		float target_speed_l = target_motor_speed.v_left * ENCODER_PPR / (2 * PI * WHEEL_RADIUS)*SAMPLE_TIME;
		float target_speed_r = target_motor_speed.v_right * ENCODER_PPR / (2 * PI * WHEEL_RADIUS)*SAMPLE_TIME;
		//传入的单位是编码器脉冲，输出偏差值

//		printf("target_speed_r:%lf ,filtered_speed_b:%lf，linear_speed_right：%lf\n",0.33 * ENCODER_PPR / (2 * PI * WHEEL_RADIUS), filtered_speed_b,linear_speed_right);
		float pid_l = PID_Calc(&pid_a,target_speed_l, filtered_speed_a);
		float pid_r = PID_Calc(&pid_b,target_speed_r, filtered_speed_b);
//	printf("target_speed_l:%lf ,filtered_speed_a:%lf，linear_speed_left：%lf,pid_l:%lf\n",target_speed_l, filtered_speed_a,linear_speed_left,pid_l);
		pwm_left += pid_l;
		pwm_right += pid_r;
		if(fabs(pwm_left)<5) pwm_left = 0;
		if(fabs(pwm_right)<5) pwm_right = 0;
	    // 限制 PWM 范围，避免超出定时器配置
    if (pwm_left > PWM_MAX) pwm_left = PWM_MAX;
    if (pwm_left < -PWM_MAX) pwm_left = -PWM_MAX;
    if (pwm_right > PWM_MAX) pwm_right = PWM_MAX;
    if (pwm_right < -PWM_MAX) pwm_right = -PWM_MAX;
//	  printf("pwm_left:%f,pwm_right:%f\n",pwm_left,pwm_right);
		Motor_Set_PWM((int)pwm_left, (int)pwm_right);

}
