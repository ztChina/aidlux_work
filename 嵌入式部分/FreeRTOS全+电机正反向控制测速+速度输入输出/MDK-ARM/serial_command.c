#include "serial_command.h"
#include "motor_control.h"
#include "usart.h"   // <- 为了用 huart1
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// 接收数据
uint8_t rx_data = 0;   // <- 让 main.c 可以 extern 用
char rx_buffer[64];
uint8_t rx_index = 0;

// 提前声明
static void ParseCommand(char *cmd);


void SerialCommand_Init(void)
{
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_buffer[rx_index], 1);
}

// 外部从回调中调用
void SerialCommand_ProcessChar(uint8_t ch)
{
    if (ch == '\r' || ch == '\n') return;

    if (rx_index == 0 && ch != 'V') return;

    if (rx_index < sizeof(rx_buffer) - 1) {
        rx_buffer[rx_index++] = ch;
    }

    if (ch == 'E') {
        rx_buffer[rx_index] = '\0';
        ParseCommand(rx_buffer);
        rx_index = 0;
    }
}

// 私有函数，仅在本文件使用
void ParseCommand(char *cmd)
{
    int len = strlen(cmd);
    if (cmd[0] != 'V' || cmd[len - 1] != 'E') return;
    cmd[len - 1] = '\0';  // 去掉尾部 E

    char *p = &cmd[1];

    char *split = NULL;
    for (int i = 1; i < strlen(p); ++i) {
        if (p[i] == '+' || p[i] == '-') {
            split = &p[i];
            break;
        }
    }
    if (!split) return;

    char left_str[16] = {0}, right_str[16] = {0};
    strncpy(left_str, p, split - p);
    left_str[split - p] = '\0';
    strcpy(right_str, split);

    // 统一解释为 m/s
    float speedL = atof(left_str);
    float speedR = atof(right_str);

    int dirL = (speedL >= 0) ? 1 : 0;
    int dirR = (speedR >= 0) ? 1 : 0;

    int pwmL = SpeedToPWM(speedL);
    int pwmR = SpeedToPWM(speedR);

    Motor_Set(pwmL, pwmR, dirL, dirR);
    Motor_UpdateSpeed();
	
//	float v_a = Motor_GetFilteredSpeedA();
//	float v_b = Motor_GetFilteredSpeedB();

//	int filtered_a = (int)filtered_speed_a;
//	int filtered_b = (int)filtered_speed_b;

	float v_a = Motor_GetFilteredSpeedA();
    float v_b = Motor_GetFilteredSpeedB();

//    printf("Target Speed: L=%.2f m/s, R=%.2f m/s -> PWM: L=%d, R=%d\r\n",
//           speedL, speedR, pwmL, pwmR);
    printf("Actual Speed: L=%.3f m/s, R=%.3f m/s\r\n", v_a, v_b);


//	printf("%s, %d, %s, %d\r\n", dirL ? "+" : "-", abs(filtered_speed_a), dirR ? "+" : "-", abs(filtered_speed_b));
	
//    printf("%d, %s\r\n", abs(speed_a), dirL ? "+" : "-");
//    printf("%d, %s\r\n", abs(speed_b), dirR ? "+" : "-");
}
