#include "serial_command.h"
#include "motor_control.h"
#include "usart.h"   // <- Ϊ���� huart1
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "pid.h"

// ��������
uint8_t rx_data = 0;   // <- �� main.c ���� extern ��
char rx_buffer[64];
uint8_t rx_index = 0;

// ��ǰ����
static void ParseCommand(char *cmd);


void SerialCommand_Init(void)
{
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_buffer[rx_index], 1);
}

// �ⲿ�ӻص��е���
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

// ˽�к��������ڱ��ļ�ʹ��
void ParseCommand(char *cmd)
{
    int len = strlen(cmd);
    if (cmd[0] != 'V' || cmd[len - 1] != 'E') return;
    cmd[len - 1] = '\0';  // ȥ��β�� E

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

    float speedL = atof(left_str);
    float speedR = atof(right_str);

    if (fabs(speedL) < 0.01f && fabs(speedR) < 0.01f) {
        filtered_speed_a = 0.0f;
        filtered_speed_b = 0.0f;
    }

    Motor_SetTargetSpeed(speedL, speedR);

    // ------------------ PID ���Ʋ��� ------------------

    pid_a.current = Motor_GetFilteredSpeedA();
    pid_b.current = Motor_GetFilteredSpeedB();

    pid_a.output = PID_Calc(&pid_a, Motor_GetFilteredSpeedA());
	pid_b.output = PID_Calc(&pid_b, Motor_GetFilteredSpeedB());

    Motor_SetPWM(pid_a.output, pid_b.output);
    Motor_UpdateSpeed();

    // ------------------ Debug ��� ------------------
    float v_a = pid_a.current;
    float v_b = pid_b.current;

    printf("Target L=%.2f R=%.2f | Real L=%.2f R=%.2f | PWM=%.1f %.1f\r\n",pid_a.target, pid_b.target, v_a, v_b,pid_a.output, pid_b.output);
}
