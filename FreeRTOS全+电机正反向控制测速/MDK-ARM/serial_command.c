#include "serial_command.h"
#include "motor_control.h"
#include "usart.h"   // <- Ϊ���� huart1
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

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

    char left_str[10] = {0}, right_str[10] = {0};
    strncpy(left_str, p, split - p);
    left_str[split - p] = '\0';
    strcpy(right_str, split);

    int speedL = atoi(left_str);
    int speedR = atoi(right_str);

    int dirL = (speedL >= 0) ? 1 : 0;
    int dirR = (speedR >= 0) ? 1 : 0;

    Motor_Set(abs(speedL) * 10, abs(speedR) * 10, dirL, dirR);
    Motor_UpdateSpeed();

    printf("%d, %s\r\n", abs(speed_a), dirL ? "+" : "-");
    printf("%d, %s\r\n", abs(speed_b), dirR ? "+" : "-");
}
