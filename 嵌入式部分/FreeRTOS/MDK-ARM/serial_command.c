#include "serial_command.h"
#include "motor_control.h"
#include "usart.h"   // <- 为了用 huart1
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <main.h>
#include "cmsis_os2.h"
//#include "oled.h"

// 接收数据
uint8_t rx_data = 0;   // <- 让 main.c 可以 extern 用
char rx_buffer[64];
uint8_t rx_index = 0;

float get_lv = 0.0f;
float get_rv = 0.0f;

// 提前声明
static void ParseCommand(char *cmd);
TargetSpeed_t target_speed = {0};//用于控制
osMutexId_t targetSpeedMutexHandle;

void SerialCommand_Init(void)
{
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_buffer[rx_index], 1);
}

// 外部从回调中调用
//void SerialCommand_ProcessChar(uint8_t ch)
//{
//    if (ch == '\r' || ch == '\n') return;

//    if (rx_index == 0 && ch != 'V') return;

//    if (rx_index < sizeof(rx_buffer) - 1) {
//        rx_buffer[rx_index++] = ch;
//    }

//    if (ch == 'E') {
//        rx_buffer[rx_index] = '\0';
//        ParseCommand(rx_buffer);
//        rx_index = 0;
//    }
//}


void SerialCommand_ProcessChar(uint8_t rx_data)
{
	static uint8_t frame[64];
	static int state = 0;
	static uint8_t data_len = 0;
	static int index = 0;

	switch (state)
	{
			case 0: // 等待帧头1
					if (rx_data == 0xAA)
							state = 1;
					break;
			case 1: // 等待帧头2
					if (rx_data == 0x55)
							state = 2;
					else
							state = 0;
					break;
			case 2: // 长度字段
					data_len = rx_data;
					index = 0;
					frame[0] = data_len;
					state = 3;
					break;
			case 3: // 接收数据
					frame[1 + index] = rx_data;
					index++;
					if(index >= data_len)  // 接收到完整帧
					{
							// 校验
							uint8_t checksum = 0;
							for(int i = 0; i < data_len - 1; i++)
									checksum ^= frame[i];

							if(checksum == frame[data_len - 1]) {
									Process_Command_Frame(frame);  // 处理功能码
							}
							state = 0;  // 准备接收下一帧
					}
					break;
	}

}




void Process_Command_Frame(uint8_t* frame)
{
    // 检查帧头
    if (frame[0] != 0xAA || frame[1] != 0x55)
        return;

    uint8_t length = frame[2];   // 从 func_code 到 checksum 的总长度
    uint8_t func_code = frame[3];

    // 最小长度：func_code + checksum
    if (length < 2)
        return;

    // 计算总帧长度
    uint8_t total_len = 3 + length;
//		printf("Frame Dump: ");
//		for (int i = 0; i < 3 + length; i++) {
//				printf("%02X ", frame[i]);
//		}
//		printf("\n");
		// 校验 checksum（从 frame[2] 开始到 frame[2 + length - 2]，即不含最后校验字节）
		uint8_t checksum = 0;
		for (int i = 2; i < 2 + length; ++i) {
				checksum ^= frame[i];
		}
		if (checksum != frame[2 + length]) {
				printf("Checksum error. Calc: 0x%02X, Got: 0x%02X\n", checksum, frame[2 + length]);
				return;
		}

    // 有效载荷长度 = length - func_code(1) - checksum(1)
    uint8_t data_len = length - 2;
    uint8_t float_count = data_len / 4;

    // 处理不同功能码
    switch (func_code) {
        case 0x02: {
            if (float_count < 2) return;

            float v_left = 0.0f, v_right = 0.0f;
            memcpy(&v_left,  &frame[4], 4);
            memcpy(&v_right, &frame[8], 4);

//            printf("Got speed cmd: vL=%.3f, vR=%.3f\n", v_left, v_right);

            Motor_Control_SetSpeed(v_left, v_right);  // 直接驱动控制函数
						get_lv = v_left;
						get_rv = v_right;
//						Display_Speeds(v_left, v_right);  // 显示速度到 OLED
            break;
        }

        case 0x01: {
            // 可扩展处理其他功能码
            break;
        }

        default:
            printf("Unknown function code: 0x%02X\n", func_code);
            break;
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

    char left_str[10] = {0}, right_str[10] = {0};
    strncpy(left_str, p, split - p);
    left_str[split - p] = '\0';
    strcpy(right_str, split);

    int speedL = atoi(left_str);
    int speedR = atoi(right_str);

    int dirL = (speedL >= 0) ? 1 : 0;
    int dirR = (speedR >= 0) ? 1 : 0;

    Motor_Set(abs(speedL), abs(speedR), dirL, dirR);
    Motor_UpdateSpeed();

	printf("%s, %d, %s, %d\r\n", dirL ? "+" : "-", abs(speed_a), dirR ? "+" : "-", abs(speed_b));
	
//    printf("%d, %s\r\n", abs(speed_a), dirL ? "+" : "-");
//    printf("%d, %s\r\n", abs(speed_b), dirR ? "+" : "-");
}

void Serial_SendByte(uint8_t Byte)
{
//	USART_SendData(USART1, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
//	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
	HAL_UART_Transmit(&huart3, &Byte, 1, HAL_MAX_DELAY);
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}


 

