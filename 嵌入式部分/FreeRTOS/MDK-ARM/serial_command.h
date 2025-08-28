#ifndef SERIAL_COMMAND_H
#define SERIAL_COMMAND_H

#include "main.h"
#include "cmsis_os2.h"

void SerialCommand_Init(void);
//void SerialCommand_ProcessChar(uint8_t ch);  // �ⲿ���ã�����ÿһ�����յ��ֽ�
void SerialCommand_ProcessChar(uint8_t rx_data);  // �ⲿ���ã�����ÿһ�����յ��ֽ�
void Serial_SendByte(uint8_t Byte);
void Process_Command_Frame(uint8_t* frame);
extern char rx_buffer[64];
extern uint8_t rx_index;
extern uint8_t rx_data;

typedef struct {
    float v_left;
    float v_right;
} TargetSpeed_t;

extern TargetSpeed_t target_speed;
extern osMutexId_t targetSpeedMutexHandle;


#endif // SERIAL_COMMAND_H
