#ifndef SERIAL_COMMAND_H
#define SERIAL_COMMAND_H

#include "main.h"

void SerialCommand_Init(void);
void SerialCommand_ProcessChar(uint8_t ch);  // 外部调用，处理每一个接收的字节

extern char rx_buffer[64];
extern uint8_t rx_index;
extern uint8_t rx_data;

#endif // SERIAL_COMMAND_H
