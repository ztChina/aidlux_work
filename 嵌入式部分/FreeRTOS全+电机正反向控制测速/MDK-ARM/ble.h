#ifndef BLE_H
#define BLE_H

#include "usart.h"  // 串口头文件
#include <stdint.h>  // 标准类型定义

// 声明蓝牙缓冲区变量（在ble.c中定义）
extern uint8_t ble_rx_data;
extern uint8_t ble_rx_buffer[256];
extern uint16_t ble_rx_index;

// 蓝牙数据长度宏定义
#define BLE_DATA_LEN 10  

// 声明中断回调函数
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void BLE_UART_ReceiveCallback(uint8_t data);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif
