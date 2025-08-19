#include "ble.h"

#include "usart.h"

// 蓝牙接收缓冲区
uint8_t ble_rx_data;         // 单字节接收
uint8_t ble_rx_buffer[256];  // 数据缓冲区
uint16_t ble_rx_index = 0;  // 缓冲区索引


// 假设蓝牙固定发10字节数据
#define BLE_DATA_LEN 10  




void BLE_UART_ReceiveCallback(uint8_t data)
{
    ble_rx_buffer[ble_rx_index++] = data;

    if (ble_rx_index >= BLE_DATA_LEN) {
        ble_rx_buffer[BLE_DATA_LEN] = '\0';
        printf("[蓝牙数据] %s\r\n", ble_rx_buffer);
        ble_rx_index = 0;
    }
}

// 错误处理回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        printf("蓝牙通信错误！\r\n");
        HAL_UART_Receive_IT(&huart2, &ble_rx_data, 1);  // 重新启动接收
    }
}


