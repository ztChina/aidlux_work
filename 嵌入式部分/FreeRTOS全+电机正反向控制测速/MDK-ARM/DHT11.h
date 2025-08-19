#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f4xx_hal.h"

/* 引脚定义（修改为PF12） */
#define DHT11_GPIO_PORT  GPIOF
#define DHT11_GPIO_PIN   GPIO_PIN_12
#define DHT11_GPIO_CLK_ENABLE() __HAL_RCC_GPIOF_CLK_ENABLE()

/* IO操作宏 */
#define DHT11_DQ_OUT(x)  HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define DHT11_DQ_IN      HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN)

/* 函数声明 */
uint8_t dht11_init(void);
uint8_t dht11_read_data(uint8_t *temp, uint8_t *humi);

#endif
