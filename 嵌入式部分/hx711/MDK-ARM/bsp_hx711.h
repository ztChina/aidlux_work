#ifndef __BSP_HX711_H__
#define __BSP_HX711_H__

#include "stm32f4xx_hal.h"

// 引脚定义 - 使用PB15和PF12
#define HX711_SCK_PIN      GPIO_PIN_15
#define HX711_SCK_PORT     GPIOB
#define HX711_DT_PIN       GPIO_PIN_12
#define HX711_DT_PORT      GPIOF

// 时钟使能宏
#define HX711_SCK_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define HX711_DT_CLK_ENABLE()   __HAL_RCC_GPIOF_CLK_ENABLE()

// 操作宏
#define HX711_SCK_SET()    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET)
#define HX711_SCK_RESET()  HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET)
#define HX711_DT_READ()    HAL_GPIO_ReadPin(HX711_DT_PORT, HX711_DT_PIN)

// 函数声明
void HX711_Init(void);
uint32_t HX711_ReadData(void);
void HX711_Calibrate(uint32_t knownWeight);
float HX711_GetWeight(void);

#endif /* __BSP_HX711_H__ */