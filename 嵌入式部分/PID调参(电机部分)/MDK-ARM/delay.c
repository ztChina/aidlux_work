#include "delay.h"  // 引用自己的头文件

void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);  // 计数器归零
    HAL_TIM_Base_Start(&htim1);        // 启动定时器
    while (__HAL_TIM_GET_COUNTER(&htim1) < us); // 等待
    HAL_TIM_Base_Stop(&htim1);         // 停止定时器
}

void delay_ms(uint32_t ms) {
    while (ms--) {
        delay_us(1000);  // 每次延时 1ms
    }
}
