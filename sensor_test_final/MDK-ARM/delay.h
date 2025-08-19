#ifndef DELAY_H
#define DELAY_H

#include "tim.h"  // timer 's head file

// 
extern TIM_HandleTypeDef htim1;

// clare delay function
void delay_us(uint16_t us);
void delay_ms(uint32_t ms);


#endif
