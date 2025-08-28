#ifndef __HX711_H
#define __HX711_H

#include "stm32f4xx_hal.h"

// 定义数据类型
typedef uint32_t u32;
typedef int32_t s32;
typedef uint8_t u8;

// 全局变量声明
extern u32 HX711_Buffer;
extern u32 Weight_Maopi;
extern u32 Weight_error;
extern s32 Weight_Shiwu;
extern u8 Flag_Error;

// 校准参数
#define GapValue 439.2f

// 函数声明
void Init_HX711pin(void);
u32 HX711_Read(void);
void Get_Maopi(void);
void Get_Weight(void);

#endif