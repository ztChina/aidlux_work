#ifndef __HX711_H
#define __HX711_H

#include "stm32f4xx_hal.h"

// ������������
typedef uint32_t u32;
typedef int32_t s32;
typedef uint8_t u8;

// ȫ�ֱ�������
extern u32 HX711_Buffer;
extern u32 Weight_Maopi;
extern s32 Weight_Shiwu;
extern u8 Flag_Error;

// У׼����
#define GapValue 40.2f

// ��������
void Init_HX711pin(void);
u32 HX711_Read(void);
void Get_Maopi(void);
void Get_Weight(void);

#endif