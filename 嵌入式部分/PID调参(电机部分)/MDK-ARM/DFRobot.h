#ifndef SENSORS_H
#define SENSORS_H

#include "adc.h"  // ADC头文件
#include <stdint.h>  // 标准类型
#include <stdbool.h> // 布尔类型

// 土壤湿度校准值宏定义
#define AIR_VALUE    2500
#define WATER_VALUE  1200

// 声明湿度区间变量（在sensors.c中定义）
extern const int intervals;

// 声明传感器函数
uint32_t Read_Soil_Moisture(void);
void Check_Moisture_Level(uint32_t moisture);
float Get_Soil_Humidity(uint32_t adc_value);
const char* Get_Humidity_Status(float humidity);

#endif
