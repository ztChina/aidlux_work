#include "DFRobot.h"
#include <stdio.h>  


/* 用户自定义校准值（需实测） */
#define AIR_VALUE    2500    // 传感器在空气中（干燥）的ADC值
#define WATER_VALUE  1200    // 传感器在水中（湿润）的ADC值
/* 湿度区间计算 */
const int intervals = (AIR_VALUE - WATER_VALUE) / 3;

/* 读取土壤湿度 */
uint32_t Read_Soil_Moisture(void) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        return HAL_ADC_GetValue(&hadc1); // 返回12位ADC值（0-4095）
    }
    return 0;
}

/* 判断湿度等级 */
void Check_Moisture_Level(uint32_t moisture) {
    if (moisture > WATER_VALUE && moisture < (WATER_VALUE + intervals)) {
        printf("Very Wet\n");
    } 
    else if (moisture > (WATER_VALUE + intervals) && moisture < (AIR_VALUE - intervals)) {
        printf("Wet\n");
    } 
    else if (moisture < AIR_VALUE && moisture > (AIR_VALUE - intervals)) {
        printf("Dry\n");
    }
}
