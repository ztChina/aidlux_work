#include "bsp_hx711.h"
#include "main.h"

static uint32_t hx711_offset = 0;
static float hx711_scale = 1.0f;

// 微秒级延时函数
static void delay_us(uint32_t us) {
    uint32_t ticks = us * (SystemCoreClock / 1000000) / 5;
    while(ticks--);
}

void HX711_Init(void) {
    // 启用时钟
    HX711_SCK_CLK_ENABLE();
    HX711_DT_CLK_ENABLE();
    
    // 初始化SCK为输出
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = HX711_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(HX711_SCK_PORT, &GPIO_InitStruct);
    
    // 初始化DT为输入
    GPIO_InitStruct.Pin = HX711_DT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(HX711_DT_PORT, &GPIO_InitStruct);
    
    // 初始状态
    HX711_SCK_RESET();
}

uint32_t HX711_ReadData(void) {
    uint32_t data = 0;
    
    // 等待DT准备好
    while(HX711_DT_READ());
    
    // 读取24位数据
    for(uint8_t i = 0; i < 24; i++) {
        HX711_SCK_SET();
        delay_us(1);
        data <<= 1;
        HX711_SCK_RESET();
        delay_us(1);
        if(HX711_DT_READ()) data++;
    }
    
    // 第25个脉冲选择通道和增益
    HX711_SCK_SET();
    delay_us(1);
    data ^= 0x800000; // 补码转换
    HX711_SCK_RESET();
    
    return data;
}

void HX711_Calibrate(uint32_t knownWeight) {
    // 获取零点偏移
    hx711_offset = HX711_ReadData();
    
    // 放置已知重量获取比例系数
    if(knownWeight > 0) {
        uint32_t raw = HX711_ReadData();
        hx711_scale = (float)(raw - hx711_offset) / knownWeight;
    }
}

float HX711_GetWeight(void) {
    uint32_t raw = HX711_ReadData();
    if(raw > hx711_offset) {
        return (float)(raw - hx711_offset) / hx711_scale;
    }
    return 0.0f;
}