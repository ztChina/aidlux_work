#include "bsp_hx711.h"
#include "main.h"

static uint32_t hx711_offset = 0;
static float hx711_scale = 1.0f;

// ΢�뼶��ʱ����
static void delay_us(uint32_t us) {
    uint32_t ticks = us * (SystemCoreClock / 1000000) / 5;
    while(ticks--);
}

void HX711_Init(void) {
    // ����ʱ��
    HX711_SCK_CLK_ENABLE();
    HX711_DT_CLK_ENABLE();
    
    // ��ʼ��SCKΪ���
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = HX711_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(HX711_SCK_PORT, &GPIO_InitStruct);
    
    // ��ʼ��DTΪ����
    GPIO_InitStruct.Pin = HX711_DT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(HX711_DT_PORT, &GPIO_InitStruct);
    
    // ��ʼ״̬
    HX711_SCK_RESET();
}

uint32_t HX711_ReadData(void) {
    uint32_t data = 0;
    
    // �ȴ�DT׼����
    while(HX711_DT_READ());
    
    // ��ȡ24λ����
    for(uint8_t i = 0; i < 24; i++) {
        HX711_SCK_SET();
        delay_us(1);
        data <<= 1;
        HX711_SCK_RESET();
        delay_us(1);
        if(HX711_DT_READ()) data++;
    }
    
    // ��25������ѡ��ͨ��������
    HX711_SCK_SET();
    delay_us(1);
    data ^= 0x800000; // ����ת��
    HX711_SCK_RESET();
    
    return data;
}

void HX711_Calibrate(uint32_t knownWeight) {
    // ��ȡ���ƫ��
    hx711_offset = HX711_ReadData();
    
    // ������֪������ȡ����ϵ��
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