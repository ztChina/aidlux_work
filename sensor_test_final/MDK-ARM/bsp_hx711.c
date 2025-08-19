 #include "bsp_hx711.h"
#include "main.h"

// ȫ�ֱ�������
u32 HX711_Buffer = 0;
u32 Weight_Maopi = 0;
s32 Weight_Shiwu = 0;
u8 Flag_Error = 0;

// ���Ŷ���
#define HX711_SCK_PORT GPIOE
#define HX711_SCK_PIN GPIO_PIN_8
#define HX711_DOUT_PORT GPIOE
#define HX711_DOUT_PIN GPIO_PIN_9

// ΢����ʱ����
static void delay_us(uint32_t us)
{
    uint32_t ticks = us * (SystemCoreClock / 1000000) / 5;
    while(ticks--);
}

void Init_HX711pin(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // ����GPIOBʱ��
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // ����SCK����Ϊ���
    GPIO_InitStruct.Pin = HX711_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(HX711_SCK_PORT, &GPIO_InitStruct);
    
    // ����DOUT����Ϊ����
    GPIO_InitStruct.Pin = HX711_DOUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(HX711_DOUT_PORT, &GPIO_InitStruct);
    
    // ��ʼ��SCKΪ�͵�ƽ
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
}

u32 HX711_Read(void)
{
    unsigned long count = 0;
    unsigned char i;
    
    // �ȴ�DOUT���
    while(HAL_GPIO_ReadPin(HX711_DOUT_PORT, HX711_DOUT_PIN) == GPIO_PIN_SET);
    
    // ��ȡ24λ����
    for(i = 0; i < 24; i++)
    {
        // ����ʱ������
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
        delay_us(1);
        
        count <<= 1;
        
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
        delay_us(1);
        
        if(HAL_GPIO_ReadPin(HX711_DOUT_PORT, HX711_DOUT_PIN) == GPIO_PIN_SET)
            count++;
    }
    
    // ��25������
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
    delay_us(1);
    count ^= 0x800000; // ����ת��
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
    
    return count;
}

void Get_Maopi(void)
{
    Weight_Maopi = HX711_Read();
}

void Get_Weight(void)
{
    HX711_Buffer = HX711_Read();
    
    if(HX711_Buffer > Weight_Maopi)
    {
        Weight_Shiwu = HX711_Buffer;
        Weight_Shiwu = Weight_Shiwu - Weight_Maopi;
        Weight_Shiwu = (s32)((float)Weight_Shiwu / GapValue);
    }
}