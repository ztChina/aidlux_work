#include "ds18b20.h"
#include "delay.h"

/* ??§Ü??????? */
static void DS18B20_Rst(void);
static void DS18B20_WriteByte(uint8_t dat);
static uint8_t DS18B20_ReadByte(void);
static uint8_t DS18B20_ReadBit(void);

/**
  * @brief  ??¦ËDS18B20
  */
static void DS18B20_Rst(void)
{
    DS18B20_DQ_OUT(0);     // ????DQ
    delay_us(750);         // ???????750us
    DS18B20_DQ_OUT(1);     // ???????
    delay_us(15);          // ???15us
}

/**
  * @brief  ???DS18B20??????
  * @retval 0-????, 1-??????
  */
uint8_t DS18B20_Check(void)
{
    uint8_t retry = 0;
    
    DS18B20_Rst();
    
    // ???DQ???????????
    while (DS18B20_DQ_IN && retry < 200) {
        retry++;
        delay_us(1);
    }
    if (retry >= 200) return 1;
    
    retry = 0;
    // ???DQ????60~240us??
    while (!DS18B20_DQ_IN && retry < 240) {
        retry++;
        delay_us(1);
    }
    if (retry >= 240) return 1;
    
    return 0;
}

/**
  * @brief  ??DS18B20??????¦Ë
  * @retval ???????¦Ë?(1/0)
  */
static uint8_t DS18B20_ReadBit(void)
{
    uint8_t data;
    
    DS18B20_DQ_OUT(0);     // ???????????
    delay_us(2);
    DS18B20_DQ_OUT(1);     // ???????
    delay_us(12);          // ???12us?????
    
    if (DS18B20_DQ_IN) data = 1;
    else data = 0;
    
    delay_us(50);          // ???60us????
    return data;
}

/**
  * @brief  ??DS18B20?????????
  * @retval ??????????
  */
static uint8_t DS18B20_ReadByte(void)
{
    uint8_t i, dat = 0;
    
    for (i = 0; i < 8; i++) {
        dat >>= 1;
        if (DS18B20_ReadBit()) dat |= 0x80;
    }
    
    return dat;
}

/**
  * @brief  ??DS18B20§Õ????????
  * @param  dat: ?§Õ??????
  */
static void DS18B20_WriteByte(uint8_t dat)
{
    uint8_t j;
    uint8_t testb;
    
    for (j = 0; j < 8; j++) {
        testb = dat & 0x01;
        dat >>= 1;
        
        if (testb) {
            // §Õ1
            DS18B20_DQ_OUT(0);
            delay_us(2);
            DS18B20_DQ_OUT(1);
            delay_us(60);
        } else {
            // §Õ0
            DS18B20_DQ_OUT(0);
            delay_us(60);
            DS18B20_DQ_OUT(1);
            delay_us(2);
        }
    }
}

/**
  * @brief  ?????DS18B20
  * @retval 0-???, 1-???
  */
uint8_t DS18B20_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* ???GPIO??? */
    DS18B20_CLK_ENABLE();
    
    /* ????GPIO??????? */
    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);
    
    /* ????????????? */
    return DS18B20_Check();
}

/**
  * @brief  ?????????
  */
void DS18B20_Start(void)
{
    DS18B20_Rst();
    DS18B20_Check();
    DS18B20_WriteByte(0xCC);   // ????ROM???
    DS18B20_WriteByte(0x44);    // ??????????
}

/**
  * @brief  ???????
  * @retval ????(????0.1??C, -550~1250)
  */
int16_t DS18B20_GetTemp(void)
{
    uint8_t TL, TH;
    int16_t temp;
    uint8_t sign = 0;
    
    DS18B20_Start();            // ??????????
    delay_ms(750);              // ?????????
    
    DS18B20_Rst();
    DS18B20_Check();
    DS18B20_WriteByte(0xCC);    // ????ROM???
    DS18B20_WriteByte(0xBE);    // ????????
    
    TL = DS18B20_ReadByte();    // ???LSB
    TH = DS18B20_ReadByte();    // ???MSB
    
    // ?????????
    if (TH > 7) {
        TH = ~TH;
        TL = ~TL;
        sign = 1;
    }
    
    temp = TH;
    temp <<= 8;
    temp += TL;
    temp = (double)temp * 0.625; // ????0.1??C????
    
    return sign ? -temp : temp;
}
