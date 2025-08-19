/**
 ****************************************************************************************************
 * @file        main.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-09-06
 * @brief       DHT11������ʪ�ȴ����� ʵ��
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ������ H743������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/SDRAM/sdram.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/MPU/mpu.h"
#include "./BSP/DHT11/dht11.h"


int main(void)
{
    uint8_t t = 0;
    uint8_t temperature;
    uint8_t humidity;

    sys_cache_enable();                     /* ��L1-Cache */
    HAL_Init();                             /* ��ʼ��HAL�� */
    sys_stm32_clock_init(160, 5, 2, 4);     /* ����ʱ��, 400Mhz */
    delay_init(400);                        /* ��ʱ��ʼ�� */
    usart_init(115200);                     /* ���ڳ�ʼ�� */
    mpu_memory_protection();                /* ������ش洢���� */
    led_init();                             /* ��ʼ��LED */
    sdram_init();                           /* ��ʼ��SDRAM */
    lcd_init();                             /* ��ʼ��LCD */

    lcd_show_string(30, 50, 200, 16, 16, "STM32", RED);
    lcd_show_string(30, 70, 200, 16, 16, "DHT11 TEST", RED);
    lcd_show_string(30, 90, 200, 16, 16, "ATOM@ALIENTEK", RED);

    while (dht11_init())                                         /* DHT11��ʼ�� */
    {
        lcd_show_string(30, 110, 200, 16, 16, "DHT11 Error", RED);
        delay_ms(200);
        lcd_fill(30, 110, 239, 130 + 16, WHITE);
        delay_ms(200);
    }

    lcd_show_string(30, 110, 200, 16, 16, "DHT11 OK", RED);
    lcd_show_string(30, 130, 200, 16, 16, "Temp:  C", BLUE);
    lcd_show_string(30, 150, 200, 16, 16, "Humi:  %", BLUE);

    while (1)
    {
        if (t % 10 == 0)                                          /* ÿ100ms��ȡһ�� */
        {
            dht11_read_data(&temperature, &humidity);             /* ��ȡ��ʪ��ֵ */
            lcd_show_num(30 + 40, 130, temperature, 2, 16, BLUE); /* ��ʾ�¶� */
            lcd_show_num(30 + 40, 150, humidity, 2, 16, BLUE);    /* ��ʾʪ�� */
        }

        delay_ms(10);
        t++;

        if (t == 20)
        {
            t = 0;
            LED0_TOGGLE();      /* LED0��˸ */
        }
    }
}


