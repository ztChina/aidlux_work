/**
 ****************************************************************************************************
 * @file        atk_md0700_v3_fsmc.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MD0700 V3ģ��FSMC�ӿ���������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� M144Z-M4��Сϵͳ��STM32F407��
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __ATK_MD0700_V3_FSMC_H
#define __ATK_MD0700_V3_FSMC_H


/* ATK-MD0700 V3ģ��FSMC�ӿڶ��� */
#define ATK_MD0700_V3_FSMC_BANK                     FSMC_NORSRAM_BANK4                          /* ATK-MD0700 V3ģ������FSMC��Bank */
#define ATK_MD0700_V3_FSMC_BANK_ADDR                (0x6C000000)
#define ATK_MD0700_V3_FSMC_REG_SEL                  (6)
#define ATK_MD0700_V3_FSMC_READ_AST                 0x0F                                        /* ��ʱ��ĵ�ַ����ʱ�䣬��λ��HCLK */
#define ATK_MD0700_V3_FSMC_READ_DST                 0x3C                                        /* ��ʱ������ݽ���ʱ�䣬��λ��HCLK */
#define ATK_MD0700_V3_FSMC_WRITE_AST                0x04                                        /* дʱ��ĵ�ַ����ʱ�䣬��λ��HCLK */
#define ATK_MD0700_V3_FSMC_WRITE_DST                0x04                                        /* дʱ������ݽ���ʱ�䣬��λ��HCLK */
#define ATK_MD0700_V3_FSMC_CLK_ENABLE()             do{ __HAL_RCC_FSMC_CLK_ENABLE(); }while(0)  /* ATK-MD0700 V3ģ������FSMCʱ��ʹ�� */

/* ATK-MD0700 V3ģ��FSMC�ӿڶ�д������ݵ�ַ */
#define ATK_MD0700_V3_FSMC_REG_ADDR                 (ATK_MD0700_V3_FSMC_BANK_ADDR | (((1U << ATK_MD0700_V3_FSMC_REG_SEL) - 1) << 1))
#define ATK_MD0700_V3_FSMC_DAT_ADDR                 (ATK_MD0700_V3_FSMC_BANK_ADDR | ((1U << ATK_MD0700_V3_FSMC_REG_SEL) << 1))

/* ATK-MD0700 V3ģ��FSMC�ӿڶ�д������� */
#define ATK_MD0700_V3_FSMC_REG                      (*(volatile uint16_t *)ATK_MD0700_V3_FSMC_REG_ADDR)
#define ATK_MD0700_V3_FSMC_DAT                      (*(volatile uint16_t *)ATK_MD0700_V3_FSMC_DAT_ADDR)

/* ���Ŷ��� */
#define ATK_MD0700_V3_FSMC_RS_GPIO_PORT             GPIOF
#define ATK_MD0700_V3_FSMC_RS_GPIO_PIN              GPIO_PIN_12
#define ATK_MD0700_V3_FSMC_RS_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_RS_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_CS_GPIO_PORT             GPIOG
#define ATK_MD0700_V3_FSMC_CS_GPIO_PIN              GPIO_PIN_12
#define ATK_MD0700_V3_FSMC_CS_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_CS_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_RD_GPIO_PORT             GPIOD
#define ATK_MD0700_V3_FSMC_RD_GPIO_PIN              GPIO_PIN_4
#define ATK_MD0700_V3_FSMC_RD_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_RD_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_WR_GPIO_PORT             GPIOD
#define ATK_MD0700_V3_FSMC_WR_GPIO_PIN              GPIO_PIN_5
#define ATK_MD0700_V3_FSMC_WR_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_WR_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D0_GPIO_PORT             GPIOD
#define ATK_MD0700_V3_FSMC_D0_GPIO_PIN              GPIO_PIN_14
#define ATK_MD0700_V3_FSMC_D0_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D0_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D1_GPIO_PORT             GPIOD
#define ATK_MD0700_V3_FSMC_D1_GPIO_PIN              GPIO_PIN_15
#define ATK_MD0700_V3_FSMC_D1_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D1_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D2_GPIO_PORT             GPIOD
#define ATK_MD0700_V3_FSMC_D2_GPIO_PIN              GPIO_PIN_0
#define ATK_MD0700_V3_FSMC_D2_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D2_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D3_GPIO_PORT             GPIOD
#define ATK_MD0700_V3_FSMC_D3_GPIO_PIN              GPIO_PIN_1
#define ATK_MD0700_V3_FSMC_D3_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D3_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D4_GPIO_PORT             GPIOE
#define ATK_MD0700_V3_FSMC_D4_GPIO_PIN              GPIO_PIN_7
#define ATK_MD0700_V3_FSMC_D4_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D4_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D5_GPIO_PORT             GPIOE
#define ATK_MD0700_V3_FSMC_D5_GPIO_PIN              GPIO_PIN_8
#define ATK_MD0700_V3_FSMC_D5_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D5_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D6_GPIO_PORT             GPIOE
#define ATK_MD0700_V3_FSMC_D6_GPIO_PIN              GPIO_PIN_9
#define ATK_MD0700_V3_FSMC_D6_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D6_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D7_GPIO_PORT             GPIOE
#define ATK_MD0700_V3_FSMC_D7_GPIO_PIN              GPIO_PIN_10
#define ATK_MD0700_V3_FSMC_D7_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D7_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D8_GPIO_PORT             GPIOE
#define ATK_MD0700_V3_FSMC_D8_GPIO_PIN              GPIO_PIN_11
#define ATK_MD0700_V3_FSMC_D8_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D8_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D9_GPIO_PORT             GPIOE
#define ATK_MD0700_V3_FSMC_D9_GPIO_PIN              GPIO_PIN_12
#define ATK_MD0700_V3_FSMC_D9_GPIO_AF               GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D9_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D10_GPIO_PORT            GPIOE
#define ATK_MD0700_V3_FSMC_D10_GPIO_PIN             GPIO_PIN_13
#define ATK_MD0700_V3_FSMC_D10_GPIO_AF              GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D10_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D11_GPIO_PORT            GPIOE
#define ATK_MD0700_V3_FSMC_D11_GPIO_PIN             GPIO_PIN_14
#define ATK_MD0700_V3_FSMC_D11_GPIO_AF              GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D11_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D12_GPIO_PORT            GPIOE
#define ATK_MD0700_V3_FSMC_D12_GPIO_PIN             GPIO_PIN_15
#define ATK_MD0700_V3_FSMC_D12_GPIO_AF              GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D12_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D13_GPIO_PORT            GPIOD
#define ATK_MD0700_V3_FSMC_D13_GPIO_PIN             GPIO_PIN_8
#define ATK_MD0700_V3_FSMC_D13_GPIO_AF              GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D13_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D14_GPIO_PORT            GPIOD
#define ATK_MD0700_V3_FSMC_D14_GPIO_PIN             GPIO_PIN_9
#define ATK_MD0700_V3_FSMC_D14_GPIO_AF              GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D14_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)
#define ATK_MD0700_V3_FSMC_D15_GPIO_PORT            GPIOD
#define ATK_MD0700_V3_FSMC_D15_GPIO_PIN             GPIO_PIN_10
#define ATK_MD0700_V3_FSMC_D15_GPIO_AF              GPIO_AF12_FSMC
#define ATK_MD0700_V3_FSMC_D15_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)

/* �������� */
void atk_md0700_v3_fsmc_init(void); /* ATK-MD0700 V3ģ��FSMC�ӿڳ�ʼ�� */

#endif
