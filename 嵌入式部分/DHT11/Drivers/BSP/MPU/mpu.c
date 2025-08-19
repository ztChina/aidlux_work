/**
 ****************************************************************************************************
 * @file        mpu.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-09-06
 * @brief       MPU 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 阿波罗 H743开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20220906
 * 第一次发布
 *
 ****************************************************************************************************
 */
 
#include "./BSP/MPU/mpu.h"
#include "./BSP/LED/led.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
 
 
/**
 * @brief       设置某个区域的MPU保护
 * @param       baseaddr:MPU保护区域的基址(首地址)
 * @param       size:MPU保护区域的大小(必须是32的倍数,单位为字节),可设置的值参考:CORTEX_MPU_Region_Size
 * @param       rnum:MPU保护区编号,范围:0~7,最大支持8个保护区域,可设置的值参考：CORTEX_MPU_Region_Number
 * @param       de:禁止指令访问;0,允许指令访问;1,禁止指令访问
 * @param       ap:访问权限,访问关系如下:可设置的值参考：CORTEX_MPU_Region_Permission_Attributes
 *   @arg       MPU_REGION_NO_ACCESS,无访问（特权&用户都不可访问）
 *   @arg       MPU_REGION_PRIV_RW,仅支持特权读写访问
 *   @arg       MPU_REGION_PRIV_RW_URO,禁止用户写访问（特权可读写访问）
 *   @arg       MPU_REGION_FULL_ACCESS,全访问（特权&用户都可访问）
 *   @arg       MPU_REGION_PRIV_RO,仅支持特权读访问
 *   @arg       MPU_REGION_PRIV_RO_URO,只读（特权&用户都不可以写）
 *   @arg       详见:STM32F7编程手册.pdf,4.6节,Table 89.
 * @param       sen  : 是否允许共用;MPU_ACCESS_NOT_SHAREABLE,不允许;MPU_ACCESS_SHAREABLE,允许
 * @param       cen  : 是否允许cache;MPU_ACCESS_NOT_CACHEABLE,不允许;MPU_ACCESS_CACHEABLE,允许
 * @param       ben  : 是否允许缓冲;MPU_ACCESS_NOT_BUFFERABLE,不允许;MPU_ACCESS_BUFFERABLE,允许
 * @retval      0,成功.
 *              其他,错误.
 */
uint8_t mpu_set_protection(uint32_t baseaddr, uint32_t size, uint32_t rnum, uint8_t de, uint8_t ap, uint8_t sen, uint8_t cen, uint8_t ben)
{
    MPU_Region_InitTypeDef mpu_region_init_handle;

    HAL_MPU_Disable();                                                   /* 配置MPU之前先关闭MPU,配置完成以后在使能MPU */

    mpu_region_init_handle.Enable = MPU_REGION_ENABLE;                   /* 使能该保护区域 */
    mpu_region_init_handle.Number = rnum;                                /* 设置保护区域 */
    mpu_region_init_handle.BaseAddress = baseaddr;                       /* 设置基址 */
    mpu_region_init_handle.DisableExec = de;                             /* 是否允许指令访问 */
    mpu_region_init_handle.Size = size;                                  /* 设置保护区域大小 */
    mpu_region_init_handle.SubRegionDisable = 0X00;                      /* 禁止子区域 */
    mpu_region_init_handle.TypeExtField = MPU_TEX_LEVEL0;                /* 设置类型扩展域为level0 */
    mpu_region_init_handle.AccessPermission = (uint8_t)ap;               /* 设置访问权限, */
    mpu_region_init_handle.IsShareable = sen;                            /* 是否共用? */
    mpu_region_init_handle.IsCacheable = cen;                            /* 是否cache? */
    mpu_region_init_handle.IsBufferable = ben;                           /* 是否缓冲? */
    HAL_MPU_ConfigRegion(&mpu_region_init_handle);                       /* 配置MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);                              /* 开启MPU */
    return 0;
}

/**
 * @brief       设置需要保护的存储块
 * @param       无
 * @note        必须对部分存储区域进行MPU保护,否则可能导致程序运行异常
 *              比如MCU屏不显示,摄像头采集数据出错等等问题...
 */
void mpu_memory_protection(void)
{
    /* 保护整个DTCM,共128K字节 */
    mpu_set_protection( 0x20000000,                 /* 基地址 */
                        MPU_REGION_SIZE_128KB,      /* 长度 */
                        MPU_REGION_NUMBER1, 0,      /* NUMER1,允许指令访问 */
                        MPU_REGION_FULL_ACCESS,     /* 全访问 */
                        MPU_ACCESS_NOT_SHAREABLE,   /* 禁止共用 */
                        MPU_ACCESS_CACHEABLE,       /* 允许cache */
                        MPU_ACCESS_BUFFERABLE);     /* 允许缓冲 */

    /* 保护整个AXI SRAM,共512K字节 */
    mpu_set_protection( 0x24000000,                 /* 基地址 */
                        MPU_REGION_SIZE_512KB,      /* 长度 */
                        MPU_REGION_NUMBER2, 0,      /* NUMER2,允许指令访问 */
                        MPU_REGION_FULL_ACCESS,     /* 全访问 */
                        MPU_ACCESS_SHAREABLE,       /* 允许共用 */
                        MPU_ACCESS_CACHEABLE,       /* 允许cache */
                        MPU_ACCESS_NOT_BUFFERABLE); /* 禁止缓冲 */
    
    /* 保护整个SRAM1~SRAM3,共512K字节 */
    mpu_set_protection( 0x30000000,                 /* 基地址 */
                        MPU_REGION_SIZE_512KB,      /* 长度 */
                        MPU_REGION_NUMBER3, 0,      /* NUMER3,允许指令访问 */
                        MPU_REGION_FULL_ACCESS,     /* 全访问 */
                        MPU_ACCESS_NOT_SHAREABLE,   /* 禁止共用 */
                        MPU_ACCESS_CACHEABLE,       /* 允许cache */
                        MPU_ACCESS_BUFFERABLE);     /* 允许缓冲 */
                        
    /* 保护整个SRAM4,共64K字节 */
    mpu_set_protection( 0x38000000,                 /* 基地址 */
                        MPU_REGION_SIZE_64KB,       /* 长度 */
                        MPU_REGION_NUMBER4, 0,      /* NUMER4,允许指令访问 */
                        MPU_REGION_FULL_ACCESS,     /* 全访问 */
                        MPU_ACCESS_NOT_SHAREABLE,   /* 禁止共用 */
                        MPU_ACCESS_CACHEABLE,       /* 允许cache */
                        MPU_ACCESS_BUFFERABLE);     /* 允许缓冲 */
                        
    /* 保护MCU LCD屏所在的FMC区域,共64M字节 */
    mpu_set_protection( 0x60000000,                 /* 基地址 */
                        MPU_REGION_SIZE_64MB,       /* 长度 */
                        MPU_REGION_NUMBER5, 0,      /* NUMER5,允许指令访问 */
                        MPU_REGION_FULL_ACCESS,     /* 全访问 */
                        MPU_ACCESS_NOT_SHAREABLE,   /* 禁止共用 */
                        MPU_ACCESS_NOT_CACHEABLE,   /* 禁止cache */
                        MPU_ACCESS_NOT_BUFFERABLE); /* 禁止缓冲 */
                        
    /* 保护SDRAM区域,共32M字节 */
    mpu_set_protection( 0xC0000000,                 /* 基地址 */
                        MPU_REGION_SIZE_32MB,       /* 长度 */
                        MPU_REGION_NUMBER6, 0,      /* NUMER6,允许指令访问 */
                        MPU_REGION_FULL_ACCESS,     /* 全访问 */
                        MPU_ACCESS_NOT_SHAREABLE,   /* 禁止共用 */
                        MPU_ACCESS_CACHEABLE,       /* 允许cache */
                        MPU_ACCESS_BUFFERABLE);     /* 允许缓冲 */

                        
    /* 保护整个NAND FLASH区域,共256M字节 */
    mpu_set_protection( 0x80000000,                 /* 基地址 */
                        MPU_REGION_SIZE_256MB,      /* 长度 */
                        MPU_REGION_NUMBER7, 1,      /* NUMER7,禁止指令访问 */
                        MPU_REGION_FULL_ACCESS,     /* 全访问 */
                        MPU_ACCESS_NOT_SHAREABLE,   /* 禁止共用 */
                        MPU_ACCESS_NOT_CACHEABLE,   /* 禁止cache */
                        MPU_ACCESS_NOT_BUFFERABLE); /* 禁止缓冲 */
}





