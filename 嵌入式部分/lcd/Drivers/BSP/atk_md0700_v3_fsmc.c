/**
 ****************************************************************************************************
 * @file        atk_md0700_v3_fsmc.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MD0700 V3模块FSMC接口驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 M144Z-M4最小系统板STM32F407版
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "./atk_md0700_v3_fsmc.h"
#include "stm32f4xx_hal.h"  // For STM32F407 (adjust this to your specific MCU family)
#include "stm32f4xx_hal_sram.h"


// Assuming SysTick timer is used for delay
void delay_ms(uint32_t ms) {
    HAL_Delay(ms);  // This function is available in HAL library for STM32
}

void delay_us(uint32_t us) {
    uint32_t tickstart = HAL_GetTick();
    uint32_t delay = us / 1000;  // Assuming 1 ms per tick (adjust this for finer precision)
    while ((HAL_GetTick() - tickstart) < delay);
}


/* ATK-MD0700 V3模块FSMC句柄 */
static SRAM_HandleTypeDef g_sram_handle = {0};

/**
 * @brief       ATK-MD0700 V3模块FSMC接口初始化
 * @param       无
 * @retval      无
 */
void atk_md0700_v3_fsmc_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    FSMC_NORSRAM_TimingTypeDef read_timing = {0};
    FSMC_NORSRAM_TimingTypeDef write_timing = {0};
    
    /* 使能时钟 */
    ATK_MD0700_V3_FSMC_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_RS_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_CS_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_RD_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_WR_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D0_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D1_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D2_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D3_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D4_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D5_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D6_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D7_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D8_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D9_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D10_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D11_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D12_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D13_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D14_GPIO_CLK_ENABLE();
    ATK_MD0700_V3_FSMC_D15_GPIO_CLK_ENABLE();
    
    /* 初始化RS引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_RS_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_RS_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_RS_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化CS引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_CS_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_CS_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_CS_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化RD引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_RD_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_RD_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_RD_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化WR引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_WR_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_WR_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_WR_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D0引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D0_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D0_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D0_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D1引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D1_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D1_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D1_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D2引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D2_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D2_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D2_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D3引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D3_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D3_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D3_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D4引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D4_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D4_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D4_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D5引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D5_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D5_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D5_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D6引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D6_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D6_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D6_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D7引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D7_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D7_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D7_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D8引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D8_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D8_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D8_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D9引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D9_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D9_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D9_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D10引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D10_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D10_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D10_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D11引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D11_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D11_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D11_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D12引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D12_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D12_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D12_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D13引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D13_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D13_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D13_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D14引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D14_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D14_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D14_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化D15引脚 */
    gpio_init_struct.Pin        = ATK_MD0700_V3_FSMC_D15_GPIO_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate  = ATK_MD0700_V3_FSMC_D15_GPIO_AF;
    HAL_GPIO_Init(ATK_MD0700_V3_FSMC_D15_GPIO_PORT, &gpio_init_struct);
    
    /* FSMC配置 */
    g_sram_handle.Instance                  = FSMC_NORSRAM_DEVICE;
    g_sram_handle.Extended                  = FSMC_NORSRAM_EXTENDED_DEVICE;
    g_sram_handle.Init.NSBank               = ATK_MD0700_V3_FSMC_BANK;
    g_sram_handle.Init.DataAddressMux       = FSMC_DATA_ADDRESS_MUX_DISABLE;
    g_sram_handle.Init.MemoryType           = FSMC_MEMORY_TYPE_SRAM;
    g_sram_handle.Init.MemoryDataWidth      = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    g_sram_handle.Init.BurstAccessMode      = FSMC_BURST_ACCESS_MODE_DISABLE;
    g_sram_handle.Init.WaitSignalPolarity   = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    g_sram_handle.Init.WrapMode             = FSMC_WRAP_MODE_DISABLE;
    g_sram_handle.Init.WaitSignalActive     = FSMC_WAIT_TIMING_BEFORE_WS;
    g_sram_handle.Init.WriteOperation       = FSMC_WRITE_OPERATION_ENABLE;
    g_sram_handle.Init.WaitSignal           = FSMC_WAIT_SIGNAL_DISABLE;
    g_sram_handle.Init.ExtendedMode         = FSMC_EXTENDED_MODE_ENABLE;
    g_sram_handle.Init.AsynchronousWait     = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    g_sram_handle.Init.WriteBurst           = FSMC_WRITE_BURST_DISABLE;
    g_sram_handle.Init.PageSize             = FSMC_PAGE_SIZE_NONE;
    /* FSMC读时序配置 */
    read_timing.AddressSetupTime            = ATK_MD0700_V3_FSMC_READ_AST;
    read_timing.DataSetupTime               = ATK_MD0700_V3_FSMC_READ_DST;
    read_timing.BusTurnAroundDuration       = 0;
    read_timing.AccessMode                  = FSMC_ACCESS_MODE_A;
    /* FSMC写时序配置 */
    write_timing.AddressSetupTime           = ATK_MD0700_V3_FSMC_WRITE_AST;
    write_timing.DataSetupTime              = ATK_MD0700_V3_FSMC_WRITE_DST;
    write_timing.BusTurnAroundDuration      = 0;
    write_timing.AccessMode                 = FSMC_ACCESS_MODE_A;
    /* 初始化FSMC */
    HAL_SRAM_Init(&g_sram_handle, &read_timing, &write_timing);
}
