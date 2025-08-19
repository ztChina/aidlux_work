/**
 ****************************************************************************************************
 * @file        sdram.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-09-06
 * @brief       SDRAM 驱动代码
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

#include "./BSP/SDRAM/sdram.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/sys/sys.h"
#include "./BSP/LCD/lcd.h"


SDRAM_HandleTypeDef g_sdram_handle;                                           /* SRAM句柄 */

/**
 * @brief       初始化SDRAM
 * @param       无
 * @retval      无
 */
void sdram_init(void)
{
    FMC_SDRAM_TimingTypeDef sdram_timing;

    g_sdram_handle.Instance = FMC_SDRAM_DEVICE;                               /* SDRAM在BANK5,6 */
    g_sdram_handle.Init.SDBank = FMC_SDRAM_BANK1;                             /* 第一个SDRAM BANK */
    g_sdram_handle.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;       /* 列数量 */
    g_sdram_handle.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;            /* 行数量 */
    g_sdram_handle.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;         /* 数据宽度为16位 */
    g_sdram_handle.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;    /* 一共4个BANK */
    g_sdram_handle.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;                 /* CAS为2 */
    g_sdram_handle.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE; /* 失能写保护 */
    g_sdram_handle.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;             /* SDRAM时钟=pll2_r_ck/2=220M/2=110M=9.1ns */
    g_sdram_handle.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;                  /* 使能突发 */
    g_sdram_handle.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;              /* 读通道延时 */
    
    sdram_timing.LoadToActiveDelay = 2;                                       /* 加载模式寄存器到激活时间的延迟为2个时钟周期 */
    sdram_timing.ExitSelfRefreshDelay = 8;                                    /* 退出自刷新延迟为8个时钟周期 */
    sdram_timing.SelfRefreshTime = 7;                                         /* 自刷新时间为7个时钟周期 */
    sdram_timing.RowCycleDelay = 7;                                           /* 行循环延迟为7个时钟周期 */
    sdram_timing.WriteRecoveryTime = 2;                                       /* 恢复延迟为2个时钟周期 */
    sdram_timing.RPDelay = 2;                                                 /* 行预充电延迟为2个时钟周期 */
    sdram_timing.RCDDelay = 2;                                                /* 行到列延迟为2个时钟周期 */
    HAL_SDRAM_Init(&g_sdram_handle, &sdram_timing);

    sdram_initialization_sequence();                                          /* 发送SDRAM初始化序列 */

    /**
     * 刷新频率计数器(以SDCLK频率计数),计算方法:
     * COUNT=SDRAM刷新周期/行数-20=SDRAM刷新周期(us)*SDCLK频率(Mhz)/行数
     * 我们使用的SDRAM刷新周期为64ms,SDCLK=220/2=110Mhz,行数为8192(2^13).
     * 所以,COUNT=64*1000*110/8192-20=839
     */
    HAL_SDRAM_ProgramRefreshRate(&g_sdram_handle, 839);                       /* 设置刷新频率 */
}

/**
 * @brief       发送SDRAM初始化序列
 * @param       无
 * @retval      无
 */
void sdram_initialization_sequence(void)
{
    uint32_t temp = 0;

    /* SDRAM控制器初始化完成以后还需要按照如下顺序初始化SDRAM */
    sdram_send_cmd(0, FMC_SDRAM_CMD_CLK_ENABLE, 1, 0);                /* 时钟配置使能 */
    delay_us(500);                                                    /* 至少延时500us */
    sdram_send_cmd(0, FMC_SDRAM_CMD_PALL, 1, 0);                      /* 对所有存储区预充电 */
    sdram_send_cmd(0, FMC_SDRAM_CMD_AUTOREFRESH_MODE, 8, 0);          /* 设置自刷新次数 */

    /* 配置模式寄存器,SDRAM的bit0~bit2为指定突发访问的长度，
     * bit3为指定突发访问的类型，bit4~bit6为CAS值，bit7和bit8为运行模式
     * bit9为指定的写突发模式，bit10和bit11位保留位 */
    temp = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1  |                  /* 设置突发长度:1(可以是1/2/4/8) */
              SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL  |                  /* 设置突发类型:连续(可以是连续/交错) */
              SDRAM_MODEREG_CAS_LATENCY_2          |                  /* 设置CAS值:2(可以是2/3) */
              SDRAM_MODEREG_OPERATING_MODE_STANDARD|                  /* 设置操作模式:0,标准模式 */
              SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;                   /* 设置突发写模式:1,单点访问 */
    sdram_send_cmd(0, FMC_SDRAM_CMD_LOAD_MODE, 1, temp);              /* 设置SDRAM的模式寄存器 */
}

/**
 * @brief       SDRAM底层驱动，引脚配置，时钟使能
 * @note        此函数会被HAL_SDRAM_Init()调用
 * @param       hsdram   : SDRAM句柄
 * @retval      无
 */
void HAL_SDRAM_MspInit(SDRAM_HandleTypeDef *hsdram)
{
    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_FMC_CLK_ENABLE();                  /* 使能FMC时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();                /* 使能GPIOC时钟 */
    __HAL_RCC_GPIOD_CLK_ENABLE();                /* 使能GPIOD时钟 */
    __HAL_RCC_GPIOE_CLK_ENABLE();                /* 使能GPIOE时钟 */
    __HAL_RCC_GPIOF_CLK_ENABLE();                /* 使能GPIOF时钟 */
    __HAL_RCC_GPIOG_CLK_ENABLE();                /* 使能GPIOG时钟 */
    
    gpio_init_struct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;  
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;     /* 推挽复用 */
    gpio_init_struct.Pull = GPIO_PULLUP;         /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_HIGH;    /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF12_FMC;  /* 复用为FMC */
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);     /* 初始化PC0,2,3 */
    
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);     /* 初始化PD0,1,8,9,10,14,15 */
    
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;              
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);     /* 初始化PE0,1,7,8,9,10,11,12,13,14,15 */
    
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;              
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);     /* 初始化PF0,1,2,3,4,5,11,12,13,14,15 */
    
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_15;              
    HAL_GPIO_Init(GPIOG, &gpio_init_struct);     /* 初始化PG0,1,2,4,5,8,15 */
}

/**
 * @brief       向SDRAM发送命令
 * @param       bankx   : 0,向BANK5上面的SDRAM发送指令
 *   @arg                 1,向BANK6上面的SDRAM发送指令
 * @param       cmd     : 指令(0,正常模式/1,时钟配置使能/2,预充电所有存储区/3,自动刷新/4,加载模式寄存器/5,自刷新/6,掉电)
 * @param       refresh : 自刷新次数
 * @retval      返回值:0,正常;1,失败.
 * @retval      模式寄存器的定义
 */
uint8_t sdram_send_cmd(uint8_t bankx, uint8_t cmd, uint8_t refresh, uint16_t regval)
{
    uint32_t target_bank = 0;
    FMC_SDRAM_CommandTypeDef command;
    
    if (bankx == 0)
    {
        target_bank = FMC_SDRAM_CMD_TARGET_BANK1;
    }
    else if (bankx == 1)
    {
        target_bank = FMC_SDRAM_CMD_TARGET_BANK2;
    }

    command.CommandMode = cmd;                                              /* 命令 */
    command.CommandTarget = target_bank;                                    /* 目标SDRAM存储区域 */
    command.AutoRefreshNumber = refresh;                                    /* 自刷新次数 */
    command.ModeRegisterDefinition = regval;                                /* 要写入模式寄存器的值 */

    if (HAL_SDRAM_SendCommand(&g_sdram_handle, &command, 0X1000) == HAL_OK) /* 向SDRAM发送命令 */
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/**
 * @brief       在指定地址(WriteAddr+BANK5_SDRAM_ADDR)开始,连续写入n个字节
 * @param       pbuf      : 字节指针
 * @param       writeaddr : 要写入的地址
 * @param       n         : 要写入的字节数
 * @retval      无
*/
void fmc_sdram_write_buffer(uint8_t *pbuf, uint32_t writeaddr, uint32_t n)
{
    for (; n != 0; n--)
    {
        *(volatile uint8_t *)(BANK5_SDRAM_ADDR + writeaddr) = *pbuf;
        writeaddr++;
        pbuf++;
    }
}

/**
 * @brief       在指定地址((WriteAddr+BANK5_SDRAM_ADDR))开始,连续读出n个字节.
 * @param       pbuf     : 字节指针
 * @param       readaddr : 要读出的起始地址
 * @param       n        : 要写入的字节数
 * @retval      无
*/
void fmc_sdram_read_buffer(uint8_t *pbuf, uint32_t readaddr, uint32_t n)
{
    for (; n != 0; n--)
    {
        *pbuf++ = *(volatile uint8_t *)(BANK5_SDRAM_ADDR + readaddr);
        readaddr++;
    }
}
