/**
 ****************************************************************************************************
 * @file        atk_md0700_v3.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MD0700 V3模块驱动代码
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

#include "atk_md0700_v3.h"
#include "atk_md0700_v3_font.h"
#include "atk_md0700_v3_fsmc.h"
//#include "./SYSTEM/delay/delay.h"
//#include "./SYSTEM/usart/usart.h"

/* ATK-MD0700 V3模块LCD时序结构体 */
typedef struct
{
    uint8_t clock_freq_in_mhz;
    uint16_t hactive;
    uint8_t hback_porch;
    uint8_t hfront_porch;
    uint8_t hsync_len;
    uint16_t vactive;
    uint8_t vback_porch;
    uint8_t vfront_porch;
    uint8_t vsync_len;
} atk_md0700_v3_lcd_timing_t;

/* ATK-MD0700 V3模块参数结构体 */
typedef struct
{
    uint8_t id;
    uint16_t pid;
    uint16_t width;
    uint16_t height;
    atk_md0700_v3_lcd_timing_t timing;
} atk_md0700_v3_lcd_param_t;

/* ATK-MD0700 V3模块参数匹配表 */
static const atk_md0700_v3_lcd_param_t atk_md0700_v3_lcd_param[] = {
    {0,  ATK_MD0700_V3_PID_800480,  800, 480, {33,  800,  46, 210,  1, 480, 23, 22, 1}},  /* ATK-MD0700 V3 800*480 */
    {1, ATK_MD0700_V3_PID_1024600, 1024, 600, {45, 1024, 140, 160, 20, 600, 20, 12, 3}},  /* ATK-MD0700 V3 1024*600 */
};

/* ATK-MD0700 V3模块状态数据结构体 */
static struct
{
    const atk_md0700_v3_lcd_param_t *param;
    uint16_t width;
    uint16_t height;
    uint8_t disp_dir;
    uint8_t scan_dir;
} g_atk_md0700_v3_sta = {0};

/**
 * @brief       ATK-MD0700 V3模块读状态寄存器
 * @param       无
 * @retval      ATK-MD0700 V3模块状态寄存器值
 */
uint16_t atk_md0700_v3_read_stsr(void)
{
    return ATK_MD0700_V3_FSMC_REG;
}

/**
 * @brief       ATK-MD0700 V3模块设置寄存器
 * @param       reg: 寄存器地址
 * @param       dat: 数据
 * @retval      无
 */
void atk_md0700_v3_set_reg(volatile uint16_t reg, volatile uint16_t dat)
{
    ATK_MD0700_V3_FSMC_REG = reg;
    ATK_MD0700_V3_FSMC_DAT = dat;
}

/**
 * @brief       获取ATK-MD0700 V3模块寄存器值
 * @param       reg: 寄存器地址
 * @retval      寄存器值
 */
uint16_t atk_md0700_v3_get_reg(volatile uint16_t reg)
{
    volatile uint16_t dat;
    
    ATK_MD0700_V3_FSMC_REG = reg;
    dat = ATK_MD0700_V3_FSMC_DAT;
    
    return dat;
}

/**
 * @brief       ATK-MD0700 V3模块修改寄存器
 * @param       reg   : 寄存器地址
 * @param       clrbit: 清除位
 * @param       setbit: 设置位
 * @retval      无
 */
void atk_md0700_v3_modify_reg(volatile uint16_t reg, uint16_t clrbit, uint16_t setbit)
{
    volatile uint16_t original;
    
    ATK_MD0700_V3_FSMC_REG = reg;
    original = ATK_MD0700_V3_FSMC_DAT;
    original &= ~(clrbit);
    original |= (setbit);
    ATK_MD0700_V3_FSMC_DAT = original;
}

/**
 * @brief       获取ATK-MD0700 V3模块ID
 * @param       无
 * @retval      ATK-MD0700 V3模块ID
 */
uint8_t atk_md0700_v3_get_id(void)
{
    uint8_t id = 0;
    
    atk_md0700_v3_modify_reg(0x85, (3 << 0), 0);        /* 关闭PWM[0]，使能GPIOC[7] */
    atk_md0700_v3_modify_reg(0x01, (1 << 5), 0);        /* 关闭按键功能，使能GPIOB[4] */
    atk_md0700_v3_modify_reg(0xF3, 0, (1 << 7));        /* 设置GPIOC[7]为输入 */
    delay_ms(100);                                      /* 必要延时，否则ID可能读取失败 */
    
    id = (atk_md0700_v3_get_reg(0xF2) & (1 << 4)) >> 4; /* 读GPIOB[4] */
    id |= (atk_md0700_v3_get_reg(0xF4) & (1 << 7)) >> 6;/* 读GPIOC[7] */
    
    return id;
}

/**
 * @brief       ATK-MD0700 V3模块参数初始化
 * @note        通过ATK-MD0700 V3模块的ID确定LCD的尺寸和时序等参数
 * @param       id: ATK-MD0700 V3模块ID
 * @retval      ATK_MD0700_V3_EOK   : ATK-MD0700 V3模块参数初始化成功
 *              ATK_MD0700_V3_EINVAL: 无效的ATK-MD0700 V3模块ID
 */
static uint8_t atk_md0700_v3_setup_param_by_id(uint8_t id)
{
    uint8_t index;
    
    for (index=0; index<(sizeof(atk_md0700_v3_lcd_param) / sizeof(atk_md0700_v3_lcd_param[0])); index++)
    {
        if (id == atk_md0700_v3_lcd_param[index].id)
        {
            g_atk_md0700_v3_sta.param = &atk_md0700_v3_lcd_param[index];
            return ATK_MD0700_V3_EOK;
        }
    }
    
    return ATK_MD0700_V3_EINVAL;
}

/**
 * @brief       ATK-MD0700 V3模块LT7381初始化
 * @note        通过ATK-MD0700 V3模块ID确定的LCD参数初始化LT7381
 * @param       param: ATK-MD0700 V3模块ID确定的LCD参数
 * @retval      无
 */
static void atk_md0700_v3_setup_lt7381_by_param(const atk_md0700_v3_lcd_param_t *param)
{
    uint8_t hback_porch = param->timing.hback_porch;
    uint8_t hfront_porch = param->timing.hfront_porch;
    uint8_t hsync_len = param->timing.hsync_len;
    
    if (hback_porch < 8)
    {
        hback_porch += 8;
    }
    if (hfront_porch < 8)
    {
        hfront_porch = 8;
    }
    if (hsync_len < 8)
    {
        hsync_len = 8;
    }
    
    /* 等待LT7381进入正常操作模式 */
    while ((atk_md0700_v3_read_stsr() & (1 << 1)) != 0);
    
    /* 配置PLL
     * PPLL：产生PCLK提供LCD屏幕扫描工作频率，最大80MHz
     * MPLL：产生MCLK提供给内部显存（SDRAM）使用，最大133MHz
     * CPLL：产生CCLK提供给主机接口、BTE引擎、绘图引擎、文本DMA数据传输等的使用，最大100MHz
     * 一般配置CCLK = MCLK = 2 * PCLK
     */
    atk_md0700_v3_set_reg(0x05, ((2 << 6) | (5 << 1) | ((param->timing.clock_freq_in_mhz >> 8) & 0x01)));   /* PCLK = 10MHz * (clock_freq_in_mhz / 5) / 2 = pixel_freq_in_mhz(MHz) */
    atk_md0700_v3_set_reg(0x06, (param->timing.clock_freq_in_mhz & 0xFF));
    atk_md0700_v3_set_reg(0x07, ((2 << 6) | (5 << 1) | ((133 >> 8) & 0x01)));                               /* MCLK = 10MHz * (133 / 5) / 2 = 133(MHz) */
    atk_md0700_v3_set_reg(0x08, (133 & 0xFF));
    atk_md0700_v3_set_reg(0x09, ((2 << 6) | (5 << 1) | ((100 >> 8) & 0x01)));                               /* CCLK = 10MHz * (100 / 5) / 2 = 100(MHz) */
    atk_md0700_v3_set_reg(0x0A, (100 & 0xFF));
    
    atk_md0700_v3_modify_reg(0x00, 0, (1 << 7));                                                            /* 重新配置PLL */
    while ((atk_md0700_v3_get_reg(0x01) & (1 << 7)) == 0);                                                  /* 等待PLL就绪 */
    
    /* 配置显存（SDRAM）控制器 */
    atk_md0700_v3_set_reg(0xE0, 0x20);                                                                      /* Bank数量（4） 行地址比特位数（11） 列地址比特位数（8） */
    atk_md0700_v3_set_reg(0xE1, 0x03);                                                                      /* CAS间隔时间 */
    atk_md0700_v3_set_reg(0xE2, (64000 * (133 * 2)) / 2048);                                                /* 自刷新时间间隔 */
    atk_md0700_v3_set_reg(0xE3, ((64000 * (133 * 2)) / 2048) >> 8);
    atk_md0700_v3_modify_reg(0xE4, 0, (1 << 0));                                                            /* 初始化SDRAM */
    while ((atk_md0700_v3_get_reg(0xE4) & (1 << 0)) == 0);
    
    /* 芯片配置寄存器 */
    atk_md0700_v3_modify_reg(0x01, (1 << 6) | (3 << 3), (2 << 3) | (1 << 0));                               /* WAIT#引脚掩码 TFT屏接口（16比特） 主机数据总线宽度（16比特） */
    
    /* 显存访问控制寄存器 */
    atk_md0700_v3_modify_reg(0x02, (3 << 6), 0);                                                            /* 主机图像数据访问格式（16比特 16bpp） */
    
    /* 输入控制寄存器 */
    atk_md0700_v3_modify_reg(0x03, (1 << 2) | (3 << 0), 0);                                                 /* 文本模式使能（图像模式） 内存访问目标（显存） */
    
    /* 配置Main Window */
    atk_md0700_v3_set_reg(0x20, 0 & ~(3 << 0));                                                             /* Main Window起始地址 */
    atk_md0700_v3_set_reg(0x21, 0 >> 8);
    atk_md0700_v3_set_reg(0x22, 0 >> 16);
    atk_md0700_v3_set_reg(0x23, 0 >> 24);
    atk_md0700_v3_set_reg(0x24, param->width & ~(3 << 0));                                                  /* Main Window宽度 */
    atk_md0700_v3_set_reg(0x25, param->width >> 8);
    atk_md0700_v3_set_reg(0x26, 0 & ~(3 << 0));                                                             /* Main Window左上角X坐标 */
    atk_md0700_v3_set_reg(0x27, 0 >> 8);
    atk_md0700_v3_set_reg(0x28, 0);                                                                         /* Main Window左上角Y坐标 */
    atk_md0700_v3_set_reg(0x29, 0 >> 8);
    atk_md0700_v3_modify_reg(0x10, (3 << 2), (1 << 2));                                                     /* Main Window色深（16bpp） */
    
    /* TFT屏幕相关配置 */
    atk_md0700_v3_modify_reg(0x10, (1 << 0), 0);                                                            /* Sync Mode */
    atk_md0700_v3_modify_reg(0x12, (1 << 7), (1 << 7));                                                     /* PCLK Falling */
    atk_md0700_v3_modify_reg(0x13, (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4), 0);                           /* (HSYNC High) (VSYNC High) (PDE High) (PDE IDLE Low) */
    atk_md0700_v3_set_reg(0x14, (param->timing.hactive / 8) - 1);                                           /* Horizontal Active */
    atk_md0700_v3_set_reg(0x15, (param->timing.hactive % 8));
    atk_md0700_v3_set_reg(0x16, (hback_porch / 8) - 1);                                                     /* Horizontal Back Porch */
    atk_md0700_v3_set_reg(0x17, (hback_porch % 8));
    atk_md0700_v3_set_reg(0x18, (hfront_porch / 8) - 1);                                                    /* Horizontal Front Porch */
    atk_md0700_v3_set_reg(0x19, (hsync_len / 8) - 1);                                                       /* Horizontal Sync Length */
    atk_md0700_v3_set_reg(0x1A, (param->timing.vactive - 1));                                               /* Vertical Active */
    atk_md0700_v3_set_reg(0x1B, (param->timing.vactive - 1) >> 8);
    atk_md0700_v3_set_reg(0x1C, (param->timing.vback_porch - 1));                                           /* Vertical Back Porch */
    atk_md0700_v3_set_reg(0x1D, (param->timing.vback_porch - 1) >> 8);
    atk_md0700_v3_set_reg(0x1E, (param->timing.vfront_porch - 1));                                          /* Vertical Front Porch */
    atk_md0700_v3_set_reg(0x1F, (param->timing.vsync_len - 1));                                             /* Vertical Sync Length */
    
    /* 配置Canvas Window */
    atk_md0700_v3_set_reg(0x50, 0 & ~(3 << 0));                                                             /* Canvas Window起始地址 */
    atk_md0700_v3_set_reg(0x51, 0 >> 8);
    atk_md0700_v3_set_reg(0x52, 0 >> 16);
    atk_md0700_v3_set_reg(0x53, 0 >> 24);
    atk_md0700_v3_set_reg(0x54, param->width & ~(3 << 0));                                                  /* Canvas Window宽度 */
    atk_md0700_v3_set_reg(0x55, param->width >> 8);
    atk_md0700_v3_modify_reg(0x5E, (1 << 2) | (3 << 0), (1 << 0));                                          /* Canvas Window寻址模式（Block模式（X-Y坐标寻址）） Canvas Window和Active Window图像色深（16bpp） */
    
    /* 配置Active Window */
    atk_md0700_v3_set_reg(0x56, 0);                                                                         /* Active Window左上角X坐标 */
    atk_md0700_v3_set_reg(0x57, 0 >> 8);
    atk_md0700_v3_set_reg(0x58, 0);                                                                         /* Active Window左上角Y坐标 */
    atk_md0700_v3_set_reg(0x59, 0 >> 8);
    atk_md0700_v3_set_reg(0x5A, param->width);                                                              /* Active Window宽度 */
    atk_md0700_v3_set_reg(0x5B, param->width >> 8);
    atk_md0700_v3_set_reg(0x5C, param->height);                                                             /* Active Window高度 */
    atk_md0700_v3_set_reg(0x5D, param->height >> 8);
    
    /* 配置读写位置坐标 */
    atk_md0700_v3_set_reg(0x5F, 0);                                                                         /* 读写位置X坐标 */
    atk_md0700_v3_set_reg(0x60, 0 >> 8);
    atk_md0700_v3_set_reg(0x61, 0);                                                                         /* 读写位置Y坐标 */
    atk_md0700_v3_set_reg(0x62, 0 >> 8);
    
    /* 开启显示 */
    atk_md0700_v3_modify_reg(0x12, 0, (1 << 6));
    
    /* 配置PWM背光 */
    atk_md0700_v3_set_reg(0x84, (((param->timing.clock_freq_in_mhz * 2) * 1000000) / 500000) - 1);          /* Timer时钟分频系数（Timer1计数频率为500KHz） */
    atk_md0700_v3_modify_reg(0x85, (3 << 6) | (3 << 2), 0);                                                 /* Timer1不分频 PWM[1]不输出PWM */
    atk_md0700_v3_modify_reg(0x86, 0, (1 << 5));                                                            /* Timer1自动重装载 */
    atk_md0700_v3_set_reg(0x8C, 0);                                                                         /* Timer1计数比较值 */
    atk_md0700_v3_set_reg(0x8D, (0 >> 8));
    atk_md0700_v3_set_reg(0x8E, 100);                                                                       /* Timer1重装载值（PWM[1]频率 = Timer1计数频率 / Timer1重装载值 = 500KHz / 100 = 5KHz） */
    atk_md0700_v3_set_reg(0x8F, (100 >> 8));
    atk_md0700_v3_modify_reg(0x86, 0, (1 << 4));                                                            /* Timer1开始计数 */
    
    /* 开启彩条 */
//    atk_md0700_v3_modify_reg(0x12, 0, (1 << 5));
}

/**
 * @brief       开始访问ATK-MD0700 V3模块显存
 * @param       无
 * @retval      无
 */
void atk_md0700_v3_start_access_memory(void)
{
    ATK_MD0700_V3_FSMC_REG = 0x04;
}

/**
 * @brief       配置ATK-MD0700 V3模块显存访问起始坐标
 * @param       无
 * @retval      无
 */
void atk_md0700_v3_config_memory_access_coordinate(uint16_t x, uint16_t y)
{
    if (g_atk_md0700_v3_sta.disp_dir == ATK_MD0700_V3_LCD_DISP_DIR_0)
    {
        x = x ^ y;
        y = x ^ y;
        x = x ^ y;
        y = g_atk_md0700_v3_sta.width - y - 1;
    }
    
    atk_md0700_v3_set_reg(0x5F, x);
    atk_md0700_v3_set_reg(0x60, x >> 8);
    atk_md0700_v3_set_reg(0x61, y);
    atk_md0700_v3_set_reg(0x62, y >> 8);
}

/**
 * @brief       配置ATK-MD0700 V3模块显存访问窗口
 * @param       无
 * @retval      无
 */
void atk_md0700_v3_config_memory_access_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height)
{
    if (g_atk_md0700_v3_sta.disp_dir == ATK_MD0700_V3_LCD_DISP_DIR_0)
    {
        width = width ^ height;
        height = width ^ height;
        width = width ^ height;
        sx = sx ^ sy;
        sy = sx ^ sy;
        sx = sx ^ sy;
        sy = g_atk_md0700_v3_sta.width - sy - height;
    }
    atk_md0700_v3_set_reg(0x56, sx);
    atk_md0700_v3_set_reg(0x57, sx >> 8);
    atk_md0700_v3_set_reg(0x58, sy);
    atk_md0700_v3_set_reg(0x59, sy >> 8);
    atk_md0700_v3_set_reg(0x5A, width);
    atk_md0700_v3_set_reg(0x5B, width >> 8);
    atk_md0700_v3_set_reg(0x5C, height);
    atk_md0700_v3_set_reg(0x5D, height >> 8);
    atk_md0700_v3_config_memory_access_coordinate(sx, sy);
}

/**
 * @brief       平方函数，x^y
 * @param       x: 底数
 *              y: 指数
 * @retval      x^y
 */
static uint32_t atk_md0700_v3_pow(uint8_t x, uint8_t y)
{
    uint8_t loop;
    uint32_t res = 1;
    
    for (loop=0; loop<y; loop++)
    {
        res *= x;
    }
    
    return res;
}

/**
 * @brief       ATK-MD0700 V3模块初始化
 * @param       无
 * @retval      ATK_MD0700_V3_EOK  : ATK_MD0700_V3模块初始化成功
 *              ATK_MD0700_V3_ERROR: ATK_MD0700_V3模块初始化失败
 */
uint8_t atk_md0700_v3_init(void)
{
    uint8_t id;
    uint8_t ret;
    
    atk_md0700_v3_fsmc_init();                  /* ATK-MD0700 V3模块FSMC接口初始化 */
    id = atk_md0700_v3_get_id();                /* 获取ATK-MD0700 V3模块ID */
    ret = atk_md0700_v3_setup_param_by_id(id);  /* ATK-MD0700 V3模块参数初始化 */
    if (ret != ATK_MD0700_V3_EOK)
    {
        return ATK_MD0700_V3_ERROR;
    }
    atk_md0700_v3_setup_lt7381_by_param(g_atk_md0700_v3_sta.param);
    
    atk_md0700_v3_set_disp_dir(ATK_MD0700_V3_LCD_DISP_DIR_90);
    atk_md0700_v3_clear(ATK_MD0700_V3_WHITE);
    atk_md0700_v3_backlight_config(100);
    atk_md0700_v3_display_on(); 
    return ATK_MD0700_V3_EOK;
}

/**
 * @brief       获取ATK-MD0700 V3模块LCD宽度
 * @param       无
 * @retval      ATK-MD0700 V3模块LCD宽度
 */
uint16_t atk_md0700_v3_get_lcd_width(void)
{
    return g_atk_md0700_v3_sta.width;
}

/**
 * @brief       获取ATK-MD0700 V3模块LCD高度
 * @param       无
 * @retval      ATK-MD0700 V3模块LCD高度
 */
uint16_t atk_md0700_v3_get_lcd_height(void)
{
    return g_atk_md0700_v3_sta.height;
}

/**
 * @brief       设置ATK-MD0700 V3模块LCD背光亮度
 * @param       percent: LCD背光亮度百分比
 * @retval      无
 */
void atk_md0700_v3_backlight_config(uint8_t percent)
{
    atk_md0700_v3_set_reg(0x8C, percent);               /* LT7381 Timer1计数比较值 */
    atk_md0700_v3_set_reg(0x8D, (percent >> 8));
    if (percent == 0)
    {
        atk_md0700_v3_modify_reg(0x85, (3 << 2), 0);    /* 关闭LT7381 PWM[1]输出 */
    }
    else
    {
        atk_md0700_v3_modify_reg(0x85, 0, (2 << 2));    /* 开启LT7381 PWM[1]输出 */
    }
}

/**
 * @brief       开启ATK-MD0700 V3模块LCD显示
 * @param       无
 * @retval      无
 */
void atk_md0700_v3_display_on(void)
{
    atk_md0700_v3_modify_reg(0x12, 0, (1 << 6));
}

/**
 * @brief       关闭ATK-MD0700 V3模块LCD显示
 * @param       无
 * @retval      无
 */
void atk_md0700_v3_display_off(void)
{
    atk_md0700_v3_modify_reg(0x12, (1 << 6), 0);
}

/**
 * @brief       设置ATK-MD0700 V3模块LCD扫描方向
 * @note        更改扫描方向会影响显存的写入速度，
 *              当扫描方向为ATK_MD0700_V3_LCD_SCAN_DIR_U2D_R2L@ATK_MD0700_V3_LCD_DISP_DIR_0或ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D@ATK_MD0700_V3_LCD_DISP_DIR_90时，
 *              FSMC/FMC的WR信号最大允许20MHz
 *              当扫描方向为其他扫描方向时，
 *              FSMC/FMC的WR信号最大允许10MHz
 *              若配置FSMC/FMC的写入速度大于上述情况时，请在连续写入显存时，检查状态寄存器(atk_md0700_v3_read_stsr())的bit6为1后再写入数据
 * @param       scan_dir: 扫描方向
 *                  disp_dir为ATK_MD0700_V3_LCD_DISP_DIR_0时，支持
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D: 从左到右，从上到下
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D: 从右到左，从上到下
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_U2D_R2L: 从上到下，从右到左
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_D2U_R2L: 从下到上，从右到左
 *                  disp_dir为ATK_MD0700_V3_LCD_DISP_DIR_90时，支持
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D: 从左到右，从上到下
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D: 从右到左，从上到下
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_U2D_L2R: 从上到下，从左到右
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_D2U_L2R: 从下到上，从左到右
 * @retval      ATK_MD0700_V3_EOK   : 设置ATK-MD0700 V3模块LCD扫描方向成功
 *              ATK_MD0700_V3_EINVAL: 传入参数错误
 */
uint8_t atk_md0700_v3_set_scan_dir(uint8_t scan_dir)
{
    if (g_atk_md0700_v3_sta.disp_dir == ATK_MD0700_V3_LCD_DISP_DIR_0)
    {
        switch (scan_dir)
        {
            case ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D:
            {
                scan_dir = ATK_MD0700_V3_LCD_SCAN_DIR_D2U_L2R;
                break;
            }
            case ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D:
            {
                scan_dir = ATK_MD0700_V3_LCD_SCAN_DIR_U2D_L2R;
                break;
            }
            case ATK_MD0700_V3_LCD_SCAN_DIR_U2D_R2L:
            {
                scan_dir = ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D;
                break;
            }
            case ATK_MD0700_V3_LCD_SCAN_DIR_D2U_R2L:
            {
                scan_dir = ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D;
                break;
            }
            default:
            {
                return ATK_MD0700_V3_EINVAL;
            }
        }
    }
    else
    {
        switch (scan_dir)
        {
            case ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D:
            case ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D:
            case ATK_MD0700_V3_LCD_SCAN_DIR_U2D_L2R:
            case ATK_MD0700_V3_LCD_SCAN_DIR_D2U_L2R:
            {
                break;
            }
            default:
            {
                return ATK_MD0700_V3_EINVAL;
            }
        }
    }
    
    switch (scan_dir)
    {
        case ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D:
        {
            atk_md0700_v3_modify_reg(0x02, (3 << 4) | (3 << 1), (0 << 4) | (0 << 1));
            break;
        }
        case ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D:
        {
            atk_md0700_v3_modify_reg(0x02, (3 << 4) | (3 << 1), (1 << 4) | (1 << 1));
            break;
        }
        case ATK_MD0700_V3_LCD_SCAN_DIR_U2D_L2R:
        {
            atk_md0700_v3_modify_reg(0x02, (3 << 4) | (3 << 1), (2 << 4) | (2 << 1));
            break;
        }
        case ATK_MD0700_V3_LCD_SCAN_DIR_D2U_L2R:
        {
            atk_md0700_v3_modify_reg(0x02, (3 << 4) | (3 << 1), (3 << 4) | (3 << 1));
            break;
        }
        default:
        {
            return ATK_MD0700_V3_EINVAL;
        }
    }
    
    g_atk_md0700_v3_sta.scan_dir = scan_dir;
    
    return ATK_MD0700_V3_EOK;
}

/**
 * @brief       设置ATK-MD0700 V3模块LCD显示方向
 * @param       disp_dir: ATK_MD0700_V3_LCD_DISP_DIR_0  : LCD顺时针旋转0°显示内容
 *                        ATK_MD0700_V3_LCD_DISP_DIR_90 : LCD顺时针旋转90°显示内容
 * @retval      ATK_MD0700_V3_EOK   : 设置ATK-MD0700 V3模块LCD显示方向成功
 *              ATK_MD0700_V3_EINVAL: 传入参数错误
 */
uint8_t atk_md0700_v3_set_disp_dir(uint8_t disp_dir)
{
    switch (disp_dir)
    {
        case ATK_MD0700_V3_LCD_DISP_DIR_0:
        {
            g_atk_md0700_v3_sta.width = g_atk_md0700_v3_sta.param->height;
            g_atk_md0700_v3_sta.height = g_atk_md0700_v3_sta.param->width;
            break;
        }
        case ATK_MD0700_V3_LCD_DISP_DIR_90:
        {
            g_atk_md0700_v3_sta.width = g_atk_md0700_v3_sta.param->width;
            g_atk_md0700_v3_sta.height = g_atk_md0700_v3_sta.param->height;
            break;
        }
        default:
        {
            return ATK_MD0700_V3_EINVAL;
        }
    }
    
    g_atk_md0700_v3_sta.disp_dir = disp_dir;
    atk_md0700_v3_set_scan_dir(ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D);
    
    return ATK_MD0700_V3_EOK;
}

/**
 * @brief       获取ATK-MD0700 V3模块LCD扫描方向
 * @param       无
 * @retval      ATK-MD0700 V3模块LCD扫描方向
 */
uint8_t atk_md0700_v3_get_scan_dir(void)
{
    return g_atk_md0700_v3_sta.scan_dir;
}

/**
 * @brief       获取ATK-MD0700 V3模块LCD显示方向
 * @param       无
 * @retval      ATK-MD0700 V3模块LCD显示方向
 */
uint8_t atk_md0700_v3_get_disp_dir(void)
{
    return g_atk_md0700_v3_sta.disp_dir;
}

/**
 * @brief       ATK-MD0700 V3模块LCD区域填充
 * @param       xs   : 区域起始X坐标
 *              ys   : 区域起始Y坐标
 *              xe   : 区域终止X坐标
 *              ye   : 区域终止Y坐标
 *              color: 区域填充颜色
 * @retval      无
 */
void atk_md0700_v3_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color)
{
    uint16_t window_width;
    uint16_t window_height;
    uint32_t window_size;
    uint32_t window_index;
    
    window_width = ex - sx + 1;
    window_height = ey - sy + 1;
    window_size = window_width * window_height;
    atk_md0700_v3_config_memory_access_window(sx, sy, window_width, window_height);
    atk_md0700_v3_start_access_memory();
    for (window_index=0; window_index<window_size; window_index++)
    {
        /* 连续写入时，检查显存写入FiFo，详细请见atk_md0700_v3_set_scan_dir()的描述 */
        if ((window_index % 8) == 0)
        {
            while ((atk_md0700_v3_read_stsr() & (1 << 7)) != 0);
            atk_md0700_v3_start_access_memory();
        }
        
        ATK_MD0700_V3_FSMC_DAT = color;
    }
    atk_md0700_v3_config_memory_access_window(0, 0, g_atk_md0700_v3_sta.width, g_atk_md0700_v3_sta.height);
}

/**
 * @brief       ATK-MD0700 V3模块LCD清屏
 * @param       color: 清屏颜色
 * @retval      无
 */
void atk_md0700_v3_clear(uint16_t color)
{
    atk_md0700_v3_fill(0, 0, g_atk_md0700_v3_sta.width - 1, g_atk_md0700_v3_sta.height - 1, color);
}

/**
 * @brief       ATK-MD0700 V3模块LCD画点
 * @param       x    : 待画点的X坐标
 *              y    : 待画点的Y坐标
 *              color: 待画点的颜色
 * @retval      无
 */
void atk_md0700_v3_draw_point(uint16_t x, uint16_t y, volatile uint16_t color)
{
    atk_md0700_v3_config_memory_access_coordinate(x, y);
    atk_md0700_v3_start_access_memory();
    ATK_MD0700_V3_FSMC_DAT = color;
}

/**
 * @brief       ATK-MD0700 V3模块LCD读点
 * @param       x    : 待读点的X坐标
 *              y    : 待读点的Y坐标
 * @retval      待读点的颜色
 */
uint16_t atk_md0700_v3_read_point(uint16_t x, uint16_t y)
{
    volatile uint16_t color;
    
    if ((x >= g_atk_md0700_v3_sta.width) || (y >= g_atk_md0700_v3_sta.height))
    {
        return ATK_MD0700_V3_EINVAL;
    }
    
    atk_md0700_v3_config_memory_access_coordinate(x, y);
    atk_md0700_v3_start_access_memory();
    
    color = ATK_MD0700_V3_FSMC_DAT;
    delay_us(2);
    color = ATK_MD0700_V3_FSMC_DAT;
    
    return color;
}

/**
 * @brief       ATK-MD0700 V3模块LCD画线段
 * @param       x1   : 待画线段端点1的X坐标
 *              y1   : 待画线段端点1的Y坐标
 *              x2   : 待画线段端点2的X坐标
 *              y2   : 待画线段端点2的Y坐标
 *              color: 待画线段的颜色
 * @retval      无
 */
void atk_md0700_v3_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t x_delta;
    uint16_t y_delta;
    int16_t x_sign;
    int16_t y_sign;
    int16_t error;
    int16_t error2;
    
    x_delta = (x1 < x2) ? (x2 - x1) : (x1 - x2);
    y_delta = (y1 < y2) ? (y2 - y1) : (y1 - y2);
    x_sign = (x1 < x2) ? 1 : -1;
    y_sign = (y1 < y2) ? 1 : -1;
    error = x_delta - y_delta;
    
    atk_md0700_v3_draw_point(x2, y2, color);
    
    while ((x1 != x2) || (y1 != y2))
    {
        atk_md0700_v3_draw_point(x1, y1, color);
        
        error2 = error << 1;
        if (error2 > -y_delta)
        {
            error -= y_delta;
            x1 += x_sign;
        }
        
        if (error2 < x_delta)
        {
            error += x_delta;
            y1 += y_sign;
        }
    }
}

/**
 * @brief       ATK-MD0700 V3模块LCD画矩形框
 * @param       x1   : 待画矩形框端点1的X坐标
 *              y1   : 待画矩形框端点1的Y坐标
 *              x2   : 待画矩形框端点2的X坐标
 *              y2   : 待画矩形框端点2的Y坐标
 *              color: 待画矩形框的颜色
 * @retval      无
 */
void atk_md0700_v3_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    atk_md0700_v3_draw_line(x1, y1, x2, y1, color);
    atk_md0700_v3_draw_line(x1, y2, x2, y2, color);
    atk_md0700_v3_draw_line(x1, y1, x1, y2, color);
    atk_md0700_v3_draw_line(x2, y1, x2, y2, color);
}

/**
 * @brief       ATK-MD0700 V3模块LCD画圆形框
 * @param       x    : 待画圆形框原点的X坐标
 *              y    : 待画圆形框原点的Y坐标
 *              r    : 待画圆形框的半径
 *              color: 待画圆形框的颜色
 * @retval      无
 */
void atk_md0700_v3_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
    int32_t x_t;
    int32_t y_t;
    int32_t error;
    int32_t error2;
    
    x_t = -r;
    y_t = 0;
    error = 2 - 2 * r;
    
    do {
        atk_md0700_v3_draw_point(x - x_t, y + y_t, color);
        atk_md0700_v3_draw_point(x + x_t, y + y_t, color);
        atk_md0700_v3_draw_point(x + x_t, y - y_t, color);
        atk_md0700_v3_draw_point(x - x_t, y - y_t, color);
        
        error2 = error;
        if (error2 <= y_t)
        {
            y_t++;
            error = error + (y_t * 2 + 1);
            if ((-x_t == y_t) && (error2 <= x_t))
            {
                error2 = 0;
            }
        }
        
        if (error2 > x_t)
        {
            x_t++;
            error = error + (x_t * 2 + 1);
        }
    } while (x_t <= 0);
}

/**
 * @brief       ATK-MD0700 V3模块LCD画圆形
 * @param       x    : 待画圆形原点的X坐标
 *              y    : 待画圆形原点的Y坐标
 *              r    : 待画圆形的半径
 *              color: 待画圆形的颜色
 * @retval      无
 */
void atk_md0700_v3_fill_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
    uint32_t i;
    uint32_t imax = ((uint32_t)r * 707) / 1000 + 1;
    uint32_t sqmax = (uint32_t)r * (uint32_t)r + (uint32_t)r / 2;
    uint32_t xr = r;
    
    atk_md0700_v3_draw_line(x - r, y, x + r, y, color);

    for (i = 1; i <= imax; i++)
    {
        if ((i * i + xr * xr) > sqmax)
        {
            /* draw lines from outside */
            if (xr > imax)
            {
                atk_md0700_v3_draw_line(x - i + 1, y + xr, x + i - 1, y + xr, color);
                atk_md0700_v3_draw_line(x - i + 1, y - xr, x + i - 1, y - xr, color);
            }

            xr--;
        }

        /* draw lines from inside (center) */
        atk_md0700_v3_draw_line(x - xr, y + i, x + xr, y + i, color);
        atk_md0700_v3_draw_line(x - xr, y - i, x + xr, y - i, color);
    }
}

/**
 * @brief       ATK-MD0700 V3模块LCD显示1个字符
 * @param       x    : 待显示字符的X坐标
 *              y    : 待显示字符的Y坐标
 *              ch   : 待显示字符
 *              font : 待显示字符的字体
 *              color: 待显示字符的颜色
 * @retval      无
 */
void atk_md0700_v3_show_char(uint16_t x, uint16_t y, char ch, uint8_t font, uint16_t color)
{
    const uint8_t *ch_code;
    uint8_t ch_width;
    uint8_t ch_height;
    uint8_t ch_size;
    uint8_t ch_offset;
    uint8_t byte_index;
    uint8_t byte_code;
    uint8_t bit_index;
    uint8_t width_index = 0;
    uint8_t height_index = 0;
    
    ch_offset = ch - ' ';
    
    switch (font)
    {
#if (ATK_MD0700_V3_FONT_12 != 0)
        case ATK_MD0700_V3_LCD_FONT_12:
        {
            ch_code = atk_md0700_v3_font_1206[ch_offset];
            ch_width = ATK_MD0700_V3_FONT_12_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_12_CHAR_HEIGHT;
            ch_size = ATK_MD0700_V3_FONT_12_CHAR_SIZE;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_16 != 0)
        case ATK_MD0700_V3_LCD_FONT_16:
        {
            ch_code = atk_md0700_v3_font_1608[ch_offset];
            ch_width = ATK_MD0700_V3_FONT_16_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_16_CHAR_HEIGHT;
            ch_size = ATK_MD0700_V3_FONT_16_CHAR_SIZE;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_24 != 0)
        case ATK_MD0700_V3_LCD_FONT_24:
        {
            ch_code = atk_md0700_v3_font_2412[ch_offset];
            ch_width = ATK_MD0700_V3_FONT_24_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_24_CHAR_HEIGHT;
            ch_size = ATK_MD0700_V3_FONT_24_CHAR_SIZE;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_32 != 0)
        case ATK_MD0700_V3_LCD_FONT_32:
        {
            ch_code = atk_md0700_v3_font_3216[ch_offset];
            ch_width = ATK_MD0700_V3_FONT_32_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_32_CHAR_HEIGHT;
            ch_size = ATK_MD0700_V3_FONT_32_CHAR_SIZE;
            break;
        }
#endif
        default:
        {
            return;
        }
    }
    
    if ((x + ch_width > g_atk_md0700_v3_sta.width) || (y + ch_height > g_atk_md0700_v3_sta.height))
    {
        return;
    }
    
    for (byte_index=0; byte_index<ch_size; byte_index++)
    {
        byte_code = ch_code[byte_index];
        for (bit_index=0; bit_index<8; bit_index++)
        {
            if ((byte_code & 0x80) != 0)
            {
                atk_md0700_v3_draw_point(x + width_index, y + height_index, color);
            }
            height_index++;
            if (height_index == ch_height)
            {
                height_index = 0;
                width_index++;
                break;
            }
            byte_code <<= 1;
        }
    }
}

/**
 * @brief       ATK-MD0700 V3模块LCD显示字符串
 * @note        会自动换行和换页
 * @param       x     : 待显示字符串的X坐标
 *              y     : 待显示字符串的Y坐标
 *              width : 待显示字符串的显示高度
 *              height: 待显示字符串的显示宽度
 *              str   : 待显示字符串
 *              font  : 待显示字符串的字体
 *              color : 待显示字符串的颜色
 * @retval      无
 */
void atk_md0700_v3_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, char *str, uint8_t font, uint16_t color)
{
    uint8_t ch_width;
    uint8_t ch_height;
    uint16_t x_raw;
    uint16_t y_raw;
    uint16_t x_limit;
    uint16_t y_limit;
    
    switch (font)
    {
#if (ATK_MD0700_V3_FONT_12 != 0)
        case ATK_MD0700_V3_LCD_FONT_12:
        {
            ch_width = ATK_MD0700_V3_FONT_12_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_12_CHAR_HEIGHT;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_16 != 0)
        case ATK_MD0700_V3_LCD_FONT_16:
        {
            ch_width = ATK_MD0700_V3_FONT_16_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_16_CHAR_HEIGHT;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_24 != 0)
        case ATK_MD0700_V3_LCD_FONT_24:
        {
            ch_width = ATK_MD0700_V3_FONT_24_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_24_CHAR_HEIGHT;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_32 != 0)
        case ATK_MD0700_V3_LCD_FONT_32:
        {
            ch_width = ATK_MD0700_V3_FONT_32_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_32_CHAR_HEIGHT;
            break;
        }
#endif
        default:
        {
            return;
        }
    }
    
    x_raw = x;
    y_raw = y;
    x_limit = ((x + width + 1) > g_atk_md0700_v3_sta.width) ? g_atk_md0700_v3_sta.width : (x + width + 1);
    y_limit = ((y + height + 1) > g_atk_md0700_v3_sta.height) ? g_atk_md0700_v3_sta.height : (y + height + 1);
    
    while ((*str >= ' ') && (*str <= '~'))
    {
        if (x + ch_width > x_limit)
        {
            x = x_raw;
            y += ch_height;
        }
        
        if (y + ch_height > y_limit)
        {
            y = x_raw;
            x = y_raw;
        }
        
        atk_md0700_v3_show_char(x, y, *str, font, color);
        
        x += ch_width;
        str++;
    }
}

/**
 * @brief       ATK-MD0700 V3模块LCD显示数字，可控制显示高位0
 * @param       x    : 待显示数字的X坐标
 *              y    : 待显示数字的Y坐标
 *              num  : 待显示数字
 *              len  : 待显示数字的位数
 *              mode : ATK_MD0700_NUM_SHOW_NOZERO: 数字高位0不显示
 *                     ATK_MD0700_NUM_SHOW_ZERO  : 数字高位0显示
 *              font : 待显示数字的字体
 *              color: 待显示数字的颜色
 * @retval      无
 */
void atk_md0700_v3_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t mode, uint8_t font, uint16_t color)
{
    uint8_t ch_width;
    uint8_t len_index;
    uint8_t num_index;
    uint8_t first_nozero = 0;
    char pad;
    
    switch (font)
    {
#if (ATK_MD0700_V3_FONT_12 != 0)
        case ATK_MD0700_V3_LCD_FONT_12:
        {
            ch_width = ATK_MD0700_V3_FONT_12_CHAR_WIDTH;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_16 != 0)
        case ATK_MD0700_V3_LCD_FONT_16:
        {
            ch_width = ATK_MD0700_V3_FONT_16_CHAR_WIDTH;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_24 != 0)
        case ATK_MD0700_V3_LCD_FONT_24:
        {
            ch_width = ATK_MD0700_V3_FONT_24_CHAR_WIDTH;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_32 != 0)
        case ATK_MD0700_V3_LCD_FONT_32:
        {
            ch_width = ATK_MD0700_V3_FONT_32_CHAR_WIDTH;
            break;
        }
#endif
        default:
        {
            return;
        }
    }
    
    switch (mode)
    {
        case ATK_MD0700_V3_NUM_SHOW_NOZERO:
        {
            pad = ' ';
            break;
        }
        case ATK_MD0700_V3_NUM_SHOW_ZERO:
        {
            pad = '0';
            break;
        }
        default:
        {
            return;
        }
    }
    
    for (len_index=0; len_index<len; len_index++)
    {
        num_index = (num / atk_md0700_v3_pow(10, len - len_index - 1)) % 10;
        if ((first_nozero == 0) && (len_index < (len - 1)))
        {
            if (num_index == 0)
            {
                atk_md0700_v3_show_char(x + ch_width * len_index, y, pad, font, color);
                continue;
            }
            else
            {
                first_nozero = 1;
            }
        }
        
        atk_md0700_v3_show_char(x + ch_width * len_index, y, num_index + '0', font, color);
    }
}

/**
 * @brief       ATK-MD0700 V3模块LCD显示数字，不显示高位0
 * @param       x    : 待显示数字的X坐标
 *              y    : 待显示数字的Y坐标
 *              num  : 待显示数字
 *              len  : 待显示数字的位数
 *              font : 待显示数字的字体
 *              color: 待显示数字的颜色
 * @retval      无
 */
void atk_md0700_v3_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t font, uint16_t color)
{
    atk_md0700_v3_show_xnum(x, y, num, len, ATK_MD0700_V3_NUM_SHOW_NOZERO, font, color);
}

/**
 * @brief       ATK-MD0700 V3模块LCD图片
 * @note        图片取模方式: 水平扫描、RGB565、高位在前
 * @param       x     : 待显示图片的X坐标
 *              y     : 待显示图片的Y坐标
 *              width : 待显示图片的宽度
 *              height: 待显示图片的高度
 *              pic   : 待显示图片数组首地址
 * @retval      无
 */
void atk_md0700_v3_show_pic(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *pic)
{
    uint32_t window_size;
    uint32_t window_index;
    
    window_size = width * height;
    atk_md0700_v3_config_memory_access_window(x, y, width, height);
    atk_md0700_v3_start_access_memory();
    for (window_index=0; window_index<window_size; window_index++, pic++)
    {
        /* 连续写入时，检查显存写入FiFo，详细请见atk_md0700_v3_set_scan_dir()的描述 */
        if ((window_index % 8) == 0)
        {
            while ((atk_md0700_v3_read_stsr() & (1 << 7)) != 0);
            atk_md0700_v3_start_access_memory();
        }
        
        ATK_MD0700_V3_FSMC_DAT = *pic;
    }
    atk_md0700_v3_config_memory_access_window(0, 0, g_atk_md0700_v3_sta.width, g_atk_md0700_v3_sta.height);
}
