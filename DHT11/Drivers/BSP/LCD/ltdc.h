/**
 ****************************************************************************************************
 * @file        ltdc.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-09-06
 * @brief       LTDC 驱动代码
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

#ifndef _LTDC_H
#define _LTDC_H

#include "./SYSTEM/sys/sys.h"


/* LCD LTDC重要参数集 */
typedef struct  
{
    uint32_t pwidth;      /* LTDC面板的宽度,固定参数,不随显示方向改变,如果为0,说明没有任何RGB屏接入 */
    uint32_t pheight;     /* LTDC面板的高度,固定参数,不随显示方向改变 */
    uint16_t hsw;         /* 水平同步宽度 */
    uint16_t vsw;         /* 垂直同步宽度 */
    uint16_t hbp;         /* 水平后廊 */
    uint16_t vbp;         /* 垂直后廊 */
    uint16_t hfp;         /* 水平前廊 */
    uint16_t vfp;         /* 垂直前廊  */
    uint8_t activelayer;  /* 当前层编号:0/1 */
    uint8_t dir;          /* 0,竖屏;1,横屏; */
    uint16_t width;       /* LTDC宽度 */
    uint16_t height;      /* LTDC高度 */
    uint32_t pixsize;     /* 每个像素所占字节数 */
}_ltdc_dev; 

extern _ltdc_dev lcdltdc;                   /* 管理LCD LTDC参数 */
extern LTDC_HandleTypeDef g_ltdc_handle;    /* LTDC句柄 */
extern DMA2D_HandleTypeDef g_dma2d_handle;  /* DMA2D句柄 */

/******************************************************************************************/
/* LTDC_BL/DE/VSYNC/HSYNC/CLK 引脚 定义 
 * LTDC_R3~R7, G2~G7, B3~B7,由于引脚太多,就不在这里定义了,直接在ltcd_init里面修改.所以在移植的时候,
 * 除了改这5个IO口, 还得改ltcd_init里面的R3~R7, G2~G7, B3~B7所在的IO口.
 */

/* BL和MCU屏背光脚共用 所以这里不用定义 BL引脚 */
#define LTDC_BL_GPIO_PORT               GPIOB
#define LTDC_BL_GPIO_PIN                GPIO_PIN_5
#define LTDC_BL_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* 所在IO口时钟使能 */

#define LTDC_DE_GPIO_PORT               GPIOF
#define LTDC_DE_GPIO_PIN                GPIO_PIN_10
#define LTDC_DE_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)    /* 所在IO口时钟使能 */

#define LTDC_VSYNC_GPIO_PORT            GPIOI
#define LTDC_VSYNC_GPIO_PIN             GPIO_PIN_9
#define LTDC_VSYNC_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOI_CLK_ENABLE(); }while(0)    /* 所在IO口时钟使能 */

#define LTDC_HSYNC_GPIO_PORT            GPIOI
#define LTDC_HSYNC_GPIO_PIN             GPIO_PIN_10
#define LTDC_HSYNC_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOI_CLK_ENABLE(); }while(0)    /* 所在IO口时钟使能 */

#define LTDC_CLK_GPIO_PORT              GPIOG
#define LTDC_CLK_GPIO_PIN               GPIO_PIN_7
#define LTDC_CLK_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)    /* 所在IO口时钟使能 */


/* 定义颜色像素格式,一般用RGB565 */
#define LTDC_PIXFORMAT           LTDC_PIXEL_FORMAT_RGB565

/* 定义默认背景层颜色 */
#define LTDC_BACKLAYERCOLOR      0X00000000

/* LTDC帧缓冲区首地址,这里定义在SDRAM里面. */
#define LTDC_FRAME_BUF_ADDR      0XC0000000  

/* LTDC背光控制 */
#define LTDC_BL(x)   do{ x ? \
                      HAL_GPIO_WritePin(LTDC_BL_GPIO_PORT, LTDC_BL_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LTDC_BL_GPIO_PORT, LTDC_BL_GPIO_PIN, GPIO_PIN_RESET); \
                     }while(0)

/******************************************************************************************/

void ltdc_switch(uint8_t sw);                                                                                   /* LTDC开关 */
void ltdc_layer_switch(uint8_t layerx, uint8_t sw);                                                             /* 层开关 */
void ltdc_select_layer(uint8_t layerx);                                                                         /* 层选择 */
void ltdc_display_dir(uint8_t dir);                                                                             /* 显示方向控制 */
void ltdc_draw_point(uint16_t x, uint16_t y, uint32_t color);                                                   /* 画点函数 */
uint32_t ltdc_read_point(uint16_t x, uint16_t y);                                                               /* 读点函数 */
void ltdc_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color);                             /* 矩形单色填充函数 */
void ltdc_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color);                      /* 矩形彩色填充函数 */
void ltdc_clear(uint32_t color);                                                                                /* 清屏函数 */
uint8_t ltdc_clk_set(uint32_t pll3n, uint32_t pll3m, uint32_t pll3r);                                           /* LTDC时钟配置 */
void ltdc_layer_window_config(uint8_t layerx, uint16_t sx, uint16_t sy, uint16_t width, uint16_t height);       /* LTDC层窗口设置 */
void ltdc_layer_parameter_config(uint8_t layerx, uint32_t bufaddr, uint8_t pixformat, uint8_t alpha, uint8_t alpha0, uint8_t bfac1, uint8_t bfac2, uint32_t bkcolor); /* LTDC基本参数设置 */
uint16_t ltdc_panelid_read(void);                                                                               /* LTDC ID读取函数 */
void ltdc_init(void);                                                                                           /* LTDC初始化函数 */

#endif 
