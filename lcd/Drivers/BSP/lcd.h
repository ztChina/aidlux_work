/**
 ****************************************************************************************************
 * @file        lcd.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MD0700 V3模块LCD兼容驱动代码
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

#ifndef __LCD_H
#define __LCD_H

#include "atk_md0700_v3.h"
#include "atk_md0700_v3_fsmc.h"

typedef struct
{
    volatile uint16_t LCD_REG;
    volatile uint16_t LCD_RAM;
} LCD_TypeDef;

#define LCD_BASE    (uint32_t)(ATK_MD0700_V3_FSMC_REG_ADDR)
#define LCD         ((LCD_TypeDef *)LCD_BASE)

#define WHITE       ATK_MD0700_V3_WHITE
#define BLACK       ATK_MD0700_V3_BLACK
#define BLUE        ATK_MD0700_V3_BLUE
#define BRED        ATK_MD0700_V3_BRED
#define GRED        ATK_MD0700_V3_GRED
#define GBLUE       ATK_MD0700_V3_GBLUE
#define RED         ATK_MD0700_V3_RED
#define MAGENTA     ATK_MD0700_V3_MAGENTA
#define GREEN       ATK_MD0700_V3_GREEN
#define CYAN        ATK_MD0700_V3_CYAN
#define YELLOW      ATK_MD0700_V3_YELLOW
#define BROWN       ATK_MD0700_V3_BROWN
#define BRRED       ATK_MD0700_V3_BRRED
#define GRAY        0x8430
#define DARKBLUE    0x01CF
#define LIGHTBLUE   0x7D7C
#define GRAYBLUE    0x5458
#define LIGHTGREEN  0x841F
#define LGRAY       0xC618
#define LGRAYBLUE   0xA651
#define LBBLUE      0x2B12

typedef struct
{
    uint16_t width;
    uint16_t height;
    uint16_t id;
    uint8_t dir;
} _lcd_dev;

extern _lcd_dev lcddev;

extern uint32_t g_point_color;
extern uint32_t g_back_color;

/* 函数声明 */
void lcd_init(void);                                                                                                    /* 初始化LCD */
void lcd_display_on(void);                                                                                              /* 开显示 */
void lcd_display_off(void);                                                                                             /* 关显示 */
void lcd_scan_dir(uint8_t dir);                                                                                         /* 设置屏幕扫描方向 */
void lcd_display_dir(uint8_t dir);                                                                                      /* 设置屏幕显示方向 */
void lcd_write_ram_prepare(void);                                                                                       /* 准备写GRAM */
void lcd_set_cursor(uint16_t x, uint16_t y);                                                                            /* 设置光标 */
uint32_t lcd_read_point(uint16_t x, uint16_t y);                                                                        /* 读点 */
void lcd_draw_point(uint16_t x, uint16_t y, uint32_t color);                                                            /* 画点 */
void lcd_clear(uint16_t color);                                                                                         /* LCD清屏 */
void lcd_fill_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);                                               /* 填充实心圆 */
void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);                                              /* 画圆 */
void lcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t color);                                              /* 画水平线 */
void lcd_set_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height);                                         /* 设置窗口 */
void lcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color);                                      /* 纯色填充矩形 */
void lcd_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color);                               /* 彩色填充矩形 */
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                                 /* 画直线 */
void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                            /* 画矩形 */
void lcd_show_char(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color);                       /* 显示字符 */
void lcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t color);                     /* 显示数字 */
void lcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode, uint16_t color);      /* 显示数字，可控制高位0 */
void lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p, uint16_t color);   /* 显示字符串 */

#endif
