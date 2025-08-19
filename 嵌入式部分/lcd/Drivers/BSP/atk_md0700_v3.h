/**
 ****************************************************************************************************
 * @file        atk_md0700_v3.h
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


#include <stdint.h>

#ifndef __ATK_MD0700_V3_H
#define __ATK_MD0700_V3_H


/* 定义是否使用ATK-MD0700 V3模块触摸 */
#define ATK_MD0700_V3_USING_TOUCH           1

/* 定义ATK-MD0700 V3模块启用的字体 */
#define ATK_MD0700_V3_FONT_12               1
#define ATK_MD0700_V3_FONT_16               1
#define ATK_MD0700_V3_FONT_24               1
#define ATK_MD0700_V3_FONT_32               1

/* 默认启用触摸 */
#ifndef ATK_MD0700_V3_USING_TOUCH
#define ATK_MD0700_V3_USING_TOUCH 1
#endif

/* 默认启用12号字体 */
#if ((ATK_MD0700_V3_FONT_12 == 0) && (ATK_MD0700_V3_FONT_16 == 0) && (ATK_MD0700_V3_FONT_24 == 0) && (ATK_MD0700_V3_FONT_32 == 0))
#undef ATK_MD0700_V3_FONT_12
#defien ATK_MD0700_V3_FONT_12 1
#endif



/* ATK-MD0700 V3模块PID定义 */
#define ATK_MD0700_V3_PID_800480            0x748A  /* ATK-MD0700 V3 800*480 */
#define ATK_MD0700_V3_PID_1024600           0x716A  /* ATK-MD0700 V3 1024*600 */

/* ATK-MD0700 V3模块LCD扫描方向定义 */
#define ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D  0x00    /* 从左到右，从上到下 */
#define ATK_MD0700_V3_LCD_SCAN_DIR_L2R_D2U  0x01    /* 从左到右，从下到上 */
#define ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D  0x02    /* 从右到左，从上到下 */
#define ATK_MD0700_V3_LCD_SCAN_DIR_R2L_D2U  0x03    /* 从右到左，从下到上 */
#define ATK_MD0700_V3_LCD_SCAN_DIR_U2D_L2R  0x04    /* 从上到下，从左到右 */
#define ATK_MD0700_V3_LCD_SCAN_DIR_U2D_R2L  0x05    /* 从上到下，从右到左 */
#define ATK_MD0700_V3_LCD_SCAN_DIR_D2U_L2R  0x06    /* 从下到上，从左到右 */
#define ATK_MD0700_V3_LCD_SCAN_DIR_D2U_R2L  0x07    /* 从下到上，从右到左 */

/* ATK-MD0700 V3模块LCD旋转方向定义 */
#define ATK_MD0700_V3_LCD_DISP_DIR_0        0x00    /* LCD顺时针旋转0°显示内容 */
#define ATK_MD0700_V3_LCD_DISP_DIR_90       0x01    /* LCD顺时针旋转90°显示内容 */

/* ATK-MD0700 V3模块LCD显示字体定义 */
#if (ATK_MD0700_V3_FONT_12 != 0)
#define ATK_MD0700_V3_LCD_FONT_12           12      /* 12号字体 */
#endif
#if (ATK_MD0700_V3_FONT_16 != 0)
#define ATK_MD0700_V3_LCD_FONT_16           16      /* 16号字体 */
#endif
#if (ATK_MD0700_V3_FONT_24 != 0)
#define ATK_MD0700_V3_LCD_FONT_24           24      /* 24号字体 */
#endif
#if (ATK_MD0700_V3_FONT_32 != 0)
#define ATK_MD0700_V3_LCD_FONT_32           32      /* 32号字体 */
#endif

/* ATK-MD0700 V3模块LCD显示数字模式定义 */
#define ATK_MD0700_V3_NUM_SHOW_NOZERO       0x00    /* 数字高位0不显示 */
#define ATK_MD0700_V3_NUM_SHOW_ZERO         0x01    /* 数字高位0显示 */

/* 常用颜色定义（RGB565） */
#define ATK_MD0700_V3_WHITE                 0xFFFF
#define ATK_MD0700_V3_BLACK                 0x0000
#define ATK_MD0700_V3_BLUE                  0x001F
#define ATK_MD0700_V3_BRED                  0XF81F
#define ATK_MD0700_V3_GRED                  0XFFE0
#define ATK_MD0700_V3_GBLUE                 0X07FF
#define ATK_MD0700_V3_RED                   0xF800
#define ATK_MD0700_V3_MAGENTA               0xF81F
#define ATK_MD0700_V3_GREEN                 0x07E0
#define ATK_MD0700_V3_CYAN                  0x7FFF
#define ATK_MD0700_V3_YELLOW                0xFFE0
#define ATK_MD0700_V3_BROWN                 0XBC40
#define ATK_MD0700_V3_BRRED                 0XFC07

/* 错误代码 */
#define ATK_MD0700_V3_EOK                   0       /* 没有错误 */
#define ATK_MD0700_V3_ERROR                 1       /* 错误 */
#define ATK_MD0700_V3_EINVAL                2       /* 非法参数 */

/* 操作函数 */
uint8_t atk_md0700_v3_get_id(void);                                                                                                 /* 获取ATK-MD0700 V3模块ID */
void atk_md0700_v3_start_access_memory(void);                                                                                       /* 开始访问ATK-MD0700 V3模块显存 */
void atk_md0700_v3_config_memory_access_coordinate(uint16_t x, uint16_t y);                                                         /* 配置ATK-MD0700 V3模块显存访问起始坐标 */
void atk_md0700_v3_config_memory_access_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height);                          /* 配置ATK-MD0700 V3模块显存访问窗口 */
uint8_t atk_md0700_v3_init(void);                                                                                                   /* ATK-MD0700 V3模块初始化 */
uint16_t atk_md0700_v3_get_lcd_width(void);                                                                                         /* 获取ATK-MD0700 V3模块LCD宽度 */
uint16_t atk_md0700_v3_get_lcd_height(void);                                                                                        /* 获取ATK-MD0700 V3模块LCD高度 */
void atk_md0700_v3_backlight_config(uint8_t percent);                                                                               /* 设置ATK-MD0700 V3模块LCD背光亮度 */
void atk_md0700_v3_display_on(void);                                                                                                /* 开启ATK-MD0700 V3模块LCD显示 */
void atk_md0700_v3_display_off(void);                                                                                               /* 关闭ATK-MD0700 V3模块LCD显示 */
uint8_t atk_md0700_v3_set_scan_dir(uint8_t scan_dir);                                                                               /* 设置ATK-MD0700 V3模块LCD扫描方向 */
uint8_t atk_md0700_v3_set_disp_dir(uint8_t disp_dir);                                                                               /* 设置ATK-MD0700 V3模块LCD显示方向 */
uint8_t atk_md0700_v3_get_scan_dir(void);                                                                                           /* 获取ATK-MD0700 V3模块LCD扫描方向 */
uint8_t atk_md0700_v3_get_disp_dir(void);                                                                                           /* 获取ATK-MD0700 V3模块LCD显示方向 */
void atk_md0700_v3_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color);                                        /* ATK-MD0700 V3模块LCD区域填充 */
void atk_md0700_v3_clear(uint16_t color);                                                                                           /* ATK-MD0700 V3模块LCD清屏 */
void atk_md0700_v3_draw_point(uint16_t x, uint16_t y, volatile uint16_t color);                                                     /* ATK-MD0700 V3模块LCD画点 */
uint16_t atk_md0700_v3_read_point(uint16_t x, uint16_t y);                                                                          /* ATK-MD0700 V3模块LCD读点 */
void atk_md0700_v3_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                                   /* ATK-MD0700 V3模块LCD画线段 */
void atk_md0700_v3_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                                   /* ATK-MD0700 V3模块LCD画矩形框 */
void atk_md0700_v3_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);                                                 /* ATK-MD0700 V3模块LCD画圆形框 */
void atk_md0700_v3_fill_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);                                                 /* ATK-MD0700 V3模块LCD画圆形 */
void atk_md0700_v3_show_char(uint16_t x, uint16_t y, char ch, uint8_t font, uint16_t color);                                        /* ATK-MD0700 V3模块LCD显示1个字符 */
void atk_md0700_v3_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, char *str, uint8_t font, uint16_t color);   /* ATK-MD0700 V3模块LCD显示字符串 */
void atk_md0700_v3_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t mode, uint8_t font, uint16_t color);        /* ATK-MD0700 V3模块LCD显示数字，可控制显示高位0 */
void atk_md0700_v3_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t font, uint16_t color);                       /* ATK-MD0700 V3模块LCD显示数字，不显示高位0 */
void atk_md0700_v3_show_pic(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *pic);                                /* ATK-MD0700 V3模块LCD图片 */

#endif
