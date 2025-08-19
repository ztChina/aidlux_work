/**
 ****************************************************************************************************
 * @file        atk_md0700_v3_ex.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MD0700 V3模块扩展驱动代码
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

#ifndef __ATK_MD0700_V3_EX_H
#define __ATK_MD0700_V3_EX_H



/* ATK-MD0700 V3模块使能定义 */
#define ATK_MD0700_V3_EX_DISABLE                0   /* 禁止 */
#define ATK_MD0700_V3_EX_ENABLE                 1   /* 使能 */

/* ATK-MD0700 V3模块文本字体定义 */
#define ATK_MD0700_V3_EX_TEXT_SIZE_16           16  /* 字体高度为16 */
#define ATK_MD0700_V3_EX_TEXT_SIZE_24           24  /* 字体高度为24 */
#define ATK_MD0700_V3_EX_TEXT_SIZE_32           32  /* 字体高度为32 */

/* ATK-MD0700 V3模块文本字符集定义 */
#define ATK_MD0700_V3_EX_TEXT_SETS_8859_1       1   /* ISO/IEC 8859-1 */
#define ATK_MD0700_V3_EX_TEXT_SETS_8859_2       2   /* ISO/IEC 8859-2 */
#define ATK_MD0700_V3_EX_TEXT_SETS_8859_4       4   /* ISO/IEC 8859-4 */
#define ATK_MD0700_V3_EX_TEXT_SETS_8859_5       5   /* ISO/IEC 8859-5 */

/* ATK-MD0700 V3模块文本背景颜色模式定义 */
#define ATK_MD0700_V3_EX_TEXT_BG_MODE_CUSTOM    0   /* 指定背景颜色 */
#define ATK_MD0700_V3_EX_TEXT_BG_MODE_ORIGINAL  1   /* 不填充背景颜色 */

/* ATK-MD0700 V3模块文本缩放定义 */
#define ATK_MD0700_V3_EX_TEXT_SCALE_X1          1   /* 放大1倍数（保持不变） */
#define ATK_MD0700_V3_EX_TEXT_SCALE_X2          2   /* 放大2倍 */
#define ATK_MD0700_V3_EX_TEXT_SCALE_X3          3   /* 放大3倍 */
#define ATK_MD0700_V3_EX_TEXT_SCALE_X4          4   /* 放大4倍 */

/* ATK-MD0700 V3模块BTE ROP操作指令定义 */
#define ATK_MD0700_V3_EX_BTE_ROP_0              0   /* 0(Blackness) */
#define ATK_MD0700_V3_EX_BTE_ROP_1              1   /* ~S0*~S1 or (S0+S1) */
#define ATK_MD0700_V3_EX_BTE_ROP_2              2   /* ~S0*S1 */
#define ATK_MD0700_V3_EX_BTE_ROP_3              3   /* ~S0 */
#define ATK_MD0700_V3_EX_BTE_ROP_4              4   /* S0*~S1 */
#define ATK_MD0700_V3_EX_BTE_ROP_5              5   /* ~S1 */
#define ATK_MD0700_V3_EX_BTE_ROP_6              6   /* S0^S1 */
#define ATK_MD0700_V3_EX_BTE_ROP_7              7   /* ~S0+~S1 or ~(S0*S1) */
#define ATK_MD0700_V3_EX_BTE_ROP_8              8   /* S0*S1 */
#define ATK_MD0700_V3_EX_BTE_ROP_9              9   /* ~(S0^S1) */
#define ATK_MD0700_V3_EX_BTE_ROP_10             10  /* S1 */
#define ATK_MD0700_V3_EX_BTE_ROP_11             11  /* ~S0+S1 */
#define ATK_MD0700_V3_EX_BTE_ROP_12             12  /* S0 */
#define ATK_MD0700_V3_EX_BTE_ROP_13             13  /* S0+~S1 */
#define ATK_MD0700_V3_EX_BTE_ROP_14             14  /* S0+S1 */
#define ATK_MD0700_V3_EX_BTE_ROP_15             15  /* 1(Whiteness) */

/* ATK-MD0700 V3模块BTE操作指令定义 */
#define ATK_MD0700_V3_EX_BTE_MODE_0             0   /* MCU Write with ROP */
#define ATK_MD0700_V3_EX_BTE_MODE_1             2   /* Memory Copy (move) with ROP */
#define ATK_MD0700_V3_EX_BTE_MODE_2             4   /* MCU Write with chroma keying (w/o ROP) */
#define ATK_MD0700_V3_EX_BTE_MODE_3             5   /* Memory Copy (move) with chroma keying (w/o ROP) */
#define ATK_MD0700_V3_EX_BTE_MODE_4             6   /* Pattern Fill with ROP */
#define ATK_MD0700_V3_EX_BTE_MODE_5             7   /* Pattern Fill with chroma keying (w/o ROP) */
#define ATK_MD0700_V3_EX_BTE_MODE_6             8   /* MCU Write with Color Expansion (w/o ROP) */
#define ATK_MD0700_V3_EX_BTE_MODE_7             9   /* MCU Write with Color Expansion and chroma keying (w/o ROP) */
#define ATK_MD0700_V3_EX_BTE_MODE_8             10  /* Memory Copy with opacity (w/o ROP) */
#define ATK_MD0700_V3_EX_BTE_MODE_9             11  /* MCU Write with opacity (w/o ROP) */
#define ATK_MD0700_V3_EX_BTE_MODE_10            12  /* Solid Fill (w/o ROP) */
#define ATK_MD0700_V3_EX_BTE_MODE_11            14  /* Memory Copy with Color Expansion (w/o ROP) */
#define ATK_MD0700_V3_EX_BTE_MODE_12            15  /* Memory Copy with Color Expansion and chroma keying (w/o ROP) */

void atk_md0700_v3_ex_draw_ellipse(uint16_t x, uint16_t y, uint16_t r1, uint16_t r2, uint16_t color, uint8_t fill);                                         /* ATK-MD0700 V3模块扩展画椭圆 */
void atk_md0700_v3_ex_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color, uint8_t fill);                                                        /* ATK-MD0700 V3模块扩展画圆 */
void atk_md0700_v3_ex_draw_curve(uint16_t x, uint16_t y, uint16_t r1, uint16_t r2, uint16_t color, uint8_t fill, uint8_t part);                             /* ATK-MD0700 V3模块扩展画曲线 */
void atk_md0700_v3_ex_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint8_t fill);                                     /* ATK-MD0700 V3模块扩展画矩形 */
void atk_md0700_v3_ex_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                                                        /* ATK-MD0700 V3模块扩展画线段 */
void atk_md0700_v3_ex_draw_triangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color, uint8_t fill);            /* ATK-MD0700 V3模块扩展画三角形 */
void atk_md0700_v3_ex_draw_rounded_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t r1, uint16_t r2, uint16_t color, uint8_t fill);   /* ATK-MD0700 V3模块扩展画圆角矩形 */
void atk_md0700_v3_ex_show_text_advanced(uint16_t x,                                                                                                        /* ATK-MD0700 V3模块扩展显示文字（高级） */
                                   uint16_t y,
                                   char *text,
                                   uint8_t size,
                                   uint8_t sets,
                                   uint8_t align,
                                   uint8_t bg_color_mode,
                                   uint16_t bg_color,
                                   uint16_t color,
                                   uint8_t width_scale,
                                   uint8_t height_scale);
void atk_md0700_v3_ex_show_text(uint16_t x, uint16_t y, char *text, uint8_t size, uint16_t color);                                                          /* ATK-MD0700 V3模块扩展显示文字 */
void atk_md0700_v3_ex_canvas_window_config(uint32_t address, uint16_t width);                                                                               /* ATK-MD0700 V3模块扩展配置Canvas Window */
void atk_md0700_v3_ex_main_window_config(uint32_t address, uint16_t width, uint16_t x, uint16_t y);                                                         /* ATK-MD0700 V3模块扩展配置Main Window */
void atk_md0700_v3_ex_active_window_config(uint16_t x, uint16_t y, uint16_t width, uint16_t height);                                                        /* ATK-MD0700 V3模块扩展配置Active Window */
void atk_md0700_v3_ex_s0_window_config(uint32_t address, uint16_t width, uint16_t x, uint16_t y);                                                           /* ATK-MD0700 V3模块扩展配置Source0 Window */
void atk_md0700_v3_ex_s1_window_config(uint32_t address, uint16_t width, uint16_t x, uint16_t y);                                                           /* ATK-MD0700 V3模块扩展配置Source1 Window */
void atk_md0700_v3_ex_d_window_config(uint32_t address, uint16_t width, uint16_t x, uint16_t y);                                                            /* ATK-MD0700 V3模块扩展配置Destination Window */
void atk_md0700_v3_ex_bte_window_config(uint16_t width, uint16_t height);                                                                                   /* ATK-MD0700 V3模块扩展配置BLT Window */
void atk_md0700_v3_ex_bte_enable(uint8_t rop, uint8_t mode);                                                                                                /* ATK-MD0700 V3模块扩展使能BLT */
void atk_md0700_v3_ex_bte_chroma_key_color_config(uint16_t color);                                                                                          /* ATK-MD0700 V3模块扩展配置BLT关键色 */

#endif
