/**
 ****************************************************************************************************
 * @file        atk_md0700_v3_ex.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MD0700 V3ģ����չ��������
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
#include <stdint.h>

#ifndef __ATK_MD0700_V3_EX_H
#define __ATK_MD0700_V3_EX_H



/* ATK-MD0700 V3ģ��ʹ�ܶ��� */
#define ATK_MD0700_V3_EX_DISABLE                0   /* ��ֹ */
#define ATK_MD0700_V3_EX_ENABLE                 1   /* ʹ�� */

/* ATK-MD0700 V3ģ���ı����嶨�� */
#define ATK_MD0700_V3_EX_TEXT_SIZE_16           16  /* ����߶�Ϊ16 */
#define ATK_MD0700_V3_EX_TEXT_SIZE_24           24  /* ����߶�Ϊ24 */
#define ATK_MD0700_V3_EX_TEXT_SIZE_32           32  /* ����߶�Ϊ32 */

/* ATK-MD0700 V3ģ���ı��ַ������� */
#define ATK_MD0700_V3_EX_TEXT_SETS_8859_1       1   /* ISO/IEC 8859-1 */
#define ATK_MD0700_V3_EX_TEXT_SETS_8859_2       2   /* ISO/IEC 8859-2 */
#define ATK_MD0700_V3_EX_TEXT_SETS_8859_4       4   /* ISO/IEC 8859-4 */
#define ATK_MD0700_V3_EX_TEXT_SETS_8859_5       5   /* ISO/IEC 8859-5 */

/* ATK-MD0700 V3ģ���ı�������ɫģʽ���� */
#define ATK_MD0700_V3_EX_TEXT_BG_MODE_CUSTOM    0   /* ָ��������ɫ */
#define ATK_MD0700_V3_EX_TEXT_BG_MODE_ORIGINAL  1   /* ����䱳����ɫ */

/* ATK-MD0700 V3ģ���ı����Ŷ��� */
#define ATK_MD0700_V3_EX_TEXT_SCALE_X1          1   /* �Ŵ�1���������ֲ��䣩 */
#define ATK_MD0700_V3_EX_TEXT_SCALE_X2          2   /* �Ŵ�2�� */
#define ATK_MD0700_V3_EX_TEXT_SCALE_X3          3   /* �Ŵ�3�� */
#define ATK_MD0700_V3_EX_TEXT_SCALE_X4          4   /* �Ŵ�4�� */

/* ATK-MD0700 V3ģ��BTE ROP����ָ��� */
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

/* ATK-MD0700 V3ģ��BTE����ָ��� */
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

void atk_md0700_v3_ex_draw_ellipse(uint16_t x, uint16_t y, uint16_t r1, uint16_t r2, uint16_t color, uint8_t fill);                                         /* ATK-MD0700 V3ģ����չ����Բ */
void atk_md0700_v3_ex_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color, uint8_t fill);                                                        /* ATK-MD0700 V3ģ����չ��Բ */
void atk_md0700_v3_ex_draw_curve(uint16_t x, uint16_t y, uint16_t r1, uint16_t r2, uint16_t color, uint8_t fill, uint8_t part);                             /* ATK-MD0700 V3ģ����չ������ */
void atk_md0700_v3_ex_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint8_t fill);                                     /* ATK-MD0700 V3ģ����չ������ */
void atk_md0700_v3_ex_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                                                        /* ATK-MD0700 V3ģ����չ���߶� */
void atk_md0700_v3_ex_draw_triangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color, uint8_t fill);            /* ATK-MD0700 V3ģ����չ�������� */
void atk_md0700_v3_ex_draw_rounded_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t r1, uint16_t r2, uint16_t color, uint8_t fill);   /* ATK-MD0700 V3ģ����չ��Բ�Ǿ��� */
void atk_md0700_v3_ex_show_text_advanced(uint16_t x,                                                                                                        /* ATK-MD0700 V3ģ����չ��ʾ���֣��߼��� */
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
void atk_md0700_v3_ex_show_text(uint16_t x, uint16_t y, char *text, uint8_t size, uint16_t color);                                                          /* ATK-MD0700 V3ģ����չ��ʾ���� */
void atk_md0700_v3_ex_canvas_window_config(uint32_t address, uint16_t width);                                                                               /* ATK-MD0700 V3ģ����չ����Canvas Window */
void atk_md0700_v3_ex_main_window_config(uint32_t address, uint16_t width, uint16_t x, uint16_t y);                                                         /* ATK-MD0700 V3ģ����չ����Main Window */
void atk_md0700_v3_ex_active_window_config(uint16_t x, uint16_t y, uint16_t width, uint16_t height);                                                        /* ATK-MD0700 V3ģ����չ����Active Window */
void atk_md0700_v3_ex_s0_window_config(uint32_t address, uint16_t width, uint16_t x, uint16_t y);                                                           /* ATK-MD0700 V3ģ����չ����Source0 Window */
void atk_md0700_v3_ex_s1_window_config(uint32_t address, uint16_t width, uint16_t x, uint16_t y);                                                           /* ATK-MD0700 V3ģ����չ����Source1 Window */
void atk_md0700_v3_ex_d_window_config(uint32_t address, uint16_t width, uint16_t x, uint16_t y);                                                            /* ATK-MD0700 V3ģ����չ����Destination Window */
void atk_md0700_v3_ex_bte_window_config(uint16_t width, uint16_t height);                                                                                   /* ATK-MD0700 V3ģ����չ����BLT Window */
void atk_md0700_v3_ex_bte_enable(uint8_t rop, uint8_t mode);                                                                                                /* ATK-MD0700 V3ģ����չʹ��BLT */
void atk_md0700_v3_ex_bte_chroma_key_color_config(uint16_t color);                                                                                          /* ATK-MD0700 V3ģ����չ����BLT�ؼ�ɫ */

#endif
