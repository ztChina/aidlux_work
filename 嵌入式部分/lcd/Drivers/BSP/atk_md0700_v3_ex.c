/**
 ****************************************************************************************************
 * @file        atk_md0700_v3_ex.c
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

#include "atk_md0700_v3_ex.h"
#include "atk_md0700_v3.h"
#include "atk_md0700_v3_fsmc.h"
#include <string.h>

extern uint16_t atk_md0700_v3_read_stsr(void);
extern void atk_md0700_v3_set_reg(volatile uint16_t reg, volatile uint16_t dat);
extern uint16_t atk_md0700_v3_get_reg(volatile uint16_t reg);
extern void atk_md0700_v3_modify_reg(volatile uint16_t reg, uint16_t clrbit, uint16_t setbit);

/**
 * @brief       ����ʱATK-MD0700 V3ģ����չ����ת��
 * @param       x: ת�������X����
 * @param       y: ת�������Y����
 * @retval      ��
 */
static inline void atk_md0700_v3_ex_coordinate_transformat(uint16_t *x, uint16_t *y)
{
    *x = *x ^ *y;
    *y = *x ^ *y;
    *x = *x ^ *y;
    *y = atk_md0700_v3_get_lcd_width() - *y - 1;
}

/**
 * @brief       ATK-MD0700 V3ģ����չ����ǰ��ɫ
 * @param       color: ǰ��ɫ
 * @retval      ��
 */
static void atk_md0700_v3_ex_foreground_color_config(uint16_t color)
{
    atk_md0700_v3_set_reg(0xD2, color >> 8);
    atk_md0700_v3_set_reg(0xD3, color >> 3);
    atk_md0700_v3_set_reg(0xD4, color << 3);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ���ñ���ɫ
 * @param       color: ����ɫ
 * @retval      ��
 */
static void atk_md0700_v3_ex_background_color_config(uint16_t color)
{
    atk_md0700_v3_set_reg(0xD5, color >> 8);
    atk_md0700_v3_set_reg(0xD6, color >> 3);
    atk_md0700_v3_set_reg(0xD7, color << 3);
}

/**
 * @brief       �ȴ�ATK-MD0700 V3ģ��滭����
 * @note        ������ڿ�����ͼ��ָ��󣬻���дָ���Ĵ���
 * @param       ��
 * @retval      ��
 */
static void atk_md0700_v3_ex_wait_draw_busy(void)
{
    while ((ATK_MD0700_V3_FSMC_DAT & (1 << 7)) != 0);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ���ö���1
 * @note        ���Ρ��߶Ρ������Ρ�Բ�Ǿ��εĶ���1
 * @param       x: ����1X����
 * @param       y: ����1Y����
 * @retval      ��
 */
static inline void atk_md0700_v3_ex_vertex1_config(uint16_t x, uint16_t y)
{
    atk_md0700_v3_set_reg(0x68, x);
    atk_md0700_v3_set_reg(0x69, x >> 8);
    atk_md0700_v3_set_reg(0x6A, y);
    atk_md0700_v3_set_reg(0x6B, y >> 8);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ���ö���2
 * @note        ���Ρ��߶Ρ������Ρ�Բ�Ǿ��εĶ���2
 * @param       x: ����2X����
 * @param       y: ����2Y����
 * @retval      ��
 */
static inline void atk_md0700_v3_ex_vertex2_config(uint16_t x, uint16_t y)
{
    atk_md0700_v3_set_reg(0x6C, x);
    atk_md0700_v3_set_reg(0x6D, x >> 8);
    atk_md0700_v3_set_reg(0x6E, y);
    atk_md0700_v3_set_reg(0x6F, y >> 8);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ���ö���3
 * @note        �����εĶ���3
 * @param       x: ����3X����
 * @param       y: ����3Y����
 * @retval      ��
 */
static inline void atk_md0700_v3_ex_vertex3_config(uint16_t x, uint16_t y)
{
    atk_md0700_v3_set_reg(0x70, x);
    atk_md0700_v3_set_reg(0x71, x >> 8);
    atk_md0700_v3_set_reg(0x72, y);
    atk_md0700_v3_set_reg(0x73, y >> 8);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ����Բ��
 * @note        ��Բ��Բ�����ߵ�Բ��
 * @param       x: Բ��X����
 * @param       y: Բ��Y����
 * @retval      ��
 */
static inline void atk_md0700_v3_ex_center_config(uint16_t x, uint16_t y)
{
    atk_md0700_v3_set_reg(0x7B, x);
    atk_md0700_v3_set_reg(0x7C, x >> 8);
    atk_md0700_v3_set_reg(0x7D, y);
    atk_md0700_v3_set_reg(0x7E, y >> 8);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ���ð뾶
 * @note        ��Բ��Բ�����ߡ�Բ�Ǿ��εİ뾶
 * @param       r1: �뾶1
 * @param       r2: �뾶1
 * @retval      ��
 */
static inline void atk_md0700_v3_ex_radius_config(uint16_t r1, uint16_t r2)
{
    atk_md0700_v3_set_reg(0x77, r1);
    atk_md0700_v3_set_reg(0x78, r1 >> 8);
    atk_md0700_v3_set_reg(0x79, r2);
    atk_md0700_v3_set_reg(0x7A, r2 >> 8);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ����Բ
 * @note        ����ϵ�����Canvas Window
 * @param       x    : ��ԲԲ��X����
 * @param       y    : ��ԲԲ��Y����
 * @param       r1   : ��Բ���뾶
 * @param       r2   : ��Բ�̰뾶
 * @param       color: ��Բ��ɫ
 * @param       fill : ʹ�����
 * @retval      ��
 */
void atk_md0700_v3_ex_draw_ellipse(uint16_t x, uint16_t y, uint16_t r1, uint16_t r2, uint16_t color, uint8_t fill)
{
    if (atk_md0700_v3_get_disp_dir() == ATK_MD0700_V3_LCD_DISP_DIR_0)
    {
        atk_md0700_v3_ex_coordinate_transformat(&x, &y);
    }
    atk_md0700_v3_ex_center_config(x, y);
    atk_md0700_v3_ex_radius_config(r1, r2);
    atk_md0700_v3_ex_foreground_color_config(color);
    atk_md0700_v3_modify_reg(0x76, (3 << 4), 0);
    switch (fill)
    {
        case 0:
        {
            atk_md0700_v3_modify_reg(0x76, (1 << 6), 0);
            break;
        }
        case 1:
        default:
        {
            atk_md0700_v3_modify_reg(0x76, 0, (1 << 6));
            break;
        }
    }
    atk_md0700_v3_modify_reg(0x76, 0, (1 << 7));
    atk_md0700_v3_ex_wait_draw_busy();
}

/**
 * @brief       ATK-MD0700 V3ģ����չ��Բ
 * @note        ����ϵ�����Canvas Window
 * @param       x    : ԲԲ��X����
 * @param       y    : ԲԲ��Y����
 * @param       r    : Բ�뾶
 * @param       color: Բ��ɫ
 * @param       fill : ʹ�����
 * @retval      ��
 */
void atk_md0700_v3_ex_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color, uint8_t fill)
{
    atk_md0700_v3_ex_draw_ellipse(x, y, r, r, color, fill);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ������
 * @note        ����ϵ�����Canvas Window
 * @param       x    : ����Բ��X����
 * @param       y    : ����Բ��Y����
 * @param       r1   : ���߳��뾶
 * @param       r2   : ���߶̰뾶
 * @param       color: ������ɫ
 * @param       fill : ʹ�����
 * @param       part : ���·����ߣ�0�������Ϸ����ߣ�1�������Ϸ����ߣ�2�������·����ߣ�3��
 * @retval      ��
 */
void atk_md0700_v3_ex_draw_curve(uint16_t x, uint16_t y, uint16_t r1, uint16_t r2, uint16_t color, uint8_t fill, uint8_t part)
{
    if (atk_md0700_v3_get_disp_dir() == ATK_MD0700_V3_LCD_DISP_DIR_0)
    {
        atk_md0700_v3_ex_coordinate_transformat(&x, &y);
    }
    atk_md0700_v3_ex_center_config(x, y);
    atk_md0700_v3_ex_radius_config(r1, r2);
    atk_md0700_v3_ex_foreground_color_config(color);
    atk_md0700_v3_modify_reg(0x76, (3 << 4), (1 << 4));
    switch (part)
    {
        case 0:
        default:
        {
            atk_md0700_v3_modify_reg(0x76, (3 << 0), (0 << 0));
            break;
        }
        case 1:
        {
            atk_md0700_v3_modify_reg(0x76, (3 << 0), (1 << 0));
            break;
        }
        case 2:
        {
            atk_md0700_v3_modify_reg(0x76, (3 << 0), (2 << 0));
            break;
        }
        case 3:
        {
            atk_md0700_v3_modify_reg(0x76, (3 << 0), (3 << 0));
            break;
        }
    }
    switch (fill)
    {
        case 0:
        {
            atk_md0700_v3_modify_reg(0x76, (1 << 6), 0);
            break;
        }
        case 1:
        default:
        {
            atk_md0700_v3_modify_reg(0x76, 0, (1 << 6));
            break;
        }
    }
    atk_md0700_v3_modify_reg(0x76, 0, (1 << 7));
    atk_md0700_v3_ex_wait_draw_busy();
}

/**
 * @brief       ATK-MD0700 V3ģ����չ������
 * @note        ����ϵ�����Canvas Window
 * @param       x1   : ���ζ���1X����
 * @param       y1   : ���ζ���1Y����
 * @param       x2   : ���ζ���2X����
 * @param       y2   : ���ζ���2Y����
 * @param       color: ������ɫ
 * @param       fill : ʹ�����
 * @retval      ��
 */
void atk_md0700_v3_ex_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint8_t fill)
{
    if (atk_md0700_v3_get_disp_dir() == ATK_MD0700_V3_LCD_DISP_DIR_0)
    {
        atk_md0700_v3_ex_coordinate_transformat(&x1, &y1);
        atk_md0700_v3_ex_coordinate_transformat(&x2, &y2);
    }
    atk_md0700_v3_ex_vertex1_config(x1, y1);
    atk_md0700_v3_ex_vertex2_config(x2, y2);
    atk_md0700_v3_ex_foreground_color_config(color);
    atk_md0700_v3_modify_reg(0x76, (3 << 4), (2 << 4));
    switch (fill)
    {
        case 0:
        {
            atk_md0700_v3_modify_reg(0x76, (1 << 6), 0);
            break;
        }
        case 1:
        default:
        {
            atk_md0700_v3_modify_reg(0x76, 0, (1 << 6));
            break;
        }
    }
    atk_md0700_v3_modify_reg(0x76, 0, (1 << 7));
    atk_md0700_v3_ex_wait_draw_busy();
}

/**
 * @brief       ATK-MD0700 V3ģ����չ���߶�
 * @note        ����ϵ�����Canvas Window
 * @param       x1   : �߶ζ���1X����
 * @param       y1   : �߶ζ���1Y����
 * @param       x2   : �߶ζ���2X����
 * @param       y2   : �߶ζ���2Y����
 * @param       color: �߶���ɫ
 * @retval      ��
 */
void atk_md0700_v3_ex_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    if (atk_md0700_v3_get_disp_dir() == ATK_MD0700_V3_LCD_DISP_DIR_0)
    {
        atk_md0700_v3_ex_coordinate_transformat(&x1, &y1);
        atk_md0700_v3_ex_coordinate_transformat(&x2, &y2);
    }
    atk_md0700_v3_ex_vertex1_config(x1, y1);
    atk_md0700_v3_ex_vertex2_config(x2, y2);
    atk_md0700_v3_ex_foreground_color_config(color);
    atk_md0700_v3_modify_reg(0x67, (0xF << 1), 0);
    atk_md0700_v3_modify_reg(0x67, 0, (1 << 7));
    atk_md0700_v3_ex_wait_draw_busy();
}

/**
 * @brief       ATK-MD0700 V3ģ����չ��������
 * @note        ����ϵ�����Canvas Window
 * @param       x1   : �����ζ���1X����
 * @param       y1   : �����ζ���1Y����
 * @param       x2   : �����ζ���2X����
 * @param       y2   : �����ζ���2Y����
 * @param       x3   : �����ζ���3X����
 * @param       y3   : �����ζ���3Y����
 * @param       color: ��������ɫ
 * @param       fill : ʹ�����
 * @retval      ��
 */
void atk_md0700_v3_ex_draw_triangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color, uint8_t fill)
{
    if (atk_md0700_v3_get_disp_dir() == ATK_MD0700_V3_LCD_DISP_DIR_0)
    {
        atk_md0700_v3_ex_coordinate_transformat(&x1, &y1);
        atk_md0700_v3_ex_coordinate_transformat(&x2, &y2);
        atk_md0700_v3_ex_coordinate_transformat(&x3, &y3);
    }
    atk_md0700_v3_ex_vertex1_config(x1, y1);
    atk_md0700_v3_ex_vertex2_config(x2, y2);
    atk_md0700_v3_ex_vertex3_config(x3, y3);
    atk_md0700_v3_ex_foreground_color_config(color);
    atk_md0700_v3_modify_reg(0x67, (0xF << 1), (1 << 1));
    switch (fill)
    {
        case 0:
        {
            atk_md0700_v3_modify_reg(0x67, (1 << 5), 0);
            break;
        }
        case 1:
        default:
        {
            atk_md0700_v3_modify_reg(0x67, 0, (1 << 5));
            break;
        }
    }
    atk_md0700_v3_modify_reg(0x67, 0, (1 << 7));
    atk_md0700_v3_ex_wait_draw_busy();
}

/**
 * @brief       ATK-MD0700 V3ģ����չ��Բ�Ǿ���
 * @note        ����ϵ�����Canvas Window
 * @param       x1   : Բ�Ǿ��ζ���1X����
 * @param       y1   : Բ�Ǿ��ζ���1Y����
 * @param       x2   : Բ�Ǿ��ζ���2X����
 * @param       y2   : Բ�Ǿ��ζ���2Y����
 * @param       r1   : Բ�ǳ��뾶
 * @param       r2   : Բ�Ƕ̰뾶
 * @param       color: Բ�Ǿ�����ɫ
 * @param       fill : ʹ�����
 * @retval      ��
 */
void atk_md0700_v3_ex_draw_rounded_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t r1, uint16_t r2, uint16_t color, uint8_t fill)
{
    if (atk_md0700_v3_get_disp_dir() == ATK_MD0700_V3_LCD_DISP_DIR_0)
    {
        atk_md0700_v3_ex_coordinate_transformat(&x1, &y1);
        atk_md0700_v3_ex_coordinate_transformat(&x2, &y2);
    }
    atk_md0700_v3_ex_vertex1_config(x1, y1);
    atk_md0700_v3_ex_vertex2_config(x2, y2);
    atk_md0700_v3_ex_radius_config(r1, r2);
    atk_md0700_v3_ex_foreground_color_config(color);
    atk_md0700_v3_modify_reg(0x76, 0, (3 << 4));
    switch (fill)
    {
        case 0:
        {
            atk_md0700_v3_modify_reg(0x76, (1 << 6), 0);
            break;
        }
        case 1:
        default:
        {
            atk_md0700_v3_modify_reg(0x76, 0, (1 << 6));
            break;
        }
    }
    atk_md0700_v3_modify_reg(0x76, 0, (1 << 7));
    atk_md0700_v3_ex_wait_draw_busy();
}

/**
 * @brief       ATK-MD0700 V3ģ����չ��ʾ���֣��߼���
 * @note        ��֧����ת�������ϵ
 * @param       x            : д�����ֵ�X����
 * @param       y            : д�����ֵ�Y����
 * @param       text         : ����ʾ�ı�
 * @param       size         : �ı���С
 * @param       sets         : �ı��ַ���
 * @param       align        : �ı�����
 * @param       bg_color_mode: �ı�����ɫģʽ
 * @param       bg_color     : �ı�����ɫ��bg_color_modeΪATK_MD0700_V3_EX_TEXT_BG_MODE_ORIGINALʱ����Ч��
 * @param       color        : �ı���ɫ
 * @param       width_scale  : �ı��������
 * @param       height_scale : �ı��߶�����
 * @retval      ��
 */
void atk_md0700_v3_ex_show_text_advanced(uint16_t x,
                                   uint16_t y,
                                   char *text,
                                   uint8_t size,
                                   uint8_t sets,
                                   uint8_t align,
                                   uint8_t bg_color_mode,
                                   uint16_t bg_color,
                                   uint16_t color,
                                   uint8_t width_scale,
                                   uint8_t height_scale)
{
    uint32_t text_index;
    
    atk_md0700_v3_set_reg(0x63, x);
    atk_md0700_v3_set_reg(0x64, x >> 8);
    atk_md0700_v3_set_reg(0x65, y);
    atk_md0700_v3_set_reg(0x66, y >> 8);
    atk_md0700_v3_modify_reg(0xCC, (3 << 6), 0);
    atk_md0700_v3_modify_reg(0xCC, (3 << 4), (size << 4));
    atk_md0700_v3_modify_reg(0xCC, (3 << 0), (sets << 0));
    atk_md0700_v3_modify_reg(0xCD, (1 << 7), (align << 7));
    atk_md0700_v3_modify_reg(0xCD, (1 << 6), (bg_color_mode << 6));
    atk_md0700_v3_modify_reg(0xCD, (1 << 4), (0 << 4));
    atk_md0700_v3_modify_reg(0xCD, (3 << 2), (width_scale << 2));
    atk_md0700_v3_modify_reg(0xCD, (3 << 0), (height_scale << 0));
    atk_md0700_v3_ex_foreground_color_config(color);
    atk_md0700_v3_ex_background_color_config(bg_color);
    atk_md0700_v3_modify_reg(0x03, 0, (1 << 2));
    for (text_index=0; text_index<strlen(text); text_index++)
    {
        while ((atk_md0700_v3_read_stsr() & (1 << 7)) != 0);
        atk_md0700_v3_set_reg(0x04, text[text_index]);
    }
    while ((atk_md0700_v3_read_stsr() & (1 << 3)) != 0);
    atk_md0700_v3_modify_reg(0x03, (1 << 2), 0);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ��ʾ����
 * @note        ��֧����ת�������ϵ
 * @param       x    : д�����ֵ�X����
 * @param       y    : д�����ֵ�Y����
 * @param       text : ����ʾ�ı�
 * @param       size : �ı���С
 * @param       color: �ı���ɫ
 * @retval      ��
 */
void atk_md0700_v3_ex_show_text(uint16_t x, uint16_t y, char *text, uint8_t size, uint16_t color)
{
    atk_md0700_v3_ex_show_text_advanced(x,
                                  y,
                                  text,
                                  size,
                                  ATK_MD0700_V3_EX_TEXT_SETS_8859_1,
                                  ATK_MD0700_V3_EX_DISABLE,
                                  ATK_MD0700_V3_EX_TEXT_BG_MODE_ORIGINAL,
                                  0,
                                  color,
                                  ATK_MD0700_V3_EX_TEXT_SCALE_X1,
                                  ATK_MD0700_V3_EX_TEXT_SCALE_X1);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ����Canvas Window
 * @note        ��֧����ת�������ϵ
 * @param       address: Canvas Window��ʼ��ַ
 * @param       width  : Canvas Window���
 * @retval      ��
 */
void atk_md0700_v3_ex_canvas_window_config(uint32_t address, uint16_t width)
{
    atk_md0700_v3_set_reg(0x50, address & ~(3 << 0));
    atk_md0700_v3_set_reg(0x51, address >> 8);
    atk_md0700_v3_set_reg(0x52, address >> 16);
    atk_md0700_v3_set_reg(0x53, address >> 24);
    atk_md0700_v3_set_reg(0x54, width & ~(3 << 0));
    atk_md0700_v3_set_reg(0x55, width >> 8);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ����Main Window
 * @note        ��֧����ת�������ϵ
 * @param       address: Main Window��ʼ��ַ
 * @param       width  : Main Window���
 * @param       x      : Main Window���Ͻ�X����
 * @param       y      : Main Window���Ͻ�Y����
 * @retval      ��
 */
void atk_md0700_v3_ex_main_window_config(uint32_t address, uint16_t width, uint16_t x, uint16_t y)
{
    atk_md0700_v3_set_reg(0x20, address & ~(3 << 0));
    atk_md0700_v3_set_reg(0x21, address >> 8);
    atk_md0700_v3_set_reg(0x22, address >> 16);
    atk_md0700_v3_set_reg(0x23, address >> 24);
    atk_md0700_v3_set_reg(0x24, width & ~(3 << 0));
    atk_md0700_v3_set_reg(0x25, width >> 8);
    atk_md0700_v3_set_reg(0x26, x & ~(3 << 0));
    atk_md0700_v3_set_reg(0x27, x >> 8);
    atk_md0700_v3_set_reg(0x28, y);
    atk_md0700_v3_set_reg(0x29, y >> 8);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ����Active Window
 * @note        ����ϵ�����Canvas Window����֧����ת�������ϵ
 * @param       x     : Active Window���Ͻ�X����
 * @param       y     : Active Window���Ͻ�Y����
 * @param       width : Active Window���
 * @param       height: Active Window�߶�
 * @retval      ��
 */
void atk_md0700_v3_ex_active_window_config(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    atk_md0700_v3_set_reg(0x56, x);
    atk_md0700_v3_set_reg(0x57, x >> 8);
    atk_md0700_v3_set_reg(0x58, y);
    atk_md0700_v3_set_reg(0x59, y >> 8);
    atk_md0700_v3_set_reg(0x5A, width);
    atk_md0700_v3_set_reg(0x5B, width >> 8);
    atk_md0700_v3_set_reg(0x5C, height);
    atk_md0700_v3_set_reg(0x5D, height >> 8);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ����Source0 Window
 * @note        ��֧����ת�������ϵ
 * @param       address: Source0 Window��ʼ��ַ
 * @param       width  : Source0 Window���
 * @param       x      : Source0 Window���Ͻ�X����
 * @param       y      : Source0 Window���Ͻ�Y����
 * @retval      ��
 */
void atk_md0700_v3_ex_s0_window_config(uint32_t address, uint16_t width, uint16_t x, uint16_t y)
{
    atk_md0700_v3_modify_reg(0x92, (3 << 5), (1 << 5));
    atk_md0700_v3_set_reg(0x93, address & ~(3 << 0));
    atk_md0700_v3_set_reg(0x94, address >> 8);
    atk_md0700_v3_set_reg(0x95, address >> 16);
    atk_md0700_v3_set_reg(0x96, address >> 24);
    atk_md0700_v3_set_reg(0x97, width & ~(3 << 0));
    atk_md0700_v3_set_reg(0x98, width >> 8);
    atk_md0700_v3_set_reg(0x99, x);
    atk_md0700_v3_set_reg(0x9A, x >> 8);
    atk_md0700_v3_set_reg(0x9B, y);
    atk_md0700_v3_set_reg(0x9C, y >> 8);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ����Source1 Window
 * @note        ��֧����ת�������ϵ
 * @param       address: Source1 Window��ʼ��ַ
 * @param       width  : Source1 Window���
 * @param       x      : Source1 Window���Ͻ�X����
 * @param       y      : Source1 Window���Ͻ�Y����
 * @retval      ��
 */
void atk_md0700_v3_ex_s1_window_config(uint32_t address, uint16_t width, uint16_t x, uint16_t y)
{
    atk_md0700_v3_modify_reg(0x92, (7 << 2), (1 << 2));
    atk_md0700_v3_set_reg(0x9D, address & ~(3 << 0));
    atk_md0700_v3_set_reg(0x9E, address >> 8);
    atk_md0700_v3_set_reg(0x9F, address >> 16);
    atk_md0700_v3_set_reg(0xA0, address >> 24);
    atk_md0700_v3_set_reg(0xA1, width & ~(3 << 0));
    atk_md0700_v3_set_reg(0xA2, width >> 8);
    atk_md0700_v3_set_reg(0xA3, x);
    atk_md0700_v3_set_reg(0xA4, x >> 8);
    atk_md0700_v3_set_reg(0xA5, y);
    atk_md0700_v3_set_reg(0xA6, y >> 8);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ����Destination Window
 * @note        ��֧����ת�������ϵ
 * @param       address: Destination Window��ʼ��ַ
 * @param       width  : Destination Window���
 * @param       x      : Destination Window���Ͻ�X����
 * @param       y      : Destination Window���Ͻ�Y����
 * @retval      ��
 */
void atk_md0700_v3_ex_d_window_config(uint32_t address, uint16_t width, uint16_t x, uint16_t y)
{
    atk_md0700_v3_modify_reg(0x92, (3 << 0), (1 << 0));
    atk_md0700_v3_set_reg(0xA7, address & ~(3 << 0));
    atk_md0700_v3_set_reg(0xA8, address >> 8);
    atk_md0700_v3_set_reg(0xA9, address >> 16);
    atk_md0700_v3_set_reg(0xAA, address >> 24);
    atk_md0700_v3_set_reg(0xAB, width & ~(3 << 0));
    atk_md0700_v3_set_reg(0xAC, width >> 8);
    atk_md0700_v3_set_reg(0xAD, x);
    atk_md0700_v3_set_reg(0xAE, x >> 8);
    atk_md0700_v3_set_reg(0xAF, y);
    atk_md0700_v3_set_reg(0xB0, y >> 8);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ����BTE Window
 * @note        ��֧����ת�������ϵ
 * @param       address: BTE Window��ʼ��ַ
 * @param       width  : BTE Window���
 * @param       x      : BTE Window���Ͻ�X����
 * @param       y      : BTE Window���Ͻ�Y����
 * @retval      ��
 */
void atk_md0700_v3_ex_bte_window_config(uint16_t width, uint16_t height)
{
    atk_md0700_v3_set_reg(0xB1, width);
    atk_md0700_v3_set_reg(0xB2, width >> 8);
    atk_md0700_v3_set_reg(0xB3, height);
    atk_md0700_v3_set_reg(0xB4, height >> 8);
}

/**
 * @brief       ATK-MD0700 V3ģ����չʹ��BLT
 * @param       rop : BTE ROP����
 * @param       mode: BTE����
 * @retval      ��
 */
void atk_md0700_v3_ex_bte_enable(uint8_t rop, uint8_t mode)
{
    atk_md0700_v3_modify_reg(0x91, (0xF << 4), (rop << 4));
    atk_md0700_v3_modify_reg(0x91, (0xF << 0), (mode << 0));
    atk_md0700_v3_modify_reg(0x90, 0, (1 << 4));
    while ((atk_md0700_v3_get_reg(0x90) & (1 << 4)) != 0);
}

/**
 * @brief       ATK-MD0700 V3ģ����չ����BLT�ؼ�ɫ
 * @param       color: �ؼ�ɫ
 * @retval      ��
 */
void atk_md0700_v3_ex_bte_chroma_key_color_config(uint16_t color)
{
    atk_md0700_v3_ex_background_color_config(color);
}
