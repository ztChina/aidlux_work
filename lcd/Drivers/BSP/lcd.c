/**
 ****************************************************************************************************
 * @file        lcd.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MD0700 V3ģ��LCD������������
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

#include "lcd.h"

_lcd_dev lcddev = {0};

uint32_t g_point_color = 0XF800;
uint32_t g_back_color  = 0XFFFF;

/**
 * @brief       ��ʼ��LCD
 * @param       ��
 * @retval      ��
 */
void lcd_init(void)
{
    uint8_t id;
    
    atk_md0700_v3_init();
    atk_md0700_v3_set_disp_dir(ATK_MD0700_V3_LCD_DISP_DIR_0);
    
    lcddev.width = atk_md0700_v3_get_lcd_width();
    lcddev.height = atk_md0700_v3_get_lcd_height();
    id = atk_md0700_v3_get_id();
    switch (id)
    {
        case 0:
        {
            lcddev.id = 0x784A;
            break;
        }
        case 1:
        {
            lcddev.id = 0x716A;
            break;
        }
        default:
        {
            lcddev.id = 0xFFFF;
            break;
        }
    }
}

/**
 * @brief       ����ʾ
 * @param       ��
 * @retval      ��
 */
void lcd_display_on(void)
{
    atk_md0700_v3_display_on();
}

/**
 * @brief       ����ʾ
 * @param       ��
 * @retval      ��
 */
void lcd_display_off(void)
{
    atk_md0700_v3_display_off();
}

/**
 * @brief       ������Ļɨ�跽��
 * @param       dir: ��Ļɨ�跽��
 * @retval      ��
 */
void lcd_scan_dir(uint8_t dir)
{
    atk_md0700_v3_set_scan_dir(dir);
}

/**
 * @brief       ������Ļ��ʾ����
 * @param       dir: ��Ļ��ʾ����
 * @retval      ��
 */
void lcd_display_dir(uint8_t dir)
{
    atk_md0700_v3_set_disp_dir(dir);
    lcddev.width = atk_md0700_v3_get_lcd_width();
    lcddev.height = atk_md0700_v3_get_lcd_height();
}

/**
 * @brief       ׼��дGRAM
 * @param       ��
 * @retval      ��
 */
void lcd_write_ram_prepare(void)
{
    atk_md0700_v3_start_access_memory();
}

/**
 * @brief       ���ù��
 * @param       x: ���X����
 * @param       y: ���Y����
 * @retval      ��
 */
void lcd_set_cursor(uint16_t x, uint16_t y)
{
    atk_md0700_v3_config_memory_access_coordinate(x, y);
}

/**
 * @brief       ����
 * @param       x: ������X����
 * @param       y: ������Y����
 * @retval      �����ɫ����
 */
uint32_t lcd_read_point(uint16_t x, uint16_t y)
{
    return (uint32_t)atk_md0700_v3_read_point(x, y);
}

/**
 * @brief       ����
 * @param       x: ������X����
 * @param       y: ������Y����
 * @param       color: ��������ɫ����
 * @retval      ��
 */
void lcd_draw_point(uint16_t x, uint16_t y, uint32_t color)
{
    atk_md0700_v3_draw_point(x, y, (uint16_t)color);
}

/**
 * @brief       LCD����
 * @param       color: ������ɫ����
 * @retval      ��
 */
void lcd_clear(uint16_t color)
{
    atk_md0700_v3_clear(color);
}

/**
 * @brief       ���ʵ��Բ
 * @param       x: Բ��X����
 * @param       y: Բ��Y����
 * @param       r: Բ�뾶
 * @param       color: Բ����ɫ����
 * @retval      ��
 */
void lcd_fill_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
    atk_md0700_v3_fill_circle(x, y, r, color);
}

/**
 * @brief       ��Բ
 * @param       x: Բ��X����
 * @param       y: Բ��Y����
 * @param       r: Բ�뾶
 * @param       color: Բ����ɫ����
 * @retval      ��
 */
void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    atk_md0700_v3_draw_circle(x0, y0, r, color);
}

/**
 * @brief       ��ˮƽ��
 * @param       x: ˮƽ����ʼ��X����
 * @param       y: ˮƽ����ʼ��Y����
 * @param       len: ˮƽ�߳���
 * @param       color: ˮƽ�ߵ���ɫ����
 * @retval      ��
 */
void lcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t color)
{
    if (len == 0)
    {
        return;
    }
    
    atk_md0700_v3_fill(x, y, x + len - 1, y, color);
}

/**
 * @brief       ���ô���
 * @param       sx: �������Ͻ�X����
 * @param       sy: �������Ͻ�Y����
 * @param       width: ���ڿ��
 * @param       height: ���ڸ߶�
 * @retval      ��
 */
void lcd_set_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height)
{
    atk_md0700_v3_config_memory_access_window(sx, sy, width, height);
}

/**
 * @brief       ��ɫ������
 * @param       sx: �������Ͻ�X����
 * @param       sy: �������Ͻ�Y����
 * @param       ex: �������½�X����
 * @param       ey: �������½�Y����
 * @param       color: ���ε���ɫ����
 * @retval      ��
 */
void lcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color)
{
    atk_md0700_v3_fill(sx, sy, ex + 1, ey + 1, (uint16_t)color);
}

/**
 * @brief       ��ɫ������
 * @param       sx: �������Ͻ�X����
 * @param       sy: �������Ͻ�Y����
 * @param       ex: �������½�X����
 * @param       ey: �������½�Y����
 * @param       color: ���ε���ɫ����
 * @retval      ��
 */
void lcd_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color)
{
    atk_md0700_v3_show_pic(sx, sy, ex - sx, ey - sy, color);
}

/**
 * @brief       ��ֱ��
 * @param       x1: ֱ�ߵ���ʼX����
 * @param       y1: ֱ�ߵ���ʼY����
 * @param       x2: ֱ�ߵ��յ�X����
 * @param       y2: ֱ�ߵ��յ�Y����
 * @param       color: ֱ�ߵ���ɫ����
 * @retval      ��
 */
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    atk_md0700_v3_draw_line(x1, y1, x2, y2, color);
}

/**
 * @brief       ������
 * @param       x1: �������Ͻ�X����
 * @param       y1: �������Ͻ�Y����
 * @param       x2: �������½�X����
 * @param       y2: �������½�Y����
 * @param       color: ���ε���ɫ����
 * @retval      ��
 */
void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    atk_md0700_v3_draw_rect(x1, y1, x2, y2, color);
}

/**
 * @brief       ��ʾ�ַ�
 * @param       x: �ַ���ʾ��X����
 * @param       y: �ַ���ʾ��Y����
 * @param       chr: �ַ�
 * @param       size: �ַ���С
 * @param       color: �ַ�����ɫ����
 * @retval      ��
 */
void lcd_show_char(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color)
{
    if (mode == 0)
    {
        atk_md0700_v3_fill(x, y, x + (size >> 1) - 1, y + size - 1, g_back_color);
    }
    
    atk_md0700_v3_show_char(x, y, chr, size, color);
}

/**
 * @brief       ��ʾ����
 * @param       x: ������ʾ��X����
 * @param       y: ������ʾ��Y����
 * @param       num: ����
 * @param       len: ���ֳ���
 * @param       size: ���ִ�С
 * @param       color: ���ֵ���ɫ����
 * @retval      ��
 */
void lcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t color)
{
    atk_md0700_v3_show_num(x, y, num, len, size, color);
}

/**
 * @brief       ��ʾ���֣��ɿ��Ƹ�λ0
 * @param       x: ������ʾ��X����
 * @param       y: ������ʾ��Y����
 * @param       num: ����
 * @param       len: ���ֳ���
 * @param       size: ���ִ�С
 * @param       mode: ���Ƹ�λ0��ʾ
 * @arg         0: ����ʾ��λ0
 * @arg         1: ��ʾ��λ0
 * @param       color: ���ֵ���ɫ����
 * @retval      ��
 */
void lcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode, uint16_t color)
{
    atk_md0700_v3_show_xnum(x, y, num, len, mode, size, color);
}

/**
 * @brief       ��ʾ�ַ���
 * @param       x: �ַ�����ʾ��X����
 * @param       y: �ַ�����ʾ��Y����
 * @param       width: �ַ�����ʾ���
 * @param       height: �ַ�����ʾ�߶�
 * @param       size: �ַ�����С
 * @param       p: �ַ���
 * @param       color: �ַ�������ɫ����
 * @retval      ��
 */
void lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p, uint16_t color)
{
    atk_md0700_v3_show_string(x, y, width, height, p, size, color);
}
