/**
 ****************************************************************************************************
 * @file        lcd.c
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

#include "lcd.h"

_lcd_dev lcddev = {0};

uint32_t g_point_color = 0XF800;
uint32_t g_back_color  = 0XFFFF;

/**
 * @brief       初始化LCD
 * @param       无
 * @retval      无
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
 * @brief       开显示
 * @param       无
 * @retval      无
 */
void lcd_display_on(void)
{
    atk_md0700_v3_display_on();
}

/**
 * @brief       关显示
 * @param       无
 * @retval      无
 */
void lcd_display_off(void)
{
    atk_md0700_v3_display_off();
}

/**
 * @brief       设置屏幕扫描方向
 * @param       dir: 屏幕扫描方向
 * @retval      无
 */
void lcd_scan_dir(uint8_t dir)
{
    atk_md0700_v3_set_scan_dir(dir);
}

/**
 * @brief       设置屏幕显示方向
 * @param       dir: 屏幕显示方向
 * @retval      无
 */
void lcd_display_dir(uint8_t dir)
{
    atk_md0700_v3_set_disp_dir(dir);
    lcddev.width = atk_md0700_v3_get_lcd_width();
    lcddev.height = atk_md0700_v3_get_lcd_height();
}

/**
 * @brief       准备写GRAM
 * @param       无
 * @retval      无
 */
void lcd_write_ram_prepare(void)
{
    atk_md0700_v3_start_access_memory();
}

/**
 * @brief       设置光标
 * @param       x: 光标X坐标
 * @param       y: 光标Y坐标
 * @retval      无
 */
void lcd_set_cursor(uint16_t x, uint16_t y)
{
    atk_md0700_v3_config_memory_access_coordinate(x, y);
}

/**
 * @brief       读点
 * @param       x: 待读点X坐标
 * @param       y: 待读点Y坐标
 * @retval      点的颜色数据
 */
uint32_t lcd_read_point(uint16_t x, uint16_t y)
{
    return (uint32_t)atk_md0700_v3_read_point(x, y);
}

/**
 * @brief       画点
 * @param       x: 待画点X坐标
 * @param       y: 待画点Y坐标
 * @param       color: 待画点颜色数据
 * @retval      无
 */
void lcd_draw_point(uint16_t x, uint16_t y, uint32_t color)
{
    atk_md0700_v3_draw_point(x, y, (uint16_t)color);
}

/**
 * @brief       LCD清屏
 * @param       color: 清屏颜色数据
 * @retval      无
 */
void lcd_clear(uint16_t color)
{
    atk_md0700_v3_clear(color);
}

/**
 * @brief       填充实心圆
 * @param       x: 圆心X坐标
 * @param       y: 圆心Y坐标
 * @param       r: 圆半径
 * @param       color: 圆的颜色数据
 * @retval      无
 */
void lcd_fill_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
    atk_md0700_v3_fill_circle(x, y, r, color);
}

/**
 * @brief       画圆
 * @param       x: 圆心X坐标
 * @param       y: 圆心Y坐标
 * @param       r: 圆半径
 * @param       color: 圆的颜色数据
 * @retval      无
 */
void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    atk_md0700_v3_draw_circle(x0, y0, r, color);
}

/**
 * @brief       画水平线
 * @param       x: 水平线起始点X坐标
 * @param       y: 水平线起始点Y坐标
 * @param       len: 水平线长度
 * @param       color: 水平线的颜色数据
 * @retval      无
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
 * @brief       设置窗口
 * @param       sx: 窗口左上角X坐标
 * @param       sy: 窗口左上角Y坐标
 * @param       width: 窗口宽度
 * @param       height: 窗口高度
 * @retval      无
 */
void lcd_set_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height)
{
    atk_md0700_v3_config_memory_access_window(sx, sy, width, height);
}

/**
 * @brief       纯色填充矩形
 * @param       sx: 矩形左上角X坐标
 * @param       sy: 矩形左上角Y坐标
 * @param       ex: 矩形右下角X坐标
 * @param       ey: 矩形右下角Y坐标
 * @param       color: 矩形的颜色数据
 * @retval      无
 */
void lcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color)
{
    atk_md0700_v3_fill(sx, sy, ex + 1, ey + 1, (uint16_t)color);
}

/**
 * @brief       彩色填充矩形
 * @param       sx: 矩形左上角X坐标
 * @param       sy: 矩形左上角Y坐标
 * @param       ex: 矩形右下角X坐标
 * @param       ey: 矩形右下角Y坐标
 * @param       color: 矩形的颜色数据
 * @retval      无
 */
void lcd_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color)
{
    atk_md0700_v3_show_pic(sx, sy, ex - sx, ey - sy, color);
}

/**
 * @brief       画直线
 * @param       x1: 直线的起始X坐标
 * @param       y1: 直线的起始Y坐标
 * @param       x2: 直线的终点X坐标
 * @param       y2: 直线的终点Y坐标
 * @param       color: 直线的颜色数据
 * @retval      无
 */
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    atk_md0700_v3_draw_line(x1, y1, x2, y2, color);
}

/**
 * @brief       画矩形
 * @param       x1: 矩形左上角X坐标
 * @param       y1: 矩形左上角Y坐标
 * @param       x2: 矩形右下角X坐标
 * @param       y2: 矩形右下角Y坐标
 * @param       color: 矩形的颜色数据
 * @retval      无
 */
void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    atk_md0700_v3_draw_rect(x1, y1, x2, y2, color);
}

/**
 * @brief       显示字符
 * @param       x: 字符显示的X坐标
 * @param       y: 字符显示的Y坐标
 * @param       chr: 字符
 * @param       size: 字符大小
 * @param       color: 字符的颜色数据
 * @retval      无
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
 * @brief       显示数字
 * @param       x: 数字显示的X坐标
 * @param       y: 数字显示的Y坐标
 * @param       num: 数字
 * @param       len: 数字长度
 * @param       size: 数字大小
 * @param       color: 数字的颜色数据
 * @retval      无
 */
void lcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t color)
{
    atk_md0700_v3_show_num(x, y, num, len, size, color);
}

/**
 * @brief       显示数字，可控制高位0
 * @param       x: 数字显示的X坐标
 * @param       y: 数字显示的Y坐标
 * @param       num: 数字
 * @param       len: 数字长度
 * @param       size: 数字大小
 * @param       mode: 控制高位0显示
 * @arg         0: 不显示高位0
 * @arg         1: 显示高位0
 * @param       color: 数字的颜色数据
 * @retval      无
 */
void lcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode, uint16_t color)
{
    atk_md0700_v3_show_xnum(x, y, num, len, mode, size, color);
}

/**
 * @brief       显示字符串
 * @param       x: 字符串显示的X坐标
 * @param       y: 字符串显示的Y坐标
 * @param       width: 字符串显示宽度
 * @param       height: 字符串显示高度
 * @param       size: 字符串大小
 * @param       p: 字符串
 * @param       color: 字符串的颜色数据
 * @retval      无
 */
void lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p, uint16_t color)
{
    atk_md0700_v3_show_string(x, y, width, height, p, size, color);
}
