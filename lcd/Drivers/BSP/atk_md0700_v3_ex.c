/**
 ****************************************************************************************************
 * @file        atk_md0700_v3_ex.c
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

#include "atk_md0700_v3_ex.h"
#include "atk_md0700_v3.h"
#include "atk_md0700_v3_fsmc.h"
#include <string.h>

extern uint16_t atk_md0700_v3_read_stsr(void);
extern void atk_md0700_v3_set_reg(volatile uint16_t reg, volatile uint16_t dat);
extern uint16_t atk_md0700_v3_get_reg(volatile uint16_t reg);
extern void atk_md0700_v3_modify_reg(volatile uint16_t reg, uint16_t clrbit, uint16_t setbit);

/**
 * @brief       竖屏时ATK-MD0700 V3模块扩展坐标转换
 * @param       x: 转换坐标的X坐标
 * @param       y: 转换坐标的Y坐标
 * @retval      无
 */
static inline void atk_md0700_v3_ex_coordinate_transformat(uint16_t *x, uint16_t *y)
{
    *x = *x ^ *y;
    *y = *x ^ *y;
    *x = *x ^ *y;
    *y = atk_md0700_v3_get_lcd_width() - *y - 1;
}

/**
 * @brief       ATK-MD0700 V3模块扩展配置前景色
 * @param       color: 前景色
 * @retval      无
 */
static void atk_md0700_v3_ex_foreground_color_config(uint16_t color)
{
    atk_md0700_v3_set_reg(0xD2, color >> 8);
    atk_md0700_v3_set_reg(0xD3, color >> 3);
    atk_md0700_v3_set_reg(0xD4, color << 3);
}

/**
 * @brief       ATK-MD0700 V3模块扩展配置背景色
 * @param       color: 背景色
 * @retval      无
 */
static void atk_md0700_v3_ex_background_color_config(uint16_t color)
{
    atk_md0700_v3_set_reg(0xD5, color >> 8);
    atk_md0700_v3_set_reg(0xD6, color >> 3);
    atk_md0700_v3_set_reg(0xD7, color << 3);
}

/**
 * @brief       等待ATK-MD0700 V3模块绘画结束
 * @note        必须跟在开启绘图的指令后，或先写指定寄存器
 * @param       无
 * @retval      无
 */
static void atk_md0700_v3_ex_wait_draw_busy(void)
{
    while ((ATK_MD0700_V3_FSMC_DAT & (1 << 7)) != 0);
}

/**
 * @brief       ATK-MD0700 V3模块扩展设置顶点1
 * @note        矩形、线段、三角形、圆角矩形的顶点1
 * @param       x: 顶点1X坐标
 * @param       y: 顶点1Y坐标
 * @retval      无
 */
static inline void atk_md0700_v3_ex_vertex1_config(uint16_t x, uint16_t y)
{
    atk_md0700_v3_set_reg(0x68, x);
    atk_md0700_v3_set_reg(0x69, x >> 8);
    atk_md0700_v3_set_reg(0x6A, y);
    atk_md0700_v3_set_reg(0x6B, y >> 8);
}

/**
 * @brief       ATK-MD0700 V3模块扩展设置顶点2
 * @note        矩形、线段、三角形、圆角矩形的顶点2
 * @param       x: 顶点2X坐标
 * @param       y: 顶点2Y坐标
 * @retval      无
 */
static inline void atk_md0700_v3_ex_vertex2_config(uint16_t x, uint16_t y)
{
    atk_md0700_v3_set_reg(0x6C, x);
    atk_md0700_v3_set_reg(0x6D, x >> 8);
    atk_md0700_v3_set_reg(0x6E, y);
    atk_md0700_v3_set_reg(0x6F, y >> 8);
}

/**
 * @brief       ATK-MD0700 V3模块扩展设置顶点3
 * @note        三角形的顶点3
 * @param       x: 顶点3X坐标
 * @param       y: 顶点3Y坐标
 * @retval      无
 */
static inline void atk_md0700_v3_ex_vertex3_config(uint16_t x, uint16_t y)
{
    atk_md0700_v3_set_reg(0x70, x);
    atk_md0700_v3_set_reg(0x71, x >> 8);
    atk_md0700_v3_set_reg(0x72, y);
    atk_md0700_v3_set_reg(0x73, y >> 8);
}

/**
 * @brief       ATK-MD0700 V3模块扩展设置圆心
 * @note        椭圆、圆、曲线的圆心
 * @param       x: 圆心X坐标
 * @param       y: 圆心Y坐标
 * @retval      无
 */
static inline void atk_md0700_v3_ex_center_config(uint16_t x, uint16_t y)
{
    atk_md0700_v3_set_reg(0x7B, x);
    atk_md0700_v3_set_reg(0x7C, x >> 8);
    atk_md0700_v3_set_reg(0x7D, y);
    atk_md0700_v3_set_reg(0x7E, y >> 8);
}

/**
 * @brief       ATK-MD0700 V3模块扩展设置半径
 * @note        椭圆、圆、曲线、圆角矩形的半径
 * @param       r1: 半径1
 * @param       r2: 半径1
 * @retval      无
 */
static inline void atk_md0700_v3_ex_radius_config(uint16_t r1, uint16_t r2)
{
    atk_md0700_v3_set_reg(0x77, r1);
    atk_md0700_v3_set_reg(0x78, r1 >> 8);
    atk_md0700_v3_set_reg(0x79, r2);
    atk_md0700_v3_set_reg(0x7A, r2 >> 8);
}

/**
 * @brief       ATK-MD0700 V3模块扩展画椭圆
 * @note        坐标系相对于Canvas Window
 * @param       x    : 椭圆圆心X坐标
 * @param       y    : 椭圆圆心Y坐标
 * @param       r1   : 椭圆长半径
 * @param       r2   : 椭圆短半径
 * @param       color: 椭圆颜色
 * @param       fill : 使能填充
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展画圆
 * @note        坐标系相对于Canvas Window
 * @param       x    : 圆圆心X坐标
 * @param       y    : 圆圆心Y坐标
 * @param       r    : 圆半径
 * @param       color: 圆颜色
 * @param       fill : 使能填充
 * @retval      无
 */
void atk_md0700_v3_ex_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color, uint8_t fill)
{
    atk_md0700_v3_ex_draw_ellipse(x, y, r, r, color, fill);
}

/**
 * @brief       ATK-MD0700 V3模块扩展画曲线
 * @note        坐标系相对于Canvas Window
 * @param       x    : 曲线圆心X坐标
 * @param       y    : 曲线圆心Y坐标
 * @param       r1   : 曲线长半径
 * @param       r2   : 曲线短半径
 * @param       color: 曲线颜色
 * @param       fill : 使能填充
 * @param       part : 左下方曲线（0），左上方曲线（1），右上方曲线（2），右下方曲线（3）
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展画矩形
 * @note        坐标系相对于Canvas Window
 * @param       x1   : 矩形顶点1X坐标
 * @param       y1   : 矩形顶点1Y坐标
 * @param       x2   : 矩形顶点2X坐标
 * @param       y2   : 矩形顶点2Y坐标
 * @param       color: 矩形颜色
 * @param       fill : 使能填充
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展画线段
 * @note        坐标系相对于Canvas Window
 * @param       x1   : 线段顶点1X坐标
 * @param       y1   : 线段顶点1Y坐标
 * @param       x2   : 线段顶点2X坐标
 * @param       y2   : 线段顶点2Y坐标
 * @param       color: 线段颜色
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展画三角形
 * @note        坐标系相对于Canvas Window
 * @param       x1   : 三角形顶点1X坐标
 * @param       y1   : 三角形顶点1Y坐标
 * @param       x2   : 三角形顶点2X坐标
 * @param       y2   : 三角形顶点2Y坐标
 * @param       x3   : 三角形顶点3X坐标
 * @param       y3   : 三角形顶点3Y坐标
 * @param       color: 三角形颜色
 * @param       fill : 使能填充
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展画圆角矩形
 * @note        坐标系相对于Canvas Window
 * @param       x1   : 圆角矩形顶点1X坐标
 * @param       y1   : 圆角矩形顶点1Y坐标
 * @param       x2   : 圆角矩形顶点2X坐标
 * @param       y2   : 圆角矩形顶点2Y坐标
 * @param       r1   : 圆角长半径
 * @param       r2   : 圆角短半径
 * @param       color: 圆角矩形颜色
 * @param       fill : 使能填充
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展显示文字（高级）
 * @note        不支持旋转后的坐标系
 * @param       x            : 写入文字的X坐标
 * @param       y            : 写入文字的Y坐标
 * @param       text         : 待显示文本
 * @param       size         : 文本大小
 * @param       sets         : 文本字符集
 * @param       align        : 文本对齐
 * @param       bg_color_mode: 文本背景色模式
 * @param       bg_color     : 文本背景色（bg_color_mode为ATK_MD0700_V3_EX_TEXT_BG_MODE_ORIGINAL时候无效）
 * @param       color        : 文本颜色
 * @param       width_scale  : 文本宽度缩放
 * @param       height_scale : 文本高度缩放
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展显示文字
 * @note        不支持旋转后的坐标系
 * @param       x    : 写入文字的X坐标
 * @param       y    : 写入文字的Y坐标
 * @param       text : 待显示文本
 * @param       size : 文本大小
 * @param       color: 文本颜色
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展配置Canvas Window
 * @note        不支持旋转后的坐标系
 * @param       address: Canvas Window起始地址
 * @param       width  : Canvas Window宽度
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展配置Main Window
 * @note        不支持旋转后的坐标系
 * @param       address: Main Window起始地址
 * @param       width  : Main Window宽度
 * @param       x      : Main Window左上角X坐标
 * @param       y      : Main Window左上角Y坐标
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展配置Active Window
 * @note        坐标系相对于Canvas Window，不支持旋转后的坐标系
 * @param       x     : Active Window左上角X坐标
 * @param       y     : Active Window左上角Y坐标
 * @param       width : Active Window宽度
 * @param       height: Active Window高度
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展配置Source0 Window
 * @note        不支持旋转后的坐标系
 * @param       address: Source0 Window起始地址
 * @param       width  : Source0 Window宽度
 * @param       x      : Source0 Window左上角X坐标
 * @param       y      : Source0 Window左上角Y坐标
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展配置Source1 Window
 * @note        不支持旋转后的坐标系
 * @param       address: Source1 Window起始地址
 * @param       width  : Source1 Window宽度
 * @param       x      : Source1 Window左上角X坐标
 * @param       y      : Source1 Window左上角Y坐标
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展配置Destination Window
 * @note        不支持旋转后的坐标系
 * @param       address: Destination Window起始地址
 * @param       width  : Destination Window宽度
 * @param       x      : Destination Window左上角X坐标
 * @param       y      : Destination Window左上角Y坐标
 * @retval      无
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
 * @brief       ATK-MD0700 V3模块扩展配置BTE Window
 * @note        不支持旋转后的坐标系
 * @param       address: BTE Window起始地址
 * @param       width  : BTE Window宽度
 * @param       x      : BTE Window左上角X坐标
 * @param       y      : BTE Window左上角Y坐标
 * @retval      无
 */
void atk_md0700_v3_ex_bte_window_config(uint16_t width, uint16_t height)
{
    atk_md0700_v3_set_reg(0xB1, width);
    atk_md0700_v3_set_reg(0xB2, width >> 8);
    atk_md0700_v3_set_reg(0xB3, height);
    atk_md0700_v3_set_reg(0xB4, height >> 8);
}

/**
 * @brief       ATK-MD0700 V3模块扩展使能BLT
 * @param       rop : BTE ROP操作
 * @param       mode: BTE操作
 * @retval      无
 */
void atk_md0700_v3_ex_bte_enable(uint8_t rop, uint8_t mode)
{
    atk_md0700_v3_modify_reg(0x91, (0xF << 4), (rop << 4));
    atk_md0700_v3_modify_reg(0x91, (0xF << 0), (mode << 0));
    atk_md0700_v3_modify_reg(0x90, 0, (1 << 4));
    while ((atk_md0700_v3_get_reg(0x90) & (1 << 4)) != 0);
}

/**
 * @brief       ATK-MD0700 V3模块扩展配置BLT关键色
 * @param       color: 关键色
 * @retval      无
 */
void atk_md0700_v3_ex_bte_chroma_key_color_config(uint16_t color)
{
    atk_md0700_v3_ex_background_color_config(color);
}
