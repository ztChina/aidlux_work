/**
 ****************************************************************************************************
 * @file        atk_md0700_v3.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MD0700 V3ģ����������
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

#ifndef __ATK_MD0700_V3_H
#define __ATK_MD0700_V3_H


/* �����Ƿ�ʹ��ATK-MD0700 V3ģ�鴥�� */
#define ATK_MD0700_V3_USING_TOUCH           1

/* ����ATK-MD0700 V3ģ�����õ����� */
#define ATK_MD0700_V3_FONT_12               1
#define ATK_MD0700_V3_FONT_16               1
#define ATK_MD0700_V3_FONT_24               1
#define ATK_MD0700_V3_FONT_32               1

/* Ĭ�����ô��� */
#ifndef ATK_MD0700_V3_USING_TOUCH
#define ATK_MD0700_V3_USING_TOUCH 1
#endif

/* Ĭ������12������ */
#if ((ATK_MD0700_V3_FONT_12 == 0) && (ATK_MD0700_V3_FONT_16 == 0) && (ATK_MD0700_V3_FONT_24 == 0) && (ATK_MD0700_V3_FONT_32 == 0))
#undef ATK_MD0700_V3_FONT_12
#defien ATK_MD0700_V3_FONT_12 1
#endif



/* ATK-MD0700 V3ģ��PID���� */
#define ATK_MD0700_V3_PID_800480            0x748A  /* ATK-MD0700 V3 800*480 */
#define ATK_MD0700_V3_PID_1024600           0x716A  /* ATK-MD0700 V3 1024*600 */

/* ATK-MD0700 V3ģ��LCDɨ�跽���� */
#define ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D  0x00    /* �����ң����ϵ��� */
#define ATK_MD0700_V3_LCD_SCAN_DIR_L2R_D2U  0x01    /* �����ң����µ��� */
#define ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D  0x02    /* ���ҵ��󣬴��ϵ��� */
#define ATK_MD0700_V3_LCD_SCAN_DIR_R2L_D2U  0x03    /* ���ҵ��󣬴��µ��� */
#define ATK_MD0700_V3_LCD_SCAN_DIR_U2D_L2R  0x04    /* ���ϵ��£������� */
#define ATK_MD0700_V3_LCD_SCAN_DIR_U2D_R2L  0x05    /* ���ϵ��£����ҵ��� */
#define ATK_MD0700_V3_LCD_SCAN_DIR_D2U_L2R  0x06    /* ���µ��ϣ������� */
#define ATK_MD0700_V3_LCD_SCAN_DIR_D2U_R2L  0x07    /* ���µ��ϣ����ҵ��� */

/* ATK-MD0700 V3ģ��LCD��ת������ */
#define ATK_MD0700_V3_LCD_DISP_DIR_0        0x00    /* LCD˳ʱ����ת0����ʾ���� */
#define ATK_MD0700_V3_LCD_DISP_DIR_90       0x01    /* LCD˳ʱ����ת90����ʾ���� */

/* ATK-MD0700 V3ģ��LCD��ʾ���嶨�� */
#if (ATK_MD0700_V3_FONT_12 != 0)
#define ATK_MD0700_V3_LCD_FONT_12           12      /* 12������ */
#endif
#if (ATK_MD0700_V3_FONT_16 != 0)
#define ATK_MD0700_V3_LCD_FONT_16           16      /* 16������ */
#endif
#if (ATK_MD0700_V3_FONT_24 != 0)
#define ATK_MD0700_V3_LCD_FONT_24           24      /* 24������ */
#endif
#if (ATK_MD0700_V3_FONT_32 != 0)
#define ATK_MD0700_V3_LCD_FONT_32           32      /* 32������ */
#endif

/* ATK-MD0700 V3ģ��LCD��ʾ����ģʽ���� */
#define ATK_MD0700_V3_NUM_SHOW_NOZERO       0x00    /* ���ָ�λ0����ʾ */
#define ATK_MD0700_V3_NUM_SHOW_ZERO         0x01    /* ���ָ�λ0��ʾ */

/* ������ɫ���壨RGB565�� */
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

/* ������� */
#define ATK_MD0700_V3_EOK                   0       /* û�д��� */
#define ATK_MD0700_V3_ERROR                 1       /* ���� */
#define ATK_MD0700_V3_EINVAL                2       /* �Ƿ����� */

/* �������� */
uint8_t atk_md0700_v3_get_id(void);                                                                                                 /* ��ȡATK-MD0700 V3ģ��ID */
void atk_md0700_v3_start_access_memory(void);                                                                                       /* ��ʼ����ATK-MD0700 V3ģ���Դ� */
void atk_md0700_v3_config_memory_access_coordinate(uint16_t x, uint16_t y);                                                         /* ����ATK-MD0700 V3ģ���Դ������ʼ���� */
void atk_md0700_v3_config_memory_access_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height);                          /* ����ATK-MD0700 V3ģ���Դ���ʴ��� */
uint8_t atk_md0700_v3_init(void);                                                                                                   /* ATK-MD0700 V3ģ���ʼ�� */
uint16_t atk_md0700_v3_get_lcd_width(void);                                                                                         /* ��ȡATK-MD0700 V3ģ��LCD��� */
uint16_t atk_md0700_v3_get_lcd_height(void);                                                                                        /* ��ȡATK-MD0700 V3ģ��LCD�߶� */
void atk_md0700_v3_backlight_config(uint8_t percent);                                                                               /* ����ATK-MD0700 V3ģ��LCD�������� */
void atk_md0700_v3_display_on(void);                                                                                                /* ����ATK-MD0700 V3ģ��LCD��ʾ */
void atk_md0700_v3_display_off(void);                                                                                               /* �ر�ATK-MD0700 V3ģ��LCD��ʾ */
uint8_t atk_md0700_v3_set_scan_dir(uint8_t scan_dir);                                                                               /* ����ATK-MD0700 V3ģ��LCDɨ�跽�� */
uint8_t atk_md0700_v3_set_disp_dir(uint8_t disp_dir);                                                                               /* ����ATK-MD0700 V3ģ��LCD��ʾ���� */
uint8_t atk_md0700_v3_get_scan_dir(void);                                                                                           /* ��ȡATK-MD0700 V3ģ��LCDɨ�跽�� */
uint8_t atk_md0700_v3_get_disp_dir(void);                                                                                           /* ��ȡATK-MD0700 V3ģ��LCD��ʾ���� */
void atk_md0700_v3_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color);                                        /* ATK-MD0700 V3ģ��LCD������� */
void atk_md0700_v3_clear(uint16_t color);                                                                                           /* ATK-MD0700 V3ģ��LCD���� */
void atk_md0700_v3_draw_point(uint16_t x, uint16_t y, volatile uint16_t color);                                                     /* ATK-MD0700 V3ģ��LCD���� */
uint16_t atk_md0700_v3_read_point(uint16_t x, uint16_t y);                                                                          /* ATK-MD0700 V3ģ��LCD���� */
void atk_md0700_v3_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                                   /* ATK-MD0700 V3ģ��LCD���߶� */
void atk_md0700_v3_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                                   /* ATK-MD0700 V3ģ��LCD�����ο� */
void atk_md0700_v3_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);                                                 /* ATK-MD0700 V3ģ��LCD��Բ�ο� */
void atk_md0700_v3_fill_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);                                                 /* ATK-MD0700 V3ģ��LCD��Բ�� */
void atk_md0700_v3_show_char(uint16_t x, uint16_t y, char ch, uint8_t font, uint16_t color);                                        /* ATK-MD0700 V3ģ��LCD��ʾ1���ַ� */
void atk_md0700_v3_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, char *str, uint8_t font, uint16_t color);   /* ATK-MD0700 V3ģ��LCD��ʾ�ַ��� */
void atk_md0700_v3_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t mode, uint8_t font, uint16_t color);        /* ATK-MD0700 V3ģ��LCD��ʾ���֣��ɿ�����ʾ��λ0 */
void atk_md0700_v3_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t font, uint16_t color);                       /* ATK-MD0700 V3ģ��LCD��ʾ���֣�����ʾ��λ0 */
void atk_md0700_v3_show_pic(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *pic);                                /* ATK-MD0700 V3ģ��LCDͼƬ */

#endif
