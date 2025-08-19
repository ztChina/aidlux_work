/**
 ****************************************************************************************************
 * @file        atk_md0700_v3.c
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

#include "atk_md0700_v3.h"
#include "atk_md0700_v3_font.h"
#include "atk_md0700_v3_fsmc.h"
//#include "./SYSTEM/delay/delay.h"
//#include "./SYSTEM/usart/usart.h"

/* ATK-MD0700 V3ģ��LCDʱ��ṹ�� */
typedef struct
{
    uint8_t clock_freq_in_mhz;
    uint16_t hactive;
    uint8_t hback_porch;
    uint8_t hfront_porch;
    uint8_t hsync_len;
    uint16_t vactive;
    uint8_t vback_porch;
    uint8_t vfront_porch;
    uint8_t vsync_len;
} atk_md0700_v3_lcd_timing_t;

/* ATK-MD0700 V3ģ������ṹ�� */
typedef struct
{
    uint8_t id;
    uint16_t pid;
    uint16_t width;
    uint16_t height;
    atk_md0700_v3_lcd_timing_t timing;
} atk_md0700_v3_lcd_param_t;

/* ATK-MD0700 V3ģ�����ƥ��� */
static const atk_md0700_v3_lcd_param_t atk_md0700_v3_lcd_param[] = {
    {0,  ATK_MD0700_V3_PID_800480,  800, 480, {33,  800,  46, 210,  1, 480, 23, 22, 1}},  /* ATK-MD0700 V3 800*480 */
    {1, ATK_MD0700_V3_PID_1024600, 1024, 600, {45, 1024, 140, 160, 20, 600, 20, 12, 3}},  /* ATK-MD0700 V3 1024*600 */
};

/* ATK-MD0700 V3ģ��״̬���ݽṹ�� */
static struct
{
    const atk_md0700_v3_lcd_param_t *param;
    uint16_t width;
    uint16_t height;
    uint8_t disp_dir;
    uint8_t scan_dir;
} g_atk_md0700_v3_sta = {0};

/**
 * @brief       ATK-MD0700 V3ģ���״̬�Ĵ���
 * @param       ��
 * @retval      ATK-MD0700 V3ģ��״̬�Ĵ���ֵ
 */
uint16_t atk_md0700_v3_read_stsr(void)
{
    return ATK_MD0700_V3_FSMC_REG;
}

/**
 * @brief       ATK-MD0700 V3ģ�����üĴ���
 * @param       reg: �Ĵ�����ַ
 * @param       dat: ����
 * @retval      ��
 */
void atk_md0700_v3_set_reg(volatile uint16_t reg, volatile uint16_t dat)
{
    ATK_MD0700_V3_FSMC_REG = reg;
    ATK_MD0700_V3_FSMC_DAT = dat;
}

/**
 * @brief       ��ȡATK-MD0700 V3ģ��Ĵ���ֵ
 * @param       reg: �Ĵ�����ַ
 * @retval      �Ĵ���ֵ
 */
uint16_t atk_md0700_v3_get_reg(volatile uint16_t reg)
{
    volatile uint16_t dat;
    
    ATK_MD0700_V3_FSMC_REG = reg;
    dat = ATK_MD0700_V3_FSMC_DAT;
    
    return dat;
}

/**
 * @brief       ATK-MD0700 V3ģ���޸ļĴ���
 * @param       reg   : �Ĵ�����ַ
 * @param       clrbit: ���λ
 * @param       setbit: ����λ
 * @retval      ��
 */
void atk_md0700_v3_modify_reg(volatile uint16_t reg, uint16_t clrbit, uint16_t setbit)
{
    volatile uint16_t original;
    
    ATK_MD0700_V3_FSMC_REG = reg;
    original = ATK_MD0700_V3_FSMC_DAT;
    original &= ~(clrbit);
    original |= (setbit);
    ATK_MD0700_V3_FSMC_DAT = original;
}

/**
 * @brief       ��ȡATK-MD0700 V3ģ��ID
 * @param       ��
 * @retval      ATK-MD0700 V3ģ��ID
 */
uint8_t atk_md0700_v3_get_id(void)
{
    uint8_t id = 0;
    
    atk_md0700_v3_modify_reg(0x85, (3 << 0), 0);        /* �ر�PWM[0]��ʹ��GPIOC[7] */
    atk_md0700_v3_modify_reg(0x01, (1 << 5), 0);        /* �رհ������ܣ�ʹ��GPIOB[4] */
    atk_md0700_v3_modify_reg(0xF3, 0, (1 << 7));        /* ����GPIOC[7]Ϊ���� */
    delay_ms(100);                                      /* ��Ҫ��ʱ������ID���ܶ�ȡʧ�� */
    
    id = (atk_md0700_v3_get_reg(0xF2) & (1 << 4)) >> 4; /* ��GPIOB[4] */
    id |= (atk_md0700_v3_get_reg(0xF4) & (1 << 7)) >> 6;/* ��GPIOC[7] */
    
    return id;
}

/**
 * @brief       ATK-MD0700 V3ģ�������ʼ��
 * @note        ͨ��ATK-MD0700 V3ģ���IDȷ��LCD�ĳߴ��ʱ��Ȳ���
 * @param       id: ATK-MD0700 V3ģ��ID
 * @retval      ATK_MD0700_V3_EOK   : ATK-MD0700 V3ģ�������ʼ���ɹ�
 *              ATK_MD0700_V3_EINVAL: ��Ч��ATK-MD0700 V3ģ��ID
 */
static uint8_t atk_md0700_v3_setup_param_by_id(uint8_t id)
{
    uint8_t index;
    
    for (index=0; index<(sizeof(atk_md0700_v3_lcd_param) / sizeof(atk_md0700_v3_lcd_param[0])); index++)
    {
        if (id == atk_md0700_v3_lcd_param[index].id)
        {
            g_atk_md0700_v3_sta.param = &atk_md0700_v3_lcd_param[index];
            return ATK_MD0700_V3_EOK;
        }
    }
    
    return ATK_MD0700_V3_EINVAL;
}

/**
 * @brief       ATK-MD0700 V3ģ��LT7381��ʼ��
 * @note        ͨ��ATK-MD0700 V3ģ��IDȷ����LCD������ʼ��LT7381
 * @param       param: ATK-MD0700 V3ģ��IDȷ����LCD����
 * @retval      ��
 */
static void atk_md0700_v3_setup_lt7381_by_param(const atk_md0700_v3_lcd_param_t *param)
{
    uint8_t hback_porch = param->timing.hback_porch;
    uint8_t hfront_porch = param->timing.hfront_porch;
    uint8_t hsync_len = param->timing.hsync_len;
    
    if (hback_porch < 8)
    {
        hback_porch += 8;
    }
    if (hfront_porch < 8)
    {
        hfront_porch = 8;
    }
    if (hsync_len < 8)
    {
        hsync_len = 8;
    }
    
    /* �ȴ�LT7381������������ģʽ */
    while ((atk_md0700_v3_read_stsr() & (1 << 1)) != 0);
    
    /* ����PLL
     * PPLL������PCLK�ṩLCD��Ļɨ�蹤��Ƶ�ʣ����80MHz
     * MPLL������MCLK�ṩ���ڲ��Դ棨SDRAM��ʹ�ã����133MHz
     * CPLL������CCLK�ṩ�������ӿڡ�BTE���桢��ͼ���桢�ı�DMA���ݴ���ȵ�ʹ�ã����100MHz
     * һ������CCLK = MCLK = 2 * PCLK
     */
    atk_md0700_v3_set_reg(0x05, ((2 << 6) | (5 << 1) | ((param->timing.clock_freq_in_mhz >> 8) & 0x01)));   /* PCLK = 10MHz * (clock_freq_in_mhz / 5) / 2 = pixel_freq_in_mhz(MHz) */
    atk_md0700_v3_set_reg(0x06, (param->timing.clock_freq_in_mhz & 0xFF));
    atk_md0700_v3_set_reg(0x07, ((2 << 6) | (5 << 1) | ((133 >> 8) & 0x01)));                               /* MCLK = 10MHz * (133 / 5) / 2 = 133(MHz) */
    atk_md0700_v3_set_reg(0x08, (133 & 0xFF));
    atk_md0700_v3_set_reg(0x09, ((2 << 6) | (5 << 1) | ((100 >> 8) & 0x01)));                               /* CCLK = 10MHz * (100 / 5) / 2 = 100(MHz) */
    atk_md0700_v3_set_reg(0x0A, (100 & 0xFF));
    
    atk_md0700_v3_modify_reg(0x00, 0, (1 << 7));                                                            /* ��������PLL */
    while ((atk_md0700_v3_get_reg(0x01) & (1 << 7)) == 0);                                                  /* �ȴ�PLL���� */
    
    /* �����Դ棨SDRAM�������� */
    atk_md0700_v3_set_reg(0xE0, 0x20);                                                                      /* Bank������4�� �е�ַ����λ����11�� �е�ַ����λ����8�� */
    atk_md0700_v3_set_reg(0xE1, 0x03);                                                                      /* CAS���ʱ�� */
    atk_md0700_v3_set_reg(0xE2, (64000 * (133 * 2)) / 2048);                                                /* ��ˢ��ʱ���� */
    atk_md0700_v3_set_reg(0xE3, ((64000 * (133 * 2)) / 2048) >> 8);
    atk_md0700_v3_modify_reg(0xE4, 0, (1 << 0));                                                            /* ��ʼ��SDRAM */
    while ((atk_md0700_v3_get_reg(0xE4) & (1 << 0)) == 0);
    
    /* оƬ���üĴ��� */
    atk_md0700_v3_modify_reg(0x01, (1 << 6) | (3 << 3), (2 << 3) | (1 << 0));                               /* WAIT#�������� TFT���ӿڣ�16���أ� �����������߿�ȣ�16���أ� */
    
    /* �Դ���ʿ��ƼĴ��� */
    atk_md0700_v3_modify_reg(0x02, (3 << 6), 0);                                                            /* ����ͼ�����ݷ��ʸ�ʽ��16���� 16bpp�� */
    
    /* ������ƼĴ��� */
    atk_md0700_v3_modify_reg(0x03, (1 << 2) | (3 << 0), 0);                                                 /* �ı�ģʽʹ�ܣ�ͼ��ģʽ�� �ڴ����Ŀ�꣨�Դ棩 */
    
    /* ����Main Window */
    atk_md0700_v3_set_reg(0x20, 0 & ~(3 << 0));                                                             /* Main Window��ʼ��ַ */
    atk_md0700_v3_set_reg(0x21, 0 >> 8);
    atk_md0700_v3_set_reg(0x22, 0 >> 16);
    atk_md0700_v3_set_reg(0x23, 0 >> 24);
    atk_md0700_v3_set_reg(0x24, param->width & ~(3 << 0));                                                  /* Main Window��� */
    atk_md0700_v3_set_reg(0x25, param->width >> 8);
    atk_md0700_v3_set_reg(0x26, 0 & ~(3 << 0));                                                             /* Main Window���Ͻ�X���� */
    atk_md0700_v3_set_reg(0x27, 0 >> 8);
    atk_md0700_v3_set_reg(0x28, 0);                                                                         /* Main Window���Ͻ�Y���� */
    atk_md0700_v3_set_reg(0x29, 0 >> 8);
    atk_md0700_v3_modify_reg(0x10, (3 << 2), (1 << 2));                                                     /* Main Windowɫ�16bpp�� */
    
    /* TFT��Ļ������� */
    atk_md0700_v3_modify_reg(0x10, (1 << 0), 0);                                                            /* Sync Mode */
    atk_md0700_v3_modify_reg(0x12, (1 << 7), (1 << 7));                                                     /* PCLK Falling */
    atk_md0700_v3_modify_reg(0x13, (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4), 0);                           /* (HSYNC High) (VSYNC High) (PDE High) (PDE IDLE Low) */
    atk_md0700_v3_set_reg(0x14, (param->timing.hactive / 8) - 1);                                           /* Horizontal Active */
    atk_md0700_v3_set_reg(0x15, (param->timing.hactive % 8));
    atk_md0700_v3_set_reg(0x16, (hback_porch / 8) - 1);                                                     /* Horizontal Back Porch */
    atk_md0700_v3_set_reg(0x17, (hback_porch % 8));
    atk_md0700_v3_set_reg(0x18, (hfront_porch / 8) - 1);                                                    /* Horizontal Front Porch */
    atk_md0700_v3_set_reg(0x19, (hsync_len / 8) - 1);                                                       /* Horizontal Sync Length */
    atk_md0700_v3_set_reg(0x1A, (param->timing.vactive - 1));                                               /* Vertical Active */
    atk_md0700_v3_set_reg(0x1B, (param->timing.vactive - 1) >> 8);
    atk_md0700_v3_set_reg(0x1C, (param->timing.vback_porch - 1));                                           /* Vertical Back Porch */
    atk_md0700_v3_set_reg(0x1D, (param->timing.vback_porch - 1) >> 8);
    atk_md0700_v3_set_reg(0x1E, (param->timing.vfront_porch - 1));                                          /* Vertical Front Porch */
    atk_md0700_v3_set_reg(0x1F, (param->timing.vsync_len - 1));                                             /* Vertical Sync Length */
    
    /* ����Canvas Window */
    atk_md0700_v3_set_reg(0x50, 0 & ~(3 << 0));                                                             /* Canvas Window��ʼ��ַ */
    atk_md0700_v3_set_reg(0x51, 0 >> 8);
    atk_md0700_v3_set_reg(0x52, 0 >> 16);
    atk_md0700_v3_set_reg(0x53, 0 >> 24);
    atk_md0700_v3_set_reg(0x54, param->width & ~(3 << 0));                                                  /* Canvas Window��� */
    atk_md0700_v3_set_reg(0x55, param->width >> 8);
    atk_md0700_v3_modify_reg(0x5E, (1 << 2) | (3 << 0), (1 << 0));                                          /* Canvas WindowѰַģʽ��Blockģʽ��X-Y����Ѱַ���� Canvas Window��Active Windowͼ��ɫ�16bpp�� */
    
    /* ����Active Window */
    atk_md0700_v3_set_reg(0x56, 0);                                                                         /* Active Window���Ͻ�X���� */
    atk_md0700_v3_set_reg(0x57, 0 >> 8);
    atk_md0700_v3_set_reg(0x58, 0);                                                                         /* Active Window���Ͻ�Y���� */
    atk_md0700_v3_set_reg(0x59, 0 >> 8);
    atk_md0700_v3_set_reg(0x5A, param->width);                                                              /* Active Window��� */
    atk_md0700_v3_set_reg(0x5B, param->width >> 8);
    atk_md0700_v3_set_reg(0x5C, param->height);                                                             /* Active Window�߶� */
    atk_md0700_v3_set_reg(0x5D, param->height >> 8);
    
    /* ���ö�дλ������ */
    atk_md0700_v3_set_reg(0x5F, 0);                                                                         /* ��дλ��X���� */
    atk_md0700_v3_set_reg(0x60, 0 >> 8);
    atk_md0700_v3_set_reg(0x61, 0);                                                                         /* ��дλ��Y���� */
    atk_md0700_v3_set_reg(0x62, 0 >> 8);
    
    /* ������ʾ */
    atk_md0700_v3_modify_reg(0x12, 0, (1 << 6));
    
    /* ����PWM���� */
    atk_md0700_v3_set_reg(0x84, (((param->timing.clock_freq_in_mhz * 2) * 1000000) / 500000) - 1);          /* Timerʱ�ӷ�Ƶϵ����Timer1����Ƶ��Ϊ500KHz�� */
    atk_md0700_v3_modify_reg(0x85, (3 << 6) | (3 << 2), 0);                                                 /* Timer1����Ƶ PWM[1]�����PWM */
    atk_md0700_v3_modify_reg(0x86, 0, (1 << 5));                                                            /* Timer1�Զ���װ�� */
    atk_md0700_v3_set_reg(0x8C, 0);                                                                         /* Timer1�����Ƚ�ֵ */
    atk_md0700_v3_set_reg(0x8D, (0 >> 8));
    atk_md0700_v3_set_reg(0x8E, 100);                                                                       /* Timer1��װ��ֵ��PWM[1]Ƶ�� = Timer1����Ƶ�� / Timer1��װ��ֵ = 500KHz / 100 = 5KHz�� */
    atk_md0700_v3_set_reg(0x8F, (100 >> 8));
    atk_md0700_v3_modify_reg(0x86, 0, (1 << 4));                                                            /* Timer1��ʼ���� */
    
    /* �������� */
//    atk_md0700_v3_modify_reg(0x12, 0, (1 << 5));
}

/**
 * @brief       ��ʼ����ATK-MD0700 V3ģ���Դ�
 * @param       ��
 * @retval      ��
 */
void atk_md0700_v3_start_access_memory(void)
{
    ATK_MD0700_V3_FSMC_REG = 0x04;
}

/**
 * @brief       ����ATK-MD0700 V3ģ���Դ������ʼ����
 * @param       ��
 * @retval      ��
 */
void atk_md0700_v3_config_memory_access_coordinate(uint16_t x, uint16_t y)
{
    if (g_atk_md0700_v3_sta.disp_dir == ATK_MD0700_V3_LCD_DISP_DIR_0)
    {
        x = x ^ y;
        y = x ^ y;
        x = x ^ y;
        y = g_atk_md0700_v3_sta.width - y - 1;
    }
    
    atk_md0700_v3_set_reg(0x5F, x);
    atk_md0700_v3_set_reg(0x60, x >> 8);
    atk_md0700_v3_set_reg(0x61, y);
    atk_md0700_v3_set_reg(0x62, y >> 8);
}

/**
 * @brief       ����ATK-MD0700 V3ģ���Դ���ʴ���
 * @param       ��
 * @retval      ��
 */
void atk_md0700_v3_config_memory_access_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height)
{
    if (g_atk_md0700_v3_sta.disp_dir == ATK_MD0700_V3_LCD_DISP_DIR_0)
    {
        width = width ^ height;
        height = width ^ height;
        width = width ^ height;
        sx = sx ^ sy;
        sy = sx ^ sy;
        sx = sx ^ sy;
        sy = g_atk_md0700_v3_sta.width - sy - height;
    }
    atk_md0700_v3_set_reg(0x56, sx);
    atk_md0700_v3_set_reg(0x57, sx >> 8);
    atk_md0700_v3_set_reg(0x58, sy);
    atk_md0700_v3_set_reg(0x59, sy >> 8);
    atk_md0700_v3_set_reg(0x5A, width);
    atk_md0700_v3_set_reg(0x5B, width >> 8);
    atk_md0700_v3_set_reg(0x5C, height);
    atk_md0700_v3_set_reg(0x5D, height >> 8);
    atk_md0700_v3_config_memory_access_coordinate(sx, sy);
}

/**
 * @brief       ƽ��������x^y
 * @param       x: ����
 *              y: ָ��
 * @retval      x^y
 */
static uint32_t atk_md0700_v3_pow(uint8_t x, uint8_t y)
{
    uint8_t loop;
    uint32_t res = 1;
    
    for (loop=0; loop<y; loop++)
    {
        res *= x;
    }
    
    return res;
}

/**
 * @brief       ATK-MD0700 V3ģ���ʼ��
 * @param       ��
 * @retval      ATK_MD0700_V3_EOK  : ATK_MD0700_V3ģ���ʼ���ɹ�
 *              ATK_MD0700_V3_ERROR: ATK_MD0700_V3ģ���ʼ��ʧ��
 */
uint8_t atk_md0700_v3_init(void)
{
    uint8_t id;
    uint8_t ret;
    
    atk_md0700_v3_fsmc_init();                  /* ATK-MD0700 V3ģ��FSMC�ӿڳ�ʼ�� */
    id = atk_md0700_v3_get_id();                /* ��ȡATK-MD0700 V3ģ��ID */
    ret = atk_md0700_v3_setup_param_by_id(id);  /* ATK-MD0700 V3ģ�������ʼ�� */
    if (ret != ATK_MD0700_V3_EOK)
    {
        return ATK_MD0700_V3_ERROR;
    }
    atk_md0700_v3_setup_lt7381_by_param(g_atk_md0700_v3_sta.param);
    
    atk_md0700_v3_set_disp_dir(ATK_MD0700_V3_LCD_DISP_DIR_90);
    atk_md0700_v3_clear(ATK_MD0700_V3_WHITE);
    atk_md0700_v3_backlight_config(100);
    atk_md0700_v3_display_on(); 
    return ATK_MD0700_V3_EOK;
}

/**
 * @brief       ��ȡATK-MD0700 V3ģ��LCD���
 * @param       ��
 * @retval      ATK-MD0700 V3ģ��LCD���
 */
uint16_t atk_md0700_v3_get_lcd_width(void)
{
    return g_atk_md0700_v3_sta.width;
}

/**
 * @brief       ��ȡATK-MD0700 V3ģ��LCD�߶�
 * @param       ��
 * @retval      ATK-MD0700 V3ģ��LCD�߶�
 */
uint16_t atk_md0700_v3_get_lcd_height(void)
{
    return g_atk_md0700_v3_sta.height;
}

/**
 * @brief       ����ATK-MD0700 V3ģ��LCD��������
 * @param       percent: LCD�������Ȱٷֱ�
 * @retval      ��
 */
void atk_md0700_v3_backlight_config(uint8_t percent)
{
    atk_md0700_v3_set_reg(0x8C, percent);               /* LT7381 Timer1�����Ƚ�ֵ */
    atk_md0700_v3_set_reg(0x8D, (percent >> 8));
    if (percent == 0)
    {
        atk_md0700_v3_modify_reg(0x85, (3 << 2), 0);    /* �ر�LT7381 PWM[1]��� */
    }
    else
    {
        atk_md0700_v3_modify_reg(0x85, 0, (2 << 2));    /* ����LT7381 PWM[1]��� */
    }
}

/**
 * @brief       ����ATK-MD0700 V3ģ��LCD��ʾ
 * @param       ��
 * @retval      ��
 */
void atk_md0700_v3_display_on(void)
{
    atk_md0700_v3_modify_reg(0x12, 0, (1 << 6));
}

/**
 * @brief       �ر�ATK-MD0700 V3ģ��LCD��ʾ
 * @param       ��
 * @retval      ��
 */
void atk_md0700_v3_display_off(void)
{
    atk_md0700_v3_modify_reg(0x12, (1 << 6), 0);
}

/**
 * @brief       ����ATK-MD0700 V3ģ��LCDɨ�跽��
 * @note        ����ɨ�跽���Ӱ���Դ��д���ٶȣ�
 *              ��ɨ�跽��ΪATK_MD0700_V3_LCD_SCAN_DIR_U2D_R2L@ATK_MD0700_V3_LCD_DISP_DIR_0��ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D@ATK_MD0700_V3_LCD_DISP_DIR_90ʱ��
 *              FSMC/FMC��WR�ź��������20MHz
 *              ��ɨ�跽��Ϊ����ɨ�跽��ʱ��
 *              FSMC/FMC��WR�ź��������10MHz
 *              ������FSMC/FMC��д���ٶȴ����������ʱ����������д���Դ�ʱ�����״̬�Ĵ���(atk_md0700_v3_read_stsr())��bit6Ϊ1����д������
 * @param       scan_dir: ɨ�跽��
 *                  disp_dirΪATK_MD0700_V3_LCD_DISP_DIR_0ʱ��֧��
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D: �����ң����ϵ���
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D: ���ҵ��󣬴��ϵ���
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_U2D_R2L: ���ϵ��£����ҵ���
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_D2U_R2L: ���µ��ϣ����ҵ���
 *                  disp_dirΪATK_MD0700_V3_LCD_DISP_DIR_90ʱ��֧��
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D: �����ң����ϵ���
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D: ���ҵ��󣬴��ϵ���
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_U2D_L2R: ���ϵ��£�������
 *                      ATK_MD0700_V3_LCD_SCAN_DIR_D2U_L2R: ���µ��ϣ�������
 * @retval      ATK_MD0700_V3_EOK   : ����ATK-MD0700 V3ģ��LCDɨ�跽��ɹ�
 *              ATK_MD0700_V3_EINVAL: �����������
 */
uint8_t atk_md0700_v3_set_scan_dir(uint8_t scan_dir)
{
    if (g_atk_md0700_v3_sta.disp_dir == ATK_MD0700_V3_LCD_DISP_DIR_0)
    {
        switch (scan_dir)
        {
            case ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D:
            {
                scan_dir = ATK_MD0700_V3_LCD_SCAN_DIR_D2U_L2R;
                break;
            }
            case ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D:
            {
                scan_dir = ATK_MD0700_V3_LCD_SCAN_DIR_U2D_L2R;
                break;
            }
            case ATK_MD0700_V3_LCD_SCAN_DIR_U2D_R2L:
            {
                scan_dir = ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D;
                break;
            }
            case ATK_MD0700_V3_LCD_SCAN_DIR_D2U_R2L:
            {
                scan_dir = ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D;
                break;
            }
            default:
            {
                return ATK_MD0700_V3_EINVAL;
            }
        }
    }
    else
    {
        switch (scan_dir)
        {
            case ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D:
            case ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D:
            case ATK_MD0700_V3_LCD_SCAN_DIR_U2D_L2R:
            case ATK_MD0700_V3_LCD_SCAN_DIR_D2U_L2R:
            {
                break;
            }
            default:
            {
                return ATK_MD0700_V3_EINVAL;
            }
        }
    }
    
    switch (scan_dir)
    {
        case ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D:
        {
            atk_md0700_v3_modify_reg(0x02, (3 << 4) | (3 << 1), (0 << 4) | (0 << 1));
            break;
        }
        case ATK_MD0700_V3_LCD_SCAN_DIR_R2L_U2D:
        {
            atk_md0700_v3_modify_reg(0x02, (3 << 4) | (3 << 1), (1 << 4) | (1 << 1));
            break;
        }
        case ATK_MD0700_V3_LCD_SCAN_DIR_U2D_L2R:
        {
            atk_md0700_v3_modify_reg(0x02, (3 << 4) | (3 << 1), (2 << 4) | (2 << 1));
            break;
        }
        case ATK_MD0700_V3_LCD_SCAN_DIR_D2U_L2R:
        {
            atk_md0700_v3_modify_reg(0x02, (3 << 4) | (3 << 1), (3 << 4) | (3 << 1));
            break;
        }
        default:
        {
            return ATK_MD0700_V3_EINVAL;
        }
    }
    
    g_atk_md0700_v3_sta.scan_dir = scan_dir;
    
    return ATK_MD0700_V3_EOK;
}

/**
 * @brief       ����ATK-MD0700 V3ģ��LCD��ʾ����
 * @param       disp_dir: ATK_MD0700_V3_LCD_DISP_DIR_0  : LCD˳ʱ����ת0����ʾ����
 *                        ATK_MD0700_V3_LCD_DISP_DIR_90 : LCD˳ʱ����ת90����ʾ����
 * @retval      ATK_MD0700_V3_EOK   : ����ATK-MD0700 V3ģ��LCD��ʾ����ɹ�
 *              ATK_MD0700_V3_EINVAL: �����������
 */
uint8_t atk_md0700_v3_set_disp_dir(uint8_t disp_dir)
{
    switch (disp_dir)
    {
        case ATK_MD0700_V3_LCD_DISP_DIR_0:
        {
            g_atk_md0700_v3_sta.width = g_atk_md0700_v3_sta.param->height;
            g_atk_md0700_v3_sta.height = g_atk_md0700_v3_sta.param->width;
            break;
        }
        case ATK_MD0700_V3_LCD_DISP_DIR_90:
        {
            g_atk_md0700_v3_sta.width = g_atk_md0700_v3_sta.param->width;
            g_atk_md0700_v3_sta.height = g_atk_md0700_v3_sta.param->height;
            break;
        }
        default:
        {
            return ATK_MD0700_V3_EINVAL;
        }
    }
    
    g_atk_md0700_v3_sta.disp_dir = disp_dir;
    atk_md0700_v3_set_scan_dir(ATK_MD0700_V3_LCD_SCAN_DIR_L2R_U2D);
    
    return ATK_MD0700_V3_EOK;
}

/**
 * @brief       ��ȡATK-MD0700 V3ģ��LCDɨ�跽��
 * @param       ��
 * @retval      ATK-MD0700 V3ģ��LCDɨ�跽��
 */
uint8_t atk_md0700_v3_get_scan_dir(void)
{
    return g_atk_md0700_v3_sta.scan_dir;
}

/**
 * @brief       ��ȡATK-MD0700 V3ģ��LCD��ʾ����
 * @param       ��
 * @retval      ATK-MD0700 V3ģ��LCD��ʾ����
 */
uint8_t atk_md0700_v3_get_disp_dir(void)
{
    return g_atk_md0700_v3_sta.disp_dir;
}

/**
 * @brief       ATK-MD0700 V3ģ��LCD�������
 * @param       xs   : ������ʼX����
 *              ys   : ������ʼY����
 *              xe   : ������ֹX����
 *              ye   : ������ֹY����
 *              color: ���������ɫ
 * @retval      ��
 */
void atk_md0700_v3_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color)
{
    uint16_t window_width;
    uint16_t window_height;
    uint32_t window_size;
    uint32_t window_index;
    
    window_width = ex - sx + 1;
    window_height = ey - sy + 1;
    window_size = window_width * window_height;
    atk_md0700_v3_config_memory_access_window(sx, sy, window_width, window_height);
    atk_md0700_v3_start_access_memory();
    for (window_index=0; window_index<window_size; window_index++)
    {
        /* ����д��ʱ������Դ�д��FiFo����ϸ���atk_md0700_v3_set_scan_dir()������ */
        if ((window_index % 8) == 0)
        {
            while ((atk_md0700_v3_read_stsr() & (1 << 7)) != 0);
            atk_md0700_v3_start_access_memory();
        }
        
        ATK_MD0700_V3_FSMC_DAT = color;
    }
    atk_md0700_v3_config_memory_access_window(0, 0, g_atk_md0700_v3_sta.width, g_atk_md0700_v3_sta.height);
}

/**
 * @brief       ATK-MD0700 V3ģ��LCD����
 * @param       color: ������ɫ
 * @retval      ��
 */
void atk_md0700_v3_clear(uint16_t color)
{
    atk_md0700_v3_fill(0, 0, g_atk_md0700_v3_sta.width - 1, g_atk_md0700_v3_sta.height - 1, color);
}

/**
 * @brief       ATK-MD0700 V3ģ��LCD����
 * @param       x    : �������X����
 *              y    : �������Y����
 *              color: ���������ɫ
 * @retval      ��
 */
void atk_md0700_v3_draw_point(uint16_t x, uint16_t y, volatile uint16_t color)
{
    atk_md0700_v3_config_memory_access_coordinate(x, y);
    atk_md0700_v3_start_access_memory();
    ATK_MD0700_V3_FSMC_DAT = color;
}

/**
 * @brief       ATK-MD0700 V3ģ��LCD����
 * @param       x    : �������X����
 *              y    : �������Y����
 * @retval      ���������ɫ
 */
uint16_t atk_md0700_v3_read_point(uint16_t x, uint16_t y)
{
    volatile uint16_t color;
    
    if ((x >= g_atk_md0700_v3_sta.width) || (y >= g_atk_md0700_v3_sta.height))
    {
        return ATK_MD0700_V3_EINVAL;
    }
    
    atk_md0700_v3_config_memory_access_coordinate(x, y);
    atk_md0700_v3_start_access_memory();
    
    color = ATK_MD0700_V3_FSMC_DAT;
    delay_us(2);
    color = ATK_MD0700_V3_FSMC_DAT;
    
    return color;
}

/**
 * @brief       ATK-MD0700 V3ģ��LCD���߶�
 * @param       x1   : �����߶ζ˵�1��X����
 *              y1   : �����߶ζ˵�1��Y����
 *              x2   : �����߶ζ˵�2��X����
 *              y2   : �����߶ζ˵�2��Y����
 *              color: �����߶ε���ɫ
 * @retval      ��
 */
void atk_md0700_v3_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t x_delta;
    uint16_t y_delta;
    int16_t x_sign;
    int16_t y_sign;
    int16_t error;
    int16_t error2;
    
    x_delta = (x1 < x2) ? (x2 - x1) : (x1 - x2);
    y_delta = (y1 < y2) ? (y2 - y1) : (y1 - y2);
    x_sign = (x1 < x2) ? 1 : -1;
    y_sign = (y1 < y2) ? 1 : -1;
    error = x_delta - y_delta;
    
    atk_md0700_v3_draw_point(x2, y2, color);
    
    while ((x1 != x2) || (y1 != y2))
    {
        atk_md0700_v3_draw_point(x1, y1, color);
        
        error2 = error << 1;
        if (error2 > -y_delta)
        {
            error -= y_delta;
            x1 += x_sign;
        }
        
        if (error2 < x_delta)
        {
            error += x_delta;
            y1 += y_sign;
        }
    }
}

/**
 * @brief       ATK-MD0700 V3ģ��LCD�����ο�
 * @param       x1   : �������ο�˵�1��X����
 *              y1   : �������ο�˵�1��Y����
 *              x2   : �������ο�˵�2��X����
 *              y2   : �������ο�˵�2��Y����
 *              color: �������ο����ɫ
 * @retval      ��
 */
void atk_md0700_v3_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    atk_md0700_v3_draw_line(x1, y1, x2, y1, color);
    atk_md0700_v3_draw_line(x1, y2, x2, y2, color);
    atk_md0700_v3_draw_line(x1, y1, x1, y2, color);
    atk_md0700_v3_draw_line(x2, y1, x2, y2, color);
}

/**
 * @brief       ATK-MD0700 V3ģ��LCD��Բ�ο�
 * @param       x    : ����Բ�ο�ԭ���X����
 *              y    : ����Բ�ο�ԭ���Y����
 *              r    : ����Բ�ο�İ뾶
 *              color: ����Բ�ο����ɫ
 * @retval      ��
 */
void atk_md0700_v3_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
    int32_t x_t;
    int32_t y_t;
    int32_t error;
    int32_t error2;
    
    x_t = -r;
    y_t = 0;
    error = 2 - 2 * r;
    
    do {
        atk_md0700_v3_draw_point(x - x_t, y + y_t, color);
        atk_md0700_v3_draw_point(x + x_t, y + y_t, color);
        atk_md0700_v3_draw_point(x + x_t, y - y_t, color);
        atk_md0700_v3_draw_point(x - x_t, y - y_t, color);
        
        error2 = error;
        if (error2 <= y_t)
        {
            y_t++;
            error = error + (y_t * 2 + 1);
            if ((-x_t == y_t) && (error2 <= x_t))
            {
                error2 = 0;
            }
        }
        
        if (error2 > x_t)
        {
            x_t++;
            error = error + (x_t * 2 + 1);
        }
    } while (x_t <= 0);
}

/**
 * @brief       ATK-MD0700 V3ģ��LCD��Բ��
 * @param       x    : ����Բ��ԭ���X����
 *              y    : ����Բ��ԭ���Y����
 *              r    : ����Բ�εİ뾶
 *              color: ����Բ�ε���ɫ
 * @retval      ��
 */
void atk_md0700_v3_fill_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
    uint32_t i;
    uint32_t imax = ((uint32_t)r * 707) / 1000 + 1;
    uint32_t sqmax = (uint32_t)r * (uint32_t)r + (uint32_t)r / 2;
    uint32_t xr = r;
    
    atk_md0700_v3_draw_line(x - r, y, x + r, y, color);

    for (i = 1; i <= imax; i++)
    {
        if ((i * i + xr * xr) > sqmax)
        {
            /* draw lines from outside */
            if (xr > imax)
            {
                atk_md0700_v3_draw_line(x - i + 1, y + xr, x + i - 1, y + xr, color);
                atk_md0700_v3_draw_line(x - i + 1, y - xr, x + i - 1, y - xr, color);
            }

            xr--;
        }

        /* draw lines from inside (center) */
        atk_md0700_v3_draw_line(x - xr, y + i, x + xr, y + i, color);
        atk_md0700_v3_draw_line(x - xr, y - i, x + xr, y - i, color);
    }
}

/**
 * @brief       ATK-MD0700 V3ģ��LCD��ʾ1���ַ�
 * @param       x    : ����ʾ�ַ���X����
 *              y    : ����ʾ�ַ���Y����
 *              ch   : ����ʾ�ַ�
 *              font : ����ʾ�ַ�������
 *              color: ����ʾ�ַ�����ɫ
 * @retval      ��
 */
void atk_md0700_v3_show_char(uint16_t x, uint16_t y, char ch, uint8_t font, uint16_t color)
{
    const uint8_t *ch_code;
    uint8_t ch_width;
    uint8_t ch_height;
    uint8_t ch_size;
    uint8_t ch_offset;
    uint8_t byte_index;
    uint8_t byte_code;
    uint8_t bit_index;
    uint8_t width_index = 0;
    uint8_t height_index = 0;
    
    ch_offset = ch - ' ';
    
    switch (font)
    {
#if (ATK_MD0700_V3_FONT_12 != 0)
        case ATK_MD0700_V3_LCD_FONT_12:
        {
            ch_code = atk_md0700_v3_font_1206[ch_offset];
            ch_width = ATK_MD0700_V3_FONT_12_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_12_CHAR_HEIGHT;
            ch_size = ATK_MD0700_V3_FONT_12_CHAR_SIZE;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_16 != 0)
        case ATK_MD0700_V3_LCD_FONT_16:
        {
            ch_code = atk_md0700_v3_font_1608[ch_offset];
            ch_width = ATK_MD0700_V3_FONT_16_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_16_CHAR_HEIGHT;
            ch_size = ATK_MD0700_V3_FONT_16_CHAR_SIZE;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_24 != 0)
        case ATK_MD0700_V3_LCD_FONT_24:
        {
            ch_code = atk_md0700_v3_font_2412[ch_offset];
            ch_width = ATK_MD0700_V3_FONT_24_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_24_CHAR_HEIGHT;
            ch_size = ATK_MD0700_V3_FONT_24_CHAR_SIZE;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_32 != 0)
        case ATK_MD0700_V3_LCD_FONT_32:
        {
            ch_code = atk_md0700_v3_font_3216[ch_offset];
            ch_width = ATK_MD0700_V3_FONT_32_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_32_CHAR_HEIGHT;
            ch_size = ATK_MD0700_V3_FONT_32_CHAR_SIZE;
            break;
        }
#endif
        default:
        {
            return;
        }
    }
    
    if ((x + ch_width > g_atk_md0700_v3_sta.width) || (y + ch_height > g_atk_md0700_v3_sta.height))
    {
        return;
    }
    
    for (byte_index=0; byte_index<ch_size; byte_index++)
    {
        byte_code = ch_code[byte_index];
        for (bit_index=0; bit_index<8; bit_index++)
        {
            if ((byte_code & 0x80) != 0)
            {
                atk_md0700_v3_draw_point(x + width_index, y + height_index, color);
            }
            height_index++;
            if (height_index == ch_height)
            {
                height_index = 0;
                width_index++;
                break;
            }
            byte_code <<= 1;
        }
    }
}

/**
 * @brief       ATK-MD0700 V3ģ��LCD��ʾ�ַ���
 * @note        ���Զ����кͻ�ҳ
 * @param       x     : ����ʾ�ַ�����X����
 *              y     : ����ʾ�ַ�����Y����
 *              width : ����ʾ�ַ�������ʾ�߶�
 *              height: ����ʾ�ַ�������ʾ���
 *              str   : ����ʾ�ַ���
 *              font  : ����ʾ�ַ���������
 *              color : ����ʾ�ַ�������ɫ
 * @retval      ��
 */
void atk_md0700_v3_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, char *str, uint8_t font, uint16_t color)
{
    uint8_t ch_width;
    uint8_t ch_height;
    uint16_t x_raw;
    uint16_t y_raw;
    uint16_t x_limit;
    uint16_t y_limit;
    
    switch (font)
    {
#if (ATK_MD0700_V3_FONT_12 != 0)
        case ATK_MD0700_V3_LCD_FONT_12:
        {
            ch_width = ATK_MD0700_V3_FONT_12_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_12_CHAR_HEIGHT;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_16 != 0)
        case ATK_MD0700_V3_LCD_FONT_16:
        {
            ch_width = ATK_MD0700_V3_FONT_16_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_16_CHAR_HEIGHT;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_24 != 0)
        case ATK_MD0700_V3_LCD_FONT_24:
        {
            ch_width = ATK_MD0700_V3_FONT_24_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_24_CHAR_HEIGHT;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_32 != 0)
        case ATK_MD0700_V3_LCD_FONT_32:
        {
            ch_width = ATK_MD0700_V3_FONT_32_CHAR_WIDTH;
            ch_height = ATK_MD0700_V3_FONT_32_CHAR_HEIGHT;
            break;
        }
#endif
        default:
        {
            return;
        }
    }
    
    x_raw = x;
    y_raw = y;
    x_limit = ((x + width + 1) > g_atk_md0700_v3_sta.width) ? g_atk_md0700_v3_sta.width : (x + width + 1);
    y_limit = ((y + height + 1) > g_atk_md0700_v3_sta.height) ? g_atk_md0700_v3_sta.height : (y + height + 1);
    
    while ((*str >= ' ') && (*str <= '~'))
    {
        if (x + ch_width > x_limit)
        {
            x = x_raw;
            y += ch_height;
        }
        
        if (y + ch_height > y_limit)
        {
            y = x_raw;
            x = y_raw;
        }
        
        atk_md0700_v3_show_char(x, y, *str, font, color);
        
        x += ch_width;
        str++;
    }
}

/**
 * @brief       ATK-MD0700 V3ģ��LCD��ʾ���֣��ɿ�����ʾ��λ0
 * @param       x    : ����ʾ���ֵ�X����
 *              y    : ����ʾ���ֵ�Y����
 *              num  : ����ʾ����
 *              len  : ����ʾ���ֵ�λ��
 *              mode : ATK_MD0700_NUM_SHOW_NOZERO: ���ָ�λ0����ʾ
 *                     ATK_MD0700_NUM_SHOW_ZERO  : ���ָ�λ0��ʾ
 *              font : ����ʾ���ֵ�����
 *              color: ����ʾ���ֵ���ɫ
 * @retval      ��
 */
void atk_md0700_v3_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t mode, uint8_t font, uint16_t color)
{
    uint8_t ch_width;
    uint8_t len_index;
    uint8_t num_index;
    uint8_t first_nozero = 0;
    char pad;
    
    switch (font)
    {
#if (ATK_MD0700_V3_FONT_12 != 0)
        case ATK_MD0700_V3_LCD_FONT_12:
        {
            ch_width = ATK_MD0700_V3_FONT_12_CHAR_WIDTH;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_16 != 0)
        case ATK_MD0700_V3_LCD_FONT_16:
        {
            ch_width = ATK_MD0700_V3_FONT_16_CHAR_WIDTH;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_24 != 0)
        case ATK_MD0700_V3_LCD_FONT_24:
        {
            ch_width = ATK_MD0700_V3_FONT_24_CHAR_WIDTH;
            break;
        }
#endif
#if (ATK_MD0700_V3_FONT_32 != 0)
        case ATK_MD0700_V3_LCD_FONT_32:
        {
            ch_width = ATK_MD0700_V3_FONT_32_CHAR_WIDTH;
            break;
        }
#endif
        default:
        {
            return;
        }
    }
    
    switch (mode)
    {
        case ATK_MD0700_V3_NUM_SHOW_NOZERO:
        {
            pad = ' ';
            break;
        }
        case ATK_MD0700_V3_NUM_SHOW_ZERO:
        {
            pad = '0';
            break;
        }
        default:
        {
            return;
        }
    }
    
    for (len_index=0; len_index<len; len_index++)
    {
        num_index = (num / atk_md0700_v3_pow(10, len - len_index - 1)) % 10;
        if ((first_nozero == 0) && (len_index < (len - 1)))
        {
            if (num_index == 0)
            {
                atk_md0700_v3_show_char(x + ch_width * len_index, y, pad, font, color);
                continue;
            }
            else
            {
                first_nozero = 1;
            }
        }
        
        atk_md0700_v3_show_char(x + ch_width * len_index, y, num_index + '0', font, color);
    }
}

/**
 * @brief       ATK-MD0700 V3ģ��LCD��ʾ���֣�����ʾ��λ0
 * @param       x    : ����ʾ���ֵ�X����
 *              y    : ����ʾ���ֵ�Y����
 *              num  : ����ʾ����
 *              len  : ����ʾ���ֵ�λ��
 *              font : ����ʾ���ֵ�����
 *              color: ����ʾ���ֵ���ɫ
 * @retval      ��
 */
void atk_md0700_v3_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t font, uint16_t color)
{
    atk_md0700_v3_show_xnum(x, y, num, len, ATK_MD0700_V3_NUM_SHOW_NOZERO, font, color);
}

/**
 * @brief       ATK-MD0700 V3ģ��LCDͼƬ
 * @note        ͼƬȡģ��ʽ: ˮƽɨ�衢RGB565����λ��ǰ
 * @param       x     : ����ʾͼƬ��X����
 *              y     : ����ʾͼƬ��Y����
 *              width : ����ʾͼƬ�Ŀ��
 *              height: ����ʾͼƬ�ĸ߶�
 *              pic   : ����ʾͼƬ�����׵�ַ
 * @retval      ��
 */
void atk_md0700_v3_show_pic(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *pic)
{
    uint32_t window_size;
    uint32_t window_index;
    
    window_size = width * height;
    atk_md0700_v3_config_memory_access_window(x, y, width, height);
    atk_md0700_v3_start_access_memory();
    for (window_index=0; window_index<window_size; window_index++, pic++)
    {
        /* ����д��ʱ������Դ�д��FiFo����ϸ���atk_md0700_v3_set_scan_dir()������ */
        if ((window_index % 8) == 0)
        {
            while ((atk_md0700_v3_read_stsr() & (1 << 7)) != 0);
            atk_md0700_v3_start_access_memory();
        }
        
        ATK_MD0700_V3_FSMC_DAT = *pic;
    }
    atk_md0700_v3_config_memory_access_window(0, 0, g_atk_md0700_v3_sta.width, g_atk_md0700_v3_sta.height);
}
