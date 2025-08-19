/**
 ****************************************************************************************************
 * @file        ltdc.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-09-06
 * @brief       LTDC 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 阿波罗 H743开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20220906
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "./BSP/LCD/ltdc.h"
#include "./BSP/LCD/lcd.h"
#include "./SYSTEM/delay/delay.h"


LTDC_HandleTypeDef  g_ltdc_handle;       /* LTDC句柄 */
DMA2D_HandleTypeDef g_dma2d_handle;      /* DMA2D句柄 */

#if !(__ARMCC_VERSION >= 6010050)                                                     /* 不是AC6编译器，即使用AC5编译器时 */

/* 根据不同的颜色格式,定义帧缓存数组 */
#if LTDC_PIXFORMAT == LTDC_PIXFORMAT_ARGB8888 || LTDC_PIXFORMAT == LTDC_PIXFORMAT_RGB888
    uint32_t ltdc_lcd_framebuf[1280][800] __attribute__((at(LTDC_FRAME_BUF_ADDR)));   /* 定义最大屏分辨率时,LTDC所需的帧缓存数组大小 */
#else
    uint16_t ltdc_lcd_framebuf[1280][800] __attribute__((at(LTDC_FRAME_BUF_ADDR)));   /* 定义最大屏分辨率时,LTDC所需的帧缓存数组大小 */
//    uint16_t ltdc_lcd_framebuf[1280][800] __attribute__((at(LTDC_FRAME_BUF_ADDR + 1280 * 800 * 2)));/* 使用LTDC层2时使用（默认使用LTDC层1） */
#endif

#else      /* 使用AC6编译器时 */

/* 根据不同的颜色格式,定义帧缓存数组 */
#if LTDC_PIXFORMAT == LTDC_PIXFORMAT_ARGB8888 || LTDC_PIXFORMAT == LTDC_PIXFORMAT_RGB888
    uint32_t ltdc_lcd_framebuf[1280][800] __attribute__((section(".bss.ARM.__at_0XC0000000")));  /* 定义最大屏分辨率时,LTDC所需的帧缓存数组大小 */
#else
    uint16_t ltdc_lcd_framebuf[1280][800] __attribute__((section(".bss.ARM.__at_0XC0000000")));  /* 定义最大屏分辨率时,LTDC所需的帧缓存数组大小 */
#endif

#endif

uint32_t *g_ltdc_framebuf[2];              /* LTDC LCD帧缓存数组指针,必须指向对应大小的内存区域 */
_ltdc_dev lcdltdc;                         /* 管理LCD LTDC的重要参数 */

/**
 * @brief       LTDC开关
 * @param       sw   : 1 打开,0，关闭
 * @retval      无
 */
void ltdc_switch(uint8_t sw)
{
    if (sw)
    {
        __HAL_LTDC_ENABLE(&g_ltdc_handle);
    }
    else
    {
        __HAL_LTDC_DISABLE(&g_ltdc_handle);
    }
}

/**
 * @brief       LTDC开关指定层
 * @param       layerx       : 0,第一层; 1,第二层
 * @param       sw           : 1 打开;   0关闭
 * @retval      无
 */
void ltdc_layer_switch(uint8_t layerx, uint8_t sw)
{
    if (sw) 
    {
        __HAL_LTDC_LAYER_ENABLE(&g_ltdc_handle, layerx);
    }
    else
    {
        __HAL_LTDC_LAYER_DISABLE(&g_ltdc_handle, layerx);
    }

    __HAL_LTDC_RELOAD_CONFIG(&g_ltdc_handle);
}

/**
 * @brief       LTDC选择层
 * @param       layerx   : 层号;0,第一层; 1,第二层;
 * @retval      无
 */
void ltdc_select_layer(uint8_t layerx)
{
    lcdltdc.activelayer = layerx;
}

/**
 * @brief       LTDC显示方向设置
 * @param       dir          : 0,竖屏；1,横屏
 * @retval      无
 */
void ltdc_display_dir(uint8_t dir)
{
    lcdltdc.dir = dir;    /* 显示方向 */

    if (dir == 0)         /* 竖屏 */
    {
        lcdltdc.width = lcdltdc.pheight;
        lcdltdc.height = lcdltdc.pwidth;
    }
    else if (dir == 1)    /* 横屏 */
    {
        lcdltdc.width = lcdltdc.pwidth;
        lcdltdc.height = lcdltdc.pheight;
    }
}

/**
 * @brief       LTDC画点函数
 * @param       x,y         : 写入坐标
 * @param       color       : 颜色值
 * @retval      无
 */
void ltdc_draw_point(uint16_t x, uint16_t y, uint32_t color)
{ 
#if LTDC_PIXFORMAT == LTDC_PIXFORMAT_ARGB8888 || LTDC_PIXFORMAT == LTDC_PIXFORMAT_RGB888
    if (lcdltdc.dir)   /* 横屏 */
    {
        *(uint32_t *)((uint32_t)g_ltdc_framebuf[lcdltdc.activelayer] + lcdltdc.pixsize * (lcdltdc.pwidth * y + x)) = color;
    }
    else               /* 竖屏 */
    {
        *(uint32_t *)((uint32_t)g_ltdc_framebuf[lcdltdc.activelayer] + lcdltdc.pixsize * (lcdltdc.pwidth * (lcdltdc.pheight - x - 1) + y)) = color; 
    }
#else
    if (lcdltdc.dir)   /* 横屏 */
    {
        *(uint16_t *)((uint32_t)g_ltdc_framebuf[lcdltdc.activelayer] + lcdltdc.pixsize * (lcdltdc.pwidth * y + x)) = color;
    }
    else              /* 竖屏 */
    {
        *(uint16_t *)((uint32_t)g_ltdc_framebuf[lcdltdc.activelayer] + lcdltdc.pixsize * (lcdltdc.pwidth * (lcdltdc.pheight - x - 1) + y)) = color; 
    }
#endif
}

/**
 * @brief       LTDC读点函数
 * @param       x,y       : 读取点的坐标
 * @retval      返回值:颜色值
 */
uint32_t ltdc_read_point(uint16_t x, uint16_t y)
{ 
#if LTDC_PIXFORMAT == LTDC_PIXFORMAT_ARGB8888 || LTDC_PIXFORMAT == LTDC_PIXFORMAT_RGB888
    if (lcdltdc.dir)   /* 横屏 */
    {
        return *(uint32_t *)((uint32_t)g_ltdc_framebuf[lcdltdc.activelayer] + lcdltdc.pixsize * (lcdltdc.pwidth * y + x));
    }
    else               /* 竖屏 */
    {
        return *(uint32_t *)((uint32_t)g_ltdc_framebuf[lcdltdc.activelayer] + lcdltdc.pixsize * (lcdltdc.pwidth * (lcdltdc.pheight - x - 1) + y)); 
    }
#else
    if (lcdltdc.dir)   /* 横屏 */
    {
        return *(uint16_t *)((uint32_t)g_ltdc_framebuf[lcdltdc.activelayer] + lcdltdc.pixsize * (lcdltdc.pwidth * y + x));
    }
    else               /* 竖屏 */
    {
        return *(uint16_t *)((uint32_t)g_ltdc_framebuf[lcdltdc.activelayer] + lcdltdc.pixsize * (lcdltdc.pwidth * (lcdltdc.pheight - x - 1) + y)); 
    }
#endif 
}

/**
 * @brief       LTDC填充矩形,DMA2D填充
 * @note       (sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex - sx + 1) * (ey - sy + 1)
 *              注意:sx,ex,不能大于lcddev.width - 1; sy,ey,不能大于lcddev.height - 1
 * @param       sx,sy       : 起始坐标
 * @param       ex,ey       : 结束坐标
 * @param       color       : 填充的颜色
 * @retval      无
 */
void ltdc_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color)
{ 
    uint32_t psx, psy, pex, pey;        /* 以LCD面板为基准的坐标系,不随横竖屏变化而变化 */
    uint32_t timeout = 0; 
    uint16_t offline;
    uint32_t addr; 

    /* 坐标系转换 */
    if (lcdltdc.dir)                    /* 横屏 */
    {
        psx = sx;
        psy = sy;
        pex = ex;
        pey = ey;
    }
    else                                /* 竖屏 */
    {
        if (ex >= lcdltdc.pheight)
        {
            ex = lcdltdc.pheight - 1;   /* 限制范围 */
        }

        if (sx >= lcdltdc.pheight)
        {
            sx = lcdltdc.pheight - 1;   /* 限制范围 */
        }
        psx = sy;
        psy = lcdltdc.pheight - ex - 1;
        pex = ey;
        pey = lcdltdc.pheight - sx - 1;
    }

    offline = lcdltdc.pwidth - (pex - psx + 1);
    addr = ((uint32_t)g_ltdc_framebuf[lcdltdc.activelayer] + lcdltdc.pixsize * (lcdltdc.pwidth * psy + psx));

    __HAL_RCC_DMA2D_CLK_ENABLE();                              /* 使能DM2D时钟 */

    DMA2D->CR &= ~(DMA2D_CR_START);                            /* 先停止DMA2D */
    DMA2D->CR = DMA2D_R2M;                                     /* 寄存器到存储器模式 */
    DMA2D->OPFCCR = LTDC_PIXFORMAT;                            /* 设置颜色格式 */
    DMA2D->OOR = offline;                                      /* 设置行偏移  */

    DMA2D->OMAR = addr;                                        /* 输出存储器地址 */
    DMA2D->NLR = (pey - psy + 1) | ((pex - psx + 1) << 16);    /* 设定行数寄存器 */
    DMA2D->OCOLR = color;                                      /* 设定输出颜色寄存器 */
    DMA2D->CR |= DMA2D_CR_START;                               /* 启动DMA2D */

    while ((DMA2D->ISR & (DMA2D_FLAG_TC)) == 0)                /* 等待传输完成 */
    {
        timeout++;
        if (timeout > 0X1FFFFF)break;                          /* 超时退出 */
    } 
    DMA2D->IFCR |= DMA2D_FLAG_TC;                              /* 清除传输完成标志 */
}

///* 使用DMA2D相关的HAL函数使用DMA2D外设(不推荐) */
///**
// * @brief       在指定区域内填充单个颜色
// * @param       (sx,sy),(ex,ey)  : 填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)
// * @param       color            : 要填充的颜色
// * @retval      无
// */
//void ltdc_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color)
//{
//    uint32_t psx,psy,pex,pey;   /* 以LCD面板为基准的坐标系,不随横竖屏变化而变化 */
//    uint32_t timeout = 0; 
//    uint16_t offline;
//    uint32_t addr;  
//
//    if (ex >= lcdltdc.width)
//    {
//        ex = lcdltdc.width - 1;
//    }
//
//    if (ey >= lcdltdc.height)
//    {
//        ey = lcdltdc.height - 1;
//    }
//
//    /* 坐标系转换 */
//    if (lcdltdc.dir)   /* 横屏 */
//    {
//        psx = sx; 
//        psy = sy;
//        pex = ex;
//        pey = ey;
//    }
//    else             /* 竖屏 */
//    {
//        psx = sy;
//        psy = lcdltdc.pheight - ex - 1;
//        pex = ey;
//        pey = lcdltdc.pheight - sx - 1;
//    }

//    offline = lcdltdc.pwidth - (pex - psx + 1);
//    addr =((uint32_t)g_ltdc_framebuf[lcdltdc.activelayer] + lcdltdc.pixsize * (lcdltdc.pwidth * psy + psx));
//
//    if (LTDC_PIXFORMAT == LTDC_PIXEL_FORMAT_RGB565)  /* 如果是RGB565格式的话需要进行颜色转换，将16bit转换为32bit的 */
//    {
//        color = ((color & 0X0000F800) << 8) | ((color & 0X000007E0) << 5 ) | ((color & 0X0000001F) << 3);
//    }
//
//    /* 配置DMA2D的模式 */
//    g_dma2d_handle.Instance = DMA2D;
//    g_dma2d_handle.Init.Mode = DMA2D_R2M;                                                    /* 内存到存储器模式 */
//    g_dma2d_handle.Init.ColorMode = LTDC_PIXFORMAT;                                          /* 设置颜色格式 */
//    g_dma2d_handle.Init.OutputOffset = offline;                                              /* 输出偏移  */
//    HAL_DMA2D_Init(&g_dma2d_handle);                                                         /* 初始化DMA2D */
//
//    HAL_DMA2D_ConfigLayer(&g_dma2d_handle, lcdltdc.activelayer);                             /* 层配置 */
//    HAL_DMA2D_Start(&g_dma2d_handle, color,(uint32_t)addr,pex - psx + 1,pey - psy + 1);      /* 开启传输 */
//    HAL_DMA2D_PollForTransfer(&g_dma2d_handle, 1000);                                        /* 传输数据 */
//
//    while((__HAL_DMA2D_GET_FLAG( &g_dma2d_handle,DMA2D_FLAG_TC) == 0) && (timeout < 0X5000)) /* 等待DMA2D完成 */
//    {
//        timeout++;
//    }
//    __HAL_DMA2D_CLEAR_FLAG(&g_dma2d_handle,DMA2D_FLAG_TC);                                   /* 清除传输完成标志 */
//}

/**
 * @brief       在指定区域内填充指定颜色块,DMA2D填充
 * @note        此函数仅支持uint16_t,RGB565格式的颜色数组填充.
 *              (sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex - sx + 1) * (ey - sy + 1)
 *              注意:sx,ex,不能大于lcddev.width - 1; sy,ey,不能大于lcddev.height - 1
 * @param       sx,sy       : 起始坐标
 * @param       ex,ey       : 结束坐标
 * @param       color       : 填充的颜色数组首地址
 * @retval      无
 */
void ltdc_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color)
{
    uint32_t psx, psy, pex, pey;   /* 以LCD面板为基准的坐标系,不随横竖屏变化而变化 */
    uint32_t timeout = 0; 
    uint16_t offline;
    uint32_t addr;
  
    /* 坐标系转换 */
    if (lcdltdc.dir)               /* 横屏 */
    {
        psx = sx;
        psy = sy;
        pex = ex;
        pey = ey;
    }
    else                          /* 竖屏 */
    {
        psx = sy;
        psy = lcdltdc.pheight - ex - 1;
        pex = ey;
        pey = lcdltdc.pheight - sx - 1;
    }

    offline = lcdltdc.pwidth - (pex - psx + 1);
    addr = ((uint32_t)g_ltdc_framebuf[lcdltdc.activelayer] + lcdltdc.pixsize * (lcdltdc.pwidth * psy + psx));

    __HAL_RCC_DMA2D_CLK_ENABLE();                             /* 使能DM2D时钟 */

    DMA2D->CR &= ~(DMA2D_CR_START);                           /* 先停止DMA2D */
    DMA2D->CR = DMA2D_M2M;                                    /* 存储器到存储器模式 */
    DMA2D->FGPFCCR = LTDC_PIXFORMAT;                          /* 设置颜色格式 */
    DMA2D->FGOR = 0;                                          /* 前景层行偏移为0 */
    DMA2D->OOR = offline;                                     /* 设置行偏移 */

    DMA2D->FGMAR = (uint32_t)color;                           /* 源地址 */
    DMA2D->OMAR = addr;                                       /* 输出存储器地址 */
    DMA2D->NLR = (pey - psy + 1) | ((pex - psx + 1) << 16);   /* 设定行数寄存器 */
    DMA2D->CR |= DMA2D_CR_START;                              /* 启动DMA2D */

    while((DMA2D->ISR & (DMA2D_FLAG_TC)) == 0)                /* 等待传输完成 */
    {
        timeout++;
        if (timeout > 0X1FFFFF)break;                         /* 超时退出 */
    } 
    DMA2D->IFCR |= DMA2D_FLAG_TC;                             /* 清除传输完成标志 */
}  

/**
 * @brief       LTCD清屏
 * @param       color          : 颜色值
 * @retval      无
 */
void ltdc_clear(uint32_t color)
{
    ltdc_fill(0, 0, lcdltdc.width - 1, lcdltdc.height - 1, color);
}

/**
 * @brief       LTDC时钟(Fdclk)设置函数
 * @param       pll3n     : PLL3倍频系数(PLL倍频),           取值范围:4~512.
 * @param       pll3m     : PLL3预分频系数(进PLL之前的分频),  取值范围:2~63.
 * @param       pll3r     : PLL3的r分频系数(PLL之后的分频),   取值范围:1~128.
 *
 * @note        Fvco = Fs * (pll3n / pll3m);
 *              Fr = Fvco / pll3r = Fs * (pll3n / (pll3m * pll3r));
 *              Fdclk = Fr;
 *              其中:
 *              Fvco: VCO频率
 *              Fr: PLL3的r分频输出时钟频率
 *              Fs: PLL3输入时钟频率,可以是HSI,CSI,HSE等.
 *
 *              LTDC,我们一般设置pll3m = 25, pll3n = 300, 这样,可以得到Fvco = 300Mhz
 *              然后,只 需要通过修改pll3r, 来匹配不同的液晶屏时序即可.

 *              假设:外部晶振为25M, pllm = 25 的时候, Fs = 25Mhz， pllm分频后频率 为1Mhz.
 *              例如: 要得到33M的LTDC时钟, 则可以设置: pll3n = 300, pllm = 25, pll3r = 9
 *              Fdclk= ((25 / 25) * 300) / 9 = 33 Mhz
 * @retval      0, 成功;
 *              其他, 失败;
 */
uint8_t ltdc_clk_set(uint32_t pll3n, uint32_t pll3m, uint32_t pll3r)
{
    RCC_PeriphCLKInitTypeDef periphclk_initure;

    /* LTDC输出像素时钟，需要根据自己所使用的LCD数据手册来配置！ */
    periphclk_initure.PeriphClockSelection = RCC_PERIPHCLK_LTDC;     /* LTDC时钟 */
    periphclk_initure.PLL3.PLL3M = pll3m;
    periphclk_initure.PLL3.PLL3N = pll3n;
    periphclk_initure.PLL3.PLL3P = 2;
    periphclk_initure.PLL3.PLL3Q = 2;
    periphclk_initure.PLL3.PLL3R = pll3r;

    if (HAL_RCCEx_PeriphCLKConfig(&periphclk_initure) == HAL_OK)     /* 配置像素时钟 */
    {
        return 0;                                                    /* 成功 */
    }
    else 
    {
        return 1;                                                    /* 失败 */
    }
}

/**
 * @brief       LTDC层窗口设置, 窗口以LCD面板坐标系为基准
 * @note        此函数必须在ltdc_layer_parameter_config之后再设置.另外,当设置的窗口值不等于面板的尺
 *              寸时,GRAM的操作(读/写点函数),也要根据窗口的宽高来进行修改,否则显示不正常(本例程就未做修改).
 * @param       layerx      : 0,第一层; 1,第二层;
 * @param       sx, sy      : 起始坐标
 * @param       width,height: 宽度和高度
 * @retval      无
 */
void ltdc_layer_window_config(uint8_t layerx, uint16_t sx, uint16_t sy, uint16_t width, uint16_t height)
{
    HAL_LTDC_SetWindowPosition(&g_ltdc_handle, sx, sy, layerx);     /* 设置窗口的位置 */
    HAL_LTDC_SetWindowSize(&g_ltdc_handle, width, height, layerx);  /* 设置窗口大小 */
}

/**
 * @brief       LTDC层基本参数设置
 * @note        此函数,必须在ltdc_layer_window_config之前设置.
 * @param       layerx      : 0,第一层; 1,第二层;
 * @param       bufaddr     : 层颜色帧缓存起始地址
 * @param       pixformat   : 颜色格式. 0,ARGB8888; 1,RGB888; 2,RGB565; 3,ARGB1555; 4,ARGB4444; 5,L8; 6;AL44; 7;AL88
 * @param       alpha       : 层颜色Alpha值, 0,全透明;255,不透明
 * @param       alpha0      : 默认颜色Alpha值, 0,全透明;255,不透明
 * @param       bfac1       : 混合系数1, 4(100),恒定的Alpha; 6(101),像素Alpha*恒定Alpha
 * @param       bfac2       : 混合系数2, 5(101),恒定的Alpha; 7(111),像素Alpha*恒定Alpha
 * @param       bkcolor     : 层默认颜色,32位,低24位有效,RGB888格式
 * @retval      无
 */
void ltdc_layer_parameter_config(uint8_t layerx, uint32_t bufaddr, uint8_t pixformat, uint8_t alpha, uint8_t alpha0, uint8_t bfac1, uint8_t bfac2, uint32_t bkcolor)
{
    LTDC_LayerCfgTypeDef playercfg;

    playercfg.WindowX0 = 0;                                            /* 窗口起始X坐标 */
    playercfg.WindowY0 = 0;                                            /* 窗口起始Y坐标 */
    playercfg.WindowX1 = lcdltdc.pwidth;                               /* 窗口终止X坐标 */
    playercfg.WindowY1 = lcdltdc.pheight;                              /* 窗口终止Y坐标 */
    playercfg.PixelFormat = pixformat;                                 /* 像素格式 */
    playercfg.Alpha = alpha;                                           /* Alpha值设置，0~255,255为完全不透明 */
    playercfg.Alpha0 = alpha0;                                         /* 默认Alpha值 */
    playercfg.BlendingFactor1 = (uint32_t)bfac1 << 8;                  /* 设置层混合系数 */
    playercfg.BlendingFactor2 = (uint32_t)bfac2;                       /* 设置层混合系数 */
    playercfg.FBStartAdress = bufaddr;                                 /* 设置层颜色帧缓存起始地址 */
    playercfg.ImageWidth = lcdltdc.pwidth;                             /* 设置颜色帧缓冲区的宽度 */
    playercfg.ImageHeight = lcdltdc.pheight;                           /* 设置颜色帧缓冲区的高度 */
    playercfg.Backcolor.Red = (uint8_t)(bkcolor & 0X00FF0000) >> 16;   /* 背景颜色红色部分 */
    playercfg.Backcolor.Green = (uint8_t)(bkcolor & 0X0000FF00) >> 8;  /* 背景颜色绿色部分 */
    playercfg.Backcolor.Blue = (uint8_t)bkcolor & 0X000000FF;          /* 背景颜色蓝色部分 */
    HAL_LTDC_ConfigLayer(&g_ltdc_handle, &playercfg, layerx);          /* 设置所选中的层 */
}  

/**
 * @brief       LTDC读取面板ID
 * @note        利用LCD RGB线的最高位(R7,G7,B7)来识别面板ID
 *              PG6 = R7(M0); PI2 = G7(M1); PI7 = B7(M2);
 *              M2:M1:M0
 *              0 :0 :0     4.3 寸480*272  RGB屏,ID = 0X4342
 *              0 :0 :1     7   寸800*480  RGB屏,ID = 0X7084
 *              0 :1 :0     7   寸1024*600 RGB屏,ID = 0X7016
 *              0 :1 :1     7   寸1280*800 RGB屏,ID = 0X7018
 *              1 :0 :0     4.3 寸800*480  RGB屏,ID = 0X4384
 *              1 :0 :1     10.1寸1280*800 RGB屏,ID = 0X1018
 * @param       无
 * @retval      0, 非法; 
 *              其他, LCD ID
 */
uint16_t ltdc_panelid_read(void)
{
    uint8_t idx = 0;

    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_GPIOG_CLK_ENABLE();                               /* 使能GPIOG时钟 */
    __HAL_RCC_GPIOI_CLK_ENABLE();                               /* 使能GPIOI时钟 */
    
    gpio_init_struct.Pin = GPIO_PIN_6;                          /* PG6 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_HIGH;                   /* 高速 */
    HAL_GPIO_Init(GPIOG, &gpio_init_struct);                    /* 初始化 */
    
    gpio_init_struct.Pin = GPIO_PIN_2 | GPIO_PIN_7;             /* PI2,7 */
    HAL_GPIO_Init(GPIOI, &gpio_init_struct);                    /* 初始化 */

    delay_us(2);
    idx = (uint8_t)HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6);         /* 读取M0 */
    idx|= (uint8_t)HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_2) << 1;    /* 读取M1 */
    idx|= (uint8_t)HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_7) << 2;    /* 读取M2 */

    switch (idx)
    {
        case 0 :
            return 0X4342;                    /* 4.3寸屏,480*272分辨率 */
        
        case 1 :
            return 0X7084;                    /* 7  寸屏,800*480分辨率 */
        
        case 2 :
            return 0X7016;                    /* 7  寸屏,1024*600分辨率 */
        
        case 3 :
            return 0X7018;                    /* 7  寸屏,1280*800分辨率 */
        
        case 4 :
            return 0X4384;                    /* 4.3寸屏,800*480分辨率 */
        
        case 5 :
            return 0X1018;                    /* 10.1寸屏,1280*800分辨率 */
        
        default :
            return 0;
    }
}

/**
 * @brief       LTDC初始化函数
 * @param       无
 * @retval      无
 */
void ltdc_init(void)
{
    uint16_t lcdid = 0;

    lcdid = ltdc_panelid_read();                /* 读取LCD面板ID */
    if (lcdid == 0X4342)
    {
        lcdltdc.pwidth = 480;                   /* 面板宽度,单位:像素 */
        lcdltdc.pheight = 272;                  /* 面板高度,单位:像素 */
        lcdltdc.hsw = 1;                        /* 水平同步宽度 */
        lcdltdc.vsw = 1;                        /* 垂直同步宽度 */
        lcdltdc.hbp = 40;                       /* 水平后廊 */
        lcdltdc.vbp = 8;                        /* 垂直后廊 */
        lcdltdc.hfp = 5;                        /* 水平前廊 */
        lcdltdc.vfp = 8;                        /* 垂直前廊 */
        ltdc_clk_set(300, 25, 33);              /* 设置像素时钟 9Mhz */
        /* 其他参数待定. */
    }
    else if (lcdid == 0X7084)
    {
        lcdltdc.pwidth = 800;                   /* 面板宽度,单位:像素 */
        lcdltdc.pheight = 480;                  /* 面板高度,单位:像素 */
        lcdltdc.hsw = 1;                        /* 水平同步宽度 */
        lcdltdc.vsw = 1;                        /* 垂直同步宽度 */
        lcdltdc.hbp = 46;                       /* 水平后廊 */
        lcdltdc.vbp = 23;                       /* 垂直后廊 */
        lcdltdc.hfp = 210;                      /* 水平前廊 */
        lcdltdc.vfp = 22;                       /* 垂直前廊 */
        ltdc_clk_set(300, 25, 9);               /* 设置像素时钟 33M(如果开双显,需要降低DCLK到:18.75Mhz  300/4/4,才会比较好) */
    }
    else if (lcdid == 0X7016)
    {
        lcdltdc.pwidth = 1024;                  /* 面板宽度,单位:像素 */
        lcdltdc.pheight = 600;                  /* 面板高度,单位:像素 */
        lcdltdc.hsw = 20;                       /* 水平同步宽度 */
        lcdltdc.vsw = 3;                        /* 垂直同步宽度 */
        lcdltdc.hbp = 140;                      /* 水平后廊 */
        lcdltdc.vbp = 20;                       /* 垂直后廊 */
        lcdltdc.hfp = 160;                      /* 水平前廊 */
        lcdltdc.vfp = 12;                       /* 垂直前廊 */
        ltdc_clk_set(300, 25, 6);               /* 设置像素时钟  40Mhz 6 */
        /* 其他参数待定.*/
    }
    else if (lcdid == 0X7018)
    {
        lcdltdc.pwidth = 1280;                  /* 面板宽度,单位:像素 */
        lcdltdc.pheight = 800;                  /* 面板高度,单位:像素 */
        /* 其他参数待定.*/
    }
    else if (lcdid == 0X4384)
    {
        lcdltdc.pwidth = 800;                   /* 面板宽度,单位:像素 */
        lcdltdc.pheight = 480;                  /* 面板高度,单位:像素 */
        lcdltdc.hbp = 88;                       /* 水平后廊 */
        lcdltdc.hfp = 40;                       /* 水平前廊 */
        lcdltdc.hsw = 48;                       /* 水平同步宽度 */
        lcdltdc.vbp = 32;                       /* 垂直后廊 */
        lcdltdc.vfp = 13;                       /* 垂直前廊 */
        lcdltdc.vsw = 3;                        /* 垂直同步宽度 */
        ltdc_clk_set(300, 25, 9);               /* 设置像素时钟 33M */
        /* 其他参数待定. */
    }
    else if (lcdid == 0X1018)                   /* 10.1寸1280*800 RGB屏 */
    {
        lcdltdc.pwidth = 1280;                  /* 面板宽度,单位:像素 */
        lcdltdc.pheight = 800;                  /* 面板高度,单位:像素 */
        lcdltdc.hbp = 140;                      /* 水平后廊 */
        lcdltdc.hfp = 10;                       /* 水平前廊 */
        lcdltdc.hsw = 10;                       /* 水平同步宽度 */
        lcdltdc.vbp = 10;                       /* 垂直后廊 */
        lcdltdc.vfp = 10;                       /* 垂直前廊 */
        lcdltdc.vsw = 3;                        /* 垂直同步宽度 */
        ltdc_clk_set(300, 25, 6);               /* 设置像素时钟  45Mhz */
    } 

    lcddev.width = lcdltdc.pwidth;
    lcddev.height = lcdltdc.pheight;
    
#if LTDC_PIXFORMAT == LTDC_PIXFORMAT_ARGB8888 || LTDC_PIXFORMAT == LTDC_PIXFORMAT_RGB888 
    g_ltdc_framebuf[0] = (uint32_t*) &ltdc_lcd_framebuf;
    lcdltdc.pixsize = 4;                        /* 每个像素占4个字节 */
#else 
    lcdltdc.pixsize = 2;                        /* 每个像素占2个字节 */
    g_ltdc_framebuf[0] = (uint32_t*)&ltdc_lcd_framebuf;
//    g_ltdc_framebuf[1] = (uint32_t*)&ltdc_lcd_framebuf1;
#endif 
    /* LTDC配置 */
    g_ltdc_handle.Instance = LTDC;
    g_ltdc_handle.Init.HSPolarity = LTDC_HSPOLARITY_AL;         /* 水平同步极性 */
    g_ltdc_handle.Init.VSPolarity = LTDC_VSPOLARITY_AL;         /* 垂直同步极性 */
    g_ltdc_handle.Init.DEPolarity = LTDC_DEPOLARITY_AL;         /* 数据使能极性 */
    g_ltdc_handle.State = HAL_LTDC_STATE_RESET;
    
    if (lcdid == 0X1018)
    {
        g_ltdc_handle.Init.PCPolarity = LTDC_PCPOLARITY_IIPC;   /* 像素时钟极性 */
    }
    else 
    {
        g_ltdc_handle.Init.PCPolarity = LTDC_PCPOLARITY_IPC;    /* 像素时钟极性 */
    }

    g_ltdc_handle.Init.HorizontalSync = lcdltdc.hsw - 1;                                            /* 水平同步宽度 */
    g_ltdc_handle.Init.VerticalSync = lcdltdc.vsw - 1;                                              /* 垂直同步宽度 */
    g_ltdc_handle.Init.AccumulatedHBP = lcdltdc.hsw + lcdltdc.hbp - 1;                              /* 水平同步后沿宽度 */
    g_ltdc_handle.Init.AccumulatedVBP = lcdltdc.vsw + lcdltdc.vbp - 1;                              /* 垂直同步后沿高度 */
    g_ltdc_handle.Init.AccumulatedActiveW = lcdltdc.hsw + lcdltdc.hbp + lcdltdc.pwidth - 1;         /* 有效宽度 */
    g_ltdc_handle.Init.AccumulatedActiveH = lcdltdc.vsw + lcdltdc.vbp + lcdltdc.pheight - 1;        /* 有效高度 */
    g_ltdc_handle.Init.TotalWidth = lcdltdc.hsw + lcdltdc.hbp + lcdltdc.pwidth + lcdltdc.hfp - 1;   /* 总宽度 */
    g_ltdc_handle.Init.TotalHeigh = lcdltdc.vsw + lcdltdc.vbp + lcdltdc.pheight + lcdltdc.vfp - 1;  /* 总高度 */
    g_ltdc_handle.Init.Backcolor.Red = 0;                                                           /* 屏幕背景层红色部分 */
    g_ltdc_handle.Init.Backcolor.Green = 0;                                                         /* 屏幕背景层绿色部分 */
    g_ltdc_handle.Init.Backcolor.Blue = 0;                                                          /* 屏幕背景色蓝色部分 */
    HAL_LTDC_Init(&g_ltdc_handle);

    /* 层配置 */
    ltdc_layer_parameter_config(0, (uint32_t)g_ltdc_framebuf[0], LTDC_PIXFORMAT, 255, 0, 6, 7, 0X000000);   /* 层参数配置 */
    ltdc_layer_window_config(0, 0, 0, lcdltdc.pwidth, lcdltdc.pheight);                                     /* 层窗口配置,以LCD面板坐标系为基准,不要随便修改! */

    //ltdc_layer_parameter_config(1,(uint32_t)g_ltdc_framebuf[1],LTDC_PIXFORMAT,127,0,6,7,0X000000);        /* 层参数配置 */
    //ltdc_layer_window_config(1,0,0,lcdltdc.pwidth,lcdltdc.pheight);                                       /* 层窗口配置,以LCD面板坐标系为基准,不要随便修改! */

//    ltdc_display_dir(0);                 /* 默认竖屏 */
    ltdc_select_layer(0);                  /* 选择第1层 */
    LTDC_BL(1);                            /* 点亮背光 */
    ltdc_clear(0XFFFFFFFF);                /* 清屏 */
}

/**
 * @brief       LTDC底层IO初始化和时钟使能
 * @note        此函数会被HAL_LTDC_Init()调用
 * @param       hltdc:LTDC句柄
 * @retval      无
 */
void HAL_LTDC_MspInit(LTDC_HandleTypeDef *hltdc)
{
    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_LTDC_CLK_ENABLE();                      /* 使能LTDC时钟 */
    __HAL_RCC_DMA2D_CLK_ENABLE();                     /* 使能DMA2D时钟 */

    /* 以下是LTDC信号控制引脚 BL/DE/VSYNC/HSYNC/CLK等的配置 */
    LTDC_BL_GPIO_CLK_ENABLE();                        /* LTDC_BL脚时钟使能 */
    LTDC_DE_GPIO_CLK_ENABLE();                        /* LTDC_DE脚时钟使能 */
    LTDC_VSYNC_GPIO_CLK_ENABLE();                     /* LTDC_VSYNC脚时钟使能 */
    LTDC_HSYNC_GPIO_CLK_ENABLE();                     /* LTDC_HSYNC脚时钟使能 */
    LTDC_CLK_GPIO_CLK_ENABLE();                       /* LTDC_CLK脚时钟使能 */
    __HAL_RCC_GPIOH_CLK_ENABLE();                     /* GPIOH时钟使能 */
    
    /* 初始化PB5，背光引脚 */
    gpio_init_struct.Pin = LTDC_BL_GPIO_PIN;          /* LTDC_BL引脚模式设置 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;      /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;              /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_HIGH;         /* 高速 */
    HAL_GPIO_Init(LTDC_BL_GPIO_PORT, &gpio_init_struct);
    
    gpio_init_struct.Pin = LTDC_DE_GPIO_PIN;          /* LTDC_DE引脚模式设置 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;          /* 复用 */
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_HIGH;
    gpio_init_struct.Alternate = GPIO_AF14_LTDC;      /* 复用为LTDC */
    HAL_GPIO_Init(LTDC_DE_GPIO_PORT, &gpio_init_struct);
    
    gpio_init_struct.Pin = LTDC_VSYNC_GPIO_PIN;       /* LTDC_VSYNC引脚模式设置 */
    HAL_GPIO_Init(LTDC_VSYNC_GPIO_PORT, &gpio_init_struct);
    
    gpio_init_struct.Pin = LTDC_HSYNC_GPIO_PIN;       /* LTDC_HSYNC引脚模式设置 */
    HAL_GPIO_Init(LTDC_HSYNC_GPIO_PORT, &gpio_init_struct);
    
    gpio_init_struct.Pin = LTDC_CLK_GPIO_PIN;         /* LTDC_CLK引脚模式设置 */
    HAL_GPIO_Init(LTDC_CLK_GPIO_PORT, &gpio_init_struct);

    /* 初始化PG6,11 */
    gpio_init_struct.Pin = GPIO_PIN_6 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOG, &gpio_init_struct);
    
    /* 初始化PH9,10,11,12,13,14,15 */
    gpio_init_struct.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | \
                     GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOH, &gpio_init_struct);
    
    /* 初始化PI0,1,2,4,5,6,7 */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5| \
                     GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOI, &gpio_init_struct); 
}

