/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "bf0_hal.h"
#include "drv_lcd.h"
#include "mem_section.h"
#include "bf0_hal_epic.h"


/* 输入层参数 */
#define FG_W        150
#define FG_H        100
#define FG_X        50
#define FG_Y        50

#define BG_W        150
#define BG_H        100
#define BG_X        100
#define BG_Y        100

/*
 * EPIC 输出区域：覆盖 fg 和 bg 的并集
 * fg 区域: x=[50, 199], y=[50, 149]
 * bg 区域: x=[100, 249], y=[100, 199]
 * 并集:    x=[50, 249], y=[50, 199] → 宽200, 高150
 */
#define OUT_X       0
#define OUT_Y       0
#define OUT_W       250
#define OUT_H       200

/*
 * 输入层 buffer 大小 = 输出区域大小 (OUT_W x OUT_H)，
 * 用 HAL_EPIC_LayerSetDataOffset 从中裁剪出实际图层区域。
 */
#define INPUT_BUF_SIZE (OUT_W * OUT_H * 2)

/* 输出 buffer 保持全屏大小，供 LCD 送显 */
#define SCREEN_BUF_SIZE (LCD_HOR_RES_MAX * LCD_VER_RES_MAX * 2)

// 分配显存空间
L2_NON_RET_BSS_SECT_BEGIN(epic_buffers)
L2_NON_RET_BSS_SECT(epic_buffers,
                    ALIGN(64) static uint8_t buffer0[INPUT_BUF_SIZE]);
L2_NON_RET_BSS_SECT(epic_buffers,
                    ALIGN(64) static uint8_t buffer1[INPUT_BUF_SIZE]);
L2_NON_RET_BSS_SECT(epic_buffers,
                    ALIGN(64) static uint8_t buffer2[SCREEN_BUF_SIZE]);
L2_NON_RET_BSS_SECT_END

static EPIC_HandleTypeDef epic_handle;
static EZIP_HandleTypeDef ezip_handle;


void lcd_display_update(uint8_t *buffer)
{
    rt_kprintf("in lcd_display...\n");
    rt_device_t lcd_dev = rt_device_find("lcd");
    if (!lcd_dev)
    {
        rt_kprintf("Failed to find LCD device\n");
        return;
    }

    if (rt_device_open(lcd_dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Failed to open LCD device\n");
        return;
    }

    uint16_t buffer_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
    rt_device_control(lcd_dev, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &buffer_format);

    struct rt_device_graphic_info info;
    rt_memset(&info, 0, sizeof(info));
    rt_device_control(lcd_dev, RTGRAPHIC_CTRL_GET_INFO, &info);

    rt_kprintf("LCD Info: Width=%d, Height=%d, BPP=%d\n",
               info.width, info.height, info.bits_per_pixel);

    struct rt_device_graphic_ops *ops = rt_graphix_ops(lcd_dev);
    if (ops && ops->draw_rect_async)
    {
        mpu_dcache_clean(buffer, SCREEN_BUF_SIZE);
        ops->set_window(0, 0, LCD_HOR_RES_MAX - 1, LCD_VER_RES_MAX - 1);
        ops->draw_rect_async((const char *)buffer, 0, 0,
                             LCD_HOR_RES_MAX - 1, LCD_VER_RES_MAX - 1);
        rt_kprintf("draw_rect_async called\n");
    }
    else
    {
        rt_kprintf("draw_rect_async not available!\n");
    }
}

void epic_blend_layers(void)
{
    rt_kprintf("Starting EPIC blend demo...\n");

    // Layer 1: 蓝色图层
    EPIC_LayerConfigTypeDef fg_layer;
    HAL_EPIC_LayerConfigInit(&fg_layer);
    fg_layer.data = buffer0;
    fg_layer.color_mode = EPIC_COLOR_RGB565;
    fg_layer.width = FG_W;
    fg_layer.height = FG_H;
    fg_layer.x_offset = FG_X;
    fg_layer.y_offset = FG_Y;
    fg_layer.total_width = OUT_W;      /* buffer 行宽 = 输出区域宽度 */
    fg_layer.color_en = false;
    fg_layer.alpha = 128;
    fg_layer.ax_mode = ALPHA_BLEND_RGBCOLOR;
    /* 将 data 指针偏移到 buffer0 中 (FG_X, FG_Y) 位置，从该处开始读取像素 */
    HAL_EPIC_LayerSetDataOffset((EPIC_BlendingDataType *)&fg_layer, FG_X, FG_Y);

    // Layer 2: 红色图层
    EPIC_LayerConfigTypeDef bg_layer;
    HAL_EPIC_LayerConfigInit(&bg_layer);
    bg_layer.data = buffer1;
    bg_layer.color_mode = EPIC_COLOR_RGB565;
    bg_layer.width = BG_W;
    bg_layer.height = BG_H;
    bg_layer.x_offset = BG_X;
    bg_layer.y_offset = BG_Y;
    bg_layer.total_width = OUT_W;      /* buffer 行宽 = 输出区域宽度 */
    bg_layer.color_en = false;
    bg_layer.alpha = 128;
    bg_layer.ax_mode = ALPHA_BLEND_RGBCOLOR;
    /* 将 data 指针偏移到 buffer1 中 (BG_X, BG_Y) 位置 */
    HAL_EPIC_LayerSetDataOffset((EPIC_BlendingDataType *)&bg_layer, BG_X, BG_Y);

    EPIC_LayerConfigTypeDef input_layers[] = {fg_layer, bg_layer};
    uint8_t input_layer_num =
        sizeof(input_layers) / sizeof(EPIC_LayerConfigTypeDef);

    //输出层：EPIC 只写入全屏 buffer 中 (0,0)-(249,199) 的小区域。

    EPIC_LayerConfigTypeDef output_layer;
    HAL_EPIC_LayerConfigInit(&output_layer);
    output_layer.data = buffer2;
    output_layer.color_mode = EPIC_COLOR_RGB565;
    output_layer.width = OUT_W;
    output_layer.height = OUT_H;
    output_layer.x_offset = OUT_X;
    output_layer.y_offset = OUT_Y;
    output_layer.total_width = LCD_HOR_RES_MAX;  /* 全屏行宽，匹配 buffer2 布局 */
    output_layer.color_en = true;
    output_layer.color_r = 0x00;
    output_layer.color_g = 0x00;
    output_layer.color_b = 0x00;
    output_layer.alpha = 255;

    // 混合操作
    HAL_StatusTypeDef ret = HAL_EPIC_BlendStartEx(
                                &epic_handle, input_layers, input_layer_num, &output_layer);
    if (ret != HAL_OK)
    {
        rt_kprintf("EPIC blend failed (%d)\n", ret);
    }
    else
    {
        rt_kprintf("EPIC blend succeeded\n");
        lcd_display_update(buffer2);
    }
}

int epic_demo_init(void)
{
    rt_memset(buffer0, 0, INPUT_BUF_SIZE);
    rt_memset(buffer1, 0, INPUT_BUF_SIZE);
    rt_memset(buffer2, 0, SCREEN_BUF_SIZE);

    mpu_dcache_clean(buffer0, INPUT_BUF_SIZE);
    mpu_dcache_clean(buffer1, INPUT_BUF_SIZE);
    mpu_dcache_clean(buffer2, SCREEN_BUF_SIZE);

    // 初始化，包括中断等资源
    HAL_NVIC_SetPriority(EPIC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EPIC_IRQn);

    epic_handle.hezip = &ezip_handle;
    epic_handle.hezip->Instance = EZIP;
    HAL_EZIP_Init(epic_handle.hezip);
    epic_handle.Instance = hwp_epic;
    HAL_EPIC_Init(&epic_handle);
    rt_kprintf("HAL_EPIC_Init ok\n");

    /* 用 EPIC 填充蓝色到 buffer0 (250x200 整个区域) */
    EPIC_FillingCfgTypeDef fill_cfg;
    HAL_EPIC_FillDataInit(&fill_cfg);
    fill_cfg.start = buffer0;
    fill_cfg.color_mode = EPIC_COLOR_RGB565;
    fill_cfg.width = OUT_W;
    fill_cfg.height = OUT_H;
    fill_cfg.total_width = OUT_W;
    fill_cfg.color_r = 0x00;
    fill_cfg.color_g = 0x00;
    fill_cfg.color_b = 0xFF;
    fill_cfg.alpha = 255;

    HAL_StatusTypeDef ret = HAL_EPIC_FillStart(&epic_handle, &fill_cfg);
    if (ret != HAL_OK)
    {
        rt_kprintf("Fill blue rect failed\n");
        return -1;
    }

    /* 用 EPIC 填充红色到 buffer1 (250x200 整个区域) */
    fill_cfg.start = buffer1;
    fill_cfg.color_r = 0xFF;
    fill_cfg.color_g = 0x00;
    fill_cfg.color_b = 0x00;
    ret = HAL_EPIC_FillStart(&epic_handle, &fill_cfg);
    if (ret != HAL_OK)
    {
        rt_kprintf("Fill red rect failed\n");
        return -1;
    }

    // 开始混合
    epic_blend_layers();

    return 0;
}

int main(int argc, char *argv[])
{
    rt_kprintf("Application started\n");
    epic_demo_init();
    while (1)
    {
        rt_thread_mdelay(1000);
    }
    return 0;
}
