/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdint.h>
#include "drv_lcd_private.h"

#define FB_WIDTH            200
#define FB_HEIGHT           228
#define REFRESH_INTERVAL_MS 16
#define LCD_BRIGHTNESS      100

#define COLOR_RED   0x00FF0000
#define COLOR_GREEN 0x0000FF00

static struct rt_device_graphic_info lcd_info;
static rt_device_t lcd_device = RT_NULL;

static uint32_t offset_x;
static uint32_t offset_y;

static uint8_t *fb_red   = RT_NULL;
static uint8_t *fb_green = RT_NULL;

static uint32_t make_color(uint16_t cf, uint32_t rgb888)
{
    uint8_t r = (rgb888 >> 16) & 0xFF;
    uint8_t g = (rgb888 >> 8)  & 0xFF;
    uint8_t b = (rgb888 >> 0)  & 0xFF;

    if (cf == RTGRAPHIC_PIXEL_FORMAT_RGB565)
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
    else
        return (r << 16) | (g << 8) | b;
}

/* Fill the framebuffer with a solid color */
static void fill_buffer_fast(void *buf, uint32_t total_bytes, uint16_t cf, uint32_t rgb888)
{
    uint32_t color = make_color(cf, rgb888);

    if (cf == RTGRAPHIC_PIXEL_FORMAT_RGB565)
    {
        /* RGB565: 2 bytes/pixel, write 2 pixels (4 bytes) per iteration */
        uint32_t dual_pixel = (color << 16) | color;
        uint32_t *p32 = (uint32_t *)buf;
        uint32_t count = total_bytes / 4;
        uint32_t i;

        for (i = 0; i < count; i++)
            p32[i] = dual_pixel;
    }
    else
    {
        /* RGB888: fill pixel by pixel */
        uint8_t *p = (uint8_t *)buf;
        uint32_t pixels = total_bytes / 3;
        uint32_t i;

        for (i = 0; i < pixels; i++)
        {
            *p++ = (color >> 0)  & 0xFF;
            *p++ = (color >> 8)  & 0xFF;
            *p++ = (color >> 16) & 0xFF;
        }
    }
}

/* Clear the full screen to black */
static void clear_screen_black(uint32_t buf_bytes)
{
    uint32_t pixel_size = (lcd_info.bits_per_pixel == 16) ? 2 : 3;
    uint32_t strip_h    = buf_bytes / (lcd_info.width * pixel_size);
    uint32_t y, h;

    memset(fb_red, 0, buf_bytes);
    SCB_CleanDCache();

    for (y = 0; y < lcd_info.height; y += strip_h)
    {
        h = (y + strip_h > lcd_info.height) ? (lcd_info.height - y) : strip_h;
        rt_graphix_ops(lcd_device)->set_window(0, y, lcd_info.width - 1, y + h - 1);
        rt_graphix_ops(lcd_device)->draw_rect((const char *)fb_red, 0, y, lcd_info.width - 1, y + h - 1);
        rt_thread_mdelay(5);
    }
}

static void send_frame(const char *fb)
{
    rt_graphix_ops(lcd_device)->set_window(offset_x, offset_y, offset_x + FB_WIDTH - 1, offset_y + FB_HEIGHT - 1);
    rt_graphix_ops(lcd_device)->draw_rect(fb, offset_x, offset_y, offset_x + FB_WIDTH - 1, offset_y + FB_HEIGHT - 1);
}

static void lcd_init(void)
{
    rt_err_t result;
    uint16_t cf;
    uint8_t brightness;

    lcd_device = rt_device_find("lcd");
    RT_ASSERT(lcd_device);

    result = rt_device_open(lcd_device, RT_DEVICE_OFLAG_RDWR);
    RT_ASSERT(result == RT_EOK);

    result = rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_INFO, &lcd_info);
    RT_ASSERT(result == RT_EOK);

    cf = (lcd_info.bits_per_pixel == 16)
         ? RTGRAPHIC_PIXEL_FORMAT_RGB565
         : RTGRAPHIC_PIXEL_FORMAT_RGB888;
    rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &cf);

    offset_x = (lcd_info.width > FB_WIDTH) ? (lcd_info.width - FB_WIDTH) / 2 : 0;
    offset_y = (lcd_info.height > FB_HEIGHT) ? (lcd_info.height - FB_HEIGHT) / 2 : 0;

    brightness = LCD_BRIGHTNESS;
    rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BRIGHTNESS, &brightness);

    rt_kprintf("LCD: %dx%d %dbpp, center region: %dx%d, offset: (%d,%d)\n",
               lcd_info.width, lcd_info.height, lcd_info.bits_per_pixel, FB_WIDTH, FB_HEIGHT,
               offset_x, offset_y);
}

int main(void)
{
    uint32_t pixel_size;
    uint32_t buf_bytes;
    uint16_t cf;
    const char *fb[2];
    uint8_t idx;

    rt_pm_request(PM_SLEEP_MODE_IDLE);
    HAL_LPAON_Sleep();

    /* Device and LCD initialization */
    lcd_init();

    /* Dynamically allocate framebuffers from PSRAM memheap */
    pixel_size = (lcd_info.bits_per_pixel == 16) ? 2 : 3;
    buf_bytes  = FB_WIDTH * FB_HEIGHT * pixel_size;
    cf         = (lcd_info.bits_per_pixel == 16)
                 ? RTGRAPHIC_PIXEL_FORMAT_RGB565
                 : RTGRAPHIC_PIXEL_FORMAT_RGB888;

    fb_red   = (uint8_t *)rt_malloc(buf_bytes);
    fb_green = (uint8_t *)rt_malloc(buf_bytes);
    RT_ASSERT(fb_red && fb_green);

    /* Clear screen */
    clear_screen_black(buf_bytes);

    HAL_RCC_DisableModule(RCC_MOD_EPIC);
    HAL_RCC_DisableModule(RCC_MOD_EZIP);

    fb[0] = (const char *)fb_red;
    fb[1] = (const char *)fb_green;

    /* Refresh frames */
    idx = 0;
    while (1)
    {
        rt_tick_t frame_start;
        rt_tick_t frame_end;
        rt_tick_t elapsed_ms;

        frame_start = rt_tick_get_millisecond();

        if (idx == 0)
            fill_buffer_fast(fb_red, buf_bytes, cf, COLOR_RED);
        else
            fill_buffer_fast(fb_green, buf_bytes, cf, COLOR_GREEN);

        send_frame(fb[idx]);

        frame_end = rt_tick_get_millisecond();
        elapsed_ms = frame_end - frame_start;

        idx ^= 1;

        if (elapsed_ms < REFRESH_INTERVAL_MS)
            rt_thread_mdelay(REFRESH_INTERVAL_MS - elapsed_ms);
    }

    return RT_EOK;
}
