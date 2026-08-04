/*
 * SPDX-FileCopyrightText: 2025-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "drivers/rt_drv_pwm.h"
#include "drv_rgbled.h"

static rt_uint32_t
    led_color_buffer[BSP_RGB_LED_COUNT];    // Static buffer for DMA safety

#define RGB_COLOR   (0x00ff00)
#define RGBLED_NAME    "rgbled"

typedef enum {
    RGB_ANIM_STATIC = 0,
    RGB_ANIM_BREATHING,
    RGB_ANIM_BLINK,
    RGB_ANIM_RAINBOW,
    RGB_ANIM_FADE,
    RGB_ANIM_MODE_COUNT
} rgb_animation_mode_t;

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness;
    rgb_animation_mode_t mode;
    uint32_t period; // ms
    uint32_t repeat_times; // 0 = infinite
} rgb_led_params_t;

static struct rt_device *rgbled_device = RT_NULL;

#define STRESS_STACK_SIZE       1024
#define MATH_THREAD_COUNT       10
#define MATH_THREAD_PRIO_BASE   21

struct rt_color {
    char *color_name;
    uint32_t color;
};

static struct rt_color rgb_color_arry[] = {
    {"black", 0x000000}, {"blue", 0x0000ff}, {"green", 0x00ff00},
    {"cyan", 0x00ffff}, {"red", 0xff0000}, {"purple", 0xff00ff},
    {"yellow", 0xffff00}, {"white", 0xffffff}
};

static float simple_sin(int degrees)
{
    float radians = degrees * 3.14159f / 180.0f;
    float x = radians;
    float x2 = x * x;
    float x3 = x2 * x;
    float x5 = x3 * x2;
    return x - (x3 / 6.0f) + (x5 / 120.0f);
}

static uint32_t apply_brightness(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    if (brightness >= 100)
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    uint32_t scaled_r = (r * brightness) / 100;
    uint32_t scaled_g = (g * brightness) / 100;
    uint32_t scaled_b = (b * brightness) / 100;
    return (scaled_r << 16) | (scaled_g << 8) | scaled_b;
}

static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t region, remainder, p, q, t;
    if (s == 0) { *r = v; *g = v; *b = v; return; }
    region = h / 43;
    remainder = (h - (region * 43)) * 6;
    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
    switch (region) {
    case 0: *r = v; *g = t; *b = p; break;
    case 1: *r = q; *g = v; *b = p; break;
    case 2: *r = p; *g = v; *b = t; break;
    case 3: *r = p; *g = q; *b = v; break;
    case 4: *r = t; *g = p; *b = v; break;
    default: *r = v; *g = p; *b = q; break;
    }
}


void rgb_led_init(void)
{
#ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA32, GPTIM2_CH1, PIN_NOPULL, 1);   // pwmt2_cc1
#elif defined SF32LB58X
    HAL_PIN_Set(PAD_PB39, GPTIM3_CH4, PIN_NOPULL, 0);   // pwmt3_cc4
#elif defined SF32LB56X
    HAL_PIN_Set(PAD_PB25, GPTIM3_CH4, PIN_NOPULL, 0);//566   pwm4_cc4
#endif
    /*rgbled poweron*/
#ifdef SF32LB52X
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO3_3V3, true, true);
#endif
    rgbled_device = rt_device_find(RGBLED_NAME);//find rgb
    if (!rgbled_device)
    {
        RT_ASSERT(0);
    }
}

static void rgb_led_set_color(uint32_t color)
{
#ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA32, GPTIM2_CH1, PIN_NOPULL, 1);   // RGB LED 52x  pwm3_cc1
#elif defined SF32LB58X
    HAL_PIN_Set(PAD_PB39, GPTIM3_CH4, PIN_NOPULL, 0);//58x          pwm4_cc4
#elif defined SF32LB56X
    HAL_PIN_Set(PAD_PB25, GPTIM3_CH4, PIN_NOPULL, 0);//566
#endif

    for (int i = 0; i < BSP_RGB_LED_COUNT; i++)
    {
        led_color_buffer[i] = color;
    }

    struct rt_rgbled_multi_configuration configuration;
    configuration.led_count = BSP_RGB_LED_COUNT;
    configuration.color_array = led_color_buffer;
    // rt_kprintf("rgb_led_set_all_color:0x%06x\n", color);
    rt_device_control(rgbled_device, RGB_CMD_SET_MULTI_COLOR, &configuration);
    // struct rt_rgbled_configuration configuration;
    // configuration.color_rgb = color;
    // rt_device_control(rgbled_device, PWM_CMD_SET_COLOR, &configuration);
}


void rgb_color_array_display()
{
    // ...existing code...
}
// 新增：動畫模式主控
void rgb_led_animate(rgb_led_params_t *params)
{
    uint32_t color = 0;
    uint32_t i = 0;
    uint32_t max_cycles = (params->repeat_times == 0) ? 0xFFFFFFFF : params->repeat_times;
    if (params->mode == RGB_ANIM_STATIC) max_cycles = 1;
    while (i < max_cycles) {
        switch (params->mode) {
        case RGB_ANIM_STATIC:
            color = apply_brightness(params->red, params->green, params->blue, params->brightness);
            rgb_led_set_color(color);
            break;
        case RGB_ANIM_BREATHING: {
            const int steps = 120;
            uint32_t step_delay = params->period / steps;
            for (int degree = 0; degree < 360; degree += 3) {
                float sin_val = simple_sin(degree);
                uint8_t breath_brightness = (uint8_t)((sin_val + 1.0f) * 50.0f);
                breath_brightness = (breath_brightness * params->brightness) / 100;
                color = apply_brightness(params->red, params->green, params->blue, breath_brightness);
                rgb_led_set_color(color);
                rt_thread_mdelay(step_delay);
            }
            break;
        }
        case RGB_ANIM_BLINK: {
            uint32_t half_period = params->period / 2;
            color = apply_brightness(params->red, params->green, params->blue, params->brightness);
            rgb_led_set_color(color);
            rt_thread_mdelay(half_period);
            rgb_led_set_color(0x000000);
            rt_thread_mdelay(half_period);
            break;
        }
        case RGB_ANIM_RAINBOW: {
            const int steps = 256;
            uint32_t step_delay = params->period / steps;
            for (int hue = 0; hue < 256; hue++) {
                uint8_t r, g, b;
                hsv_to_rgb(hue, 255, 255, &r, &g, &b);
                color = apply_brightness(r, g, b, params->brightness);
                rgb_led_set_color(color);
                rt_thread_mdelay(step_delay);
            }
            break;
        }
        case RGB_ANIM_FADE: {
            const int steps = 100;
            uint32_t step_delay = params->period / steps;
            for (int fade_step = 0; fade_step < 100; fade_step++) {
                uint8_t start_r, start_g, start_b, end_r, end_g, end_b;
                if (fade_step < 50) {
                    start_r = 0; start_g = 0; start_b = 0;
                    end_r = params->red; end_g = params->green; end_b = params->blue;
                } else {
                    start_r = params->red; start_g = params->green; start_b = params->blue;
                    end_r = 0; end_g = 0; end_b = 0;
                }
                uint8_t fade_r = start_r + ((end_r - start_r) * (fade_step % 50)) / 50;
                uint8_t fade_g = start_g + ((end_g - start_g) * (fade_step % 50)) / 50;
                uint8_t fade_b = start_b + ((end_b - start_b) * (fade_step % 50)) / 50;
                color = apply_brightness(fade_r, fade_g, fade_b, params->brightness);
                rgb_led_set_color(color);
                rt_thread_mdelay(step_delay);
            }
            break;
        }
        default:
            return;
        }
        i++;
    }
}

/*
 * Math load thread: continuous floating-point and integer operations
 * to stress CPU while LED animation runs independently.
 */
static void math_load_entry(void *param)
{
    uint32_t id = (uint32_t)(rt_uint32_t)param;
    volatile float result = 0.0f;
    volatile uint32_t count = 0;

    rt_kprintf("[MATH] Thread%u started\n", id);

    while (1) {
        /* Floating-point workload */
        for (int i = 1; i < 500; i++) {
            result += simple_sin(i % 360);
            result *= 1.0001f;
        }
        /* Integer workload */
        volatile uint32_t val = 0x12345678;
        for (int i = 0; i < 1000; i++) {
            val = val * 1103515245 + 12345;
            val ^= (val >> 16);
        }
        count++;
        if ((count % 1000) == 0) {
            rt_kprintf("[MATH] Thread%u: loops=%u result=%.2f\n", id, count, result);
        }
        rt_thread_mdelay(1);
    }
}

/**
  * @brief  Main program - Multi-task stress test
  *         LED task runs green fade, math tasks load the CPU.
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_thread_t tid;

    rt_kprintf("\n========================================\n");
    rt_kprintf("  RGB LED Stress Test (CPU Load)\n");
    rt_kprintf("  1 LED task + %d math tasks\n", MATH_THREAD_COUNT);
    rt_kprintf("========================================\n\n");

    rgb_led_init();

    /* Create math load threads (priority 21~30) */
    for (int i = 0; i < MATH_THREAD_COUNT; i++) {
        char name[8];
        rt_snprintf(name, sizeof(name), "math%d", i);
        tid = rt_thread_create(name, math_load_entry, (void *)(rt_uint32_t)i,
                               STRESS_STACK_SIZE, MATH_THREAD_PRIO_BASE + i, 10);
        if (tid) rt_thread_startup(tid);
        else rt_kprintf("[STRESS] Failed to create %s\n", name);
    }

    /* LED fade animation runs in main thread context */
    rgb_led_params_t params;
    params.red = 0x00;
    params.green = 0x00;
    params.blue = 0xff;
    params.brightness = 100;
    params.mode = RGB_ANIM_FADE;
    params.period = 2000;
    params.repeat_times = 0; /* infinite */

    rt_kprintf("[LED] Green fade started\n");
    rgb_led_animate(&params);

    while (1)
    {
        rt_thread_mdelay(1000);
    }
    return 0;
}
