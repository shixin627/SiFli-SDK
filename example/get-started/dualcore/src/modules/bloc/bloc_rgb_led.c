/*
 * SPDX-FileCopyrightText: 2021-2021 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#if defined(RGB_SK6812MINI_HS_ENABLE)
#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "board.h"
#include "drivers/rt_drv_pwm.h"
#include "drv_rgbled.h"
#include "bloc_rgb_led.h"

#define DBG_TAG "bloc.led"
#define DBG_LVL DBG_WARNING
#include "rtdbg.h"


    #define RGB_COLOR (0x00ff00)
    #define RGBLED_NAME "rgbled"

static struct rt_device *rgbled_device = RT_NULL;
static rgb_led_state_t led_state = {0};
static rgb_led_params_t g_rgb_led_params = {
    0}; // Global storage for LED parameters
static rt_uint32_t
    led_color_buffer[BSP_RGB_LED_COUNT];    // Static buffer for DMA safety
static rt_timer_t rgb_anim_timer = RT_NULL; // Timer for animation updates
static bool rgb_led_stop_flag = true;       // Flag to stop animation

static struct rt_color rgb_color_arry[] = {
    {"black", 0x000000},  {"blue", 0x0000ff}, {"green", 0x00ff00},
    {"cyan", 0x00ffff},   {"red", 0xff0000},  {"purple", 0xff00ff},
    {"yellow", 0xffff00}, {"white", 0xffffff}};

static uint32_t apply_brightness(uint8_t r, uint8_t g, uint8_t b,
                                 uint8_t brightness);
static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g,
                       uint8_t *b);
static void rgb_led_send_buffer(void);
void rgb_led_set_all_color(uint32_t color);
void rgb_led_init(void)
{
    #ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA32, GPTIM2_CH1, PIN_NOPULL, 1); // RGB LED 52x  pwm3_cc1
    #elif defined SF32LB58X
    HAL_PIN_Set(PAD_PB39, GPTIM3_CH4, PIN_NOPULL, 0); // 58x pwm4_cc4
    #elif defined SF32LB56X
    HAL_PIN_Set(PAD_PB25, GPTIM3_CH4, PIN_NOPULL, 0); // 566 pwm4_cc4
    #endif

    /* rgbled poweron */
    #ifdef SF32LB52X
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO3_3V3, true, true);
    #endif

    rgbled_device = rt_device_find(RGBLED_NAME);
    if (!rgbled_device)
    {
        LOG_E("Error: RGB LED device not found!");
        return;
    }

    LOG_I("RGB LED initialized successfully.");
}

void rgb_led_set_color(uint32_t color)
{
    if (!rgbled_device)
    {
        LOG_E("Error: RGB LED device not initialized!");
        return;
    }

    #ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA32, GPTIM2_CH1, PIN_NOPULL, 1); // RGB LED 52x  pwm3_cc1
    #elif defined SF32LB58X
    HAL_PIN_Set(PAD_PB39, GPTIM3_CH4, PIN_NOPULL, 0); // 58x pwm4_cc4
    #elif defined SF32LB56X
    HAL_PIN_Set(PAD_PB25, GPTIM3_CH4, PIN_NOPULL, 0); // 566 pwm4_cc4
    #endif

    struct rt_rgbled_configuration configuration;
    configuration.color_rgb = color;
    rt_device_control(rgbled_device, PWM_CMD_SET_COLOR, &configuration);
}

void rgb_color_cycle(void)
{
    static uint16_t color_index = 0;

    if (color_index < sizeof(rgb_color_arry) / sizeof(struct rt_color))
    {
        // rgb_led_set_color(rgb_color_arry[color_index].color);
        rgb_led_set_all_color(rgb_color_arry[color_index].color);
    }

    color_index++;
    if (color_index >= sizeof(rgb_color_arry) / sizeof(struct rt_color))
    {
        color_index = 0;
    }
}

void rgb_fade_cycle(void)
{
    static uint8_t brightness = 0;
    static int8_t step = 5; // Fade step

    brightness += step;
    if (brightness == 0 || brightness == 255)
    {
        step = -step; // Reverse direction at limits
    }

    uint32_t color =
        apply_brightness(0, 255, 0, brightness); // Fade green color
    rgb_led_set_all_color(color);
}

static void rgb_parse_hex_color(uint32_t hex_color, uint8_t *r, uint8_t *g,
                                uint8_t *b)
{
    *r = (hex_color >> 16) & 0xFF;
    *g = (hex_color >> 8) & 0xFF;
    *b = hex_color & 0xFF;
}
void rgb_fade_cycle_base_on_battery_level(uint8_t level)
{
    LOG_D("Fading RGB based on battery level: %d%%", level);
    static uint8_t fade = 0;
    static int8_t fade_step = 5;

    if (level > 100)
        level = 100;

    fade += fade_step;
    if (fade == 0 || fade == 255)
    {
        fade_step = -fade_step;
    }
    uint8_t r, g, b;
    if (level < 20)
    {
        rgb_parse_hex_color(0xFFFFFF, &r, &g, &b);
    }
    else if (level < 40)
    {
        rgb_parse_hex_color(0xFF6A00, &r, &g, &b);
    }
    else if (level < 60)
    {
        rgb_parse_hex_color(0xFFBB00, &r, &g, &b);
    }
    else if (level < 80)
    {
        rgb_parse_hex_color(0xFFFF00, &r, &g, &b);
    }
    else
    {
        rgb_parse_hex_color(0x00FF00, &r, &g, &b);
    }

    for (int i = 0; i < BSP_RGB_LED_COUNT; i++)
    {
        uint8_t threshold_low = i * 20;
        uint8_t threshold_high = (i + 1) * 20;
        uint8_t base_brightness;

        if (level >= threshold_high)
        {
            base_brightness = 100;
        }
        else if (level <= threshold_low)
        {
            base_brightness = 0;
        }
        else
        {
            base_brightness = (level - threshold_low) * 100 / 20;
        }

        // Apply fade modulation: scale base_brightness by fade (0-255)
        uint8_t brightness = (uint8_t)((uint16_t)base_brightness * fade / 255);
        led_color_buffer[i] = apply_brightness(r, g, b, brightness);
    }

    rgb_led_send_buffer();
}

void rgb_led_off(void)
{
    rgb_led_set_color(0x000000);
}

void rgb_led_set_all_color(uint32_t color)
{
    if (!rgbled_device)
    {
        LOG_E("Error: RGB LED device not initialized!");
        return;
    }

    #ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA32, GPTIM2_CH1, PIN_NOPULL, 1); // RGB LED 52x  pwm3_cc1
    #elif defined SF32LB58X
    HAL_PIN_Set(PAD_PB39, GPTIM3_CH4, PIN_NOPULL, 0); // 58x pwm4_cc4
    #elif defined SF32LB56X
    HAL_PIN_Set(PAD_PB25, GPTIM3_CH4, PIN_NOPULL, 0); // 566 pwm4_cc4
    #endif

    // Use static buffer to ensure data persists during DMA transfer
    for (int i = 0; i < BSP_RGB_LED_COUNT; i++)
    {
        led_color_buffer[i] = color;
    }

    struct rt_rgbled_multi_configuration configuration;
    configuration.led_count = BSP_RGB_LED_COUNT;
    configuration.color_array = led_color_buffer;
    rt_device_control(rgbled_device, RGB_CMD_SET_MULTI_COLOR, &configuration);
}

void rgb_led_all_off(void)
{
    rgb_led_set_all_color(0x000000);
}

extern void main_send_rgb_start_event(void);
extern void main_send_rgb_stop_event(void);

// Forward declarations
static void rgb_led_stop_internal(void);

// Event handler functions (called from main task)
void bloc_rgb_led_handle_start_event(void)
{
    LOG_I("[bloc_rgb_led_handle_start_event] mode=%d, period=%d, repeat=%d",
          g_rgb_led_params.mode, g_rgb_led_params.period,
          g_rgb_led_params.repeat_times);
    rgb_led_animate(&g_rgb_led_params);
}

void bloc_rgb_led_handle_stop_event(void)
{
    rgb_led_stop_internal();
}

// Animation execution (similar to bloc_motor_vibrate)
void rgb_led_animate(rgb_led_params_t *params)
{
    led_state.enabled = 1;
    led_state.red = params->red;
    led_state.green = params->green;
    led_state.blue = params->blue;
    led_state.brightness = params->brightness;
    led_state.mode = params->mode;
    led_state.period = params->period;
    led_state.repeat_times = params->repeat_times;
    led_state.step = 0;
    led_state.fade_step = 0;
    led_state.cycle_count = 0;
    rgb_led_stop_flag = false;

    uint32_t color = 0;
    uint32_t i = 0;

    // For infinite loop (repeat_times = 0), we need a different approach
    // But for now, let's implement the counted version like motor
    uint32_t max_cycles =
        (params->repeat_times == 0) ? 0xFFFFFFFF : params->repeat_times;

    if (led_state.mode == RGB_ANIM_STATIC)
    {
        max_cycles = 1; // Static mode only needs one cycle
    }
    while (i < max_cycles && !rgb_led_stop_flag)
    {
        switch (led_state.mode)
        {
        case RGB_ANIM_STATIC:
        {
            // Static color, just apply brightness once
            color = apply_brightness(led_state.red, led_state.green,
                                     led_state.blue, led_state.brightness);
            rgb_led_set_all_color(color);
            break;
        }
        case RGB_ANIM_MARQUEE:
        {
            // Marquee effect: single lit LED moves through the strip
            // One cycle = all LEDs traversed once
            uint32_t step_delay = params->period / BSP_RGB_LED_COUNT;

            color = apply_brightness(led_state.red, led_state.green,
                                     led_state.blue, led_state.brightness);

            for (int pos = 0; pos < BSP_RGB_LED_COUNT; pos++)
            {
                if (rgb_led_stop_flag)
                    break;

                // Clear all LEDs, light up only the current position
                for (int j = 0; j < BSP_RGB_LED_COUNT; j++)
                {
                    led_color_buffer[j] = (j == pos) ? color : 0x000000;
                }
                rgb_led_send_buffer();
                rt_thread_delay(step_delay);
            }
            led_state.step++;
            break;
        }

        case RGB_ANIM_BLINK:
        {
            // Blink on/off - each cycle is one complete blink (on + off)
            // period is the total duration of one complete cycle (on + off)
            uint32_t half_period = params->period / 2;

            // Turn on
            color = apply_brightness(led_state.red, led_state.green,
                                     led_state.blue, led_state.brightness);
            rgb_led_set_all_color(color);
            rt_thread_delay(half_period);

            // Turn off
            color = 0x000000;
            rgb_led_set_all_color(color);
            rt_thread_delay(half_period);

            led_state.step++;
            break;
        }

        case RGB_ANIM_RAINBOW:
        {
            // Rainbow color cycle: one complete cycle through all hues (0-255)
            // period is the total duration of one complete cycle
            const int steps = 256;
            uint32_t step_delay = params->period / steps;

            for (int hue = 0; hue < 256; hue++)
            {
                if (rgb_led_stop_flag)
                    break;

                uint8_t r, g, b;
                hsv_to_rgb(hue, 255, 255, &r, &g, &b);
                color = apply_brightness(r, g, b, led_state.brightness);
                rgb_led_set_all_color(color);
                rt_thread_delay(step_delay);
            }
            led_state.step++;
            break;
        }

        case RGB_ANIM_FADE:
        {
            // Fade: one complete cycle (fade in then fade out, 100 steps)
            // period is the total duration of one complete cycle
            const int steps = 100;
            uint32_t step_delay = params->period / steps;

            for (int fade_step = 0; fade_step < 100; fade_step++)
            {
                if (rgb_led_stop_flag)
                    break;

                uint8_t start_r, start_g, start_b, end_r, end_g, end_b;
                if (fade_step < 50)
                {
                    // Fade in: black -> color
                    start_r = 0;
                    start_g = 0;
                    start_b = 0;
                    end_r = led_state.red;
                    end_g = led_state.green;
                    end_b = led_state.blue;
                }
                else
                {
                    // Fade out: color -> black
                    start_r = led_state.red;
                    start_g = led_state.green;
                    start_b = led_state.blue;
                    end_r = 0;
                    end_g = 0;
                    end_b = 0;
                }
                uint8_t fade_r =
                    start_r + ((end_r - start_r) * (fade_step % 50)) / 50;
                uint8_t fade_g =
                    start_g + ((end_g - start_g) * (fade_step % 50)) / 50;
                uint8_t fade_b =
                    start_b + ((end_b - start_b) * (fade_step % 50)) / 50;

                color = apply_brightness(fade_r, fade_g, fade_b,
                                         led_state.brightness);
                rgb_led_set_all_color(color);
                rt_thread_delay(step_delay);
            }
            led_state.step++;
            break;
        }

        default:
            return;
        }

        if (rgb_led_stop_flag)
        {
            break;
        }
        i++;
    }

    rgb_led_stop_flag = true;
    if (led_state.mode != RGB_ANIM_STATIC)
    {
        rgb_led_stop_internal();
    }
}

static void rgb_led_stop_internal(void)
{
    led_state.enabled = 0;
    rgb_led_all_off();
}

// Legacy function for compatibility - kept but not used in new architecture
void rgb_led_process_update(void)
{
    // This function is no longer used in the event-driven architecture
    // Kept for backward compatibility if needed
}

// Helper function to send the LED color buffer to the device
static void rgb_led_send_buffer(void)
{
    if (!rgbled_device)
        return;

    #ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA32, GPTIM2_CH1, PIN_NOPULL, 1);
    #elif defined SF32LB58X
    HAL_PIN_Set(PAD_PB39, GPTIM3_CH4, PIN_NOPULL, 0);
    #elif defined SF32LB56X
    HAL_PIN_Set(PAD_PB25, GPTIM3_CH4, PIN_NOPULL, 0);
    #endif

    struct rt_rgbled_multi_configuration configuration;
    configuration.led_count = BSP_RGB_LED_COUNT;
    configuration.color_array = led_color_buffer;
    rt_device_control(rgbled_device, RGB_CMD_SET_MULTI_COLOR, &configuration);
}

// Helper function to apply brightness to color
static uint32_t apply_brightness(uint8_t r, uint8_t g, uint8_t b,
                                 uint8_t brightness)
{
    if (brightness >= 100)
    {
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }

    uint32_t scaled_r = (r * brightness) / 100;
    uint32_t scaled_g = (g * brightness) / 100;
    uint32_t scaled_b = (b * brightness) / 100;
    return (scaled_r << 16) | (scaled_g << 8) | scaled_b;
}

// Helper function for HSV to RGB conversion (for rainbow effect)
static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g,
                       uint8_t *b)
{
    uint8_t region, remainder, p, q, t;

    if (s == 0)
    {
        *r = v;
        *g = v;
        *b = v;
        return;
    }

    region = h / 43;
    remainder = (h - (region * 43)) * 6;

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
    case 0:
        *r = v;
        *g = t;
        *b = p;
        break;
    case 1:
        *r = q;
        *g = v;
        *b = p;
        break;
    case 2:
        *r = p;
        *g = v;
        *b = t;
        break;
    case 3:
        *r = p;
        *g = q;
        *b = v;
        break;
    case 4:
        *r = t;
        *g = p;
        *b = v;
        break;
    default:
        *r = v;
        *g = p;
        *b = q;
        break;
    }
}

// Public API to start RGB LED animation (similar to motor_vibrate_start)
void rgb_led_start(rgb_led_params_t *params)
{
    // Save LED parameters to global variable
    g_rgb_led_params = *params;
    // Send start event to main task
    main_send_rgb_start_event();
}

// Public API to stop RGB LED animation
void rgb_led_stop(void)
{
    rgb_led_stop_flag = true;
}

// Updated API with repeat_times parameter
void bloc_peripheral_control_rgb_led(uint8_t enable, uint8_t red, uint8_t green,
                                     uint8_t blue, uint8_t brightness,
                                     uint8_t animation_mode, uint32_t period_ms,
                                     uint32_t repeat_times)
{
    if (!enable)
    {
        rgb_led_stop();
        return;
    }

    rgb_led_params_t params;
    params.red = red;
    params.green = green;
    params.blue = blue;
    params.brightness = brightness;
    params.mode = (rgb_animation_mode_t)animation_mode;
    params.period = period_ms;
    params.repeat_times = repeat_times; // 0 = infinite, >0 = counted

    rgb_led_start(&params);
}
#endif // RGB_SK6812MINI_HS_ENABLE