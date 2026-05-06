/*
 * SPDX-FileCopyrightText: 2021-2021 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BLOC_LED_H__
#define __BLOC_LED_H__

#include <rtthread.h>

struct rt_color
{
    char *color_name;
    uint32_t color;
};

// Animation control structures
typedef enum
{
    RGB_ANIM_STATIC = 0,
    RGB_ANIM_MARQUEE,
    RGB_ANIM_BLINK,
    RGB_ANIM_RAINBOW,
    RGB_ANIM_FADE,
    RGB_ANIM_MODE_COUNT
} rgb_animation_mode_t;

// RGB LED parameters (similar to motor_params_t)
typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness;
    rgb_animation_mode_t mode;
    uint32_t period; // Period in milliseconds for each animation cycle
    uint32_t
        repeat_times; // Number of times to repeat the animation (0 = infinite)
} rgb_led_params_t;

typedef struct
{
    uint8_t enabled;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness;
    rgb_animation_mode_t mode;
    uint32_t step;         // Animation step counter
    uint8_t fade_step;     // Fade animation step
    uint32_t period;       // Period in milliseconds
    uint32_t repeat_times; // Remaining repeat times
    uint32_t cycle_count;  // Current cycle counter
} rgb_led_state_t;

/**
 * @brief Initialize RGB LED
 */
void rgb_led_init(void);

void rgb_led_animate(rgb_led_params_t *params);
void rgb_fade_cycle_base_on_battery_level(uint8_t level);

/**
 * @brief Set all RGB LEDs to the same color
 * @param color RGB color value (0xRRGGBB)
 */
void rgb_led_set_all_color(uint32_t color);

/**
 * @brief Turn off all RGB LEDs
 */
void rgb_led_all_off(void);

/**
 * @brief Control RGB LED with enable, color components, brightness, animation,
 * and repeat count
 * @param enable Enable/disable LED (1=on, 0=off)
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 * @param brightness Brightness level (0-100)
 * @param animation_mode Animation mode:
 *        0 = Static (no animation)
 *        1 = Marquee (running light)
 *        2 = Blink (on/off)
 *        3 = Rainbow (color cycle)
 *        4 = Fade (color fade transition)
 * @param period_ms Period in milliseconds for each animation cycle
 * @param repeat_times Number of times to repeat (0 = infinite)
 */
void bloc_peripheral_control_rgb_led(uint8_t enable, uint8_t red, uint8_t green,
                                     uint8_t blue, uint8_t brightness,
                                     uint8_t animation_mode, uint32_t period_ms,
                                     uint32_t repeat_times);

/**
 * @brief Event handlers for RGB LED (called from main task)
 */
void bloc_rgb_led_handle_start_event(void);
void bloc_rgb_led_handle_stop_event(void);

#endif /* __BLOC_LED_H__ */
