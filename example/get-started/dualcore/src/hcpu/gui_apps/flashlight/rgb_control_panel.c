/**
 ******************************************************************************
 * @file   rgb_control_panel.c
 * @author Skaiwalk software development team
 * @brief  RGB LED control panel implementation
 ******************************************************************************
 */

/*********************
 *      INCLUDES
 *********************/
#include "rgb_control_panel.h"
#include "lvsf_comp.h"
#include "common_widget.h"
#include "bloc_peripheral.h"
#include "lvsf_font.h"
#include "ui_helper.h"

#define DBG_TAG "rgb.panel"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/*********************
 *      DEFINES
 *********************/
#define PANEL_WIDTH         LV_HOR_RES
#define PANEL_HEIGHT        LV_VER_RES
#define COLORWHEEL_SIZE     180
#define SLIDER_WIDTH        280

// Debounce delay to prevent rapid updates during colorwheel/slider drag (ms)
#define UPDATE_DEBOUNCE_MS  50


/**********************
 *  STATIC PROTOTYPES
 **********************/
static void close_btn_event_cb(lv_event_t *e);
static void colorwheel_event_cb(lv_event_t *e);
static void brightness_event_cb(lv_event_t *e);
static void mode_btn_event_cb(lv_event_t *e);
static void update_led_color(void);
static void update_led_color_debounced(void);
static void update_timer_cb(lv_timer_t *timer);
static void start_animation(rgb_animation_mode_t mode);

/**********************
 *  STATIC VARIABLES
 **********************/
static lv_obj_t *panel_obj = NULL;
static lv_obj_t *colorwheel_obj = NULL;
static lv_obj_t *brightness_slider = NULL;
static lv_obj_t *mode_buttons[RGB_ANIM_MODE_COUNT] = {NULL};
static rgb_led_state_t *p_rgb_state = NULL;

// Debounce control
static lv_timer_t *update_timer = NULL;
static bool pending_update = false;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Initialize RGB LED state structure
 */
void rgb_led_state_init(rgb_led_state_t *state)
{
    if (!state) return;

    state->enabled = false;
    state->base_color.green = 0;
    state->base_color.red = 255;
    state->base_color.blue = 0;
    state->brightness = 50;
    state->animation_mode = RGB_ANIM_STATIC;
    state->period_ms = 500;        // Default animation period
    state->repeat_times = 3;      // Infinite by default

    LOG_I("RGB LED state initialized (period=%d, repeat=%d)", state->period_ms, state->repeat_times);
}

/**
 * @brief Cleanup RGB LED state and stop animations
 */
void rgb_led_state_cleanup(rgb_led_state_t *state)
{
    if (!state) return;

    // Clean up debounce timer
    if (update_timer != NULL)
    {
        lv_timer_del(update_timer);
        update_timer = NULL;
    }

    if (peripheral_provider.control_rgb_led) { peripheral_provider.control_rgb_led(false, NULL); }
    LOG_I("RGB LED state cleanup complete");
}

/**
 * @brief Create and show the RGB control panel
 */
void create_rgb_control_panel(lv_obj_t *parent, rgb_led_state_t *state)
{
    if (panel_obj != NULL || state == NULL) return;

    p_rgb_state = state;

    // Create full-screen panel (scrollable)
    panel_obj = lv_obj_create(parent);
    lv_obj_set_size(panel_obj, PANEL_WIDTH, PANEL_HEIGHT);
    lv_obj_set_style_bg_color(panel_obj, lv_color_hex(0x121212), 0);
    lv_obj_set_style_border_width(panel_obj, 0, 0);
    lv_obj_align(panel_obj, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_scroll_dir(panel_obj, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(panel_obj, LV_SCROLLBAR_MODE_AUTO);

    // Create close button
    lv_obj_t *close_btn = lv_btn_create(panel_obj);
    lv_obj_set_size(close_btn, 50, 50);
    lv_obj_align(close_btn, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_set_style_radius(close_btn, 25, 0);
    lv_obj_set_style_bg_color(close_btn, lv_color_hex(0x333333), 0);
    lv_obj_add_event_cb(close_btn, close_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *close_label = lv_label_create(close_btn);
    lv_label_set_text(close_label, "X");
    lv_obj_set_style_text_color(close_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(close_label);

    // Create colorwheel (centered at top)
    colorwheel_obj = lv_colorwheel_create(panel_obj, true);
    lv_obj_set_size(colorwheel_obj, 160, 160);
    lv_obj_align(colorwheel_obj, LV_ALIGN_TOP_MID, 0, 60);
    lv_colorwheel_set_mode(colorwheel_obj, LV_COLORWHEEL_MODE_HUE);
    lv_colorwheel_set_mode_fixed(colorwheel_obj, true);
    lv_obj_add_event_cb(colorwheel_obj, colorwheel_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Create brightness slider (below colorwheel)
    lv_obj_t *brightness_label = lv_label_create(panel_obj);
    lv_label_set_text(brightness_label, "Brightness");
    lv_obj_set_style_text_color(brightness_label, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(brightness_label, LV_ALIGN_TOP_MID, -90, 210);

    brightness_slider = lv_slider_create(panel_obj);
    lv_slider_set_range(brightness_slider, 0, 100);
    lv_slider_set_value(brightness_slider, state->brightness, LV_ANIM_OFF);
    lv_obj_set_width(brightness_slider, 240);
    lv_obj_set_height(brightness_slider, 20);
    lv_obj_align(brightness_slider, LV_ALIGN_TOP_MID, 0, 235);
    lv_obj_set_style_bg_color(brightness_slider, lv_color_hex(0x555555), LV_PART_MAIN);
    lv_obj_set_style_bg_color(brightness_slider, lv_color_hex(0x0078D7), LV_PART_INDICATOR);
    lv_obj_add_event_cb(brightness_slider, brightness_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Create animation mode label
    lv_obj_t *mode_label = lv_label_create(panel_obj);
    lv_label_set_text(mode_label, "Animation Mode");
    lv_obj_set_style_text_color(mode_label, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(mode_label, LV_ALIGN_TOP_LEFT, 20, 275);

    // Create animation mode buttons in vertical column layout
    const char *mode_names[] = {"Static", "Breathing", "Blink", "Rainbow", "Fade"};
    int btn_y_start = 310;
    int btn_spacing = 58;
    for (int i = 0; i < RGB_ANIM_MODE_COUNT; i++)
    {
        mode_buttons[i] = lv_btn_create(panel_obj);
        lv_obj_set_size(mode_buttons[i], 320, 52);
        lv_obj_align(mode_buttons[i], LV_ALIGN_TOP_LEFT, 20, btn_y_start + i * btn_spacing);
        lv_obj_set_style_radius(mode_buttons[i], 10, 0);
        lv_obj_add_flag(mode_buttons[i], LV_OBJ_FLAG_CHECKABLE);
        lv_obj_add_event_cb(mode_buttons[i], mode_btn_event_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);

        lv_obj_t *btn_label = lv_label_create(mode_buttons[i]);
        lv_label_set_text(btn_label, mode_names[i]);
        lv_obj_set_style_text_color(btn_label, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(btn_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
        lv_obj_center(btn_label);

        // Set initial state
        if (i == state->animation_mode)
        {
            lv_obj_add_state(mode_buttons[i], LV_STATE_CHECKED);
            lv_obj_set_style_bg_color(mode_buttons[i], lv_color_hex(0x0078D7), 0);
        }
        else
        {
            lv_obj_set_style_bg_color(mode_buttons[i], lv_color_hex(0x333333), 0);
        }
    }

    // Auto-enable LED when panel opens
    state->enabled = true;
    start_animation(state->animation_mode);

    LOG_I("RGB control panel created");
}

/**
 * @brief Close the RGB control panel
 */
void close_rgb_control_panel(void)
{
    if (panel_obj != NULL)
    {
        // Clean up debounce timer
        if (update_timer != NULL)
        {
            lv_timer_del(update_timer);
            update_timer = NULL;
        }
        pending_update = false;

        // Auto-disable LED when panel closes
        if (p_rgb_state != NULL)
        {
            p_rgb_state->enabled = false;
            if (peripheral_provider.control_rgb_led)
            {
                peripheral_provider.control_rgb_led(false, NULL);
            }
        }

        lv_obj_del(panel_obj);
        panel_obj = NULL;
        colorwheel_obj = NULL;
        brightness_slider = NULL;
        for (int i = 0; i < RGB_ANIM_MODE_COUNT; i++)
        {
            mode_buttons[i] = NULL;
        }
        p_rgb_state = NULL;
        LOG_I("RGB control panel closed");
    }
}

/**
 * @brief Check if RGB control panel is currently open
 */
bool is_rgb_panel_open(void)
{
    return (panel_obj != NULL);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * @brief Close button event callback
 */
static void close_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        close_rgb_control_panel();
    }
}

/**
 * @brief Colorwheel event callback
 */
static void colorwheel_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED && p_rgb_state != NULL)
    {
        lv_color_t rgb = lv_colorwheel_get_rgb(colorwheel_obj);
        p_rgb_state->base_color.red = rgb.ch.red;
        p_rgb_state->base_color.green = rgb.ch.green;
        p_rgb_state->base_color.blue = rgb.ch.blue;

        LOG_D("Color changed: R=%d G=%d B=%d",
              p_rgb_state->base_color.red,
              p_rgb_state->base_color.green,
              p_rgb_state->base_color.blue);

        // Always update, regardless of animation mode
        update_led_color_debounced();
    }
}

/**
 * @brief Brightness slider event callback
 */
static void brightness_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED && p_rgb_state != NULL)
    {
        p_rgb_state->brightness = (uint8_t)lv_slider_get_value(brightness_slider);
        LOG_D("Brightness changed: %d%%", p_rgb_state->brightness);

        // Always update, regardless of animation mode
        update_led_color_debounced();
    }
}

/**
 * @brief Animation mode button event callback
 */
static void mode_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED && p_rgb_state != NULL)
    {
        int mode = (int)(intptr_t)lv_event_get_user_data(e);

        // Update button states
        for (int i = 0; i < RGB_ANIM_MODE_COUNT; i++)
        {
            if (i == mode)
            {
                lv_obj_add_state(mode_buttons[i], LV_STATE_CHECKED);
                lv_obj_set_style_bg_color(mode_buttons[i], lv_color_hex(0x0078D7), 0);
            }
            else
            {
                lv_obj_clear_state(mode_buttons[i], LV_STATE_CHECKED);
                lv_obj_set_style_bg_color(mode_buttons[i], lv_color_hex(0x333333), 0);
            }
        }

        p_rgb_state->animation_mode = (rgb_animation_mode_t)mode;
        LOG_I("Animation mode changed to: %d", mode);

        if (p_rgb_state->enabled)
        {
            start_animation((rgb_animation_mode_t)mode);
        }
    }
}

/**
 * @brief Update LED color with current state (immediate)
 */
static void update_led_color(void)
{
    if (p_rgb_state != NULL && p_rgb_state->enabled)
    {
        if (peripheral_provider.control_rgb_led)
        {
            rgb_led_params_t params = {
                .color = p_rgb_state->base_color,
                .brightness = p_rgb_state->brightness,
                .animation_mode = p_rgb_state->animation_mode,
                .period_ms = p_rgb_state->period_ms,
                .repeat_times = p_rgb_state->repeat_times
            };
            peripheral_provider.control_rgb_led(true, &params);
        }
    }
}

/**
 * @brief Timer callback for debounced LED update
 */
static void update_timer_cb(lv_timer_t *timer)
{
    if (pending_update)
    {
        update_led_color();
        pending_update = false;
    }

    // Delete timer after execution to ensure clean state
    if (update_timer != NULL)
    {
        lv_timer_del(update_timer);
        update_timer = NULL;
    }
}

/**
 * @brief Update LED color with debounce
 *
 * This function prevents rapid consecutive LED updates during colorwheel/slider drag.
 * It waits for UPDATE_DEBOUNCE_MS of idle time before sending the command,
 * ensuring that PWM signals have time to complete transmission.
 */
static void update_led_color_debounced(void)
{
    // Mark that an update is pending
    pending_update = true;

    // Create or reset the debounce timer
    if (update_timer == NULL)
    {
        update_timer = lv_timer_create(update_timer_cb, UPDATE_DEBOUNCE_MS, NULL);
        lv_timer_set_repeat_count(update_timer, 1);
    }
    else
    {
        lv_timer_reset(update_timer);
    }
}

/**
 * @brief Start animation with specified mode
 *
 * This function updates the animation mode and sends the control command to the LED driver.
 * It uses the current state's period_ms and repeat_times settings.
 *
 * @param mode Animation mode to start
 */
static void start_animation(rgb_animation_mode_t mode)
{
    if (p_rgb_state == NULL) return;

    // Send control command to driver with the animation mode
    if (peripheral_provider.control_rgb_led)
    {
        rgb_led_params_t params = {
            .color = p_rgb_state->base_color,
            .brightness = p_rgb_state->brightness,
            .animation_mode = mode,
            .period_ms = p_rgb_state->period_ms,
            .repeat_times = p_rgb_state->repeat_times
        };
        peripheral_provider.control_rgb_led(true, &params);
        LOG_I("RGB LED mode changed to: %d (period=%d, repeat=%d)",
              mode, p_rgb_state->period_ms, p_rgb_state->repeat_times);
    }
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
