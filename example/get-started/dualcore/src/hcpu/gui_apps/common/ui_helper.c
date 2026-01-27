/**
 ******************************************************************************
 * @file   ui_helper.c
 * @author Skaiwalk software development team
 * @brief  UI helper functions for the Skai Watch application
 *
 * This file contains utility functions for UI navigation, screen rotation,
 * touch handling, and various UI-related operations in the Skai Watch system.
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk
 * integrated circuit in a product or a software update for such product, must
 * reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Includes */
#include <rtthread.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "ui_helper.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "ui_handler.h"
#include "watch_system_interact.h"
#include "app_schedule_port.h"

/* Debug configuration */
#define DBG_TAG "app.ui_helper"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* Constants */
#define SKAI_AI_ID "skai_ai" // skai_speech

/* Forward declarations */
static void add_mask_event_cb(lv_event_t *e);

static bool anim_time_init;
void set_scroll_anim_time(bool init)
{
    anim_time_init = init;
}
bool is_scroll_anim_time_init(void)
{
    return anim_time_init;
}

extern void *app_anim_buf_alloc(size_t nbytes, uint8_t index);
extern void *app_anim_buf_free(void *ptr);
static lv_img_dsc_t *screenshot_img_desc;
lv_img_dsc_t *create_widget_snapshot_img(lv_obj_t *target_obj)
{
    LOG_D("create_widget_snapshot_img");
    char *trans_anim_buf = NULL;

    trans_anim_buf =
        (char *)app_anim_buf_alloc(APP_TRANS_ANIM_SNAPSHOT_SIZE, 0);

    RT_ASSERT(trans_anim_buf);
    // img_desc = app_trans_create_img_buf((uint8_t *)trans_anim_buf);
    lv_mem_free(screenshot_img_desc);
    screenshot_img_desc = lv_mem_alloc(sizeof(lv_img_dsc_t));
    RT_ASSERT(screenshot_img_desc != NULL);

    lv_coord_t w = lv_obj_get_width(target_obj);
    lv_coord_t h = lv_obj_get_height(target_obj);
    memset(screenshot_img_desc, 0, sizeof(lv_img_dsc_t));
    screenshot_img_desc->header.cf = LV_IMG_CF_TRUE_COLOR;
    screenshot_img_desc->header.w = w;
    screenshot_img_desc->header.h = h;
    screenshot_img_desc->data = (uint8_t *)trans_anim_buf;
    screenshot_img_desc->data_size =
        lv_snapshot_buf_size_needed(target_obj, LV_IMG_CF_TRUE_COLOR);

    lv_refr_dump_buf_to_img_now(screenshot_img_desc);
    lv_snapshot_take_to_buf(
        target_obj, screenshot_img_desc->header.cf, screenshot_img_desc,
        (uint8_t *)screenshot_img_desc->data, screenshot_img_desc->data_size);
    // 建立一個 img 物件顯示 snapshot
    // lv_obj_t *img_obj = lv_img_create(parent_obj);
    // lv_img_set_src(img_obj, img_desc);
    // lv_obj_update_layout(img_obj);
    // // lv_obj_set_size(img_obj, 200, 200);
    // int img_h = lv_obj_get_height(img_obj);
    // if (img_h > 175)
    // {
    //     uint16_t zoom = (175 * 256) / img_h; // 256 = LV_IMG_ZOOM_NONE
    //     lv_img_set_zoom(img_obj, zoom);
    // }
    // lv_obj_update_layout(img_obj);
    // LOG_D("create_widget_snapshot_img: img_obj=%p, w=%d, h=%d",
    // img_obj, lv_obj_get_width(img_obj), lv_obj_get_height(img_obj));
    // lv_obj_align(img_obj, LV_ALIGN_CENTER, 0, 0);
    return screenshot_img_desc;
}

static bool scrolling_motor_vibrate = true;

bool get_scrolling_motor_vibrate_status(void)
{
    return scrolling_motor_vibrate;
}

void enable_scrolling_motor_vibrate(void)
{
    scrolling_motor_vibrate = true;
}

void disable_scrolling_motor_vibrate(void)
{
    scrolling_motor_vibrate = false;
}
/**
 * @brief Navigate to the Skai AI chat page
 *
 * Launches the Skai AI application with "chat" intent if it's not already
 * active. This function provides a convenient way to access the AI chat
 * functionality from other parts of the application.
 */
void goto_skai_ai_page(void)
{
    if (!gui_app_is_actived(SKAI_AI_ID))
    {
        LOG_D("Opening Skai AI chat page");
        intent_t intent = intent_init(SKAI_AI_ID);
        if (intent == NULL)
        {
            LOG_E("Failed to initialize intent for Skai AI");
            return;
        }
        intent_set_string(intent, "intent", "chat");
        intent_runapp(intent);
    }
    else
    {
        LOG_D("Skai AI app is already active");
    }
}

/**
 * @brief Check if speech interaction is currently active
 *
 * Determines whether any speech-related functionality is currently being used,
 * including recorder, speech recognition, or message display.
 *
 * @return true if recorder, speech or message page is active
 * @return false otherwise
 */
bool check_if_speech_interact(void)
{
    bool recorder_active = gui_app_is_actived(APP_ID_RECORDER);
    bool speech_active = gui_app_is_actived(APP_ID_SPEECH);
    bool message_visible = false;

    if (myLancher[app_index_message].pagetileview == NULL)
    {
        LOG_D("Message page tileview is NULL");
    }
    else
    {
        message_visible = !lv_obj_has_flag(
            myLancher[app_index_message].pagetileview, LV_OBJ_FLAG_HIDDEN);
    }

    LOG_D("Speech interaction check: recorder=%d, speech=%d, message=%d",
          recorder_active, speech_active, message_visible);

    return recorder_active || speech_active || message_visible;
}

/**
 * @brief Show or hide the speech indicator
 *
 * Controls the visibility of the speech indicator UI element.
 *
 * @param show true to show the indicator, false to hide it
 */
void show_speech_indicator(bool show)
{
    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_SPEECH_INDICATOR,
                      .data.action = show};
    lvgl_send_msg(msg);
}

/**
 * @brief Show or hide the AI processing indicator
 *
 * Controls the visibility of the AI processing indicator UI element.
 *
 * @param show true to show the indicator, false to hide it
 */
void show_ai_processing_indicator(bool show)
{
    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_WAITING_INDICATOR,
                      .data.action = show};
    lvgl_send_msg(msg);
}

void hidden_speech_indicator(void)
{
    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_HIDDEN_INDICATOR};
    lvgl_send_msg(msg);
}

/* Interact mode management */
static bool _isInteractMode = false;

/**
 * @brief Get the current interact mode state
 *
 * @return true if interact mode is enabled, false otherwise
 */
bool ui_helper_get_interact_mode(void)
{
    return _isInteractMode;
}

/**
 * @brief Set the interact mode state
 *
 * @param flag true to enable interact mode, false to disable
 */
void ui_helper_set_interact_mode(bool flag)
{
    _isInteractMode = flag;
}

/**
 * @brief Calibrate coordinates for reverse rotation
 *
 * Adjusts touch coordinates when the screen is rotated 180 degrees.
 * This function inverts both x and y coordinates to maintain proper
 * touch response in reverse orientation.
 *
 * @param x Pointer to x-coordinate to be adjusted
 * @param y Pointer to y-coordinate to be adjusted
 */
void calibrate_reverse_rotation(int16_t *x, int16_t *y)
{
    if (x == NULL || y == NULL)
    {
        LOG_E("Invalid pointer passed to calibrate_reverse_rotation");
        return;
    }

    /* Use LVGL display resolution constants */
    *x = lv_disp_get_hor_res(NULL) - *x;
    *y = lv_disp_get_ver_res(NULL) - *y;
}

uint8_t get_system_font_size(int adjust)
{
    int font_size = SkaiWatchSys.font_size + adjust;
    if (font_size < 0)
    {
        font_size = 0;
    }
    else if (font_size > 6)
    {
        font_size = 6;
    }
    return (uint8_t)font_size;
}
/**
 * @brief Run an application with intent parameters
 *
 * Launches an application with specific intent and optional parameters.
 * This function handles the complete app launch process including
 * system wake-up and intent management.
 *
 * @param appIntent Structure containing app_id, intent and optional parameter
 */
void watch_run_app_by_intent(AppIntent *appIntent)
{
    LOG_I("Running app: ID=%s, Param=%s", appIntent->app_id, appIntent->intent);
    watch_system_interact(HCPU_WAKEUP, NULL);
    rt_thread_mdelay(200);
    if (appIntent == NULL)
    {
        LOG_E("appIntent is NULL");
        return;
    }

    if (!gui_app_is_actived(appIntent->app_id))
    {
        LOG_D("Launching app_id=%s, intent=%s, param=%s", appIntent->app_id,
              appIntent->intent, appIntent->param ? appIntent->param : "NULL");

        intent_t intent = intent_init(appIntent->app_id);
        if (intent == NULL)
        {
            LOG_E("Failed to initialize intent for app: %s", appIntent->app_id);
            return;
        }

        intent_set_string(intent, "intent", appIntent->intent);
        if (appIntent->param)
        {
            intent_set_string(intent, "param", appIntent->param);
        }
        intent_runapp(intent);
    }
    else
    {
        LOG_D("App %s is already active", appIntent->app_id);
    }
}

/**
 * @brief Exit an application safely
 *
 * Safely terminates an application if it's currently active.
 * This function includes proper error checking and logging.
 *
 * @param app_id ID of the application to exit
 */
void watch_exit_app(char *app_id)
{
    if (app_id == NULL)
    {
        LOG_E("Attempted to exit NULL app_id");
        return;
    }

    if (gui_app_is_actived(app_id))
    {
        LOG_D("Exiting app: %s", app_id);
        gui_app_exit(app_id);
    }
    else
    {
        LOG_D("App %s is not active, no need to exit", app_id);
    }
}

/**
 * @brief Navigate to the testing application
 *
 * Launches the test application with "all" intent for comprehensive testing.
 */
void goto_to_testing_app(void)
{
    AppIntent intent = {0};
    strcpy(intent.app_id, "test");
    strcpy(intent.intent, "all");
    intent.param = NULL;
    watch_run_app_by_intent(&intent);
}

/**
 * @brief Exit the testing application if active
 *
 * Safely exits the testing application if it's currently running.
 */
void exit_testing_app(void)
{
    LOG_D("Exiting testing application");
    watch_exit_app("test");
}

/* Message refresh timestamp management */
static rt_tick_t last_refresh_input_message_tick = 0;

/**
 * @brief Get the timestamp of the last input message refresh
 *
 * @return rt_tick_t Timestamp value of the last input message refresh
 */
rt_tick_t get_last_refresh_input_message_tick(void)
{
    return last_refresh_input_message_tick;
}

/**
 * @brief Update the timestamp for input message refresh
 *
 * @param tick New timestamp value for input message refresh
 */
void set_last_refresh_input_message_tick(rt_tick_t tick)
{
    last_refresh_input_message_tick = tick;
}

static rt_tick_t last_refresh_ai_reply_message_tick = 0;

/**
 * @brief Get the timestamp of the last AI reply message refresh
 *
 * @return rt_tick_t Timestamp value of the last AI reply message refresh
 */
rt_tick_t get_last_refresh_ai_reply_message_tick(void)
{
    return last_refresh_ai_reply_message_tick;
}

/**
 * @brief Update the timestamp for AI reply message refresh
 *
 * @param tick New timestamp value for AI reply message refresh
 */
void set_last_refresh_ai_reply_message_tick(rt_tick_t tick)
{
    last_refresh_ai_reply_message_tick = tick;
}

/**
 * @brief Display a hint toast message
 *
 * Shows a temporary toast notification with the specified hint text.
 * The message supports variable arguments for formatted text.
 *
 * @param hint Format string for the hint message
 * @param ... Variable arguments for the format string
 */
void ui_show_hint_toast(const char *hint, ...)
{
#ifdef BSP_USING_UI_HANDLER
    va_list args;
    va_start(args, hint);
    char text[127];
    vsnprintf(text, sizeof(text), hint, args);
    va_end(args);

    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_TOAST;
    msg.data.app_message = lv_mem_alloc(strlen(text) + 1);
    if (msg.data.app_message == NULL)
    {
        LOG_E("Failed to allocate memory for toast message");
        return;
    }
    else
    {
        LOG_D("Toast message: %s, len: %d", text, (int)(strlen(text) + 1));
    }
    strcpy(msg.data.app_message, text);
    lvgl_send_msg(msg);
#endif
}

/**
 * @brief Animate to the previous column page
 *
 * Navigates to the previous page in a tileview with animation.
 *
 * @param tv Pointer to the tileview object
 * @param current_page Pointer to the current page index
 */
void animate_to_prev_col_page(lv_obj_t *tv, uint8_t *current_page)
{
    if (*current_page > 0)
    {
        (*current_page)--;
        lv_obj_set_tile_id(tv, *current_page, 0, LV_ANIM_ON);
    }
}

/**
 * @brief Animate to the next column page
 *
 * Navigates to the next page in a tileview with animation.
 *
 * @param tv Pointer to the tileview object
 * @param current_page Pointer to the current page index
 * @param max_page Maximum number of pages
 */
void animate_to_next_col_page(lv_obj_t *tv, uint8_t *current_page,
                              uint8_t max_page)
{
    if (*current_page < max_page - 1)
    {
        (*current_page)++;
        lv_obj_set_tile_id(tv, *current_page, 0, LV_ANIM_ON);
    }
}

/**
 * @brief Rotate screen to 90 degrees
 *
 * Rotates the display to 90-degree orientation based on the current
 * display configuration and reverse settings.
 */
void screen_rotate_to_90_degree(void)
{
#ifdef WATCH_DISPLAY_REVERSE_180
    if (lv_disp_get_rotation(NULL) != LV_DISP_ROT_90)
    {
        gui_set_screen_rotation(0);
        lv_disp_set_rotation(NULL, LV_DISP_ROT_90);
    }
#else
    if (lv_disp_get_rotation(NULL) != LV_DISP_ROT_270)
    {
        lv_disp_set_rotation(NULL, LV_DISP_ROT_270);
    }
#endif
}

/**
 * @brief Rotate screen back to original direction
 *
 * Restores the display to its original orientation based on the
 * current display configuration and reverse settings.
 */
void screen_rotate_back_to_original_direction(void)
{
#ifdef WATCH_DISPLAY_REVERSE_180
    if (lv_disp_get_rotation(NULL) != LV_DISP_ROT_180)
    {
        gui_set_screen_rotation(180);
    }
#else
    if (lv_disp_get_rotation(NULL) != LV_DISP_ROT_NONE)
    {
        lv_disp_set_rotation(NULL, LV_DISP_ROT_NONE);
    }
#endif
}

/**
 * @brief Log touch sample rate
 *
 * Monitors and logs the touch input sampling rate for debugging purposes.
 * This function tracks touch events and reports the frequency once per second.
 */
void log_touch_sample_rate(void)
{
    static uint32_t sample_count = 0;
    static uint32_t last_log_time = 0;

    sample_count++;
    uint32_t now = rt_tick_get() / RT_TICK_PER_SECOND;
    if (now != last_log_time)
    {
        LOG_I("Touch sample rate: %lu Hz", sample_count);
        sample_count = 0;
        last_log_time = now;
    }
}

/* Gradient label support - only enabled on specific platforms */
#ifndef SF32LB55X
    #if (LV_USE_LABEL && LV_USE_CANVAS && LV_DRAW_COMPLEX) &&                  \
        defined(BSP_USING_PSRAM)
        #define ENABLE_GRADIENT_LABEL
    #endif
#endif /* SF32LB55X */

#ifdef ENABLE_GRADIENT_LABEL
    /*
     * Pull up hidden control panel with gradient text effect
     */
    #define MASK_WIDTH 200
    #define MASK_HEIGHT 45

/**
 * @brief Event callback for gradient label mask
 *
 * Handles the drawing events for the gradient label mask effect.
 * This function manages the mask creation, application, and cleanup.
 *
 * @param e Pointer to the LVGL event structure
 */
static void add_mask_event_cb(lv_event_t *e)
{
    static lv_draw_mask_map_param_t m;
    static int16_t mask_id;

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    lv_opa_t *mask_map = lv_event_get_user_data(e);

    if (code == LV_EVENT_COVER_CHECK)
    {
        lv_event_set_cover_res(e, LV_COVER_RES_MASKED);
    }
    else if (code == LV_EVENT_DRAW_MAIN_BEGIN)
    {
        lv_draw_mask_map_init(&m, &obj->coords, mask_map);
        mask_id = lv_draw_mask_add(&m, NULL);
    }
    else if (code == LV_EVENT_DRAW_MAIN_END)
    {
        lv_draw_mask_free_param(&m);
        lv_draw_mask_remove_id(mask_id);
    }
    else if (code == LV_EVENT_DELETE)
    {
        app_cache_free(mask_map);
    }
}

/**
 * @brief Create a gradient label with text mask effect
 *
 * Creates a label with gradient background that shows text through a mask.
 * This creates a visually appealing gradient text effect.
 *
 * @param parent Parent object for the gradient label
 * @param text Text to display in the gradient label
 * @return lv_obj_t* Pointer to the created gradient label object
 */
lv_obj_t *common_gradient_label(lv_obj_t *parent, const char *text)
{
    /* Create the mask of a text by drawing it to a canvas */
    lv_opa_t *mask_map =
        app_cache_alloc(MASK_WIDTH * MASK_HEIGHT, IMAGE_CACHE_PSRAM);

    LV_ASSERT(mask_map);

    /* Create a "8 bit alpha" canvas and clear it */
    lv_obj_t *canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(canvas, mask_map, MASK_WIDTH, MASK_HEIGHT,
                         LV_IMG_CF_ALPHA_8BIT);
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_TRANSP);

    /* Draw a label to the canvas. The result "image" will be used as mask */
    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_color_white();
    label_dsc.align = LV_TEXT_ALIGN_CENTER;
    lv_canvas_draw_text(canvas, 5, 5, MASK_WIDTH, &label_dsc, text);

    /* The mask is ready, canvas is not required anymore */
    lv_obj_del(canvas);

    /* Create an object from where the text will be masked out.
     * Now it's a rectangle with a gradient but it could be an image too */
    lv_obj_t *grad = lv_obj_create(parent);
    lv_obj_set_size(grad, MASK_WIDTH, MASK_HEIGHT);
    lv_obj_center(grad);
    lv_obj_set_style_bg_color(grad, lv_color_hex(0xff0000), 0);
    lv_obj_set_style_bg_grad_color(grad, lv_color_hex(0x0000ff), 0);
    lv_obj_set_style_bg_grad_dir(grad, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_radius(grad, 0, 0);
    lv_obj_add_event_cb(grad, add_mask_event_cb, LV_EVENT_ALL, mask_map);

    return grad;
}
#endif /* ENABLE_GRADIENT_LABEL */

void handle_download_progress_update(int progress)
{
	if (!gui_app_is_actived(APP_ID_INTERACT))
	{
		AppIntent appIntent;
		strcpy(appIntent.app_id, APP_ID_INTERACT);
		strcpy(appIntent.intent, "ota_update");
		watch_run_app_by_intent(&appIntent);
	}
	else
	{
		if (progress > 0 && progress <= 100)
		{
			SkaiWatchSys.ota_progress = progress;
			lvgl_msg_t msg;
			msg.type = LVGL_MSG_TYPE_OTA_UPDATE;
			msg.data.ota_update = SkaiWatchSys.ota_progress;
			lvgl_send_msg(msg);
		}
		else if (progress < 0)
		{
			gui_app_exit(APP_ID_INTERACT);
		}
	}
}


static bool open_display_to_app_list = false;
bool is_user_want_to_open_display_to_app_list(void)
{
  return open_display_to_app_list;
}

void set_user_want_to_open_display_to_app_list(bool state)
{
  open_display_to_app_list = state;
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/