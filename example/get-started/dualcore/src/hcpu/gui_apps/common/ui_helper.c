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

static bool anim_time_init;
static uint16_t anim_time_time = 300;
void set_scroll_anim_time(bool init, uint16_t time)
{
    anim_time_init = init;
    if (time == NULL)
    {
        anim_time_time = 300;
    }
    else
    {
        anim_time_time = time;
    }
}
/* Called from vendored LVGL (external/lvgl_v8 lv_obj_scroll.c, #ifdef SkaiwalkWatchOS) */
bool is_scroll_anim_time_init(void)
{
    return anim_time_init;
}
uint16_t get_scroll_anim_time(void)
{
    return anim_time_time;
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
 * @brief 12/24-hour clock formatting helpers.
 *
 * Single source of truth for the user's hour_format preference
 * (SkaiWatchSys.flag_field.hour_format, written by the settings app).
 * Every place that renders a wall-clock time MUST go through these so the
 * 12/24-hour toggle actually takes effect everywhere — not just where it was
 * remembered to be wired up. hour_format: 0 = 12-hour, 1 = 24-hour.
 *
 * Durations (stopwatch / timer / exercise elapsed) are NOT wall-clock times
 * and must keep their own HH:MM:SS formatting — don't route them here.
 */
bool ui_time_is_24h(void)
{
    return SkaiWatchSys.flag_field.hour_format == 1;
}

uint8_t ui_time_display_hour(uint8_t hour_24)
{
    if (ui_time_is_24h())
    {
        return hour_24;
    }
    uint8_t h = hour_24 % 12;
    return (h == 0) ? 12 : h;
}

const char *ui_time_ampm(uint8_t hour_24)
{
    if (ui_time_is_24h())
    {
        return RT_NULL;
    }
    return (hour_24 < 12) ? "AM" : "PM";
}

int ui_time_format_hhmm(char *buf, rt_size_t buf_len, uint8_t hour_24,
                        uint8_t minute)
{
    if (ui_time_is_24h())
    {
        return rt_snprintf(buf, buf_len, "%02d:%02d", hour_24, minute);
    }
    /* 12-hour: no leading zero on the hour, trailing AM/PM. */
    return rt_snprintf(buf, buf_len, "%d:%02d %s",
                       ui_time_display_hour(hour_24), minute,
                       (hour_24 < 12) ? "AM" : "PM");
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
    watch_system_wakeup();
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

void datac_send_data(datac_handle_t handle, uint16_t msg_id, uint8_t *data, uint16_t data_len)
{
    data_msg_t msg;
    uint8_t *msg_payload;

    msg_payload = data_service_init_msg(&msg, msg_id, data_len);
    memcpy(msg_payload, data, data_len);
    datac_send_msg(handle, &msg);
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
    if (lv_disp_get_rotation(NULL) != LV_DISPLAY_ROTATION_90)
    {
        gui_set_screen_rotation(0);
        lv_disp_set_rotation(NULL, LV_DISPLAY_ROTATION_90);
    }
#else
    if (lv_disp_get_rotation(NULL) != LV_DISPLAY_ROTATION_270)
    {
        lv_disp_set_rotation(NULL, LV_DISPLAY_ROTATION_270);
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
    if (lv_disp_get_rotation(NULL) != LV_DISPLAY_ROTATION_180)
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

void handle_download_progress_update(int progress)
{
	/* Silent OTA: progress is tracked internally but no UI is launched. */
	if (progress > 0 && progress <= 100)
	{
		SkaiWatchSys.ota_progress = progress;
	}
	(void)progress;
}


static bool open_display_to_instruction_list = false;
bool is_user_want_to_open_display_to_instruction_list(void)
{
  return open_display_to_instruction_list;
}

void set_user_want_to_open_display_to_instruction_list(bool state)
{
  open_display_to_instruction_list = state;
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/