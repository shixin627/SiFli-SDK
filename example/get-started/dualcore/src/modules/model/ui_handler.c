/**
 ******************************************************************************
 * @file   ui_handler.c
 * @author Skaiwalk software development team
 * @brief  UI message handler for the Skai Watch application
 *
 * This file handles all UI-related messages and events in the Skai Watch
 * system. It manages message queues, app state tracking, and coordinates
 * between different UI components and system services.
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
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
#include <string.h>
#include "ui_handler.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "bloc_setting.h"
#include "watch_system_interact.h"
#include "bloc_skaiwalk.h"
#include "bloc_v2t.h"
#include "bloc_weather.h"
#include "gui_app_fwk.h"
#include "lvgl.h"
#ifdef BSP_USING_PM
    #include "gui_app_pm.h"
#endif
#include "ui_helper.h"

/* Debug configuration */
#define DBG_TAG "ui.handler"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* Configuration */
#define USE_LV_TIMER

/* External image declarations */
LV_IMG_DECLARE(icon_mic);
LV_IMG_DECLARE(instagram);

/* Forward declarations for cross-module helpers reached from individual
   message cases. Kept here rather than inline-extern'd in each case body. */
extern void open_v2t_mic(void);
extern void update_ai_process_indicator_text(app_gesture_indicator_t *indicator,
                                              const char *message,
                                              bool is_active);
extern void set_skai_widget_processing_text(const char *text);
extern void mouse_apply_v2t_input(const char *text);
extern void toggle_keyboard_visibility(void);
extern void open_selected_widget(bool need_widget_img_anima);
extern void open_ai_tap_hint_bg(bool open);
extern void reset_skai_widget_input_text(void);
extern void dial_header_music_pause_cb(void);
#ifdef SHOW_BAD_SIGNAL_INDICATOR
extern void set_signal_bad(bool bad);
#endif

/* ============== Global Variables ============== */
static rt_mq_t lvgl_mq; /**< Message queue for LVGL messages */

#ifdef USE_LV_TIMER
static lvgl_msg_t lvgl_msg; /**< LVGL message buffer for timer mode */
#endif

/* Captured at first process_lvgl_message() invocation. Used by callers in
   BLE / file-receive threads (KE_EVT2, 4KB stack) to detect whether they
   can safely run LVGL ops directly, or must defer via lvgl_send_msg().
   Direct LVGL ops on KE_EVT2 cascade through lv_event_send and STKOF. */
static rt_thread_t s_lvgl_thread = RT_NULL;

bool is_on_lvgl_thread(void)
{
    return s_lvgl_thread != RT_NULL && rt_thread_self() == s_lvgl_thread;
}

lvgl_msg_handler_t lvgl_msg_handler; /**< Message handler function table */
static rt_tick_t last_refresh_tick;  /**< Last screen refresh timestamp */
static watch_app_id_t watch_app_id =
    app_id_mainmenu; /**< Current active app ID */
/* ============== Helper Functions ============== */
void clear_all_handlers(void)
{
    memset(&lvgl_msg_handler, 0, sizeof(lvgl_msg_handler_t));
}
/**
 * @brief Get the timestamp of the last screen refresh
 *
 * @return rt_tick_t Last refresh timestamp
 */
rt_tick_t get_last_refresh_tick(void)
{
    return last_refresh_tick;
}

/**
 * @brief Trigger screen refresh with rate limiting
 *
 * Prevents excessive screen refreshes by limiting refresh frequency
 * to once every 500ms to improve performance.
 */
static void trigger_activity(void)
{
    rt_tick_t tick = rt_tick_get();
    if (tick - last_refresh_tick > 500)
    {
        lv_disp_trig_activity(NULL);
        last_refresh_tick = rt_tick_get();
    }
}

/* ============== Task and Message Handling ============== */

/**
 * @brief Process LVGL messages from the message queue
 *
 * This function handles all incoming LVGL messages and routes them to
 * appropriate handler functions based on message type.
 *
 * @param msg Pointer to the LVGL message structure
 */

static void process_lvgl_message(lvgl_msg_t *msg)
{
    if (s_lvgl_thread == RT_NULL)
        s_lvgl_thread = rt_thread_self();

    uint8_t type = msg->type;

    switch (type)
    {
    case LVGL_MSG_TYPE_MEDIA_TITLE:
        if (lvgl_msg_handler.handle_bar_media_title)
        {
            lvgl_msg_handler.handle_bar_media_title(msg->data.media_data.title);
        }
        if (lvgl_msg_handler.handle_dial_media_title)
        {
            lvgl_msg_handler.handle_dial_media_title(
                msg->data.media_data.title);
        }
        if (lvgl_msg_handler.handle_dial_media_header_title)
        {
            lvgl_msg_handler.handle_dial_media_header_title(
                msg->data.media_data.title);
        }
        break;
    case LVGL_MSG_TYPE_MEDIA_IMG:
        if (lvgl_msg_handler.handle_app_media_img)
        {
            lvgl_msg_handler.handle_app_media_img(msg->data.media_data.title);
        }
        if (lvgl_msg_handler.handle_dial_media_img)
        {
            lvgl_msg_handler.handle_dial_media_img(msg->data.media_data.title);
        }
        LOG_D("REMOVE PREV MEDIA IMG: %s", msg->data.media_data.img_path);
        lv_img_cache_invalidate_src(msg->data.media_data.img_path);
        break;
    case LVGL_MSG_TYPE_MEDIA_HEADER_IMG:
        if (lvgl_msg_handler.handle_dial_media_header_img)
        {
            lvgl_msg_handler.handle_dial_media_header_img(
                msg->data.media_data.title);
        }
        LOG_D("REMOVE PREV MEDIA HEADER IMG: %s",
              msg->data.media_data.img_path);
        // lv_img_cache_invalidate_src(msg->data.media_data.img_path);

        // remove(msg->data.media_data.img_path);
        break;
    case LVGL_MSG_TYPE_MEDIA_VOLUME:
        if (lvgl_msg_handler.handle_media_volume)
        {
            lvgl_msg_handler.handle_media_volume(&msg->data.media_volume);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_IMU_ACC:
        if (lvgl_msg_handler.handle_imu_acc)
        {
            lvgl_msg_handler.handle_imu_acc(&msg->data.imu_acc);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_IMU_GYRO:
        if (lvgl_msg_handler.handle_imu_gyro)
        {
            lvgl_msg_handler.handle_imu_gyro(&msg->data.imu_gyro);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_IMU_ATTITUDE:
        if (lvgl_msg_handler.handle_imu_attitude)
        {
            lvgl_msg_handler.handle_imu_attitude(&msg->data.imu_attitude);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_HR:
        if (lvgl_msg_handler.handle_hr)
        {
            lvgl_msg_handler.handle_hr(msg->data.hr);
        }
        break;

    case LVGL_MSG_TYPE_BATTERY_VOLTAGE:
        if (lvgl_msg_handler.handle_battery_voltage)
        {
            lvgl_msg_handler.handle_battery_voltage(&msg->data.battery_voltage);
            // trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_BATTERY_LEVEL:
        if (lvgl_msg_handler.refresh_battery_level)
        {
            lvgl_msg_handler.refresh_battery_level(msg->data.battery_level);
        }
        if (lvgl_msg_handler.handle_battery_percentage)
        {
            lvgl_msg_handler.handle_battery_percentage(msg->data.battery_level);
        }
        // trigger_activity();
        break;

    case LVGL_MSG_TYPE_CHARGE_STATUS:
        if (lvgl_msg_handler.handle_charge_status)
        {
            lvgl_msg_handler.handle_charge_status(&msg->data.charge_status);
            // trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_NOTIFICATION:
        if (lvgl_msg_handler.handle_notification)
        {
            lvgl_msg_handler.handle_notification(NULL);
            trigger_activity();
        }
        if (lvgl_msg_handler.handle_new_notification)
        {
            lvgl_msg_handler.handle_new_notification();
            trigger_activity();
        }
        if (lvgl_msg_handler.handle_dial_header_new_notification)
        {
            lvgl_msg_handler.handle_dial_header_new_notification();
        }
        break;
    case LVGL_MSG_TYPE_CALNEDAR:
        if (lvgl_msg_handler.refresh_calendar)
        {
            lvgl_msg_handler.refresh_calendar();
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_REFRESH_CALENDAR_WIDGET:
        if (lvgl_msg_handler.handle_refresh_calendar_widget)
        {
            lvgl_msg_handler.handle_refresh_calendar_widget();
            trigger_activity();
        }
        if (lvgl_msg_handler.handle_refresh_dial_calendar_widget)
        {
            lvgl_msg_handler.handle_refresh_dial_calendar_widget();
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_REFRESH_WEATHER_WIDGET:
        if (lvgl_msg_handler.handle_refresh_weather_widget)
        {
            lvgl_msg_handler.handle_refresh_weather_widget();
            trigger_activity();
        }
        if (lvgl_msg_handler.handle_refresh_dial_weather_widget)
        {
            lvgl_msg_handler.handle_refresh_dial_weather_widget();
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_MESSAGE:
        if (lvgl_msg_handler.refresh_message)
        {
            lvgl_msg_handler.refresh_message();
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_MESSAGE_STREAM:
        if (lvgl_msg_handler.refresh_message_stream && msg->data.app_message)
        {
            lvgl_msg_handler.refresh_message_stream(msg->data.app_message);
            lv_mem_free(msg->data.app_message);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_INPUT_MESSAGE:
        if (lvgl_msg_handler.handle_input_message)
        {
            lvgl_msg_handler.handle_input_message(msg->data.message);
            trigger_activity();
            LOG_D("LVGL_MSG_TYPE_INPUT_MESSAGE: %s", msg->data.message);
        }
        LOG_D("FREE msg->data.message: %p", msg->data.message);
        break;

    case LVGL_MSG_TYPE_WATCHFACE:
        if (lvgl_msg_handler.handle_watchface)
        {
            lvgl_msg_handler.handle_watchface(&msg->data.watchface);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_MIC:
        if (lvgl_msg_handler.handle_mic)
        {
            lvgl_msg_handler.handle_mic(&msg->data.mic_state);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_VAD_STATUS:
        if (lvgl_msg_handler.handle_vad_status)
        {
            lvgl_msg_handler.handle_vad_status(msg->data.mic_state);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_PAGEVIEW_ACTION:
        if (lvgl_msg_handler.handle_pageview_action)
        {
            lvgl_msg_handler.handle_pageview_action(&msg->data.action);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_LAUNCHER_ACTION:
        if (lvgl_msg_handler.handle_launcher_action)
        {
            lvgl_msg_handler.handle_launcher_action(&msg->data.action);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_UNGRAB_EVENT:
        if (lvgl_msg_handler.handle_ungrab_event)
        {
            lvgl_msg_handler.handle_ungrab_event();
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_LONGPRESS_EVENT:
        if (lvgl_msg_handler.handle_longpress_event)
        {
            lvgl_msg_handler.handle_longpress_event();
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_BACK_EVENT:
        if (lvgl_msg_handler.handle_back_event)
        {
            lvgl_msg_handler.handle_back_event();
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_TAP_INDICATOR:
    {
#ifdef SHOW_TAP_GESTURE_INDICATOR
        uint8_t gesture = msg->data.gesture;
        if (!gui_app_is_actived(APP_ID_GAME_DINOSAUR) || gesture == 0)
        {
            toggle_tap_indicator(gui_app_get_gesture_indicator(), gesture);
        }
#endif
        trigger_activity();
    }
    break;

    case LVGL_MSG_TYPE_GYRO_SCROLL_LIST:
    {
        if (lvgl_msg_handler.handle_gyro_scroll_list)
        {
            lvgl_msg_handler.handle_gyro_scroll_list(msg->data.scroll_up);
            trigger_activity();
        }
    }
    break;

    case LVGL_MSG_TYPE_UNGRAB_INDICATOR:
    {
#ifdef SHOW_TAP_GESTURE_INDICATOR
        toggle_ungrab_indicator(gui_app_get_gesture_indicator(),
                                msg->data.gesture);
#endif
    }
    break;

#ifdef SHOW_UNKNOWN_GESTURE_INDICATOR
    case LVGL_MSG_TYPE_UNKNOWN_INDICATOR:
    {
        bool action = msg->data.action;
        if (action)
        {
            unknown_indicator_create(gui_app_get_gesture_indicator());
        }
        else
        {
            unknown_indicator_hidden(gui_app_get_gesture_indicator());
        }
    }
    break;
#endif

#ifdef SHOW_UNGRAB_ENABLE_INDICATOR
    case LVGL_MSG_TYPE_RELEASE_INDICATOR:
        show_gesture_release_indicator(msg->data.action);
        break;
#endif

#ifdef SHOW_BAD_SIGNAL_INDICATOR
    case LVGL_MSG_TYPE_BAD_SIGNAL_INDICATOR:
    {
        if (msg->data.action)
        {
            set_signal_bad(true);
            bad_signal_indicator_create();
        }
        else
        {
            set_signal_bad(false);
            bad_signal_indicator_hidden();
        }
    }
    break;
#endif

    case LVGL_MSG_TYPE_SPEECH_SHOW_BG:
        voice_recognition_hint_bg_show(gui_app_get_gesture_indicator());
        break;

    case LVGL_MSG_TYPE_SPEECH_INDICATOR:
        if (msg->data.action)
        {
            voice_recognition_hint_create(gui_app_get_gesture_indicator());
        }
        else
        {
            if (is_at_ai_interface())
            {
                LOG_D("ai interface, keep ai hint");
                quick_ai_hint_hidden(gui_app_get_gesture_indicator());
            }
        }
        break;

    case LVGL_MSG_TYPE_WAITING_INDICATOR:
    {
        wait_for_ai_reply_message();
        break;
    }

    case LVGL_MSG_TYPE_HIDDEN_INDICATOR:
    {
        voice_recognition_hint_hidden(gui_app_get_gesture_indicator());
        break;
    }

    case LVGL_MSG_TYPE_UPDATE_PROCESS_TOOLKIT:
    {
        /* Payload is now a description string allocated by the sender. */
        char *description = msg->data.app_message;
        if (description != NULL)
        {
            update_ai_process_indicator_text(gui_app_get_gesture_indicator(),
                                             description, true);
            set_skai_widget_processing_text(description);
            lv_mem_free(description);
        }
        break;
    }

    case LVGL_MSG_TYPE_MOUSE_OPEN_V2T:
        open_v2t_mic();
        break;

    case LVGL_MSG_TYPE_MOUSE_INPUT_TEXT:
        mouse_apply_v2t_input(msg->data.message);
        break;

    case LVGL_MSG_TYPE_MOUSE_OPEN_KEYBOARD:
        toggle_keyboard_visibility();
        break;

    case LVGL_MSG_TYPE_LOADING:
        if (lvgl_msg_handler.handle_loading)
        {
            lvgl_msg_handler.handle_loading(msg->data.loading);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_TIME_TEXT:
        if (lvgl_msg_handler.handle_time_text)
        {
            lvgl_msg_handler.handle_time_text();
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_BLUETOOTH_CONNECTION:
        if (lvgl_msg_handler.handle_bluetooth_connection)
        {
            lvgl_msg_handler.handle_bluetooth_connection(
                msg->data.bluetooth_connection);
        }
        break;

    case LVGL_MSG_TYPE_APP_LIST_OPA:
        if (lvgl_msg_handler.handle_set_instruction_list_opa)
        {
            lvgl_msg_handler.handle_set_instruction_list_opa(msg->data.opa);
            trigger_activity();
        }
        break;
    case LVGL_MSG_TYPE_APP_LIST_SCROLL_BAR_OFFSET:
    {
        if (lvgl_msg_handler.handle_set_arc_stripe_external_offset)
        {
            lvgl_msg_handler.handle_set_arc_stripe_external_offset(
                msg->data.scroll_offset);
            // trigger_activity();
        }

        break;
    }
    case LVGL_MSG_TYPE_SEND_MESSAGE:
        if (lvgl_msg_handler.handle_send_message)
        {
            lvgl_msg_handler.handle_send_message();
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_SWITCH_SELECTED:
        if (lvgl_msg_handler.handle_switch_selected)
        {
            lvgl_msg_handler.handle_switch_selected(
                (bool *)msg->data.switch_selected);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_CONTROL_QUICK_BTN:
        if (lvgl_msg_handler.handle_control_quick_btn)
        {
            lvgl_msg_handler.handle_control_quick_btn(
                msg->data.quick_btn_action);
        }
        break;

    case LVGL_MSG_TYPE_WIDGETS_CONTROL:
        if (lvgl_msg_handler.handle_widgets_control)
        {
            lvgl_msg_handler.handle_widgets_control(msg->data.widgets_control);
            trigger_activity();
        }
        break;
    case LVGL_MSG_TYPE_WIDGET_LIST_SELECT:
        open_selected_widget(true);
        break;
    case LVGL_MSG_TYPE_HAND_UP:
        if (lvgl_msg_handler.handle_hand_up)
        {
            lvgl_msg_handler.handle_hand_up(msg->data.action);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_NAV_BAR_CONTROL:
        if (lvgl_msg_handler.handle_nav_bar_control)
        {
            lvgl_msg_handler.handle_nav_bar_control(msg->data.action);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_MEDIA_CONTROL:
        if (lvgl_msg_handler.handle_media_control)
        {
            lvgl_msg_handler.handle_media_control(msg->data.media_control);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_OTA_UPDATE:
        if (lvgl_msg_handler.handle_ota_update)
        {
            uint8_t progress = msg->data.ota_update;
            lvgl_msg_handler.handle_ota_update(progress);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_VOLUME_CONTROL:
        if (lvgl_msg_handler.handle_volume_control)
        {
            lvgl_msg_handler.handle_volume_control(msg->data.volume_control);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_CLEAR_NOTIFICATION_BAR_INDICATOR:
        if (lvgl_msg_handler.handle_clear_notification_bar_indicator)
        {
            lvgl_msg_handler.handle_clear_notification_bar_indicator();
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_OPEN_MESSAGE_PAGE:
        if (lvgl_msg_handler.handle_open_message_page_content)
        {
            lvgl_msg_handler.handle_open_message_page_content();
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_SYNC_STATUS:
        if (lvgl_msg_handler.handle_file_sync)
        {
            lvgl_msg_handler.handle_file_sync(msg->data.sync_state);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_SYNC_PROGRESS:
        if (lvgl_msg_handler.handle_sync_progress)
        {
            lvgl_msg_handler.handle_sync_progress(msg->data.action);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_TOAST:
        if (lvgl_msg_handler.handle_toast && msg->data.app_message)
        {
            lvgl_msg_handler.handle_toast(msg->data.app_message);
            lv_mem_free(msg->data.app_message);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_GRAVITY_INDICATOR:
        if (lvgl_msg_handler.handle_gravity_indicator)
        {
            lvgl_msg_handler.handle_gravity_indicator(msg->data.action);
            trigger_activity();
        }
        break;

    case LVGL_MSG_TYPE_TRIGGER_ACTIVITY:
    {
        trigger_activity();
        break;
    }

    case LVGL_MSG_TYPE_AI_TAP_HINT:
        open_ai_tap_hint_bg(true);
        break;

    case LVGL_MSG_TYPE_RESET_AI_WIDGET:
        reset_skai_widget_input_text();
        break;

    case LVGL_MSG_TYPE_DIAL_HEADER_TIMER:
        dial_header_music_pause_cb();
        break;

    case LVGL_MSG_TYPE_REFRESH_INSTRUCTION_LIST:
    {
        extern void refresh_custom_instructions(void);
        refresh_custom_instructions();
        break;
    }

    case LVGL_MSG_TYPE_RESET_INSTRUCTION_LIST:
    {
        extern void apply_instruction_list_reset_on_lvgl_thread(void);
        apply_instruction_list_reset_on_lvgl_thread();
        break;
    }

    default:
    {
        type = LVGL_MSG_TYPE_UNKNOWN;
        LOG_W("Unknown message type: %d", type);
        break;
    }
    }

    if (type != LVGL_MSG_TYPE_UNKNOWN)
    {
        LOG_D("GUI Task: msg type: %d", type);
    }
}

void reset_lvgl_msg_handler(void)
{
    lvgl_msg_handler.handle_widgets_control = NULL;
    lvgl_msg_handler.handle_tap_indicator = NULL;
}

/* ============== Task Entry Functions ============== */

#ifndef USE_LV_TIMER
/**
 * @brief LVGL task entry function for thread-based message processing
 *
 * Creates a message queue and continuously processes LVGL messages
 * in a dedicated thread.
 *
 * @param param Thread parameter (unused)
 */
static void lvgl_task(void *param)
{
    lvgl_msg_t msg;
    lvgl_mq = rt_mq_create("lvgl_mq", sizeof(lvgl_msg_t), 10, RT_IPC_FLAG_FIFO);

    while (1)
    {
        if (rt_mq_recv(lvgl_mq, &msg, sizeof(lvgl_msg_t), RT_WAITING_FOREVER) ==
            RT_EOK)
        {
            process_lvgl_message(&msg);
        }
    }
}
#else
/**
 * @brief UI refresh task entry function for timer-based message processing
 *
 * Processes LVGL messages from the queue using LVGL timer system.
 * This function is called periodically by the LVGL timer.
 *
 * @param task LVGL timer task pointer
 */
static void ui_refresh_task_entry(struct _lv_timer_t *task)
{
    lvgl_msg_t msg;
    while (rt_mq_recv(lvgl_mq, &msg, sizeof(lvgl_msg_t), RT_WAITING_NO) ==
           RT_EOK)
    {
        process_lvgl_message(&msg);
    }
}
#endif

/* ============== Initialization and Public Functions ============== */

/* Thread configuration */
#define THREAD_STACK_SIZE 14 * 256
#define THREAD_PRIORITY 9
#define THREAD_TIMESLICE 20

/**
 * @brief Initialize the LVGL task and message queue
 *
 * Sets up the message queue and either creates a dedicated thread
 * or LVGL timer for message processing based on configuration.
 *
 * @return int RT_EOK on success
 */
int lvgl_task_init(void)
{
#ifdef USE_LV_TIMER
    lvgl_mq = rt_mq_create("lvgl_mq", sizeof(lvgl_msg_t), 10, RT_IPC_FLAG_FIFO);
    lv_timer_create(ui_refresh_task_entry, 16, 0);
#else
    rt_thread_t tid =
        rt_thread_create("lvgl_task", lvgl_task, RT_NULL, THREAD_STACK_SIZE,
                         THREAD_PRIORITY, THREAD_TIMESLICE);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
#endif
    return RT_EOK;
}
INIT_APP_EXPORT(lvgl_task_init);

/**
 * @brief Send a message to the LVGL message queue
 *
 * Queues a message for processing by the UI handler task.
 * This is the primary interface for sending UI-related messages.
 *
 * @param msg Message structure to send
 */
void lvgl_send_msg(lvgl_msg_t msg)
{
    rt_mq_send(lvgl_mq, &msg, sizeof(lvgl_msg_t));
    // process_lvgl_message(&msg);
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/