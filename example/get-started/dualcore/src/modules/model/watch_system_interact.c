/**
 ******************************************************************************
 * @file   watch_system_interact.c
 * @author Skaiwalk software development team
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

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <board.h>
#include "watch_system_interact.h"
#include "watch_system_core_task.h"
#include "data_service_subscriber.h"
#include "power_manager_service.h"
#include "lv_ext_resource_manager.h"
#include "bf0_ble_common.h"
#include "app_mainmenu.h"
#include "ui_handler.h"
#include "ui_helper.h"
#ifdef BSP_USING_MAHONY_AHRS
    #include "sensor_fusion.h"
#endif
#ifdef BSP_USING_BLOC
    #include "bloc_control.h"
    #include "bloc_peripheral.h"
    #include "bloc_notification.h"
    #include "drv_touch.h"
    #include "bloc_setting.h"
    #include "bloc_v2t.h"
    #include "bloc_skaiwalk.h"
    #include "bloc_flash.h"
    #include "bloc_motion_tracking.h"
    #include "bloc_system_perception.h"
#endif
#ifdef BSP_USING_WATCH_SYS_CLIENT
    #include "watch_sys_service.h"
#endif
#ifdef BSP_USING_COMMUNICATE
    #include "communicate_protocol.h"
#endif
#ifdef BSP_USING_PM
    #include "bf0_pm.h"
    #include "gui_app_pm.h"
#endif
#include "bf0_sibles_internal.h"
#include "bf0_ble_gap.h"

#define DBG_TAG "watch.interact"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/**
 * @brief Is
 *
 * Launches the test application with "all" intent
 */
bool get_idle_state(void)
{
    return SkaiWatchSys.idle_state;
}

void set_idle_state(bool state)
{
    SkaiWatchSys.idle_state = state;
}

void switch_watch_motion_control_mode(bool enable, bool animation)
{
    if (SkaiWatchSys.motion_control_lock == !enable)
    {
        return;
    }
    SkaiWatchSys.motion_control_lock = !enable;
    LOG_D("%s %d", __func__, SkaiWatchSys.motion_control_lock);
    if (animation)
    {
        if (enable)
        {
            lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_UNGRAB_INDICATOR,
                              .data.gesture = 1};
            lvgl_send_msg(msg);
        }
        else
        {
            lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_UNGRAB_INDICATOR,
                              .data.gesture = 0};
            lvgl_send_msg(msg);
        }
    }
    // send_sys_interact_event(SYS_EVENT_WATCH_LOCK);
    watch_sys_sync.sync_api_lock(SkaiWatchSys.motion_control_lock);
}

void handle_gesture_unlock(void)
{
    if (SkaiWatchSys.sys_power_status == SYS_POWER_STATUS_ON)
    {
        lv_disp_trig_activity(NULL);
    }

    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_UNGRAB_EVENT};
    lvgl_send_msg(msg);

    if (gui_app_is_actived(APP_ID_GAME_DINOSAUR))
    {
        return;
    }

    if (is_at_home())
    {
        switch_watch_motion_control_mode(true, true);
        extern void set_open_scrolling_app_flag(bool flag);
        set_open_scrolling_app_flag(true);
        extern void set_q_vertical_movement_magnification(float mag);
        set_q_vertical_movement_magnification(5.0f);
        watch_system_interact(INTERACT_MOTOR_VIBRATE_UNLOCKED, NULL);
        animate_to_app_list();
    }
    else
    {
        switch_watch_motion_control_mode(true, true);
    }
}

void motor_pattern_calling(void)
{
    if (get_motor_switch_state())
    {
        motor_params_t params = {
            .duty_cycle = 51,
            .period = 100000, // 400ms
            .repeat_times = 3,
        };
        peripheral_provider.control_motor(true, &params);
    }
}

void motor_pattern_notification(void)
{
    if (SkaiWatchSys.DNDMode.config.status) // DND mode
    {
        return;
    }
    if (get_motor_switch_state())
    {
        motor_params_t params = {
            .duty_cycle = 51,
            .period = 100000, // 175ms
            .repeat_times = 2,
        };
        peripheral_provider.control_motor(true, &params);
    }
}

// Extract sensor handling into a separate function
static void
handle_sensor_subscription(sensor_subscription_t sensor_subscription)
{
    switch (sensor_subscription.type)
    {
    case SENSOR_TYPE_ACCELEROMETER:
    {
        if (sensor_subscription.status)
        {
            LOG_D("request accelerometer subscribe on");
            if (sensor_subscription.thread_safe)
            {
                peripheral_provider.subscribe_accelerometer_sensor(true);
            }
            else
            {
                accelerometer_subscribe();
            }
        }
        else
        {
            LOG_D("request accelerometer subscribe off");
            if (sensor_subscription.thread_safe)
            {
                peripheral_provider.subscribe_accelerometer_sensor(false);
            }
            else
            {
                accelerometer_unsubscribe();
            }
        }
        break;
    }
    case SENSOR_TYPE_GYROSCOPE:
    {
        if (sensor_subscription.status)
        {
            LOG_D("request gyroscope subscribe on");
            peripheral_provider.subscribe_gyroscope_sensor(true);
        }
        else
        {
            LOG_D("request gyroscope subscribe off");
            peripheral_provider.subscribe_gyroscope_sensor(false);
        }
        break;
    }
    case SENSOR_TYPE_MAGNETOMETER:
    {
        if (sensor_subscription.status)
        {
            LOG_D("request magnetometer subscribe on");
            peripheral_provider.subscribe_magnetometer_sensor(true);
        }
        else
        {
            LOG_D("request magnetometer subscribe off");
            peripheral_provider.subscribe_magnetometer_sensor(false);
        }
        break;
    }
    case SENSOR_TYPE_PPG:
    {
#if !kReleaseMode
        extern bool ppg_data_collection;
        if (sensor_subscription.status)
        {
            LOG_D("request ppg subscribe on");
            peripheral_provider.subscribe_ppg_signal(true);
            ppg_data_collection = true;
        }
        else
        {
            LOG_D("request ppg subscribe off");
            peripheral_provider.subscribe_ppg_signal(false);
            ppg_data_collection = false;
        }
#endif
        break;
    }
    case SENSOR_TYPE_HEART_RATE:
    {
        if (sensor_subscription.status)
        {
            LOG_D("request heart rate subscribe on");
            peripheral_provider.subscribe_hr_sensor(true);
        }
        else
        {
            LOG_D("request heart rate subscribe off");
            peripheral_provider.subscribe_hr_sensor(false);
        }
        break;
    }
    case SENSOR_TYPE_MIC:
    {
        if (sensor_subscription.status)
        {
            LOG_D("request mic subscribe on");
            peripheral_provider.subscribe_audio_mic_sensor(true);
        }
        else
        {
            LOG_D("request mic subscribe off");
            peripheral_provider.subscribe_audio_mic_sensor(false);
        }
        break;
    }
    default:
        break;
    }
}

static bool motor_switch_state = 1;

bool get_motor_switch_state(void)
{
    return motor_switch_state;
}
void set_motor_switch_state(uint8_t state)
{
    if (motor_switch_state != state)
    {
        motor_switch_state = state;
    }
}

// Motor pattern functions - each pattern wrapped in a separate function
static void motor_pattern_wheel_scrolling(void)
{
    if (get_motor_switch_state())
    {
        motor_params_t params = {
            .duty_cycle = 100,
            .period = 100000, // 100ms
            .repeat_times = 1,
        };
        peripheral_provider.control_motor(true, &params);
    }
}

void motor_pattern_scrolling_app(void)
{
    if (get_motor_switch_state())
    {
        motor_params_t params = {
            .duty_cycle = 100,
            .period = 12000, // 12ms
            .repeat_times = 1,
        };
        peripheral_provider.control_motor(true, &params);
    }
}

static void motor_pattern_touchpad_slide(void)
{
    if (get_motor_switch_state())
    {
        motor_params_t params = {
            .duty_cycle = 100,
            .period = 10000, // 10ms
            .repeat_times = 1,
        };
        peripheral_provider.control_motor(true, &params);
    }
}

static void motor_pattern_screen_on_longpress(void)
{
    if (get_motor_switch_state())
    {
        motor_params_t params = {
            .duty_cycle = 100,
            .period = 200000, // 200ms
            .repeat_times = 1,
        };
        peripheral_provider.control_motor(true, &params);
    }
}

static void motor_pattern_alarm(void)
{
    if (get_motor_switch_state())
    {
        motor_params_t params = {
            .duty_cycle = 51,
            .period = 500000, // 500ms
            .repeat_times = 30,
        };
        peripheral_provider.control_motor(true, &params);
    }
}

static void motor_pattern_ble_connected(void)
{
    if (get_motor_switch_state())
    {
        motor_params_t params = {
            .duty_cycle = 51,
            .period = 100000, // 100ms
            .repeat_times = 1,
        };
        peripheral_provider.control_motor(true, &params);
    }
}

static void motor_pattern_timer_reminder(void)
{
    if (get_motor_switch_state())
    {
        motor_params_t params = {
            .duty_cycle = 51,
            .period = 500000, // 500ms
            .repeat_times = 10,
        };
        peripheral_provider.control_motor(true, &params);
    }
}

static void motor_pattern_unlocked(void)
{
    if (get_motor_switch_state())
    {
        motor_params_t params = {
            .duty_cycle = 100,
            .period = 30000, // 30ms
            .repeat_times = 1,
        };
        peripheral_provider.control_motor(true, &params);
    }
}

static void motor_pattern_test(void)
{
    if (get_motor_switch_state())
    {
        motor_params_t params = {
            .duty_cycle = 70,
            .period = 90000, // 90ms
            .repeat_times = 1,
        };
        peripheral_provider.control_motor(true, &params);
    }
}

static void led_pattern_rgb_led_colse(void)
{
    peripheral_provider.control_rgb_led(false, NULL);
    LOG_D("led_pattern_rgb_led_colse");
    rgb_led_params_t params = {
        .color = {.red = 0, .green = 0, .blue = 0},
        .brightness = 0,
        .animation_mode = RGB_ANIM_STATIC,
        .period_ms = 500,
        .repeat_times = 0, // infinite
    };
    peripheral_provider.control_rgb_led(true, &params);
}

static void led_pattern_rgb_led_open_write(void)
{
    peripheral_provider.control_rgb_led(false, NULL);
    LOG_D("led_pattern_rgb_led_open_write");
    rgb_led_params_t params = {
        .color = {.red = 255, .green = 255, .blue = 255},
        .brightness = 50,
        .animation_mode = RGB_ANIM_STATIC,
        .period_ms = 500,
        .repeat_times = 0, // infinite
    };
    peripheral_provider.control_rgb_led(true, &params);
}

static void led_pattern_rgb_led_open_green(void)
{
    peripheral_provider.control_rgb_led(false, NULL);
    rgb_led_params_t params = {
        .color = {.red = 0, .green = 255, .blue = 0},
        .brightness = 30,
        .animation_mode = RGB_ANIM_STATIC,
        .period_ms = 500,
        .repeat_times = 0, // infinite
    };
    peripheral_provider.control_rgb_led(true, &params);
}

static void led_pattern_rgb_led_breathing_green(void)
{
    rgb_led_params_t params = {
        .color = {.red = 0, .green = 255, .blue = 0},
        .brightness = 30,
        .animation_mode = RGB_ANIM_FADE,
        .period_ms = 2000,
        .repeat_times = 0, // infinite
    };
    peripheral_provider.control_rgb_led(true, &params);
}

static void led_pattern_rgb_led_fad_wight(void)
{
    rgb_led_params_t params = {
        .color = {.red = 255, .green = 255, .blue = 255},
        .brightness = 30,
        .animation_mode = RGB_ANIM_FADE,
        .period_ms = 1000,
        .repeat_times = 1, // infinite
    };
    peripheral_provider.control_rgb_led(true, &params);
}

extern void handle_wakeup_event(void);

// Extract app management handling into a separate function
static void handle_app_management(INTERACT_Type type, void *pValue)
{
    switch (type)
    {
    case INTERACT_FUNCTION_MENU_MAIN:
        gui_app_run("Main");
        break;
    case INTERACT_CAMERA_MENU:
        break;
    case INTERACT_REBOOT_MENU:
        break;

    case INTERACT_BAT_LOW_LEVEL:
    {
        bool enable = *(bool *)pValue;
        if (enable)
        {
            AppIntent appIntent;
            strcpy(appIntent.app_id, APP_ID_INTERACT);
            strcpy(appIntent.intent, "low_power_warning");
            watch_run_app_by_intent(&appIntent);
        }
        else
        {
            watch_exit_app(APP_ID_INTERACT);
        }
        break;
    }

    case INTERACT_TASK_LOADING:
    {
        bool loading = *(bool *)pValue;
        lvgl_msg_t msg;
        msg.type = LVGL_MSG_TYPE_LOADING;
        msg.data.loading = loading;
        lvgl_send_msg(msg);
        break;
    }

    case INTERACT_SHOW_QRCODE:
    {
        LOG_D("[INTERACT_SHOW_QRCODE] QRCODE:%s", pValue);
        extern char qrcode_data[256];
        strcpy(qrcode_data, pValue);
        AppIntent appIntent;
        strcpy(appIntent.app_id, APP_ID_INTERACT);
        strcpy(appIntent.intent, "show_qrcode");
        watch_run_app_by_intent(&appIntent);
        break;
    }

    case INTERACT_FIND_WATCH:
    {
        AppIntent appIntent;
        strcpy(appIntent.app_id, APP_ID_INTERACT);
        strcpy(appIntent.intent, "find_me");
        watch_run_app_by_intent(&appIntent);
        break;
    }

    case INTERACT_TIMER_REMINDER:
    {
        // watch_hcpu_resume_with_reason(WAKEUP_REASON_OTHER);
        if (!gui_is_active())
        {
            gui_pm_fsm(GUI_PM_ACTION_BUTTON_CLICKED);
            peripheral_provider.hcpu_resume();
            rt_thread_mdelay(500);
        }
        gui_app_run(APP_ID_TIMER);
        break;
    }

    case INTERACT_LONG_SIT_ALERT:
    {
        if (!gui_app_is_actived(APP_ID_INTERACT))
        {
            AppIntent appIntent;
            strcpy(appIntent.app_id, APP_ID_INTERACT);
            strcpy(appIntent.intent, "long_sit_alert");
            watch_run_app_by_intent(&appIntent);
        }
        break;
    }

    case INTERACT_MIC_LISTEN:
    {
        bool enable = *(bool *)pValue;
        if (enable)
        {
            LOG_D("[INTERACT_MIC_LISTEN] Start voice recognition");
            voice_provider.vad_init();
            start_voice_recognition(V2T_INTENT_CHAT);
        }
        else
        {
            LOG_D("[INTERACT_MIC_LISTEN] Stop voice recognition");
            stop_voice_recognition(V2T_INTENT_MIC_INPUTE);
            voice_provider.vad_deinit();
        }
        break;
    }

    case INTERACT_MIC_V2T_INPUT:
    {
        LOG_D("[INTERACT_MIC_V2T_INPUT] Start voice recognition for V2T input");
        voice_provider.vad_init();
        start_voice_recognition(V2T_INTENT_MIC_INPUTE);
    }

    case INTERACT_VOICE_RECOGNITION:
    {
        app_gesture_indicator_t *gesture_indicator =
            gui_app_get_gesture_indicator();
        VOICE_RECOGNITION_PAYLOAD *msgData =
            (VOICE_RECOGNITION_PAYLOAD *)pValue;
        extern uint8_t get_ai_coding(void);
        extern bool get_is_open_app_list_ai(void);
        if (get_speech_coding() != msgData->header)
        {
            break;
        }
        if (gui_app_is_actived(APP_ID_SPEECH) ||
            gui_app_is_actived(APP_ID_NOTE_CHATROOM) || is_at_note_list())
        {
            LOG_D("[INTERACT_VOICE_RECOGNITION]:%d, coding:%d", msgData->header,
                  get_speech_coding());
            handle_v2t_result(msgData);
        }
        else if (check_if_user_speaking_to_ai())
        {
            LOG_D("[INTERACT_VOICE_RECOGNITION]:%d, coding:%d, ai_coding:%d",
                  msgData->header, get_speech_coding(), get_ai_coding());
            handle_v2t_result(msgData);
            lvgl_msg_t msg;
            msg.type = LVGL_MSG_TYPE_SPEECH_SHOW_BG;
            lvgl_send_msg(msg);
            extern void append_text_to_input_message();
            append_text_to_input_message();
        }
        break;
    }

    case INTERACT_CHAT_RESULT:
    {
        LOG_D("[INTERACT_CHAT_RESULT] handle chat result");
        MSG_DATA_PAYLOAD *msgData = (MSG_DATA_PAYLOAD *)pValue;
        if (!lv_obj_has_flag(gui_app_get_gesture_indicator()->speech_bg,
                             LV_OBJ_FLAG_HIDDEN))
        {
            handle_skai_message("quick_ai", msgData);
        }
        else
        {
            LOG_E("Unknown app_id");
        }
        break;
    }

    case INTERACT_LOGIN:
    {
        LOG_D("login success");
        SkaiWatchSys.flag_field.device_had_logged = true;
        // send_sys_interact_event(SYS_EVENT_BATT_VOLTAGE);
        // send_sys_interact_event(SYS_EVENT_BATT_CHARGE);
    }
    break;

    case INTERACT_CANCEL_BOND:
    {
        LOG_D("[INTERACT]Cancel Bond");
        SkaiWatchSys.flag_field.bond_state = false;
        SkaiWatchSys.flag_field.device_had_logged = false;

        /*clear user data*/
        SkaiWatchSys.gPedoData.global_steps = 0;
        SkaiWatchSys.gPedoData.global_distance = 0;
        SkaiWatchSys.gPedoData.global_calories = 0;
        SkaiWatchSys.gPedoData.quarter_steps = 0;
        SkaiWatchSys.gPedoData.quarter_distance = 0;
        SkaiWatchSys.gPedoData.quarter_calories = 0;

        /*erase user id */
        memset((void *)SkaiWatchSys.user_data.user_id, 0x00, USER_ID_LENGTH);
        // reset phone os version
        SkaiWatchSys.phone_os_version = NONE;
        SkaiWatchSys.flag_field.auto_sync_enable = false;
        peripheral_provider.save_watch_shared_prefs(WATCH_PREFS_KEY_FLAG_FIELD);
        rt_thread_mdelay(300);
        peripheral_provider.save_watch_shared_prefs(WATCH_PREFS_KEY_USER_DATA);
    }
    break;

    case INTERACT_BONDED:
    {
        if (SkaiWatchSys.flag_field.bond_state == false)
        {
            LOG_D("[INTERACT_BONDED] Just Bonded");
            memcpy((void *)SkaiWatchSys.user_data.user_id, pValue,
                   USER_ID_LENGTH);
            // change bond status machine
            SkaiWatchSys.flag_field.bond_state = true;
            /*set current user state*/
            SkaiWatchSys.user_data.user_profile.data = 0;
            SkaiWatchSys.gPedoData.daily_step_target = 10000;
            peripheral_provider.save_watch_shared_prefs(
                WATCH_PREFS_KEY_FLAG_FIELD);
            rt_thread_mdelay(300);
            peripheral_provider.save_watch_shared_prefs(
                WATCH_PREFS_KEY_USER_DATA);
        }
        else
        {
            LOG_D("[INTERACT_BONDED] Already bonded");
        }
    }
    break;

    case INTERACT_PAIRING:
    {
        bool enable = *(bool *)pValue;
        if (enable)
        {
            AppIntent appIntent;
            strcpy(appIntent.app_id, APP_ID_INTERACT);
            strcpy(appIntent.intent, "ble_pairing");
            watch_run_app_by_intent(&appIntent);
        }
        else
        {
            watch_exit_app(APP_ID_INTERACT);
        }
    }
    break;

    case INTERACT_CAMERA:
    {
        uint8_t status = *(uint8_t *)pValue;
        LOG_D("Camera status: %d", status);
        if (status == 0x00)
        {
            SkaiWatchSys.flag_field.phone_camera_status = true;
        }
        else if (status == 0x01)
        {
            SkaiWatchSys.flag_field.phone_camera_status = false;
        }
    }
    break;

    default:
        break;
    }
}

// Extract health monitoring handling into a separate function
static void handle_health_monitoring(INTERACT_Type type, void *pValue)
{
    switch (type)
    {
    case INTERACT_HeartRate:
    {
        uint8_t heartRate = *(uint8_t *)pValue;
        LOG_D("[INTERACT_HeartRate] %d", heartRate);
        break;
    }
    case INTERACT_BloodPressure:
        break;

    case INTERACT_HRS_DISPALY_VALUE:
        break;

    case INTERACT_SLEEP_STATE:
    {
        SkaiWatchSys.sleep_state = *(watch_sys_sleep_state_t *)pValue;
        LOG_D("[Sleep State]total_seconds=%d | total_restful_seconds=%d",
              SkaiWatchSys.sleep_state.total_seconds,
              SkaiWatchSys.sleep_state.total_restful_seconds);
        // if (SkaiWatchSys.watch_sleep_status != sleepState->mode)
        // {
        //   switch (SkaiWatchSys.watch_sleep_status)
        //   {
        //   case 1:
        //     SkaiWatchSys.sleep_data_show.light_sleep_time +=
        //     (sleepState->timestamp -
        //     SkaiWatchSys.sleep_data_show.prev_sleep_stamp); break;
        //   case 2:
        //     SkaiWatchSys.sleep_data_show.deep_sleep_time +=
        //     (sleepState->timestamp -
        //     SkaiWatchSys.sleep_data_show.prev_sleep_stamp); break;
        //   case 3:
        //     SkaiWatchSys.sleep_data_show.wake_up_time +=
        //     (sleepState->timestamp -
        //     SkaiWatchSys.sleep_data_show.prev_sleep_stamp); break;
        //   default:
        //     break;
        //   }
        //   SkaiWatchSys.watch_sleep_status = sleepState->mode;
        //   SkaiWatchSys.sleep_data_show.prev_sleep_stamp =
        //   sleepState->timestamp;
        // }
        // L1SendData data;
        // data.event = L1SEND_SLEEP_DATA;
        // data.res.status = sleepState->mode;
        // L1_send_event(data);
        break;
    }
    case INTERACT_HEARTRATEHIGH:
        LOG_D("[INTERACT_HEARTRATEHIGH]");
        break;
    case INTERACT_TARGET:
        LOG_D("[INTERACT_TARGET] daily health target achieved");
        break;
    case INTERACT_NO_MOVEMENT:
        LOG_D("[INTERACT_NO_MOVEMENT]");
        break;
    default:
        break;
    }
}

// Extract system control handling into a separate function
static void handle_system_control(INTERACT_Type type, void *pValue)
{
    switch (type)
    {
    case INTERACT_FIND_PHONE:
    {
        bool enable = *(bool *)pValue;
        break;
    }
    case INTERACT_MOTOR_VIBRATE_ALARM:
    {
        motor_pattern_alarm();
        break;
    }
    case INTERACT_MOTOR_VIBRATE_NOTIFICATION:
    {
        motor_pattern_notification();
        break;
    }
    case INTERACT_MOTOR_VIBRATE_SCROLLING:
    {
        motor_pattern_wheel_scrolling();
        break;
    }
    case INTERACT_MOTOR_VIBRATE_SCROLLING_APP:
    {
        motor_pattern_scrolling_app();
        break;
    }
    case INTERACT_MOTOR_VIBRATE_SLIDING:
    {
        motor_pattern_touchpad_slide();
        break;
    }
    case INTERACT_MOTOR_VIBRATE_LONG_PRESSED:
    {
        motor_pattern_screen_on_longpress();
        break;
    }
    case INTERACT_MOTOR_VIBRATE_BUTTON_PRESSED:
    {
        motor_pattern_ble_connected();
        break;
    }
    case INTERACT_MOTOR_VIBRATE_BUTTON_RELEASED:
    {
        motor_pattern_ble_connected();
        break;
    }
    case INTERACT_MOTOR_VIBRATE_BLE_CONNECTED:
    {
        motor_pattern_ble_connected();
        break;
    }
    case INTERACT_MOTOR_VIBRATE_TIMER_REMINDER:
    {
        motor_pattern_timer_reminder();
        break;
    }
    case INTERACT_MOTOR_VIBRATE_UNLOCKED:
    {
        motor_pattern_unlocked();
        break;
    }
    case INTERACT_MOTOR_VIBRATE_TEST:
    {
        motor_pattern_test();
        break;
    }
    case INTERACT_STOP_MOTOR_ONLY:
    {
        peripheral_provider.control_motor(false, NULL);
        break;
    }
    case INTERACT_STOP_OLED_ONLY:
        break;
    case INTERACT_STOP_MOTOR_AND_OLED:
        break;

    case INTERACT_RGB_LED_OPEN_WRITE:
    {
        led_pattern_rgb_led_open_write();
        break;
    }
    case INTERACT_RGB_LED_OPEN_GREEN:
    {
        led_pattern_rgb_led_open_green();
        break;
    }
    case INTERACT_RGB_LED_CLOSE:
    {
        led_pattern_rgb_led_colse();
        break;
    }
    case INTERACT_RGB_LED_BREATHING_GREEN:
    {
        led_pattern_rgb_led_breathing_green();
        break;
    }
    case INTERACT_RGB_LED_FADE_WIGHT:
    {
        led_pattern_rgb_led_fad_wight();
        break;
    }
#ifdef BSP_USING_BLOC_CONTROL
    case INTERACT_SHOW_MEDIA_TITLE:
    {
        char *title = (char *)pValue;
        control_provider.set_media_title(title);
        break;
    }

    case INTERACT_SYNC_MEDIA_STATUS:
    {
        uint8_t status = *(uint8_t *)pValue;
        if (status == 0x00)
        {
            LOG_D("[INTERACT_SYNC_MEDIA_STATUS]remote media pause");
            control_provider.notify_bt_speaker_media_status(false);
        }
        else if (status == 0x01)
        {
            LOG_D("[INTERACT_SYNC_MEDIA_STATUS]remote media play");
            control_provider.notify_bt_speaker_media_status(true);
        }
        break;
    }

#endif

    default:
        break;
    }
}

// Extract system settings into a separate function
static void handle_system_settings(INTERACT_Type type, void *pValue)
{
#ifdef BSP_USING_BLOC_SETTING

    switch (type)
    {
    case LANGUAGE_SET:
    {
        char *language = (char *)pValue;
        LOG_D("[LANGUAGE_SET] Language:%s", language);
        setting_provider.set_language(language);
        extern void load_app_list(void);
        load_app_list();
        break;
    }
    case WATCH_WATCHFACE_SET:
    {
        T_CLOCK_MENU_TYPE index = *(uint8_t *)pValue;
        LOG_D("Watchface index:%d", index);
        setting_provider.set_watch_face(index);
        break;
    }
    case WATCH_DND_MODE_SET:
    {
        uint8_t status = *(uint8_t *)pValue;
        LOG_D("DND mode:%d", status);
        setting_provider.set_dnd_status(status);
        break;
    }
    case WATCH_ALARM_INIT:
    {
        extern int subscribe_alarm_client(void);
        subscribe_alarm_client();
        break;
    }
    case WATCH_ALARM_GET:
        break;
    case WATCH_ALARM_SET:
        break;
    case WATCH_BRIGHTNESS_SET:
    {
        uint8_t brightness = *(uint8_t *)pValue;
        LOG_D("[WATCH_BRIGHTNESS_SET] Brightness:%d", brightness);
        gui_set_brightness(brightness, true);
        break;
    }
    case TIME_FORMAT_SET:
    {
        uint8_t format = *(uint8_t *)pValue;
        LOG_D("[TIME_FORMAT_SET] Time format:%d", format);
        setting_provider.set_hour_format(format);
        break;
    }
    case SCREEN_TIME_SET:
    {
        uint8_t time = *(uint8_t *)pValue;
        LOG_D("[SCREEN_TIME_SET] Screen time:%d", time);
        setting_provider.set_screen_time(time);
        break;
    }
    case LIFT_WRIST_DETECT_SET:
    {
        bool status = *(bool *)pValue;
        LOG_D("[LIFT_WRIST_DETECT_SET] Lift wrist detect:%d", status);
        setting_provider.set_lift_switch_status(status);
        break;
    }

    default:
        break;
    }

#endif
}

// Extract power management handling into a separate function
static void handle_power_management(INTERACT_Type type, void *pValue)
{
    switch (type)
    {
    case WATCH_OPEN_DISPLAY:
        // set_watch_ready_to_open_display(true);
        break;
    case WATCH_OPEN_DISPLAY_TO_APP_LIST:
        // set_watch_ready_to_open_display(true);
        set_user_want_to_open_display_to_app_list(true);
        break;
    case WATCH_PREPARE_SLEEP:
        // send_sys_interact_event(SYS_EVENT_PREPARE_SLEEP);
        break;
    case HCPU_WAKEUP:
    {
        // watch_hcpu_resume_with_reason(*(uint8_t *)pValue);
        if (!gui_is_active())
        {
            gui_pm_fsm(GUI_PM_ACTION_BUTTON_CLICKED);
            peripheral_provider.hcpu_resume();
        }
        break;
    }
    case WATCH_SLEEP:
    {
        // send_sys_interact_event(SYS_EVENT_HCPU_SUSPEND);
        peripheral_provider.hcpu_suspend();
        gui_pm_fsm(GUI_PM_ACTION_SLEEP);
        break;
    }
    case WATCH_GESTURE_UNLOCK:
    {
        handle_gesture_unlock();
        break;
    }
    case WATCH_REBOOT:
    {
        peripheral_provider.hcpu_reboot();
        break;
    }
    case STANDBY_WAKEUP:
    {
        SkaiWatchSys.sys_power_status = 0;
        sys_poweron_fsm(SYS_PWRON_EVT_BUTTON_LONG_PRESSED);
        break;
    }
    case WATCH_POWER_MANAGE:
    {
        uint8_t target = ((uint8_t *)pValue)[0];
        uint8_t status = ((uint8_t *)pValue)[1];
        if (target == 0x01)
        {
            LOG_D("[WATCH_POWER_MANAGER] System load switch 3.3v enable:%d",
                  status);
            // TODO: enable/disable 3.3v
        }
        break;
    }
    case WATCH_REQUEST_BATTERY:
    {
#ifdef BSP_USING_WATCH_SYS_CLIENT
        watch_sys_sync.request_battery_voltage();
#endif
        break;
    }

    case WATCH_REQUEST_CHARGE_STATUS:
    {
#ifdef BSP_USING_WATCH_SYS_CLIENT
        watch_sys_sync.request_charge_status();
#endif
        break;
    }

    default:
        break;
    }
}

/// @brief
/// @param type
/// @param pValue
/// @return pointer to the value
void *watch_system_interact(INTERACT_Type type, void *pValue)
{
    if (is_ble_dfu_thread_running())
    {
        LOG_W("BLE DFU thread is running, cannot interact with the system");
        return NULL;
    }

    if (type == SENSOR_INTERACT_TYPE)
    {
        sensor_subscription_t sensor_subscription =
            *(sensor_subscription_t *)pValue;
        handle_sensor_subscription(sensor_subscription);
    }
    else if (type >= APP_INTERACT_TYPE_BEGIN && type <= APP_INTERACT_TYPE_END)
    {
        handle_app_management(type, pValue);
    }
    else if (type >= HEALTH_INTERACT_TYPE_BEGIN &&
             type <= HEALTH_INTERACT_TYPE_END)
    {
        handle_health_monitoring(type, pValue);
    }
    else if (type >= CONTROL_INTERACT_TYPE_BEGIN &&
             type <= CONTROL_INTERACT_TYPE_END)
    {
        handle_system_control(type, pValue);
    }
    else if (type >= SETTINGS_INTERACT_TYPE_BEGIN &&
             type <= SETTINGS_INTERACT_TYPE_END)
    {
        handle_system_settings(type, pValue);
    }
    else if (type >= POWER_INTERACT_TYPE_BEGIN &&
             type <= POWER_INTERACT_TYPE_END)
    {
        handle_power_management(type, pValue);
    }
    return NULL;
}

#if !kReleaseMode
/// 系統設置
static int set_watch_system(int argc, char *argv[])
{
    if (argc >= 2)
    {
        // 設定語言
        // - language：[args] 要設定的語言縮寫。
        //   - en_us
        //   - zh_cn
        if (strcmp(argv[1], "-language") == 0)
        {
            if (argc == 3)
            {
                watch_system_interact(LANGUAGE_SET, argv[2]);
            }
        }
        // 設定錶盤
        // - watchface：[args] 要設定的錶盤編號。
        else if (strcmp(argv[1], "-watchface") == 0)
        {
            if (argc == 3)
            {
                uint8_t index = atoi(argv[2]);
                if (index >= 0 && index <= 8)
                {
                    watch_system_interact(WATCH_WATCHFACE_SET, &index);
                }
                else
                {
                    LOG_E("The watchface index should be between 0 and 8.");
                }
            }
        }
        // 設定DND模式
        // - dnd：[args] 要設定的DND模式。
        else if (strcmp(argv[1], "-dnd") == 0)
        {
            if (argc == 3)
            {
                uint8_t status = 0;
                if (strcmp(argv[2], "on") == 0)
                {
                    status = 1;
                    watch_system_interact(WATCH_DND_MODE_SET, &status);
                }
                else if (strcmp(argv[2], "off") == 0)
                {
                    watch_system_interact(WATCH_DND_MODE_SET, &status);
                }
            }
        }
        // 設定時間格式
        // - time_format：[args] 要設定的時間格式。
        else if (strcmp(argv[1], "-time_format") == 0)
        {
            if (argc == 3)
            {
                uint8_t format_code = 0;
                if (strcmp(argv[2], "12") == 0)
                {
                    watch_system_interact(TIME_FORMAT_SET, &format_code);
                }
                else if (strcmp(argv[2], "24") == 0)
                {
                    format_code = 1;
                    watch_system_interact(TIME_FORMAT_SET, &format_code);
                }
            }
        }
        // 設定螢幕亮度
        // - screen_time：[args] 要設定的螢幕亮度。
        else if (strcmp(argv[1], "-brightness") == 0)
        {
            if (argc == 3)
            {
                uint8_t brightness = atoi(argv[2]);
                if (brightness >= 0 && brightness <= 100)
                {
                    watch_system_interact(WATCH_BRIGHTNESS_SET, &brightness);
                }
                else
                {
                    LOG_E("The brightness should be between 0 and 100.");
                }
            }
        }
        // 亮屏時間
        // - screen_time：[args] 要設定的亮屏時間。
        else if (strcmp(argv[1], "-screen_time") == 0)
        {
            if (argc == 3)
            {
                uint8_t time = atoi(argv[2]);
                if (time >= 5 && time <= 30)
                {
                    watch_system_interact(SCREEN_TIME_SET, &time);
                }
                else
                {
                    LOG_E(
                        "The screen time should be between 5 and 30 seconds.");
                }
            }
        }
        // 設定翻腕亮屏
        // - lift_wrist：[args] 要設定的翻腕亮屏狀態。
        else if (strcmp(argv[1], "-lift_wrist") == 0)
        {
            if (argc == 3)
            {
                bool status = false;
                if (strcmp(argv[2], "on") == 0)
                {
                    status = true;
                    watch_system_interact(LIFT_WRIST_DETECT_SET, &status);
                }
                else if (strcmp(argv[2], "off") == 0)
                {
                    watch_system_interact(LIFT_WRIST_DETECT_SET, &status);
                }
            }
        }
        else if (strcmp(argv[1], "-shutdown") == 0)
        {
            LOG_D("TODO:Shutting down the watch...");
        }
        else if (strcmp(argv[1], "-reboot_hcpu") == 0)
        {
            watch_system_interact(WATCH_REBOOT, NULL);
        }
        else if (strcmp(argv[1], "-reboot_lcpu") == 0)
        {
            LOG_D("TODO:Rebooting the LCPU...");
        }
        else if (strcmp(argv[1], "-sleep") == 0)
        {
            watch_system_interact(WATCH_SLEEP, NULL);
        }
        // ----- Bluetooth
        else if (strcmp(argv[1], "-set_ble_rf") == 0)
        {
            int watch_ble_rf_power = atoi(argv[2]);
            extern void blebredr_rf_power_set(uint8_t type, int8_t txpwr);
            blebredr_rf_power_set(0, watch_ble_rf_power);
        }
        else if (strcmp(argv[1], "-get_rssi") == 0)
        {
            ble_gap_get_rssi_t rssi;
            rssi.conn_idx = 0;
            uint8_t ret = ble_gap_get_remote_rssi(&rssi);
            LOG_D("ble_gap_get_remote_rssi ret:%d", ret);
        }
    }
    return 0;
}
MSH_CMD_EXPORT(set_watch_system, "set_watch_system [OPTION] ...");

/// 控制馬達
static int control_motor(int argc, char *argv[])
{
    if (argc >= 2)
    {
        // 停止馬達
        if (strcmp(argv[1], "-stop") == 0)
        {
            watch_system_interact(INTERACT_STOP_MOTOR_ONLY, NULL);
        }
        else if (strcmp(argv[1], "-alarm") == 0)
        {
            watch_system_interact(INTERACT_MOTOR_VIBRATE_ALARM, NULL);
        }
        else if (strcmp(argv[1], "-find_watch") == 0)
        {
            watch_system_interact(INTERACT_FIND_WATCH, NULL);
        }
        else if (strcmp(argv[1], "-scrolling_app") == 0)
        {
            watch_system_interact(INTERACT_MOTOR_VIBRATE_SCROLLING_APP, NULL);
        }
        else if (strcmp(argv[1], "-sliding") == 0)
        {
            watch_system_interact(INTERACT_MOTOR_VIBRATE_SLIDING, NULL);
        }
        else if (strcmp(argv[1], "-long_pressed") == 0)
        {
            watch_system_interact(INTERACT_MOTOR_VIBRATE_LONG_PRESSED, NULL);
        }
    }
    return 0;
}
MSH_CMD_EXPORT(control_motor, "control_motor [OPTION] ...");

static int control_led(int argc, char *argv[])
{
    if (argc >= 2)
    {
        if (strcmp(argv[1], "-close") == 0)
        {
            watch_system_interact(INTERACT_RGB_LED_CLOSE, NULL);
        }
        else if (strcmp(argv[1], "-open_write") == 0)
        {
            watch_system_interact(INTERACT_RGB_LED_OPEN_WRITE, NULL);
        }
        else if (strcmp(argv[1], "-open_green") == 0)
        {
            watch_system_interact(INTERACT_RGB_LED_OPEN_GREEN, NULL);
        }
        else if (strcmp(argv[1], "-breathing_green") == 0)
        {
            watch_system_interact(INTERACT_RGB_LED_BREATHING_GREEN, NULL);
        }
        else if (strcmp(argv[1], "-fade_wight") == 0)
        {
            watch_system_interact(INTERACT_RGB_LED_FADE_WIGHT, NULL);
        }
    }
    return 0;
}
MSH_CMD_EXPORT(control_led, "control_led [OPTION] ...");

static int utest_user_speech_intent(int argc, char *argv[])
{
    if (argc >= 2)
    {
        if (strcmp(argv[1], "-reply") == 0)
        {
            if (argc == 3)
            {
                app_voice_set_voice2text_intent(V2T_INTENT_REMOTE_INPUT);
                strcpy(replying_notification_id, get_cur_notification()->id);
                setVoice2Text(argv[2]);
                char *text = get_combined_voice2text();
                handle_user_speech_intent(V2T_INTENT_REMOTE_INPUT, text);
                clearVoice2Text();
            }
            else
            {
                LOG_E("Please input the text to reply");
            }
        }
        else if (strcmp(argv[1], "-chat") == 0)
        {
            if (argc == 3)
            {
                app_voice_set_voice2text_intent(V2T_INTENT_CHAT);
                setVoice2Text(argv[2]);
                char *text = get_combined_voice2text();
                handle_user_speech_intent(V2T_INTENT_CHAT, text);
                clearVoice2Text();
            }
            else
            {
                LOG_E("Please input the text to chat");
            }
        }
    }
    return 0;
}
MSH_CMD_EXPORT(utest_user_speech_intent,
               "utest_user_speech_intent [OPTION] ...");

/// 測試高斯模糊
static int test_gaussian_blur(int argc, char *argv[])
{
    if (argc >= 2)
    {
        /// 創建高斯模糊
        if (strcmp(argv[1], "-create") == 0)
        {
            lvgl_msg_t msg;
            msg.type = LVGL_MSG_TYPE_GAUSSIAN_BLUR;
            lvgl_send_msg(msg);
        }
    }
    return 0;
}
MSH_CMD_EXPORT(test_gaussian_blur, "test_gaussian_blur");
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/