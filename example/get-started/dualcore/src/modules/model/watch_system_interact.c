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
#include "data_service_subscriber.h"
#include "power_manager_service.h"
#include "lv_ext_resource_manager.h"
#ifdef RT_USING_BLUETOOTH
#include "bf0_ble_common.h"
#endif
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
#endif
#include "gui_app_pm.h"  /* always — gui_pm_fsm/gui_is_active called unconditionally */
#ifdef RT_USING_BLUETOOTH
#include "bf0_sibles_internal.h"
#include "bf0_ble_gap.h"
#endif

#define DBG_TAG "watch.interact"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* Cross-module symbols reached from individual case bodies. Moved here from
   inline `extern` decls inside switch cases. */
extern void set_open_scrolling_app_flag(bool flag);
extern char qrcode_data[256];
extern uint8_t get_ai_coding(void);
extern bool get_is_open_app_list_ai(void);
extern bool get_is_open_instruction_list_ai(void);
extern void append_text_to_input_message(void);
extern void append_text_to_mouse_input(void);
extern void load_instruction_list(void);
extern int subscribe_alarm_client(void);
#if !kReleaseMode
extern bool ppg_data_collection;
extern void blebredr_rf_power_set(uint8_t type, int8_t txpwr);
extern void chack_tile_page(void);
#endif

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
        switch_watch_motion_control_mode(true, false);
        set_open_scrolling_app_flag(true);
        animate_to_instruction_list();
    }
    else
    {
        switch_watch_motion_control_mode(true, true);
    }
}

/* Common haptic-fire path: respects the global motor-on-off switch and
   forwards to the peripheral provider. Period is in microseconds. */
static void motor_play_if_enabled(uint8_t duty, uint32_t period_us,
                                   uint8_t repeats)
{
    if (!get_motor_switch_state()) return;
    motor_params_t params = {
        .duty_cycle   = duty,
        .period       = period_us,
        .repeat_times = repeats,
    };
    peripheral_provider.control_motor(true, &params);
}

void motor_pattern_calling(void)
{
    motor_play_if_enabled(51, 1000000, 3); /* 1000ms × 3 */
}

void motor_pattern_notification(void)
{
    if (SkaiWatchSys.DNDMode.config.status) return; /* DND mode */
    motor_play_if_enabled(51, 50000, 2); /* 50ms × 2 */
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
    case SENSOR_TYPE_PPG:
    {
#if !kReleaseMode
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

/* All single-burst motor patterns differ only in (duty, period_us, repeats).
   Period comments below match the actual µs values; the legacy comments on
   damping/tap/unlocked were stale (claimed 200ms / 30ms but values were
   9ms/9ms/25ms). Wire-equivalent: same params struct, same provider call. */
void motor_pattern_wheel_scrolling(void)     { motor_play_if_enabled(100, 100000, 1); /* 100ms */ }
void motor_pattern_scrolling_app(void)       { motor_play_if_enabled(100,  12000, 1); /*  12ms */ }
void motor_pattern_touchpad_slide(void)      { motor_play_if_enabled(100,  10000, 1); /*  10ms */ }
void motor_pattern_screen_on_longpress(void) { motor_play_if_enabled(100, 200000, 1); /* 200ms */ }
void motor_pattern_damping(void)             { motor_play_if_enabled(100,   9000, 1); /*   9ms */ }
void motor_pattern_tap(void)                 { motor_play_if_enabled(100,   9000, 1); /*   9ms */ }
void motor_pattern_alarm(void)               { motor_play_if_enabled( 51, 500000, 30); /* 500ms × 30 */ }
void motor_pattern_normal(void)              { motor_play_if_enabled( 51, 100000, 1); /* 100ms */ }
void motor_pattern_timer_reminder(void)      { motor_play_if_enabled( 51, 500000, 10); /* 500ms × 10 */ }
void motor_pattern_unlocked(void)            { motor_play_if_enabled(100,  25000, 1); /*  25ms */ }

void motor_pattern_stop(void)
{
    peripheral_provider.control_motor(false, NULL);
}

#ifdef RGB_LED_CONTROL_PIN
static void led_pattern_rgb_led_close(void)
{
    peripheral_provider.control_rgb_led(false, NULL);
    LOG_D("led_pattern_rgb_led_close");
    rgb_led_params_t params = {
        .color = {.red = 0, .green = 0, .blue = 0},
        .brightness = 0,
        .animation_mode = RGB_ANIM_STATIC,
        .period_ms = 500,
        .repeat_times = 0, // infinite
    };
    peripheral_provider.control_rgb_led(true, &params);
}

static void led_pattern_rgb_led_open_write(uint8_t brightness)
{
    peripheral_provider.control_rgb_led(false, NULL);
    LOG_D("led_pattern_rgb_led_open_write");
    rgb_led_params_t params = {
        .color = {.red = 255, .green = 255, .blue = 255},
        .brightness = brightness,
        .animation_mode = RGB_ANIM_STATIC,
        .period_ms = 500,
        .repeat_times = 0, // infinite
    };
    peripheral_provider.control_rgb_led(true, &params);
}

static void led_pattern_rgb_led_breathing_blue(uint8_t brightness)
{
    rgb_led_params_t params = {
        .color = {.red = 9, .green = 0, .blue = 255},
        .brightness = brightness,
        .animation_mode = RGB_ANIM_FADE,
        .period_ms = 500,
        .repeat_times = 1, // infinite
    };
    peripheral_provider.control_rgb_led(true, &params);
}

static void led_pattern_rgb_led_open_green(uint8_t brightness)
{
    peripheral_provider.control_rgb_led(false, NULL);
    rgb_led_params_t params = {
        .color = {.red = 0, .green = 255, .blue = 0},
        .brightness = brightness,
        .animation_mode = RGB_ANIM_STATIC,
        .period_ms = 500,
        .repeat_times = 0, // infinite
    };
    peripheral_provider.control_rgb_led(true, &params);
}

static void led_pattern_rgb_led_breathing_green(uint8_t brightness)
{
    rgb_led_params_t params = {
        .color = {.red = 0, .green = 255, .blue = 0},
        .brightness = brightness,
        .animation_mode = RGB_ANIM_FADE,
        .period_ms = 2000,
        .repeat_times = 0, // infinite
    };
    peripheral_provider.control_rgb_led(true, &params);
}

static void led_pattern_rgb_led_fad_wight(uint8_t brightness)
{
    rgb_led_params_t params = {
        .color = {.red = 255, .green = 255, .blue = 255},
        .brightness = brightness,
        .animation_mode = RGB_ANIM_FADE,
        .period_ms = 1000,
        .repeat_times = 1, // infinite
    };
    peripheral_provider.control_rgb_led(true, &params);
}
#endif // RGB_LED_CONTROL_PIN

// Extract app management handling into a separate function
static void handle_app_management(INTERACT_Type type, void *pValue)
{
    switch (type)
    {
    case INTERACT_FUNCTION_MENU_MAIN:
        gui_app_run("Main");
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
        if (get_speech_coding() != msgData->header)
        {
            break;
        }
        if (gui_app_is_actived(APP_ID_SPEECH) ||
            gui_app_is_actived(APP_ID_MESSAGE))
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
            append_text_to_input_message();
        }
        else if (gui_app_is_actived(APP_ID_MOUSE))
        {
            handle_v2t_result(msgData);
            append_text_to_mouse_input();
        }
        break;
    }

    case INTERACT_CHAT_RESULT:
    {
        LOG_D("[INTERACT_CHAT_RESULT] handle chat result");
        MSG_DATA_PAYLOAD *msgData = (MSG_DATA_PAYLOAD *)pValue;
        /* Accept reply if voice_recognition's speech_bg is shown OR the
           instruction-list AI widget is open (speech_bg may be NULL in the
           widget-integrated flow). */
        lv_obj_t *sbg = gui_app_get_gesture_indicator()->speech_bg;
        bool speech_bg_shown = sbg && lv_obj_is_valid(sbg) &&
                               !lv_obj_has_flag(sbg, LV_OBJ_FLAG_HIDDEN);
        if (speech_bg_shown || get_is_open_instruction_list_ai())
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
        LOG_D("[INTERACT_LOGIN]");
        SkaiWatchSys.flag_field.device_had_logged = true;
    }
    break;

    case INTERACT_CANCEL_BOND:
    {
        LOG_D("[INTERACT_CANCEL_BOND]");
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

#ifdef BSP_USING_WATCH_SYS_CLIENT
void set_watch_sleep_state(const watch_sys_sleep_state_t *state)
{
    SkaiWatchSys.sleep_state = *state;
    LOG_D("[Sleep State]total_seconds=%d | total_restful_seconds=%d",
          SkaiWatchSys.sleep_state.total_seconds,
          SkaiWatchSys.sleep_state.total_restful_seconds);
}
#endif

// Extract system control handling into a separate function
static void handle_system_control(INTERACT_Type type, void *pValue)
{
    switch (type)
    {
#ifdef RGB_LED_CONTROL_PIN
    case INTERACT_RGB_LED_OPEN_WRITE:
    {
        led_pattern_rgb_led_open_write(*(uint8_t *)pValue);
        break;
    }
    case INTERACT_RGB_LED_OPEN_GREEN:
    {
        led_pattern_rgb_led_open_green(*(uint8_t *)pValue);
        break;
    }
    case INTERACT_RGB_LED_OPEN_BLUE:
    {
        led_pattern_rgb_led_breathing_blue(*(uint8_t *)pValue);
        break;
    }
    case INTERACT_RGB_LED_CLOSE:
    {
        led_pattern_rgb_led_close();
        break;
    }
    case INTERACT_RGB_LED_BREATHING_GREEN:
    {
        led_pattern_rgb_led_breathing_green(*(uint8_t *)pValue);
        break;
    }
    case INTERACT_RGB_LED_FADE_WIGHT:
    {
        led_pattern_rgb_led_fad_wight(*(uint8_t *)pValue);
        break;
    }
#endif // RGB_LED_CONTROL_PIN
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
        load_instruction_list();
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
        subscribe_alarm_client();
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
    case WATCH_OPEN_DISPLAY_TO_APP_LIST:
        set_user_want_to_open_display_to_instruction_list(true);
        break;
    case HCPU_WAKEUP:
    {
        if (!gui_is_active())
        {
            gui_pm_fsm(GUI_PM_ACTION_BUTTON_CLICKED);
            peripheral_provider.hcpu_resume();
        }
        break;
    }
    case WATCH_SLEEP:
    {
        if (!get_motor_status())
        {
            peripheral_provider.hcpu_suspend();
            gui_pm_fsm(GUI_PM_ACTION_SLEEP);
        }
        break;
    }
    case WATCH_GESTURE_UNLOCK:
        handle_gesture_unlock();
        break;

    case WATCH_REBOOT:
        peripheral_provider.hcpu_reboot();
        break;

    case STANDBY_WAKEUP:
    {
        SkaiWatchSys.sys_power_status = 0;
        sys_poweron_fsm(SYS_PWRON_EVT_BUTTON_LONG_PRESSED);
        break;
    }
#ifdef BSP_USING_WATCH_SYS_CLIENT
    case WATCH_REQUEST_BATTERY:
        watch_sys_sync.request_battery_voltage();
        break;

    case WATCH_REQUEST_CHARGE_STATUS:
        watch_sys_sync.request_charge_status();
        break;
#endif
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
        else if (strcmp(argv[1], "-reboot_hcpu") == 0)
        {
            watch_system_interact(WATCH_REBOOT, NULL);
        }
        else if (strcmp(argv[1], "-sleep") == 0)
        {
            watch_system_interact(WATCH_SLEEP, NULL);
        }
#ifdef RT_USING_BLUETOOTH
        // ----- Bluetooth
        else if (strcmp(argv[1], "-set_ble_rf") == 0)
        {
            int watch_ble_rf_power = atoi(argv[2]);
            blebredr_rf_power_set(0, watch_ble_rf_power);
        }
        else if (strcmp(argv[1], "-get_rssi") == 0)
        {
            ble_gap_get_rssi_t rssi;
            rssi.conn_idx = 0;
            uint8_t ret = ble_gap_get_remote_rssi(&rssi);
            LOG_D("ble_gap_get_remote_rssi ret:%d", ret);
        }
#endif
        else if (strcmp(argv[1], "-chack_tile") == 0)
        {
            chack_tile_page();
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
            peripheral_provider.control_motor(false, NULL);
        }
        else if (strcmp(argv[1], "-alarm") == 0)
        {
            motor_pattern_alarm();
        }
        else if (strcmp(argv[1], "-find_watch") == 0)
        {
            watch_system_interact(INTERACT_FIND_WATCH, NULL);
        }
        else if (strcmp(argv[1], "-scrolling_app") == 0)
        {
            motor_pattern_scrolling_app();
        }
        else if (strcmp(argv[1], "-sliding") == 0)
        {
            motor_pattern_touchpad_slide();
        }
        else if (strcmp(argv[1], "-long_pressed") == 0)
        {
            motor_pattern_screen_on_longpress();
        }
        else if (strcmp(argv[1], "-damping") == 0)
        {
            motor_pattern_damping();
        }
    }
    return 0;
}
MSH_CMD_EXPORT(control_motor, "control_motor [OPTION] ...");

    #ifdef RGB_LED_CONTROL_PIN
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
            uint8_t led_brightness = 20;
            watch_system_interact(INTERACT_RGB_LED_OPEN_WRITE, &led_brightness);
        }
        else if (strcmp(argv[1], "-open_green") == 0)
        {
            uint8_t led_brightness = 20;
            watch_system_interact(INTERACT_RGB_LED_OPEN_GREEN, &led_brightness);
        }
        else if (strcmp(argv[1], "-breathing_green") == 0)
        {
            uint8_t led_brightness = 20;
            watch_system_interact(INTERACT_RGB_LED_BREATHING_GREEN,
                                  &led_brightness);
        }
        else if (strcmp(argv[1], "-fade_wight") == 0)
        {
            uint8_t led_brightness = 20;
            watch_system_interact(INTERACT_RGB_LED_FADE_WIGHT, &led_brightness);
        }
        else if (strcmp(argv[1], "-breathing_blue") == 0)
        {
            uint8_t led_brightness = 20;
            watch_system_interact(INTERACT_RGB_LED_OPEN_BLUE, &led_brightness);
        }
    }
    return 0;
}
MSH_CMD_EXPORT(control_led, "control_led [OPTION] ...");
    #endif // RGB_LED_CONTROL_PIN

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
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/