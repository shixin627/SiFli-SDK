/**
 ******************************************************************************
 * @file   watch_system_interact.h
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered, decompiled, modified,
 *    or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __WATCH_SYSTEM_INTERACT_H__
#define __WATCH_SYSTEM_INTERACT_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#ifdef BSP_USING_MODEL_WATCH_GLOBAL_DATA
#include "watch_global_data.h"
#endif
#ifdef BSP_USING_WATCH_SYS_CLIENT
#include "watch_sys_service.h"
#endif
#ifdef BSP_USING_BLE_LOG
#include "wristband_ble_log.h"
#else
extern void BLE_LOG_D(const char *format, ...);
extern void BLE_LOG_I(const char *format, ...);
extern void BLE_LOG_W(const char *format, ...);
extern void BLE_LOG_E(const char *format, ...);
#endif

#define SYS_EVENT_POWER_ON (1 << 0)
#define SYS_EVENT_REBOOT_SYS (1 << 1)
#define SYS_EVENT_POWER_OFF (1 << 2)
#define SYS_EVENT_REBOOT_LCPU (1 << 3)
#define SYS_EVENT_SAVE_SHARE_PREFS (1 << 4)
#define SYS_EVENT_BATT_VOLTAGE (1 << 5)
#define SYS_EVENT_BATT_CHARGE (1 << 6)
#define SYS_EVENT_PREPARE_SLEEP (1 << 7)
#define SYS_EVENT_HCPU_RESUME (1 << 8)
#define SYS_EVENT_HCPU_SUSPEND (1 << 9)
#define SYS_EVENT_WATCH_LOCK (1 << 10)
#define SYS_EVENT_RELOAD_GESTURE_MODEL (1 << 11)

#define WATCH_SYS_ALL_EVENT (             \
    SYS_EVENT_POWER_ON |                  \
    SYS_EVENT_REBOOT_SYS |                \
    SYS_EVENT_POWER_OFF |                 \
    SYS_EVENT_REBOOT_LCPU |               \
    SYS_EVENT_SAVE_SHARE_PREFS |          \
    SYS_EVENT_BATT_VOLTAGE |              \
    SYS_EVENT_BATT_CHARGE |               \
    SYS_EVENT_PREPARE_SLEEP |             \
    SYS_EVENT_HCPU_RESUME |               \
    SYS_EVENT_HCPU_SUSPEND |              \
    SYS_EVENT_WATCH_LOCK |               \
    SYS_EVENT_RELOAD_GESTURE_MODEL)

    typedef enum
    {
        V2T_INTENT_NOTHING = 0x00,
        V2T_INTENT_CHAT = 0x01,
        V2T_INTENT_REMOTE_INPUT = 0x02,
        V2T_INTENT_NOTE_CREATING = 0x03,
        V2T_INTENT_MIC_INPUTE = 0x04,
    } V2T_INTENT;

    /// @brief
    typedef enum
    {
        /***** Menu *******/
        INTERACT_BEGIN = 1,
/**** All menu should place in this group ****/
//////////
/// App manage

// App Management Types
#define APP_INTERACT_TYPE_BEGIN INTERACT_FUNCTION_MENU_MAIN
#define APP_INTERACT_TYPE_END INTERACT_CAMERA
        INTERACT_FUNCTION_MENU_MAIN,
        INTERACT_BAT_LOW_LEVEL,
        INTERACT_TASK_LOADING,
        INTERACT_SHOW_QRCODE,
        INTERACT_FIND_WATCH,
        INTERACT_TIMER_REMINDER,
        INTERACT_MIC_LISTEN,
        INTERACT_MIC_V2T_INPUT,
        INTERACT_VOICE_RECOGNITION,
        INTERACT_CHAT_RESULT,
        INTERACT_LOGIN,
        INTERACT_CANCEL_BOND,
        INTERACT_BONDED,
        INTERACT_PAIRING,
        INTERACT_CAMERA,

// System Control Types
#define CONTROL_INTERACT_TYPE_BEGIN INTERACT_RGB_LED_OPEN_WRITE
#define CONTROL_INTERACT_TYPE_END INTERACT_SYNC_MEDIA_STATUS

        /*****  LED *******/
        INTERACT_RGB_LED_OPEN_WRITE,
        INTERACT_RGB_LED_OPEN_GREEN,
        INTERACT_RGB_LED_OPEN_BLUE,
        INTERACT_RGB_LED_CLOSE,
        INTERACT_RGB_LED_BREATHING_GREEN,
        INTERACT_RGB_LED_FADE_WIGHT,
        /***** Media ******/
        INTERACT_SHOW_MEDIA_TITLE,
        INTERACT_SYNC_MEDIA_STATUS,
///////////////////////////////
/// Settings

// System Settings Types
#define SETTINGS_INTERACT_TYPE_BEGIN WATCH_WATCHFACE_SET
#define SETTINGS_INTERACT_TYPE_END LIFT_WRIST_DETECT_SET

        /***** Watchface *****/
        WATCH_WATCHFACE_SET,
        /***** DND mode *****/
        WATCH_DND_MODE_SET,
        /***** Alarm *****/
        WATCH_ALARM_INIT,
        /***** Brightness *****/
        WATCH_BRIGHTNESS_SET,
        /***** Time *****/
        TIME_FORMAT_SET,
        SCREEN_TIME_SET,
        /***** Language *****/
        LANGUAGE_SET,
        /***** Interact *****/
        LIFT_WRIST_DETECT_SET,
///////////////////////
/// Power

// Power Management Types
#define POWER_INTERACT_TYPE_BEGIN WATCH_SLEEP
#define POWER_INTERACT_TYPE_END WATCH_REQUEST_CHARGE_STATUS
        WATCH_SLEEP,
        HCPU_WAKEUP,
        WATCH_OPEN_DISPLAY_TO_APP_LIST,
        WATCH_GESTURE_UNLOCK,
        WATCH_REBOOT,
        STANDBY_WAKEUP,
        WATCH_REQUEST_BATTERY,
        WATCH_REQUEST_CHARGE_STATUS,
/// Sensors

// Sensor Types
#define SENSOR_INTERACT_TYPE WATCH_SENSOR_SUBSCRIBE

        WATCH_SENSOR_SUBSCRIBE,

        INTERACT_END,

    } INTERACT_Type; // include interact priority

    extern rt_sem_t go_to_sleep_sem;
    extern char qrcode_data[256];

    extern void *watch_system_interact(INTERACT_Type type, void *pValue);
    extern void ble_app_advertising_start(bool mouse_mode, bool pairing_mode);
    extern void switch_watch_motion_control_mode(bool enable, bool animation);
    extern bool get_idle_state(void);
    extern void set_idle_state(bool state);
    extern bool is_user_touching_screen(void);
#ifdef BSP_USING_WATCH_SYS_CLIENT
    extern void set_watch_sleep_state(const watch_sys_sleep_state_t *state);
#endif

    /// Motor control for watch system interact
    extern bool get_motor_switch_state(void);
    extern void set_motor_switch_state(uint8_t state);
    extern void motor_pattern_scrolling_app(void);
    extern void motor_pattern_wheel_scrolling(void);
    extern void motor_pattern_screen_on_longpress(void);
    extern void motor_pattern_timer_reminder(void);
    extern void motor_pattern_normal(void);
    extern void motor_pattern_unlocked(void);
    extern void motor_pattern_touchpad_slide(void);
    extern void motor_pattern_damping(void);
    extern void motor_pattern_stop(void);
    extern void motor_pattern_tap(void);


#ifdef __cplusplus
}
#endif

#endif //__WATCH_SYSTEM_INTERACT_H__
       /************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/