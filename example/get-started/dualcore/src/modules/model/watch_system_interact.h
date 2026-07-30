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
        V2T_INTENT_SKAIBAR = 0x03,
        V2T_INTENT_MIC_INPUTE = 0x04,
        V2T_INTENT_MEMO = 0x05,     /* SkaiApp memo voice fill (ADR-0037): the phone
                                       routes the transcript to setMemoText for the
                                       memo named in the preceding KEY_SKAIAPP_VOICE. */
    } V2T_INTENT;

    extern rt_sem_t go_to_sleep_sem;
    extern char qrcode_data[256];

    extern void ble_app_advertising_start(bool mouse_mode, bool pairing_mode);
    extern void switch_watch_motion_control_mode(bool enable, bool animation);
    extern bool get_idle_state(void);
    extern bool is_user_touching_screen(void);
    extern void watch_system_wakeup(void);
    extern void watch_system_sleep(void);

    /// Drive the BLE link profile from screen state: screen on -> MEDIUM
    /// (responsive), screen off -> SLOW (power). Call from the GUI PM
    /// resume/suspend handler so every wake path is covered.
    extern void watch_system_ble_on_screen(bool screen_on);

    /// Sensor subscription dispatch (formerly WATCH_SENSOR_SUBSCRIBE).
    extern void interact_sensor_subscription(sensor_subscription_t sub);

    /// Non-trivial app/control/settings/power interactions. Functions that
    /// would be one-line wrappers around a provider call have been inlined at
    /// their call sites — call `setting_provider.set_*`, `control_provider.*`,
    /// `peripheral_provider.*`, `watch_sys_sync.*`, `handle_gesture_unlock()`,
    /// or `subscribe_alarm_client()` directly for those.
    extern void interact_bat_low_level(bool enable);
    extern void interact_show_qrcode(const char *qrcode);
    extern void interact_find_watch(void);
    extern void interact_timer_reminder(void);
    extern void interact_mic_listen(bool enable);
    extern void interact_mic_v2t_input(void);
    extern void interact_memo_v2t_input(void);  /* SkaiApp memo 🎤 voice fill (ADR-0037) */
    extern void interact_voice_recognition(VOICE_RECOGNITION_PAYLOAD *msgData);
    extern void interact_chat_result(MSG_DATA_PAYLOAD *msgData);
    extern void interact_cancel_bond(void);
    extern void interact_bonded(const uint8_t *user_id);
#ifdef BSP_USING_BLOC_CONTROL
    extern void interact_sync_media_status(uint8_t status);
#endif
#ifdef BSP_USING_BLOC_SETTING
    /* Bundles set_language with load_instruction_list (translation reload). */
    extern void interact_language_set(const char *language);
#endif
    extern void handle_gesture_unlock(void);

#ifdef BSP_USING_WATCH_SYS_CLIENT
    extern void set_watch_sleep_state(const watch_sys_sleep_state_t *state);
#endif

    /// Motor control for watch system interact
    extern bool get_motor_switch_state(void);
    extern void set_motor_switch_state(uint8_t state);
    extern void motor_pattern_alarm(void);
    extern void motor_pattern_calling(void);
    extern void motor_pattern_notification(void);
    extern void motor_pattern_scrolling_app(void);
    extern void motor_pattern_screen_on_longpress(void);
    extern void motor_pattern_timer_reminder(void);
    extern void motor_pattern_unlocked(void);
    extern void motor_pattern_touchpad_slide(void);
    extern void motor_pattern_damping(void);
    extern void motor_pattern_stop(void);
    extern void motor_pattern_tap(void);
    extern void motor_pattern_air_hint(void); /* 小震 50% 力道:飛鼠觸發提示(founder 2026-07-24) */


#ifdef __cplusplus
}
#endif

#endif //__WATCH_SYSTEM_INTERACT_H__
       /************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/