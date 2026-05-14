/**
 ******************************************************************************
 * @file   bloc_setting.c
 * @author Skaiwalk software development team
 * @brief  Setting management for Skaiwalk watch
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <rtthread.h>
#include "app_clock_main.h"
#include "bloc_setting.h"
#include "bloc_system_perception.h"
#include "communicate_protocol.h"
#include "communicate_task.h"
#include "lv_ext_resource_manager.h"
#include "lvsf/lvsf_font.h"
#include "watch_global_data.h"
#include "watch_system_interact.h"
#include "bloc_peripheral.h"
#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
#endif

/* Debug configuration -------------------------------------------------------*/
#define DBG_TAG "bloc.setting"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* Private defines -----------------------------------------------------------*/
#define THREAD_PRIORITY 25
#define THREAD_STACK_SIZE 1024
#define THREAD_TIMESLICE 5
#define MIN_BRIGHTNESS_PERCENT 10
#define MAX_BRIGHTNESS_PERCENT 100
#define MIN_SCREEN_TIME 5
#define MAX_SCREEN_TIME 60
#define INFINITE_SCREEN_TIME 255
#define PREFS_STORAGE_TIMEOUT 3000

/* Private variables ---------------------------------------------------------*/
SettingProvider setting_provider;
static rt_thread_t thread_shared_prefs = RT_NULL;
static watch_prefs_key key_to_store = 0;
static rt_timer_t shared_prefs_storage_timer = RT_NULL;
static uint8_t _power_save_enable = 0;
static int _gesture_detect_threshold = 15;

/* Forward declarations -----------------------------------------------------*/
static void set_language(const char *language);
static void notify_language(void);
static void set_dnd_status(uint8_t status);
static void set_watch_face(T_CLOCK_MENU_TYPE watch_face);
static void set_hour_format(uint8_t format);
static void set_watch_time(T_UTC_TIME datetime);
static void set_brightness(uint8_t percent);
static void set_screen_time(uint8_t time);
static void set_lift_switch_status(bool status);
static uint8_t get_power_save_mode(void);
static void set_power_save_mode(uint8_t mode);
static void set_user_profile(userprofile_union_t profile);
static void shared_prefs_storage_timer_start(void);
static void shared_prefs_storage_timer_callback(void *param);
static void notify_watch_face_changed(uint8_t watch_face);
static void notify_brightness(void);
static void notify_screen_time(void);
static void notify_power_save_mode(void);
static void notify_lift_switch_status(void);
static void notify_user_profile(void);
static uint8_t convert_string_to_language_code(const char *str);

/* Private function implementations -----------------------------------------*/
/**
 * @brief  Timer callback for delayed preferences storage
 * @param  param: Timer parameter
 * @retval None
 */
static void shared_prefs_storage_timer_callback(void *param)
{
    peripheral_provider.save_watch_shared_prefs(key_to_store);
}

/**
 * @brief  Start or reset the preferences storage timer
 * @retval None
 */
static void shared_prefs_storage_timer_start(void)
{
    if (!shared_prefs_storage_timer)
    {
        shared_prefs_storage_timer = rt_timer_create(
            "shared_prefs_smooth_storage", shared_prefs_storage_timer_callback,
            RT_NULL, PREFS_STORAGE_TIMEOUT, RT_TIMER_FLAG_ONE_SHOT);
    }
    else
    {
        rt_timer_stop(shared_prefs_storage_timer);
    }
    rt_timer_start(shared_prefs_storage_timer);
}

/**
 * @brief  Set the key to be stored in shared preferences
 * @param  key: Preference key to store
 * @retval None
 */
static void set_key_to_store(watch_prefs_key key)
{
    key_to_store = key;
}

/* Watch Face & Time functions ----------------------------------------------*/

/**
 * @brief  Notify system about watch face change
 * @param  watch_face: New watch face value
 * @retval None
 */
static void notify_watch_face_changed(uint8_t watch_face)
{
    commu_send_dial_change();

#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_WATCHFACE;
    msg.data.watchface = (uint8_t)watch_face;
    lvgl_send_msg(msg);
#endif

    set_key_to_store(WATCH_PREFS_KEY_CLOCK_STATUS);
    shared_prefs_storage_timer_start();
}

/**
 * @brief  Set the watch face type
 * @param  watch_face: New watch face to set
 * @retval None
 */
static void set_watch_face(T_CLOCK_MENU_TYPE watch_face)
{
    if (SkaiWatchSys.clock_status == watch_face)
    {
        return;
    }

    SkaiWatchSys.clock_status = watch_face;
    notify_watch_face_changed(watch_face);
}

/**
 * @brief  Set hour format (12/24 hour)
 * @param  format: Hour format to set
 * @retval None
 */
static void set_hour_format(uint8_t format)
{
    SkaiWatchSys.flag_field.hour_format = format;

    commu_send_hour_format();

    peripheral_provider.save_watch_shared_prefs(WATCH_PREFS_KEY_FLAG_FIELD);
}

/**
 * @brief  Set watch date and time
 * @param  datetime: UTC time structure to set
 * @retval None
 */
static void set_watch_time(T_UTC_TIME datetime)
{
#ifndef BSP_USING_PC_SIMULATOR
    extern rt_err_t set_date(rt_uint32_t year, rt_uint32_t month,
                             rt_uint32_t day);
    extern rt_err_t set_time(rt_uint32_t hour, rt_uint32_t minute,
                             rt_uint32_t second);

    set_date(datetime.year, datetime.month, datetime.day);
    set_time(datetime.hour, datetime.minutes, datetime.seconds);

    peripheral_provider.save_watch_shared_prefs(WATCH_PREFS_KEY_GLOBAL_TIME);
#endif
}

/* Display functions --------------------------------------------------------*/

/**
 * @brief  Notify system about brightness change
 * @retval None
 */
static void notify_brightness(void)
{
    peripheral_provider.save_watch_shared_prefs(
        WATCH_PREFS_KEY_BACKLIGHT_PERCENT);
}

/**
 * @brief  Set display brightness
 * @param  percent: Brightness percentage (10-100)
 * @retval None
 */
void set_brightness(uint8_t percent)
{
    if (percent > MAX_BRIGHTNESS_PERCENT)
    {
        percent = MAX_BRIGHTNESS_PERCENT;
    }
    else if (percent < MIN_BRIGHTNESS_PERCENT)
    {
        percent = MIN_BRIGHTNESS_PERCENT;
    }

    if (percent != SkaiWatchSys.brightness)
    {
        SkaiWatchSys.brightness = percent;
        notify_brightness();
    }
}

/**
 * @brief  Notify system about screen timeout change
 * @retval None
 */
static void notify_screen_time(void)
{
    commu_send_oled_display_time(SkaiWatchSys.oled_display_time);

    peripheral_provider.save_watch_shared_prefs(
        WATCH_PREFS_KEY_OLED_DISPLAY_TIME);
}

/**
 * @brief  Set screen timeout
 * @param  time: Screen timeout in seconds (5-60, or 255 for infinite)
 * @retval None
 */
static void set_screen_time(uint8_t time)
{
    if (time > MAX_SCREEN_TIME)
    {
        time = INFINITE_SCREEN_TIME;
    }
    else if (time < MIN_SCREEN_TIME)
    {
        time = MIN_SCREEN_TIME;
    }

    if (time != SkaiWatchSys.oled_display_time)
    {
        SkaiWatchSys.oled_display_time = time;
        notify_screen_time();
    }
}

/* Power management functions ------------------------------------------------*/

/**
 * @brief  Get current power save mode
 * @retval Power save mode status (0: disabled, 1: enabled)
 */
static uint8_t get_power_save_mode(void)
{
    return _power_save_enable;
}

/**
 * @brief  Notify system about power save mode change
 * @retval None
 */
static void notify_power_save_mode(void)
{
    /* Power save mode notification mechanism not implemented yet */
}

/**
 * @brief  Set power save mode
 * @param  mode: Power save mode (0: disabled, 1: enabled)
 * @retval None
 */
static void set_power_save_mode(uint8_t mode)
{
    _power_save_enable = mode;
    notify_power_save_mode();
}

/**
 * @brief  Notify system about lift switch status change
 * @retval None
 */
static void notify_lift_switch_status(void)
{
    peripheral_provider.save_watch_shared_prefs(WATCH_PREFS_KEY_FLAG_FIELD);
}

/**
 * @brief  Set lift-to-wake feature status
 * @param  status: Lift switch status (true: enabled, false: disabled)
 * @retval None
 */
static void set_lift_switch_status(bool status)
{
    if (status)
    {
        SkaiWatchSys.flag_field.lift_switch_status = 1;
        LOG_D("Lift switch on");
        /* TODO: rtk_gsa_act_switch(GSA_ACT_LIFT, true); */
    }
    else
    {
        SkaiWatchSys.flag_field.lift_switch_status = 0;
        LOG_D("Lift switch off");
        /* TODO: rtk_gsa_act_switch(GSA_ACT_LIFT, false); */
    }
    notify_lift_switch_status();
}

/* Language functions --------------------------------------------------------*/

/**
 * @brief  Get current language setting as string code
 * @retval Language string code ("en_us" or "zh_cn")
 */
char *bloc_setting_get_language(void)
{
    if (SkaiWatchSys.language == LV_LOCALE_EN_US)
    {
        return "en_us";
    }
    else if (SkaiWatchSys.language == LV_LOCALE_ZH_CN)
    {
        return "zh_cn";
    }

    return "en_us"; /* Default to English */
}

/**
 * @brief  Convert string language code to numeric language code
 * @param  str: Language string code ("en_us" or "zh_cn")
 * @retval Numeric language code
 */
static uint8_t convert_string_to_language_code(const char *str)
{
    if (strcmp(str, "en_us") == 0)
    {
        return LV_LOCALE_EN_US;
    }
    else if (strcmp(str, "zh_cn") == 0)
    {
        return LV_LOCALE_ZH_CN;
    }

    return LV_LOCALE_INVALID;
}

/**
 * @brief  Notify system about language change
 * @retval None
 */
static void notify_language(void)
{
    commu_send_language();

    lv_ext_set_locale(NULL, bloc_setting_get_language());
    peripheral_provider.save_watch_shared_prefs(WATCH_PREFS_KEY_LANGUAGE);
}

/**
 * @brief  Set system language
 * @param  language: Language string code ("en_us" or "zh_cn")
 * @retval None
 */
static void set_language(const char *language)
{
    uint8_t code = convert_string_to_language_code(language);

    if (code == LV_LOCALE_INVALID)
    {
        LOG_E("[UI]Invalid language code: %s", language);
        return;
    }

    if (code != SkaiWatchSys.language)
    {
        SkaiWatchSys.language = code;
        notify_language();
        LOG_D("[UI]Language set to: %s", language);
    }
}

/* Reminder functions -------------------------------------------------------*/

/**
 * @brief  Set Do Not Disturb mode status
 * @param  status: DND mode status to set
 * @retval None
 */
static void set_dnd_status(uint8_t status)
{
    SkaiWatchSys.DNDMode.config.status = status;

    // peripheral_provider.save_watch_shared_prefs(WATCH_PREFS_KEY_DNDMODE);
}

/* User profile functions ---------------------------------------------------*/

/**
 * @brief  Notify system about user profile changes
 * @retval None
 */
static void notify_user_profile(void)
{
    peripheral_provider.save_watch_shared_prefs(WATCH_PREFS_KEY_USER_DATA);
    watch_sys_sync.notify_user_profile();
    LOG_D("[UI]User profile updated and synchronized");
}

/**
 * @brief  Set user profile information
 * @param  profile: User profile information structure
 * @retval None
 */
static void set_user_profile(userprofile_union_t profile)
{
    /* Only update user profile when setting different data */
    if (memcmp((void *)&SkaiWatchSys.user_data.user_profile.data,
               (void *)&profile.data, sizeof(userprofile_union_t)) != 0)
    {
        LOG_I("[UI]Update user profile[age:%d, height:%d(cm), weight:%d(kg)]",
              profile.bit_field.age, profile.bit_field.height,
              profile.bit_field.weight);

        SkaiWatchSys.user_data.user_profile.data = profile.data;
        notify_user_profile();
    }
}

/* Development settings functions -------------------------------------------*/

/**
 * @brief  Get gesture detection threshold value
 * @retval Current threshold value (default: 50)
 */
int bloc_setting_get_gesture_detect_threshold(void)
{
    return _gesture_detect_threshold;
}

/**
 * @brief  Set gesture detection threshold value
 * @param  threshold: New threshold value to set
 * @retval None
 */
void bloc_setting_set_gesture_detect_threshold(int threshold)
{
    _gesture_detect_threshold = threshold;
}

/* Initialization functions --------------------------------------------------*/

/**
 * @brief  Register setting provider function pointers
 * @retval 0 on success
 */
static int bloc_setting_provider_register(void)
{
    setting_provider.set_language = set_language;
    setting_provider.notify_language = notify_language;
    setting_provider.set_dnd_status = set_dnd_status;
    setting_provider.set_watch_face = set_watch_face;
    setting_provider.set_hour_format = set_hour_format;
    setting_provider.set_watch_time = set_watch_time;
    setting_provider.set_lift_switch_status = set_lift_switch_status;
    setting_provider.set_brightness = set_brightness;
    setting_provider.set_screen_time = set_screen_time;
    setting_provider.get_power_save_mode = get_power_save_mode;
    setting_provider.set_power_save_mode = set_power_save_mode;
    setting_provider.set_user_profile = set_user_profile;

    return 0;
}

/**
 * @brief  Load initial watch system settings
 * @retval None
 */
void bloc_setting_load_watch_system(void)
{
    lv_ext_set_locale(NULL, bloc_setting_get_language());
    LOG_I("bloc_setting_load_watch_system oled_display_time: %d, brightness: "
          "%d, font_size: %d",
          SkaiWatchSys.oled_display_time, SkaiWatchSys.brightness,
          SkaiWatchSys.font_size);
    if (SkaiWatchSys.oled_display_time == 0)
    {
        SkaiWatchSys.oled_display_time =
            10; /* Default to 10 seconds if unset */
    }
    if (SkaiWatchSys.brightness == 0)
    {
        SkaiWatchSys.brightness = 100; /* Default to 100% brightness if unset */
    }
    if (SkaiWatchSys.font_size == 0)
    {
        SkaiWatchSys.font_size = LVSF_FONT_TITLE;
    }
    SkaiWatchSys.flag_field.is_wearing = 1; /* Default wearing status */
}

/* Export the setting provider registration function */
INIT_APP_EXPORT(bloc_setting_provider_register);

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/