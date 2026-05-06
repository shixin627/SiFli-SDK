/*********************************************************************************************************
 *               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_parse_setting.c
 * @brief    Resolves SET_CONFIG_COMMAND_ID payloads (time/alarm/profile/dial
 *           and friends) sent from the phone. Each case validates length,
 *           updates SkaiWatchSys + provider state, and may echo back via
 *           commu_send_*.
 *********************************************************************************************************
 */
#include <rtthread.h>
#include "communicate_parse.h"
#include "communicate_protocol.h"
#include "communicate_task.h"

#include "string.h"
#include "watch_system_interact.h"
#ifdef BSP_USING_MODEL_WATCH_GLOBAL_DATA
    #include "watch_global_data.h"
#endif
#ifdef BSP_USING_BLOC
    #include "bloc_setting.h"
    #include "bloc_control.h"
    #include "bloc_peripheral.h"
#endif
#ifdef BSP_USING_ALARM_CLIENT
    #include "alarm_client.h"
#endif

#define DBG_TAG "commu.parse.setting"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

void resolve_settings_config_command(uint8_t key, const uint8_t *pValue,
                                     uint16_t length)
{
    switch (key)
    {
    case KEY_PHONE_OS_VERSION:
    {
        if (length == 0x02)
        {
            LOG_D(
                "Phone OS Version:%d; Pair Flag = %d", pValue[0],
                SkaiWatchSys.paired_info.paired_flag); // 0x01:ios,0x02:android

            SkaiWatchSys.phone_os_version = pValue[0];

            if (IOS == SkaiWatchSys.phone_os_version &&
                !SkaiWatchSys.paired_info.paired_flag)
            {
                // TODO: le_bond_pair(SkaiWatchSys.watch_conn_id);
            }
        }
    }
    break;
    case KEY_TIME_SETTINGS:
    {
        if (length == 4)
        {
            time_union_t time;
            time.data = read_be32(pValue);
            // if set time pass a day,reset step count
            LOG_D("Set DateTime: %d-%d-%d %d:%d:%d", time.time.year,
                  time.time.month, time.time.day, time.time.hours,
                  time.time.minute, time.time.seconds);
            uint32_t old_sec = SkaiWatchSys.SecondCountRTC;
            SkaiWatchSys.Global_Time.year = 2000 + time.time.year;
            SkaiWatchSys.Global_Time.month = time.time.month;
            SkaiWatchSys.Global_Time.day = time.time.day;
            SkaiWatchSys.Global_Time.hour = time.time.hours;
            SkaiWatchSys.Global_Time.minutes = time.time.minute;
            SkaiWatchSys.Global_Time.seconds = time.time.seconds;

#ifdef BSP_USING_BLOC_SETTING
            setting_provider.set_watch_time(SkaiWatchSys.Global_Time);
#endif
            extern void app_clock_reset_time(void);
            app_clock_reset_time();
        }
    }
    break;
    case KEY_ALARM_SETTINGS:
    {
        if (length % 5 == 0)
        {
            uint8_t num = length / 5;
            if (num > MAX_ALARM_NUM)
            {
                num = MAX_ALARM_NUM;
            }
            SkaiWatchSys.alarm_num = num;
            for (uint8_t i = 0; i < num; i++)
            {
                /* 5-byte BE pack of the 40-bit T_ALARM bit-field, mirror of
                   write_be40 in communicate_task.c::commu_send_alarm_settings.
                   Phone packs the enabled flag into reserved[0]; preserve it
                   bit-for-bit so apply_alarms_from_ble can read it. */
                const uint8_t *p = pValue + i * 5;
                T_ALARM alarm;
                alarm.data = ((uint64_t)p[0] << 32) | ((uint64_t)p[1] << 24) |
                             ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 8)  |
                              (uint64_t)p[4];

                memcpy(&SkaiWatchSys.alarms[i], &alarm, sizeof(T_ALARM));
                LOG_D("Set alarm day:%d, hour:%d, min:%d,repeat_flag:0x%x",
                      alarm.alarm.day, alarm.alarm.hour, alarm.alarm.minute,
                      alarm.alarm.day_repeat_flag);
            }
#ifdef BSP_USING_ALARM_CLIENT
            /* Push the freshly parsed list to alarm_manager_service so the
               HW alarm gets armed (and survives reboot via share_prefs). */
            apply_alarms_from_ble(SkaiWatchSys.alarms, num);
#endif
            /* Persist to share_prefs so SkaiWatchSys.alarms[] survives reboot.
               watch_config_struct_flash_read() loads them back on boot. */
            watch_prefs_save_alarms();
        }
    }
    break;
    case KEY_REQUEST_ALARM_SETTINGS: // request alarm clock list
    {
        if (length == 0)
        {
            LOG_D("Request alarm clock list");
            commu_send_alarm_settings();
        }
    }
    break;
    case KEY_STEP_TARGET_SETTINGS:
    {
        if (length == 4)
        {
            uint32_t target = read_be32(pValue);
            if (target == 0)
            {
                target = 10000;
            }
            SkaiWatchSys.gPedoData.daily_step_target = target;
            LOG_D("Set daily step target:%d",
                  SkaiWatchSys.gPedoData.daily_step_target);
#if FEATURE_USE_FLASH
            uint32_t temp = SkaiWatchSys.gPedoData.daily_step_target;
            ftl_save(&temp, STEP_TARGET_OFFSET, STEP_TARGET_SIZE);
#endif

            if (SkaiWatchSys.gPedoData.daily_step_target >=
                SkaiWatchSys.gPedoData.global_steps)
            {
                SkaiWatchSys.flag_field.daily_target_achieved = false;
            }
            // TODO: Set daily step target
        }
    }
    break;
    case KEY_SLEEP_TARGET_SETTINGS:
    {
        if (length == 2)
        {
            uint16_t target = read_be16(pValue);
            if (target == 0)
            {
                target = 480;
            }
            SkaiWatchSys.gPedoData.daily_sleep_target = target;
            LOG_D("Set daily sleep target:%d",
                  SkaiWatchSys.gPedoData.daily_sleep_target);
            // TODO: Set daily sleep target
#if FEATURE_USE_FLASH
            uint32_t temp = SkaiWatchSys.gPedoData.daily_sleep_target;
            ftl_save(&temp, SLEEP_TARGET_OFFSET, SLEEP_TARGET_SIZE);
#endif
        }
    }
    break;
    case KEY_PROFILE_SETTINGS:
    {
        if (length == 4)
        {
            userprofile_union_t profile;
            profile.data = read_be32(pValue);
            setting_provider.set_user_profile(profile);
        }
    }
    break;
    case KEY_INCOMMING_MESSAGE_SETTINGS:
    {
        if (length == 1)
        {
            uint8_t messageType = pValue[0];
            LOG_D("Set incomming messageType = %d", messageType);
            /* Each pair: (set_code, clear_code) toggles one notify-source bit. */
            #define MSG_TOGGLE(set_code, clear_code, field) \
                case set_code:   SkaiWatchSys.msg_switch.bit.field = true;  break; \
                case clear_code: SkaiWatchSys.msg_switch.bit.field = false; break;

            switch (messageType)
            {
                MSG_TOGGLE(0x01, 0x02, switch_call_msg)
                MSG_TOGGLE(0x03, 0x04, switch_qq_msg)
                MSG_TOGGLE(0x05, 0x06, switch_wechat_msg)
                MSG_TOGGLE(0x07, 0x08, switch_shortmessage_msg)
                MSG_TOGGLE(0x09, 0x0a, switch_line_msg)
                MSG_TOGGLE(0x0b, 0x0c, switch_twitter_msg)
            default:
                break;
            }
            #undef MSG_TOGGLE

            if (SkaiWatchSys.msg_switch.data != 0)
            {
                if (IOS == SkaiWatchSys.phone_os_version &&
                    !SkaiWatchSys.paired_info.paired_flag)
                {
                    // TODO: le_bond_pair(SkaiWatchSys.watch_conn_id);
                }
                else if (IOS == SkaiWatchSys.phone_os_version &&
                         SkaiWatchSys.paired_info.paired_flag)
                {
                    // TODO:
                    // ancs_set_data_source_notify(SkaiWatchSys.watch_conn_id,
                    // true);
                }
            }
            else
            {
                if (IOS == SkaiWatchSys.phone_os_version &&
                    SkaiWatchSys.paired_info.paired_flag)
                {
                    // TODO:
                    // ancs_set_data_source_notify(SkaiWatchSys.watch_conn_id,
                    // false);
                }
            }
        }
    }
    break;
    case KEY_DIAL_SETTING:
    {
        if (length == 0x01)
        {
            uint8_t index = pValue[0];
            if (index <= CLOCK_MAX_MENU)
            {
                if (SkaiWatchSys.clock_status != index)
                {
                    LOG_D("Set dial:%d", index);
                    setting_provider.set_watch_face((T_CLOCK_MENU_TYPE)index);
                }
            }
        }
    }
    break;
    case KEY_DIAL_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request dial");
            commu_send_dial_change();
        }
    }
    break;
    case KEY_HOUR_FORMAT_SETTING:
    {
        if (length == 0x01)
        {
            SkaiWatchSys.flag_field.hour_format = pValue[0];
            LOG_D("Set hour format:%d", SkaiWatchSys.flag_field.hour_format);
            // TODO: gui_update_SettingTimeFormat();
        }
    }
    break;
    case KEY_HOUR_FORMAT_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request hour format");
            commu_send_hour_format();
        }
    }
    break;
    case KEY_DISTANCE_UNIT_SETTING:
    {
        if (length == 0x01)
        {
            SkaiWatchSys.flag_field.distance_unit = pValue[0];
            LOG_D("Set distance unit:%d",
                  SkaiWatchSys.flag_field.distance_unit);
        }
    }
    break;
    case KEY_DISTANCE_UNIT_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request distance unit");
            commu_send_distance_unit();
        }
    }
    break;
    case KEY_OLED_DISPLAY_TIME_SETTING:
    {
        if (length == 0x01)
        {
            if (pValue[0] > 4 && pValue[0] <= 30)
            {
                SkaiWatchSys.oled_display_time = pValue[0];
                LOG_D("Set OLED display time:%d",
                      SkaiWatchSys.oled_display_time);
            }
        }
    }
    break;
    case KEY_OLED_DISPLAY_TIME_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request OLED display time");
            commu_send_oled_display_time(SkaiWatchSys.oled_display_time);
        }
    }
    break;
    case KEY_LANGUAGE_SETTING:
    {
        if (length == 1)
        {
            SkaiWatchSys.language = pValue[0];
            LOG_D("[Remote]Set language:%d", SkaiWatchSys.language);
#ifdef BSP_USING_BLOC_SETTING
            setting_provider.notify_language();
#endif
        }
    }
    break;
    case KEY_LANGUAGE_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request language");
            commu_send_language();
        }
    }
    break;
    case KEY_DEVICEINFO_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request device info");
            commu_send_device_info();
        }
    }
    break;
#if !kReleaseMode
    case KEY_GESTURE_ACCEL_LIMIT_SETTING:
    {
        if (length == 0x01)
        {
            uint8_t threshold = pValue[0];
            // bloc_setting_set_gesture_detect_threshold(threshold);
            watch_sys_sync.notify_debug_param_update(threshold);
        }
    }
    break;
#endif
    default:
        break;
    }
}
