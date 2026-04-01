/*********************************************************************************************************
 *               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_parse.c
 * @brief
 * @details
 * @author
 * @date
 * @version  v0.1
 *********************************************************************************************************
 */
#include <rtthread.h>
#include "communicate_parse.h"
#include "communicate_protocol.h"

#include "string.h"
#include "communicate_sync_pedo.h"
#include "watch_system_interact.h"
#include "watch_system_core_task.h"
#ifdef BSP_USING_MODEL_WATCH_GLOBAL_DATA
    #include "watch_global_data.h"
#endif
#ifdef BSP_USING_BLOC
    #include "bloc_setting.h"
    #include "bloc_control.h"
    #include "bloc_peripheral.h"
#endif
#define DBG_TAG "commu.parse.setting"
#define DBG_LVL DBG_LOG
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
        if (length == 0x04)
        {
            time_union_t time;

            time.data = 0;
            time.data |= pValue[3];
            time.data |= pValue[2] << 8;
            time.data |= pValue[1] << 16;
            time.data |= pValue[0] << 24;
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
            uint8_t index;
            uint64_t alarmData;
            uint8_t num;
            T_ALARM alarm;

            num = length / 5;

            if (num > MAX_ALARM_NUM)
            {
                num = MAX_ALARM_NUM;
            }
            SkaiWatchSys.alarm_num = num;
            for (index = 0; index < num; index++)
            {
                alarmData = pValue[0 + index * 5];
                alarm.data = alarmData << 32;
                alarmData = pValue[1 + index * 5];
                alarm.data |= alarmData << 24;
                alarmData = pValue[2 + index * 5];
                alarm.data |= alarmData << 16;
                alarmData = pValue[3 + index * 5];
                alarm.data |= alarmData << 8;
                alarmData = pValue[4 + index * 5];
                alarm.data |= alarmData;
                if (alarm.alarm.day_repeat_flag == 0)
                {
                    alarm.alarm.reserved = 0x1;
                }

                memcpy((void *)&(SkaiWatchSys.alarms[index]), &alarm,
                       sizeof(T_ALARM));
#if 1
                LOG_D("Set alarm day:%d, hour:%d, min:%d,repeat_flag:0x%x\n",
                      alarm.alarm.day, alarm.alarm.hour, alarm.alarm.minute,
                      alarm.alarm.day_repeat_flag);

                // TODO: set_alarm(&alarm);
#endif
            }
#if FEATURE_USE_FLASH
            ftl_save((void *)SkaiWatchSys.alarms, ALARM_OFFSET, ALARM_SIZE);
            uint32_t temp = SkaiWatchSys.alarm_num;
            ftl_save(&temp, ALARM_NUM_OFFSET, ALARM_NUM_SIZE);
#endif
        }
    }
    break;
    case KEY_REQUEST_ALARM_SETTINGS: // request alarm clock list
    {
        if (length == 0)
        {
            LOG_D("Request alarm clock list");
            L1SendData data;
            data.event = L1SEND_RETURN_ALARM_EVENT;
            L1_send_event(data);
        }
    }
    break;
    case KEY_STEP_TARGET_SETTINGS:
    {
        if (length == 4)
        {
            uint32_t target = 0;
            target |= pValue[3];
            target |= pValue[2] << 8;
            target |= pValue[1] << 16;
            target |= pValue[0] << 24;

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
            uint16_t target = 0;
            target |= pValue[1];
            target |= pValue[0] << 8;

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
        if (length == 0x04)
        {
            userprofile_union_t profile;
            profile.data = 0;
            profile.data |= pValue[3];
            profile.data |= pValue[2] << 8;
            profile.data |= pValue[1] << 16;
            profile.data |= pValue[0] << 24;
            setting_provider.set_user_profile(profile);
        }
    }
    break;
    case KEY_LONG_TIME_SIT_ALERT:
    {
        if (length == 8)
        {
            T_SIT_ALERT sit_alert;
            memcpy((uint8_t *)&(sit_alert), pValue, sizeof(T_SIT_ALERT));
            setting_provider.set_sit_alert(sit_alert);
        }
    }
    break;
    case KEY_LIFT_SWITCH_SETTING:
    {
        if (length == 0x01)
        {
            setting_provider.set_lift_switch_status(pValue[0]);
        }
    }
    break;
    case KEY_LIFT_SWITCH_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request lift switch status");
            L1SendData data;
            data.event = L1SEND_RETURN_LIFT_SWITCH_EVENT;
            L1_send_event(data);
        }
    }
    break;
    case KEY_TWIST_SWITCH_SETTING:
    {
        if (length == 0x01)
        {
            if (pValue[0] == 0x01)
            {
                SkaiWatchSys.flag_field.twist_switch_status = true;
                LOG_D("twist switch on");
                // TODO: wrist gsa action twist enable
            }
            else
            {
                SkaiWatchSys.flag_field.twist_switch_status = false;
                LOG_D("twist switch off");
                // TODO: wrist gsa action twist disable
            }
        }
    }
    break;
    case KEY_TWIST_SWITCH_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request twist switch status");
            L1SendData data;
            data.event = L1SEND_RETURN_TWIST_SWITCH_EVENT;
            L1_send_event(data);
        }
    }
    break;
    case KEY_INCOMMING_MESSAGE_SETTINGS:
    {
        if (length == 1)
        {
            uint8_t messageType = pValue[0];
            LOG_D("Set incomming messageType = %d", messageType);
            switch (messageType)
            {
            case 0x01:
                SkaiWatchSys.msg_switch.bit.switch_call_msg = true;
                break;
            case 0x02:
                SkaiWatchSys.msg_switch.bit.switch_call_msg = false;
                break;
            case 0x03:
                SkaiWatchSys.msg_switch.bit.switch_qq_msg = true;
                break;
            case 0x04:
                SkaiWatchSys.msg_switch.bit.switch_qq_msg = false;
                break;
            case 0x05:
                SkaiWatchSys.msg_switch.bit.switch_wechat_msg = true;
                break;
            case 0x06:
                SkaiWatchSys.msg_switch.bit.switch_wechat_msg = false;
                break;
            case 0x07:
                SkaiWatchSys.msg_switch.bit.switch_shortmessage_msg = true;
                break;
            case 0x08:
                SkaiWatchSys.msg_switch.bit.switch_shortmessage_msg = false;
                break;
            case 0x09:
                SkaiWatchSys.msg_switch.bit.switch_line_msg = true;
                break;
            case 0x0a:
                SkaiWatchSys.msg_switch.bit.switch_line_msg = false;
                break;
            case 0x0b:
                SkaiWatchSys.msg_switch.bit.switch_twitter_msg = true;
                break;
            case 0x0c:
                SkaiWatchSys.msg_switch.bit.switch_twitter_msg = false;
                break;
            default:
                break;
            }

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
    case KEY_INCOMMING_MESSAGE_ALL_SETTINGS:
    {
        if (length == 4)
        {
            LOG_D("Set incomming message all settings");
            SkaiWatchSys.msg_switch.data = pValue[3];
            SkaiWatchSys.msg_switch.data |= pValue[2] << 8;
            SkaiWatchSys.msg_switch.data |= pValue[1] << 16;
            SkaiWatchSys.msg_switch.data |= pValue[0] << 24;

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
    case KEY_LONG_TIME_SIT_SETTING_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request long time sit setting");
            L1SendData data;
            data.event = L1SEND_SIT_SETTING_RETURN;
            L1_send_event(data);
        }
    }
    break;
    case KEY_INCOMMING_MESSAGE_SETTINGS_REQUEST:
    {

        if (length == 0x00)
        {
            if ((!SkaiWatchSys.paired_info.paired_flag) &&
                (SkaiWatchSys.phone_os_version == IOS) &&
                (SkaiWatchSys.msg_switch.data != 0))
            {
                // TODO: le_bond_pair(SkaiWatchSys.watch_conn_id);
            }
            LOG_D("request incomming message settings");
            L1SendData data;
            data.event = L1SEND_RETURN_INCOMMING_MESSAGE_SETTINGS;
            L1_send_event(data);
        }
    }
    break;
    case KEY_FUNCTIONS_REQUEST:
    {
        LOG_D(
            "<><><><><><><><><><> KEY_FUNCTIONS_REQUEST <><><><><><><><><><>");
        if (length == 0x00)
        {
            L1SendData data;
            data.event = L1SEND_RETURN_FUNCTIONS_EVENT;
            L1_send_event(data);
            data.event = L1SEND_RETURN_BBPRO_CONN_INFO;
            L1_send_event(data);
            data.event = L1SEND_RETURN_HIDDEN_FUNC;
            L1_send_event(data);
        }
    }
    break;
    case KEY_EXERCISEMODE_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request exercise mode");
            L1SendData data;
            data.event = L1SEND_RETURN_EXERCISEMODE_EVENT;
            L1_send_event(data);
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
            L1SendData data;
            data.event = L1SEND_RETURN_DIAL_CHANGE;
            L1_send_event(data);
        }
    }
    break;
    case KEY_HR_SAMPLE_REQUEST:
    {
        if (length == 0x00)
        {
            L1SendData data;
            data.event = L1SEND_RETURN_HR_SAMPLE_EVENT;
            L1_send_event(data);
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
            L1SendData data;
            data.event = L1SEND_RETURN_HOUR_FORMAT_SETTING;
            L1_send_event(data);
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
            L1SendData data;
            data.event = L1SEND_RETURN_DISTANCE_UNIT_SETTING;
            L1_send_event(data);
        }
    }
    break;
    case KEY_DNDM_SETTING:
    {
        if (length == 0x03)
        {
            bool temp_DNDM_start = SkaiWatchSys.DNDMode.config.DNDM_start;
            memset((void *)&SkaiWatchSys.DNDMode, 0x00, sizeof(T_DND_MODE));
            SkaiWatchSys.DNDMode.data |= pValue[0] << 16;
            SkaiWatchSys.DNDMode.data |= pValue[1] << 8;
            SkaiWatchSys.DNDMode.data |= pValue[2];
            SkaiWatchSys.DNDMode.config.DNDM_start = temp_DNDM_start;
            LOG_D("Set DNDM:%d, start:%d, end:%d",
                  SkaiWatchSys.DNDMode.config.status,
                  SkaiWatchSys.DNDMode.config.start_hour,
                  SkaiWatchSys.DNDMode.config.end_hour);
        }
    }
    break;
    case KEY_DNDM_REQUEST:
    {
        if (length == 0x00)
        {
            L1SendData data;
            data.event = L1SEND_RETURN_DNDM_SETTING;
            L1_send_event(data);
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
            L1SendData data;
            data.event = L1SEND_RETURN_OLED_DISPLAY_TIME;
            L1_send_event(data);
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
            L1SendData data;
            data.event = L1SEND_RETURN_LANGUAGE;
            L1_send_event(data);
        }
    }
    break;
    case KEY_DEVICEINFO_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request device info");
            L1SendData data;
            data.event = L1SEND_RETURN_DEVICE_INFO;
            L1_send_event(data);
        }
    }
    break;
    case KEY_BACKLIGHT_SETTING:
    {
        if (length == 0x01)
        {
            if (pValue[0] > 3 && pValue[0] < 101)
            {
                LOG_D("Set brightness from remote:%d", pValue[0]);
            }
        }
    }
    break;
    case KEY_BACKLIGHT_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request backlight");
            L1SendData data;
            data.event = L1SEND_RETURN_BACKLIGHT_EVENT;
            L1_send_event(data);
        }
    }
    break;
    case KEY_HIDDEN_FUNC_SETTING:
    {
        if (length == 0x04)
        {
            Hidden_FunC_t hidden_func;
            hidden_func.data = 0;
            hidden_func.data |= pValue[3];
            hidden_func.data |= pValue[2] << 8;
            hidden_func.data |= pValue[1] << 16;
            hidden_func.data |= pValue[0] << 24;

            SkaiWatchSys.flag_field.stopwatch_status =
                hidden_func.status.stopwatch_sw;
            SkaiWatchSys.flag_field.findphone_status =
                hidden_func.status.findPhone_sw;
            SkaiWatchSys.flag_field.system_lock_screen =
                hidden_func.status.lockScreen_sw;
            LOG_D(
                "Set hidden func:%d, stopwatch:%d, findphone:%d, lockscreen:%d",
                hidden_func.data, hidden_func.status.stopwatch_sw,
                hidden_func.status.findPhone_sw,
                hidden_func.status.lockScreen_sw);
        }
    }
    break;
    case KEY_HIDDEN_FUNC_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request hidden func");
            L1SendData data;
            data.event = L1SEND_RETURN_HIDDEN_FUNC;
            L1_send_event(data);
        }
    }
    break;
    case KEY_BBPRO_CONNECTED_STATE_REQUEST:
    {
        if (length == 0x00)
        {
            LOG_D("request BT audio connected state");
            L1SendData data;
            data.event = L1SEND_RETURN_BBPRO_CONNECTED_STATE;
            L1_send_event(data);
        }
    }
    break;
    case KEY_BBPRO_CREATE_CONNECTION_REQUEST:
    {
        if (length == 0x00)
        {
            /*iOS Initiate pairing, wirstband display dynamic picture*/
            SkaiWatchSys.flag_field.headset_pair_button = true;
            SkaiWatchSys.flag_field.headset_pair_handler = true;
            SkaiWatchSys.flag_field.headset_pair_state = false;
            LOG_D("request BT audio create connection");
        }
    }
    break;
    case KEY_MOTOR_STRENGTH_SETTING:
    {
        if (length == 0x01)
        {
            uint8_t motor_strength = pValue[0];
            if (motor_strength > 0 && motor_strength < 101)
            {
                LOG_D("Set motor strength:%d", motor_strength);
                // TODO: motor_set_strength(motor_strength);
            }
        }
    }
    break;
    case KEY_MOTOR_PERIOD_SETTING:
    {
        if (length == 0x01)
        {
            uint8_t motor_period = pValue[0];
            if (motor_period > 0 && motor_period < 101)
            {
                LOG_D("Set motor period:%d", motor_period);
                // TODO: motor_set_period(motor_period);
            }
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
    case KEY_ACCEL_SUBSCRIBE_REQUEST:
    {
        if (length == 0x01)
        {
            uint8_t value = pValue[0];
            sensor_subscription_t sensor_subscription;
            sensor_subscription.type = SENSOR_TYPE_ACCELEROMETER;
            sensor_subscription.thread_safe = true;
            if (value == 0x00)
            {
                sensor_subscription.status = false;
            }
            else if (value == 0x01)
            {
                sensor_subscription.status = true;
            }
            watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
        }
    }
    break;
    case KEY_GYRO_SUBSCRIBE_REQUEST:
    {
        if (length == 0x01)
        {
            uint8_t value = pValue[0];
            sensor_subscription_t sensor_subscription;
            sensor_subscription.type = SENSOR_TYPE_GYROSCOPE;
            sensor_subscription.thread_safe = true;
            if (value == 0x00)
            {
                sensor_subscription.status = false;
            }
            else if (value == 0x01)
            {
                sensor_subscription.status = true;
            }
            watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
        }
    }
    break;
    case KEY_MAG_SUBSCRIBE_REQUEST:
    {
        if (length == 0x01)
        {
            uint8_t value = pValue[0];
            sensor_subscription_t sensor_subscription;
            sensor_subscription.type = SENSOR_TYPE_MAGNETOMETER;
            sensor_subscription.thread_safe = true;
            if (value == 0x00)
            {
                sensor_subscription.status = false;
            }
            else if (value == 0x01)
            {
                sensor_subscription.status = true;
            }
            watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
        }
    }
    break;
    case KEY_HR_SUBSCRIBE_REQUEST:
    {
        if (length == 0x01)
        {
            uint8_t value = pValue[0];
            sensor_subscription_t sensor_subscription;
            sensor_subscription.type = SENSOR_TYPE_PPG;
            sensor_subscription.thread_safe = true;
            if (value == 0x00)
            {
                sensor_subscription.status = false;
            }
            else if (value == 0x01)
            {
                sensor_subscription.status = true;
            }
            watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
        }
    }
    break;
    case KEY_AUDIO_SUBSCRIBE_REQUEST:
    {
        if (length == 0x01)
        {
            uint8_t value = pValue[0];
            sensor_subscription_t sensor_subscription;
            sensor_subscription.type = SENSOR_TYPE_MIC;
            sensor_subscription.thread_safe = true;
            if (value == 0x00)
            {
                sensor_subscription.status = false;
            }
            else if (value == 0x01)
            {
                sensor_subscription.status = true;
            }
            watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
        }
        break;
    }
    default:
        break;
    }
}
