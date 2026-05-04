/**
*****************************************************************************************
*     Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
*****************************************************************************************
* @file      communicate_task.c
* @brief     Routines to create App task and handle events & messages
* @author    shixin
* @date      2019-12-26
* @version   v1.0
**************************************************************************************
* @attention
* <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor
* Corporation</center></h2>
**************************************************************************************
*/

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <rtthread.h>
#include <string.h>
#include "communicate_protocol.h"
#include "communicate_parse.h"
#include "communicate_parse_notify.h"
#include "communicate_sync_pedo.h"
#include "communicate_sync_sleep.h"
#include "communicate_sync_heart_rate.h"
#include "communicate_update_image.h"
#include "watch_global_data.h"
#include "watch_system_interact.h"

#ifdef BSP_USING_BLOC
    #include "bloc_control.h"
    #include "bloc_v2t.h"
    #include "bloc_notification.h"
    #include "bloc_skaiwalk.h"
    #include "bloc_filesystem.h"
    #include "bloc_peripheral.h"
#endif

#ifndef BSP_USING_PC_SIMULATOR
    #include "audio_codec_i2s.h"
#endif

#define DBG_TAG "communicate.task"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/** @addtogroup  PERIPH_DEMO
 * @{
 */

/** @defgroup  PERIPH_APP_TASK Peripheral App Task
 * @brief This file handles the implementation of application task related
 * functions.
 *
 * Create App task and handle events & messages
 * @{
 */
/*============================================================================*
 *                              Packet Builder Macros
 *============================================================================*/
// Helper macros to build packet headers
#define BUILD_PACKET_HEADER(buf, cmd_id, key)                                  \
    do                                                                         \
    {                                                                          \
        (buf)[0] = (cmd_id);                                                   \
        (buf)[1] = L2_HEADER_VERSION;                                          \
        (buf)[2] = (key);                                                      \
    } while (0)

#define SET_PACKET_LENGTH(buf, len)                                            \
    do                                                                         \
    {                                                                          \
        (buf)[3] = ((len) >> 8) & 0xFF;                                        \
        (buf)[4] = (len) & 0xFF;                                               \
    } while (0)

#define BUILD_SIMPLE_PACKET(buf, cmd_id, key, len)                             \
    do                                                                         \
    {                                                                          \
        BUILD_PACKET_HEADER(buf, cmd_id, key);                                 \
        SET_PACKET_LENGTH(buf, len);                                           \
    } while (0)

// Maximum payload buffer size
#define MAX_PACKET_PAYLOAD_SIZE 507

// Status codes
#define BOND_SUCCESS 0x00
#define BOND_FAIL 0x01
#define LOGIN_SUCCESS 0x00
#define LOGIN_FAIL 0x01

/*============================================================================*
 *                              Helpers
 *============================================================================*/

// Helper: Build simple status packet (1 byte payload)
static inline uint16_t build_status_packet(uint8_t *buf, uint8_t cmd_id,
                                           uint8_t key, uint8_t status)
{
    BUILD_SIMPLE_PACKET(buf, cmd_id, key, 1);
    buf[5] = status;
    return 6;
}

// Helper: connection state check shared by all direct send APIs
static inline bool commu_can_send(void)
{
    return SkaiWatchSys.connected_to_phone && !is_ble_dfu_thread_running();
}

// Helper: connection state check for OTA-related sends (skip DFU guard)
static inline bool commu_can_send_ota(void)
{
    return SkaiWatchSys.gap_conn_state == GAP_CONN_STATE_CONNECTED &&
           SkaiWatchSys.connected_to_phone;
}

/*============================================================================*
 *                              Direct Send API
 *  Public functions that build the packet and send via BLE notify directly,
 *  replacing the legacy event-dispatch pattern.
 *============================================================================*/

bool commu_send_bond_success(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, BOND_COMMAND_ID,
                                       KEY_BOND_RESPOSE, BOND_SUCCESS);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_bond_fail(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, BOND_COMMAND_ID,
                                       KEY_BOND_RESPOSE, BOND_FAIL);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_login_success(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, BOND_COMMAND_ID,
                                       KEY_LOGIN_RESPONSE, LOGIN_SUCCESS);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_login_fail(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, BOND_COMMAND_ID,
                                       KEY_LOGIN_RESPONSE, LOGIN_FAIL);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_alarm_settings(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[5 + 5 * 16];
    BUILD_PACKET_HEADER(buf, SET_CONFIG_COMMAND_ID, KEY_RETURN_ALARM_SETTINGS);

    uint8_t alarm_item_count = 0;
    uint8_t alarm_num = SkaiWatchSys.alarm_num;
    for (uint8_t index = 0; index < alarm_num; index++)
    {
        if (SkaiWatchSys.alarms[index].alarm.day_repeat_flag == 0 &&
            SkaiWatchSys.alarms[index].alarm.reserved == 0)
        {
            SkaiWatchSys.alarm_num--;
            LOG_I("delete alarm once item...\n");
            continue;
        }
        buf[5 + alarm_item_count * 5] = SkaiWatchSys.alarms[index].data >> 32;
        buf[6 + alarm_item_count * 5] = SkaiWatchSys.alarms[index].data >> 24;
        buf[7 + alarm_item_count * 5] = SkaiWatchSys.alarms[index].data >> 16;
        buf[8 + alarm_item_count * 5] = SkaiWatchSys.alarms[index].data >> 8;
        buf[9 + alarm_item_count * 5] = SkaiWatchSys.alarms[index].data;
        alarm_item_count++;
    }

    SET_PACKET_LENGTH(buf, alarm_item_count * 5);
    return skaiwatch_ble_notify(buf, 5 + alarm_item_count * 5);
}

bool commu_send_lift_switch(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, SET_CONFIG_COMMAND_ID,
                                       KEY_LIFT_SWITCH_RETURN,
                                       SkaiWatchSys.flag_field.lift_switch_status);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_twist_switch(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, SET_CONFIG_COMMAND_ID,
                                       KEY_TWIST_SWITCH_RETURN,
                                       SkaiWatchSys.flag_field.twist_switch_status);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_incoming_message_settings(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, SET_CONFIG_COMMAND_ID,
                                       KEY_INCOMMING_MESSAGE_SETTINGS_RETURN,
                                       SkaiWatchSys.msg_switch.data);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_hour_format(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, SET_CONFIG_COMMAND_ID,
                                       KEY_HOUR_FORMAT_RETURN,
                                       SkaiWatchSys.flag_field.hour_format);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_distance_unit(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, SET_CONFIG_COMMAND_ID,
                                       KEY_DISTANCE_UNIT_RETURN,
                                       SkaiWatchSys.flag_field.distance_unit);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_dndm_setting(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[8];
    BUILD_SIMPLE_PACKET(buf, SET_CONFIG_COMMAND_ID, KEY_DNDM_RETURN, 3);
    buf[5] = SkaiWatchSys.DNDMode.data >> 16;
    buf[6] = SkaiWatchSys.DNDMode.data >> 8;
    buf[7] = SkaiWatchSys.DNDMode.data;
    return skaiwatch_ble_notify(buf, 8);
}

bool commu_send_oled_display_time(uint8_t time)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, SET_CONFIG_COMMAND_ID,
                                       KEY_OLED_DISPLAY_TIME_RETURN, time);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_language(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, SET_CONFIG_COMMAND_ID,
                                       KEY_LANGUAGE_RETURN,
                                       SkaiWatchSys.language);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_dial_change(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, SET_CONFIG_COMMAND_ID,
                                       KEY_DIAL_RETURN,
                                       SkaiWatchSys.clock_status);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_backlight(uint8_t brightness)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, SET_CONFIG_COMMAND_ID,
                                       KEY_BACKLIGHT_RETURN, brightness);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_sport_data(void)
{
    if (!commu_can_send()) return false;
    uint16_t length = sizeof(watch_sys_heath_info_t);
    uint8_t buf[5 + sizeof(watch_sys_heath_info_t)];
    BUILD_PACKET_HEADER(buf, HEALTH_DATA_COMMAND_ID, KEY_RETURN_SPORTS_DATA);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, &SkaiWatchSys.health_info_today, length);
    return skaiwatch_ble_notify(buf, 5 + length);
}

bool commu_send_heart_data(int hr)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, HEALTH_DATA_COMMAND_ID,
                                       KEY_HEART_DATA_RETURN, (uint8_t)hr);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_heart_setting(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[7];
    BUILD_SIMPLE_PACKET(buf, HEALTH_DATA_COMMAND_ID,
                        KEY_RETURN_HEART_SAMPLE_SETTING, 2);
    buf[5] = SkaiWatchSys.hrs_detect_period ? 0x01 : 0x00;
    buf[6] = SkaiWatchSys.hrs_detect_period;
    return skaiwatch_ble_notify(buf, 7);
}

bool commu_send_heart_rate_series(const float *ppg, uint16_t count)
{
    if (!commu_can_send()) return false;
    uint16_t bytes = count * sizeof(float);
    uint8_t buf[5 + 256];
    if (bytes + 5 > sizeof(buf)) return false;
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_HEART_RATE_SENSOR_SAMPLE);
    SET_PACKET_LENGTH(buf, bytes);
    memcpy(buf + 5, ppg, bytes);
    return skaiwatch_ble_notify(buf, 5 + bytes);
}

bool commu_send_phone_control_cmd(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[5];
    BUILD_SIMPLE_PACKET(buf, CONTROL_COMMAND_ID, KEY_TAKE_PHOTO, 0);
    return skaiwatch_ble_notify(buf, 5);
}

bool commu_send_find_mobile(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[5];
    BUILD_SIMPLE_PACKET(buf, CONTROL_COMMAND_ID, KEY_FIND_PHONE, 0);
    return skaiwatch_ble_notify(buf, 5);
}

bool commu_send_media_control(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, CONTROL_COMMAND_ID,
                                       KEY_PHONE_MEDIA_CONTROL,
                                       app_audio_get_control_command());
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_mqtt_control(uint8_t status)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, CONTROL_COMMAND_ID,
                                       KEY_MQTT_CONTROL, status);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_volume_percentage(uint8_t volume)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, CONTROL_COMMAND_ID,
                                       KEY_RETURN_VOLUMN, volume);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_voice_record_intent(uint32_t millisecondsFromEpoch)
{
    if (!commu_can_send()) return false;
    uint8_t buf[10];
    BUILD_SIMPLE_PACKET(buf, CONTROL_COMMAND_ID, KEY_RETURN_VOICE_RECORD_INTENT,
                        5);
    buf[5] = app_voice_get_recording_intent();
    buf[6] = millisecondsFromEpoch >> 24;
    buf[7] = millisecondsFromEpoch >> 16;
    buf[8] = millisecondsFromEpoch >> 8;
    buf[9] = millisecondsFromEpoch;
    return skaiwatch_ble_notify(buf, 10);
}

bool commu_send_virtual_gesture(uint8_t label)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, CONTROL_COMMAND_ID,
                                       KEY_VIRTUAL_GESTURE, label);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_finger_tap(uint8_t label)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, CONTROL_COMMAND_ID,
                                       KEY_TP_GESTURE, label);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_charge_status(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, NOTIFY_COMMAND_ID,
                                       KEY_BATTERY_CHARGE_STATUS,
                                       SkaiWatchSys.charger_status);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_weather_request(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[5];
    BUILD_SIMPLE_PACKET(buf, NOTIFY_COMMAND_ID, KEY_REQUEST_WEATHER, 0);
    return skaiwatch_ble_notify(buf, 5);
}

bool commu_send_calendar_request(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[5];
    BUILD_SIMPLE_PACKET(buf, NOTIFY_COMMAND_ID, KEY_REQUEST_CALENDAR, 0);
    return skaiwatch_ble_notify(buf, 5);
}

bool commu_send_gesture_detect(uint8_t label)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, NOTIFY_COMMAND_ID,
                                       KEY_GESTURE_DETECT, label);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_remote_input(const char *json)
{
    if (!commu_can_send() || !json) return false;
    uint16_t length = strlen(json);
    uint8_t buf[5 + MAX_PACKET_PAYLOAD_SIZE];
    if (length + 5 > sizeof(buf)) return false;
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_REMOTE_INPUT);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, json, length);
    return skaiwatch_ble_notify(buf, 5 + length);
}

bool commu_send_dismiss_notification(const char *id)
{
    if (!commu_can_send() || !id) return false;
    uint16_t length = strlen(id);
    uint8_t buf[5 + MAX_PACKET_PAYLOAD_SIZE];
    if (length + 5 > sizeof(buf)) return false;
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_DISMISS_NOTIFICATION);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, id, length);
    return skaiwatch_ble_notify(buf, 5 + length);
}

bool commu_send_create_note(const char *json)
{
    if (!commu_can_send() || !json) return false;
    uint16_t length = strlen(json);
    uint8_t buf[5 + MAX_PACKET_PAYLOAD_SIZE];
    if (length + 5 > sizeof(buf)) return false;
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_CREATE_NOTE);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, json, length);
    return skaiwatch_ble_notify(buf, 5 + length);
}

bool commu_send_user_speaking_state(uint8_t status)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, NOTIFY_COMMAND_ID,
                                       KEY_USER_SPEAKING_STATE, status);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_chat_with_ai(const char *json)
{
    if (!commu_can_send() || !json) return false;
    uint16_t length = strlen(json);
    uint8_t buf[5 + MAX_PACKET_PAYLOAD_SIZE];
    if (length + 5 > sizeof(buf)) return false;
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_RETURN_CHAT_INTENT);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, json, length);
    return skaiwatch_ble_notify(buf, 5 + length);
}

bool commu_send_quaternion_data(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[21];
    BUILD_SIMPLE_PACKET(buf, NOTIFY_COMMAND_ID, KEY_QUATERNION_DATA, 16);
    memcpy(buf + 5, &quaternion_buffer, 16);
    return skaiwatch_ble_notify(buf, 21);
}

bool commu_send_battery_voltage(uint16_t voltage)
{
    if (!commu_can_send()) return false;
    uint8_t buf[7];
    BUILD_SIMPLE_PACKET(buf, NOTIFY_COMMAND_ID, KEY_BATTERY_VOLTAGE, 2);
    buf[5] = voltage >> 8;
    buf[6] = voltage & 0xFF;
    return skaiwatch_ble_notify(buf, 7);
}

bool commu_send_battery_level(uint8_t level)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, NOTIFY_COMMAND_ID,
                                       KEY_BATTERY_LEVEL, level);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_holding_displacement(uint8_t event, int x, int y)
{
    if (!commu_can_send()) return false;
    uint8_t buf[14];
    BUILD_SIMPLE_PACKET(buf, NOTIFY_COMMAND_ID, KEY_HOLDING_DISPLACEMENT, 9);
    buf[5] = event;
    memcpy(buf + 6, &x, sizeof(x));
    memcpy(buf + 10, &y, sizeof(y));
    return skaiwatch_ble_notify(buf, 14);
}

bool commu_send_update_instruction(const char *json)
{
    if (!commu_can_send() || !json) return false;
    uint16_t length = strlen(json);
    uint8_t buf[5 + MAX_PACKET_PAYLOAD_SIZE];
    if (length + 5 > sizeof(buf)) return false;
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_SKAI_CREATION_INSTRUCTIONS);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, json, length);
    return skaiwatch_ble_notify(buf, 5 + length);
}

bool commu_send_get_instruction_img(const char *id)
{
    if (!commu_can_send() || !id) return false;
    uint16_t length = strlen(id);
    uint8_t buf[5 + MAX_PACKET_PAYLOAD_SIZE];
    if (length + 5 > sizeof(buf)) return false;
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_SKAI_INSTRUCTION_IMAGE);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, id, length);
    return skaiwatch_ble_notify(buf, 5 + length);
}

bool commu_send_linear_acce_buffer(const uint8_t *acce, uint16_t length)
{
    if (!commu_can_send() || !acce) return false;
    LOG_D("commu_send_linear_acce_buffer length=%d", length);

    if (length > 484)
    {
        /* Segmented transmission, each segment up to 484 bytes */
        uint8_t buf[6 + 484];
        uint16_t offset = 0;
        uint8_t segment_index = 1;
        while (offset < length)
        {
            uint16_t chunk_size = (length - offset > 484) ? 484
                                                          : (length - offset);
            BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_GSENSOR_SAMPLE);
            buf[3] = (chunk_size + 1) >> 8;
            buf[4] = (chunk_size + 1) & 0xFF;
            buf[5] = segment_index;
            memcpy(buf + 6, acce + offset, chunk_size);
            if (!skaiwatch_ble_notify(buf, chunk_size + 6))
            {
                return false;
            }
            offset += chunk_size;
            segment_index++;
        }
        return true;
    }

    uint8_t buf[6 + 484];
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_GSENSOR_SAMPLE);
    buf[3] = (length + 1) >> 8;
    buf[4] = (length + 1) & 0xFF;
    buf[5] = 0; /* no segmentation */
    memcpy(buf + 6, acce, length);
    return skaiwatch_ble_notify(buf, length + 6);
}

bool commu_send_imu_buffer(const uint8_t *imu, uint16_t length)
{
    if (!commu_can_send() || !imu) return false;
    uint8_t buf[5 + 512];
    if (length + 5 > sizeof(buf)) return false;
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_IMU_BUFFER);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, imu, length);
    return skaiwatch_ble_notify(buf, length + 5);
}

bool commu_send_baro_buffer(float pressure)
{
    if (!commu_can_send()) return false;
    uint8_t buf[9];
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_BARO_BUFFER);
    SET_PACKET_LENGTH(buf, 4);
    memcpy(buf + 5, &pressure, 4);
    return skaiwatch_ble_notify(buf, 9);
}

bool commu_send_gsensor_gravity_data(void)
{
    if (!commu_can_send()) return false;
    uint8_t len = sizeof(watch_sensor.motion_data.gravity);
    uint8_t buf[5 + sizeof(watch_sensor.motion_data.gravity)];
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_GSENSOR_GRAVITY_DATA);
    SET_PACKET_LENGTH(buf, len);
    memcpy(buf + 5, &watch_sensor.motion_data.gravity, len);
    return skaiwatch_ble_notify(buf, len + 5);
}

bool commu_send_start_sync_file(uint32_t total_size)
{
    if (!commu_can_send()) return false;
    char *file_path = get_sync_in_file_path();
    uint8_t path_len = strlen(file_path);
    uint8_t buf[9 + 256];
    if (9 + path_len > sizeof(buf)) return false;

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_START_SYNC_FILE);
    /* Protocol: [total_size(4)][file_path] */
    buf[3] = 0;
    buf[4] = path_len + 4;
    buf[5] = (total_size >> 24) & 0xFF;
    buf[6] = (total_size >> 16) & 0xFF;
    buf[7] = (total_size >> 8) & 0xFF;
    buf[8] = total_size & 0xFF;
    memcpy(buf + 9, file_path, path_len);
    return skaiwatch_ble_notify(buf, 9 + path_len);
}

bool commu_send_sync_file(const uint8_t *chunk, uint16_t length)
{
    if (!commu_can_send() || !chunk) return false;
    uint8_t buf[5 + MAX_PACKET_PAYLOAD_SIZE];
    if (length + 5 > sizeof(buf)) return false;
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_SYNC_FILE);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, chunk, length);
    return skaiwatch_ble_notify(buf, length + 5);
}

bool commu_send_end_sync_file(void)
{
    if (!commu_can_send()) return false;
    uint8_t buf[5];
    BUILD_SIMPLE_PACKET(buf, NOTIFY_COMMAND_ID, KEY_END_SYNC_FILE, 0);
    return skaiwatch_ble_notify(buf, 5);
}

bool commu_send_file_compare_result(uint8_t result)
{
    if (!commu_can_send()) return false;
    uint8_t buf[6];
    BUILD_SIMPLE_PACKET(buf, NOTIFY_COMMAND_ID, KEY_FILE_COMPARE_RESULT, 1);
    buf[5] = result;
    return skaiwatch_ble_notify(buf, 6);
}

bool commu_send_bluetooth_log(const char *log)
{
    if (!commu_can_send() || !log) return false;
    uint16_t buffer_len = strlen(log);
    uint8_t buf[5 + MAX_PACKET_PAYLOAD_SIZE];
    if (buffer_len + 5 > sizeof(buf)) return false;
    BUILD_PACKET_HEADER(buf, BLUETOOTH_LOG_COMMAND_ID, KEY_DEBUG);
    SET_PACKET_LENGTH(buf, buffer_len);
    memcpy(buf + 5, log, buffer_len);
    return skaiwatch_ble_notify(buf, 5 + buffer_len);
}

bool commu_send_watch_system_sync(void)
{
    if (!commu_can_send()) return false;
    size_t total_size = sizeof(SkaiWatchSys);
    LOG_D("commu_send_watch_system_sync total_size = %d", total_size);
    uint8_t buf[5 + sizeof(SkaiWatchSys)];
    if (total_size + 5 > sizeof(buf)) return false;
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_WATCH_SYS_RETURN);
    SET_PACKET_LENGTH(buf, total_size);
    memcpy(buf + 5, (uint8_t *)&SkaiWatchSys, total_size);
    return skaiwatch_ble_notify(buf, total_size + 5);
}

bool commu_send_ota_status(uint8_t status)
{
    if (!commu_can_send_ota()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, NOTIFY_COMMAND_ID,
                                       KEY_OTA_STATUS, status);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_utest_state(uint8_t state)
{
    if (!commu_can_send_ota()) return false;
    uint8_t buf[6];
    uint16_t len = build_status_packet(buf, SKAI_LINK_COMMAND_ID,
                                       KEY_UNIT_TEST_SYNC, state);
    return skaiwatch_ble_notify(buf, len);
}

bool commu_send_minute_activity(void)
{
    if (!commu_can_send()) return false;
    uint8_t len = sizeof(SkaiWatchSys.activity);
    uint8_t buf[5 + sizeof(SkaiWatchSys.activity)];
    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_MINUTE_ACTIVITY);
    buf[3] = 0;
    buf[4] = len;
    memcpy(buf + 5, &SkaiWatchSys.activity, len);
    return skaiwatch_ble_notify(buf, 5 + len);
}

bool commu_send_device_info(void)
{
    if (!commu_can_send()) return false;
    uint16_t deviceID = CUSTOMER_BOARD_VER;
    uint8_t buf[10];
    BUILD_SIMPLE_PACKET(buf, SET_CONFIG_COMMAND_ID, KEY_DEVICEINFO_RETURN, 5);
    buf[5] = deviceID >> 8;
    buf[6] = deviceID & 0xFF;
    buf[7] = VERSION_MAJOR;
    buf[8] = VERSION_MINOR;
    buf[9] = VERSION_REVISION;
    return skaiwatch_ble_notify(buf, 10);
}

/*============================================================================*
 *                              Audio Send Helper
 *============================================================================*/

extern int audio_profile_send_voice_data(uint8_t *voice_data,
                                         uint16_t voice_data_len);
bool skaiwatch_ble_audio_send(uint8_t *buf, uint16_t length)
{
    int res = audio_profile_send_voice_data(buf, length);
    return res > 0;
}

/**
 * @brief  Initialize communicate task module
 * @return 0 on success
 */
int communicate_task_init(void)
{
    return 0;
}
INIT_APP_EXPORT(communicate_task_init);