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
 *                              Macros
 *============================================================================*/
#define USING_L1_MESSAGE_QUEUE 0

#if USING_L1_MESSAGE_QUEUE
    #define L1SEND_TASK_PRIORITY 6
    #define L1SEND_TASK_STACK_SIZE 512 * 4
    #define L1SEND_TASK_TICK 20
    #define MAX_L1SEND_MSG_SIZE 0x10
#endif

// #define USING_BLE_RINGBUFFER_STATION

#ifdef USING_BLE_RINGBUFFER_STATION
    #define BLE_TRANSMIT_INTERVAL_DEFAULT 60
    #define BLE_NOTIFY_EVENT (1 << 0)
    #define MAX_BLE_RINGBUFFER_SIZE 16384

    #define BLE_STATION_STACK_SIZE 2048
    #define BLE_STATION_PRIORITY 21
    #define BLE_STATION_TICK 10

#endif

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

// Standard header size (cmd_id + version + key + length_h + length_l)
#define PACKET_HEADER_SIZE 5

// Maximum payload buffer size
#define MAX_PACKET_PAYLOAD_SIZE 507

// Status codes
#define BOND_SUCCESS 0x00
#define BOND_FAIL 0x01
#define LOGIN_SUCCESS 0x00
#define LOGIN_FAIL 0x01

/*============================================================================*
 *                              Packet Builder Infrastructure
 *============================================================================*/
// Packet builder context for zero-copy construction
typedef struct
{
    uint8_t *buf;      // Pointer to buffer (could be ringbuffer space)
    uint16_t capacity; // Available buffer capacity
    uint16_t length;   // Current packet length (including header)
    bool direct_send;  // true: send directly, false: via ringbuffer
} PacketBuilder;

// Handler function type for building packets
typedef uint16_t (*PacketHandler)(PacketBuilder *builder, L1SendData *data);

/*============================================================================*
 *                              Provider Pattern (類似 bloc 風格)
 *============================================================================*/
// Communication Handler Provider - 使用 Provider Pattern 替代查找表
typedef struct
{
    // Bond & Login handlers
    PacketHandler handle_bond_fail;
    PacketHandler handle_bond_success;
    PacketHandler handle_login_fail;
    PacketHandler handle_login_success;

    // Config & Settings handlers
    PacketHandler handle_return_alarm;
    PacketHandler handle_lift_switch;
    PacketHandler handle_twist_switch;
    PacketHandler handle_incoming_message_settings;
    PacketHandler handle_sit_setting;
    PacketHandler handle_hour_format;
    PacketHandler handle_distance_unit;
    PacketHandler handle_dndm_setting;
    PacketHandler handle_oled_display_time;
    PacketHandler handle_language;
    PacketHandler handle_device_info;
    PacketHandler handle_dial_change;
    PacketHandler handle_backlight;

    // Health Data handlers
    PacketHandler handle_data_sync_start;
    PacketHandler handle_data_sync_end;
    PacketHandler handle_sport_data;
    PacketHandler handle_sleep_data;
    PacketHandler handle_heart_data;
    PacketHandler handle_cancel_heart_sample;
    PacketHandler handle_return_heart_setting;
    PacketHandler handle_heart_rate_series;

    // Control handlers
    PacketHandler handle_phone_control_cmd;
    PacketHandler handle_find_mobile;
    PacketHandler handle_call_reject;
    PacketHandler handle_media_control;
    PacketHandler handle_mqtt_control;
    PacketHandler handle_volume_percentage;
    PacketHandler handle_voice2text_intent;
    PacketHandler handle_voice_record_intent;
    PacketHandler handle_gesture_mode_state;
    PacketHandler handle_virtual_gesture;
    PacketHandler handle_finger_tap;
    PacketHandler handle_cursor_movement;
    PacketHandler handle_virtual_movement;

    // Notification handlers
    PacketHandler handle_charge_status;
    PacketHandler handle_weather_request;
    PacketHandler handle_calendar_request;
    PacketHandler handle_coordinate;
    PacketHandler handle_gesture_detect;
    PacketHandler handle_remote_input;
    PacketHandler handle_create_note;
    PacketHandler handle_create_calendar;
    PacketHandler handle_tp_coordinate;
    PacketHandler handle_tp_gesture;
    PacketHandler handle_user_speaking_state;
    PacketHandler handle_chat_with_ai;
    PacketHandler handle_quaternion_data;
    PacketHandler handle_battery_voltage;
    PacketHandler handle_battery_level;

    // Sensor Data handlers
    PacketHandler handle_soft_adt_status;
    PacketHandler handle_linear_acce_buffer;
    PacketHandler handle_gsensor_fft_buffer;
    PacketHandler handle_gsensor_ppg_buffer;
    PacketHandler handle_gsensor_gravity_data;
    PacketHandler handle_imu_buffer;
    PacketHandler handle_baro_buffer;

    // File Sync handlers
    PacketHandler handle_start_sync_file;
    PacketHandler handle_sync_file;
    PacketHandler handle_end_sync_file;
    PacketHandler handle_file_compare_result;

    // Other handlers
    PacketHandler handle_bluetooth_log;
    PacketHandler handle_watch_system_sync;
    PacketHandler handle_audio_data;
    PacketHandler handle_audio_file;
    PacketHandler handle_ota_status;
    PacketHandler handle_utest_state;
    PacketHandler handle_minute_activity;
    PacketHandler handle_holding_displacement;
    PacketHandler handle_update_instruction;
} CommunicateHandlerProvider;

static CommunicateHandlerProvider commu_handler_provider;

// Handler count for statistics
#define COMMU_HANDLER_COUNT 70

// Statistics for error tracking
typedef struct
{
    uint32_t send_success;
    uint32_t send_failed;
    uint32_t ringbuf_full;
    uint32_t queue_full;
    uint32_t invalid_event;
} CommuStats;

static CommuStats commu_stats = {0};

/*============================================================================*
 *                              Variables
 *============================================================================*/

#if USING_L1_MESSAGE_QUEUE
static rt_thread_t l1send_task_handle = RT_NULL;
static rt_mq_t l1send_queue_handle = RT_NULL;
#endif

#ifdef USING_BLE_RINGBUFFER_STATION
static struct rt_ringbuffer *commu_rb;
static rt_uint8_t temp_send_buf[512];
static struct rt_mutex commu_rb_mutex;

static void commu_station_api_lock(void)
{
    rt_mutex_take(&commu_rb_mutex, RT_WAITING_FOREVER);
}
static void commu_station_api_unlock(void)
{
    rt_mutex_release(&commu_rb_mutex);
}

static struct rt_event commu_event;

#endif

/*============================================================================*
 *                              Packet Handler Functions
 *============================================================================*/

// Forward declaration
static uint16_t dispatch_packet_handler(L1SendData *data);
static bool communicate_send_api(PacketBuilder *builder);

// Helper: Build simple status packet (1 byte payload)
static inline uint16_t build_status_packet(uint8_t *buf, uint8_t cmd_id,
                                           uint8_t key, uint8_t status)
{
    BUILD_SIMPLE_PACKET(buf, cmd_id, key, 1);
    buf[5] = status;
    return 6;
}

// Handler for bond fail event
static uint16_t handle_bond_fail(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, BOND_COMMAND_ID,
                                          KEY_BOND_RESPOSE, BOND_FAIL);
    return builder->length;
}

// Handler for bond success event
static uint16_t handle_bond_success(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, BOND_COMMAND_ID,
                                          KEY_BOND_RESPOSE, BOND_SUCCESS);
    return builder->length;
}

// Handler for login fail event
static uint16_t handle_login_fail(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, BOND_COMMAND_ID,
                                          KEY_LOGIN_RESPONSE, LOGIN_FAIL);
    return builder->length;
}

// Handler for login success event
static uint16_t handle_login_success(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, BOND_COMMAND_ID,
                                          KEY_LOGIN_RESPONSE, LOGIN_SUCCESS);
    return builder->length;
}

// Handler for alarm return event
static uint16_t handle_return_alarm(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
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
    builder->length = 5 + alarm_item_count * 5;
    return builder->length;
}

// Handler for phone control command
static uint16_t handle_phone_control_cmd(PacketBuilder *builder,
                                         L1SendData *data)
{
    BUILD_SIMPLE_PACKET(builder->buf, CONTROL_COMMAND_ID, KEY_TAKE_PHOTO, 0);
    builder->length = 5;
    return builder->length;
}

// Handler for data sync start
static uint16_t handle_data_sync_start(PacketBuilder *builder, L1SendData *data)
{
    BUILD_SIMPLE_PACKET(builder->buf, HEALTH_DATA_COMMAND_ID,
                        KEY_DATA_SYNC_START, 0);
    builder->length = 5;
    return builder->length;
}

// Handler for data sync end
static uint16_t handle_data_sync_end(PacketBuilder *builder, L1SendData *data)
{
    BUILD_SIMPLE_PACKET(builder->buf, HEALTH_DATA_COMMAND_ID, KEY_DATA_SYNC_END,
                        0);
    builder->length = 5;
    return builder->length;
}

// Handler for sport data
static uint16_t handle_sport_data(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    BUILD_PACKET_HEADER(buf, HEALTH_DATA_COMMAND_ID, KEY_RETURN_SPORTS_DATA);

    uint8_t length = sizeof(watch_sys_heath_info_t);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, &SkaiWatchSys.health_info_today, length);

    builder->length = 5 + length;
    return builder->length;
}

// Handler for sleep data
static uint16_t handle_sleep_data(PacketBuilder *builder, L1SendData *data)
{
    builder->length =
        build_status_packet(builder->buf, HEALTH_DATA_COMMAND_ID,
                            KEY_RETURN_SLEEP_DATA, data->res.status);
    return builder->length;
}

// Handler for heart rate data
static uint16_t handle_heart_data(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, HEALTH_DATA_COMMAND_ID,
                                          KEY_HEART_DATA_RETURN, data->res.hr);
    return builder->length;
}

// Handler for cancel heart sample
static uint16_t handle_cancel_heart_sample(PacketBuilder *builder,
                                           L1SendData *data)
{
    BUILD_SIMPLE_PACKET(builder->buf, HEALTH_DATA_COMMAND_ID,
                        KEY_CANCEL_HEART_SAMPLE, 0);
    builder->length = 5;
    return builder->length;
}

// Handler for heart rate setting return
static uint16_t handle_return_heart_setting(PacketBuilder *builder,
                                            L1SendData *data)
{
    uint8_t *buf = builder->buf;
    BUILD_SIMPLE_PACKET(buf, HEALTH_DATA_COMMAND_ID,
                        KEY_RETURN_HEART_SAMPLE_SETTING, 2);
    buf[5] = SkaiWatchSys.hrs_detect_period ? 0x01 : 0x00;
    buf[6] = SkaiWatchSys.hrs_detect_period;
    builder->length = 7;
    return builder->length;
}

// Handler for find mobile command
static uint16_t handle_find_mobile(PacketBuilder *builder, L1SendData *data)
{
    BUILD_SIMPLE_PACKET(builder->buf, CONTROL_COMMAND_ID, KEY_FIND_PHONE, 0);
    builder->length = 5;
    return builder->length;
}

// Handler for lift switch return
static uint16_t handle_lift_switch(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(
        builder->buf, SET_CONFIG_COMMAND_ID, KEY_LIFT_SWITCH_RETURN,
        SkaiWatchSys.flag_field.lift_switch_status);
    return builder->length;
}

// Handler for twist switch return
static uint16_t handle_twist_switch(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(
        builder->buf, SET_CONFIG_COMMAND_ID, KEY_TWIST_SWITCH_RETURN,
        SkaiWatchSys.flag_field.twist_switch_status);
    return builder->length;
}

// Handler for incoming message settings
static uint16_t handle_incoming_message_settings(PacketBuilder *builder,
                                                 L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, SET_CONFIG_COMMAND_ID,
                                          KEY_INCOMMING_MESSAGE_SETTINGS_RETURN,
                                          SkaiWatchSys.msg_switch.data);
    return builder->length;
}

// Handler for sit setting return
static uint16_t handle_sit_setting(PacketBuilder *builder, L1SendData *data)
{
    uint8_t status =
        (SkaiWatchSys.sit_alert_data.sit_alert.on_off == 0x01) ? 0x01 : 0x00;
    builder->length =
        build_status_packet(builder->buf, SET_CONFIG_COMMAND_ID,
                            KEY_LONG_TIME_SIT_SETTING_RETURN, status);
    return builder->length;
}

// Handler for call reject command
static uint16_t handle_call_reject(PacketBuilder *builder, L1SendData *data)
{
    BUILD_SIMPLE_PACKET(builder->buf, NOTIFY_COMMAND_ID,
                        KEY_INCOMMING_CALL_REJECT, 0);
    builder->length = 5;
    return builder->length;
}

// Handler for hour format setting
static uint16_t handle_hour_format(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, SET_CONFIG_COMMAND_ID,
                                          KEY_HOUR_FORMAT_RETURN,
                                          SkaiWatchSys.flag_field.hour_format);
    return builder->length;
}

// Handler for distance unit setting
static uint16_t handle_distance_unit(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(
        builder->buf, SET_CONFIG_COMMAND_ID, KEY_DISTANCE_UNIT_RETURN,
        SkaiWatchSys.flag_field.distance_unit);
    return builder->length;
}

// Handler for DND mode setting
static uint16_t handle_dndm_setting(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    BUILD_SIMPLE_PACKET(buf, SET_CONFIG_COMMAND_ID, KEY_DNDM_RETURN, 3);
    buf[5] = SkaiWatchSys.DNDMode.data >> 16;
    buf[6] = SkaiWatchSys.DNDMode.data >> 8;
    buf[7] = SkaiWatchSys.DNDMode.data;
    builder->length = 8;
    return builder->length;
}

// Handler for OLED display time
static uint16_t handle_oled_display_time(PacketBuilder *builder,
                                         L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, SET_CONFIG_COMMAND_ID,
                                          KEY_OLED_DISPLAY_TIME_RETURN,
                                          data->res.lcd_display_time);
    return builder->length;
}

// Handler for language return
static uint16_t handle_language(PacketBuilder *builder, L1SendData *data)
{
    builder->length =
        build_status_packet(builder->buf, SET_CONFIG_COMMAND_ID,
                            KEY_LANGUAGE_RETURN, SkaiWatchSys.language);
    return builder->length;
}

// Handler for charge status
static uint16_t handle_charge_status(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, NOTIFY_COMMAND_ID,
                                          KEY_BATTERY_CHARGE_STATUS,
                                          SkaiWatchSys.charger_status);
    return builder->length;
}

// Handler for weather data request
static uint16_t handle_weather_request(PacketBuilder *builder, L1SendData *data)
{
    BUILD_SIMPLE_PACKET(builder->buf, NOTIFY_COMMAND_ID, KEY_REQUEST_WEATHER,
                        0);
    builder->length = 5;
    return builder->length;
}

// Handler for calendar data request
static uint16_t handle_calendar_request(PacketBuilder *builder,
                                        L1SendData *data)
{
    BUILD_SIMPLE_PACKET(builder->buf, NOTIFY_COMMAND_ID, KEY_REQUEST_CALENDAR,
                        0);
    builder->length = 5;
    return builder->length;
}

// Handler for device info return
static uint16_t handle_device_info(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    uint16_t deviceID = CUSTOMER_BOARD_VER;

    BUILD_SIMPLE_PACKET(buf, SET_CONFIG_COMMAND_ID, KEY_DEVICEINFO_RETURN, 5);
    buf[5] = deviceID >> 8;
    buf[6] = deviceID & 0xFF;
    buf[7] = VERSION_MAJOR;
    buf[8] = VERSION_MINOR;
    buf[9] = VERSION_REVISION;

    builder->length = 10;
    return builder->length;
}

// Handler for dial change
static uint16_t handle_dial_change(PacketBuilder *builder, L1SendData *data)
{
    builder->length =
        build_status_packet(builder->buf, SET_CONFIG_COMMAND_ID,
                            KEY_DIAL_RETURN, SkaiWatchSys.clock_status);
    return builder->length;
}

// Handler for backlight event
static uint16_t handle_backlight(PacketBuilder *builder, L1SendData *data)
{
    builder->length =
        build_status_packet(builder->buf, SET_CONFIG_COMMAND_ID,
                            KEY_BACKLIGHT_RETURN, data->res.lcd_brightness);
    return builder->length;
}

// Handler for soft ADT status
static uint16_t handle_soft_adt_status(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(
        builder->buf, NOTIFY_COMMAND_ID, KEY_SOFT_ADT_STATUS, data->res.status);
    return builder->length;
}

// Handler for linear acceleration buffer (with segmentation support)
static uint16_t handle_linear_acce_buffer(PacketBuilder *builder,
                                          L1SendData *data)
{
    uint8_t *buf = builder->buf;
    uint16_t len = data->res.imu_data.length;

    LOG_D("L1SEND_LINEAR_ACCE_BUFFER length=%d", len);

    if (len > 483)
    {
        // 分段傳送，每段最多 483 bytes
        uint16_t offset = 0;
        uint8_t segment_index = 1;
        while (offset < len)
        {
            uint16_t chunk_size = (len - offset > 483) ? 483 : (len - offset);
            BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_GSENSOR_SAMPLE);
            buf[3] = (chunk_size + 1) >> 8;
            buf[4] = (chunk_size + 1) & 0xFF;
            buf[5] = segment_index;
            memcpy(buf + 6, data->res.imu_data.data + offset, chunk_size);
            builder->length = chunk_size + 6;
            communicate_send_api(builder);
            offset += chunk_size;
            segment_index++;
        }
        // Return 0 since we already sent all segments
        return 0;
    }
    else
    {
        BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_GSENSOR_SAMPLE);
        buf[3] = (len + 1) >> 8;
        buf[4] = (len + 1) & 0xFF;
        buf[5] = 0; // no segmentation
        memcpy(buf + 6, data->res.imu_data.data, len);
        builder->length = len + 6;
    }

    return builder->length;
}

// Handler for GSensor FFT buffer
static uint16_t handle_gsensor_fft_buffer(PacketBuilder *builder,
                                          L1SendData *data)
{
    uint8_t *buf = builder->buf;
    uint16_t len = data->res.imu_data.length;

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_GSENSOR_FFT_DATA);
    SET_PACKET_LENGTH(buf, len);
    memcpy(buf + 5, data->res.imu_data.data, len);

    builder->length = len + 5;
    return builder->length;
}

// Handler for GSensor PPG buffer
static uint16_t handle_gsensor_ppg_buffer(PacketBuilder *builder,
                                          L1SendData *data)
{
    uint8_t *buf = builder->buf;
    uint16_t len = data->res.imu_data.length;

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_GSENSOR_PPG_DATA);
    SET_PACKET_LENGTH(buf, len);
    memcpy(buf + 5, data->res.imu_data.data, len);

    builder->length = len + 5;
    return builder->length;
}

// Handler for GSensor gravity data
static uint16_t handle_gsensor_gravity_data(PacketBuilder *builder,
                                            L1SendData *data)
{
    uint8_t *buf = builder->buf;
    uint8_t len = sizeof(watch_sensor.motion_data.gravity);

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_GSENSOR_GRAVITY_DATA);
    SET_PACKET_LENGTH(buf, len);
    memcpy(buf + 5, &watch_sensor.motion_data.gravity, len);

    builder->length = len + 5;
    return builder->length;
}

// Handler for IMU buffer
static uint16_t handle_imu_buffer(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    uint16_t len = data->res.imu_data.length;

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_IMU_BUFFER);
    SET_PACKET_LENGTH(buf, len);
    memcpy(buf + 5, data->res.imu_data.data, len);

    builder->length = len + 5;
    return builder->length;
}

static uint16_t handle_baro_buffer(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_BARO_BUFFER);
    SET_PACKET_LENGTH(buf, 4);
    // copy float data (4 bytes)
    memcpy(buf + 5, &data->res.baro_data, 4);
    builder->length = 9;
    return builder->length;
}

// Handler for Bluetooth log
static uint16_t handle_bluetooth_log(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    uint16_t buffer_len = strlen(data->res.log_buffer_ptr);

    BUILD_PACKET_HEADER(buf, BLUETOOTH_LOG_COMMAND_ID, KEY_DEBUG);
    SET_PACKET_LENGTH(buf, buffer_len);
    memcpy(buf + 5, data->res.log_buffer_ptr, buffer_len);

    builder->length = 5 + buffer_len;
    return builder->length;
}

// Handler for voice2text intent
static uint16_t handle_voice2text_intent(PacketBuilder *builder,
                                         L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, CONTROL_COMMAND_ID,
                                          KEY_RETURN_VOICE2TEXT_INTENT,
                                          data->res.voice2text_intent);
    return builder->length;
}

// Handler for voice record intent
static uint16_t handle_voice_record_intent(PacketBuilder *builder,
                                           L1SendData *data)
{
    uint8_t *buf;
    uint32_t millisecondsFromEpoch;

    buf = builder->buf;
    millisecondsFromEpoch = data->res.id;

    BUILD_SIMPLE_PACKET(buf, CONTROL_COMMAND_ID, KEY_RETURN_VOICE_RECORD_INTENT,
                        5);

    buf[5] = app_voice_get_recording_intent();
    buf[6] = millisecondsFromEpoch >> 24;
    buf[7] = millisecondsFromEpoch >> 16;
    buf[8] = millisecondsFromEpoch >> 8;
    buf[9] = millisecondsFromEpoch;

    builder->length = 10;
    return builder->length;
}

// Handler for coordinate
static uint16_t handle_coordinate(PacketBuilder *builder, L1SendData *data)
{
    builder->length =
        build_status_packet(builder->buf, NOTIFY_COMMAND_ID,
                            KEY_WRIST_COORDINATE, data->res.coordinate);
    return builder->length;
}

// Handler for gesture mode state
static uint16_t handle_gesture_mode_state(PacketBuilder *builder,
                                          L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, CONTROL_COMMAND_ID,
                                          KEY_GESTURE_MODE_STATUS,
                                          app_control_get_gesture_mode());
    return builder->length;
}

// Handler for gesture detect
static uint16_t handle_gesture_detect(PacketBuilder *builder, L1SendData *data)
{
    builder->length =
        build_status_packet(builder->buf, NOTIFY_COMMAND_ID, KEY_GESTURE_DETECT,
                            data->res.gesture_label);
    return builder->length;
}

// Handler for remote input
static uint16_t handle_remote_input(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    char *string_ptr = data->res.json_string_ptr;
    uint16_t length = strlen(string_ptr);

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_REMOTE_INPUT);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, string_ptr, length);

    builder->length = 5 + length;
    return builder->length;
}

// Handler for create note
static uint16_t handle_create_note(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    char *string_ptr = data->res.json_string_ptr;
    uint16_t len = strlen(string_ptr);

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_CREATE_NOTE);
    SET_PACKET_LENGTH(buf, len);
    memcpy(buf + 5, string_ptr, len);

    builder->length = 5 + len;
    return builder->length;
}

// Handler for update instruction
static uint16_t handle_update_instruction(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    char *string_ptr = data->res.json_string_ptr;
    uint16_t len = strlen(string_ptr);

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_SKAI_CREATION_INSTRUCTIONS);
    SET_PACKET_LENGTH(buf, len);
    memcpy(buf + 5, string_ptr, len);

    builder->length = 5 + len;
    return builder->length;
}

// Handler for create calendar
static uint16_t handle_create_calendar(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    char *string_ptr = data->res.json_string_ptr;
    uint16_t len = strlen(string_ptr);

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_CREATE_CALENDAR);
    SET_PACKET_LENGTH(buf, len);
    memcpy(buf + 5, string_ptr, len);

    builder->length = 5 + len;
    return builder->length;
}

// Handler for heart rate series
static uint16_t handle_heart_rate_series(PacketBuilder *builder,
                                         L1SendData *data)
{
    uint8_t *buf = builder->buf;
    uint16_t len = data->res.ppg_data.length * sizeof(float);

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_HEART_RATE_SENSOR_SAMPLE);
    SET_PACKET_LENGTH(buf, len);
    memcpy(buf + 5, data->res.ppg_data.data, len);

    builder->length = 5 + len;
    return builder->length;
}

// Handler for watch system sync
static uint16_t handle_watch_system_sync(PacketBuilder *builder,
                                         L1SendData *data)
{
    uint8_t *buf = builder->buf;
    size_t total_size = sizeof(SkaiWatchSys);

    LOG_D("L1SEND_WATCH_SYSTEM_SYNC total_size = %d", total_size);

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_WATCH_SYS_RETURN);
    SET_PACKET_LENGTH(buf, total_size);
    memcpy(buf + 5, (uint8_t *)&SkaiWatchSys, total_size);

    builder->length = total_size + 5;
    return builder->length;
}

// Handler for media control
static uint16_t handle_media_control(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, CONTROL_COMMAND_ID,
                                          KEY_PHONE_MEDIA_CONTROL,
                                          app_audio_get_control_command());
    return builder->length;
}

// Handler for MQTT control
static uint16_t handle_mqtt_control(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, CONTROL_COMMAND_ID,
                                          KEY_MQTT_CONTROL, data->res.status);
    return builder->length;
}

// Handler for volume percentage
static uint16_t handle_volume_percentage(PacketBuilder *builder,
                                         L1SendData *data)
{
    builder->length =
        build_status_packet(builder->buf, CONTROL_COMMAND_ID, KEY_RETURN_VOLUMN,
                            data->res.bt_speaker_volume);
    return builder->length;
}

// Handler for TP coordinate
static uint16_t handle_tp_coordinate(PacketBuilder *builder, L1SendData *data)
{
    Send_Cursor_Report();
    builder->length = 0; // Already sent directly
    return 0;
}

// Handler for TP gesture
static uint16_t handle_tp_gesture(PacketBuilder *builder, L1SendData *data)
{
    Send_Touchpad_Gesture();
    builder->length = 0; // Already sent directly
    return 0;
}

// Handler for user speaking state
static uint16_t handle_user_speaking_state(PacketBuilder *builder,
                                           L1SendData *data)
{
    builder->length =
        build_status_packet(builder->buf, NOTIFY_COMMAND_ID,
                            KEY_USER_SPEAKING_STATE, data->res.status);
    return builder->length;
}

// Handler for chat with AI
static uint16_t handle_chat_with_ai(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    char *string_ptr = data->res.json_string_ptr;
    uint16_t length = strlen(string_ptr);

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_RETURN_CHAT_INTENT);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, string_ptr, length);

    builder->length = 5 + length;
    return builder->length;
}

// Handler for quaternion data
static uint16_t handle_quaternion_data(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    BUILD_SIMPLE_PACKET(buf, NOTIFY_COMMAND_ID, KEY_QUATERNION_DATA, 16);
    memcpy(buf + 5, &quaternion_buffer, 16);
    builder->length = 21;
    return builder->length;
}

// Handler for battery voltage
static uint16_t handle_battery_voltage(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    BUILD_SIMPLE_PACKET(buf, NOTIFY_COMMAND_ID, KEY_BATTERY_VOLTAGE, 2);
    buf[5] = data->res.battery_voltage >> 8;
    buf[6] = data->res.battery_voltage & 0xFF;
    builder->length = 7;
    return builder->length;
}

// Handler for battery level
static uint16_t handle_battery_level(PacketBuilder *builder, L1SendData *data)
{
    builder->length =
        build_status_packet(builder->buf, NOTIFY_COMMAND_ID, KEY_BATTERY_LEVEL,
                            data->res.battery_level);
    return builder->length;
}

// Handler for audio data
static uint16_t handle_audio_data(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    uint16_t length = data->res.audio_buffer.length;

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_AUDIO_DATA);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, data->res.audio_buffer.data, length);

    builder->length = length + 5;
    return builder->length;
}

// Handler for audio file
static uint16_t handle_audio_file(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    uint16_t length = data->res.audio_buffer.length;

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_AUDIO_FILE);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, data->res.audio_buffer.data, length);

    builder->length = length + 5;
    return builder->length;
}

// Handler for start sync file
static uint16_t handle_start_sync_file(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    char *file_path = get_sync_in_file_path();
    uint8_t path_len = strlen(file_path);
    uint32_t total_size = data->res.id; /* Get file size from data */

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_START_SYNC_FILE);

    /* Protocol: [total_size(4)][file_path] */
    buf[3] = 0;
    buf[4] = path_len + 4; /* payload length = 4 bytes for size + path_len */

    /* Write total file size (big endian) */
    buf[5] = (total_size >> 24) & 0xFF;
    buf[6] = (total_size >> 16) & 0xFF;
    buf[7] = (total_size >> 8) & 0xFF;
    buf[8] = total_size & 0xFF;

    /* Write file path */
    memcpy(buf + 9, file_path, path_len);

    builder->length = 9 + path_len;
    return builder->length;
}

// Handler for sync file
static uint16_t handle_sync_file(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    uint16_t length = data->res.file_buffer.length;

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_SYNC_FILE);
    SET_PACKET_LENGTH(buf, length);
    memcpy(buf + 5, data->res.file_buffer.data, length);

    builder->length = length + 5;
    return builder->length;
}

// Handler for end sync file
static uint16_t handle_end_sync_file(PacketBuilder *builder, L1SendData *data)
{
    BUILD_SIMPLE_PACKET(builder->buf, NOTIFY_COMMAND_ID, KEY_END_SYNC_FILE, 0);
    builder->length = 5;
    return builder->length;
}

static uint16_t send_file_compare_result(PacketBuilder *builder,
                                         L1SendData *data)
{
    BUILD_SIMPLE_PACKET(builder->buf, NOTIFY_COMMAND_ID,
                        KEY_FILE_COMPARE_RESULT, 1);
    builder->buf[5] = data->res.status;
    builder->length = 6;
    return builder->length;
}

// Handler for virtual gesture
static uint16_t handle_virtual_gesture(PacketBuilder *builder, L1SendData *data)
{
    builder->length =
        build_status_packet(builder->buf, CONTROL_COMMAND_ID,
                            KEY_VIRTUAL_GESTURE, data->res.gesture_label);
    return builder->length;
}

// Handler for finger tap
static uint16_t handle_finger_tap(PacketBuilder *builder, L1SendData *data)
{
    builder->length =
        build_status_packet(builder->buf, CONTROL_COMMAND_ID, KEY_TP_GESTURE,
                            data->res.gesture_label);
    return builder->length;
}

// Handler for cursor movement
static uint16_t handle_cursor_movement(PacketBuilder *builder, L1SendData *data)
{
    Send_Cursor_Report();
    builder->length = 0; // Already sent directly
    return 0;
}

// Handler for virtual movement
static uint16_t handle_virtual_movement(PacketBuilder *builder,
                                        L1SendData *data)
{
    uint8_t *buf = builder->buf;
    BUILD_SIMPLE_PACKET(buf, CONTROL_COMMAND_ID, KEY_TP_COORDINATE, 3);
    buf[5] = data->res.virtual_movement.direction;
    buf[6] = data->res.virtual_movement.distance;
    buf[7] = data->res.virtual_movement.is_tap;
    builder->length = 8;
    return builder->length;
}

// Handler for OTA status
static uint16_t handle_ota_status(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, NOTIFY_COMMAND_ID,
                                          KEY_OTA_STATUS, data->res.status);
    return builder->length;
}

// Handler for unit test state
static uint16_t handle_utest_state(PacketBuilder *builder, L1SendData *data)
{
    builder->length = build_status_packet(builder->buf, SKAI_LINK_COMMAND_ID,
                                          KEY_UNIT_TEST_SYNC, data->res.status);
    return builder->length;
}

// Handler for minute activity
static uint16_t handle_minute_activity(PacketBuilder *builder, L1SendData *data)
{
    uint8_t *buf = builder->buf;
    uint8_t len = sizeof(SkaiWatchSys.activity);

    BUILD_PACKET_HEADER(buf, NOTIFY_COMMAND_ID, KEY_MINUTE_ACTIVITY);
    buf[3] = 0;
    buf[4] = len;
    memcpy(buf + 5, &SkaiWatchSys.activity, len);

    builder->length = 5 + len;
    return builder->length;
}

// Handler for holding displacement
static uint16_t handle_holding_displacement(PacketBuilder *builder,
                                            L1SendData *data)
{
    uint8_t *buf = builder->buf;
    BUILD_SIMPLE_PACKET(buf, NOTIFY_COMMAND_ID, KEY_HOLDING_DISPLACEMENT, 9);
    buf[5] = data->res.holding_displacement.event;
    memcpy(buf + 6, &data->res.holding_displacement.x,
           sizeof(data->res.holding_displacement.x));
    memcpy(buf + 10, &data->res.holding_displacement.y,
           sizeof(data->res.holding_displacement.y));
    builder->length = 14;
    return builder->length;
}

/*============================================================================*
 *                              Provider Registration (類似
 * bloc_control_provider_register)
 *============================================================================*/
static int commu_handler_provider_register(void)
{
    // Bond & Login
    commu_handler_provider.handle_bond_fail = handle_bond_fail;
    commu_handler_provider.handle_bond_success = handle_bond_success;
    commu_handler_provider.handle_login_fail = handle_login_fail;
    commu_handler_provider.handle_login_success = handle_login_success;

    // Config & Settings
    commu_handler_provider.handle_return_alarm = handle_return_alarm;
    commu_handler_provider.handle_lift_switch = handle_lift_switch;
    commu_handler_provider.handle_twist_switch = handle_twist_switch;
    commu_handler_provider.handle_incoming_message_settings =
        handle_incoming_message_settings;
    commu_handler_provider.handle_sit_setting = handle_sit_setting;
    commu_handler_provider.handle_hour_format = handle_hour_format;
    commu_handler_provider.handle_distance_unit = handle_distance_unit;
    commu_handler_provider.handle_dndm_setting = handle_dndm_setting;
    commu_handler_provider.handle_oled_display_time = handle_oled_display_time;
    commu_handler_provider.handle_language = handle_language;
    commu_handler_provider.handle_device_info = handle_device_info;
    commu_handler_provider.handle_dial_change = handle_dial_change;
    commu_handler_provider.handle_backlight = handle_backlight;

    // Health Data
    commu_handler_provider.handle_data_sync_start = handle_data_sync_start;
    commu_handler_provider.handle_data_sync_end = handle_data_sync_end;
    commu_handler_provider.handle_sport_data = handle_sport_data;
    commu_handler_provider.handle_sleep_data = handle_sleep_data;
    commu_handler_provider.handle_heart_data = handle_heart_data;
    commu_handler_provider.handle_cancel_heart_sample =
        handle_cancel_heart_sample;
    commu_handler_provider.handle_return_heart_setting =
        handle_return_heart_setting;
    commu_handler_provider.handle_heart_rate_series = handle_heart_rate_series;

    // Control
    commu_handler_provider.handle_phone_control_cmd = handle_phone_control_cmd;
    commu_handler_provider.handle_find_mobile = handle_find_mobile;
    commu_handler_provider.handle_call_reject = handle_call_reject;
    commu_handler_provider.handle_media_control = handle_media_control;
    commu_handler_provider.handle_mqtt_control = handle_mqtt_control;
    commu_handler_provider.handle_volume_percentage = handle_volume_percentage;
    commu_handler_provider.handle_voice2text_intent = handle_voice2text_intent;
    commu_handler_provider.handle_voice_record_intent =
        handle_voice_record_intent;
    commu_handler_provider.handle_gesture_mode_state =
        handle_gesture_mode_state;
    commu_handler_provider.handle_virtual_gesture = handle_virtual_gesture;
    commu_handler_provider.handle_finger_tap = handle_finger_tap;
    commu_handler_provider.handle_cursor_movement = handle_cursor_movement;
    commu_handler_provider.handle_virtual_movement = handle_virtual_movement;

    // Notification
    commu_handler_provider.handle_charge_status = handle_charge_status;
    commu_handler_provider.handle_weather_request = handle_weather_request;
    commu_handler_provider.handle_calendar_request = handle_calendar_request;
    commu_handler_provider.handle_coordinate = handle_coordinate;
    commu_handler_provider.handle_gesture_detect = handle_gesture_detect;
    commu_handler_provider.handle_remote_input = handle_remote_input;
    commu_handler_provider.handle_create_note = handle_create_note;
    commu_handler_provider.handle_update_instruction = handle_update_instruction;
    commu_handler_provider.handle_create_calendar = handle_create_calendar;
    commu_handler_provider.handle_tp_coordinate = handle_tp_coordinate;
    commu_handler_provider.handle_tp_gesture = handle_tp_gesture;
    commu_handler_provider.handle_user_speaking_state =
        handle_user_speaking_state;
    commu_handler_provider.handle_chat_with_ai = handle_chat_with_ai;
    commu_handler_provider.handle_quaternion_data = handle_quaternion_data;
    commu_handler_provider.handle_battery_voltage = handle_battery_voltage;
    commu_handler_provider.handle_battery_level = handle_battery_level;

    // Sensor Data
    commu_handler_provider.handle_soft_adt_status = handle_soft_adt_status;
    commu_handler_provider.handle_linear_acce_buffer =
        handle_linear_acce_buffer;
    commu_handler_provider.handle_gsensor_fft_buffer =
        handle_gsensor_fft_buffer;
    commu_handler_provider.handle_gsensor_ppg_buffer =
        handle_gsensor_ppg_buffer;
    commu_handler_provider.handle_gsensor_gravity_data =
        handle_gsensor_gravity_data;
    commu_handler_provider.handle_imu_buffer = handle_imu_buffer;
    commu_handler_provider.handle_baro_buffer = handle_baro_buffer;

    // File Sync
    commu_handler_provider.handle_start_sync_file = handle_start_sync_file;
    commu_handler_provider.handle_sync_file = handle_sync_file;
    commu_handler_provider.handle_end_sync_file = handle_end_sync_file;
    commu_handler_provider.handle_file_compare_result =
        send_file_compare_result;
    // Other
    commu_handler_provider.handle_bluetooth_log = handle_bluetooth_log;
    commu_handler_provider.handle_watch_system_sync = handle_watch_system_sync;
    commu_handler_provider.handle_audio_data = handle_audio_data;
    commu_handler_provider.handle_audio_file = handle_audio_file;
    commu_handler_provider.handle_ota_status = handle_ota_status;
    commu_handler_provider.handle_utest_state = handle_utest_state;
    commu_handler_provider.handle_minute_activity = handle_minute_activity;
    commu_handler_provider.handle_holding_displacement =
        handle_holding_displacement;

    LOG_I("Communication handler provider registered");
    return 0;
}

// Dispatch function - O(1) lookup via provider pattern
static uint16_t dispatch_packet_handler(L1SendData *data)
{
    PacketBuilder builder;
    uint8_t temp_buf[512];
    PacketHandler handler = NULL;

    builder.buf = temp_buf;
    builder.capacity = sizeof(temp_buf);
    builder.length = 0;
    builder.direct_send = false;

    // O(1) switch dispatch - 編譯器會優化成跳轉表
    switch (data->event)
    {
    // Bond & Login
    case L1SEND_BOND_FAIL_EVENT:
        handler = commu_handler_provider.handle_bond_fail;
        break;
    case L1SEND_BOND_SUCCESS_EVENT:
        handler = commu_handler_provider.handle_bond_success;
        break;
    case L1SEND_LOGIN_FAIL_EVENT:
        handler = commu_handler_provider.handle_login_fail;
        break;
    case L1SEND_LOGIN_SUCCESS_EVENT:
        handler = commu_handler_provider.handle_login_success;
        break;

    // Config & Settings
    case L1SEND_RETURN_ALARM_EVENT:
        handler = commu_handler_provider.handle_return_alarm;
        break;
    case L1SEND_RETURN_LIFT_SWITCH_EVENT:
        handler = commu_handler_provider.handle_lift_switch;
        break;
    case L1SEND_RETURN_TWIST_SWITCH_EVENT:
        handler = commu_handler_provider.handle_twist_switch;
        break;
    case L1SEND_RETURN_INCOMMING_MESSAGE_SETTINGS:
        handler = commu_handler_provider.handle_incoming_message_settings;
        break;
    case L1SEND_SIT_SETTING_RETURN:
        handler = commu_handler_provider.handle_sit_setting;
        break;
    case L1SEND_RETURN_HOUR_FORMAT_SETTING:
        handler = commu_handler_provider.handle_hour_format;
        break;
    case L1SEND_RETURN_DISTANCE_UNIT_SETTING:
        handler = commu_handler_provider.handle_distance_unit;
        break;
    case L1SEND_RETURN_DNDM_SETTING:
        handler = commu_handler_provider.handle_dndm_setting;
        break;
    case L1SEND_RETURN_OLED_DISPLAY_TIME:
        handler = commu_handler_provider.handle_oled_display_time;
        break;
    case L1SEND_RETURN_LANGUAGE:
        handler = commu_handler_provider.handle_language;
        break;
    case L1SEND_RETURN_DEVICE_INFO:
        handler = commu_handler_provider.handle_device_info;
        break;
    case L1SEND_RETURN_DIAL_CHANGE:
        handler = commu_handler_provider.handle_dial_change;
        break;
    case L1SEND_RETURN_BACKLIGHT_EVENT:
    {
        LOG_D("Dispatching backlight event");
        handler = commu_handler_provider.handle_backlight;
        break;
    }

    // Health Data
    case L1SEND_HISTORY_DATA_SYNC_START:
        handler = commu_handler_provider.handle_data_sync_start;
        break;
    case L1SEND_HISTORY_DATA_SYNC_END:
        handler = commu_handler_provider.handle_data_sync_end;
        break;
    case L1SEND_SPORT_DATA:
        handler = commu_handler_provider.handle_sport_data;
        break;
    case L1SEND_SLEEP_DATA:
        handler = commu_handler_provider.handle_sleep_data;
        break;
    case L1SEND_HEART_DATA:
        handler = commu_handler_provider.handle_heart_data;
        break;
    case L1SEND_RETURN_CANCEL_HEART_SAMPLE:
        handler = commu_handler_provider.handle_cancel_heart_sample;
        break;
    case L1SEND_RETURN_HEART_SETTING:
        handler = commu_handler_provider.handle_return_heart_setting;
        break;
    case L1SEND_HEART_RATE_SERIES:
        handler = commu_handler_provider.handle_heart_rate_series;
        break;

    // Control
    case L1SEND_RETURN_PHONE_CONTROL_CMD_EVENT:
        handler = commu_handler_provider.handle_phone_control_cmd;
        break;
    case L1SEND_RETURN_FIND_MOBILE_COMMAND:
        handler = commu_handler_provider.handle_find_mobile;
        break;
    case L1SEND_RETURN_CALL_REJECT_COMMAND:
        handler = commu_handler_provider.handle_call_reject;
        break;
    case L1SEND_MEDIA_CONTROL:
        handler = commu_handler_provider.handle_media_control;
        break;
    case L1SEND_MQTT_CONTROL:
        handler = commu_handler_provider.handle_mqtt_control;
        break;
    case L1SEND_VOLUME_PERCENTAGE:
        handler = commu_handler_provider.handle_volume_percentage;
        break;
    case L1SEND_RETURN_VOICE2TEXT_INTENT:
        handler = commu_handler_provider.handle_voice2text_intent;
        break;
    case L1SEND_RETURN_VOICE_RECORD_INTENT:
        handler = commu_handler_provider.handle_voice_record_intent;
        break;
    case L1SEND_RETURN_GESTURE_MODE_STATE:
        handler = commu_handler_provider.handle_gesture_mode_state;
        break;
    case L1SEND_VIRTUAL_GESTURE:
        handler = commu_handler_provider.handle_virtual_gesture;
        break;
    case L1SEND_FINGER_TAP:
        handler = commu_handler_provider.handle_finger_tap;
        break;
    case L1SEND_CURSOR_MOVEMENT:
        handler = commu_handler_provider.handle_cursor_movement;
        break;
    case L1SEND_VIRTUAL_MOVEMENT:
        handler = commu_handler_provider.handle_virtual_movement;
        break;

    // Notification
    case L1SEND_RETURN_CHARGE_STATUS:
        handler = commu_handler_provider.handle_charge_status;
        break;
    case L1SEND_RETURN_WEATHER_DATA_GET:
        handler = commu_handler_provider.handle_weather_request;
        break;
    case L1SEND_RETURN_CALENDAR_DATA_GET:
        handler = commu_handler_provider.handle_calendar_request;
        break;
    case L1SEND_COORDINATE:
        handler = commu_handler_provider.handle_coordinate;
        break;
    case L1SEND_GESTURE_DETECT:
        handler = commu_handler_provider.handle_gesture_detect;
        break;
    case L1SEND_REMOTE_INPUT:
        handler = commu_handler_provider.handle_remote_input;
        break;
    case L1SEND_CREATE_NOTE:
        handler = commu_handler_provider.handle_create_note;
        break;
    case L1SEND_CREATE_CALENDAR:
        handler = commu_handler_provider.handle_create_calendar;
        break;
    case L1SEND_RETURN_TP_COORDINATE:
        handler = commu_handler_provider.handle_tp_coordinate;
        break;
    case L1SEND_RETURN_TP_GESTURE:
        handler = commu_handler_provider.handle_tp_gesture;
        break;
    case L1SEND_RETURN_USER_SPEAKING_STATE:
        handler = commu_handler_provider.handle_user_speaking_state;
        break;
    case L1SEND_CHAT_WITH_AI:
        handler = commu_handler_provider.handle_chat_with_ai;
        break;
    case L1SEND_QUATERNION_DATA:
        handler = commu_handler_provider.handle_quaternion_data;
        break;
    case L1SEND_RETURN_BATTERY_VOLTAGE:
        handler = commu_handler_provider.handle_battery_voltage;
        break;
    case L1SEND_RETURN_BATTERY_LEVEL:
        handler = commu_handler_provider.handle_battery_level;
        break;

    // Sensor Data
    case L1SEND_RETURN_SOFT_ADT_STATUS:
        handler = commu_handler_provider.handle_soft_adt_status;
        break;
    case L1SEND_LINEAR_ACCE_BUFFER:
        handler = commu_handler_provider.handle_linear_acce_buffer;
        break;
    case L1SEND_GSENSOR_FFT_BUFFER:
        handler = commu_handler_provider.handle_gsensor_fft_buffer;
        break;
    case L1SEND_GSENSOR_PPG_BUFFER:
        handler = commu_handler_provider.handle_gsensor_ppg_buffer;
        break;
    case L1SEND_GSENSOR_GRAVITY_DATA:
        handler = commu_handler_provider.handle_gsensor_gravity_data;
        break;
    case L1SEND_IMU_BUFFER:
        handler = commu_handler_provider.handle_imu_buffer;
        break;
    case L1SEND_BARO_BUFFER:
        handler = commu_handler_provider.handle_baro_buffer;
        break;

    // File Sync
    case L1SEND_START_SYNC_FILE:
        handler = commu_handler_provider.handle_start_sync_file;
        break;
    case L1SEND_SYNC_FILE:
        handler = commu_handler_provider.handle_sync_file;
        break;
    case L1SEND_END_SYNC_FILE:
        handler = commu_handler_provider.handle_end_sync_file;
        break;
    case L1SEND_FILE_COMPARE_RESULT:
        handler = commu_handler_provider.handle_file_compare_result;
        break;

    // Other
    case L1SEND_BLUETOOTH_LOG:
        handler = commu_handler_provider.handle_bluetooth_log;
        break;
    case L1SEND_WATCH_SYSTEM_SYNC:
        handler = commu_handler_provider.handle_watch_system_sync;
        break;
    case L1SEND_AUDIO_DATA:
        handler = commu_handler_provider.handle_audio_data;
        break;
    case L1SEND_AUDIO_FILE:
        handler = commu_handler_provider.handle_audio_file;
        break;
    case L1SEND_RETURN_OTA_STATUS:
        handler = commu_handler_provider.handle_ota_status;
        break;
    case L1SEND_RETURN_UTEST_STATE:
        handler = commu_handler_provider.handle_utest_state;
        break;
    case L1SEND_RETURN_MINUTE_ACTIVITY:
        handler = commu_handler_provider.handle_minute_activity;
        break;
    case L1SEND_HOLDING_DISPLACEMENT:
        handler = commu_handler_provider.handle_holding_displacement;
        break;
    case L1SEND_UPDATE_INSTRUCTION:
        handler = commu_handler_provider.handle_update_instruction;
        break;

    default:
        commu_stats.invalid_event++;
        LOG_E("No handler for event type: %d", data->event);
        return 0;
    }

    // Call the handler if found
    if (handler != NULL)
    {
        uint16_t len = handler(&builder, data);

        // Send the packet if length > 0 (zero-copy: write directly to
        // ringbuffer)
        if (len > 0)
        {
            if (communicate_send_api(&builder))
            {
                return len;
            }
            else
            {
                LOG_E("communicate_send_api failed for event type: %d",
                      data->event);
                return 0;
            }
        }
        else
        {
            LOG_W("Handler for event type %d returned zero length",
                  data->event);
        }
        return len;
    }

    return 0;
}

/*============================================================================*
 *                              Debug & Statistics Functions
 *============================================================================*/

/**
 * @brief Print communication statistics
 * @return void
 */
void commu_print_stats(void)
{
    LOG_I("=== Communication Statistics ===");
    LOG_I("Send success: %u", commu_stats.send_success);
    LOG_I("Send failed: %u", commu_stats.send_failed);
    LOG_I("Ringbuf full: %u", commu_stats.ringbuf_full);
    LOG_I("Queue full: %u", commu_stats.queue_full);
    LOG_I("Invalid event: %u", commu_stats.invalid_event);
    LOG_I("Registered handlers: %d", COMMU_HANDLER_COUNT);
    LOG_I("===============================");
}

/**
 * @brief Reset communication statistics
 * @return void
 */
void commu_reset_stats(void)
{
    commu_stats.send_success = 0;
    commu_stats.send_failed = 0;
    commu_stats.ringbuf_full = 0;
    commu_stats.queue_full = 0;
    commu_stats.invalid_event = 0;
    LOG_I("Communication statistics reset");
}

#ifdef RT_USING_FINSH
    #include <finsh.h>
MSH_CMD_EXPORT(commu_print_stats, Print communication statistics);
MSH_CMD_EXPORT(commu_reset_stats, Reset communication statistics);
#endif

/*============================================================================*
 *                              Functions
 *============================================================================*/
/**
 * @brief Zero-copy packet builder - 直接在 ringbuffer 中構建封包
 * @param builder PacketBuilder context with ringbuffer pointer
 * @return true on success, false on failure
 */
static bool communicate_send_api(PacketBuilder *builder)
{
#ifdef USING_BLE_RINGBUFFER_STATION
    uint16_t length = builder->length;

    // Check if there's enough space in the ringbuffer
    commu_station_api_lock();

    if (rt_ringbuffer_space_len(commu_rb) < length + 2)
    {
        commu_station_api_unlock();
        commu_stats.ringbuf_full++;
        rt_thread_mdelay(50);
        return false;
    }

    // Write length header (2 bytes) directly to ringbuffer
    uint8_t len_header[2];
    len_header[0] = (length >> 8) & 0xFF;
    len_header[1] = length & 0xFF;

    rt_size_t res1 = rt_ringbuffer_put(commu_rb, len_header, 2);
    if (res1 != 2)
    {
        commu_station_api_unlock();
        return false;
    }

    // Write packet data directly to ringbuffer (零拷貝!)
    rt_size_t res2 = rt_ringbuffer_put(commu_rb, builder->buf, length);
    commu_station_api_unlock();

    if (res2 != length)
    {
        return false;
    }

    // Signal the event to process the new data
    if (rt_event_send(&commu_event, BLE_NOTIFY_EVENT) == RT_EOK)
    {
        return true;
    }
    else
    {
        return false;
    }
#else
    return skaiwatch_ble_notify(builder->buf, builder->length);
#endif
}

extern int audio_profile_send_voice_data(uint8_t *voice_data,
                                         uint16_t voice_data_len);
bool skaiwatch_ble_audio_send(uint8_t *buf, uint16_t length)
{
    int res = audio_profile_send_voice_data(buf, length);
    return res > 0;
}

#ifdef USING_BLE_RINGBUFFER_STATION

static void ble_send_commu_station_buffer(rt_uint8_t *temp_buf,
                                          bool *has_more_data)
{
    if (commu_rb == RT_NULL)
    {
        *has_more_data = false;
        return;
    }

    commu_station_api_lock();
    rt_size_t data_len = rt_ringbuffer_data_len(commu_rb);

    if (data_len > 0)
    {
        // Read the first two bytes to determine data length
        rt_ringbuffer_get(commu_rb, temp_buf, 2);
        uint16_t buf_len = (temp_buf[0] << 8) | temp_buf[1];

        // Read the actual data
        rt_size_t res = rt_ringbuffer_get(commu_rb, temp_buf, buf_len);

        // Check if there's more data in the buffer
        *has_more_data = (rt_ringbuffer_data_len(commu_rb) > 0);

        commu_station_api_unlock();

        if (res != buf_len)
        {
            LOG_E("[ble_send_commu_station_buffer]: read data from ringbuffer "
                  "failed, res:%d, buf_len:%d",
                  res, buf_len);
            return;
        }

        // Send data over BLE
        skaiwatch_ble_notify(temp_buf, buf_len);
    }
    else
    {
        commu_station_api_unlock();
        *has_more_data = false;
    }
}

static void ble_send_processing(rt_uint8_t *temp_buf)
{
    bool has_more_data = false;

    do
    {
        // Process each chunk of data in the ringbuffer
        ble_send_commu_station_buffer(temp_buf, &has_more_data);

        // Add a small delay to prevent flooding the BLE connection
        // if (has_more_data)
        // {
        //     rt_thread_mdelay(5);
        // }
    } while (has_more_data);
}

void commu_station_entry(void *parameter)
{
    rt_uint8_t temp_buf[512];
    rt_uint32_t recv_set = 0;

    // Initialize mutex for thread-safe ringbuffer access
    rt_mutex_init(&commu_rb_mutex, "commu_station_mutex", RT_IPC_FLAG_FIFO);

    // Create ringbuffer for BLE communication
    commu_rb = rt_ringbuffer_create(MAX_BLE_RINGBUFFER_SIZE);
    if (commu_rb == RT_NULL)
    {
        LOG_E("Failed to create ringbuffer");
        goto exit;
    }

    // Initialize event for BLE notification
    if (rt_event_init(&commu_event, "commu_event", RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        LOG_E("Failed to create event");
        goto exit;
    }

    while (1)
    {
        // Wait for notification events
        if (rt_event_recv(&commu_event, (BLE_NOTIFY_EVENT),
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &recv_set) == RT_EOK)
        {
            switch (recv_set)
            {
            case BLE_NOTIFY_EVENT:
            {
                ble_send_processing(temp_buf);
                break;
            }

            default:
                break;
            }
        }
    }

exit:
    // Clean up resources
    if (commu_rb != RT_NULL)
    {
        rt_ringbuffer_destroy(commu_rb);
    }
    rt_mutex_detach(&commu_rb_mutex);
    rt_event_detach(&commu_event);
}

static rt_thread_t tid_commu_station;

static int commu_station_init(void)
{
    tid_commu_station = rt_thread_create(
        "commu_station", commu_station_entry, NULL, BLE_STATION_STACK_SIZE,
        BLE_STATION_PRIORITY, BLE_STATION_TICK);
    rt_thread_startup(tid_commu_station);
    return 0;
}

INIT_APP_EXPORT(commu_station_init);
#endif

int L1_send_event(L1SendData data)
{
    if (!SkaiWatchSys.connected_to_phone)
    {
        return -1;
    }
    if (is_ble_dfu_thread_running())
    {
        return -2;
    }
#if USING_L1_MESSAGE_QUEUE
    if (rt_mq_send(l1send_queue_handle, &data, sizeof(data)) != RT_EOK)
    {
        LOG_E("Failed to send L1 event data: %d", data.event);
        return -3;
    }
#else
    return dispatch_packet_handler(&data);
#endif
}

void L1_send_ota_event(L1SendData data)
{
    if (SkaiWatchSys.gap_conn_state != GAP_CONN_STATE_CONNECTED)
    {
        return;
    }
    if (SkaiWatchSys.connected_to_phone)
    {
#if USING_L1_MESSAGE_QUEUE
        if (rt_mq_send(l1send_queue_handle, &data, sizeof(data)) != RT_EOK)
        {
            LOG_E("send_msg_to_l1send_task_fail");
        }
#else
        dispatch_packet_handler(&data);
#endif
    }
}

#if USING_L1_MESSAGE_QUEUE
/**
 * @brief        L1send task
 * @param[in]    p_params    Parameters sending to the task
 * @return       void
 */
void l1send_task(void *pvParameters)
{
    L1SendData data;

    l1send_queue_handle = rt_mq_create("l1send_queue", sizeof(L1SendData),
                                       MAX_L1SEND_MSG_SIZE, RT_IPC_FLAG_FIFO);
    if (l1send_queue_handle == RT_NULL)
    {
        LOG_E("l1send_queue_handle create failed");
        return;
    }

    LOG_I("l1send_task started with %d registered handlers",
          COMMU_HANDLER_COUNT);

    while (true)
    {
        if (rt_mq_recv(l1send_queue_handle, &data, sizeof(data),
                       RT_WAITING_FOREVER) == RT_EOK)
        {
            if (SkaiWatchSys.gap_conn_state == GAP_CONN_STATE_CONNECTED &&
                SkaiWatchSys.connected_to_phone)
            {
                // Use new dispatcher instead of switch-case
                dispatch_packet_handler(&data);
            }
        }
    }
}
#endif

/**
 * @brief  Initialize App task
 * @return void
 */
int communicate_task_init(void)
{
    commu_handler_provider_register();
#if USING_L1_MESSAGE_QUEUE
    l1send_task_handle =
        rt_thread_create("l1send", l1send_task, RT_NULL, L1SEND_TASK_STACK_SIZE,
                         L1SEND_TASK_PRIORITY, L1SEND_TASK_TICK);
    if (l1send_task_handle != RT_NULL)
    {
        rt_thread_startup(l1send_task_handle);
    }
#endif
    return 0;
}
INIT_APP_EXPORT(communicate_task_init);