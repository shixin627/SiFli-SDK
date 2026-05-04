/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      communicate_task.h
   * @brief     Routines to create App task and handle events & messages
   * @author    howie
   * @date      2019-12-26
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */
#ifndef _COMMUNICATE_TASK_H_
#define _COMMUNICATE_TASK_H_

#include <stdbool.h>
#include <stdint.h>

int communicate_task_init(void);

/*============================================================================*
 *                              Direct Send API
 *  Each function builds the packet and sends it via BLE notify directly.
 *  Returns true on success, false if disconnected or send failed.
 *============================================================================*/

bool commu_send_bond_success(void);
bool commu_send_bond_fail(void);
bool commu_send_login_success(void);
bool commu_send_login_fail(void);

/* Settings */
bool commu_send_alarm_settings(void);
bool commu_send_hour_format(void);
bool commu_send_distance_unit(void);
bool commu_send_oled_display_time(uint8_t time);
bool commu_send_language(void);
bool commu_send_dial_change(void);

/* Health */
bool commu_send_sport_data(void);
bool commu_send_heart_data(int hr);
bool commu_send_heart_rate_series(const float *ppg, uint16_t count);

/* Control */
bool commu_send_phone_control_cmd(void);
bool commu_send_find_mobile(void);
bool commu_send_media_control(void);
bool commu_send_volume_percentage(uint8_t volume);

/* Notification */
bool commu_send_charge_status(void);
bool commu_send_weather_request(void);
bool commu_send_calendar_request(void);
bool commu_send_gesture_detect(uint8_t label);
bool commu_send_remote_input(const char *json);
bool commu_send_dismiss_notification(const char *id);
bool commu_send_user_speaking_state(uint8_t status);
bool commu_send_chat_with_ai(const char *json);
bool commu_send_battery_level(uint8_t level);
bool commu_send_update_instruction(const char *json);
bool commu_send_get_instruction_img(const char *id);

/* Sensor */
bool commu_send_linear_acce_buffer(const uint8_t *acce, uint16_t length);

/* File sync */
bool commu_send_start_sync_file(uint32_t total_size);
bool commu_send_sync_file(const uint8_t *chunk, uint16_t length);
bool commu_send_end_sync_file(void);
bool commu_send_file_compare_result(uint8_t result);

/* Other */
bool commu_send_bluetooth_log(const char *log);
bool commu_send_watch_system_sync(void);
bool commu_send_ota_status(uint8_t status);
bool commu_send_device_info(void);

#endif

