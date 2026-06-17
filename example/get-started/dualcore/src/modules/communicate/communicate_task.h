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

/*============================================================================*
 *                              Direct Send API
 *  Each function builds the packet and sends it via BLE notify directly.
 *  Returns true on success, false if disconnected or send failed.
 *============================================================================*/

/* ADR-0008 E7: watch→phone uplink — the watch's active-target device selection
   (sent on tileview page change). JSON {"device_id":"..."} over 0x20/0x05. */
bool commu_send_active_device(const char *device_id);

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
/* Background daily HR-curve point: a timestamped ambient HR sample (taken
   ~every 15 min by the LCPU sampler). Phone accumulates these into a daily
   heart-rate curve. Distinct from commu_send_heart_data (live single value). */
bool commu_send_heart_curve_sample(uint32_t timestamp, uint8_t bpm);
/* Why an HR point was NOT recorded for a 5-min bucket (timestamp + reason code).
   Mirrors commu_send_heart_curve_sample; phone draws it as a coloured gap on
   the HR curve. Reason codes frozen in ADR-0011 D2. */
bool commu_send_heart_curve_skip(uint32_t timestamp, uint8_t reason);
/* Wear-detect diagnostic record (KEY_WEAR_DIAG 0x12, 14B LE). Cable-less units
   have no serial console; the phone appends these to a daily CSV for offline
   night-time threshold analysis. evt codes 1..8 frozen (watch_sys_service.h). */
bool commu_send_wear_diag(uint32_t ts, uint8_t evt, uint8_t status,
                          uint16_t dc_q4, uint16_t pi_e6,
                          uint16_t pi_range_e6, uint16_t imu_var_e4);
bool commu_send_heart_rate_series(const float *ppg, uint16_t count);
/* Push current SkaiWatchSys.sleep_state to the phone via KEY_RETURN_SLEEP_DATA.
   Called on every stage transition received from LCPU. The dart-side
   parser must mirror the layout of watch_sys_sleep_state_t. */
bool commu_send_sleep_data(void);

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
/* Raw battery voltage in millivolts, sent as uint16 big-endian alongside the
 * standard BAS level notify so the phone-side dev tooling can log the analog
 * value at receive time. */
bool commu_send_battery_voltage(uint16_t millivolts);
bool commu_send_update_instruction(const char *json);
bool commu_send_get_instruction_img(const char *id);
bool commu_send_skaibar_selected(uint8_t idx);
bool commu_send_skaibar_committed(uint8_t idx);
/* device-list option uplink, SKAI_LINK group, by 0-based index */
bool commu_send_option_commit(uint8_t idx);   /* option TAPPED   -> KEY_ACTION_SELECT (0x06) */
bool commu_send_option_focus(uint8_t idx);    /* option SCROLLED -> KEY_ACTION_FOCUS  (0x07) */

/* device-page trackpad relay uplink, SKAI_LINK group. Only the right-side device
   page's hosted mouse routes here (the standalone APP_ID_MOUSE app keeps BLE HID);
   the phone actuates these on the active target device. */
bool commu_send_mouse_move(int dx, int dy);            /* -> KEY_MOUSE_MOVE   (0x08) */
bool commu_send_mouse_button(uint8_t btn, uint8_t act);/* -> KEY_MOUSE_BUTTON (0x09) btn 0=L 1=R; act 0=up 1=down 2=click */
bool commu_send_mouse_scroll(int dx, int dy);          /* -> KEY_MOUSE_SCROLL (0x0A) wheel=dy pan=dx */
bool commu_send_mouse_back(void);                      /* -> KEY_MOUSE_BACK   (0x0B) */
bool commu_send_skaibar_dismiss(void);                 /* -> KEY_SKAIBAR_DISMISS (0x0C) cancel-close, no commit */
bool commu_send_skaibar_view(char cat);                /* -> KEY_SKAIBAR_VIEW_CHANGE (0x0D) view opened: '@'/'/'/0=bar */
bool commu_send_skaibar_open_device(void);             /* -> KEY_SKAIBAR_OPEN_DEVICE (0x0E) mouse app: open single controlled device's skaibar */

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
bool commu_send_model_versions(void);

#endif

