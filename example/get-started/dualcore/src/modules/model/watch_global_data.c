/**
 ******************************************************************************
 * @file   watch_global_data.c
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

#include "board.h"
#include "watch_global_data.h"
#include "communicate_protocol.h"
#include "gesture_recognition_task.h"
#include "ui_handler.h"

#define DBG_TAG "watch.global.data"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
/******************************************************************************
 *                              User setting and flag parameters
 ******************************************************************************/
#define USING_STORAGE_API_LOCK 0

#ifdef BSP_USING_PC_SIMULATOR
SkaiWatchSysType_t SkaiWatchSys;
#else
SkaiWatchSysType_t SkaiWatchSys __attribute__((aligned(4)));
#endif

uint8_t gsensorSamplesBuffer[BLE_G_SENSOR_BUF_SIZE] = {0};

#ifdef BSP_SHARE_PREFS

WatchPrefs_t WatchPrefs;
static uint16_t usage_count = 0;

#if USING_STORAGE_API_LOCK
static rt_mutex_t storage_api_lock;
#endif

void watch_storage_api_lock(void)
{
#if USING_STORAGE_API_LOCK
  LOG_D("watch_storage_api_lock");
  rt_mutex_take(storage_api_lock, RT_WAITING_FOREVER);
#endif
}

void watch_storage_api_unlock(void)
{
#if USING_STORAGE_API_LOCK
  rt_thread_mdelay(50);
  rt_mutex_release(storage_api_lock);
  LOG_D("watch_storage_api_unlock");
#endif
}

static share_prefs_t *open_watch_prefs()
{
  if (is_ble_dfu_thread_running())
  {
    return NULL;
  }
  watch_storage_api_lock();
  if (WatchPrefs.pref != NULL)
  {
    return WatchPrefs.pref;
  }
  WatchPrefs.pref = share_prefs_open("watch", SHAREPREFS_MODE_PRIVATE);
  if (WatchPrefs.pref == NULL)
  {
    LOG_E("open watch prefs failed");
    watch_storage_api_unlock();
    return NULL;
  }
  usage_count++;
  LOG_D("open watch prefs success (%d)", usage_count);
  return WatchPrefs.pref;
}

static void close_watch_prefs(share_prefs_t *pref)
{
  rt_err_t res = RT_EOK;
  res = share_prefs_close(pref);
  if (res == RT_EOK)
  {
    WatchPrefs.pref = NULL;
    usage_count--;
    LOG_D("close watch prefs success (%d)", usage_count);
  }
  else
  {
    LOG_E("close watch prefs failed");
  }
  watch_storage_api_unlock();
}
/// Preference Table
// This table lists the fields that need to be stored in flash according to the comments in your code. The data type of each field is also listed.
// ===============================================
// Field Name	              | Data Type
// ------------------------|----------------
// flag_field	              | T_FLAG_FIELD
static void read_flag_field(share_prefs_t *pref);
static void write_flag_field(share_prefs_t *pref);
// msg_switch	              | T_MSG_SWITCH
static void read_msg_switch(share_prefs_t *pref);
static void write_msg_switch(share_prefs_t *pref);
// phone_os_version	        | uint8_t
static void read_phone_os_version(share_prefs_t *pref);
static void write_phone_os_version(share_prefs_t *pref);
// alarm_num	              | uint8_t
// alarms	                  | T_ALARM[MAX_ALARM_NUM]
static void read_alarm();
static void write_alarm();
// oled_display_time	      | uint8_t
static void read_oled_display_time(share_prefs_t *pref);
static void write_oled_display_time(share_prefs_t *pref);
// language	                | uint8_t
static void read_language(share_prefs_t *pref);
static void write_language(share_prefs_t *pref);
// clock_screen_num	        | uint8_t
static void read_clock_screen_num(share_prefs_t *pref);
static void write_clock_screen_num(share_prefs_t *pref);
// backlight_percent	      | uint8_t
static void read_brightness_percent(share_prefs_t *pref);
static void write_brightness_percent(share_prefs_t *pref);
// gPedoData	              | T_PEDO_DATA
static void read_pedo_data(share_prefs_t *pref);
static void write_pedo_data(share_prefs_t *pref);
// DNDMode	                | T_DND_MODE
static void read_dnd_mode(share_prefs_t *pref);
static void write_dnd_mode(share_prefs_t *pref);
// user_data               	| T_USER_DATA
static void read_user_data(share_prefs_t *pref);
static void write_user_data(share_prefs_t *pref);
// sit_alert_data	          | T_SIT_ALERT
static void read_sit_alert(share_prefs_t *pref);
static void write_sit_alert(share_prefs_t *pref);
// Global_Time	            | T_UTC_TIME
static void read_global_time(share_prefs_t *pref);
static void write_global_time(share_prefs_t *pref);
// sleep_data_show	        | T_SLEEP_DATA
static void read_sleep_data(share_prefs_t *pref);
static void write_sleep_data(share_prefs_t *pref);
// clock_status	            | T_CLOCK_MENU_TYPE
static void read_clock_status(share_prefs_t *pref);
static void write_clock_status(share_prefs_t *pref);
// gesture_threshold            | int
static void read_gesture_threshold(share_prefs_t *pref);
static void write_gesture_threshold(share_prefs_t *pref);
// ===============================================
static void read_flag_field(share_prefs_t *pref)
{
  share_prefs_get_block(pref, "flag_field", (void *)(&SkaiWatchSys.flag_field), sizeof(T_FLAG_FIELD));
}

static void write_flag_field(share_prefs_t *pref)
{
  share_prefs_set_block(pref, "flag_field", (void *)&SkaiWatchSys.flag_field, sizeof(T_FLAG_FIELD));
}

static void read_msg_switch(share_prefs_t *pref)
{
  share_prefs_get_block(pref, "msg_switch", (void *)&SkaiWatchSys.msg_switch, sizeof(T_MSG_SWITCH));
}

static void write_msg_switch(share_prefs_t *pref)
{
  share_prefs_set_block(pref, "msg_switch", (void *)&SkaiWatchSys.msg_switch, sizeof(T_MSG_SWITCH));
}

static void read_phone_os_version(share_prefs_t *pref)
{
  int32_t version = share_prefs_get_int(pref, "phone_os_version", -1);
  if (version > 0)
  {
    SkaiWatchSys.phone_os_version = version;
  }
}

static void write_phone_os_version(share_prefs_t *pref)
{
  int32_t version = SkaiWatchSys.phone_os_version;
  share_prefs_set_int(pref, "phone_os_version", version);
}

static void read_alarm()
{
  rt_err_t res = RT_EOK;
  share_prefs_t *pref = share_prefs_open("alarm", SHAREPREFS_MODE_PRIVATE);
  int32_t list_len;
  /* Read alarm list*/
  list_len = share_prefs_get_int(pref, "list_len", -1);
  SkaiWatchSys.alarm_num = list_len;
  if (list_len > 0)
  {
    res = share_prefs_get_block(pref, "list", (void *)&SkaiWatchSys.alarms, list_len * sizeof(T_ALARM));
  }
  res = share_prefs_close(pref);
}

static void write_alarm()
{
  rt_err_t res = RT_EOK;
  share_prefs_t *pref = share_prefs_open("alarm", SHAREPREFS_MODE_PRIVATE);
  int32_t list_len = SkaiWatchSys.alarm_num;
  /* Write alarm list*/
  res = share_prefs_set_int(pref, "list_len", list_len);
  if (list_len > 0)
  {
    res = share_prefs_set_block(pref, "list", (void *)&SkaiWatchSys.alarms, list_len * sizeof(T_ALARM));
  }
  res = share_prefs_close(pref);
}

static void read_oled_display_time(share_prefs_t *pref)
{
  int32_t oled_display_time = share_prefs_get_int(pref, "oled_display_time", -1);
  if (oled_display_time > 0)
  {
    SkaiWatchSys.oled_display_time = oled_display_time;
  }
  else
  {
    SkaiWatchSys.oled_display_time = 5;
  }
}

static void write_oled_display_time(share_prefs_t *pref)
{
  int32_t oled_display_time = SkaiWatchSys.oled_display_time;
  share_prefs_set_int(pref, "oled_display_time", oled_display_time);
}

static void read_language(share_prefs_t *pref)
{
  int32_t language = share_prefs_get_int(pref, "language", -1);
  if (language >= 0)
  {
    SkaiWatchSys.language = language;
  }
}

static void write_language(share_prefs_t *pref)
{
  int32_t language = SkaiWatchSys.language;
  share_prefs_set_int(pref, "language", language);
}

static void read_clock_screen_num(share_prefs_t *pref)
{
  int32_t clock_screen_num = share_prefs_get_int(pref, "clock_screen_num", -1);
  if (clock_screen_num > 0)
  {
    SkaiWatchSys.clock_screen_num = clock_screen_num;
  }
}

static void write_clock_screen_num(share_prefs_t *pref)
{
  int32_t clock_screen_num = SkaiWatchSys.clock_screen_num;
  share_prefs_set_int(pref, "clock_screen_num", clock_screen_num);
}

static void write_clock_widget_num(share_prefs_t *pref)
{
  int32_t clock_widget_num = SkaiWatchSys.clock_widget_num;
  LOG_D("Write clock widget num: %d", clock_widget_num);
  share_prefs_set_int(pref, "clock_widget_num", clock_widget_num);
}

static void read_clock_widget_num(share_prefs_t *pref)
{
  int32_t clock_widget_num = share_prefs_get_int(pref, "clock_widget_num", -1);
  LOG_D("Read clock widget num: %d", clock_widget_num);
  if (clock_widget_num > 0)
  {
    SkaiWatchSys.clock_widget_num = clock_widget_num;
  }
  else
  {
    SkaiWatchSys.clock_widget_num = app_id_media;
  }
}

static void read_watch_restart_num(share_prefs_t *pref)
{
  int32_t watch_restart_num = share_prefs_get_int(pref, "watch_restart_num", -1);
  if ((watch_restart_num + 1) >= 9999)
  {
    SkaiWatchSys.watch_restart_num = 0;
  }
  else
  {
    SkaiWatchSys.watch_restart_num = watch_restart_num + 1;
  }
  LOG_D("Watch restart num: %d", SkaiWatchSys.watch_restart_num);
  share_prefs_set_int(pref, "watch_restart_num", watch_restart_num + 1);
}

static void reset_watch_restart_num(share_prefs_t *pref)
{
  SkaiWatchSys.watch_restart_num = 0;
  int32_t watch_restart_num = SkaiWatchSys.watch_restart_num;
  share_prefs_set_int(pref, "watch_restart_num", watch_restart_num);
}

static void read_brightness_percent(share_prefs_t *pref)
{
  int32_t brightness_percent = share_prefs_get_int(pref, "brightness", -1);
  LOG_D("Loaded brightness percent: %d", brightness_percent);
  if (brightness_percent >= 10 && brightness_percent <= 100)
  {
    SkaiWatchSys.brightness = brightness_percent;
  }
  else
  {
    SkaiWatchSys.brightness = 100;
  }
}

static void write_brightness_percent(share_prefs_t *pref)
{
  int32_t brightness_percent = SkaiWatchSys.brightness;
  share_prefs_set_int(pref, "brightness", brightness_percent);
}

static void read_pedo_data(share_prefs_t *pref)
{
  share_prefs_get_block(pref, "pedo_data", (void *)&SkaiWatchSys.gPedoData, sizeof(T_PEDO_DATA));
}

static void write_pedo_data(share_prefs_t *pref)
{
  share_prefs_set_block(pref, "pedo_data", (void *)&SkaiWatchSys.gPedoData, sizeof(T_PEDO_DATA));
}

static void read_dnd_mode(share_prefs_t *pref)
{
  share_prefs_get_block(pref, "dnd_mode", (void *)&SkaiWatchSys.DNDMode, sizeof(T_DND_MODE));
}

static void write_dnd_mode(share_prefs_t *pref)
{
  share_prefs_set_block(pref, "dnd_mode", (void *)&SkaiWatchSys.DNDMode, sizeof(T_DND_MODE));
}

static void read_user_data(share_prefs_t *pref)
{
  share_prefs_get_block(pref, "user_data", (void *)&SkaiWatchSys.user_data, sizeof(T_USER_DATA));
}

static void write_user_data(share_prefs_t *pref)
{
  share_prefs_set_block(pref, "user_data", (void *)&SkaiWatchSys.user_data, sizeof(T_USER_DATA));
}

static void read_sit_alert(share_prefs_t *pref)
{
  share_prefs_get_block(pref, "sit_alert", (void *)&SkaiWatchSys.sit_alert_data, sizeof(T_SIT_ALERT));
}

static void write_sit_alert(share_prefs_t *pref)
{
  share_prefs_set_block(pref, "sit_alert", (void *)&SkaiWatchSys.sit_alert_data, sizeof(T_SIT_ALERT));
}

static void read_global_time(share_prefs_t *pref)
{
  share_prefs_get_block(pref, "global_time", (void *)&SkaiWatchSys.Global_Time, sizeof(T_UTC_TIME));
}

static void write_global_time(share_prefs_t *pref)
{
  share_prefs_set_block(pref, "global_time", (void *)&SkaiWatchSys.Global_Time, sizeof(T_UTC_TIME));
}

static void read_sleep_data(share_prefs_t *pref)
{
  share_prefs_get_block(pref, "sleep_data_show", (void *)&SkaiWatchSys.sleep_data_show, sizeof(T_SLEEP_DATA));
}

static void write_sleep_data(share_prefs_t *pref)
{
  share_prefs_set_block(pref, "sleep_data_show", (void *)&SkaiWatchSys.sleep_data_show, sizeof(T_SLEEP_DATA));
}

static void read_clock_status(share_prefs_t *pref)
{
  int32_t clock_status = share_prefs_get_int(pref, "clock_status", -1);
  if (clock_status >= 0)
  {
    SkaiWatchSys.clock_status = clock_status;
  }
}

static void write_clock_status(share_prefs_t *pref)
{
  int32_t clock_status = SkaiWatchSys.clock_status;
  share_prefs_set_int(pref, "clock_status", clock_status);
}
static void read_gesture_threshold(share_prefs_t *pref)
{
  int32_t threshold = share_prefs_get_int(pref, "gesture_threshold", -1);
  if (threshold >= 50 && threshold <= 100)
  {
    set_gesture_recognition_threshold(threshold);
    LOG_I("Loaded gesture threshold: %d", threshold);
  }
  else
  {
    set_gesture_recognition_threshold(DEFAULT_GESTURE_THRESHOLD);
    LOG_I("Using default gesture threshold: %d", DEFAULT_GESTURE_THRESHOLD);
  }
}
static void write_gesture_threshold(share_prefs_t *pref)
{
  int32_t threshold = get_gesture_recognition_threshold();
  share_prefs_set_int(pref, "gesture_threshold", threshold);
  LOG_I("Saved gesture threshold: %d", threshold);
}

static int watch_prefs_register(void)
{
  WatchPrefs.pref = NULL;
#if USING_STORAGE_API_LOCK
  storage_api_lock = rt_mutex_create("storage_api_lock", RT_IPC_FLAG_FIFO);
  RT_ASSERT(storage_api_lock != NULL);
#endif
  WatchPrefs.read_flag_field = read_flag_field;
  WatchPrefs.write_flag_field = write_flag_field;
  WatchPrefs.read_msg_switch = read_msg_switch;
  WatchPrefs.write_msg_switch = write_msg_switch;
  WatchPrefs.read_phone_os_version = read_phone_os_version;
  WatchPrefs.write_phone_os_version = write_phone_os_version;
  WatchPrefs.read_oled_display_time = read_oled_display_time;
  WatchPrefs.write_oled_display_time = write_oled_display_time;
  WatchPrefs.read_language = read_language;
  WatchPrefs.write_language = write_language;
  WatchPrefs.read_clock_screen_num = read_clock_screen_num;
  WatchPrefs.write_clock_screen_num = write_clock_screen_num;
  WatchPrefs.read_brightness = read_brightness_percent;
  WatchPrefs.write_brightness = write_brightness_percent;
  WatchPrefs.read_restart_num = read_watch_restart_num;
  WatchPrefs.reset_restart_num = reset_watch_restart_num;
  WatchPrefs.read_clock_widget_num = read_clock_widget_num;
  WatchPrefs.write_clock_widget_num = write_clock_widget_num;
  WatchPrefs.read_pedo_data = read_pedo_data;
  WatchPrefs.write_pedo_data = write_pedo_data;
  WatchPrefs.read_dnd_mode = read_dnd_mode;
  WatchPrefs.write_dnd_mode = write_dnd_mode;
  WatchPrefs.read_user_data = read_user_data;
  WatchPrefs.write_user_data = write_user_data;
  WatchPrefs.read_sit_alert = read_sit_alert;
  WatchPrefs.write_sit_alert = write_sit_alert;
  WatchPrefs.read_global_time = read_global_time;
  WatchPrefs.write_global_time = write_global_time;
  WatchPrefs.read_sleep_data = read_sleep_data;
  WatchPrefs.write_sleep_data = write_sleep_data;
  WatchPrefs.read_clock_status = read_clock_status;
  WatchPrefs.write_clock_status = write_clock_status;
  WatchPrefs.read_gesture_threshold = read_gesture_threshold;
  WatchPrefs.write_gesture_threshold = write_gesture_threshold;
  return 0;
}
INIT_APP_EXPORT(watch_prefs_register);

void watch_config_struct_flash_read(void)
{
  share_prefs_t *pref = open_watch_prefs();
  if (pref == NULL)
  {
    LOG_E("%s: open watch prefs failed", __func__);
    return;
  }
  LOG_D("read watch prefs");
  WatchPrefs.read_flag_field(pref);
  WatchPrefs.read_msg_switch(pref);
  WatchPrefs.read_phone_os_version(pref);
  // WatchPrefs.read_alarm();
  WatchPrefs.read_oled_display_time(pref);
  WatchPrefs.read_language(pref);
  WatchPrefs.read_clock_screen_num(pref);
  WatchPrefs.read_brightness(pref);
  WatchPrefs.read_restart_num(pref);
  WatchPrefs.read_clock_widget_num(pref);
  WatchPrefs.read_pedo_data(pref);
  WatchPrefs.read_dnd_mode(pref);
  WatchPrefs.read_user_data(pref);
  WatchPrefs.read_sit_alert(pref);
  WatchPrefs.read_global_time(pref);
  WatchPrefs.read_sleep_data(pref);
  WatchPrefs.read_clock_status(pref);
  WatchPrefs.read_gesture_threshold(pref);
  close_watch_prefs(pref);
}

void reset_watch_restart_number(void)
{
  share_prefs_t *pref = open_watch_prefs();
  if (pref == NULL)
  {
    LOG_E("%s: open watch prefs failed", __func__);
    return;
  }
  LOG_D("reset watch restart num");
  WatchPrefs.reset_restart_num(pref);
  close_watch_prefs(pref);
}

void write_clock_widget_number(void)
{
  share_prefs_t *pref = open_watch_prefs();
  if (pref == NULL)
  {
    LOG_E("%s: open watch prefs failed", __func__);
    return;
  }
  LOG_D("write clock widget num");
  WatchPrefs.write_clock_widget_num(pref);
  close_watch_prefs(pref);
}

void watch_config_struct_flash_write(void)
{
  share_prefs_t *pref = open_watch_prefs();
  if (pref == NULL)
  {
    LOG_E("%s: open watch prefs failed", __func__);
    return;
  }
  LOG_D("write watch prefs");
  WatchPrefs.write_flag_field(pref);
  WatchPrefs.write_msg_switch(pref);
  WatchPrefs.write_phone_os_version(pref);
  // WatchPrefs.write_alarm();
  WatchPrefs.write_oled_display_time(pref);
  WatchPrefs.write_language(pref);
  WatchPrefs.write_clock_screen_num(pref);
  WatchPrefs.write_brightness(pref);
  WatchPrefs.write_pedo_data(pref);
  WatchPrefs.write_dnd_mode(pref);
  WatchPrefs.write_user_data(pref);
  WatchPrefs.write_sit_alert(pref);
  WatchPrefs.write_global_time(pref);
  WatchPrefs.write_sleep_data(pref);
  WatchPrefs.write_clock_status(pref);
  WatchPrefs.write_gesture_threshold(pref);
  close_watch_prefs(pref);
}

void store_watch_prefs(watch_prefs_key key)
{
  share_prefs_t *pref = open_watch_prefs();
  if (pref == NULL)
  {
    LOG_E("%s: open watch prefs failed", __func__);
    return;
  }
  LOG_D("store watch prefs %d", key);
  switch (key)
  {
  case WATCH_PREFS_KEY_FLAG_FIELD:
    WatchPrefs.write_flag_field(pref);
    break;
  case WATCH_PREFS_KEY_MSG_SWITCH:
    WatchPrefs.write_msg_switch(pref);
    break;
  case WATCH_PREFS_KEY_PHONE_OS_VERSION:
    WatchPrefs.write_phone_os_version(pref);
    break;
  case WATCH_PREFS_KEY_OLED_DISPLAY_TIME:
    WatchPrefs.write_oled_display_time(pref);
    break;
  case WATCH_PREFS_KEY_LANGUAGE:
    WatchPrefs.write_language(pref);
    break;
  case WATCH_PREFS_KEY_CLOCK_SCREEN_NUM:
    WatchPrefs.write_clock_screen_num(pref);
    break;
  case WATCH_PREFS_KEY_BACKLIGHT_PERCENT:
    WatchPrefs.write_brightness(pref);
    break;
  case WATCH_PREFS_KEY_PEDODATA:
    WatchPrefs.write_pedo_data(pref);
    break;
  case WATCH_PREFS_KEY_DNDMODE:
    WatchPrefs.write_dnd_mode(pref);
    break;
  case WATCH_PREFS_KEY_USER_DATA:
    WatchPrefs.write_user_data(pref);
    break;
  case WATCH_PREFS_KEY_SIT_ALERT_DATA:
    WatchPrefs.write_sit_alert(pref);
    break;
  case WATCH_PREFS_KEY_GLOBAL_TIME:
    WatchPrefs.write_global_time(pref);
    break;
  case WATCH_PREFS_KEY_SLEEP_DATA_SHOW:
    WatchPrefs.write_sleep_data(pref);
    break;
  case WATCH_PREFS_KEY_CLOCK_STATUS:
    WatchPrefs.write_clock_status(pref);
    break;
  case WATCH_PREFS_KEY_GESTURE_THRESHOLD:
    WatchPrefs.write_gesture_threshold(pref);
    break;
  default:
    break;
  }
  close_watch_prefs(pref);
}

static int utest_watch_shared_preferences(int argc, char *argv[])
{
  if (argc >= 2)
  {
    if (strcmp(argv[1], "-read") == 0)
    {
      if (argc == 3)
      {
        share_prefs_t *pref = open_watch_prefs();
        if (pref == NULL)
        {
          LOG_E("open watch prefs failed");
          return -1;
        }
        if (strcmp(argv[2], "flag_field") == 0)
        {
          read_flag_field(pref);
        }
        else if (strcmp(argv[2], "msg_switch") == 0)
        {
          read_msg_switch(pref);
        }
        else if (strcmp(argv[2], "phone_os_version") == 0)
        {
          read_phone_os_version(pref);
        }
        else if (strcmp(argv[2], "oled_display_time") == 0)
        {
          read_oled_display_time(pref);
        }
        else if (strcmp(argv[2], "language") == 0)
        {
          read_language(pref);
        }
        else if (strcmp(argv[2], "clock_screen_num") == 0)
        {
          read_clock_screen_num(pref);
        }
        else if (strcmp(argv[2], "brightness") == 0)
        {
          read_brightness_percent(pref);
        }
        else if (strcmp(argv[2], "pedo_data") == 0)
        {
          read_pedo_data(pref);
        }
        else if (strcmp(argv[2], "dnd_mode") == 0)
        {
          read_dnd_mode(pref);
        }
        else if (strcmp(argv[2], "user_data") == 0)
        {
          read_user_data(pref);
        }
        else if (strcmp(argv[2], "sit_alert") == 0)
        {
          read_sit_alert(pref);
        }
        else if (strcmp(argv[2], "global_time") == 0)
        {
          read_global_time(pref);
        }
        else if (strcmp(argv[2], "sleep_data") == 0)
        {
          read_sleep_data(pref);
        }
        else if (strcmp(argv[2], "clock_status") == 0)
        {
          read_clock_status(pref);
        }
        else
        {
          LOG_E("Invalid parameter");
        }
        close_watch_prefs(pref);
      }
    }
    else if (strcmp(argv[1], "-write") == 0)
    {
      if (argc == 3)
      {
        share_prefs_t *pref = open_watch_prefs();
        if (pref == NULL)
        {
          LOG_E("open watch prefs failed");
          return -1;
        }
        if (strcmp(argv[2], "flag_field") == 0)
        {
          write_flag_field(pref);
        }
        else if (strcmp(argv[2], "msg_switch") == 0)
        {
          write_msg_switch(pref);
        }
        else if (strcmp(argv[2], "phone_os_version") == 0)
        {
          write_phone_os_version(pref);
        }
        else if (strcmp(argv[2], "oled_display_time") == 0)
        {
          write_oled_display_time(pref);
        }
        else if (strcmp(argv[2], "language") == 0)
        {
          write_language(pref);
        }
        else if (strcmp(argv[2], "clock_screen_num") == 0)
        {
          write_clock_screen_num(pref);
        }
        else if (strcmp(argv[2], "brightness") == 0)
        {
          write_brightness_percent(pref);
        }
        else if (strcmp(argv[2], "pedo_data") == 0)
        {
          write_pedo_data(pref);
        }
        else if (strcmp(argv[2], "dnd_mode") == 0)
        {
          write_dnd_mode(pref);
        }
        else if (strcmp(argv[2], "user_data") == 0)
        {
          write_user_data(pref);
        }
        else if (strcmp(argv[2], "sit_alert") == 0)
        {
          write_sit_alert(pref);
        }
        else if (strcmp(argv[2], "global_time") == 0)
        {
          write_global_time(pref);
        }
        else if (strcmp(argv[2], "sleep_data") == 0)
        {
          write_sleep_data(pref);
        }
        else if (strcmp(argv[2], "clock_status") == 0)
        {
          write_clock_status(pref);
        }
        else if (strcmp(argv[2], "clock_widget") == 0)
        {
          SkaiWatchSys.clock_widget_num = app_id_media;
          write_clock_widget_num(pref);
        }
        else
        {
          LOG_E("Invalid parameter");
        }
        close_watch_prefs(pref);
      }
    }
  }
  return 0;
}
MSH_CMD_EXPORT(utest_watch_shared_preferences, "utest_watch_shared_preferences [OPTION] ...");
#endif

// Task for shared prefernece

uint8_t reboot_reason;
void show_SkaiWatchSys_info(void)
{
  // uint32_t size;

  // size = sizeof(SkaiWatchSysType_t);
  // DBG_DIRECT("size of SkaiWatchSysType_t:%d", size);

  // size = sizeof(SkaiWatchSys);
  // DBG_DIRECT("size of SkaiWatchSys:%d", size);

  // size = sizeof(SkaiWatchSys.flag_field);
  // DBG_DIRECT("size of flag_field: %d", size);

  // size = sizeof(SkaiWatchSys.msg_switch);
  // DBG_DIRECT("size of msg_switch: %d", size);

  // size = sizeof(SkaiWatchSys.battery_level_value);
  // DBG_DIRECT("size of battery_level_value: %d", size);

  // size = sizeof(SkaiWatchSys.hrs_detect_period);
  // DBG_DIRECT("size of hrs_detect_period: %d", size);

  // size = sizeof(SkaiWatchSys.hrs_start_up_mode);
  // DBG_DIRECT("size of hrs_start_up_mode: %d", size);

  // size = sizeof(SkaiWatchSys.phone_os_version);
  // DBG_DIRECT("size of phone_os_version: %d", size);

  // size = sizeof(SkaiWatchSys.alarm_num);
  // DBG_DIRECT("size of alarm_num: %d", size);

  // size = sizeof(SkaiWatchSys.oled_display_time);
  // DBG_DIRECT("size of oled_display_time: %d", size);

  // size = sizeof(SkaiWatchSys.language);
  // DBG_DIRECT("size of language: %d", size);

  // size = sizeof(SkaiWatchSys.clock_screen_num);
  // DBG_DIRECT("size of clock_screen_num: %d", size);

  // size = sizeof(SkaiWatchSys.weather_sync_hour);
  // DBG_DIRECT("size of weather_sync_hour: %d", size);

  // size = sizeof(SkaiWatchSys.weather_moment_count);
  // DBG_DIRECT("size of weather_moment_count: %d", size);

  // size = sizeof(SkaiWatchSys.weather_day_count);
  // DBG_DIRECT("size of weather_day_count: %d", size);

  // size = sizeof(SkaiWatchSys.backlight_percent);
  // DBG_DIRECT("size of backlight_percent: %d", size);

  // size = sizeof(SkaiWatchSys.battery_vol_value);
  // DBG_DIRECT("size of battery_vol_value: %d", size);

  // size = sizeof(SkaiWatchSys.current_stationary_time);
  // DBG_DIRECT("size of current_stationary_time: %d", size);

  // size = sizeof(SkaiWatchSys.SecondCountRTC);
  // DBG_DIRECT("size of SecondCountRTC: %d", size);

  // size = sizeof(SkaiWatchSys.pre_rtc_tick_count);
  // DBG_DIRECT("size of pre_rtc_tick_count: %d", size);

  // size = sizeof(SkaiWatchSys.weather_sync_secondcount);
  // DBG_DIRECT("size of weather_sync_secondcount: %d", size);

  // size = sizeof(SkaiWatchSys.wristband_sleep_status);
  // DBG_DIRECT("size of wristband_sleep_status: %d", size);

  // size = sizeof(SkaiWatchSys.charger_status);
  // DBG_DIRECT("size of charger_status: %d", size);

  // size = sizeof(SkaiWatchSys.sport_address);
  // DBG_DIRECT("size of sport_address: %d", size);

  // size = sizeof(SkaiWatchSys.sleep_address);
  // DBG_DIRECT("size of sleep_address: %d", size);

  // size = sizeof(SkaiWatchSys.heart_address);
  // DBG_DIRECT("size of heart_address: %d", size);

  // size = sizeof(SkaiWatchSys.gps_address);
  // DBG_DIRECT("size of gps_address: %d", size);

  // size = sizeof(SkaiWatchSys.weather_location_address);
  // DBG_DIRECT("size of weather_location_address: %d", size);

  // size = sizeof(SkaiWatchSys.weather_current_address);
  // DBG_DIRECT("size of weather_current_address: %d", size);

  // size = sizeof(SkaiWatchSys.weather_future_hour_address);
  // DBG_DIRECT("size of weather_future_hour_address: %d", size);

  // size = sizeof(SkaiWatchSys.weather_future_day_address);
  // DBG_DIRECT("size of weather_future_day_address: %d", size);

  // size = sizeof(SkaiWatchSys.gPedoData);
  // DBG_DIRECT("size of gPedoData: %d", size);

  // size = sizeof(SkaiWatchSys.DNDMode);
  // DBG_DIRECT("size of DNDMode: %d", size);

  // size = sizeof(SkaiWatchSys.alarms);
  // DBG_DIRECT("size of alarms: %d", size);

  // size = sizeof(SkaiWatchSys.user_data);
  // DBG_DIRECT("size of user_data: %d", size);

  // size = sizeof(SkaiWatchSys.sit_alert_data);
  // DBG_DIRECT("size of sit_alert_data: %d", size);

  // size = sizeof(SkaiWatchSys.Global_Time);
  // DBG_DIRECT("size of Global_Time: %d", size);

  // size = sizeof(SkaiWatchSys.msg_data_config);
  // DBG_DIRECT("size of msg_data_config: %d", size);

  // size = sizeof(SkaiWatchSys.sleep_data_show);
  // DBG_DIRECT("size of sleep_data_show: %d", size);

  // size = sizeof(SkaiWatchSys.bbpro_hci_link_status);
  // DBG_DIRECT("size of bbpro_hci_link_status: %d", size);

  // size = sizeof(SkaiWatchSys.bbpro_device_status);
  // DBG_DIRECT("size of bbpro_device_status: %d", size);

  // size = sizeof(SkaiWatchSys.paired_info);
  // DBG_DIRECT("size of paired_info: %d", size);

  // size = sizeof(SkaiWatchSys.gap_dev_state);
  // DBG_DIRECT("size of gap_dev_state: %d", size);

  // size = sizeof(SkaiWatchSys.gap_conn_state);
  // DBG_DIRECT("size of gap_conn_state: %d", size);

  // size = sizeof(SkaiWatchSys.clock_status);
  // DBG_DIRECT("size of clock_status: %d", size);

  // size = sizeof(SkaiWatchSys.watch_conn_id);
  // DBG_DIRECT("size of watch_conn_id: %d", size);

  // size = sizeof(SkaiWatchSys.conn_interval);
  // DBG_DIRECT("size of conn_interval:%d", size);

  // size = sizeof(SkaiWatchSys.conn_latency);
  // DBG_DIRECT("size of conn_latency:%d", size);

  // size = sizeof(SkaiWatchSys.conn_superv_tout);
  // DBG_DIRECT("size of conn_superv_tout:%d", size);

  // size = sizeof(SkaiWatchSys.watch_mtu);
  // DBG_DIRECT("size of watch_mtu:%d", size);

  // size = sizeof(SkaiWatchSys.notification_number);
  // DBG_DIRECT("size of notification_number:%d", size);

  // size = sizeof(SkaiWatchSys.todolist_number);
  // DBG_DIRECT("size of todolist_number:%d", size);
}

void wristband_config_data_init(void)
{
  //   user_wdg_cb = (BOOL_WDG_CB)wristband_wdg_reboot_callback;
  //   // show_SkaiWatchSys_info();
  //   memset((uint8_t *)&SkaiWatchSys, 0x00, sizeof(SkaiWatchSys));
  //   // Set the OLED display time to 5 seconds
  //   SkaiWatchSys.oled_display_time = 5;
  //   // Set the backlight brightness to 20 percent
  //   SkaiWatchSys.backlight_percent = 20;
  //   // Set the clock status to the 6th menu option
  //   SkaiWatchSys.clock_status = CLOCK_6TH_MENU;
  //   // Enable the lift switch status
  //   SkaiWatchSys.flag_field.lift_switch_status = true;
  //   // Enable the twist switch status
  //   SkaiWatchSys.flag_field.twist_switch_status = true;

  // #if 1
  //   reboot_reason = wristband_get_reboot_reason();
  //   DBG_DIRECT("[WRISTBAND CONFIG INIT] reason = %d!", reboot_reason);
  //   // load_wristband_config();
  // #endif
}

void wristband_config_struct_flash_reset(void)
{
  // DBG_DIRECT("wristband_config_struct_flash_reset");
  // uint8_t prev_bp_lv = 0;
  // flash_sw_protect_unlock_by_addr_locked(WRISTBAND_CONFIG_START_ADDR, &prev_bp_lv);
  // flash_erase_locked(FLASH_ERASE_SECTOR, WRISTBAND_CONFIG_START_ADDR);
  // flash_set_block_protect_locked(prev_bp_lv);
}

void wristband_hw_reboot_handle(void)
{
  // /* reset config data when HW reboot reason*/
  // wristband_config_struct_flash_reset();
  // /* reset health data when hardware reset */
  // WristBandPedoDataBlockInit();
  // WristBandSleepDataBlockInit();
  // WristBandHeartDataBlockInit();
  // WristBandMSGDataBlockInit();
  // DBG_DIRECT("[WRISTBAND RESET => HW]!");
}

void wristband_sw_reboot_handle(void)
{
  // DBG_DIRECT("[WRISTBAND RESET => SW]!");
}

uint8_t wristband_get_reboot_reason(void)
{
  uint32_t reason = 0;
  // if (ftl_load(&reason, REBOOT_REASON_OFFSET, REBOOT_REASON_SIZE) == 0)
  // {
  //   if (reason == 0xF0 || reason == 0xAB)
  //   {
  //     DBG_DIRECT("soft reboot reason, reason value = %d", reason);
  //     wristband_sw_reboot_handle();
  //   }
  //   else if (reason == 0x01)
  //   {
  //     DBG_DIRECT("soft reboot reason, reason value = %d", reason);
  //     wristband_sw_reboot_handle();
  //   }
  //   else if (reason == 0x00)
  //   {
  //     DBG_DIRECT("HW reboot reason, reason value = %d", reason);
  //     wristband_hw_reboot_handle();
  //   }
  //   else if (reason == 0xF1)
  //   {
  //     DBG_DIRECT("UNBOND reboot reason, reason value = %d", reason);
  //     wristband_hw_reboot_handle();
  //   }
  //   else
  //   {
  //     DBG_DIRECT("Unknow reboot reason");
  //     wristband_hw_reboot_handle();
  //   }
  // }
  // else
  // {
  //   DBG_DIRECT("wristband load reboot Fail");
  //   wristband_hw_reboot_handle();
  //   reason = 0;
  // }
  // uint32_t original_reason = 0;
  // ftl_save(&original_reason, REBOOT_REASON_OFFSET, REBOOT_REASON_SIZE);
  return reason;
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/