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
#include "ble_device_manager.h"

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
// alarm_num + alarms       | uint8_t + T_ALARM[MAX_ALARM_NUM] (read_alarms / write_alarms below)
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

static void read_oled_display_time(share_prefs_t *pref)
{
  int32_t oled_display_time = share_prefs_get_int(pref, "oled_display_time", -1);
  LOG_D("Loaded OLED display time: %d", oled_display_time);
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
  LOG_D("Write OLED display time: %d", oled_display_time);
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
  if (threshold >= 1 && threshold <= 100)
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

static void read_alarms(share_prefs_t *pref)
{
  int32_t num = share_prefs_get_int(pref, "alarm_num", 0);
  if (num < 0)                  num = 0;
  if (num > MAX_ALARM_NUM)       num = MAX_ALARM_NUM;
  SkaiWatchSys.alarm_num = (uint8_t)num;
  if (num > 0)
  {
    share_prefs_get_block(pref, "alarms", (void *)&SkaiWatchSys.alarms,
                          sizeof(T_ALARM) * MAX_ALARM_NUM);
  }
  else
  {
    memset(&SkaiWatchSys.alarms, 0, sizeof(SkaiWatchSys.alarms));
  }
  LOG_I("Loaded %d alarms from prefs", num);
}

static void write_alarms(share_prefs_t *pref)
{
  share_prefs_set_int(pref, "alarm_num", SkaiWatchSys.alarm_num);
  share_prefs_set_block(pref, "alarms", (void *)&SkaiWatchSys.alarms,
                        sizeof(T_ALARM) * MAX_ALARM_NUM);
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
  WatchPrefs.read_global_time = read_global_time;
  WatchPrefs.write_global_time = write_global_time;
  WatchPrefs.read_sleep_data = read_sleep_data;
  WatchPrefs.write_sleep_data = write_sleep_data;
  WatchPrefs.read_clock_status = read_clock_status;
  WatchPrefs.write_clock_status = write_clock_status;
  WatchPrefs.read_gesture_threshold = read_gesture_threshold;
  WatchPrefs.write_gesture_threshold = write_gesture_threshold;
  WatchPrefs.read_alarms = read_alarms;
  WatchPrefs.write_alarms = write_alarms;
  return 0;
}
INIT_APP_EXPORT(watch_prefs_register);

void watch_prefs_save_alarms(void)
{
  share_prefs_t *pref = open_watch_prefs();
  if (pref == NULL) { LOG_E("watch_prefs_save_alarms: open failed"); return; }
  write_alarms(pref);
  close_watch_prefs(pref);
}

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
  WatchPrefs.read_oled_display_time(pref);
  WatchPrefs.read_language(pref);
  WatchPrefs.read_clock_screen_num(pref);
  WatchPrefs.read_brightness(pref);
  WatchPrefs.read_restart_num(pref);
  WatchPrefs.read_clock_widget_num(pref);
  WatchPrefs.read_pedo_data(pref);
  WatchPrefs.read_dnd_mode(pref);
  WatchPrefs.read_user_data(pref);
  WatchPrefs.read_global_time(pref);
  WatchPrefs.read_sleep_data(pref);
  WatchPrefs.read_clock_status(pref);
  WatchPrefs.read_gesture_threshold(pref);
  WatchPrefs.read_alarms(pref);
  close_watch_prefs(pref);

  /* Restore HW alarms in alarm_manager_service from the freshly-loaded
     SkaiWatchSys.alarms[]. apply_alarms_from_ble re-arms each entry. */
#if defined(BSP_USING_ALARM_CLIENT)
  if (SkaiWatchSys.alarm_num > 0)
  {
    extern void apply_alarms_from_ble(const T_ALARM *alarms, uint8_t num);
    apply_alarms_from_ble(SkaiWatchSys.alarms, SkaiWatchSys.alarm_num);
  }
#endif
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
  WatchPrefs.write_oled_display_time(pref);
  WatchPrefs.write_language(pref);
  WatchPrefs.write_clock_screen_num(pref);
  WatchPrefs.write_brightness(pref);
  WatchPrefs.write_pedo_data(pref);
  WatchPrefs.write_dnd_mode(pref);
  WatchPrefs.write_user_data(pref);
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

#if !kReleaseMode
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

// BLE device manager share_prefs operations
int ble_dev_prefs_save(const bonded_devices_db_t *db)
{
  if (!db)
  {
    LOG_E("ble_dev_prefs_save: invalid parameter");
    return -1;
  }

  watch_storage_api_lock();

  share_prefs_t *pref = share_prefs_open("ble_dev", SHAREPREFS_MODE_PRIVATE);
  if (!pref)
  {
    LOG_E("Failed to open share_prefs for BLE devices");
    watch_storage_api_unlock();
    return -2;
  }

  int ret = share_prefs_set_int(pref, "device_count", db->count);
  if (ret < 0)
  {
    LOG_E("Failed to save device count");
    share_prefs_close(pref);
    watch_storage_api_unlock();
    return -3;
  }

  ret = share_prefs_set_int(pref, "active_idx", db->active_device_idx);
  if (ret < 0)
  {
    LOG_E("Failed to save active device index");
    share_prefs_close(pref);
    watch_storage_api_unlock();
    return -4;
  }

  ret = share_prefs_set_block(pref, "devices",
                              (void *)&db->devices,
                              sizeof(db->devices));
  if (ret < 0)
  {
    LOG_E("Failed to save device database");
    share_prefs_close(pref);
    watch_storage_api_unlock();
    return -5;
  }

  share_prefs_close(pref);
  watch_storage_api_unlock();

  LOG_I("Successfully saved %d bonded devices to Flash", db->count);
  return 0;
}

int ble_dev_prefs_load(bonded_devices_db_t *db)
{
  if (!db)
  {
    LOG_E("ble_dev_prefs_load: invalid parameter");
    return -1;
  }

  watch_storage_api_lock();

  share_prefs_t *pref = share_prefs_open("ble_dev", SHAREPREFS_MODE_PRIVATE);
  if (!pref)
  {
    LOG_W("No saved BLE device data found");
    watch_storage_api_unlock();
    return 0;
  }

  int32_t count = share_prefs_get_int(pref, "device_count", -1);
  if (count < 0 || count > MAX_BONDED_DEVICES)
  {
    LOG_W("Invalid device count in Flash: %d", count);
    share_prefs_close(pref);
    watch_storage_api_unlock();
    return 0;
  }

  int32_t active_idx = share_prefs_get_int(pref, "active_idx", -1);
  if (active_idx >= MAX_BONDED_DEVICES)
  {
    active_idx = 0xFF;
  }

  bonded_device_t temp_devices[MAX_BONDED_DEVICES];
  int ret = share_prefs_get_block(pref, "devices", (void *)temp_devices,
                                  sizeof(temp_devices));

  if (ret >= 0)
  {
    memcpy(db->devices, temp_devices, sizeof(temp_devices));
    db->count = count;
    db->active_device_idx = active_idx;

    LOG_I("Successfully loaded %d bonded devices from Flash", count);
    LOG_I("Active device index: %d", active_idx);
  }
  else
  {
    LOG_W("Failed to load device database from Flash");
  }

  share_prefs_close(pref);
  watch_storage_api_unlock();

  return (ret >= 0) ? 0 : -2;
}
#endif

// Task for shared preference

uint8_t reboot_reason;

/* Diagnostic-dump and wristband-reboot stubs preserved as no-ops for symbol
   compatibility. The original ftl-based reboot-reason / config-flash flow
   was rewritten on top of share_prefs and these entry points have no live
   callers in the example tree; left as empty bodies so they don't get
   stripped if any out-of-tree code still extern-decls them. */
void show_SkaiWatchSys_info(void) {}
void wristband_config_data_init(void) {}
void wristband_config_struct_flash_reset(void) {}
void wristband_hw_reboot_handle(void) {}
void wristband_sw_reboot_handle(void) {}
uint8_t wristband_get_reboot_reason(void) { return 0; }

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/