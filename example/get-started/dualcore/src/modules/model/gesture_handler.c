/**
 ******************************************************************************
 * @file   gesture_handler.c
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

#include <rtthread.h>
#include "gesture_handler.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#include "communicate_update_image.h"
#include "bloc_peripheral.h"
#include "bloc_control.h"
#include "bloc_motion_tracking.h"
#include "bloc_v2t.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "ui_handler.h"
#include "watch_global_data.h"
#include "watch_sys_service.h"

#define DBG_TAG "gesture.handler"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

// #define USE_DOUBLE_TAP_GESTURE

// create an event to handle hold event detection
static rt_event_t virtual_gesture_detect_event;
static rt_uint32_t last_gesture_event;

void send_virtual_gesture_event(rt_uint32_t e)
{
  if (!SkaiWatchSys.flag_field.is_wearing)
  {
    LOG_W("Not wearing");
    return;
  }
  rt_event_send(virtual_gesture_detect_event, e);
  last_gesture_event = e;
}

static void print_gesture_event(rt_uint32_t recv_set)
{
  switch (recv_set)
  {
  case GESTURE_EVENT_PRESS:
    LOG_I("GESTURE_EVENT_PRESS");
    break;
  case GESTURE_EVENT_TAP:
    LOG_I("GESTURE_EVENT_TAP");
    break;
  case GESTURE_EVENT_HOLD:
    LOG_I("GESTURE_EVENT_HOLD");
    break;
  case GESTURE_EVENT_FINGER_RELEASE:
    LOG_I("GESTURE_EVENT_FINGER_RELEASE");
    break;
  case GESTURE_EVENT_FORCE_RELEASE:
    LOG_I("GESTURE_EVENT_FORCE_RELEASE");
    break;
  case GESTURE_EVENT_BACK:
    LOG_I("GESTURE_EVENT_BACK");
    break;
  case GESTURE_EVENT_WRIST_PRONATION:
    LOG_I("GESTURE_EVENT_WRIST_PRONATION");
    break;
  case GESTURE_EVENT_HAND_RELEASE:
    LOG_I("GESTURE_EVENT_HAND_RELEASE");
    break;
  case GESTURE_EVENT_MOVE_FORWARD:
    LOG_I("GESTURE_EVENT_MOVE_FORWARD");
    break;
  case GESTURE_EVENT_MOVE_BACKWARD:
    LOG_I("GESTURE_EVENT_MOVE_BACKWARD");
    break;
  default:
    LOG_E("Unknown gesture event");
    break;
  }
}

#define GESTURE_BREATHE_INTERVAL_MS 500    // 0.5 seconds
#define GESTURE_DOUBLE_TAP_INTERVAL_MS 500 // 0.5 seconds
#define GESTURE_LONG_TAP_UNLOCK_MS 500
#define HALF_SCREEN_HEIGHT (LV_VER_RES / 2)
#define HALF_SCREEN_WIDTH (LV_HOR_RES / 2)

static rt_tick_t last_gesture_time = 0;
static uint8_t gesture_detect_state = gesture_finger_release;
static void set_gesture(rt_tick_t ts, gesture_detect_state_t gesture)
{
  LOG_I("set_gesture: %d to %d", gesture_detect_state, gesture);
  switch (gesture)
  {
  case gesture_press:
    last_gesture_time = ts;
    gesture_detect_state = gesture;
    lv_disp_trig_activity(NULL);
    break;
  case gesture_tap:
    last_gesture_time = ts;
    gesture_detect_state = gesture;
    lv_disp_trig_activity(NULL);
    break;
  case gesture_hold:
    last_gesture_time = ts;
    gesture_detect_state = gesture;
    lv_disp_trig_activity(NULL);
    break;
  case gesture_finger_release:
    last_gesture_time = ts;
    gesture_detect_state = gesture;
    lv_disp_trig_activity(NULL);
    break;
#if GestureLongPress
  case gesture_longpress:
    last_gesture_time = ts;
    gesture_detect_state = gesture;
    break;
#endif
  case gesture_back:
    last_gesture_time = ts;
    gesture_detect_state = gesture;
    lv_disp_trig_activity(NULL);
    break;
  case gesture_double_tap:
    last_gesture_time = ts;
    gesture_detect_state = gesture;
    lv_disp_trig_activity(NULL);
    break;
  case gesture_wrist_pronation:
    last_gesture_time = ts;
    gesture_detect_state = gesture;
    lv_disp_trig_activity(NULL);
    break;
  case gesture_hand_release:
    last_gesture_time = ts;
    gesture_detect_state = gesture;
    lv_disp_trig_activity(NULL);
    break;
  case gesture_unknown:
    gesture_detect_state = gesture;
    LOG_W("gesture unknown");
    break;
  default:
    LOG_E("gesture undefined");
    break;
  }
}

static bool open_geture_model = false;
bool open_gesture_model(void)
{
  if (gesture_detect_state == gesture_back && last_gesture_time + 500 > rt_tick_get())
  {
    open_geture_model = false;
  }
  else
  {
    open_geture_model = true;
  }
  return open_geture_model;
}

#ifdef USE_DOUBLE_TAP_GESTURE
static void handle_double_tap(rt_tick_t ts);
#endif

static void handle_finger_tap(rt_tick_t ts);
static void handle_finger_release(rt_tick_t ts);
static rt_tick_t hold_time = NULL;

static void handle_finger_pressed(rt_tick_t ts)
{
#ifdef USE_DOUBLE_TAP_GESTURE
  if ((gesture_detect_state == gesture_press || gesture_detect_state == gesture_tap || gesture_detect_state == gesture_hold) &&
      ts - last_gesture_time < rt_tick_from_millisecond(GESTURE_DOUBLE_TAP_INTERVAL_MS))
  {
    handle_double_tap(ts);
    return;
  }
#endif
  control_provider.trigger_finger_event(1);
  set_gesture(ts, gesture_press);
  peripheral_provider.set_tap_status(true);
#if ENABLE_VIRTUAL_TOUCH
  gesture_touch_event_handler();
#endif
}

#if ENABLE_VIRTUAL_TOUCH
static bool scrolling_object_flag = false;
void scrolling_object(bool open_scrolling_object_flag)
{
  LOG_D("scrolling_object_start:%d,%d,%d", SkaiWatchSys.motion_control_lock, scrolling_object_flag, open_scrolling_object_flag);
  if (SkaiWatchSys.motion_control_lock || (scrolling_object_flag && !open_scrolling_object_flag))
  {
    return;
  }
  scrolling_object_flag = true;
  peripheral_provider.set_tap_status(true);
  simulate_press_by_orientation();
  LOG_D("scrolling_object_end");
}

void stop_scrolling_object(void)
{
  LOG_D("stop_scrolling_object_start");
  if (!scrolling_object_flag)
  {
    return;
  }
  scrolling_object_flag = false;
  peripheral_provider.set_tap_status(false);
  LOG_D("stop_scrolling_object_end");
  if (SkaiWatchSys.motion_control_lock)
  {
    return;
  }
}

#endif

static void handle_finger_tap(rt_tick_t ts)
{
  set_gesture(ts, gesture_tap);
  control_provider.trigger_finger_event(0);
  peripheral_provider.set_tap_status(false);
}

static void handle_finger_release(rt_tick_t ts)
{
  set_gesture(ts, gesture_finger_release);
  control_provider.trigger_finger_event(0);
  peripheral_provider.set_tap_status(false);

  if (app_control_get_mouse_mode())
  {
    return;
  }

#if ENABLE_VIRTUAL_TOUCH
  extern void release_navigation_bar(void);
  release_navigation_bar();
#endif
}

extern bool get_hid_mouse_handfree_mode(void);
extern bool get_imu_data_collection_status(void);
static void handle_wrist_back(rt_tick_t ts)
{
  if ((ts - last_gesture_time < rt_tick_from_millisecond(GESTURE_BREATHE_INTERVAL_MS)) && gesture_detect_state == gesture_back)
  {
    return;
  }
  set_gesture(ts, gesture_back);
  if ((app_control_get_mouse_mode() && get_hid_mouse_handfree_mode()))
  {
    control_provider.ble_hid_consumer_back();
    return;
  }
  lvgl_set_global_keypad_esc_cmd();
}

#ifdef USE_DOUBLE_TAP_GESTURE
static void handle_double_tap(rt_tick_t ts)
{
  set_gesture(ts, gesture_double_tap);
  control_provider.trigger_finger_event(0);
  peripheral_provider.set_tap_status(false);
  watch_system_interact(WATCH_GESTURE_UNLOCK, NULL);
}
#endif

void force_release_finger(void)
{
  LOG_D("%s", __func__);
  if (gesture_detect_state != gesture_finger_release)
  {
    send_virtual_gesture_event(GESTURE_EVENT_FORCE_RELEASE);
  }
}

extern bool imu_data_collection;
static void gesture_event_handler_hcpu(rt_uint32_t recv_set)
{
  rt_tick_t current_time = rt_tick_get();
  print_gesture_event(recv_set);
  switch (recv_set)
  {
  // 偵測到手指按下，視為press
  case GESTURE_EVENT_PRESS:
  {
    if ((gesture_detect_state == gesture_wrist_pronation || gesture_detect_state == gesture_hand_release || gesture_detect_state == gesture_back) && current_time - last_gesture_time < 600)
    {
      return;
    }
    handle_finger_pressed(current_time);
  }
  break;
  // 偵測到手指按下後250ms之內放開，視為tap
  case GESTURE_EVENT_TAP:
  {
    // if (gesture_detect_state == gesture_press)
    {
      handle_finger_tap(current_time);
    }
  }
  break;
  // 偵測到手指按下後超過250ms，視為hold
  case GESTURE_EVENT_HOLD:
  {
    if (gesture_detect_state == gesture_press)
    {
      set_gesture(current_time, gesture_hold);
    }
  }
  break;
  // 判別為hold後偵測到放開，一律視為release
  case GESTURE_EVENT_FINGER_RELEASE:
  {
    if (gesture_detect_state == gesture_hold)
    {
      if (has_user_started_controlling_with_arm())
      {
        LOG_D("just_release_hold");
        handle_finger_release(current_time);
      }
      else
      {
        LOG_D("just_release_tap after hold");
        handle_finger_tap(current_time);
      }
    }
  }
  break;

  case GESTURE_EVENT_FORCE_RELEASE:
  {
    handle_finger_release(current_time);
    break;
  }

#if GestureLongPress
    // 判別為hold後300ms之內沒放開，視為 longpress
  case GESTURE_EVENT_LONGPRESS:
  {
    set_gesture(gesture_longpress);
    control_provider.trigger_longpress_event();
  }
  break;
#endif

  // 手轉一下，視為swipe back
  case GESTURE_EVENT_BACK:
  {
    if (imu_data_collection || get_imu_data_collection_status())
    {
      return;
    }
    // if (!is_at_home())
    {
      handle_wrist_back(current_time);
    }
    break;
  }

  case GESTURE_EVENT_WRIST_PRONATION:
  {
    if (imu_data_collection)
    {
      return;
    }

    set_gesture(current_time, gesture_wrist_pronation);
    if (!gui_app_is_actived(APP_ID_MAIN))
    {
      gui_app_run("Main");
    }
    else
    {
      if (is_at_home())
      {
        switch_watch_motion_control_mode(true, false);
        animate_to_message_list();
      }
    }
    break;
  }

  case GESTURE_EVENT_HAND_RELEASE:
  {
    LOG_I("[GESTURE]Hand release: %d", current_time);
    if ((gesture_detect_state == gesture_press || gesture_detect_state == gesture_tap || gesture_detect_state == gesture_hold) &&
        current_time - last_gesture_time < 100)
    {
      return;
    }

    set_gesture(current_time, gesture_hand_release);
    watch_system_interact(WATCH_GESTURE_UNLOCK, NULL);
    break;
  }

  default:
    break;
  }
}

#define THREAD_STACK_SIZE 2048
#define THREAD_PRIORITY RT_THREAD_PRIORITY_LOW - 1
#define THREAD_TIMESLICE 5

static void virtual_gesture_processor(void *parameter)
{
  virtual_gesture_detect_event = rt_event_create("vir_gesture_evt", RT_IPC_FLAG_FIFO);
  rt_uint32_t recv_set = 0;
  while (1)
  {
    rt_event_recv(virtual_gesture_detect_event,
                  GESTURE_ALL_EVENTS, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recv_set);

    // Don't process gestures when BLE DFU is running
    if (is_ble_dfu_thread_running())
    {
      continue;
    }
    gesture_event_handler_hcpu(recv_set);
  }
}

static int create_virtual_gesture_processor(void)
{
  rt_thread_t virtual_gesture_process_thread = rt_thread_create("vir_gesture_task", virtual_gesture_processor,
                                                                RT_NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  if (virtual_gesture_process_thread != RT_NULL)
  {
    rt_thread_startup(virtual_gesture_process_thread);
  }
  return 0;
}
INIT_APP_EXPORT(create_virtual_gesture_processor);

#if 0
typedef struct
{
  const char *name;
  rt_uint32_t event;
} gesture_test_item_t;

static const gesture_test_item_t gesture_test_items[] = {
    {"press", GESTURE_EVENT_PRESS},
    {"tap", GESTURE_EVENT_TAP},
    {"hold", GESTURE_EVENT_HOLD},
    {"release", GESTURE_EVENT_FINGER_RELEASE},
    {"force_release", GESTURE_EVENT_FORCE_RELEASE},
    {"back", GESTURE_EVENT_BACK},
    {"wrist_pronation", GESTURE_EVENT_WRIST_PRONATION},
    {"hand_release", GESTURE_EVENT_HAND_RELEASE},
    {"move_forward", GESTURE_EVENT_MOVE_FORWARD},
    {"move_backward", GESTURE_EVENT_MOVE_BACKWARD},
};

static void print_gesture_help(void)
{
  rt_kprintf("Usage: utest_gesture [OPTION]\n");
  rt_kprintf("Options:\n");
  rt_kprintf("  -send <event>    Send a gesture event\n");
  rt_kprintf("  -status          Show current gesture status\n");
  rt_kprintf("  -list            List all available gesture events\n");
  rt_kprintf("  -sequence        Test common gesture sequence\n");
  rt_kprintf("\nAvailable events:\n");
  for (int i = 0; i < sizeof(gesture_test_items) / sizeof(gesture_test_items[0]); i++)
  {
    rt_kprintf("  %s\n", gesture_test_items[i].name);
  }
  rt_kprintf("\nExamples:\n");
  rt_kprintf("  utest_gesture -send tap\n");
  rt_kprintf("  utest_gesture -send back\n");
  rt_kprintf("  utest_gesture -sequence\n");
}

static void test_gesture_sequence(void)
{
  LOG_I("Starting gesture sequence test...");

  LOG_I("Test 1: Press -> Tap");
  send_virtual_gesture_event(GESTURE_EVENT_PRESS);
  rt_thread_mdelay(100);
  send_virtual_gesture_event(GESTURE_EVENT_TAP);
  rt_thread_mdelay(500);

  LOG_I("Test 2: Press -> Hold -> Release");
  send_virtual_gesture_event(GESTURE_EVENT_PRESS);
  rt_thread_mdelay(300);
  send_virtual_gesture_event(GESTURE_EVENT_HOLD);
  rt_thread_mdelay(200);
  send_virtual_gesture_event(GESTURE_EVENT_FINGER_RELEASE);
  rt_thread_mdelay(500);

  LOG_I("Test 3: Back gesture");
  send_virtual_gesture_event(GESTURE_EVENT_BACK);
  rt_thread_mdelay(500);

  LOG_I("Gesture sequence test completed");
}

static void show_gesture_status(void)
{
  rt_kprintf("\n=== Gesture System Status ===\n");
  rt_kprintf("Last gesture time: %u\n", last_gesture_time);
  rt_kprintf("Last gesture event: 0x%08X\n", last_gesture_event);
  rt_kprintf("Current gesture state: %d\n", gesture_detect_state);
  rt_kprintf("Open gesture model: %s\n", open_geture_model ? "true" : "false");
  rt_kprintf("Is wearing: %s\n", SkaiWatchSys.flag_field.is_wearing ? "true" : "false");
  rt_kprintf("=============================\n\n");
}

static int utest_gesture(int argc, char *argv[])
{
  if (argc < 2)
  {
    print_gesture_help();
    return 0;
  }

  if (strcmp(argv[1], "-list") == 0)
  {
    rt_kprintf("\n=== Available Gesture Events ===\n");
    for (int i = 0; i < sizeof(gesture_test_items) / sizeof(gesture_test_items[0]); i++)
    {
      rt_kprintf("%2d. %-20s (0x%08X)\n",
                 i + 1,
                 gesture_test_items[i].name,
                 gesture_test_items[i].event);
    }
    rt_kprintf("================================\n\n");
    return 0;
  }
  else if (strcmp(argv[1], "-status") == 0)
  {
    show_gesture_status();
    return 0;
  }
  else if (strcmp(argv[1], "-sequence") == 0)
  {
    test_gesture_sequence();
    return 0;
  }
  else if (strcmp(argv[1], "-press") == 0)
  {
    send_virtual_gesture_event(GESTURE_EVENT_PRESS);
    return 0;
  }
  else if (strcmp(argv[1], "-tap") == 0)
  {
    send_virtual_gesture_event(GESTURE_EVENT_TAP);
    return 0;
  }
  else if (strcmp(argv[1], "-release") == 0)
  {
    watch_system_interact(WATCH_GESTURE_UNLOCK, NULL);
    return 0;
  }
  else if (strcmp(argv[1], "-send") == 0)
  {
    if (argc < 3)
    {
      LOG_E("Missing event name. Use '-list' to see available events");
      return -1;
    }

    bool found = false;
    for (int i = 0; i < sizeof(gesture_test_items) / sizeof(gesture_test_items[0]); i++)
    {
      if (strcmp(argv[2], gesture_test_items[i].name) == 0)
      {
        LOG_I("Sending gesture event: %s", gesture_test_items[i].name);
        send_virtual_gesture_event(gesture_test_items[i].event);
        found = true;
        break;
      }
    }

    if (!found)
    {
      LOG_E("Unknown gesture event: %s", argv[2]);
      LOG_E("Use '-list' to see available events");
      return -1;
    }
    return 0;
  }
  else
  {
    LOG_E("Unknown option: %s", argv[1]);
    print_gesture_help();
    return -1;
  }

  return 0;
}
MSH_CMD_EXPORT(utest_gesture, "Test gesture events");
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/