/**
 ******************************************************************************
 * @file   bloc_control.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
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
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include <math.h>
#include "rtdevice.h"
#include "communicate_protocol.h"
#include "communicate_parse.h"
#include "app_mainmenu.h"
#include "communicate_parse_control.h"
#include "watch_global_data.h"
#ifdef BSP_USING_GESTURE_RECOGNITION
#include "gesture_recognition_task.h"
#endif
#include "bloc_control.h"
#include "bloc_setting.h"
#include "bloc_peripheral.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#include "bloc_motion_tracking.h"
#include "bloc_v2t.h"
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#endif
#ifdef AUDIO_USING_MANAGER
#include "audio_server.h"
#endif

#define DBG_TAG "bloc.control"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

enum
{
	AUDIO_CMD_PAUSE = 0,
	AUDIO_CMD_PLAY = 1,
	AUDIO_CMD_PREVIOUS = 2,
	AUDIO_CMD_NEXT = 3,
	AUDIO_CMD_VOLUME_UP = 4,
	AUDIO_CMD_VOLUME_DOWN = 5,
};

typedef struct
{
	int8_t x;
	int8_t y;
} CURSOR_COORDINATE_T;

#define CURSOR_SENSITIVITY_SCALE 1.0f
#define MOUSE_SENSITIVITY_SCALE 2.5f
/// AMOLED

static uint16_t cur_brightness = 0;
static uint16_t cur_screen_time = 0;
static rt_timer_t smooth_brightness_changed_timer = RT_NULL;

static void oled_brightness_changed_callback(void *param)
{
#ifdef BSP_USING_BLOC_SETTING
	setting_provider.set_brightness(cur_brightness);
#endif
	LOG_D("oled_brightness_changed_callback %d", cur_brightness);
}
static void oled_brightness_changed_timer_start(uint16_t brightness)
{
	if (!smooth_brightness_changed_timer)
	{
		smooth_brightness_changed_timer = rt_timer_create("oled_brightness_timer", oled_brightness_changed_callback, RT_NULL, 2000, RT_TIMER_FLAG_ONE_SHOT);
	}
	else
	{
		rt_timer_stop(smooth_brightness_changed_timer);
	}
	rt_timer_start(smooth_brightness_changed_timer);
}

/* display time */
static void oled_display_time_changed_cb(void *param)
{
#ifdef BSP_USING_BLOC_SETTING
	setting_provider.set_screen_time(cur_screen_time);
#endif
}

static rt_timer_t smooth_display_changed_timer = RT_NULL;
static void display_time_changed_timer_start(uint16_t display_time)
{
	if (!smooth_display_changed_timer)
	{
		smooth_display_changed_timer = rt_timer_create("display_time_timer", oled_display_time_changed_cb, RT_NULL, 2000, RT_TIMER_FLAG_ONE_SHOT);
	}
	else
	{
		rt_timer_stop(smooth_display_changed_timer);
	}
	rt_timer_start(smooth_display_changed_timer);
}

static void set_screen_brightness_smoothly(uint16_t brightness)
{
	if (cur_brightness == brightness || brightness > 100 || brightness < 3)
	{
		return;
	}
	cur_brightness = brightness;
	LOG_D("set_screen_brightness_smoothly %d", cur_brightness);
	oled_brightness_changed_timer_start(cur_brightness);
}

static void control_screen_time(uint16_t time)
{
	if (time > 60)
	{
		time = 255;
	}
	else if (time < 5)
	{
		time = 5;
	}
	if (cur_screen_time != time)
	{
		cur_screen_time = time;
		LOG_D("control_screen_time %d", cur_screen_time);
		gui_set_screen_timeout(cur_screen_time);
	}
}

static void set_screen_time_smoothly(uint16_t time)
{
	if (time > 60)
	{
		time = 255;
	}
	else if (time < 5)
	{
		time = 5;
	}
	if (cur_screen_time != time)
	{
		cur_screen_time = time;
		LOG_D("set_screen_time_smoothly %d", cur_screen_time);
		display_time_changed_timer_start(cur_screen_time);
	}
}

int movement_threshold = 55;
/* gesture mode variables */
bool isGestureMode = false;
bool app_control_get_gesture_mode(void)
{
	return isGestureMode;
}
void app_control_set_gesture_mode(bool mode)
{
	if (isGestureMode != mode)
	{
		isGestureMode = mode;
	}
}

static coordinate_t _virtualPlane = {.x = 0, .y = 0, .z = 0, .state = true};
coordinate_t *getCoordinate(void)
{
	return &_virtualPlane;
}

void setCoordinateX(int value)
{
	if (_virtualPlane.x != value)
	{
		_virtualPlane.x = value;
		_virtualPlane.state = true;
	}
}

void setCoordinateY(int value)
{
	if (_virtualPlane.y != value)
	{
		_virtualPlane.y = value;
		_virtualPlane.state = true;
		// LOG_D("setCoordinateY %d", _virtualPlane.y);
	}
}

void setCoordinateZ(int value)
{
	if (_virtualPlane.z != value)
	{
		_virtualPlane.z = value;
		_virtualPlane.state = true;
	}
}

static gesture_t _gesture = {.index = -1, .state = true};
gesture_t *getGesture(void)
{
	return &_gesture;
}

static void notifyGesture()
{
	// L1SendData data;
	// data.event = L1SEND_GESTURE_DETECT;
	// data.res.gesture_label = label;
	// L1_send_event(data);
	_gesture.state = true;
}

void setGestureIndex(int index)
{
	if (index != _gesture.index)
	{
		_gesture.index = index;
		notifyGesture();
	}
}

music_press_flags music_press_flag;
void reset_audio_processing_flag(void)
{
	music_press_flag.touch_flags = 0;
}

/* Media Title */
static media_t current_media_object;
char *get_media_title(void)
{
	return current_media_object.title;
}
static void notify_media_title(void)
{
#ifdef BSP_USING_UI_HANDLER
	lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_MEDIA_TITLE,
					  .data.media_data.title = get_media_title()};
	lvgl_send_msg(msg);
#endif
}

static void set_media_title(char *title)
{
	strcpy(current_media_object.title, title);
	current_media_object.state = true;
	notify_media_title();
	notify_provider.notification_refresh();
	if (myLancher[app_index_app_list].reset_list != NULL && !is_at_app_list())
	{
		myLancher[app_index_app_list].reset_list();
	}
}

/* Media Status */
static bool bt_media_playing = false;
static uint8_t media_control_command = 0;
static uint8_t volume_percent = 8; // 0~100,4% per step

static bool bt_speaker_get_status(void)
{
	return bt_media_playing;
}
static void bt_speaker_set_status(bool status)
{
	bt_media_playing = status;
}
static void notify_bt_speaker_media_status(bool status)
{
	bt_media_playing = status;
#ifdef BSP_USING_UI_HANDLER
	lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_MEDIA_PLAY_STATE,
					  .data.media_play_state = bt_media_playing};
	lvgl_send_msg(msg);
#endif
}

uint8_t app_audio_get_control_command(void)
{
	return media_control_command;
}
void app_audio_set_control_command(uint8_t command)
{
	if (media_control_command != command)
	{
		media_control_command = command;
	}
}
uint8_t bt_speaker_get_volume_percent(void)
{
	return volume_percent;
}
static void bt_speaker_set_volume_percent(uint8_t percent)
{
	if (percent > 100)
	{
		percent = 100;
	}
	else if (percent < 0)
	{
		percent = 0;
	}
	volume_percent = percent;
	// notify ui
#ifdef BSP_USING_UI_HANDLER
	lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_MEDIA_VOLUME, .data.media_volume = volume_percent};
	lvgl_send_msg(msg);
#endif
}
static rt_timer_t smooth_volume_changed_timer = RT_NULL;
static void smooth_volume_changed_timer_callback(void *param)
{
	L1SendData data = {.event = L1SEND_VOLUME_PERCENTAGE, .res.bt_speaker_volume = volume_percent};
	L1_send_event(data);
}

static void smooth_volume_changed_timer_start(void)
{
	if (!smooth_volume_changed_timer)
	{
		smooth_volume_changed_timer = rt_timer_create("smooth_volume_changed_timer", smooth_volume_changed_timer_callback, RT_NULL, 200, RT_TIMER_FLAG_ONE_SHOT);
	}
	else
	{
		rt_timer_stop(smooth_volume_changed_timer);
	}
	rt_timer_start(smooth_volume_changed_timer);
}
void notify_volume_changed(void)
{
	smooth_volume_changed_timer_start();
}
void app_audio_volume_unmute(void)
{
	smooth_volume_changed_timer_start();
	volume_percent = 8;
}
void bt_speaker_set_volumn_smoothly(uint8_t percent, bool notify)
{
	if (volume_percent != percent)
	{
		volume_percent = percent;
		if (notify)
		{
			notify_volume_changed();
		}
	}
}

void bt_speaker_media_volume_up(void)
{
	LOG_D("Volume Up");
	app_audio_set_control_command(AUDIO_CMD_VOLUME_UP);
	L1SendData data;
	data.event = L1SEND_MEDIA_CONTROL;
	L1_send_event(data);
}

void bt_speaker_media_volume_down(void)
{
	LOG_D("Volume Down");
	app_audio_set_control_command(AUDIO_CMD_VOLUME_DOWN);
	L1SendData data;
	data.event = L1SEND_MEDIA_CONTROL;
	L1_send_event(data);
}

static void ble_hid_play_pause(void)
{
	LOG_D("Play/Pause");
	extern void play_pause_through_hid(void);
	play_pause_through_hid();
}

static void ble_hid_consumer_back(void)
{
	extern void HID_CONSUMER_GoBack(void);
	HID_CONSUMER_GoBack();
}

static void bt_speaker_media_play(void)
{
	LOG_D("Play Audio");
#ifdef AUDIO_USING_MANAGER
#endif
	notify_bt_speaker_media_status(true);
	app_audio_set_control_command(AUDIO_CMD_PLAY);
	L1SendData data;
	data.event = L1SEND_MEDIA_CONTROL;
	L1_send_event(data);
}
static void bt_speaker_media_pause(void)
{
	LOG_D("Pause Audio");
#ifdef AUDIO_USING_MANAGER
#endif
	notify_bt_speaker_media_status(false);
	app_audio_set_control_command(AUDIO_CMD_PAUSE);
	L1SendData data;
	data.event = L1SEND_MEDIA_CONTROL;
	L1_send_event(data);
}
static void bt_speaker_media_next(void)
{
	LOG_D("Next Audio");
	app_audio_set_control_command(AUDIO_CMD_NEXT);
	L1SendData data;
	data.event = L1SEND_MEDIA_CONTROL;
	L1_send_event(data);
}
static void bt_speaker_media_prev(void)
{
	LOG_D("Previous Audio");
	app_audio_set_control_command(AUDIO_CMD_PREVIOUS);
	L1SendData data;
	data.event = L1SEND_MEDIA_CONTROL;
	L1_send_event(data);
}

/* skaios mode state */
// use 2-bit to define the state on two devices
// 0x00: both devices are not in skaios mode
// 0x01: device 1 is in skaios mode, device 2 is not in skaios mode
// 0x10: device 1 is not in skaios mode, device 2 is in skaios mode
// 0x11: both devices are in skaios mode
static uint8_t skaios_mode_state = 0x00;
uint8_t bloc_control_get_skaios_mode_state(void)
{
	return skaios_mode_state;
}
static void bloc_control_set_skaios_mode_state(uint8_t state)
{
	skaios_mode_state = state;
}

void bloc_control_send_skaios_mode_state(void)
{
	uint8_t psendbuf[6];
	psendbuf[0] = SKAI_LINK_COMMAND_ID;			  /*command id*/
	psendbuf[1] = L2_HEADER_VERSION;			  /*L2 header version */
	psendbuf[2] = SKAILINK_KEY_SKAIOS_MODE_STATE; /*first key, */
	psendbuf[3] = 0;
	psendbuf[4] = 1; /* length  = 1 */
	psendbuf[5] = skaios_mode_state;
	skaiwatch_ble_notify(psendbuf, 6);
}

/*cursor*/
CURSOR_COORDINATE_T cursor_coordinate;
bool isCursorMode = false;
bool app_control_get_cursor_mode(void)
{
	return isCursorMode;
}
void app_control_set_cursor_mode(bool flag)
{
	isCursorMode = flag;
}
uint8_t cursor_gesture = 0;
void app_control_set_cursor_gesture(uint8_t gesture)
{
	cursor_gesture = gesture;
}
void set_cursor_movement(int8_t deltaX, int8_t deltaY, int speed)
{
	int scaled_dx = deltaX * speed;
	int scaled_dy = deltaY * speed;

	cursor_coordinate.x = scaled_dx;
	cursor_coordinate.y = scaled_dy;
}

void Send_Cursor_Report(void)
{
	uint8_t psendbuf[7];
	psendbuf[0] = CONTROL_COMMAND_ID; /*command id*/
	psendbuf[1] = L2_HEADER_VERSION;  /*L2 header version */
	psendbuf[2] = KEY_TP_COORDINATE;  /*first key, */
	psendbuf[3] = 0;
	psendbuf[4] = 2; /* length  = 2 */
	psendbuf[5] = cursor_coordinate.x;
	psendbuf[6] = cursor_coordinate.y;
	skaiwatch_ble_notify(psendbuf, 7);
}
void Send_Touchpad_Gesture(void)
{
	uint8_t psendbuf[6];
	psendbuf[0] = CONTROL_COMMAND_ID; /*command id*/
	psendbuf[1] = L2_HEADER_VERSION;  /*L2 header version */
	psendbuf[2] = KEY_TP_GESTURE;	  /*first key, */
	psendbuf[3] = 0;
	psendbuf[4] = 1; /* length  = 1 */
	psendbuf[5] = cursor_gesture;
	skaiwatch_ble_notify(psendbuf, 6);
}

static bool hid_event_flag = true;
void app_control_set_hid_event_flag(bool flag)
{
	hid_event_flag = flag;
}
bool app_control_get_hid_event_flag(void)
{
	return hid_event_flag;
}

/* Mouse */
static bool isMouseMode = false;
bool app_control_get_mouse_mode(void)
{
	return isMouseMode;
}
void app_control_set_mouse_mode(bool flag)
{
	if (flag)
	{
		// skaiwatch_ble_set_performance(true);
		bloc_control_set_skaios_mode_state(true);
		bloc_control_send_skaios_mode_state();
	}
	else
	{
		// skaiwatch_ble_set_performance(false);
		bloc_control_set_skaios_mode_state(false);
		bloc_control_send_skaios_mode_state();
	}
	isMouseMode = flag;
}
/* Game */
static bool _isGameMode = false;
bool app_control_get_game_mode(void)
{
	return _isGameMode;
}
void app_control_set_game_mode(bool flag)
{
	_isGameMode = flag;
	watch_sys_sync.set_multi_gesture_mode(flag);
}

// motion_tracking
static bool _enableMotionTracking = true;
bool app_control_get_motion_tracking(void)
{
	return _enableMotionTracking;
}
void app_control_set_motion_tracking(bool flag)
{
	if (flag != _enableMotionTracking)
	{
		LOG_I("app_control_set_motion_tracking:%d", flag);
		_enableMotionTracking = flag;
	}
}

/* unit test */

uint16_t UnitTest_Unicode = 0x0000;
void set_Unicode(uint16_t unicode)
{
	UnitTest_Unicode = unicode;
}
uint16_t get_Unicode(void)
{
	return UnitTest_Unicode;
}

static bool _gui_interactive_mode = false;
bool bloc_control_get_gui_interactive_mode(void)
{
	return _gui_interactive_mode;
}
void bloc_control_set_gui_interactive_mode(bool flag)
{
	if (_gui_interactive_mode != flag)
	{
		_gui_interactive_mode = flag;
	}
}

// Function implementation
void notify_pageview_action(uint8_t action)
{
#ifdef BSP_USING_UI_HANDLER
	lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_PAGEVIEW_ACTION,
					  .data.action = action};
	lvgl_send_msg(msg);
#endif
}
void notify_launcher_action(uint8_t action)
{
#ifdef BSP_USING_UI_HANDLER
	lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_LAUNCHER_ACTION,
					  .data.action = action};
	lvgl_send_msg(msg);
#endif
}
static void trigger_tap_event(void)
{
#ifdef BSP_USING_UI_HANDLER
	lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_TAP_EVENT};
	lvgl_send_msg(msg);
#endif
}

static void send_remote_control_event(uint8_t action)
{
	L1SendData data;
	data.event = L1SEND_FINGER_TAP;
	data.res.gesture_label = action;
	L1_send_event(data);
}

/*Tap indicator*/
// 0: press
// 1: release
// 2: anti-press
// 3: anti-release
static void trigger_finger_event(uint8_t finger_event)
{
	LOG_D("trigger_finger_event %d", finger_event);
	bool is_tap = (finger_event == 1);
#ifdef BSP_USING_UI_HANDLER
	// lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_TAP_INDICATOR, .data.gesture = finger_event};
	// lvgl_send_msg(msg);
	if (is_tap)
	{
		uint8_t led_brightness = 20;
		watch_system_interact(INTERACT_RGB_LED_OPEN_WRITE, &led_brightness);
	}
	else
	{
		watch_system_interact(INTERACT_RGB_LED_CLOSE, NULL);
	}
#endif
	if (finger_event < 2)
	{
		if (!get_enable_tap_and_hold())
		{
			lvgl_set_global_keypad_enter_cmd();
		}
	}
	else
	{
		send_remote_control_event(finger_event);
	}
}

/*Tap indicator*/
static void trigger_unknown_event(void)
{
	extern bool is_test_mode(void);
	if (is_test_mode() == false)
	{
		return;
	}
#ifdef BSP_USING_UI_HANDLER
	lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_UNKNOWN_INDICATOR, .data.action = 1};
	lvgl_send_msg(msg);
#endif
}

static void send_virtual_gesture(uint8_t gesture)
{
	L1SendData data;
	data.event = L1SEND_VIRTUAL_GESTURE;
	data.res.gesture_label = gesture;
	L1_send_event(data);
}

static void trigger_longpress_event(void)
{
#ifdef BSP_USING_UI_HANDLER
	lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_LONGPRESS_EVENT};
	lvgl_send_msg(msg);
#endif
}

static void trigger_back_event(void)
{
#ifdef BSP_USING_UI_HANDLER
	lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_BACK_EVENT};
	lvgl_send_msg(msg);
#endif
}

static void test_all(uint8_t action)
{
#if !kReleaseMode
	LOG_I("Test all %d", action);
	if (action == 1)
	{
		extern void goto_to_testing_app(void);
		goto_to_testing_app();
	}
	else
	{
		extern void close_operational_test_thread(void);
		close_operational_test_thread();
		extern void exit_testing_app(void);
		exit_testing_app();
	}
#endif
}
// remote control
static void take_photo(void)
{
	L1SendData data;
	data.event = L1SEND_RETURN_PHONE_CONTROL_CMD_EVENT;
	L1_send_event(data);
}

static void find_phone(void)
{
	L1SendData data;
	data.event = L1SEND_RETURN_FIND_MOBILE_COMMAND;
	L1_send_event(data);
}

extern void volume_up_through_hid(void);
extern void volume_down_through_hid(void);

extern uint8_t get_volume_control_difference(void);
static struct rt_event sys_media_event;

static void remote_volume_up(void)
{
	if (SkaiWatchSys.connected_to_phone)
	{
		control_provider.bt_speaker_media_volume_up();
	}
	else
	{
		volume_up_through_hid();
	}
}

static void remote_volume_down(void)
{
	if (SkaiWatchSys.connected_to_phone)
	{
		control_provider.bt_speaker_media_volume_down();
	}
	else
	{
		volume_down_through_hid();
	}
}

#if ENABLE_MEDIA_VOLUMN_BAR_CONTROL
static int volume_count = 0;
#endif

void media_control_event_handler(uint32_t event)
{
	switch (event)
	{
	case SYS_EVENT_PLAY_PAUSE:
	{
		LOG_D("SYS_EVENT_PLAY_PAUSE");
		if (SkaiWatchSys.connected_to_phone)
		{
			if (bt_speaker_get_status())
			{
				control_provider.bt_speaker_media_pause();
			}
			else
			{
				control_provider.bt_speaker_media_play();
			}
		}
		else
		{
			control_provider.ble_hid_play_pause();
		}
		break;
	}
	case SYS_EVENT_NEXT:
	{
		LOG_D("SYS_EVENT_NEXT");
		if (SkaiWatchSys.connected_to_phone)
		{
			control_provider.bt_speaker_media_next();
		}
		else
		{
			extern void play_next_through_hid(void);
			play_next_through_hid();
		}
		break;
	}
	case SYS_EVENT_PREV:
	{
		LOG_D("SYS_EVENT_PREV");
		if (SkaiWatchSys.connected_to_phone)
		{
			control_provider.bt_speaker_media_prev();
		}
		else
		{
			extern void play_prev_through_hid(void);
			play_prev_through_hid();
		}
		break;
	}
	case SYS_EVENT_MULTIPLE_PAGES:
	{
		LOG_D("SYS_EVENT_MULTIPLE_PAGES");
		motor_pattern_scrolling_app();
		if (control_provider.ble_hid_keyboard_multitask != NULL)
			control_provider.ble_hid_keyboard_multitask(true);
		break;
	}

#if ENABLE_MEDIA_VOLUMN_BAR_CONTROL
	case SYS_EVENT_VOLUME_UP:
	{
		volume_count = 0;
		for (int i = 0; i < get_volume_control_difference(); i++)
		{
			volume_count++;
			remote_volume_up();
			rt_thread_mdelay(150);
		}
		break;
	}
	case SYS_EVENT_VOLUME_DOWN:
	{
		volume_count = 0;
		for (int i = 0; i < get_volume_control_difference(); i++)
		{
			volume_count++;
			remote_volume_down();
			rt_thread_mdelay(150);
		}
		break;
	}

#endif
	case SYS_EVENT_VOLUME_BTN_UP:
	{
		LOG_D("SYS_EVENT_VOLUME_BTN_UP");
		// remote_volume_up();
		volume_up_through_hid();
		break;
	}

	case SYS_EVENT_VOLUME_BTN_DOWN:
	{
		LOG_D("SYS_EVENT_VOLUME_BTN_DOWN");
		// remote_volume_down();
		volume_down_through_hid();
		break;
	}

	default:
		break;
	}
}

void sys_media_task_handler(void *param)
{
	rt_uint32_t recv_set = 0;

	while (1)
	{
		if (rt_event_recv(&sys_media_event, MEDIA_SYS_ALL_EVENT, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
						  RT_WAITING_FOREVER, &recv_set) == RT_EOK)
		{
			media_control_event_handler(recv_set);
		}
	}
}

int sys_media_event_init(void)
{
	rt_thread_t sys_media_task;

	if (rt_event_init(&sys_media_event, "sys_media_event", RT_IPC_FLAG_FIFO) != RT_EOK)
	{
		LOG_E("Failed to init sys_media_event");
		return -1;
	}
	sys_media_task = rt_thread_create("sys_media_task", sys_media_task_handler, RT_NULL, 1024, 10, 10);
	if (sys_media_task != RT_NULL)
	{
		rt_thread_startup(sys_media_task);
	}
	return 0;
}
INIT_APP_EXPORT(sys_media_event_init);

void sys_media_event_set(uint32_t event)
{
	rt_event_send(&sys_media_event, event);
}

// 媒體播放器
static int control_bt_speaker(int argc, char *argv[])
{
	if (argc >= 2)
	{
		// 控制播放暫停
		if (strcmp(argv[1], "-play_pause") == 0)
		{
			sys_media_event_set(SYS_EVENT_PLAY_PAUSE);
		}
		// 設定下一首
		else if (strcmp(argv[1], "-next") == 0)
		{
			sys_media_event_set(SYS_EVENT_NEXT);
		}
		// 設定上一首
		else if (strcmp(argv[1], "-prev") == 0)
		{
			sys_media_event_set(SYS_EVENT_PREV);
		}
		// 設定音量
		else if (strcmp(argv[1], "-volume") == 0)
		{
			if (strcmp(argv[2], "up") == 0)
			{
				sys_media_event_set(SYS_EVENT_VOLUME_BTN_UP);
			}
			else if (strcmp(argv[2], "down") == 0)
			{
				sys_media_event_set(SYS_EVENT_VOLUME_BTN_DOWN);
			}
			else
			{
				uint8_t volume = atoi(argv[2]);
				control_provider.bt_speaker_set_volume(volume, 1);
			}
		}
	}
	return 0;
}
MSH_CMD_EXPORT(control_bt_speaker, "control_bt_speaker [OPTION] ...");

// Combine function in provider object
ControlProvider control_provider;

static int bloc_control_provider_register(void)
{
	control_provider.notify_pageview_action = notify_pageview_action;
	control_provider.notify_unit_test_action = test_all;
	control_provider.trigger_tap_event = trigger_tap_event;
	control_provider.trigger_longpress_event = trigger_longpress_event;
	control_provider.trigger_back_event = trigger_back_event;
	control_provider.trigger_finger_event = trigger_finger_event;
	control_provider.trigger_unknown_event = trigger_unknown_event;
	control_provider.send_virtual_gesture = send_virtual_gesture;
	control_provider.bt_speaker_get_status = bt_speaker_get_status;
	control_provider.bt_speaker_set_status = bt_speaker_set_status;
	control_provider.notify_bt_speaker_media_status = notify_bt_speaker_media_status;
	control_provider.bt_speaker_set_volume = bt_speaker_set_volumn_smoothly;
	control_provider.bt_speaker_get_volume = bt_speaker_get_volume_percent;
	control_provider.bt_phone_set_volume = bt_speaker_set_volume_percent;
	control_provider.bt_speaker_media_volume_up = bt_speaker_media_volume_up;
	control_provider.bt_speaker_media_volume_down = bt_speaker_media_volume_down;
	control_provider.ble_hid_play_pause = ble_hid_play_pause;
	control_provider.ble_hid_consumer_back = ble_hid_consumer_back;
	control_provider.bt_speaker_media_play = bt_speaker_media_play;
	control_provider.bt_speaker_media_pause = bt_speaker_media_pause;
	control_provider.bt_speaker_media_next = bt_speaker_media_next;
	control_provider.bt_speaker_media_prev = bt_speaker_media_prev;
	control_provider.set_media_title = set_media_title;
	control_provider.get_media_title = get_media_title;
	control_provider.take_photo = take_photo;
	control_provider.find_phone = find_phone;
	control_provider.screen_brightness_smoothly = set_screen_brightness_smoothly;
	control_provider.screen_time_smoothly = set_screen_time_smoothly;
	return 0;
}

INIT_APP_EXPORT(bloc_control_provider_register);

static int bloc_control_cmd(int argc, char *argv[])
{
	if (argc >= 2)
	{
		// 設定播放狀態
		if (strcmp(argv[1], "take_photo") == 0)
		{
			control_provider.take_photo();
		}
		else if (strcmp(argv[1], "find_phone") == 0)
		{
			control_provider.find_phone();
		}
		else if (strcmp(argv[1], "-rotate") == 0)
		{
			int degree = atoi(argv[2]);
			if (degree == 0)
			{
				lv_disp_set_rotation(NULL, LV_DISP_ROT_NONE);
			}
			else if (degree == 90)
			{
				lv_disp_set_rotation(NULL, LV_DISP_ROT_270);
			}
		}
	}
	return 0;
}
MSH_CMD_EXPORT(bloc_control_cmd, "bloc_control_cmd [OPTION] ...");
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
