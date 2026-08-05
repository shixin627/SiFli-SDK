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
#include <rthw.h> /* rt_hw_interrupt_disable/enable(media_title KE_EVT2→GUI 轉送);
                     PC sim 的 MSVC 鏈不會從 rtthread.h 連帶宣告到,要明 include */
#include <math.h>
#include <cJSON.h>
#include "rtdevice.h"
#include "communicate_protocol.h"
#include "communicate_parse.h"
#include "communicate_task.h"
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
#include "gui_app_fwk.h"
#include "gui_app_int.h"   /* gui_runing_app_t + app_schedule_get_active() (thread-safe active-app read) */
#endif
#ifdef AUDIO_USING_MANAGER
#include "audio_server.h"
#endif
#ifdef BSP_BLE_AMS
/* iOS cross-app media goes through Apple Media Service (the SDK exposes it as
   the "AMS" data service). The phone app can't read Spotify/Apple Music on iOS,
   so the 0x46/0x04 path below is Android-only. */
#include "data_service_subscriber.h"
#include "ams_service.h"
#endif

#define DBG_TAG "bloc.control"
#include "bsp_board.h"
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

#define CURSOR_SENSITIVITY_SCALE 1.0f
#define MOUSE_SENSITIVITY_SCALE 2.5f
/// AMOLED

static uint16_t cur_brightness = 0;
static uint16_t cur_screen_time = 0;
static rt_timer_t smooth_brightness_changed_timer = RT_NULL;
static rt_timer_t smooth_display_changed_timer = RT_NULL;

/* Lazy-create the one-shot timer if needed, otherwise stop + restart it. */
static void debounce_timer_kick(rt_timer_t *t, const char *name,
                                void (*cb)(void *), uint32_t period_ms)
{
	if (!*t)
	{
		/* SOFT_TIMER: brightness / display-time callbacks call into
		   setting_provider which talks to LCPU via data_service (dserv mutex).
		   Hard timers run in SysTick and cannot take mutexes. */
		*t = rt_timer_create(name, cb, RT_NULL, period_ms, RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
	}
	else
	{
		rt_timer_stop(*t);
	}
	if (*t) rt_timer_start(*t);
}

static void oled_brightness_changed_callback(void *param)
{
#ifdef BSP_USING_BLOC_SETTING
	setting_provider.set_brightness(cur_brightness);
#endif
	LOG_D("oled_brightness_changed_callback %d", cur_brightness);
}

/* display time */
static void oled_display_time_changed_cb(void *param)
{
#ifdef BSP_USING_BLOC_SETTING
	setting_provider.set_screen_time(cur_screen_time);
#endif
}

static void set_screen_brightness_smoothly(uint16_t brightness)
{
	if (cur_brightness == brightness || brightness > 100 || brightness < 3)
	{
		return;
	}
	cur_brightness = brightness;
	LOG_D("set_screen_brightness_smoothly %d", cur_brightness);
	debounce_timer_kick(&smooth_brightness_changed_timer,
	                    "oled_brightness_timer",
	                    oled_brightness_changed_callback, 2000);
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
		debounce_timer_kick(&smooth_display_changed_timer,
		                    "display_time_timer",
		                    oled_display_time_changed_cb, 2000);
	}
}

int movement_threshold = 55;

static gesture_t _gesture = {.index = -1, .state = true};

static void notifyGesture()
{
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

/* Media Title */
static media_t current_media_object;
char *get_media_title(void)
{
	LOG_D("get_media_title: title=%s", current_media_object.title);
	return current_media_object.title;
}
char *get_media_artist(void)
{
	LOG_D("get_media_artist: artist=%s", current_media_object.artist);
	return current_media_object.artist;
}
extern void handle_media_widget_title(char *media_title_text);
extern void handle_media_title(char *media_title_text);
extern void handle_dial_header_media_title(char *media_title_text);
extern void mouse_mode_handle_media_title(const char *title);
static void notify_media_title(void)
{
	handle_media_title(get_media_title());
	handle_media_widget_title(get_media_title());
	handle_dial_header_media_title(get_media_title());
	mouse_mode_handle_media_title(get_media_title());
// #ifdef BSP_USING_UI_HANDLER
// 	lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_MEDIA_TITLE,
// 					  .data.media_data.title = get_media_title()};
// 	lvgl_send_msg(msg);
// #endif
}

/* Input may be either a plain title string (legacy) or a JSON object of the
   form {"title": "...", "artist": "..."}. Parse accordingly and fill the
   media object's title/artist fields. */
static void set_media_title(char *title)
{
	if (title == NULL)
	{
		current_media_object.title[0] = '\0';
		current_media_object.artist[0] = '\0';
	}
	else
	{
		bool parsed_as_json = false;
		/* Quick check: only attempt JSON parse if it looks like an object. */
		const char *p = title;
		while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
		if (*p == '{')
		{
			cJSON *root = cJSON_Parse(title);
			if (root != NULL)
			{
				cJSON *title_item  = cJSON_GetObjectItem(root, "title");
				cJSON *artist_item = cJSON_GetObjectItem(root, "artist");
				if (title_item && cJSON_IsString(title_item) &&
					title_item->valuestring)
				{
					strncpy(current_media_object.title,
							title_item->valuestring,
							sizeof(current_media_object.title) - 1);
					current_media_object.title
						[sizeof(current_media_object.title) - 1] = '\0';
				}
				else
				{
					current_media_object.title[0] = '\0';
				}
				if (artist_item && cJSON_IsString(artist_item) &&
					artist_item->valuestring)
				{
					strncpy(current_media_object.artist,
							artist_item->valuestring,
							sizeof(current_media_object.artist) - 1);
					current_media_object.artist
						[sizeof(current_media_object.artist) - 1] = '\0';
				}
				else
				{
					current_media_object.artist[0] = '\0';
				}
				cJSON_Delete(root);
				parsed_as_json = true;
			}
		}
		if (!parsed_as_json)
		{
			/* Fallback: treat the whole input as a plain title. */
			strncpy(current_media_object.title, title,
					sizeof(current_media_object.title) - 1);
			current_media_object.title
				[sizeof(current_media_object.title) - 1] = '\0';
			current_media_object.artist[0] = '\0';
		}
	}
	current_media_object.state = true;
	notify_media_title();
	notify_provider.notification_refresh();
	if (myLancher[app_index_instruction_list].reset_list != NULL && !is_at_instruction_list())
	{
		myLancher[app_index_instruction_list].reset_list();
	}
}

/* ── KE_EVT2 → GUI 轉送槽（2026-07-18 真機 STKOF boot-loop 治本）──
   0x46 媒體標題以前直接在 BLE host 事件緒(KE_EVT2,4KB stack、常態 99% 深)上跑
   set_media_title:cJSON parse + notify fanout + notification_refresh + reset_list
   整串 UI 工作疊上去,長 CJK 標題(如 YouTube 影片名)一到就 SCB_CFSR_UFSR STKOF。
   改為:KE_EVT2 只 malloc+memcpy 原始 payload 到單槽(last-writer-wins,標題語意
   本就是最新蓋舊)、發 LVGL_MSG_TYPE_MEDIA_TITLE_RAW;GUI thread 再跑原本整串。 */
static char *volatile s_media_title_pending = NULL;

void media_title_defer_to_gui(const uint8_t *payload, uint16_t length)
{
	char *buf = (char *)rt_malloc((rt_size_t)length + 1);
	if (buf == NULL) return;
	memcpy(buf, payload, length);
	buf[length] = '\0';
	rt_base_t level = rt_hw_interrupt_disable();
	char *old = s_media_title_pending;
	s_media_title_pending = buf;
	rt_hw_interrupt_enable(level);
	if (old != NULL) rt_free(old); /* GUI 還沒消化的舊標題直接淘汰 */
	lvgl_msg_t msg;
	msg.type = LVGL_MSG_TYPE_MEDIA_TITLE_RAW;
	lvgl_send_msg(msg);
}

/* GUI thread(ui_handler LVGL_MSG_TYPE_MEDIA_TITLE_RAW)。槽空=較新 msg 已先消化,no-op。 */
void media_title_apply_pending(void)
{
	rt_base_t level = rt_hw_interrupt_disable();
	char *buf = s_media_title_pending;
	s_media_title_pending = NULL;
	rt_hw_interrupt_enable(level);
	if (buf == NULL) return;
	set_media_title(buf);
	rt_free(buf);
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
extern void handle_media_play_state(bool media_state);
extern void handle_media_widget_play_state(bool media_state);
extern void handle_dial_header_media_play_state(bool playing);
extern void mouse_mode_handle_media_play_state(bool playing);
static void notify_bt_speaker_media_status(bool status)
{
	bt_media_playing = status;
	handle_media_widget_play_state(bt_media_playing);
    handle_media_play_state(bt_media_playing);
	handle_dial_header_media_play_state(bt_media_playing);
	mouse_mode_handle_media_play_state(bt_media_playing);
// #ifdef BSP_USING_UI_HANDLER
// 	lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_MEDIA_PLAY_STATE,
// 					  .data.media_play_state = bt_media_playing};
// 	lvgl_send_msg(msg);
// #endif
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
extern void set_widget_vol_bar_value(uint8_t volume);
extern void set_app_vol_bar_value(uint8_t volume);
static void bt_speaker_set_volume_percent(uint8_t percent)
{
	if (percent > 100) percent = 100;
	LOG_D("bt_speaker_set_volume_percent %d", percent);
	volume_percent = percent;
	set_app_vol_bar_value(volume_percent);
	set_widget_vol_bar_value(volume_percent);
	// notify ui
// #ifdef BSP_USING_UI_HANDLER
// 	lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_MEDIA_VOLUME, .data.media_volume = volume_percent};
// 	lvgl_send_msg(msg);
// #endif
}
static rt_timer_t smooth_volume_changed_timer = RT_NULL;
static void smooth_volume_changed_timer_callback(void *param)
{
	commu_send_volume_percentage(volume_percent);
}

static void smooth_volume_changed_timer_start(void)
{
	if (!smooth_volume_changed_timer)
	{
		/* SOFT_TIMER: callback walks commu_send_* → skaiwatch_ble_notify which
		   now takes _tx_mutex (commit 7e95e8658). Without SOFT_TIMER the
		   one-shot fires from SysTick ISR and rt_mutex_take asserts. */
		smooth_volume_changed_timer = rt_timer_create("smooth_volume_changed_timer", smooth_volume_changed_timer_callback, RT_NULL, 200, RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
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

#ifdef BSP_BLE_AMS
/*===========================================================================*
 *        iOS media — Apple Media Service (AMS) consumer + control
 *===========================================================================*
 * Mirrors the ANCS pattern (gui_apps/message): subscribe to the SDK "AMS"
 * data service, feed now-playing into the same UI the Android 0x46 path uses,
 * and send transport commands the iPhone honours for ANY active app (Spotify,
 * Apple Music, Podcasts...). Active only when the peer is iOS; Android never
 * exposes AMS so this stays silent there and 0x04 keeps working unchanged. */
static datac_handle_t s_ams_handle = DATA_CLIENT_INVALID_HANDLE;
/* Last play/paused state applied from AMS PlaybackInfo (-1 = unknown). Used to
   de-bounce the streaming PlaybackInfo updates so the play/pause icon only
   moves on a real state flip. */
static int s_ams_play_state = -1;

/* Watch's internal AUDIO_CMD_* -> AMS RemoteCommandID. -1 = no mapping. */
static int ams_cmd_from_audio_cmd(uint8_t audio_cmd)
{
	switch (audio_cmd)
	{
	case AUDIO_CMD_PLAY:        return BLE_AMS_CMD_PLAY;
	case AUDIO_CMD_PAUSE:       return BLE_AMS_CMD_PAUSE;
	case AUDIO_CMD_NEXT:        return BLE_AMS_CMD_NEXT;
	case AUDIO_CMD_PREVIOUS:    return BLE_AMS_CMD_PREV;
	case AUDIO_CMD_VOLUME_UP:   return BLE_AMS_CMD_VOL_UP;
	case AUDIO_CMD_VOLUME_DOWN: return BLE_AMS_CMD_VOL_DOWN;
	default:                    return -1;
	}
}

/* Route a media command to AMS when on iOS. Returns true if handled (caller
   then skips the Android 0x04 send), false to fall through to 0x04. */
static bool ams_try_send_media_cmd(uint8_t audio_cmd)
{
	if (IOS != SkaiWatchSys.phone_os_version)
		return false;
	if (DATA_CLIENT_INVALID_HANDLE == s_ams_handle)
		return false;
	int cmd = ams_cmd_from_audio_cmd(audio_cmd);
	if (cmd < 0)
		return false;
	ams_service_config_t config;
	memset(&config, 0, sizeof(config));
	config.command = AMS_SERVICE_SEND_REMOTE_COMMAND;
	config.data.remote_cmd = (ble_ams_cmd_t)cmd;
	datac_config(s_ams_handle, sizeof(config), (uint8_t *)&config);
	return true;
}

/* AMS Entity Update -> now-playing UI. Title/Artist arrive as separate
   notifications; we update the matching field and refresh. PlaybackInfo is
   "state,rate,elapsed" with state 1 = Playing. */
static int ams_data_callback(data_callback_arg_t *arg)
{
	if (MSG_SERVICE_DATA_NTF_IND != arg->msg_id)
		return 0;
	ble_ams_entity_attr_value_t *v = (ble_ams_entity_attr_value_t *)arg->data;
	if (v == NULL)
		return 0;

	bool track_changed = false;
	if (v->entity_id == BLE_AMS_ENTITY_ID_TRACK &&
		v->attr_id == BLE_AMS_TRACK_ATTR_ID_TILTE)
	{
		uint16_t n = v->len < sizeof(current_media_object.title)
						 ? v->len : sizeof(current_media_object.title) - 1;
		rt_memcpy(current_media_object.title, v->value, n);
		current_media_object.title[n] = '\0';
		track_changed = true;
	}
	else if (v->entity_id == BLE_AMS_ENTITY_ID_TRACK &&
			 v->attr_id == BLE_AMS_TRACK_ATTR_ID_ARTIST)
	{
		uint16_t n = v->len < sizeof(current_media_object.artist)
						 ? v->len : sizeof(current_media_object.artist) - 1;
		rt_memcpy(current_media_object.artist, v->value, n);
		current_media_object.artist[n] = '\0';
		track_changed = true;
	}
	else if (v->entity_id == BLE_AMS_ENTITY_ID_PLAYER &&
			 v->attr_id == BLE_AMS_PLAYER_ATTR_ID_PB_INFO)
	{
		/* PlaybackInfo = "state,rate,elapsed" and streams on every elapsed
		   tick; only react when the play/paused state actually flips, else
		   buffered updates around a pause/resume bounce the icon. */
		char buf[8];
		uint16_t n = v->len < sizeof(buf) ? v->len : sizeof(buf) - 1;
		rt_memcpy(buf, v->value, n);
		buf[n] = '\0';
		int playing = (atoi(buf) == 1);
		if (playing != s_ams_play_state)
		{
			s_ams_play_state = playing;
			notify_bt_speaker_media_status(playing);
		}
	}

	if (track_changed)
	{
		/* Mirror set_media_title()'s full fan-out: live labels (dial header /
		   media app) AND rebuild the message-list media widget via
		   notification_refresh(), so now-playing shows in the notification
		   list too, not only the header. */
		current_media_object.state = true;
		notify_media_title();
		notify_provider.notification_refresh();
		if (myLancher[app_index_instruction_list].reset_list != NULL &&
			!is_at_instruction_list())
		{
			myLancher[app_index_instruction_list].reset_list();
		}
	}
	return 0;
}

static int bloc_media_ams_init(void)
{
	s_ams_handle = datac_open();
	RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != s_ams_handle);
	datac_subscribe(s_ams_handle, "AMS", ams_data_callback, 0);
	return 0;
}
INIT_APP_EXPORT(bloc_media_ams_init);
#endif /* BSP_BLE_AMS */

static void bt_send_media_cmd(uint8_t cmd)
{
	app_audio_set_control_command(cmd);
#ifdef BSP_BLE_AMS
	/* iOS: send via Apple Media Service. Only fall through to the Android
	   0x04 path when not on iOS (or AMS not ready yet). */
	if (ams_try_send_media_cmd(cmd))
		return;
#endif
	commu_send_media_control();
}

void bt_speaker_media_volume_up(void)   { LOG_D("Volume Up");   bt_send_media_cmd(AUDIO_CMD_VOLUME_UP); }
void bt_speaker_media_volume_down(void) { LOG_D("Volume Down"); bt_send_media_cmd(AUDIO_CMD_VOLUME_DOWN); }

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
	notify_bt_speaker_media_status(true);
	bt_send_media_cmd(AUDIO_CMD_PLAY);
}
static void bt_speaker_media_pause(void)
{
	LOG_D("Pause Audio");
	notify_bt_speaker_media_status(false);
	bt_send_media_cmd(AUDIO_CMD_PAUSE);
}
static void bt_speaker_media_next(void) { LOG_D("Next Audio");     bt_send_media_cmd(AUDIO_CMD_NEXT); }
static void bt_speaker_media_prev(void) { LOG_D("Previous Audio"); bt_send_media_cmd(AUDIO_CMD_PREVIOUS); }

/* Mouse */
static bool isMouseMode = false;
bool app_control_get_mouse_mode(void)
{
	return isMouseMode;
}
void app_control_set_mouse_mode(bool flag)
{
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

/* Send a {.type, .data.action} LVGL message; no-op if UI handler disabled. */
static void send_action_msg(uint8_t msg_type, uint8_t action)
{
#ifdef BSP_USING_UI_HANDLER
	lvgl_msg_t msg = {.type = msg_type, .data.action = action};
	lvgl_send_msg(msg);
#endif
}
void notify_pageview_action(uint8_t action) { send_action_msg(LVGL_MSG_TYPE_PAGEVIEW_ACTION, action); }

/*Tap indicator*/
// 0: press
// 1: release
// 2: anti-press
// 3: anti-release
static void trigger_finger_event(uint8_t finger_event)
{
	LOG_D("trigger_finger_event %d", finger_event);
	if (finger_event == 1)
	{
		if (!get_enable_tap_and_hold())
		{
			lvgl_set_global_keypad_enter_cmd();
		}
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

/* Bare-event LVGL message (no payload); no-op if UI handler disabled. */
static void send_bare_msg(uint8_t msg_type)
{
#ifdef BSP_USING_UI_HANDLER
	lvgl_msg_t msg = {.type = msg_type};
	lvgl_send_msg(msg);
#endif
}
static void trigger_longpress_event(void) { send_bare_msg(LVGL_MSG_TYPE_LONGPRESS_EVENT); }
static void trigger_back_event(void)      { send_bare_msg(LVGL_MSG_TYPE_BACK_EVENT); }

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
		extern void exit_testing_app(void);
		exit_testing_app();
	}
#endif
}
// remote control
static void take_photo(void)
{
	commu_send_phone_control_cmd();
}

static void find_phone(void)
{
	commu_send_find_mobile();
}

extern void volume_up_through_hid(void);
extern void volume_down_through_hid(void);

extern uint8_t get_volume_control_difference(void);
static struct rt_event sys_media_event;

static void remote_volume_up(void)
{
	if (SkaiWatchSys.connected_to_phone) control_provider.bt_speaker_media_volume_up();
	else                                 volume_up_through_hid();
}
static void remote_volume_down(void)
{
	if (SkaiWatchSys.connected_to_phone) control_provider.bt_speaker_media_volume_down();
	else                                 volume_down_through_hid();
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

#if !kReleaseMode
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
#endif

// Combine function in provider object
ControlProvider control_provider;

static int bloc_control_provider_register(void)
{
	control_provider.notify_pageview_action = notify_pageview_action;
	control_provider.notify_unit_test_action = test_all;
	control_provider.trigger_longpress_event = trigger_longpress_event;
	control_provider.trigger_back_event = trigger_back_event;
	control_provider.trigger_finger_event = trigger_finger_event;
	control_provider.trigger_unknown_event = trigger_unknown_event;
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
	control_provider.get_media_artist = get_media_artist;
	control_provider.take_photo = take_photo;
	control_provider.find_phone = find_phone;
	control_provider.screen_brightness_smoothly = set_screen_brightness_smoothly;
	control_provider.screen_time_smoothly = set_screen_time_smoothly;
	return 0;
}

INIT_APP_EXPORT(bloc_control_provider_register);

#ifdef USING_FSR_ADC_SAMPLER
/*
 * FSR-402 ADC sampler thread — 10Hz, 獨立 thread 不阻塞 motion loop。
 * 休眠時拉長 delay 到 1s,避免持續採樣耗電。
 *
 * Latched 值供 motion_tracking (100Hz) 讀取(透過 bloc_control_fsr_adc_latest());
 * 同時做邊緣觸發的 HID mouse handfree 切換 + 左鍵 press/release。
 */
extern void set_hid_mouse_handfree_mode_to(bool v);

/* True only while the STANDALONE mouse app (APP_ID_MOUSE) is in the foreground.
   Runs on the fsr_smp thread, NOT the gui scheduler, so gui_app_is_actived()
   (which asserts caller == gui thread) would crash here. Read the active-app
   pointer directly instead — worst case a one-tick-stale snapshot. The
   device-page-hosted trackpad runs under APP_ID_MAIN, so it is intentionally
   excluded. */
static bool fsr_mouse_app_is_foreground(void)
{
	gui_runing_app_t *cur = app_schedule_get_active();
	return cur != RT_NULL && strcmp(cur->id, APP_ID_MOUSE) == 0;
}

static volatile rt_uint32_t g_fsr_adc_latest = 0;
static rt_thread_t fsr_adc_sampler_thread = RT_NULL;

/* Press detection is RELATIVE to an auto-captured resting baseline, as a
   PERCENTAGE drop — not an absolute ADC delta. The FSR's press swing scales with
   the baseline (a press from 18000→17000 is −5.6%, the same press from 4000→3777
   is also −5.6%, i.e. only −223 absolute), so a fixed delta would mis-fire once
   the baseline drifts. A ratio is the invariant. FSR-402 wiring: pressing LOWERS
   the reading. Baseline = resting (finger-off) value, captured once at boot and
   re-capturable on demand via the `fsr_recal` MSH command.
   NOTE: at a very low baseline the absolute swing shrinks toward the ADC noise
   floor, so detection there is inherently marginal — restore the sensor baseline
   (mechanical preload / moisture / damage), no threshold model can fix lost range. */
#define FSR_BASELINE_DEFAULT 18000  /* fallback baseline until first calibration */
#define FSR_BASELINE_MIN      6000  /* readings below this look pressed/faulty — don't calibrate to them */
#define FSR_CAL_SAMPLES          8  /* average this many resting samples (~0.8s @10Hz) for the baseline */
/* Thresholds are PERMILLE of the baseline, with hysteresis: an ON threshold to
   ENTER the state (drop below) and a higher OFF threshold to LEAVE it (rise
   above). The gap stops a press from dropping out on a small force change — which
   near a hard press is a BIG ADC swing because the FSR is very non-linear there
   (e.g. v jumps 3882→11556 for what feels like a slight relax). OFF must stay
   below baseline so a genuine finger-off (v→baseline) always releases. */
#define FSR_LIGHT_ON_PERMILLE  944  /* < 94.4% of baseline → move / handfree ON   (was 17000/18000) */
#define FSR_LIGHT_OFF_PERMILLE 970  /* > 97.0% → handfree OFF */
#define FSR_HEAVY_ON_PERMILLE  556  /* < 55.6% of baseline → left-button PRESS     (was 10000/18000) */
#define FSR_HEAVY_OFF_PERMILLE 780  /* > 78.0% → left-button RELEASE (sticky hold band) */

static rt_uint32_t g_fsr_baseline = FSR_BASELINE_DEFAULT;
static volatile bool g_fsr_recal_request = true;  /* true → (re)capture baseline at next opportunity */

rt_uint32_t bloc_control_fsr_adc_latest(void)
{
	return g_fsr_adc_latest;
}

static void fsr_adc_sampler_thread_entry(void *parameter)
{
	/* 邊緣觸發：避免每 100ms 重複 set / press / release 造成 BLE 流量 */
	bool prev_handfree = false;
	bool left_pressed = false;
	while (1)
	{
		if (SkaiWatchSys.sys_power_status != SYS_POWER_STATUS_ON)
		{
			rt_thread_mdelay(1000);
			continue;
		}
		g_fsr_adc_latest = fsr_adc_read_value();
		// LOG_D("FSR ADC: %d (baseline %d)", g_fsr_adc_latest, g_fsr_baseline);

		/* Baseline calibration (boot / on-demand): average a few resting samples,
		   assuming the FSR is NOT pressed. A reading below FSR_BASELINE_MIN looks
		   pressed/faulty, so it restarts the window instead of poisoning the
		   baseline. While calibrating we don't drive press/handfree. */
		if (g_fsr_recal_request)
		{
			static rt_uint32_t cal_acc = 0;
			static rt_uint8_t  cal_cnt = 0;
			if (g_fsr_adc_latest >= FSR_BASELINE_MIN)
			{
				cal_acc += g_fsr_adc_latest;
				if (++cal_cnt >= FSR_CAL_SAMPLES)
				{
					g_fsr_baseline = cal_acc / cal_cnt;
					cal_acc = 0;
					cal_cnt = 0;
					g_fsr_recal_request = false;
					LOG_I("FSR baseline calibrated: %d", g_fsr_baseline);
				}
			}
			else
			{
				cal_acc = 0;
				cal_cnt = 0;
			}
			rt_thread_mdelay(100);
			continue;
		}

		/* Thresholds are a PERCENTAGE of the baseline (so the same physical press
		   triggers regardless of where the baseline sits) WITH hysteresis: pick the
		   ON threshold while in the released state, the higher OFF threshold while
		   held. (baseline*permille max ~18000*970 fits int32 with room to spare.) */
		rt_int32_t v = (rt_int32_t)g_fsr_adc_latest;
		rt_int32_t base = (rt_int32_t)g_fsr_baseline;
		rt_int32_t light_on  = base * FSR_LIGHT_ON_PERMILLE  / 1000;
		rt_int32_t light_off = base * FSR_LIGHT_OFF_PERMILLE / 1000;
		/* Press mode clicks on the light (move) threshold; default mode on the
		   heavy (deeper) threshold. */
		bool press_mode = SkaiWatchSys.flag_field.mouse_press_mode;
		rt_int32_t click_on  = base * (press_mode ? FSR_LIGHT_ON_PERMILLE  : FSR_HEAVY_ON_PERMILLE)  / 1000;
		rt_int32_t click_off = base * (press_mode ? FSR_LIGHT_OFF_PERMILLE : FSR_HEAVY_OFF_PERMILLE) / 1000;

		/* Hysteresis: released → press only below *_on; held → release only above
		   *_off. Between the two it keeps its current state (sticky). */
		bool want_handfree = prev_handfree ? (v < light_off) : (v < light_on);
		if (press_mode)
			want_handfree = true;  /* press mode: always free to move */
		bool want_left_press = left_pressed ? (v < click_off) : (v < click_on);

		/* founder 2026-07-17: FSR squeeze mouse-control disabled for now — lifting the
		   wrist (arm / sleeve pressing the pad) kept mis-triggering handfree fly + clicks.
		   ADC keeps sampling (g_fsr_adc_latest still readable, fsr_recal still calibrates);
		   we just stop driving handfree + the left button. Remove these two lines to restore. */
		want_handfree = false;
		want_left_press = false;

		/* Squeeze only drives the air-mouse while the standalone mouse app is
		   open. Outside it a squeeze must not leak a BLE HID click. Forcing
		   want_* false here lets the edge-triggered release below clean up a
		   press/handfree that was live when the app closed mid-squeeze. */
		if (!fsr_mouse_app_is_foreground())
		{
			want_handfree = false;
			want_left_press = false;
		}

		/* Edge-only logs: print just the press/release transitions (not every tick)
		   so the press判定 can be eyeballed. v / baseline / threshold included. */
		if (want_handfree != prev_handfree)
		{
			LOG_I("FSR handfree %s  v=%d base=%d on=%d off=%d", want_handfree ? "ON" : "OFF",
			      v, g_fsr_baseline, light_on, light_off);
			set_hid_mouse_handfree_mode_to(want_handfree);
			prev_handfree = want_handfree;
		}

		if (want_left_press && !left_pressed)
		{
			LOG_I("FSR LEFT PRESS    v=%d base=%d on=%d off=%d", v, g_fsr_baseline, click_on, click_off);
			if (control_provider.ble_hid_mouse_left_press)
				control_provider.ble_hid_mouse_left_press();
			left_pressed = true;
		}
		else if (!want_left_press && left_pressed)
		{
			LOG_I("FSR LEFT RELEASE  v=%d base=%d on=%d off=%d", v, g_fsr_baseline, click_on, click_off);
			if (control_provider.ble_hid_mouse_left_release)
				control_provider.ble_hid_mouse_left_release();
			left_pressed = false;
		}

		rt_thread_mdelay(100); /* 10Hz */
	}
}

static int fsr_adc_sampler_thread_init(void)
{
	fsr_adc_sampler_thread = rt_thread_create(
		"fsr_smp", fsr_adc_sampler_thread_entry, RT_NULL, 1024,
		RT_THREAD_PRIORITY_MAX - 4, 10);
	if (fsr_adc_sampler_thread != RT_NULL)
	{
		rt_thread_startup(fsr_adc_sampler_thread);
	}
	return 0;
}
INIT_APP_EXPORT(fsr_adc_sampler_thread_init);

/* Re-capture the FSR resting baseline. Run with the FSR untouched; the sampler
   averages the next FSR_CAL_SAMPLES resting samples into the new baseline. */
static int fsr_recal(int argc, char **argv)
{
	g_fsr_recal_request = true;
	rt_kprintf("FSR baseline recalibration requested — keep finger OFF\n");
	return 0;
}
MSH_CMD_EXPORT(fsr_recal, "recapture FSR mouse resting baseline (keep finger off)");
#endif // USING_FSR_ADC_SAMPLER

#if !kReleaseMode
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
				lv_disp_set_rotation(NULL, LV_DISPLAY_ROTATION_270);
			}
		}
		else if (strcmp(argv[1], "-media_title") == 0)
		{
			   control_provider.set_media_title(argc > 2 ? argv[2] : "");
		}
	}
	return 0;
}
MSH_CMD_EXPORT(bloc_control_cmd, "bloc_control_cmd [OPTION] ...");
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
