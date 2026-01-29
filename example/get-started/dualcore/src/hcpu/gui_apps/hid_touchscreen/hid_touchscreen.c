/**
 ******************************************************************************
 * @file   hid_touchscreen.c
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
/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#ifdef BSP_USING_BLOC
#include "bloc_control.h"
#include "bloc_setting.h"
#endif
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#include "communicate_protocol.h"
#include "ui_handler.h"
#include "ui_helper.h"

#ifdef APP_ID_TOUCHSCREEN

#define DBG_TAG "hid.touchscreen"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

LV_IMG_DECLARE(img_touchscreen);

#define TOUCH_AREA_WIDTH 128
#define TOUCH_AREA_HEIGHT 128
typedef struct touch_screen_coordinate
{
	uint16_t x;
	uint16_t y;
} touch_screen_coordinate_t;
touch_screen_coordinate_t touch_screen_coordinate = {.x = TOUCH_AREA_WIDTH / 2, .y = TOUCH_AREA_HEIGHT / 2};
touch_screen_coordinate_t *get_touch_screen_coordinate(void)
{
	return &touch_screen_coordinate;
}
void recenter_touch_screen_coordinate(void)
{
	touch_screen_coordinate.x = TOUCH_AREA_WIDTH / 2;
	touch_screen_coordinate.y = TOUCH_AREA_HEIGHT / 2;
}
void set_touch_screen_coordinate(int dx, int dy)
{
	int32_t newX = touch_screen_coordinate.x + dx;
	int32_t newY = touch_screen_coordinate.y + dy;
	if (newX < 0)
	{
		touch_screen_coordinate.x = 0;
	}
	else if (newX > TOUCH_AREA_WIDTH)
	{
		touch_screen_coordinate.x = TOUCH_AREA_WIDTH;
	}
	else
	{
		touch_screen_coordinate.x = newX;
	}

	if (newY < 0)
	{
		newY = 0;
	}
	else if (newY > TOUCH_AREA_HEIGHT)
	{
		newY = TOUCH_AREA_HEIGHT;
	}
	else
	{
		touch_screen_coordinate.y = newY;
	}
}
void map_touch_screen_coordinate(int x, int y)
{
	uint16_t newX = x * TOUCH_AREA_WIDTH / LV_HOR_RES;
	if (newX < 3)
	{
		touch_screen_coordinate.x = 3;
	}
	else if (newX > TOUCH_AREA_WIDTH)
	{
		touch_screen_coordinate.x = TOUCH_AREA_WIDTH - 3;
	}
	uint16_t newY = y * TOUCH_AREA_HEIGHT / (LV_VER_RES - 6);
	touch_screen_coordinate.x = newX;
	touch_screen_coordinate.y = newY;
}

static void close_btn_event_cb(lv_event_t *e)
{
	lv_event_code_t event = lv_event_get_code(e);

	if (LV_EVENT_CLICKED == event)
	{
		gui_app_self_exit();
	}
}

static lv_point_t start_point;
static lv_point_t last_point;
static uint32_t press_time = 0;

static void gesture_event_cb(lv_event_t *e)
{
	lv_event_code_t code = lv_event_get_code(e);
	lv_indev_t *indev = lv_indev_get_act();
	if (code == LV_EVENT_GESTURE)
	{
		lv_dir_t g = lv_indev_get_gesture_dir(indev);
		if (g == LV_DIR_RIGHT)
		{
			if (start_point.x < 30)
			{
				watch_system_interact(INTERACT_MOTOR_VIBRATE_SLIDING, NULL);
			}
		}
		else if (g == LV_DIR_LEFT)
		{
			if (start_point.x > LV_HOR_RES - 30)
			{
				watch_system_interact(INTERACT_MOTOR_VIBRATE_SLIDING, NULL);
			}
		}
	}
}

static void exit_btn_event_cb(lv_event_t *e)
{
	lv_event_code_t event = lv_event_get_code(e);

	if (LV_EVENT_CLICKED == event)
	{
		// gui_app_exit(APP_ID_TOUCHSCREEN);
		gui_app_self_exit();
	}
}

static bool pressed = false;

static void plain_event_cb(lv_event_t *e)
{
	lv_event_code_t code = lv_event_get_code(e);
	lv_obj_t *obj = lv_event_get_target(e);
	lv_indev_t *indev = lv_indev_get_act();

	if (code == LV_EVENT_PRESSED)
	{
		lv_indev_get_point(indev, &start_point);
		lv_indev_get_point(indev, &last_point);
		press_time = lv_tick_get();
		pressed = false;
		// LOG_D("press x: %d, y: %d", start_point.x, start_point.y);
	}
	else if (code == LV_EVENT_PRESSING)
	{
		lv_point_t now_point;
		lv_indev_get_point(indev, &now_point);
		// LOG_D("x: %d, y: %d", now_point.x, now_point.y);

		int16_t dx = now_point.x - last_point.x;
		int16_t dy = now_point.y - last_point.y;

		if (dy != 0)
		{
			if (dy >= 2 || dy <= -2)
			{
				last_point = now_point;
			}
		}
		if (!pressed)
		{
			// if (lv_tick_elaps(press_time) >= CLICK_TIME_MS)
			{
				pressed = true;
				LOG_D("pressed");
			}
		}
		else
		{
			if (control_provider.ble_hid_touch_screen_press != NULL)
			{
				map_touch_screen_coordinate(now_point.x, now_point.y);
				touch_screen_coordinate_t *coordinate = get_touch_screen_coordinate();
				// LOG_D("touch_screen: x=%d, y=%d", coordinate->x, coordinate->y);
				control_provider.ble_hid_touch_screen_press(coordinate->x, coordinate->y);
			}
		}

		// 畫線
		// lv_obj_t *line = lv_obj_create(lv_scr_act());
		// lv_obj_set_size(line, 2, 2);
		// lv_obj_set_pos(line, now_point.x, now_point.y);
		// lv_obj_set_style_bg_color(line, lv_color_hex(0xFFFFFF), 0);
	}
	else if (code == LV_EVENT_RELEASED)
	{
		if (pressed)
		{
			if (control_provider.ble_hid_touch_screen_release != NULL)
			{
				touch_screen_coordinate_t *coordinate = get_touch_screen_coordinate();
				control_provider.ble_hid_touch_screen_release(coordinate->x, coordinate->y);
			}
			pressed = false;
		}
	}
	// click on the screen
	else if (code == LV_EVENT_CLICKED)
	{
	}
}

extern void gesture_event_handler(lv_event_t *e);
void lv_create_touch_screen(lv_obj_t *scr)
{
	lv_obj_add_event_cb(scr, gesture_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_t *bg = common_watch_bg(scr);
	lv_obj_add_event_cb(bg, plain_event_cb, LV_EVENT_ALL, NULL);

	lv_obj_t *line1 = lv_obj_create(scr);
	lv_obj_set_size(line1, 460, 2);
	lv_obj_align(line1, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_bg_color(line1, lv_color_hex(0x666666), 0);
	lv_obj_t *line2 = lv_obj_create(scr);
	lv_obj_set_size(line2, 2, 460);
	lv_obj_align(line2, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_bg_color(line2, lv_color_hex(0x666666), 0);

	lv_obj_t *exit_btn = common_text_button(scr, "Exit", NULL, 100, 50, exit_btn_event_cb);
	lv_obj_align(exit_btn, LV_ALIGN_TOP_MID, 0, 20);
}

static void on_start(lv_obj_t *scr)
{
	lv_create_touch_screen(scr);
	app_control_set_cursor_mode(true);
}

static void on_resume(void)
{
	setting_provider.set_power_save_mode(0);
}

static void on_pause(void)
{
	setting_provider.set_power_save_mode(1);
}

static void on_stop(void)
{
	app_control_set_cursor_mode(false);
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
	switch (msg)
	{
	case GUI_APP_MSG_ONSTART:
	{
		lv_obj_t *scr = lv_scr_act();
		on_start(scr);
	}
	break;

	case GUI_APP_MSG_ONRESUME:
		on_resume();
		break;

	case GUI_APP_MSG_ONPAUSE:
		on_pause();
		break;

	case GUI_APP_MSG_ONSTOP:
		on_stop();
		break;
	default:
		break;
	}
}

static int app_main(intent_t i)
{
	gui_app_regist_msg_handler(APP_ID_TOUCHSCREEN, msg_handler);

	return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(touchscreen), LV_EXT_IMG_GET(img_touchscreen), APP_ID_TOUCHSCREEN, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/