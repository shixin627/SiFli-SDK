/**
 ******************************************************************************
 * @file   lv_control_app_list_layout.c
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

#include "lvgl.h"
#include "lv_simplified_obj.h"
#include "lv_ext_resource_manager.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include "watch_system_interact.h"
#include "custom_trans_anim.h"
#include <math.h>
#include "ui_helper.h"
#include <rtthread.h>
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#include "ui_img_helper.h"
#endif
#ifdef BSP_USING_GESTURE_HANDLER
#include "gesture_handler.h"
#endif
#include "watch_global_data.h"
#ifdef BSP_USING_BLOC
#include "bloc_v2t.h"
#include "bloc_peripheral.h"
#include "bloc_weather.h"
#include "bloc_calendar.h"
#include "bloc_motion_tracking.h"
#define Disc_Control
#endif

#ifdef APP_ID_WIDGETS

#define DBG_TAG "app.list.layout"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define APP_ID "control_app_list"
#include <stdio.h>
#include <stdint.h>

#define LIST_ITEM_SPACING (60)
#define LIST_ITEM_CONTROL_BTN_WIDTH (150)
#define LIST_ITEM_CONTROL_BTN_HEIGHT (150)

#define LIST_RADIUS (600)
#define LIST_CONTROL_ADJUST_Y_POSITION (170)

#define LIST_ITEM_RADIUS (240)
#define LIST_ITEM_BORDER_SIDE LV_BORDER_SIDE_RIGHT

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
// #define USE_QUICK_OPEN_AI

// LV_IMG_DECLARE(img_flashlight);
// LV_IMG_DECLARE(img_activity);
LV_IMG_DECLARE(weather);
LV_IMG_DECLARE(img_messages);
LV_IMG_DECLARE(img_note);
// LV_IMG_DECLARE(img_alarm);
LV_IMG_DECLARE(message_bar);
LV_IMG_DECLARE(img_mouse);
LV_IMG_DECLARE(img_touchscreen);
LV_IMG_DECLARE(img_touchpad);
LV_IMG_DECLARE(img_workout);
LV_IMG_DECLARE(select_prompt);
LV_IMG_DECLARE(mouse_mode_icon);
LV_IMG_DECLARE(micro_icon);
LV_IMG_DECLARE(icon_qrcode);
LV_IMG_DECLARE(icon_dnd_mode);
LV_IMG_DECLARE(img_media_pause);
LV_IMG_DECLARE(img_media_play);
LV_IMG_DECLARE(control_cursor_img);
LV_IMG_DECLARE(control_background_img);
LV_IMG_DECLARE(control_selection_bg);

typedef struct
{
	const char *title;
	const char *icon;
	// lv_obj_t *widget;
	char *app_id;
} control_app_list_item_t;

uint16_t CONTROL_APP_LIST_ITEMS_DEFINITION[] = {
#ifdef APP_ID_SKAI
// app_id_ai,
#endif
#ifdef APP_ID_MEDIA
// app_id_media,
#endif
	// app_id_calendar,

	// app_id_weather,
	app_id_exercise,
#ifdef APP_ID_TIMER
	app_id_timer,
#endif
#ifdef APP_ID_GESTURE
	app_id_gesture,
#endif
	app_id_flashlight,
#ifdef APP_ID_RECORDER
	app_id_recorder,
#endif

#ifdef APP_ID_HEART_RATE
// app_id_heart_rate,
#endif
#ifdef APP_ID_ACTIVITY
// app_id_activity,
#endif

#ifdef APP_ID_CALCULATOR
// app_id_calculator,
#endif

#ifdef APP_ID_ALARM
// app_id_alarm,
#endif
#ifdef APP_ID_SETTING
// app_id_setting,
#endif
#ifdef APP_ID_MOUSE
	app_id_mouse,
#endif

#ifdef APP_ID_TOUCHSCREEN
	app_id_touchscreen,
#endif
#ifdef APP_ID_TOUCHPAD
	app_id_touchpad,
#endif
#ifdef APP_ID_NOTE_CHATROOM
// app_id_note,
#endif

};

typedef struct
{
	lv_obj_t *list;
	lv_obj_t *p_control_app_list_bg;
	lv_obj_t *quick_open_app_bg;
	lv_obj_t *p_control_app_list_title;
	lv_obj_t *p_control_app_indicator_btn[ARRAY_SIZE(CONTROL_APP_LIST_ITEMS_DEFINITION)];
	lv_obj_t *p_control_app_indicator_select_prompt[ARRAY_SIZE(CONTROL_APP_LIST_ITEMS_DEFINITION)];
} control_app_list_layout_t;
static control_app_list_layout_t *p_control_app_list_layout;

const char *get_control_app_icon(uint8_t control_app_id)
{
	switch (control_app_id)
    {
#ifdef APP_ID_SKAI
    case app_id_ai:
        return IMG_LOGO;
#endif
#ifdef APP_ID_RECORDER
    case app_id_recorder:
        return IMG_RECORDER;
#endif
#ifdef APP_ID_NOTE_CHATROOM
    case app_id_note:
        return IMG_NOTE;
#endif
    case app_id_calendar:
        return IMG_CALENDAR;
    case app_id_weather:
        return IMG_GROUP;
    case app_id_exercise:
        return IMG_WORKOUT;
    case app_id_flashlight:
        return IMG_FLASHLIGHT;
#ifdef APP_ID_MEDIA
    case app_id_media:
        return IMG_ITUNES;
#endif
#ifdef APP_ID_GAME_DINOSAUR
    case app_id_game_dinosaur:
        return IMG_GAME;
#endif
#ifdef APP_ID_IOT_GATE
    case app_id_iot_gate:
        return IMG_GAME;
#endif
#ifdef APP_ID_HEART_RATE
    case app_id_heart_rate:
        return IMG_HEART_RATE;
#endif
#ifdef APP_ID_ACTIVITY
    case app_id_activity:
        return IMG_ACTIVITY;
#endif
#ifdef APP_ID_PHOTO
    case app_id_photo:
        return IMG_PHOTO;
#endif
#ifdef APP_ID_CALCULATOR
    case app_id_calculator:
        return IMG_CALCULATOR;
#endif
#ifdef APP_ID_TIMER
    case app_id_timer:
        return IMG_ALARM_2;
#endif
#ifdef APP_ID_ALARM
    case app_id_alarm:
        return IMG_ALARM;
#endif
#ifdef APP_ID_SETTING
    case app_id_setting:
        return IMG_SETTINGS;
#endif
#ifdef APP_ID_MESSAGE_LIST
    case app_id_message_list:
        return IMG_MESSAGES;
#endif
#ifdef APP_ID_MOUSE
    case app_id_mouse:
        return IMG_MOUSE;
#endif
#ifdef APP_ID_TOUCHSCREEN
    case app_id_touchscreen:
        return IMG_TOUCHSCREEN;
#endif
#ifdef APP_ID_TOUCHPAD
    case app_id_touchpad:
        return IMG_TOUCHPAD;
#endif
    default:
        return IMG_LOGO;
    }
}

typedef struct
{
	lv_obj_t *right;
	lv_obj_t *left;
	lv_obj_t *top;
	lv_obj_t *bottom;
	lv_obj_t *right_img;
	lv_obj_t *left_img;
	lv_obj_t *top_img;
	lv_obj_t *bottom_img;
} quick_open_control_app_t;

control_app_list_item_t control_app_list_items[app_id_thirty];
void load_control_app_list(void);

static bool left_hand_mode = true;
static bool Title_moves_left = false;
static bool open_action_flag = false;
static bool is_hidden = false;
static bool scroll_begin = false;
static bool open_shock = false;
static uint8_t app_scroll_target_item = 1;
static uint16_t selected_item_index = 0;
static uint16_t old_selected_item_index = 0;
static uint16_t strict_selected_item_index = 0;
static lv_obj_t *selected_label;
static lv_obj_t *app_list_main_status_bar;
static uint8_t start_app = 0;

static void scroll_list(lv_obj_t *obj);
extern void scrolling_object(bool open_scrolling_object_flag);

// extern void set_lock_up(bool lock);
// extern void set_lock_down(bool lock);
// static uint8_t lock_move = 0;
static void scroll_list(lv_obj_t *obj)
{
	uint16_t min_offset = LV_HOR_RES;
	uint8_t child_cnt = obj->spec_attr->child_cnt;
	uint16_t intermediate_component_x;
	lv_coord_t x_diff = 0;
	lv_coord_t x_diff2 = 0;
	for (uint8_t i = 0; i < child_cnt; i++)
	{
		lv_obj_t *child = obj->spec_attr->children[i];
		lv_coord_t x_center = child->coords.x1 + LIST_ITEM_CONTROL_BTN_WIDTH / 2;
		x_diff = x_center - LV_VER_RES / 2;
		x_diff2 = x_diff;
		x_diff = LV_ABS(x_diff);
		lv_coord_t y_trans;
		if (x_diff >= LIST_RADIUS)
		{
			if (left_hand_mode)
			{
				y_trans = 0;
			}
			else
			{
				y_trans = LIST_RADIUS;
			}
		}
		else
		{
			if (x_diff < min_offset)
			{
				min_offset = x_diff;
				if (i + 1 <= ARRAY_SIZE(CONTROL_APP_LIST_ITEMS_DEFINITION))
				{
					selected_item_index = i;
				}
				intermediate_component_x = x_diff;
			}
			uint32_t y_sqr = LIST_RADIUS * LIST_RADIUS - x_diff * x_diff;
			lv_sqrt_res_t res;
			lv_sqrt(y_sqr, &res, 0x8000);
			if (left_hand_mode)
			{
				y_trans = LIST_RADIUS - res.i + LIST_CONTROL_ADJUST_Y_POSITION; //-300
			}
			else
			{
				y_trans = LIST_RADIUS - res.i;
			}
		}
		// lv_obj_set_style_translate_y(child, y_trans, LV_STATE_DEFAULT);
		lv_style_value_t *value = (lv_style_value_t *)child->styles->style->v_p.values_and_props;
		value[3].num = y_trans;
		// lv_obj_set_style_y(child,y_trans, LV_PART_MAIN);
		lv_obj_mark_layout_as_dirty(child);
	}
	if (selected_item_index != old_selected_item_index)
	{
		old_selected_item_index = selected_item_index;
		LOG_D("selected_app_index: %d", selected_item_index);
		lv_label_set_text(p_control_app_list_layout->p_control_app_list_title, control_app_list_items[selected_item_index].title);
		for (uint8_t i = 0; i < child_cnt; i++)
		{
			lv_obj_t *child = obj->spec_attr->children[i];
			if (i == selected_item_index)
			{
				lv_img_set_zoom(p_control_app_list_layout->p_control_app_indicator_btn[i], 256 * 1.4);
				lv_obj_align(p_control_app_list_layout->p_control_app_indicator_btn[i], LV_ALIGN_CENTER, 0, 0);
				lv_obj_clear_flag(p_control_app_list_layout->p_control_app_indicator_select_prompt[i], LV_OBJ_FLAG_HIDDEN);
			}
			else
			{
				lv_img_set_zoom(p_control_app_list_layout->p_control_app_indicator_btn[i], 256);
				lv_obj_align(p_control_app_list_layout->p_control_app_indicator_btn[i], LV_ALIGN_CENTER, 0, 0);
				lv_obj_add_flag(p_control_app_list_layout->p_control_app_indicator_select_prompt[i], LV_OBJ_FLAG_HIDDEN);
			}
		}
		motor_pattern_scrolling_app();
	}
}

static void list_window_scroll_event_cb(lv_event_t *evt)
{
	lv_obj_t *obj = evt->target;
	switch (evt->code)
	{
	case LV_EVENT_SCROLL_BEGIN:
	{
		scroll_begin = true;
		break;
	}
	case LV_EVENT_SCROLL:
	{
		scroll_list(obj);
		break;
	}
	case LV_EVENT_SCROLL_END:
	{
		// scrolling_object(false);
		break;
	}
	default:
		break;
	}
	if (obj == NULL)
	{
		return;
	}
}
static bool reset_screen = true;
static void on_item_tap(control_app_list_item_t *item)
{
	gui_app_run(item->app_id);
	reset_screen = false;
	gui_app_exit(APP_ID_CONTROL_APP);
}
static void list_item_click_event_cb(lv_event_t *evt)
{
	control_app_list_item_t *item = (control_app_list_item_t *)evt->user_data;
	LOG_D("ID: %s", item->app_id);
	on_item_tap(item);
}

static bool open_quick_app = false;

static void reset_list(void)
{
	// LOG_D("reset_list");
	if (p_control_app_list_layout->list == NULL)
	{
		return;
	}
	open_shock = false;
	app_scroll_target_item = 0;
	lv_obj_scroll_to_view(lv_obj_get_child(p_control_app_list_layout->list, 1), LV_ANIM_OFF);
	app_scroll_target_item = 1;
	scroll_list(p_control_app_list_layout->list);
	open_shock = true;
}

#ifndef Disc_Control
static void on_tap(uint8_t press)
{
	if (press == 1)
	{
		on_item_tap(&control_app_list_items[selected_item_index]);
	}
}

lv_obj_t *lv_control_app_list_layout_create(lv_obj_t *parent)
{
	RT_ASSERT(NULL == p_control_app_list_layout);
	p_control_app_list_layout = (control_app_list_layout_t *)lv_mem_alloc(sizeof(control_app_list_layout_t));
	memset(p_control_app_list_layout, 0, sizeof(control_app_list_layout_t));
	load_control_app_list();
	lv_obj_t *p_control_app_list_bg = lv_obj_create(parent);
	p_control_app_list_layout->p_control_app_list_bg = p_control_app_list_bg;
	lv_obj_set_style_bg_opa(p_control_app_list_bg, LV_OPA_0, 0);
	lv_obj_set_size(p_control_app_list_bg, LV_HOR_RES, LV_VER_RES);
	lv_obj_align(p_control_app_list_bg, LV_ALIGN_CENTER, 0, 0);
	p_control_app_list_layout->p_control_app_list_title = lv_label_create(p_control_app_list_bg);
	lv_label_set_text(p_control_app_list_layout->p_control_app_list_title, "Widgets");
	lv_obj_set_style_text_font(p_control_app_list_layout->p_control_app_list_title, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
	lv_obj_set_style_text_color(p_control_app_list_layout->p_control_app_list_title, lv_color_hex(0xFFFFFF), 0);
	lv_obj_align(p_control_app_list_layout->p_control_app_list_title, LV_ALIGN_CENTER, 0, -130);

	lv_obj_t *p_control_app_list = lv_obj_create(p_control_app_list_bg);
	p_control_app_list_layout->list = p_control_app_list;
	lv_obj_set_size(p_control_app_list, LV_HOR_RES, LV_VER_RES);
	lv_obj_set_style_bg_opa(p_control_app_list, LV_OPA_0, 0);
	lv_obj_add_flag(p_control_app_list, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_scrollbar_mode(p_control_app_list, LV_SCROLLBAR_MODE_OFF);
	lv_obj_set_scroll_dir(p_control_app_list, LV_DIR_HOR);
	lv_obj_set_scroll_snap_x(p_control_app_list, LV_SCROLL_SNAP_CENTER);
	lv_obj_set_style_pad_hor(p_control_app_list, LV_VER_RES / 2, 0);
	lv_obj_align(p_control_app_list, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_event_cb(p_control_app_list, list_window_scroll_event_cb, LV_EVENT_ALL, NULL);

	for (uint8_t i = 0; i < ARRAY_SIZE(CONTROL_APP_LIST_ITEMS_DEFINITION); i++)
	{
		lv_obj_t *item = lv_simplified_obj_create(p_control_app_list);
		lv_obj_set_size(item, LIST_ITEM_CONTROL_BTN_WIDTH, LIST_ITEM_CONTROL_BTN_HEIGHT);
		lv_obj_set_pos(item, 67 + (LIST_ITEM_CONTROL_BTN_WIDTH + LIST_ITEM_SPACING) * i, 30);
		lv_obj_add_flag(item, LV_OBJ_FLAG_CLICKABLE);

		lv_obj_t *icon_btn = lv_obj_create(item);
		lv_obj_set_size(icon_btn, LIST_ITEM_CONTROL_BTN_WIDTH, LIST_ITEM_CONTROL_BTN_WIDTH);
		lv_obj_align(icon_btn, LV_ALIGN_CENTER, 0, 0);
		lv_obj_set_style_radius(icon_btn, LV_RADIUS_CIRCLE, 0);
		lv_obj_set_style_bg_opa(icon_btn, LV_OPA_0, 0);
		p_control_app_list_layout->p_control_app_indicator_select_prompt[i] = lv_img_create(icon_btn);
		lv_img_set_src(p_control_app_list_layout->p_control_app_indicator_select_prompt[i], &select_prompt);
		lv_obj_align(p_control_app_list_layout->p_control_app_indicator_select_prompt[i], LV_ALIGN_CENTER, 0, 0);
		lv_obj_add_flag(p_control_app_list_layout->p_control_app_indicator_select_prompt[i], LV_OBJ_FLAG_HIDDEN);

		p_control_app_list_layout->p_control_app_indicator_btn[i] = lv_img_create(icon_btn);
		lv_img_set_src(p_control_app_list_layout->p_control_app_indicator_btn[i], control_app_list_items[i].icon);
		lv_obj_align(p_control_app_list_layout->p_control_app_indicator_btn[i], LV_ALIGN_CENTER, 0, 0);
		lv_obj_t *app_icon = p_control_app_list_layout->p_control_app_indicator_btn[i];
		lv_obj_add_event_cb(icon_btn, list_item_click_event_cb, LV_EVENT_CLICKED, (void *)&control_app_list_items[i]);
		if (i == selected_item_index)
		{
			lv_img_set_zoom(p_control_app_list_layout->p_control_app_indicator_btn[i], 256 * 1.4);
			lv_obj_align(p_control_app_list_layout->p_control_app_indicator_btn[i], LV_ALIGN_CENTER, 0, 0);
			lv_obj_clear_flag(p_control_app_list_layout->p_control_app_indicator_select_prompt[i], LV_OBJ_FLAG_HIDDEN);
		}
	}
	reset_list();
	// myLancher[app_index_app_list].reset_list = reset_list;
	// myLancher[app_index_app_list].reset_list();

	lv_event_send(p_control_app_list, LV_EVENT_SCROLL, NULL);
	// myLancher[app_index_app_list].on_tap = on_tap;

	return p_control_app_list_bg;
}
#else
static lv_obj_t *mouse_mode_btn;
static void mouse_mode_btn_event_cb(lv_event_t *e)
{
	gui_app_run(APP_ID_MOUSE);
	gui_app_exit(APP_ID_CONTROL_APP);
}

static void recorder_btn_event_cb(lv_event_t *e)
{
	gui_app_run(APP_ID_RECORDER);
	gui_app_exit(APP_ID_CONTROL_APP);
}

static void flishlight_icon_event_cb(lv_event_t *e)
{
	gui_app_run(APP_ID_FLASHLIGHT);
	gui_app_exit(APP_ID_CONTROL_APP);
}

static void qrcode_btn_event_cb(lv_event_t *e)
{
	AppIntent intent = {0};
	strcpy(intent.app_id, "JA_app1");
	watch_run_app_by_intent(&intent);
	gui_app_exit(APP_ID_CONTROL_APP);
}

static lv_obj_t *control_tool[8];
static lv_obj_t *control_text;
static int8_t button_selection_index = -1; // 用於記錄當前選中的按鈕索引
static void on_tap(uint8_t press)
{
	if (press == 1 && button_selection_index >= 0)
	{
		if (button_selection_index == 0 || button_selection_index == 1 || button_selection_index == 7)
		{
			lv_event_send(lv_obj_get_child(control_tool[button_selection_index], 0), LV_EVENT_CLICKED, NULL);
		}
		else
		{
			lv_event_send(control_tool[button_selection_index], LV_EVENT_CLICKED, NULL);
		}
	}
}

static lv_obj_t *position_indicator; // 位置指示點
static lv_obj_t *control_icon_bg;	 // 控制器指針
static rt_bool_t dndmode_enabled = RT_FALSE;
// 記錄每個控制工具上次的zoom_factor，用於檢測位置變化
static uint16_t last_zoom_factors[8] = {0};
static void set_dnd_mode(bool dnd_mode)
{
	if (dndmode_enabled != dnd_mode)
	{
		dndmode_enabled = dnd_mode;
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
		watch_system_interact(WATCH_DND_MODE_SET, &dndmode_enabled);
#endif
		if (dndmode_enabled)
		{
			lv_obj_set_style_bg_opa(control_tool[6], LV_OPA_90, 0);
			lv_obj_set_style_bg_color(control_tool[6], lv_color_hex(0xCECECE), 0);
		}
		else
		{
			lv_obj_set_style_bg_opa(control_tool[6], LV_OPA_10, 0);
			lv_obj_set_style_bg_color(control_tool[6], lv_color_hex(0xFFFFFF), 0);
		}
	}
}
static void dnd_mode_btn_event_cb(lv_event_t *e)
{
	if (control_tool[6] == NULL)
	{
		return;
	}
	bool mode = !dndmode_enabled;
	set_dnd_mode(mode);
}

static void change_icon_image(lv_obj_t *icon, const void *new_img_src)
{
	/* Get the image object from the button */
	lv_obj_t *img = lv_obj_get_child(icon, 0);

	/* Change the image source */
	lv_img_set_src(img, new_img_src);
}

static void handle_control_media_play_state(void *param)
{
	if (lv_obj_is_valid(control_tool[0]) == false)
	{
		return;
	}
	bool media_state = *(bool *)param;
	change_icon_image(lv_obj_get_child(control_tool[0], 0), media_state ? &img_media_pause : &img_media_play);
}

static void reset_tools_selection(void)
{
	// 重置位置記錄，確保下次更新時會生效
	for (int i = 0; i < 8; i++)
	{
		last_zoom_factors[i] = 0;
	}

	for (int i = 0; i < 8; i++)
	{
		if (control_tool[i] != NULL)
		{
			// if (i <= 1 || i == 7)
			// {
			// 	lv_img_set_zoom(lv_obj_get_child(lv_obj_get_child(control_tool[i], 0), 0), 256 * 0.5);
			// }
			// else
			// {
			// 	lv_img_set_zoom(lv_obj_get_child(control_tool[i], 0), 256 * 0.9); // 恢復按鈕大小
			// }
			// lv_obj_set_style_border_width(control_tool[i], 0, 0); // 恢復邊框寬度
			lv_obj_set_style_img_opa(control_tool[i], LV_OPA_0, 0); // 恢復背景透明度
		}
	}
	// 移除這裡的 control_icon_bg 位置重置，讓它持續跟隨手勢移動
	// if (control_icon_bg != NULL)
	// {
	// 	lv_obj_align(control_icon_bg, LV_ALIGN_CENTER, 0, 0);
	// }
}
static lv_obj_t *control_app_create(lv_obj_t *par, const lv_img_dsc_t *img_src, lv_coord_t w, lv_coord_t h, lv_event_cb_t event_cb)
{
	lv_obj_t *control_app_bg = lv_img_create(par);
	lv_img_set_src(control_app_bg, &control_selection_bg);
	lv_obj_set_size(control_app_bg, 110, 110);
	// common_image_button(control_app_bg, img_src, w, h, event_cb);
	lv_obj_add_event_cb(control_app_bg, event_cb, LV_EVENT_CLICKED, NULL);

	lv_obj_t *img = lv_img_create(control_app_bg);
	lv_img_set_src(img, img_src);
	lv_obj_center(img);
	return control_app_bg;
}

static lv_obj_t *control_charapp_create(lv_obj_t *par, const char *img_src, lv_coord_t w, lv_coord_t h, lv_event_cb_t event_cb)
{
	lv_obj_t *control_app_bg = lv_img_create(par);
	lv_img_set_src(control_app_bg, &control_selection_bg);
	lv_obj_set_size(control_app_bg, 110, 110);
	// common_image_button(control_app_bg, img_src, w, h, event_cb);
	lv_obj_add_event_cb(control_app_bg, event_cb, LV_EVENT_CLICKED, NULL);

	lv_obj_t *img = lv_img_create(control_app_bg);
	lv_img_set_src(img, img_src);
	lv_obj_center(img);
	return control_app_bg;
}

extern lv_obj_t *media_prev_btn_create(lv_obj_t *parent);
extern lv_obj_t *media_next_btn_create(lv_obj_t *parent);
extern lv_obj_t *media_play_pause_btn_create(lv_obj_t *parent);
lv_obj_t *lv_control_app_list_layout_create(lv_obj_t *parent)
{
	lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE); // 清除滾動標誌
	lv_obj_t *bg = lv_obj_create(parent);
	lv_obj_set_size(bg, 540, 540);
	lv_obj_center(bg);
	lv_obj_set_style_bg_opa(bg, LV_OPA_0, 0);

	// 計算8個按鈕在圓形邊緣的位置
	const int radius = 178; // 調整半徑讓按鈕適合在466x466區域內
	const int center_x = 233;
	const int center_y = 233;
	const double pi = 3.14159265359;
	const double angle_step = 2 * pi / 8; // 360度/8個按鈕

	control_icon_bg = lv_obj_create(bg);
	lv_obj_set_size(control_icon_bg, 466, 466);
	lv_obj_align(control_icon_bg, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_bg_color(control_icon_bg, lv_color_hex(0x000000), 0);

	// 創建位置指示點
	position_indicator = lv_obj_create(bg);
	lv_obj_set_size(position_indicator, 200, 200);
	lv_obj_align(position_indicator, LV_ALIGN_CENTER, 0, 0); // 初始位置在中心
	lv_obj_set_style_bg_opa(position_indicator, LV_OPA_0, 0);
	lv_obj_add_flag(position_indicator, LV_OBJ_FLAG_HIDDEN); // 初始時隱藏
	lv_obj_t *control_cursor = lv_img_create(position_indicator);
	lv_img_set_src(control_cursor, &control_cursor_img);
	lv_obj_set_size(control_cursor, 200, 200);
	lv_obj_align(control_cursor, LV_ALIGN_CENTER, 0, 0); // 將指示點對齊到中心

	lv_obj_t *control_background = lv_img_create(bg);
	lv_img_set_src(control_background, &control_background_img);
	lv_obj_set_size(control_background, 233, 233);
	lv_obj_align(control_background, LV_ALIGN_CENTER, 0, 0); // 將背景圖片對齊到指示點中心
	control_text = lv_label_create(bg);
	lv_label_set_text(control_text, "Control");
	lv_obj_set_style_text_font(control_text, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
	lv_obj_set_style_text_color(control_text, lv_color_hex(0xFFFFFF), 0);
	lv_label_set_long_mode(control_text, LV_LABEL_LONG_DOT);
	lv_obj_set_height(control_text, 150);
	lv_obj_set_width(control_text, 250);
	lv_obj_set_style_text_align(control_text, LV_TEXT_ALIGN_CENTER, 0);

	lv_obj_align(control_text, LV_ALIGN_CENTER, 0, 40); //

	// 創建按鈕並設置位置
	control_tool[0] = media_play_pause_btn_create(control_icon_bg); // media_play_pause_btn_create(bg);
	control_tool[7] = media_prev_btn_create(control_icon_bg);		// media_prev_btn_create(bg);
	control_tool[1] = media_next_btn_create(control_icon_bg);		// media_next_btn_create(bg);
	control_tool[2] = control_app_create(control_icon_bg, &mouse_mode_icon, 110, 110, mouse_mode_btn_event_cb);
	control_tool[3] = control_app_create(control_icon_bg, &micro_icon, 110, 110, recorder_btn_event_cb);
	control_tool[4] = control_charapp_create(control_icon_bg, FLISHLIGHT_ICON, 110, 110, flishlight_icon_event_cb);
	control_tool[5] = control_app_create(control_icon_bg, &icon_qrcode, 110, 110, qrcode_btn_event_cb);
	control_tool[6] = control_app_create(control_icon_bg, &icon_dnd_mode, 110, 110, dnd_mode_btn_event_cb);

	// 將按鈕均勻分佈在圓形邊緣
	for (int i = 0; i < 8; i++)
	{
		lv_obj_clear_flag(control_tool[i], LV_OBJ_FLAG_SCROLLABLE); // 清除滾動標誌
		if (control_tool[i] != NULL)
		{
			double angle = i * angle_step - pi / 2;				// 從頂部開始(-90度)
			int x = center_x + (int)(radius * cos(angle)) - 55; // 減去按鈕寬度的一半
			int y = center_y + (int)(radius * sin(angle)) - 55; // 減去按鈕高度的一半
			lv_obj_set_pos(control_tool[i], x, y);
		}
		lv_obj_set_style_border_color(control_tool[i], lv_color_hex(0xFFFFFF), 0); // 白色邊框
		lv_obj_set_style_border_width(control_tool[i], 0, 0);
	}

	reset_tools_selection();

	return bg;
}

static char *media_title_text = "Media";
static void handle_media_title(void *param)
{
	if (lv_obj_is_valid(control_text) == false)
	{
		return;
	}
	media_title_text = (char *)param;
	if (media_title_text)
	{
		lv_label_set_text(control_text, media_title_text);
	}
}

// 獲取控制app的名稱
static const char *get_control_app_name(int tool_index)
{
	switch (tool_index)
	{
	case 0:
		return media_title_text;
	case 1:
		return media_title_text;
	case 2:
		return "Mouse";
	case 3:
		return "Recorder";
	case 4:
		return "Flashlight";
	case 5:
		return "QR Code";
	case 6:
		return "DND";
	case 7:
		return media_title_text;
	default:
		return "Control";
	}
}

/**
 * @brief Set the icon size of a control widget based on a zoom factor.
 *
 * 根據給定的 zoom_factor 調整指定控制元件的圖示大小。
 * zoom_factor 的範圍為最小 415，最大 50。
 * 只有當新的位置與上次位置距離超過5時才會更新大小。
 *
 * @param i 控制元件的索引 (index of the control widget in control_tool array)
 * @param zoom_factor 縮放因子，範圍為 415 (最小) 到 50 (最大)
 */
static void set_control_icon_size(uint8_t i, uint16_t zoom_factor)
{
	if (control_tool[i] != NULL && i < 8)
	{
		// 檢查與上次位置的距離差異是否超過5
		uint16_t distance_diff = (zoom_factor > last_zoom_factors[i]) ? (zoom_factor - last_zoom_factors[i]) : (last_zoom_factors[i] - zoom_factor);

		// 只有距離變化超過5或是第一次設定時才更新
		if (distance_diff > 10 || last_zoom_factors[i] == 0)
		{
			// 更新記錄的位置
			last_zoom_factors[i] = zoom_factor;

			// 反向映射: 415(最小) ~ 50(最大)
			float ratio_conversion = 1.0f;
			// Clamp zoom_factor to [50, 415]
			if (zoom_factor < 50)
				zoom_factor = 50;
			if (zoom_factor > 415)
				zoom_factor = 415;
			// 反向計算
			float t = (float)(415 - zoom_factor) / (415 - 50); // t: 0~1, 0=最小, 1=最大
			if (i <= 1 || i == 7)
			{
				ratio_conversion = 0.2f + t * (1.1f - 0.2f);
				lv_img_set_zoom(lv_obj_get_child(lv_obj_get_child(control_tool[i], 0), 0), 256 * ratio_conversion);
			}
			else
			{
				ratio_conversion = 0.4f + t * (1.2f - 0.4f);
				lv_img_set_zoom(lv_obj_get_child(control_tool[i], 0), 256 * ratio_conversion);
			}
		}
	}
}

static int prev_icon_bg_offset_x = 0; // 上次背景偏移量的x分量
static int prev_icon_bg_offset_y = 0; // 上次背景偏移量的y
static void button_selection(gesture_position_t gesture_position)
{
	// 將輸入的xy與畫面xy對調
	const int input_x = gesture_position.gesture_position_x;
	const int input_y = 466 - gesture_position.gesture_position_y;

	// LOG_D("button_selection input: x=%d, y=%d (swapped from original y=%d, x=%d)",
	// 	  input_x, input_y, gesture_position.gesture_position_x, gesture_position.gesture_position_y);

	// 圓形佈局參數（與創建時相同）
	const int radius = 183;
	const int center_x = 233;
	const int center_y = 233;
	const double pi = 3.14159265359;
	const double angle_step = 2 * pi / 8;

	// 更新位置指示點
	if (position_indicator != NULL)
	{
		// 顯示指示點
		lv_obj_clear_flag(position_indicator, LV_OBJ_FLAG_HIDDEN);

		// 將手勢輸入範圍(466x466)映射到圓盤範圍內
		// 計算手勢相對於中心的位置（範圍：-233 到 +233）
		int gesture_relative_x = input_x - 233;
		int gesture_relative_y = input_y - 233;

		// 計算縮放比例：讓指示點移動範圍更小
		// 限制指示點在更小的範圍內移動
		const double scale_factor = 160.0 / 233.0; // 縮小移動範圍：30像素範圍 / 手勢半徑233

		// 應用縮放
		int relative_x = (int)(gesture_relative_x * scale_factor);
		int relative_y = (int)(gesture_relative_y * scale_factor);

		// 限制在更小的圓盤內
		double distance_from_center = sqrt(relative_x * relative_x + relative_y * relative_y);
		const int max_indicator_radius = 160; // 最大移動半徑縮小到30像素
		if (distance_from_center > max_indicator_radius)
		{
			double angle = atan2(relative_y, relative_x);
			relative_x = (int)(max_indicator_radius * cos(angle));
			relative_y = (int)(max_indicator_radius * sin(angle));
		}

		// 如果沒有選中任何按鈕，指示點跟隨手勢移動
		// 如果選中了按鈕，指示點將在後面設置到按鈕位置
		if (button_selection_index == -1)
		{
			// 設置指示點位置（相對於圓盤中心的偏移）
			lv_obj_align(position_indicator, LV_ALIGN_CENTER, relative_x, relative_y);
		}

		// 讓 control_icon_bg 往反方向移動一點點，限制在20像素內
		if (control_icon_bg != NULL)
		{
			// 計算反方向移動的偏移量，限制在20像素內
			const int max_bg_offset = 20;
			int bg_offset_x = -(relative_x * max_bg_offset) / 160; // 反方向且按比例縮小
			int bg_offset_y = -(relative_y * max_bg_offset) / 160; // 反方向且按比例縮小

			// 限制偏移量不超過20像素
			if (bg_offset_x > max_bg_offset)
				bg_offset_x = max_bg_offset;
			if (bg_offset_x < -max_bg_offset)
				bg_offset_x = -max_bg_offset;
			if (bg_offset_y > max_bg_offset)
				bg_offset_y = max_bg_offset;
			if (bg_offset_y < -max_bg_offset)
				bg_offset_y = -max_bg_offset;

			if (abs(prev_icon_bg_offset_x - bg_offset_x) > 2 || abs(prev_icon_bg_offset_y - bg_offset_y) > 2)
			{
				prev_icon_bg_offset_x = bg_offset_x; // 更新上次偏移量
				prev_icon_bg_offset_y = bg_offset_y; // 更新上次偏移量
				lv_obj_align(control_icon_bg, LV_ALIGN_CENTER, bg_offset_x, bg_offset_y);
			}
		}

		// LOG_D("Position indicator at: center_offset(%d, %d), gesture_relative(%d, %d), distance: %.1f",
		// 	  relative_x, relative_y, gesture_relative_x, gesture_relative_y, distance_from_center);
	}

	int closest_tool_index = -1;
	double min_distance = 999999.0; // 設置一個很大的初始值

	// 檢查每個按鈕的位置並計算距離
	for (int i = 0; i < 8; i++)
	{
		if (control_tool[i] != NULL)
		{
			// 計算按鈕的理論位置（與創建時相同的計算）
			double angle = i * angle_step - pi / 2;
			int button_x = center_x + (int)(radius * cos(angle));
			int button_y = center_y + (int)(radius * sin(angle));

			// 計算輸入座標與按鈕中心的距離
			double distance = sqrt((input_x - button_x) * (input_x - button_x) +
								   (input_y - button_y) * (input_y - button_y));
			// LOG_D("Tool[%d] position: (%d, %d), distance: %.2f", i, button_x, button_y, distance);
			set_control_icon_size(i, distance); // 根據距離調整圖標大小
			// 如果這是最近的按鈕，記錄它
			if (distance < min_distance)
			{
				min_distance = distance;
				closest_tool_index = i;
			}
		}
	}

	// 計算手勢距離中心的距離
	double distance_from_center = sqrt((input_x - center_x) * (input_x - center_x) + (input_y - center_y) * (input_y - center_y));

	// 如果距離中心不超過60像素，不選取任何按鈕
	if (distance_from_center <= 150)
	{
		if (button_selection_index != -1)
		{
			reset_tools_selection();
			button_selection_index = -1;
			// 恢復顯示預設文字
			if (control_text != NULL)
			{
				lv_label_set_text(control_text, "Control");
			}
			// 移除這裡的 control_icon_bg 位置重置，讓它持續跟隨手勢移動
			// if (control_icon_bg != NULL)
			// {
			// 	lv_obj_align(control_icon_bg, LV_ALIGN_CENTER, 0, 0);
			// }
		}
	}
	else if (button_selection_index != closest_tool_index)
	{
		// 重置之前選中的按鈕
		reset_tools_selection();
		button_selection_index = closest_tool_index;
		if (closest_tool_index >= 0)
		{
			motor_pattern_scrolling_app();
			// lv_obj_set_style_border_width(control_tool[closest_tool_index], 2, 0); // 設置選中按鈕的邊框寬度
			lv_obj_set_style_img_opa(control_tool[closest_tool_index], LV_OPA_100, 0); // 設置選中按鈕的背景透明度
			// set_control_icon_size(closest_tool_index, 1.5f); // 設置選中按鈕的圖標大小

			// 將位置指示點設置到選中按鈕的位置
			if (position_indicator != NULL)
			{
				// 計算選中按鈕的理論位置（與創建時相同的計算）
				double angle = closest_tool_index * angle_step - pi / 2;
				int button_x = center_x + (int)(radius * cos(angle));
				int button_y = center_y + (int)(radius * sin(angle));

				// 將按鈕位置轉換為相對於中心的偏移
				int button_relative_x = button_x - center_x;
				int button_relative_y = button_y - center_y;

				// 先計算 control_icon_bg 的反方向偏移量
				int bg_offset_x = 0;
				int bg_offset_y = 0;
				if (control_icon_bg != NULL)
				{
					// 計算反方向移動的偏移量，限制在20像素內
					const int max_bg_offset = 20;
					bg_offset_x = -(button_relative_x * max_bg_offset) / 183; // 反方向且按比例縮小
					bg_offset_y = -(button_relative_y * max_bg_offset) / 183; // 反方向且按比例縮小

					// 限制偏移量不超過20像素
					if (bg_offset_x > max_bg_offset) bg_offset_x = max_bg_offset;
					if (bg_offset_x < -max_bg_offset) bg_offset_x = -max_bg_offset;
					if (bg_offset_y > max_bg_offset) bg_offset_y = max_bg_offset;
					if (bg_offset_y < -max_bg_offset) bg_offset_y = -max_bg_offset;

				// 	// 設置 control_icon_bg 的偏移
				// 	lv_obj_align(control_icon_bg, LV_ALIGN_CENTER, bg_offset_x, bg_offset_y);
				}
				// 設置位置指示點到按鈕位置，考慮 control_icon_bg 的偏移
				// 由於 control_icon_bg 向反方向偏移了，我們需要補償這個偏移來讓指示點正確指向按鈕
				lv_obj_align(position_indicator, LV_ALIGN_CENTER, button_relative_x + bg_offset_x, button_relative_y + bg_offset_y);
			}

			// 更新中間顯示的app名稱
			if (control_text != NULL)
			{
				lv_label_set_text(control_text, get_control_app_name(closest_tool_index));
			}
			if (closest_tool_index == 0 || closest_tool_index == 1 || closest_tool_index == 7)
			{
				lvgl_msg_handler.handle_app_media_title = handle_media_title;
			}
			else
			{
				lvgl_msg_handler.handle_app_media_title = NULL;
			}
		}
	}
}
#endif

static uint8_t last_scroll_time = 0;
void scroll_control_app_list_to_index(uint8_t index)
{
	if (p_control_app_list_layout->list == NULL)
		return;
	app_scroll_target_item = index;
	LOG_D("app_scroll_target_item: %d", app_scroll_target_item);
	extern void set_scroll_anim_time(bool init);
	uint32_t now = rt_tick_get_millisecond();
	if (now - last_scroll_time < 100)
	{
		set_scroll_anim_time(true);
	}
	last_scroll_time = now;
	lv_disp_trig_activity(NULL);
	lv_obj_scroll_to_view(lv_obj_get_child(p_control_app_list_layout->list, index), LV_ANIM_ON);
	set_scroll_anim_time(false);
	scroll_list(p_control_app_list_layout->list);
}

void control_app_list_scroll_to_app(bool up)
{
	int target = 0;
	if (up)
	{
		target = app_scroll_target_item + 1;
	}
	else
	{
		target = app_scroll_target_item - 1;
	}
	if (target >= 0 && target < ARRAY_SIZE(CONTROL_APP_LIST_ITEMS_DEFINITION))
	{
		scroll_control_app_list_to_index(target);
	}
}

control_app_list_item_t map_control_app_id(uint8_t app_id)
{
	control_app_list_item_t item;
	switch (app_id)
    {
#ifdef APP_ID_SKAI
    case app_id_ai:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(skai_ai, "AI");
        item.icon = IMG_LOGO;
        item.app_id = APP_ID_SKAI;
    }
    break;
#endif
#ifdef APP_ID_RECORDER
    case app_id_recorder:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(recorder, "Recorder");
        item.icon = IMG_RECORDER;
        item.app_id = APP_ID_RECORDER;
    }
    break;
#endif
#ifdef APP_ID_NOTE_CHATROOM
    case app_id_note:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(note, "Note");
        item.icon = IMG_NOTE;
        item.app_id = APP_ID_NOTE_CHATROOM;
    }
    break;
#endif

#ifdef APP_ID_CALENDAR
    case app_id_calendar:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(calendar, "Calendar");
        item.icon = IMG_CALENDAR;
        item.app_id = APP_ID_CALENDAR;
        break;
    }
#endif

#ifdef APP_ID_WEATHER
    case app_id_weather:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(weather, "Weather");
        item.icon = IMG_GROUP;
        item.app_id = APP_ID_WEATHER;
        break;
    }
#endif

#ifdef APP_ID_EXERCISE
    case app_id_exercise:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(exercise, "Exercise");
        item.icon = IMG_WORKOUT;
        item.app_id = APP_ID_EXERCISE;
        break;
    }
#endif

#ifdef APP_ID_FLASHLIGHT
    case app_id_flashlight:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(flashlight, "Flashlight");
        item.icon = IMG_FLASHLIGHT;
        item.app_id = APP_ID_FLASHLIGHT;
        break;
    }
#endif

#ifdef APP_ID_MEDIA
    case app_id_media:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(media, "Media");
        item.icon = IMG_ITUNES;
        item.app_id = APP_ID_MEDIA;
    }
    break;
#endif
#ifdef APP_ID_PHOTO
    case app_id_photo:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(photo, "Photo");
        item.icon = IMG_PHOTO;
        item.app_id = APP_ID_PHOTO;
    }
    break;
#endif
#ifdef APP_ID_GAME_DINOSAUR
    case app_id_game_dinosaur:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(game, "Game");
        item.icon = IMG_GAME;
        item.app_id = APP_ID_GAME_DINOSAUR;
    }
    break;
#endif

#ifdef APP_ID_IOT_GATE
    case app_id_iot_gate:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(iot_gate, "iot_gate");
        item.icon = IMG_GAME;
        item.app_id = APP_ID_IOT_GATE;
    }
    break;
#endif

#ifdef APP_ID_HEART_RATE
    case app_id_heart_rate:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(heart_rate, "Heart Rate");
        item.icon = IMG_HEART_RATE;
        item.app_id = APP_ID_HEART_RATE;
        break;
    }
#endif

#ifdef APP_ID_ACTIVITY
    case app_id_activity:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(activity, "Activity");
        item.icon = IMG_ACTIVITY;
        item.app_id = APP_ID_ACTIVITY;
        break;
    }
#endif

#ifdef APP_ID_CALCULATOR
    case app_id_calculator:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(calculator, "Calculator");
        item.icon = IMG_CALCULATOR;
        item.app_id = APP_ID_CALCULATOR;
        break;
    }
#endif

#ifdef APP_ID_TIMER
    case app_id_timer:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(timer, "Timer");
        item.icon = IMG_ALARM_2;
        item.app_id = APP_ID_TIMER;
        break;
    }

#endif

#ifdef APP_ID_ALARM
    case app_id_alarm:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(alarm, "Alarm");
        item.icon = IMG_ALARM;
        item.app_id = APP_ID_ALARM;
        break;
    }
#endif

#ifdef APP_ID_SETTING
    case app_id_setting:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(setting, "Setting");
        item.icon = IMG_SETTINGS;
        item.app_id = APP_ID_SETTING;
        break;
    }
#endif

#ifdef APP_ID_MESSAGE_LIST
    case app_id_message_list:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(message, "Message");
        item.icon = IMG_MESSAGES;
        item.app_id = APP_ID_MESSAGE_LIST;
        break;
    }
#endif

#ifdef APP_ID_MOUSE
    case app_id_mouse:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(mouse, "Mouse");
        item.icon = IMG_MOUSE;
        item.app_id = APP_ID_MOUSE;
        break;
    }
#endif

#ifdef APP_ID_TOUCHSCREEN
    case app_id_touchscreen:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(touchscreen, "Touchscreen");
        item.icon = &img_touchscreen;
        item.app_id = APP_ID_TOUCHSCREEN;
        break;
    }
#endif

#ifdef APP_ID_TOUCHPAD
    case app_id_touchpad:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(touchpad, "Touchpad");
        item.icon = &img_touchpad;
        item.app_id = APP_ID_TOUCHPAD;
        break;
    }
#endif

    default:
    {
        item.title = "Unknown";
        item.icon = IMG_LOGO;
        item.app_id = APP_ID_MAIN;
        break;
    }
    }
	return item;
}

void load_control_app_list(void)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(CONTROL_APP_LIST_ITEMS_DEFINITION); i++)
	{
		control_app_list_items[i] = map_control_app_id(CONTROL_APP_LIST_ITEMS_DEFINITION[i]);
	}
}

static rt_int32_t init(lv_obj_t *parent)
{
	cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);
	screen_rotate_to_90_degree();
	lv_control_app_list_layout_create(parent);
	return RT_EOK;
}

static bool pause_control_app_list = true;

rt_int32_t control_app_list_resume(void)
{
#ifndef Disc_Control
	if (pause_control_app_list == false)
	{
		return RT_EOK;
	}
	pause_control_app_list = false;
	LOG_I("app_list_resume");
#ifdef USE_QUICK_OPEN_AI
	open_vibration = true;
#endif
	check_is_at_app_list();
	lvgl_msg_handler.handle_tap_indicator = on_tap;
	request_weather_within_six_hours(false);
	request_calendar_on_mobile(false);
	return RT_EOK;
#else
	check_is_at_app_list();
	extern void set_movement_scale_ratio(float ratio);
	set_movement_scale_ratio(3.0f); // 設置移動縮放比例
	reset_control_pos();
	set_open_control_options(true);
	lvgl_msg_handler.handle_app_media_play_state = handle_control_media_play_state;
	lvgl_msg_handler.handle_widgets_control = button_selection;
	lvgl_msg_handler.handle_tap_indicator = on_tap;
	return RT_EOK;
#endif
}

rt_int32_t control_app_list_pause(void)
{
#ifndef Disc_Control
	if (pause_control_app_list == true)
	{
		return RT_EOK;
	}
	pause_control_app_list = true;
	lvgl_msg_handler.handle_tap_indicator = NULL;
	check_is_at_app_list();
	LOG_I("control_app_list_pause");
	return RT_EOK;
#else
	set_open_control_options(false);
	extern void set_movement_scale_ratio(float ratio);
	set_movement_scale_ratio(1.0f); // 設置移動縮放比例
	if (lvgl_msg_handler.handle_widgets_control == button_selection)
	{
		lvgl_msg_handler.handle_widgets_control = NULL;
	}
	// 隱藏位置指示點
	if (position_indicator != NULL)
	{
		lv_obj_add_flag(position_indicator, LV_OBJ_FLAG_HIDDEN);
	}
	if (lvgl_msg_handler.handle_app_media_play_state == handle_control_media_play_state)
	{
		lvgl_msg_handler.handle_app_media_play_state = NULL;
	}
	check_is_at_app_list();
	return RT_EOK;
#endif
}

rt_int32_t control_app_list_deinit(void)
{
#ifndef Disc_Control
	if (reset_screen)
	{
		screen_rotate_back_to_original_direction();
		reset_screen = true;
	}
	if (p_control_app_list_layout)
	{
		if (p_control_app_list_layout->quick_open_app_bg)
		{
			lv_obj_del(p_control_app_list_layout->quick_open_app_bg);
		}
		if (p_control_app_list_layout->list)
		{
			lv_obj_del(p_control_app_list_layout->list);
		}
		if (p_control_app_list_layout->p_control_app_list_bg)
		{
			lv_obj_del(p_control_app_list_layout->p_control_app_list_bg);
		}
		lv_mem_free(p_control_app_list_layout);
		p_control_app_list_layout = NULL;
	}
	LOG_I("control_app_list_deinit");
	pause_control_app_list = true;
	return RT_EOK;
#else
	screen_rotate_back_to_original_direction();
	return RT_EOK;
#endif
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
	switch (msg)
	{
	case GUI_APP_MSG_ONSTART:
	{
		lv_obj_t *scr = lv_scr_act();
		init(scr);
	}
	break;

	case GUI_APP_MSG_ONRESUME:
		control_app_list_resume();
		break;

	case GUI_APP_MSG_ONPAUSE:
		control_app_list_pause();
		break;

	case GUI_APP_MSG_ONSTOP:
		control_app_list_deinit();
		break;
	default:
		break;
	}
}

static int app_main(intent_t i)
{
	gui_app_regist_msg_handler(APP_ID_CONTROL_APP, msg_handler);

	return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(widgets), SKAIWALKICON, APP_ID_CONTROL_APP, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/