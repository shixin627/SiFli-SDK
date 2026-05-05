/**
 ******************************************************************************
 * @file   app_calendar_straight.c
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
#include <stdio.h>
#include <stdint.h>
#include "lvgl.h"
#include "lv_simplified_obj.h"
#include "lv_ext_resource_manager.h"
#include "watch_global_data.h"
#include "app_mainmenu.h"
#include "app_clock_main.h"
#ifdef BSP_USING_GESTURE_HANDLER
#include "gesture_handler.h"
#endif
#include "custom_trans_anim.h"
#include "common_widget.h"
#include "bloc_calendar.h"
#include "bloc_peripheral.h"
#include "bloc_weather.h"
#include "bloc_motion_tracking.h"
#include <math.h>
#include "ui_handler.h"
#include "ui_img_helper.h"

#ifdef APP_ID_CALENDAR

#define DBG_TAG "app_calendar_straight"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

LV_IMG_DECLARE(img_clock);
LV_IMG_DECLARE(img_settings);
LV_IMG_DECLARE(img_messages);
LV_IMG_DECLARE(weather);
LV_IMG_DECLARE(football_card);

#define LIST_ITEM_WIDTH (88)
#define LIST_ITEM_HEIGHT (88)

typedef struct
{
	lv_obj_t *main_window;
	lv_obj_t *selected_label;
	lv_obj_t *direction_icon[7][4];
	lv_obj_t *calendar_all_day_tailview[7];
	lv_obj_t *calendar_day_list[7];
} app_calendar_t;

static app_calendar_t *p_app_calendar = NULL;
static calendar_model_t one_calendar_data;
calendar_model_t *get_on_coming_calendar_model(void)
{
	return &one_calendar_data;
}

// Forward declarations
static lv_obj_t *calendar_tileview_builder(lv_obj_t *parent);
extern lv_obj_t *lv_calendar_list_layout_create(lv_obj_t *parent, uint8_t page_index);

/**
 * @brief Convert UTC time structure to tm structure
 * @param src Source UTC time structure
 * @param dst Destination tm structure
 */
void convert_to_tm(T_UTC_TIME *src, struct tm *dst)
{
	dst->tm_year = src->year - 1900;
	dst->tm_mon = src->month - 1;
	dst->tm_mday = src->day;
	dst->tm_hour = src->hour;
	dst->tm_min = src->minutes;
	dst->tm_sec = src->seconds;
	dst->tm_isdst = -1;
}

/**
 * @brief Event handler for calendar navigation
 */
static void set_button_selection(bool reset);
static uint8_t old_active_pos = 0;
void calendar_event(lv_event_t *e)
{
	lv_event_code_t code = lv_event_get_code(e);
	lv_obj_t *obj = lv_event_get_target(e);

	if (code == LV_EVENT_VALUE_CHANGED)
	{
		rt_uint32_t active_pos = (rt_uint32_t)lv_event_get_param(e);
		if (active_pos != old_active_pos)
		{
			old_active_pos = active_pos;
			set_button_selection(true);
			force_release_finger();
		}
	}
}

/**
 * @brief Create the calendar tileview
 * @param parent Parent object
 * @return The created tileview object
 */
static lv_obj_t *calendar_tileview_builder(lv_obj_t *parent)
{
	lv_obj_t *pages[7];
	lv_obj_t *tileview;
	// Create tileview
	tileview = lv_tileview_create(parent);
	lv_obj_add_event_cb(tileview, calendar_event, LV_EVENT_ALL, NULL);
	lv_obj_set_style_bg_color(tileview, lv_color_hex(0xFFFFFF), 0);
	lv_obj_set_style_bg_opa(tileview, LV_OPA_0, 0);
	lv_obj_clear_flag(tileview, LV_OBJ_FLAG_SCROLL_ONE);
	lv_obj_set_scrollbar_mode(tileview, LV_SCROLLBAR_MODE_OFF);
	lv_obj_set_style_bg_opa(tileview, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Create tiles for each day
	for (rt_uint16_t i = 0; i < 7; i++)
	{
		pages[i] = lv_tileview_add_tile(tileview, i, 0, LV_DIR_HOR);
		common_black_bg(pages[i]);

		// Set up page styling
		lv_obj_set_style_bg_opa(pages[i], LV_OPA_0, LV_PART_MAIN | LV_STATE_DEFAULT);
		lv_color_t color = (i % 2) ? lv_color_hex(0x000000) : lv_color_hex(0xFFFFFFF);
		lv_obj_set_size(pages[i], LV_HOR_RES_MAX, LV_VER_RES_MAX);
		lv_obj_set_pos(pages[i], (LV_HOR_RES_MAX * i), 0);
		lv_obj_set_scrollbar_mode(pages[i], LV_SCROLLBAR_MODE_OFF);

		// Store reference to the page
		p_app_calendar->calendar_all_day_tailview[i] = pages[i];
		// Create the calendar list layout for this day
		p_app_calendar->calendar_day_list[i] = lv_calendar_list_layout_create(pages[i], i);
		// Add scroll indicator if not the last page
		if (i != 6)
		{
			lv_obj_t *down_grip = lv_obj_create(pages[i]);
			lv_obj_set_size(down_grip, 466, 40);
			lv_obj_align(down_grip, LV_ALIGN_BOTTOM_MID, 0, 0);
			lv_obj_add_flag(down_grip, LV_OBJ_FLAG_SCROLLABLE);
			lv_obj_set_style_opa(down_grip, LV_OPA_0, 0);
		}

		// Create top header bar
		lv_obj_t *header_bar = lv_obj_create(pages[i]);
		lv_obj_set_size(header_bar, 466, 70);
		lv_obj_align(header_bar, LV_ALIGN_TOP_MID, 0, 0);
		lv_obj_add_flag(header_bar, LV_OBJ_FLAG_SCROLLABLE);
		lv_obj_set_style_bg_color(header_bar, lv_color_hex(0x001F1F), 0);
		// Add header separator line
		lv_obj_t *header_sep = lv_obj_create(header_bar);
		lv_obj_set_size(header_sep, 466, 2);
		lv_obj_align(header_sep, LV_ALIGN_BOTTOM_MID, 0, 0);
		lv_obj_set_style_bg_color(header_sep, lv_color_hex(0x000000), 0);
		// Add date label for this day
		lv_obj_t *title_date = lv_label_create(pages[i]);
		lv_obj_set_style_text_font(title_date, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
		// Calculate the date for this day
		T_UTC_TIME time_now = SkaiWatchSys.Global_Time;
		struct tm time_now_tm;
		convert_to_tm(&time_now, &time_now_tm);
		struct tm future_time = time_now_tm;
		future_time.tm_mday += i;
		mktime(&future_time);
		// Format and display the date
		char date_str[10];
		snprintf(date_str, sizeof(date_str), "%02d/%02d", future_time.tm_mon + 1, future_time.tm_mday);
		lv_label_set_text(title_date, date_str);
		lv_obj_align(title_date, LV_ALIGN_TOP_MID, 0, 20);
		p_app_calendar->direction_icon[i][0] = lv_img_create(pages[i]);
		lv_img_set_src(p_app_calendar->direction_icon[i][0], UP_ARROW);
		lv_obj_align(p_app_calendar->direction_icon[i][0], LV_ALIGN_TOP_MID, 0, 80);
		p_app_calendar->direction_icon[i][1] = lv_img_create(pages[i]);
		lv_img_set_src(p_app_calendar->direction_icon[i][1], DOWN_ARROW);
		lv_obj_align(p_app_calendar->direction_icon[i][1], LV_ALIGN_BOTTOM_MID, 0, 0);
		p_app_calendar->direction_icon[i][2] = lv_img_create(pages[i]);
		lv_img_set_src(p_app_calendar->direction_icon[i][2], PREVIOUS_ARROW);
		lv_obj_align(p_app_calendar->direction_icon[i][2], LV_ALIGN_TOP_MID, 0, 80);
		p_app_calendar->direction_icon[i][3] = lv_img_create(pages[i]);
		lv_img_set_src(p_app_calendar->direction_icon[i][3], NEXT_ARROW);
		lv_obj_align(p_app_calendar->direction_icon[i][3], LV_ALIGN_BOTTOM_MID, 0, 0);
	}
	// Set active tile to first day
	lv_obj_set_tile_id(tileview, 0, 0, false);

	return tileview;
}

static int selected_calendar_list_index = 0;
void set_calendar_list_index(int index)
{

	selected_calendar_list_index = index;
	if (p_app_calendar->direction_icon[old_active_pos][0] == NULL)
	{
		return;
	}
	if (get_calendar_day_sync_amout(old_active_pos) == 0)
	{
		lv_obj_add_flag(p_app_calendar->direction_icon[old_active_pos][0], LV_OBJ_FLAG_HIDDEN);
		lv_obj_clear_flag(p_app_calendar->direction_icon[old_active_pos][2], LV_OBJ_FLAG_HIDDEN);
		lv_obj_add_flag(p_app_calendar->direction_icon[old_active_pos][1], LV_OBJ_FLAG_HIDDEN);
		lv_obj_clear_flag(p_app_calendar->direction_icon[old_active_pos][3], LV_OBJ_FLAG_HIDDEN);
		return;
	}
	if (selected_calendar_list_index == 0)
	{
		lv_obj_add_flag(p_app_calendar->direction_icon[old_active_pos][0], LV_OBJ_FLAG_HIDDEN);
		lv_obj_clear_flag(p_app_calendar->direction_icon[old_active_pos][2], LV_OBJ_FLAG_HIDDEN);
	}
	else
	{
		lv_obj_add_flag(p_app_calendar->direction_icon[old_active_pos][2], LV_OBJ_FLAG_HIDDEN);
		lv_obj_clear_flag(p_app_calendar->direction_icon[old_active_pos][0], LV_OBJ_FLAG_HIDDEN);
	}

	if (selected_calendar_list_index == get_calendar_day_sync_amout(old_active_pos) - 1)
	{
		lv_obj_add_flag(p_app_calendar->direction_icon[old_active_pos][1], LV_OBJ_FLAG_HIDDEN);
		lv_obj_clear_flag(p_app_calendar->direction_icon[old_active_pos][3], LV_OBJ_FLAG_HIDDEN);
	}
	else
	{
		lv_obj_add_flag(p_app_calendar->direction_icon[old_active_pos][3], LV_OBJ_FLAG_HIDDEN);
		lv_obj_clear_flag(p_app_calendar->direction_icon[old_active_pos][1], LV_OBJ_FLAG_HIDDEN);
	}
}

static bool scroll_up = false;
static gesture_position_t old_gesture_position = {0};
static void set_button_selection(bool reset)
{
	// if (peripheral_provider.get_tap_status())
	// 	return;
	if (old_gesture_position.gesture_position_x > 233 && ((scroll_up == true) || reset))
	{
		scroll_up = false;
		lv_obj_set_style_img_opa(p_app_calendar->direction_icon[old_active_pos][0], LV_OPA_20, 0);
		lv_obj_set_style_img_opa(p_app_calendar->direction_icon[old_active_pos][1], LV_OPA_100, 0);
		lv_obj_set_style_img_opa(p_app_calendar->direction_icon[old_active_pos][2], LV_OPA_20, 0);
		lv_obj_set_style_img_opa(p_app_calendar->direction_icon[old_active_pos][3], LV_OPA_100, 0);
	}
	else if (old_gesture_position.gesture_position_x < 233 && ((scroll_up == false) || reset))
	{
		scroll_up = true;
		lv_obj_set_style_img_opa(p_app_calendar->direction_icon[old_active_pos][0], LV_OPA_100, 0);
		lv_obj_set_style_img_opa(p_app_calendar->direction_icon[old_active_pos][1], LV_OPA_20, 0);
		lv_obj_set_style_img_opa(p_app_calendar->direction_icon[old_active_pos][2], LV_OPA_100, 0);
		lv_obj_set_style_img_opa(p_app_calendar->direction_icon[old_active_pos][3], LV_OPA_20, 0);
	}
	set_calendar_list_index(selected_calendar_list_index);
}

static void button_selection(gesture_position_t gesture_position)
{
	old_gesture_position = gesture_position;
	set_button_selection(false);
}

static uint8_t total_calendar_amount = 0;
void set_calendar_list_amount(uint8_t amount)
{
	total_calendar_amount = amount;
}

static void scroll_calendar_list(uint8_t press)
{
	if (press != 1)
	{
		return; // Only handle press event
	}
	LOG_D("SCROLL CALENDAR LIST");
	if (scroll_up)
	{
		if (selected_calendar_list_index - 1 < 0)
		{
			if (old_active_pos == 0)
			{
				return;
			}
			selected_calendar_list_index = get_calendar_day_sync_amout(old_active_pos - 1) - 1;
			lv_obj_scroll_to_view(lv_obj_get_child(myLancher[app_index_calendar_list].pagetileview, old_active_pos - 1), LV_ANIM_ON);
		}
		else
		{
			lv_obj_scroll_to_view(lv_obj_get_child(p_app_calendar->calendar_day_list[old_active_pos], selected_calendar_list_index - 1), LV_ANIM_ON);
		}
	}
	else
	{
		if (selected_calendar_list_index + 1 >= get_calendar_day_sync_amout(old_active_pos))
		{
			if (old_active_pos == 6)
			{
				return;
			}
			selected_calendar_list_index = 0;
			lv_obj_scroll_to_view(lv_obj_get_child(myLancher[app_index_calendar_list].pagetileview, old_active_pos + 1), LV_ANIM_ON);
		}
		else
		{
			lv_obj_scroll_to_view(lv_obj_get_child(p_app_calendar->calendar_day_list[old_active_pos], selected_calendar_list_index + 1), LV_ANIM_ON);
		}
	}
}

/**
 * @brief Create the calendar widget
 * @param parent Parent object
 * @return The created widget
 */
lv_obj_t *lv_calendar_widget_builder(lv_obj_t *parent, calendar_model_t *calendar_model)
{
	lv_obj_t *widget = parent;
    lv_obj_set_style_bg_opa(widget, LV_OPA_100, 0);
    lv_obj_set_style_radius(widget, 50, LV_PART_MAIN);
	lv_obj_set_size(widget, 400, 230);
	lv_obj_set_style_bg_color(widget, lv_color_hex(0x000000), 0);
	// lv_obj_set_style_bg_opa(widget, LV_OPA_70, 0);

	lv_obj_t *event_category = lv_obj_create(widget);
	lv_obj_set_size(event_category, 10, 35);
	lv_obj_align(event_category, LV_ALIGN_TOP_LEFT, 30, 28);
	lv_obj_set_style_bg_color(event_category, lv_color_hex(0x369EB2), 0);
	lv_obj_set_style_bg_opa(event_category, LV_OPA_90, 0);
	lv_obj_set_style_radius(event_category, 100, 0);
	calendar_model->event_category = event_category;
	lv_obj_add_flag(event_category, LV_OBJ_FLAG_HIDDEN);
	// Calendar time display
	lv_obj_t *calendar_time = lv_label_create(widget);
	lv_label_set_text(calendar_time, "");
	lv_obj_set_style_text_font(calendar_time, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
	lv_obj_set_style_text_color(calendar_time, lv_color_hex(0xFFFFFF), 0);
	lv_obj_align_to(calendar_time, event_category, LV_ALIGN_RIGHT_MID, 20, 0);
	lv_obj_set_style_text_opa(calendar_time, LV_OPA_80, 0);
	calendar_model->time = calendar_time;

	// Calendar content
	lv_obj_t *content = lv_label_create(widget);
	lv_label_set_long_mode(content, LV_LABEL_LONG_DOT);
	lv_obj_set_height(content, 230 - 90);
	lv_obj_set_width(content, 430 - 60);
	lv_obj_set_style_text_font(content, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
	lv_obj_set_style_text_color(content, lv_color_hex(0xFFFFFF), 0);
	lv_obj_get_style_text_opa(content, LV_OPA_70);
	lv_label_set_text(content, "No Calendar");
	lv_obj_align(content, LV_ALIGN_TOP_LEFT, 28, 73);
	calendar_model->summary = content;
	return widget;
}

/**
 * @brief Refresh the calendar widget with calendar_event data
 */
void refresh_calendar_model(calendar_model_t *calendar_model, calendar_event_t *calendar_event)
{
	if (calendar_model == NULL || calendar_event == NULL)
	{
		LOG_E("Invalid parameters in refresh_calendar_model");
		return;
	}

	char date_str[32];

	// Validate UI objects
	if (!lv_obj_is_valid(calendar_model->time) ||
		!lv_obj_is_valid(calendar_model->summary))
	{
		return;
	}
	// Update event display
	if (!calendar_event->summary || strlen(calendar_event->summary) == 0)
	{
		// No calendar events
		lv_label_set_text(calendar_model->time, "");
		lv_obj_align(calendar_model->summary, LV_ALIGN_CENTER, 0, 0);
		lv_label_set_text(calendar_model->summary, "No Calendar");
		lv_obj_add_flag(calendar_model->event_category, LV_OBJ_FLAG_HIDDEN);
	}
	else
	{
		// Display calendar event details
		snprintf(date_str, sizeof(date_str), "%02d:%02d ~ %02d:%02d",
				 calendar_event->startTime.hour, calendar_event->startTime.minutes,
				 calendar_event->endTime.hour, calendar_event->endTime.minutes);
		lv_label_set_text(calendar_model->time, date_str);
		lv_obj_update_layout(calendar_model->time);
		lv_obj_align(calendar_model->summary, LV_ALIGN_TOP_LEFT, 28, 73);
		lv_label_set_text(calendar_model->summary, calendar_event->summary);
		lv_obj_clear_flag(calendar_model->event_category, LV_OBJ_FLAG_HIDDEN);
	}

	calendar_event->notified = true;
}

void calendar_today_refresh(void)
{
	LOG_D("Refreshing calendar today data");
	calendar_model_t *calendar_model = get_on_coming_calendar_model();
	// 獲取第0天的日曆
	calendar_t *calendar_day = get_calendar_day(0);
	if (!calendar_day || calendar_day->event_count == 0)
	{
		// 沒有日曆事件時的處理

		// Validate UI objects
		if (
			!lv_obj_is_valid(calendar_model->time) ||
			!lv_obj_is_valid(calendar_model->summary))
		{
			return;
		}
		// No calendar events
		lv_label_set_text(calendar_model->time, "");
		lv_obj_align(calendar_model->summary, LV_ALIGN_CENTER, 0, 0);
		lv_label_set_text(calendar_model->summary, "No Calendar");
		lv_obj_add_flag(calendar_model->event_category, LV_OBJ_FLAG_HIDDEN);
		return;
	}

	calendar_event_t *calendar_event = &calendar_day->events[0];
	refresh_calendar_model(calendar_model, calendar_event);
}

void calendar_widget_start(void)
{
	lvgl_msg_handler.handle_refresh_calendar_widget = calendar_today_refresh;
}

void calendar_widget_stop(void)
{	
	if (lvgl_msg_handler.handle_refresh_calendar_widget == calendar_today_refresh)
	{
		lvgl_msg_handler.handle_refresh_calendar_widget = NULL;
	}
}

calendar_model_t temp_calendar_model;
lv_obj_t *lv_calendar_object_builder(lv_obj_t *parent, void *data)
{
	if (!parent || !data)
	{
		LOG_E("Invalid parameters in lv_calendar_object_builder");
		return NULL;
	}
	calendar_event_t *calendar_event = (calendar_event_t *)data;
	lv_obj_t *widget = lv_calendar_widget_builder(parent, &temp_calendar_model);
	lv_obj_set_size(widget, 360, 240);
	refresh_calendar_model(&temp_calendar_model, calendar_event);

	return widget;
}

// 定義 dial calendar widget 實例結構
typedef struct dial_calendar_widget_instance {
	lv_obj_t *widget;              // widget 根物件
	lv_obj_t *event_category;      // 事件類別標記
	lv_obj_t *time;                // 時間標籤
	lv_obj_t *summary;             // 摘要標籤
	struct dial_calendar_widget_instance *next;
} dial_calendar_widget_instance_t;

static dial_calendar_widget_instance_t *dial_calendar_widget_list = NULL;

// 新增 widget 實例到鏈表
static dial_calendar_widget_instance_t* dial_calendar_widget_add_instance(void)
{
	dial_calendar_widget_instance_t *instance = lv_mem_alloc(sizeof(dial_calendar_widget_instance_t));
	if (instance == NULL) {
		LOG_E("Failed to allocate dial_calendar_widget_instance");
		return NULL;
	}
	memset(instance, 0, sizeof(dial_calendar_widget_instance_t));
	instance->next = dial_calendar_widget_list;
	dial_calendar_widget_list = instance;
	return instance;
}

// 移除無效的 widget 實例
static void dial_calendar_widget_cleanup_invalid(void)
{
	dial_calendar_widget_instance_t **pp = &dial_calendar_widget_list;
	while (*pp != NULL) {
		if (!lv_obj_is_valid((*pp)->widget)) {
			dial_calendar_widget_instance_t *to_free = *pp;
			*pp = (*pp)->next;
			lv_mem_free(to_free);
		} else {
			pp = &(*pp)->next;
		}
	}
}

lv_obj_t *lv_dial_calendar_object_builder(lv_obj_t *parent)
{
	// 先清理無效的實例
	dial_calendar_widget_cleanup_invalid();
	
	// 創建新實例
	dial_calendar_widget_instance_t *instance = dial_calendar_widget_add_instance();
	if (instance == NULL) {
		return NULL;
	}

	lv_obj_t *event_category = lv_obj_create(parent);
	lv_obj_set_size(event_category, 10, 35);
	lv_obj_align(event_category, LV_ALIGN_TOP_LEFT, 30, 28);
	lv_obj_set_style_bg_color(event_category, lv_color_hex(0x369EB2), 0);
	lv_obj_set_style_bg_opa(event_category, LV_OPA_90, 0);
	lv_obj_set_style_radius(event_category, 100, 0);
	instance->event_category = event_category;
	lv_obj_add_flag(event_category, LV_OBJ_FLAG_HIDDEN);
	// Calendar time display
	lv_obj_t *calendar_time = lv_label_create(parent);
	lv_label_set_text(calendar_time, "");
	lv_obj_set_style_text_font(calendar_time, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
	lv_obj_set_style_text_color(calendar_time, lv_color_hex(0xFFFFFF), 0);
	lv_obj_align_to(calendar_time, event_category, LV_ALIGN_RIGHT_MID, 20, 0);
	lv_obj_set_style_text_opa(calendar_time, LV_OPA_80, 0);
	instance->time = calendar_time;

	// Calendar content
	lv_obj_t *content = lv_label_create(parent);
	lv_label_set_long_mode(content, LV_LABEL_LONG_DOT);
	lv_obj_set_height(content, 150 - 90);
	lv_obj_set_width(content, 330 - 60);
	lv_obj_set_style_text_font(content, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
	lv_obj_set_style_text_color(content, lv_color_hex(0xFFFFFF), 0);
	lv_obj_get_style_text_opa(content, LV_OPA_70);
	lv_label_set_text(content, "No Calendar");
	lv_obj_align(content, LV_ALIGN_CENTER, 0, 0);
	instance->summary = content;
	return parent;
}

// 更新單個 widget 實例的顯示
static void refresh_dial_calendar_instance(dial_calendar_widget_instance_t *instance, calendar_event_t *calendar_event)
{
	if (instance == NULL) return;
	
	// Validate UI objects
	if (!lv_obj_is_valid(instance->time) ||
		!lv_obj_is_valid(instance->summary) ||
		!lv_obj_is_valid(instance->event_category))
	{
		return;
	}
	
	char date_str[32];
	
	// Update event display
	if (calendar_event == NULL || !calendar_event->summary || strlen(calendar_event->summary) == 0)
	{
		// No calendar events
		lv_label_set_text(instance->time, "");
		lv_obj_align(instance->summary, LV_ALIGN_CENTER, 0, 0);
		lv_label_set_text(instance->summary, "No Calendar");
		lv_obj_add_flag(instance->event_category, LV_OBJ_FLAG_HIDDEN);
	}
	else
	{
		// Display calendar event details
		snprintf(date_str, sizeof(date_str), "%02d:%02d ~ %02d:%02d",
				 calendar_event->startTime.hour, calendar_event->startTime.minutes,
				 calendar_event->endTime.hour, calendar_event->endTime.minutes);
		lv_label_set_text(instance->time, date_str);
		lv_obj_update_layout(instance->time);
		lv_obj_align(instance->summary, LV_ALIGN_TOP_LEFT, 28, 73);
		lv_label_set_text(instance->summary, calendar_event->summary);
		lv_obj_clear_flag(instance->event_category, LV_OBJ_FLAG_HIDDEN);
	}
}

void refresh_dial_calendar_model(calendar_event_t *calendar_event)
{
	if (calendar_event == NULL)
	{
		LOG_E("Invalid parameters in refresh_calendar_model");
		return;
	}
	
	// 先清理無效實例
	dial_calendar_widget_cleanup_invalid();
	
	// 遍歷所有有效實例並更新
	dial_calendar_widget_instance_t *instance = dial_calendar_widget_list;
	while (instance != NULL) {
		refresh_dial_calendar_instance(instance, calendar_event);
		instance = instance->next;
	}

	calendar_event->notified = true;
}

void dial_calendar_today_refresh(void)
{
	LOG_D("Refreshing calendar today data");
	
	// 先清理無效實例
	dial_calendar_widget_cleanup_invalid();
	
	// 如果沒有實例則返回
	if (dial_calendar_widget_list == NULL) {
		return;
	}
	
	// 獲取第0天的日曆
	calendar_t *dial_calendar_day = get_calendar_day(0);
	calendar_event_t *dial_calendar_event = NULL;
	
	if (dial_calendar_day && dial_calendar_day->event_count > 0) {
		dial_calendar_event = &dial_calendar_day->events[0];
	}
	
	// 遍歷所有實例並更新
	dial_calendar_widget_instance_t *instance = dial_calendar_widget_list;
	while (instance != NULL) {
		refresh_dial_calendar_instance(instance, dial_calendar_event);
		instance = instance->next;
	}
	
	if (dial_calendar_event) {
		dial_calendar_event->notified = true;
	}
}


void dial_calendar_widget_start(void)
{
	lvgl_msg_handler.handle_refresh_dial_calendar_widget = dial_calendar_today_refresh;
}

void dial_calendar_widget_deinit(void)
{
	// 清理無效實例
	dial_calendar_widget_cleanup_invalid();
	
	// 只有當沒有有效實例時才取消註冊 handler
	if (dial_calendar_widget_list == NULL) {
		if (lvgl_msg_handler.handle_refresh_dial_calendar_widget == dial_calendar_today_refresh)
		{
			lvgl_msg_handler.handle_refresh_dial_calendar_widget = NULL;
		}
	}
}

void lv_dial_calendar_widget_builder(lv_obj_t *parent)
{
	if (!parent)
	{
		LOG_E("Invalid parent in lv_dial_calendar_widget_builder");
		return;
	}
	lv_obj_t *widget = lv_dial_calendar_object_builder(parent);
	dial_calendar_today_refresh();
	dial_calendar_widget_start();
}

/**
 * @brief Refresh the calendar screen
 */
static void refresh_scr(void)
{
	if (p_app_calendar == NULL)
	{
		LOG_E("p_app_calendar is NULL in refresh_scr");
		return;
	}

	LOG_D("Refreshing calendar screen UI only (not clearing data)");

	// NOTE: We don't clean up calendar memory here - only refresh UI
	// Memory cleanup happens only when new data arrives via handle_calendar

	// Delete the old window completely
	if (lv_obj_is_valid(p_app_calendar->main_window))
	{
		lv_obj_del(p_app_calendar->main_window);
		p_app_calendar->main_window = NULL;
	}

	// Create a fresh window
	p_app_calendar->main_window = calendar_tileview_builder(lv_scr_act());
	myLancher[app_index_calendar_list].pagetileview = p_app_calendar->main_window;
}

/**
 * @brief Initialize the app
 * @param parent Parent object
 */
static void on_start(lv_obj_t *parent)
{
	LOG_D("calendar_app_on_start");
	screen_rotate_back_to_original_direction();
	RT_ASSERT(NULL == p_app_calendar);
	old_active_pos = 0;
	selected_calendar_list_index = 0;
	// Allocate and initialize app data
	p_app_calendar = (app_calendar_t *)lv_mem_alloc(sizeof(app_calendar_t));
	if (p_app_calendar == NULL)
	{
		LOG_E("Failed to allocate memory for calendar app");
		return;
	}

	memset(p_app_calendar, 0, sizeof(app_calendar_t));

	// Create main window
	p_app_calendar->main_window = calendar_tileview_builder(parent);
	myLancher[app_index_calendar_list].pagetileview = p_app_calendar->main_window;

	// Request calendar data from mobile device
	request_calendar_on_mobile(true);
}

/**
 * @brief Resume the app
 */
static void on_resume(void)
{
	set_open_control_options(true);
	set_free_control_with_arm(false);
	set_media_control_threshold(1000);
#ifdef BSP_USING_UI_HANDLER
	lvgl_msg_handler.refresh_calendar = refresh_scr;
	lvgl_msg_handler.handle_tap_indicator = scroll_calendar_list;
	lvgl_msg_handler.handle_widgets_control = button_selection;
#endif
	set_open_control_options(true);
}

/**
 * @brief Pause the app
 */
static void on_pause(void)
{
#ifdef BSP_USING_UI_HANDLER
	lvgl_msg_handler.refresh_calendar = NULL;
	lvgl_msg_handler.handle_tap_indicator = NULL;
	lvgl_msg_handler.handle_widgets_control = NULL;
#endif
	set_media_control_threshold(3000);
	set_open_control_options(false);
}

/**
 * @brief Stop the app and clean up resources
 */
static void on_stop(void)
{
	// NOTE: We don't clean up calendar memory here to preserve data between app sessions
	// Calendar memory will be cleaned up only when new data arrives (in refresh_scr or notify_calendar)

	if (p_app_calendar)
	{
		if (lv_obj_is_valid(p_app_calendar->main_window))
		{
			lv_obj_del(p_app_calendar->main_window);
		}
		p_app_calendar->main_window = NULL;
		lv_mem_free(p_app_calendar);
		p_app_calendar = NULL;
	}
}

/**
 * @brief Message handler for app lifecycle events
 */
static void msg_handler(gui_app_msg_type_t msg, void *param)
{
	switch (msg)
	{
	case GUI_APP_MSG_ONSTART:
		on_start(lv_scr_act());
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

/**
 * @brief Main entry point for the calendar app
 */
static int app_main(intent_t i)
{
	gui_app_regist_msg_handler(APP_ID_CALENDAR, msg_handler);
	return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(calendar), IMG_CALENDAR, APP_ID_CALENDAR, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/