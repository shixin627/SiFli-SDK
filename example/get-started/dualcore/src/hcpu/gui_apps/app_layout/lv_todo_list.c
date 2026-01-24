/**
 ******************************************************************************
 * @file   lv_todo_list.c
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
#include "lv_arc_scrollbar.h"
#include "lv_ext_resource_manager.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "bloc_task.h"
#include <bloc_control.h>
#include "lvsf_font.h"

#ifdef APP_ID_TODOLIST

#define LIST_TODO_LIST_WIDTH (450) // 380
#define LIST_TODO_LIST_HEIGHT (80)
#define LIST_TODO_LIST_SPACING (20)

#define LIST_RADIUS (1000) // 1000

#define LIST_TODO_LIST_RADIUS (260)
#define LIST_TODO_LIST_BORDER_SIDE LV_BORDER_SIDE_RIGHT

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define FONT_SIZE FONT_BIGL
#define LIST_ENABLE_ARC_SCROLLBAR 0

LV_IMG_DECLARE(plus);
extern lv_obj_t *build_widget_media(lv_obj_t *parent);
extern lv_obj_t *build_widget_flashlight(lv_obj_t *parent);
extern lv_obj_t *build_widget_setting(lv_obj_t *parent);
extern lv_obj_t *build_widget_heart_rate(lv_obj_t *parent);
extern lv_obj_t *build_widget_weather(lv_obj_t *parent);
static struct
{
	const char *title;
	// build function
	lv_obj_t *(*build_widget_func)(lv_obj_t *parent);

} APP_LIST_WIDGETS[] = {
	{.title = "新的待辦", .build_widget_func = NULL}
	// calendar
	// stock
	// activity
	// timer
	// Alarm
};

static const lv_style_const_prop_t LIST_TODO_LIST_STYLE_PROPS[] = {
	LV_STYLE_CONST_WIDTH(LIST_TODO_LIST_WIDTH),
	LV_STYLE_CONST_HEIGHT(LIST_TODO_LIST_HEIGHT),
	LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t LIST_TODO_LIST_TITLE_STYLE_PROPS[] = {
	LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_14),
	LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0xFF, 0xFF, 0xFF)),
	LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
	LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(LIST_TODO_LIST_STYLE, LIST_TODO_LIST_STYLE_PROPS);
LV_STYLE_CONST_INIT(LIST_TODO_LIST_TITLE_STYLE, LIST_TODO_LIST_TITLE_STYLE_PROPS);
static lv_obj_t *p_todo_list;

static bool open_action_flag = true;
static bool left_hand_mode = true;

static void todo_list_create(lv_obj_t *p_todo_list);

static void todo_list_window_scroll_event_cb(lv_event_t *evt)
{
	uint8_t todo_list_num_on_screen = 0;
	uint16_t last_todo_list_index_on_screen = 0;
	lv_obj_t *obj = evt->target;
	uint16_t min_offset = LV_VER_RES;

	uint8_t child_cnt = obj->spec_attr->child_cnt;
	for (uint8_t i = 0; i < child_cnt; i++)
	{
		lv_obj_t *child = obj->spec_attr->children[i];
		if (child->coords.y2 <= 0)
		{
			continue;
		}
		else if (child->coords.y1 >= (lv_coord_t)LV_HOR_RES)
		{
			break;
		}

		lv_coord_t y_center = child->coords.y1 + LIST_TODO_LIST_HEIGHT / 2;
		// LOG_D("todo_list_widget_distance_from_center[0]: %d", todo_list_widget_distance_from_center[0]);
		lv_coord_t y_diff = y_center - LV_VER_RES / 2;
		y_diff = LV_ABS(y_diff);
		lv_coord_t x_trans;

		if (y_diff >= LIST_RADIUS)
		{
			if (left_hand_mode)
			{
				x_trans = 0;
			}
			else
			{
				x_trans = LIST_RADIUS;
			}
		}
		else
		{
			if (y_diff < min_offset)
			{
				min_offset = y_diff;
				selected_task_index = i;
			}
			todo_list_num_on_screen++;
			last_todo_list_index_on_screen = i;
			uint32_t x_sqr = LIST_RADIUS * LIST_RADIUS - y_diff * y_diff;
			lv_sqrt_res_t res;
			lv_sqrt(x_sqr, &res, 0x8000);
			if (left_hand_mode)
			{
				// LOG_D("i: %d, x_trans: %d", i, res.i);
				x_trans = res.i - 992; // - 955
			}
			else
			{
				x_trans = LIST_RADIUS - res.i;
			}
		}
		// Risky but fastest way to set LV_STYLE_X value and update layout
		lv_style_value_t *value = (lv_style_value_t *)child->styles->style->v_p.values_and_props;
		value->num = x_trans;
		lv_obj_mark_layout_as_dirty(child);
	}

	// Add this code to change the background color of the centered object
	for (uint8_t i = 0; i < child_cnt; i++)
	{
		lv_obj_t *child = obj->spec_attr->children[i];
		if (i == selected_task_index)
		{
			// Set the background color to blue
			lv_obj_set_style_bg_color(child, lv_color_hex(0xFFFFFF), 0);
			lv_obj_set_style_radius(child, 200, LV_PART_MAIN);
			lv_obj_set_style_bg_opa(child, LV_OPA_30, 0);
		}
		else
		{
			// Set the background color to original color
			lv_obj_set_style_bg_opa(child, LV_OPA_0, 0);
		}
	}

	lv_obj_mark_layout_as_dirty(obj);
}

#if LV_GDX_PATCH_USE_FAST_TILEVIEW
#include "lv_layout_router.h"
static void list_contorolcenter_widget_click_event_cb(lv_event_t *evt)
{
	lv_obj_t *obj = lv_event_get_target(evt);
	void *dat = lv_event_get_user_data(evt);
	if (dat == (&APP_LIST_WIDGETS[6]))
	{
		lv_layout_router_show_ota(obj);
	}
}
#else
static void list_todo_list_click_event_cb(lv_event_t *evt)
{
	task_t *task = (task_t *)evt->user_data;
	char *content = task->content;
	LOG_D("list_todo_list_click_event_cb %s", content);
	// static bool is_clicked = false;
	// static bool is_style_blue_initialized = false;
	// static bool is_style_original_initialized = false;
	// lv_obj_t* obj = lv_event_get_target(evt);

	// static lv_style_t style_blue;
	// static lv_style_t style_original;

	//// Initialize the styles if they haven't been initialized yet
	// if (!is_style_blue_initialized) {
	//	//lv_style_init(&style_blue);
	//	lv_obj_set_style_bg_color(obj, lv_color_hex(0x007BFF) , 0); // light blue color
	//	is_style_blue_initialized = true;
	// }
	////if (!is_style_original_initialized) {
	////	lv_style_init(&style_original);
	////	lv_style_set_bg_color(&style_original, lv_color_hex(0xFFFFFF)); // replace with original color
	////	is_style_original_initialized = true;
	////}

	//// Toggle the color
	// if (is_clicked) {
	//	LOG_D("is_clicked");
	//	//lv_obj_add_style(obj, &style_original, LV_PART_MAIN );
	//	lv_obj_set_style_bg_color(obj, lv_color_hex(0x007BFF), 0);
	//	lv_obj_set_style_bg_opa(obj, LV_OPA_100, 0);
	//	is_clicked = false;
	// }
	// else
	//{
	//	LOG_D("is_not_clicked");
	//	//lv_obj_add_style(obj, &style_blue, LV_PART_MAIN );
	//	lv_obj_set_style_bg_color(obj, lv_color_hex(0x007BFF), 0);
	//	lv_obj_set_style_bg_opa(obj, LV_OPA_0, 0);
	//	is_clicked = true;
	// }
}
#endif

static void refresh_todo_list(void)
{
	todo_list_create(p_todo_list);
}

static void create_empty_task(lv_event_t *e)
{
	task_t new_task = empty_task();
	strcat(new_task.content, "new task");
	task_t *todo_list = current_task_list();
	add_task(todo_list, &task_items_amount, new_task);
	refresh_todo_list();
}

static void checkbox_event_cb(lv_event_t *e)
{
	lv_obj_t *checkbox = lv_event_get_target(e);
	task_t *task = (task_t *)lv_event_get_user_data(e);

	if (lv_obj_has_state(checkbox, LV_STATE_CHECKED))
	{
		LOG_D("Checkbox checked");
		task->completed = true;
	}
	else
	{
		LOG_D("Checkbox unchecked");
		task->completed = false;
	}
	refresh_todo_list();
}
#define LV_FONT_DEFAULT &lv_font_montserrat_14
static void todo_list_create(lv_obj_t *p_todo_list)
{
	lv_obj_clean(p_todo_list);
	LOG_D("task_items_amount: %d", task_items_amount);
	uint8_t uncompleted_task_items_amount = 0;

	lv_obj_t *create_btn = common_list_widget(p_todo_list, (lv_style_t *)&LIST_TODO_LIST_STYLE, 0, (LIST_TODO_LIST_HEIGHT + LIST_TODO_LIST_SPACING) * uncompleted_task_items_amount);
	lv_obj_add_flag(create_btn, LV_OBJ_FLAG_CLICKABLE);
	// 創建一個新的圖片 widget
	lv_obj_t *icon = lv_img_create(create_btn);
	lv_img_set_src(icon, &plus); // 將圖片源設置為你的圖標
	lv_img_set_zoom(icon, 256 * 0.6);
	lv_obj_align(icon, LV_ALIGN_LEFT_MID, 90, 0);
	lv_obj_t *label = lv_label_create(create_btn);
	lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
	lv_obj_set_style_text_color(label, lv_color_white(), 0);
	lv_label_set_text_static(label, "add task");
	lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_event_cb(create_btn, create_empty_task, LV_EVENT_CLICKED, NULL);
	uncompleted_task_items_amount++;

	for (int i = task_items_amount - 1; i >= 0; i--)
	{
		task_t *task = get_task(i);
		if (task->completed)
		{
			continue;
		}

		lv_obj_t *todo_list = common_list_widget(p_todo_list, (lv_style_t *)&LIST_TODO_LIST_STYLE, 0, (LIST_TODO_LIST_HEIGHT + LIST_TODO_LIST_SPACING) * uncompleted_task_items_amount);
		lv_obj_add_flag(todo_list, LV_OBJ_FLAG_CLICKABLE);
		lv_obj_add_event_cb(todo_list, list_todo_list_click_event_cb, LV_EVENT_CLICKED, (void *)task);

		lv_obj_t *checkbox = lv_checkbox_create(todo_list);
		lv_obj_set_style_radius(checkbox, LV_RADIUS_CIRCLE, LV_PART_INDICATOR); // 將 checkbox 的外型改為圓形
		// 設置圓形按鈕的背景為透明
		lv_obj_set_style_bg_opa(checkbox, LV_OPA_0, LV_PART_INDICATOR);
		lv_checkbox_set_text(checkbox, task->content);
		lv_font_t *font = lvsf_get_font_from_size(FONT_BIGL);
		lv_obj_set_style_text_font(checkbox, font, 0);
		lv_obj_set_style_text_color(checkbox, lv_color_white(), 0);
		lv_obj_set_style_pad_all(checkbox, -1, LV_PART_INDICATOR);

		lv_obj_align(checkbox, LV_ALIGN_LEFT_MID, 80, 0);
		lv_obj_add_event_cb(checkbox, checkbox_event_cb, LV_EVENT_VALUE_CHANGED, (void *)task); // 為 checkbox 添加事件回調
		uncompleted_task_items_amount++;
	}
	if (uncompleted_task_items_amount > 0)
	{
		LOG_D("selected_task_index: %d", selected_task_index);
		lv_obj_scroll_to_view(lv_obj_get_child(p_todo_list, 1), LV_ANIM_OFF);
	}
}

static lv_style_t style_cb;
lv_obj_t *lv_todo_list_layout_create(lv_obj_t *parent)
{
#if LV_GDX_PATCH_DISABLE_STYLE_REFRESH && !LV_GDX_PATCH_USE_FAST_TILEVIEW
	lv_obj_enable_style_refresh(false);
	lv_disp_t *disp = lv_obj_get_disp(parent);
	lv_disp_enable_invalidation(disp, false);
#endif
	lv_style_init(&style_cb);
	lv_style_set_radius(&style_cb, LV_RADIUS_CIRCLE);
	common_watch_bg(parent);
	lv_obj_t *p_window = lv_obj_create(parent);
	lv_obj_set_style_bg_opa(p_window, LV_OPA_0, 0);
	lv_obj_set_size(p_window, LV_HOR_RES, LV_VER_RES);

#if LIST_ENABLE_ARC_SCROLLBAR
	p_todo_list = lv_simplified_obj_create(p_window);
#else
	p_todo_list = p_window;
#endif // LIST_ENABLE_ARC_SCROLLBAR

	lv_obj_set_size(p_todo_list, LV_HOR_RES, LV_VER_RES);
	lv_obj_add_flag(p_todo_list, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_scrollbar_mode(p_todo_list, LV_SCROLLBAR_MODE_OFF);
	lv_obj_set_scroll_dir(p_todo_list, LV_DIR_VER);
	lv_obj_set_scroll_snap_y(p_todo_list, LV_SCROLL_SNAP_CENTER);
	lv_obj_set_style_pad_ver(p_todo_list, LV_VER_RES / 2, 0);

	lv_obj_add_event_cb(p_todo_list, todo_list_window_scroll_event_cb, LV_EVENT_SCROLL, NULL);

	todo_list_create(p_todo_list);

#if LIST_ENABLE_ARC_SCROLLBAR
	lv_obj_t *bar = lv_arc_scrollbar_create(p_window);
	lv_arc_scrollbar_attach(bar, p_todo_list);
#endif // LIST_ENABLE_ARC_SCROLLBAR

	lv_obj_scroll_to_view(lv_obj_get_child(p_todo_list, 1), LV_ANIM_OFF);

#if LV_GDX_PATCH_DISABLE_STYLE_REFRESH && !LV_GDX_PATCH_USE_FAST_TILEVIEW
	lv_disp_enable_invalidation(disp, true);
	lv_obj_enable_style_refresh(true);
	lv_obj_refresh_style(p_todo_list, LV_PART_ANY, LV_STYLE_PROP_ANY);
#endif

	lv_event_send(p_todo_list, LV_EVENT_SCROLL, NULL);
	return p_window;
}

static rt_int32_t init(lv_obj_t *parent)
{
	lv_todo_list_layout_create(parent);
	return RT_EOK;
}

static rt_int32_t pause(void)
{
	return RT_EOK;
}

static rt_int32_t resume(void)
{
	return RT_EOK;
}

static rt_int32_t deinit(void)
{
	return RT_EOK;
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
	switch (msg)
	{
	case GUI_APP_MSG_ONSTART:
		init(lv_scr_act());
		break;

	case GUI_APP_MSG_ONRESUME:
		resume();
		break;

	case GUI_APP_MSG_ONPAUSE:
		pause();
		break;

	case GUI_APP_MSG_ONSTOP:
		deinit();
		break;
	default:
		break;
	}
}

static int app_main(intent_t i)
{
	gui_app_regist_msg_handler(APP_ID_TODOLIST, msg_handler);

	return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(tasks), LV_EXT_IMG_GET(img_tasks), APP_ID_TODOLIST, app_main);

#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/