/**
 ******************************************************************************
 * @file   lv_calendar_list_layout.c
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
#include "lv_ext_resource_manager.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#ifdef BSP_USING_GESTURE_HANDLER
#include "gesture_handler.h"
#endif
#include "watch_global_data.h"
#include "bloc_calendar.h"
#include "bloc_control.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#ifdef BSP_USING_BLOC_NOTIFY
#include "bloc_notification.h"
#endif
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#endif
#define DBG_TAG "calendar.layout"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define LIST_CALENDAR_LIST_WIDTH (420)
#define LIST_CALENDAR_LIST_HEIGHT (230)
#define LIST_CALENDAR_LIST_SPACING (10)
#define LIST_RADIUS (1000)

// Structure to hold calendar list view related data
typedef struct
{
    bool open_action_flag;
    bool left_hand_mode;
    uint16_t selected_calendar_list_index;
    uint16_t page;
    uint16_t oneday_list_length[7];
    lv_obj_t *tileview;
    lv_obj_t *app_list[7];
    char time_str[20];
} calendar_list_view_t;

static calendar_list_view_t calendar_view = {
    .open_action_flag = true,
    .left_hand_mode = true,
    .selected_calendar_list_index = 0,
    .page = 0,
    .oneday_list_length = {0},
    .tileview = NULL,
    .app_list = {NULL},
    .time_str = {0}};

static const lv_style_const_prop_t LIST_CALENDAR_LIST_STYLE_PROPS[] = {
    LV_STYLE_CONST_WIDTH(LIST_CALENDAR_LIST_WIDTH),
    LV_STYLE_CONST_HEIGHT(LIST_CALENDAR_LIST_HEIGHT),
    LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(LIST_CALENDAR_LIST_STYLE, LIST_CALENDAR_LIST_STYLE_PROPS);

MyStruct_T myList[CALENDAR_NULL];

extern void set_calendar_list_index(uint8_t index);
extern void set_calendar_list_amount(uint8_t amount);
static uint8_t calendar_list_amount = 0;
static void scroll_list(lv_obj_t *obj, int16_t drift, uint8_t reset_page)
{
    uint16_t min_offset = LV_VER_RES;
    uint8_t child_cnt = obj->spec_attr->child_cnt;
    lv_coord_t y_diff = 0;
    lv_coord_t x_trans;
    uint32_t x_sqr;
    lv_sqrt_res_t res;
    uint8_t i;
    lv_obj_t *child;
    lv_coord_t y_center;

    if (calendar_list_amount != child_cnt)
    {
        calendar_list_amount = child_cnt;
#ifdef APP_ID_CALENDAR
        set_calendar_list_amount(child_cnt);
#endif
    }
    for (i = 0; i < child_cnt; i++)
    {
        child = obj->spec_attr->children[i];
        y_center = child->coords.y1 + LIST_CALENDAR_LIST_HEIGHT / 2 + drift;

        if (reset_page == 2)
        {
            y_diff = y_center - LV_VER_RES / 2 - LV_VER_RES;
        }
        else if (reset_page == 0)
        {
            y_diff = y_center - LV_VER_RES / 2 + LV_VER_RES;
        }
        else
        {
            y_diff = y_center - LV_VER_RES / 2;
        }
        y_diff = LV_ABS(y_diff);

        if (y_diff >= LIST_RADIUS)
        {
            if (calendar_view.left_hand_mode)
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
                calendar_view.selected_calendar_list_index = i;
#ifdef APP_ID_CALENDAR
                set_calendar_list_index(i);
#endif
            }
            x_sqr = LIST_RADIUS * LIST_RADIUS - y_diff * y_diff;
            lv_sqrt(x_sqr, &res, 0x8000);
            if (calendar_view.left_hand_mode)
            {
                x_trans = res.i - 977;
            }
            else
            {
                x_trans = LIST_RADIUS - res.i;
            }
        }
    }

    if (calendar_view.selected_calendar_list_index == 0)
    {
        lv_coord_t scroll_y = lv_obj_get_scroll_y(obj);
        if (calendar_view.open_action_flag == true)
        {
            if (scroll_y < -130)
            {
                lv_obj_t *current_tile = lv_tileview_get_tile_act(myLancher[app_index_calendar_list].pagetileview);
                int index = lv_obj_get_index(current_tile);
                if (index != 0)
                {
                    lv_obj_scroll_to_view(lv_obj_get_child(myLancher[app_index_calendar_list].pagetileview, index - 1), LV_ANIM_ON);
                    force_release_finger();
                }
                calendar_view.open_action_flag = false;
            }
        }
        else if (scroll_y > -20)
            calendar_view.open_action_flag = true;
    }
    if (calendar_view.selected_calendar_list_index == child_cnt - 1)
    {
        lv_coord_t scroll_y = lv_obj_get_scroll_y(obj);
        lv_obj_t *current_tile = lv_tileview_get_tile_act(myLancher[app_index_calendar_list].pagetileview);
        int index = lv_obj_get_index(current_tile);
        if (calendar_view.open_action_flag == true && calendar_view.app_list[index] == obj)
        {
            if (scroll_y > (calendar_view.oneday_list_length[index]) - 60)
            {
                if (index != 6)
                {
                    lv_obj_scroll_to_view(lv_obj_get_child(myLancher[app_index_calendar_list].pagetileview, index + 1), LV_ANIM_ON);
#ifndef BSP_USING_PC_SIMULATOR
                    force_release_finger();
#endif
                }
                calendar_view.open_action_flag = false;
            }
        }
        else if (scroll_y < (calendar_view.oneday_list_length[index]) - 80)
        {
            calendar_view.open_action_flag = true;
        }
    }

    // Update styles for selected and non-selected items
    for (uint8_t i = 0; i < child_cnt; i++)
    {
        lv_obj_t *child = obj->spec_attr->children[i];
        if (i == calendar_view.selected_calendar_list_index)
        {
            lv_obj_set_style_bg_color(child, lv_color_hex(0x000000), 0);
            lv_obj_set_style_radius(child, 25, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(child, LV_OPA_80, 0);
            lv_obj_set_style_border_color(child, lv_color_hex(0x4F4F4F), LV_PART_MAIN);
            lv_obj_set_style_border_width(child, 2, LV_PART_MAIN);
        }
        else
        {
            lv_obj_set_style_border_width(child, 1, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(child, LV_OPA_40, 0);
        }
    }

    lv_obj_mark_layout_as_dirty(obj);
}

static void list_window_scroll_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = evt->target;
    if (obj == NULL)
    {
        return;
    }
    scroll_list(obj, 0, 1);
}

static void reset_list_top(uint8_t day)
{
    scroll_list(calendar_view.app_list[day], 0, 0);
}

static void reset_list_down(uint8_t day)
{
    scroll_list(calendar_view.app_list[day], 0, 2);
}

static void list_calendar_list_click_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = lv_event_get_target(evt);
    void *dat = lv_event_get_user_data(evt);
    calendar_event_t *calendar_event = (calendar_event_t *)dat;

    if (calendar_event != NULL && calendar_event->summary != NULL)
    {
        rt_kprintf("calendar_event->summary: %s\n", calendar_event->summary);
    }
}

static void refresh_list(lv_obj_t *list, uint8_t new_item_count, uint8_t day)
{
    uint16_t calendar_list_length = 0;
    /* Delete all children of the list */
    if (list != RT_NULL)
    {
        lv_obj_clean(list);
    }

    /* Get the calendar day data */
    calendar_t *calendar_day = get_calendar_day(day);
    if (calendar_day == NULL || calendar_day->event_count == 0)
    {
        return;
    }

    /* Repopulate the list with the new number of items */
    for (uint8_t i = 0; i < new_item_count && i < calendar_day->event_count; i++)
    {
        calendar_event_t *calendar_event = &calendar_day->events[i];
        if (calendar_event == NULL)
        {
            continue;
        }

        lv_obj_t *calendar_list_widget = lv_obj_create(list);
        lv_obj_set_size(calendar_list_widget, LIST_CALENDAR_LIST_WIDTH, LIST_CALENDAR_LIST_HEIGHT);
        lv_obj_set_pos(calendar_list_widget, 23, calendar_list_length + (LIST_CALENDAR_LIST_SPACING * i) - 110);
        lv_obj_add_flag(calendar_list_widget, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(calendar_list_widget, list_calendar_list_click_event_cb, LV_EVENT_CLICKED, calendar_event);
        lv_obj_set_style_radius(calendar_list_widget, 25, LV_PART_MAIN);
        lv_obj_set_style_bg_color(calendar_list_widget, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(calendar_list_widget, LV_OPA_20, 0);

        lv_obj_t *label = lv_label_create(calendar_list_widget);
        lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
        sprintf(calendar_view.time_str, "%02d:%02d~%02d:%02d\n",
                calendar_event->startTime.hour, calendar_event->startTime.minutes,
                calendar_event->endTime.hour, calendar_event->endTime.minutes);
        lv_label_set_text(label, calendar_view.time_str);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);

        lv_obj_t *content = lv_label_create(calendar_list_widget);
        lv_label_set_long_mode(content, LV_LABEL_LONG_DOT);
        lv_obj_set_width(content, LIST_CALENDAR_LIST_WIDTH - 20);
        lv_obj_set_style_text_font(content, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        // Use the event's summary, with fallback for NULL
        const char *summary_text = (calendar_event->summary && strlen(calendar_event->summary) > 0)
                                       ? calendar_event->summary
                                       : "No Title";
        lv_label_set_text(content, summary_text);
        lv_obj_align(content, LV_ALIGN_CENTER, 0, 0);

        lv_obj_update_layout(content);
        lv_obj_update_layout(label);

        uint16_t label_height = lv_obj_get_height(label) + lv_obj_get_height(content) - 10;
        if (label_height > 186)
        {
            label_height = 186;
        }
        calendar_list_length += label_height;
        if ((calendar_list_length) > calendar_view.oneday_list_length[day])
        {
            calendar_view.oneday_list_length[day] = calendar_list_length;
        }
        lv_obj_set_height(calendar_list_widget, label_height);
        calendar_event->notified = true;
    }
}

void refresh_calendar_list(lv_obj_t *list, uint8_t page_index)
{
    if (get_calendar_day_sync_amout(page_index) > 0)
    {
        if (list != RT_NULL)
        {
            refresh_list(list, get_calendar_day_sync_amout(page_index), page_index);
        }
        else
        {
            refresh_list(RT_NULL, get_calendar_day_sync_amout(page_index), page_index);
        }
    }
    else
    {
        lv_obj_t *no_calendars_widget = common_list_widget(calendar_view.app_list[page_index],
                                                           (lv_style_t *)&LIST_CALENDAR_LIST_STYLE, 23, -110);
        lv_obj_set_style_bg_color(no_calendars_widget, lv_color_hex(0x000000), 0);
        lv_obj_set_style_radius(no_calendars_widget, 25, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(no_calendars_widget, LV_OPA_80, 0);
        lv_obj_set_style_border_color(no_calendars_widget, lv_color_hex(0x4F4F4F), LV_PART_MAIN);
        lv_obj_set_style_border_width(no_calendars_widget, 2, LV_PART_MAIN);
        lv_obj_t *label = lv_label_create(no_calendars_widget);
        lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
        lv_label_set_text(label, "No Calendar");
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
        calendar_view.oneday_list_length[page_index] = 230;
    }
    if (list != RT_NULL)
    {
        myList[page_index].reset_list_top = reset_list_top;
        myList[page_index].reset_list_down = reset_list_down;
    }
}

lv_obj_t *lv_calendar_list_layout_create(lv_obj_t *parent, uint8_t page_index)
{
    lv_obj_t *p_window = lv_obj_create(parent);
    lv_obj_set_style_bg_opa(p_window, LV_OPA_0, 0);
    lv_obj_set_size(p_window, LV_HOR_RES, LV_VER_RES);
    calendar_view.app_list[page_index] = p_window;
    lv_obj_set_size(calendar_view.app_list[page_index], LV_HOR_RES, LV_VER_RES);
    lv_obj_add_flag(calendar_view.app_list[page_index], LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(calendar_view.app_list[page_index], LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(calendar_view.app_list[page_index], LV_DIR_VER);
    lv_obj_set_scroll_snap_y(calendar_view.app_list[page_index], LV_SCROLL_SNAP_CENTER);
    lv_obj_set_style_pad_ver(calendar_view.app_list[page_index], LV_VER_RES / 2, 0);
    lv_obj_add_event_cb(calendar_view.app_list[page_index], list_window_scroll_event_cb, LV_EVENT_SCROLL, NULL);
    refresh_calendar_list(calendar_view.app_list[page_index], page_index);
    // lv_event_send(calendar_view.app_list[page_index], LV_EVENT_SCROLL, NULL);
    return p_window;
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/