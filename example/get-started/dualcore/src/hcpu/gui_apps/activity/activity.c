/**
 ******************************************************************************
 * @file   activity.c
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
#include "sensor.h"
#include "hr_service.h"
#include "ui_datasrv_subscriber.h"
#include "app_mainmenu.h"
#include "bloc_motion_tracking.h"
#include "common_widget.h"
#ifdef BSP_USING_BLOC
#include "bloc_setting.h"
#include "bloc_peripheral.h"
#endif
#include "data_service_provider.h"
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#include "ui_img_helper.h"
#endif
#include "watch_global_data.h"
#define DBG_TAG "app.activity"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_ACTIVITY

#define RING_WIDTH 20

// LV_IMG_DECLARE(img_activity);

typedef struct app_activity
{
    lv_obj_t *calories_ring;
    lv_obj_t *steps_ring;
    lv_obj_t *activity_ring;
    lv_obj_t *calories_label;
    lv_obj_t *steps_label;
    lv_obj_t *activity_label;
} app_activity_t;

static app_activity_t *p_app_activity = NULL;
static app_activity_t *p_context_widget_activity;

static uint32_t get_steps_today(void)
{
    return SkaiWatchSys.health_info_today.steps;
}
static float get_distance_today(void)
{
    return SkaiWatchSys.health_info_today.distance;
}
static float get_calories_today(void)
{
    return SkaiWatchSys.health_info_today.calories;
}

static lv_obj_t *create_ring(lv_obj_t *parent, lv_coord_t size, lv_color_t color, lv_obj_t *align_to, lv_align_t align, lv_coord_t x_ofs, lv_coord_t y_ofs)
{
    lv_obj_t *ring = lv_arc_create(parent);
    lv_obj_set_size(ring, size, size);
    lv_arc_set_range(ring, 0, 100);
    lv_arc_set_value(ring, 0);

    // 設置圓環顏色
    lv_obj_set_style_arc_color(ring, color, LV_PART_INDICATOR);

    lv_arc_set_rotation(ring, 270);
    // 0°對應的位置是3點位置，並且沿著順時針方向增加。
    lv_arc_set_bg_angles(ring, 0, 360);
    // 設置圓環的寬度
    lv_obj_set_style_arc_width(ring, RING_WIDTH, LV_PART_MAIN);
    lv_obj_set_style_arc_width(ring, RING_WIDTH, LV_PART_INDICATOR);

    // 背景圓環的顏色設置為半透明
    lv_obj_set_style_arc_color(ring, lv_color_hex(0xE0E0E0), LV_PART_MAIN);
    lv_obj_set_style_arc_opa(ring, 77, LV_PART_MAIN); // LV_OPA_30 = 77

    lv_obj_remove_style(ring, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(ring, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align_to(ring, align_to, align, x_ofs, y_ofs);
    return ring;
}

static void create_activity_rings(app_activity_t *p_activity, lv_obj_t *parent, lv_coord_t size)
{
    if (size < 150)
    {
        size = 150;
    }
    p_activity->calories_ring = create_ring(parent, size, lv_palette_main(LV_PALETTE_RED), NULL, LV_ALIGN_CENTER, 0, 0);

    p_activity->steps_ring = create_ring(parent, size - 50, lv_palette_main(LV_PALETTE_GREEN), p_activity->calories_ring, LV_ALIGN_CENTER, 0, 0);

    p_activity->activity_ring = create_ring(parent, size - 100, lv_palette_main(LV_PALETTE_BLUE), p_activity->calories_ring, LV_ALIGN_CENTER, 0, 0);
}

static void refresh_activity(app_activity_t *activity)
{
    if (activity)
    {
        uint32_t steps = get_steps_today();
        float distance = get_distance_today() / 1000000; // km
        float calories = get_calories_today() / 1000;    // k-calories

        // 計算圓環進度百分比
        int32_t calories_percent = (int32_t)(calories * 100 / 300); // 300 k-calories
        int32_t steps_percent = (int32_t)(steps * 100 / 10000);     // 10000 steps
        int32_t distance_percent = (int32_t)(distance * 100 / 7.5); // 7.5km

        // 限制百分比在0-100之間
        calories_percent = calories_percent > 100 ? 100 : (calories_percent < 0 ? 0 : calories_percent);
        steps_percent = steps_percent > 100 ? 100 : (steps_percent < 0 ? 0 : steps_percent);
        distance_percent = distance_percent > 100 ? 100 : (distance_percent < 0 ? 0 : distance_percent);

        // 直接設置圓環進度，不使用動畫
        if (lv_obj_is_valid(activity->calories_ring))
        {
            lv_arc_set_value(activity->calories_ring, calories_percent);
        }

        if (lv_obj_is_valid(activity->steps_ring))
        {
            lv_arc_set_value(activity->steps_ring, steps_percent);
        }

        if (lv_obj_is_valid(activity->activity_ring))
        {
            lv_arc_set_value(activity->activity_ring, distance_percent);
        }

        char buf[32];
        if (lv_obj_is_valid(activity->calories_label))
        {
            if (calories > 0)
            {
                if (calories >= 1000)
                {
                    snprintf(buf, sizeof(buf), "%.1f kcal", calories);
                }
                else
                {
                    snprintf(buf, sizeof(buf), "%.0f kcal", calories);
                }
                lv_label_set_text(activity->calories_label, buf);
            }
            else
            {
                lv_label_set_text(activity->calories_label, "0 kcal");
            }
        }

        if (lv_obj_is_valid(activity->steps_label))
        {
            if (steps > 0)
            {
                snprintf(buf, sizeof(buf), "%d step", steps);
                lv_label_set_text(activity->steps_label, buf);
            }
            else
            {
                lv_label_set_text(activity->steps_label, "0 step");
            }
        }

        if (lv_obj_is_valid(activity->activity_label))
        {
            if (distance > 0)
            {
                if (distance >= 1.0f)
                {
                    snprintf(buf, sizeof(buf), "%.1f km", distance);
                }
                else
                {
                    snprintf(buf, sizeof(buf), "%.0f m", distance * 1000);
                }
                lv_label_set_text(activity->activity_label, buf);
            }
            else
            {
                lv_label_set_text(activity->activity_label, "0 m");
            }
        }
    }
}

static void on_start(lv_obj_t *parent)
{
    p_app_activity = (app_activity_t *)rt_malloc(sizeof(app_activity_t));
    memset(p_app_activity, 0, sizeof(app_activity_t));
    uint32_t steps = get_steps_today();
    float distance = get_distance_today() / 1000000; // km
    float calories = get_calories_today() / 1000;    // k-calories

    create_activity_rings(p_app_activity, parent, 400);

    // 創建中央文字容器
    lv_obj_t *center_cont = lv_obj_create(parent);
    lv_obj_set_size(center_cont, 200, 150);
    lv_obj_set_style_bg_opa(center_cont, LV_OPA_0, 0);
    lv_obj_set_style_border_width(center_cont, 0, 0);
    lv_obj_set_style_pad_all(center_cont, 0, 0);
    lv_obj_clear_flag(center_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(center_cont, LV_ALIGN_CENTER, 0, 0);

    // 卡路里標籤 - 置中顯示
    p_app_activity->calories_label = lv_label_create(center_cont);
    lv_obj_set_style_text_font(p_app_activity->calories_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(p_app_activity->calories_label, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_text_align(p_app_activity->calories_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(p_app_activity->calories_label, LV_ALIGN_TOP_MID, 0, 10);

    // 步數標籤 - 置中顯示
    p_app_activity->steps_label = lv_label_create(center_cont);
    lv_obj_set_style_text_font(p_app_activity->steps_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(p_app_activity->steps_label, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_align(p_app_activity->steps_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(p_app_activity->steps_label, LV_ALIGN_CENTER, 0, 0);

    // 距離標籤 - 置中顯示
    p_app_activity->activity_label = lv_label_create(center_cont);
    lv_obj_set_style_text_font(p_app_activity->activity_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(p_app_activity->activity_label, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_style_text_align(p_app_activity->activity_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(p_app_activity->activity_label, LV_ALIGN_BOTTOM_MID, 0, -10);

    refresh_activity(p_app_activity);
}

static void on_pause(void)
{
    setting_provider.set_power_save_mode(1);
}

static void on_resume(void)
{
    // switch_watch_motion_control_mode(true, false);
    set_open_control_options(false);
    set_free_control_with_arm(false);
    setting_provider.set_power_save_mode(0);
    lvgl_msg_handler.handle_tap_indicator = NULL;
}

static void on_stop(void)
{
    if (p_app_activity)
    {
        if (p_app_activity->calories_ring)
            lv_obj_del(p_app_activity->calories_ring);
        if (p_app_activity->steps_ring)
            lv_obj_del(p_app_activity->steps_ring);
        if (p_app_activity->activity_ring)
            lv_obj_del(p_app_activity->activity_ring);
        if (p_app_activity->calories_label)
            lv_obj_del(p_app_activity->calories_label);
        if (p_app_activity->steps_label)
            lv_obj_del(p_app_activity->steps_label);
        if (p_app_activity->activity_label)
            lv_obj_del(p_app_activity->activity_label);
        rt_free(p_app_activity);
        p_app_activity = NULL;
    }
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
    gui_app_regist_msg_handler(APP_ID_ACTIVITY, msg_handler);

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(activity), IMG_ACTIVITY, APP_ID_ACTIVITY, app_main);

void refresh_activity_widget(void)
{
    if (p_context_widget_activity == NULL)
    {
        return;
    }
    refresh_activity(p_context_widget_activity);
}

lv_obj_t *lv_activity_widget_builder(lv_obj_t *parent)
{
    lv_obj_t *widget = common_widget_container(parent);
    // 左側容器 - 圓環區域
    lv_obj_t *cont_left = lv_obj_create(widget);
    lv_obj_set_size(cont_left, 250, 220);
    lv_obj_set_style_bg_opa(cont_left, LV_OPA_0, 0);
    lv_obj_set_style_border_width(cont_left, 0, 0);
    lv_obj_set_style_pad_all(cont_left, 0, 0);
    lv_obj_clear_flag(cont_left, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(cont_left, LV_ALIGN_LEFT_MID, 10, 0);
    create_activity_rings(p_context_widget_activity, cont_left, 200);

    // 右側容器 - 文字區域
    lv_obj_t *cont_right = lv_obj_create(widget);
    lv_obj_set_size(cont_right, 160, 220);
    lv_obj_set_style_bg_opa(cont_right, LV_OPA_0, 0);
    lv_obj_set_style_border_width(cont_right, 0, 0);
    lv_obj_set_style_pad_all(cont_right, 0, 0);
    lv_obj_clear_flag(cont_right, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(cont_right, LV_ALIGN_RIGHT_MID, -10, 0);

    // 卡路里標籤 - 置中顯示
    lv_obj_t *calories_label = lv_label_create(cont_right);
    lv_obj_set_style_text_font(calories_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(calories_label, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_text_align(calories_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(calories_label, LV_ALIGN_TOP_MID, 0, 10);
    p_context_widget_activity->calories_label = calories_label;

    // 步數標籤 - 置中顯示
    lv_obj_t *steps_label = lv_label_create(cont_right);
    lv_obj_set_style_text_font(steps_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(steps_label, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_align(steps_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(steps_label, LV_ALIGN_CENTER, 0, 0);
    p_context_widget_activity->steps_label = steps_label;

    // 距離標籤 - 置中顯示
    lv_obj_t *activity_label = lv_label_create(cont_right);
    lv_obj_set_style_text_font(activity_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(activity_label, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_style_text_align(activity_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(activity_label, LV_ALIGN_BOTTOM_MID, 0, -10);
    p_context_widget_activity->activity_label = activity_label;

    refresh_activity_widget();

    return widget;
}

void activity_widget_start(void)
{
    LOG_D("activity_widget_start");
    RT_ASSERT(NULL == p_context_widget_activity);
    p_context_widget_activity = (app_activity_t *)lv_mem_alloc(sizeof(app_activity_t));
    memset(p_context_widget_activity, 0, sizeof(app_activity_t));
}

void activity_widget_stop(void)
{
    LOG_D("activity_widget_stop");
    if (p_context_widget_activity)
    {
        if (p_context_widget_activity->calories_ring && lv_obj_is_valid(p_context_widget_activity->calories_ring))
            lv_obj_del(p_context_widget_activity->calories_ring);
        if (p_context_widget_activity->steps_ring && lv_obj_is_valid(p_context_widget_activity->steps_ring))
            lv_obj_del(p_context_widget_activity->steps_ring);
        if (p_context_widget_activity->activity_ring && lv_obj_is_valid(p_context_widget_activity->activity_ring))
            lv_obj_del(p_context_widget_activity->activity_ring);
        if (p_context_widget_activity->calories_label && lv_obj_is_valid(p_context_widget_activity->calories_label))
            lv_obj_del(p_context_widget_activity->calories_label);
        if (p_context_widget_activity->steps_label && lv_obj_is_valid(p_context_widget_activity->steps_label))
            lv_obj_del(p_context_widget_activity->steps_label);
        if (p_context_widget_activity->activity_label && lv_obj_is_valid(p_context_widget_activity->activity_label))
            lv_obj_del(p_context_widget_activity->activity_label);
        lv_mem_free(p_context_widget_activity);
        p_context_widget_activity = NULL;
    }
}

#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/