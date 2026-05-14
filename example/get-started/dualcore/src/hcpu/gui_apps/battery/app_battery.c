/**
 ******************************************************************************
 * @file   app_battery.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk
 * integrated circuit in a product or a software update for such product, must
 * reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "custom_trans_anim.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "watch_global_data.h"
#ifdef BSP_USING_BLOC
    #include "bloc_setting.h"
    #include "bloc_peripheral.h"
#endif
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
#endif
#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
    #include "ui_img_helper.h"
#endif
#ifdef BSP_USING_COMMUNICATE
    #include "communicate_protocol.h"
#endif
#define DBG_TAG "app.test"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#ifdef APP_ID_BATTERY

typedef struct
{
    lv_obj_t *battery_level_img;
    lv_obj_t *hint_label;
} app_battery_t;

static app_battery_t *p_app_batt = NULL;

static void set_text_hint(lv_obj_t *label, uint8_t battery_level)
{
    char text[8];
    sprintf(text, "%d%%", battery_level);
    lv_label_set_text(label, text);
}

static void refresh_battery_level(uint8_t level)
{
    set_text_hint(p_app_batt->hint_label, level);
}

static void set_zoom(void *img, int32_t v)
{
    lv_img_set_zoom(img, v);
}

static void on_screen_touch(lv_event_t *e)
{
    gui_app_exit(APP_ID_BATTERY);
}

static lv_obj_t *on_start(lv_obj_t *parent)
{
    p_app_batt = (app_battery_t *)rt_malloc(sizeof(app_battery_t));
    memset(p_app_batt, 0, sizeof(app_battery_t));

    lv_obj_t *level_img = lv_img_create(parent);
    lv_img_set_src(level_img, IMG_CHARGING);
    lv_obj_align(level_img, LV_ALIGN_CENTER, 0, -20);
    p_app_batt->battery_level_img = level_img;

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, level_img);
    lv_anim_set_exec_cb(&a, set_zoom);
    lv_anim_set_values(&a, 0.7 * 256, 256);
    lv_anim_set_playback_time(&a, 1000);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);

    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)),
                               0);
    lv_obj_align_to(label, level_img, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    set_text_hint(label, SkaiWatchSys.battery_level_value);
    p_app_batt->hint_label = label;

    // Add touch event listener to the parent
    lv_obj_add_event_cb(parent, on_screen_touch, LV_EVENT_CLICKED, NULL);

    cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);

    return parent;
}

static void on_resume(void)
{
    lvgl_msg_handler.handle_battery_percentage = refresh_battery_level;
    show_instruction_list_time(false);
    show_battery(false);
}

static void on_pause(void)
{
    lvgl_msg_handler.handle_battery_percentage = NULL;
    show_instruction_list_time(true);
    show_battery(true);
}

static void on_stop(void)
{
    if (p_app_batt)
    {
        if (p_app_batt->battery_level_img)
            lv_obj_del(p_app_batt->battery_level_img);
        if (p_app_batt->hint_label)
            lv_obj_del(p_app_batt->hint_label);
        rt_free(p_app_batt);
        p_app_batt = NULL;
    }
}

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

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_BATTERY, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(battery), IMG_LOGO, APP_ID_BATTERY, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/