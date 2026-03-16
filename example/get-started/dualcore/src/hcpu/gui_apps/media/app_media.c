/**
 ******************************************************************************
 * @file   app_media.c
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
#include "custom_trans_anim.h"
#include "bloc_setting.h"
#include "bloc_motion_tracking.h"
#include "bloc_control.h"
#include "bloc_peripheral.h"
#include "ui_handler.h"
#include "ui_img_helper.h"
#include "bloc_v2t.h"
#define DBG_TAG "app.media"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
#endif

#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
#endif

#define MEDIA_WIDGET_BTN_Y -60

LV_IMG_DECLARE(img_media_ellipse);
LV_IMG_DECLARE(img_media_play);
LV_IMG_DECLARE(img_media_pause);
LV_IMG_DECLARE(img_media_previous);
LV_IMG_DECLARE(img_media_next);
LV_IMG_DECLARE(volume_up);
LV_IMG_DECLARE(volume_down);
LV_IMG_DECLARE(mouse_mode_icon);
LV_IMG_DECLARE(micro_icon);
LV_IMG_DECLARE(media_gaus_bg);
LV_IMG_DECLARE(control_selection_bg);

static app_media_t *p_app_media = NULL;
static app_media_t *p_widget_media = NULL;

/* Declare the action functions for the buttons */
static void prev_btn_event_cb(lv_event_t *e);
static void play_pause_btn_event_cb(lv_event_t *e);
static void handle_media_play_state(void *param);
static void handle_media_title(void *param);
static void next_btn_event_cb(lv_event_t *e);
static void volume_up_btn_event_cb(lv_event_t *e);
static void volume_down_btn_event_cb(lv_event_t *e);
static void first_app_btn_event_cb(lv_event_t *e);
static void second_app_btn_event_cb(lv_event_t *e);
static void third_app_btn_event_cb(lv_event_t *e);
static void fourth_app_btn_event_cb(lv_event_t *e);

static bool gesture_open = false;
static bool open_volume_control = false;
static uint8_t volume_control_value = 0;
static uint8_t button_selection_index = 5;

static void change_icon_image(lv_obj_t *icon, const void *new_img_src)
{
    /* Get the image object from the button */
    lv_obj_t *img = lv_obj_get_child(icon, 0);

    /* Change the image source */
    lv_img_set_src(img, new_img_src);
}

void music_ui_build(app_media_t *p_app_media, lv_obj_t *parent, float size)
{
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

    uint16_t zoom = 256 * size;

    /* Create the previous button */
    lv_obj_t *btn_prev =
        common_icon_button(parent, &img_media_previous, prev_btn_event_cb);
    lv_obj_align(btn_prev, LV_ALIGN_BOTTOM_LEFT, 0, -119);
    lv_obj_t *img_prev = lv_obj_get_child(btn_prev, 0);
    lv_img_set_zoom(img_prev, zoom);

    /* Create the next button */
    lv_obj_t *btn_next =
        common_icon_button(parent, &img_media_next, next_btn_event_cb);
    lv_obj_align(btn_next, LV_ALIGN_BOTTOM_RIGHT, 0, -119);
    lv_obj_t *img_next = lv_obj_get_child(btn_next, 0);
    lv_img_set_zoom(img_next, zoom);

    /* Create the play/pause button */
    const lv_img_dsc_t *img_src = control_provider.bt_speaker_get_status()
                                      ? &img_media_pause
                                      : &img_media_play;
    p_app_media->icon_btn_play_pause =
        common_icon_button(parent, img_src, play_pause_btn_event_cb);
    lv_obj_align(p_app_media->icon_btn_play_pause, LV_ALIGN_BOTTOM_MID, 0,
                 -110);
    lv_obj_t *img_pause = lv_obj_get_child(p_app_media->icon_btn_play_pause, 0);
    lv_img_set_zoom(img_pause, zoom);

    /* Create the volume down button */
    lv_obj_t *btn_vol_down = common_image_button(parent, &volume_down, 145, 100,
                                                 volume_down_btn_event_cb);
    lv_obj_align(btn_vol_down, LV_ALIGN_BOTTOM_LEFT, 70, -40);

    /* Create the volume up button */
    lv_obj_t *btn_vol_up = common_image_button(parent, &volume_up, 145, 100,
                                               volume_up_btn_event_cb);
    lv_obj_align(btn_vol_up, LV_ALIGN_BOTTOM_RIGHT, -70, -40);
}

static lv_obj_t *dial_widget_vol_bar = NULL;
static lv_obj_t *dial_widget_vol_icon_btn = NULL;
static bool vol_bar_expanded = false;
static lv_timer_t *vol_bar_collapse_timer = NULL;
#define VOL_BAR_COLLAPSE_TIMEOUT 3000
#define VOL_BAR_EXPANDED_WIDTH 270

static void vol_bar_anim_width_cb(void *var, int32_t v)
{
    lv_obj_set_width((lv_obj_t *)var, v);
    lv_obj_align_to((lv_obj_t *)var, dial_widget_vol_icon_btn,
                    LV_ALIGN_OUT_TOP_MID, 0, -20);
    uint8_t opacity = (v * 255) / VOL_BAR_EXPANDED_WIDTH;
    if (opacity > 255)
        opacity = 255;
    lv_obj_set_style_bg_opa(dial_widget_vol_bar, opacity, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(dial_widget_vol_bar, opacity, LV_PART_INDICATOR);
}

static void vol_bar_collapse_anim_ready_cb(lv_anim_t *a)
{
    if (lv_obj_is_valid(dial_widget_vol_bar))
    {
        lv_obj_add_flag(dial_widget_vol_bar, LV_OBJ_FLAG_HIDDEN);
    }
}

static void vol_bar_collapse(void)
{
    if (!vol_bar_expanded)
        return;
    vol_bar_expanded = false;

    if (!lv_obj_is_valid(dial_widget_vol_bar))
        return;

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, dial_widget_vol_bar);
    lv_anim_set_values(&a, lv_obj_get_width(dial_widget_vol_bar), 0);
    lv_anim_set_time(&a, 300);
    lv_anim_set_exec_cb(&a, vol_bar_anim_width_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_ready_cb(&a, vol_bar_collapse_anim_ready_cb);
    lv_anim_start(&a);
}

static void vol_bar_collapse_timer_cb(lv_timer_t *timer)
{
    vol_bar_collapse_timer = NULL;
    vol_bar_collapse();
}

static void vol_bar_reset_collapse_timer(void)
{
    if (vol_bar_collapse_timer)
    {
        lv_timer_del(vol_bar_collapse_timer);
        vol_bar_collapse_timer = NULL;
    }
    vol_bar_collapse_timer =
        lv_timer_create(vol_bar_collapse_timer_cb, VOL_BAR_COLLAPSE_TIMEOUT, NULL);
    lv_timer_set_repeat_count(vol_bar_collapse_timer, 1);
}

static void vol_bar_expand(void)
{
    if (vol_bar_expanded)
    {
        vol_bar_reset_collapse_timer();
        return;
    }
    vol_bar_expanded = true;

    if (!lv_obj_is_valid(dial_widget_vol_bar))
        return;

    lv_obj_clear_flag(dial_widget_vol_bar, LV_OBJ_FLAG_HIDDEN);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, dial_widget_vol_bar);
    lv_anim_set_values(&a, 0, VOL_BAR_EXPANDED_WIDTH);
    lv_anim_set_time(&a, 300);
    lv_anim_set_exec_cb(&a, vol_bar_anim_width_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);

    vol_bar_reset_collapse_timer();
}

static void vol_icon_click_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        if (!vol_bar_expanded)
            vol_bar_expand();
        else
            vol_bar_collapse();
    }
}

static void bar_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *bar = lv_event_get_target(e);
    if (code == LV_EVENT_PRESSED)
    {
        vol_bar_reset_collapse_timer();
    }
    else if (code == LV_EVENT_RELEASED)
    {
        vol_bar_reset_collapse_timer();
    }
    else if (code == LV_EVENT_PRESSING)
    {
        LOG_D("LV_EVENT_PRESSING_volume Bar");
        lv_obj_t *bar = lv_event_get_target(e);

        lv_point_t p;
        lv_indev_get_point(lv_indev_get_act(), &p);

        lv_coord_t min = lv_bar_get_min_value(bar);
        lv_coord_t max = lv_bar_get_max_value(bar);

        lv_coord_t w = lv_obj_get_width(bar);
        lv_area_t coords;
        lv_obj_get_coords(bar, &coords);
        lv_coord_t rel_x = p.x - coords.x1;
        if (rel_x < 0)
            rel_x = 0;
        if (rel_x > w)
            rel_x = w;

        lv_coord_t value = (rel_x * (max - min)) / w + min;
        if (value < 0)
            value = 0;
        lv_bar_set_value(bar, value, LV_ANIM_OFF);
        uint16_t brightness = lv_bar_get_value(bar);
        control_provider.bt_speaker_set_volume(brightness, true);
    }
}

/* ---- Media widget volume bar (independent from dial widget) ---- */
static lv_obj_t *widget_vol_bar = NULL;
static lv_obj_t *widget_vol_icon_btn = NULL;
static bool widget_vol_bar_expanded = false;
static lv_timer_t *widget_vol_bar_collapse_timer = NULL;
#define WIDGET_VOL_BAR_WIDTH 370

static void widget_vol_bar_anim_width_cb(void *var, int32_t v)
{
    lv_obj_set_width((lv_obj_t *)var, v);
    lv_obj_align((lv_obj_t *)var, LV_ALIGN_BOTTOM_MID, 0,
                 MEDIA_WIDGET_BTN_Y );
    uint8_t opacity = (v * 255) / WIDGET_VOL_BAR_WIDTH;
    if (opacity > 255)
        opacity = 255;
    lv_obj_set_style_bg_opa(widget_vol_bar, opacity, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(widget_vol_bar, opacity, LV_PART_INDICATOR);
}

static void widget_vol_bar_collapse_anim_ready_cb(lv_anim_t *a)
{
    if (lv_obj_is_valid(widget_vol_bar))
    {
        lv_obj_add_flag(widget_vol_bar, LV_OBJ_FLAG_HIDDEN);
    }
}

static void widget_vol_bar_collapse(void)
{
    if (!widget_vol_bar_expanded)
        return;
    widget_vol_bar_expanded = false;

    if (!lv_obj_is_valid(widget_vol_bar))
        return;

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, widget_vol_bar);
    lv_anim_set_values(&a, lv_obj_get_width(widget_vol_bar), 0);
    lv_anim_set_time(&a, 300);
    lv_anim_set_exec_cb(&a, widget_vol_bar_anim_width_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_ready_cb(&a, widget_vol_bar_collapse_anim_ready_cb);
    lv_anim_start(&a);
}

static void widget_vol_bar_collapse_timer_cb(lv_timer_t *timer)
{
    widget_vol_bar_collapse_timer = NULL;
    widget_vol_bar_collapse();
}

static void widget_vol_bar_reset_collapse_timer(void)
{
    if (widget_vol_bar_collapse_timer)
    {
        lv_timer_del(widget_vol_bar_collapse_timer);
        widget_vol_bar_collapse_timer = NULL;
    }
    widget_vol_bar_collapse_timer =
        lv_timer_create(widget_vol_bar_collapse_timer_cb, VOL_BAR_COLLAPSE_TIMEOUT, NULL);
    lv_timer_set_repeat_count(widget_vol_bar_collapse_timer, 1);
}

static void widget_vol_bar_expand(void)
{
    if (widget_vol_bar_expanded)
    {
        widget_vol_bar_reset_collapse_timer();
        return;
    }
    widget_vol_bar_expanded = true;

    if (!lv_obj_is_valid(widget_vol_bar))
        return;

    lv_obj_clear_flag(widget_vol_bar, LV_OBJ_FLAG_HIDDEN);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, widget_vol_bar);
    lv_anim_set_values(&a, 0, WIDGET_VOL_BAR_WIDTH);
    lv_anim_set_time(&a, 300);
    lv_anim_set_exec_cb(&a, widget_vol_bar_anim_width_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);

    widget_vol_bar_reset_collapse_timer();
}

static void widget_vol_icon_click_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        if (!widget_vol_bar_expanded)
            widget_vol_bar_expand();
        else
            widget_vol_bar_collapse();
    }
}

static void widget_bar_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *bar = lv_event_get_target(e);
    if (code == LV_EVENT_PRESSED)
    {
        widget_vol_bar_reset_collapse_timer();
    }
    else if (code == LV_EVENT_RELEASED)
    {
        widget_vol_bar_reset_collapse_timer();
    }
    else if (code == LV_EVENT_PRESSING)
    {
        lv_obj_t *bar = lv_event_get_target(e);

        lv_point_t p;
        lv_indev_get_point(lv_indev_get_act(), &p);

        lv_coord_t min = lv_bar_get_min_value(bar);
        lv_coord_t max = lv_bar_get_max_value(bar);

        lv_coord_t w = lv_obj_get_width(bar);
        lv_area_t coords;
        lv_obj_get_coords(bar, &coords);
        lv_coord_t rel_x = p.x - coords.x1;
        if (rel_x < 0)
            rel_x = 0;
        if (rel_x > w)
            rel_x = w;

        lv_coord_t value = (rel_x * (max - min)) / w + min;
        if (value < 0)
            value = 0;
        lv_bar_set_value(bar, value, LV_ANIM_OFF);
        uint16_t brightness = lv_bar_get_value(bar);
        control_provider.bt_speaker_set_volume(brightness, true);
    }
}

static void set_widget_vol_bar_value(void *param)
{
    if (lv_obj_is_valid(widget_vol_bar))
    {
        uint8_t volume = *(uint8_t *)param;
        lv_bar_set_value(widget_vol_bar, volume, LV_ANIM_ON);
        if (!widget_vol_bar_expanded)
            widget_vol_bar_expand();
        else
            widget_vol_bar_reset_collapse_timer();
    }
}

/* ---- Music app volume bar (independent from dial/media widget) ---- */
static lv_obj_t *app_vol_bar = NULL;
static lv_obj_t *app_vol_icon_btn = NULL;
static bool app_vol_bar_expanded = false;
static lv_timer_t *app_vol_bar_collapse_timer = NULL;
#define APP_VOL_BAR_WIDTH 270

static void app_vol_bar_anim_width_cb(void *var, int32_t v)
{
    lv_obj_set_width((lv_obj_t *)var, v);
    lv_obj_align_to((lv_obj_t *)var, app_vol_icon_btn,
                    LV_ALIGN_OUT_TOP_MID, 0, -20);
    uint8_t opacity = (v * 255) / APP_VOL_BAR_WIDTH;
    if (opacity > 255)
        opacity = 255;
    lv_obj_set_style_bg_opa(app_vol_bar, opacity, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(app_vol_bar, opacity, LV_PART_INDICATOR);
}

static void app_vol_bar_collapse_anim_ready_cb(lv_anim_t *a)
{
    if (lv_obj_is_valid(app_vol_bar))
    {
        lv_obj_add_flag(app_vol_bar, LV_OBJ_FLAG_HIDDEN);
    }
}

static void app_vol_bar_collapse(void)
{
    if (!app_vol_bar_expanded)
        return;
    app_vol_bar_expanded = false;

    if (!lv_obj_is_valid(app_vol_bar))
        return;

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, app_vol_bar);
    lv_anim_set_values(&a, lv_obj_get_width(app_vol_bar), 0);
    lv_anim_set_time(&a, 300);
    lv_anim_set_exec_cb(&a, app_vol_bar_anim_width_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_ready_cb(&a, app_vol_bar_collapse_anim_ready_cb);
    lv_anim_start(&a);
}

static void app_vol_bar_collapse_timer_cb(lv_timer_t *timer)
{
    app_vol_bar_collapse_timer = NULL;
    app_vol_bar_collapse();
}

static void app_vol_bar_reset_collapse_timer(void)
{
    if (app_vol_bar_collapse_timer)
    {
        lv_timer_del(app_vol_bar_collapse_timer);
        app_vol_bar_collapse_timer = NULL;
    }
    app_vol_bar_collapse_timer =
        lv_timer_create(app_vol_bar_collapse_timer_cb, VOL_BAR_COLLAPSE_TIMEOUT, NULL);
    lv_timer_set_repeat_count(app_vol_bar_collapse_timer, 1);
}

static void app_vol_bar_expand(void)
{
    if (app_vol_bar_expanded)
    {
        app_vol_bar_reset_collapse_timer();
        return;
    }
    app_vol_bar_expanded = true;

    if (!lv_obj_is_valid(app_vol_bar))
        return;

    lv_obj_clear_flag(app_vol_bar, LV_OBJ_FLAG_HIDDEN);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, app_vol_bar);
    lv_anim_set_values(&a, 0, APP_VOL_BAR_WIDTH);
    lv_anim_set_time(&a, 300);
    lv_anim_set_exec_cb(&a, app_vol_bar_anim_width_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);

    app_vol_bar_reset_collapse_timer();
}

static void app_vol_icon_click_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        if (!app_vol_bar_expanded)
            app_vol_bar_expand();
        else
            app_vol_bar_collapse();
    }
}

static void app_bar_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *bar = lv_event_get_target(e);
    if (code == LV_EVENT_PRESSED)
    {
        app_vol_bar_reset_collapse_timer();
    }
    else if (code == LV_EVENT_RELEASED)
    {
        app_vol_bar_reset_collapse_timer();
    }
    else if (code == LV_EVENT_PRESSING)
    {
        lv_point_t p;
        lv_indev_get_point(lv_indev_get_act(), &p);

        lv_coord_t min = lv_bar_get_min_value(bar);
        lv_coord_t max = lv_bar_get_max_value(bar);

        lv_coord_t w = lv_obj_get_width(bar);
        lv_area_t coords;
        lv_obj_get_coords(bar, &coords);
        lv_coord_t rel_x = p.x - coords.x1;
        if (rel_x < 0)
            rel_x = 0;
        if (rel_x > w)
            rel_x = w;

        lv_coord_t value = (rel_x * (max - min)) / w + min;
        if (value < 0)
            value = 0;
        lv_bar_set_value(bar, value, LV_ANIM_OFF);
        uint16_t brightness = lv_bar_get_value(bar);
        control_provider.bt_speaker_set_volume(brightness, true);
    }
}

static void set_app_vol_bar_value(void *param)
{
    if (lv_obj_is_valid(app_vol_bar))
    {
        uint8_t volume = *(uint8_t *)param;
        lv_bar_set_value(app_vol_bar, volume, LV_ANIM_ON);
        if (!app_vol_bar_expanded)
            app_vol_bar_expand();
        else
            app_vol_bar_reset_collapse_timer();
    }
}

typedef struct
{
    lv_obj_t *btn_prev_bg;
    lv_obj_t *btn_prev_img;
    lv_obj_t *btn_next_bg;
    lv_obj_t *btn_next_img;
    lv_obj_t *btn_vol_down;
    lv_obj_t *btn_vol_down_img;
    lv_obj_t *btn_vol_up;
    lv_obj_t *btn_vol_up_img;
    lv_obj_t *btn_play_pause;
    lv_obj_t *btn_play_pause_img;

    lv_obj_t *btn_quick_first_app_bg;
    lv_obj_t *btn_quick_second_app_bg;
    lv_obj_t *btn_quick_third_app_bg;
    lv_obj_t *btn_quick_fourth_app_bg;
    lv_obj_t *btn_select_background;
    lv_obj_t *btn_madia_img_bg;
    lv_obj_t *btn_madia_img_mask;
    lv_obj_t *volume_bar;
} music_app_obj_t;

typedef struct
{
    lv_obj_t *widget_btn_prev_bg;
    lv_obj_t *widget_btn_prev_img;
    lv_obj_t *widget_btn_next_bg;
    lv_obj_t *widget_btn_next_img;
    lv_obj_t *widget_btn_play_pause;
    lv_obj_t *widget_btn_play_pause_img;
    lv_obj_t *widget_btn_vol_down;
    lv_obj_t *widget_btn_vol_down_img;
    lv_obj_t *widget_btn_vol_up;
    lv_obj_t *widget_btn_vol_up_img;
    lv_obj_t *widget_volume_bar;
    lv_obj_t *widget_select_background;
} music_widget_obj_t;

extern char *get_media_title(void);
static music_app_obj_t music_app_obj;
static music_widget_obj_t music_widget_obj;
#define WIDGET_ICON_ZOOM_SIZE 0.6
static lv_obj_t *music_app_ui_build(lv_obj_t *parent)
{
    uint16_t app_zoom = 256 * WIDGET_ICON_ZOOM_SIZE;
    if (!parent)
    {
        LOG_E("lv_media_widget_builder parent is NULL");
        return NULL;
    }
    lv_obj_t *p_window = lv_obj_create(parent);
    lv_obj_set_size(p_window, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(p_window, lv_color_hex(0x000000), 0);
    lv_obj_align(p_window, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(p_window, LV_RADIUS_CIRCLE, 0);
    lv_obj_center(p_window);
    if (!p_window)
        return NULL;
    music_app_obj.btn_select_background = lv_obj_create(p_window);
    lv_obj_set_size(music_app_obj.btn_select_background, 116, 80);
    lv_obj_set_style_radius(music_app_obj.btn_select_background, 20, 0);
    lv_obj_set_style_bg_color(music_app_obj.btn_select_background,
                              lv_color_hex(0x737373), 0);
    lv_obj_set_style_bg_opa(music_app_obj.btn_select_background, LV_OPA_0, 0);
    lv_obj_align(music_app_obj.btn_select_background, LV_ALIGN_CENTER, 0, 30);
    lv_obj_set_style_border_width(music_app_obj.btn_select_background, 2, 0);
    lv_obj_set_style_border_color(music_app_obj.btn_select_background,
                                  lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(music_app_obj.btn_select_background, LV_OPA_50,
                                0);
    music_app_obj.btn_madia_img_bg = lv_img_create(p_window);
    lv_obj_align(music_app_obj.btn_madia_img_bg, LV_ALIGN_CENTER, 0, 0);

    music_app_obj.btn_madia_img_mask = lv_img_create(p_window);
    lv_img_set_src(music_app_obj.btn_madia_img_mask, MEDIA_MASK);
    lv_obj_align(music_app_obj.btn_madia_img_mask, LV_ALIGN_CENTER, 0, 0);

    music_app_obj.btn_prev_bg = lv_btn_create(p_window);
    lv_obj_set_size(music_app_obj.btn_prev_bg, 116, 80);
    lv_obj_set_style_radius(music_app_obj.btn_prev_bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(music_app_obj.btn_prev_bg, lv_color_hex(0xFFFFFF),
                              0);
    lv_obj_align(music_app_obj.btn_prev_bg, LV_ALIGN_LEFT_MID, 25, 20);
    lv_obj_set_style_bg_opa(music_app_obj.btn_prev_bg, LV_OPA_0, 0);
    lv_obj_t *btn_prev = common_icon_button(
        music_app_obj.btn_prev_bg, &img_media_previous, prev_btn_event_cb);
    lv_obj_align(btn_prev, LV_ALIGN_CENTER, 0, 0);
    music_app_obj.btn_prev_img = lv_obj_get_child(btn_prev, 0);
    lv_img_set_zoom(music_app_obj.btn_prev_img, app_zoom);
    music_app_obj.btn_next_bg = lv_btn_create(p_window);
    lv_obj_set_size(music_app_obj.btn_next_bg, 116, 80);
    lv_obj_set_style_radius(music_app_obj.btn_next_bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(music_app_obj.btn_next_bg, lv_color_hex(0xFFFFFF),
                              0);
    lv_obj_align(music_app_obj.btn_next_bg, LV_ALIGN_RIGHT_MID, -25, 20);
    lv_obj_set_style_bg_opa(music_app_obj.btn_next_bg, LV_OPA_0, 0);
    lv_obj_t *btn_next = common_icon_button(music_app_obj.btn_next_bg,
                                            &img_media_next, next_btn_event_cb);
    lv_obj_align(btn_next, LV_ALIGN_CENTER, 0, 0);
    music_app_obj.btn_next_img = lv_obj_get_child(btn_next, 0);
    lv_img_set_zoom(music_app_obj.btn_next_img, app_zoom);
    music_app_obj.btn_play_pause = lv_btn_create(p_window);
    lv_obj_set_size(music_app_obj.btn_play_pause, 116, 80);
    lv_obj_set_style_radius(music_app_obj.btn_play_pause, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(music_app_obj.btn_play_pause,
                              lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(music_app_obj.btn_play_pause, LV_OPA_0, 0);
    lv_obj_align(music_app_obj.btn_play_pause, LV_ALIGN_CENTER, 0, 20);
    const lv_img_dsc_t *img_src = control_provider.bt_speaker_get_status()
                                      ? &img_media_pause
                                      : &img_media_play;
    p_app_media->icon_btn_play_pause = common_icon_button(
        music_app_obj.btn_play_pause, img_src, play_pause_btn_event_cb);
    lv_obj_align(p_app_media->icon_btn_play_pause, LV_ALIGN_CENTER, 0, 0);
    music_app_obj.btn_play_pause_img =
        lv_obj_get_child(p_app_media->icon_btn_play_pause, 0);
    lv_img_set_zoom(music_app_obj.btn_play_pause_img, app_zoom);
    music_app_obj.btn_vol_up = lv_btn_create(p_window);
    lv_obj_set_size(music_app_obj.btn_vol_up, 116, 80);
    lv_obj_set_style_radius(music_app_obj.btn_vol_up, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(music_app_obj.btn_vol_up, lv_color_hex(0xFFFFFF),
                              0);
    lv_obj_align(music_app_obj.btn_vol_up, LV_ALIGN_CENTER, 80, 130);
    lv_obj_set_style_bg_opa(music_app_obj.btn_vol_up, LV_OPA_0, 0);
    lv_obj_t *btn_vol_up = common_icon_button(
        music_app_obj.btn_vol_up, &volume_up, volume_up_btn_event_cb);
    lv_obj_align(btn_vol_up, LV_ALIGN_CENTER, 0, 0);
    music_app_obj.btn_vol_up_img = lv_obj_get_child(btn_vol_up, 0);
    lv_img_set_zoom(music_app_obj.btn_vol_up_img, app_zoom);
    music_app_obj.btn_vol_down = lv_btn_create(p_window);
    lv_obj_set_size(music_app_obj.btn_vol_down, 116, 80);
    lv_obj_set_style_radius(music_app_obj.btn_vol_down, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(music_app_obj.btn_vol_down,
                              lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(music_app_obj.btn_vol_down, LV_ALIGN_CENTER, -80, 130);
    lv_obj_set_style_bg_opa(music_app_obj.btn_vol_down, LV_OPA_0, 0);
    lv_obj_t *btn_vol_down = common_icon_button(
        music_app_obj.btn_vol_down, &volume_down, volume_down_btn_event_cb);
    lv_obj_align(btn_vol_down, LV_ALIGN_CENTER, 0, 0);
    music_app_obj.btn_vol_down_img = lv_obj_get_child(btn_vol_down, 0);
    lv_img_set_zoom(music_app_obj.btn_vol_down_img, app_zoom);

    p_app_media->media_title = lv_label_create(p_window);
    char *title = get_media_title();
    if (title && title[0] != '\0')
    {
        lv_label_set_text(p_app_media->media_title, title);
        lv_obj_clear_flag(music_app_obj.btn_madia_img_mask, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(music_app_obj.btn_madia_img_bg, LV_OBJ_FLAG_HIDDEN);
        lv_img_set_src(music_app_obj.btn_madia_img_bg, MEDIA_IMG);
    }
    else
    {
        lv_obj_add_flag(music_app_obj.btn_madia_img_mask, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(music_app_obj.btn_madia_img_bg, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(p_app_media->media_title, "Media Title");
    }
    lv_obj_set_size(p_app_media->media_title, 350, 100);
    lv_label_set_long_mode(p_app_media->media_title, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(p_app_media->media_title, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(p_app_media->media_title,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(p_app_media->media_title, lv_color_white(), 0);
    lv_obj_align(p_app_media->media_title, LV_ALIGN_CENTER, 0, -85);

    /* Volume icon button (bottom center) */
    app_vol_icon_btn = lv_btn_create(p_window);
    lv_obj_set_size(app_vol_icon_btn, 40, 32);
    lv_obj_set_style_radius(app_vol_icon_btn, 16, 0);
    lv_obj_set_style_bg_color(app_vol_icon_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(app_vol_icon_btn, 20, 0);
    lv_obj_set_style_shadow_width(app_vol_icon_btn, 0, 0);
    lv_obj_align(app_vol_icon_btn, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_add_event_cb(app_vol_icon_btn, app_vol_icon_click_cb,
                        LV_EVENT_ALL, NULL);
    lv_obj_t *app_vol_icon = lv_img_create(app_vol_icon_btn);
    lv_img_set_src(app_vol_icon, &volume_up);
    lv_img_set_zoom(app_vol_icon, 255 * 30 / 85);
    lv_obj_align(app_vol_icon, LV_ALIGN_CENTER, 0, 0);

    /* Volume bar (initially hidden, expands from icon) */
    app_vol_bar = lv_bar_create(p_window);
    lv_bar_set_range(app_vol_bar, 0, 100);
    lv_obj_set_width(app_vol_bar, APP_VOL_BAR_WIDTH);
    lv_obj_set_height(app_vol_bar, 60);
    lv_obj_align_to(app_vol_bar, app_vol_icon_btn,
                    LV_ALIGN_OUT_TOP_MID, 0, -20);
    lv_obj_set_style_bg_color(app_vol_bar, lv_color_hex(0xCDCDCD),
                              LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(app_vol_bar, lv_color_hex(0x2F2F2F),
                              LV_PART_MAIN);
    lv_obj_set_style_radius(app_vol_bar, 16, LV_PART_MAIN);
    lv_obj_set_style_radius(app_vol_bar, 16, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(app_vol_bar, LV_OPA_100, LV_PART_MAIN);
    lv_bar_set_value(app_vol_bar, 0, LV_ANIM_ON);
    lv_obj_add_event_cb(app_vol_bar, app_bar_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(app_vol_bar, LV_OBJ_FLAG_HIDDEN);
    app_vol_bar_expanded = false;

    button_selection_index = 5;
    return p_window;
}

#define WIDGET_ICON_ZOOM_SIZE 0.6
static lv_obj_t *widget_btn_prev_bg = NULL;
static lv_obj_t *widget_btn_next_bg = NULL;
static lv_obj_t *widget_btn_play_pause = NULL;
static lv_obj_t *widget_img_bg = NULL;
static lv_obj_t *widget_selection_bg = NULL;

void clear_media_widget(void)
{
    widget_selection_bg = NULL;
    widget_btn_prev_bg = NULL;
    widget_btn_next_bg = NULL;
    widget_btn_play_pause = NULL;
    widget_img_bg = NULL;
    widget_vol_icon_btn = NULL;
    widget_vol_bar = NULL;
    widget_vol_bar_expanded = false;
    if (widget_vol_bar_collapse_timer)
    {
        lv_timer_del(widget_vol_bar_collapse_timer);
        widget_vol_bar_collapse_timer = NULL;
    }
}

lv_obj_t *lv_media_widget_builder(lv_obj_t *parent)
{
    uint16_t widget_zoom = 256 * WIDGET_ICON_ZOOM_SIZE;
    if (!parent)
    {
        LOG_E("lv_media_widget_builder parent is NULL");
        return NULL;
    }
    lv_obj_t *widget = lv_obj_create(parent);
    lv_obj_set_size(widget, 420, 260);
    lv_obj_set_style_bg_color(widget, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(widget, LV_OPA_100, 0);
    lv_obj_align(widget, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(widget, 50, 0);
    lv_obj_set_style_clip_corner(widget, true, 0);
    lv_obj_clear_flag(widget, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_center(widget);
    widget_img_bg = lv_img_create(widget);
    lv_obj_align(widget_img_bg, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_src(widget_img_bg, MEDIA_IMG);
    lv_obj_set_style_img_opa(widget_img_bg, LV_OPA_30, 0);
    if (!widget)
        return NULL;

    widget_selection_bg = lv_obj_create(widget);
    lv_obj_set_size(widget_selection_bg, 80, 80);
    lv_obj_set_style_radius(widget_selection_bg, 20, 0);
    lv_obj_set_style_bg_color(widget_selection_bg, lv_color_hex(0x737373), 0);
    lv_obj_set_style_bg_opa(widget_selection_bg, LV_OPA_0, 0);
    lv_obj_align(widget_selection_bg, LV_ALIGN_BOTTOM_RIGHT, -25, MEDIA_WIDGET_BTN_Y);
    lv_obj_set_style_border_width(widget_selection_bg, 2, 0);
    lv_obj_set_style_border_color(widget_selection_bg, lv_color_hex(0xFFFFFF),
                                  0);
    lv_obj_set_style_border_opa(widget_selection_bg, LV_OPA_50, 0);

    widget_btn_prev_bg = lv_btn_create(widget);
    lv_obj_set_size(widget_btn_prev_bg, 100, 100);
    lv_obj_set_style_radius(widget_btn_prev_bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(widget_btn_prev_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(widget_btn_prev_bg, LV_ALIGN_BOTTOM_LEFT, 25, MEDIA_WIDGET_BTN_Y+10);
    lv_obj_set_style_bg_opa(widget_btn_prev_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(widget_btn_prev_bg, 2, 0);
    lv_obj_set_style_border_color(widget_btn_prev_bg, lv_color_hex(0xFFFFFF),
                                  0);
    lv_obj_set_style_border_opa(widget_btn_prev_bg, LV_OPA_0, 0);
    lv_obj_t *btn_prev = common_icon_button(
        widget_btn_prev_bg, &img_media_previous, prev_btn_event_cb);
    lv_obj_align(btn_prev, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_zoom(lv_obj_get_child(btn_prev, 0), widget_zoom);
    // lv_obj_set_style_img_opa(lv_obj_get_child(btn_prev, 0), LV_OPA_50, 0);

    widget_btn_next_bg = lv_btn_create(widget);
    lv_obj_set_size(widget_btn_next_bg, 100, 100);
    lv_obj_set_style_radius(widget_btn_next_bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(widget_btn_next_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(widget_btn_next_bg, LV_ALIGN_BOTTOM_RIGHT, -25, MEDIA_WIDGET_BTN_Y+10);
    lv_obj_set_style_bg_opa(widget_btn_next_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(widget_btn_next_bg, 2, 0);
    lv_obj_set_style_border_color(widget_btn_next_bg, lv_color_hex(0xFFFFFF),
                                  0);
    lv_obj_set_style_border_opa(widget_btn_next_bg, LV_OPA_0, 0);
    lv_obj_t *btn_next = common_icon_button(widget_btn_next_bg, &img_media_next,
                                            next_btn_event_cb);
    // lv_obj_set_style_img_opa(lv_obj_get_child(btn_next, 0), LV_OPA_50, 0);
    lv_obj_align(btn_next, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_zoom(lv_obj_get_child(btn_next, 0), widget_zoom);

    widget_btn_play_pause = lv_btn_create(widget);
    lv_obj_set_size(widget_btn_play_pause, 100, 100);
    lv_obj_set_style_radius(widget_btn_play_pause, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(widget_btn_play_pause, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(widget_btn_play_pause, LV_OPA_0, 0);
    lv_obj_align(widget_btn_play_pause, LV_ALIGN_BOTTOM_MID, 0, MEDIA_WIDGET_BTN_Y+10);
    lv_obj_set_style_border_width(widget_btn_play_pause, 2, 0);
    lv_obj_set_style_border_color(widget_btn_play_pause, lv_color_hex(0xFFFFFF),
                                  0);
    lv_obj_set_style_border_opa(widget_btn_play_pause, LV_OPA_0, 0);
    const lv_img_dsc_t *img_src = control_provider.bt_speaker_get_status()
                                      ? &img_media_pause
                                      : &img_media_play;
    p_widget_media->icon_btn_play_pause = common_icon_button(
        widget_btn_play_pause, img_src, play_pause_btn_event_cb);
    lv_obj_align(p_widget_media->icon_btn_play_pause, LV_ALIGN_CENTER, 0, 0);
    music_widget_obj.widget_btn_play_pause_img =
        lv_obj_get_child(p_widget_media->icon_btn_play_pause, 0);
    lv_img_set_zoom(music_widget_obj.widget_btn_play_pause_img, widget_zoom);

    /* Volume icon button (bottom center, below control buttons) */
    widget_vol_icon_btn = lv_btn_create(widget);
    lv_obj_set_size(widget_vol_icon_btn, 40, 32);
    lv_obj_set_style_radius(widget_vol_icon_btn, 16, 0);
    lv_obj_set_style_bg_color(widget_vol_icon_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(widget_vol_icon_btn, 20, 0);
    lv_obj_set_style_shadow_width(widget_vol_icon_btn, 0, 0);
    lv_obj_align(widget_vol_icon_btn, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_add_event_cb(widget_vol_icon_btn, widget_vol_icon_click_cb,
                        LV_EVENT_ALL, NULL);
    lv_obj_t *vol_icon = lv_img_create(widget_vol_icon_btn);
    lv_img_set_src(vol_icon, &volume_up);
    lv_img_set_zoom(vol_icon, 255 * 30 / 85);
    lv_obj_align(vol_icon, LV_ALIGN_CENTER, 0, 0);

    /* Volume bar (initially hidden, expands to cover prev/play/next buttons) */
    widget_vol_bar = lv_bar_create(widget);
    lv_bar_set_range(widget_vol_bar, 0, 100);
    lv_obj_set_width(widget_vol_bar, WIDGET_VOL_BAR_WIDTH);
    lv_obj_set_height(widget_vol_bar, 80);
    lv_obj_align(widget_vol_bar, LV_ALIGN_BOTTOM_MID, 0, MEDIA_WIDGET_BTN_Y );
    lv_obj_set_style_bg_color(widget_vol_bar, lv_color_hex(0xCDCDCD),
                              LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(widget_vol_bar, lv_color_hex(0x2F2F2F),
                              LV_PART_MAIN);
    lv_obj_set_style_radius(widget_vol_bar, 16, LV_PART_MAIN);
    lv_obj_set_style_radius(widget_vol_bar, 16, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(widget_vol_bar, LV_OPA_100, LV_PART_MAIN);
    lv_bar_set_value(widget_vol_bar, 0, LV_ANIM_ON);
    lv_obj_add_event_cb(widget_vol_bar, widget_bar_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(widget_vol_bar, LV_OBJ_FLAG_HIDDEN);
    widget_vol_bar_expanded = false;

    
    p_widget_media->media_title = lv_label_create(widget);
    lv_label_set_text(p_widget_media->media_title, get_media_title());
    lv_obj_set_style_text_opa(p_widget_media->media_title, LV_OPA_70, 0);
    lv_obj_set_size(p_widget_media->media_title, 400, 150);
    lv_label_set_long_mode(p_widget_media->media_title, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(p_widget_media->media_title,
                                LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(p_widget_media->media_title,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(p_widget_media->media_title, lv_color_white(),
                                0);
    lv_obj_align(p_widget_media->media_title, LV_ALIGN_TOP_MID, 0, 15);
    button_selection_index = 5;
    return widget;
}

static lv_timer_t *media_bg_timer = NULL;
// 計時器回調函數，用於恢復透明度
static void media_bg_reset_cb(lv_timer_t *timer)
{
    if (music_app_obj.btn_select_background)
    {
        lv_obj_set_style_bg_opa(music_app_obj.btn_select_background, LV_OPA_0,
                                0);
    }
    lv_timer_del(media_bg_timer);
    media_bg_timer = NULL;
}

void media_btn_press_cb(void)
{
    if (!music_app_obj.btn_select_background)
        return;

    // 立即設置透明度為50
    lv_obj_set_style_bg_opa(music_app_obj.btn_select_background, LV_OPA_80, 0);

    // 創建或重置計時器，500ms後恢復透明度
    if (!media_bg_timer)
    {
        media_bg_timer = lv_timer_create(media_bg_reset_cb, 300, NULL);
    }
    else
    {
        lv_timer_reset(media_bg_timer);
    }

    LOG_D("media_widget_btn_press_anim");
}

static lv_timer_t *media_widget_bg_timer = NULL;
// 計時器回調函數，用於恢復透明度
static void media_widget_bg_reset_cb(lv_timer_t *timer)
{
    if (widget_selection_bg)
    {
        lv_obj_set_style_bg_opa(widget_selection_bg, LV_OPA_0, 0);
    }
    lv_timer_del(media_widget_bg_timer);
    media_widget_bg_timer = NULL;
}

void media_widget_btn_press_cb(void)
{
    if (!widget_selection_bg)
        return;

    // 立即設置透明度為50
    lv_obj_set_style_bg_opa(widget_selection_bg, LV_OPA_80, 0);

    // 創建或重置計時器，500ms後恢復透明度
    if (!media_widget_bg_timer)
    {
        media_widget_bg_timer =
            lv_timer_create(media_widget_bg_reset_cb, 300, NULL);
    }
    else
    {
        lv_timer_reset(media_widget_bg_timer);
    }

    LOG_D("media_widget_btn_press_anim");
}

static void animate_media_widget_selection_bg(lv_coord_t move_offset,
                                              lv_coord_t prev_move_offset)
{
    if (widget_selection_bg == NULL)
        return;

    // 設定動畫
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, widget_selection_bg);
    lv_anim_set_values(&a, prev_move_offset, move_offset);
    lv_anim_set_time(&a, 75);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
    lv_anim_start(&a);

    // Y軸固定
    lv_obj_align(widget_selection_bg, LV_ALIGN_BOTTOM_MID, prev_move_offset,
                 MEDIA_WIDGET_BTN_Y);
}

static void animate_media_selection_bg(lv_coord_t move_offset,
                                       lv_coord_t prev_move_offset)
{
    if (music_app_obj.btn_select_background == NULL)
        return;

    // 設定動畫
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, music_app_obj.btn_select_background);
    lv_anim_set_values(&a, prev_move_offset, move_offset);
    lv_anim_set_time(&a, 75);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
    lv_anim_start(&a);

    // Y軸固定
    lv_obj_align(music_app_obj.btn_select_background, LV_ALIGN_CENTER,
                 prev_move_offset, 30);
}

static uint8_t prev_widget_selection_index = 1;
static lv_coord_t prev_widget_offset = 0;
static lv_coord_t prev_widget_move_offset = 0;
static void set_media_widget_selection_bg_pos(uint8_t selection_index,
                                              lv_coord_t offset)
{
    if (widget_selection_bg == NULL ||
        (prev_widget_selection_index == selection_index &&
         prev_widget_offset == offset))
        return;
    int move_offset = 0;
    switch (selection_index)
    {
    case 0:
        move_offset = -140 + offset;
        if (prev_widget_selection_index != 0)
        {
            animate_media_widget_selection_bg(move_offset,
                                              prev_widget_move_offset);
            motor_pattern_scrolling_app();
        }
        else
            lv_obj_align(widget_selection_bg, LV_ALIGN_BOTTOM_MID, move_offset,
                         MEDIA_WIDGET_BTN_Y);
        break;
    case 1:
        move_offset = offset;
        if (prev_widget_selection_index != 1)
        {
            animate_media_widget_selection_bg(move_offset,
                                              prev_widget_move_offset);
            motor_pattern_scrolling_app();
        }
        else
            lv_obj_align(widget_selection_bg, LV_ALIGN_BOTTOM_MID, move_offset,
                         MEDIA_WIDGET_BTN_Y);
        break;
    case 2:
        move_offset = 140 + offset;
        if (prev_widget_selection_index != 2)
        {
            animate_media_widget_selection_bg(move_offset,
                                              prev_widget_move_offset);
            motor_pattern_scrolling_app();
        }
        else
            lv_obj_align(widget_selection_bg, LV_ALIGN_BOTTOM_MID, move_offset,
                         MEDIA_WIDGET_BTN_Y);
        break;
    default:
        break;
    }
    prev_widget_move_offset = move_offset;
    prev_widget_selection_index = selection_index;
    prev_widget_offset = offset;
}

static uint8_t prev_selection_index = 1;
static lv_coord_t prev_offset = 0;
static lv_coord_t prev_move_offset = 0;
static void set_media_app_selection_bg_pos(uint8_t selection_index,
                                           lv_coord_t offset)
{
    if (music_app_obj.btn_select_background == NULL ||
        (prev_selection_index == selection_index && prev_offset == offset))
        return;
    int move_offset = 0;
    switch (selection_index)
    {
    case 0:
        move_offset = -150 + offset;
        if (prev_selection_index != 0)
        {
            animate_media_selection_bg(move_offset, prev_move_offset);
            motor_pattern_scrolling_app();
        }
        else
            lv_obj_align(music_app_obj.btn_select_background, LV_ALIGN_CENTER,
                         move_offset, 30);
        break;
    case 1:
        move_offset = offset;
        if (prev_selection_index != 1)
        {
            animate_media_selection_bg(move_offset, prev_move_offset);
            motor_pattern_scrolling_app();
        }
        else
            lv_obj_align(music_app_obj.btn_select_background, LV_ALIGN_CENTER,
                         move_offset, 30);
        break;
    case 2:
        move_offset = 150 + offset;
        if (prev_selection_index != 2)
        {
            animate_media_selection_bg(move_offset, prev_move_offset);
            motor_pattern_scrolling_app();
        }
        else
            lv_obj_align(music_app_obj.btn_select_background, LV_ALIGN_CENTER,
                         move_offset, 30);
        break;
    default:
        break;
    }
    prev_move_offset = move_offset;
    prev_selection_index = selection_index;
    prev_offset = offset;
}

static uint8_t media_widget_selection_index = 1;
uint8_t get_media_widget_selection_index(void)
{
    return media_widget_selection_index;
}
void media_widget_trigger_drag_by_py(int p_y)
{
    lv_coord_t diff = 0;
    if (p_y >= 0 && p_y < 155)
    {
        // 將 0~155 映射到 0~-12
        diff = -(12 * p_y) / 155;
        media_widget_selection_index = 2; // 向右拖拽
        set_paused_control_with_arm(true);
    }
    else if (p_y > 311 && p_y <= 466)
    {
        // 將 311~466 映射到 12~0
        diff = 12 - (12 * (p_y - 311)) / (466 - 311);
        media_widget_selection_index = 0; // 向左拖拽
        set_paused_control_with_arm(true);
    }
    else if (p_y >= 155 && p_y <= 311)
    {
        diff = 12 - (24 * (p_y - 155)) / (311 - 155);
        media_widget_selection_index = 1; // 無動作
        set_paused_control_with_arm(false);
    }
    // LOG_D("p_y: %d, diff: %d, media_widget_selection_index: %d", p_y, diff,
    // media_widget_selection_index);
    set_media_widget_selection_bg_pos(media_widget_selection_index, diff);
}

static uint8_t media_selection_index = 1;
void media_trigger_drag_by_py(int p_y)
{
    lv_coord_t diff = 0;
    if (p_y >= 0 && p_y < 155)
    {
        // 將 0~155 映射到 0~-12
        diff = -(12 * p_y) / 155;
        media_selection_index = 2; // 向右拖拽
    }
    else if (p_y > 311 && p_y <= 466)
    {
        // 將 311~466 映射到 12~0
        diff = 12 - (12 * (p_y - 311)) / (466 - 311);
        media_selection_index = 0; // 向左拖拽
    }
    else if (p_y >= 155 && p_y <= 311)
    {
        diff = 12 - (24 * (p_y - 155)) / (311 - 155);
        media_selection_index = 1; // 無動作
    }
    // LOG_D("p_y: %d, diff: %d, media_selection_index: %d", p_y, diff,
    // media_selection_index);
    set_media_app_selection_bg_pos(media_selection_index, diff);
}

void media_widget_tap_event_cb(void)
{
    media_widget_btn_press_cb();
    if (media_widget_selection_index == 0)
    {
        sys_media_event_set(SYS_EVENT_PREV);
    }
    else if (media_widget_selection_index == 2)
    {
        sys_media_event_set(SYS_EVENT_NEXT);
    }
    else if (media_widget_selection_index == 1)
    {
        sys_media_event_set(SYS_EVENT_PLAY_PAUSE);
    }
}

lv_obj_t *media_prev_btn_create(lv_obj_t *parent)
{
    if (!parent)
    {
        LOG_E("media_prev_btn_create parent is NULL");
        return NULL;
    }
    lv_obj_t *btn_prev_bg = lv_img_create(parent);
    lv_obj_set_size(btn_prev_bg, 110, 110);
    lv_img_set_src(btn_prev_bg, &control_selection_bg);
    lv_obj_t *btn_prev =
        common_icon_button(btn_prev_bg, &img_media_previous, prev_btn_event_cb);
    // lv_obj_align(btn_prev, LV_ALIGN_LEFT_MID, 0, 0);
    lv_img_set_zoom(lv_obj_get_child(btn_prev, 0), 256 * WIDGET_ICON_ZOOM_SIZE);
    return btn_prev_bg;
}

lv_obj_t *media_next_btn_create(lv_obj_t *parent)
{
    if (!parent)
    {
        LOG_E("media_next_btn_create parent is NULL");
        return NULL;
    }
    lv_obj_t *btn_next_bg = lv_img_create(parent);
    lv_obj_set_size(btn_next_bg, 110, 110);
    lv_img_set_src(btn_next_bg, &control_selection_bg);
    lv_obj_t *btn_next =
        common_icon_button(btn_next_bg, &img_media_next, next_btn_event_cb);
    // lv_obj_align(btn_next, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_img_set_zoom(lv_obj_get_child(btn_next, 0), 256 * WIDGET_ICON_ZOOM_SIZE);
    return btn_next_bg;
}

lv_obj_t *media_play_pause_btn_create(lv_obj_t *parent)
{
    if (!parent)
    {
        LOG_E("media_play_pause_btn_create parent is NULL");
        return NULL;
    }
    lv_obj_t *btn_play_pause_bg = lv_img_create(parent);
    lv_obj_set_size(btn_play_pause_bg, 110, 110);
    lv_img_set_src(btn_play_pause_bg, &control_selection_bg);
    const lv_img_dsc_t *img_src = control_provider.bt_speaker_get_status()
                                      ? &img_media_pause
                                      : &img_media_play;
    lv_obj_t *btn_play_pause =
        common_icon_button(btn_play_pause_bg, img_src, play_pause_btn_event_cb);
    // lv_obj_align(btn_play_pause, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_img_set_zoom(lv_obj_get_child(btn_play_pause, 0),
                    256 * WIDGET_ICON_ZOOM_SIZE);
    return btn_play_pause_bg;
}

void reset_media_widget(void)
{
    lv_obj_set_style_border_opa(widget_btn_prev_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_opa(widget_btn_play_pause, LV_OPA_0, 0);
    lv_obj_set_style_border_opa(widget_btn_next_bg, LV_OPA_0, 0);
}

void selection_media_widget(uint8_t index)
{
    switch (index)
    {
    case 0:
        lv_obj_set_style_border_opa(widget_btn_prev_bg, LV_OPA_80, 0);
        break;
    case 1:
        lv_obj_set_style_border_opa(widget_btn_play_pause, LV_OPA_80, 0);
        break;
    case 2:
        lv_obj_set_style_border_opa(widget_btn_next_bg, LV_OPA_80, 0);
        break;
    default:
        break;
    }
}

static void title_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        LOG_D("title_btn_event_cb");
        gui_app_run(APP_ID_MEDIA);
    }
}

extern void dial_widget_event(lv_event_t *e);

static lv_obj_t *dial_widget_btn_prev_bg = NULL;
static lv_obj_t *dial_widget_btn_next_bg = NULL;
static lv_obj_t *dial_widget_btn_play_pause = NULL;
static lv_obj_t *dial_widget_btn_play_pause_icon = NULL;

static void set_vol_bar_value(void *param)
{
    if (lv_obj_is_valid(dial_widget_vol_bar))
    {
        uint8_t volume = *(uint8_t *)param;
        lv_bar_set_value(dial_widget_vol_bar, volume, LV_ANIM_ON);
        if (!vol_bar_expanded)
            vol_bar_expand();
        else
            vol_bar_reset_collapse_timer();
    }
}

static lv_obj_t *dial_media_title = NULL;
static lv_obj_t *dial_media_img_bg = NULL;
static void dial_media_widget_init(void);
void lv_dial_media_widget_builder(lv_obj_t *parent)
{
    uint16_t widget_zoom = 256 * WIDGET_ICON_ZOOM_SIZE;

    dial_media_img_bg = lv_img_create(parent);
    lv_obj_align(dial_media_img_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(dial_media_img_bg, LV_OBJ_FLAG_HIDDEN);
    // lv_obj_clear_state(dial_media_img_bg, LV_STATE_PRESSED);
    lv_obj_clear_flag(dial_media_img_bg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_img_opa(dial_media_img_bg, LV_OPA_40, 0);

    // dial_media_title = lv_label_create(parent);
    // lv_obj_set_size(dial_media_title, 300, 50);
    // lv_label_set_long_mode(dial_media_title, LV_LABEL_LONG_DOT);
    // lv_obj_set_style_text_align(dial_media_title, LV_TEXT_ALIGN_CENTER,
    //                             LV_PART_MAIN);
    // lv_obj_set_style_text_font(dial_media_title,
    //                            LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    // lv_obj_set_style_text_color(dial_media_title, lv_color_white(), 0);
    // lv_obj_set_style_text_opa(dial_media_title, LV_OPA_70, 0);
    // lv_obj_align(dial_media_title, LV_ALIGN_TOP_MID, 0, 5);

    // lv_obj_t *title_btn = lv_obj_create(parent);
    // lv_obj_set_size(title_btn, 300, 50);
    // lv_obj_add_event_cb(title_btn, dial_widget_event, LV_EVENT_ALL, NULL);

    // lv_obj_align(title_btn, LV_ALIGN_TOP_MID, 0, 5);
    // lv_obj_set_style_bg_opa(title_btn, LV_OPA_0, 0);

    // char *title = control_provider.get_media_title();
    // if (strlen(title) > 0)
    // {
    //     lv_label_set_text(dial_media_title, title);
    //     lv_obj_clear_flag(dial_media_img_bg, LV_OBJ_FLAG_HIDDEN);
    //     lv_img_set_src(dial_media_img_bg, MEDIA_IMG);
    // }
    // else
    // {
    //     lv_label_set_text(dial_media_title, "Media");
    //     lv_obj_add_flag(dial_media_img_bg, LV_OBJ_FLAG_HIDDEN);
    // }
    // LOG_D("DIAL_MEDIA IMG CREATE:%s, %s", title, MEDIA_IMG);

    dial_widget_btn_prev_bg = lv_btn_create(parent);
    lv_obj_set_size(dial_widget_btn_prev_bg, 100, 100);
    lv_obj_set_style_radius(dial_widget_btn_prev_bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dial_widget_btn_prev_bg, lv_color_hex(0xFFFFFF),
                              0);
    lv_obj_align(dial_widget_btn_prev_bg, LV_ALIGN_LEFT_MID, 10, -20);
    lv_obj_set_style_bg_opa(dial_widget_btn_prev_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(dial_widget_btn_prev_bg, 2, 0);
    lv_obj_set_style_border_color(dial_widget_btn_prev_bg,
                                  lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(dial_widget_btn_prev_bg, LV_OPA_0, 0);
    lv_obj_t *btn_prev = common_icon_button(
        dial_widget_btn_prev_bg, &img_media_previous, prev_btn_event_cb);
    lv_obj_set_style_img_opa(lv_obj_get_child(btn_prev, 0), LV_OPA_70, 0);
    lv_obj_align(btn_prev, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_zoom(lv_obj_get_child(btn_prev, 0), widget_zoom);
    // lv_obj_set_style_img_opa(lv_obj_get_child(btn_prev, 0), LV_OPA_50, 0);
    dial_widget_btn_next_bg = lv_btn_create(parent);
    lv_obj_set_size(dial_widget_btn_next_bg, 100, 100);
    lv_obj_set_style_radius(dial_widget_btn_next_bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dial_widget_btn_next_bg, lv_color_hex(0xFFFFFF),
                              0);
    lv_obj_align(dial_widget_btn_next_bg, LV_ALIGN_RIGHT_MID, -10, -20);
    lv_obj_set_style_bg_opa(dial_widget_btn_next_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(dial_widget_btn_next_bg, 2, 0);
    lv_obj_set_style_border_color(dial_widget_btn_next_bg,
                                  lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(dial_widget_btn_next_bg, LV_OPA_0, 0);
    lv_obj_t *btn_next = common_icon_button(dial_widget_btn_next_bg,
                                            &img_media_next, next_btn_event_cb);
    lv_obj_set_style_img_opa(lv_obj_get_child(btn_next, 0), LV_OPA_70, 0);
    lv_obj_align(btn_next, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_zoom(lv_obj_get_child(btn_next, 0), widget_zoom);
    dial_widget_btn_play_pause = lv_btn_create(parent);
    lv_obj_set_size(dial_widget_btn_play_pause, 100, 100);
    lv_obj_set_style_radius(dial_widget_btn_play_pause, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dial_widget_btn_play_pause,
                              lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(dial_widget_btn_play_pause, LV_OPA_0, 0);
    lv_obj_align(dial_widget_btn_play_pause, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_border_width(dial_widget_btn_play_pause, 2, 0);
    lv_obj_set_style_border_color(dial_widget_btn_play_pause,
                                  lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(dial_widget_btn_play_pause, LV_OPA_0, 0);
    const lv_img_dsc_t *img_src = control_provider.bt_speaker_get_status()
                                      ? &img_media_pause
                                      : &img_media_play;
    dial_widget_btn_play_pause_icon = common_icon_button(
        dial_widget_btn_play_pause, img_src, play_pause_btn_event_cb);
    lv_obj_align(dial_widget_btn_play_pause_icon, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_zoom(lv_obj_get_child(dial_widget_btn_play_pause_icon, 0),
                    widget_zoom);
    lv_obj_set_style_img_opa(
        lv_obj_get_child(dial_widget_btn_play_pause_icon, 0), LV_OPA_70, 0);
    /* Volume icon button (always visible, left side) */
    dial_widget_vol_icon_btn = lv_btn_create(parent);
    lv_obj_set_size(dial_widget_vol_icon_btn, 40, 32);
    lv_obj_set_style_radius(dial_widget_vol_icon_btn, 16, 0);
    lv_obj_set_style_bg_color(dial_widget_vol_icon_btn, lv_color_hex(0xFFFFFF),
                              0);
    lv_obj_set_style_bg_opa(dial_widget_vol_icon_btn, 20, 0);
    lv_obj_set_style_shadow_width(dial_widget_vol_icon_btn, 0, 0);
    lv_obj_align(dial_widget_vol_icon_btn, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_add_event_cb(dial_widget_vol_icon_btn, vol_icon_click_cb,
                        LV_EVENT_ALL, NULL);
    lv_obj_t *vol_icon = lv_img_create(dial_widget_vol_icon_btn);
    lv_img_set_src(vol_icon, &volume_up);
    lv_img_set_zoom(vol_icon, 255 * 30 / 85);
    lv_obj_align(vol_icon, LV_ALIGN_CENTER, 0, 0);

    /* Volume bar (initially hidden, expands left-right from icon) */
    dial_widget_vol_bar = lv_bar_create(parent);
    lv_bar_set_range(dial_widget_vol_bar, 0, 100);
    lv_obj_set_width(dial_widget_vol_bar, VOL_BAR_EXPANDED_WIDTH);
    lv_obj_set_height(dial_widget_vol_bar, 60);
    lv_obj_align_to(dial_widget_vol_bar, dial_widget_vol_icon_btn,
                    LV_ALIGN_OUT_TOP_MID, 0, -20);
    lv_obj_set_style_bg_color(dial_widget_vol_bar, lv_color_hex(0xCDCDCD),
                              LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(dial_widget_vol_bar, lv_color_hex(0x2F2F2F),
                              LV_PART_MAIN);
    lv_obj_set_style_radius(dial_widget_vol_bar, 16, LV_PART_MAIN);
    lv_obj_set_style_radius(dial_widget_vol_bar, 16, LV_PART_INDICATOR);
    // lv_obj_set_style_bg_opa(dial_widget_vol_bar, LV_OPA_50, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(dial_widget_vol_bar, LV_OPA_100, LV_PART_MAIN);
    lv_bar_set_value(dial_widget_vol_bar, 0, LV_ANIM_ON);
    lv_obj_add_event_cb(dial_widget_vol_bar, bar_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(dial_widget_vol_bar, LV_OBJ_FLAG_HIDDEN);
    vol_bar_expanded = false;
    dial_media_widget_init();
    lvgl_msg_handler.handle_media_volume = set_vol_bar_value;
}

void set_dial_media_widget_opa(uint8_t opa)
{
    if (lv_obj_is_valid(dial_widget_btn_prev_bg))
    {
        lv_obj_set_style_img_opa(lv_obj_get_child(lv_obj_get_child(dial_widget_btn_prev_bg, 0),0), opa, 0);
    }
    if (lv_obj_is_valid(dial_widget_btn_play_pause))
    {
        lv_obj_set_style_img_opa(lv_obj_get_child(dial_widget_btn_play_pause_icon, 0), opa, 0);
    }
    if (lv_obj_is_valid(dial_widget_btn_next_bg))
    {
        lv_obj_set_style_img_opa(lv_obj_get_child(lv_obj_get_child(dial_widget_btn_next_bg, 0),0), opa, 0);
    }
    if (lv_obj_is_valid(dial_widget_vol_icon_btn))
    {
        lv_obj_set_style_img_opa(lv_obj_get_child(dial_widget_vol_icon_btn, 0), opa, 0);
        uint8_t btn_opa = opa*20/LV_OPA_COVER;
        lv_obj_set_style_bg_opa(dial_widget_vol_icon_btn, btn_opa, 0);
    }
}

static void handle_dial_media_widget_play_state(void *param)
{
    if (lv_obj_is_valid(dial_widget_btn_play_pause_icon) == false)
    {
        return;
    }
    bool media_state = *(bool *)param;
    change_icon_image(dial_widget_btn_play_pause_icon,
                      media_state ? &img_media_pause : &img_media_play);
}

static void handle_dial_media_widget_title(void *param)
{
    if (lv_obj_is_valid(dial_media_title) == false)
    {
        return;
    }
    char *media_title_text = (char *)param;
    if (media_title_text)
    {
        lv_label_set_text(dial_media_title, media_title_text);
    }
}

static void handle_dial_media_widget_img(void *param)
{
    if (lv_obj_is_valid(dial_media_img_bg) == false)
    {
        return;
    }
    char *media_title_text = (char *)param;
    if (media_title_text && media_title_text[0] != '\0')
    {
        LOG_D("DIAL_MEDIA IMG SET :%s", MEDIA_IMG);
        lv_obj_clear_flag(dial_media_img_bg, LV_OBJ_FLAG_HIDDEN);
        lv_img_set_src(dial_media_img_bg, MEDIA_IMG);
    }
    else
    {
        LOG_D("DIAL_MEDIA IMG HIDE");
        lv_obj_add_flag(dial_media_img_bg, LV_OBJ_FLAG_HIDDEN);
    }
}

/* dial_media_header moved to lv_message_list_layout.c */

static void dial_media_widget_init(void)
{
    lvgl_msg_handler.handle_dial_media_play_state =
        handle_dial_media_widget_play_state;
    lvgl_msg_handler.handle_dial_media_title = handle_dial_media_widget_title;
    lvgl_msg_handler.handle_dial_media_img = handle_dial_media_widget_img;
}

void dial_media_widget_deinit(void)
{
    if (lvgl_msg_handler.handle_dial_media_play_state ==
        handle_dial_media_widget_play_state)
        lvgl_msg_handler.handle_dial_media_play_state = NULL;
    if (lvgl_msg_handler.handle_dial_media_title ==
        handle_dial_media_widget_title)
        lvgl_msg_handler.handle_dial_media_title = NULL;
    if (lvgl_msg_handler.handle_dial_media_img == handle_dial_media_widget_img)
        lvgl_msg_handler.handle_dial_media_img = NULL;
}

/* dial_media_header_init/deinit moved to lv_message_list_layout.c */

static lv_obj_t *lv_create_media_screen(lv_obj_t *scr)
{
    lv_obj_t *cont = lv_obj_create(scr);
    lv_obj_set_size(cont, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_center(cont);
    lv_obj_set_style_bg_opa(cont, LV_OPA_0, 0);
    p_app_media->media_title = lv_label_create(cont);
    char *title = control_provider.get_media_title();
    if (strlen(title) > 0)
    {
        lv_label_set_text(p_app_media->media_title, title);
    }
    else
    {
        lv_label_set_text(p_app_media->media_title, "Media Title");
    }
    lv_obj_set_size(p_app_media->media_title, 300, 200);
    lv_label_set_long_mode(p_app_media->media_title, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(p_app_media->media_title, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(p_app_media->media_title,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(p_app_media->media_title, lv_color_white(), 0);
    lv_obj_align(p_app_media->media_title, LV_ALIGN_TOP_MID, 0, 50);
    music_ui_build(p_app_media, cont, 0.8);
    return cont;
}

static lv_obj_t *lv_create_media_app_screen(lv_obj_t *scr)
{
    lv_obj_t *cont = lv_obj_create(scr);
    lv_obj_set_size(cont, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(cont, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(cont, LV_RADIUS_CIRCLE, 0);
    lv_obj_center(cont);
    // music_app_ui_build(p_app_media);
    return cont;
}

/* Define the action functions for the buttons */
static void next_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        LOG_D("next_btn_event_cb");
        sys_media_event_set(SYS_EVENT_NEXT);
    }
}

static void prev_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        LOG_D("prev_btn_event_cb");
        sys_media_event_set(SYS_EVENT_PREV);
    }
}

static void volume_up_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        sys_media_event_set(SYS_EVENT_VOLUME_BTN_UP);
    }
}

static void volume_down_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        sys_media_event_set(SYS_EVENT_VOLUME_BTN_DOWN);
    }
}

static void first_app_btn_event_cb(lv_event_t *e)
{
    voice_provider.start_v2t();
}

static void second_app_btn_event_cb(lv_event_t *e)
{
    gui_app_run(APP_ID_MOUSE);
}
static void third_app_btn_event_cb(lv_event_t *e)
{
    gui_app_run(APP_ID_FLASHLIGHT);
}
static void fourth_app_btn_event_cb(lv_event_t *e)
{
    gui_app_run(APP_ID_RECORDER);
}

static void reset_btn_state(void)
{
    lv_obj_set_style_shadow_opa(music_app_obj.btn_play_pause, LV_OPA_0, 0);
    lv_img_set_zoom(music_app_obj.btn_play_pause_img,
                    256 * WIDGET_ICON_ZOOM_SIZE);
    lv_obj_set_style_shadow_opa(music_app_obj.btn_prev_bg, LV_OPA_0, 0);
    lv_img_set_zoom(music_app_obj.btn_prev_img, 256 * WIDGET_ICON_ZOOM_SIZE);
    lv_obj_set_style_shadow_opa(music_app_obj.btn_next_bg, LV_OPA_0, 0);
    lv_img_set_zoom(music_app_obj.btn_next_img, 256 * WIDGET_ICON_ZOOM_SIZE);
    lv_obj_set_style_shadow_opa(music_app_obj.btn_vol_down, LV_OPA_0, 0);
    lv_img_set_zoom(music_app_obj.btn_vol_down_img,
                    256 * WIDGET_ICON_ZOOM_SIZE);
    lv_obj_set_style_shadow_opa(music_app_obj.btn_vol_up, LV_OPA_0, 0);
    lv_img_set_zoom(music_app_obj.btn_vol_up_img, 256 * WIDGET_ICON_ZOOM_SIZE);
}

static void button_selection(gesture_position_t gesture_position)
{
    media_trigger_drag_by_py(gesture_position.gesture_position_y);
}

// 添加計時器回調函數
static void clear_highlight_cb(void *param)
{
    // 清除所有按鈕的高亮
    lv_obj_set_style_bg_opa(music_app_obj.btn_play_pause, LV_OPA_0, 0);
    lv_obj_set_style_bg_opa(music_app_obj.btn_prev_bg, LV_OPA_0, 0);
    lv_obj_set_style_bg_opa(music_app_obj.btn_next_bg, LV_OPA_0, 0);
    lv_obj_set_style_bg_opa(music_app_obj.btn_vol_up, LV_OPA_0, 0);
    lv_obj_set_style_bg_opa(music_app_obj.btn_vol_down, LV_OPA_0, 0);
}

// 添加計時器句柄
static rt_timer_t highlight_timer;
static void start_highlight_timer(void)
{
    if (!highlight_timer)
    {
        highlight_timer = rt_timer_create(
            "highlight_timer", clear_highlight_cb, RT_NULL,
            rt_tick_from_millisecond(300), RT_TIMER_FLAG_ONE_SHOT);
    }
    else
    {
        rt_timer_stop(highlight_timer);
    }
    rt_timer_start(highlight_timer);
}

static void stop_highlight_timer(void)
{
    if (highlight_timer)
    {
        rt_timer_stop(highlight_timer);
    }
}

static void set_volume_control_value(uint8_t value)
{
    // 更新進度條和音量設置
    lv_bar_set_value(music_app_obj.volume_bar, value, LV_ANIM_ON);
    control_provider.bt_speaker_set_volume(value, true);
}

bool get_volume_control_status(void)
{
    return open_volume_control;
}
uint8_t get_volume_control_value(void)
{
    return volume_control_value;
}

// 修改 handle_tap_event 函數
static void handle_tap_event(void)
{
    media_btn_press_cb();
    if (media_selection_index == 0)
    {
        sys_media_event_set(SYS_EVENT_PREV);
    }
    else if (media_selection_index == 2)
    {
        sys_media_event_set(SYS_EVENT_NEXT);
    }
    else if (media_selection_index == 1)
    {
        sys_media_event_set(SYS_EVENT_PLAY_PAUSE);
    }
}

void media_widget_handle_tap_event(void)
{
    handle_tap_event();
}

void media_widget_handle_press_event(uint8_t press)
{
    LOG_D("media_widget_handle_press_event:%d", press);
    if (press == 1)
    {
        handle_tap_event();
    }
}
static void play_pause_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        LOG_D("play_pause_btn_event_cb");
        sys_media_event_set(SYS_EVENT_PLAY_PAUSE);
    }
}

static void handle_media_play_state(void *param)
{
    if (lv_obj_is_valid(p_app_media->icon_btn_play_pause) == false)
    {
        return;
    }
    bool media_state = *(bool *)param;
    change_icon_image(p_app_media->icon_btn_play_pause,
                      media_state ? &img_media_pause : &img_media_play);
}

static void handle_media_widget_play_state(void *param)
{
    if (lv_obj_is_valid(p_widget_media->icon_btn_play_pause) == false)
    {
        return;
    }
    bool media_state = *(bool *)param;
    change_icon_image(p_widget_media->icon_btn_play_pause,
                      media_state ? &img_media_pause : &img_media_play);
}

static void handle_media_title(void *param)
{
    if (lv_obj_is_valid(p_app_media->media_title) == false)
    {
        return;
    }
    char *media_title_text = (char *)param;
    if (media_title_text && media_title_text[0] != '\0')
    {
        lv_label_set_text(p_app_media->media_title, media_title_text);
    }
    else
    {
        lv_label_set_text(p_app_media->media_title, "Media Title");
    }
}

static void handle_media_img(void *param)
{
    if (lv_obj_is_valid(music_app_obj.btn_madia_img_bg) == false ||
        lv_obj_is_valid(music_app_obj.btn_madia_img_mask) == false)
    {
        return;
    }
    char *media_title_text = (char *)param;
    if (media_title_text && media_title_text[0] != '\0')
    {
        lv_obj_clear_flag(music_app_obj.btn_madia_img_mask, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(music_app_obj.btn_madia_img_bg, LV_OBJ_FLAG_HIDDEN);
        lv_img_set_src(music_app_obj.btn_madia_img_bg, MEDIA_IMG);
    }
    else
    {
        lv_obj_add_flag(music_app_obj.btn_madia_img_mask, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(music_app_obj.btn_madia_img_bg, LV_OBJ_FLAG_HIDDEN);
    }
}

static void handle_media_widget_title(void *param)
{
    if (lv_obj_is_valid(p_widget_media->media_title) == false)
    {
        return;
    }
    char *media_title_text = (char *)param;
    if (media_title_text)
    {
        lv_label_set_text(p_widget_media->media_title, media_title_text);
    }
}

static void handle_media_widget_img(void *param)
{
    if (lv_obj_is_valid(widget_img_bg) == false)
    {
        return;
    }
    char *media_title_text = (char *)param;
    if (media_title_text && media_title_text[0] != '\0')
    {
        LOG_D("WIDGET MEDIA IMG SET: %s", MEDIA_IMG);
        lv_obj_clear_flag(widget_img_bg, LV_OBJ_FLAG_HIDDEN);
        lv_img_set_src(widget_img_bg, MEDIA_IMG);
    }
    else
    {
        LOG_D("WIDGET MEDIA IMG HIDE");
        lv_obj_add_flag(widget_img_bg, LV_OBJ_FLAG_HIDDEN);
    }
}

void build_media_contorll_widget(app_media_t *p_app_media, lv_obj_t *parent)
{
    p_app_media->media_title = lv_label_create(parent);
    lv_label_set_text(p_app_media->media_title, "Media Title");
    lv_obj_set_size(p_app_media->media_title, 300, 100);
    lv_label_set_long_mode(p_app_media->media_title, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(p_app_media->media_title, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(p_app_media->media_title,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(p_app_media->media_title, lv_color_white(), 0);
    lv_obj_align(p_app_media->media_title, LV_ALIGN_TOP_MID, 0, 0);
    music_ui_build(p_app_media, parent, 0.8);
}

static void media_page_control(gesture_position_t control)
{
    LOG_D("media_page_control:%d", control);
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_MEDIA_CONTROL;
    msg.data.media_control = control;
    lvgl_send_msg(msg);
}

void media_widget_start(void)
{
    // screen_rotate_to_90_degree();
    RT_ASSERT(NULL == p_widget_media);
    p_widget_media = (app_media_t *)rt_malloc(sizeof(app_media_t));
    memset(p_widget_media, 0, sizeof(app_media_t));
    // music_app_ui_build(scr);
    // set_open_control_options(true);
    LOG_D("media_widget_start");
    // lvgl_msg_handler.handle_widgets_control = button_selection;
    // lvgl_msg_handler.handle_tap_indicator = media_widget_handle_press_event;
    lvgl_msg_handler.handle_app_media_play_state =
        handle_media_widget_play_state;
    lvgl_msg_handler.handle_app_media_title = handle_media_widget_title;
    lvgl_msg_handler.handle_app_media_img = handle_media_widget_img;
    // lvgl_msg_handler.handle_media_control = button_selection;
}

void media_widget_stop(void)
{
    if (p_widget_media)
    {
        rt_free(p_widget_media);
        p_widget_media = NULL;
    }
    LOG_D("media_widget_stop");
    // set_open_control_options(false);
    // screen_rotate_back_to_original_direction();
    // lvgl_msg_handler.handle_widgets_control = NULL;
    // lvgl_msg_handler.handle_tap_indicator = NULL;
    lvgl_msg_handler.handle_app_media_play_state = NULL;
    lvgl_msg_handler.handle_app_media_title = NULL;
    lvgl_msg_handler.handle_app_media_img = NULL;
    // lvgl_msg_handler.handle_media_control = NULL;
}

void media_on_start(lv_obj_t *scr)
{
    // screen_rotate_to_90_degree();
    RT_ASSERT(NULL == p_app_media);
    p_app_media = (app_media_t *)rt_malloc(sizeof(app_media_t));
    memset(p_app_media, 0, sizeof(app_media_t));
    music_app_ui_build(scr);
    set_open_control_options(true);
    lvgl_msg_handler.handle_widgets_control = button_selection;
    lvgl_msg_handler.handle_tap_indicator = media_widget_handle_press_event;
    lvgl_msg_handler.handle_app_media_play_state = handle_media_play_state;
    lvgl_msg_handler.handle_app_media_title = handle_media_title;
    lvgl_msg_handler.handle_app_media_img = handle_media_img;
    lvgl_msg_handler.handle_media_volume = set_app_vol_bar_value;
}

void media_on_resume(void)
{
    LOG_D("media_app_on_resume");
#ifdef BSP_USING_UI_HANDLER
    if (gesture_open)
    {
        // lvgl_msg_handler.handle_tap_indicator = handle_finger_event;
        // lvgl_msg_handler.handle_media_control = button_selection;
        // lvgl_msg_handler.handle_volume_control = set_volume_control_value;
    }
#endif
}

void media_on_pause(void)
{
    LOG_D("media_app_on_pause");
#ifdef BSP_USING_UI_HANDLER
    // lvgl_msg_handler.handle_tap_indicator = NULL;
    // lvgl_msg_handler.handle_media_control = NULL;
    // lvgl_msg_handler.handle_volume_control = NULL;
#endif
    // stop_highlight_timer();
}

void media_on_stop(void)
{
    if (p_app_media)
    {
        rt_free(p_app_media);
        p_app_media = NULL;
    }
    LOG_D("media_widget_stop");
    set_open_control_options(false);
    screen_rotate_back_to_original_direction();
    lvgl_msg_handler.handle_widgets_control = NULL;
    lvgl_msg_handler.handle_tap_indicator = NULL;
    lvgl_msg_handler.handle_app_media_play_state = NULL;
    lvgl_msg_handler.handle_app_media_title = NULL;
    lvgl_msg_handler.handle_media_control = NULL;
    lvgl_msg_handler.handle_app_media_img = NULL;
    lvgl_msg_handler.handle_media_volume = NULL;
    app_vol_icon_btn = NULL;
    app_vol_bar = NULL;
    app_vol_bar_expanded = false;
    if (app_vol_bar_collapse_timer)
    {
        lv_timer_del(app_vol_bar_collapse_timer);
        app_vol_bar_collapse_timer = NULL;
    }
}

#ifdef APP_ID_MEDIA

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        media_on_start(lv_scr_act());
        break;

    case GUI_APP_MSG_ONRESUME:
        media_on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        media_on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        media_on_stop();
        break;

    default:
        break;
    }
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_MEDIA, msg_handler);

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(media), IMG_ITUNES, APP_ID_MEDIA, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/