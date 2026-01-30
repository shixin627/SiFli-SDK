/**
 ******************************************************************************
 * @file   app_clock_main.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
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
#include "rtconfig.h"
#include <time.h>
#include "app_mainmenu.h"
#include "common_widget.h"
#include "app_clock_main.h"
#include "app_clock_status_bar.h"
#include "watch_system_core_task.h"
#include "lvsf_gesture.h"
#include "lvsf.h"
#ifdef RT_USING_XIP_MODULE
    #include "dlmodule.h"
    #include "dlfcn.h"
    #include "dfs_posix.h"
#endif /* RT_USING_XIP_MODULE */
#include "watch_system_interact.h"
#ifdef BSP_USING_MODEL_WATCH_GLOBAL_DATA
    #include "watch_global_data.h"
#endif
#include "ui_helper.h"
#include "ui_img_helper.h"
#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
#endif
#ifdef BSP_USING_BLOC_SETTING
    #include "bloc_setting.h"
#endif
#include "bloc_peripheral.h"
#include "bloc_weather.h"

#define DBG_TAG "app.clock"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

LV_IMG_DECLARE(img_clock);
LV_IMG_DECLARE(common_background_vague);
LV_IMG_DECLARE(weather_sun);

static const char *battery_level_img_dsc[] = {
    APP_ELC_5, APP_ELC_20, APP_ELC_40, APP_ELC_60, APP_ELC_80, APP_ELC_100,
};

#define APP_ID "clock"
#define APP_CLOCK_ID_MAX_LEN 8
#define CLOCK_UPDATE_INTERVAL_IN_MS 100

typedef enum
{
    STATE_DEINIT = 0,
    STATE_PAUSED, /*clock be inited and paused*/
    STATE_ACTIVE,
} CLOCK_STATE;

/**
 *  description of one clock
 *
 */
typedef struct
{
    lv_obj_t *parent;                  //!< clock's root parent obj
    char id[APP_CLOCK_ID_MAX_LEN + 1]; //!< clock's name
    const app_clock_ops_t *ops;        //!< clock UI state cbk func
    uint8_t state;
    rt_list_t node; //!< list node for link all clocks
    void *mod;
} app_clock_desc_t;

/**
 *
 *
 */
typedef struct
{
    lv_obj_t *tileview;     //!< tileview object for clock framework
    lv_obj_t *tileview_btn; //!< tileview overlay button to capture events
    lv_point_t *p_tileview_valid_pos; //!< tileview valid positions
    rt_uint32_t app_clock_list_len;   //!< clock list length
    rt_list_t list;                   //!< head of all clocks list

    rt_timer_t soft_timer; //!<  template vaiable for simulator
    lv_obj_t *app_list_time_h;
    lv_obj_t *app_list_time_m;
    lv_obj_t *app_list_time_symbol;
    lv_obj_t *app_list_time_bg;
    lv_obj_t *app_list_weather_icon;
    lv_obj_t *app_list_bluetooth_disconnection;
    lv_obj_t *app_list_battery_bg;
    lv_obj_t *app_list_battery;
    lv_obj_t *app_list_battery_label;
    lv_obj_t *top_bar;    // 畫面上方黑條
    lv_obj_t *bottom_bar; // 畫面下方黑條
} app_clock_main_t;

#ifndef BSP_USING_LVGL_INPUT_AGENT
static
#endif
    app_clock_main_t *p_app_clock_main = NULL;

void set_app_list_time_opa(uint8_t opa)
{
    if (lv_obj_is_valid(p_app_clock_main->app_list_time_h) == 0 ||
        lv_obj_is_valid(p_app_clock_main->app_list_time_m) == 0 ||
        lv_obj_is_valid(p_app_clock_main->app_list_time_symbol) == 0 ||
        lv_obj_is_valid(p_app_clock_main->app_list_weather_icon) == 0)
    {
        LOG_W("app_list_time_h is not valid");
        return;
    }
    lv_obj_set_style_text_opa(p_app_clock_main->app_list_time_h, opa, 0);
    lv_obj_set_style_text_opa(p_app_clock_main->app_list_time_m, opa, 0);
    lv_obj_set_style_text_opa(p_app_clock_main->app_list_time_symbol, opa, 0);
    lv_obj_set_style_img_opa(p_app_clock_main->app_list_weather_icon, opa, 0);
}

static lv_obj_t *charge_icon_obj = NULL;

/**
 * @brief 對單一物件進行縮放，根據物件類型自動選擇縮放方法
 * @param obj 要縮放的物件
 * @param zoom_factor 縮放比例 (256 = 100%, 128 = 50%, 384 = 150%)
 * @note 文字物件 (label) 不會進行縮放
 */
static void scale_single_obj(lv_obj_t *obj, uint16_t zoom_factor)
{
    if (!lv_obj_is_valid(obj))
    {
        return;
    }

    // 文字物件不進行縮放
    if (lv_obj_check_type(obj, &lv_label_class))
    {
        return;
    }

    // 檢查是否為圖片物件
    if (lv_obj_check_type(obj, &lv_img_class))
    {
        // 圖片物件使用 lv_img_set_zoom
        lv_img_set_zoom(obj, zoom_factor);
    }
    else
    {
        // 一般物件使用 style transform zoom
        lv_obj_set_size(obj, (lv_obj_get_width(obj) * zoom_factor) / 250,
                        (lv_obj_get_height(obj) * zoom_factor) / 250);
    }
}

/**
 * @brief 縮放指定物件及其在父容器中位於上方的所有兄弟物件
 * @param obj 目標物件，函式會縮放此物件及其上方所有物件
 * @param zoom_factor 縮放比例 (256 = 100%, 128 = 50%, 384 = 150%)
 * @note LV_IMG_ZOOM_NONE = 256 表示 100%
 *       使用比例: zoom_factor = 256 * percentage / 100
 *       例如: 80% = 256 * 80 / 100 = 204
 *             50% = 256 * 50 / 100 = 128
 */
void scale_obj_and_above(lv_obj_t *parent, uint16_t zoom_factor)
{
    if (!lv_obj_is_valid(parent))
    {
        LOG_W("scale_obj_and_above: object is not valid");
        return;
    }

    // 取得目標物件在父容器中的索引
    int32_t target_index = lv_obj_get_index(parent);
    int32_t child_cnt = lv_obj_get_child_cnt(parent);

    // 縮放目標物件及其上方所有物件 (索引 0 到 target_index)
    for (int32_t i = 0; i <= target_index && i < child_cnt; i++)
    {
        lv_obj_t *child = lv_obj_get_child(parent, i);
        LOG_D("Scaling child index %d", i);
        if (lv_obj_is_valid(child))
        {
            scale_single_obj(child, zoom_factor);
        }
    }
}

/**
 * @brief 按百分比縮放指定物件及其在父容器中位於上方的所有兄弟物件
 * @param obj 目標物件，函式會縮放此物件及其上方所有物件
 * @param percentage 縮放百分比 (100 = 原始大小, 50 = 縮小一半)
 */
void scale_obj_and_above_percent(lv_obj_t *obj, uint8_t percentage)
{
    if (percentage > 200)
    {
        percentage = 200; // 限制最大放大到 200%
    }
    uint16_t zoom_factor = (uint16_t)(256 * percentage / 100);
    scale_obj_and_above(obj, zoom_factor);
}

void set_app_list_battery_opa(uint8_t opa)
{
    if (lv_obj_is_valid(p_app_clock_main->app_list_battery) == 0 ||
        lv_obj_is_valid(p_app_clock_main->app_list_battery_label) == 0 ||
        lv_obj_is_valid(charge_icon_obj) == 0)
    {
        LOG_W("app_list_battery is not valid");
        return;
    }
    lv_obj_set_style_img_opa(p_app_clock_main->app_list_battery, opa, 0);
    lv_obj_set_style_text_opa(p_app_clock_main->app_list_battery_label, opa, 0);
    lv_obj_set_style_img_opa(charge_icon_obj, opa, 0);
    lv_obj_set_style_img_opa(p_app_clock_main->app_list_bluetooth_disconnection,
                             opa, 0);
}

lv_obj_t *get_app_list_time_bg(void)
{
    return p_app_clock_main->app_list_time_bg;
}

void set_app_list_time_bg_opa(uint8_t opa)
{
    if (lv_obj_is_valid(get_app_list_time_bg()))
    {
        lv_obj_set_style_bg_opa(get_app_list_time_bg(), opa, 0);
    }
}

lv_obj_t *get_app_list_bluetooth_disconnection(void)
{
    return p_app_clock_main->app_list_bluetooth_disconnection;
}

lv_obj_t *get_app_list_battery(void)
{
    return p_app_clock_main->app_list_battery;
}

lv_obj_t *get_app_list_battery_label(void)
{
    return p_app_clock_main->app_list_battery_label;
}

lv_obj_t *get_app_list_battery_bg(void)
{
    return p_app_clock_main->app_list_battery_bg;
}

void set_app_list_battery_bg_opa(uint8_t opa)
{
    if (lv_obj_is_valid(get_app_list_battery_bg()))
    {
        lv_obj_set_style_bg_opa(get_app_list_battery_bg(), opa, 0);
    }
}

void show_battery(bool show)
{
    if (!lv_obj_is_valid(get_app_list_battery_bg()))
    {
        return;
    }
    if (show)
    {
        lv_obj_clear_flag(get_app_list_battery_bg(), LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(get_app_list_battery_bg(), LV_OBJ_FLAG_HIDDEN);
    }
}

void show_app_list_time(bool show)
{
    if (!lv_obj_is_valid(get_app_list_time_bg()))
    {
        return;
    }
    if (show)
    {
        lv_obj_clear_flag(get_app_list_time_bg(), LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(get_app_list_time_bg(), LV_OBJ_FLAG_HIDDEN);
    }
}

static void refresh_charge_icon(void *param)
{
    if (lv_obj_is_valid(charge_icon_obj))
    {
        if (SkaiWatchSys.charger_status > NoCharge)
        {
            lv_obj_clear_flag(charge_icon_obj, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(charge_icon_obj, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void set_battery_image(lv_obj_t *img, uint8_t battery_level)
{
    const char *img_dsc;
    if (battery_level < 5)
    {
        img_dsc = battery_level_img_dsc[0];
    }
    else if (battery_level < 20)
    {
        img_dsc = battery_level_img_dsc[1];
    }
    else if (battery_level < 40)
    {
        img_dsc = battery_level_img_dsc[2];
    }
    else if (battery_level < 60)
    {
        img_dsc = battery_level_img_dsc[3];
    }
    else if (battery_level < 80)
    {
        img_dsc = battery_level_img_dsc[4];
    }
    else
    {
        img_dsc = battery_level_img_dsc[5];
    }

    if (lv_obj_is_valid(img))
    {
        lv_img_set_src(img, img_dsc);
    }
    else
    {
        LOG_E("set_battery_image: img is invalid");
    }
}

static rt_uint32_t total_milliseconds = 0;
static clock_t latched_clock;
static uint16_t last_active_clock = 0;
uint8_t get_last_active_clock(void)
{
    return last_active_clock + 1;
}

void app_clock_main_get_current_time(app_clock_time_t *t)
{
    if (t)
    {
        uint32_t ms;
        rt_uint32_t cur_seconds;
        clock_t clk = clock();
        clock_t elp;

        if (clk >= latched_clock)
        {
            elp = clk - latched_clock;
        }
        else
        {
            elp = INT32_MAX - latched_clock + 1 + clk;
        }
        latched_clock = clk;

        ms = (uint64_t)elp * 1000 / RT_TICK_PER_SECOND + total_milliseconds;
        cur_seconds = ms / 1000;

        t->ms = ms % 1000;
        t->s = cur_seconds % 60;
        t->h = (cur_seconds / 3600) % 24;
        t->m = (cur_seconds / 60) % 60;

        total_milliseconds = ms;

#ifdef GRAPHIC_REFRESH_TIME_ANALYSIS
        if (refer_ana_enable)
        {
            t->h = 10;
            t->m = 10;
            t->s = 37;

            {
                lv_disp_t *disp;
                lv_area_t scr_area;

                disp = lv_disp_get_default();

                scr_area.x1 = 0;
                scr_area.y1 = 0;
                scr_area.x2 = lv_disp_get_hor_res(disp) - 1;
                scr_area.y2 = lv_disp_get_ver_res(disp) - 1;

                _lv_inv_area(disp, &scr_area);
            }
        }
#endif

        t->day = SkaiWatchSys.Global_Time.day;
    }
}

static const char *app_clock_state_to_name(uint8_t state)
{
#define STATE_TO_NAME_CASE(e)                                                  \
    case e:                                                                    \
        return #e
    switch (state)
    {
        STATE_TO_NAME_CASE(STATE_DEINIT);
        STATE_TO_NAME_CASE(STATE_PAUSED);
        STATE_TO_NAME_CASE(STATE_ACTIVE);

    default:
        return "UNKNOW";
    }
}

static char *change_context;
static lv_obj_t *clk_parent;
char *app_clock_change_context(void)
{
    return change_context;
}

lv_obj_t *gui_app_get_clock_parent(void)
{
    return clk_parent;
}

static void show_dial_widget_select_list(lv_obj_t *parent);
static uint8_t dial_widget_app_id = app_id_media;
static bool edit_mode = false;

void dial_widget_event(lv_event_t *e)
{
    static bool dial_widget_press_valid = false;
    static lv_point_t press_point = {0, 0};
    static uint32_t start_press_time = 0;
    lv_indev_t *indev;
    lv_point_t cur;
    lv_event_code_t code = lv_event_get_code(e);
    switch (e->code)
    {
        break;
    case LV_EVENT_PRESSED:
    {
        indev = lv_indev_get_act();
        if (indev)
        {
            dial_widget_press_valid = true;
            lv_indev_get_point(indev, &press_point);
        }
        start_press_time = rt_tick_get_millisecond();
        break;
    }
    case LV_EVENT_PRESSING:
    {
        if (!dial_widget_press_valid)
            return;
        indev = lv_indev_get_act();
        if (indev)
        {
            lv_indev_get_point(indev, &cur);
            if (abs(cur.x - press_point.x) > 10 ||
                abs(cur.y - press_point.y) > 10)
            {
                dial_widget_press_valid = false;
            }
            rt_time_t press_time = rt_tick_get_millisecond();
            if (press_time - start_press_time >= 1000)
            {
                show_dial_widget_select_list(lv_scr_act());
                dial_widget_press_valid = false;
            }
        }
        break;
    }
    case LV_EVENT_RELEASED:
    {
        if (dial_widget_press_valid)
        {
            dial_widget_press_valid = false;
            if (edit_mode)
                show_dial_widget_select_list(lv_scr_act());
            else
            {
                switch (dial_widget_app_id)
                {
                case app_id_calendar:
                    gui_app_run(APP_ID_CALENDAR);
                    break;
                case app_id_media:
                    gui_app_run(APP_ID_MEDIA);
                    break;
                case app_id_weather:
                    gui_app_run(APP_ID_WEATHER);
                    break;
                default:
                    break;
                }
            }
        }
        break;
    }
    default:
        break;
    }
}

extern void write_clock_widget_number(void);
static lv_obj_t *dial_widget = NULL;
static lv_obj_t *dial_widget_img_bg = NULL;
static void swich_dial_widget_builder(uint8_t app_id, lv_obj_t *parent);
// 生成一個列表用於切換 dial_widget_app_id，選擇後自動刪除
static lv_obj_t *dial_widget_select_list = NULL;
static void dial_widget_select_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED)
    {
        uint32_t id = (uint32_t)lv_event_get_user_data(e);
        dial_widget_app_id = (uint8_t)id;
        SkaiWatchSys.clock_widget_num = dial_widget_app_id;
        write_clock_widget_number();
        // 關閉並刪除列表
        if (lv_obj_is_valid(dial_widget_select_list))
        {
            lv_obj_del(dial_widget_select_list);
            dial_widget_select_list = NULL;
        }
        // 立即刷新 widget
        if (p_app_clock_main)
        {
            rt_list_t *pos;
            rt_list_for_each(pos, (&p_app_clock_main->list))
            {
                app_clock_desc_t *clk_desc =
                    rt_list_entry(pos, app_clock_desc_t, node);
                if (clk_desc->state == STATE_ACTIVE && clk_desc->parent &&
                    lv_obj_is_valid(clk_desc->parent))
                {

                    lv_obj_del(dial_widget);
                    swich_dial_widget_builder(dial_widget_app_id,
                                              clk_desc->parent);
                    break;
                }
            }
        }
    }
}

static void show_dial_widget_select_list(lv_obj_t *parent)
{
    if (dial_widget_select_list && lv_obj_is_valid(dial_widget_select_list))
    {
        lv_obj_del(dial_widget_select_list);
        dial_widget_select_list = NULL;
    }
    dial_widget_select_list = lv_list_create(parent);
    lv_obj_set_size(dial_widget_select_list, 466, 466);
    lv_obj_align(dial_widget_select_list, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *btn_calendar =
        lv_list_add_btn(dial_widget_select_list, NULL, "Calendar");
    lv_obj_add_event_cb(btn_calendar, dial_widget_select_event_cb,
                        LV_EVENT_CLICKED, (void *)app_id_calendar);

    lv_obj_t *btn_media =
        lv_list_add_btn(dial_widget_select_list, NULL, "Media");
    lv_obj_add_event_cb(btn_media, dial_widget_select_event_cb,
                        LV_EVENT_CLICKED, (void *)app_id_media);

    lv_obj_t *btn_weather =
        lv_list_add_btn(dial_widget_select_list, NULL, "Weather");
    lv_obj_add_event_cb(btn_weather, dial_widget_select_event_cb,
                        LV_EVENT_CLICKED, (void *)app_id_weather);
}

extern void dial_calendar_widget_deinit(void);
extern void dial_media_widget_deinit(void);
extern void dial_weather_widget_deinit(void);
extern void lv_dial_calendar_widget_builder(lv_obj_t *parent);
extern void lv_dial_media_widget_builder(lv_obj_t *parent);
extern void lv_dial_weather_widget_builder(lv_obj_t *parent);
static void swich_dial_widget_builder(uint8_t app_id, lv_obj_t *parent)
{
    LOG_I("swich_dial_widget_builder to app_id=%d", app_id);
    dial_media_widget_deinit();
    dial_calendar_widget_deinit();
    dial_weather_widget_deinit();
    dial_widget = lv_obj_create(parent);
    lv_obj_set_style_radius(dial_widget, 50, LV_PART_MAIN);
    lv_obj_align(dial_widget, LV_ALIGN_CENTER, 0, 115);
    lv_obj_set_size(dial_widget, 330, 150);
    lv_obj_set_style_bg_color(dial_widget, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(dial_widget, LV_OPA_80, 0);
    lv_obj_set_style_border_width(dial_widget, 2, 0);
    lv_obj_set_style_border_color(dial_widget, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(dial_widget, LV_OPA_10, 0);
    lv_obj_set_style_clip_corner(dial_widget, true, 0);     // 啟用裁切
    lv_obj_clear_flag(dial_widget, LV_OBJ_FLAG_SCROLLABLE); // 禁用滾動

    dial_widget_img_bg = lv_img_create(dial_widget);
    lv_obj_set_style_radius(dial_widget_img_bg, 50, LV_PART_MAIN);
    lv_img_set_src(dial_widget_img_bg, GAUS_DEFAULT_PICTURE);
    lv_img_set_zoom(dial_widget_img_bg, 256 * 2); // 100%
    lv_obj_align(dial_widget_img_bg, LV_ALIGN_CENTER, 0, -115);
    if (app_id == app_id_calendar)
    {
        request_calendar_on_mobile(false);
        lv_dial_calendar_widget_builder(dial_widget);
    }
    else if (app_id == app_id_media)
    {

        lv_dial_media_widget_builder(dial_widget);
    }
    else if (app_id == app_id_weather)
    {
        request_weather_within_six_hours(false);
        lv_dial_weather_widget_builder(dial_widget);
    }
}

static void swich_dial_widget_deinit(uint8_t app_id)
{
    if (app_id == app_id_calendar)
    {
        dial_calendar_widget_deinit();
    }
    else if (app_id == app_id_media)
    {
        dial_media_widget_deinit();
    }
    else if (app_id == app_id_weather)
    {
        dial_weather_widget_deinit();
    }
}
static void get_clock_main_status_img_path(char *clk_id)
{
    char folder[64];
    snprintf(folder, sizeof(folder), "/%s", clk_id);
    DIR *dir = opendir(folder);
    int found = 0;
    extern char *GAUS_DEFAULT_PICTURE;
    if (dir)
    {
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL)
        {
            if (strncmp(entry->d_name, "picture_", 8) == 0)
            {
                // 取 picture_ 後面的檔名
                const char *filename = entry->d_name + 8;
                static char gaus_path[128];
                snprintf(gaus_path, sizeof(gaus_path),
                         "/assets/gaus_images/gaus_%s", filename);
                if (GAUS_DEFAULT_PICTURE)
                {
                    GAUS_DEFAULT_PICTURE = strdup(gaus_path);
                }
                found = 1;
                break;
            }
        }
        closedir(dir);
    }
    if (!found)
    {
        if (GAUS_DEFAULT_PICTURE)
        {
            GAUS_DEFAULT_PICTURE =
                strdup("/assets/gaus_images/gaus_default_picture.bin");
        }
    }
}

extern void set_clock_main_status_img(const void *img_src);
static void clock_change_page(char *clk_id)
{
    get_clock_main_status_img_path(clk_id);
    LOG_D("clock_change_page: %s", clk_id);
    if (strcmp(clk_id, "JW_wf1") == 0)
    {
        lv_img_set_src(dial_widget_img_bg, GAUS_CLOCK1_BG);
        lv_img_set_zoom(dial_widget_img_bg, 256 * 2); // 100%
        lv_obj_align(dial_widget_img_bg, LV_ALIGN_CENTER, 0, -115);
        set_clock_main_status_img(GAUS_CLOCK1_BG);
    }
    else if (strcmp(clk_id, "JW_wf2") == 0)
    {
        lv_img_set_src(dial_widget_img_bg, GAUS_DEFAULT_PICTURE);
        lv_img_set_zoom(dial_widget_img_bg, 256 * 2); // 100%
        lv_obj_align(dial_widget_img_bg, LV_ALIGN_CENTER, 0, -115);
        set_clock_main_status_img(GAUS_DEFAULT_PICTURE);
    }
    else if (strcmp(clk_id, "JW_wf3") == 0)
    {
        lv_img_set_src(dial_widget_img_bg, GAUS_DEFAULT_PICTURE);
        lv_img_set_zoom(dial_widget_img_bg, 256 * 2); // 100%
        lv_obj_align(dial_widget_img_bg, LV_ALIGN_CENTER, 0, -115);
        set_clock_main_status_img(GAUS_DEFAULT_PICTURE);
    }
    else if (strcmp(clk_id, "JW_wf4") == 0)
    {
        lv_img_set_src(dial_widget_img_bg, GAUS_CLOCK4_BG);
        lv_img_set_zoom(dial_widget_img_bg, 256 * 2); // 100%
        lv_obj_align(dial_widget_img_bg, LV_ALIGN_CENTER, 0, -115);
        set_clock_main_status_img(GAUS_CLOCK4_BG);
    }
    else if (strcmp(clk_id, "JW_wf5") == 0)
    {
        lv_img_set_src(dial_widget_img_bg, GAUS_CLOCK5_BG);
        lv_img_set_zoom(dial_widget_img_bg, 256 * 2); // 100%
        lv_obj_align(dial_widget_img_bg, LV_ALIGN_CENTER, 0, -115);
        set_clock_main_status_img(GAUS_CLOCK5_BG);
    }
}

static void app_clock_change_state(app_clock_desc_t *p_clock, uint8_t new_state)
{
    if (p_clock->state == new_state)
        return;

    change_context = p_clock->id;
    clk_parent = p_clock->parent;

    switch (new_state)
    {
    case STATE_DEINIT:
    {
        if (STATE_ACTIVE == p_clock->state)
        {
            LOG_D("Deiniting active clock id=%s", p_clock->id);
            if (p_clock->ops->pause)
            {
                p_clock->ops->pause();
                if (strcmp(p_clock->id, "JW_wf1") == 0)
                    swich_dial_widget_deinit(dial_widget_app_id);
                else if (strcmp(p_clock->id, "JW_wf3") == 0)
                    swich_dial_widget_deinit(dial_widget_app_id);
            }
        }
        LOG_D("Deiniting clock id=%s", p_clock->id);
        if (p_clock->ops->deinit)
        {
            p_clock->ops->deinit();
        }

        if (p_clock->parent && lv_obj_is_valid(p_clock->parent))
        {
            if (lv_obj_get_child_cnt(p_clock->parent) > 0)
            {
                lv_obj_clean(p_clock->parent);
            }
        }
    }
    break;

    case STATE_PAUSED:
    {
        if (STATE_ACTIVE == p_clock->state)
        {
            if (p_clock->ops->pause)
            {
                p_clock->ops->pause();
                if (strcmp(p_clock->id, "JW_wf1") == 0)
                    swich_dial_widget_deinit(dial_widget_app_id);
                else if (strcmp(p_clock->id, "JW_wf3") == 0)
                    swich_dial_widget_deinit(dial_widget_app_id);
            }
        }
        else if (p_clock->ops->init)
        {
            p_clock->ops->init(p_clock->parent);
            if (strcmp(p_clock->id, "JW_wf1") == 0)
                swich_dial_widget_builder(dial_widget_app_id, p_clock->parent);
            else if (strcmp(p_clock->id, "JW_wf3") == 0)
                swich_dial_widget_builder(dial_widget_app_id, p_clock->parent);
        }
    }
    break;

    case STATE_ACTIVE:
    {
        if (STATE_DEINIT == p_clock->state)
        {
            if (p_clock->ops->init)
            {
                p_clock->ops->init(p_clock->parent);
                if (strcmp(p_clock->id, "JW_wf1") == 0)
                {
                    swich_dial_widget_builder(dial_widget_app_id,
                                              p_clock->parent);
                }
                else if (strcmp(p_clock->id, "JW_wf3") == 0)
                {
                    swich_dial_widget_builder(dial_widget_app_id,
                                              p_clock->parent);
                }
            }
        }
        LOG_D("Resuming clock id11=%s", p_clock->id);
        clock_change_page(p_clock->id);
        if (p_clock->ops->resume)
        {
            p_clock->ops->resume();
        }
    }
    break;

    default:
        break;
    }

    p_clock->state = new_state;
}

static void app_clock_change_state_by_id(uint16_t idx, uint8_t new_state)
{
    uint16_t i = 0;
    rt_list_t *pos;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        app_clock_desc_t *clk_desc = rt_list_entry(pos, app_clock_desc_t, node);

        if (idx == i)
        {
            app_clock_change_state(clk_desc, new_state);
            break;
        }
        i++;
    }
}

static void app_clock_main_select(uint16_t clock_idx)
{
    rt_uint16_t left_clock_idx, right_clock_idx, i;
    rt_list_t *pos;
    app_clock_desc_t *clk_desc;

    if (clock_idx >= p_app_clock_main->app_clock_list_len)
        clock_idx = p_app_clock_main->app_clock_list_len - 1;

    if (clock_idx > 0)
        left_clock_idx = clock_idx - 1;
    else
        left_clock_idx =
            p_app_clock_main->app_clock_list_len; // invalid left clock

    right_clock_idx = clock_idx + 1;

    if (right_clock_idx > p_app_clock_main->app_clock_list_len)
        right_clock_idx =
            p_app_clock_main->app_clock_list_len; // invalid right clock

    /*deinit all other clock , to free memory*/
    i = 0;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        clk_desc = rt_list_entry(pos, app_clock_desc_t, node);

        if ((i != clock_idx) && (i != left_clock_idx) && (i != right_clock_idx))
        {
            app_clock_change_state(clk_desc, STATE_DEINIT);
        }
        i++;
    }

    /*pause left&right clock , to free memory*/
    i = 0;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        clk_desc = rt_list_entry(pos, app_clock_desc_t, node);

        if ((i == left_clock_idx) || (i == right_clock_idx))
        {
            app_clock_change_state(clk_desc, STATE_PAUSED);
        }
        i++;
    }

    /* active selected clock */
    i = 0;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        clk_desc = rt_list_entry(pos, app_clock_desc_t, node);

        if (i == clock_idx)
        {
            app_clock_change_state(clk_desc, STATE_ACTIVE);
        }

        i++;
    }

    last_active_clock = clock_idx;
#ifdef BSP_USING_BLOC_SETTING
    setting_provider.set_watch_face(last_active_clock);
#endif
}

static void app_clock_main_drag_begin(uint16_t clock_idx)
{
    rt_uint16_t left_clock_idx, right_clock_idx, i;
    rt_list_t *pos;
    app_clock_desc_t *clk_desc;

    if (clock_idx >= p_app_clock_main->app_clock_list_len)
        clock_idx = p_app_clock_main->app_clock_list_len - 1;

    if (clock_idx > 0)
        left_clock_idx = clock_idx - 1;
    else
        left_clock_idx =
            p_app_clock_main->app_clock_list_len; // invalid left clock

    right_clock_idx = clock_idx + 1;

    if (right_clock_idx > p_app_clock_main->app_clock_list_len)
        right_clock_idx =
            p_app_clock_main->app_clock_list_len; // invalid right clock

    /* active the left&right clock*/
    i = 0;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        clk_desc = rt_list_entry(pos, app_clock_desc_t, node);

        if ((i == left_clock_idx) || (i == right_clock_idx))
        {
            app_clock_change_state(clk_desc, STATE_ACTIVE);
        }

        i++;
    }
}

static void tileview_bar_anim(void *bg, int32_t v)
{
    lv_obj_set_y(bg, v);
}

extern void choose_photo_list(uint16_t page);
extern void set_status_bar_area_up_state(bool state);
extern void set_status_bar_area_down_state(bool state);
extern void set_status_bar_area_left_state(bool state);
extern void set_status_bar_area_right_state(bool state);
static void tileview_event_cb(lv_event_t *event)
{
    static lv_point_t press_point = {0, 0};
    static bool press_valid = false;
    static bool longpress_fired = false;
    static uint32_t press_time = 0;
    const int MOVE_THRESHOLD = 20;     // px
    const uint32_t LONGPRESS_MS = 600; // 可自訂長按毫秒
    lv_indev_t *indev;
    lv_point_t cur;
    int dx, dy;
    uint32_t now;
    switch (event->code)
    {
        break;
    case LV_EVENT_VALUE_CHANGED:
    {
        rt_uint16_t active_pos = (rt_uint16_t)lv_event_get_param(event);

        if (gui_app_is_actived("Main"))
            app_clock_main_select(active_pos);
        else
            last_active_clock = active_pos;
    }
    break;
#if (LV_HOR_RES_MAX == 240) &&                                                 \
    (LV_HOR_RES_MAX == 240) // Active neighbor clock may cause malloc mem
                            // failure in high resolution
    case LV_EVENT_SCROLL_BEGIN:
    {
        app_clock_main_drag_begin(last_active_clock);
    }
    break;
#endif
    case LV_EVENT_PRESSED:
        // 記錄按下時座標與時間
        indev = lv_indev_get_act();
        if (indev)
        {
            lv_indev_get_point(indev, &press_point);
            press_valid = true;
            longpress_fired = false;
            press_time = lv_tick_get();
        }
        break;
    case LV_EVENT_PRESSING:
        if (!press_valid || longpress_fired)
            break;
        indev = lv_indev_get_act();
        if (indev)
        {
            lv_indev_get_point(indev, &cur);
            dx = cur.x - press_point.x;
            dy = cur.y - press_point.y;
            if (dx < 0)
                dx = -dx;
            if (dy < 0)
                dy = -dy;
            if (dx > MOVE_THRESHOLD || dy > MOVE_THRESHOLD)
            {
                press_valid = false; // 超過閾值，失效
                break;
            }
            now = lv_tick_get();
            if (now - press_time >= LONGPRESS_MS)
            {
                // 觸發自訂長按
                longpress_fired = true;
                press_valid = false;
                if (p_app_clock_main &&
                    lv_obj_is_valid(p_app_clock_main->tileview))
                {
                    lv_obj_add_flag(p_app_clock_main->tileview,
                                    LV_OBJ_FLAG_SCROLLABLE);
                    edit_mode = true;
                    LOG_D("Long press tileview btn hidden");
                    set_status_bar_area_up_state(false);
                    set_status_bar_area_down_state(false);
                    set_status_bar_area_left_state(false);
                    set_status_bar_area_right_state(false);
                    // 顯示上下黑條動畫
                    if (lv_obj_is_valid(p_app_clock_main->top_bar))
                    {
                        lv_obj_clear_flag(p_app_clock_main->top_bar,
                                          LV_OBJ_FLAG_HIDDEN);
                        lv_obj_set_y(
                            p_app_clock_main->top_bar,
                            -lv_obj_get_height(p_app_clock_main->top_bar));
                        lv_anim_t a;
                        lv_anim_init(&a);
                        lv_anim_set_var(&a, p_app_clock_main->top_bar);
                        lv_anim_set_values(
                            &a, -lv_obj_get_height(p_app_clock_main->top_bar),
                            0);
                        lv_anim_set_time(&a, 200);
                        lv_anim_set_exec_cb(
                            &a, (lv_anim_exec_xcb_t)tileview_bar_anim);
                        lv_anim_start(&a);
                    }
                    if (lv_obj_is_valid(p_app_clock_main->bottom_bar))
                    {
                        lv_obj_clear_flag(p_app_clock_main->bottom_bar,
                                          LV_OBJ_FLAG_HIDDEN);
                        lv_coord_t scr_h = lv_disp_get_ver_res(NULL);
                        lv_coord_t bar_h =
                            lv_obj_get_height(p_app_clock_main->bottom_bar);
                        lv_obj_set_y(p_app_clock_main->bottom_bar, scr_h);
                        lv_anim_t a;
                        lv_anim_init(&a);
                        lv_anim_set_var(&a, p_app_clock_main->bottom_bar);
                        lv_anim_set_values(&a, bar_h, 0);
                        lv_anim_set_time(&a, 200);
                        lv_anim_set_exec_cb(
                            &a, (lv_anim_exec_xcb_t)tileview_bar_anim);
                        lv_anim_start(&a);
                    }
                    LOG_D("Long press");
                }
            }
        }
        break;
    case LV_EVENT_RELEASED:
        if (edit_mode && (lv_tick_get() - press_time < LONGPRESS_MS) &&
            press_valid)
        {
            uint16_t active_pos = last_active_clock + 1;
            if (active_pos == 2 || active_pos == 3)
                choose_photo_list(active_pos);
        }
        press_valid = false;
        longpress_fired = false;
        break;
    default:
        break;
    }
}

static void tileview_bar_hide_anim_ready_cb(lv_anim_t *a)
{
    lv_obj_t *obj = (lv_obj_t *)a->var;
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

static void dial_editing_exit_event_cb(lv_event_t *event)
{
    // 長按 tileview 時縮小
    if (p_app_clock_main && lv_obj_is_valid(p_app_clock_main->tileview))
    {
        // scale_obj_and_above_percent(p_app_clock_main->tileview, 50); //
        lv_obj_clear_flag(p_app_clock_main->tileview, LV_OBJ_FLAG_SCROLLABLE);
        edit_mode = false;
        LOG_D("Long press tileview btn shown");

        set_status_bar_area_up_state(true);
        set_status_bar_area_down_state(true);
        set_status_bar_area_left_state(true);
        set_status_bar_area_right_state(true);
        // // 顯示上下黑條動畫
        if (lv_obj_is_valid(p_app_clock_main->top_bar))
        {
            // 先將 top_bar 移到畫面外
            lv_obj_set_y(p_app_clock_main->top_bar,
                         -lv_obj_get_height(p_app_clock_main->top_bar));
            // 動畫移動進入
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, p_app_clock_main->top_bar);
            lv_anim_set_values(&a, 0,
                               -lv_obj_get_height(p_app_clock_main->top_bar));
            lv_anim_set_time(&a, 200);
            lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)tileview_bar_anim);
            lv_anim_set_ready_cb(&a, tileview_bar_hide_anim_ready_cb);
            lv_anim_start(&a);
        }
        if (lv_obj_is_valid(p_app_clock_main->bottom_bar))
        {
            lv_coord_t scr_h = lv_disp_get_ver_res(NULL);
            lv_coord_t bar_h = lv_obj_get_height(p_app_clock_main->bottom_bar);
            lv_obj_set_y(p_app_clock_main->bottom_bar, scr_h);
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, p_app_clock_main->bottom_bar);
            lv_anim_set_values(&a, 0, bar_h);
            lv_anim_set_time(&a, 200);
            lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)tileview_bar_anim);
            lv_anim_set_ready_cb(&a, tileview_bar_hide_anim_ready_cb);
            lv_anim_start(&a);
        }
        LOG_D("Long press");
    }
    LOG_D("dial_editing_exit_event_cb");
}

extern const lv_img_dsc_t *weather_icon_get(char *weather);
static void refersh_time(T_UTC_TIME *current_time)
{
    char time_str[3];
    LOG_D("refersh_time:%d:%d:%d", current_time->hour, current_time->minutes,
          current_time->seconds);
    if (lv_obj_is_valid(p_app_clock_main->app_list_time_h) &&
        lv_obj_is_valid(p_app_clock_main->app_list_time_m))
    {
        rt_snprintf(time_str, 3, "%02d", current_time->hour);
        lv_label_set_text(p_app_clock_main->app_list_time_h, time_str);
        rt_snprintf(time_str, 3, "%02d", current_time->minutes);
        lv_label_set_text(p_app_clock_main->app_list_time_m, time_str);
    }
    else
    {
        LOG_W("app_list_time is not valid");
    }
}

void refersh_weather_icon(void)
{
    weather_t *get_weather_data = get_weather(WEATHER_TODAT_ITEM_AMOUNT - 1);
    if (lv_obj_is_valid(p_app_clock_main->app_list_weather_icon) == 0)
    {
        LOG_W("app_list_weather_icon is not valid");
        return;
    }
    lv_img_set_src(p_app_clock_main->app_list_weather_icon,
                   weather_icon_get(get_weather_data->description));
}

static void refersh_battery(uint8_t battery_level)
{
    char battery_str[5];
    rt_snprintf(battery_str, 5, "%d%%", battery_level);
    if (p_app_clock_main)
    {
        if (lv_obj_is_valid(p_app_clock_main->app_list_battery_label))
        {
            lv_label_set_text(p_app_clock_main->app_list_battery_label,
                              battery_str);
            if (SkaiWatchSys.charger_status == InCharging)
            {
                lv_obj_set_style_text_color(
                    p_app_clock_main->app_list_battery_label,
                    lv_color_hex(0x00CC00), 0);
            }
            else
            {
                lv_obj_set_style_text_color(
                    p_app_clock_main->app_list_battery_label,
                    lv_color_hex(0xFFFFFF), 0);
            }
        }
        else
        {
            LOG_W("app_list_battery_label is not valid");
        }
        refresh_charge_icon(NULL);
        set_battery_image(p_app_clock_main->app_list_battery, battery_level);
    }
    else
    {
        LOG_W("app_list_battery is NULL");
    }
}

static bool is_bluetooth_connected = false;
bool get_bluetooth_connection_status(void)
{
    return is_bluetooth_connected;
}

// 斷線提示視窗相關變數
static lv_obj_t *connection_tips_window = NULL;
static lv_obj_t *disconnect_icon = NULL;
static lv_obj_t *tips_label = NULL;
static lv_timer_t *connection_tips_timer = NULL;

// 淡出動畫完成後的回調函式
static void fade_out_anim_ready_cb(lv_anim_t *anim)
{
    // 動畫完成後刪除視窗
    if (lv_obj_is_valid(connection_tips_window))
    {
        lv_obj_del(connection_tips_window);
        connection_tips_window = NULL;
    }
}

static void set_connection_tips_window_opa(void *obj, int32_t opa)
{
    if (lv_obj_is_valid(connection_tips_window))
    {
        uint8_t window_opa = opa * LV_OPA_90 / LV_OPA_100;
        lv_obj_set_style_bg_opa(connection_tips_window, window_opa,
                                LV_PART_MAIN);
        uint8_t border_opa = opa * LV_OPA_50 / LV_OPA_100;
        lv_obj_set_style_border_opa(connection_tips_window, border_opa,
                                    LV_PART_MAIN);
        lv_obj_set_style_text_opa(tips_label, opa, LV_PART_MAIN);
        lv_obj_set_style_img_opa(disconnect_icon, opa, LV_PART_MAIN);
    }
}

// 執行淡出動畫的函式
static void start_fade_out_animation(void)
{
    if (lv_obj_is_valid(connection_tips_window))
    {
        // 創建淡出動畫
        lv_anim_t fade_out_anim;
        lv_anim_init(&fade_out_anim);
        lv_anim_set_var(&fade_out_anim, connection_tips_window);
        lv_anim_set_exec_cb(&fade_out_anim, set_connection_tips_window_opa);
        lv_anim_set_values(&fade_out_anim, LV_OPA_100, LV_OPA_TRANSP);
        lv_anim_set_time(&fade_out_anim, 1000); // 500毫秒淡出
        lv_anim_set_path_cb(&fade_out_anim, lv_anim_path_ease_in);
        lv_anim_set_ready_cb(&fade_out_anim, fade_out_anim_ready_cb);
        lv_anim_start(&fade_out_anim);
    }
}

// 自動關閉提示視窗的計時器回調函式
static void connection_tips_timer_cb(lv_timer_t *timer)
{
    // 清理計時器
    if (connection_tips_timer)
    {
        lv_timer_del(connection_tips_timer);
        connection_tips_timer = NULL;
    }
    // 開始淡出動畫
    start_fade_out_animation();
}

// 銷毀斷線提示視窗的函式
void destroy_connection_tips(void)
{
    if (connection_tips_timer)
    {
        lv_timer_del(connection_tips_timer);
        connection_tips_timer = NULL;
    }

    // 如果視窗存在，執行淡出動畫而不是直接刪除
    if (lv_obj_is_valid(connection_tips_window))
    {
        lv_obj_del(connection_tips_window);
        connection_tips_window = NULL;
    }
}

// 創建斷線提示視窗
void create_connection_tips(void)
{
    // 如果視窗已存在，先銷毀舊的
    if (connection_tips_window != NULL)
    {
        destroy_connection_tips();
    }
    // 創建提示視窗背景
    connection_tips_window = lv_obj_create(lv_scr_act());
    lv_obj_set_size(connection_tips_window, 280, 80);
    lv_obj_align(connection_tips_window, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_color(connection_tips_window, lv_color_hex(0x000000),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(connection_tips_window, LV_OPA_90, LV_PART_MAIN);
    lv_obj_set_style_border_color(connection_tips_window,
                                  lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_border_width(connection_tips_window, 1, LV_PART_MAIN);
    lv_obj_set_style_border_opa(connection_tips_window, LV_OPA_50,
                                LV_PART_MAIN);
    lv_obj_set_style_radius(connection_tips_window, 50, LV_PART_MAIN);
    lv_obj_clear_flag(connection_tips_window, LV_OBJ_FLAG_SCROLLABLE);
    // 創建斷線圖示
    disconnect_icon = lv_img_create(connection_tips_window);
    lv_img_set_src(disconnect_icon, ICON_BLUETOOTH_DISCONNECTION);
    lv_obj_align(disconnect_icon, LV_ALIGN_LEFT_MID, 15, 0);
    // 創建提示文字
    tips_label = lv_label_create(connection_tips_window);
    lv_label_set_text(tips_label, "disconnected");
    lv_obj_set_style_text_color(tips_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(
        tips_label, LV_EXT_FONT_GET(get_system_font_size(0)), LV_PART_MAIN);
    lv_obj_align(tips_label, LV_ALIGN_CENTER, 20, 0);
    // 創建4秒後自動關閉的計時器
    connection_tips_timer =
        lv_timer_create(connection_tips_timer_cb, 1000, NULL);
    lv_timer_set_repeat_count(connection_tips_timer, 1);
}

static void refresh_bluetooth_disconnection(bool connected)
{
    is_bluetooth_connected = connected;
    if (lv_obj_is_valid(p_app_clock_main->app_list_bluetooth_disconnection))
    {
        if (connected)
        {
            lv_obj_add_flag(p_app_clock_main->app_list_bluetooth_disconnection,
                            LV_OBJ_FLAG_HIDDEN);
            // 當重新連接時，銷毀斷線提示視窗
            destroy_connection_tips();
        }
        else
        {
            lv_obj_clear_flag(
                p_app_clock_main->app_list_bluetooth_disconnection,
                LV_OBJ_FLAG_HIDDEN);
        }
    }
    else
    {
        LOG_W("app_list_bluetooth_disconnection is not valid");
    }
}

static T_UTC_TIME aligned_time;
static lv_timer_t *gui_state_update_timer = NULL;

void app_list_main_time_update(void)
{
    aligned_time = SkaiWatchSys.Global_Time;
    refersh_time(&aligned_time);
}

static void gui_state_update_timer_callback(lv_timer_t *timer)
{
#ifndef BSP_USING_PC_SIMULATOR
    // get current time
    time_t now = get_current_time();
    struct tm *time_info;
    time_info = localtime(&now);
    SkaiWatchSys.Global_Time.year = time_info->tm_year + 1900;
    SkaiWatchSys.Global_Time.month = time_info->tm_mon + 1;
    SkaiWatchSys.Global_Time.day = time_info->tm_mday;
    SkaiWatchSys.Global_Time.hour = time_info->tm_hour;
    SkaiWatchSys.Global_Time.minutes = time_info->tm_min;
    SkaiWatchSys.Global_Time.seconds = time_info->tm_sec;
    SkaiWatchSys.Global_Time.weekday = time_info->tm_wday;
    if (aligned_time.minutes != SkaiWatchSys.Global_Time.minutes)
    {
        app_list_main_time_update();
    }
    check_is_at_control_center();
    check_is_at_home();
#endif
}

static void gui_state_update_timer_start(void)
{
    if (!gui_state_update_timer)
    {
        gui_state_update_timer =
            lv_timer_create(gui_state_update_timer_callback, 1000, NULL);
        LOG_D("gui_state_update_timer created");
    }
}

static void gui_state_update_timer_stop(void)
{
    if (gui_state_update_timer)
    {
        lv_timer_del(gui_state_update_timer);
        gui_state_update_timer = NULL;
        LOG_D("gui_state_update_timer_stop");
    }
}

static lv_timer_t *open_widget_list_timer = NULL;
static void open_widget_list_timer_callback(lv_timer_t *timer)
{
    animate_to_app_list();
    set_user_want_to_open_display_to_app_list(false);
    lv_timer_del(open_widget_list_timer);
}

static void open_widget_list_timer_start(void)
{
    if (!open_widget_list_timer)
    {
        open_widget_list_timer =
            lv_timer_create(open_widget_list_timer_callback, 250, NULL);
    }
}

static void handle_watchface_changed_cb(void *param)
{
    uint8_t index = *(uint8_t *)param;
    if (p_app_clock_main && p_app_clock_main->tileview)
        lv_obj_set_tile_id(p_app_clock_main->tileview, index, 0, true);
}
static void on_tap(void)
{
    LOG_D("Tap in Clock");
}

static void update_time_symbol(void)
{
    if (lv_obj_is_valid(p_app_clock_main->app_list_time_symbol))
    {
        if (lv_obj_has_flag(p_app_clock_main->app_list_time_symbol,
                            LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(p_app_clock_main->app_list_time_symbol,
                              LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(p_app_clock_main->app_list_time_symbol,
                            LV_OBJ_FLAG_HIDDEN);
        }
    }
}
static void colon_blink_timer_cb(lv_timer_t *timer)
{
    update_time_symbol();
}

static lv_timer_t *colon_blink_timer = NULL;
void top_digital_time_builder(lv_obj_t *parent)
{
    p_app_clock_main->app_list_time_bg = lv_obj_create(parent);
    lv_obj_set_style_bg_color(p_app_clock_main->app_list_time_bg,
                              lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(p_app_clock_main->app_list_time_bg, LV_OPA_0, 0);
    lv_obj_set_size(p_app_clock_main->app_list_time_bg, 466, 50);
    lv_obj_align(p_app_clock_main->app_list_time_bg, LV_ALIGN_TOP_MID, -10, 0);
    lv_obj_clear_flag(p_app_clock_main->app_list_time_bg,
                      LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *app_list_time_h =
        lv_label_create(p_app_clock_main->app_list_time_bg);
    lv_obj_set_style_text_align(app_list_time_h, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(app_list_time_h,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_align(app_list_time_h, LV_ALIGN_CENTER, -20, 0);
    lv_obj_t *app_list_time_m =
        lv_label_create(p_app_clock_main->app_list_time_bg);
    lv_obj_set_style_text_align(app_list_time_m, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(app_list_time_m,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_align(app_list_time_m, LV_ALIGN_CENTER, 21, 0);
    lv_obj_t *app_list_time_symbol =
        lv_label_create(p_app_clock_main->app_list_time_bg);
    lv_obj_set_style_text_align(app_list_time_symbol, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(app_list_time_symbol,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_align(app_list_time_symbol, LV_ALIGN_CENTER, 0, -1);
    lv_obj_t *app_list_weather_icon =
        lv_img_create(p_app_clock_main->app_list_time_bg);
    weather_t *get_weather_data = get_weather(WEATHER_TODAT_ITEM_AMOUNT - 1);
    lv_img_set_src(app_list_weather_icon,
                   weather_icon_get(get_weather_data->description));
    lv_img_set_zoom(app_list_weather_icon, 128); // zoom 80%
    lv_obj_align(app_list_weather_icon, LV_ALIGN_CENTER, 58, 0);
    p_app_clock_main->app_list_weather_icon = app_list_weather_icon;
    uint8_t minutes = SkaiWatchSys.Global_Time.minutes;
    uint8_t hour = SkaiWatchSys.Global_Time.hour;
    char time_str[3];
    rt_snprintf(time_str, 3, "%02d", hour); // hh
    lv_label_set_text(app_list_time_h, time_str);
    rt_snprintf(time_str, 3, "%02d", minutes); // mm
    lv_label_set_text(app_list_time_m, time_str);
    lv_label_set_text(app_list_time_symbol, ":");

    lv_obj_set_style_text_opa(app_list_time_h, LV_OPA_TRANSP, 0);
    p_app_clock_main->app_list_time_h = app_list_time_h;
    lv_obj_set_style_text_opa(app_list_time_m, LV_OPA_TRANSP, 0);
    p_app_clock_main->app_list_time_m = app_list_time_m;
    lv_obj_set_style_text_opa(app_list_time_symbol, LV_OPA_TRANSP, 0);
    p_app_clock_main->app_list_time_symbol = app_list_time_symbol;
    lv_obj_set_style_img_opa(app_list_weather_icon, LV_OPA_TRANSP, 0);
    p_app_clock_main->app_list_weather_icon = app_list_weather_icon;

    if (colon_blink_timer == NULL)
    {
        colon_blink_timer = lv_timer_create(colon_blink_timer_cb, 1000, NULL);
    }
}

static void battery_status_indicator_builder(lv_obj_t *parent)
{
    lv_obj_t *app_list_battery_bg = lv_obj_create(parent);
    lv_obj_set_style_bg_color(app_list_battery_bg, lv_color_black(),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(app_list_battery_bg, LV_OPA_0, 0);
    lv_obj_set_size(app_list_battery_bg, 466, 80);
    lv_obj_align(app_list_battery_bg, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_clear_flag(app_list_battery_bg, LV_OBJ_FLAG_CLICKABLE);
    p_app_clock_main->app_list_battery_bg = app_list_battery_bg;

    lv_obj_t *app_list_battery = lv_img_create(parent);
    lv_obj_align(app_list_battery, LV_ALIGN_TOP_MID, 30, 18);
    lv_obj_set_style_img_opa(app_list_battery, LV_OPA_TRANSP, 0);
    p_app_clock_main->app_list_battery = app_list_battery;

    lv_obj_t *app_list_battery_label = lv_label_create(parent);
    lv_obj_set_style_text_align(app_list_battery_label, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_align(app_list_battery_label, LV_ALIGN_TOP_MID, -30, 12);
    lv_obj_set_style_text_opa(app_list_battery_label, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(app_list_battery_label, lv_color_white(), 0);
    p_app_clock_main->app_list_battery_label = app_list_battery_label;

    charge_icon_obj = lv_img_create(app_list_battery);
    lv_img_set_src(charge_icon_obj, CHARGE_ICON);
    lv_obj_align(charge_icon_obj, LV_ALIGN_CENTER, -2, -1);
    lv_obj_set_style_img_opa(charge_icon_obj, LV_OPA_TRANSP, 0);

    lv_obj_t *app_list_bluetooth_disconnection = lv_img_create(parent);
    lv_img_set_src(app_list_bluetooth_disconnection,
                   ICON_BLUETOOTH_DISCONNECTION);
    lv_obj_align(app_list_bluetooth_disconnection, LV_ALIGN_TOP_MID, 0, 45);
    lv_obj_set_style_img_opa(app_list_bluetooth_disconnection, LV_OPA_TRANSP,
                             0);
    p_app_clock_main->app_list_bluetooth_disconnection =
        app_list_bluetooth_disconnection;

    lvgl_msg_handler.handle_charge_status = refresh_charge_icon;
}

static void clock_main_top_bar_builder(lv_obj_t *parent)
{
    p_app_clock_main->top_bar = lv_obj_create(parent);
    lv_obj_set_size(p_app_clock_main->top_bar,
                    lv_disp_get_hor_res(lv_disp_get_default()), 30);
    lv_obj_align(p_app_clock_main->top_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(p_app_clock_main->top_bar, lv_color_black(),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(p_app_clock_main->top_bar, LV_OPA_80, LV_PART_MAIN);
    lv_obj_add_flag(p_app_clock_main->top_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(p_app_clock_main->top_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(p_app_clock_main->top_bar, dial_editing_exit_event_cb,
                        LV_EVENT_CLICKED, NULL);

    // 在按鈕上放置 icon_x
    lv_obj_t *icon = lv_img_create(p_app_clock_main->top_bar);
    lv_img_set_src(icon, ICON_X);
    lv_img_set_zoom(icon, 69); // 100%
    lv_obj_align(icon, LV_ALIGN_BOTTOM_MID, 0, 15);
}

static void clock_main_bottom_bar_builder(lv_obj_t *parent)
{
    p_app_clock_main->bottom_bar = lv_obj_create(parent);
    lv_obj_set_size(p_app_clock_main->bottom_bar,
                    lv_disp_get_hor_res(lv_disp_get_default()), 30);
    lv_obj_align(p_app_clock_main->bottom_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(p_app_clock_main->bottom_bar, lv_color_black(),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(p_app_clock_main->bottom_bar, LV_OPA_80,
                            LV_PART_MAIN);
    lv_obj_add_flag(p_app_clock_main->bottom_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(p_app_clock_main->bottom_bar, LV_OBJ_FLAG_SCROLLABLE);
}

static void app_clock_main_init(lv_obj_t *scr)
{
    rt_uint16_t i;
    lv_coord_t scr_hor_res, scr_ver_res;
    rt_uint16_t last_active_clock_bak; /*when tileview created,
                                          LV_EVENT_VALUE_CHANGED (idx is 0)
                                          will be send */
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    dial_widget_app_id = SkaiWatchSys.clock_widget_num;
    scr_hor_res = lv_disp_get_hor_res(lv_disp_get_default());
    scr_ver_res = lv_disp_get_ver_res(lv_disp_get_default());
    last_active_clock_bak = last_active_clock;

    p_app_clock_main->tileview = lv_tileview_create(scr);
    lv_obj_set_style_bg_opa(p_app_clock_main->tileview, LV_OPA_TRANSP,
                            LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(p_app_clock_main->tileview,
                              LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(p_app_clock_main->tileview, LV_OBJ_FLAG_SCROLL_ELASTIC);
    lv_obj_clear_flag(p_app_clock_main->tileview, LV_OBJ_FLAG_SCROLLABLE);
    p_app_clock_main->p_tileview_valid_pos = (lv_point_t *)lv_mem_alloc(
        sizeof(lv_point_t) * p_app_clock_main->app_clock_list_len);
    for (i = 0; i < p_app_clock_main->app_clock_list_len; i++)
    {
        p_app_clock_main->p_tileview_valid_pos[i].x = i;
        p_app_clock_main->p_tileview_valid_pos[i].y = 0;
    }
    rt_list_t *pos;
    i = 0;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        app_clock_desc_t *clk_desc = rt_list_entry(pos, app_clock_desc_t, node);
        lv_obj_t *page;

        page =
            lv_tileview_add_tile(p_app_clock_main->tileview, i, 0, LV_DIR_HOR);
        lv_obj_set_size(page, scr_hor_res / 2, scr_ver_res / 2);
        lv_obj_set_pos(page, scr_hor_res * i, 0);
        lv_obj_add_event_cb(page, tileview_event_cb, LV_EVENT_ALL, NULL);
        clk_desc->parent = page;
        i++;
    }

    lv_obj_add_event_cb(p_app_clock_main->tileview, tileview_event_cb,
                        LV_EVENT_ALL, NULL);

    // 創建上方黑條
    clock_main_top_bar_builder(scr);

    // 創建下方黑條
    clock_main_bottom_bar_builder(scr);

    if (last_active_clock_bak < p_app_clock_main->app_clock_list_len)
        lv_obj_set_tile_id(p_app_clock_main->tileview, last_active_clock_bak, 0,
                           false);
    else
        last_active_clock_bak = 0;

#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_watchface = handle_watchface_changed_cb;
#endif
    gui_state_update_timer_start();
    if (is_user_want_to_open_display_to_app_list())
    {
        open_widget_list_timer_start();
    }
}

#ifdef RT_USING_XIP_MODULE
static void app_clock_load_one_dyn_wf(const char *module_name, const char *path)
{
    struct rt_dlmodule *mod;
    uint32_t old_len;
    app_clock_desc_t *clk_desc;

    old_len = p_app_clock_main->app_clock_list_len;

    mod = dlrun(module_name, path);

    if (mod && (old_len != p_app_clock_main->app_clock_list_len))
    {
        clk_desc =
            rt_list_tail_entry(&p_app_clock_main->list, app_clock_desc_t, node);
        clk_desc->mod = mod;
    }
}

static void app_clock_load_dyn_wf(void)
{
    DIR *dir;
    struct dirent *dir_entry;
    struct stat *ent_stat;
    char *full_path;
    const char *path = "watchface";
    uint32_t name_len;

    dir = opendir(path);
    if (!dir)
    {
        return;
    }
    ent_stat = rt_malloc(sizeof(*ent_stat));
    RT_ASSERT(stat);

    do
    {
        dir_entry = readdir(dir);
        if (!dir_entry)
        {
            break;
        }

        memset(ent_stat, 0, sizeof(*ent_stat));

        /* build full path for each file */
        full_path = dfs_normalize_path(path, dir_entry->d_name);
        if (full_path == NULL)
        {
            break;
        }

        if (stat(full_path, ent_stat) == 0)
        {
            if (!S_ISDIR(ent_stat->st_mode))
            {
                name_len = strlen(dir_entry->d_name);
                if (('m' == dir_entry->d_name[name_len - 1]) &&
                    ('.' ==
                     dir_entry->d_name[name_len - 2])) /* ending with .m */
                {
                    /* remove suffix .m */
                    dir_entry->d_name[name_len - 2] = 0;
                    app_clock_load_one_dyn_wf(dir_entry->d_name, path);
                }
            }
        }
        rt_free(full_path);
    } while (true);

    rt_free(ent_stat);
    closedir(dir);
}

#endif /* RT_USING_XIP_MODULE */

extern void app_clock_media_register(void);

#ifdef PKG_USING_FFMPEG
extern void app_clock_video_audio_register(void);
#endif /* PKG_USING_FFMPEG */

void app_clock_reset_time(void)
{
    struct tm *time_info;

#ifdef WIN32
    __time32_t raw_time;
    _time32(&raw_time);
    time_info = _localtime32(&raw_time);
#else
    time_t raw_time;
    time(&raw_time);
    time_info = localtime(&raw_time);
#endif
    latched_clock = clock();
    total_milliseconds = (((time_info->tm_hour * 60) + time_info->tm_min) * 60 +
                          time_info->tm_sec) *
                         1000;

    LOG_D("service_reset_time:  %d:%d:%d - %d:%d:%d - %d", time_info->tm_year,
          time_info->tm_mon, time_info->tm_mday, time_info->tm_hour,
          time_info->tm_min, time_info->tm_sec, time_info->tm_wday);
}

lv_obj_t *lv_home_listview_layout_create(lv_obj_t *parent)
{
    p_app_clock_main =
        (app_clock_main_t *)lv_mem_alloc(sizeof(app_clock_main_t));
    memset(p_app_clock_main, 0, sizeof(app_clock_main_t));
    rt_list_init(&p_app_clock_main->list);

#if 0
    extern void app_clock_digital_elegant_register(void);
    app_clock_digital_elegant_register();
#endif
#ifdef PKG_USING_FFMPEG
    app_clock_video_audio_register();
#endif /* PKG_USING_FFMPEG */
    gui_script_watch_face_register();

#ifdef RT_USING_XIP_MODULE
    app_clock_load_dyn_wf();
#endif /* RT_USING_XIP_MODULE */
    app_clock_reset_time();
    last_active_clock = SkaiWatchSys.clock_status;
    app_clock_main_init(parent);

    return parent;
}

static bool clock_initiated = false;
static void on_start(lv_obj_t *scr)
{
    if (clock_initiated)
        return;
    app_clock_main_init(scr);
}

static bool pause_clock = true;

rt_int32_t clock_on_resume(void)
{
    notify_provider.bluetooth_connection();
    if (pause_clock == false)
        return -RT_EOK;
    pause_clock = false;

    // 確保 tileview 位置正確對齊到當前錶盤
    if (p_app_clock_main && lv_obj_is_valid(p_app_clock_main->tileview))
    {
        lv_obj_set_tile_id(p_app_clock_main->tileview, last_active_clock, 0,
                           false);
    }

    app_clock_main_select(last_active_clock);
    LOG_D("clock_on_resume");
    clock_initiated = true;
    return 1;
}

rt_int32_t clock_on_pause(void)
{
    if (pause_clock == true)
    {
        return -RT_EOK;
    }
    rt_list_t *pos;
    uint16_t i = 0;
    pause_clock = true;

    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        app_clock_desc_t *clk_desc;
        clk_desc = rt_list_entry(pos, app_clock_desc_t, node);
        app_clock_change_state(clk_desc, STATE_DEINIT);
    }
    LOG_D("clock_on_pause");
    return RT_EOK;
}

void clock_on_stop(void)
{
    rt_list_t *pos;
    uint16_t i = 0;
    pause_clock = true;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        app_clock_desc_t *clk_desc;
        clk_desc = rt_list_entry(pos, app_clock_desc_t, node);
        app_clock_change_state(clk_desc, STATE_DEINIT);
    }

    gui_state_update_timer_stop();
    LOG_D("clock_on_stop");
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_watchface = NULL;
    lvgl_msg_handler.handle_time_text = NULL;
    lvgl_msg_handler.handle_bluetooth_connection = NULL;
    lvgl_msg_handler.refresh_battery_level = NULL;
    lvgl_msg_handler.handle_charge_status = NULL;
#endif

    if (p_app_clock_main)
    {
        rt_list_t *pos;

        rt_list_for_each(pos, (&p_app_clock_main->list))
        {
            app_clock_desc_t *clk_desc;
            clk_desc = rt_list_entry(pos, app_clock_desc_t, node);
            app_clock_change_state(clk_desc, STATE_DEINIT);

#ifdef RT_USING_XIP_MODULE
            if (clk_desc->mod)
            {
                dlclose(clk_desc->mod);
            }
#endif /* RT_USING_XIP_MODULE */
            lv_mem_free(clk_desc);
        }
        lv_obj_del(p_app_clock_main->app_list_time_h);
        lv_obj_del(p_app_clock_main->app_list_time_m);
        lv_obj_del(p_app_clock_main->app_list_time_symbol);
        lv_obj_del(p_app_clock_main->app_list_weather_icon);
        lv_obj_del(p_app_clock_main->app_list_bluetooth_disconnection);
        lv_obj_del(p_app_clock_main->app_list_battery);
        lv_obj_del(p_app_clock_main->app_list_battery_bg);
        lv_obj_del(p_app_clock_main->app_list_battery_label);
        lv_obj_del(p_app_clock_main->app_list_time_bg);

        lv_mem_free(p_app_clock_main->p_tileview_valid_pos);
        if (colon_blink_timer != NULL)
        {
            lv_timer_del(colon_blink_timer);
            colon_blink_timer = NULL;
        }
#ifdef ENABLE_NOTIFICATION_CENTER
        app_clock_main_status_bar_deinit();
#endif

        lv_mem_free(p_app_clock_main);
        p_app_clock_main = NULL;

        clock_initiated = false;
    }
}

int32_t app_clock_register(const char *id, const app_clock_ops_t *operations)
{
    app_clock_desc_t *new_clock;
    uint16_t id_len;

    if ((!id) || (!operations))
        return RT_EINVAL;

    new_clock = (app_clock_desc_t *)lv_mem_alloc(sizeof(app_clock_desc_t));

    id_len = strlen(id);
    if (id_len > APP_CLOCK_ID_MAX_LEN)
        id_len = APP_CLOCK_ID_MAX_LEN;

    memcpy(new_clock->id, id, id_len);
    new_clock->id[id_len] = '\0';

    new_clock->ops = operations;
    new_clock->state = STATE_DEINIT;
    new_clock->mod = NULL;

    rt_list_init(&new_clock->node);
    rt_list_insert_before(&p_app_clock_main->list, &new_clock->node);

    p_app_clock_main->app_clock_list_len++;

    return 0;
}

lv_obj_t *build_home_view(lv_obj_t *parent)
{
    lv_obj_t *obj = lv_home_listview_layout_create(parent);
    app_clock_main_select(last_active_clock);

#ifdef ENABLE_NOTIFICATION_CENTER
    app_clock_main_status_bar_init(lv_scr_act());
    app_clock_ai_status_bar_init(lv_scr_act());
#endif

    top_digital_time_builder(lv_layer_top());

    battery_status_indicator_builder(lv_layer_top());
    refresh_bluetooth_disconnection(SkaiWatchSys.gap_conn_state ==
                                    GAP_CONN_STATE_CONNECTED);
    app_list_main_time_update();
    refersh_battery(SkaiWatchSys.battery_level_value);
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_time_text = app_list_main_time_update;
    lvgl_msg_handler.handle_bluetooth_connection =
        refresh_bluetooth_disconnection;
    lvgl_msg_handler.refresh_battery_level = refersh_battery;
#endif
    return obj;
}
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/