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
LV_IMG_DECLARE(media_gaus_bg);

/* Forward declarations for wear status indicator */
static void refresh_wear_status_indicator(void);
static void wear_status_indicator_builder(lv_obj_t *parent);

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
    lv_obj_t *clock_container;      //!< container for current clock
    rt_uint32_t app_clock_list_len; //!< clock list length
    rt_list_t list;                 //!< head of all clocks list

    rt_timer_t soft_timer; //!<  template vaiable for simulator
    lv_obj_t *instruction_list_time_h;
    lv_obj_t *instruction_list_time_m;
    lv_obj_t *instruction_list_time_symbol;
    lv_obj_t *instruction_list_time_bg;
    lv_obj_t *instruction_list_weather_icon;
    lv_obj_t *instruction_list_bluetooth_disconnection;
    lv_obj_t *instruction_list_battery_bg;
    lv_obj_t *instruction_list_battery;
    lv_obj_t *instruction_list_battery_label;
} app_clock_main_t;

#ifndef BSP_USING_LVGL_INPUT_AGENT
static
#endif
    app_clock_main_t *p_app_clock_main = NULL;

void set_instruction_list_time_opa(uint8_t opa)
{
    if (lv_obj_is_valid(p_app_clock_main->instruction_list_time_h) == 0 ||
        lv_obj_is_valid(p_app_clock_main->instruction_list_time_m) == 0 ||
        lv_obj_is_valid(p_app_clock_main->instruction_list_time_symbol) == 0 ||
        lv_obj_is_valid(p_app_clock_main->instruction_list_weather_icon) == 0)
    {
        LOG_W("instruction_list_time_h is not valid");
        return;
    }
    lv_obj_set_style_text_opa(p_app_clock_main->instruction_list_time_h, opa,
                              0);
    lv_obj_set_style_text_opa(p_app_clock_main->instruction_list_time_m, opa,
                              0);
    lv_obj_set_style_text_opa(p_app_clock_main->instruction_list_time_symbol,
                              opa, 0);
    lv_obj_set_style_img_opa(p_app_clock_main->instruction_list_weather_icon,
                             opa, 0);
}

static lv_obj_t *charge_icon_obj = NULL;

void set_instruction_list_battery_opa(uint8_t opa)
{
    if (lv_obj_is_valid(p_app_clock_main->instruction_list_battery) == 0 ||
        lv_obj_is_valid(p_app_clock_main->instruction_list_battery_label) ==
            0 ||
        lv_obj_is_valid(charge_icon_obj) == 0)
    {
        LOG_W("instruction_list_battery is not valid");
        return;
    }
    lv_obj_set_style_img_opa(p_app_clock_main->instruction_list_battery, opa,
                             0);
    lv_obj_set_style_text_opa(p_app_clock_main->instruction_list_battery_label,
                              opa, 0);
    lv_obj_set_style_img_opa(charge_icon_obj, opa, 0);
    lv_obj_set_style_img_opa(
        p_app_clock_main->instruction_list_bluetooth_disconnection, opa, 0);
}

lv_obj_t *get_instruction_list_time_bg(void)
{
    return p_app_clock_main->instruction_list_time_bg;
}

void set_instruction_list_time_bg_opa(uint8_t opa)
{
    if (lv_obj_is_valid(get_instruction_list_time_bg()))
    {
        lv_obj_set_style_bg_opa(get_instruction_list_time_bg(), opa, 0);
    }
}

lv_obj_t *get_instruction_list_bluetooth_disconnection(void)
{
    return p_app_clock_main->instruction_list_bluetooth_disconnection;
}

lv_obj_t *get_instruction_list_battery(void)
{
    return p_app_clock_main->instruction_list_battery;
}

lv_obj_t *get_instruction_list_battery_label(void)
{
    return p_app_clock_main->instruction_list_battery_label;
}

lv_obj_t *get_instruction_list_battery_bg(void)
{
    return p_app_clock_main->instruction_list_battery_bg;
}

void set_instruction_list_battery_bg_opa(uint8_t opa)
{
    if (lv_obj_is_valid(get_instruction_list_battery_bg()))
    {
        lv_obj_set_style_bg_opa(get_instruction_list_battery_bg(), opa, 0);
    }
}

void show_battery(bool show)
{
    if (!lv_obj_is_valid(get_instruction_list_battery_bg()))
    {
        return;
    }
    if (show)
    {
        lv_obj_clear_flag(get_instruction_list_battery_bg(),
                          LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(get_instruction_list_battery_bg(), LV_OBJ_FLAG_HIDDEN);
    }
}

void show_instruction_list_time(bool show)
{
    if (!lv_obj_is_valid(get_instruction_list_time_bg()))
    {
        return;
    }
    if (show)
    {
        lv_obj_clear_flag(get_instruction_list_time_bg(), LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(get_instruction_list_time_bg(), LV_OBJ_FLAG_HIDDEN);
    }
}

void refresh_charge_icon(void)
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
#ifdef APP_ID_MEDIA
static uint8_t dial_widget_app_id = app_id_media;
#else
static uint8_t dial_widget_app_id = app_id_calendar;
#endif
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
        LOG_D("dial_widget_event LV_EVENT_RELEASED:%d,%d",
              dial_widget_press_valid, edit_mode);
        if (dial_widget_press_valid)
        {
            dial_widget_press_valid = false;
            if (edit_mode)
                show_dial_widget_select_list(lv_scr_act());
            else
            {
                switch (dial_widget_app_id)
                {
#ifdef APP_ID_CALENDAR
                case app_id_calendar:
                    gui_app_run(APP_ID_CALENDAR);
                    break;
#endif
#ifdef APP_ID_MEDIA
                case app_id_media:
                    gui_app_run(APP_ID_MEDIA);
                    break;
#endif
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

lv_obj_t *app_clock_main_get_dial_widget(void)
{
    return dial_widget;
}

// Level bar for flat detection
#define LEVEL_BAR_WIDTH 200
#define LEVEL_BAR_HEIGHT 20
#define LEVEL_DOT_SIZE 14
#define LEVEL_CENTER_BOX_W 40
#define LEVEL_CENTER_BOX_H 20
#define LEVEL_BAR_Y_OFFSET 195

static lv_obj_t *level_bar_container = NULL;
static lv_obj_t *level_bar_line = NULL;
static lv_obj_t *level_bar_dot = NULL;
static lv_obj_t *level_bar_arc_left = NULL;
static lv_obj_t *level_bar_arc_right = NULL;

static void level_bar_builder(lv_obj_t *parent)
{
    // Container
    level_bar_container = lv_obj_create(parent);
    lv_obj_set_size(level_bar_container, LEVEL_BAR_WIDTH + LEVEL_DOT_SIZE,
                    LEVEL_CENTER_BOX_H + 4);
    lv_obj_align(level_bar_container, LV_ALIGN_CENTER, 0, LEVEL_BAR_Y_OFFSET);
    lv_obj_set_style_bg_opa(level_bar_container, LV_OPA_0, 0);
    lv_obj_set_style_border_opa(level_bar_container, LV_OPA_0, 0);
    lv_obj_set_style_pad_all(level_bar_container, 0, 0);
    lv_obj_clear_flag(level_bar_container, LV_OBJ_FLAG_SCROLLABLE);

    // Horizontal bar line
    level_bar_line = lv_obj_create(level_bar_container);
    lv_obj_set_size(level_bar_line, LEVEL_BAR_WIDTH, LEVEL_BAR_HEIGHT);
    lv_obj_align(level_bar_line, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(level_bar_line, lv_color_hex(0x666666), 0);
    lv_obj_set_style_bg_opa(level_bar_line, LV_OPA_30, 0);
    lv_obj_set_style_radius(level_bar_line, LEVEL_BAR_HEIGHT / 2, 0);

    // Left arc (flat zone left boundary)
    level_bar_arc_left = lv_arc_create(level_bar_container);
    lv_obj_set_size(level_bar_arc_left, 24, 24);
    lv_arc_set_rotation(level_bar_arc_left, 120);
    lv_arc_set_bg_angles(level_bar_arc_left, 0, 120);
    lv_arc_set_angles(level_bar_arc_left, 0, 0);
    lv_obj_set_style_arc_width(level_bar_arc_left, 2, LV_PART_MAIN);
    lv_obj_set_style_arc_color(level_bar_arc_left, lv_color_hex(0xFFFFFF),
                               LV_PART_MAIN);
    lv_obj_set_style_arc_opa(level_bar_arc_left, LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_arc_width(level_bar_arc_left, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(level_bar_arc_left, LV_OPA_0, LV_PART_KNOB);
    lv_obj_clear_flag(level_bar_arc_left, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(level_bar_arc_left, LV_ALIGN_CENTER, -(LEVEL_CENTER_BOX_W / 2),
                 0);

    // Right arc (flat zone right boundary)
    level_bar_arc_right = lv_arc_create(level_bar_container);
    lv_obj_set_size(level_bar_arc_right, 24, 24);
    lv_arc_set_rotation(level_bar_arc_right, 300);
    lv_arc_set_bg_angles(level_bar_arc_right, 0, 120);
    lv_arc_set_angles(level_bar_arc_right, 0, 0);
    lv_obj_set_style_arc_width(level_bar_arc_right, 2, LV_PART_MAIN);
    lv_obj_set_style_arc_color(level_bar_arc_right, lv_color_hex(0xFFFFFF),
                               LV_PART_MAIN);
    lv_obj_set_style_arc_opa(level_bar_arc_right, LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_arc_width(level_bar_arc_right, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(level_bar_arc_right, LV_OPA_0, LV_PART_KNOB);
    lv_obj_clear_flag(level_bar_arc_right, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(level_bar_arc_right, LV_ALIGN_CENTER, (LEVEL_CENTER_BOX_W / 2),
                 0);

    // Moving dot
    level_bar_dot = lv_obj_create(level_bar_container);
    lv_obj_set_size(level_bar_dot, LEVEL_DOT_SIZE, LEVEL_DOT_SIZE);
    lv_obj_align(level_bar_dot, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(level_bar_dot, lv_color_hex(0x9D9D9D), 0);
    lv_obj_set_style_bg_opa(level_bar_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(level_bar_dot, LEVEL_DOT_SIZE / 2, 0);
    lv_obj_set_style_border_opa(level_bar_dot, LV_OPA_0, 0);
}

static void level_bar_deinit(void)
{
    level_bar_container = NULL;
    level_bar_line = NULL;
    level_bar_dot = NULL;
    level_bar_arc_left = NULL;
    level_bar_arc_right = NULL;
}

// Flat zone threshold: dot pixel range vs arc position
// Arc centers at ±(LEVEL_CENTER_BOX_W/2) = ±20px
// Dot x_offset = value * ((LEVEL_BAR_WIDTH-20)/2) / 100
// Flat when |x_offset| <= LEVEL_CENTER_BOX_W/2
// => |value| <= LEVEL_CENTER_BOX_W/2 * 100 / ((LEVEL_BAR_WIDTH-20)/2)
#define LEVEL_FLAT_THRESHOLD                                                   \
    ((LEVEL_CENTER_BOX_W / 2) * 100 / ((LEVEL_BAR_WIDTH - 20) / 2)) - 5

static bool level_is_flat = false;

bool level_bar_is_flat(void)
{
    return level_is_flat;
}

// value: -100 ~ +100, 0 = flat (center)
void level_bar_update(int16_t value)
{
    if (value < -100)
        value = -100;
    if (value > 100)
        value = 100;

    bool was_flat = level_is_flat;
    level_is_flat =
        (value >= -LEVEL_FLAT_THRESHOLD && value <= LEVEL_FLAT_THRESHOLD);
}
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
                    // swich_dial_widget_builder(dial_widget_app_id,
                    //                           clk_desc->parent);
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

#ifdef APP_ID_MEDIA
    lv_obj_t *btn_media =
        lv_list_add_btn(dial_widget_select_list, NULL, "Media");
    lv_obj_add_event_cb(btn_media, dial_widget_select_event_cb,
                        LV_EVENT_CLICKED, (void *)app_id_media);
#endif

    lv_obj_t *btn_weather =
        lv_list_add_btn(dial_widget_select_list, NULL, "Weather");
    lv_obj_add_event_cb(btn_weather, dial_widget_select_event_cb,
                        LV_EVENT_CLICKED, (void *)app_id_weather);
}

extern void set_dial_media_widget_opa(uint8_t opa);
void set_dial_widget_opa(uint8_t opa)
{
    if (lv_obj_is_valid(dial_widget))
    {
        if (lv_obj_is_valid(dial_widget_img_bg))
        {
            lv_obj_set_style_img_opa(dial_widget_img_bg, opa, 0);
        }
    }
    set_dial_media_widget_opa(opa);
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
    lv_obj_set_style_radius(dial_widget, 25, 0);
    lv_obj_align(dial_widget, LV_ALIGN_CENTER, 0, 99);
    lv_obj_set_size(dial_widget, 340, 160);
    lv_obj_set_style_bg_color(dial_widget, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(dial_widget, LV_OPA_0, 0);
    lv_obj_set_style_border_width(dial_widget, 2, 0);
    lv_obj_set_style_border_color(dial_widget, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(dial_widget, LV_OPA_0, 0);
    lv_obj_set_style_clip_corner(dial_widget, true, 0);     // 啟用裁切
    lv_obj_clear_flag(dial_widget, LV_OBJ_FLAG_SCROLLABLE); // 禁用滾動

    dial_widget_img_bg = lv_img_create(dial_widget);
    // lv_obj_set_style_radius(dial_widget_img_bg, 50, LV_PART_MAIN);
    // lv_obj_set_style_bg_opa(dial_widget_img_bg, LV_OPA_100, 0);
    lv_img_set_src(dial_widget_img_bg, &media_gaus_bg);
    // lv_img_set_zoom(dial_widget_img_bg, 256 ); // 100%
    lv_obj_align(dial_widget_img_bg, LV_ALIGN_CENTER, 0, 0);
    if (app_id == app_id_calendar)
    {
        request_calendar_on_mobile(false);
        lv_dial_calendar_widget_builder(dial_widget);
        lv_obj_add_event_cb(dial_widget, dial_widget_event, LV_EVENT_ALL, NULL);
    }
#ifdef APP_ID_MEDIA
    else if (app_id == app_id_media)
    {
        lv_dial_media_widget_builder(dial_widget);
    }
#endif
    else if (app_id == app_id_weather)
    {
        request_weather_within_six_hours(false);
        lv_dial_weather_widget_builder(dial_widget);
    }

    // Build level bar below the widget
    // level_bar_builder(parent);
}

static void swich_dial_widget_deinit(uint8_t app_id)
{
    if (app_id == app_id_calendar)
    {
        dial_calendar_widget_deinit();
    }
#ifdef APP_ID_MEDIA
    else if (app_id == app_id_media)
    {
        dial_media_widget_deinit();
    }
#endif
    else if (app_id == app_id_weather)
    {
        dial_weather_widget_deinit();
    }
    level_bar_deinit();
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
        // if (lv_obj_is_valid(dial_widget_img_bg))
        // {
        //     // lv_img_set_src(dial_widget_img_bg, &gaus_clock1_bg);
        //     lv_img_set_zoom(dial_widget_img_bg, 256 * 2); // 100%
        //     lv_obj_align(dial_widget_img_bg, LV_ALIGN_CENTER, 0, -115);
        // }
        set_clock_main_status_img(&gaus_clock1_bg);
    }
    else if (strcmp(clk_id, "JW_wf2") == 0)
    {
        // if (lv_obj_is_valid(dial_widget_img_bg))
        // {
        //     // lv_img_set_src(dial_widget_img_bg, GAUS_DEFAULT_PICTURE);
        //     lv_img_set_zoom(dial_widget_img_bg, 256 * 2); // 100%
        //     lv_obj_align(dial_widget_img_bg, LV_ALIGN_CENTER, 0, -115);
        // }
        set_clock_main_status_img(GAUS_DEFAULT_PICTURE);
    }
    else if (strcmp(clk_id, "JW_wf3") == 0)
    {
        // if (lv_obj_is_valid(dial_widget_img_bg))
        // {
        //     // lv_img_set_src(dial_widget_img_bg, GAUS_DEFAULT_PICTURE);
        //     lv_img_set_zoom(dial_widget_img_bg, 256 * 2); // 100%
        //     lv_obj_align(dial_widget_img_bg, LV_ALIGN_CENTER, 0, -115);
        // }
        set_clock_main_status_img(GAUS_DEFAULT_PICTURE);
    }
    else if (strcmp(clk_id, "JW_wf4") == 0)
    {
        // if (lv_obj_is_valid(dial_widget_img_bg))
        // {
        //     // lv_img_set_src(dial_widget_img_bg, &gaus_clock4_bg);
        //     lv_img_set_zoom(dial_widget_img_bg, 256 * 2); // 100%
        //     lv_obj_align(dial_widget_img_bg, LV_ALIGN_CENTER, 0, -115);
        // }
        set_clock_main_status_img(&gaus_clock4_bg);
    }
    else if (strcmp(clk_id, "JW_wf5") == 0)
    {
        // if (lv_obj_is_valid(dial_widget_img_bg))
        // {
        //     // lv_img_set_src(dial_widget_img_bg, GAUS_CLOCK5_BG);
        //     lv_img_set_zoom(dial_widget_img_bg, 256 * 2); // 100%
        //     lv_obj_align(dial_widget_img_bg, LV_ALIGN_CENTER, 0, -115);
        // }
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
                // if (strcmp(p_clock->id, "JW_wf1") == 0)
                // {
                //     swich_dial_widget_deinit(dial_widget_app_id);
                // }
                // else if (strcmp(p_clock->id, "JW_wf3") == 0)
                //     swich_dial_widget_deinit(dial_widget_app_id);
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
                // if (strcmp(p_clock->id, "JW_wf1") == 0)
                //     swich_dial_widget_deinit(dial_widget_app_id);
                // else if (strcmp(p_clock->id, "JW_wf3") == 0)
                //     swich_dial_widget_deinit(dial_widget_app_id);
            }
        }
        else if (p_clock->ops->init)
        {
            p_clock->ops->init(p_clock->parent);
            // if (strcmp(p_clock->id, "JW_wf1") == 0)
            // {
            //     swich_dial_widget_builder(dial_widget_app_id,
            //     p_clock->parent);
            // }
            // else if (strcmp(p_clock->id, "JW_wf3") == 0)
            //     swich_dial_widget_builder(dial_widget_app_id,
            //     p_clock->parent);
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
                // if (strcmp(p_clock->id, "JW_wf1") == 0)
                // {
                //     swich_dial_widget_builder(dial_widget_app_id,
                //                               p_clock->parent);
                // }
                // else if (strcmp(p_clock->id, "JW_wf3") == 0)
                // {
                //     swich_dial_widget_builder(dial_widget_app_id,
                //                               p_clock->parent);
                // }
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
    rt_uint16_t i;
    rt_list_t *pos;
    app_clock_desc_t *clk_desc;

    // Validate clock index
    if (clock_idx >= p_app_clock_main->app_clock_list_len)
        clock_idx = p_app_clock_main->app_clock_list_len - 1;

    // Deinit all other clocks
    i = 0;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        clk_desc = rt_list_entry(pos, app_clock_desc_t, node);

        if (i != clock_idx)
        {
            app_clock_change_state(clk_desc, STATE_DEINIT);
        }
        i++;
    }

    // Activate selected clock
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

/**
 * @brief Switch to a specific clock by index
 * @param clock_idx Index of the clock to switch to (0-based)
 * @note This is a public API for external watch face selection pages
 */
void app_clock_switch_to(uint16_t clock_idx)
{
    app_clock_main_select(clock_idx);
}

extern const lv_img_dsc_t *weather_icon_get(char *weather);
static void refersh_time(T_UTC_TIME *current_time)
{
    char time_str[3];
    LOG_D("refersh_time:%d:%d:%d", current_time->hour, current_time->minutes,
          current_time->seconds);
    if (lv_obj_is_valid(p_app_clock_main->instruction_list_time_h) &&
        lv_obj_is_valid(p_app_clock_main->instruction_list_time_m))
    {
        rt_snprintf(time_str, 3, "%02d", current_time->hour);
        lv_label_set_text(p_app_clock_main->instruction_list_time_h, time_str);
        rt_snprintf(time_str, 3, "%02d", current_time->minutes);
        lv_label_set_text(p_app_clock_main->instruction_list_time_m, time_str);
    }
    else
    {
        LOG_W("instruction_list_time is not valid");
    }
}

void refersh_weather_icon(void)
{
    weather_t *get_weather_data = get_weather(WEATHER_TODAT_ITEM_AMOUNT - 1);
    if (lv_obj_is_valid(p_app_clock_main->instruction_list_weather_icon) == 0)
    {
        LOG_W("instruction_list_weather_icon is not valid");
        return;
    }
    lv_img_set_src(p_app_clock_main->instruction_list_weather_icon,
                   weather_icon_get(get_weather_data->description));
}

void refersh_battery(uint8_t battery_level)
{
    if (p_app_clock_main)
    {
        char battery_str[5];
        rt_snprintf(battery_str, 5, "%d%%", battery_level);
        if (lv_obj_is_valid(p_app_clock_main->instruction_list_battery_label))
        {
            lv_label_set_text(p_app_clock_main->instruction_list_battery_label,
                              battery_str);
            if (SkaiWatchSys.charger_status == InCharging)
            {
                lv_obj_set_style_text_color(
                    p_app_clock_main->instruction_list_battery_label,
                    lv_color_hex(0x00CC00), 0);
            }
            else
            {
                lv_obj_set_style_text_color(
                    p_app_clock_main->instruction_list_battery_label,
                    lv_color_hex(0xFFFFFF), 0);
            }
        }
        else
        {
            LOG_W("instruction_list_battery_label is not valid");
        }
        refresh_charge_icon();
        set_battery_image(p_app_clock_main->instruction_list_battery,
                          battery_level);
    }
    else
    {
        LOG_W("instruction_list_battery is NULL");
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

extern void ble_dev_mgr_start_main_phone_check_timer(uint32_t interval_ms);
static void refresh_bluetooth_disconnection(bool connected)
{
    is_bluetooth_connected = connected;
    if (lv_obj_is_valid(
            p_app_clock_main->instruction_list_bluetooth_disconnection))
    {
        if (connected)
        {
            lv_obj_add_flag(
                p_app_clock_main->instruction_list_bluetooth_disconnection,
                LV_OBJ_FLAG_HIDDEN);
            // 當重新連接時，銷毀斷線提示視窗
            destroy_connection_tips();
        }
        else
        {
            ble_dev_mgr_start_main_phone_check_timer(5000);
            lv_obj_clear_flag(
                p_app_clock_main->instruction_list_bluetooth_disconnection,
                LV_OBJ_FLAG_HIDDEN);
        }
    }
    else
    {
        LOG_W("instruction_list_bluetooth_disconnection is not valid");
    }
}

static T_UTC_TIME aligned_time;
static lv_timer_t *gui_state_update_timer = NULL;

void instruction_list_main_time_update(void)
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
        instruction_list_main_time_update();
    }
    refresh_wear_status_indicator();
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
    animate_to_instruction_list();
    set_user_want_to_open_display_to_instruction_list(false);
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
    if (p_app_clock_main)
        app_clock_switch_to(index);
}
static void on_tap(void)
{
    LOG_D("Tap in Clock");
}

static void update_time_symbol(void)
{
    if (lv_obj_is_valid(p_app_clock_main->instruction_list_time_symbol))
    {
        if (lv_obj_has_flag(p_app_clock_main->instruction_list_time_symbol,
                            LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(p_app_clock_main->instruction_list_time_symbol,
                              LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(p_app_clock_main->instruction_list_time_symbol,
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
    p_app_clock_main->instruction_list_time_bg = lv_obj_create(parent);
    lv_obj_set_style_bg_color(p_app_clock_main->instruction_list_time_bg,
                              lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(p_app_clock_main->instruction_list_time_bg,
                            LV_OPA_0, 0);
    lv_obj_set_size(p_app_clock_main->instruction_list_time_bg, 466, 50);
    lv_obj_align(p_app_clock_main->instruction_list_time_bg, LV_ALIGN_TOP_MID,
                 -10, 0);
    lv_obj_clear_flag(p_app_clock_main->instruction_list_time_bg,
                      LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *instruction_list_time_h =
        lv_label_create(p_app_clock_main->instruction_list_time_bg);
    lv_obj_set_style_text_align(instruction_list_time_h, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(instruction_list_time_h,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_align(instruction_list_time_h, LV_ALIGN_CENTER, -20, 0);
    lv_obj_t *instruction_list_time_m =
        lv_label_create(p_app_clock_main->instruction_list_time_bg);
    lv_obj_set_style_text_align(instruction_list_time_m, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(instruction_list_time_m,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_align(instruction_list_time_m, LV_ALIGN_CENTER, 21, 0);
    lv_obj_t *instruction_list_time_symbol =
        lv_label_create(p_app_clock_main->instruction_list_time_bg);
    lv_obj_set_style_text_align(instruction_list_time_symbol,
                                LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(instruction_list_time_symbol,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_align(instruction_list_time_symbol, LV_ALIGN_CENTER, 0, -1);
    lv_obj_t *instruction_list_weather_icon =
        lv_img_create(p_app_clock_main->instruction_list_time_bg);
    weather_t *get_weather_data = get_weather(WEATHER_TODAT_ITEM_AMOUNT - 1);
    lv_img_set_src(instruction_list_weather_icon,
                   weather_icon_get(get_weather_data->description));
    lv_img_set_zoom(instruction_list_weather_icon, 128); // zoom 80%
    lv_obj_align(instruction_list_weather_icon, LV_ALIGN_CENTER, 58, 0);
    p_app_clock_main->instruction_list_weather_icon =
        instruction_list_weather_icon;
    uint8_t minutes = SkaiWatchSys.Global_Time.minutes;
    uint8_t hour = SkaiWatchSys.Global_Time.hour;
    char time_str[3];
    rt_snprintf(time_str, 3, "%02d", hour); // hh
    lv_label_set_text(instruction_list_time_h, time_str);
    rt_snprintf(time_str, 3, "%02d", minutes); // mm
    lv_label_set_text(instruction_list_time_m, time_str);
    lv_label_set_text(instruction_list_time_symbol, ":");

    lv_obj_set_style_text_opa(instruction_list_time_h, LV_OPA_TRANSP, 0);
    p_app_clock_main->instruction_list_time_h = instruction_list_time_h;
    lv_obj_set_style_text_opa(instruction_list_time_m, LV_OPA_TRANSP, 0);
    p_app_clock_main->instruction_list_time_m = instruction_list_time_m;
    lv_obj_set_style_text_opa(instruction_list_time_symbol, LV_OPA_TRANSP, 0);
    p_app_clock_main->instruction_list_time_symbol =
        instruction_list_time_symbol;
    lv_obj_set_style_img_opa(instruction_list_weather_icon, LV_OPA_TRANSP, 0);
    p_app_clock_main->instruction_list_weather_icon =
        instruction_list_weather_icon;

    if (colon_blink_timer == NULL)
    {
        colon_blink_timer = lv_timer_create(colon_blink_timer_cb, 1000, NULL);
    }
}

/* Wear status indicator */
static lv_obj_t *wear_status_indicator = NULL;

static void refresh_wear_status_indicator(void)
{
    if (!lv_obj_is_valid(wear_status_indicator))
        return;

    if (SkaiWatchSys.flag_field.is_wearing)
    {
        lv_obj_add_flag(wear_status_indicator, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(wear_status_indicator, lv_color_hex(0x00FF00),
                                  0);
    }
    else
    {
        lv_obj_clear_flag(wear_status_indicator, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(wear_status_indicator, lv_color_hex(0xFF0000),
                                  0);
    }
}

static void wear_status_indicator_builder(lv_obj_t *parent)
{
    wear_status_indicator = lv_obj_create(parent);
    lv_obj_set_size(wear_status_indicator, 12, 12);
    lv_obj_align(wear_status_indicator, LV_ALIGN_TOP_MID, 60, 20);
    lv_obj_set_style_radius(wear_status_indicator, 6, 0);
    lv_obj_set_style_border_opa(wear_status_indicator, LV_OPA_0, 0);
    lv_obj_set_style_bg_opa(wear_status_indicator, LV_OPA_COVER, 0);
    lv_obj_clear_flag(wear_status_indicator,
                      LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    refresh_wear_status_indicator();
}

static void battery_status_indicator_builder(lv_obj_t *parent)
{
    lv_obj_t *instruction_list_battery_bg = lv_obj_create(parent);
    lv_obj_set_style_bg_color(instruction_list_battery_bg, lv_color_black(),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(instruction_list_battery_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(instruction_list_battery_bg, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(instruction_list_battery_bg, 0, LV_PART_MAIN);
    lv_obj_set_size(instruction_list_battery_bg, 466, 80);
    lv_obj_align(instruction_list_battery_bg, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_clear_flag(instruction_list_battery_bg,
                      LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    p_app_clock_main->instruction_list_battery_bg = instruction_list_battery_bg;

    lv_obj_t *instruction_list_battery = lv_img_create(instruction_list_battery_bg);
    lv_obj_align(instruction_list_battery, LV_ALIGN_TOP_MID, 30, 18);
    lv_obj_set_style_img_opa(instruction_list_battery, LV_OPA_TRANSP, 0);
    p_app_clock_main->instruction_list_battery = instruction_list_battery;

    lv_obj_t *instruction_list_battery_label = lv_label_create(instruction_list_battery_bg);
    lv_obj_set_style_text_align(instruction_list_battery_label,
                                LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(instruction_list_battery_label, LV_ALIGN_TOP_MID, -30, 12);
    lv_obj_set_style_text_opa(instruction_list_battery_label, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(instruction_list_battery_label,
                                lv_color_white(), 0);
    p_app_clock_main->instruction_list_battery_label =
        instruction_list_battery_label;

    charge_icon_obj = lv_img_create(instruction_list_battery);
    lv_img_set_src(charge_icon_obj, CHARGE_ICON);
    lv_obj_align(charge_icon_obj, LV_ALIGN_CENTER, -2, -1);
    lv_obj_set_style_img_opa(charge_icon_obj, LV_OPA_TRANSP, 0);

    lv_obj_t *instruction_list_bluetooth_disconnection = lv_img_create(parent);
    lv_img_set_src(instruction_list_bluetooth_disconnection,
                   ICON_BLUETOOTH_DISCONNECTION);
    lv_obj_align(instruction_list_bluetooth_disconnection, LV_ALIGN_TOP_MID, 0,
                 45);
    lv_obj_set_style_img_opa(instruction_list_bluetooth_disconnection,
                             LV_OPA_TRANSP, 0);
    p_app_clock_main->instruction_list_bluetooth_disconnection =
        instruction_list_bluetooth_disconnection;
}

static void app_clock_main_init(lv_obj_t *scr)
{
    lv_coord_t scr_hor_res, scr_ver_res;
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    dial_widget_app_id = SkaiWatchSys.clock_widget_num;
    scr_hor_res = lv_disp_get_hor_res(lv_disp_get_default());
    scr_ver_res = lv_disp_get_ver_res(lv_disp_get_default());

    // Create single container for clock
    p_app_clock_main->clock_container = lv_obj_create(scr);
    lv_obj_set_size(p_app_clock_main->clock_container, scr_hor_res,
                    scr_ver_res);
    lv_obj_set_style_bg_opa(p_app_clock_main->clock_container, LV_OPA_TRANSP,
                            LV_PART_MAIN);
    lv_obj_set_style_border_width(p_app_clock_main->clock_container, 0,
                                  LV_PART_MAIN);
    lv_obj_set_style_pad_all(p_app_clock_main->clock_container, 0,
                             LV_PART_MAIN);
    lv_obj_clear_flag(p_app_clock_main->clock_container,
                      LV_OBJ_FLAG_SCROLLABLE);

    // Set all clocks to use the same parent
    rt_list_t *pos;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        app_clock_desc_t *clk_desc = rt_list_entry(pos, app_clock_desc_t, node);
        clk_desc->parent = p_app_clock_main->clock_container;
    }

    // Initialize media header
    extern void lv_dial_header_builder(lv_obj_t * parent);
    lv_dial_header_builder(scr);

#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_watchface = handle_watchface_changed_cb;
#endif
    gui_state_update_timer_start();
    if (is_user_want_to_open_display_to_instruction_list())
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

#if 1
    // extern void app_clock_digital_elegant_register(void);
    // app_clock_digital_elegant_register();
    extern void app_clock_earth_digital_register(void);
    app_clock_earth_digital_register();
#endif
#ifdef PKG_USING_FFMPEG
    app_clock_video_audio_register();
#endif /* PKG_USING_FFMPEG */
    // gui_script_watch_face_register();

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

extern void dial_media_header_init(void);
rt_int32_t clock_on_resume(void)
{
    notify_provider.bluetooth_connection();
    if (pause_clock == false)
        return -RT_EOK;
    pause_clock = false;

    dial_media_header_init();
    app_clock_main_select(last_active_clock);
    LOG_D("clock_on_resume");
    clock_initiated = true;
    return 1;
}

extern void dial_media_header_deinit(void);
rt_int32_t clock_on_pause(void)
{
    if (pause_clock == true)
    {
        return -RT_EOK;
    }
    rt_list_t *pos;
    uint16_t i = 0;
    pause_clock = true;
    dial_media_header_deinit();
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
    dial_media_header_deinit();
    gui_state_update_timer_stop();
    LOG_D("clock_on_stop");
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_watchface = NULL;
    lvgl_msg_handler.handle_time_text = NULL;
    lvgl_msg_handler.handle_bluetooth_connection = NULL;
    lvgl_msg_handler.refresh_battery_level = NULL;
#endif

    if (p_app_clock_main)
    {
        rt_list_t *pos, *n;

        /* Use _safe variant: clk_desc (containing the node) is freed inside
         * the loop body, so we must capture pos->next before freeing.
         * Plain rt_list_for_each would read pos->next from freed memory. */
        rt_list_for_each_safe(pos, n, (&p_app_clock_main->list))
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
            rt_list_remove(&clk_desc->node);
            lv_mem_free(clk_desc);
        }
#define SAFE_OBJ_DEL(obj) do { if ((obj) && lv_obj_is_valid(obj)) lv_obj_del(obj); } while (0)
        SAFE_OBJ_DEL(p_app_clock_main->instruction_list_time_h);
        SAFE_OBJ_DEL(p_app_clock_main->instruction_list_time_m);
        SAFE_OBJ_DEL(p_app_clock_main->instruction_list_time_symbol);
        SAFE_OBJ_DEL(p_app_clock_main->instruction_list_weather_icon);
        SAFE_OBJ_DEL(p_app_clock_main->instruction_list_bluetooth_disconnection);
        SAFE_OBJ_DEL(p_app_clock_main->instruction_list_battery);
        SAFE_OBJ_DEL(p_app_clock_main->instruction_list_battery_bg);
        SAFE_OBJ_DEL(p_app_clock_main->instruction_list_battery_label);
        SAFE_OBJ_DEL(p_app_clock_main->instruction_list_time_bg);
#undef SAFE_OBJ_DEL

        wear_status_indicator = NULL;

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
    // app_clock_ai_status_bar_init(lv_scr_act());
#endif
    app_clock_device_change_bar_init(lv_layer_top());
    top_digital_time_builder(lv_layer_top());

    battery_status_indicator_builder(lv_layer_top());
    wear_status_indicator_builder(lv_layer_top());
    refresh_bluetooth_disconnection(SkaiWatchSys.gap_conn_state ==
                                    GAP_CONN_STATE_CONNECTED);
    instruction_list_main_time_update();
    refersh_battery(SkaiWatchSys.battery_level_value);
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_time_text = instruction_list_main_time_update;
    lvgl_msg_handler.handle_bluetooth_connection =
        refresh_bluetooth_disconnection;
#endif
    return obj;
}
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/