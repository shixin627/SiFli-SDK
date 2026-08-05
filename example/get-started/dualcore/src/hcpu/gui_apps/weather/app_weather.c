/**
 ******************************************************************************
 * @file   app_weather.c
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
#include <stdio.h>
#include <math.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "bloc_control.h"
#include "bloc_motion_tracking.h"
#include "bloc_weather.h"
#include "app_mainmenu.h"
#include "app_clock_main.h"
#include "common_widget.h"
#include "watch_global_data.h"
#include "ui_handler.h"
#include "ui_img_helper.h"
#include "communicate_protocol.h"
#include "arc_scroll.h"

#define DBG_TAG "app.weather"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_WEATHER
LV_IMG_DECLARE(weather_clear);
LV_IMG_DECLARE(weather_cloudy);
LV_IMG_DECLARE(weather_heavy_rain);
LV_IMG_DECLARE(weather_sun);
LV_IMG_DECLARE(weather_rain);
LV_IMG_DECLARE(weather_thunder);

typedef struct
{
    lv_ex_data_t *screen_data;
} app_weather_data_ctx_t;

typedef struct
{
    lv_obj_t *bg;
    lv_obj_t *main_window;
    lv_obj_t *tileview; // Add tileview object
    lv_obj_t *tile1;    // Current weather tile
    lv_obj_t *tile2;    // Daily summary tile

    // Main weather display elements
    lv_obj_t *weather_location_label;
    lv_obj_t *weather_icon;
    lv_obj_t *cur_tem_label;
    lv_obj_t *weather_label;
    lv_obj_t *no_dat_label;

    // Forecast weather elements
    lv_obj_t *future_weather_icon[3];
    lv_obj_t *future_temp_val[3];
    lv_obj_t *future_date[3];

    // Daily summary elements
    lv_obj_t *daily_date[5];
    lv_obj_t *daily_weather_icon[5];
    lv_obj_t *daily_rain_label[5];
    lv_obj_t *daily_max_temp[5];
    lv_obj_t *daily_min_temp[5];

    arc_scroll_handle_t *arc_handle; // 共用右側弧形觸控滾動 instance
} app_weather_t;

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */

static app_weather_data_ctx_t app_weather_data_ctx = {0};
static app_weather_t *p_app_weather = NULL;
static char buffer[32];
static weather_data_t weather_data[3] = {0};

LV_IMG_DECLARE(weather_widget_bg);
/*
 * FORWARD DECLARATIONS
 *****************************************************************************************
 */
const lv_img_dsc_t *weather_icon_get(char *weather);
lv_obj_t *create_weather_obj(lv_obj_t *parent, weather_data_t *children,
                             weather_t *weather);
void create_forecast_widget(lv_obj_t *parent, lv_obj_t *base, int index,
                            int x_offset, weather_t *data);
void weather_layout_update(void);
static void refresh_ui(lv_obj_t *_, void *para);
void create_daily_forecast_widget(lv_obj_t *parent, lv_obj_t *base, int index,
                                  weather_t *data);
lv_obj_t *lv_daily_weather_page_create(lv_obj_t *parent);
void update_daily_weather_layout(void);

/*
 * EVENT HANDLERS
 *****************************************************************************************
 */
static void weather_event_cb(lv_event_t *e)
{
    if (!e)
        return;

    switch (e->code)
    {
    case LV_EVENT_DELETE:
    case LV_EVENT_READY:
    case LV_EVENT_CANCEL:
        // Handle specific events if needed
        break;
    default:
        break;
    }
}

/* arc-scroll snap：放手後從目前 scroll_y 算最近的 tile，直接 lv_obj_set_tile
 * 讓 tileview 自己更新 tile_act 跟動畫滾過去。
 * 不能單純 return tile 給 arc_scroll 的 released_cb 走 lv_obj_scroll_to_view —
 * 那只動 scroll 不更新 tile_act，後續 LV_DIR_TOP/BOTTOM 滑動或 motion 控制會
 * 拿到舊的 active tile，行為錯亂；arc_scroll 拖動時又是用 _lv_obj_scroll_by_raw
 * 直接刷 scroll，tileview 內部 snap-on-scroll-end 不一定有觸發過 */
static lv_obj_t *weather_arc_snap_cb(void *ctx)
{
    (void)ctx;
    if (p_app_weather == NULL) return NULL;
    if (p_app_weather->tileview == NULL ||
        !lv_obj_is_valid(p_app_weather->tileview))
        return NULL;
    lv_coord_t scroll_y = lv_obj_get_scroll_y(p_app_weather->tileview);
    int idx = (scroll_y + LV_VER_RES / 2) / LV_VER_RES;
    if (idx < 0) idx = 0;
    if (idx > 1) idx = 1;
    lv_obj_t *target = (idx == 0) ? p_app_weather->tile1 : p_app_weather->tile2;
    if (target != NULL && lv_obj_is_valid(target))
    {
        lv_obj_set_tile(p_app_weather->tileview, target, LV_ANIM_ON);
    }
    /* 已經自己 snap 了，回 NULL 讓 arc_scroll 的 released_cb 不再 lv_obj_scroll_to_view */
    return NULL;
}

/*
 * HELPER FUNCTIONS
 *****************************************************************************************
 */
const lv_img_dsc_t *weather_icon_get(char *weather)
{
    if (!weather)
        return &weather_sun; // Default icon for NULL

    if (strcmp(weather, "Clear") == 0)
    {
        return &weather_clear;
    }
    else if (strcmp(weather, "Clouds") == 0)
    {
        return &weather_cloudy;
    }
    else if (strcmp(weather, "Rain") == 0)
    {
        return &weather_rain;
    }
    else if (strcmp(weather, "Thunderstorm") == 0)
    {
        return &weather_thunder;
    }
    else if (strcmp(weather, "Snow") == 0)
    {
        return &weather_thunder;
    }
    else if (strcmp(weather, "Sun") == 0)
    {
        return &weather_sun;
    }

    return &weather_sun; // Default icon
}

lv_obj_t *create_weather_obj(lv_obj_t *parent, weather_data_t *children,
                             weather_t *weather)
{
    if (!parent || !children || !weather)
    {
        LOG_E("Invalid parameters in create_weather_obj");
        return NULL;
    }

    lv_obj_t *weather_widget = lv_obj_create(parent);
    lv_obj_set_size(weather_widget, 120, 210);
    lv_obj_set_style_bg_opa(weather_widget, LV_OPA_0, 0);

    // Create time label
    lv_obj_t *time = lv_label_create(weather_widget);
    children->time = time;
    lv_obj_set_style_text_font(time, LV_EXT_FONT_GET(get_system_font_size(-1)),
                               0);
    lv_obj_set_style_text_color(time, lv_color_white(), 0);
    ui_time_format_hhmm(buffer, sizeof(buffer), weather->time.hour,
                        weather->time.minutes);
    lv_label_set_text(time, buffer);
    lv_obj_align(time, LV_ALIGN_TOP_MID, 0, -10);
    lv_obj_set_style_text_opa(time, LV_OPA_70, 0);

    // Create temperature label
    lv_obj_t *temperature = lv_label_create(weather_widget);
    children->temperature = temperature;
    lv_obj_set_style_text_font(temperature,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(temperature, lv_color_white(), 0);
    snprintf(buffer, sizeof(buffer), "%0.f", round(weather->temperature));
    lv_label_set_text(temperature, buffer);
    lv_obj_align_to(temperature, time, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);

    lv_obj_t *degree = lv_label_create(weather_widget);
    lv_obj_set_style_text_font(degree, LV_EXT_FONT_GET(get_system_font_size(0)),
                               0);
    lv_obj_set_style_text_color(degree, lv_color_white(), 0);
    lv_label_set_text(degree, "°");
    lv_obj_align_to(degree, temperature, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    children->degree = degree;

    // Create weather icon
    lv_obj_t *img = lv_img_create(weather_widget);
    children->img = img;
    lv_img_set_src(img, weather_icon_get(weather->description));
    lv_obj_align_to(img, temperature, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    // Create rainfall label
    lv_obj_t *chance_of_rainfall = lv_label_create(weather_widget);
    children->chance_of_rain = chance_of_rainfall;
    lv_obj_set_style_text_font(chance_of_rainfall,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(chance_of_rainfall, lv_color_white(), 0);
    snprintf(buffer, sizeof(buffer), "%d %%",
             weather->precipitationProbability);
    lv_label_set_text(chance_of_rainfall, buffer);
    lv_obj_align_to(chance_of_rainfall, img, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
    if (weather->precipitationProbability > 0)
    {
        lv_obj_clear_flag(children->chance_of_rain, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(children->chance_of_rain, LV_OBJ_FLAG_HIDDEN);
    }

    return weather_widget;
}

static void update_weather_obj(weather_data_t *children, weather_t *weather)
{
    if (!children || !weather)
    {
        LOG_E("Invalid parameters in update_weather_obj");
        return;
    }

    ui_time_format_hhmm(buffer, sizeof(buffer), weather->time.hour,
                        weather->time.minutes);

    if (lv_obj_is_valid(children->time))
    {
        lv_label_set_text(children->time, buffer);
    }

    if (lv_obj_is_valid(children->temperature))
    {
        snprintf(buffer, sizeof(buffer), "%0.f", round(weather->temperature));
        lv_label_set_text(children->temperature, buffer);
        lv_obj_align_to(children->temperature, children->time,
                        LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
        lv_obj_align_to(children->degree, children->temperature,
                        LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    }

    if (lv_obj_is_valid(children->img))
    {
        lv_img_set_src(children->img, weather_icon_get(weather->description));
    }

    if (lv_obj_is_valid(children->chance_of_rain))
    {
        snprintf(buffer, sizeof(buffer), "%d %%",
                 weather->precipitationProbability);
        lv_label_set_text(children->chance_of_rain, buffer);
        if (weather->precipitationProbability > 0)
        {
            lv_obj_clear_flag(children->chance_of_rain, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(children->chance_of_rain, LV_OBJ_FLAG_HIDDEN);
        }
    }

    weather->notified = true;
}

void create_forecast_widget(lv_obj_t *parent, lv_obj_t *base, int index,
                            int x_offset, weather_t *data)
{
    if (!parent || !base || !data || !p_app_weather || index < 0 || index > 2)
    {
        LOG_E("Invalid parameters in create_forecast_widget");
        return;
    }

    lv_color_t color = lv_color_make(0xA0, 0xA0, 0xA0);

    // Create date label
    p_app_weather->future_date[index] = lv_label_create(parent);
    ui_time_format_hhmm(buffer, sizeof(buffer), data->time.hour,
                        data->time.minutes);
    lv_label_set_text(p_app_weather->future_date[index], buffer);
    lv_obj_set_style_text_font(p_app_weather->future_date[index],
                               LV_EXT_FONT_GET(get_system_font_size(-2)), 0);
    lv_obj_set_style_text_color(p_app_weather->future_date[index], color, 0);
    lv_obj_align_to(p_app_weather->future_date[index], base,
                    LV_ALIGN_OUT_BOTTOM_MID, x_offset, 10);

    // Create weather icon
    p_app_weather->future_weather_icon[index] = lv_img_create(parent);
    lv_img_set_src(p_app_weather->future_weather_icon[index],
                   weather_icon_get(data->description));
    lv_obj_align_to(p_app_weather->future_weather_icon[index],
                    p_app_weather->future_date[index], LV_ALIGN_OUT_BOTTOM_MID,
                    0, 10);

    // Create temperature label
    p_app_weather->future_temp_val[index] = lv_label_create(parent);
    snprintf(buffer, sizeof(buffer), "%0.f°C", round(data->temperature));
    lv_label_set_text(p_app_weather->future_temp_val[index], buffer);
    lv_obj_set_style_text_font(p_app_weather->future_temp_val[index],
                               LV_EXT_FONT_GET(get_system_font_size(-2)), 0);
    lv_obj_set_style_text_color(p_app_weather->future_temp_val[index], color,
                                0);
    lv_obj_align_to(p_app_weather->future_temp_val[index],
                    p_app_weather->future_weather_icon[index],
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
}

static const char *weekday_full_i18n(int wday)
{
    switch (wday) {
        case 0: return LV_EXT_STR_GET_BY_KEY(Sun, "Sun");
        case 1: return LV_EXT_STR_GET_BY_KEY(Mon, "Mon");
        case 2: return LV_EXT_STR_GET_BY_KEY(Tue, "Tue");
        case 3: return LV_EXT_STR_GET_BY_KEY(Wed, "Wed");
        case 4: return LV_EXT_STR_GET_BY_KEY(Thu, "Thu");
        case 5: return LV_EXT_STR_GET_BY_KEY(Fri, "Fri");
        case 6: return LV_EXT_STR_GET_BY_KEY(Sat, "Sat");
        default: return "";
    }
}

void create_daily_forecast_widget(lv_obj_t *parent, lv_obj_t *base, int index,
                                  weather_t *data)
{
    if (!parent || !base || !data || !p_app_weather || index < 0 || index > 4)
    {
        LOG_E("Invalid parameters in create_daily_forecast_widget");
        return;
    }

    lv_color_t text_color = lv_color_make(0xCC, 0xCC, 0xCC);
    int y_offset = index * 80; // Reduced vertical spacing

    // Create weekday label
    p_app_weather->daily_date[index] = lv_label_create(parent);
    struct tm timeinfo = {0};
    timeinfo.tm_year = data->time.year - 1900;
    timeinfo.tm_mon = data->time.month - 1;
    timeinfo.tm_mday = data->time.day;
    mktime(&timeinfo);
    lv_label_set_text(p_app_weather->daily_date[index],
                      index == 0 ? LV_EXT_STR_GET_BY_KEY(today, "Today") : weekday_full_i18n(timeinfo.tm_wday));
    lv_obj_set_style_text_font(p_app_weather->daily_date[index],
                               LV_EXT_FONT_GET(get_system_font_size(-2)), 0);
    lv_obj_set_style_text_color(p_app_weather->daily_date[index], text_color,
                                0);
    lv_obj_align(p_app_weather->daily_date[index], LV_ALIGN_TOP_LEFT, 80,
                 60 + y_offset);

    // Create weather icon
    p_app_weather->daily_weather_icon[index] = lv_img_create(parent);
    lv_img_set_src(p_app_weather->daily_weather_icon[index],
                   weather_icon_get(data->description));
    lv_obj_align_to(p_app_weather->daily_weather_icon[index],
                    p_app_weather->daily_date[index], LV_ALIGN_OUT_RIGHT_MID,
                    50, 0);

    // Create precipitation probability for rainy weather
    p_app_weather->daily_rain_label[index] = lv_label_create(parent);
    lv_obj_set_style_text_color(p_app_weather->daily_rain_label[index],
                                lv_color_make(0x40, 0x80, 0xFF), 0);
    lv_obj_align_to(p_app_weather->daily_rain_label[index],
                    p_app_weather->daily_weather_icon[index],
                    LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    lv_obj_add_flag(p_app_weather->daily_rain_label[index], LV_OBJ_FLAG_HIDDEN);

    // Create temperature labels (min and max)
    p_app_weather->daily_min_temp[index] = lv_label_create(parent);
    p_app_weather->daily_max_temp[index] = lv_label_create(parent);

    snprintf(buffer, sizeof(buffer), "%d°", (int)round(data->min_temperature));
    lv_label_set_text(p_app_weather->daily_min_temp[index], buffer);
    lv_obj_set_style_text_font(p_app_weather->daily_min_temp[index],
                               LV_EXT_FONT_GET(get_system_font_size(-2)), 0);
    lv_obj_set_style_text_color(p_app_weather->daily_min_temp[index],
                                text_color, 0);

    snprintf(buffer, sizeof(buffer), "%d°", (int)round(data->max_temperature));
    lv_label_set_text(p_app_weather->daily_max_temp[index], buffer);
    lv_obj_set_style_text_font(p_app_weather->daily_max_temp[index],
                               LV_EXT_FONT_GET(get_system_font_size(-2)), 0);
    lv_obj_set_style_text_color(p_app_weather->daily_max_temp[index],
                                text_color, 0);

    // Align temperatures to the right
    lv_obj_align(p_app_weather->daily_max_temp[index], LV_ALIGN_TOP_RIGHT, -80,
                 60 + y_offset);
    lv_obj_align_to(p_app_weather->daily_min_temp[index],
                    p_app_weather->daily_max_temp[index], LV_ALIGN_OUT_LEFT_MID,
                    -30, 0);

    // Add separator line (except for last item)
    if (index < WEATHER_DAILY_ITEM_AMOUNT - 1)
    {
        static lv_point_precise_t line_pts[] = {{0, 0}, {300, 0}};
        lv_obj_t *separator = lv_line_create(parent);
        lv_obj_set_style_line_width(separator, 1, 0);
        lv_obj_set_style_line_color(separator, lv_color_make(0x40, 0x40, 0x40),
                                    0);
        lv_line_set_points(separator, line_pts, 2);
        lv_obj_align_to(separator, p_app_weather->daily_date[index],
                        LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    }
}

lv_obj_t *lv_daily_weather_page_create(lv_obj_t *parent)
{
    if (!parent || !p_app_weather)
    {
        LOG_E("Invalid parameters in lv_daily_weather_page_create");
        return NULL;
    }

    // Create container with dark background
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(cont, lv_color_make(0x20, 0x20, 0x20), 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_0, 0);

    // Create daily forecast widgets
    weather_t *daily_data;
    for (int i = 0; i < 5; i++)
    {
        daily_data = &current_weather_week_list()[i];
        if (daily_data)
        {
            create_daily_forecast_widget(cont, cont, i, daily_data);
        }
    }

    return cont;
}

void update_daily_weather_layout(void)
{
    if (!p_app_weather)
    {
        LOG_E("Weather app not initialized");
        return;
    }

    weather_t *daily_data;
    for (int i = 0; i < WEATHER_DAILY_ITEM_AMOUNT; i++)
    {
        daily_data =
            &current_weather_week_list()[WEATHER_DAILY_ITEM_AMOUNT - 1 - i];
        if (!daily_data)
        {
            continue;
        }

        if (lv_obj_is_valid(p_app_weather->daily_date[i]))
        {
            snprintf(buffer, sizeof(buffer), "%02d/%02d",
                     daily_data->time.month, daily_data->time.day);
            lv_label_set_text(p_app_weather->daily_date[i], buffer);
        }

        if (lv_obj_is_valid(p_app_weather->daily_weather_icon[i]))
        {
            lv_img_set_src(p_app_weather->daily_weather_icon[i],
                           weather_icon_get(daily_data->description));
        }

        if (lv_obj_is_valid(p_app_weather->daily_rain_label[i]))
        {
            if (strstr(daily_data->description, "Rain") != NULL ||
                strstr(daily_data->description, "Drizzle") != NULL ||
                strstr(daily_data->description, "Thunderstorm") != NULL)
            {
                if (daily_data->precipitationProbability > 0)
                {
                    snprintf(buffer, sizeof(buffer), "%d%%",
                             (int)daily_data->precipitationProbability);
                    lv_label_set_text(p_app_weather->daily_rain_label[i],
                                      buffer);
                    lv_obj_clear_flag(p_app_weather->daily_rain_label[i],
                                      LV_OBJ_FLAG_HIDDEN);
                }
            }
        }

        if (lv_obj_is_valid(p_app_weather->daily_max_temp[i]))
        {
            snprintf(buffer, sizeof(buffer), "%0.f°",
                     round(daily_data->max_temperature));
            lv_label_set_text(p_app_weather->daily_max_temp[i], buffer);
        }

        if (lv_obj_is_valid(p_app_weather->daily_min_temp[i]))
        {
            snprintf(buffer, sizeof(buffer), "%0.f°",
                     round(daily_data->min_temperature));
            lv_label_set_text(p_app_weather->daily_min_temp[i], buffer);
        }
    }
}

/*
 * WIDGET BUILDERS
 *****************************************************************************************
 */
lv_obj_t *lv_weather_object_builder(lv_obj_t *parent, void *data)
{
    if (!parent || !data)
    {
        LOG_E("Invalid parameters in lv_weather_object_builder");
        return NULL;
    }

    weather_t *weather = (weather_t *)data;

    // Create main weather widget container
    // lv_obj_t *weather_widget = lv_obj_create(parent);
    lv_obj_t *weather_widget = parent;
    lv_obj_set_size(weather_widget, 360, 240);
    lv_obj_set_style_bg_color(weather_widget, lv_color_hex(0x404040),
                              0); // Light gray background
    lv_obj_set_style_bg_opa(weather_widget, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(weather_widget, 10, 0); // Rounded corners
    lv_obj_set_style_border_width(weather_widget, 0, 0);
    lv_obj_set_style_pad_all(weather_widget, 15, 0);

    // Create location name (top left)
    lv_obj_t *location_label = lv_label_create(weather_widget);
    lv_obj_set_style_text_font(location_label,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(location_label, lv_color_white(), 0);
    const char *location_text = get_current_location();
    if (location_text[0] == '\0')
    {
        location_text = "No Location";
    }
    lv_label_set_text(location_label, location_text);
    lv_label_set_long_mode(location_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(location_label, 240);
    lv_obj_align(location_label, LV_ALIGN_TOP_LEFT, 0, 0);

    // Create temperature (bottom left)
    lv_obj_t *temp_label = lv_label_create(weather_widget);
    lv_obj_set_style_text_font(temp_label,
                               LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(temp_label, lv_color_white(), 0);
    char temp_buffer[16];
    snprintf(temp_buffer, sizeof(temp_buffer), "%.0f°",
             round(weather->temperature));
    lv_label_set_text(temp_label, temp_buffer);
    lv_obj_align(temp_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);

    // Create weather icon (top right)
    lv_obj_t *icon = lv_img_create(weather_widget);
    lv_img_set_src(icon, weather_icon_get(weather->description));
    lv_obj_align(icon, LV_ALIGN_TOP_RIGHT, 0, 0);

    // Create weather condition (middle right)
    lv_obj_t *weather_condition = lv_label_create(weather_widget);
    lv_obj_set_style_text_font(weather_condition,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(weather_condition, lv_color_white(), 0);
    lv_label_set_text(weather_condition, weather->description);
    lv_obj_align(weather_condition, LV_ALIGN_RIGHT_MID, 0, 0);

    // Create high/low temperatures (bottom right)
    lv_obj_t *high_low_label = lv_label_create(weather_widget);
    lv_obj_set_style_text_font(high_low_label,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(high_low_label, lv_color_white(), 0);
    char high_low_buffer[32];
    snprintf(high_low_buffer, sizeof(high_low_buffer), "H:%.0f° L:%.0f°",
             round(weather->max_temperature), round(weather->min_temperature));
    lv_label_set_text(high_low_label, high_low_buffer);
    lv_obj_align(high_low_label, LV_ALIGN_BOTTOM_RIGHT, 0, 0);

    return weather_widget;
}

lv_obj_t *lv_weather_widget_builder(lv_obj_t *parent)
{
    if (!parent)
    {
        LOG_E("Invalid parent in lv_weather_widget_builder");
        return NULL;
    }
    lv_obj_t *widget = common_widget_container(parent);
    lv_obj_set_style_clip_corner(widget, true, 0);
    lv_obj_set_style_bg_opa(widget, LV_OPA_0, 0);
    lv_obj_set_size(widget, 400, 250);
    lv_obj_clear_flag(widget, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *bg = lv_img_create(widget);
    lv_img_set_src(bg, &weather_widget_bg);

    if (!widget)
        return NULL;
    for (int i = 0; i < 3; i++)
    {
        weather_t *get_weather_data =
            get_weather(WEATHER_TODAT_ITEM_AMOUNT - 1 - i);
        if (!get_weather_data)
        {
            LOG_E("Failed to get weather data for index %d",
                  WEATHER_TODAT_ITEM_AMOUNT - 1 - i);
            continue;
        }
        lv_obj_t *weather_widget =
            create_weather_obj(widget, &weather_data[i], get_weather_data);

        if (i == 0 && lv_obj_is_valid(weather_data[i].time))
        {
            lv_label_set_text(weather_data[i].time, "Now");
        }

        if (weather_widget)
        {
            lv_obj_align(weather_widget, LV_ALIGN_LEFT_MID, 13 + i * 140, 10);
        }
        // Add separator line between weather items
        if (i < 2)
        {
            lv_obj_t *line = lv_obj_create(widget);
            lv_obj_set_size(line, 1, 190);
            lv_obj_align_to(line, weather_widget, LV_ALIGN_OUT_RIGHT_TOP, 10,
                            0);
            lv_obj_set_style_bg_color(line, LV_COLOR_WHITE, 0);
            lv_obj_set_style_bg_opa(line, LV_OPA_30, 0);
        }
    }
    return widget;
}

lv_obj_t *lv_card_layout_weather_create(lv_obj_t *parent_tv_obj)
{
    if (!parent_tv_obj || !p_app_weather)
    {
        LOG_E("Invalid parameters in lv_card_layout_weather_create");
        return NULL;
    }

    // Create tileview
    p_app_weather->tileview = lv_tileview_create(parent_tv_obj);
    lv_obj_set_size(p_app_weather->tileview, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_opa(p_app_weather->tileview, LV_OPA_0, 0);

    // Create first tile (current weather)
    p_app_weather->tile1 =
        lv_tileview_add_tile(p_app_weather->tileview, 0, 0, LV_DIR_BOTTOM);

    // Create second tile (daily summary)
    p_app_weather->tile2 =
        lv_tileview_add_tile(p_app_weather->tileview, 0, 1, LV_DIR_TOP);

    // Create current weather page
    lv_obj_t *current_weather_page = lv_obj_create(p_app_weather->tile1);
    lv_obj_set_size(current_weather_page, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_clear_flag(current_weather_page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(current_weather_page, LV_OPA_0, 0);

    /* Tail of the today array (WEATHER_TODAT_ITEM_AMOUNT-1) is the "current
       hour" slot. Earlier indices (WEATHER_TODAT_ITEM_AMOUNT-2 ... 0) are
       the next-N-hour forecasts (newer entries shift toward 0 in
       update_weather). Using the symbolic constant keeps every weather
       widget in sync if the array size is retuned. */
    weather_t *get_weather_dayone_data =
        get_weather(WEATHER_TODAT_ITEM_AMOUNT - 1);
    if (!get_weather_dayone_data)
    {
        LOG_E("Failed to get main weather data");
        return p_app_weather->tileview;
    }

    // Weather Title (Location Name)
    p_app_weather->weather_location_label =
        lv_label_create(current_weather_page);
    const char *location_text = get_current_location();
    if (location_text[0] == '\0')
    {
        location_text = "No Location";
    }
    lv_label_set_text(p_app_weather->weather_location_label, location_text);
    lv_color_t color = lv_color_white();
    lv_obj_set_style_text_font(p_app_weather->weather_location_label,
                               LV_EXT_FONT_GET(FONT_SUBTITLE), LV_PART_MAIN);
    lv_obj_set_style_text_color(p_app_weather->weather_location_label, color,
                                0);
    lv_obj_align(p_app_weather->weather_location_label, LV_ALIGN_TOP_MID, 0,
                 30);

    // Weather Icon
    p_app_weather->weather_icon = lv_img_create(current_weather_page);
    lv_img_set_src(p_app_weather->weather_icon,
                   weather_icon_get(get_weather_dayone_data->description));
    lv_obj_align_to(p_app_weather->weather_icon,
                    p_app_weather->weather_location_label,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    // Temperature Value
    p_app_weather->cur_tem_label = lv_label_create(current_weather_page);
    snprintf(buffer, sizeof(buffer), "%0.f°C",
             round(get_weather_dayone_data->temperature));
    lv_label_set_text(p_app_weather->cur_tem_label, buffer);
    color = lv_color_make(0xFF, 0xFF, 0xFF);
    lv_obj_set_style_text_font(p_app_weather->cur_tem_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(p_app_weather->cur_tem_label, color, 0);
    lv_obj_align_to(p_app_weather->cur_tem_label, p_app_weather->weather_icon,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    // Weather Label
    p_app_weather->weather_label = lv_label_create(current_weather_page);
    lv_label_set_text(p_app_weather->weather_label,
                      get_weather_dayone_data->description);
    color = lv_color_make(0x99, 0x99, 0x99);
    lv_obj_set_style_text_font(p_app_weather->weather_label,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(p_app_weather->weather_label, color, 0);
    lv_obj_align_to(p_app_weather->weather_label, p_app_weather->cur_tem_label,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    // No data label (initially hidden)
    p_app_weather->no_dat_label = lv_label_create(current_weather_page);
    lv_label_set_text(p_app_weather->no_dat_label,
                      "No weather data \n please update on the phone");
    lv_obj_set_style_text_align(p_app_weather->no_dat_label,
                                LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(p_app_weather->no_dat_label,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(p_app_weather->no_dat_label, color, 0);
    lv_obj_align(p_app_weather->no_dat_label, LV_ALIGN_CENTER, 0, -60);
    lv_obj_add_flag(p_app_weather->no_dat_label, LV_OBJ_FLAG_HIDDEN);

    // Only show the no-data label if data is outdated
    if (get_weather_dayone_data->time.day != SkaiWatchSys.Global_Time.day &&
        get_weather_dayone_data->time.month != SkaiWatchSys.Global_Time.month)
    {
        lv_obj_clear_flag(p_app_weather->no_dat_label, LV_OBJ_FLAG_HIDDEN);
    }

    // Dividing Line
    static lv_point_precise_t line_pts[] = {{0, 0}, {400, 0}};
    lv_obj_t *p_line = lv_line_create(current_weather_page);
    lv_obj_set_style_line_width(p_line, 2, 0);
    lv_obj_set_style_line_color(p_line, lv_color_make(0x4B, 0x4B, 0x4B), 0);
    lv_obj_set_style_line_rounded(p_line, true, 0);
    lv_line_set_points(p_line, line_pts, 2);
    lv_obj_align_to(p_line, p_app_weather->weather_label,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
    lv_obj_align(p_line, LV_ALIGN_CENTER, 0, 10);

    /* Forecast row: leftmost = hour+1, middle = hour+2, rightmost = hour+3.
       Centre widget already shows hour 0 (= WEATHER_TODAT_ITEM_AMOUNT - 1). */
    weather_t *forecast_data;
    for (int i = 0; i < 3; i++)
    {
        int offset = (i - 1) * 130; // -130, 0, 130
        forecast_data = get_weather(WEATHER_TODAT_ITEM_AMOUNT - 2 - i);
        if (forecast_data)
        {
            create_forecast_widget(current_weather_page, p_line, i, offset,
                                   forecast_data);
        }
    }

    // Create daily summary page
    lv_daily_weather_page_create(p_app_weather->tile2);

    lv_obj_add_event_cb(p_app_weather->tileview, weather_event_cb, LV_EVENT_ALL,
                        NULL);

    /* 右側弧形觸控滾動 — 在 tileview 兩個 tile 間切換。
     * slot_height = LV_VER_RES（一個 tile 一螢幕），item_count = 2，
     * slot_angle = 60°（拖過 60° 弧 = 1 個 tile，比 instruction list 寬鬆些）。
     * 預設模式（沒給 drag_cb）— arc 直接滾 tileview，放手 snap_cb 回傳目標 tile，
     * arc_scroll 的 released_cb 會 lv_obj_scroll_to_view 動畫過去 + tileview 自己
     * 的 LV_DIR_BOTTOM/TOP snap 接手 */
    arc_scroll_config_t arc_cfg = {
        .parent          = parent_tv_obj,
        .list            = p_app_weather->tileview,
        .slot_height_px  = LV_VER_RES,
        .item_height_px  = LV_VER_RES,
        .slot_angle_deg  = 60,
        .item_count      = 2,
        .band_thickness  = 90,
        .lock_ancestors  = false,
        .tap_cb          = NULL,
        .snap_cb         = weather_arc_snap_cb,
        .ctx             = NULL,
    };
    p_app_weather->arc_handle = arc_scroll_create(&arc_cfg);

    return p_app_weather->tileview;
}

/*
 * UPDATE FUNCTIONS
 *****************************************************************************************
 */
static void weather_widget_layout_update(void)
{
    for (int i = 0; i < 3; i++)
    {
        weather_t *get_weather_data =
            get_weather(WEATHER_TODAT_ITEM_AMOUNT - 1 - i);
        if (!get_weather_data)
        {
            continue;
        }

        update_weather_obj(&weather_data[i], get_weather_data);

        // Set "Now" for the current time
        if (i == 0 && lv_obj_is_valid(weather_data[i].time))
        {
            lv_label_set_text(weather_data[i].time, "Now");
        }
    }
    /* Also refresh the small weather icon shown in the dial face's
       instruction list — phone weather updates fan out from this single
       handler. */
    extern void refersh_weather_icon(void);
    refersh_weather_icon();

    lvgl_msg_handler.handle_refresh_weather_widget =
        weather_widget_layout_update;
}

void weather_layout_update(void)
{
    // Check if UI is initialized
    if (!p_app_weather)
    {
        LOG_E("Weather app not initialized");
        return;
    }

    // Add validity checks for UI elements
    if (!lv_obj_is_valid(p_app_weather->weather_location_label) ||
        !lv_obj_is_valid(p_app_weather->weather_icon) ||
        !lv_obj_is_valid(p_app_weather->cur_tem_label) ||
        !lv_obj_is_valid(p_app_weather->weather_label) ||
        !lv_obj_is_valid(p_app_weather->no_dat_label))
    {
        LOG_E("Weather UI elements are invalid");
        return;
    }

    /* Indices must match lv_card_layout_weather_create():
       current = WEATHER_TODAT_ITEM_AMOUNT-1, forecasts = AMOUNT-2..0.
       Don't early-return on NULL — render whatever slots are filled so the
       first widget refresh after entering the app picks up the freshly
       received data instead of waiting for the next app re-entry. */
    weather_t *current_weather = get_weather(WEATHER_TODAT_ITEM_AMOUNT - 1);
    if (!current_weather)
    {
        LOG_W("weather_layout_update: current weather slot is NULL");
        return;
    }

    // Update location label
    const char *location_text = get_current_location();
    if (location_text[0] == '\0')
    {
        location_text = "No Location";
    }
    lv_label_set_text(p_app_weather->weather_location_label, location_text);

    lv_img_set_src(p_app_weather->weather_icon,
                   weather_icon_get(current_weather->description));
    snprintf(buffer, sizeof(buffer), "%0.f°C",
             round(current_weather->temperature));
    lv_label_set_text(p_app_weather->cur_tem_label, buffer);
    lv_label_set_text(p_app_weather->weather_label,
                      current_weather->description);

    /* refresh_ui is only triggered when fresh weather data arrives, so by
       definition the data is not stale — always reveal the temperature /
       description and hide the "no data" placeholder. The previous
       day/month comparison incorrectly hid cur_tem_label whenever the
       phone-supplied epoch decoded to a date that didn't match the as-yet-
       unsynced Global_Time on first entry. */
    lv_obj_add_flag(p_app_weather->no_dat_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(p_app_weather->cur_tem_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(p_app_weather->weather_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align_to(p_app_weather->weather_label, p_app_weather->cur_tem_label,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    /* Forecast slots: build path uses WEATHER_TODAT_ITEM_AMOUNT-2 .. 0
       (see lv_card_layout_weather_create). Mirror that here. */
    for (int i = 0; i < 3; i++)
    {
        if (!lv_obj_is_valid(p_app_weather->future_date[i]) ||
            !lv_obj_is_valid(p_app_weather->future_weather_icon[i]) ||
            !lv_obj_is_valid(p_app_weather->future_temp_val[i]))
        {
            continue;
        }

        weather_t *forecast = get_weather(WEATHER_TODAT_ITEM_AMOUNT - 2 - i);
        if (!forecast)
        {
            continue; /* skip this slot; don't abort the whole refresh */
        }

        // Update time
        ui_time_format_hhmm(buffer, sizeof(buffer), forecast->time.hour,
                            forecast->time.minutes);
        lv_label_set_text(p_app_weather->future_date[i], buffer);

        // Update icon
        lv_img_set_src(p_app_weather->future_weather_icon[i],
                       weather_icon_get(forecast->description));

        // Update temperature
        snprintf(buffer, sizeof(buffer), "%0.f°C",
                 round(forecast->temperature));
        lv_label_set_text(p_app_weather->future_temp_val[i], buffer);
    }
}

/**
 * @brief Update UI when timer data changes
 */
static void refresh_ui(lv_obj_t *_, void *para)
{
    weather_layout_update();
    update_daily_weather_layout();
}

/*
 * LIFECYCLE FUNCTIONS
 *****************************************************************************************
 */
void notify_weather_screen_update(void)
{
    if (app_weather_data_ctx.screen_data)
    {
        lv_ex_data_set_value(app_weather_data_ctx.screen_data, (void *)NULL);
    }
}

static void on_start(lv_obj_t *scr)
{
    if (!scr)
    {
        LOG_E("Invalid screen in on_start");
        return;
    }

    RT_ASSERT(NULL == p_app_weather);
    p_app_weather = (app_weather_t *)lv_mem_alloc(sizeof(app_weather_t));

    if (!p_app_weather)
    {
        LOG_E("Failed to allocate memory for weather app");
        return;
    }

    memset(p_app_weather, 0, sizeof(app_weather_t));

    // Create UI
    p_app_weather->bg = common_black_bg(scr);
    p_app_weather->main_window = lv_card_layout_weather_create(scr);

    // Set up data binding
    lv_ex_binding_t binding;
    app_weather_data_ctx.screen_data =
        lv_ex_data_create("weather.data", LV_EX_DATA_POINTER);
    binding.target = p_app_weather->main_window;
    binding.arg_type = LV_EX_DATA_POINTER;
    binding.setter = (void *)refresh_ui;
    lv_ex_bind_data(app_weather_data_ctx.screen_data, &binding);
    request_weather_within_six_hours(true);
}

static void on_resume(void)
{
    // switch_watch_motion_control_mode(true, false);
    set_open_control_options(true);
    set_free_control_with_arm(false);
    lvgl_msg_handler.handle_tap_indicator = NULL;
}

static void on_pause(void)
{
}

static void on_stop(void)
{
    // Clean up data binding
    if (app_weather_data_ctx.screen_data)
    {
        lv_ex_data_t *data = app_weather_data_ctx.screen_data;
        app_weather_data_ctx.screen_data = NULL;
        lv_ex_data_delete(data);
    }

    // Clean up UI elements
    if (p_app_weather)
    {
        /* arc_scroll overlay 是 scr 的 child（跟 tileview 同層），自己不會被
         * tileview del 連帶清掉 — 顯式 destroy */
        if (p_app_weather->arc_handle != NULL)
        {
            arc_scroll_destroy(p_app_weather->arc_handle);
            p_app_weather->arc_handle = NULL;
        }

        if (lv_obj_is_valid(p_app_weather->bg))
        {
            lv_obj_del(p_app_weather->bg);
            p_app_weather->bg = NULL;
        }

        if (lv_obj_is_valid(p_app_weather->main_window))
        {
            lv_obj_del(p_app_weather->main_window);
            p_app_weather->main_window = NULL;
        }

        lv_mem_free(p_app_weather);
        p_app_weather = NULL;
    }
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
    {
        /* app_run 直接開啟不經 Main 狀態機，左緣右滑返回 bar 仍隱藏，這裡補開 */
        extern void display_gesture_detect_objs(uint32_t idx, bool display);
        display_gesture_detect_objs(0, true);
        lv_obj_t *scr = lv_scr_act();
        if (scr)
        {
            on_start(scr);
        }
        break;
    }
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
    gui_app_regist_msg_handler(APP_ID_WEATHER, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(weather), IMG_GROUP,
                   APP_ID_WEATHER, app_main, 1);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/