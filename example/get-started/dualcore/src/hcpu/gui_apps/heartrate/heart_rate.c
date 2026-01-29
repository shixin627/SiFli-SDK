/**
 ******************************************************************************
 * @file   heart_rate.c
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
#include "common_widget.h"
#ifdef BSP_USING_BLOC
#include "bloc_setting.h"
#include "bloc_peripheral.h"
#endif
#include "data_service_provider.h"
#include "ui_handler.h"
#include "ui_img_helper.h"

#ifdef APP_ID_HEART_RATE

#define DBG_TAG "app.heart_rate"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* Forward declarations */
static void subscribe_hr_sensor(bool subscribe);
static void measure_heart_rate_redraw(lv_timer_t *task);
static int hr_data_callback(data_callback_arg_t *arg);

/* Image declarations */
LV_IMG_DECLARE(img_red_heart);
LV_IMG_DECLARE(img_left_arrow);


/* Constants */
#define HEART_RATE_CANVAS_WIDTH 150
#define HEART_RATE_CANVAS_HEIGHT 150
#define HEART_RATE_CANVAS_BUF_SIZE LV_CANVAS_BUF_SIZE_TRUE_COLOR(HEART_RATE_CANVAS_WIDTH, HEART_RATE_CANVAS_HEIGHT)
#define HEART_RATE_REDRAW_INTERVAL_MS 100
#define MAC_BPM_STR_LEN (30)
#define CHART_WIDTH (LV_HOR_RES_MAX * 5 / 6)
#define CHART_HEIGTH (LV_VER_RES_MAX / 2)

#ifndef ABS
#define ABS(_x) ((_x) < 0 ? -(_x) : (_x))
#endif

/* Type definitions */
typedef struct
{
    lv_ex_data_t *hr_data; /* page 2 */
    lv_ex_data_t *max_data;
    lv_ex_data_t *min_data;
    lv_ex_data_t *cur_rhr;
    lv_ex_data_t *ave_rhr; /* page 3 */
    lv_ex_data_t *max_rhr;
    lv_ex_data_t *min_rhr;
    datac_handle_t data_handle;
    bool active;
} app_hr_data_ctx_t;

/* Global variables */
static app_hr_data_ctx_t app_hr_data_ctx = {0};
static custom_hr_data_table_t app_hr_data_table = {0};
static rt_uint8_t *scr_heart_rate_canvas_buffer = RT_NULL;
static lv_timer_t *heart_rate_redraw_test_task = NULL;
static rt_uint32_t heart_rate_bpm = 0;
static lv_chart_type_t chart_type = LV_CHART_TYPE_LINE;

/* UI elements */
static lv_obj_t *measure_heart_rate_canvas;
static lv_obj_t *measure_heart_rate_ecg;
static lv_obj_t *measure_heart_rate_label;
static lv_obj_t *max_heart_rate_label;
static lv_obj_t *min_heart_rate_label;
static lv_obj_t *resting_heart_rate_label;
static lv_obj_t *measure_heart_rate_chart;
static lv_obj_t *average_rhr_label;
static lv_obj_t *max_rhr_label;
static lv_obj_t *min_rhr_label;
static lv_obj_t *rhr_month_chart;
static lv_obj_t *heath_hr_label;

/* Button event handler */
static void close_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED)
    {
        gui_app_self_exit();
    }
}

/* Widget update function */
void refresh_heartrate_widget(uint8_t hr)
{
    if (!lv_obj_is_valid(heath_hr_label))
    {
        return;
    }

    char buf[16];
    snprintf(buf, sizeof(buf), "%d", hr);
    lv_label_set_text(heath_hr_label, buf);
}

/* Animation helper */
static void set_zoom(void *img, int32_t v)
{
    lv_img_set_zoom(img, v);
}

/* Create animated heart image */
static lv_obj_t *create_heart_rate_image(lv_obj_t *parent, bool animate)
{
    lv_obj_t *img = lv_img_create(parent);
    lv_img_set_src(img, LV_EXT_IMG_GET(img_red_heart));

    if (animate)
    {
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, img);
        lv_anim_set_exec_cb(&a, set_zoom);
        lv_anim_set_values(&a, 204, 256);                      /* 0.8 to 1.0 (204 to 256) */
        lv_anim_set_playback_time(&a, 1000);                   /* 1000 milliseconds for playback */
        lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE); /* Infinite repeat */
        lv_anim_start(&a);
    }
    return img;
}

/* Create heart rate unit label */
static lv_obj_t *create_heart_rate_unit_label(lv_obj_t *parent)
{
    lv_obj_t *hr_unit_label = lv_label_create(parent);
    lv_label_set_text(hr_unit_label, LV_EXT_STR_GET_BY_KEY(hr_value_display_text, "BPM"));
    lv_obj_set_style_text_color(hr_unit_label, lv_color_hex(0xFF0000), 0); /* Red color */
    return hr_unit_label;
}

/* Animation timer callback */
void measure_heart_rate_redraw(lv_timer_t *task)
{
    static rt_uint32_t zoom = 0;

    if (heart_rate_bpm)
    {
        zoom = LV_IMG_ZOOM_NONE + ((zoom + (heart_rate_bpm * HEART_RATE_REDRAW_INTERVAL_MS / 600)) % 100);
    }
    else
    {
        zoom = LV_IMG_ZOOM_NONE;
    }

    if (RT_NULL != scr_heart_rate_canvas_buffer)
    {
        memset(scr_heart_rate_canvas_buffer, 0, HEART_RATE_CANVAS_BUF_SIZE);
        lvsf_canvas_rotate(measure_heart_rate_canvas,
                           (lv_img_dsc_t *)LV_EXT_IMG_GET(img_red_heart),
                           0, zoom, 0, 0, 0, 0);
    }
}

/* Data callback function */
static int hr_data_callback(data_callback_arg_t *arg)
{
    if ((!app_hr_data_ctx.active) && (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
    {
        return 0;
    }

    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        SkaiWatchSys.hrs_start_up_mode = 1;
        RT_ASSERT(rsp);
    }
    break;

    case MSG_SERVICE_DATA_NTF_IND:
    {
        struct rt_sensor_data *data;
        RT_ASSERT(arg->data);
        data = (struct rt_sensor_data *)arg->data;
        RT_ASSERT(arg->data_len == sizeof(*data));

        if ((data->data.hr >= 0) && (data->data.hr != heart_rate_bpm) && app_hr_data_ctx.hr_data)
        {
            char *s = lv_mem_alloc(MAC_BPM_STR_LEN);
            RT_ASSERT(s);

            if (data->data.hr == 0)
            {
                rt_snprintf(s, MAC_BPM_STR_LEN, "  --  ");
            }
            else if (data->data.hr < 100)
            {
                rt_snprintf(s, MAC_BPM_STR_LEN, "  %d  ", data->data.hr);
            }
            else
            {
                rt_snprintf(s, MAC_BPM_STR_LEN, " %d   ", data->data.hr);
            }

            lv_label_set_text(measure_heart_rate_label, (const char *)s);
            heart_rate_bpm = data->data.hr;
            lv_mem_free(s);

            if (app_hr_data_table.max < data->data.hr)
            {
                app_hr_data_table.max = data->data.hr;
            }
            if (app_hr_data_table.min > data->data.hr)
            {
                app_hr_data_table.min = data->data.hr;
            }

            notify_provider.hr(data->data.hr);
            refresh_heartrate_widget(data->data.hr);
        }
    }
    break;

    case MSG_SERVICE_HR_MAX_MIN_RSP:
    {
        if (arg->data_len == HRS_MAX_MIN_LEN)
        {
            uint8_t *value = (uint8_t *)arg->data;
            char *s = lv_mem_alloc(MAC_BPM_STR_LEN);
            if (!s)
                break;

            app_hr_data_table.max = value[0];
            app_hr_data_table.min = value[1];
            app_hr_data_table.rhr = value[2];

            rt_snprintf(s, 4, "%d", app_hr_data_table.max);
            lv_label_set_text(max_heart_rate_label, (const char *)s);

            rt_snprintf(s, 4, "%d", app_hr_data_table.min);
            lv_label_set_text(min_heart_rate_label, (const char *)s);

            rt_snprintf(s, 4, "%d", app_hr_data_table.rhr);
            lv_label_set_text(resting_heart_rate_label, (const char *)s);

            LOG_D("Get max min %d: %d, rhr %d",
                  app_hr_data_table.max,
                  app_hr_data_table.min,
                  app_hr_data_table.rhr);

            lv_mem_free(s);
        }
    }
    break;

    case MSG_SERVICE_HR_DAY_TABLE_RSP:
    {
        if (arg->data_len == HRS_DAY_TABLE_LEN)
        {
            uint8_t *value = (uint8_t *)arg->data;
            int i;

            for (i = 0; i < HRS_DAY_TABLE_LEN; i++)
            {
                app_hr_data_table.today[i] = (uint16_t)value[i];
            }

            lv_chart_series_t *ser1 = lv_chart_add_series(
                measure_heart_rate_chart,
                lv_palette_main(LV_PALETTE_RED),
                LV_CHART_AXIS_PRIMARY_Y);

            for (i = 0; i < HRS_DAY_TABLE_LEN; i++)
            {
                ser1->y_points[i] = app_hr_data_table.today[i];
            }

            lv_chart_refresh(measure_heart_rate_chart);
        }
    }
    break;

    case MSG_SERVICE_HR_MON_TABLE_RSP:
    {
        if (arg->data_len == HRS_MON_TABLE_LEN)
        {
            uint8_t *value = (uint8_t *)arg->data;
            int i;

            for (i = 0; i < HRS_MON_TABLE_LEN; i++)
            {
                app_hr_data_table.mon[i] = (uint16_t)value[i];
            }

            lv_chart_series_t *ser1 = lv_chart_add_series(
                rhr_month_chart,
                lv_palette_main(LV_PALETTE_RED),
                LV_CHART_AXIS_PRIMARY_Y);

            for (i = 0; i < HRS_MON_TABLE_LEN; i++)
            {
                ser1->y_points[i] = app_hr_data_table.mon[i];
            }

            lv_chart_refresh(rhr_month_chart);
        }
    }
    break;

    case MSG_SERVICE_RHR_VALUE_RSP:
    {
        if (arg->data_len == HRS_RHR_HIST_LEN)
        {
            uint8_t *value = (uint8_t *)arg->data;
            char *s = lv_mem_alloc(MAC_BPM_STR_LEN);
            if (!s)
                break;

            app_hr_data_table.max_rhr = value[0];
            app_hr_data_table.min_rhr = value[1];
            app_hr_data_table.ave_rhr = value[2];

            rt_snprintf(s, MAC_BPM_STR_LEN - 1, "AVERAGE: %d", app_hr_data_table.ave_rhr);
            lv_label_set_text(average_rhr_label, (const char *)s);

            rt_snprintf(s, 4, "%d", app_hr_data_table.max_rhr);
            lv_label_set_text(max_rhr_label, (const char *)s);

            rt_snprintf(s, 4, "%d", app_hr_data_table.min_rhr);
            lv_label_set_text(min_rhr_label, (const char *)s);

            LOG_D("Get rhr %d: %d, %d",
                  app_hr_data_table.max_rhr,
                  app_hr_data_table.min_rhr,
                  app_hr_data_table.ave_rhr);

            lv_mem_free(s);
        }
    }
    break;

    case MSG_SERVICE_HR_REGION_RSP:
    {
        if (arg->data_len == HRS_REGION_LEN)
        {
            uint8_t *value = (uint8_t *)arg->data;

            for (int i = 0; i < 5; i++)
            {
                app_hr_data_table.region[i] = value[i];
            }

            LOG_D("Get region %d: %d, %d, %d, %d, %d",
                  app_hr_data_table.region[0],
                  app_hr_data_table.region[1],
                  app_hr_data_table.region[2],
                  app_hr_data_table.region[3],
                  app_hr_data_table.region[4]);
        }
    }
    break;
    }

    return 0;
}

/* Create UI pages */
void create_heart_rate_region_page(lv_obj_t *parent)
{
    lv_ex_binding_t binding;

    /* Create animated heart */
    lv_obj_t *red_heart_animation_icon = create_heart_rate_image(parent, true);
    lv_obj_align(red_heart_animation_icon, LV_ALIGN_TOP_MID, 0, 120);

    /* Create measuring label */
    lv_obj_t *measuring_label = lv_label_create(parent);
    lv_label_set_text(measuring_label, LV_EXT_STR_GET_BY_KEY(hr_measuring_text, "Measuring..."));
    lv_obj_align_to(measuring_label, red_heart_animation_icon, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    /* Create heart rate display */
    measure_heart_rate_label = lv_label_create(parent);
    lv_label_set_text(measure_heart_rate_label, "  --  ");
    lv_obj_set_style_text_font(measure_heart_rate_label, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_align_to(measure_heart_rate_label, measuring_label, LV_ALIGN_OUT_BOTTOM_MID, -20, 20);

    /* Create unit label */
    lv_obj_t *hr_unit_label = create_heart_rate_unit_label(parent);
    lv_obj_align_to(hr_unit_label, measure_heart_rate_label, LV_ALIGN_OUT_RIGHT_MID, 20, 0);

    /* Setup data binding */
    app_hr_data_ctx.hr_data = lv_ex_data_create("hr.hr_val", LV_EX_DATA_STRING);
    binding.target = measure_heart_rate_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.hr_data, &binding);
}

void create_measure_heart_rate_page(lv_obj_t *parent)
{
    lv_ex_binding_t binding;

    /* Create title */
    lv_obj_t *title_container = lv_obj_create(parent);
    lv_obj_set_size(title_container, LV_HOR_RES_MAX, (LV_VER_RES_MAX / 6));
    lv_obj_align(title_container, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_t *title = lv_label_create(title_container);
    lv_label_set_text(title, LV_EXT_STR_GET_BY_KEY(hr_title, "HR Title"));
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);

    /* Create chart */
    measure_heart_rate_chart = lv_chart_create(parent);
    lv_obj_set_size(measure_heart_rate_chart, CHART_WIDTH, CHART_HEIGTH);
    lv_obj_align_to(measure_heart_rate_chart, parent, LV_ALIGN_CENTER, 0, 20);

    /* Set chart styling */
    lv_obj_set_style_bg_opa(measure_heart_rate_chart, LV_OPA_50, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(measure_heart_rate_chart, LV_GRAD_DIR_VER, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(measure_heart_rate_chart, 255, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(measure_heart_rate_chart, 0, LV_PART_ITEMS | LV_STATE_DEFAULT);

    lv_chart_set_type(measure_heart_rate_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_div_line_count(measure_heart_rate_chart, 3, 3);

    lv_obj_set_style_text_font(measure_heart_rate_chart, LV_EXT_FONT_GET(get_system_font_size(-3)), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(measure_heart_rate_chart, 40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(measure_heart_rate_chart, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_color(measure_heart_rate_chart, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(measure_heart_rate_chart, 80, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_chart_set_axis_tick(measure_heart_rate_chart, LV_CHART_AXIS_PRIMARY_X, 10, 5, 5, 6, true, 40);
    lv_chart_set_axis_tick(measure_heart_rate_chart, LV_CHART_AXIS_PRIMARY_Y, 10, 1, 4, 1, true, 40);

    lv_chart_set_range(measure_heart_rate_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 200);
    lv_chart_set_range(measure_heart_rate_chart, LV_CHART_AXIS_PRIMARY_X, 0, 24);
    lv_chart_set_point_count(measure_heart_rate_chart, 24);

    lv_chart_refresh(measure_heart_rate_chart);

    /* Create bottom indicators */
    lv_obj_t *max = lv_img_create(parent);
    lv_img_set_src(max, UP_ARROW);
    lv_obj_align(max, LV_ALIGN_BOTTOM_LEFT, 60, -20);

    max_heart_rate_label = lv_label_create(parent);
    lv_label_set_text(max_heart_rate_label, "0");
    lv_obj_align_to(max_heart_rate_label, max, LV_ALIGN_OUT_RIGHT_MID, 2, 0);

    app_hr_data_ctx.max_data = lv_ex_data_create("max_val", LV_EX_DATA_STRING);
    binding.target = max_heart_rate_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.max_data, &binding);

    lv_obj_t *min = lv_img_create(parent);
    lv_img_set_src(min, DOWN_ARROW);
    lv_obj_align(min, LV_ALIGN_BOTTOM_RIGHT, -80, -20);

    min_heart_rate_label = lv_label_create(parent);
    lv_label_set_text(min_heart_rate_label, "0");
    lv_obj_align_to(min_heart_rate_label, min, LV_ALIGN_OUT_RIGHT_MID, 2, 0);

    app_hr_data_ctx.min_data = lv_ex_data_create("min_val", LV_EX_DATA_STRING);
    binding.target = min_heart_rate_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.min_data, &binding);

    lv_obj_t *rhr = lv_label_create(parent);
    lv_label_set_text(rhr, "RHR");
    lv_obj_align(rhr, LV_ALIGN_BOTTOM_MID, 0, 0);

    resting_heart_rate_label = lv_label_create(parent);
    lv_label_set_text(resting_heart_rate_label, "0");
    lv_obj_align_to(resting_heart_rate_label, rhr, LV_ALIGN_OUT_RIGHT_MID, 4, 0);

    app_hr_data_ctx.cur_rhr = lv_ex_data_create("cur_rhr", LV_EX_DATA_STRING);
    binding.target = resting_heart_rate_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.cur_rhr, &binding);
}

void create_heart_rate_lastmonth_page(lv_obj_t *parent)
{
    lv_ex_binding_t binding;

    /* Create title */
    lv_obj_t *title_container = lv_obj_create(parent);
    lv_obj_set_size(title_container, LV_HOR_RES_MAX, (LV_VER_RES_MAX / 4));
    lv_obj_align_to(title_container, NULL, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_t *title = lv_label_create(title_container);
    lv_label_set_text(title, LV_EXT_STR_GET_BY_KEY(hr_past_30_day, "Past 30 Days"));
    lv_obj_align_to(title, title_container, LV_ALIGN_TOP_MID, 0, 0);

    /* Create average display */
    average_rhr_label = lv_label_create(parent);
    lv_label_set_text(average_rhr_label, "AVERAGE: 60");

    app_hr_data_ctx.ave_rhr = lv_ex_data_create("ave_rhr", LV_EX_DATA_STRING);
    binding.target = average_rhr_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.ave_rhr, &binding);

    lv_obj_align_to(average_rhr_label, title_container, LV_ALIGN_BOTTOM_MID, 0, 0);

    /* Create chart */
    rhr_month_chart = lv_chart_create(parent);
    lv_obj_set_size(rhr_month_chart, CHART_WIDTH, CHART_HEIGTH);
    lv_obj_align_to(rhr_month_chart, parent, LV_ALIGN_CENTER, -10, 20);

    /* Set chart styling */
    lv_obj_set_style_bg_opa(rhr_month_chart, LV_OPA_50, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(rhr_month_chart, LV_GRAD_DIR_VER, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(rhr_month_chart, 255, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(rhr_month_chart, 0, LV_PART_ITEMS | LV_STATE_DEFAULT);

    lv_chart_set_type(rhr_month_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_div_line_count(rhr_month_chart, 4, 2);

    lv_obj_set_style_text_font(rhr_month_chart, LV_EXT_FONT_GET(get_system_font_size(-3)), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(rhr_month_chart, 40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(rhr_month_chart, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_dash_gap(rhr_month_chart, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_color(rhr_month_chart, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(rhr_month_chart, 80, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_chart_set_axis_tick(rhr_month_chart, LV_CHART_AXIS_PRIMARY_X, 10, 5, 4, 10, true, 40);
    lv_chart_set_axis_tick(rhr_month_chart, LV_CHART_AXIS_PRIMARY_Y, 10, 1, 4, 1, true, 40);

    lv_chart_set_range(rhr_month_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 120);
    lv_chart_set_range(rhr_month_chart, LV_CHART_AXIS_PRIMARY_X, 0, 30);
    lv_chart_set_point_count(rhr_month_chart, 30);

    lv_chart_refresh(rhr_month_chart);

    /* Create bottom indicators */
    lv_obj_t *max = lv_img_create(parent);
    lv_img_set_src(max, UP_ARROW);
    lv_obj_align(max, LV_ALIGN_BOTTOM_LEFT, 60, -20);

    max_rhr_label = lv_label_create(parent);
    lv_label_set_text(max_rhr_label, "72");

    app_hr_data_ctx.max_rhr = lv_ex_data_create("max_rhr", LV_EX_DATA_STRING);
    binding.target = max_rhr_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.max_rhr, &binding);

    lv_obj_align_to(max_rhr_label, max, LV_ALIGN_OUT_RIGHT_MID, 2, 0);

    lv_obj_t *min = lv_img_create(parent);
    lv_img_set_src(min, DOWN_ARROW);
    lv_obj_align(min, LV_ALIGN_BOTTOM_RIGHT, -80, -20);

    min_rhr_label = lv_label_create(parent);
    lv_label_set_text(min_rhr_label, "65");

    app_hr_data_ctx.min_rhr = lv_ex_data_create("min_rhr", LV_EX_DATA_STRING);
    binding.target = min_rhr_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.min_rhr, &binding);

    lv_obj_align_to(min_rhr_label, min, LV_ALIGN_OUT_RIGHT_MID, 2, 0);
}

/* Create health widget */
lv_obj_t *lv_health_widget_builder(lv_obj_t *parent)
{
    lv_obj_t *widget = common_widget_container(parent);

    // lv_obj_t *bar = lv_obj_create(widget);
    // lv_obj_set_size(bar, 400, 90);
    // lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 0);
    // lv_obj_set_style_bg_color(bar, lv_color_hex(0xFF2323), 0);

    /* Recent HR label */
    lv_obj_t *recent_hr_label = lv_label_create(widget);
    lv_label_set_text(recent_hr_label, "Recent:");
    lv_obj_align(recent_hr_label, LV_ALIGN_LEFT_MID, 50, 0);

    /* Heart rate value */
    heath_hr_label = lv_label_create(widget);
    lv_label_set_text(heath_hr_label, "  --  ");
    lv_obj_set_style_text_font(heath_hr_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_label_set_long_mode(heath_hr_label, LV_LABEL_LONG_WRAP);
    lv_obj_align_to(heath_hr_label, recent_hr_label, LV_ALIGN_OUT_RIGHT_MID, 30, 0);

    /* HR unit label */
    lv_obj_t *hr_unit_label = create_heart_rate_unit_label(widget);
    lv_obj_align_to(hr_unit_label, heath_hr_label, LV_ALIGN_OUT_RIGHT_MID, 20, 0);

    return widget;
}

/* Initialize heart rate UI */
void heart_rate_init(lv_obj_t *scr)
{
    /* Create tileview navigation */
    lv_obj_t *tileview = lv_tileview_create(lv_scr_act());
    lv_obj_t *pages[3];

    /* Create tileview pages */
    for (rt_uint16_t i = 0; i < 3; i++)
    {
        if (0 == i)
        {
            pages[i] = lv_tileview_add_tile(tileview, 0, i, LV_DIR_BOTTOM);
        }
        else if (2 == i)
        {
            pages[i] = lv_tileview_add_tile(tileview, 0, i, LV_DIR_TOP);
        }
        else
        {
            pages[i] = lv_tileview_add_tile(tileview, 0, i, LV_DIR_VER);
        }
    }

    /* Create page content */
    create_heart_rate_region_page(pages[0]);
    create_measure_heart_rate_page(pages[1]);
    create_heart_rate_lastmonth_page(pages[2]);
}

/* Subscribe/unsubscribe to HR sensor */
static void subscribe_hr_sensor(bool subscribe)
{
    if (subscribe)
    {
        if (DATA_CLIENT_INVALID_HANDLE == app_hr_data_ctx.data_handle)
        {
            app_hr_data_ctx.data_handle = datac_open();
            RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != app_hr_data_ctx.data_handle);
            ui_datac_subscribe(app_hr_data_ctx.data_handle, "HR", hr_data_callback, 0);
        }
        else
        {
            LOG_E("hr_service_handle is invalid\n");
            return;
        }

        /* Request sensor data */
        if (DATA_CLIENT_INVALID_HANDLE != app_hr_data_ctx.data_handle)
        {
            data_msg_t msg;
            uint8_t *msg_payload;

            /* Request day table */
            msg_payload = data_service_init_msg(&msg, MSG_SERVICE_HR_DAY_TABLE_REQ, 1);
            msg_payload[0] = HRS_DAY_TABLE_LEN;
            datac_send_msg(app_hr_data_ctx.data_handle, &msg);

            /* Request max min */
            msg_payload = data_service_init_msg(&msg, MSG_SERVICE_HR_MAX_MIN_REQ, 1);
            msg_payload[0] = HRS_MAX_MIN_LEN;
            datac_send_msg(app_hr_data_ctx.data_handle, &msg);

            /* Request region */
            msg_payload = data_service_init_msg(&msg, MSG_SERVICE_HR_REGION_REQ, 1);
            msg_payload[0] = HRS_REGION_LEN;
            datac_send_msg(app_hr_data_ctx.data_handle, &msg);

            /* Request month table */
            msg_payload = data_service_init_msg(&msg, MSG_SERVICE_HR_MON_TABLE_REQ, 1);
            msg_payload[0] = HRS_MON_TABLE_LEN;
            datac_send_msg(app_hr_data_ctx.data_handle, &msg);

            /* Request RHR value */
            msg_payload = data_service_init_msg(&msg, MSG_SERVICE_RHR_VALUE_REQ, 1);
            msg_payload[0] = HRS_RHR_HIST_LEN;
            datac_send_msg(app_hr_data_ctx.data_handle, &msg);
        }
    }
    else
    {
        /* Close data client */
        if (app_hr_data_ctx.data_handle != DATA_CLIENT_INVALID_HANDLE)
        {
            if (datac_close(app_hr_data_ctx.data_handle) == 0)
            {
                app_hr_data_ctx.data_handle = DATA_CLIENT_INVALID_HANDLE;
                SkaiWatchSys.hrs_start_up_mode = 0;
            }
        }
    }
}

/* App lifecycle functions */
static void on_start(lv_obj_t *scr)
{
    app_hr_data_ctx.active = true;
    heart_rate_init(scr);
}

static void on_resume(void)
{
    setting_provider.set_power_save_mode(0);
    subscribe_hr_sensor(true);

    if (NULL == heart_rate_redraw_test_task)
    {
        heart_rate_redraw_test_task = lv_timer_create(
            measure_heart_rate_redraw,
            HEART_RATE_REDRAW_INTERVAL_MS,
            NULL);
    }
}

static void on_pause(void)
{
    app_hr_data_ctx.active = false;
    subscribe_hr_sensor(false);

    if (heart_rate_redraw_test_task)
    {
        lv_timer_del(heart_rate_redraw_test_task);
        heart_rate_redraw_test_task = NULL;
    }

    setting_provider.set_power_save_mode(1);
}

static void on_stop(void)
{
    /* Free all data objects */
    lv_ex_data_t *data_objects[] = {
        app_hr_data_ctx.hr_data,
        app_hr_data_ctx.max_data,
        app_hr_data_ctx.min_data,
        app_hr_data_ctx.cur_rhr,
        app_hr_data_ctx.ave_rhr,
        app_hr_data_ctx.max_rhr,
        app_hr_data_ctx.min_rhr};

    for (int i = 0; i < sizeof(data_objects) / sizeof(data_objects[0]); i++)
    {
        if (data_objects[i])
        {
            lv_ex_data_delete(data_objects[i]);
            data_objects[i] = NULL;
        }
    }

    /* Reset data context pointers */
    memset(&app_hr_data_ctx, 0, sizeof(app_hr_data_ctx));

    /* Free canvas buffer */
    if (scr_heart_rate_canvas_buffer)
    {
        lv_mem_free(scr_heart_rate_canvas_buffer);
        scr_heart_rate_canvas_buffer = RT_NULL;
    }

    heart_rate_bpm = 0;
}

/* App message handler */
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

/* App entry point */
static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_HEART_RATE, msg_handler);
    return 0;
}

/* Register app */
BUILTIN_APP_EXPORT(LV_EXT_STR_ID(heart_rate), IMG_HEART_RATE, APP_ID_HEART_RATE, app_main);

#endif /* APP_ID_HEART_RATE */
       /************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/