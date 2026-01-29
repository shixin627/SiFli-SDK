/**
 ******************************************************************************
 * @file   barometer.c
 * @author Sifli software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Sifli Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated
 * circuit in a product or a software update for such product, must reproduce
 * the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * 3. The names of Sifli or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
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
#include "sensor.h"
#include "baro_service.h"
#include "ui_datasrv_subscriber.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include "data_service_provider.h"
#include "ui_handler.h"
#include "bloc_setting.h"

#define DBG_TAG "app.barometer"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_BAROMETER

/* Forward declarations */
static void subscribe_baro_sensor(bool subscribe);
static int baro_data_callback(data_callback_arg_t *arg);

/* Image declarations */
LV_IMG_DECLARE(img_left_arrow);
LV_IMG_DECLARE(img_barometer);

    /* Constants */
    #define MAX_PRESSURE_STR_LEN (30)

/* Type definitions */
typedef struct
{
    lv_ex_data_t *cur_baro_data;
    lv_ex_data_t *max_baro_data;
    lv_ex_data_t *min_baro_data;
    lv_ex_data_t *cur_alti_data;
    lv_ex_data_t *max_alti_data;
    lv_ex_data_t *min_alti_data;
    datac_handle_t data_handle;
    bool active;
} app_baro_data_ctx_t;

/* Global variables */
static app_baro_data_ctx_t app_baro_data_ctx = {0};
static custom_baro_data_table_t app_baro_data_table = {0};

/* UI elements */
static lv_obj_t *cur_pressure_label;
static lv_obj_t *max_pressure_label;
static lv_obj_t *min_pressure_label;
static lv_obj_t *cur_altitude_label;
static lv_obj_t *max_altitude_label;
static lv_obj_t *min_altitude_label;

/* Button event handler */
static void close_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED)
    {
        gui_app_self_exit();
    }
}

/* Data callback function */
static int baro_data_callback(data_callback_arg_t *arg)
{
    if ((!app_baro_data_ctx.active) &&
        (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
    {
        return 0;
    }

    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        LOG_I("Barometer service subscribed successfully");
        skaiwatch_ble_set_performance(true);
    }
    break;

    case MSG_SERVICE_UNSUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        LOG_I("Barometer service unsubscribed successfully");
        skaiwatch_ble_set_performance(false);
    }
    break;

    case MSG_SERVICE_DATA_NTF_IND:
    {
        struct rt_sensor_data *data;
        RT_ASSERT(arg->data);
        data = (struct rt_sensor_data *)arg->data;
        RT_ASSERT(arg->data_len == sizeof(*data));

        if ((data->data.baro > 0))
        {
            L1SendData l1event = {.event = L1SEND_BARO_BUFFER,
                                  .res.baro_data =
                                      (float)data->data.baro / 100};
            L1_send_event(l1event);
            if ((data->data.baro != app_baro_data_table.cur_baro))
            {
                char *s = lv_mem_alloc(MAX_PRESSURE_STR_LEN);
                RT_ASSERT(s);

                // Update current pressure (Pa -> hPa)
                app_baro_data_table.cur_baro = data->data.baro;
                rt_snprintf(s, MAX_PRESSURE_STR_LEN, "%d.%02d hPa",
                            data->data.baro / 100, (data->data.baro % 100));
                if (lv_obj_is_valid(cur_pressure_label))
                {
                    lv_label_set_text(cur_pressure_label, (const char *)s);
                }
                LOG_D("Current Pressure: %s", s);
                lv_mem_free(s);
            }
        }
    }
    break;

    case MSG_SERVICE_BARO_RANGE_RSP:
    {
        if (arg->data_len == BAROS_BARO_RANGE_LEN)
        {
            uint32_t *value = (uint32_t *)arg->data;
            char *s = lv_mem_alloc(MAX_PRESSURE_STR_LEN);
            if (!s)
                break;

            app_baro_data_table.max_baro = value[0];
            app_baro_data_table.min_baro = value[1];

            // Update max pressure
            rt_snprintf(s, MAX_PRESSURE_STR_LEN, "%d.%02d hPa",
                        app_baro_data_table.max_baro / 100,
                        (app_baro_data_table.max_baro % 100));
            if (lv_obj_is_valid(max_pressure_label))
            {
                lv_label_set_text(max_pressure_label, (const char *)s);
            }

            // Update min pressure
            rt_snprintf(s, MAX_PRESSURE_STR_LEN, "%d.%02d hPa",
                        app_baro_data_table.min_baro / 100,
                        (app_baro_data_table.min_baro % 100));
            if (lv_obj_is_valid(min_pressure_label))
            {
                lv_label_set_text(min_pressure_label, (const char *)s);
            }

            LOG_D("Baro Range - Max: %d, Min: %d", app_baro_data_table.max_baro,
                  app_baro_data_table.min_baro);

            lv_mem_free(s);
        }
    }
    break;

    case MSG_SERVICE_ALTITUDE_RANGE_RSP:
    {
        if (arg->data_len == BAROS_ALTI_RANGE_LEN)
        {
            int32_t *value = (int32_t *)arg->data;
            char *s = lv_mem_alloc(MAX_PRESSURE_STR_LEN);
            if (!s)
                break;

            app_baro_data_table.cur_alti = value[0];
            app_baro_data_table.max_alti = value[1];
            app_baro_data_table.min_alti = value[2];

            // Update current altitude (convert from cm to m)
            rt_snprintf(s, MAX_PRESSURE_STR_LEN, "%d.%02d m",
                        app_baro_data_table.cur_alti / 100,
                        abs(app_baro_data_table.cur_alti % 100));
            if (lv_obj_is_valid(cur_altitude_label))
            {
                lv_label_set_text(cur_altitude_label, (const char *)s);
            }

            // Update max altitude
            rt_snprintf(s, MAX_PRESSURE_STR_LEN, "%d.%02d m",
                        app_baro_data_table.max_alti / 100,
                        abs(app_baro_data_table.max_alti % 100));
            if (lv_obj_is_valid(max_altitude_label))
            {
                lv_label_set_text(max_altitude_label, (const char *)s);
            }

            // Update min altitude
            rt_snprintf(s, MAX_PRESSURE_STR_LEN, "%d.%02d m",
                        app_baro_data_table.min_alti / 100,
                        abs(app_baro_data_table.min_alti % 100));
            if (lv_obj_is_valid(min_altitude_label))
            {
                lv_label_set_text(min_altitude_label, (const char *)s);
            }

            LOG_D("Altitude Range - Cur: %d, Max: %d, Min: %d",
                  app_baro_data_table.cur_alti, app_baro_data_table.max_alti,
                  app_baro_data_table.min_alti);

            lv_mem_free(s);
        }
    }
    break;
    }

    return 0;
}

/* Create barometer UI page */
void create_barometer_page(lv_obj_t *parent)
{
    lv_ex_binding_t binding;

    /* Create title */
    lv_obj_t *title_container = lv_obj_create(parent);
    lv_obj_set_size(title_container, LV_HOR_RES_MAX, (LV_VER_RES_MAX / 8));
    lv_obj_align(title_container, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_clear_flag(title_container, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(title_container);
    lv_label_set_text(title, "Barometer");
    lv_obj_set_style_text_font(title, LV_EXT_FONT_GET(get_system_font_size(0)),
                               0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);

    /* Create close button */
    lv_obj_t *close_btn = lv_btn_create(title_container);
    lv_obj_set_size(close_btn, 60, 40);
    lv_obj_align(close_btn, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_add_event_cb(close_btn, close_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *back_img = lv_img_create(close_btn);
    lv_img_set_src(back_img, LV_EXT_IMG_GET(img_left_arrow));
    lv_obj_align(back_img, LV_ALIGN_CENTER, 0, 0);

    /* ============ Pressure Section ============ */
    lv_obj_t *pressure_container = lv_obj_create(parent);
    lv_obj_set_size(pressure_container, LV_HOR_RES_MAX - 40,
                    (LV_VER_RES_MAX / 3));
    lv_obj_align_to(pressure_container, title_container,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
    lv_obj_clear_flag(pressure_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(pressure_container, lv_color_hex(0x2C3E50), 0);

    /* Pressure title */
    lv_obj_t *pressure_title = lv_label_create(pressure_container);
    lv_label_set_text(pressure_title, "Air Pressure");
    lv_obj_set_style_text_color(pressure_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(pressure_title, LV_ALIGN_TOP_MID, 0, 10);

    /* Current pressure */
    lv_obj_t *cur_pressure_title = lv_label_create(pressure_container);
    lv_label_set_text(cur_pressure_title, "Current:");
    lv_obj_set_style_text_color(cur_pressure_title, lv_color_hex(0xBDC3C7), 0);
    lv_obj_align(cur_pressure_title, LV_ALIGN_TOP_LEFT, 20, 50);

    cur_pressure_label = lv_label_create(pressure_container);
    lv_label_set_text(cur_pressure_label, "-- hPa");
    lv_obj_set_style_text_color(cur_pressure_label, lv_color_hex(0x3498DB), 0);
    lv_obj_set_style_text_font(cur_pressure_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(cur_pressure_label, cur_pressure_title,
                    LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    /* Max pressure */
    lv_obj_t *max_pressure_title = lv_label_create(pressure_container);
    lv_label_set_text(max_pressure_title, "Max:");
    lv_obj_set_style_text_color(max_pressure_title, lv_color_hex(0xBDC3C7), 0);
    lv_obj_align(max_pressure_title, LV_ALIGN_LEFT_MID, 20, 10);

    max_pressure_label = lv_label_create(pressure_container);
    lv_label_set_text(max_pressure_label, "-- hPa");
    lv_obj_set_style_text_color(max_pressure_label, lv_color_hex(0xE74C3C), 0);
    lv_obj_align_to(max_pressure_label, max_pressure_title,
                    LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    /* Min pressure */
    lv_obj_t *min_pressure_title = lv_label_create(pressure_container);
    lv_label_set_text(min_pressure_title, "Min:");
    lv_obj_set_style_text_color(min_pressure_title, lv_color_hex(0xBDC3C7), 0);
    lv_obj_align(min_pressure_title, LV_ALIGN_BOTTOM_LEFT, 20, -10);

    min_pressure_label = lv_label_create(pressure_container);
    lv_label_set_text(min_pressure_label, "-- hPa");
    lv_obj_set_style_text_color(min_pressure_label, lv_color_hex(0x2ECC71), 0);
    lv_obj_align_to(min_pressure_label, min_pressure_title,
                    LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    /* Setup data binding for pressure */
    app_baro_data_ctx.cur_baro_data =
        lv_ex_data_create("baro.cur_baro", LV_EX_DATA_STRING);
    binding.target = cur_pressure_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_baro_data_ctx.cur_baro_data, &binding);

    app_baro_data_ctx.max_baro_data =
        lv_ex_data_create("baro.max_baro", LV_EX_DATA_STRING);
    binding.target = max_pressure_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_baro_data_ctx.max_baro_data, &binding);

    app_baro_data_ctx.min_baro_data =
        lv_ex_data_create("baro.min_baro", LV_EX_DATA_STRING);
    binding.target = min_pressure_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_baro_data_ctx.min_baro_data, &binding);

    /* ============ Altitude Section ============ */
    lv_obj_t *altitude_container = lv_obj_create(parent);
    lv_obj_set_size(altitude_container, LV_HOR_RES_MAX - 40,
                    (LV_VER_RES_MAX / 3));
    lv_obj_align_to(altitude_container, pressure_container,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
    lv_obj_clear_flag(altitude_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(altitude_container, lv_color_hex(0x34495E), 0);

    /* Altitude title */
    lv_obj_t *altitude_title = lv_label_create(altitude_container);
    lv_label_set_text(altitude_title, "Altitude");
    lv_obj_set_style_text_color(altitude_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(altitude_title, LV_ALIGN_TOP_MID, 0, 10);

    /* Current altitude */
    lv_obj_t *cur_altitude_title = lv_label_create(altitude_container);
    lv_label_set_text(cur_altitude_title, "Current:");
    lv_obj_set_style_text_color(cur_altitude_title, lv_color_hex(0xBDC3C7), 0);
    lv_obj_align(cur_altitude_title, LV_ALIGN_TOP_LEFT, 20, 50);

    cur_altitude_label = lv_label_create(altitude_container);
    lv_label_set_text(cur_altitude_label, "-- m");
    lv_obj_set_style_text_color(cur_altitude_label, lv_color_hex(0x9B59B6), 0);
    lv_obj_set_style_text_font(cur_altitude_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(cur_altitude_label, cur_altitude_title,
                    LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    /* Max altitude */
    lv_obj_t *max_altitude_title = lv_label_create(altitude_container);
    lv_label_set_text(max_altitude_title, "Max:");
    lv_obj_set_style_text_color(max_altitude_title, lv_color_hex(0xBDC3C7), 0);
    lv_obj_align(max_altitude_title, LV_ALIGN_LEFT_MID, 20, 10);

    max_altitude_label = lv_label_create(altitude_container);
    lv_label_set_text(max_altitude_label, "-- m");
    lv_obj_set_style_text_color(max_altitude_label, lv_color_hex(0xF39C12), 0);
    lv_obj_align_to(max_altitude_label, max_altitude_title,
                    LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    /* Min altitude */
    lv_obj_t *min_altitude_title = lv_label_create(altitude_container);
    lv_label_set_text(min_altitude_title, "Min:");
    lv_obj_set_style_text_color(min_altitude_title, lv_color_hex(0xBDC3C7), 0);
    lv_obj_align(min_altitude_title, LV_ALIGN_BOTTOM_LEFT, 20, -10);

    min_altitude_label = lv_label_create(altitude_container);
    lv_label_set_text(min_altitude_label, "-- m");
    lv_obj_set_style_text_color(min_altitude_label, lv_color_hex(0x1ABC9C), 0);
    lv_obj_align_to(min_altitude_label, min_altitude_title,
                    LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    /* Setup data binding for altitude */
    app_baro_data_ctx.cur_alti_data =
        lv_ex_data_create("baro.cur_alti", LV_EX_DATA_STRING);
    binding.target = cur_altitude_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_baro_data_ctx.cur_alti_data, &binding);

    app_baro_data_ctx.max_alti_data =
        lv_ex_data_create("baro.max_alti", LV_EX_DATA_STRING);
    binding.target = max_altitude_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_baro_data_ctx.max_alti_data, &binding);

    app_baro_data_ctx.min_alti_data =
        lv_ex_data_create("baro.min_alti", LV_EX_DATA_STRING);
    binding.target = min_altitude_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_baro_data_ctx.min_alti_data, &binding);
}

/* Initialize barometer UI */
void barometer_init(lv_obj_t *scr)
{
    create_barometer_page(scr);
}

/* Subscribe/unsubscribe to barometer sensor */
static void subscribe_baro_sensor(bool subscribe)
{
    if (subscribe)
    {
        if (DATA_CLIENT_INVALID_HANDLE == app_baro_data_ctx.data_handle)
        {
            app_baro_data_ctx.data_handle = datac_open();
            RT_ASSERT(DATA_CLIENT_INVALID_HANDLE !=
                      app_baro_data_ctx.data_handle);
            ui_datac_subscribe(app_baro_data_ctx.data_handle, "BARO",
                               baro_data_callback, 0);
        }
        else
        {
            LOG_E("baro_service_handle is invalid\n");
            return;
        }

        /* Request sensor data */
        if (DATA_CLIENT_INVALID_HANDLE != app_baro_data_ctx.data_handle)
        {
            data_msg_t msg;
            uint8_t *msg_payload;

            /* Request barometer range */
            msg_payload =
                data_service_init_msg(&msg, MSG_SERVICE_BARO_RANGE_REQ, 1);
            msg_payload[0] = BAROS_BARO_RANGE_LEN;
            datac_send_msg(app_baro_data_ctx.data_handle, &msg);

            /* Request altitude range */
            msg_payload =
                data_service_init_msg(&msg, MSG_SERVICE_ALTITUDE_RANGE_REQ, 1);
            msg_payload[0] = BAROS_ALTI_RANGE_LEN;
            datac_send_msg(app_baro_data_ctx.data_handle, &msg);
        }
    }
    else
    {
        /* Close data client */
        if (app_baro_data_ctx.data_handle != DATA_CLIENT_INVALID_HANDLE)
        {
            if (datac_close(app_baro_data_ctx.data_handle) == 0)
            {
                app_baro_data_ctx.data_handle = DATA_CLIENT_INVALID_HANDLE;
            }
        }
    }
}

/* App lifecycle functions */
static void on_start(lv_obj_t *scr)
{
    app_baro_data_ctx.active = true;
    barometer_init(scr);
}

static void on_resume(void)
{
    setting_provider.set_power_save_mode(0);
    subscribe_baro_sensor(true);
}

static void on_pause(void)
{
    app_baro_data_ctx.active = false;
    subscribe_baro_sensor(false);
    setting_provider.set_power_save_mode(1);
}

static void on_stop(void)
{
    /* Free all data objects */
    lv_ex_data_t *data_objects[] = {
        app_baro_data_ctx.cur_baro_data, app_baro_data_ctx.max_baro_data,
        app_baro_data_ctx.min_baro_data, app_baro_data_ctx.cur_alti_data,
        app_baro_data_ctx.max_alti_data, app_baro_data_ctx.min_alti_data};

    for (int i = 0; i < sizeof(data_objects) / sizeof(data_objects[0]); i++)
    {
        if (data_objects[i])
        {
            lv_ex_data_delete(data_objects[i]);
            data_objects[i] = NULL;
        }
    }

    /* Reset data context pointers */
    memset(&app_baro_data_ctx, 0, sizeof(app_baro_data_ctx));
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
    gui_app_regist_msg_handler(APP_ID_BAROMETER, msg_handler);
    return 0;
}

/* Register app */
BUILTIN_APP_EXPORT(LV_EXT_STR_ID(barometer), LV_EXT_IMG_GET(img_barometer),
                   APP_ID_BAROMETER, app_main);

#endif /* APP_ID_BAROMETER */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
