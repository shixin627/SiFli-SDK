/**
 ******************************************************************************
 * @file   app_iot_gate.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
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
#include "common_widget.h"
#include "app_mainmenu.h"
#include "custom_trans_anim.h"
#include "bloc_setting.h"
#include "bloc_control.h"
#include "bloc_peripheral.h"
#include "bloc_motion_tracking.h"
#include "ui_handler.h"
#include "communicate_protocol.h"

#define DBG_TAG "app.iot_gate"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif

#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#include "ui_img_helper.h"
#endif

#ifdef APP_ID_IOT_GATE

// 使用media控制的圖標資源
LV_IMG_DECLARE(close_garage);    // 用於關閉鐵門
LV_IMG_DECLARE(img_media_pause); // 用於暫停
LV_IMG_DECLARE(open_garage);     // 用於開啟鐵門
LV_IMG_DECLARE(control_selection_bg);

typedef enum
{
    GATE_STATE_CLOSED = 0,
    GATE_STATE_PAUSED,
    GATE_STATE_OPENED
} gate_state_t;

//     turnOnLight1(0x01),  // 開燈1
//     turnOffLight1(0x02), // 關燈1
//     turnOnLight2(0x03),  // 開燈2
//     turnOffLight2(0x04), // 關燈2
//     turnOnFan(0x05),     // 開風扇
//     turnOffFan(0x06),    // 關風扇
//     turnOnAc(0x07),      // 開空調
//     turnOffAc(0x08),     // 關空調
//     openDoor(0x09),      // 開門
//     closeDoor(0x0A),     // 關門
//     customCommand(0xFF); // 自訂命令

typedef struct
{
    lv_obj_t *icon_btn_close;
    lv_obj_t *icon_btn_pause;
    lv_obj_t *icon_btn_open;
    lv_obj_t *gate_status_label;
    lv_obj_t *window;
    gate_state_t current_state;
} app_iot_gate_t;

typedef struct
{
    lv_obj_t *btn_close_bg;
    lv_obj_t *btn_close_img;
    lv_obj_t *btn_pause_bg;
    lv_obj_t *btn_pause_img;
    lv_obj_t *btn_open_bg;
    lv_obj_t *btn_open_img;
    lv_obj_t *btn_select_background;
} iot_gate_app_obj_t;

typedef struct
{
    lv_obj_t *widget_btn_close_bg;
    lv_obj_t *widget_btn_close_img;
    lv_obj_t *widget_btn_pause_bg;
    lv_obj_t *widget_btn_pause_img;
    lv_obj_t *widget_btn_open_bg;
    lv_obj_t *widget_btn_open_img;
    lv_obj_t *widget_select_background;
} iot_gate_widget_obj_t;

static app_iot_gate_t *p_app_iot_gate = NULL;
static app_iot_gate_t *p_widget_iot_gate = NULL;
static iot_gate_app_obj_t iot_gate_app_obj;
static iot_gate_widget_obj_t iot_gate_widget_obj;

static uint8_t button_selection_index = 1;
static uint8_t gate_selection_index = 1;
static bool gesture_open = false;

/* Declare the action functions for the buttons */
static void close_btn_event_cb(lv_event_t *e);
static void pause_btn_event_cb(lv_event_t *e);
static void open_btn_event_cb(lv_event_t *e);

static void update_gate_status_label(gate_state_t state)
{
    if (!p_app_iot_gate || !lv_obj_is_valid(p_app_iot_gate->gate_status_label))
        return;

    switch (state)
    {
    case GATE_STATE_CLOSED:
        lv_label_set_text(p_app_iot_gate->gate_status_label, "Gate Closed");
        break;
    case GATE_STATE_PAUSED:
        lv_label_set_text(p_app_iot_gate->gate_status_label, "Gate Paused");
        break;
    case GATE_STATE_OPENED:
        lv_label_set_text(p_app_iot_gate->gate_status_label, "Gate Opened");
        break;
    }
}

static void send_gate_command_to_client(uint8_t state)
{
    L1SendData data;
    data.event = L1SEND_MQTT_CONTROL;
    data.res.status = state;
    L1_send_event(data);
}

static void send_gate_command(gate_state_t state)
{
    // TODO: 實現與IOT鐵門的通信
    // 這裡可以使用BLE、WiFi或其他通信方式
    LOG_I("Send gate command: %d", state);
    if (state == GATE_STATE_OPENED)
    {
        send_gate_command_to_client(0x09); // 開門
        send_gate_command_to_client(0x01); // 開燈1
        send_gate_command_to_client(0x03); // 開燈2
    }
    else if (state == GATE_STATE_CLOSED)
    {
        send_gate_command_to_client(0x0A); // 關門
        send_gate_command_to_client(0x02); // 關燈1
        send_gate_command_to_client(0x04); // 關燈2
    }
    else if (state == GATE_STATE_PAUSED)
    {
    }

    if (p_app_iot_gate)
    {
        p_app_iot_gate->current_state = state;
        update_gate_status_label(state);
    }

    // 觸發震動反饋
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    watch_system_interact(INTERACT_MOTOR_VIBRATE_SCROLLING_APP, NULL);
#endif
}

static void close_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        LOG_D("close_btn_event_cb");
        send_gate_command(GATE_STATE_CLOSED);
    }
}

static void pause_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        LOG_D("pause_btn_event_cb");
        send_gate_command(GATE_STATE_PAUSED);
    }
}

static void open_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        LOG_D("open_btn_event_cb");
        send_gate_command(GATE_STATE_OPENED);
    }
}

#define WIDGET_ICON_ZOOM_SIZE 0.6

static lv_obj_t *iot_gate_app_ui_build(lv_obj_t *parent)
{
    uint16_t app_zoom = 256 * WIDGET_ICON_ZOOM_SIZE;
    if (!parent)
    {
        LOG_E("iot_gate_app_ui_build parent is NULL");
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

    // 創建選擇背景 (參考media app的樣式)
    iot_gate_app_obj.btn_select_background = lv_obj_create(p_window);
    lv_obj_set_size(iot_gate_app_obj.btn_select_background, 116, 80);
    lv_obj_set_style_radius(iot_gate_app_obj.btn_select_background, 20, 0);
    lv_obj_set_style_bg_color(iot_gate_app_obj.btn_select_background, lv_color_hex(0x737373), 0);
    lv_obj_set_style_bg_opa(iot_gate_app_obj.btn_select_background, LV_OPA_0, 0);
    lv_obj_align(iot_gate_app_obj.btn_select_background, LV_ALIGN_CENTER, 0, 30);
    lv_obj_set_style_border_width(iot_gate_app_obj.btn_select_background, 2, 0);
    lv_obj_set_style_border_color(iot_gate_app_obj.btn_select_background, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(iot_gate_app_obj.btn_select_background, LV_OPA_50, 0);

    // 創建開啟按鈕 (左側) - 參考media app的樣式
    iot_gate_app_obj.btn_open_bg = lv_btn_create(p_window);
    lv_obj_set_size(iot_gate_app_obj.btn_open_bg, 116, 80);
    lv_obj_set_style_radius(iot_gate_app_obj.btn_open_bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(iot_gate_app_obj.btn_open_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(iot_gate_app_obj.btn_open_bg, LV_ALIGN_LEFT_MID, 25, 30);
    lv_obj_set_style_bg_opa(iot_gate_app_obj.btn_open_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(iot_gate_app_obj.btn_open_bg, 2, 0);
    lv_obj_set_style_border_color(iot_gate_app_obj.btn_open_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(iot_gate_app_obj.btn_open_bg, LV_OPA_0, 0);
    lv_obj_t *btn_open = common_icon_button(iot_gate_app_obj.btn_open_bg, &open_garage, open_btn_event_cb);
    lv_obj_align(btn_open, LV_ALIGN_CENTER, 0, 0);
    iot_gate_app_obj.btn_open_img = lv_obj_get_child(btn_open, 0);
    lv_img_set_zoom(iot_gate_app_obj.btn_open_img, app_zoom);

    // 創建關閉按鈕 (右側) - 參考media app的樣式
    iot_gate_app_obj.btn_close_bg = lv_btn_create(p_window);
    lv_obj_set_size(iot_gate_app_obj.btn_close_bg, 116, 80);
    lv_obj_set_style_radius(iot_gate_app_obj.btn_close_bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(iot_gate_app_obj.btn_close_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(iot_gate_app_obj.btn_close_bg, LV_ALIGN_RIGHT_MID, -25, 30);
    lv_obj_set_style_bg_opa(iot_gate_app_obj.btn_close_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(iot_gate_app_obj.btn_close_bg, 2, 0);
    lv_obj_set_style_border_color(iot_gate_app_obj.btn_close_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(iot_gate_app_obj.btn_close_bg, LV_OPA_0, 0);
    lv_obj_t *btn_close = common_icon_button(iot_gate_app_obj.btn_close_bg, &close_garage, close_btn_event_cb);
    lv_obj_align(btn_close, LV_ALIGN_CENTER, 0, 0);
    iot_gate_app_obj.btn_close_img = lv_obj_get_child(btn_close, 0);
    lv_img_set_zoom(iot_gate_app_obj.btn_close_img, app_zoom);

    // 創建暫停按鈕 (中間) - 參考media app的樣式
    iot_gate_app_obj.btn_pause_bg = lv_btn_create(p_window);
    lv_obj_set_size(iot_gate_app_obj.btn_pause_bg, 116, 80);
    lv_obj_set_style_radius(iot_gate_app_obj.btn_pause_bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(iot_gate_app_obj.btn_pause_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(iot_gate_app_obj.btn_pause_bg, LV_OPA_0, 0);
    lv_obj_align(iot_gate_app_obj.btn_pause_bg, LV_ALIGN_CENTER, 0, 30);
    lv_obj_set_style_border_width(iot_gate_app_obj.btn_pause_bg, 2, 0);
    lv_obj_set_style_border_color(iot_gate_app_obj.btn_pause_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(iot_gate_app_obj.btn_pause_bg, LV_OPA_0, 0);
    p_app_iot_gate->icon_btn_pause = common_icon_button(iot_gate_app_obj.btn_pause_bg, &img_media_pause, pause_btn_event_cb);
    lv_obj_align(p_app_iot_gate->icon_btn_pause, LV_ALIGN_CENTER, 0, 0);
    iot_gate_app_obj.btn_pause_img = lv_obj_get_child(p_app_iot_gate->icon_btn_pause, 0);
    lv_img_set_zoom(iot_gate_app_obj.btn_pause_img, app_zoom);

    // 創建狀態標籤 (與widget一樣的樣式)
    p_app_iot_gate->gate_status_label = lv_label_create(p_window);
    lv_label_set_text(p_app_iot_gate->gate_status_label, "Gate Control");
    lv_obj_set_style_text_opa(p_app_iot_gate->gate_status_label, LV_OPA_70, 0);
    lv_obj_set_size(p_app_iot_gate->gate_status_label, LV_HOR_RES_MAX, 200);
    lv_label_set_long_mode(p_app_iot_gate->gate_status_label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(p_app_iot_gate->gate_status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(p_app_iot_gate->gate_status_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(p_app_iot_gate->gate_status_label, lv_color_white(), 0);
    lv_obj_align(p_app_iot_gate->gate_status_label, LV_ALIGN_TOP_MID, 0, 15);

    button_selection_index = 1;
    return p_window;
}

static lv_obj_t *widget_btn_close_bg = NULL;
static lv_obj_t *widget_btn_pause_bg = NULL;
static lv_obj_t *widget_btn_open_bg = NULL;
static lv_obj_t *widget_selection_bg = NULL;

void clear_iot_gate_widget(void)
{
    widget_selection_bg = NULL;
    widget_btn_close_bg = NULL;
    widget_btn_pause_bg = NULL;
    widget_btn_open_bg = NULL;
}

lv_obj_t *lv_iot_gate_widget_builder(lv_obj_t *parent)
{
    uint16_t widget_zoom = 256 * WIDGET_ICON_ZOOM_SIZE;
    if (!parent)
    {
        LOG_E("lv_iot_gate_widget_builder parent is NULL");
        return NULL;
    }
    lv_obj_t *widget = lv_obj_create(parent);
    lv_obj_set_size(widget, 410, 250);
    lv_obj_set_style_bg_color(widget, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(widget, LV_OPA_100, 0);
    lv_obj_align(widget, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(widget, 50, 0);
    lv_obj_center(widget);
    if (!widget)
        return NULL;
    widget_selection_bg = lv_obj_create(widget);
    lv_obj_set_size(widget_selection_bg, 80, 80);
    lv_obj_set_style_radius(widget_selection_bg, 20, 0);
    lv_obj_set_style_bg_color(widget_selection_bg, lv_color_hex(0x737373), 0);
    lv_obj_set_style_bg_opa(widget_selection_bg, LV_OPA_0, 0);
    lv_obj_align(widget_selection_bg, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_border_width(widget_selection_bg, 2, 0);
    lv_obj_set_style_border_color(widget_selection_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(widget_selection_bg, LV_OPA_50, 0);
    // 開啟按鈕 (左側)
    widget_btn_open_bg = lv_btn_create(widget);
    lv_obj_set_size(widget_btn_open_bg, 100, 100);
    lv_obj_set_style_radius(widget_btn_open_bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(widget_btn_open_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(widget_btn_open_bg, LV_ALIGN_BOTTOM_LEFT, 25, 0);
    lv_obj_set_style_bg_opa(widget_btn_open_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(widget_btn_open_bg, 2, 0);
    lv_obj_set_style_border_color(widget_btn_open_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(widget_btn_open_bg, LV_OPA_0, 0);
    lv_obj_t *btn_open = common_icon_button(widget_btn_open_bg, &open_garage, open_btn_event_cb);
    lv_obj_align(btn_open, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_zoom(lv_obj_get_child(btn_open, 0), widget_zoom);
    // 關閉按鈕 (右側)
    widget_btn_close_bg = lv_btn_create(widget);
    lv_obj_set_size(widget_btn_close_bg, 100, 100);
    lv_obj_set_style_radius(widget_btn_close_bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(widget_btn_close_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(widget_btn_close_bg, LV_ALIGN_BOTTOM_RIGHT, -25, 0);
    lv_obj_set_style_bg_opa(widget_btn_close_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(widget_btn_close_bg, 2, 0);
    lv_obj_set_style_border_color(widget_btn_close_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(widget_btn_close_bg, LV_OPA_0, 0);
    lv_obj_t *btn_close = common_icon_button(widget_btn_close_bg, &close_garage, close_btn_event_cb);
    lv_obj_align(btn_close, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_zoom(lv_obj_get_child(btn_close, 0), widget_zoom);
    // 暫停按鈕 (中間)
    widget_btn_pause_bg = lv_btn_create(widget);
    lv_obj_set_size(widget_btn_pause_bg, 100, 100);
    lv_obj_set_style_radius(widget_btn_pause_bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(widget_btn_pause_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(widget_btn_pause_bg, LV_OPA_0, 0);
    lv_obj_align(widget_btn_pause_bg, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_border_width(widget_btn_pause_bg, 2, 0);
    lv_obj_set_style_border_color(widget_btn_pause_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(widget_btn_pause_bg, LV_OPA_0, 0);
    p_widget_iot_gate->icon_btn_pause = common_icon_button(widget_btn_pause_bg, &img_media_pause, pause_btn_event_cb);
    lv_obj_align(p_widget_iot_gate->icon_btn_pause, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_zoom(lv_obj_get_child(p_widget_iot_gate->icon_btn_pause, 0), widget_zoom);
    // 狀態標籤
    p_widget_iot_gate->gate_status_label = lv_label_create(widget);
    lv_label_set_text(p_widget_iot_gate->gate_status_label, "Gate Control");
    lv_obj_set_style_text_opa(p_widget_iot_gate->gate_status_label, LV_OPA_70, 0);
    lv_obj_set_size(p_widget_iot_gate->gate_status_label, 410, 200);
    lv_label_set_long_mode(p_widget_iot_gate->gate_status_label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(p_widget_iot_gate->gate_status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(p_widget_iot_gate->gate_status_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(p_widget_iot_gate->gate_status_label, lv_color_white(), 0);
    lv_obj_align(p_widget_iot_gate->gate_status_label, LV_ALIGN_TOP_MID, 0, 15);
    button_selection_index = 1;

    return widget;
}

static lv_timer_t *iot_widget_bg_timer = NULL;
// 計時器回調函數，用於恢復透明度
static void iot_widget_bg_reset_cb(lv_timer_t *timer)
{
    if (widget_selection_bg)
    {
        lv_obj_set_style_bg_opa(widget_selection_bg, LV_OPA_0, 0);
    }
    lv_timer_del(iot_widget_bg_timer);
    iot_widget_bg_timer = NULL;
}

void reset_iot_widget_btn_bg(void)
{
    if (widget_selection_bg)
    {
        lv_obj_set_style_bg_opa(widget_selection_bg, LV_OPA_0, 0);
    }
}

static void iot_gate_widget_btn_press_cb(void)
{
    if (!widget_selection_bg)
        return;

    // 立即設置透明度為80
    lv_obj_set_style_bg_opa(widget_selection_bg, LV_OPA_80, 0);

    // 創建或重置計時器，300ms後恢復透明度
    if (!iot_widget_bg_timer)
    {
        iot_widget_bg_timer = lv_timer_create(iot_widget_bg_reset_cb, 300, NULL);
        lv_timer_set_repeat_count(iot_widget_bg_timer, 1);
    }
    else
    {
        lv_timer_reset(iot_widget_bg_timer);
    }
}

void animate_iot_widget_selection_bg(lv_coord_t move_offset, lv_coord_t prev_move_offset)
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
    lv_obj_align(widget_selection_bg, LV_ALIGN_BOTTOM_MID, prev_move_offset, -10);
}

static uint8_t prev_selection_index = 1;
static lv_coord_t prev_offset = 0;
static lv_coord_t prev_move_offset = 0;

void set_iot_widget_selection_bg_pos(uint8_t selection_index, lv_coord_t offset)
{
    if (widget_selection_bg == NULL || (prev_selection_index == selection_index && prev_offset == offset))
        return;

    int move_offset = 0;
    switch (selection_index)
    {
    case 0: // 開啟按鈕 (左側)
        move_offset = -140 + offset;
        if (prev_selection_index != 0)
        {
            animate_iot_widget_selection_bg(move_offset, prev_move_offset);
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
            watch_system_interact(INTERACT_MOTOR_VIBRATE_SCROLLING_APP, NULL);
#endif
        }
        else
            lv_obj_align(widget_selection_bg, LV_ALIGN_BOTTOM_MID, move_offset, -10);
        break;
    case 1: // 暫停按鈕 (中間)
        move_offset = offset;
        if (prev_selection_index != 1)
        {
            animate_iot_widget_selection_bg(move_offset, prev_move_offset);
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
            watch_system_interact(INTERACT_MOTOR_VIBRATE_SCROLLING_APP, NULL);
#endif
        }
        else
            lv_obj_align(widget_selection_bg, LV_ALIGN_BOTTOM_MID, move_offset, -10);
        break;
    case 2: // 關閉按鈕 (右側)
        move_offset = 140 + offset;
        if (prev_selection_index != 2)
        {
            animate_iot_widget_selection_bg(move_offset, prev_move_offset);
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
            watch_system_interact(INTERACT_MOTOR_VIBRATE_SCROLLING_APP, NULL);
#endif
        }
        else
            lv_obj_align(widget_selection_bg, LV_ALIGN_BOTTOM_MID, move_offset, -10);
        break;
    default:
        break;
    }
    prev_move_offset = move_offset;
    prev_selection_index = selection_index;
    prev_offset = offset;
}

/**
 * @brief 使用Y軸位置控制鐵門按鈕選擇
 * @param p_y Y軸位置 (0~466)
 *            0~155: 選擇關閉按鈕 (右側)
 *            155~311: 選擇暫停按鈕 (中間)
 *            311~466: 選擇開啟按鈕 (左側)
 */
void iot_gate_trigger_drag_by_py(int p_y)
{
    lv_coord_t diff = 0;
    if (p_y >= 0 && p_y < 155)
    {
        // 將 0~155 映射到 0~-12 (向右拖拽，選擇關閉按鈕)
        diff = -(12 * p_y) / 155;
        gate_selection_index = 2; // 關閉按鈕 (右側)
        set_paused_control_with_arm(false);
    }
    else if (p_y > 311 && p_y <= 466)
    {
        // 將 311~466 映射到 12~0 (向左拖拽，選擇開啟按鈕)
        diff = 12 - (12 * (p_y - 311)) / (466 - 311);
        gate_selection_index = 0; // 開啟按鈕 (左側)
        set_paused_control_with_arm(true);
    }
    else if (p_y >= 155 && p_y <= 311)
    {
        // 將 155~311 映射到 12~-12 (中間區域，選擇暫停按鈕)
        diff = 12 - (24 * (p_y - 155)) / (311 - 155);
        gate_selection_index = 1; // 暫停按鈕 (中間)
        set_paused_control_with_arm(true);
    }
    // LOG_D("p_y: %d, diff: %d, gate_selection_index: %d", p_y, diff, gate_selection_index);
    set_iot_widget_selection_bg_pos(gate_selection_index, diff);
}

void iot_gate_widget_tap_event_cb(void)
{
    iot_gate_widget_btn_press_cb();
    if (gate_selection_index == 0)
    {
        send_gate_command(GATE_STATE_OPENED);  // 開啟 (左側)
    }
    else if (gate_selection_index == 2)
    {
        send_gate_command(GATE_STATE_CLOSED);  // 關閉 (右側)
    }
    else if (gate_selection_index == 1)
    {
        send_gate_command(GATE_STATE_PAUSED);
    }
}

lv_obj_t *iot_gate_close_btn_create(lv_obj_t *parent)
{
    if (!parent)
    {
        LOG_E("iot_gate_close_btn_create parent is NULL");
        return NULL;
    }
    lv_obj_t *btn_close_bg = lv_img_create(parent);
    lv_obj_set_size(btn_close_bg, 110, 110);
    lv_img_set_src(btn_close_bg, &control_selection_bg);
    lv_obj_t *btn_close = common_icon_button(btn_close_bg, &close_garage, close_btn_event_cb);
    lv_img_set_zoom(lv_obj_get_child(btn_close, 0), 256 * WIDGET_ICON_ZOOM_SIZE);
    return btn_close_bg;
}

lv_obj_t *iot_gate_open_btn_create(lv_obj_t *parent)
{
    if (!parent)
    {
        LOG_E("iot_gate_open_btn_create parent is NULL");
        return NULL;
    }
    lv_obj_t *btn_open_bg = lv_img_create(parent);
    lv_obj_set_size(btn_open_bg, 110, 110);
    lv_img_set_src(btn_open_bg, &control_selection_bg);
    lv_obj_t *btn_open = common_icon_button(btn_open_bg, &open_garage, open_btn_event_cb);
    lv_img_set_zoom(lv_obj_get_child(btn_open, 0), 256 * WIDGET_ICON_ZOOM_SIZE);
    return btn_open_bg;
}

lv_obj_t *iot_gate_pause_btn_create(lv_obj_t *parent)
{
    if (!parent)
    {
        LOG_E("iot_gate_pause_btn_create parent is NULL");
        return NULL;
    }
    lv_obj_t *btn_pause_bg = lv_img_create(parent);
    lv_obj_set_size(btn_pause_bg, 110, 110);
    lv_img_set_src(btn_pause_bg, &control_selection_bg);
    lv_obj_t *btn_pause = common_icon_button(btn_pause_bg, &img_media_pause, pause_btn_event_cb);
    lv_img_set_zoom(lv_obj_get_child(btn_pause, 0), 256 * WIDGET_ICON_ZOOM_SIZE);
    return btn_pause_bg;
}

void reset_iot_gate_widget(void)
{
    lv_obj_set_style_border_opa(widget_btn_close_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_opa(widget_btn_pause_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_opa(widget_btn_open_bg, LV_OPA_0, 0);
}

void selection_iot_gate_widget(uint8_t index)
{
    switch (index)
    {
    case 0:
        lv_obj_set_style_border_opa(widget_btn_open_bg, LV_OPA_80, 0);  // 開啟 (左側)
        break;
    case 1:
        lv_obj_set_style_border_opa(widget_btn_pause_bg, LV_OPA_80, 0);  // 暫停 (中間)
        break;
    case 2:
        lv_obj_set_style_border_opa(widget_btn_close_bg, LV_OPA_80, 0);  // 關閉 (右側)
        break;
    default:
        break;
    }
}

static void reset_btn_state(void)
{
    lv_obj_set_style_border_opa(iot_gate_app_obj.btn_pause_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_opa(iot_gate_app_obj.btn_close_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_opa(iot_gate_app_obj.btn_open_bg, LV_OPA_0, 0);
}

static void animate_app_selection_bg(lv_coord_t move_offset, lv_coord_t prev_move_offset)
{
    if (iot_gate_app_obj.btn_select_background == NULL)
        return;

    // 設定動畫
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, iot_gate_app_obj.btn_select_background);
    lv_anim_set_values(&a, prev_move_offset, move_offset);
    lv_anim_set_time(&a, 75);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
    lv_anim_start(&a);

    // Y軸固定 (參考media app使用LV_ALIGN_CENTER)
    lv_obj_align(iot_gate_app_obj.btn_select_background, LV_ALIGN_CENTER, prev_move_offset, 30);
}

static uint8_t app_prev_selection_index = 1;
static lv_coord_t app_prev_offset = 0;
static lv_coord_t app_prev_move_offset = 0;

static void set_app_selection_bg_pos(uint8_t selection_index, lv_coord_t offset)
{
    if (iot_gate_app_obj.btn_select_background == NULL || (app_prev_selection_index == selection_index && app_prev_offset == offset))
        return;

    int move_offset = 0;
    switch (selection_index)
    {
    case 0: // 開啟按鈕 (左側)
        move_offset = -150 + offset;
        if (app_prev_selection_index != 0)
        {
            animate_app_selection_bg(move_offset, app_prev_move_offset);
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
            watch_system_interact(INTERACT_MOTOR_VIBRATE_SCROLLING_APP, NULL);
#endif
        }
        else
            lv_obj_align(iot_gate_app_obj.btn_select_background, LV_ALIGN_CENTER, move_offset, 30);
        break;
    case 1: // 暫停按鈕 (中間)
        move_offset = offset;
        if (app_prev_selection_index != 1)
        {
            animate_app_selection_bg(move_offset, app_prev_move_offset);
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
            watch_system_interact(INTERACT_MOTOR_VIBRATE_SCROLLING_APP, NULL);
#endif
        }
        else
            lv_obj_align(iot_gate_app_obj.btn_select_background, LV_ALIGN_CENTER, move_offset, 30);
        break;
    case 2: // 關閉按鈕 (右側)
        move_offset = 150 + offset;
        if (app_prev_selection_index != 2)
        {
            animate_app_selection_bg(move_offset, app_prev_move_offset);
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
            watch_system_interact(INTERACT_MOTOR_VIBRATE_SCROLLING_APP, NULL);
#endif
        }
        else
            lv_obj_align(iot_gate_app_obj.btn_select_background, LV_ALIGN_CENTER, move_offset, 30);
        break;
    default:
        break;
    }
    app_prev_move_offset = move_offset;
    app_prev_selection_index = selection_index;
    app_prev_offset = offset;
}

/**
 * @brief 使用Y軸位置控制鐵門按鈕選擇
 * @param gesture_position 手勢位置
 *        gesture_position_y 範圍 (0~466):
 *            0~155: 選擇關閉按鈕 (右側)
 *            155~311: 選擇暫停按鈕 (中間)
 *            311~466: 選擇開啟按鈕 (左側)
 */
static void button_selection(gesture_position_t gesture_position)
{
    if (peripheral_provider.get_tap_status())
        return;

    int p_y = gesture_position.gesture_position_y;
    lv_coord_t diff = 0;
    uint8_t category;

    // 根據Y軸位置判斷選擇哪個按鈕
    if (p_y >= 0 && p_y < 155)
    {
        // 將 0~155 映射到 0~-12 (向右拖拽，選擇關閉按鈕)
        diff = -(12 * p_y) / 155;
        category = 2; // 關閉按鈕 (右側)
    }
    else if (p_y > 311 && p_y <= 466)
    {
        // 將 311~466 映射到 12~0 (向左拖拽，選擇開啟按鈕)
        diff = 12 - (12 * (p_y - 311)) / (466 - 311);
        category = 0; // 開啟按鈕 (左側)
    }
    else if (p_y >= 155 && p_y <= 311)
    {
        // 將 155~311 映射到 12~-12 (中間區域，選擇暫停按鈕)
        diff = 12 - (24 * (p_y - 155)) / (311 - 155);
        category = 1; // 暫停按鈕 (中間)
    }
    else
    {
        category = 1; // 默認選擇暫停按鈕
    }

    button_selection_index = category;
    // LOG_D("p_y: %d, diff: %d, category: %d", p_y, diff, category);
    set_app_selection_bg_pos(category, diff);
}

static void clear_highlight_cb(void *param)
{
    lv_obj_set_style_bg_opa(iot_gate_app_obj.btn_pause_bg, LV_OPA_0, 0);
    lv_obj_set_style_bg_opa(iot_gate_app_obj.btn_close_bg, LV_OPA_0, 0);
    lv_obj_set_style_bg_opa(iot_gate_app_obj.btn_open_bg, LV_OPA_0, 0);
}

static rt_timer_t highlight_timer;

static void start_highlight_timer(void)
{
    if (!highlight_timer)
    {
        highlight_timer = rt_timer_create("app_iot_highlight_timer", clear_highlight_cb,
                                          RT_NULL, rt_tick_from_millisecond(300),
                                          RT_TIMER_FLAG_ONE_SHOT);
    }
    else
    {
        rt_timer_stop(highlight_timer);
    }
    rt_timer_start(highlight_timer);
}

static void handle_tap_event(void)
{
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    LOG_I("button_selection_index:%d", button_selection_index);

    switch (button_selection_index)
    {
    case 0:
        send_gate_command(GATE_STATE_OPENED);  // 開啟 (左側)
        lv_obj_set_style_bg_opa(iot_gate_app_obj.btn_open_bg, LV_OPA_30, 0);
        break;
    case 1:
        send_gate_command(GATE_STATE_PAUSED);  // 暫停 (中間)
        lv_obj_set_style_bg_opa(iot_gate_app_obj.btn_pause_bg, LV_OPA_30, 0);
        break;
    case 2:
        send_gate_command(GATE_STATE_CLOSED);  // 關閉 (右側)
        lv_obj_set_style_bg_opa(iot_gate_app_obj.btn_close_bg, LV_OPA_30, 0);
        break;
    default:
        break;
    }

    start_highlight_timer();
#endif
}

void iot_gate_widget_handle_tap_event(void)
{
    handle_tap_event();
}

void iot_gate_widget_handle_press_event(uint8_t press)
{
    LOG_D("iot_gate_widget_handle_press_event:%d", press);
    if (press == 1)
    {
        handle_tap_event();
    }
}

void iot_gate_widget_start(void)
{
    RT_ASSERT(NULL == p_widget_iot_gate);
    p_widget_iot_gate = (app_iot_gate_t *)rt_malloc(sizeof(app_iot_gate_t));
    memset(p_widget_iot_gate, 0, sizeof(app_iot_gate_t));
    LOG_D("iot_gate_widget_start");
}

void iot_gate_widget_stop(void)
{
    if (p_widget_iot_gate)
    {
        rt_free(p_widget_iot_gate);
        p_widget_iot_gate = NULL;
    }
    LOG_D("iot_gate_widget_stop");
}

void iot_gate_on_start(lv_obj_t *scr)
{
    RT_ASSERT(NULL == p_app_iot_gate);
    p_app_iot_gate = (app_iot_gate_t *)rt_malloc(sizeof(app_iot_gate_t));
    memset(p_app_iot_gate, 0, sizeof(app_iot_gate_t));
    p_app_iot_gate->current_state = GATE_STATE_PAUSED;
    iot_gate_app_ui_build(scr);
    set_open_control_options(true);
    lvgl_msg_handler.handle_widgets_control = button_selection;
    lvgl_msg_handler.handle_tap_indicator = iot_gate_widget_handle_press_event;
}

void iot_gate_on_resume(void)
{
    LOG_D("iot_gate_on_resume");
}

void iot_gate_on_pause(void)
{
    LOG_D("iot_gate_on_pause");
}

void iot_gate_on_stop(void)
{
    if (p_app_iot_gate)
    {
        rt_free(p_app_iot_gate);
        p_app_iot_gate = NULL;
    }
    LOG_D("iot_gate_on_stop");
    set_open_control_options(false);
    screen_rotate_back_to_original_direction();
    lvgl_msg_handler.handle_widgets_control = NULL;
    lvgl_msg_handler.handle_tap_indicator = NULL;
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        iot_gate_on_start(lv_scr_act());
        break;

    case GUI_APP_MSG_ONRESUME:
        iot_gate_on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        iot_gate_on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        iot_gate_on_stop();
        break;

    default:
        break;
    }
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_IOT_GATE, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(iot_gate), IMG_LOGO, APP_ID_IOT_GATE, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
