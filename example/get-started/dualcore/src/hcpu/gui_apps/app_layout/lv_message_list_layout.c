/**
 ******************************************************************************
 * @file   lv_message_list_layout.c
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

#include "lvgl.h"
#include "lv_ext_resource_manager.h"
#include "drv_touch.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include "bloc_control.h"
#include "lvgl/lvgl.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <rtthread.h>
#ifdef BSP_USING_BLOC_NOTIFY
    #include "bloc_notification.h"
    #include "bloc_motor.h"
    #include "bloc_motion_tracking.h"
#endif
#include "ui_helper.h"
#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
    #include "ui_img_helper.h"
#endif
#ifdef BSP_USING_GESTURE_HANDLER
    #include "gesture_handler.h"
#endif
#include "watch_global_data.h"
#include "watch_system_interact.h"
#include "watch_system_core_task.h"

#define DBG_TAG "app.notification"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define ENABLE_CURVE_LIST 1
#define LIST_MESSAGE_WIDTH (442)
#define LIST_MESSAGE_HEIGHT (252)
#define LIST_OPEN_INSTRUCTION_LIST_WIDTH (70)
#define LIST_OPEN_INSTRUCTION_LIST_HEIGHT (70)
#define LIST_MESSAGE_SPACING (40)
// #define APP_ID "message_list"
#define LIST_RADIUS (1000)

#define MESSAGE_NEED_MEDIA_WIDGET

typedef struct
{
    lv_obj_t *main_window;
    lv_obj_t *list;
    lv_obj_t *no_notifications_widget;
    lv_obj_t *media_widget;
    // lv_obj_t *app_list_btn;
} app_notification_t;
static app_notification_t *p_app_notification;

typedef struct notification_widget
{
    lv_obj_t *card;
    lv_obj_t *title;
    lv_obj_t *content;
    lv_obj_t *icon;
} notification_widget_t;

static const lv_style_const_prop_t LIST_MESSAGE_STYLE_PROPS[] = {
    LV_STYLE_CONST_WIDTH(LIST_MESSAGE_WIDTH),
    LV_STYLE_CONST_HEIGHT(LIST_MESSAGE_HEIGHT),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t LIST_MESSAGE_OPEN_INSTRUCTION_LIST[] = {
    LV_STYLE_CONST_WIDTH(LIST_OPEN_INSTRUCTION_LIST_WIDTH),
    LV_STYLE_CONST_HEIGHT(LIST_OPEN_INSTRUCTION_LIST_HEIGHT),
    LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(LIST_MESSAGE_STYLE, LIST_MESSAGE_STYLE_PROPS);
LV_STYLE_CONST_INIT(LIST_MESSAGE_OPEN_INSTRUCTION_LIST_STYLE,
                    LIST_MESSAGE_OPEN_INSTRUCTION_LIST);

LV_IMG_DECLARE(icon_apple_facetime);
LV_IMG_DECLARE(icon_apple_mail);
LV_IMG_DECLARE(icon_dingtalk);
LV_IMG_DECLARE(icon_facebook);
LV_IMG_DECLARE(icon_gmail);
LV_IMG_DECLARE(icon_google_calendar)
LV_IMG_DECLARE(icon_google_chat);
LV_IMG_DECLARE(icon_instagram);
LV_IMG_DECLARE(icon_kakaotalk);
LV_IMG_DECLARE(icon_line);
LV_IMG_DECLARE(icon_linkedin);
LV_IMG_DECLARE(icon_messenger);
// LV_IMG_DECLARE(icon_other);
LV_IMG_DECLARE(icon_qq);
LV_IMG_DECLARE(icon_skype);
LV_IMG_DECLARE(icon_sms);
LV_IMG_DECLARE(icon_snap);
LV_IMG_DECLARE(icon_twitter);
LV_IMG_DECLARE(icon_wechat);
LV_IMG_DECLARE(icon_whatsapp);
LV_IMG_DECLARE(icon_viber);
LV_IMG_DECLARE(icon_vk);
LV_IMG_DECLARE(icon_discord);
LV_IMG_DECLARE(icon_youtube);
LV_IMG_DECLARE(bluetooth_broadcasting);
LV_IMG_DECLARE(mouse_mode_icon);
LV_IMG_DECLARE(message_widget_bg);
LV_IMG_DECLARE(header_bg);

const char *const icon_list[NOTIFICATION_APP_QUANTITY] = {
    ICON_GOOGLE_CALENDAR,
    ICON_FACEBOOK,
    ICON_APPLE_FACETIME,
    ICON_INSTAGRAM,
    ICON_KAKAOTALK,
    ICON_LINE,
    ICON_LINKEDIN,
    ICON_APPLE_MAIL,
    ICON_MESSENGER,
    ICON_OTHER,
    ICON_QQ,
    ICON_SKYPE,
    ICON_SMS,
    ICON_SNAP,
    ICON_TWITTER,
    ICON_WECHAT,
    ICON_WHATSAPP,
    ICON_GMAIL,
    ICON_DINGTALK,
    ICON_GOOGLE_CHAT,
    ICON_DISCORD,
    ICON_YOUTUBE,
    ICON_TIKTOK,
    ICON_TELEGRAM,
    ICON_TWITCH,
    ICON_SLACK,
    ICON_LARK,
    ICON_REDDIT,
    ICON_SKAIWALK,
};

static bool dial_header_music_hidden_by_pause = false;
static lv_timer_t *dial_header_shrink_timer = NULL;
static bool dial_header_music_active = false;
static bool dial_header_showing_notification = false;

static bool open_shock = false;
static bool open_action_flag = true;
static bool left_hand_mode = true;
static bool need_correction = false;
static uint16_t selected_message_index = 0;
static uint16_t old_selected_message_index = -1;
static uint16_t lest_wiget_x_pos = 0;
static uint16_t message_index = 0;
static uint16_t notification_count = 0;
static lv_coord_t last_y_diff_on_selected = 0;
static rt_tick_t last_gohame_time = 0;

static uint8_t button_selection_index = 1;
static lv_obj_t *selected_message = NULL;
static lv_obj_t *page_up = NULL;
static lv_obj_t *page_down = NULL;

// 添加拖拽相關變數
static bool is_dragging = false;
static lv_coord_t drag_start_x = 0;
static lv_coord_t drag_current_x = 0;
static lv_obj_t *dragging_widget = NULL;
static lv_coord_t original_x = 0;
static lv_obj_t *message_page = NULL;
#define DRAG_THRESHOLD 10
#define MAX_DRAG_DISTANCE (LIST_MESSAGE_WIDTH / 6)

// 背景色塊相關變數（已移除）

// 拖拽計時相關變數
static lv_timer_t *drag_timer = NULL;
static bool drag_action_executed = false;
static bool touching_screen = false;
static bool have_media_widget = false;
#ifdef MESSAGE_NEED_MEDIA_WIDGET
static bool is_at_media_widget = false;
#endif
static void hide_background_blocks(void);
static notification_t *selection_notification = NULL;
static void scroll_list(lv_obj_t *obj, int16_t drift)
{
    uint16_t min_offset = LV_VER_RES;
    uint8_t child_cnt = obj->spec_attr->child_cnt;
    lv_coord_t y_diff = 0;
    lv_coord_t last_y_diff = 0;
    lv_coord_t first_y_diff = 0;
    lv_coord_t x_trans = 0;
    // LOG_D("child_cnt: %d", child_cnt);
    // 獲取當前活動的 tile
    if (message_page->coords.y1 == -466)
        need_correction = true;
    else
        need_correction = false;
    // 打印當前活動 tile 的行和列
    for (uint8_t i = 0; i < child_cnt; i++)
    {
        lv_obj_t *child = obj->spec_attr->children[i];

        // 跳過應用列表按鈕，不對其應用曲線變換
        // if (p_app_notification && child == p_app_notification->app_list_btn)
        // {
        // 	continue;
        // }

        // LOG_D("i:%d,child->coords.y1: %d, child->coords.y2: %d", i,
        // child->coords.y1, child->coords.y2);
        if (child->coords.y2 <= 0)
        {
            // continue;
        }
        else if (child->coords.y1 >= (lv_coord_t)LV_HOR_RES)
        {
            // break;
        }
        // LOG_D("i:%d,child->coords.x1: %d, child->coords.x2: %d", i,
        // child->coords.y1, child->coords.y2);
        lv_coord_t y_center =
            child->coords.y1 + LIST_MESSAGE_HEIGHT / 2 + drift;
        if (!need_correction)
        {
            y_diff = y_center - LV_VER_RES / 2;
        }
        else
        {
            y_diff = y_center - LV_VER_RES / 2 + 466;
        }
        if (i == notification_count - 1)
        {
            last_y_diff = y_diff;
            // LOG_D("last_y_diff: %d", last_y_diff);
        }
        if (i == 0)
        {
            first_y_diff = y_diff;
        }
        y_diff = LV_ABS(y_diff);
        lv_coord_t x_trans;

        if (y_diff >= LIST_RADIUS)
        {
#if ENABLE_CURVE_LIST
            if (left_hand_mode)
            {
                x_trans = 0;
            }
            else
            {
                x_trans = LIST_RADIUS;
            }
#endif
        }
        else
        {
            if (y_diff < min_offset)
            {
                min_offset = y_diff;
                if (i + 1 <= (notification_count + (have_media_widget ? 1 : 0)))
                {
                    selected_message_index = i;
                }
                // LOG_D("selected_message_index: %d", selected_message_index);
            }
#if ENABLE_CURVE_LIST
            {
                rt_uint32_t x_sqr = LIST_RADIUS * LIST_RADIUS - y_diff * y_diff;
                lv_sqrt_res_t res;
                lv_sqrt(x_sqr, &res, 0x8000);
                // if (i == notification_count - 1)
                // {
                // 	lest_wiget_x_pos = res.i;
                // }
                if (left_hand_mode)
                {
                    x_trans = res.i - 987;
                }
                else
                {
                    x_trans = LIST_RADIUS - res.i;
                }
            }
#endif
        }
#if ENABLE_CURVE_LIST
        lv_style_value_t *value =
            (lv_style_value_t *)child->styles->style->v_p.values_and_props;
        value->num = x_trans;
        lv_obj_mark_layout_as_dirty(child);
#endif
    }
    hide_background_blocks();

    if (selected_message_index >= notification_count - 1)
    {
        if (notification_count == 0)
            message_index = 0;
        else
            message_index = notification_count - 1;
    }
    if (touching_screen)
    {
        int target_value = get_total_moving_distance() +
                           (first_y_diff * get_total_moving_distance() /
                            (last_y_diff - first_y_diff));
        if (target_value < 0)
        {
            target_value = 0;
        }
        else if (target_value > get_total_moving_distance())
        {
            target_value = get_total_moving_distance();
        }
        set_prev_sensor_quat(target_value);
    }
    if (old_selected_message_index != selected_message_index)
    {
        selected_message = obj->spec_attr->children[selected_message_index];
        set_selected_object(obj->spec_attr->children[selected_message_index]);
        // LOG_D("message_OBJ: %p",
        // obj->spec_attr->children[selected_message_index]);
        LOG_D("selected_message_index: %d", selected_message_index);
        if ( open_shock)//!is_user_touching_screen() &&
        {
            motor_pattern_scrolling_app();
        }
        old_selected_message_index = selected_message_index;

        // if (selected_message_index == 0)
        // {
        // 	lv_obj_set_style_img_opa(page_up, LV_OPA_10, 0);
        // 	button_selection_index = 1;
        // }
        // else if (selected_message_index == notification_count - 1 &&
        // !have_media_widget)
        // {
        // 	lv_obj_set_style_img_opa(page_down, LV_OPA_10, 0);
        // 	button_selection_index = 1;
        // }
        // else if (selected_message_index == notification_count &&
        // have_media_widget)
        // {
        // 	lv_obj_set_style_img_opa(page_down, LV_OPA_10, 0);
        // 	button_selection_index = 1;
        // }
    }
#ifdef MESSAGE_NEED_MEDIA_WIDGET
    if (p_app_notification->media_widget != NULL)
    {

        lv_coord_t no_notif_y_center =
            p_app_notification->media_widget->coords.y1 +
            LIST_MESSAGE_HEIGHT / 2 + drift;
        lv_coord_t no_notif_y_diff;
        no_notif_y_diff = no_notif_y_center - LV_VER_RES / 2;
        if (need_correction)
        {
            no_notif_y_diff += 466;
        }
        if (!is_at_media_widget && no_notif_y_diff == 0)
        {
            is_at_media_widget = true;
            // lv_obj_set_style_img_opa(page_down, LV_OPA_10, 0);
            button_selection_index = 1;
        }
        else if (is_at_media_widget && no_notif_y_diff != 0)
        {
            is_at_media_widget = false;
        }
        // LOG_D("no_notif_y_diff: %d,is_at_media_widget:%d", no_notif_y_diff,
        // is_at_media_widget);
        if (no_notif_y_diff < -150 && rt_tick_get() - last_gohame_time > 500)
        {
            last_gohame_time = rt_tick_get();
            animate_to_home_from_instruction_list();
        }
    }
    else
#endif
    {
#ifdef MESSAGE_NEED_MEDIA_WIDGET
        if (is_at_media_widget)
        {
            is_at_media_widget = false;
        }
#endif
        if (is_at_message() && touching_screen)
        {
            if (notification_count > 0)
            {
                if (selected_message_index == notification_count - 1)
                {
                    if (last_y_diff < -150 && rt_tick_get() - last_gohame_time >
                                                  500) // last_y_diff < -100
                    {
                        last_gohame_time = rt_tick_get();
                        animate_to_home_from_instruction_list();
                    }
                    // LOG_D("last_y_diff: %d", last_y_diff);
                }
            }
            else
            {
                // 當沒有訊息時，檢測 no_notifications_widget 的位置
                if (p_app_notification->no_notifications_widget != NULL)
                {
                    lv_coord_t no_notif_y_center =
                        p_app_notification->no_notifications_widget->coords.y1 +
                        LIST_MESSAGE_HEIGHT / 2 + drift;
                    lv_coord_t no_notif_y_diff;
                    no_notif_y_diff = no_notif_y_center - LV_VER_RES / 2;

                    if (no_notif_y_diff < -150 &&
                        rt_tick_get() - last_gohame_time > 500)
                    {
                        last_gohame_time = rt_tick_get();
                        animate_to_home_from_instruction_list();
                    }
                    // LOG_D("no_notif_y_diff: %d", no_notif_y_diff);
                }
            }
        }
    }
    // if (!is_at_media_widget && notification_count > 0)
    if (notification_count > 0)
    {
        selection_notification =
            get_notification_in_reversed_ui(selected_message_index);
    }
    // Add this code to change the background color of the centered object
    for (uint8_t i = 0; i < child_cnt; i++)
    {
        lv_obj_t *child = obj->spec_attr->children[i];
        if (i == selected_message_index && button_selection_index == 1)
        {
            // Set the background color to blue
            // lv_obj_set_style_bg_color(child, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_radius(child, 80, LV_PART_MAIN);
            // lv_obj_set_style_bg_opa(child, LV_OPA_COVER, 0);
            // lv_obj_set_style_border_color(child, lv_color_hex(0xFFFFFF),
            // LV_PART_MAIN); lv_obj_set_style_border_width(child, 2,
            // LV_PART_MAIN); // 設置外框寬度為2
        }
        else
        {
            // Set the background color to original color
            // lv_obj_set_style_border_width(child, 1, LV_PART_MAIN); //
            // 設置外框寬度為0 lv_obj_set_style_bg_color(child,
            // lv_color_hex(0x000000), 0); lv_obj_set_style_bg_opa(child,
            // LV_OPA_COVER, 0);
        }
    }

    lv_obj_mark_layout_as_dirty(obj);
}

static uint8_t page_count = 0;
static void selected_message_widget_timer_callback(void *parameter)
{
    if (selected_message_index != 0 &&
        selected_message_index != (page_count - 1))
    {
        list_auto_positioning();
    }
}

static rt_timer_t selected_message_widget_timer = NULL;
static void selected_message_widget_timer_start(void)
{
    if (!selected_message_widget_timer)
    {
        selected_message_widget_timer =
            rt_timer_create("selected_message_widget_timer",
                            selected_message_widget_timer_callback, NULL, 250,
                            RT_TIMER_FLAG_ONE_SHOT);
    }
    else
    {
        rt_timer_stop(selected_message_widget_timer);
    }
    rt_timer_start(selected_message_widget_timer);
}

static void list_window_scroll_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = evt->target;
    // LOG_D("list_window_scroll_event_cb");
    if (obj == NULL)
    {
        return;
    }
    switch (evt->code)
    {
    case LV_EVENT_SCROLL_BEGIN:
        if (is_user_touching_screen())
        {
            if (!touching_screen)
            {
                touching_screen = true;
            }
        }
        break;
    case LV_EVENT_SCROLL:
        if (is_user_touching_screen())
        {
            if (!touching_screen)
            {
                touching_screen = true;
            }
        }
        scroll_list(obj, 0);
        break;

    case LV_EVENT_SCROLL_END:

        touching_screen = false;
        selected_message_widget_timer_start();
        break;
    default:
        break;
    }
}

static uint8_t top_align_space = 0;
static lv_obj_t *clear_all_btn;
static void clear_all_btn_event_cb(lv_event_t *evt)
{
    LOG_D("clear_all_btn_event_cb");
}

// 應用列表按鈕點擊事件回調
// static void app_list_btn_event_cb(lv_event_t *evt)
// {
// 	lv_event_code_t code = lv_event_get_code(evt);
// 	switch (code)
// 	{
// 	case LV_EVENT_PRESSED:
// 		lv_obj_set_style_bg_color(p_app_notification->app_list_btn,
// lv_color_hex(0x494949), LV_PART_MAIN); 		break;

// 	case LV_EVENT_CLICKED:
// 		gui_app_run(APP_ID_INSTRUCTION_LIST);
// 		break;

// 	case LV_EVENT_RELEASED:
// 		LOG_D("app_list_btn_event_cb - Opening app list");
// 		lv_obj_set_style_bg_color(p_app_notification->app_list_btn,
// lv_color_hex(0x676767), LV_PART_MAIN); 		break;

// 	default:
// 		break;
// 	}
// }

static uint16_t reset_count = 0;

static uint8_t message_scoll_target_item = 0;

static void reset_list(bool scroll_to_last)
{
    if (p_app_notification->list == NULL)
    {
        return;
    }
    open_shock = false;
    old_selected_message_index = -1;
    if (!is_at_message() || scroll_to_last)
    {
        if (have_media_widget && !dial_header_music_hidden_by_pause
            && !dial_header_showing_notification)
        {
            // 當有 media_widget 且 header 不在顯示通知時，滾動到 media_widget
            // 的位置 (notification_count 位置)
            reset_count = notification_count;
            message_scoll_target_item = reset_count;
            lv_obj_scroll_to_view(
                lv_obj_get_child(p_app_notification->list, reset_count),
                LV_ANIM_OFF);
        }
        else
        {
            if (notification_count > 0)
                reset_count = notification_count - 1;
            else
                reset_count = 0;
            message_scoll_target_item = reset_count;
            lv_obj_scroll_to_view(
                lv_obj_get_child(p_app_notification->list, reset_count),
                LV_ANIM_OFF);
        }
    }
    else
    {
        if (selected_message_index >= page_count)
        {
            selected_message_index = page_count - 1;
        }
        lv_obj_scroll_to_view(
            lv_obj_get_child(p_app_notification->list, selected_message_index),
            LV_ANIM_OFF);
    }
    scroll_list(p_app_notification->list, 0);
    lv_obj_update_layout(p_app_notification->list);
    open_shock = true;
}

static void list_message_click_cb(notification_t *notification)
{
    extern void app_message_set_open_from_message_list(bool open);
    app_message_set_open_from_message_list(true);
    animate_to_home_from_notification_center();
    navigate_notification_info(notification);
}

extern void app_message_set_from_temp(notification_t *notification);
static void list_message_reply(notification_t *notification)
{
    if (notification->can_reply)
    {
        app_message_set_from_temp(notification);
        notify_provider.navigate_to_reply(notification);
    }
}

static void list_message_click_event_cb(lv_event_t *evt)
{
    void *dat = lv_event_get_user_data(evt);
    notification_t *notification = (notification_t *)dat;
    list_message_click_cb(notification);
}

// 創建背景色塊
static void create_background_blocks(lv_obj_t *parent)
{
    // 背景色塊已移除，不再需要顯示拖拽背景
}

// 更新背景色塊位置和可見性（已移除背景色塊顯示）
static void update_background_blocks(lv_obj_t *obj, lv_coord_t diff)
{
}

// 隱藏背景色塊（已移除背景色塊）
static void hide_background_blocks(void)
{
}

static bool new_touching_obj = true;
extern void remove_notification_by_id(const char *id);
// 拖拽計時器回調函數
static void drag_timer_cb(lv_timer_t *timer)
{
    if (is_dragging && dragging_widget && selected_message->coords.y1 == 107)
    {
        // 執行刪除動作
        new_touching_obj = false;
        motor_pattern_scrolling_app();
        notification_t *notification =
            get_notification_in_reversed_ui(selected_message_index);
        remove_notification_by_id(notification->id);

        drag_action_executed = true;

        // 停止計時器
        if (drag_timer)
        {
            lv_timer_del(drag_timer);
            drag_timer = NULL;
        }
    }
}

// 開始拖拽計時器
static void start_drag_timer(void)
{
    // 清除舊的計時器
    if (drag_timer)
    {
        lv_timer_del(drag_timer);
        drag_timer = NULL;
    }
    // 創建新的計時器，200ms後執行刪除
    drag_timer = lv_timer_create(drag_timer_cb, 200, NULL);
    lv_timer_set_repeat_count(drag_timer, 1); // 只執行一次
    drag_action_executed = false;
}

// 停止拖拽計時器
static void stop_drag_timer(void)
{
    if (drag_timer)
    {
        lv_timer_del(drag_timer);
        drag_timer = NULL;
    }
    drag_action_executed = false;
}

// 拖拽動畫完成回調
static void drag_anim_ready_cb(lv_anim_t *a)
{
    is_dragging = false;
    dragging_widget = NULL;
    hide_background_blocks();
    stop_drag_timer(); // 確保停止計時器
}

// 回到原位的動畫
static void animate_to_original_position(lv_obj_t *obj, lv_coord_t target_x)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
    lv_anim_set_values(&a, lv_obj_get_x(obj), target_x);
    lv_anim_set_time(&a, 300);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a, drag_anim_ready_cb);
    lv_anim_start(&a);
}

static void set_pos_ready_cb(lv_anim_t *a)
{
    motor_pattern_scrolling_app();
}
static void set_pos_ready_cb_on_original(lv_anim_t *a)
{
    hide_background_blocks();
}
static void animate_to_position(lv_obj_t *obj, lv_coord_t target_x,
                                bool end_motor)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
    lv_anim_set_values(&a, lv_obj_get_x(obj), target_x);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_bounce);
    if (end_motor)
    {
        lv_anim_set_ready_cb(&a, set_pos_ready_cb);
    }
    else
    {
        lv_anim_set_ready_cb(&a, set_pos_ready_cb_on_original);
    }
    lv_anim_start(&a);
}

// 重力拖拽已移除，僅保留手指右拖拽
static uint8_t gesture_control = 1;

// 拖拽事件處理
static void widget_drag_event_cb(lv_event_t *evt)
{
    lv_event_code_t code = lv_event_get_code(evt);
    lv_obj_t *obj = lv_event_get_current_target(evt);
    lv_indev_t *indev = lv_indev_get_act();
    lv_point_t point;
    lv_indev_get_point(indev, &point);

    switch (code)
    {
    case LV_EVENT_PRESSED:
        if (!is_dragging)
        {
            drag_start_x = point.x;
            drag_current_x = point.x;
            dragging_widget = obj;
            original_x = lv_obj_get_x(obj);
            is_dragging = false; // 還未開始拖拽
            stop_drag_timer();   // 確保停止任何之前的計時器
            new_touching_obj = true;
        }
        break;

    case LV_EVENT_PRESSING:
        if (dragging_widget == obj && selected_message->coords.y1 == 107 &&
            new_touching_obj)
        {
            lv_coord_t diff = point.x - drag_start_x;

            // 只允許向右拖拽（正值），忽略向左拖拽
            if (diff < 0)
                diff = 0;

            // 檢查是否超過拖拽閾值
            if (!is_dragging && diff > DRAG_THRESHOLD)
            {
                is_dragging = true;

                if (diff >= MAX_DRAG_DISTANCE)
                {
                    start_drag_timer();
                }
            }

            if (is_dragging)
            {
                // 限制拖拽距離
                if (diff > MAX_DRAG_DISTANCE)
                {
                    diff = MAX_DRAG_DISTANCE;
                }

                // 達到觸發距離時啟動刪除計時器
                if (diff >= (MAX_DRAG_DISTANCE - 20) && !drag_timer &&
                    !drag_action_executed)
                {
                    start_drag_timer();
                }
                else if (diff < (MAX_DRAG_DISTANCE - 20))
                {
                    // 手指回縮未達觸發距離，停止計時器
                    stop_drag_timer();
                }

                lv_obj_set_x(obj, original_x + diff);
            }
        }
        else
        {
            stop_drag_timer(); // 停止計時器
        }
        break;

    case LV_EVENT_RELEASED:
        stop_drag_timer(); // 放開時停止計時器
        // LOG_D("coords.y1: %d", selected_message->coords.y1);
        if (dragging_widget == obj && is_dragging)
        {
            // 動畫回到原位
            animate_to_original_position(obj, original_x);
        }
        else if (dragging_widget == obj && !is_dragging &&
                 selected_message->coords.y1 == 107)
        {
            // 如果沒有拖拽，執行點擊事件
            void *dat = lv_event_get_user_data(evt);
            if (dat != NULL)
            {
                notification_t *notification = (notification_t *)dat;
                list_message_click_cb(notification);
            }
        }
        break;

    default:
        break;
    }
}

static notification_widget_t notification_widgets[ITEM_AMOUNT_NOTIFICATION];
lv_obj_t *notification_card_builder(lv_obj_t *list, uint8_t i)
{
    lv_coord_t x_offset = 0;
    lv_coord_t y_offset = (LIST_MESSAGE_HEIGHT + LIST_MESSAGE_SPACING) * i;
    lv_obj_t *message_widget = common_list_widget(
        list, (lv_style_t *)&LIST_MESSAGE_STYLE, x_offset, y_offset);
    lv_obj_add_flag(message_widget, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_radius(message_widget, 80, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(message_widget, LV_OPA_0, 0);
    lv_obj_set_style_border_width(message_widget, 0, LV_PART_MAIN);

    lv_obj_t *bg_img = lv_img_create(message_widget);
    lv_obj_align(bg_img, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_src(bg_img, &message_widget_bg);

    // 禁用此widget的滾動，讓父容器處理滾動
    lv_obj_clear_flag(message_widget, LV_OBJ_FLAG_SCROLLABLE);

    // 設置較高的 Z-index，確保在背景色塊之上
    lv_obj_move_foreground(message_widget);

    // 允許子物件超出邊界顯示（讓 icon 壓在 widget 上方）
    lv_obj_add_flag(message_widget, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    lv_obj_t *label = lv_label_create(message_widget);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    lv_obj_set_height(label, 40);
    lv_obj_set_width(label, 205);
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)),
                               0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10);
    notification_widgets[i].title = label;

    lv_obj_t *content = lv_label_create(message_widget);
    lv_label_set_long_mode(content, LV_LABEL_LONG_DOT);
    lv_obj_set_height(content, LIST_MESSAGE_HEIGHT - 90);
    lv_obj_set_width(content, LIST_MESSAGE_WIDTH - 65);
    lv_obj_set_style_text_font(content,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(content, lv_color_hex(0xB3B3B3), 0);
    lv_obj_align(content, LV_ALIGN_TOP_MID, 0, 65);
    lv_obj_clear_flag(message_widget, LV_OBJ_FLAG_SCROLLABLE);
    notification_widgets[i].content = content;

    // icon 最後創建，確保繪製在 widget 邊框之上
    lv_obj_t *icon_bg = lv_obj_create(message_widget);
    lv_obj_align(icon_bg, LV_ALIGN_TOP_LEFT, 35, -35);
    lv_obj_set_size(icon_bg, 90, 90);
    lv_obj_clear_flag(icon_bg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(icon_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(icon_bg, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_style_bg_opa(icon_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(icon_bg, 0, 0);
    lv_obj_t *icon = lv_img_create(icon_bg);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(icon, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(icon, LV_OBJ_FLAG_CLICKABLE);
    notification_widgets[i].icon = icon;

    return message_widget;
}

bool check_special_characters(const char *str, int i)
{
    if (strncmp(&str[i], "\xE2\x80\xAF", 3) == 0 ||
        strncmp(&str[i], "\xE2\x80\x89", 3) == 0 ||
        strncmp(&str[i], "\xE2\x80\x8E", 3) == 0 ||
        strncmp(&str[i], "\xE2\x80\xAA", 3) == 0 ||
        strncmp(&str[i], "\xE2\x80\xAC", 3) == 0)
    {
        return true;
    }
    return false;
}

char *replace_nbsp(const char *str)
{
    if (!str)
        return NULL;

    size_t len = strlen(str);
    char *result = (char *)lv_mem_alloc(len + 1);
    size_t j = 0;

    for (size_t i = 0; i < len; i++)
    {
        // 替換不間斷空格(0xA0)為一般空格(0x20)
        if (check_special_characters(str, i))
        {
            i += 2; // 跳過接下來的2個位元組(因為已經讀取了第一個)
            continue;
        }

        result[j++] = str[i];
    }
    result[j] = '\0';
    return result;
}

static void refresh_list(uint8_t new_item_count)
{
    for (uint8_t i = 0; i < ITEM_AMOUNT_NOTIFICATION; i++)
    {
        if (i < new_item_count)
        {
            notification_t *notification =
                get_notification(new_item_count - i - 1);
            LOG_D("notification->type: %s", notification->type);
            if (lv_obj_is_valid(notification_widgets[i].card))
            {
                lv_obj_clear_flag(notification_widgets[i].card,
                                  LV_OBJ_FLAG_HIDDEN);

                // 清除舊的事件處理器
                lv_obj_remove_event_cb(notification_widgets[i].card,
                                       list_message_click_event_cb);
                lv_obj_remove_event_cb(notification_widgets[i].card,
                                       widget_drag_event_cb);
                lv_obj_remove_event_cb(notification_widgets[i].card,
                                       widget_drag_event_cb);
                lv_obj_remove_event_cb(notification_widgets[i].card,
                                       widget_drag_event_cb);

                // 添加新的拖拽事件處理器
                lv_obj_add_event_cb(notification_widgets[i].card,
                                    widget_drag_event_cb, LV_EVENT_PRESSED,
                                    (void *)notification);
                lv_obj_add_event_cb(notification_widgets[i].card,
                                    widget_drag_event_cb, LV_EVENT_PRESSING,
                                    (void *)notification);
                lv_obj_add_event_cb(notification_widgets[i].card,
                                    widget_drag_event_cb, LV_EVENT_RELEASED,
                                    (void *)notification);
                char *clean_title = replace_nbsp(notification->title);
                lv_label_set_text(notification_widgets[i].title, clean_title);
                lv_mem_free(clean_title);
                char *clean_message = replace_nbsp(notification->message);
                lv_label_set_text(notification_widgets[i].content,
                                  clean_message);
                lv_mem_free(clean_message);
                if (notification->type > NOTIFICATION_APP_QUANTITY)
                {
                    LOG_W(
                        "Notification type %d exceeds max, using default icon",
                        notification->type);
                    notification->type = Notify_others; // 使用預設圖示
                }
                lv_img_set_src(notification_widgets[i].icon,
                               icon_list[notification->type]);
                lv_img_set_zoom(notification_widgets[i].icon,
                                226); // (90/100)*255
                if (i == new_item_count - 1)
                {
                    selected_message = notification_widgets[i].card;
                }
            }
            // lv_obj_align_to(notification_widgets[i].title,
            // notification_widgets[i].icon, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
            // lv_obj_align_to(notification_widgets[i].content,
            // notification_widgets[i].icon, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
        }
        else
        {
            lv_obj_add_flag(notification_widgets[i].card, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_event_cb(notification_widgets[i].card,
                                   list_message_click_event_cb);
            lv_obj_remove_event_cb(notification_widgets[i].card,
                                   widget_drag_event_cb);
            lv_obj_remove_event_cb(notification_widgets[i].card,
                                   widget_drag_event_cb);
            lv_obj_remove_event_cb(notification_widgets[i].card,
                                   widget_drag_event_cb);
        }
    }
    reset_list(false);
}

// static bool open_notification_select = false;
extern bool message_has_read;
extern char *get_media_title(void);
extern void refreh_notification_bar_indicator(uint16_t count);
extern void clear_media_widget(void);
static uint16_t message_gesture_starting_value = 0;
void refresh_notification_list(void *param)
{
    if (p_app_notification == NULL)
    {
        return;
    }

    // if (myLancher[app_index_message].pagetileview != NULL)
    // {
    // 	need_correction =
    // lv_obj_has_flag(myLancher[app_index_message].pagetileview,
    // LV_OBJ_FLAG_HIDDEN);
    // }
    // else
    // {
    // 	LOG_W("myLancher[app_index_message].pagetileview is NULL");
    // }
#ifdef MESSAGE_NEED_MEDIA_WIDGET
    const char *media_title = get_media_title();
    if (media_title && media_title[0] != '\0')
    {
        have_media_widget = true;
    }
    else
#endif
    {
        have_media_widget = false;
        lv_obj_del(p_app_notification->media_widget);
        p_app_notification->media_widget = NULL;
        clear_media_widget();
        extern void media_widget_stop(void);
        media_widget_stop();
    }

    notification_count = notification_center_get_info_count();
    page_count = notification_count + (have_media_widget ? 1 : 0);
    uint16_t page_range = 125;
    uint16_t total_height = page_count * page_range;
    message_gesture_starting_value = page_range / 2;
    if (is_at_message())
    {
        uint16_t refresh_message_gesture_starting_value = 0;
#ifdef MESSAGE_NEED_MEDIA_WIDGET
        if (is_at_media_widget)
        {
            // refresh_message_gesture_starting_value = total_height -
            // (message_gesture_starting_value +
            // (notification_count)*page_range);
            refresh_message_gesture_starting_value = 0;
        }
        else
#endif
        {
            refresh_message_gesture_starting_value =
                total_height - (message_gesture_starting_value +
                                selected_message_index * page_range);
        }
        set_scroll_segment_count(page_count);
        set_prev_sensor_quat(refresh_message_gesture_starting_value);
    }
    // if (notification_count < 1)
    // {
    // 	open_notification_select = false;
    // }
    // else
    // {
    // 	open_notification_select = true;
    // }

    // refreh_notification_bar_indicator(notification_count);
    if (notification_count > 0 || have_media_widget)
    {
        if (lv_obj_is_valid(p_app_notification->no_notifications_widget))
        {
            LOG_D("Deleted no_notifications_widget :%p",
                  p_app_notification->no_notifications_widget);
            lv_obj_clean(p_app_notification->no_notifications_widget);
            lv_obj_del(p_app_notification->no_notifications_widget);
            p_app_notification->no_notifications_widget = NULL;
        }
        else
        {
            LOG_W("no_notifications_widget is NULL");
        }
    }
    else if (!have_media_widget &&
             p_app_notification->no_notifications_widget == NULL)
    {
        p_app_notification->no_notifications_widget = common_list_widget(
            p_app_notification->list, (lv_style_t *)&LIST_MESSAGE_STYLE, 0, 0);
        lv_obj_set_style_radius(p_app_notification->no_notifications_widget, 80,
                                LV_PART_MAIN);
        LOG_D("Created no_notifications_widget :%p",
              p_app_notification->no_notifications_widget);
        lv_obj_set_style_bg_opa(p_app_notification->no_notifications_widget,
                                LV_OPA_0, 0);

        lv_obj_t *bg_img =
            lv_img_create(p_app_notification->no_notifications_widget);
        lv_obj_align(bg_img, LV_ALIGN_CENTER, 0, 0);
        lv_img_set_src(bg_img, &message_widget_bg);
        selected_message = p_app_notification->no_notifications_widget;
        lv_obj_t *label =
            lv_label_create(p_app_notification->no_notifications_widget);
        lv_obj_set_style_text_font(label,
                                   LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_label_set_text(label, "No notifications");
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    }
    if (p_app_notification->media_widget == NULL && have_media_widget)
    {
        extern lv_obj_t *lv_media_widget_builder(lv_obj_t * parent);
        extern void media_widget_start(void);
        media_widget_start();
        p_app_notification->media_widget = common_list_widget(
            p_app_notification->list, (lv_style_t *)&LIST_MESSAGE_STYLE, 0,
            (LIST_MESSAGE_HEIGHT + LIST_MESSAGE_SPACING) * notification_count);
        lv_media_widget_builder(p_app_notification->media_widget);
        lv_obj_set_size(p_app_notification->media_widget, LIST_MESSAGE_WIDTH,
                        LIST_MESSAGE_HEIGHT);
        lv_obj_set_style_radius(p_app_notification->media_widget, 80,
                                LV_PART_MAIN);
        lv_obj_set_style_bg_opa(p_app_notification->media_widget, LV_OPA_0, 0);
        lv_obj_set_style_border_width(p_app_notification->media_widget, 0,
                                      LV_PART_MAIN);
        lv_obj_set_style_bg_img_src(p_app_notification->media_widget,
                                    &message_widget_bg, 0);
        lv_obj_t *bg_img = lv_img_create(p_app_notification->media_widget);
        lv_obj_align(bg_img, LV_ALIGN_CENTER, 0, 0);
        lv_img_set_src(bg_img, &message_widget_bg);
    }

    if (p_app_notification->media_widget != NULL)
    {
        // if (notification_count == 0)
        // {
        // 	lv_obj_set_pos(p_app_notification->media_widget, 0,
        // (LIST_MESSAGE_HEIGHT + LIST_MESSAGE_SPACING));
        // }
        // else
        {
            lv_obj_set_pos(p_app_notification->media_widget, 0,
                           (LIST_MESSAGE_HEIGHT + LIST_MESSAGE_SPACING) *
                               notification_count);
        }
    }

    // 更新應用列表按鈕位置 - 放在最後一個項目下方
    // if (p_app_notification->app_list_btn != NULL)
    // {
    // 	lv_coord_t btn_y_pos;
    // 	if (have_media_widget)
    // 	{
    // 		// 如果有 media widget，按鈕放在 media widget 下方
    // 		btn_y_pos = (LIST_MESSAGE_HEIGHT + LIST_MESSAGE_SPACING) *
    // (notification_count + 1) + 20;
    // 	}
    // 	else if (notification_count > 0)
    // 	{
    // 		// 如果只有通知，按鈕放在最後一個通知下方
    // 		btn_y_pos = (LIST_MESSAGE_HEIGHT + LIST_MESSAGE_SPACING) *
    // notification_count + 20;
    // 	}
    // 	else
    // 	{
    // 		// 如果沒有任何內容，按鈕放在 no_notifications_widget 下方
    // 		btn_y_pos = LIST_MESSAGE_HEIGHT + LIST_MESSAGE_SPACING + 20;
    // 	}
    // 	lv_obj_set_pos(p_app_notification->app_list_btn, (LV_HOR_RES - 200) / 2,
    // btn_y_pos);
    // }

    if (lv_obj_is_valid(clear_all_btn))
    {
        if (notification_count > 0)
        {
            lv_obj_clear_flag(clear_all_btn, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(clear_all_btn, LV_OBJ_FLAG_HIDDEN);
        }
    }
    refresh_list(notification_count);
}

uint16_t get_message_gesture_starting_value(void)
{
    return message_gesture_starting_value;
}

uint8_t get_message_page_count(void)
{
    return page_count;
}

static void new_message_cb(void *param)
{
    message_has_read = false;
    /* 在 refresh_notification_list 更新 notification_count 之前，
       檢查是否為真正的新通知且音樂正在播放 */
    if (dial_header_music_active &&
        notification_center_get_info_count() > notification_count)
    {
        dial_header_showing_notification = true;
    }
    refresh_notification_list(param);
}

uint8_t get_gesture_control_state(void)
{
    return gesture_control;
}

// 事件轉發函數
static void event_forwarder(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        // 將事件轉發給目標物件
        lv_event_send(myLancher[app_index_message].pagetileview, code,
                      lv_event_get_param(e));
    }
}

static void reset_list_cb(void)
{
    reset_list(true);
}

static void on_tap(void);
static void button_selection(gesture_position_t gesture_position);
lv_obj_t *lv_message_list_layout_create(lv_obj_t *parent)
{
    RT_ASSERT(NULL == p_app_notification);
    p_app_notification =
        (app_notification_t *)lv_mem_alloc(sizeof(app_notification_t));
    memset(p_app_notification, 0, sizeof(app_notification_t));
    message_page = parent;

    lv_obj_t *notification_center = lv_obj_create(parent);
    lv_obj_set_style_bg_opa(notification_center, LV_OPA_TRANSP,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_size(notification_center, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_scrollbar_mode(notification_center, LV_SCROLLBAR_MODE_OFF);

    // 創建背景色塊
    create_background_blocks(notification_center);

    lv_obj_t *p_window = lv_obj_create(notification_center);
    lv_obj_set_style_bg_color(p_window, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(p_window, LV_OPA_0, 0);
    lv_obj_set_size(p_window, LV_HOR_RES, LV_VER_RES);

    p_app_notification->list = p_window;

    lv_obj_add_flag(p_window, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(p_window, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(p_window, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(p_window, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_style_pad_ver(p_window, LV_VER_RES / 2, 0);
    lv_obj_add_event_cb(p_window, list_window_scroll_event_cb, LV_EVENT_ALL,
                        NULL);

    /* Repopulate the list with the new number of items */
    for (uint8_t i = 0; i < 10; i++) // new_item_count
    {
        notification_widgets[i].card = notification_card_builder(p_window, i);
    }

    // lv_obj_t *down_grip_img = lv_obj_create(notification_center);
    // lv_obj_set_size(down_grip_img, 50, 10);
    // lv_obj_align(down_grip_img, LV_ALIGN_BOTTOM_MID, 0, 0); // 定位到底部中間
    // lv_obj_set_style_bg_color(down_grip_img, lv_color_hex(0x5B5B5B), 0);
    // lv_obj_t *down_grip = lv_obj_create(notification_center);
    // lv_obj_set_size(down_grip, 466, 40);
    // lv_obj_align(down_grip, LV_ALIGN_BOTTOM_MID, 0, 0); // 定位到底部中間
    // lv_obj_add_flag(down_grip, LV_OBJ_FLAG_SCROLLABLE);
    // lv_obj_add_event_cb(down_grip, event_forwarder, LV_OBJ_FLAG_SCROLLABLE,
    // p_window); lv_obj_set_style_opa(down_grip, LV_OPA_0, 0); page_up =
    // lv_img_create(notification_center); lv_obj_set_size(page_up, 40, 40);
    // lv_obj_align(page_up, LV_ALIGN_TOP_MID, 0, 60);
    // lv_img_set_src(page_up, UP_ARROW);
    // lv_obj_set_style_img_opa(page_up, LV_OPA_10, 0);

    // page_down = lv_img_create(notification_center);
    // lv_obj_set_size(page_down, 40, 40);
    // lv_obj_align(page_down, LV_ALIGN_BOTTOM_MID, 0, -60);
    // lv_img_set_src(page_down, DOWN_ARROW);
    // lv_obj_set_style_img_opa(page_down, LV_OPA_10, 0);

    // 創建應用列表按鈕 (添加到可滾動的列表容器中)
    // p_app_notification->app_list_btn = lv_btn_create(p_window);
    // lv_obj_set_size(p_app_notification->app_list_btn, 200, 50);
    // // 初始位置將在 refresh_notification_list 中設置
    // lv_obj_set_pos(p_app_notification->app_list_btn, (LV_HOR_RES - 200) / 2,
    // LIST_MESSAGE_HEIGHT + LIST_MESSAGE_SPACING + 20);
    // lv_obj_set_style_radius(p_app_notification->app_list_btn, 25,
    // LV_PART_MAIN);
    // lv_obj_set_style_bg_color(p_app_notification->app_list_btn,
    // lv_color_hex(0x676767), LV_PART_MAIN);
    // lv_obj_set_style_bg_opa(p_app_notification->app_list_btn, LV_OPA_COVER,
    // LV_PART_MAIN);
    // lv_obj_set_style_border_width(p_app_notification->app_list_btn, 1,
    // LV_PART_MAIN);
    // lv_obj_set_style_border_color(p_app_notification->app_list_btn,
    // lv_color_hex(0x808080), LV_PART_MAIN);

    // // 按鈕文字標籤
    // lv_obj_t *btn_label = lv_label_create(p_app_notification->app_list_btn);
    // lv_label_set_text(btn_label, "App List");
    // lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);
    // lv_obj_set_style_text_font(btn_label,
    // LV_EXT_FONT_GET(get_system_font_size(0)), 0); lv_obj_center(btn_label);

    // // 添加點擊事件
    // lv_obj_add_event_cb(p_app_notification->app_list_btn,
    // app_list_btn_event_cb, LV_EVENT_ALL, NULL);

#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_notification = new_message_cb;
#endif
    p_app_notification->main_window = p_window;
    refresh_notification_list(p_app_notification->main_window);
    lv_event_send(p_app_notification->main_window, LV_EVENT_SCROLL, NULL);
    myLancher[app_index_message].reset_list = reset_list_cb;
    myLancher[app_index_message].on_tap = on_tap;
    lvgl_msg_handler.handle_widgets_control = button_selection;

    return p_app_notification->main_window;
}

static void reset_button_selection(void)
{
    lv_obj_set_style_img_opa(page_up, LV_OPA_10, 0);
    lv_obj_set_style_img_opa(page_down, LV_OPA_10, 0);
    // lv_obj_set_style_bg_opa(selected_message, LV_OPA_40, LV_PART_MAIN);
    if (lv_obj_is_valid(selected_message))
    {
        lv_obj_set_style_border_width(selected_message, 1, LV_PART_MAIN);
    }
}

extern uint8_t get_gesture_control_state(void);
extern uint8_t get_media_widget_selection_index(void);
extern void media_widget_trigger_drag_by_py(int p_y);
static void button_selection(gesture_position_t gesture_position)
{
    const int p_y = gesture_position.gesture_position_y;
    if (abs(p_y - 233) < 50)
    {
        set_paused_control_with_arm(false);
    }
    else
    {
        set_paused_control_with_arm(true);
    }
// 重力拖拽已移除，不再透過 p_y 觸發左右拖拽動作
#ifdef MESSAGE_NEED_MEDIA_WIDGET
    if (is_at_media_widget)
    {
        media_widget_trigger_drag_by_py(p_y);
    }
#endif

    // else
    // {
    // 	LOG_D("msg_list_btn_sele:%d,%d,%d,%d", is_at_media_widget,
    // lv_obj_is_valid(selected_message), notification_center_get_info_count()
    // <= 0, is_at_media_widget);
    // }

    // if (page_count <= 0)
    // {
    // 	return;
    // }
    // const int p_x = gesture_position.gesture_position_x;

    // int category = 1;
    // if (get_gesture_control_state() != 1 ||
    // get_media_widget_selection_index() != 1)
    // {
    // 	category = 1;
    // }
    // else if (p_x > 300)
    // {
    // 	category = 2;
    // }
    // else if (p_x < 166)
    // {
    // 	category = 0;
    // }
    // else
    // {
    // 	category = 1;
    // }
    // // LOG_D("btn_selec:%d,%d,%d,%d,%d,%d,%d,", button_selection_index,
    // category, selected_message_index, notification_count, is_at_media_widget,
    // p_x, p_y); if (button_selection_index == category)
    // {
    // 	return;
    // }
    // if (((category == 0 && selected_message_index == 0) ||
    // 	 (category == 2 && selected_message_index == notification_count - 1)) &&
    // 	!have_media_widget)
    // {
    // 	return; // 如果已經在第一頁，則不允許向上滾動
    // }
    // else if (((category == 0 && selected_message_index == 0) ||
    // 		 (category == 2 && is_at_media_widget)) && have_media_widget)
    // {
    // 	return; // 如果已經在第一頁，則不允許向上滾動
    // }
    // button_selection_index = category;
    // motor_pattern_scrolling_app();
    // // LOG_D("button_selection_index: %d", button_selection_index);
    // reset_button_selection();
    // if (category == 0)
    // {
    // 	lv_obj_set_style_img_opa(page_up, LV_OPA_80, 0);
    // }
    // else if (category == 2)
    // {
    // 	lv_obj_set_style_img_opa(page_down, LV_OPA_80, 0);
    // }
    // else if (category == 1 && lv_obj_is_valid(selected_message))
    // {
    // 	// lv_obj_set_style_bg_opa(selected_message, LV_OPA_80, LV_PART_MAIN);
    // 	lv_obj_set_style_border_width(selected_message, 2, LV_PART_MAIN);
    // 	lv_obj_set_style_border_color(selected_message, lv_color_hex(0x4F4F4F),
    // LV_PART_MAIN);
    // }
}

int scroll_message_list(int argc, char **argv)
{
    if (argc < 1)
        return -1;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, p_app_notification->list);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_scroll_by);
    lv_anim_set_values(&a, 1200, 1200);
    lv_anim_set_time(&a, 200);
    lv_anim_start(&a);
    return 0;
}
MSH_CMD_EXPORT(scroll_message_list, reset list)

static rt_tick_t last_scroll_time = 0;
void scroll_message_list_to_index(int8_t page)
{
    if (p_app_notification == NULL || get_message_page_count() == 0)
    {
        return;
    }
    message_scoll_target_item = page;
    rt_tick_t now = rt_tick_get();
    if (now - last_scroll_time < 100)
    {
        set_scroll_anim_time(false, NULL);
    }
    LOG_D("scroll_message_list_to_index: %d,%d,%d", page, notification_count,
          have_media_widget);
    last_scroll_time = now;
    lv_obj_scroll_to_view(lv_obj_get_child(p_app_notification->list, page),
                          LV_ANIM_ON);
    set_scroll_anim_time(false, NULL);
    scroll_list(p_app_notification->list, 0);
    lv_obj_update_layout(p_app_notification->list);
}

void mesage_list_scroll_to_app(int8_t action)
{
    int target = 0;
    bool up = action == 1;
    if (up)
    {
        target = message_scoll_target_item - 1;
    }
    else
    {
        target = message_scoll_target_item + 1;
    }
    if (notification_count == 0)
    {
        return;
    }

    if (target >= 0 && target < notification_count && !have_media_widget)
    {
        scroll_message_list_to_index(target);
    }
    else if (have_media_widget && target >= 0 && target <= notification_count)
    {
        scroll_message_list_to_index(target);
    }
    else
    {
        LOG_W("Target index out of bounds: %d,%d,%d", target, have_media_widget,
              notification_count);
    }
}

extern void media_widget_tap_event_cb(void);
extern void media_widget_btn_press_cb(void);
static void on_tap(void)
{
    LOG_D("on_tap, button_selection_index: %d,%d,%d,%d", button_selection_index,
          is_at_media_widget, gesture_control,
          notification_center_get_info_count());

    if (button_selection_index == 0)
    {
        mesage_list_scroll_to_app(1); // 上一頁
    }
    else if (button_selection_index == 2)
    {
        mesage_list_scroll_to_app(0); // 下一頁
    }
// else
#ifdef MESSAGE_NEED_MEDIA_WIDGET
    else if (is_at_media_widget)
    {
        media_widget_tap_event_cb();
        return;
    }
    else
#endif
        if (gesture_control == 2)
    {
        LOG_D("message on_tap :control%d", gesture_control);
        notification_t *notification =
            get_notification_in_reversed_ui(selected_message_index);
        remove_notification_by_id(notification->id);
        gesture_control = 1;
        return; // 如果是拖拽動作，則不處理點擊
    }
#ifdef MESSAGE_NEED_MEDIA_WIDGET
    else if (notification_center_get_info_count() > 0 && !is_at_media_widget)
#else
    else if (notification_center_get_info_count() > 0)
#endif
    {
        LOG_D("message on_tap :click");
        notification_t *notification =
            get_notification_in_reversed_ui(selected_message_index);
        list_message_click_cb(notification);
        force_release_finger();
    }
    else
    {
        LOG_D("message on_tap :none");
    }
}

static void press_cb(uint8_t press)
{
    if (press)
    {
        on_tap();
    }
}

static bool have_message_now = false;
bool is_have_message_now(void)
{
    return have_message_now;
}
static void handle_dial_header_new_notification(void);
static notification_widget_t new_notification_widgets;
static void refresh_new_message_widget(void)
{
    notification_t *notification =
        get_notification(0); // notification_center_get_info_count()
    if (notification_center_get_info_count() <= 0)
    {
        lv_label_set_text(new_notification_widgets.title, "No notifications");
        lv_label_set_text(new_notification_widgets.content, "");
        lv_obj_add_flag(new_notification_widgets.icon, LV_OBJ_FLAG_HIDDEN);
        lv_img_set_src(new_notification_widgets.icon, ICON_OTHER);
        have_message_now = false;
    }
    else
    {
        char *clean_title = replace_nbsp(notification->title);
        lv_label_set_text(new_notification_widgets.title, clean_title);
        lv_mem_free(clean_title);
        char *clean_message = replace_nbsp(notification->message);
        lv_label_set_text(new_notification_widgets.content, clean_message);
        lv_mem_free(clean_message);
        lv_img_set_src(new_notification_widgets.icon,
                       icon_list[notification->type]);
        lv_obj_clear_flag(new_notification_widgets.icon, LV_OBJ_FLAG_HIDDEN);
        have_message_now = true;
    }
    /* handle_dial_header_new_notification is called directly by ui_handler
       via LVGL_MSG_TYPE_NOTIFICATION; it has its own count check to filter
       out refreshes that are not truly new notifications. */
}

lv_obj_t *lv_message_widget_builder(lv_obj_t *parent)
{
    lv_obj_t *widget = common_widget_container(parent);
    lv_obj_set_size(widget, 400, 230);
    lv_obj_set_style_bg_color(widget, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(widget, LV_OPA_70, 0);

    notification_t *notification =
        get_notification(notification_center_get_info_count() - 1);
    lv_obj_t *icon_bg = lv_obj_create(widget);
    lv_obj_align(icon_bg, LV_ALIGN_TOP_LEFT, 35, 20);
    lv_obj_set_size(icon_bg, 60, 60);
    lv_obj_clear_flag(icon_bg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(icon_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(icon_bg, LV_OPA_0, 0);
    lv_obj_t *icon = lv_img_create(icon_bg);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, 0);
    if (notification_center_get_info_count() <= 0)
    {
        lv_img_set_src(icon, ICON_OTHER);
        lv_obj_add_flag(icon, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_img_set_src(icon, icon_list[notification->type]);
    }
    lv_img_set_zoom(icon, 152); // (60/100)*255
    new_notification_widgets.icon = icon;

    lv_obj_t *label = lv_label_create(widget);
    lv_obj_set_height(label, 40);
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(-1)),
                               0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10);
    new_notification_widgets.title = label;
    if (notification_center_get_info_count() <= 0)
    {
        lv_label_set_text(label, "No notifications");
        have_message_now = false;
    }
    else
    {
        have_message_now = true;
        char *clean_title = replace_nbsp(notification->title);
        lv_label_set_text(label, clean_title);
        lv_mem_free(clean_title);
    }

    lv_obj_t *content = lv_label_create(widget);
    lv_label_set_long_mode(content, LV_LABEL_LONG_WRAP);
    lv_obj_set_height(content, 100);
    lv_obj_set_width(content, 300);
    lv_obj_set_style_text_font(content,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(content, lv_color_white(), 0);
    lv_obj_align_to(content, icon_bg, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    new_notification_widgets.content = content;
    if (notification_center_get_info_count() <= 0)
    {
        lv_label_set_text(content, "");
    }
    else
    {
        char *clean_message = replace_nbsp(notification->message);
        lv_label_set_text(content, clean_message);
        lv_mem_free(clean_message);
    }
    lv_obj_clear_flag(widget, LV_OBJ_FLAG_SCROLLABLE);

    return widget;
}

/*******************************************************************************
 * Dial Header: shows music or notification on clock face
 ******************************************************************************/
static lv_obj_t *dial_header_bg = NULL;
static lv_obj_t *dial_header_title = NULL;
static lv_obj_t *dial_header_img = NULL;
static lv_obj_t *dial_header_red_dot = NULL;
static lv_obj_t *dial_header_bg_mask = NULL;
static bool dial_header_was_music_before_notif = false;
static rt_timer_t dial_header_music_pause_timer = NULL;
#define MUSIC_PAUSE_HIDE_TIMEOUT_MS 30000
static void dial_header_music_pause_timer_stop(void);
static void dial_header_music_pause_timer_start(void);

static void dial_header_fadeout_ready_cb(lv_anim_t *anim)
{
    lv_obj_t *obj = (lv_obj_t *)anim->var;
    if (lv_obj_is_valid(obj))
    {
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
        // lv_obj_set_style_opa(obj, LV_OPA_COVER, 0);
        lv_obj_set_style_img_opa(dial_header_bg_mask, LV_OPA_COVER, 0);
        lv_obj_set_style_img_opa(dial_header_img, LV_OPA_COVER, 0);
        lv_obj_set_style_text_opa(dial_header_title, LV_OPA_COVER, 0);
    }
    if (lv_obj_is_valid(dial_header_red_dot) &&
        notification_center_get_info_count() > 0)
        lv_obj_clear_flag(dial_header_red_dot, LV_OBJ_FLAG_HIDDEN);
}

static void dial_header_fadeout_exec_cb(void *obj, int32_t value)
{
    // lv_obj_set_style_opa((lv_obj_t *)obj, value, 0);
    // LOG_D("dial_header_fadeout_exec_cb: %d", value);
    lv_obj_set_style_img_opa(dial_header_img, value, 0);
    lv_obj_set_style_img_opa(dial_header_bg_mask, value, 0);
    lv_obj_set_style_text_opa(dial_header_title, value, 0);
    uint8_t header_border_opa =
        (value * 30) /
        LV_OPA_COVER; // Border is half the opacity of the content
    lv_obj_set_style_border_opa(dial_header_bg, header_border_opa, 0);
    uint8_t header_bg_opa =
        (value * 15) /
        LV_OPA_COVER; // Background is one-third the opacity of the content
    lv_obj_set_style_bg_opa(dial_header_bg, header_bg_opa, 0);
}

static void dial_header_show_as_red_dot(void)
{
    if (lv_obj_is_valid(dial_header_bg))
    {
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, dial_header_bg);
        lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_TRANSP);
        lv_anim_set_exec_cb(&a, dial_header_fadeout_exec_cb);
        lv_anim_set_time(&a, 500);
        lv_anim_set_ready_cb(&a, dial_header_fadeout_ready_cb);
        lv_anim_start(&a);
    }
}

static void dial_header_restore_music(void)
{
    if (!lv_obj_is_valid(dial_header_bg))
        return;
    char *media_title = get_media_title();
    if (media_title && media_title[0] != '\0')
    {
        dial_header_music_active = true;
        /* Cancel any ongoing fadeout animation */
        lv_anim_del(dial_header_bg, dial_header_fadeout_exec_cb);
        lv_obj_set_style_img_opa(dial_header_img, LV_OPA_COVER, 0);
        lv_obj_set_style_img_opa(dial_header_bg_mask, LV_OPA_COVER, 0);
        lv_obj_set_style_text_opa(dial_header_title, LV_OPA_COVER, 0);
        lv_obj_clear_flag(dial_header_bg, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(dial_header_title, media_title);
        lv_img_set_src(dial_header_img, MEDIA_HEADER_IMG);
        lv_obj_set_size(dial_header_img, 50, 50);
        lv_img_set_zoom(dial_header_img, 256);
        lv_obj_align(dial_header_img, LV_ALIGN_CENTER, 0, 0);
        lv_obj_clear_flag(dial_header_img, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(dial_header_bg_mask, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        dial_header_music_active = false;
        dial_header_show_as_red_dot();
    }
}

static void dial_header_shrink_timer_cb(lv_timer_t *timer)
{
    dial_header_shrink_timer = NULL;
    dial_header_showing_notification = false;
    if (dial_header_was_music_before_notif || dial_header_music_active)
    {
        dial_header_was_music_before_notif = false;
        dial_header_restore_music();
        /* Header 恢復音樂後，重新定位列表到音樂 widget */
        if (p_app_notification && p_app_notification->list)
            reset_list(true);
    }
    else
    {
        dial_header_show_as_red_dot();
    }
    /* Timer auto-deletes after repeat_count reaches 0 */
}

extern void motor_pattern_notification(void);
static void dial_header_show_notification(void)
{
    if (!lv_obj_is_valid(dial_header_bg))
        return;
    /* Cancel any ongoing fadeout animation to prevent it from overriding the
     * opacity we set below */
    lv_anim_del(dial_header_bg, dial_header_fadeout_exec_cb);
    uint32_t count = notification_center_get_info_count();
    if (count > 0)
    {
        notification_t *notification = get_notification(0);
        LOG_D("Dial header show notification: %s", notification->title);
        if (notification)
        {
            motor_pattern_notification();
            /* Hide red dot when showing full header */
            if (lv_obj_is_valid(dial_header_red_dot))
                lv_obj_add_flag(dial_header_red_dot, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(dial_header_bg, LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(dial_header_title, notification->title);
            lv_img_set_src(dial_header_img, icon_list[notification->type]);
            lv_obj_set_size(dial_header_img, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_img_set_zoom(dial_header_img, 102);
            lv_obj_clear_flag(dial_header_img, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(dial_header_bg_mask, LV_OBJ_FLAG_HIDDEN);
            lv_obj_align(dial_header_img, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_style_img_opa(dial_header_img, LV_OPA_COVER, 0);
            lv_obj_set_style_img_opa(dial_header_bg_mask, LV_OPA_COVER, 0);
            lv_obj_set_style_text_opa(dial_header_title, LV_OPA_COVER, 0);
            lv_obj_set_style_border_opa(dial_header_bg, 30, 0);
            lv_obj_set_style_bg_opa(dial_header_bg, 15, 0);
            return;
        }
    }
    lv_obj_add_flag(dial_header_bg, LV_OBJ_FLAG_HIDDEN);
}

static void handle_dial_header_media_title(void *param)
{
    if (!lv_obj_is_valid(dial_header_title) || !lv_obj_is_valid(dial_header_bg))
        return;
    char *media_title_text = (char *)param;
    if (media_title_text && media_title_text[0] != '\0')
    {
        dial_header_music_active = true;
        /* Song changed or resumed — cancel pause timer and restore header */
        dial_header_music_pause_timer_stop();
        dial_header_music_hidden_by_pause = false;
        /* Hide red dot when music header is active */
        if (lv_obj_is_valid(dial_header_red_dot))
            lv_obj_add_flag(dial_header_red_dot, LV_OBJ_FLAG_HIDDEN);
        /* Only update header if not currently showing a notification with timer
         */
        if (!dial_header_shrink_timer)
        {
            /* Cancel any ongoing fadeout animation so it won't hide us */
            lv_anim_del(dial_header_bg, dial_header_fadeout_exec_cb);
            lv_obj_set_style_img_opa(dial_header_img, LV_OPA_COVER, 0);
            lv_obj_set_style_img_opa(dial_header_bg_mask, LV_OPA_COVER, 0);
            lv_obj_set_style_text_opa(dial_header_title, LV_OPA_COVER, 0);
            lv_obj_clear_flag(dial_header_bg, LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(dial_header_title, media_title_text);
        }
    }
    else
    {
        dial_header_music_active = false;
        if (!dial_header_shrink_timer)
        {
            dial_header_show_notification();
            /* Start 5-second timer to shrink to red dot */
            if (notification_center_get_info_count() > 0)
            {
                dial_header_was_music_before_notif = false;
                dial_header_shrink_timer =
                    lv_timer_create(dial_header_shrink_timer_cb, 10000, NULL);
                lv_timer_set_repeat_count(dial_header_shrink_timer, 1);
            }
        }
    }
}

static void handle_dial_header_media_img(void *param)
{
    if (!lv_obj_is_valid(dial_header_img))
        return;
    if (!dial_header_music_active)
        return;
    /* Don't update image while notification is displayed */
    if (dial_header_shrink_timer)
        return;
    char *img_data = (char *)param;
    if (img_data && img_data[0] != '\0')
    {
        lv_obj_clear_flag(dial_header_img, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(dial_header_bg_mask, LV_OBJ_FLAG_HIDDEN);
        lv_img_set_src(dial_header_img, MEDIA_HEADER_IMG);
        lv_obj_set_size(dial_header_img, 50, 50);
        lv_img_set_zoom(dial_header_img, 256);
        lv_obj_align(dial_header_img, LV_ALIGN_CENTER, 0, 0);
    }
    else
    {
        lv_obj_add_flag(dial_header_img, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(dial_header_bg_mask, LV_OBJ_FLAG_HIDDEN);
    }
}

static uint32_t dial_header_prev_notif_count = 0;
static void handle_dial_header_new_notification(void)
{
    if (!lv_obj_is_valid(dial_header_bg))
        return;
    /* Only react when notification count actually increased
       (skip refreshes triggered by music operations, etc.) */
    uint32_t current_count = notification_center_get_info_count();
    if (current_count <= dial_header_prev_notif_count)
    {
        dial_header_prev_notif_count = current_count;
        return;
    }
    dial_header_prev_notif_count = current_count;
    /* Remember if music was playing before this notification */
    dial_header_was_music_before_notif = dial_header_music_active;
    dial_header_showing_notification = true;
    /* Always show notification, even if music is active */
    dial_header_show_notification();
    /* Start or restart 5-second shrink timer */
    if (dial_header_shrink_timer)
        lv_timer_del(dial_header_shrink_timer);
    dial_header_shrink_timer =
        lv_timer_create(dial_header_shrink_timer_cb, 10000, NULL);
    lv_timer_set_repeat_count(dial_header_shrink_timer, 1);
}

void lv_dial_header_builder(lv_obj_t *parent)
{
    dial_header_bg = lv_obj_create(parent);
    lv_obj_set_size(dial_header_bg, 254, 100);
    lv_obj_set_style_bg_color(dial_header_bg, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(dial_header_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(dial_header_bg, 30, 0);
    // lv_obj_set_style_outline_width(dial_header_bg, 2, 0);
    // lv_obj_set_style_outline_color(dial_header_bg, lv_color_hex(0xFFFFFF),
    // 0); lv_obj_set_style_outline_opa(dial_header_bg, LV_OPA_20, 0);
    // lv_obj_set_style_border_width(dial_header_bg, 1, 0);
    // lv_obj_set_style_border_color(dial_header_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(dial_header_bg, LV_ALIGN_TOP_MID, 0, 2);
    lv_obj_add_flag(dial_header_bg, LV_OBJ_FLAG_HIDDEN);

    dial_header_bg_mask = lv_img_create(dial_header_bg);
    // lv_obj_set_size(dial_header_bg_mask, 252, 55);
    lv_img_set_src(dial_header_bg_mask, &header_bg);
    // lv_obj_set_style_bg_color(dial_header_bg_mask, lv_color_hex(0xFFFFFF),
    // 0); lv_obj_set_style_bg_opa(dial_header_bg_mask, 15, 0);
    // lv_obj_set_style_border_width(dial_header_bg_mask, 1, 0);
    // lv_obj_set_style_border_color(dial_header_bg_mask,
    // lv_color_hex(0xFFFFFF),
    //                               0);
    // lv_obj_set_style_border_opa(dial_header_bg_mask, 30, 0);
    lv_obj_align(dial_header_bg_mask, LV_ALIGN_CENTER, 0, 15);
    // lv_obj_set_style_radius(dial_header_bg_mask, 30, 0);

    lv_obj_t *dial_header_img_bg = lv_obj_create(dial_header_bg);
    lv_obj_set_size(dial_header_img_bg, 40, 40);
    lv_obj_set_style_radius(dial_header_img_bg, 20, 0);
    lv_obj_set_style_clip_corner(dial_header_img_bg, true, 0);
    lv_obj_set_style_bg_opa(dial_header_img_bg, LV_OPA_TRANSP, 0);
    lv_obj_align(dial_header_img_bg, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_clear_flag(dial_header_img_bg, LV_OBJ_FLAG_SCROLLABLE);

    dial_header_img = lv_img_create(dial_header_img_bg);
    lv_img_set_src(dial_header_img, MEDIA_MASK);
    lv_obj_set_size(dial_header_img, 40, 40);
    lv_obj_set_style_radius(dial_header_img, 25, 0);
    lv_obj_align(dial_header_img, LV_ALIGN_CENTER, 0, 0);

    dial_header_title = lv_label_create(dial_header_bg);
    lv_obj_set_size(dial_header_title, 220, 50);
    lv_label_set_long_mode(dial_header_title, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_anim_speed(dial_header_title, 15, 0);
    lv_obj_set_style_text_align(dial_header_title, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(dial_header_title,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(dial_header_title, lv_color_white(), 0);
    lv_obj_set_style_text_opa(dial_header_title, LV_OPA_70, 0);
    lv_obj_align_to(dial_header_title, dial_header_img_bg,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 7);

    /* Create red dot indicator (shown after notification header shrinks) */
    dial_header_red_dot = lv_img_create(parent);
    // lv_obj_set_size(dial_header_red_dot, 20, 20);
    // lv_obj_set_style_bg_color(dial_header_red_dot, lv_color_hex(0xCC5252),
    //                           LV_PART_MAIN | LV_STATE_DEFAULT);
    // lv_obj_set_style_radius(dial_header_red_dot, 100,
    //                         LV_PART_MAIN | LV_STATE_DEFAULT);
    // lv_obj_set_style_border_width(dial_header_red_dot, 0, 0);
    lv_img_set_src(dial_header_red_dot, NOTIFICATION_IMG);
    lv_obj_align(dial_header_red_dot, LV_ALIGN_TOP_MID, 0, 5);
    lv_obj_add_flag(dial_header_red_dot, LV_OBJ_FLAG_HIDDEN);

    /* Show initial state based on current music / notification status */
    dial_header_music_active = false;
    dial_header_shrink_timer = NULL;
    dial_header_was_music_before_notif = false;
    dial_header_showing_notification = false;
    char *media_title = get_media_title();
    if (media_title && media_title[0] != '\0')
    {
        dial_header_music_active = true;
        lv_obj_clear_flag(dial_header_bg, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(dial_header_title, media_title);
        lv_img_set_src(dial_header_img, MEDIA_HEADER_IMG);
        lv_obj_clear_flag(dial_header_img, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(dial_header_bg_mask, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        /* If there are notifications but no music, show as red dot
           (notification header already displayed and shrunk) */
        if (notification_center_get_info_count() > 0)
            dial_header_show_as_red_dot();
    }
}

static bool dial_header_music_need_hidden_by_pause = false;
void dial_header_music_pause_cb(void)
{
    dial_header_music_need_hidden_by_pause = false;
    dial_header_music_hidden_by_pause = true;
    dial_header_music_active = false;

    /* Fade out the dial header (same animation as notification shrink) */
    dial_header_show_as_red_dot();

    /* Re-scroll message list to first notification instead of media widget */
    if (p_app_notification && p_app_notification->list &&
        notification_count > 0)
    {
        reset_list(true);
    }
    LOG_D("Music paused for 30s, hiding dial header");
}

static void dial_header_music_pause_timer_cb(void *parameter)
{
    LOG_D("dial_header_music_pause_cb: Music paused, starting timer to hide "
          "header");
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_DIAL_HEADER_TIMER;
    lvgl_send_msg(msg);
}

static rt_tick_t dial_header_music_pause_time = 0;
void wakeup_chack_music_pause(void)
{
    if (dial_header_music_need_hidden_by_pause &&
        rt_tick_get() - dial_header_music_pause_time >=
            rt_tick_from_millisecond(MUSIC_PAUSE_HIDE_TIMEOUT_MS))
    {
        dial_header_music_pause_cb();
    }
}

static void dial_header_music_pause_timer_start(void)
{
    dial_header_music_need_hidden_by_pause = true;
    dial_header_music_pause_time = rt_tick_get();
    if (!dial_header_music_pause_timer)
    {
        dial_header_music_pause_timer = rt_timer_create(
            "music_pause_hide", dial_header_music_pause_timer_cb, NULL,
            rt_tick_from_millisecond(MUSIC_PAUSE_HIDE_TIMEOUT_MS),
            RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    }
    else
    {
        rt_timer_stop(dial_header_music_pause_timer);
    }
    LOG_D("Starting music pause timer to hide dial header after %d ms",
          MUSIC_PAUSE_HIDE_TIMEOUT_MS);
    rt_timer_start(dial_header_music_pause_timer);
}

static void dial_header_music_pause_timer_stop(void)
{
    if (dial_header_music_pause_timer)
    {
        LOG_D("Stopping music pause timer");
        dial_header_music_need_hidden_by_pause = false;
        rt_timer_stop(dial_header_music_pause_timer);
        rt_timer_delete(dial_header_music_pause_timer);
        dial_header_music_pause_timer = NULL;
    }
}

static void handle_dial_header_media_play_state(void *param)
{
    bool playing = *(bool *)param;
    if (!playing)
    {
        /* Music paused — start 30s timer */
        dial_header_music_pause_timer_start();
    }
    else
    {
        /* Music resumed — cancel 30s timer and restore header if hidden */
        dial_header_music_pause_timer_stop();
        if (dial_header_music_hidden_by_pause)
        {
            dial_header_music_hidden_by_pause = false;
            dial_header_restore_music();
        }
    }
}

void dial_media_header_init(void)
{
    lvgl_msg_handler.handle_dial_media_header_title =
        handle_dial_header_media_title;
    lvgl_msg_handler.handle_dial_media_header_img =
        handle_dial_header_media_img;
    lvgl_msg_handler.handle_dial_header_new_notification =
        handle_dial_header_new_notification;
    lvgl_msg_handler.handle_dial_media_play_state =
        handle_dial_header_media_play_state;
}

void dial_media_header_deinit(void)
{
    if (dial_header_shrink_timer)
    {
        lv_timer_del(dial_header_shrink_timer);
        dial_header_shrink_timer = NULL;
    }
    // dial_header_music_pause_timer_stop();
    dial_header_music_hidden_by_pause = false;
    if (lvgl_msg_handler.handle_dial_media_header_title ==
        handle_dial_header_media_title)
        lvgl_msg_handler.handle_dial_media_header_title = NULL;
    if (lvgl_msg_handler.handle_dial_media_header_img ==
        handle_dial_header_media_img)
        lvgl_msg_handler.handle_dial_media_header_img = NULL;
    if (lvgl_msg_handler.handle_dial_header_new_notification ==
        handle_dial_header_new_notification)
        lvgl_msg_handler.handle_dial_header_new_notification = NULL;
    if (lvgl_msg_handler.handle_dial_media_play_state ==
        handle_dial_header_media_play_state)
        lvgl_msg_handler.handle_dial_media_play_state = NULL;
}

void message_widget_start(void)
{
    lvgl_msg_handler.handle_new_notification = refresh_new_message_widget;
}

void message_widget_stop(void)
{
    if (lvgl_msg_handler.handle_new_notification == refresh_new_message_widget)
        lvgl_msg_handler.handle_new_notification = NULL;
}

static rt_int32_t init(lv_obj_t *parent)
{
    lv_message_list_layout_create(parent);
    return RT_EOK;
}
rt_int32_t notification_on_pause(void)
{
    // set_media_control_threshold(3000);
    if (myLancher[app_index_message].reset_list != NULL)
    {
        myLancher[app_index_message].reset_list();
    }
    LOG_D("notification_on_pause");
    if (lvgl_msg_handler.handle_tap_indicator == press_cb)
    {
        lvgl_msg_handler.handle_tap_indicator = NULL;
    }
    set_free_control_with_arm(false);
    set_open_control_options(false);
    if (lvgl_msg_handler.handle_widgets_control == button_selection)
    {
        lvgl_msg_handler.handle_widgets_control = NULL;
    }
    if (lvgl_msg_handler.handle_nav_bar_control == scroll_message_list_to_index)
    {
        lvgl_msg_handler.handle_nav_bar_control = NULL;
    }
    return RT_EOK;
}

rt_int32_t notification_on_resume(void)
{
    // set_media_control_threshold(1000);
    // // myLancher[app_index_message].reset_list = reset_list;
    // // myLancher[app_index_message].on_tap = on_tap;
    // reset_control_pos();
    // if (get_need_open_gesture_control())
    {
        // switch_watch_motion_control_mode(true, false);
        set_open_control_options(true);
        // set_free_control_with_arm(true);
        set_paused_control_with_arm(false);
    }
    LOG_D("notification_on_resume");
    /* User has seen the notification list — hide the red dot on the dial */
    if (lv_obj_is_valid(dial_header_red_dot))
        lv_obj_add_flag(dial_header_red_dot, LV_OBJ_FLAG_HIDDEN);
    lvgl_msg_handler.handle_tap_indicator = press_cb;
    set_scroll_segment_count(get_message_page_count());
    set_control_gravity_x_range(0.6f, -0.6f, false);
    lvgl_msg_handler.handle_widgets_control = button_selection;
    lvgl_msg_handler.handle_nav_bar_control = scroll_message_list_to_index;
    return RT_EOK;
}

rt_int32_t notification_on_deinit(void)
{
    // 清理拖拽狀態
    is_dragging = false;
    dragging_widget = NULL;
    // 清理計時器
    stop_drag_timer();

    if (p_app_notification)
    {
        // lv_obj_del(p_app_notification->main_window);
        // lv_obj_del(p_app_notification->list);
        // lv_obj_del(p_app_notification->no_notifications_widget);
        lv_mem_free(p_app_notification);
        p_app_notification = NULL;
    }
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_notification = NULL;
#endif
    // if (p_app_media)
    // {
    // 	lv_mem_free(p_app_media);
    // 	p_app_media = NULL;
    // }
    LOG_D("notification_on_deinit");
    return RT_EOK;
}

#ifdef APP_ID_MESSAGE_LIST
static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        init(lv_scr_act());
        break;

    case GUI_APP_MSG_ONRESUME:
        notification_on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        notification_on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        notification_on_deinit();
        break;
    default:
        break;
    }
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_MESSAGE_LIST, msg_handler);

    return 0;
}
BUILTIN_APP_EXPORT(LV_EXT_STR_ID(notification), SKAIWALKICON,
                   APP_ID_MESSAGE_LIST, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/