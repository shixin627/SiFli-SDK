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
#include "arc_scroll.h"
#include "bloc_control.h"
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

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

/* Indicator dots — same visual style as instruction list */
#define DOT_SMOLL_PROPORTION (0.2)
#define DOT_BIG_PROPORTION (0.9)
#define DOT_BG_SIZE ((int)(100 * DOT_BIG_PROPORTION) + 2)
/* 縮放曲線指數：1.0 = 線性、2.0 = 平方（中央放大突出）、3.0 = 立方更陡峭。
 * 值越大，「中央 dot 顯著大、其他都很小」越明顯 */
#define DOT_ZOOM_EXPONENT 2.0f

#define MESSAGE_NEED_MEDIA_WIDGET

typedef struct
{
    lv_obj_t *main_window;
    lv_obj_t *list;
    lv_obj_t *no_notifications_widget;
    lv_obj_t *media_widget;
    // lv_obj_t *app_list_btn;
    arc_scroll_handle_t *arc_handle; // 共用右側弧形觸控滾動 instance
} app_notification_t;
static app_notification_t *p_app_notification;

typedef struct notification_widget
{
    lv_obj_t *card;
    lv_obj_t *title;
    lv_obj_t *content;
    lv_obj_t *icon;
} notification_widget_t;

static notification_widget_t notification_widgets[ITEM_AMOUNT_NOTIFICATION];

/* Show only the card matching selected_message_index, hide all other
   notification cards. Indicator dots are separate objects on the bg
   parent and remain visible regardless. */
static void update_notification_card_visibility(void);

/* arc-scroll detached / discrete 模式狀態 — 拖動時 arc 不動 list、由 drag_cb
 * 接管，到 page change 才 snap。完整定義在後面，scroll_list 要先 visible。
 * 用獨立 bool flag 而不是 input 的特殊值當「是否已初始化」 — elastic overshoot
 * 會讓 input 掉到負值（min_input - 50 = -13），不能再用 <0 當 sentinel */
static bool s_msg_arc_drag_active = false;
static bool s_msg_drag_initialized = false;
static int s_msg_drag_input = 0;
static int s_msg_drag_last_idx = -1;    /* 上一次中央的 dot idx */
static void message_arc_reset_drag_state(void);

/* drag_cb 在 page change 時呼叫到的 snap，定義在下方 */
void scroll_message_list_to_index(int8_t page);

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
    ICON_OTHER,
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

/* Bound-guard icon_list[] against an out-of-range notification type pushed
   from the phone. Valid indices are 0..NOTIFICATION_APP_QUANTITY-1; anything
   else falls back to the default icon instead of reading past the array. */
static const char *notif_icon_src(uint8_t type)
{
    if (type >= NOTIFICATION_APP_QUANTITY)
        type = Notify_others;
    return icon_list[type];
}

static bool dial_header_music_hidden_by_pause = false;
static lv_timer_t *dial_header_shrink_timer = NULL;
static bool dial_header_music_active = false;
static bool dial_header_showing_notification = false;
static rt_tick_t dial_header_shrink_start_tick = 0;
static uint32_t dial_header_shrink_duration_ms = 0;

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
#define MAX_DRAG_DISTANCE (LIST_MESSAGE_WIDTH / 2)
// 放開時若右滑距離超過此值才觸發刪除
#define DRAG_DELETE_THRESHOLD (MAX_DRAG_DISTANCE - 20)
// 右滑到此距離時 widget 完全透明，距離以線性比例淡出
#define DRAG_FADE_DISTANCE (300)
// 點擊判定：x 移動量 < 此值且按壓時間 < CLICK_MAX_DURATION_MS 才當成 tap
#define CLICK_CANCEL_MOVEMENT 5
#define CLICK_MAX_DURATION_MS 250

// 背景色塊相關變數（已移除）

static bool touching_screen = false;
static bool have_media_widget = false;
#ifdef MESSAGE_NEED_MEDIA_WIDGET
static bool is_at_media_widget = false;
#endif

/* 通知列表停在 media widget 時回 true — 供 bloc_motion_tracking 的
   if_watchface_visible gate 開例外（音樂控制允許任何姿態捏指）。
   is_at_message() 兜底：離開頁面後殘留的 is_at_media_widget 不得放行。 */
bool message_media_widget_focused(void)
{
#ifdef MESSAGE_NEED_MEDIA_WIDGET
    return is_at_media_widget && is_at_message();
#else
    return false;
#endif
}
static void hide_background_blocks(void);
static notification_t *selection_notification = NULL;

/* ---- Indicator dots (match instruction list style, per-notification +
   one extra dot for the media widget when present) ---- */
#define MAX_MSG_DOTS (ITEM_AMOUNT_NOTIFICATION + 1) /* +1 for media widget */
static lv_obj_t *msg_indicator_dots[MAX_MSG_DOTS] = {0};
static lv_obj_t *msg_indicator_dots_bg[MAX_MSG_DOTS] = {0};
static lv_obj_t *msg_indicator_dots_parent = NULL;
static uint16_t msg_last_zoom[MAX_MSG_DOTS] = {0};

/* Total dot count = notifications + media-widget (if active). */
static int msg_dot_total(void)
{
    int n = (int)notification_count;
    if (have_media_widget)
        n += 1;
    if (n > MAX_MSG_DOTS)
        n = MAX_MSG_DOTS;
    return n;
}

static void update_msg_indicator_dots_position(int input_value);
static void create_msg_indicator_dots(lv_obj_t *parent);
static void destroy_msg_indicator_dots(void);

static void update_msg_indicator_dots_position(int input_value)
{
    int total_dots = msg_dot_total();
    if (total_dots <= 0)
        return;

    /* 跟 app_exercise.c::apply_circular_layout 對齊：圓心在螢幕正中、
     * radius=200。原本 (120, 233, 300) 那組會讓 arc 中心偏左、半徑大、
     * dots 上下散開比較廣，視覺上跟 exercise 不一樣。
     * center_x 往左偏 30 px，避免中央 dot zoom 後右邊跑出螢幕 */
    const int circle_radius = 200;
    const int center_x = LV_HOR_RES / 2 - 20;
    const int center_y = LV_VER_RES / 2;
    const float angle_per_dot = 36.0f; /* 跟 app_exercise.c 的 ICON_SLOT_ANGLE_DEG=36 對齊 */
    float base_input = 63.0f;
    float degrees_per_200_input = angle_per_dot;
    float total_input_range = 100.0f * (float)total_dots;

    float offset_angle =
        ((total_input_range - (float)input_value) - base_input) /
        (float)(total_input_range / total_dots) * degrees_per_200_input;

    for (int i = 0; i < total_dots; i++)
    {
        if (msg_indicator_dots[i] == NULL)
            continue;

        float base_angle = (float)i * angle_per_dot;
        /* 不做 [0,360) normalize — 留 signed angle，用 |angle| > 90 一刀過濾掉
         * 「list 第一格時 dot N-1 從另一邊 wrap 過來出現在上方」的問題。 */
        float current_angle = base_angle - offset_angle;

        if (current_angle < -90.0f || current_angle > 90.0f)
        {
            if (msg_indicator_dots_bg[i] != NULL &&
                !lv_obj_has_flag(msg_indicator_dots_bg[i], LV_OBJ_FLAG_HIDDEN))
            {
                lv_obj_add_flag(msg_indicator_dots_bg[i], LV_OBJ_FLAG_HIDDEN);
            }
            continue;
        }
        /* 進到右半圓 → unhide。media widget dot 例外（建立時就 hide 不該動）*/
        if (msg_indicator_dots_bg[i] != NULL &&
            lv_obj_has_flag(msg_indicator_dots_bg[i], LV_OBJ_FLAG_HIDDEN) &&
            i < (int)notification_count)
        {
            lv_obj_clear_flag(msg_indicator_dots_bg[i], LV_OBJ_FLAG_HIDDEN);
        }

        float angle_rad = current_angle * (float)M_PI / 180.0f;
        int dot_x = center_x + (int)((float)circle_radius * cosf(angle_rad));
        int dot_y = center_y + (int)((float)circle_radius * sinf(angle_rad));

        static int last_valid_dot_x[MAX_MSG_DOTS] = {0};
        if (dot_y > 450 || dot_y < 16)
        {
            if (msg_indicator_dots_bg[i] != NULL)
                dot_x = last_valid_dot_x[i];
        }
        else
        {
            last_valid_dot_x[i] = dot_x;
        }

        /* 跟 app_exercise.c::apply_circular_layout 一致：用 cos(abs_angle) 做
         * 平滑漸層。current_angle 現在是 signed [-90,90]，直接 fabsf */
        float abs_angle_deg = fabsf(current_angle);
        float ratio = cosf(abs_angle_deg * (float)M_PI / 180.0f);

        int dot_size = DOT_BG_SIZE;
        int opacity = (int)(LV_OPA_30 + (LV_OPA_COVER - LV_OPA_30) * ratio);
        if (opacity < LV_OPA_30)
            opacity = LV_OPA_30;
        if (opacity > LV_OPA_COVER)
            opacity = LV_OPA_COVER;

        /* Hide the dot whose index matches the currently selected card.
           Dot i corresponds to display index i (same ordering as the
           notification cards / media widget). */
        // if (i == (int)selected_message_index)
        //     opacity = LV_OPA_TRANSP;

        lv_obj_set_style_img_opa(msg_indicator_dots[i], opacity, 0);

        /* 用指數曲線 ratio^N 取代線性：N>1 時中央放大、邊緣縮小都加劇 */
        float zoom_ratio = powf(ratio, DOT_ZOOM_EXPONENT);
        uint16_t zoom =
            (uint16_t)(255 *
                       (DOT_SMOLL_PROPORTION +
                        (DOT_BIG_PROPORTION - DOT_SMOLL_PROPORTION) * zoom_ratio));
        if (abs((int)zoom - (int)msg_last_zoom[i]) > 5)
        {
            lv_img_set_zoom(msg_indicator_dots[i], zoom);
            msg_last_zoom[i] = zoom;
        }
        lv_obj_center(msg_indicator_dots[i]);
        dot_x -= (dot_size - 10) / 2;
        dot_y -= dot_size / 2;
        lv_obj_set_pos(msg_indicator_dots_bg[i], dot_x, dot_y);
    }
}

/* fwd decl — msg_dot_click_cb 直接 call 後面定義的 list_message_click_cb */
static void list_message_click_cb(notification_t *notification);

/* 仿 app_exercise.c：tap 任一可見 dot 直接觸發對應 notification，
 * user_data 是 visual idx（cast 成 void*），click 時才查 notification —
 * 因為 notification 列表會動，存 ptr 會 stale */
static void msg_dot_click_cb(lv_event_t *evt)
{
    if (lv_event_get_code(evt) != LV_EVENT_CLICKED) return;
    int idx = (int)(intptr_t)lv_event_get_user_data(evt);
    if (idx < 0 || idx >= (int)notification_count) return;
    notification_t *notif = get_notification_in_reversed_ui(idx);
    if (notif == NULL) return;
    list_message_click_cb(notif);
}

static void create_msg_indicator_dots(lv_obj_t *parent)
{
    if (parent == NULL)
        return;
    msg_indicator_dots_parent = parent;

    int total_dots = msg_dot_total();

    for (int i = 0; i < total_dots; i++)
    {
        lv_obj_t *dot_bg = lv_obj_create(parent);
        lv_obj_set_size(dot_bg, DOT_BG_SIZE, DOT_BG_SIZE);
        lv_obj_set_style_bg_opa(dot_bg, LV_OPA_0, 0);
        lv_obj_set_style_border_width(dot_bg, 0, 0);
        lv_obj_clear_flag(dot_bg, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *dot = lv_img_create(dot_bg);
        lv_obj_center(dot);

        if (i < (int)notification_count)
        {
            /* Notification dot — app icon matching the card at display
               index i (display 0 = newest). */
            notification_t *notif = get_notification_in_reversed_ui(i);
            uint8_t type = Notify_others;
            if (notif != NULL && notif->type < NOTIFICATION_APP_QUANTITY)
                type = notif->type;
            lv_img_set_src(dot, icon_list[type]);
            /* dot 直接可點：tap → 開該 notification 詳細頁 */
            lv_obj_add_flag(dot_bg, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(dot_bg, msg_dot_click_cb, LV_EVENT_CLICKED,
                                (void *)(intptr_t)i);
        }
        else
        {
            /* Media widget dot — fixed iTunes icon.
               Keep the dot object so total_dots / position math is unaffected,
               but hide it so the music indicator dot is not visible. */
            lv_img_set_src(dot, IMG_ITUNES);
            lv_obj_add_flag(dot_bg, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(dot_bg, LV_OBJ_FLAG_CLICKABLE);
        }

        msg_indicator_dots_bg[i] = dot_bg;
        msg_indicator_dots[i] = dot;
        msg_last_zoom[i] = 0;
    }

    update_msg_indicator_dots_position(37);
}

static void destroy_msg_indicator_dots(void)
{
    for (int i = 0; i < MAX_MSG_DOTS; i++)
    {
        if (msg_indicator_dots_bg[i] != NULL &&
            lv_obj_is_valid(msg_indicator_dots_bg[i]))
        {
            lv_obj_del(msg_indicator_dots_bg[i]);
        }
        msg_indicator_dots_bg[i] = NULL;
        msg_indicator_dots[i] = NULL;
        msg_last_zoom[i] = 0;
    }
}

/* Public entry used by bloc_motion_tracking via
   lvgl_msg_handler.handle_set_arc_stripe_external_offset — mirrors
   set_arc_stripe_external_offset() in lv_instruction_list_layout.c. */
void set_message_list_arc_stripe_external_offset(int16_t offset_degrees)
{
    update_msg_indicator_dots_position(offset_degrees);
}

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
                /* 只在 touch 路徑（motion_control_lock=TRUE）才用「最靠近中央
                 * 的卡」來推 selected_message_index — touch 模式下面接著會
                 * call update_msg_indicator_dots_position 用 dot 角度再覆蓋一次，
                 * 結果以 dot 為準。motion 路徑由外部 set_message_list_arc_stripe_external_offset
                 * → update_msg_indicator_dots_position 直接從 dot 角度設定
                 * selected_message_index，scroll_list 是被 lv_obj_scroll_to_view
                 * 動畫間接觸發的，這裡不能把已經設好的 index 推回 card-based
                 * 那組（不然動畫途中卡片會閃變）。
                 * 同樣地，arc-scroll drag_cb 模式（s_msg_arc_drag_active）下，
                 * selected_message_index 由 page-change snap 統一管理，scroll_list
                 * 也不要再從 card y_diff 推回去 */
                if (SkaiWatchSys.motion_control_lock &&
                    !s_msg_arc_drag_active &&
                    i + 1 <= (notification_count + (have_media_widget ? 1 : 0)))
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

        /* Indicator dots — same lock as instruction list:
           only user touch scrolling updates dots here; the motion control
           path updates them via handle_set_arc_stripe_external_offset.
           Scale first_y_diff so that 1 list item == 100 input units.
           Total item count includes the media widget when present. */
        int total_items_for_dots = msg_dot_total();
        /* arc-scroll drag_cb 模式下 dot 由 message_arc_drag_cb 用累積的 input
         * 自己更新，scroll_list 不要再用 list scroll_y 反推蓋過去（會讓 dot
         * 在 page snap 動畫中跳到 list 真實位置而不是手指對應位置）*/
        if (total_items_for_dots > 0 && SkaiWatchSys.motion_control_lock &&
            !s_msg_arc_drag_active)
        {
            const int pitch = LIST_MESSAGE_HEIGHT + LIST_MESSAGE_SPACING;
            int scaled_first = (int)first_y_diff * 100 / pitch;
            int dots_value = total_items_for_dots * 100 + scaled_first - 63;
            if (dots_value < 0)
                dots_value = 0;
            update_msg_indicator_dots_position(dots_value);
        }
    }
    if (old_selected_message_index != selected_message_index)
    {
        selected_message = obj->spec_attr->children[selected_message_index];
        // LOG_D("message_OBJ: %p",
        // obj->spec_attr->children[selected_message_index]);
        LOG_D("selected_message_index: %d", selected_message_index);
        /* Honor the global haptic-feedback toggle so the user setting
         * actually affects this page (matches lv_instruction_list_layout.c:982). */
        if (get_scrolling_motor_vibrate_status() && open_shock)
        {
            motor_pattern_scrolling_app();
        }
        old_selected_message_index = selected_message_index;
        /* Only the currently selected card is visible; indicator dots on
           the bg parent remain visible because they are not touched here. */
        update_notification_card_visibility();

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

/* Delay-reset touching_screen by 500ms, matching lv_instruction_list_layout.c.
   This keeps touching_screen=true during inertial scrolling after the finger
   lifts, so scroll_list keeps updating the indicator dots until inertia stops. */
static rt_timer_t touching_screen_timer = NULL;
static void touching_screen_timer_callback(void *parameter)
{
    touching_screen = false;
}

static void start_touching_screen_timer(void)
{
    if (!touching_screen_timer)
    {
        touching_screen_timer = rt_timer_create(
            "msg_touching_screen_timer", touching_screen_timer_callback, NULL,
            rt_tick_from_millisecond(500), RT_TIMER_FLAG_ONE_SHOT);
    }
    else
    {
        rt_timer_stop(touching_screen_timer);
    }
    rt_timer_start(touching_screen_timer);
}

static void stop_touching_screen_timer(void)
{
    if (touching_screen_timer)
    {
        rt_timer_stop(touching_screen_timer);
    }
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
    case LV_EVENT_PRESSED:
        /* 手指實際壓在中央列表上 — 右側 arc overlay 的 hit_test 在 band 外會回
           false，press 才會穿透到 p_window，所以收到這個事件在物理上就證明此刻
           沒有 arc 弧形拖動在進行。s_msg_arc_drag_active 的唯一清除路徑是 arc
           overlay 的 release callback；一旦某次拖動的 release 沒以 active 正常
           送達（拖動途中 animate_to_home 換頁、或 LV_ANIM 程式化捲動重設 indev
           等），旗標會卡在 true，永久 gate 掉這條用 touch-scroll 更新 indicator
           dots 的路徑（scroll_list 的 !s_msg_arc_drag_active 判斷）→ dots 凍結、
           翻頁中央卡片也不再跟著動。在這裡自癒清掉殘留旗標。 */
        if (s_msg_arc_drag_active)
        {
            message_arc_reset_drag_state();
        }
        break;
    case LV_EVENT_SCROLL_BEGIN:
        /* Cancel any pending delayed reset from a previous scroll. */
        stop_touching_screen_timer();
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
    {
        /* Only auto-position (which resets the motion-tracking reference
           via list_auto_positioning) for touch-driven scrolls. Motion-driven
           scrolls must NOT auto-position or the motion reference frame is
           corrupted, causing offset-value jumps (e.g. 310→349, 580→548). */
        bool was_touch_scroll = touching_screen;
        if (was_touch_scroll)
        {
            selected_message_widget_timer_start();
        }
        /* Delay touching_screen=false by 500ms when the user has already
           lifted — lets inertial scroll continue to update the indicator
           dots from scroll_list. Matches instruction list behavior. */
        if (!is_user_touching_screen())
        {
            start_touching_screen_timer();
        }
        break;
    }
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

    /* scroll_list's dots update is gated by touching_screen &&
       motion_control_lock which aren't true during a programmatic reset,
       so the dots would keep their old position. Recompute the input
       value from selected_message_index and apply it directly — matches
       how lv_instruction_list_layout.c reseats its dots after refresh.
       Total item count includes the media widget dot when present. */
    {
        int total_items_for_dots = msg_dot_total();
        if (total_items_for_dots > 0)
        {
            uint16_t sel = selected_message_index;
            if (sel >= (uint16_t)total_items_for_dots)
                sel = (uint16_t)(total_items_for_dots - 1);
            int dots_value = (total_items_for_dots - (int)sel) * 100 - 63;
            if (dots_value < 0)
                dots_value = 0;
            update_msg_indicator_dots_position(dots_value);
        }
    }

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
// 點擊判定：紀錄按壓時間 + 是否在按壓過程中有過任何 x 方向移動
static uint32_t press_start_tick = 0;
static bool press_had_movement = false;
extern void remove_notification_by_id(const char *id);
/* Forward decl — used by do_drag_delete to cancel any running drag anim. */
static void set_drag_translate_x(void *obj, int32_t value);
static void apply_drag_opa_to_children(lv_obj_t *node, lv_opa_t opa);
static void drag_opa_anim_exec_cb(void *obj, int32_t value);
static void start_fade_in(lv_obj_t *card);

// 放開後若右滑距離超過閾值，執行刪除動作
static void do_drag_delete(void)
{
    if (!is_dragging || !dragging_widget || selected_message->coords.y1 != 107)
        return;

    new_touching_obj = false;
    motor_pattern_scrolling_app();
    notification_t *notification =
        get_notification_in_reversed_ui(selected_message_index);

    /* Snapshot the card we were dragging before delete triggers a
       refresh_list — refresh_list can hide this card (selected index
       shifts) and LV_EVENT_RELEASED will then never fire on it, leaving
       is_dragging = true and dragging_widget stuck. Reset the card's
       translate_x and force-clear drag state here so the next press
       starts clean. */
    lv_obj_t *card_to_reset = dragging_widget;

    remove_notification_by_id(notification->id);

    if (card_to_reset && lv_obj_is_valid(card_to_reset))
    {
        lv_anim_del(card_to_reset, set_drag_translate_x);
        lv_obj_set_style_translate_x(card_to_reset, 0, 0);
        // refresh_list 可能把這張 card 重新綁定成下一筆通知並讓它顯示，
        // 直接 snap 回 LV_OPA_COVER 會看到「在原位瞬間出現」的閃爍。
        // 改成從透明 fade-in，看起來像下一張 card 自然浮現。
        start_fade_in(card_to_reset);
    }
    is_dragging = false;
    dragging_widget = NULL;
}

// 拖拽動畫完成回調
static void drag_anim_ready_cb(lv_anim_t *a)
{
    is_dragging = false;
    dragging_widget = NULL;
    hide_background_blocks();
}

/* 遞迴對 widget 內所有子物件（圖片/文字）逐個套用透明度。
   這個 LVGL build 上 lv_obj_set_style_opa 不能用，必須分別呼叫
   lv_obj_set_style_img_opa / text_opa 才會生效。
   注意：不去動 bg_opa——label 預設 bg_opa=0，一旦設成非零會把 theme
   的白色背景亮出來，造成「先變白再淡掉」的閃爍。 */
static void apply_drag_opa_to_children(lv_obj_t *node, lv_opa_t opa)
{
    if (!node || !lv_obj_is_valid(node))
        return;

    uint32_t cnt = lv_obj_get_child_cnt(node);
    for (uint32_t i = 0; i < cnt; i++)
    {
        lv_obj_t *child = lv_obj_get_child(node, i);
        if (!child || !lv_obj_is_valid(child))
            continue;

        lv_obj_set_style_img_opa(child, opa, 0);
        lv_obj_set_style_text_opa(child, opa, 0);

        apply_drag_opa_to_children(child, opa);
    }
}

/* Render-only x offset (doesn't change layout, so the parent list won't
   try to scroll to keep the moved child visible). 透明度也跟著右滑距離
   線性下降，這樣放開時的回彈動畫會把透明度一起補回 LV_OPA_COVER。 */
static void set_drag_translate_x(void *obj, int32_t value)
{
    lv_obj_t *o = (lv_obj_t *)obj;
    lv_obj_set_style_translate_x(o, (lv_coord_t)value, 0);

    lv_coord_t diff = (lv_coord_t)value - original_x;
    if (diff < 0)
        diff = 0;
    lv_opa_t opa;
    if (diff >= DRAG_FADE_DISTANCE)
    {
        opa = LV_OPA_TRANSP;
    }
    else
    {
        opa = (lv_opa_t)(LV_OPA_COVER -
                         (diff * LV_OPA_COVER / DRAG_FADE_DISTANCE));
    }
    apply_drag_opa_to_children(o, opa);
}

// 給 lv_anim 用的 opa 動畫 exec_cb：每幀把整顆 widget 子物件的 opa 設定成 value
static void drag_opa_anim_exec_cb(void *obj, int32_t value)
{
    apply_drag_opa_to_children((lv_obj_t *)obj, (lv_opa_t)value);
}

// 從完全透明 fade-in 回 LV_OPA_COVER，~200ms。do_drag_delete 後使用
// 讓重新綁定的下一筆通知不會在原位瞬間 pop in。
static void start_fade_in(lv_obj_t *card)
{
    if (!card || !lv_obj_is_valid(card))
        return;
    lv_anim_del(card, drag_opa_anim_exec_cb);
    apply_drag_opa_to_children(card, LV_OPA_TRANSP);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, card);
    lv_anim_set_exec_cb(&a, drag_opa_anim_exec_cb);
    lv_anim_set_values(&a, LV_OPA_TRANSP, LV_OPA_COVER);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_linear);
    lv_anim_start(&a);
}

// 回到原位的動畫
static void animate_to_original_position(lv_obj_t *obj, lv_coord_t target_x)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_exec_cb(&a, set_drag_translate_x);
    lv_anim_set_values(&a, lv_obj_get_style_translate_x(obj, 0), target_x);
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
    lv_anim_set_exec_cb(&a, set_drag_translate_x);
    lv_anim_set_values(&a, lv_obj_get_style_translate_x(obj, 0), target_x);
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

// 滑出畫面+淡出完成後才真的刪除，避免看到 card 跳回原位
static void drag_delete_anim_ready_cb(lv_anim_t *a)
{
    do_drag_delete();
}

// 滑出+淡出動畫：從目前位置一路滑到 DRAG_FADE_DISTANCE，set_drag_translate_x
// 會把 opa 同步帶到 LV_OPA_TRANSP；動畫完成才呼叫 do_drag_delete()
static void animate_to_delete(lv_obj_t *obj)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_exec_cb(&a, set_drag_translate_x);
    lv_coord_t cur = lv_obj_get_style_translate_x(obj, 0);
    lv_anim_set_values(&a, cur, original_x + DRAG_FADE_DISTANCE);
    lv_anim_set_time(&a, 180);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a, drag_delete_anim_ready_cb);
    lv_anim_start(&a);
}

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
            /* We drive drag via translate_x (render-only), not lv_obj_set_x,
               so the list doesn't auto-scroll to keep the moved child visible.
               Baseline translate_x is usually 0 at rest. */
            original_x = lv_obj_get_style_translate_x(obj, 0);
            is_dragging = false; // 還未開始拖拽
            new_touching_obj = true;
            // 點擊判定狀態
            press_start_tick = lv_tick_get();
            press_had_movement = false;
            // 殺掉前一輪可能還在跑的 fade-in 動畫，避免跟手勢的 opa 互打
            lv_anim_del(obj, drag_opa_anim_exec_cb);
            // 確保起始為完全不透明
            apply_drag_opa_to_children(obj, LV_OPA_COVER);
        }
        break;

    case LV_EVENT_PRESSING:
        // 不論方向，只要 x 移動量超過閾值就標記為「有移動」，後面 RELEASED 用來阻擋 click
        if (dragging_widget == obj)
        {
            lv_coord_t abs_dx = point.x >= drag_start_x
                                    ? point.x - drag_start_x
                                    : drag_start_x - point.x;
            if (abs_dx > CLICK_CANCEL_MOVEMENT)
                press_had_movement = true;
        }
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
            }

            if (is_dragging)
            {
                // 不再限制拖拽距離，完全跟著手指移動，並讓透明度跟著淡出
                drag_current_x = drag_start_x + diff;
                set_drag_translate_x(obj, original_x + diff);
            }
        }
        break;

    case LV_EVENT_RELEASED:
        // LOG_D("coords.y1: %d", selected_message->coords.y1);
        if (dragging_widget == obj && is_dragging)
        {
            // 以實際視覺位移作為放開時的右滑距離（不依賴最後一刻的 indev 點）
            lv_coord_t release_diff =
                lv_obj_get_style_translate_x(obj, 0) - original_x;
            LOG_D("release_diff: %d", release_diff);
            if (release_diff >= DRAG_DELETE_THRESHOLD &&
                selected_message->coords.y1 == 107)
            {
                // 距離足夠，先滑出+淡出再刪除（避免看到跳回原位的瞬間）
                animate_to_delete(obj);
            }
            else
            {
                // 動畫回到原位
                animate_to_original_position(obj, original_x);
            }
        }
        else if (dragging_widget == obj && !is_dragging &&
                 !press_had_movement &&
                 lv_tick_elaps(press_start_tick) < CLICK_MAX_DURATION_MS &&
                 selected_message->coords.y1 == 107)
        {
            // 沒拖拽、沒位移、按壓時間夠短 → 視為 tap，執行點擊事件
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

static void update_notification_card_visibility(void)
{
    // if (p_app_notification == NULL)
    //     return;
    // for (uint8_t i = 0; i < notification_count && i < ITEM_AMOUNT_NOTIFICATION;
    //      i++)
    // {
    //     if (notification_widgets[i].card == NULL ||
    //         !lv_obj_is_valid(notification_widgets[i].card))
    //         continue;
    //     if (i == selected_message_index)
    //         lv_obj_clear_flag(notification_widgets[i].card, LV_OBJ_FLAG_HIDDEN);
    //     else
    //         lv_obj_add_flag(notification_widgets[i].card, LV_OBJ_FLAG_HIDDEN);
    // }
}

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
    // 強制 bg 透明，避免拖拽時 helper 動到 opa 後 theme 預設白底冒出來
    lv_obj_set_style_bg_opa(bg_img, LV_OPA_TRANSP, 0);

    // 禁用此widget的滾動，讓父容器處理滾動
    lv_obj_clear_flag(message_widget, LV_OBJ_FLAG_SCROLLABLE);

    // 設置較高的 Z-index，確保在背景色塊之上
    lv_obj_move_foreground(message_widget);

    // 允許子物件超出邊界顯示（讓 icon 壓在 widget 上方）
    lv_obj_add_flag(message_widget, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    lv_obj_t *label = lv_label_create(message_widget);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    lv_obj_set_height(label, 40);
    lv_obj_set_width(label, LIST_MESSAGE_WIDTH - 70);
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)),
                               0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_style_bg_opa(label, LV_OPA_TRANSP, 0);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 50, 10);
    notification_widgets[i].title = label;

    lv_obj_t *content = lv_label_create(message_widget);
    lv_label_set_long_mode(content, LV_LABEL_LONG_DOT);
    lv_obj_set_height(content, LIST_MESSAGE_HEIGHT - 90);
    lv_obj_set_width(content, LIST_MESSAGE_WIDTH - 65);
    lv_obj_set_style_text_font(content,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(content, lv_color_hex(0xB3B3B3), 0);
    lv_obj_set_style_text_align(content, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_style_bg_opa(content, LV_OPA_TRANSP, 0);
    lv_obj_align(content, LV_ALIGN_TOP_LEFT, 25, 65);
    lv_obj_clear_flag(message_widget, LV_OBJ_FLAG_SCROLLABLE);
    notification_widgets[i].content = content;

    /* Top-left app-icon removed — the icon now lives on the indicator dot. */
    notification_widgets[i].icon = NULL;

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
                if (notification->type >= NOTIFICATION_APP_QUANTITY)
                {
                    LOG_W(
                        "Notification type %d exceeds max, using default icon",
                        notification->type);
                    notification->type = Notify_others; // 使用預設圖示
                }
                /* Top-left icon removed — the app icon now appears on the
                   indicator dot instead. */
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

        /* Safety: clear any lingering drag translate_x so the card doesn't
           come back with a stale offset from an interrupted drag. */
        if (notification_widgets[i].card &&
            lv_obj_is_valid(notification_widgets[i].card))
        {
            lv_anim_del(notification_widgets[i].card, set_drag_translate_x);
            lv_obj_set_style_translate_x(notification_widgets[i].card, 0, 0);
        }
    }

    /* Rebuild indicator dots so each dot reflects the current notification icon
       (plus a fixed iTunes icon when the media widget is present). */
    if (msg_indicator_dots_parent != NULL &&
        lv_obj_is_valid(msg_indicator_dots_parent))
    {
        destroy_msg_indicator_dots();
        if (msg_dot_total() > 0)
            create_msg_indicator_dots(msg_indicator_dots_parent);
    }

    /* 新建的 dots 在 z-order 上會跑到 arc_zone overlay 上面，導致 dot 把 press
     * 從 arc_zone 搶走、整個 arc-scroll 失效。把 overlay 拉回最上層 */
    if (p_app_notification != NULL && p_app_notification->arc_handle != NULL)
    {
        arc_scroll_bring_to_front(p_app_notification->arc_handle);
    }

    /* Clamp selected index to current item range (notifications + media
       widget) before applying visibility, so the chosen card is valid
       after add/remove. */
    {
        int total_items = msg_dot_total();
        if (total_items > 0 &&
            selected_message_index >= (uint16_t)total_items)
            selected_message_index = (uint16_t)(total_items - 1);
    }
    update_notification_card_visibility();

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
        if (p_app_notification->media_widget != NULL &&
            lv_obj_is_valid(p_app_notification->media_widget))
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
        lv_label_set_text(label, LV_EXT_STR_GET_BY_KEY(no_notifications, "No notifications"));
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    }
    else if (!have_media_widget &&
             p_app_notification->no_notifications_widget != NULL &&
             lv_obj_is_valid(p_app_notification->no_notifications_widget))
    {
        /* Widget already built — refresh the label text only.
           Triggered by language switch: the else-if above only creates when NULL,
           so we must find the existing label and update it here. */
        uint32_t n = lv_obj_get_child_cnt(p_app_notification->no_notifications_widget);
        for (uint32_t i = 0; i < n; i++)
        {
            lv_obj_t *c = lv_obj_get_child(p_app_notification->no_notifications_widget,
                                           (int32_t)i);
            if (lv_obj_check_type(c, &lv_label_class))
            {
                lv_label_set_text(c, LV_EXT_STR_GET_BY_KEY(no_notifications,
                                                            "No notifications"));
                break;
            }
        }
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

    /* item 數變了 → 同步給共用 arc_scroll 模組做 scroll clamp。
     * 用 page_count（含 media_widget），否則最後一格永遠 clamp 卡住 */
    if (p_app_notification != NULL && p_app_notification->arc_handle != NULL)
    {
        arc_scroll_set_item_count(p_app_notification->arc_handle, page_count);
    }
}

uint8_t get_message_page_count(void)
{
    return page_count;
}

/* Notification storm debounce.
   Why: when BLE reconnects, the phone dumps cached ANCS notifications in
   tight bursts. Each one triggers a full refresh_notification_list, which
   rebuilds 10 cards (lv_mem_alloc/free per label via replace_nbsp) plus
   destroy+create of indicator dots. During the burst the FreeType cache
   (app_ft_m, 600 KB) hits 75 % and lv_freetype_clean_cache fires repeatedly
   while EPIC still has pending blits → heap magic corruption. Compressing
   the burst to one head + one tail refresh keeps the rebuild from racing
   with FT cache cleanup. */
#define NOTIFICATION_REFRESH_DEBOUNCE_MS 200
static lv_timer_t *refresh_debounce_timer = NULL;
static rt_tick_t last_refresh_tick = 0;

static void refresh_debounce_timer_cb(lv_timer_t *t)
{
    (void)t;
    refresh_debounce_timer = NULL;
    last_refresh_tick = rt_tick_get();
    refresh_notification_list(NULL);
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

    rt_tick_t now = rt_tick_get();
    if (last_refresh_tick != 0 &&
        (now - last_refresh_tick) <
            rt_tick_from_millisecond(NOTIFICATION_REFRESH_DEBOUNCE_MS))
    {
        /* Within debounce window — defer to single trailing refresh. */
        if (refresh_debounce_timer == NULL)
        {
            refresh_debounce_timer = lv_timer_create(
                refresh_debounce_timer_cb,
                NOTIFICATION_REFRESH_DEBOUNCE_MS, NULL);
            if (refresh_debounce_timer)
                lv_timer_set_repeat_count(refresh_debounce_timer, 1);
        }
        else
        {
            lv_timer_reset(refresh_debounce_timer);
        }
        return;
    }
    last_refresh_tick = now;
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
/* arc-scroll 的 detached / discrete 模式狀態 — 變數跟 forward decl 在檔案開頭，
 * 拖動時 arc 不去動 list（由 drag_cb 接管），只更新 indicator dot，
 * 偵測到「最靠近中央的 dot 換了一顆」（= page change）才 call
 * scroll_message_list_to_index 把 list snap 過去。釋放時 reset。 */
static void message_arc_reset_drag_state(void)
{
    s_msg_arc_drag_active = false;
    s_msg_drag_initialized = false;
    s_msg_drag_input = 0;
    s_msg_drag_last_idx = -1;
}

/* dot 回彈動畫 — release 時若 input 還在 elastic overshoot 區，把它從當前
 * 位置補間到 canonical（idx 對應的 input value），看得到 dot 平滑回彈 */
static int32_t s_msg_snap_anim_dummy;
static void msg_snap_anim_exec_cb(void *var, int32_t value)
{
    (void)var;
    /* 動畫進行中如果 user 又開始拖，就讓 drag_cb 蓋過去；exec_cb 不再動 dot */
    if (s_msg_arc_drag_active) return;
    update_msg_indicator_dots_position((int)value);
}

static void msg_start_snap_anim(int from, int to)
{
    lv_anim_del(&s_msg_snap_anim_dummy, msg_snap_anim_exec_cb);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, &s_msg_snap_anim_dummy);
    lv_anim_set_exec_cb(&a, msg_snap_anim_exec_cb);
    lv_anim_set_values(&a, from, to);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

/* 右側弧形觸控滾動 — 用共用模組 common/arc_scroll.h */
static void message_arc_drag_cb(lv_coord_t scroll_delta_px, void *ctx)
{
    (void)ctx;
    int total = (int)notification_count + (have_media_widget ? 1 : 0);
    if (total <= 0) return;

    s_msg_arc_drag_active = true;

    /* 第一次進來：用目前的 selected_message_index 反算 input value，
     * 避免從預設 37（= 最後一顆 dot 在中央）跳到實際 selected。
     * input 跟 idx 對應公式：input = 100*(N - idx) - 63
     * （從 update_msg_indicator_dots_position 的 offset_angle 推回，
     * 解 idx*36 = (100N - input - 63)/100 * 36） */
    if (!s_msg_drag_initialized)
    {
        /* 新的拖動開始 — 取消上一輪 release 起的 snap anim，避免它跟 drag_cb
         * 同時寫 dot 位置打架 */
        lv_anim_del(&s_msg_snap_anim_dummy, msg_snap_anim_exec_cb);
        int idx = (int)selected_message_index;
        if (idx < 0) idx = 0;
        if (idx >= total) idx = total - 1;
        s_msg_drag_input = 100 * (total - idx) - 63;
        s_msg_drag_last_idx = idx;
        s_msg_drag_initialized = true;
    }

    /* scroll_delta_px → input delta：scroll_list 用
     * scaled_first = first_y_diff * 100 / pitch；scroll_y 增加（CCW）→
     * first_y_diff 減少 → input 減少。所以 d_input = -scroll_delta * 100 / pitch */
    const int pitch = LIST_MESSAGE_HEIGHT + LIST_MESSAGE_SPACING;
    int target_input = s_msg_drag_input - ((int)scroll_delta_px * 100) / pitch;

    /* 合法 input 範圍：idx ∈ [0, total-1]
     *   idx=0:        input = 100*total - 63
     *   idx=total-1:  input = 100*1 - 63 = 37 */
    int min_input = 100 - 63;
    int max_input = 100 * total - 63;
    /* elastic overshoot：邊界外的 input 套 0.4 resistance、總 overshoot 上限
     * 50 input 單位（= 半 slot，因為 100 input = 1 slot）— 跟 arc_scroll 內建
     * 模式的 slot_height_px/2 + 0.4 對齊。snap_cb 會在釋放時把 dot 拉回 valid */
    const int MAX_OVERSHOOT = 50;
    if (target_input < min_input)
    {
        int over_now = (s_msg_drag_input < min_input) ? (min_input - s_msg_drag_input) : 0;
        int over_raw = min_input - target_input;
        if (over_raw > over_now)
        {
            int additional = (over_raw - over_now) * 4 / 10;
            int new_over = over_now + additional;
            if (new_over > MAX_OVERSHOOT) new_over = MAX_OVERSHOOT;
            target_input = min_input - new_over;
        }
    }
    else if (target_input > max_input)
    {
        int over_now = (s_msg_drag_input > max_input) ? (s_msg_drag_input - max_input) : 0;
        int over_raw = target_input - max_input;
        if (over_raw > over_now)
        {
            int additional = (over_raw - over_now) * 4 / 10;
            int new_over = over_now + additional;
            if (new_over > MAX_OVERSHOOT) new_over = MAX_OVERSHOOT;
            target_input = max_input + new_over;
        }
    }
    s_msg_drag_input = target_input;

    update_msg_indicator_dots_position(s_msg_drag_input);

    /* 反算當前最靠近中央的 idx：input = 100*(N-idx) - 63 → idx = N - (input+63)/100 */
    int closest_idx = total - ((s_msg_drag_input + 63 + 50) / 100); /* +50 = round */
    if (closest_idx < 0) closest_idx = 0;
    if (closest_idx >= total) closest_idx = total - 1;

    if (closest_idx != s_msg_drag_last_idx)
    {
        s_msg_drag_last_idx = closest_idx;
        scroll_message_list_to_index((int8_t)closest_idx);
    }
}

static lv_obj_t *message_arc_tap_cb(lv_point_t pt, void *ctx)
{
    (void)ctx;
    /* tap 的話沒有過 drag 路徑，但保險起見 reset 一下 */
    message_arc_reset_drag_state();
    /* 仿 app_exercise.c 的 tap 路徑：先用 press 點比對所有可見的 indicator dot
     * bbox，找到哪顆就 forward CLICKED 給那顆。dot 上有掛 msg_dot_click_cb，
     * 直接打開該 notification — 不必先把它 scroll 到中央再點。
     *
     * 沒落在任何 dot 上 → fallback 用「選中項目」的舊邏輯：直接 call
     * list_message_click_cb（card 的 click 是自製 drag/tap 判斷不走 CLICKED，
     * 所以這邊不能 forward 給 card），回傳 NULL 讓 arc_scroll 不再 dispatch */
    if (p_app_notification == NULL) return NULL;

    /* 1. 先看 press 點落在哪顆 dot */
    int total_dots = (int)notification_count;
    if (have_media_widget) total_dots++;
    for (int i = 0; i < total_dots && i < MAX_MSG_DOTS; i++)
    {
        lv_obj_t *dot_bg = msg_indicator_dots_bg[i];
        if (dot_bg == NULL) continue;
        if (!lv_obj_is_valid(dot_bg)) continue;
        if (lv_obj_has_flag(dot_bg, LV_OBJ_FLAG_HIDDEN)) continue;
        lv_area_t a;
        lv_obj_get_coords(dot_bg, &a);
        if (pt.x >= a.x1 && pt.x <= a.x2 && pt.y >= a.y1 && pt.y <= a.y2)
        {
            return dot_bg;
        }
    }

    /* 2. fallback：選中項目（card 的 click 不走 LV_EVENT_CLICKED，所以這邊
     *    自己 call list_message_click_cb，回傳 NULL 讓 arc_scroll 不再 send 事件）*/
    if (notification_count == 0) return NULL;
    if (selected_message_index >= notification_count) return NULL;
    notification_t *notification =
        get_notification(notification_count - selected_message_index - 1);
    if (notification == NULL) return NULL;
    list_message_click_cb(notification);
    return NULL;
}

static lv_obj_t *message_arc_snap_cb(void *ctx)
{
    (void)ctx;
    /* list 已經在 page change 當下被 snap 過了。如果 drag 結束在 elastic
     * overshoot 區（input 超出 valid 範圍），起一個 anim 把 dot 從當前位置
     * 平滑彈回 idx 對應的 canonical input，看得到回彈動畫 */
    if (s_msg_drag_last_idx >= 0)
    {
        int total = (int)notification_count + (have_media_widget ? 1 : 0);
        if (total > 0)
        {
            int snap_input = 100 * (total - s_msg_drag_last_idx) - 63;
            if (s_msg_drag_input != snap_input)
            {
                msg_start_snap_anim(s_msg_drag_input, snap_input);
            }
        }
    }
    message_arc_reset_drag_state();
    return NULL;
}

lv_obj_t *lv_message_list_layout_create(lv_obj_t *parent)
{
    LOG_I("message_list_layout_create: enter");
    RT_ASSERT(NULL == p_app_notification);
    p_app_notification =
        (app_notification_t *)lv_mem_alloc(sizeof(app_notification_t));
    memset(p_app_notification, 0, sizeof(app_notification_t));
    /* 防呆：清掉上一輪可能殘留的 arc-drag 旗標。它是 file-static，reset_list /
       重新開 app 都不會碰到，若上次弧形拖動異常結束卡在 true，會讓本次新開的
       通知列表一進來 indicator dots 就無法被 touch-scroll 更新。 */
    message_arc_reset_drag_state();
    message_page = parent;
    LOG_I("message_list_layout_create: alloc ok");

    lv_obj_t *notification_center = lv_obj_create(parent);
    lv_obj_set_style_bg_opa(notification_center, LV_OPA_TRANSP,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_size(notification_center, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_scrollbar_mode(notification_center, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(notification_center, LV_OBJ_FLAG_SCROLLABLE);

    /* Indicator dots live on the non-scrolling background so they stay
       fixed on screen while the list scrolls. */
    msg_indicator_dots_parent = notification_center;

    LOG_I("message_list_layout_create: before create_background_blocks");
    // 創建背景色塊
    create_background_blocks(notification_center);
    LOG_I("message_list_layout_create: after create_background_blocks");

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

    LOG_I("message_list_layout_create: before card builder loop");
    /* Repopulate the list with the new number of items */
    for (uint8_t i = 0; i < 10; i++) // new_item_count
    {
        LOG_I("card builder i=%d", i);
        notification_widgets[i].card = notification_card_builder(p_window, i);
    }
    LOG_I("message_list_layout_create: after card builder loop");

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
    LOG_I("message_list_layout_create: before refresh_notification_list");
    refresh_notification_list(p_app_notification->main_window);
    LOG_I("message_list_layout_create: before lv_event_send SCROLL");
    lv_event_send(p_app_notification->main_window, LV_EVENT_SCROLL, NULL);
    LOG_I("message_list_layout_create: after lv_event_send SCROLL");

    /* 右側弧形觸控滾動 — 跟 instruction_list / exercise / clock 共用同一份算法。
     * slot_height = 252 + 40 = 292（item 間距），item_height = 252（item 本身高度）。
     * slot_angle_deg=36 跟 msg_indicator_dots 的 angle_per_dot 對齊（同 exercise）。
     * message_list 是 tileview 子層，lock_ancestors 設 true 防止上下端切線拖曳被外層 tileview 搶走。
     *
     * item_count 必須用 page_count（= notification_count + media_widget），
     * 否則 media widget 的 scroll 位置會超出 arc clamp 的 max_scroll，使用者一進來
     * 就被卡住、CCW 完全動不了（只有 CW 能回到上一個 notification）*/
    arc_scroll_config_t arc_cfg = {
        .parent          = notification_center,
        /* list 雖然還是傳進來給 bounds_cb 之類用得到，drag_cb 模式下 arc 不會
         * 直接 _lv_obj_scroll_by_raw 它 — 改由 drag_cb 累積角度更新 dot，
         * 偵測到 page 換才 call scroll_message_list_to_index snap list */
        .list            = p_window,
        .slot_height_px  = LIST_MESSAGE_HEIGHT + LIST_MESSAGE_SPACING,
        .item_height_px  = LIST_MESSAGE_HEIGHT,
        .slot_angle_deg  = 36,
        .item_count      = page_count,
        /* 90 → 40: the right-edge arc band (right half, outer ring of
           thickness px) overlapped the media widget's right-hand "next"
           button, so taps on that icon were captured by the arc overlay
           instead of the button. 40 pulls the band's inner edge out past the
           next button (its far edge sits ~40px in from the screen rim). */
        .band_thickness  = 40,
        .lock_ancestors  = true,
        .tap_cb          = message_arc_tap_cb,
        .snap_cb         = message_arc_snap_cb,
        .drag_cb         = message_arc_drag_cb,
        .ctx             = NULL,
    };
    LOG_I("message_list_layout_create: before arc_scroll_create");
    p_app_notification->arc_handle = arc_scroll_create(&arc_cfg);
    LOG_I("message_list_layout_create: after arc_scroll_create");
    /* DEBUG：顯示 arc band 觸發範圍。確認位置後可以拿掉這行 */
    // arc_scroll_set_debug_visible(p_app_notification->arc_handle, true);

    myLancher[app_index_message].reset_list = reset_list_cb;
    myLancher[app_index_message].on_tap = on_tap;
    lvgl_msg_handler.handle_widgets_control = button_selection;

    LOG_I("message_list_layout_create: returning");
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
    // if (abs(p_y - 233) < 50)
    // {
    //     set_paused_control_with_arm(false);
    // }
    // else
    // {
    //     set_paused_control_with_arm(true);
    // }
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

}

#if !kReleaseMode
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
#endif

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

    /* 中間 widget 跟著「最靠近中央的 dot」走 — navigation_bar_control_with_euler_angle
     * 在 page 變了的時候會送 NAV_BAR_CONTROL 進來這個函式。先把對應的 card unhide
     * 並更新 selected_message_index，再 scroll 列表，這樣中央卡片立刻換成新的、
     * list 隨後動畫到那個位置。順序很重要 — 先 set 再 scroll，動畫途中
     * scroll_list 的 motion-mode gate 會擋住把 selected 推回去 */
    int8_t total = (int8_t)(notification_count + (have_media_widget ? 1 : 0));
    if (page >= 0 && page < total &&
        page != (int8_t)selected_message_index)
    {
        selected_message_index = (uint16_t)page;
        update_notification_card_visibility();
        /* The arc-scroll snap path pre-syncs old_selected to the new index so
         * the subsequent scroll_list() doesn't see a diff and bounce the
         * selection back during animation. That pre-sync silenced the haptic
         * trigger at line ~649 (it gates on old != selected). Fire the haptic
         * here instead, where we know an actual page change just happened. */
        old_selected_message_index = selected_message_index;
        if (get_scrolling_motor_vibrate_status() && open_shock)
        {
            motor_pattern_scrolling_app();
        }
    }

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

static void refresh_new_message_widget(void);

/* Public wrapper for ui_handler 的語言切換廣播 — 刷新錶盤主畫面上常駐的通知小卡
   (new_notification_widgets) 讓其 title label 重取當前語言字串。原 static 函式
   保持不動其 linkage；這是唯一對外入口。 */
void notification_widget_refresh_language(void)
{
    /* widget 尚未建立時 title 為 NULL，refresh_new_message_widget 第一步就會
       對 title set_text，先擋掉避免空指標。 */
    if (new_notification_widgets.title != NULL &&
        lv_obj_is_valid(new_notification_widgets.title))
    {
        refresh_new_message_widget();
    }
}

static void refresh_new_message_widget(void)
{
    notification_t *notification =
        get_notification(0); // notification_center_get_info_count()
    if (notification_center_get_info_count() <= 0)
    {
        lv_label_set_text(new_notification_widgets.title, LV_EXT_STR_GET_BY_KEY(no_notifications, "No notifications"));
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
                       notif_icon_src(notification->type));
        lv_obj_clear_flag(new_notification_widgets.icon, LV_OBJ_FLAG_HIDDEN);
        have_message_now = true;
    }
    /* handle_dial_header_new_notification is called directly by ui_handler
       via LVGL_MSG_TYPE_NOTIFICATION; it has its own count check to filter
       out refreshes that are not truly new notifications. */
}

/*******************************************************************************
 * Dial Header: shows music or notification on clock face
 ******************************************************************************/
static lv_obj_t *dial_header_bg = NULL;
/* Upper label on the dial header — shows notification->title or media title. */
static lv_obj_t *dial_header_title = NULL;
/* Lower label (occupies the original dial_header_title position) — shows
   notification->message (content). Empty for media. */
static lv_obj_t *dial_header_content = NULL;
static lv_obj_t *dial_header_img = NULL;
static lv_obj_t *dial_header_red_dot = NULL;
static lv_obj_t *dial_header_bg_mask = NULL;
static bool dial_header_was_music_before_notif = false;
static rt_timer_t dial_header_music_pause_timer = NULL;
#define MUSIC_PAUSE_HIDE_TIMEOUT_MS 30000
static void dial_header_music_pause_timer_stop(void);
static void dial_header_music_pause_timer_start(void);

static void dial_header_apply_hidden_state(void)
{
    if (lv_obj_is_valid(dial_header_bg))
    {
        lv_obj_add_flag(dial_header_bg, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_img_opa(dial_header_bg_mask, LV_OPA_COVER, 0);
        lv_obj_set_style_img_opa(dial_header_img, LV_OPA_30, 0);
        lv_obj_set_style_text_opa(dial_header_title, LV_OPA_COVER, 0);
        lv_obj_set_style_text_opa(dial_header_content, LV_OPA_COVER, 0);
        lv_obj_set_style_border_opa(dial_header_bg, 30, 0);
        lv_obj_set_style_bg_opa(dial_header_bg, 15, 0);
    }
    if (lv_obj_is_valid(dial_header_red_dot) &&
        notification_center_get_info_count() > 0)
        lv_obj_clear_flag(dial_header_red_dot, LV_OBJ_FLAG_HIDDEN);
}

static void dial_header_fadeout_ready_cb(lv_anim_t *anim)
{
    dial_header_apply_hidden_state();
}

static void dial_header_fadeout_exec_cb(void *obj, int32_t value)
{
    // lv_obj_set_style_opa((lv_obj_t *)obj, value, 0);
    // LOG_D("dial_header_fadeout_exec_cb: %d", value);
    uint8_t header_img_opa = (value * LV_OPA_30) /
                             LV_OPA_COVER; // Image is half the opacity of the content
    lv_obj_set_style_img_opa(dial_header_img, header_img_opa, 0);
    lv_obj_set_style_img_opa(dial_header_bg_mask, value, 0);
    lv_obj_set_style_text_opa(dial_header_title, value, 0);
    lv_obj_set_style_text_opa(dial_header_content, value, 0);
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
        lv_obj_set_style_img_opa(dial_header_img, LV_OPA_30, 0);
        lv_obj_set_style_img_opa(dial_header_bg_mask, LV_OPA_COVER, 0);
        lv_obj_set_style_text_opa(dial_header_title, LV_OPA_COVER, 0);
        lv_obj_set_style_text_opa(dial_header_content, LV_OPA_COVER, 0);
        lv_obj_clear_flag(dial_header_bg, LV_OBJ_FLAG_HIDDEN);
        /* Media now ships {"title","artist"} — title on top, artist below. */
        extern char *get_media_artist(void);
        char *media_artist = get_media_artist();
        lv_label_set_text(dial_header_content, media_title);
        lv_label_set_text(dial_header_title, media_artist ? media_artist : "");
        lv_img_set_src(dial_header_img, MEDIA_HEADER_IMG);
        lv_obj_set_size(dial_header_img, 100, 100);
        lv_img_set_zoom(dial_header_img, 254);
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
    dial_header_shrink_duration_ms = 0;
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

static void dial_header_show_notification(void)
{
    if (!lv_obj_is_valid(dial_header_bg))
    {
        return;
    }
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
            /* Title in the upper label, message body in the content label.
               notification->message is a char array, so pass it directly —
               an empty message is already an empty C string. */
            lv_label_set_text(dial_header_title, notification->title);
            lv_label_set_text(dial_header_content, notification->message);
            lv_img_set_src(dial_header_img, notif_icon_src(notification->type));
            lv_obj_set_size(dial_header_img, 100, 100);
            lv_img_set_zoom(dial_header_img, 254);
            lv_obj_clear_flag(dial_header_img, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(dial_header_bg_mask, LV_OBJ_FLAG_HIDDEN);
            lv_obj_align(dial_header_img, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_style_img_opa(dial_header_img, LV_OPA_30, 0);
            lv_obj_set_style_img_opa(dial_header_bg_mask, LV_OPA_COVER, 0);
            lv_obj_set_style_text_opa(dial_header_title, LV_OPA_COVER, 0);
            lv_obj_set_style_text_opa(dial_header_content, LV_OPA_COVER, 0);
            lv_obj_set_style_border_opa(dial_header_bg, 30, 0);
            lv_obj_set_style_bg_opa(dial_header_bg, 15, 0);
            return;
        }
    }
    lv_obj_add_flag(dial_header_bg, LV_OBJ_FLAG_HIDDEN);
}

void handle_dial_header_media_title(char *media_title_text)
{
    if (!lv_obj_is_valid(dial_header_title) ||
        !lv_obj_is_valid(dial_header_content) ||
        !lv_obj_is_valid(dial_header_bg))
        return;
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
            lv_obj_set_style_img_opa(dial_header_img, LV_OPA_30, 0);
            lv_obj_set_style_img_opa(dial_header_bg_mask, LV_OPA_COVER, 0);
            lv_obj_set_style_text_opa(dial_header_title, LV_OPA_COVER, 0);
            lv_obj_set_style_text_opa(dial_header_content, LV_OPA_COVER, 0);
            lv_obj_clear_flag(dial_header_bg, LV_OBJ_FLAG_HIDDEN);
            /* Title in top label; artist (if any) in content label. */
            extern char *get_media_artist(void);
            char *media_artist_text = get_media_artist();
            lv_label_set_text(dial_header_content, media_title_text);
            lv_label_set_text(dial_header_title,
                              media_artist_text ? media_artist_text : "");
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
                dial_header_shrink_duration_ms = 8000;
                dial_header_shrink_start_tick = rt_tick_get();
                dial_header_shrink_timer =
                    lv_timer_create(dial_header_shrink_timer_cb,
                                    dial_header_shrink_duration_ms, NULL);
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
        lv_obj_set_size(dial_header_img, 100, 100);
        lv_img_set_zoom(dial_header_img, 254);
        lv_obj_align(dial_header_img, LV_ALIGN_CENTER, 0, 0);
    }
    else
    {
        lv_obj_add_flag(dial_header_img, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(dial_header_bg_mask, LV_OBJ_FLAG_HIDDEN);
    }
}

static void dial_header_event_cb(lv_event_t *evt)
{
    if (evt->code == LV_EVENT_CLICKED)
    {
        /* If user taps the header while it's showing a notification, jump to the
         * first notification in the list. */
        animate_to_message_list();
    }
}

/* Last-seen arrival seq from notification center. Count-based dedup fails
   when the ring buffer is full (oldest is displaced, count stays at cap) —
   comparing the monotonic arrival seq catches every genuine new
   notification regardless of buffer state. */
static uint32_t dial_header_prev_arrival_seq = 0;
static void handle_dial_header_new_notification(void)
{
    if (!lv_obj_is_valid(dial_header_bg))
    {
        return;
    }
    /* Only react on genuinely new arrivals (skip refreshes triggered by
       music operations, list rebuilds, etc.) */
    uint32_t cur_seq = notification_center_get_arrival_seq();
    if (cur_seq == dial_header_prev_arrival_seq)
    {
        return;
    }
    dial_header_prev_arrival_seq = cur_seq;
    /* User 已經在通知列表 → 新通知會直接出現在列表裡，不需要再彈 header。
       count 仍要更新（上面已做）才不會之後一次補彈。 */
    if (is_at_message())
        return;
    /* Remember if music was playing before this notification */
    dial_header_was_music_before_notif = dial_header_music_active;
    dial_header_showing_notification = true;
    /* Always show notification, even if music is active */
    dial_header_show_notification();
    /* Start or restart 5-second shrink timer */
    if (dial_header_shrink_timer)
        lv_timer_del(dial_header_shrink_timer);
    dial_header_shrink_duration_ms = 8000;
    dial_header_shrink_start_tick = rt_tick_get();
    dial_header_shrink_timer =
        lv_timer_create(dial_header_shrink_timer_cb,
                        dial_header_shrink_duration_ms, NULL);
    lv_timer_set_repeat_count(dial_header_shrink_timer, 1);
    /* Header 開始顯示這則新通知 → 同步把(此刻隱藏的)列表停留位置移到最新那則
       (child notification_count-1，音樂 widget 正上方)。這樣稍後不論點或滑進
       列表，頁面一 render 就已經停在最新通知，不會先停在音樂 widget 再瞬跳。
       此處 flag 剛設 true 且必為 !is_at_message()(上面已 early return)，reset_list
       走 else 分支 → reset_count = notification_count-1。這步是關鍵：定位用的
       refresh 可能在 flag 設 true 之前就跑過(把列表停在音樂)，所以要在 flag 確定
       為 true 的此處主動 reset。 */
    if (p_app_notification && p_app_notification->list)
        reset_list(false);
}

void lv_dial_header_builder(lv_obj_t *parent)
{
    dial_header_bg = lv_obj_create(parent);
    lv_obj_set_size(dial_header_bg, 300, 100);
    lv_obj_set_style_bg_color(dial_header_bg, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(dial_header_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(dial_header_bg, 30, 0);
    // lv_obj_set_style_outline_width(dial_header_bg, 2, 0);
    // lv_obj_set_style_outline_color(dial_header_bg, lv_color_hex(0xFFFFFF),
    // 0); lv_obj_set_style_outline_opa(dial_header_bg, LV_OPA_20, 0);
    // lv_obj_set_style_border_width(dial_header_bg, 1, 0);
    // lv_obj_set_style_border_color(dial_header_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_clear_flag(dial_header_bg, LV_OBJ_FLAG_SCROLLABLE);
    
    lv_obj_align(dial_header_bg, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_flag(dial_header_bg, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *dial_header_img_bg = lv_obj_create(dial_header_bg);
    lv_obj_set_size(dial_header_img_bg, 100, 100);
    lv_obj_set_style_radius(dial_header_img_bg, 20, 0);
    lv_obj_set_style_clip_corner(dial_header_img_bg, true, 0);
    lv_obj_set_style_bg_opa(dial_header_img_bg, LV_OPA_TRANSP, 0);
    lv_obj_align(dial_header_img_bg, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_clear_flag(dial_header_img_bg, LV_OBJ_FLAG_SCROLLABLE);

    dial_header_img = lv_img_create(dial_header_img_bg);
    lv_img_set_src(dial_header_img, MEDIA_MASK);
    lv_obj_set_size(dial_header_img, 100, 100);
    lv_obj_set_style_radius(dial_header_img, 25, 0);
    lv_obj_set_style_img_opa(dial_header_img, LV_OPA_30, 0);
    lv_obj_align(dial_header_img, LV_ALIGN_CENTER, 0, 0);

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
    lv_obj_align(dial_header_bg_mask, LV_ALIGN_BOTTOM_MID, 0, 0);
    // lv_obj_set_style_radius(dial_header_bg_mask, 30, 0);

    /* Title label — positioned ABOVE the content label.
    Shows notification->title or media title. */
    dial_header_title = lv_label_create(dial_header_bg);
    lv_obj_set_size(dial_header_title, 150, 30);
    lv_label_set_long_mode(dial_header_title, LV_LABEL_LONG_DOT);
    lv_obj_set_style_anim_speed(dial_header_title, 15, 0);
    lv_obj_set_style_text_align(dial_header_title, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(dial_header_title,
                               LV_EXT_FONT_GET(get_system_font_size(-2)), 0);
    lv_obj_set_style_text_color(dial_header_title, lv_color_white(), 0);
    lv_obj_set_style_text_opa(dial_header_title, LV_OPA_50, 0);
    lv_obj_align(dial_header_title,
                    LV_ALIGN_TOP_MID, 0, 8);


    /* Content label — at the original dial_header_title position (below image).
       Shows notification->message for notifications, empty for media. */
    dial_header_content = lv_label_create(dial_header_bg_mask);
    lv_obj_set_size(dial_header_content, 240, 50);
    lv_label_set_long_mode(dial_header_content, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_anim_speed(dial_header_content, 15, 0);
    lv_obj_set_style_text_align(dial_header_content, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(dial_header_content,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(dial_header_content, lv_color_white(), 0);
    lv_obj_set_style_text_opa(dial_header_content, LV_OPA_70, 0);
    lv_obj_align(dial_header_content,
                    LV_ALIGN_BOTTOM_MID, 0, 0);
    

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
        /* Title in top label; artist (if any) in content label. */
        extern char *get_media_artist(void);
        char *media_artist = get_media_artist();
        lv_label_set_text(dial_header_content, media_title);
        lv_label_set_text(dial_header_title, media_artist ? media_artist : "");
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

    lv_obj_t* houch_obj = lv_obj_create(dial_header_bg);
    lv_obj_set_size(houch_obj, 300, 100);
    lv_obj_set_style_bg_opa(houch_obj, LV_OPA_TRANSP, 0);
    lv_obj_align(houch_obj, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(houch_obj, dial_header_event_cb, LV_EVENT_CLICKED, NULL);
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

void handle_dial_header_media_play_state(bool playing)
{
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

void dial_header_on_suspend(void)
{
    /* Delete the lv_timer but preserve start_tick/duration so we can
       compensate on resume. */
    if (dial_header_shrink_timer)
    {
        lv_timer_del(dial_header_shrink_timer);
        dial_header_shrink_timer = NULL;
    }
}

void dial_header_on_resume(void)
{
    if (dial_header_shrink_duration_ms == 0)
        return;
    uint32_t elapsed_ms =
        (rt_tick_get() - dial_header_shrink_start_tick) * 1000 / RT_TICK_PER_SECOND;
    if (elapsed_ms >= dial_header_shrink_duration_ms)
    {
        /* Timer expired during sleep — jump to end state without animation. */
        dial_header_shrink_duration_ms = 0;
        dial_header_showing_notification = false;
        if (dial_header_was_music_before_notif || dial_header_music_active)
        {
            dial_header_was_music_before_notif = false;
            dial_header_restore_music();
            if (p_app_notification && p_app_notification->list)
                reset_list(true);
        }
        else
        {
            dial_header_apply_hidden_state();
        }
    }
    else
    {
        /* Not yet expired — recreate timer with remaining time. */
        uint32_t remaining_ms = dial_header_shrink_duration_ms - elapsed_ms;
        dial_header_shrink_timer =
            lv_timer_create(dial_header_shrink_timer_cb, remaining_ms, NULL);
        lv_timer_set_repeat_count(dial_header_shrink_timer, 1);
    }
}

void dial_media_header_init(void)
{
    lvgl_msg_handler.handle_dial_media_header_img =
        handle_dial_header_media_img;
    lvgl_msg_handler.handle_dial_header_new_notification =
        handle_dial_header_new_notification;
    // lvgl_msg_handler.handle_dial_media_play_state =
    //     handle_dial_header_media_play_state;
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
    if (lvgl_msg_handler.handle_dial_media_header_img ==
        handle_dial_header_media_img)
        lvgl_msg_handler.handle_dial_media_header_img = NULL;
    if (lvgl_msg_handler.handle_dial_header_new_notification ==
        handle_dial_header_new_notification)
        lvgl_msg_handler.handle_dial_header_new_notification = NULL;
    // if (lvgl_msg_handler.handle_dial_media_play_state ==
    //     handle_dial_header_media_play_state)
    //     lvgl_msg_handler.handle_dial_media_play_state = NULL;
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
    /* 進場前先快照「header 當下顯示的是不是通知」。列表落點要跟著 header 狀態
       走（與點/滑無關）：header 在顯示通知（8 秒暫時窗）→ 停最新通知；header
       已變回音樂 → 停音樂 widget。下面的清理會把這個旗標抹掉，所以先存起來。 */
    bool header_showing_notification = dial_header_showing_notification;
    /* 進到通知列表 → header 不應該再顯示通知（例外：音樂一直要在）。
       同時把 shrink timer / 狀態清乾淨，避免回主畫面時還殘留通知 header。 */
    if (dial_header_shrink_timer)
    {
        lv_timer_del(dial_header_shrink_timer);
        dial_header_shrink_timer = NULL;
    }
    dial_header_shrink_duration_ms = 0;
    dial_header_showing_notification = false;
    dial_header_was_music_before_notif = false;
    if (dial_header_music_active)
    {
        /* 音樂一直要在 → 還原音樂 header */
        dial_header_restore_music();
    }
    else
    {
        /* 沒音樂 → 整個 header 收起來（紅點接著也會被下一行隱藏） */
        dial_header_apply_hidden_state();
    }
    /* User has seen the notification list — hide the red dot on the dial */
    if (lv_obj_is_valid(dial_header_red_dot))
        lv_obj_add_flag(dial_header_red_dot, LV_OBJ_FLAG_HIDDEN);
    lvgl_msg_handler.handle_tap_indicator = press_cb;
    set_scroll_segment_count(get_message_page_count());
    set_control_gravity_x_range(0.6f, -0.6f, false);
    lvgl_msg_handler.handle_widgets_control = button_selection;
    lvgl_msg_handler.handle_nav_bar_control = scroll_message_list_to_index;
    /* 依進場當下的 header 狀態主動定位列表（兩分支都要主動 reset，否則 revert
       回音樂後若沒移回去，會殘留在上次的通知位置——就是「滑上去卻停在通知」）。
       此處 is_at_message() 已為 true（check_is_at_message 先把 _at_message 設
       true 才呼叫本函式），reset_list 走 selected_message_index 分支。 */
    if (p_app_notification && p_app_notification->list)
    {
        if (header_showing_notification && notification_count > 0)
        {
            /* header 在顯示通知 → 停最新那則。child i 顯示的是
               get_notification(notification_count-i-1)，所以最新那則
               (get_notification(0)，正是 header 顯示的) 在最後一張卡片＝
               child notification_count-1（音樂 widget 的正上方），不是 child 0。 */
            selected_message_index = notification_count - 1;
            reset_list(false);
        }
        else if (have_media_widget)
        {
            /* header 已是音樂（或一直是音樂）→ 停音樂 widget（child 在所有通知
               之後，index = notification_count）。 */
            selected_message_index = notification_count;
            reset_list(false);
        }
    }
    return RT_EOK;
}

rt_int32_t notification_on_deinit(void)
{
    // 清理拖拽狀態
    is_dragging = false;
    dragging_widget = NULL;

    /* Tear down touching_screen delay timer */
    if (touching_screen_timer)
    {
        rt_timer_stop(touching_screen_timer);
        rt_timer_delete(touching_screen_timer);
        touching_screen_timer = NULL;
    }
    touching_screen = false;

    /* Tear down indicator dots */
    destroy_msg_indicator_dots();
    msg_indicator_dots_parent = NULL;

    /* Cancel any pending refresh debounce — its cb references
       refresh_notification_list which early-returns on p_app_notification ==
       NULL, but deleting the timer avoids a wasted callback after teardown. */
    if (refresh_debounce_timer)
    {
        lv_timer_del(refresh_debounce_timer);
        refresh_debounce_timer = NULL;
    }
    last_refresh_tick = 0;

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

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/