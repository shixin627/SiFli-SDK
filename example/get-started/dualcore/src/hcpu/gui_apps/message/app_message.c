/*********************o_button
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "ui_helper.h"
#include "ui_img_helper.h"
#include "bloc_notification.h"
#include "ui_handler.h"
#include "custom_trans_anim.h"
#include "app_message.h"
#include "bloc_v2t.h"
#include "bloc_motor.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#include "watch_system_core_task.h"
#endif

#define DBG_TAG "app.message"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_MESSAGE

typedef struct
{
    lv_ex_data_t *title;
    lv_ex_data_t *content;
    lv_ex_data_t *app_name;
    void *title_handle;
    void *content_handle;
    void *app_name_handle;
    bool *can_reply;
} app_message_ctx_t;

typedef struct
{
    lv_obj_t *main_window;
} app_message_t;

LV_IMG_DECLARE(icon_reply);

static app_message_ctx_t *p_app_message_ctx = NULL;
static app_message_t *p_app_message = NULL;

void cont_event_callback(lv_event_t *event)
{
}

static lv_obj_t *lbl_content;
static lv_obj_t *lbl_title;
static lv_obj_t *lbl_app_name;
static notification_t notification;

static bool open_from_message_list = false;
void app_message_set_open_from_message_list(bool open)
{
    open_from_message_list = open;
}

static void navigate_to_reply(void)
{
    strcpy(notification.title, p_app_message_ctx->title->value.s);
    notify_provider.navigate_to_reply(&notification);
    if (!open_from_message_list)
    {
        gui_app_exit(APP_ID_MESSAGE);
    }
}
static void reply_btn_event_callback(lv_event_t *event)
{
    if (LV_EVENT_CLICKED == event->code)
    {
        LOG_D("reply_btn_event_callback");
        navigate_to_reply();
    }
}

static void ingore_btn_event_callback(lv_event_t *event)
{
    if (LV_EVENT_CLICKED == event->code)
    {
        gui_app_self_exit();
    }
}

static lv_obj_t *reply_btn = NULL;
static lv_obj_t *icon_btn_send = NULL;
static rt_timer_t off_selected_timer = NULL;
static bool selected_message = false;
static void switch_selected_message(bool on)
{
    LOG_D("switch_selected_timer_cb : %d", on);
    if (on)
    {
        // lv_obj_set_style_border_opa(message_widget_standby, LV_OPA_100, LV_PART_MAIN);
        // lv_obj_set_style_img_opa(icon_btn_send, LV_OPA_100, 0);
        lv_obj_set_style_shadow_opa(reply_btn, LV_OPA_100, 0); // 設置陰影透明度
    }
    else
    {
        // lv_obj_set_style_border_opa(message_widget_standby, LV_OPA_40, LV_PART_MAIN);
        // lv_obj_set_style_img_opa(icon_btn_send, LV_OPA_40, 0);
        lv_obj_set_style_shadow_opa(reply_btn, LV_OPA_0, 0); // 設置陰影透明度
    }
}

static void switch_selected_timer_cb(void *param)
{
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_SWITCH_SELECTED;
    msg.data.switch_selected = *(bool *)param;
    lvgl_send_msg(msg);
    selected_message = false;
}

static void start_selected_timer(void)
{
    selected_message = true;
    switch_selected_timer_cb(&selected_message);
    if (off_selected_timer == NULL)
    {
        off_selected_timer = rt_timer_create("off_selected_timer", switch_selected_timer_cb, &selected_message, 500, RT_TIMER_FLAG_ONE_SHOT);
    }
    rt_timer_start(off_selected_timer);
}

static bool messagr_can_reply = false;
static bool reply_lock = true;
static lv_obj_t *tileview_page[2];
static lv_obj_t *notif_badge = NULL;
static void scroll_page_goin_message(void)
{
    LOG_D("scroll_page_goin_message");
    if (lv_obj_is_valid(tileview_page[0]))
    {
        lv_obj_set_tile_id(tileview_page[0], 0, 1, LV_ANIM_ON);
        if (messagr_can_reply)
        {
            reply_lock = false;
            LOG_D("reply_lock false");
        }
    }
}

static void wait_for_message_timer_callback(lv_timer_t *timer)
{
    if (tileview_page[0] != NULL)
    {
        scroll_page_goin_message();
    }
    else
    {
        LOG_W("tileview_page[0] is NULL");
    }
}

static void wait_for_message_timer_create(void)
{
    // 創建一個延遲1秒的定時器
    lv_timer_t *wait_for_message_timer = lv_timer_create(wait_for_message_timer_callback, 1000, NULL);
    if (wait_for_message_timer != NULL)
    {
        lv_timer_set_repeat_count(wait_for_message_timer, 1); // 設置定時器只執行一次
    }
    else
    {
        LOG_E("Failed to create wait_for_message_timer");
    }
}

static void double_tap_event_handler(void);
static void gesture_tap_event_handler(uint8_t gesture)
{
    if (gesture != 1)
    {
        return;
    }
    // start_selected_timer();
    if (reply_lock)
    {
        LOG_D("reply_lock");
        return;
    }
    navigate_to_reply();

    if (selected_message)
    {
        // double_tap_event_handler();
    }
}

static void double_tap_event_handler(void)
{
    LOG_D("double_tap_event_handler");
    if (reply_lock)
    {
        LOG_D("reply_lock");
        return;
    }
    navigate_to_reply();
}

static bool standby_widget_lock = false;
static lv_obj_t *message_widget_standby = NULL;
static lv_obj_t *message_title_standby = NULL;
static lv_obj_t *icon_standby = NULL;
static lv_obj_t *message_list = NULL;
static uint16_t bg_opa = 255;
static uint16_t icon_lock_pos[2];
static void message_view_event_cb(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);
    switch (event->code)
    {
    case LV_EVENT_SCROLL:
    {
        if (standby_widget_lock)
        {
            LOG_D("standby_widget_lock");
            break;
        }

        lv_coord_t message_scroll_y = bg_opa - (lv_obj_get_scroll_y(obj) * bg_opa / 466);
        if (message_scroll_y <= 10)
        {
            lv_obj_clear_flag(tileview_page[0], LV_OBJ_FLAG_SCROLLABLE);
            standby_widget_lock = true;
        }
        if (message_scroll_y < bg_opa && message_scroll_y > 0)
        {
            lv_obj_set_style_text_opa(message_widget_standby, message_scroll_y, 0);
            // lv_obj_set_style_img_opa(icon_standby, message_scroll_y, 0);
            lv_obj_set_style_text_opa(message_title_standby, message_scroll_y, 0);
        }
        else if (message_scroll_y >= bg_opa)
        {
            lv_obj_set_style_text_opa(message_widget_standby, bg_opa, 0);
            // lv_obj_set_style_img_opa(icon_standby, bg_opa, 0);
            lv_obj_set_style_text_opa(message_title_standby, bg_opa, 0);
        }

        lv_coord_t icon_scroll_y = 256 - (lv_obj_get_scroll_y(obj) * 90 / 466);
        if (icon_scroll_y < 256 && icon_scroll_y > 166)
        {
            uint16_t icon_move_x = (icon_scroll_y - 256) * (1.6);
            uint16_t icon_move_y = (icon_scroll_y - 256) * (0.25);
            lv_img_set_zoom(icon_standby, icon_scroll_y);

            lv_obj_align(icon_standby, LV_ALIGN_TOP_MID, icon_move_x, 70 + icon_move_y);
            icon_lock_pos[0] = icon_move_x;
            icon_lock_pos[1] = icon_move_y + 70;
        }
        else if (icon_scroll_y <= 166)
        {
            lv_img_set_zoom(icon_standby, 166);
            lv_obj_align(icon_standby, LV_ALIGN_TOP_MID, -90, 70);
        }

        break;
    }
    case LV_EVENT_VALUE_CHANGED:
    {
        rt_uint32_t active_pos = (rt_uint32_t)lv_event_get_param(event);
        if (active_pos == 0)
        {
            lv_obj_set_style_text_opa(message_widget_standby, 0, 0);
            // lv_obj_set_style_img_opa(icon_standby, 0, 0);
            lv_obj_set_style_text_opa(message_title_standby, 0, 0);
        }
        else if (active_pos == 1)
        {
            lv_obj_set_style_text_opa(message_widget_standby, bg_opa, 0);
            // lv_obj_set_style_img_opa(icon_standby, bg_opa, 0);
            lv_obj_set_style_text_opa(message_title_standby, bg_opa, 0);
            LOG_D("reply_lock false");
            if (messagr_can_reply)
            {
                reply_lock = false;
            }
        }
        break;
    }
    case LV_EVENT_CLICKED:
    {
        lv_obj_set_tile_id(tileview_page[0], 0, 1, LV_ANIM_ON);
        break;
    }
    default:
        break;
    }
}

static void message_list_view_event_cb(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);
    switch (event->code)
    {
    case LV_EVENT_SCROLL:
    {
        lv_coord_t message_scroll_y = (lv_obj_get_scroll_y(obj));
        lv_obj_align(icon_standby, LV_ALIGN_TOP_MID, icon_lock_pos[0], icon_lock_pos[1] - message_scroll_y);
        break;
    }
    default:
        break;
    }
}

static lv_obj_t *app_icon_builder(lv_obj_t *parent, uint8_t app_index)
{
    // lv_obj_t *icon_bg = lv_obj_create(parent);
    // lv_obj_set_size(icon_bg, 100, 100);
    // lv_obj_set_style_radius(icon_bg, 50, LV_PART_MAIN);
    // lv_obj_set_style_bg_color(icon_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_t *icon = lv_img_create(parent);
    lv_img_set_src(icon, icon_list[app_index]);
    return icon;
}

extern char *replace_nbsp(const char *str);
static void label_set_text(lv_obj_t *obj, const char *text)
{
    char *out_text = replace_nbsp(text);
    lv_label_set_text(obj, out_text);
}

static lv_obj_t *standby_message_title_builder(lv_obj_t *parent, lv_ex_binding_t *binding)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 220);
    lv_label_set_text(label, " ");
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    RT_ASSERT(p_app_message_ctx->title);
    binding->target = label;
    binding->arg_type = LV_EX_DATA_STRING;
    binding->setter = (void *)label_set_text;
    p_app_message_ctx->title_handle = lv_ex_bind_data(p_app_message_ctx->title, binding);
    return label;
}

static lv_obj_t *message_title_builder(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 270, 40);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 135, 75);
    char *title = replace_nbsp(p_app_message_ctx->title->value.s);
    lv_label_set_text(label, title);
    lv_mem_free(title);
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_update_layout(label);
    return label;
}

static lv_obj_t *standby_message_content_builder(lv_obj_t *parent, lv_ex_binding_t *binding)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    lv_obj_set_size(label, 300, 100);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(label, " ");
    lv_color_t color = lv_color_white();
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    RT_ASSERT(p_app_message_ctx->content);
    binding->target = label;
    binding->arg_type = LV_EX_DATA_STRING;
    binding->setter = (void *)label_set_text;
    p_app_message_ctx->content_handle = lv_ex_bind_data(p_app_message_ctx->content, binding);
    return label;
}

static lv_obj_t *message_content_builder(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, 350); // 350
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10);
    char *content = replace_nbsp(p_app_message_ctx->content->value.s);
    lv_label_set_text(label, content);
    lv_mem_free(content);
    lv_color_t color = lv_color_white();
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_update_layout(label);
    return label;
}

static lv_obj_t *reply_btn_builder(lv_obj_t *parent)
{
    reply_btn = lv_obj_create(parent);
    lv_obj_set_size(reply_btn, 366, 100);
    lv_obj_set_style_radius(reply_btn, 50, 0);
    lv_obj_set_style_bg_color(reply_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(reply_btn, LV_OPA_10, 0);
    lv_obj_set_style_shadow_width(reply_btn, 30, 0);                     // 設置陰影寬度為 10 像素
    lv_obj_set_style_shadow_color(reply_btn, lv_color_hex(0xFFFFFF), 0); // 設置陰影顏色
    lv_obj_set_style_shadow_opa(reply_btn, LV_OPA_0, 0);                 // 設置陰影透明度
    lv_obj_t *reply_btn_label = lv_label_create(reply_btn);
    lv_label_set_text(reply_btn_label, "回覆");
    lv_obj_align(reply_btn_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(reply_btn_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(reply_btn_label, lv_color_hex(0xFFFFFF), 0);
    return reply_btn;
}

static lv_obj_t *ignore_btn_builder(lv_obj_t *parent)
{
    lv_obj_t *ignore_btn = lv_obj_create(parent);
    lv_obj_set_size(ignore_btn, 326, 100);
    lv_obj_set_style_radius(ignore_btn, 50, 0);
    lv_obj_set_style_bg_color(ignore_btn, lv_color_hex(0xB3B3B3), 0);
    lv_obj_set_style_bg_opa(ignore_btn, LV_OPA_10, 0);
    lv_obj_t *ignore_btn_label = lv_label_create(ignore_btn);
    lv_label_set_text(ignore_btn_label, "返回忽略");
    lv_obj_align(ignore_btn_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(ignore_btn_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(ignore_btn_label, lv_color_hex(0xFFFFFF), 0);
    return ignore_btn;
}

lv_obj_t *app_message_init(lv_obj_t *parent)
{
    standby_widget_lock = false;
    lv_ex_binding_t binding;
    lv_obj_t *p_window = common_black_bg(parent);

    tileview_page[0] = lv_tileview_create(p_window);
    lv_obj_set_scrollbar_mode(tileview_page[0], LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(tileview_page[0], lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(tileview_page[0], LV_OPA_0, 0);
    lv_obj_set_size(tileview_page[0], LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_add_event_cb(tileview_page[0], message_view_event_cb, LV_EVENT_ALL, NULL);
    tileview_page[1] = lv_tileview_add_tile(tileview_page[0], 0, 1, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(tileview_page[1], LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(tileview_page[1], lv_color_hex(0xEA0000), 0);
    lv_obj_set_style_bg_opa(tileview_page[1], LV_OPA_0, 0);
    lv_obj_set_size(tileview_page[1], LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_add_event_cb(tileview_page[1], message_list_view_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_pos(tileview_page[1], 0, LV_VER_RES_MAX);
    /*
     * Create a small red circular badge with a white '✕' (cross) on top of
     * tileview_page[1] near the top. This serves as the requested red dot
     * with a cross displayed on it.
     */
    notif_badge = lv_obj_create(tileview_page[1]);
    lv_obj_set_size(notif_badge, 50, 50);
    lv_obj_set_style_radius(notif_badge, 50, 0);
    lv_obj_set_style_bg_color(notif_badge, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(notif_badge, LV_OPA_10, 0);
    /* position near top center; adjust y offset (e.g., 20) as needed */
    lv_obj_align(notif_badge, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_clear_flag(notif_badge, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(notif_badge, ingore_btn_event_callback, LV_EVENT_CLICKED, NULL);
    /* Add a white '✕' label centered in the badge */
    lv_obj_t *notif_label = lv_label_create(notif_badge);
    lv_label_set_text(notif_label, "X");
    lv_obj_set_style_text_color(notif_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(notif_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_center(notif_label);
    // lv_obj_set_scroll_dir(tileview_page[1], LV_DIR_VER);
    lv_obj_t *message_widget = lv_obj_create(tileview_page[1]);
    lv_obj_set_size(message_widget, 366, 150);
    lv_obj_set_style_radius(message_widget, 30, 0);
    lv_obj_set_style_bg_color(message_widget, lv_color_hex(0x808080), 0);
    lv_obj_set_style_bg_opa(message_widget, LV_OPA_10, 0);
    lv_obj_set_pos(message_widget, 50, 135);
    lv_obj_t *message_title = message_title_builder(tileview_page[1]);
    lv_obj_t *message_lable = message_content_builder(message_widget);
    LOG_D("message_title %d", lv_obj_get_height(message_title));
    if (lv_obj_get_height(message_lable) + 35 > 100) // + lv_obj_get_height(message_title)
    {
        lv_obj_set_height(message_widget, lv_obj_get_height(message_lable) + 35); // + lv_obj_get_height(message_title)
    }
    else
    {
        lv_obj_set_height(message_widget, 100);
    }
    lv_obj_t *ignore_btn = NULL;

    reply_lock = true;
    if (messagr_can_reply)
    {
        reply_lock = true;
        lv_obj_t *reply_btn = reply_btn_builder(tileview_page[1]);
        lv_obj_add_event_cb(reply_btn, reply_btn_event_callback, LV_EVENT_CLICKED, NULL);
        lv_obj_align_to(reply_btn, message_widget, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
        // lv_obj_t *ignore_btn = ignore_btn_builder(tileview_page[1]);
        // lv_obj_add_event_cb(ignore_btn, ingore_btn_event_callback, LV_EVENT_CLICKED, NULL);
        // lv_obj_align_to(ignore_btn, reply_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
        lv_obj_t *message_flyleaf = lv_obj_create(tileview_page[1]);
        lv_obj_set_size(message_flyleaf, 366, 70);
        lv_obj_align_to(message_flyleaf, reply_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
        lv_obj_set_style_bg_opa(message_flyleaf, LV_OPA_0, 0);
    }
    else
    {
        lv_obj_t *message_flyleaf = lv_obj_create(tileview_page[1]);
        lv_obj_set_size(message_flyleaf, 366, 70);
        lv_obj_align_to(message_flyleaf, message_widget, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
        lv_obj_set_style_bg_opa(message_flyleaf, LV_OPA_0, 0);
        // ignore_btn = ignore_btn_builder(tileview_page[1]);
        // lv_obj_add_event_cb(ignore_btn, ingore_btn_event_callback, LV_EVENT_CLICKED, NULL);
        // lv_obj_align_to(ignore_btn, message_widget, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    }

    icon_standby = app_icon_builder(p_window, app_message_get_app_index());
    lv_obj_align(icon_standby, LV_ALIGN_TOP_MID, 0, 70);
    message_title_standby = standby_message_title_builder(p_window, &binding);
    message_widget_standby = standby_message_content_builder(p_window, &binding);
    lv_obj_align_to(message_widget_standby, message_title_standby, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
    // if (SkaiWatchSys.pre_hcpu_wakeup_tick + 2000 < rt_tick_get())
    // {
    //     // lv_obj_set_tile_id(tileview_page[0], 0, 1, LV_ANIM_ON);
    //     wait_for_message_timer_create();
    //     LOG_D("on_watch_wakeup_time");
    // }
    if (open_from_message_list)
    {
        scroll_page_goin_message();
    }
    // lv_obj_set_style_shadow_width(message_widget, 30, 0);                     // 設置陰影寬度為 10 像素
    // lv_obj_set_style_shadow_color(message_widget, lv_color_hex(0xFFFFFF), 0); // 設置陰影顏色
    // lv_obj_set_style_shadow_opa(message_widget, LV_OPA_0, 0);                 // 設置陰影透明度

    // lv_obj_t *reply_btn = common_button(p_window);
    // icon_btn_send = lv_img_create(reply_btn);
    // lv_img_set_src(icon_btn_send, &icon_reply);
    // lv_obj_set_style_img_opa(icon_btn_send, LV_OPA_40, 0);
    // lv_obj_align(reply_btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    // lv_obj_add_event_cb(reply_btn, reply_btn_event_callback, LV_EVENT_ALL, NULL);
    return p_window;
}

static void on_start(void)
{
    RT_ASSERT(NULL == p_app_message);
    p_app_message = (app_message_t *)lv_mem_alloc(sizeof(app_message_t));
    memset(p_app_message, 0, sizeof(app_message_t));
    p_app_message->main_window = app_message_init(lv_scr_act());
    cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);
}

static void on_resume(void)
{
#ifdef BSP_USING_UI_HANDLER
    if (messagr_can_reply)
    {
        lvgl_msg_handler.handle_tap_indicator = gesture_tap_event_handler;
        lvgl_msg_handler.handle_send_message = double_tap_event_handler;
        // switch_watch_motion_control_mode(true, false);
    }
    lvgl_msg_handler.handle_ungrab_event = scroll_page_goin_message;
    lvgl_msg_handler.handle_switch_selected = switch_selected_message;
    lvgl_msg_handler.handle_open_message_page_content = scroll_page_goin_message;
#endif
}

static void on_pause(void)
{
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_tap_event = NULL;
    lvgl_msg_handler.handle_tap_indicator = NULL;
    lvgl_msg_handler.handle_send_message = NULL;
    lvgl_msg_handler.handle_switch_selected = NULL;
    lvgl_msg_handler.handle_open_message_page_content = NULL;
    if (lvgl_msg_handler.handle_ungrab_event == scroll_page_goin_message)
    {
        lvgl_msg_handler.handle_ungrab_event = NULL;
    }
#endif
}

static void on_stop(void)
{
    LOG_D("message_on_stop");
    if (p_app_message_ctx->content_handle)
    {
        lv_ex_unbind_data(p_app_message_ctx->content, p_app_message_ctx->content_handle);
    }
    if (p_app_message_ctx->title_handle)
    {
        lv_ex_unbind_data(p_app_message_ctx->title, p_app_message_ctx->title_handle);
    }
    if (p_app_message_ctx->app_name_handle)
    {
        lv_ex_unbind_data(p_app_message_ctx->app_name, p_app_message_ctx->app_name_handle);
    }
    if (p_app_message)
    {
        if (p_app_message->main_window && lv_obj_is_valid(p_app_message->main_window))
        {
            lv_obj_del(p_app_message->main_window);
            p_app_message->main_window = NULL;
        }
        lv_mem_free(p_app_message);
        p_app_message = NULL;
    }

    if (open_from_message_list)
    {
        open_from_message_list = false;
    }
}

static void main_msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start();
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

static void new_msg_popup_event_callback(lv_event_t *event)
{
    rt_kprintf("new_msg_popup_event_callback %s", lv_event_to_name(event->code));
    if (LV_EVENT_READY == event->code)
    {
        // Entry message main page
        gui_app_create_page("main", main_msg_handler);
        gui_app_remove_page("popup");
    }
    else if (LV_EVENT_CANCEL == event->code)
    {
        gui_app_goback();
    }
}

static void popupmsg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
    {
        const char *name = intent_get_string(gui_app_get_intent(), "newfrom");

        lv_obj_t *pop_container = lv_lvsfpopup_create(lv_scr_act());

        lv_lvsfpopup_set_icon(pop_container, IMG_MAIL);
        lv_lvsfpopup_set_title(pop_container, "New msg from:");
        lv_lvsfpopup_set_content(pop_container, name);

        lv_lvsfpopup_set_confirm_btn_txt(pop_container, "View");
        lv_lvsfpopup_set_cancel_btn_txt(pop_container, "Ignore");
        lv_obj_align(pop_container, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_event_cb(pop_container, new_msg_popup_event_callback, LV_EVENT_ALL, NULL);
    }
    break;

    case GUI_APP_MSG_ONRESUME:
        break;

    case GUI_APP_MSG_ONPAUSE:
        break;

    case GUI_APP_MSG_ONSTOP:
        break;
    default:
        break;
    }
}

static int app_message_main(intent_t i)
{
    const char *name = intent_get_string(i, "newfrom");
    LOG_D("app_message_main %s", name);
    if (name)
    {
        // Display new message popup page
        gui_app_create_page("popup", popupmsg_handler);
    }
    else
    {
        // Entry message main page
        // gui_app_regist_msg_handler(APP_ID_MESSAGE, msg_handler);
        const char *id = intent_get_string(i, "noti_id");
        if (id)
        {
            strncpy(notification.id, id, NOTIFICATION_ID_LEN);
        }
        LOG_D("temp_noti_id %s", notification.id);
        gui_app_create_page("main", main_msg_handler);
        const char *can_reply = intent_get_string(i, "noti_can_reply");
        if (can_reply)
        {
            messagr_can_reply = (strcmp(can_reply, "true") == 0);
        }
    }
    return 0;
}

void open_message_app(const char *notification_id)
{
    strncpy(notification.id, notification_id, NOTIFICATION_ID_LEN);
    LOG_D("temp_noti_id %s", notification.id);
    on_start();
    on_resume();
}

void close_message_app(void)
{
    on_pause();
    on_stop();
}

void app_message_data_init(void)
{
    if (!p_app_message_ctx)
    {
        p_app_message_ctx = (app_message_ctx_t *)rt_malloc(sizeof(app_message_ctx_t));
        memset(p_app_message_ctx, 0, sizeof(app_message_ctx_t));
    }

    p_app_message_ctx->title = lv_ex_data_create("msg.title", LV_EX_DATA_STRING);
    RT_ASSERT(p_app_message_ctx->title);
    p_app_message_ctx->content = lv_ex_data_create("msg.content", LV_EX_DATA_STRING);
    RT_ASSERT(p_app_message_ctx->content);
    p_app_message_ctx->app_name = lv_ex_data_create("msg.app", LV_EX_DATA_STRING);
    RT_ASSERT(p_app_message_ctx->app_name);
}

void app_message_set_app_name(const uint8_t *s)
{
    if (p_app_message_ctx->app_name)
    {
#if 0
        if ((0 == strncmp("\xE4\xBF\xA1\xE6", (char *)s, strlen("\xE4\xBF\xA1\xE6")))
                || (0 == strncmp("Message", (char *)s, strlen("Message"))))
        {
            lv_ex_data_set_value(p_app_message_ctx->app_name, (void *)"SMS");
        }
        else if ((0 == strncmp("\xe7\x94\xb5\xe8", (char *)s, strlen("\xe7\x94\xb5\xe8")))
                 || (0 == strncmp("Phone", (char *)s, strlen("Phone"))))
        {
            lv_ex_data_set_value(p_app_message_ctx->app_name, (void *)"Phone");
        }
        else
        {
            lv_ex_data_set_value(p_app_message_ctx->app_name, (void *)"WeChat");
        }
#else
        lv_ex_data_set_value(p_app_message_ctx->app_name, (void *)s);
#endif
    }
}

void app_message_set_title(const uint8_t *s)
{
    if (p_app_message_ctx->title)
    {
        lv_ex_data_set_value(p_app_message_ctx->title, (void *)s);
    }
}

void app_message_set_content(const uint8_t *s)
{
    if (p_app_message_ctx->content)
    {
        lv_ex_data_set_value(p_app_message_ctx->content, (void *)s);
    }
}

static uint8_t _app_index = 0;
void app_message_set_app_index(uint8_t index)
{
    _app_index = index;
}
uint8_t app_message_get_app_index(void)
{
    return _app_index;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(message), IMG_MAIL, APP_ID_MESSAGE, app_message_main);
#endif
