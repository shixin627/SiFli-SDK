#include "app_clock_status_bar.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
    #include "watch_system_core_task.h"
#endif
#ifdef BSP_USING_BLOC_NOTIFY
    #include "bloc_notification.h"
#endif
#ifdef BSP_USING_BLOC_CONTROL
    #include "bloc_control.h"
#endif
#ifdef BSP_USING_BLOC_SETTING
    #include "bloc_setting.h"
#endif
#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
#endif
#include "ui_helper.h"
#include "ui_img_helper.h"
#include "power_manager_service.h"
#include "ui_datasrv_subscriber.h"
#include "watch_global_data.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include "bloc_peripheral.h"
#include "bloc_motion_tracking.h"
#include "communicate_protocol.h"
#include "bloc_v2t.h"
#include "ble_device_manager.h"
#include "ble_hid.h"

extern void refresh_connected_device_label(void);
__attribute__((weak)) void refresh_connected_device_label(void)
{
}

#define DBG_TAG "app.clock.status_bar"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define APP_ID "Main"

// 狀態欄區域標識
typedef enum
{
    STATUS_BAR_AREA_LEFT = 0,
    STATUS_BAR_AREA_UP = 1,
    STATUS_BAR_AREA_DOWN = 2,
    STATUS_BAR_AREA_RIGHT = 3
} status_bar_area_t;

// 控制中心按鈕選擇索引
typedef enum
{
    CONTROL_BTN_MEDIA_PREV = 0,
    CONTROL_BTN_MEDIA_PLAY_PAUSE = 1,
    CONTROL_BTN_MEDIA_NEXT = 2,
    CONTROL_BTN_CALCULATOR = 3,
    CONTROL_BTN_FLASHLIGHT = 4,
    CONTROL_BTN_RECORDER = 5
} control_button_index_t;

LV_IMG_DECLARE(bluetooth_broadcasting);
LV_IMG_DECLARE(icon_qrcode);
LV_IMG_DECLARE(icon_dnd_mode);
LV_IMG_DECLARE(sun);
LV_IMG_DECLARE(img_media_play);
LV_IMG_DECLARE(img_media_pause);
LV_IMG_DECLARE(icon_release);
LV_IMG_DECLARE(mouse_mode_icon);
LV_IMG_DECLARE(icon_tap);
LV_IMG_DECLARE(micro_icon);
LV_IMG_DECLARE(flashlight_icon);
LV_IMG_DECLARE(img_settings);
LV_IMG_DECLARE(find_phone);
LV_IMG_DECLARE(img_logo);
LV_IMG_DECLARE(device_btn);

#define NOTIFICATION_ITEM_WIDTH 360
#define NOTIFICATION_ITEM_HEIGHT 90
#define NOTIFICATION_ITEM_PAD_ROW 30
#define NOTIFICATION_ITEM_PAD_LEFT ((LV_HOR_RES - NOTIFICATION_ITEM_WIDTH) / 2)

#define APP_MAIN_COLOR lv_color_hex(0xCECECE)
#define DEV_CHANGE_BUTTON_WIDTH 202
#define DEV_CHANGE_BUTTON_HEIGHT 102
#define DEV_CHANGE_BUTTON_GAP 20
#define DEV_CHANGE_CONTENT_SIDE_MARGIN 20
#define DEV_CHANGE_CONTENT_HEIGHT (LV_VER_RES_MAX)

static lv_obj_t *app_clock_main_status_bar;
static lv_obj_t *app_clock_main_status_bar_down;
static lv_obj_t *app_clock_ai_status_bar;
static lv_obj_t *app_clock_device_change_bar;
static lv_obj_t *device_change_bar_area_right;
static lv_obj_t *status_bar_area_up;
static lv_obj_t *status_bar_area_down;
static lv_obj_t *status_bar_area_left;
static lv_obj_t *status_bar_area_right;

static rt_bool_t bt_enabled = RT_TRUE;
static rt_bool_t dndmode_enabled = RT_FALSE;
static rt_bool_t alarm_enabled = RT_TRUE;
static rt_bool_t timer_enabled = RT_TRUE;

static const lv_btnmatrix_ctrl_t btnm_ctrl_map[] = {
    1 | LV_BTNMATRIX_CTRL_CHECKABLE,
    1 | LV_BTNMATRIX_CTRL_CHECKABLE,
    1 | LV_BTNMATRIX_CTRL_CHECKABLE,
    1 | LV_BTNMATRIX_CTRL_CHECKABLE,
};

void display_status_bar_area(uint32_t idx, bool display)
{
    if (idx >= 4)
    {
        LOG_E("Invalid index %d for status bar area", idx);
        return;
    }
    lv_obj_t *status_bar_area_objs[] = {
        status_bar_area_up,
        status_bar_area_down,
        status_bar_area_left,
        status_bar_area_right,
    };
    if (lv_obj_is_valid(status_bar_area_objs[idx]))
    {
        if (display)
            lv_obj_clear_flag(status_bar_area_objs[idx], LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(status_bar_area_objs[idx], LV_OBJ_FLAG_HIDDEN);
    }
}

static lv_obj_t *gaus_dial_bg = NULL;
static lv_obj_t *gaus_dial_img = NULL;
static lv_obj_t *dev_change_gaus_bg = NULL;
static lv_obj_t *dev_change_gaus_img = NULL;
static void set_clock_main_status_opa(uint8_t opa, bool mask);
static void set_dev_change_gaus_opa(uint8_t opa);
static void notification_status_bar_cb(lv_event_t *event)
{
    if (lv_disp_get_rotation(NULL) != LV_DISP_ROT_90 &&
        lv_disp_get_rotation(NULL) != LV_DISP_ROT_270)
    {
        lv_obj_t *obj = lv_event_get_target(event);
        status_bar_area_t area_id =
            (status_bar_area_t)lv_obj_get_user_data(obj);

        if (LV_EVENT_RELEASED == event->code)
        {
            LOG_I("LV_EVENT_RELEASED_Clock from area: %d", area_id);
            // 根據不同區域執行不同操作
            switch (area_id)
            {
            case STATUS_BAR_AREA_LEFT:
                LOG_D("Released from LEFT area");
                break;
            case STATUS_BAR_AREA_UP:
                LOG_D("Released from TOP area");
                break;
            case STATUS_BAR_AREA_DOWN:
                LOG_D("Released from BOTTOM area");
                break;
            case STATUS_BAR_AREA_RIGHT:
                LOG_D("Released from RIGHT area");
                break;
            }
            lv_obj_add_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
        }
        else if (LV_EVENT_PRESSED == event->code)
        {
            if (is_ble_dfu_thread_running())
            {
                LOG_I("notification_status_bar_cb in ble dfu => return");
                return;
            }
            LOG_I("notification_status_bar_cb from area: %d", area_id);
            // 根據不同區域執行不同操作
            switch (area_id)
            {
            case STATUS_BAR_AREA_LEFT:
                LOG_D("Pressed from LEFT area");
                break;
            case STATUS_BAR_AREA_UP:
                LOG_D("Pressed from TOP area");
                break;
            case STATUS_BAR_AREA_DOWN:
                LOG_D("Pressed from BOTTOM area");
                break;
            case STATUS_BAR_AREA_RIGHT:
                LOG_D("Pressed from RIGHT area");
                break;
            }
            lv_obj_set_tile_id(app_clock_main_status_bar, 1, 1, false);
            lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
            set_clock_main_status_opa(0, false);
        }
    }
}

static void dev_change_refresh_device_list(void);
static void dev_change_stop_connecting_timer(void);

static void device_change_bar_cb(lv_event_t *event)
{
    if (lv_disp_get_rotation(NULL) != LV_DISP_ROT_90 &&
        lv_disp_get_rotation(NULL) != LV_DISP_ROT_270)
    {
        if (LV_EVENT_RELEASED == event->code)
        {
            lv_obj_add_flag(app_clock_device_change_bar, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(dev_change_gaus_bg, LV_OBJ_FLAG_HIDDEN);
            set_dev_change_gaus_opa(LV_OPA_0);
            dev_change_stop_connecting_timer();
        }
        else if (LV_EVENT_PRESSED == event->code)
        {
            if (is_ble_dfu_thread_running())
            {
                LOG_I("app_clock_device_change_bar in ble dfu => return");
                return;
            }
            lv_obj_set_tile_id(app_clock_device_change_bar, 0, 0, false);
            lv_obj_clear_flag(app_clock_device_change_bar, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(dev_change_gaus_bg, LV_OBJ_FLAG_HIDDEN);
            dev_change_refresh_device_list();
        }
    }
}

static void ai_status_bar_cb(lv_event_t *event)
{
    if (lv_disp_get_rotation(NULL) != LV_DISP_ROT_90 &&
        lv_disp_get_rotation(NULL) != LV_DISP_ROT_270)
    {
        if (LV_EVENT_RELEASED == event->code)
        {
            lv_obj_add_flag(app_clock_ai_status_bar, LV_OBJ_FLAG_HIDDEN);
        }
        else if (LV_EVENT_PRESSED == event->code)
        {
            if (is_ble_dfu_thread_running())
            {
                LOG_I("ai_status_bar_cb in ble dfu => return");
                return;
            }
            if (!get_bluetooth_connection_status())
            {
                create_connection_tips();
                LOG_D(
                    "Bluetooth is connected, ignoring voice recognition event");
                return;
            }
            lv_obj_set_tile_id(app_clock_ai_status_bar, 0, 0, false);
            // lv_obj_clear_flag(app_clock_ai_status_bar, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

extern void check_main_page(void);
extern bool gesture_tap_collection;
extern void widget_page_flip(bool is_gravity_x_positive);
bool main_clock_tileview_scrollable = true;
static void button_selection(gesture_position_t gesture_position);
static void reset_tools_selection(void);
static void press_event(uint8_t press);
static uint8_t shady_transparency = 0;
static uint16_t bg_opa = LV_OPA_COVER;
static uint16_t bg_opa_2 = LV_OPA_COVER;
static uint16_t bg_opa_3 = LV_OPA_50;
static uint8_t middle_layer_tileview_index = 255;
static uint8_t ai_interface_tileview_index = 0;

static void app_clock_main_status_bar_event_cb(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);

    // if (LV_EVENT_PRESSING != event->code)
    //     LOG_D("app_clock_main_status_bar_event_cb %p, got event %s", obj,
    //     lv_event_to_name(event->code));
    switch (event->code)
    {
    case LV_EVENT_SCROLL:
    {
        lv_coord_t scroll_y = (466 - lv_obj_get_scroll_y(obj)) * bg_opa / 350;
        lv_coord_t scroll_x = (466 - lv_obj_get_scroll_x(obj)) * bg_opa / 350;
        if (scroll_x > bg_opa)
            scroll_x = bg_opa;
        else if (scroll_x < 0)
            scroll_x = 0;
        if (scroll_y > bg_opa)
            scroll_y = bg_opa;
        else if (scroll_y < -bg_opa)
            scroll_y = -bg_opa;

        lv_coord_t scroll_second_y =
            (466 - lv_obj_get_scroll_y(obj)) * bg_opa_2 / 466;
        lv_coord_t scroll_second_x =
            (466 - lv_obj_get_scroll_x(obj)) * bg_opa_2 / 466;

        if (scroll_second_y > bg_opa_2)
            scroll_second_y = bg_opa_2;
        else if (scroll_second_y < -bg_opa_2)
            scroll_second_y = -bg_opa_2;
        if (scroll_second_x > bg_opa_2)
            scroll_second_x = bg_opa_2;
        else if (scroll_second_x < -bg_opa_2)
            scroll_second_x = -bg_opa_2;

        if (((abs(scroll_y) < (bg_opa + 1)) && (scroll_y != 0)) ||
            ((abs(scroll_x) < (bg_opa + 1)) && (scroll_x != 0))) // 230
        {
            if (scroll_y == 0)
            {
                shady_transparency = abs(scroll_x);
                // lv_obj_set_style_bg_opa(
                //     myLancher[app_index_message].pagetileview,
                //     shady_transparency, 0);
                set_clock_main_status_opa(shady_transparency, false);
            }
            else
            {
                shady_transparency = abs(scroll_y);
                // lv_obj_set_style_bg_opa(
                //     myLancher[app_index_message].pagetileview,
                //     shady_transparency, 0);
                set_clock_main_status_opa(shady_transparency, true);
            }
            // LOG_D("scroll_y: %d, scroll_x: %d, shady_transparency: %d",
            //       scroll_y, scroll_x, shady_transparency);
        }
        if (scroll_second_y == 0)
        {
            if (scroll_second_x > 0)
            {
                shady_transparency = scroll_second_x;
                if (shady_transparency < (bg_opa_2 + 1) &&
                    shady_transparency > 0)
                {
                    set_instruction_list_time_opa(shady_transparency);
                    // set_instruction_list_time_bg_opa(shady_transparency);
                }
            }
            else
            {
                shady_transparency = -scroll_second_x;
                set_instruction_list_battery_opa(shady_transparency);
                // set_instruction_list_battery_bg_opa(shady_transparency);
            }
        }
        else
        {
            shady_transparency = abs(scroll_second_y);
            // if (scroll_second_y > 0)
            // {
            //     if (shady_transparency < (bg_opa_2 + 1) && shady_transparency
            //     > 0)
            //     {
            //         set_instruction_list_time_opa(shady_transparency);
            //         set_instruction_list_time_bg_opa(shady_transparency);
            //     }
            //     else if (shady_transparency >= bg_opa_2)
            //     {
            //         set_instruction_list_time_opa(bg_opa_2);
            //         set_instruction_list_time_bg_opa(bg_opa_2);
            //     }
            // }
            // else
            {
                if (shady_transparency < (bg_opa_2 + 1) &&
                    shady_transparency > 0)
                {
                    set_instruction_list_battery_opa(shady_transparency);
                    // set_instruction_list_battery_bg_opa(shady_transparency);
                }
                else if (shady_transparency >= bg_opa_2)
                {
                    set_instruction_list_battery_opa(bg_opa_2);
                    // set_instruction_list_battery_bg_opa(bg_opa_2);
                }
            }
        }

        // if (shady_transparency < (bg_opa_2 + 1) && shady_transparency > 0)
        // {
        //     lv_obj_set_style_img_opa(get_instruction_list_bluetooth_disconnection(),
        //     bg_opa_2 - shady_transparency, 0);
        // }
        // else if (shady_transparency >= bg_opa_2)
        // {
        //     lv_obj_set_style_img_opa(get_instruction_list_bluetooth_disconnection(),
        //     LV_OPA_COVER, 0);
        // }

        lv_coord_t scroll_third_y =
            (466 - lv_obj_get_scroll_y(obj)) * bg_opa_3 / 466;
        lv_coord_t scroll_third_x = lv_obj_get_scroll_x(obj) * bg_opa_3 / 466;
        if (scroll_second_y == 0)
        {
            shady_transparency = scroll_second_x;
        }
        else
        {
            shady_transparency = scroll_second_y;
        }
        // }
        break;
    }
    case LV_EVENT_VALUE_CHANGED:
    {
        rt_uint32_t active_pos = (rt_uint32_t)lv_event_get_param(event);
        lv_coord_t scroll_y = lv_obj_get_scroll_y(obj);
        lv_coord_t scroll_x = lv_obj_get_scroll_x(obj);

        if (abs(scroll_x) % 466 != 0 || abs(scroll_y) % 466 != 0)
        {
            break;
        }
        if (active_pos == 1 &&
            !lv_obj_has_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN) &&
            lv_obj_get_scroll_x(myLancher[app_index_message].pagetileview) ==
                466)
        {
            lv_obj_add_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
        }
        if (middle_layer_tileview_index == active_pos)
        {
            check_main_page();
            return;
        }

        middle_layer_tileview_index = active_pos;
        if (gui_app_is_actived("Main"))
        {
            if (active_pos != 1)
            {
                main_clock_tileview_scrollable = false;
            }
            else
            {
                main_clock_tileview_scrollable = true;
            }
        }
        // if (1 == active_pos)
        if (scroll_y == 466 && scroll_x == 466)
        {
            shady_transparency = 0;
            // lv_obj_set_style_bg_opa(myLancher[app_index_message].pagetileview,
            //                         shady_transparency, 0);
            set_clock_main_status_opa(shady_transparency, false);
            lv_obj_add_flag(myLancher[app_index_message].pagetileview,
                            LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
            set_clock_main_status_opa(LV_OPA_0, false);
            set_instruction_list_time_opa(LV_OPA_0);
            // if (lv_obj_is_valid(get_instruction_list_time_bg()))
            //     lv_obj_set_style_bg_opa(get_instruction_list_time_bg(),
            //     LV_OPA_0, 0);

#ifdef APP_ID_WIDGETS
            widget_page_flip(false);
#endif

            set_instruction_list_battery_opa(LV_OPA_TRANSP);
            // lv_obj_set_style_bg_opa(get_instruction_list_battery_bg(),
            // LV_OPA_TRANSP,
            //                         0);
        }
        else
        {
            lvgl_msg_t msg;
            // msg.type = LVGL_MSG_TYPE_CLEAR_NOTIFICATION_BAR_INDICATOR;
            // lvgl_send_msg(msg);

            lv_obj_clear_flag(myLancher[app_index_message].pagetileview,
                              LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
            if (active_pos ==
                MAIN_PAGE_TYPE_LEFT) // || active_pos == MAIN_PAGE_TYPE_UP
            {
                // if (lv_obj_is_valid(get_instruction_list_time_bg()))
                //     lv_obj_set_style_bg_opa(get_instruction_list_time_bg(),
                //     LV_OPA_80,
                //                             0);
                set_instruction_list_time_opa(LV_OPA_100);
            }
            else
            {
                // if (lv_obj_is_valid(get_instruction_list_time_bg()))
                //     lv_obj_set_style_bg_opa(get_instruction_list_time_bg(),
                //     LV_OPA_0,
                //                             0);
                set_instruction_list_time_opa(LV_OPA_0);
            }
            // lv_obj_set_style_bg_opa(myLancher[app_index_message].pagetileview,
            //                         LV_OPA_80, 0);
            if (active_pos == MAIN_PAGE_TYPE_UP)
            {
                set_clock_main_status_opa(LV_OPA_100, true);
            }
            else
            {
                set_clock_main_status_opa(LV_OPA_100, false);
            }
            if (active_pos == 0)
            {
                set_instruction_list_battery_opa(LV_OPA_100);
            }

#ifdef APP_ID_WIDGETS
            widget_page_flip(true);
#endif

            extern void reset_gravity_position(void);
            reset_gravity_position();
        }
        check_main_page();
        break;
    }
    case LV_EVENT_PRESSED:
    {
        LOG_D("LV_EVENT_PRESSED_Clock");
        break;
    }
    case LV_EVENT_RELEASED:
    case LV_EVENT_SHORT_CLICKED:
    case LV_EVENT_LONG_PRESSED:
    case LV_EVENT_CLICKED:
    case LV_EVENT_FOCUSED:
    default:
        // printf("Released");

        break;
    }
}

static void app_clock_ai_status_bar_event_cb(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);
    switch (event->code)
    {
    case LV_EVENT_SCROLL:
    {
    }
    break;
    case LV_EVENT_VALUE_CHANGED:
    {
        rt_uint32_t active_pos = (rt_uint32_t)lv_event_get_param(event);
        LOG_D("LV_EVENT_VALUE_CHANGED_AI %d", active_pos);
        if (ai_interface_tileview_index == active_pos)
        {
            return;
        }
        ai_interface_tileview_index = active_pos;
        if (active_pos == 0)
        {
            lv_obj_add_flag(app_clock_ai_status_bar, LV_OBJ_FLAG_HIDDEN);
            switch_watch_motion_control_mode(false, false);
        }
        else if (active_pos == 1)
        {
            lv_obj_clear_flag(app_clock_ai_status_bar, LV_OBJ_FLAG_HIDDEN);
            switch_watch_motion_control_mode(true, true);
        }
        check_is_at_ai_interface();
    }
    break;
    default:
        break;
    }
}

static void app_clock_device_change_bar_event_cb(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);
    switch (event->code)
    {
    case LV_EVENT_SCROLL:
    {
        // Animate gaussian blur opacity based on horizontal scroll
        lv_coord_t scroll_x = lv_obj_get_scroll_x(obj);
        // LOG_D("Device change bar scroll_x: %d", scroll_x);
        uint8_t opa = 0;
        if (scroll_x > 0)
        {
            if (scroll_x > 350)
                scroll_x = 350;
            opa = (scroll_x * LV_OPA_COVER) / 350;
        }
        set_dev_change_gaus_opa(opa);
    }
    break;
    case LV_EVENT_VALUE_CHANGED:
    {
        rt_uint32_t active_pos = (rt_uint32_t)lv_event_get_param(event);
        LOG_D("LV_EVENT_VALUE_CHANGED_DEVICE_CHANGE %d", active_pos);
        if (active_pos == 0)
        {
            lv_obj_add_flag(app_clock_device_change_bar, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(dev_change_gaus_bg, LV_OBJ_FLAG_HIDDEN);
            set_dev_change_gaus_opa(LV_OPA_0);
            dev_change_stop_connecting_timer();
        }
        else if (active_pos == 1)
        {
            lv_obj_clear_flag(app_clock_device_change_bar, LV_OBJ_FLAG_HIDDEN);
            set_dev_change_gaus_opa(LV_OPA_COVER);
            dev_change_refresh_device_list();
        }
    }
    break;
    default:
        break;
    }
}

uint8_t get_middle_layer_tileview_index(void)
{
    return middle_layer_tileview_index;
}

static void app_clock_main_status_bar_down_event_cb(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);

    // if (LV_EVENT_PRESSING != event->code)
    //     LOG_D("app_clock_main_status_bar_event_cb %p, got event %s", obj,
    //     lv_event_to_name(event->code));
    switch (event->code)
    {
    case LV_EVENT_RELEASED:
    {
        if (lv_obj_get_scroll_x(myLancher[app_index_message].pagetileview) ==
                466 &&
            lv_obj_get_scroll_y(myLancher[app_index_message].pagetileview) ==
                466)
        {
            lv_obj_add_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
        }
        break;
    }
    default:
        // printf("Released");

        break;
    }
}

static void app_clock_main_ai_status_bar_event_cb(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);

    switch (event->code)
    {
    case LV_EVENT_RELEASED:
    {
        LOG_D("LV_EVENT_RELEASED_Clock");
        lv_obj_add_flag(app_clock_ai_status_bar, LV_OBJ_FLAG_HIDDEN);
        break;
    }
    default:
        break;
    }
}

static void app_clock_main_device_change_bar_event_cb(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);

    switch (event->code)
    {
    case LV_EVENT_RELEASED:
    {
        LOG_D("LV_EVENT_RELEASED_Clock");
        lv_obj_add_flag(app_clock_device_change_bar, LV_OBJ_FLAG_HIDDEN);
        break;
    }
    default:
        break;
    }
}

void animate_to_notification_center(void)
{
    // set_scroll_anim_time(true);
    reset_tools_selection();
    lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(myLancher[app_index_message].pagetileview,
                      LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_tile_id(myLancher[app_index_message].pagetileview, 0, 1,
                       LV_ANIM_ON);
    // set_scroll_anim_time(false);
}

void animate_to_home_from_notification_center(void)
{
    if (lv_obj_is_valid(myLancher[app_index_message].pagetileview))
        lv_obj_set_tile_id(myLancher[app_index_message].pagetileview, 1, 1,
                           LV_ANIM_ON);
}

void animate_to_instruction_list(void)
{
    set_scroll_anim_time(true, 300);
    if (lv_obj_is_valid(myLancher[app_index_message].pagetileview))
    {
        set_need_open_gesture_control(true);
        lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(myLancher[app_index_message].pagetileview,
                          LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_tile_id(myLancher[app_index_message].pagetileview, 0, 1,
                           LV_ANIM_ON);
    }
    set_scroll_anim_time(false, NULL);
}

void animate_to_message_list(void)
{
    set_scroll_anim_time(true, 300);
    if (lv_obj_is_valid(myLancher[app_index_message].pagetileview))
    {
        set_need_open_gesture_control(true);
        lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(myLancher[app_index_message].pagetileview,
                          LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_tile_id(myLancher[app_index_message].pagetileview, 1, 0,
                           LV_ANIM_ON);
    }
    set_scroll_anim_time(false, NULL);
}

void chack_tile_page(void)
{
    LOG_D("chack_tile_page: %d, %d",
          lv_obj_get_scroll_x(myLancher[app_index_message].pagetileview),
          lv_obj_get_scroll_y(myLancher[app_index_message].pagetileview));
}

void animate_to_ai_page(void)
{
    /* Voice recognition tile was removed — AI UI lives inside the AI widget
       on the instruction list layout. This function is kept as a no-op for
       compatibility with existing callers. */
}

void animate_to_home_from_ai_page(void)
{
    LOG_D("animate_to_home_from_ai_page");
    if (lv_obj_is_valid(myLancher[app_index_ai_interface].pagetileview))
        lv_obj_set_tile_id(myLancher[app_index_ai_interface].pagetileview, 0, 0,
                           LV_ANIM_ON);
}

static app_media_t *p_app_media = NULL;
static lv_obj_t *ble_mode_btn;
static lv_obj_t *gesture_collect_btn;
static lv_obj_t *gesture_select_icon;
static lv_obj_t *dnd_mode_btn;
static void calculator_btn_event_cb(lv_event_t *e)
{
    gui_app_run(APP_ID_CALCULATOR);

    animate_to_home_from_notification_center();
}

static void set_dnd_mode(bool dnd_mode)
{
    if (dndmode_enabled != dnd_mode)
    {
        dndmode_enabled = dnd_mode;
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
        watch_system_interact(WATCH_DND_MODE_SET, &dndmode_enabled);
#endif
        if (dndmode_enabled)
        {
            lv_obj_set_style_bg_opa(dnd_mode_btn, LV_OPA_90, 0);
            lv_obj_set_style_bg_color(dnd_mode_btn, APP_MAIN_COLOR, 0);
        }
        else
        {
            lv_obj_set_style_bg_opa(dnd_mode_btn, LV_OPA_10, 0);
            lv_obj_set_style_bg_color(dnd_mode_btn, lv_color_hex(0xFFFFFF), 0);
        }
    }
}

static void dnd_mode_btn_event_cb(lv_event_t *e)
{
    if (dnd_mode_btn == NULL)
    {
        return;
    }
    bool mode = !dndmode_enabled;
    set_dnd_mode(mode);
}

extern bool imu_data_collection;
extern bool imu_data_collection_error;
static void gesture_collect_btn_event_cb(lv_event_t *e)
{
    if (gesture_collect_btn == NULL)
    {
        return;
    }
    if (imu_data_collection_error)
    {
        lv_obj_set_style_bg_opa(gesture_collect_btn, LV_OPA_10, 0);
        lv_obj_set_style_bg_color(gesture_collect_btn, lv_color_hex(0xFFFFFF),
                                  0);
        imu_data_collection_error = false;
        imu_data_collection = false;
        watch_sys_sync.notify_imu_data_collection(imu_data_collection);
        set_dnd_mode(false);
    }
    else
    {
        lv_obj_set_style_bg_opa(gesture_collect_btn, LV_OPA_90, 0);
        lv_obj_set_style_bg_color(gesture_collect_btn, APP_MAIN_COLOR, 0);
        imu_data_collection_error = true;
        imu_data_collection = true;
        watch_sys_sync.notify_imu_data_collection(imu_data_collection);
        set_dnd_mode(true);
    }
}

static void qrcode_btn_event_cb(lv_event_t *e)
{
    // AppIntent intent = {0};
    // strcpy(intent.app_id, "JA_app1");
    // watch_run_app_by_intent(&intent);
    gui_app_run(APP_ID_QRCODE);
    animate_to_home_from_notification_center();
}

static void setting_icon_event_cb(lv_event_t *e)
{
    gui_app_run(APP_ID_SETTING);
    animate_to_home_from_notification_center();
}

static void mouse_mode_icon_event_cb(lv_event_t *e)
{
    gui_app_run(APP_ID_MOUSE);
    animate_to_home_from_notification_center();
}

static void flishlight_icon_event_cb(lv_event_t *e)
{
    gui_app_run(APP_ID_FLASHLIGHT);
    animate_to_home_from_notification_center();
}

static void recorder_btn_event_cb(lv_event_t *e)
{
    gui_app_run(APP_ID_RECORDER);
    animate_to_home_from_notification_center();
}

static void gesture_select_icon_event_cb(lv_event_t *e)
{
    if (gesture_tap_collection)
    {
        lv_obj_set_style_bg_opa(gesture_select_icon, LV_OPA_10, 0);
        lv_obj_set_style_bg_color(gesture_select_icon, lv_color_hex(0xFFFFFF),
                                  0);
        gesture_tap_collection = false;
    }
    else
    {
        lv_obj_set_style_bg_opa(gesture_select_icon, LV_OPA_90, 0);
        lv_obj_set_style_bg_color(gesture_select_icon, APP_MAIN_COLOR, 0);
        gesture_tap_collection = true;
    }
}

static void find_phone_btn_event_cb(lv_event_t *e)
{
    control_provider.find_phone();
}

static void gesture_test_btn_event_cb(lv_event_t *e)
{
    gui_app_run(APP_ID_GESTURE);
    animate_to_home_from_notification_center();
}

// static void handle_media_play_state(void *param)
// {
//     if (lv_obj_is_valid(p_app_media->icon_btn_play_pause) == false)
//     {
//         return;
//     }
//     bool media_state = *(bool *)param;
//     lv_obj_t *img = lv_obj_get_child(p_app_media->icon_btn_play_pause, 0);

//     /* Change the image source */
//     lv_img_set_src(img, media_state ? &img_media_pause : &img_media_play);
// }

// static void handle_media_title(void *param)
// {
//     if (lv_obj_is_valid(p_app_media->media_title) == false)
//     {
//         return;
//     }
//     char *media_title_text = (char *)param;
//     if (media_title_text)
//     {
//         lv_label_set_text(p_app_media->media_title, media_title_text);
//     }
// }

static datac_handle_t pwr_srv_hdl = DATA_CLIENT_INVALID_HANDLE;
static lv_obj_t *brightness_bar;

static int powermgr_srv_callback(data_callback_arg_t *arg)
{
    if (!lv_obj_is_valid(brightness_bar) &&
        (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
    {
        return 0;
    }

    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        if (rsp->result >= 0)
        {
            data_msg_t msg;

            data_service_init_msg(&msg, PWRMGR_MSG_LCD_BRIGHTNESS_GET_REQ, 0);
            datac_send_msg(pwr_srv_hdl, &msg);
        }
    }
    break;

    case PWRMGR_MSG_LCD_BRIGHTNESS_GET_RSP:
    {
        range_msg_t *p_range;
        p_range = (range_msg_t *)arg->data;

        LOG_D("PWRMGR_MSG_LCD_BRIGHTNESS_SET_RSP cur=%d[%d,%d]", p_range->cur,
              p_range->min, p_range->max);
        lv_bar_set_range(brightness_bar, p_range->min, p_range->max);
        lv_bar_set_value(brightness_bar, p_range->cur, LV_ANIM_ON);
    }
    break;
    case PWRMGR_MSG_LCD_BRIGHTNESS_SET_RSP:
    {
        range_msg_t *p_range;
        p_range = (range_msg_t *)arg->data;
        LOG_D("PWRMGR_MSG_LCD_BRIGHTNESS_SET_RSP cur=%d[%d,%d]", p_range->cur,
              p_range->min, p_range->max);
    }
    break;
    case PWRMGR_MSG_LCD_ROTATE_180_GET_RSP:
    {
        uint16_t r = *((uint16_t *)arg->data);
        LOG_D("PWRMGR_MSG_LCD_ROTATE_180_GET_RSP %d", r);
    }
    break;
    case PWRMGR_MSG_LCD_ROTATE_180_SET_RSP:
    {
        uint16_t r = *((uint16_t *)arg->data);
        LOG_D("PWRMGR_MSG_LCD_ROTATE_180_SET_RSP %d", r);
    }
    break;
    default:
        break;
    }
    return 0;
}

static void subscribe_pwr_service(void)
{
    if (DATA_CLIENT_INVALID_HANDLE != pwr_srv_hdl)
    {
        return;
    }
    pwr_srv_hdl = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != pwr_srv_hdl);
    ui_datac_subscribe(pwr_srv_hdl, "powermgr", powermgr_srv_callback, 0);
}

void unsubscribe_pwr_service(void)
{
    if (DATA_CLIENT_INVALID_HANDLE != pwr_srv_hdl)
    {
        datac_close(pwr_srv_hdl);
        pwr_srv_hdl = DATA_CLIENT_INVALID_HANDLE;
    }
}

void gui_set_brightness(uint16_t brightness, bool user_action)
{
    subscribe_pwr_service();
    datac_send_data(pwr_srv_hdl, PWRMGR_MSG_LCD_BRIGHTNESS_SET_REQ,
                    (uint8_t *)&brightness, sizeof(uint16_t));
    if (user_action)
    {
#ifdef BSP_USING_BLOC_CONTROL
        control_provider.screen_brightness_smoothly(brightness);
#endif
    }
}

void gui_set_screen_timeout(uint16_t timeout_sec)
{
    subscribe_pwr_service();
    datac_send_data(pwr_srv_hdl, PWRMGR_MSG_LCD_AUTO_OFF_TIME_SET_REQ,
                    (uint8_t *)&timeout_sec, sizeof(uint16_t));
}

void gui_set_screen_rotation(uint8_t rotation)
{
    subscribe_pwr_service();
    uint16_t v = 0;
    if (rotation == 180)
    {
        v = 1;
        datac_send_data(pwr_srv_hdl, PWRMGR_MSG_LCD_ROTATE_180_SET_REQ,
                        (uint8_t *)&v, sizeof(v));
        lv_disp_set_rotation(lv_disp_get_default(), LV_DISP_ROT_180);
    }
    else if (rotation == 0)
    {
        datac_send_data(pwr_srv_hdl, PWRMGR_MSG_LCD_ROTATE_180_SET_REQ,
                        (uint8_t *)&v, sizeof(v));
        lv_disp_set_rotation(lv_disp_get_default(), LV_DISP_ROT_NONE);
    }
}

static void bar_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSING)
    {
        lv_obj_t *bar = lv_event_get_target(e);

        lv_point_t p;
        lv_indev_get_point(lv_indev_get_act(), &p);

        lv_coord_t min = lv_bar_get_min_value(bar);
        lv_coord_t max = lv_bar_get_max_value(bar);

        lv_coord_t w = lv_obj_get_width(bar);
        lv_coord_t rel_x = p.x - lv_obj_get_x(bar);
        if (rel_x < 0)
            rel_x = 0;
        if (rel_x > w)
            rel_x = w;

        lv_coord_t value = (rel_x * (max - min)) / w + min;
        if (value < 5)
            value = 5;
        lv_bar_set_value(bar, value, LV_ANIM_OFF);
        uint16_t brightness = lv_bar_get_value(bar);
        gui_set_brightness(brightness, true);
    }
    else if (code == LV_EVENT_PRESSED)
    {
        LOG_D("LV_EVENT_PRESSED_Brightness Bar");
        lv_obj_clear_flag(myLancher[app_index_message].pagetileview,
                          LV_OBJ_FLAG_SCROLLABLE);
    }
    else if (code == LV_EVENT_RELEASED)
    {
        LOG_D("LV_EVENT_RELEASED_Brightness Bar");
        lv_obj_add_flag(myLancher[app_index_message].pagetileview,
                        LV_OBJ_FLAG_SCROLLABLE);
    }
}

extern void build_media_contorll_widget(app_media_t *p_app_media,
                                        lv_obj_t *parent);
extern lv_obj_t *lv_media_widget_builder(lv_obj_t *parent);
static lv_obj_t *tools[9] = {NULL};
static lv_obj_t *control_center_window;
static lv_obj_t *control_center_layout_create(lv_obj_t *parent)
{
    control_center_window = lv_obj_create(parent);
    lv_obj_set_size(control_center_window, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_scrollbar_mode(control_center_window, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(control_center_window, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(control_center_window, LV_DIR_VER);
    lv_obj_set_style_bg_color(control_center_window, LV_COLOR_WHITE,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(control_center_window, LV_OPA_0,
                            LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *bar =
        lv_bar_create(control_center_window); // Create a progress bar
    lv_bar_set_range(bar, 0, 100); // Set the range of the progress bar
    lv_obj_set_width(bar, LV_PCT(70));
    lv_obj_set_height(bar, 80);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_color(bar, APP_MAIN_COLOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(bar, APP_MAIN_COLOR, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_90, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(bar, LV_OPA_10, LV_PART_MAIN);
    lv_bar_set_value(bar, SkaiWatchSys.brightness, LV_ANIM_ON);
    lv_obj_add_event_cb(bar, bar_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_t *icon = lv_img_create(bar);
    lv_img_set_src(icon, &sun);
    lv_obj_align(icon, LV_ALIGN_LEFT_MID, 20, 0);
    brightness_bar = bar;

    lv_obj_t *control_center_bottom = lv_obj_create(control_center_window);
    lv_obj_set_size(control_center_bottom, LV_HOR_RES, LV_VER_RES);
    // lv_obj_set_scrollbar_mode(control_center_bottom, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(control_center_bottom, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(control_center_bottom, LV_OPA_0, 0);
    lv_obj_align_to(control_center_bottom, bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    // 2 * N
    // - | -
    // - | -
    // - | -
    // setting icon
    // --- Mouse mode button
    lv_obj_t *calculator_btn =
        common_image_button(control_center_bottom, CALCULATOR_ICON, 100, 100,
                            calculator_btn_event_cb);
    lv_obj_set_style_border_width(calculator_btn, 2, 0);
    lv_obj_set_style_border_color(calculator_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(calculator_btn, LV_OPA_0, 0);
    lv_obj_set_style_bg_opa(calculator_btn, LV_OPA_10, 0);
    lv_obj_align(calculator_btn, LV_ALIGN_TOP_LEFT, 70, 0);
    tools[0] = calculator_btn;

    lv_obj_t *recorder_btn = common_image_button(
        control_center_bottom, &micro_icon, 100, 100, recorder_btn_event_cb);
    lv_obj_set_style_border_width(recorder_btn, 2, 0);
    lv_obj_set_style_border_color(recorder_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(recorder_btn, LV_OPA_0, 0);
    lv_obj_align(recorder_btn, LV_ALIGN_TOP_RIGHT, -70, 0);
    tools[2] = recorder_btn;

    lv_obj_t *flishlight_btn =
        common_image_button(control_center_bottom, FLISHLIGHT_ICON, 100, 100,
                            flishlight_icon_event_cb);
    lv_obj_set_style_border_width(flishlight_btn, 2, 0);
    lv_obj_set_style_border_color(flishlight_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(flishlight_btn, LV_OPA_0, 0);
    lv_obj_align(flishlight_btn, LV_ALIGN_TOP_MID, 0, 0);
    tools[1] = flishlight_btn;

    // setting icon (bottom left)
    lv_obj_t *setting_icon = common_image_button(
        control_center_bottom, IMG_SETTINGS, 100, 100, setting_icon_event_cb);
    lv_obj_set_style_border_width(setting_icon, 2, 0);
    lv_obj_set_style_border_color(setting_icon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(setting_icon, LV_OPA_0, 0);
    lv_obj_align_to(setting_icon, calculator_btn, LV_ALIGN_OUT_BOTTOM_MID, 0,
                    15);
    lv_obj_set_style_bg_opa(setting_icon, LV_OPA_10, 0);
    lv_obj_set_style_bg_color(setting_icon, lv_color_hex(0xFFFFFF), 0);
    tools[3] = setting_icon;

    // --- QRcode button
    lv_obj_t *qrcode_btn = common_image_button(
        control_center_bottom, ICON_QRCODE, 100, 100, qrcode_btn_event_cb);
    lv_obj_set_style_border_width(qrcode_btn, 2, 0);
    lv_obj_set_style_border_color(qrcode_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(qrcode_btn, LV_OPA_0, 0);
    lv_obj_align_to(qrcode_btn, flishlight_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);
    tools[4] = qrcode_btn;

    // --- Dont disturb mode button
    dndmode_enabled = SkaiWatchSys.DNDMode.config.status;
    dnd_mode_btn = common_image_button(control_center_bottom, ICON_DND_MODE,
                                       100, 100, dnd_mode_btn_event_cb);
    lv_obj_set_style_border_width(dnd_mode_btn, 2, 0);
    lv_obj_set_style_border_color(dnd_mode_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(dnd_mode_btn, LV_OPA_0, 0);
    lv_obj_align_to(dnd_mode_btn, recorder_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);
    if (dndmode_enabled)
    {
        lv_obj_set_style_bg_opa(dnd_mode_btn, LV_OPA_90, 0);
        lv_obj_set_style_bg_color(dnd_mode_btn, APP_MAIN_COLOR, 0);
    }
    else
    {
        lv_obj_set_style_bg_opa(dnd_mode_btn, LV_OPA_10, 0);
        lv_obj_set_style_bg_color(dnd_mode_btn, lv_color_hex(0xFFFFFF), 0);
    }
    tools[5] = dnd_mode_btn;

    lv_obj_t *mouse_btn =
        common_image_button(control_center_bottom, MOUSE_MODE_ICON, 100, 100,
                            mouse_mode_icon_event_cb);
    lv_obj_set_style_border_width(mouse_btn, 2, 0);
    lv_obj_set_style_border_color(mouse_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(mouse_btn, LV_OPA_0, 0);
    lv_obj_set_style_bg_opa(mouse_btn, LV_OPA_10, 0);
    lv_obj_set_style_bg_color(mouse_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align_to(mouse_btn, dnd_mode_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 35);
    tools[6] = mouse_btn;

    // -------------- find phone button
    lv_obj_t *find_phone_btn = common_image_button(
        control_center_bottom, FIND_PHONE, 100, 100, find_phone_btn_event_cb);
    lv_obj_set_style_border_width(find_phone_btn, 2, 0);
    lv_obj_set_style_border_color(find_phone_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(find_phone_btn, LV_OPA_0, 0);
    lv_obj_align_to(find_phone_btn, setting_icon, LV_ALIGN_OUT_BOTTOM_MID, 0,
                    35);
    tools[7] = find_phone_btn;
#if !kReleaseMode
    // Gesture test app
    lv_obj_t *gesture_test_btn = common_image_button(
        control_center_bottom, IMG_LOGO, 100, 100, gesture_test_btn_event_cb);
    lv_obj_set_style_border_width(gesture_test_btn, 2, 0);
    lv_obj_set_style_border_color(gesture_test_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(gesture_test_btn, LV_OPA_0, 0);
    lv_obj_align_to(gesture_test_btn, qrcode_btn, LV_ALIGN_OUT_BOTTOM_MID, 0,
                    35);
    tools[8] = gesture_test_btn;
#endif

#ifdef BSP_USING_UI_HANDLER
    // lvgl_msg_handler.handle_bar_media_play_state = handle_media_play_state;
    // lvgl_msg_handler.handle_bar_media_title = handle_media_title;
#endif

    return control_center_window;
}

extern void reset_media_widget(void);
static void clean_tools_selection(void)
{
    for (int i = 0; i < 7; i++)
    {
        if (tools[i] != NULL)
        {
            lv_obj_set_style_border_opa(tools[i], LV_OPA_0, 0);
        }
    }
    reset_media_widget();
}
static void scroll_control_center_to_top(void)
{
    if (lv_obj_is_valid(control_center_window))
    {
        lv_obj_scroll_to_y(control_center_window, 0, LV_ANIM_OFF);
    }
}

void control_center_on_resume(void)
{
    // scroll_control_center_to_top();
    // reset_tools_selection();
}

void control_center_on_pause(void)
{
    scroll_control_center_to_top();
}

static uint8_t button_selection_index = 4;
static void reset_tools_selection(void)
{
    for (int i = 0; i < 7; i++)
    {
        if (tools[i] != NULL)
        {
            lv_obj_set_style_border_opa(tools[i], LV_OPA_0, 0);
        }
    }
    reset_media_widget();
    button_selection_index = 4;
    lv_obj_set_style_border_opa(tools[1], LV_OPA_80, 0);
}

extern void selection_media_widget(uint8_t index);
static void button_selection(gesture_position_t gesture_position)
{
    if (middle_layer_tileview_index != 3)
    {
        return;
    }

    const int p_x = gesture_position.gesture_position_x;
    const int p_y = gesture_position.gesture_position_y;
    LOG_D("button_selection p_x: %d, p_y: %d", p_x, p_y);
    int category = 1;
    if (p_y > 266)
    {
        if (p_x < 155)
        {
            category = 0; // Previous
        }
        else if (p_x < 310)
        {
            category = 1; // Play/Pause
        }
        else
        {
            category = 2; // Next
        }
    }
    else if (p_y > 66)
    {
        if (p_x < 155)
        {
            category = 3; // Mouse Mode
        }
        else if (p_x < 310)
        {
            category = 4; // Flashlight
        }
        else
        {
            category = 5; // Recorder
        }
    }
    else
    {
        category = 6; // Settings
    }

    if (button_selection_index == category || category == 6)
    {
        return;
    }
    button_selection_index = category;
    motor_pattern_scrolling_app();
    LOG_D("button_selection_index: %d", button_selection_index);
    clean_tools_selection();
    if (category > 2)
    {
        lv_obj_set_style_border_opa(tools[category - 3], LV_OPA_80, 0);
    }
    else
    {
        selection_media_widget(category);
    }
}

static void press_event(uint8_t press)
{
    if (press == 0)
    {
        return;
    }
    switch (button_selection_index)
    {
    case CONTROL_BTN_MEDIA_PREV:
        sys_media_event_set(SYS_EVENT_PREV);
        break;
    case CONTROL_BTN_MEDIA_PLAY_PAUSE:
        sys_media_event_set(SYS_EVENT_PLAY_PAUSE);
        break;
    case CONTROL_BTN_MEDIA_NEXT:
        sys_media_event_set(SYS_EVENT_NEXT);
        break;
    case CONTROL_BTN_CALCULATOR:
        calculator_btn_event_cb(NULL);
        break;
    case CONTROL_BTN_FLASHLIGHT:
        flishlight_icon_event_cb(NULL);
        break;
    case CONTROL_BTN_RECORDER:
        recorder_btn_event_cb(NULL);
        break;
    default:
        break;
    }
}

extern bool get_bluetooth_broadcasting_status(void);
void refresh_ble_mode_btn(void)
{
    if (ble_mode_btn == NULL)
    {
        return;
    }

    if (get_bluetooth_broadcasting_status() ||
        SkaiWatchSys.gap_conn_state == GAP_CONN_STATE_CONNECTED)
    {
        lv_obj_set_style_bg_opa(ble_mode_btn, LV_OPA_90, 0);
        lv_obj_set_style_bg_color(ble_mode_btn, APP_MAIN_COLOR, 0);
    }
    else
    {
        lv_obj_set_style_bg_opa(ble_mode_btn, LV_OPA_10, 0);
        lv_obj_set_style_bg_color(ble_mode_btn, lv_color_hex(0xFFFFFF), 0);
    }
}

static void app_clock_main_tileview_event(lv_event_t *event)
{
    lv_event_code_t code = lv_event_get_code(event);

    if (code == LV_EVENT_PRESSED)
    {
        LOG_D("LV_EVENT_PRESSED_Tileview");
    }
}

static lv_obj_t *pages[5];
static bool test_mode = false;
void open_test_mode(bool open)
{
    // test_mode = open;
    // for (int i = 0; i < 5; i++)
    // {
    //     if (open)
    //     {
    //         if (lv_obj_is_valid(pages[i]))
    //             lv_obj_set_style_bg_opa(pages[i], LV_OPA_50,
    //                                     LV_PART_MAIN | LV_STATE_DEFAULT);
    //     }
    //     else
    //     {
    //         if (lv_obj_is_valid(pages[i]))
    //             lv_obj_set_style_bg_opa(pages[i], LV_OPA_TRANSP,
    //                                     LV_PART_MAIN | LV_STATE_DEFAULT);
    //     }
    // }
    // // for (int i = 0; i < 4; i++)
    // // {
    // if (open)
    // {
    //     if (lv_obj_is_valid(status_bar_area_left))
    //         lv_obj_set_style_bg_opa(status_bar_area_left, LV_OPA_50, 0);
    //     if (lv_obj_is_valid(status_bar_area_right))
    //         lv_obj_set_style_bg_opa(status_bar_area_right, LV_OPA_50, 0);
    //     if (lv_obj_is_valid(status_bar_area_up))
    //         lv_obj_set_style_bg_opa(status_bar_area_up, LV_OPA_50, 0);
    //     if (lv_obj_is_valid(status_bar_area_down))
    //         lv_obj_set_style_bg_opa(status_bar_area_down, LV_OPA_50, 0);
    // }
    // else
    // {
    //     if (lv_obj_is_valid(status_bar_area_left))
    //         lv_obj_set_style_bg_opa(status_bar_area_left, LV_OPA_TRANSP, 0);
    //     if (lv_obj_is_valid(status_bar_area_right))
    //         lv_obj_set_style_bg_opa(status_bar_area_right, LV_OPA_TRANSP, 0);
    //     if (lv_obj_is_valid(status_bar_area_up))
    //         lv_obj_set_style_bg_opa(status_bar_area_up, LV_OPA_TRANSP, 0);
    //     if (lv_obj_is_valid(status_bar_area_down))
    //         lv_obj_set_style_bg_opa(status_bar_area_down, LV_OPA_TRANSP, 0);
    // }
    // }
    watch_sys_sync.set_debug_mode(open);
}

bool is_test_mode(void)
{
    return test_mode;
}

static void set_clock_main_status_opa(uint8_t opa, bool mask)
{
    // if (lv_obj_is_valid(myLancher[app_index_message].pagetileview))
    // {
    //     lv_obj_set_style_bg_opa(myLancher[app_index_message].pagetileview,
    //     opa,
    //                             LV_PART_MAIN | LV_STATE_DEFAULT);
    // }
    // LOG_D("set_clock_main_status_opa opa: %d", opa);
    // if (lv_obj_is_valid(gaus_dial_bg))
    // {
    //     lv_obj_set_style_bg_opa(gaus_dial_bg, opa,
    //                             LV_PART_MAIN | LV_STATE_DEFAULT);
    // }
    if (lv_obj_is_valid(gaus_dial_img))
    {
        uint8_t mask_traget_opa = LV_OPA_COVER;
        // if (mask)
        // {
        //     mask_traget_opa = LV_OPA_COVER;
        // }
        uint8_t mask_opa = (opa);
        lv_obj_set_style_img_opa(gaus_dial_img, opa,
                                 LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}

void set_clock_main_status_img(const void *img_src)
{
    if (lv_obj_is_valid(gaus_dial_img))
    {
        lv_img_set_src(gaus_dial_img, img_src);
        lv_img_set_zoom(gaus_dial_img, 256 * 2);
        lv_obj_center(gaus_dial_img);
    }
}

extern uint8_t get_last_active_clock(void);
static char *get_picture_name(void)
{
    static char name[64];
    char folder[64];
    snprintf(folder, sizeof(folder), "/JW_wf%u", get_last_active_clock());
    DIR *dir = opendir(folder);
    if (!dir)
        return NULL;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (strncmp(entry->d_name, "picture_", 8) == 0)
        {
            strncpy(name, entry->d_name + 8, sizeof(name) - 1);
            name[sizeof(name) - 1] = '\0';
            closedir(dir);
            return name;
        }
    }
    closedir(dir);
    return NULL;
}

static lv_obj_t *status_bar_bg_main = NULL;
static lv_obj_t *status_bar_bg_ai = NULL;
uint8_t bar_opa = LV_OPA_TRANSP; // LV_OPA_TRANSP LV_OPA_COVER
void app_clock_main_status_bar_init(lv_obj_t *par)
{
    if (test_mode)
    {
        bar_opa = LV_OPA_50;
    }
    else
    {
        bar_opa = LV_OPA_TRANSP;
    }

    gaus_dial_bg = lv_obj_create(par);
    lv_obj_set_size(gaus_dial_bg, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(gaus_dial_bg, LV_COLOR_BLACK,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(gaus_dial_bg, LV_OPA_TRANSP,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(gaus_dial_bg);
    lv_obj_add_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
    // char dst_path[64];
    // if (get_last_active_clock() == 2 || get_last_active_clock() == 3)
    // {
    //     char *filename = get_picture_name();
    //     if (filename != NULL)
    //     {
    //         snprintf(dst_path, sizeof(dst_path),
    //         "/assets/gaus_images/gaus_%s",
    //                  filename);
    //     }
    //     else
    //     {
    //         snprintf(dst_path, sizeof(dst_path),
    //                  "/assets/gaus_images/gaus_default_picture.bin");
    //     }
    // }
    // else if (get_last_active_clock() == 1)
    // {
    //     strncpy(dst_path, "/assets/gaus_images/gaus_clock1_bg.bin",
    //             sizeof(dst_path));
    //     dst_path[sizeof(dst_path) - 1] = '\0';
    // }
    // else if (get_last_active_clock() == 4)
    // {
    //     strncpy(dst_path, "/assets/gaus_images/gaus_clock4_bg.bin",
    //             sizeof(dst_path));
    //     dst_path[sizeof(dst_path) - 1] = '\0';
    // }
    // else if (get_last_active_clock() == 5)
    // {
    //     strncpy(dst_path, "/assets/gaus_images/gaus_clock5_bg.bin",
    //             sizeof(dst_path));
    //     dst_path[sizeof(dst_path) - 1] = '\0';
    // }
    gaus_dial_img = lv_img_create(gaus_dial_bg);
    lv_img_set_src(gaus_dial_img, GAUS_CLOCK1_BG);
    lv_obj_set_style_img_opa(gaus_dial_img, LV_OPA_0,
                             LV_PART_MAIN | LV_STATE_DEFAULT);
    // lv_obj_set_size(gaus_dial_img, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_img_set_zoom(gaus_dial_img, 256 * 2);
    lv_obj_center(gaus_dial_img);

    status_bar_bg_main = lv_obj_create(par);
    lv_obj_set_size(status_bar_bg_main, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_opa(status_bar_bg_main, LV_OPA_0,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(status_bar_bg_main, LV_COLOR_WHITE,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    // lv_obj_set_scrollbar_mode(status_bar_bg_main, LV_SCROLLBAR_MODE_OFF);
    // lv_obj_set_scroll_dir(status_bar_bg_main, LV_DIR_HOR);
    lv_obj_clear_flag(status_bar_bg_main, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_pos(status_bar_bg_main, 0, 0);
    rt_uint16_t i;

    lv_obj_t *status_bar_area;
    // create a invisible object at top of parent, and shown status bar when
    // press it 上下狀態欄的拖動把手
    for (i = 0; i < 3; i++)
    {
        status_bar_area = lv_obj_create(status_bar_bg_main);
        lv_obj_set_scrollbar_mode(status_bar_area, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(
            status_bar_area,
            LV_OBJ_FLAG_PRESS_LOCK); // Allow press event to tileview
        lv_obj_set_style_bg_opa(status_bar_area, bar_opa,
                                LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_add_flag(status_bar_area, LV_OBJ_FLAG_EVENT_BUBBLE);

        if (STATUS_BAR_AREA_LEFT == i)
        {
            lv_obj_set_size(status_bar_area, (LV_HOR_RES_MAX >> 3),
                            LV_VER_RES_MAX);
            lv_obj_set_user_data(status_bar_area, (void *)STATUS_BAR_AREA_LEFT);
            lv_obj_add_event_cb(status_bar_area, notification_status_bar_cb,
                                LV_EVENT_ALL, NULL);
            lv_obj_align(status_bar_area, LV_ALIGN_LEFT_MID, 0, 0);
            status_bar_area_left = status_bar_area;
        }
        else if (STATUS_BAR_AREA_UP == i)
        {
            lv_obj_set_size(status_bar_area, LV_HOR_RES_MAX,
                            (LV_VER_RES_MAX >> 3));
            lv_obj_set_user_data(status_bar_area, (void *)STATUS_BAR_AREA_UP);
            lv_obj_add_event_cb(status_bar_area, notification_status_bar_cb,
                                LV_EVENT_ALL, NULL);
            lv_obj_align(status_bar_area, LV_ALIGN_TOP_MID, 0, 0);
            status_bar_area_up = status_bar_area;
        }
        else if (STATUS_BAR_AREA_DOWN == i)
        {
            lv_obj_set_size(status_bar_area, LV_HOR_RES_MAX,
                            (LV_VER_RES_MAX >> 4));
            lv_obj_set_user_data(status_bar_area, (void *)STATUS_BAR_AREA_DOWN);
            lv_obj_add_event_cb(status_bar_area, notification_status_bar_cb,
                                LV_EVENT_ALL, NULL);
            lv_obj_align(status_bar_area, LV_ALIGN_BOTTOM_MID, 0, 0);
            status_bar_area_down = status_bar_area;
        }
    }

    app_clock_main_status_bar = lv_tileview_create(status_bar_bg_main);
    lv_obj_set_scrollbar_mode(app_clock_main_status_bar, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(app_clock_main_status_bar, LV_COLOR_BLACK,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(app_clock_main_status_bar, LV_OPA_TRANSP,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    // 0: down, 1: home, 2: up, 3: left, 4: right
    for (i = 0; i < 4; i++)
    {
        if (i == MAIN_PAGE_TYPE_HOME)
        {
            pages[i] = lv_tileview_add_tile(app_clock_main_status_bar, 1, i,
                                            LV_DIR_ALL);
            app_clock_main_status_bar_down = pages[i];
            lv_obj_set_style_bg_color(pages[i], LV_COLOR_RED,
                                      LV_PART_MAIN | LV_STATE_DEFAULT);
            if (!test_mode)
            {
                lv_obj_set_style_bg_opa(pages[i], LV_OPA_TRANSP,
                                        LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            else
            {
                lv_obj_set_style_bg_opa(pages[i], LV_OPA_50,
                                        LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            lv_obj_add_event_cb(pages[i],
                                app_clock_main_status_bar_down_event_cb,
                                LV_EVENT_ALL, NULL);
            lv_obj_set_size(pages[i], LV_HOR_RES_MAX, LV_VER_RES_MAX);
            lv_obj_set_scrollbar_mode(pages[i], LV_SCROLLBAR_MODE_OFF);
        }
        else
        {
            if (i == MAIN_PAGE_TYPE_DOWN)
            {
                pages[i] = lv_tileview_add_tile(app_clock_main_status_bar, 1, 2,
                                                LV_DIR_VER);
            }
            else if (i == MAIN_PAGE_TYPE_LEFT)
            {
                pages[i] = lv_tileview_add_tile(app_clock_main_status_bar, 0, 1,
                                                LV_DIR_RIGHT);
            }
            else if (i == MAIN_PAGE_TYPE_UP)
            {
                pages[i] = lv_tileview_add_tile(app_clock_main_status_bar, 1, 0,
                                                LV_DIR_VER);
            }
            else if (i == MAIN_PAGE_TYPE_RIGHT)
            {
                pages[i] = lv_tileview_add_tile(app_clock_main_status_bar, 2, 1,
                                                LV_DIR_VER);
            }
            if (!test_mode)
            {
                lv_obj_set_style_bg_opa(pages[i], LV_OPA_TRANSP,
                                        LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            else
            {
                lv_obj_set_style_bg_opa(pages[i], LV_OPA_50,
                                        LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            if (i == 0)
            {
                lv_obj_set_style_bg_color(pages[i], LV_COLOR_YELLOW,
                                          LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            else if (i == 2)
            {
                lv_obj_set_style_bg_color(pages[i], LV_COLOR_GREEN,
                                          LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            else if (i == 3)
            {
                lv_obj_set_style_bg_color(pages[i], LV_COLOR_WHITE,
                                          LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            else if (i == 4)
            {
                lv_obj_set_style_bg_color(pages[i], LV_COLOR_BLUE,
                                          LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            lv_obj_set_size(pages[i], LV_HOR_RES_MAX, LV_VER_RES_MAX);
            lv_obj_set_scrollbar_mode(pages[i], LV_SCROLLBAR_MODE_OFF);
            // lv_obj_add_event_cb(pages[i], app_clock_main_tileview_event,
            // LV_EVENT_ALL, NULL);
        }
    }
    // pages[2] = lv_tileview_add_tile(app_clock_main_status_bar, 0, 1,
    // LV_DIR_RIGHT); if (TILEVIEW_HIDDEN_CLOSE)
    // {
    //     lv_obj_set_style_bg_opa(pages[2], LV_OPA_50, LV_PART_MAIN |
    //     LV_STATE_DEFAULT);
    // }
    // else
    // {
    //     lv_obj_set_style_bg_opa(pages[2], LV_OPA_TRANSP, LV_PART_MAIN |
    //     LV_STATE_DEFAULT);
    // }
    // lv_obj_set_style_bg_color(pages[2], LV_COLOR_BLACK, LV_PART_MAIN |
    // LV_STATE_DEFAULT); lv_obj_set_size(pages[2], LV_HOR_RES_MAX,
    // LV_VER_RES_MAX); lv_obj_set_scrollbar_mode(pages[2],
    // LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_tile_id(app_clock_main_status_bar, 1, 1, false);
    lv_obj_add_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
    control_center_layout_create(pages[CONTROL_CENTER_PAGE_INDEX]);

    extern lv_obj_t *lv_instruction_list_layout_create(lv_obj_t * parent);
    lv_instruction_list_layout_create(pages[INSTRUCTION_LIST_PAGE_INDEX]);
    extern lv_obj_t *lv_message_list_layout_create(lv_obj_t * parent);
    lv_message_list_layout_create(pages[MESSAGE_PAGE_INDEX]);

    LOG_D("tileview set tile id to 1,1");
    myLancher[app_index_message].pagetileview = app_clock_main_status_bar;

    lv_obj_add_event_cb(app_clock_main_status_bar,
                        app_clock_main_status_bar_event_cb, LV_EVENT_ALL, NULL);

    middle_layer_tileview_index = 1;
    check_main_page();
    LOG_D("app_clock_main_status_bar_init");
}

void app_clock_ai_status_bar_init(lv_obj_t *par)
{
    status_bar_bg_ai = lv_obj_create(par);
    lv_obj_set_size(status_bar_bg_ai, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_opa(status_bar_bg_ai, LV_OPA_0,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(status_bar_bg_ai, LV_COLOR_WHITE,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(status_bar_bg_ai, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_pos(status_bar_bg_ai, 0, 0);

    lv_obj_t *status_bar_area;
    lv_obj_t *main_page;
    status_bar_area = lv_obj_create(status_bar_bg_ai);
    lv_obj_set_size(status_bar_area, (LV_HOR_RES_MAX >> 4),
                    (LV_VER_RES_MAX >> 2));
    lv_obj_set_scrollbar_mode(status_bar_area, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(status_bar_area,
                      LV_OBJ_FLAG_PRESS_LOCK); // Allow press event to tileview
    lv_obj_set_style_bg_opa(status_bar_area, bar_opa,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(status_bar_area, ai_status_bar_cb, LV_EVENT_ALL, NULL);
    lv_obj_align(status_bar_area, LV_ALIGN_RIGHT_MID, 0, 0);
    status_bar_area_right = status_bar_area;

    app_clock_ai_status_bar = lv_tileview_create(status_bar_bg_ai);
    lv_obj_set_scrollbar_mode(app_clock_ai_status_bar, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(app_clock_ai_status_bar, LV_COLOR_BLACK,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(app_clock_ai_status_bar, LV_OPA_TRANSP,
                            LV_PART_MAIN | LV_STATE_DEFAULT);

    /* Voice recognition tile was removed — UI is now integrated into the AI
       widget (lv_instruction_list_layout.c). Only the main AI status bar
       tile remains here. */
    main_page = lv_tileview_add_tile(app_clock_ai_status_bar, 0, 0, LV_DIR_HOR);
    lv_obj_add_event_cb(main_page, app_clock_main_ai_status_bar_event_cb,
                        LV_EVENT_ALL, NULL);
    lv_obj_set_style_bg_color(main_page, LV_COLOR_BLACK,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(main_page, LV_OPA_TRANSP,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_size(main_page, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_scrollbar_mode(main_page, LV_SCROLLBAR_MODE_OFF);

    // Add event callback for AI status bar tile
    lv_obj_add_event_cb(app_clock_ai_status_bar,
                        app_clock_ai_status_bar_event_cb, LV_EVENT_ALL, NULL);

    // Set the tile ID to the AI status bar tile
    lv_obj_set_tile_id(app_clock_ai_status_bar, 0, 0, false);
    lv_obj_add_flag(app_clock_ai_status_bar, LV_OBJ_FLAG_HIDDEN);

    myLancher[app_index_ai_interface].pagetileview = app_clock_ai_status_bar;
}

// Device change bar - device list UI state
static bool dev_change_watch_mode =
    true; // true = Watch selected, false = device selected
static struct
{
    lv_obj_t *watch_list;
    lv_obj_t *device_list;
    lv_obj_t *empty_label;
    lv_obj_t *connecting_btn;
    lv_obj_t *connecting_label;
} dev_change_list_ui = {0};

static lv_timer_t *dev_change_connecting_timer = NULL;
static uint8_t dev_change_connecting_dots = 1;

static void dev_change_stop_connecting_timer(void)
{
    if (dev_change_connecting_timer)
    {
        lv_timer_del(dev_change_connecting_timer);
        dev_change_connecting_timer = NULL;
    }
}

static void dev_change_refresh_device_list(void);

static void dev_change_connecting_timer_cb(lv_timer_t *t)
{
    (void)t;
    if (!get_bluetooth_broadcasting_status())
    {
        dev_change_stop_connecting_timer();
        dev_change_refresh_device_list();
        return;
    }

    dev_change_connecting_dots = (dev_change_connecting_dots % 3) + 1;
    static const char *const texts[] = {"Connecting.", "Connecting..",
                                        "Connecting..."};
    if (dev_change_list_ui.connecting_label &&
        lv_obj_is_valid(dev_change_list_ui.connecting_label))
    {
        lv_label_set_text(dev_change_list_ui.connecting_label,
                          texts[dev_change_connecting_dots - 1]);
    }
}

static void dev_change_start_connecting_timer(void)
{
    if (dev_change_connecting_timer)
        return;
    dev_change_connecting_dots = 1;
    dev_change_connecting_timer =
        lv_timer_create(dev_change_connecting_timer_cb, 400, NULL);
}

static lv_obj_t *dev_change_create_device_item(lv_obj_t *parent,
                                               const bonded_device_t *device,
                                               uint8_t device_idx)
{
    int active_idx = ble_dev_mgr_get_active_device();
    bool is_active = !dev_change_watch_mode && (active_idx == device_idx);

    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, DEV_CHANGE_BUTTON_WIDTH, DEV_CHANGE_BUTTON_HEIGHT);
    lv_obj_set_style_radius(btn, 80, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x3A3A3A), LV_STATE_PRESSED);
    lv_obj_set_style_pad_all(btn, 6, 0);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_PRESS_LOCK);

    lv_obj_t *device_bg = lv_img_create(btn);
    lv_img_set_src(device_bg, &device_btn);
    lv_obj_align(device_bg, LV_ALIGN_CENTER, 0, 0);

    if (is_active)
    {
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(0x00AAFF), 0);
    }
    else
    {
        lv_obj_set_style_border_width(btn, 0, 0);
    }

    lv_obj_set_user_data(btn, (void *)(uintptr_t)device_idx);

    // Connection status LED indicator
    lv_obj_t *led_indicator = lv_obj_create(btn);
    lv_obj_set_size(led_indicator, 12, 12);
    lv_obj_set_style_radius(led_indicator, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(led_indicator, 0, 0);
    lv_obj_set_style_pad_all(led_indicator, 0, 0);
    lv_obj_align(led_indicator, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_clear_flag(led_indicator,
                      LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    if (device->conn_idx != 0xFF)
    {
        lv_obj_set_style_bg_color(led_indicator, lv_color_hex(0x00FF00), 0);
        lv_obj_set_style_shadow_color(led_indicator, lv_color_hex(0x00FF00), 0);
        lv_obj_set_style_shadow_width(led_indicator, 8, 0);
        lv_obj_set_style_shadow_spread(led_indicator, 2, 0);
    }
    else
    {
        lv_obj_set_style_bg_color(led_indicator, lv_color_hex(0x666666), 0);
    }

    // Device name
    lv_obj_t *name_label = lv_label_create(btn);
    lv_label_set_text(name_label, device->device_name);
    lv_obj_set_style_text_color(name_label, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_long_mode(name_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(name_label, LV_PCT(80));
    lv_obj_align(name_label, LV_ALIGN_LEFT_MID, 18, 0);
    lv_obj_clear_flag(name_label, LV_OBJ_FLAG_CLICKABLE);

    return btn;
}

// Long press delete for device change bar
#define DEV_CHANGE_EXTRA_LONG_PRESS_MS 800
static lv_timer_t *dev_change_delete_timer = NULL;
static uint8_t dev_change_delete_device_idx = 0xFF;
static lv_obj_t *dev_change_delete_confirm_msgbox = NULL;
static uint8_t dev_change_pending_delete_idx = 0xFF;

static void dev_change_delete_confirm_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_current_target(e);
    const char *btn_txt = lv_msgbox_get_active_btn_text(obj);

    if (btn_txt)
    {
        if (strcmp(btn_txt, "Yes") == 0)
        {
            if (dev_change_pending_delete_idx != 0xFF)
            {
                LOG_I("Device change bar: confirmed delete device [%d]",
                      dev_change_pending_delete_idx);
                ble_dev_mgr_disconnect_device(dev_change_pending_delete_idx);
                ble_dev_mgr_remove_device(dev_change_pending_delete_idx);
                dev_change_pending_delete_idx = 0xFF;
                dev_change_refresh_device_list();
            }
        }
        else
        {
            LOG_D("Device change bar: cancelled device deletion");
            dev_change_pending_delete_idx = 0xFF;
        }
    }

    lv_msgbox_close(obj);
    dev_change_delete_confirm_msgbox = NULL;
}

static void dev_change_show_delete_confirm(uint8_t device_idx)
{
    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    if (!db || device_idx >= MAX_BONDED_DEVICES)
        return;

    const bonded_device_t *dev = &db->devices[device_idx];
    if (!dev->is_valid)
        return;

    if (dev_change_delete_confirm_msgbox)
    {
        lv_msgbox_close(dev_change_delete_confirm_msgbox);
        dev_change_delete_confirm_msgbox = NULL;
    }

    dev_change_pending_delete_idx = device_idx;

    static char msg_buf[128];
    lv_snprintf(msg_buf, sizeof(msg_buf), "Delete device?\n%s",
                dev->device_name);

    static const char *btns[] = {"Yes", "No", ""};

    dev_change_delete_confirm_msgbox =
        lv_msgbox_create(NULL, "Confirm Delete", msg_buf, btns, false);
    lv_obj_set_style_bg_color(dev_change_delete_confirm_msgbox,
                              lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_text_color(dev_change_delete_confirm_msgbox,
                                lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(dev_change_delete_confirm_msgbox);

    lv_obj_add_event_cb(dev_change_delete_confirm_msgbox,
                        dev_change_delete_confirm_cb, LV_EVENT_VALUE_CHANGED,
                        NULL);
}

static void dev_change_stop_delete_timer(void);

static void dev_change_delete_timer_cb(lv_timer_t *timer)
{
    if (dev_change_delete_device_idx != 0xFF)
    {
        LOG_I("Device change bar: long press delete for device [%d]",
              dev_change_delete_device_idx);
        uint8_t idx_to_delete = dev_change_delete_device_idx;
        dev_change_delete_device_idx = 0xFF;

        if (dev_change_delete_timer)
        {
            lv_timer_del(dev_change_delete_timer);
            dev_change_delete_timer = NULL;
        }

        dev_change_show_delete_confirm(idx_to_delete);
    }
}

static void dev_change_start_delete_timer(uint8_t device_idx)
{
    dev_change_delete_device_idx = device_idx;

    if (dev_change_delete_timer)
    {
        lv_timer_del(dev_change_delete_timer);
        dev_change_delete_timer = NULL;
    }

    dev_change_delete_timer = lv_timer_create(
        dev_change_delete_timer_cb, DEV_CHANGE_EXTRA_LONG_PRESS_MS - 400, NULL);
    lv_timer_set_repeat_count(dev_change_delete_timer, 1);
    LOG_D("Device change bar: delete timer started for device [%d]",
          device_idx);
}

static void dev_change_stop_delete_timer(void)
{
    if (dev_change_delete_timer)
    {
        lv_timer_del(dev_change_delete_timer);
        dev_change_delete_timer = NULL;
    }
    dev_change_delete_device_idx = 0xFF;
}

static void dev_change_content_scroll_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_SCROLL_BEGIN)
    {
        dev_change_stop_delete_timer();
    }
}

// BLE 裝置狀態變化時自動刷新列表（取代舊 menu_dev_mgr_event_cb）
// 若不註冊，手機自動重連或新配對完成時 g_conn_idx 不會更新
static void dev_change_mgr_event_cb(dev_mgr_event_t event, uint8_t device_idx,
                                    void *user_data)
{
    (void)event;
    (void)device_idx;
    (void)user_data;
    dev_change_refresh_device_list();
    refresh_connected_device_label();
}

static void dev_change_watch_btn_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (event != LV_EVENT_SHORT_CLICKED)
        return;

    LOG_I("Device change bar: Watch selected, exit mouse app");
    dev_change_watch_mode = true;
    gui_app_exit(APP_ID_MOUSE);
    lv_obj_set_tile_id(app_clock_device_change_bar, 0, 0, true);
}

static void dev_change_device_item_click_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_target(e);
    uint8_t device_idx = (uint8_t)(uintptr_t)lv_obj_get_user_data(btn);

    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    if (!db || device_idx >= MAX_BONDED_DEVICES)
        return;

    const bonded_device_t *dev = &db->devices[device_idx];
    if (!dev->is_valid)
        return;

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        dev_change_stop_delete_timer();
        if (dev->conn_idx == 0xFF)
        {
            LOG_D("Device change bar: device idx=%d (%s) not connected, ignore click",
                  device_idx, dev->device_name);
            return;
        }
        LOG_D("Device change bar: select device idx=%d, name=%s, conn_idx=%d",
              device_idx, dev->device_name, dev->conn_idx);
        dev_change_watch_mode = false;
        // 同步 HID 送出目標：沒這行的話 mouse_report_send 會繼續用舊的 g_conn_idx
        ble_hid_set_conn_idx(dev->conn_idx);
        ble_dev_mgr_set_active_device(device_idx);
        if (!gui_app_is_actived(APP_ID_MOUSE))
        {
            gui_app_run(APP_ID_MOUSE);
        }
        //
        refresh_connected_device_label();
        lv_obj_set_tile_id(app_clock_device_change_bar, 0, 0, true);
    }
    else if (LV_EVENT_PRESSED == event)
    {
        extern uint8_t get_main_phonepeer_conn_idx(void);
        uint8_t main_phone_conn_idx = get_main_phonepeer_conn_idx();
        bool is_main_phone = (main_phone_conn_idx != 0xFF &&
                              dev->conn_idx == main_phone_conn_idx);
        if (!dev_change_delete_timer && !is_main_phone)
        {
            dev_change_start_delete_timer(device_idx);
        }
    }
    else if (LV_EVENT_RELEASED == event || LV_EVENT_PRESS_LOST == event)
    {
        dev_change_stop_delete_timer();
    }
}

extern void ble_app_advertising_start(bool restart_adv, bool mouse_mode,
                                      bool pairing_mode);

static void dev_change_add_device_btn_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_SHORT_CLICKED)
    {
        LOG_I("Device change bar: Add device (start advertising)");
        ble_app_advertising_start(true, true, false);
        dev_change_refresh_device_list();
    }
}

static void dev_change_refresh_device_list(void)
{
    LOG_D("Device change bar: refresh device list");
    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    if (!db)
    {
        LOG_D("Device change bar: no bonded device database (Watch/Add "
              "Device will still be shown)");
    }

    // 把 HID 目標同步到當前 active device（跟舊 menu_refresh_device_list 相同）
    // 防止自動重連/配對等情況下 g_conn_idx 停在舊值
    if (db)
    {
        int active_idx = ble_dev_mgr_get_active_device();
        if (active_idx >= 0 && active_idx < MAX_BONDED_DEVICES &&
            db->devices[active_idx].is_valid &&
            db->devices[active_idx].conn_idx != 0xFF)
        {
            ble_hid_set_conn_idx(db->devices[active_idx].conn_idx);
        }
    }
    LOG_D("Device change bar : DEBUG 1");
    if (dev_change_list_ui.watch_list &&
        lv_obj_is_valid(dev_change_list_ui.watch_list))
    {
        lv_obj_clean(dev_change_list_ui.watch_list);
    }

    if (dev_change_list_ui.device_list &&
        lv_obj_is_valid(dev_change_list_ui.device_list))
    {
        lv_obj_clean(dev_change_list_ui.device_list);
    }
    dev_change_list_ui.connecting_btn = NULL;
    dev_change_list_ui.connecting_label = NULL;

    if (dev_change_list_ui.empty_label &&
        lv_obj_is_valid(dev_change_list_ui.empty_label))
    {
        lv_obj_add_flag(dev_change_list_ui.empty_label, LV_OBJ_FLAG_HIDDEN);
    }

    if (!dev_change_list_ui.device_list ||
        !lv_obj_is_valid(dev_change_list_ui.device_list))
    {
        LOG_D("Device change bar : DEBUG 2");
        return;
    }

    // Watch button - in a dedicated row container
    {
        lv_obj_t *watch_btn = lv_btn_create(dev_change_list_ui.watch_list);
        lv_obj_set_size(watch_btn, DEV_CHANGE_BUTTON_WIDTH,
                        DEV_CHANGE_BUTTON_HEIGHT);
        lv_obj_set_style_radius(watch_btn, 80, 0);
        lv_obj_set_style_bg_opa(watch_btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_bg_color(watch_btn, lv_color_hex(0x2A2A2A), 0);
        lv_obj_set_style_bg_color(watch_btn, lv_color_hex(0x3A3A3A),
                                  LV_STATE_PRESSED);
        lv_obj_set_style_pad_all(watch_btn, 6, 0);

        lv_obj_t *device_bg = lv_img_create(watch_btn);
        lv_img_set_src(device_bg, &device_btn);
        lv_obj_align(device_bg, LV_ALIGN_CENTER, 0, 0);

        if (dev_change_watch_mode)
        {
            lv_obj_set_style_border_width(watch_btn, 2, 0);
            lv_obj_set_style_border_color(watch_btn, lv_color_hex(0x00AAFF), 0);
        }
        else
        {
            lv_obj_set_style_border_width(watch_btn, 0, 0);
        }

        lv_obj_add_event_cb(watch_btn, dev_change_watch_btn_cb,
                            LV_EVENT_SHORT_CLICKED, NULL);

        // Green LED indicator (always connected)
        lv_obj_t *watch_led = lv_obj_create(watch_btn);
        lv_obj_set_size(watch_led, 12, 12);
        lv_obj_set_style_radius(watch_led, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(watch_led, 0, 0);
        lv_obj_set_style_pad_all(watch_led, 0, 0);
        lv_obj_align(watch_led, LV_ALIGN_LEFT_MID, 0, 0);
        lv_obj_clear_flag(watch_led,
                          LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(watch_led, lv_color_hex(0x00FF00), 0);
        lv_obj_set_style_shadow_color(watch_led, lv_color_hex(0x00FF00), 0);
        lv_obj_set_style_shadow_width(watch_led, 8, 0);
        lv_obj_set_style_shadow_spread(watch_led, 2, 0);

        lv_obj_t *watch_label = lv_label_create(watch_btn);
        lv_label_set_text(watch_label, "Watch");
        lv_obj_set_style_text_color(watch_label, lv_color_hex(0xFFFFFF), 0);
        lv_obj_align(watch_label, LV_ALIGN_LEFT_MID, 18, 0);
        lv_obj_clear_flag(watch_label, LV_OBJ_FLAG_CLICKABLE);
    }

    // Device buttons - two per row, all same size
    if (db)
    {
        for (int i = 0; i < MAX_BONDED_DEVICES; i++)
        {
            if (db->devices[i].is_valid)
            {
                lv_obj_t *item = dev_change_create_device_item(
                    dev_change_list_ui.device_list, &db->devices[i], i);
                lv_obj_add_event_cb(item, dev_change_device_item_click_cb,
                                    LV_EVENT_ALL, NULL);
            }
        }
    }

    // Connecting... button (shown while BLE is advertising/pairing)
    if (get_bluetooth_broadcasting_status())
    {
        lv_obj_t *conn_btn = lv_btn_create(dev_change_list_ui.device_list);
        lv_obj_set_size(conn_btn, DEV_CHANGE_BUTTON_WIDTH,
                        DEV_CHANGE_BUTTON_HEIGHT);
        lv_obj_set_style_radius(conn_btn, 80, 0);
        lv_obj_set_style_bg_opa(conn_btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_bg_color(conn_btn, lv_color_hex(0x2A2A2A), 0);
        lv_obj_set_style_bg_color(conn_btn, lv_color_hex(0x3A3A3A),
                                  LV_STATE_PRESSED);
        lv_obj_set_style_pad_all(conn_btn, 6, 0);
        lv_obj_set_style_border_width(conn_btn, 0, 0);
        lv_obj_clear_flag(conn_btn, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_t *conn_bg = lv_img_create(conn_btn);
        lv_img_set_src(conn_bg, &device_btn);
        lv_obj_align(conn_bg, LV_ALIGN_CENTER, 0, 0);

        lv_obj_t *conn_label = lv_label_create(conn_btn);
        lv_label_set_text(conn_label, "Connecting.");
        lv_obj_set_style_text_color(conn_label, lv_color_hex(0x9CB5FF), 0);
        lv_obj_center(conn_label);
        lv_obj_clear_flag(conn_label, LV_OBJ_FLAG_CLICKABLE);

        dev_change_list_ui.connecting_btn = conn_btn;
        dev_change_list_ui.connecting_label = conn_label;
        dev_change_start_connecting_timer();
    }
    else
    {
        dev_change_stop_connecting_timer();
    }

    // Add Device button
    {
        lv_obj_t *add_btn = lv_btn_create(dev_change_list_ui.device_list);
        lv_obj_set_size(add_btn, DEV_CHANGE_BUTTON_WIDTH,
                        DEV_CHANGE_BUTTON_HEIGHT);
        lv_obj_set_style_radius(add_btn, 80, 0);
        lv_obj_set_style_bg_opa(add_btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_bg_color(add_btn, lv_color_hex(0x2A2A2A), 0);
        lv_obj_set_style_bg_color(add_btn, lv_color_hex(0x3A3A3A),
                                  LV_STATE_PRESSED);
        lv_obj_set_style_pad_all(add_btn, 6, 0);
        lv_obj_set_style_border_width(add_btn, 0, 0);
        lv_obj_add_event_cb(add_btn, dev_change_add_device_btn_cb,
                            LV_EVENT_SHORT_CLICKED, NULL);
        lv_obj_t *device_bg = lv_img_create(add_btn);
        lv_img_set_src(device_bg, &device_btn);
        lv_obj_align(device_bg, LV_ALIGN_CENTER, 0, 0);

        lv_obj_t *add_label = lv_label_create(add_btn);
        lv_label_set_text(add_label, "Add Device");
        lv_obj_set_style_text_color(add_label, lv_color_hex(0x9CB5FF), 0);
        lv_obj_center(add_label);
    }
}

static lv_obj_t *status_bar_device_bg = NULL;

static void set_dev_change_gaus_opa(uint8_t opa)
{
    if (lv_obj_is_valid(dev_change_gaus_img))
    {
        lv_obj_set_style_img_opa(dev_change_gaus_img, opa,
                                 LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}

void app_clock_device_change_bar_init(lv_obj_t *par)
{
    // Gaussian blur background
    dev_change_gaus_bg = lv_obj_create(par);
    lv_obj_set_size(dev_change_gaus_bg, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(dev_change_gaus_bg, LV_COLOR_BLACK,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(dev_change_gaus_bg, LV_OPA_TRANSP,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(dev_change_gaus_bg);
    lv_obj_add_flag(dev_change_gaus_bg, LV_OBJ_FLAG_HIDDEN);

    dev_change_gaus_img = lv_img_create(dev_change_gaus_bg);
    lv_img_set_src(dev_change_gaus_img, GAUS_CLOCK1_BG);
    lv_obj_set_style_img_opa(dev_change_gaus_img, LV_OPA_0,
                             LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_img_set_zoom(dev_change_gaus_img, 256 * 2);
    lv_obj_center(dev_change_gaus_img);

    status_bar_device_bg = lv_obj_create(par);
    lv_obj_set_size(status_bar_device_bg, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_opa(status_bar_device_bg, LV_OPA_0,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(status_bar_device_bg, LV_COLOR_WHITE,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(status_bar_device_bg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_pos(status_bar_device_bg, 0, 0);

    lv_obj_t *status_bar_area;
    rt_uint16_t i;
    lv_obj_t *pages[2];
    status_bar_area = lv_obj_create(status_bar_device_bg);
    lv_obj_set_size(status_bar_area, (LV_HOR_RES_MAX >> 4),
                    (LV_VER_RES_MAX >> 2));
    lv_obj_set_scrollbar_mode(status_bar_area, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(status_bar_area,
                      LV_OBJ_FLAG_PRESS_LOCK); // Allow press event to tileview
    lv_obj_set_style_bg_opa(status_bar_area, bar_opa,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(status_bar_area, device_change_bar_cb, LV_EVENT_ALL,
                        NULL);
    lv_obj_align(status_bar_area, LV_ALIGN_RIGHT_MID, 0, 0);
    device_change_bar_area_right = status_bar_area;

    app_clock_device_change_bar = lv_tileview_create(status_bar_device_bg);
    lv_obj_set_scrollbar_mode(app_clock_device_change_bar,
                              LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(app_clock_device_change_bar, LV_COLOR_BLACK,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(app_clock_device_change_bar, LV_OPA_TRANSP,
                            LV_PART_MAIN | LV_STATE_DEFAULT);

    // Create a single tile for AI status bar
    for (i = 0; i < 2; i++)
    {
        pages[i] =
            lv_tileview_add_tile(app_clock_device_change_bar, i, 0, LV_DIR_HOR);
        if (i == 0)
        {
            lv_obj_add_event_cb(pages[i],
                                app_clock_main_device_change_bar_event_cb,
                                LV_EVENT_ALL, NULL);
            lv_obj_set_style_bg_color(pages[i], LV_COLOR_BLACK,
                                      LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        else
        {
            lv_obj_set_style_bg_color(pages[i], LV_COLOR_WHITE,
                                      LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_clear_flag(pages[i], LV_OBJ_FLAG_SCROLLABLE);
        }
        // if (g_ble_ulog_enable)
        // {
        //     lv_obj_set_style_bg_opa(pages[i], LV_OPA_50, LV_PART_MAIN |
        //     LV_STATE_DEFAULT);
        // }
        // else
        // {
        lv_obj_set_style_bg_opa(pages[i], LV_OPA_TRANSP,
                                LV_PART_MAIN | LV_STATE_DEFAULT);
        // }
        lv_obj_set_size(pages[i], LV_HOR_RES_MAX, LV_VER_RES_MAX);
        lv_obj_set_scrollbar_mode(pages[i], LV_SCROLLBAR_MODE_OFF);
    }

    // Build vertical device list on pages[1]
    {
        lv_obj_t *dev_bg = lv_obj_create(pages[1]);
        lv_obj_t *content_area;
        lv_obj_set_size(dev_bg, LV_HOR_RES_MAX, LV_VER_RES_MAX);
        lv_obj_set_style_bg_color(dev_bg, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(dev_bg, LV_OPA_TRANSP, 0);
        lv_obj_set_style_radius(dev_bg, 0, 0);
        lv_obj_set_style_border_width(dev_bg, 0, 0);
        lv_obj_align(dev_bg, LV_ALIGN_CENTER, 0, 0);
        lv_obj_clear_flag(dev_bg, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(dev_bg, 0, 0);

        // Title
        // lv_obj_t *title_label = lv_label_create(dev_bg);
        // lv_label_set_text(title_label, "Devices");
        // lv_obj_set_style_text_color(title_label, lv_color_hex(0xFFFFFF), 0);
        // lv_obj_set_width(title_label, LV_PCT(100));
        // lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_CENTER, 0);
        // lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 15);

        // Content area under title keeps the button layout in flex mode.
        content_area = lv_obj_create(dev_bg);
        lv_obj_set_size(content_area,
                        LV_HOR_RES_MAX - (DEV_CHANGE_CONTENT_SIDE_MARGIN * 2),
                        DEV_CHANGE_CONTENT_HEIGHT);
        lv_obj_set_style_bg_opa(content_area, LV_OPA_0, 0);
        lv_obj_set_style_border_width(content_area, 0, 0);
        lv_obj_set_style_pad_top(content_area, DEV_CHANGE_BUTTON_GAP, 0);
        lv_obj_set_style_pad_bottom(content_area, 0, 0);
        lv_obj_set_style_pad_left(content_area, 0, 0);
        lv_obj_set_style_pad_right(content_area, 0, 0);
        lv_obj_set_flex_flow(content_area, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(content_area, LV_FLEX_ALIGN_START,
                              LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_row(content_area, 0, 0);
        lv_obj_set_scrollbar_mode(content_area, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scroll_dir(content_area, LV_DIR_VER);
        lv_obj_add_flag(content_area, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(content_area, LV_OBJ_FLAG_SCROLL_ELASTIC);
        lv_obj_add_event_cb(content_area, dev_change_content_scroll_cb,
                            LV_EVENT_SCROLL_BEGIN, NULL);
        // lv_obj_align_to(content_area, title_label, LV_ALIGN_OUT_BOTTOM_MID,
        // 0,
        //                 0);
        lv_obj_align(content_area, LV_ALIGN_TOP_MID, 0, 0);

        // Dedicated row for watch button (keeps watch independent from grid)
        lv_obj_t *watch_list = lv_obj_create(content_area);
        lv_obj_set_size(watch_list, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(watch_list, LV_OPA_0, 0);
        lv_obj_set_style_border_width(watch_list, 0, 0);
        lv_obj_set_style_pad_all(watch_list, 0, 0);
        lv_obj_set_flex_flow(watch_list, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(watch_list, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(watch_list, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_bottom(watch_list, DEV_CHANGE_BUTTON_GAP, 0);
        dev_change_list_ui.watch_list = watch_list;

        // Device list container (not scrollable - content_area handles
        // scrolling)
        lv_obj_t *device_list = lv_obj_create(content_area);
        lv_obj_set_size(device_list, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(device_list, LV_OPA_0, 0);
        lv_obj_set_style_border_width(device_list, 0, 0);
        lv_obj_set_style_pad_all(device_list, 0, 0);
        lv_obj_set_flex_flow(device_list, LV_FLEX_FLOW_ROW_WRAP);
        lv_obj_set_flex_align(device_list, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(device_list, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_row(device_list, DEV_CHANGE_BUTTON_GAP, 0);
        lv_obj_set_style_pad_column(device_list, DEV_CHANGE_BUTTON_GAP, 0);
        dev_change_list_ui.device_list = device_list;

        // Empty state label
        lv_obj_t *empty_label = lv_label_create(content_area);
        lv_label_set_text(empty_label, "No paired devices");
        lv_obj_set_style_text_color(empty_label, lv_color_hex(0x888888), 0);
        lv_obj_add_flag(empty_label, LV_OBJ_FLAG_HIDDEN);
        dev_change_list_ui.empty_label = empty_label;

        // Invisible spacer to force content_area to always be scrollable
        // (LVGL only allows scroll when children overflow the container)
        lv_obj_t *spacer = lv_obj_create(content_area);
        lv_obj_set_size(spacer, 1, DEV_CHANGE_BUTTON_HEIGHT);
        lv_obj_set_style_bg_opa(spacer, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(spacer, 0, 0);
        lv_obj_set_style_pad_all(spacer, 0, 0);
        lv_obj_clear_flag(spacer, LV_OBJ_FLAG_CLICKABLE);

        // Device list will be refreshed when user swipes to this page
        // (in app_clock_device_change_bar_event_cb VALUE_CHANGED)
    }

    // Add event callback for AI status bar tile
    lv_obj_add_event_cb(app_clock_device_change_bar,
                        app_clock_device_change_bar_event_cb, LV_EVENT_ALL,
                        NULL);

    // Set the tile ID to the AI status bar tile
    lv_obj_set_tile_id(app_clock_device_change_bar, 0, 0, false);
    lv_obj_add_flag(app_clock_device_change_bar, LV_OBJ_FLAG_HIDDEN);

    // 監聽 BLE 裝置事件，連線/配對/斷線時自動刷新並同步 g_conn_idx
    ble_dev_mgr_register_callback(dev_change_mgr_event_cb, NULL);
}

void set_status_bar_area_down_state(bool state)
{
    if (lv_obj_is_valid(status_bar_area_down) == false)
    {
        return;
    }
    if (state)
    {
        lv_obj_clear_flag(status_bar_area_down, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(status_bar_area_down, LV_OBJ_FLAG_HIDDEN);
    }
}

void set_status_bar_area_up_state(bool state)
{
    if (lv_obj_is_valid(status_bar_area_up) == false)
    {
        return;
    }
    if (state)
    {
        lv_obj_clear_flag(status_bar_area_up, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(status_bar_area_up, LV_OBJ_FLAG_HIDDEN);
    }
}

void set_status_bar_area_right_state(bool state)
{
    if (lv_obj_is_valid(status_bar_area_right) == false)
    {
        return;
    }
    LOG_D("set_status_bar_area_ai_state %d", state);
    if (state)
    {
        lv_obj_clear_flag(status_bar_area_right, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(status_bar_area_right, LV_OBJ_FLAG_HIDDEN);
    }
}

void set_status_bar_area_left_state(bool state)
{
    if (lv_obj_is_valid(status_bar_area_left) == false)
    {
        return;
    }
    LOG_D("set_status_bar_area_left_state %d", state);
    if (state)
    {
        lv_obj_clear_flag(status_bar_area_left, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(status_bar_area_left, LV_OBJ_FLAG_HIDDEN);
    }
}

void app_clock_main_status_bar_deinit(void)
{
    ai_interface_tileview_index = 0;
    middle_layer_tileview_index = 1;
    if (status_bar_bg_main && lv_obj_is_valid(status_bar_bg_main))
    {
        lv_obj_del(status_bar_bg_main);
        status_bar_bg_main = NULL;
    }
    if (status_bar_bg_ai && lv_obj_is_valid(status_bar_bg_ai))
    {
        lv_obj_del(status_bar_bg_ai);
        status_bar_bg_ai = NULL;
        extern void clean_ai_gesture_indicator(void);
        clean_ai_gesture_indicator();
    }
    lv_ext_font_reset();
    if (p_app_media)
    {
        lv_mem_free(p_app_media);
        p_app_media = NULL;
    }

#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_media_volume = NULL;
    lvgl_msg_handler.handle_notification = NULL;
    lvgl_msg_handler.handle_bar_media_play_state = NULL;
    lvgl_msg_handler.handle_bar_media_title = NULL;
#endif
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/