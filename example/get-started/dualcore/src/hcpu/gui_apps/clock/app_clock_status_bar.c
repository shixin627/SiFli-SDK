#include "app_clock_status_bar.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
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
#include "lv_ext_resource_manager.h"

extern void refresh_connected_device_label(void);
#ifndef _MSC_VER
__attribute__((weak)) void refresh_connected_device_label(void)
{
}
#endif

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

LV_IMG_DECLARE(sun);
LV_IMG_DECLARE(mouse_mode_icon);
LV_IMG_DECLARE(micro_icon); /* recorder glyph for the quick-settings panel */
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
static lv_obj_t *status_bar_area_up;
static lv_obj_t *status_bar_area_down;
static lv_obj_t *status_bar_area_left;
static lv_obj_t *status_bar_area_right;

static rt_bool_t dndmode_enabled = RT_FALSE;

void display_status_bar_area(uint32_t idx, bool display)
{
    if (idx >= 4)
    {
        LOG_E("Invalid index %d for status bar area", idx);
        return;
    }
    /* idx 3 = right-edge handle. Repointed from the old device_change_bar zone
       to status_bar_area_right so the watch-face right-edge pull reveals the
       device_pager tile (mirrors idx 2 / left instruction_list). The old
       right-edge zone stays permanently hidden. */
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
    if (lv_disp_get_rotation(NULL) == LV_DISP_ROT_90 ||
        lv_disp_get_rotation(NULL) == LV_DISP_ROT_270)
    {
        return;
    }

    lv_obj_t *obj = lv_event_get_target(event);
    status_bar_area_t area_id = (status_bar_area_t)lv_obj_get_user_data(obj);

    if (LV_EVENT_RELEASED == event->code)
    {
        LOG_I("LV_EVENT_RELEASED_Clock from area: %d", area_id);
        lv_obj_add_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
    }
    else if (LV_EVENT_PRESSED == event->code)
    {
        LOG_I("notification_status_bar_cb from area: %d", area_id);
        if (area_id == STATUS_BAR_AREA_LEFT)
        {
            /* L/R swap: device_pager is now the LEFT tile. Populate it NOW, on
               touch — before the left tile is dragged into view — so its content
               follows the finger instead of popping in on release (VALUE_CHANGED
               only fires on scroll-settle). Also re-reads the latest bonded set
               on every pull-out. */
            extern void device_pager_refresh(void);
            device_pager_refresh();
        }
        else if (area_id == STATUS_BAR_AREA_RIGHT)
        {
            /* Reset the App List (col 2) to its top row before it slides in — it's
               still off-screen here, so the reset is invisible — so re-entering the
               page always starts at the top instead of its last scroll position. */
            extern void lv_app_list_layout_reset_scroll(void);
            lv_app_list_layout_reset_scroll();
        }
        lv_obj_set_tile_id(app_clock_main_status_bar, 1, 1, false);
        lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
        set_clock_main_status_opa(0, false);
    }
}

/* ---- App List finger-follow from a full-screen face swipe ------------------ *
 * The watch-face swipe catcher (app_clock_main.c) calls these on a LEFTWARD pull to
 * drive the App List (col-2 tile) in from the right, finger-following — the same
 * result the 58px right edge zone gives, but the catcher (not the tileview) owns the
 * press, so we forward the scroll ourselves (sole writer → no jitter) and commit/
 * abort with set_tile_id on release; the VALUE_CHANGED settle then re-hides on a
 * land-back-HOME or wires up the App List page state. */
void clock_main_applist_follow_begin(void)
{
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    extern void lv_app_list_layout_reset_scroll(void);
    lv_app_list_layout_reset_scroll();          /* enter at the top row */
    lv_obj_set_tile_id(app_clock_main_status_bar, 1, 1, false); /* snap HOME, no anim */
    lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
    set_clock_main_status_opa(0, false);
}

void clock_main_applist_follow_update(lv_coord_t dx)
{
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    /* dx < 0 = leftward pull. HOME sits at scroll_x == LV_HOR_RES (466); the App List
       (col 2) at 2*LV_HOR_RES. Pull the scroll from HOME toward the App List. */
    lv_coord_t sx = LV_HOR_RES - dx; /* dx<0 -> sx>466 toward 932 */
    if (sx < LV_HOR_RES) sx = LV_HOR_RES;
    if (sx > 2 * LV_HOR_RES) sx = 2 * LV_HOR_RES;
    lv_obj_scroll_to_x(app_clock_main_status_bar, sx, LV_ANIM_OFF);
}

void clock_main_applist_follow_end(lv_coord_t dx, lv_coord_t vx)
{
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    /* Commit to the App List if pulled past a quarter screen OR flung left fast;
       else animate back HOME (the settle handler re-hides the tileview there). */
    bool open_it = (dx <= -LV_HOR_RES / 4) || (vx < -6);
    lv_obj_set_tile_id(app_clock_main_status_bar, open_it ? 2 : 1, 1, true);
}

static void dev_change_refresh_device_list(void);
static void dev_change_stop_connecting_timer(void);

static void device_change_bar_cb(lv_event_t *event)
{
    if (lv_disp_get_rotation(NULL) != LV_DISP_ROT_90 &&
        lv_disp_get_rotation(NULL) != LV_DISP_ROT_270)
    {
        /* T4: 右緣入口改由錶面 tileview 右 tile 的原生滑動處理；舊 bar 的觸碰區
           在 init 時已設 HIDDEN，這個 cb 平時不會觸發。保留原邏輯以防被顯示。 */
        if (LV_EVENT_RELEASED == event->code)
        {
            lv_obj_add_flag(app_clock_device_change_bar, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(dev_change_gaus_bg, LV_OBJ_FLAG_HIDDEN);
            set_dev_change_gaus_opa(LV_OPA_0);
            dev_change_stop_connecting_timer();
        }
        else if (LV_EVENT_PRESSED == event->code)
        {
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
bool main_clock_tileview_scrollable = true;
static uint8_t shady_transparency = 0;
static uint16_t bg_opa = LV_OPA_COVER;
static uint16_t bg_opa_2 = LV_OPA_COVER;
static uint16_t bg_opa_3 = LV_OPA_50;
static uint8_t middle_layer_tileview_index = 255;
static uint8_t ai_interface_tileview_index = 0;

static void app_clock_main_status_bar_event_cb(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);

    switch (event->code)
    {
    case LV_EVENT_SCROLL:
    {
        /* App List tile (col 2) reveal: home sits at scroll_x==466; pulling RIGHT
           toward the App List raises scroll_x toward 932. Finger-follow the SAME
           blurred-dial + top time/weather fade-in the left mixed-list reveal uses,
           so the App List slides in with the dial gradually blurring and the time
           fading IN — not popping straight to the settled state. Historically col 2
           was an unreachable placeholder, so only the leftward/vertical reveals had a
           finger-follow fade. The settle (VALUE_CHANGED, MAIN_PAGE_TYPE_LEFT) already
           lands on this same blurred-dial + OPA_100 time state. */
        {
            lv_coord_t sx = lv_obj_get_scroll_x(obj);
            if (sx > 466)
            {
                extern void instruction_list_bar_set_blur_amount(uint8_t opa);
                lv_coord_t pull = sx - 466; /* 0..466 */
                lv_coord_t opa = pull * 255 / 466;
                if (opa > 255) opa = 255;
                instruction_list_bar_set_blur_amount((uint8_t)opa);
                set_instruction_list_time_opa((uint8_t)opa);
                break;
            }
        }
        /* Left tile (device_pager) reveal: darken the whole watch face to black as
           the page is pulled out — a full-screen fade-to-black on gaus_dial_bg.
           Home sits at scroll_x == 466; pulling LEFT toward the device tile (now the
           (0,1) tile) lowers it toward 0, so rx = 466 - scroll_x rises with the pull.
           ropa is 0 at home, so an idle home keeps its transparent bg / blur. */
        {
            lv_coord_t rx = 466 - lv_obj_get_scroll_x(obj);
            if (rx < 0) rx = 0;
            lv_coord_t ropa = rx * 255 / 350;
            if (ropa > 255) ropa = 255;
            if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
                lv_obj_set_style_bg_opa(gaus_dial_bg, (lv_opa_t)ropa, 0);
            if (ropa > 0 && gaus_dial_img && lv_obj_is_valid(gaus_dial_img))
                lv_obj_set_style_img_opa(gaus_dial_img, LV_OPA_0, 0); /* right: black only, no blur */
        }
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
            ((abs(scroll_x) < (bg_opa + 1)) && (scroll_x != 0)))
        {
            if (scroll_y == 0)
            {
                shady_transparency = abs(scroll_x);
                set_clock_main_status_opa(shady_transparency, false);
            }
            else
            {
                shady_transparency = abs(scroll_y);
                set_clock_main_status_opa(shady_transparency, true);
            }
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
                    /* Fade the top battery out together with the time as we pull
                       toward the device page (RIGHT) — it's hidden there anyway
                       (show_battery(false) on settle), so without this it rides
                       along the whole slide and only pops out on release. Same
                       HOME->RIGHT scroll-x branch. Per the user: turn the battery
                       fully OFF (0) the moment the rightward slide starts, not a
                       gradual fade, so it never shows on the way to the mouse. */
                    set_instruction_list_battery_opa(LV_OPA_TRANSP);
                }
            }
            // else
            // {
            //     shady_transparency = -scroll_second_x;
            //     set_instruction_list_battery_opa(shady_transparency);
            // }
        }
        else
        {
            shady_transparency = abs(scroll_second_y);
            if (shady_transparency < (bg_opa_2 + 1) &&
                shady_transparency > 0)
            {
                set_instruction_list_battery_opa(shady_transparency);
            }
            else if (shady_transparency >= bg_opa_2)
            {
                set_instruction_list_battery_opa(bg_opa_2);
            }
        }

        if (scroll_second_y == 0)
        {
            shady_transparency = scroll_second_x;
        }
        else
        {
            shady_transparency = scroll_second_y;
        }
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
        /* Global input bar (lv_layer_top): show ONLY on the watch face (HOME) and
           the mouse page (RIGHT) — a tap floats the shared list in place. Hidden on
           the (empty post-R3) instruction_list LEFT tile, message (UP) and
           control-center (DOWN). The gui_app_is_actived("Main") gate matters when an
           app is layered over the watch face (e.g. notification detail / voice reply
           opened from the message list — animate_to_home_from_notification_center
           slides THIS tileview back to HOME): that settle fires VALUE_CHANGED with
           active_pos==HOME while Main is paused, which without the gate re-shows the
           bar on top of that app. With the gate it stays hidden. (R3 stage 2: the
           mouse page uses THIS bar, not device_pager's own — kept hidden in
           device_pager_set_active.) */
        {
            extern void instruction_list_bar_set_visible(bool visible);
            instruction_list_bar_set_visible((active_pos == MAIN_PAGE_TYPE_HOME ||
                                              active_pos == MAIN_PAGE_TYPE_RIGHT) &&
                                             gui_app_is_actived("Main"));
        }
        {
            /* Right tile = the device control page. Host the mouse behind the
               list while we're on it; tear it down when we leave. Also re-read
               the bonded-device set on entry. */
            extern void device_pager_refresh(void);
            extern void device_pager_set_active(bool on);
            bool on_device_page = (active_pos == MAIN_PAGE_TYPE_RIGHT);
            device_pager_set_active(on_device_page);
            /* Hide the top battery indicator on the mouse/device page (user
               request); show it again on every other page. Its per-page content
               opacity still governs whether it actually appears there, so showing
               the container on a page that keeps it transparent is harmless. */
            {
                extern void show_battery(bool show);
                show_battery(!on_device_page);
            }
            /* settle: full-screen black backdrop on the device page (gaus_dial_bg),
               cleared for the left/down/up reveals so their blur shows instead */
            if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
                lv_obj_set_style_bg_opa(gaus_dial_bg,
                                        on_device_page ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
            if (on_device_page)
            {
                if (gaus_dial_img && lv_obj_is_valid(gaus_dial_img))
                    lv_obj_set_style_img_opa(gaus_dial_img, LV_OPA_0, 0);
                device_pager_refresh();
            }
            /* Left tile = the instruction_list voice page. Auto-dismiss its open
               voice input box when we navigate away (mirrors the device page
               closing its skaibar on leave); the box only self-dismisses on a
               swipe that starts on it, so an edge-back / page switch would
               otherwise leave it open. */
            extern void instruction_list_close_ai_on_leave(void);
            if (active_pos != MAIN_PAGE_TYPE_LEFT)
            {
                instruction_list_close_ai_on_leave();
            }
        }
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
        if (scroll_y == 466 && scroll_x == 466)
        {
            shady_transparency = 0;
            set_clock_main_status_opa(shady_transparency, false);
            lv_obj_add_flag(myLancher[app_index_message].pagetileview,
                            LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
            set_clock_main_status_opa(LV_OPA_0, false);
            set_instruction_list_time_opa(LV_OPA_0);

            set_instruction_list_battery_opa(LV_OPA_TRANSP);
        }
        else
        {
            lv_obj_clear_flag(myLancher[app_index_message].pagetileview,
                              LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
            if (active_pos == MAIN_PAGE_TYPE_LEFT)
            {
                set_instruction_list_time_opa(LV_OPA_100);
            }
            else
            {
                set_instruction_list_time_opa(LV_OPA_0);
            }
            if (active_pos == MAIN_PAGE_TYPE_UP)
            {
                set_clock_main_status_opa(LV_OPA_100, true);
            }
            else if (active_pos == MAIN_PAGE_TYPE_RIGHT)
            {
                /* device page wants a PURE BLACK backdrop (gaus_dial_bg at
                   COVER), not the blurred dial — keep the blur image hidden so
                   the settle doesn't re-reveal it after the drag faded to black. */
                set_clock_main_status_opa(LV_OPA_0, false);
            }
            else
            {
                set_clock_main_status_opa(LV_OPA_100, false);
            }
            if (active_pos == 0)
            {
                set_instruction_list_battery_opa(LV_OPA_100);
            }

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
    default:
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
            lv_obj_clear_flag(dev_change_gaus_bg, LV_OBJ_FLAG_HIDDEN);
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
        lv_obj_add_flag(dev_change_gaus_bg, LV_OBJ_FLAG_HIDDEN);
        break;
    }
    default:
        break;
    }
}

void animate_to_notification_center(void)
{
    lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(myLancher[app_index_message].pagetileview,
                      LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_tile_id(myLancher[app_index_message].pagetileview, 0, 1,
                       LV_ANIM_ON);
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

/* ---- Floating instruction-list backdrop (R3 refactor) ----------------------
   The instruction list now floats on lv_layer_top instead of occupying the LEFT
   tile, so tapping the bar no longer scrolls to a tile that fades the blurred
   dial in. instruction_list_bar_set_blur(true) shows the SAME gaus_dial_bg the
   LEFT reveal used (transparent black layer + blurred dial image, not the device
   page's full-black COVER). It is gated by s_bar_blur_active so set_blur(false)
   only clears a blur WE turned on — never the tile-scroll-driven blur of another
   page. */
static bool s_bar_blur_active = false;
void instruction_list_bar_set_blur(bool on)
{
    if (!gaus_dial_bg || !lv_obj_is_valid(gaus_dial_bg)) return;
    if (on)
    {
        s_bar_blur_active = true;
        lv_obj_set_style_bg_opa(gaus_dial_bg, LV_OPA_TRANSP, 0);
        set_clock_main_status_opa(LV_OPA_100, false); /* blurred dial image in */
        lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        if (!s_bar_blur_active) return; /* not ours — leave any tile-reveal blur */
        s_bar_blur_active = false;
        set_clock_main_status_opa(LV_OPA_0, false);
        lv_obj_add_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
    }
}

/* Same gaus_dial_bg blur as instruction_list_bar_set_blur, but at an arbitrary
   strength so the left reveal can fade it in WITH the pull (finger-follow),
   mirroring app_clock_device_change_bar_event_cb's scroll→opa ramp. Turn it back
   OFF with instruction_list_bar_set_blur(false). */
void instruction_list_bar_set_blur_amount(uint8_t opa)
{
    if (!gaus_dial_bg || !lv_obj_is_valid(gaus_dial_bg)) return;
    s_bar_blur_active = true;
    lv_obj_set_style_bg_opa(gaus_dial_bg, LV_OPA_TRANSP, 0);
    set_clock_main_status_opa(opa, false);
    lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
}

/* True when the watch face (HOME) is the current main page. The floating list's
   open path uses this to blur the dial in place (no tile switch) on the watch
   face, while other pages keep the legacy tile path for now. */
bool clock_main_page_is_home(void)
{
    return middle_layer_tileview_index == MAIN_PAGE_TYPE_HOME;
}

void chack_tile_page(void)
{
    LOG_D("chack_tile_page: %d, %d",
          lv_obj_get_scroll_x(myLancher[app_index_message].pagetileview),
          lv_obj_get_scroll_y(myLancher[app_index_message].pagetileview));
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
        setting_provider.set_dnd_status(dndmode_enabled);
#endif
        if (dnd_mode_btn != NULL)
        {
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

static void find_phone_btn_event_cb(lv_event_t *e)
{
    control_provider.find_phone();
}

#if !kReleaseMode
static void gesture_test_btn_event_cb(lv_event_t *e)
{
    gui_app_run(APP_ID_GESTURE);
    animate_to_home_from_notification_center();
}
#endif

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

extern void build_media_contorll_widget(app_media_t *p_app_media,
                                        lv_obj_t *parent);
extern lv_obj_t *lv_media_widget_builder(lv_obj_t *parent);

/* Brightness slider handler for the quick-settings panel. Restored 2026-06-25
   alongside control_center_layout_create (was removed when the page was gutted).
   Drag maps touch-x → brightness; press/release toggles tileview scroll so a
   horizontal drag on the bar doesn't slide the page. */
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

        lv_coord_t value = w ? (rel_x * (max - min)) / w + min : min;
        if (value < 5)
            value = 5;
        lv_bar_set_value(bar, value, LV_ANIM_OFF);
        uint16_t brightness = lv_bar_get_value(bar);
        gui_set_brightness(brightness, true);
    }
    else if (code == LV_EVENT_PRESSED)
    {
        lv_obj_clear_flag(myLancher[app_index_message].pagetileview,
                          LV_OBJ_FLAG_SCROLLABLE);
    }
    else if (code == LV_EVENT_RELEASED)
    {
        lv_obj_add_flag(myLancher[app_index_message].pagetileview,
                        LV_OBJ_FLAG_SCROLLABLE);
    }
}

static lv_obj_t *control_center_window;
static lv_obj_t *control_center_app_list = NULL;
static lv_obj_t *control_center_layout_create(lv_obj_t *parent)
{
    control_center_window = lv_obj_create(parent);
    lv_obj_set_size(control_center_window, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_scrollbar_mode(control_center_window, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(control_center_window, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(control_center_window, LV_COLOR_WHITE,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(control_center_window, LV_OPA_0,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(control_center_window, 0, 0);
    lv_obj_set_style_pad_all(control_center_window, 0, 0);
    lv_obj_add_flag(control_center_window, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(control_center_window, LV_DIR_VER);

    /* 2026-06-25: restored the quick-settings panel (brightness + tool buttons)
       on the swipe-up-from-bottom page. The app grid moved to the right-swipe
       App List tile; quick settings live here again. Mouse-mode button dropped
       (the trackpad is reached from the left mixed list / app launch now). */
    control_center_app_list = NULL;

    /* Brightness slider (top) with a sun glyph on its left edge. */
    lv_obj_t *bar = lv_bar_create(control_center_window);
    lv_bar_set_range(bar, 0, 100);
    lv_obj_set_width(bar, LV_PCT(70));
    lv_obj_set_height(bar, 80);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_color(bar, APP_MAIN_COLOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(bar, APP_MAIN_COLOR, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_90, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(bar, LV_OPA_10, LV_PART_MAIN);
    lv_bar_set_value(bar, SkaiWatchSys.brightness, LV_ANIM_ON);
    lv_obj_add_event_cb(bar, bar_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_t *sun_icon = lv_img_create(bar);
    lv_img_set_src(sun_icon, &sun);
    lv_obj_align(sun_icon, LV_ALIGN_LEFT_MID, 20, 0);
    brightness_bar = bar;

    /* Tool-button grid below the slider. */
    lv_obj_t *cc_bottom = lv_obj_create(control_center_window);
    lv_obj_set_size(cc_bottom, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_opa(cc_bottom, LV_OPA_0, 0);
    lv_obj_set_style_border_width(cc_bottom, 0, 0);
    lv_obj_clear_flag(cc_bottom, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align_to(cc_bottom, bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /* Row 1: calculator / flashlight / recorder */
    /* calculator_icon = the dedicated control-center calculator glyph the user
       supplied (2026-06-25). CALCULATOR_ICON resolves to &calculator_icon; the
       ezip resource now exists so the symbol links on watch + PC. */
    lv_obj_t *calculator_btn = common_image_button(
        cc_bottom, CALCULATOR_ICON, 100, 100, calculator_btn_event_cb);
    lv_obj_set_style_bg_opa(calculator_btn, LV_OPA_10, 0);
    lv_obj_align(calculator_btn, LV_ALIGN_TOP_LEFT, 70, 0);

    lv_obj_t *flishlight_btn = common_image_button(
        cc_bottom, FLISHLIGHT_ICON, 100, 100, flishlight_icon_event_cb);
    lv_obj_align(flishlight_btn, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_t *recorder_btn = common_image_button(
        cc_bottom, &micro_icon, 100, 100, recorder_btn_event_cb);
    lv_obj_align(recorder_btn, LV_ALIGN_TOP_RIGHT, -70, 0);

    /* Row 2: setting / qrcode / do-not-disturb */
    lv_obj_t *setting_icon = common_image_button(
        cc_bottom, IMG_SETTINGS, 100, 100, setting_icon_event_cb);
    lv_obj_align_to(setting_icon, calculator_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);
    lv_obj_set_style_bg_opa(setting_icon, LV_OPA_10, 0);
    lv_obj_set_style_bg_color(setting_icon, lv_color_hex(0xFFFFFF), 0);

    lv_obj_t *qrcode_btn = common_image_button(
        cc_bottom, &icon_qrcode, 100, 100, qrcode_btn_event_cb);
    lv_obj_align_to(qrcode_btn, flishlight_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);

    dndmode_enabled = SkaiWatchSys.DNDMode.config.status;
    dnd_mode_btn = common_image_button(cc_bottom, &icon_dnd_mode, 100, 100,
                                       dnd_mode_btn_event_cb);
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

    /* Row 3: find-phone (+ gesture-test in debug builds) */
    lv_obj_t *find_phone_btn = common_image_button(
        cc_bottom, FIND_PHONE, 100, 100, find_phone_btn_event_cb);
    lv_obj_align_to(find_phone_btn, setting_icon, LV_ALIGN_OUT_BOTTOM_MID, 0, 35);
#if !kReleaseMode
    lv_obj_t *gesture_test_btn = common_image_button(
        cc_bottom, IMG_LOGO, 100, 100, gesture_test_btn_event_cb);
    lv_obj_align_to(gesture_test_btn, qrcode_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 35);
#endif

    return control_center_window;
}

static void scroll_control_center_to_top(void)
{
    if (control_center_app_list && lv_obj_is_valid(control_center_app_list))
    {
        lv_obj_scroll_to_y(control_center_app_list, 0, LV_ANIM_OFF);
    }
    if (lv_obj_is_valid(control_center_window))
    {
        lv_obj_scroll_to_y(control_center_window, 0, LV_ANIM_OFF);
    }
}

void control_center_on_resume(void)
{
}

void control_center_on_pause(void)
{
    scroll_control_center_to_top();
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

static lv_obj_t *pages[5];
static bool test_mode = false;
void open_test_mode(bool open)
{
    watch_sys_sync.set_debug_mode(open);
}

bool is_test_mode(void)
{
    return test_mode;
}

static void set_clock_main_status_opa(uint8_t opa, bool mask)
{
    (void)mask;
    if (lv_obj_is_valid(gaus_dial_img))
    {
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
    // i<4 (was i<3): 加右緣 zone，鏡像左緣 → 右緣拖可拉出 device_pager 右 tile。
    for (i = 0; i < 4; i++)
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
        else if (STATUS_BAR_AREA_RIGHT == i)
        {
            /* 鏡像左緣 zone：右緣 58px 把手。press 顯示 tileview(home)，接著往左
               拖 → 原生 finger-follow 滑到右 tile = device_pager。 */
            lv_obj_set_size(status_bar_area, (LV_HOR_RES_MAX >> 3),
                            LV_VER_RES_MAX);
            lv_obj_set_user_data(status_bar_area, (void *)STATUS_BAR_AREA_RIGHT);
            lv_obj_add_event_cb(status_bar_area, notification_status_bar_cb,
                                LV_EVENT_ALL, NULL);
            lv_obj_align(status_bar_area, LV_ALIGN_RIGHT_MID, 0, 0);
            status_bar_area_right = status_bar_area;
        }
    }

    app_clock_main_status_bar = lv_tileview_create(status_bar_bg_main);
    lv_obj_set_scrollbar_mode(app_clock_main_status_bar, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(app_clock_main_status_bar, LV_COLOR_BLACK,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(app_clock_main_status_bar, LV_OPA_TRANSP,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    // 0: down, 1: home, 2: up, 3: left, 4: right
    // i<5 (was i<4): 也建右 tile (2,1) 給 device_pager（鏡像左側 instruction_list）。
    for (i = 0; i < 5; i++)
    {
        if (i == MAIN_PAGE_TYPE_HOME)
        {
            /* L/R swap: HOME scrolls TOP (messages) and LEFT (device_pager /
               trackpad, now the (0,1) tile) — the trackpad enters from the LEFT
               edge. It no longer scrolls RIGHT: the old right tile is now the
               (unreachable) instruction_list placeholder at (2,1); the Action
               list floats in from the RIGHT edge via the reveal overlay, not a
               tile. The floating list lives on lv_layer_top; no live set_tile_id
               targets the placeholder (animate_to_instruction_list /
               animate_to_notification_center are dead).
               2026-06-04: DOWN (control center / app grid) stays removed — the
               built-in apps live in the floating instruction list. LV_DIR_BOTTOM
               dropped; the DOWN tile (1,2) is built but UNREACHABLE and empty. */
            /* P3: LV_DIR_LEFT removed — the device_pager / trackpad no longer
               swipes in from the left (a left-edge drag pulled it out and fought
               the @-list reveal). It is now launched as a list app (app_id_mouse
               in DEFAULT_APP_ITEMS); the left edge is the @-list reveal, the right
               edge the /-list reveal. HOME still scrolls TOP (messages). */
            /* 2026-06-25: re-enabled LV_DIR_BOTTOM so the quick-settings page
               (control center, DOWN tile at (1,2)) is reachable again by
               pulling up from the bottom edge, and LV_DIR_RIGHT so the App List
               tile (col 2 = MAIN_PAGE_TYPE_LEFT) swipes in from the right edge.
               HOME still scrolls TOP to the message list. The left edge stays the
               mixed-list reveal overlay (no LV_DIR_LEFT). */
            pages[i] = lv_tileview_add_tile(app_clock_main_status_bar, 1, i,
                                            LV_DIR_TOP | LV_DIR_BOTTOM |
                                                LV_DIR_RIGHT);
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
                /* L/R swap: the (unreachable) instruction_list placeholder moves
                   to the RIGHT (2,1). The Action list reveals from the right edge
                   as a floating overlay now, not via this tile. */
                pages[i] = lv_tileview_add_tile(app_clock_main_status_bar, 2, 1,
                                                LV_DIR_LEFT);
            }
            else if (i == MAIN_PAGE_TYPE_UP)
            {
                pages[i] = lv_tileview_add_tile(app_clock_main_status_bar, 1, 0,
                                                LV_DIR_VER);
            }
            else if (i == MAIN_PAGE_TYPE_RIGHT)
            {
                /* L/R swap: 左 tile (0,1) = device_pager / 觸控板。
                   LV_DIR_RIGHT = 從這裡往右滑回 home。home 開了 LV_DIR_LEFT，
                   所以從錶面往右滑(內容往右移、露出左 tile)就能把觸控板從左側
                   拉進來(原生 finger-follow)。入口從右改左，與右緣的 Action
                   列表 reveal 浮層左右對調。注意:此頁的邏輯頁號仍是
                   MAIN_PAGE_TYPE_RIGHT(active_pos 用 tile 加入順序，非欄位)。 */
                pages[i] = lv_tileview_add_tile(app_clock_main_status_bar, 0, 1,
                                                LV_DIR_RIGHT);
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
        }
    }
    lv_obj_set_tile_id(app_clock_main_status_bar, 1, 1, false);
    lv_obj_add_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
    control_center_layout_create(pages[CONTROL_CENTER_PAGE_INDEX]);

    extern lv_obj_t *lv_instruction_list_layout_create(lv_obj_t * parent);
    LOG_I("clock_status_bar: before instruction_list_layout_create");
    lv_instruction_list_layout_create(pages[INSTRUCTION_LIST_PAGE_INDEX]);
    LOG_I("clock_status_bar: after instruction_list_layout_create");

    /* 2026-06-25: the right-swipe tile (col 2 = INSTRUCTION_LIST_PAGE_INDEX) now
       hosts the App List grid. instruction_list_layout_create above floats its
       content on lv_layer_top and only tracks this tile as instruction_list_page
       (read-only, for scroll-snap), so the tile itself is free to render the app
       grid that swipes in from the right edge. The left mixed list keeps using
       the left-edge reveal overlay. */
    extern lv_obj_t *lv_app_list_layout_create(lv_obj_t * parent);
    lv_app_list_layout_create(pages[INSTRUCTION_LIST_PAGE_INDEX]);
    extern lv_obj_t *lv_message_list_layout_create(lv_obj_t * parent);
    LOG_I("clock_status_bar: before message_list_layout_create");
    lv_message_list_layout_create(pages[MESSAGE_PAGE_INDEX]);
    LOG_I("clock_status_bar: after message_list_layout_create");

    /* T4: device_pager 內容放右 tile (2,1)，鏡像左側 instruction_list。
       拉出靠原生 tileview 滑動。 */
    extern lv_obj_t *device_pager_create(lv_obj_t * parent);
    device_pager_create(pages[MAIN_PAGE_TYPE_RIGHT]);

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

/* Public wrapper for cross-module callers — currently ui_handler 的語言切換廣播。
   dev_change_refresh_device_list 保持 static 不動其 linkage，此為唯一對外入口。 */
void app_clock_status_bar_refresh_device_change_bar(void)
{
    dev_change_refresh_device_list();
}

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
    if (dev_change_list_ui.connecting_label &&
        lv_obj_is_valid(dev_change_list_ui.connecting_label))
    {
        char cbuf[32];
        lv_snprintf(cbuf, sizeof(cbuf), "%s", LV_EXT_STR_GET_BY_KEY(connecting, "Connecting"));
        size_t cl = strlen(cbuf);
        int dots = dev_change_connecting_dots; /* 1, 2, or 3 */
        for (int k = 0; k < dots && cl + 1 < sizeof(cbuf); k++) cbuf[cl++] = '.';
        cbuf[cl] = '\0';
        lv_label_set_text(dev_change_list_ui.connecting_label, cbuf);
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
        if (strcmp(btn_txt, LV_EXT_STR_GET_BY_KEY(yes, "Yes")) == 0)
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
    lv_snprintf(msg_buf, sizeof(msg_buf), LV_EXT_STR_GET_BY_KEY(delete_device_fmt, "Delete device?\n%s"),
                dev->device_name);

    static const char *btns[] = {NULL, NULL, ""};
    btns[0] = LV_EXT_STR_GET_BY_KEY(yes, "Yes");
    btns[1] = LV_EXT_STR_GET_BY_KEY(no, "No");

    dev_change_delete_confirm_msgbox =
        lv_msgbox_create(NULL, LV_EXT_STR_GET_BY_KEY(confirm_delete_title, "Confirm Delete"), msg_buf, btns, false);
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

extern void ble_app_advertising_start(bool mouse_mode, bool pairing_mode);

static void dev_change_add_device_btn_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_SHORT_CLICKED)
    {
        LOG_I("Device change bar: Add device (start advertising)");
        ble_app_advertising_start(true, false);
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
        lv_label_set_text(watch_label, LV_EXT_STR_GET_BY_KEY(watch, "Watch"));
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
        lv_label_set_text(conn_label, LV_EXT_STR_GET_BY_KEY(connecting, "Connecting"));
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
        lv_label_set_text(add_label, LV_EXT_STR_GET_BY_KEY(add_device, "Add Device"));
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
    /* T4: 右緣入口改用錶面 tileview 右 tile（device_pager）。停用這個 lv_layer_top
       上的舊右緣觸碰區，否則它會擋掉 tileview 的右滑。永久隱藏，沒有 un-hide 路徑。 */
    lv_obj_add_flag(status_bar_area, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_opa(status_bar_area, bar_opa,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(status_bar_area, device_change_bar_cb, LV_EVENT_ALL,
                        NULL);
    lv_obj_align(status_bar_area, LV_ALIGN_RIGHT_MID, 0, 0);

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
        lv_label_set_text(empty_label, LV_EXT_STR_GET_BY_KEY(no_paired_devices, "No paired devices"));
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

/* Return to the watch face from the device_pager (left tile (0,1)). The pager's
   horizontal carousel owns left/right swipes between devices, so it can't also
   chain a swipe back to home; instead the pager calls this when the user drags
   left past the last device (the inverse of the left-edge pull-in). Mirrors
   the reset done at init: snap the main tileview to home and hide the overlay. */
void app_clock_status_bar_return_home(void)
{
    if (!lv_obj_is_valid(app_clock_main_status_bar))
        return;
    /* Animate the slide back to the watch face so the device page — and the mouse
       base it hosts in the left tile — rides out with it (instead of jumping). The
       tileview's VALUE_CHANGED on the home tile tears the device page down
       (device_pager_set_active(false)) and hides this bar; gaus_dial_bg fades out
       via the scroll handler as scroll_x returns to centre. */
    lv_obj_set_tile_id(app_clock_main_status_bar, 1, 1, true);
}

/* Finger-follow return to the watch face for the hosted trackpad (hid_mouse's
   right-arc left-drag). The device page is the LEFT tile (scroll_x 0); home is one
   content-width to its grid-right. dx = finger delta from the press (leftward
   negative) → scroll the tileview proportionally so the watch face slides in under
   the finger. Returns progress 0..100 so the caller can threshold on release.
   Driving scroll directly (vs return_home's animate-on-threshold) keeps it under
   the finger AND defers the device-page teardown to release — no teardown while the
   finger is still down. */
int app_clock_status_bar_pull_home(int16_t dx)
{
    if (!lv_obj_is_valid(app_clock_main_status_bar))
        return 0;
    lv_coord_t home_x = lv_obj_get_content_width(app_clock_main_status_bar);
    if (home_x <= 0)
        return 0;
    lv_coord_t sx = (lv_coord_t)(-dx);
    if (sx < 0) sx = 0;
    if (sx > home_x) sx = home_x;
    lv_obj_scroll_to_x(app_clock_main_status_bar, sx, LV_ANIM_OFF);
    return (int)((int32_t)sx * 100 / home_x);
}

/* Settle the finger-follow on release: commit slides the rest of the way to home
   (the home-tile VALUE_CHANGED then tears the device page down); else snap back to
   the device tile (0,1). */
void app_clock_status_bar_pull_home_release(bool commit)
{
    if (!lv_obj_is_valid(app_clock_main_status_bar))
        return;
    lv_obj_set_tile_id(app_clock_main_status_bar, commit ? 1 : 0, 1, true);
}

// 程式化開啟 device-change 選單（不靠右側 hit-test）
// hid_mouse 用：當右弧區的左滑被 touch_bg 接走時，這裡仍能叫出選單
void app_clock_device_change_bar_open(void)
{
    if (!lv_obj_is_valid(app_clock_device_change_bar)) return;
    lv_obj_set_tile_id(app_clock_device_change_bar, 0, 0, false);
    lv_obj_clear_flag(app_clock_device_change_bar, LV_OBJ_FLAG_HIDDEN);
    if (lv_obj_is_valid(dev_change_gaus_bg))
        lv_obj_clear_flag(dev_change_gaus_bg, LV_OBJ_FLAG_HIDDEN);
    dev_change_refresh_device_list();
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