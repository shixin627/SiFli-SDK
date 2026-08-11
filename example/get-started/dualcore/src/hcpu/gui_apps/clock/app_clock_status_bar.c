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
#include "lv_top_panel.h"

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
LV_IMG_DECLARE(device_btn);
LV_IMG_DECLARE(img_left_arrow);
LV_IMG_DECLARE(img_right_arrow);

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

/* ADR-0020 右側「一台設備一欄」的**媒體中心**（founder 2026-08-11 拍板，取代
   之前的 per-device session 列表）:
     欄 2       = 手機自己的媒體中心（控制目標 = 手機,active device 清空）
     欄 2+1+i   = 第 i 台桌面設備的媒體中心（registry 順序）
   每一欄**正下方** (col,2) 是「滑鼠入口停車位」:從媒體頁往上拉、settle 在那裡
   = 進入該台的滑鼠模式（lv_top_panel_mouse_enter,控制目標在 settle 時已由
   media_col_bind 選好）。founder 2026-08-11 R2:「我是要從下往上拉出滑鼠頁面」
   —— 第一版放在上方,改到下方。退出 = 滑鼠頁頂部下拉
   (clock_main_mouse_pulldown_reveal),落回同一台的媒體頁。
   合併後的 session 列表搬到左側 (0,1),見 lv_session_pager.c。 */
#define MEDIA_COL_FIRST 2 /* 錶盤是欄 1;右邊從欄 2 開始 */
static lv_obj_t *s_media_tile[MAX_SYNCED_DEVICES];      /* (2+c,1) 媒體欄 */
static lv_obj_t *s_media_page[MAX_SYNCED_DEVICES];      /* hid_mouse_media_page_create */
static lv_obj_t *s_mouse_park_tile[MAX_SYNCED_DEVICES]; /* (2+c,2) 滑鼠入口 */
/* 每欄頂部的設備列(founder R2:「像之前那樣上面顯示設備名稱跟左右箭頭」——
   復刻舊頂部面板那組:圓點 + 名稱 + 兩顆箭頭)。每欄自帶一份,建在 tile 裡,
   內容由 media_col_headers_refresh 依 registry / 連線狀態刷新。 */
static lv_obj_t *s_media_dot[MAX_SYNCED_DEVICES];
static lv_obj_t *s_media_name[MAX_SYNCED_DEVICES];
static lv_obj_t *s_media_arr_l[MAX_SYNCED_DEVICES];
static lv_obj_t *s_media_arr_r[MAX_SYNCED_DEVICES];
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
lv_obj_t *control_center_layout_create(lv_obj_t *parent);
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
        /* Guard against a re-PRESSED mid-drag: PRESS_LOCK is cleared on these edge
           handles so the touch hands off to the tileview once the finger crosses
           into it (see the creation loop below) — but on a LEFT/RIGHT pull, undoing
           the pull brings the finger back across this same full-height/full-width
           edge strip, which re-fires PRESSED here and used to unconditionally
           snap_tile_id back to HOME + re-show, discarding the live drag and making
           the reveal impossible to cancel by reversing mid-gesture (only forward
           progress ever stuck). Skip the reset once the tileview is already
           visible — that means this is a hand-back mid-drag, not a fresh press. */
        if (lv_obj_has_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN))
        {
            LOG_I("notification_status_bar_cb from area: %d", area_id);
            if (area_id == STATUS_BAR_AREA_LEFT)
            {
                /* 左緣 = 合併 session + actions 頁 (0,1)。滑入前先捲回頂端,
                   重進永遠從最新的列開始。 */
                extern void session_list_reset_scroll(void);
                session_list_reset_scroll();
            }
            /* 右緣 = 媒體欄 (2,1),不需要進場準備 —— 內容常駐。 */
            lv_obj_set_tile_id(app_clock_main_status_bar, 1, 1, false);
            lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
            set_clock_main_status_opa(0, false);
        }
    }
}

/* ---- 右側媒體欄 finger-follow（錶面全螢幕 catcher 的向左滑 route）---------- *
 * 舊名 applist_follow(當時欄 2 是 App List)。往左滑把欄 2(手機媒體中心)拉進來,
 * catcher 擁有 press,所以 scroll 由我們自己餵(唯一 writer → 不抖),放開用
 * set_tile_id commit / abort;settle 再收尾。 */
void clock_main_applist_follow_begin(void)
{
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    lv_obj_set_tile_id(app_clock_main_status_bar, 1, 1, false); /* snap HOME, no anim */
    lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
    if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
        lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
    set_clock_main_status_opa(0, false);
}

/* ---- 左頁（合併 session + actions）finger-follow（向右滑 route）------------ *
 * ADR-0020:左頁從 reveal 浮層改成實體 tile (0,1),向右滑改走原生語意的跟手:
 * HOME 在 scroll_x==466,左頁在 0,dx>0(向右拖)把 scroll_x 往 0 壓。 */
void clock_main_leftpage_follow_begin(void)
{
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    extern void session_list_reset_scroll(void);
    session_list_reset_scroll();
    lv_obj_set_tile_id(app_clock_main_status_bar, 1, 1, false);
    lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
    if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
        lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
    set_clock_main_status_opa(0, false);
}

void clock_main_leftpage_follow_update(lv_coord_t dx)
{
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    lv_coord_t sx = LV_HOR_RES - dx; /* dx>0 (向右) -> sx 從 466 往 0 */
    if (sx < 0) sx = 0;
    if (sx > LV_HOR_RES) sx = LV_HOR_RES;
    lv_obj_scroll_to_x(app_clock_main_status_bar, sx, LV_ANIM_OFF);
}

void clock_main_leftpage_follow_end(lv_coord_t dx, lv_coord_t vx)
{
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    bool open_it = (dx >= LV_HOR_RES / 4) || (vx > 6);
    lv_obj_set_tile_id(app_clock_main_status_bar, open_it ? 0 : 1, 1, true);
}

/* ---- 媒體欄數 / 方向旗標 ---------------------------------------------------- *
 * 欄數 = 1(手機) + 桌面設備數,夾在 [1, MAX_SYNCED_DEVICES]。格子開機建滿,能不能
 * 滑到由 tile->dir 決定;設備數變動時重算並**當場**重設 scroll_dir(lv_tileview 在
 * 發 VALUE_CHANGED 前就複製了 dir,慢一拍等於這次 settle 無效 —— 2026-08-11 踩過)。 */
extern int hid_mouse_device_count(void);
extern int hid_mouse_active_device_index(void);

static int media_col_count(void)
{
    int n = 1 + hid_mouse_device_count(); /* 手機恆佔第一欄 */
    if (n < 1) n = 1;
    if (n > MAX_SYNCED_DEVICES) n = MAX_SYNCED_DEVICES;
    return n;
}

static void media_cols_apply_dirs(int col_count)
{
    for (int c = 0; c < MAX_SYNCED_DEVICES; c++)
    {
        /* LEFT = 回錶盤方向;BOTTOM = 往上拉出下方的滑鼠入口(founder R2)。 */
        lv_dir_t d = LV_DIR_LEFT | LV_DIR_BOTTOM;
        if (c + 1 < col_count)
            d |= LV_DIR_RIGHT;
        if (s_media_tile[c] && lv_obj_is_valid(s_media_tile[c]))
            ((lv_tileview_tile_t *)s_media_tile[c])->dir = d;
        /* 停車位只准往上收回媒體頁(settle 在這裡本來就立刻切進滑鼠圖層)。 */
        if (s_mouse_park_tile[c] && lv_obj_is_valid(s_mouse_park_tile[c]))
            ((lv_tileview_tile_t *)s_mouse_park_tile[c])->dir = LV_DIR_TOP;
    }
}

/* ── 媒體欄頂部設備列(復刻舊頂部面板那組,座標同 DEV_DOT_Y/DEV_LABEL_Y/
   DEV_ARROW_DX/DEV_ARROW_Y = 26/40/117/42)── */
extern const char *hid_mouse_device_name(int idx);
extern bool hid_mouse_device_online(int idx);

static void media_col_headers_refresh(void)
{
    int count = media_col_count();
    for (int c = 0; c < MAX_SYNCED_DEVICES; c++)
    {
        if (s_media_name[c] == NULL || !lv_obj_is_valid(s_media_name[c]))
            continue;
        const char *name;
        bool online;
        if (c == 0)
        {
            name = LV_EXT_STR_GET_BY_KEY(connected_phone, "Phone");
            online = get_bluetooth_connection_status();
        }
        else
        {
            name = hid_mouse_device_name(c - 1);
            online = hid_mouse_device_online(c - 1);
        }
        lv_label_set_text(s_media_name[c], (name && name[0]) ? name : "");
        if (s_media_dot[c] && lv_obj_is_valid(s_media_dot[c]))
            lv_obj_set_style_bg_color(s_media_dot[c],
                                      online ? lv_color_hex(0x4CAF50)
                                             : lv_color_hex(0xFF3B30),
                                      0);
        /* 不循環(沿用舊面板規則):到底那一側的箭頭消失。左端(欄 2 = 手機)
           左箭頭也收掉 —— 回錶盤走原生左滑,不用箭頭。 */
        if (s_media_arr_l[c] && lv_obj_is_valid(s_media_arr_l[c]))
        {
            if (c > 0) lv_obj_clear_flag(s_media_arr_l[c], LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(s_media_arr_l[c], LV_OBJ_FLAG_HIDDEN);
        }
        if (s_media_arr_r[c] && lv_obj_is_valid(s_media_arr_r[c]))
        {
            if (c + 1 < count) lv_obj_clear_flag(s_media_arr_r[c], LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(s_media_arr_r[c], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* 箭頭 = 走一格到隔壁欄(原生動畫換頁,settle 自己收尾)。user_data = 目標欄。 */
static void media_arrow_cb(lv_event_t *e)
{
    int target_col = (int)(intptr_t)lv_event_get_user_data(e);
    int count = media_col_count();
    if (target_col < MEDIA_COL_FIRST || target_col >= MEDIA_COL_FIRST + count)
        return;
    if (app_clock_main_status_bar && lv_obj_is_valid(app_clock_main_status_bar))
        lv_obj_set_tile_id(app_clock_main_status_bar, (uint8_t)target_col, 1, true);
}

static void media_col_header_build(int c)
{
    lv_obj_t *tile = s_media_tile[c];
    if (tile == NULL || !lv_obj_is_valid(tile))
        return;

    s_media_dot[c] = lv_obj_create(tile);
    lv_obj_remove_style_all(s_media_dot[c]);
    lv_obj_set_size(s_media_dot[c], 10, 10);
    lv_obj_align(s_media_dot[c], LV_ALIGN_TOP_MID, 0, 26);
    lv_obj_set_style_radius(s_media_dot[c], LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_media_dot[c], LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_media_dot[c], lv_color_hex(0x4CAF50), 0);
    lv_obj_clear_flag(s_media_dot[c], LV_OBJ_FLAG_CLICKABLE);

    s_media_name[c] = lv_label_create(tile);
    lv_obj_set_width(s_media_name[c], 200);
    lv_label_set_long_mode(s_media_name[c], LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(s_media_name[c], LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_media_name[c], lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_opa(s_media_name[c], LV_OPA_80, 0);
    lv_label_set_text(s_media_name[c], "");
    lv_obj_align(s_media_name[c], LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_clear_flag(s_media_name[c], LV_OBJ_FLAG_CLICKABLE);

    s_media_arr_l[c] = lv_img_create(tile);
    lv_img_set_src(s_media_arr_l[c], &img_left_arrow);
    lv_obj_align(s_media_arr_l[c], LV_ALIGN_TOP_MID, -117, 42);
    lv_obj_add_flag(s_media_arr_l[c], LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(s_media_arr_l[c], 24);
    lv_obj_add_event_cb(s_media_arr_l[c], media_arrow_cb, LV_EVENT_CLICKED,
                        (void *)(intptr_t)(MEDIA_COL_FIRST + c - 1));
    lv_obj_add_flag(s_media_arr_l[c], LV_OBJ_FLAG_HIDDEN);

    s_media_arr_r[c] = lv_img_create(tile);
    lv_img_set_src(s_media_arr_r[c], &img_right_arrow);
    lv_obj_align(s_media_arr_r[c], LV_ALIGN_TOP_MID, 117, 42);
    lv_obj_add_flag(s_media_arr_r[c], LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(s_media_arr_r[c], 24);
    lv_obj_add_event_cb(s_media_arr_r[c], media_arrow_cb, LV_EVENT_CLICKED,
                        (void *)(intptr_t)(MEDIA_COL_FIRST + c + 1));
    lv_obj_add_flag(s_media_arr_r[c], LV_OBJ_FLAG_HIDDEN);
}

/** 設備清單有變時呼叫:右側可滑的媒體欄數跟著設備數走。 */
void clock_main_media_cols_refresh(void)
{
    int col_count = media_col_count();
    static int s_last_logged = -1;
    if (col_count != s_last_logged)
    {
        s_last_logged = col_count;
        LOG_W("[media-col] reachable columns = %d (devices=%d)", col_count,
              hid_mouse_device_count());
    }
    media_cols_apply_dirs(col_count);
    media_col_headers_refresh();
    if (app_clock_main_status_bar && lv_obj_is_valid(app_clock_main_status_bar))
    {
        lv_obj_t *act = lv_tileview_get_tile_act(app_clock_main_status_bar);
        if (act != NULL && lv_obj_is_valid(act))
            lv_obj_set_scroll_dir(app_clock_main_status_bar,
                                  ((lv_tileview_tile_t *)act)->dir);
    }
}

/* 滑鼠模式頂部下拉(founder 2026-08-11 R2/R4:「抓著上面往下把滑鼠頁面拉掉,顯示
   下面的媒體頁面」)。
   R3 的做法(async 退出+吞 press)在真機仍會拉出通知面板:destroy 讓 indev reset,
   把 wait_release 也清掉,殘餘拖曳照樣被 tileview 用舊的四向 scroll_dir 接走。
   改用**跟舊面板 reveal 完全同一套交棒機制**(該機制在真機驗證過):下拉判定當下
   **不拆滑鼠**,只把主 tileview 亮在該欄的滑鼠停車格 (col,2) —— hid_mouse 的頂
   部帶沒有 PRESS_LOCK,press 下一 tick 自然轉給 tileview,往下拖就是停車格→媒體
   頁的**原生跟手**。settle 落在媒體頁(row 1)才真正退出滑鼠模式;落回停車格 =
   取消,tileview 收回、滑鼠模式原樣繼續。 */
static bool s_mouse_pull_reveal = false; /* reveal 的 snap settle 與取消 settle 區分用 */

void clock_main_mouse_pulldown_reveal(void)
{
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    int dev = hid_mouse_active_device_index();
    int col = MEDIA_COL_FIRST + ((dev >= 0) ? dev + 1 : 0);
    if (col >= MEDIA_COL_FIRST + media_col_count())
        col = MEDIA_COL_FIRST;
    s_mouse_pull_reveal = true; /* set_tile_id 會同步發 settle,先立旗 */
    lv_obj_set_tile_id(app_clock_main_status_bar, (uint8_t)col, 2, false);
    lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
    clock_main_media_cols_refresh(); /* 當場套上停車格的 LV_DIR_TOP */
    LOG_W("[media-col] mouse pulldown reveal at col %d", col);
}

/* 滑鼠頁底部 skaibar tap(founder 2026-08-11 R6):開「目前控制那台」的新 session,
   UI 同左頁 —— 退出滑鼠、切到左頁 (0,1)、對那台發 conv_new;清單推回來後
   session pager 自動 walk-in 進聊天室。從 hid_mouse 的事件鏈進來,退出會拆它
   自己 → async 到下一輪再做。 */
static void open_device_session_async_cb(void *unused)
{
    (void)unused;
    extern void lv_top_panel_mouse_exit(void);
    extern void session_list_open_new_for_device(const char *device_id);
    extern const char *hid_mouse_device_id(int idx);
    /* 先記下目標再退出(registry 是全域資料,退出不清 active,但保險起見先取)。 */
    int dev = hid_mouse_active_device_index();
    const char *dev_id = (dev >= 0) ? hid_mouse_device_id(dev) : NULL;
    session_list_open_new_for_device(dev_id); /* NULL = 預設目標(active/第0台) */
    lv_top_panel_mouse_exit();
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    extern void session_list_reset_scroll(void);
    session_list_reset_scroll();
    lv_obj_set_tile_id(app_clock_main_status_bar, 0, 1, false); /* 左頁 */
    lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
    if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
        lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
    LOG_W("[media-col] skaibar tap -> new session on %s",
          dev_id ? dev_id : "(default)");
}

void clock_main_open_device_session(void)
{
    lv_async_call(open_device_session_async_cb, NULL);
}

/* 目前捲動位置換算成欄 / 列（tile 加入順序的 active_pos 在補格之後沒有可讀性）。 */
static int media_scroll_col(lv_obj_t *tv)
{
    return (int)((lv_obj_get_scroll_x(tv) + LV_HOR_RES / 2) / LV_HOR_RES);
}
static int media_scroll_row(lv_obj_t *tv)
{
    return (int)((lv_obj_get_scroll_y(tv) + LV_VER_RES / 2) / LV_VER_RES);
}

/* settle 在欄 col:曲名路由綁到那一欄的媒體頁,控制目標換成那一台。
   欄 2 = 手機:清掉 active device(控制目標回到手機,媒體資訊走手機自己那份)。 */
static void media_col_bind(int col)
{
    int c = col - MEDIA_COL_FIRST;
    if (c < 0 || c >= MAX_SYNCED_DEVICES)
        return;
    if (s_media_page[c] == NULL || !lv_obj_is_valid(s_media_page[c]))
        return;
    extern void hid_mouse_media_page_bind(lv_obj_t *page);
    extern void hid_mouse_media_page_reset_title(lv_obj_t *page);
    extern void hid_mouse_clear_active_device(void);
    extern void hid_mouse_set_active_device_index(int idx);
    hid_mouse_media_page_bind(s_media_page[c]);
    if (c == 0)
    {
        if (hid_mouse_active_device_index() >= 0)
        {
            hid_mouse_media_page_reset_title(s_media_page[c]);
            hid_mouse_clear_active_device();
        }
    }
    else if (hid_mouse_active_device_index() != c - 1)
    {
        hid_mouse_media_page_reset_title(s_media_page[c]);
        hid_mouse_set_active_device_index(c - 1);
    }
}

/* ---- Notification / control+applist finger-follow --------------------------- *
   垂直軸雙向:往下拉(dy>0)開頂部面板(row 0);往上拉(dy<0)開下方的
   控制中心+App List 頁(row 2, ADR-0020 恢復)。 */
void clock_main_notify_follow_begin(void)
{
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    lv_obj_set_tile_id(app_clock_main_status_bar, 1, 1, false); /* snap HOME, no anim */
    lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
    if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
        lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
    set_clock_main_status_opa(0, false);
}

void clock_main_notify_follow_update(lv_coord_t dy)
{
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    lv_coord_t sy = LV_VER_RES - dy; /* dy>0 (下拉) -> sy<466 往 0 (通知) */
    if (sy < 0) sy = 0;
    if (sy > 2 * LV_VER_RES) sy = 2 * LV_VER_RES; /* dy<0 (上拉) -> 往 932 (控制頁) */
    lv_obj_scroll_to_y(app_clock_main_status_bar, sy, LV_ANIM_OFF);
}

void clock_main_notify_follow_end(lv_coord_t dy, lv_coord_t vy)
{
    (void)dy;
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    /* Commit by the LIVE scroll position, not the raw cumulative dy — this axis has
       two opposite targets, and a big reversal could cross the OPPOSITE side's raw
       threshold (the exact bug the 2026-07 note below the old code described). */
    lv_coord_t sy = lv_obj_get_scroll_y(app_clock_main_status_bar);
    uint8_t row;
    if ((sy <= LV_VER_RES - LV_VER_RES / 4) || (vy > 6))
        row = 0; /* 頂部面板(通知) */
    else if ((sy >= LV_VER_RES + LV_VER_RES / 4) || (vy < -6))
        row = 2; /* 控制中心 + App List */
    else
        row = 1; /* 回錶盤 */
    lv_obj_set_tile_id(app_clock_main_status_bar, 1, row, true);
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
        /* ADR-0020 之後的視覺狀態機,單一 owner:
             sx > HOME  → 右側媒體區(欄 2+):模糊底圖跟手漸入,時間/電量不顯示
             sx < HOME  → 左頁(合併 session + actions):同樣的模糊漸入
             sx == HOME → 垂直軸(上=通知面板 / 下=控制+App List)
           gaus_dial_bg 濃度只在這裡與 settle 寫,別的路徑不要碰(2026-08-11 之前
           有 7 條路徑共寫,靠執行順序決勝負,下拉看不到模糊的 bug 就活在那裡)。 */
        lv_coord_t sx = lv_obj_get_scroll_x(obj);
        lv_coord_t sy = lv_obj_get_scroll_y(obj);

        if (sx > LV_HOR_RES)
        {
            /* 右側媒體區。founder 2026-08-11:右側整區不顯示頂部時間。 */
            set_instruction_list_time_opa(LV_OPA_TRANSP);
            set_instruction_list_battery_opa(LV_OPA_TRANSP);
            lv_coord_t pull = sx - LV_HOR_RES;
            if (pull > LV_HOR_RES) pull = LV_HOR_RES; /* 欄與欄之間保持滿值 */
            lv_coord_t opa = pull * 255 / 350;
            if (opa > 255) opa = 255;
            if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
            {
                lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
                lv_obj_set_style_bg_opa(gaus_dial_bg, (lv_opa_t)opa, 0);
            }
            set_clock_main_status_opa((uint8_t)opa, false);
            break;
        }
        if (sx < LV_HOR_RES)
        {
            /* 左頁(合併 session + actions)拉出。 */
            set_instruction_list_time_opa(LV_OPA_TRANSP);
            set_instruction_list_battery_opa(LV_OPA_TRANSP);
            lv_coord_t pull = LV_HOR_RES - sx;
            lv_coord_t opa = pull * 255 / 350;
            if (opa > 255) opa = 255;
            if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
            {
                lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
                lv_obj_set_style_bg_opa(gaus_dial_bg, (lv_opa_t)opa, 0);
            }
            set_clock_main_status_opa((uint8_t)opa, false);
            break;
        }

        /* sx == HOME:垂直軸。 */
        {
            lv_coord_t up = LV_VER_RES - sy; /* >0 = 往面板;<0 = 往控制頁 */
            /* 滑鼠模式下拉面板的黑底跟手漸黑(錶盤路徑由 gaus_dial_bg 負責,
               那層在滑鼠圖層底下看不到)。 */
            lv_coord_t upc = up;
            if (upc < 0) upc = 0;
            if (upc > LV_VER_RES) upc = LV_VER_RES;
            lv_top_panel_set_backdrop_opa((uint8_t)((int32_t)upc * 204 / 466));

            lv_coord_t mag = LV_ABS(up);
            if (mag > 0)
            {
                lv_coord_t opa = mag * 255 / 350;
                if (opa > 255) opa = 255;
                if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
                    lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
                set_clock_main_status_opa((uint8_t)opa, true);
                set_instruction_list_battery_opa(LV_OPA_TRANSP);
            }
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
        if (active_pos == 1)
        {
            /* Landing back at HOME — 含取消的 reveal。catcher 要回前景;
               gaus_dial_bg(全螢幕、lv_obj_create 預設 CLICKABLE)必須藏回去,
               否則之後所有觸控都被它吃掉(歷史 bug,見 git log)。 */
            extern void clock_main_face_swipe_catcher_foreground(void);
            clock_main_face_swipe_catcher_foreground();
            if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
                lv_obj_add_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
        }

        int col_now = media_scroll_col(obj);
        int row_now = media_scroll_row(obj);
        bool on_media_col = (row_now == 1 && col_now >= MEDIA_COL_FIRST);
        bool on_mouse_park = (row_now == 2 && col_now >= MEDIA_COL_FIRST);

        /* 護欄:媒體欄的 row 0 沒有格子(停車位在 row 2)。stale scroll_dir 之類的
           殘餘拖曳若把捲動帶到那裡,settle 會吸去別的欄 —— 直接彈回該欄媒體頁。 */
        if (row_now == 0 && col_now >= MEDIA_COL_FIRST)
        {
            lv_obj_set_tile_id(obj, (uint8_t)col_now, 1, true);
            break;
        }

        /* ADR-0020 R2:媒體欄**往上拉** settle 在下方停車位 = 進入該欄那台的滑鼠
           模式。R4:已在滑鼠模式時,停車位的 settle 是「下拉退出」流程的一部分 ——
           reveal 的 snap settle(保持顯示,等 press 交棒給 tileview 跟手)或使用者
           取消(收回 tileview,滑鼠模式原樣繼續),都不能再走進入路徑。 */
        if (on_mouse_park)
        {
            if (lv_top_panel_mouse_mode())
            {
                if (s_mouse_pull_reveal)
                {
                    /* reveal 剛 snap 進來:維持顯示,等原生跟手拖曳。 */
                    s_mouse_pull_reveal = false;
                }
                else
                {
                    /* 下拉不到門檻放開 = 取消:收回 tileview,繼續滑鼠模式。 */
                    middle_layer_tileview_index = 255;
                    lv_obj_set_tile_id(obj, 1, 1, false);
                    lv_obj_add_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
                    if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
                        lv_obj_add_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
                    LOG_W("[media-col] mouse pulldown cancelled");
                }
                check_main_page();
                break;
            }
            media_col_bind(col_now);
            lv_top_panel_mouse_enter();
            middle_layer_tileview_index = 255; /* 下次 settle 一定重跑收尾 */
            lv_obj_set_tile_id(obj, 1, 1, false);
            lv_obj_add_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
            if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
                lv_obj_add_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
            check_main_page();
            break;
        }

        /* R4:settle 在媒體頁而滑鼠模式還開著 = 下拉退出 commit —— 現在才真正拆
           滑鼠圖層(press 早已交棒給 tileview,hid_mouse 不在事件鏈上,可安全拆)。 */
        if (on_media_col && lv_top_panel_mouse_mode())
        {
            extern void lv_top_panel_mouse_exit(void);
            s_mouse_pull_reveal = false;
            lv_top_panel_mouse_exit();
            LOG_W("[media-col] mouse exit -> col %d", col_now);
        }

        if (middle_layer_tileview_index == active_pos)
        {
            check_main_page();
            return;
        }
        middle_layer_tileview_index = active_pos;

        /* 設備數會隨配對/上下線變動,每次 settle 重算可滑的媒體欄數並**當場**
           重設方向(lv_tileview 在發 VALUE_CHANGED 前就複製 dir,慢一拍等於這次
           settle 無效)。 */
        clock_main_media_cols_refresh();

        /* Global input bar (lv_layer_top):只在錶盤(HOME)顯示 —— 左頁有自己的
           麥克風,媒體欄/面板/控制頁都不要。gui_app gate:通知詳情等 app 蓋在錶
           盤上時 settle 回 HOME 不能把 bar 疊上去。 */
        {
            extern void instruction_list_bar_set_visible(bool visible);
            instruction_list_bar_set_visible(active_pos == MAIN_PAGE_TYPE_HOME &&
                                             gui_app_is_actived("Main") &&
                                             !lv_top_panel_mouse_mode());
        }

        {
            bool on_panel = (active_pos == MAIN_PAGE_TYPE_UP);
            lv_top_panel_set_open(on_panel);
            lv_top_panel_set_backdrop_opa(on_panel ? 204 : 0);
        }

        /* 媒體欄 settle:曲名路由 + 控制目標跟著欄走(欄 2 = 手機)。 */
        if (on_media_col)
            media_col_bind(col_now);

        /* 離開左頁時把它的 AI 輸入框收掉(它只在自己頁面上自我關閉)。 */
        {
            extern void instruction_list_close_ai_on_leave(void);
            if (active_pos != MAIN_PAGE_TYPE_RIGHT)
                instruction_list_close_ai_on_leave();
        }

        if (gui_app_is_actived("Main"))
        {
            main_clock_tileview_scrollable = (active_pos == 1);
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
            /* settle 收尾:模糊圖滿值、黑底退場(跟 2026-08-06 之前各頁的最終
               狀態一致);時間/電量在非錶盤頁一律不顯示(founder 2026-08-11)。 */
            if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
                lv_obj_set_style_bg_opa(gaus_dial_bg, LV_OPA_TRANSP, 0);
            set_instruction_list_time_opa(LV_OPA_0);
            set_instruction_list_battery_opa(LV_OPA_TRANSP);
            if (active_pos == MAIN_PAGE_TYPE_UP)
                set_clock_main_status_opa(LV_OPA_100, true);
            else
                set_clock_main_status_opa(LV_OPA_100, false);

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

void animate_to_home_from_notification_center(void)
{
    if (lv_obj_is_valid(myLancher[app_index_message].pagetileview))
        lv_obj_set_tile_id(myLancher[app_index_message].pagetileview, 1, 1,
                           LV_ANIM_ON);
}

/* Same landing as animate_to_home_from_notification_center, but INSTANT, and it
   also resets the AI tileview. Used by the post-sleep reset in watch_demo.c:
   that runs while the black sleep overlay is still up, so an animation would
   either be invisible or leak a slide into the first awake frame — snap instead.
   Still goes through set_tile_id (not a raw scroll_to) so the VALUE_CHANGED
   settle handler runs all the per-page teardown, same as a real swipe home. */
void snap_to_home_from_any_page(void)
{
    lv_obj_t *ai_tv = myLancher[app_index_ai_interface].pagetileview;
    lv_obj_t *main_tv = myLancher[app_index_message].pagetileview;

    /* PC sim excludes the AI/speech apps, so this tile can be NULL — see
       is_at_ai_interface(). */
    if (ai_tv && lv_obj_is_valid(ai_tv))
        lv_obj_set_tile_id(ai_tv, 0, 0, LV_ANIM_OFF);
    if (main_tv && lv_obj_is_valid(main_tv))
        lv_obj_set_tile_id(main_tv, 1, 1, LV_ANIM_OFF);
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
static lv_obj_t *brightness_fill; /* self-drawn fill pill; see cc_bar_apply_fill */

/* Coalesce PRESSING sends so the data-service queue can't overflow. */
#define CC_BAR_THROTTLE_MS 40
/* The fill never shrinks below this width, so dragging to the far left bottoms
 * out at a fixed icon-sized pill instead of vanishing. */
#define CC_BAR_FILL_MIN_W 90

/* Copied from app_setting.c (setting_bar_apply_fill). After the LVGL update,
 * lv_bar on real hardware takes the EPIC GPU indicator path whenever
 * bg_radius != 0 (lv_bar.c:484) and paints the indicator across the WHOLE track
 * regardless of value -> a stray full-width oval, i.e. "no layering". The fix:
 * make MAIN/INDICATOR transparent + MAIN radius 0 (off the GPU path) and draw
 * the fill ourselves as a single rounded child whose width tracks the value. */
static void cc_bar_apply_fill(lv_obj_t *bar, lv_obj_t *fill)
{
    if (!bar || !fill || !lv_obj_is_valid(bar) || !lv_obj_is_valid(fill))
        return;
    lv_coord_t min = lv_bar_get_min_value(bar);
    lv_coord_t max = lv_bar_get_max_value(bar);
    lv_coord_t value = lv_bar_get_value(bar);
    lv_coord_t barw = lv_obj_get_width(bar);
    lv_coord_t w = CC_BAR_FILL_MIN_W;
    if (max > min && barw > CC_BAR_FILL_MIN_W)
        w += (lv_coord_t)((int32_t)(value - min) * (barw - CC_BAR_FILL_MIN_W) / (max - min));
    if (w < CC_BAR_FILL_MIN_W)
        w = CC_BAR_FILL_MIN_W;
    if (w > barw)
        w = barw;
    lv_obj_set_width(fill, w);
}

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
        cc_bar_apply_fill(brightness_bar, brightness_fill);
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
    static int16_t last_sent = -1;
    static uint32_t last_send_tick = 0;
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
        cc_bar_apply_fill(bar, brightness_fill);
        uint16_t brightness = lv_bar_get_value(bar);
        if ((int16_t)brightness != last_sent &&
            lv_tick_elaps(last_send_tick) >= CC_BAR_THROTTLE_MS)
        {
            gui_set_brightness(brightness, true);
            last_sent = (int16_t)brightness;
            last_send_tick = lv_tick_get();
        }
    }
    else if (code == LV_EVENT_PRESSED)
    {
        last_sent = -1;
        last_send_tick = 0;
        lv_obj_clear_flag(myLancher[app_index_message].pagetileview,
                          LV_OBJ_FLAG_SCROLLABLE);
    }
    else if (code == LV_EVENT_RELEASED)
    {
        lv_obj_t *bar = lv_event_get_target(e);
        uint16_t brightness = lv_bar_get_value(bar);
        if ((int16_t)brightness != last_sent)
        {
            gui_set_brightness(brightness, true);
            last_sent = (int16_t)brightness;
        }
        lv_obj_add_flag(myLancher[app_index_message].pagetileview,
                        LV_OBJ_FLAG_SCROLLABLE);
    }
}

static lv_obj_t *control_center_window;
static lv_obj_t *control_center_app_list = NULL;
lv_obj_t *control_center_layout_create(lv_obj_t *parent)
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

    /* Brightness slider (top) with a sun glyph on its left edge.
       2026-06-30: full structure copied from the Settings display page
       (app_setting_display.c). The bar there lives inside a 0x1E1E1E dark
       card (`item`); the bar alone on the transparent control-center window
       had no dark backing, so the fill/track split was invisible. Recreate
       that card here verbatim — same container + bar + sun icon. */
    // lv_obj_t *item = lv_obj_create(control_center_window);
    // lv_obj_set_size(item, LV_PCT(90), 100);
    // lv_obj_align(item, LV_ALIGN_TOP_MID, 0, 90);
    // lv_obj_set_style_bg_color(item, lv_color_hex(0x1E1E1E), 0);
    lv_obj_t *bar = lv_bar_create(control_center_window);
    lv_bar_set_range(bar, 5, 100);
    lv_obj_set_size(bar, LV_PCT(70), 80);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 90);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xE5E5EA), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xE5E5EA), LV_PART_MAIN);
    lv_obj_set_style_radius(bar, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(bar, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_bar_set_value(bar, SkaiWatchSys.brightness, LV_ANIM_ON);
    lv_obj_add_event_cb(bar, bar_event_cb, LV_EVENT_ALL, NULL);
    /* Self-drawn fill: one rounded child, left-anchored, width tracks the
       value (see cc_bar_apply_fill). lv_bar's own indicator is unusable here
       (EPIC GPU path paints it full-width on real hw, and it collapses at the
       minimum), which is why MAIN/INDICATOR above are transparent. Created
       before the icon so the sun glyph stays on top. */
    lv_obj_t *brightness_floor = lv_obj_create(bar);
    lv_obj_set_height(brightness_floor, 80);
    lv_obj_set_style_radius(brightness_floor, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(brightness_floor, lv_color_hex(0xE5E5EA), 0);
    lv_obj_set_style_bg_opa(brightness_floor, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(brightness_floor, 0, 0);
    lv_obj_clear_flag(brightness_floor, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(brightness_floor, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_t *sun_icon = lv_img_create(bar);
    lv_img_set_src(sun_icon, &sun);
    lv_obj_align(sun_icon, LV_ALIGN_LEFT_MID, 20, 0);
    brightness_bar = bar;
    brightness_fill = brightness_floor;
    lv_obj_update_layout(bar); /* resolve width before the first fill calc */
    cc_bar_apply_fill(bar, brightness_floor);

    /* ADR-0020(founder 2026-08-11):控制中心只留 亮度 / QR / 勿擾 / 找手機
       (dev 版另加 gesture),當成 App List 網格的頭部第一列;其餘工具
       (計算機/手電筒/設定/滑鼠)都在下面的 app 網格裡,不重複。 */
    lv_obj_t *cc_row = lv_obj_create(control_center_window);
    lv_obj_set_size(cc_row, LV_HOR_RES, 120);
    lv_obj_set_style_bg_opa(cc_row, LV_OPA_0, 0);
    lv_obj_set_style_border_width(cc_row, 0, 0);
    lv_obj_clear_flag(cc_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align_to(cc_row, bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 14);

    lv_obj_t *qrcode_btn = common_image_button(
        cc_row, &icon_qrcode, 100, 100, qrcode_btn_event_cb);
#if !kReleaseMode
    /* dev:四顆 —— QR / 勿擾 / 找手機 / gesture */
    lv_obj_align(qrcode_btn, LV_ALIGN_TOP_LEFT, 32, 0);
#else
    lv_obj_align(qrcode_btn, LV_ALIGN_TOP_LEFT, 70, 0);
#endif

    dndmode_enabled = SkaiWatchSys.DNDMode.config.status;
    dnd_mode_btn = common_image_button(cc_row, &icon_dnd_mode, 100, 100,
                                       dnd_mode_btn_event_cb);
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

    lv_obj_t *find_phone_btn = common_image_button(
        cc_row, FIND_PHONE, 100, 100, find_phone_btn_event_cb);

#if !kReleaseMode
    lv_obj_align_to(dnd_mode_btn, qrcode_btn, LV_ALIGN_OUT_RIGHT_MID, 4, 0);
    lv_obj_align_to(find_phone_btn, dnd_mode_btn, LV_ALIGN_OUT_RIGHT_MID, 4, 0);
    lv_obj_t *gesture_test_btn = common_image_button(
        cc_row, IMG_LOGO, 100, 100, gesture_test_btn_event_cb);
    lv_obj_align_to(gesture_test_btn, find_phone_btn, LV_ALIGN_OUT_RIGHT_MID, 4, 0);
#else
    lv_obj_align(dnd_mode_btn, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_align(find_phone_btn, LV_ALIGN_TOP_RIGHT, -70, 0);
#endif

    /* App List 網格接在控制列下方,同一個捲動容器(視覺融為一體)。 */
    {
        extern lv_obj_t *lv_app_list_layout_create_embedded(lv_obj_t *parent,
                                                            lv_coord_t y_top);
        lv_obj_update_layout(cc_row);
        lv_app_list_layout_create_embedded(control_center_window,
                                           lv_obj_get_y(cc_row) + 120 + 10);
    }

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
        /* 診斷用(2026-08-11):模糊圖在下拉過程中「應該是 255 卻看不到」,而這個 setter 是
           全檔唯一改它濃度的入口。只在值真的變動時印,所以拉一次最多幾行。 */
        static int s_last = -1;
        if ((int)opa != s_last)
        {
            s_last = (int)opa;
            LOG_W("[sp-blur] img_opa -> %d", (int)opa);
        }
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
            /* ADR-0020 (2026-08-11):HOME 四向全開 —
               TOP = 通知面板 (1,0)、BOTTOM = 控制中心+App List (1,2)、
               LEFT = 合併 session+actions 頁 (0,1)、RIGHT = 媒體欄 (2,1)。 */
            pages[i] = lv_tileview_add_tile(
                app_clock_main_status_bar, 1, i,
                LV_DIR_TOP | LV_DIR_BOTTOM | LV_DIR_LEFT | LV_DIR_RIGHT);
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
                /* LV_DIR_HOR，不再只有 LEFT：左邊回錶盤，右邊是下一台設備的 session
                   欄（founder 2026-08-10：再往左滑就進到第二台）。垂直仍然不開 —— 這一
                   欄的下拉由 session tile 自己的 catcher 手動驅動。 */
                pages[i] = lv_tileview_add_tile(app_clock_main_status_bar, 2, 1,
                                                LV_DIR_HOR);
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
    /* ADR-0020:右側媒體欄 (2+c,1) 與各自的滑鼠入口停車位 (2+c,0)。欄 2 的 tile
       就是迴圈裡建好的那格 (2,1);欄 3.. 在這裡補。全部先建滿 MAX_SYNCED_DEVICES ——
       tileview 的格子不能事後抽換,設備數是執行期才知道的;多出來的欄由
       media_cols_apply_dirs 用 tile->dir 鎖住。 */
    s_media_tile[0] = pages[INSTRUCTION_LIST_PAGE_INDEX];
    for (int c = 0; c < MAX_SYNCED_DEVICES; c++)
    {
        uint8_t col = (uint8_t)(MEDIA_COL_FIRST + c);
        if (c > 0)
        {
            s_media_tile[c] =
                lv_tileview_add_tile(app_clock_main_status_bar, col, 1, LV_DIR_HOR);
            lv_obj_set_size(s_media_tile[c], LV_HOR_RES_MAX, LV_VER_RES_MAX);
            lv_obj_set_style_bg_opa(s_media_tile[c], LV_OPA_TRANSP, 0);
            lv_obj_set_scrollbar_mode(s_media_tile[c], LV_SCROLLBAR_MODE_OFF);
        }
        /* 滑鼠入口停車位在**下方** (col,2) —— founder R2:從媒體頁往上拉出滑鼠。 */
        s_mouse_park_tile[c] =
            lv_tileview_add_tile(app_clock_main_status_bar, col, 2, LV_DIR_TOP);
        lv_obj_set_size(s_mouse_park_tile[c], LV_HOR_RES_MAX, LV_VER_RES_MAX);
        lv_obj_set_style_bg_opa(s_mouse_park_tile[c], LV_OPA_TRANSP, 0);
        lv_obj_set_scrollbar_mode(s_mouse_park_tile[c], LV_SCROLLBAR_MODE_OFF);

        /* 媒體頁常駐建在每一欄裡(內容輕:曲名 + 控制鈕);曲名路由由 settle 的
           media_col_bind 綁到當前欄。頂部設備列(圓點/名稱/箭頭)蓋在其上。 */
        {
            extern lv_obj_t *hid_mouse_media_page_create(lv_obj_t *parent);
            s_media_page[c] = hid_mouse_media_page_create(s_media_tile[c]);
        }
        media_col_header_build(c);

        /* 滑鼠入口停車位的預覽:黑底 + 滑鼠圖示。settle 在這格的瞬間會切換成
           真正的滑鼠模式圖層(同為黑底,視覺上是原地亮起)。 */
        {
            lv_obj_t *icon = lv_img_create(s_mouse_park_tile[c]);
            lv_img_set_src(icon, &mouse_mode_icon);
            lv_obj_center(icon);
            lv_obj_clear_flag(icon, LV_OBJ_FLAG_CLICKABLE);
        }
    }

    clock_main_media_cols_refresh(); /* 開機先鎖住多餘的欄(設備到齊後會再刷) */

    lv_obj_set_tile_id(app_clock_main_status_bar, 1, 1, false);
    lv_obj_add_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);

    /* actions 資料層 + 浮層 bar(lv_layer_top)。左頁的 actions 區段從這份資料
       匯出(instruction_list_export_*),所以要在 session 列表之前建好。 */
    extern lv_obj_t *lv_instruction_list_layout_create(lv_obj_t * parent);
    LOG_I("clock_status_bar: before instruction_list_layout_create");
    lv_instruction_list_layout_create(pages[MAIN_PAGE_TYPE_RIGHT]);
    LOG_I("clock_status_bar: after instruction_list_layout_create");

    /* ADR-0020 左頁 (0,1) = 跨設備合併 session 列表(通知卡樣式 + 設備副標)
       + 下接 actions 列表。 */
    extern lv_obj_t *lv_session_pager_create(lv_obj_t * parent);
    lv_session_pager_create(pages[MAIN_PAGE_TYPE_RIGHT]);

    /* 上 tile = 頂部面板(ADR-0020 之後只剩通知列表 + 滑鼠模式 Exit 鈕)。 */
    LOG_I("clock_status_bar: before top_panel_create");
    lv_top_panel_create(pages[MESSAGE_PAGE_INDEX], par);
    LOG_I("clock_status_bar: after top_panel_create");

    /* 下 tile (1,2) = 控制中心(亮度/QR/勿擾/找手機,dev 加 gesture)當頭部 +
       App List 網格(ADR-0020:合併為一頁,從錶盤往上拉進入)。 */
    control_center_layout_create(pages[MAIN_PAGE_TYPE_DOWN]);

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

/* Return to the watch face from the device_pager (left tile (0,1)). The pager's
   horizontal carousel owns left/right swipes between devices, so it can't also
   chain a swipe back to home; instead the pager calls this when the user drags
   left past the last device (the inverse of the left-edge pull-in). Mirrors
   the reset done at init: snap the main tileview to home and hide the overlay. */
/* 滑鼠模式下把面板拉出來：只負責「亮出主 tileview 並停在 HOME」，接下來的
   跟手/snap 交給 LVGL 原生 —— press 下一 tick 會轉給 tileview（與錶面上的
   notification_status_bar_cb 同一機制）。由 hid_mouse 頂部區的下拉分支呼叫
   （hid_mouse_set_pulldown_cb）。 */
void clock_main_panel_reveal(void)
{
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    lv_obj_set_tile_id(app_clock_main_status_bar, 1, 1, false);
    lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
    if (status_bar_bg_main && lv_obj_is_valid(status_bar_bg_main))
        lv_obj_move_foreground(status_bar_bg_main);
    lv_obj_move_foreground(app_clock_main_status_bar);
}

/* 面板必須永遠壓在滑鼠圖層之上（圖層是 par 的兄弟，show 時會 move_foreground）。*/
void clock_main_status_bar_to_front(void)
{
    if (status_bar_bg_main && lv_obj_is_valid(status_bar_bg_main))
        lv_obj_move_foreground(status_bar_bg_main);
}

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
    /* 面板本體隨 status_bar_bg_main 一起被刪，但滑鼠圖層是 par 的兄弟，
       要自己收（順帶把 hid_mouse 的 mode / route 還原）。 */
    lv_top_panel_deinit();
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