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

/* 頂部面板的兩個「停車位」(founder 2026-08-10)。
   面板本體平常掛在 (1,0)（錶盤正上方）；從 session tile (2,1) 下拉時,把面板的所有子
   物件整批搬到同一欄正上方的 (2,0),這樣下拉就是同一欄的垂直捲動 —— 全程跟手、沒有
   先跳回錶盤那一下（founder:「按下去的瞬間為什麼會跳到錶盤才能拉下」）。手勢結束或
   離開面板時再搬回 (1,0),錶盤自己的下拉才不會落空。
   為什麼是搬子物件而不是搬整個面板:lv_top_panel_create 把設備列 / 圓點 / pager 等
   直接掛在傳進去的 tile 上,沒有單一 root 可以 reparent。 */
static lv_obj_t *s_panel_home_tile; /* (1,0) 錶盤上方 */

/* 右側「一台設備一欄」(founder 2026-08-10):欄 2+i = 第 i 台設備的 session 列表,
   正上方 (2+i,0) 是面板停車位 —— 於是每一欄往下拉,拉出來的都是那一欄那台的媒體頁,
   而且是同一欄的垂直捲動(不跨欄,零跳轉)。
   欄的順序 = 設備 registry 的順序,跟頂部面板媒體頁的順序是同一份,所以「左右滑媒體頁」
   與「左右滑 session 欄」天生對得起來,不需要額外的映射表。 */
#define SESSION_COL_FIRST 2 /* 錶盤是欄 1;右邊從欄 2 開始 */
static lv_obj_t *s_session_tile[MAX_SYNCED_DEVICES];
static lv_obj_t *s_panel_park_tile[MAX_SYNCED_DEVICES];
static int s_panel_parked_col = -1; /* 面板目前停在哪一欄(-1 = 在錶盤那格) */
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
    if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
        lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
    set_clock_main_status_opa(0, false);
}

/* session tile (2,1) 往下拉 → 頂部面板的媒體頁（founder 2026-08-10）。
   這兩個 tile 在 tileview 裡不相鄰((2,1) vs (1,0)),所以沒有原生的手指跟隨路徑可走 —
   先 snap 回 HOME(無動畫,中間那格本來就沒東西可看),再用動畫往上開面板,看起來就是
   「從這頁把面板拉下來」。落在哪一頁由 lv_top_panel_open_media() 決定,這裡不碰。 */
/* 開始「從 session tile 下拉頂部面板」的跟手手勢。之後的 update/end 直接用錶盤那組
   (clock_main_notify_follow_update / _end) —— 同一條 Y 軸、同一個 commit 判準,面板下拉
   的手感就跟從錶盤拉完全一致(founder 2026-08-10:「我要的是跟我在錶盤拉下通知列表一樣的
   效果」,原本用 set_tile_id 一次到位所以是瞬移)。
   殘留:begin 這裡必須把 X 從第 2 欄 snap 回 HOME 欄,因為面板掛在 (1,0),跟 session
   tile (2,1) 在格子上不相鄰,沒有能同時跟手的路徑。所以手勢一開始的那一瞬間,底下的
   背景會從 session 列表換成錶面,面板才開始跟著手指下來。要完全無跳,得把面板從
   tileview 裡搬出來變成浮層 —— 那是另一個範圍的改動,沒有先問過不做。 */
/* 把面板的子物件整批搬到另一個 tile。取 child 0 反覆搬,所以到達端的順序與來源一致
   （z-order 保持）。物件指標不會因為換 parent 而失效,lv_top_panel 內部存的那些
   handle 全部照用。 */
/* 面板現在停在哪一格(NULL = 錶盤那格)。 */
static lv_obj_t *session_panel_host(void)
{
    if (s_panel_parked_col < 0 || s_panel_parked_col >= MAX_SYNCED_DEVICES)
        return s_panel_home_tile;
    return s_panel_park_tile[s_panel_parked_col];
}

/* 把面板的子物件整批搬到第 col 欄的停車位(col < 0 = 搬回錶盤那格)。 */
static void session_panel_move_to_col(int col)
{
    lv_obj_t *src = session_panel_host();
    lv_obj_t *dst = (col < 0 || col >= MAX_SYNCED_DEVICES) ? s_panel_home_tile
                                                           : s_panel_park_tile[col];
    if (!src || !dst || !lv_obj_is_valid(src) || !lv_obj_is_valid(dst) || src == dst)
    {
        s_panel_parked_col = (col >= 0 && col < MAX_SYNCED_DEVICES) ? col : -1;
        return;
    }
    while (lv_obj_get_child_cnt(src) > 0)
        lv_obj_set_parent(lv_obj_get_child(src, 0), dst);
    s_panel_parked_col = (col >= 0 && col < MAX_SYNCED_DEVICES) ? col : -1;
}

/* hid_mouse.h 在本檔這個位置還沒被 include（它在下方才進來），而 registry 的順序是欄位
   順序的唯一真相 —— 沿用本檔既有的就地 extern 慣例。 */
extern int hid_mouse_device_count(void);
extern int hid_mouse_active_device_index(void);

/* 讓「滑得到的欄數」跟著設備數走。
   格子是開機時就建滿 MAX_SYNCED_DEVICES 的（tileview 不能事後抽換格子），所以擋住多餘
   欄位的唯一辦法是改每一格註冊時的方向旗標 —— founder 2026-08-10 實測:只在 settle 裡
   夾住「顯示哪一台」完全沒用,他一路滑過四欄全是空白背景,因為能不能往右滑是 tileview
   讀 tile->dir 決定的,跟我們的顯示邏輯無關。
   最後一欄不給 LV_DIR_RIGHT;第 0 欄保留 LV_DIR_LEFT 回錶盤。設備數會變(配對/離線),
   所以每次 settle 都重算一次,成本只是幾個賦值。 */
static void session_cols_apply_dirs(int col_count)
{
    for (int c = 0; c < MAX_SYNCED_DEVICES; c++)
    {
        lv_dir_t d = LV_DIR_LEFT;
        if (c + 1 < col_count)
            d |= LV_DIR_RIGHT;
        if (s_session_tile[c] && lv_obj_is_valid(s_session_tile[c]))
            ((lv_tileview_tile_t *)s_session_tile[c])->dir = d;
        /* 停車位只上下:面板自己的左右換頁是它內部的 pager,不是 tileview 的欄。 */
        if (s_panel_park_tile[c] && lv_obj_is_valid(s_panel_park_tile[c]))
            ((lv_tileview_tile_t *)s_panel_park_tile[c])->dir = LV_DIR_VER;
    }
}

/* 有幾欄是真的:設備數,但至少 1 —— 一台都沒有時仍留一欄,使用者才有地方看到空狀態
   和「按麥克風開新的」,而不是右邊整片消失。 */
static int session_col_count(void)
{
    int n = hid_mouse_device_count();
    if (n < 1) n = 1;
    if (n > MAX_SYNCED_DEVICES) n = MAX_SYNCED_DEVICES;
    return n;
}

/* 面板不在錶盤那格時,任何「不是停在 session 面板」的 settle 都要把它搬回去,否則
   錶盤自己的下拉會拉出一格空白。 */
static void session_panel_park_home(void)
{
    if (s_panel_parked_col >= 0)
        session_panel_move_to_col(-1);
}

/* 目前捲動位置換算成欄 / 列。tile 加入順序的索引(active_pos)在補了 8 格之後已經沒有
   可讀性,而格子座標本來就是我們要的語意。 */
static int session_scroll_col(lv_obj_t *tv)
{
    return (int)((lv_obj_get_scroll_x(tv) + LV_HOR_RES / 2) / LV_HOR_RES);
}
static int session_scroll_row(lv_obj_t *tv)
{
    return (int)((lv_obj_get_scroll_y(tv) + LV_VER_RES / 2) / LV_VER_RES);
}

/* 面板停在哪一欄,由它「現在在哪一頁」決定(founder 2026-08-10):
     設備媒體頁 → session 欄 (2,0),往上收回到那台設備的 session 列表
     通知列表 / 控制中心(手機範疇) → 錶盤欄 (1,0),往上收回到錶盤
   面板是全螢幕的,而且兩欄的第 0 列都只有它,所以在 row 0 換欄的瞬間畫面完全一樣 —
   使用者看不到任何位移,只有「往上收會回到哪裡」跟著改變。
   由 lv_top_panel 在自己換頁 settle 時呼叫。 */
void clock_main_panel_park_for_page(bool device_page)
{
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    if (!s_panel_home_tile)
        return;
    /* 只有面板正開著(停在第 0 列)才需要換欄;沒開著的話 park 由既有路徑處理。 */
    if (lv_obj_get_scroll_y(app_clock_main_status_bar) != 0)
        return;
    /* 設備媒體頁 → 停在「那一台自己的欄」,所以往上收回的是那台的 session 列表。
       欄號直接取自面板正在顯示的設備索引,跟 registry 同一份順序。 */
    int want_col = -1;
    if (device_page)
    {
        int dev = hid_mouse_active_device_index();
        if (dev < 0) dev = 0;
        if (dev >= session_col_count()) dev = session_col_count() - 1;
        want_col = dev;
    }
    if (want_col == s_panel_parked_col)
        return; /* 已經停對欄 */
    session_panel_move_to_col(want_col);
    lv_obj_set_tile_id(app_clock_main_status_bar,
                       (uint8_t)((want_col < 0) ? 1 : (SESSION_COL_FIRST + want_col)), 0,
                       false);
    LOG_W("[sp-pull] panel parked in col %d", want_col);
}

void clock_main_session_panel_follow_begin(void)
{
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
    {
        LOG_W("[sp-pull] panel follow: no tileview");
        return;
    }
    /* 從哪一欄拉的,面板就停到那一欄 —— 於是拉下來的是那一欄那台的媒體頁,收回去也回
       到同一欄的 session 列表。 */
    int col = session_scroll_col(app_clock_main_status_bar) - SESSION_COL_FIRST;
    if (col < 0) col = 0;
    if (col >= session_col_count()) col = session_col_count() - 1;
    LOG_W("[sp-pull] panel follow begin col=%d", col);
    lv_top_panel_open_media_for(col); /* 落點 = 這一欄那台的媒體頁 */
    session_panel_move_to_col(col);   /* 之後全程只動 scroll_y,不跨欄 */
    /* session 頁釘在 tileview 的父物件上:捲動只帶走面板,頁面留在原地被蓋住
       (founder:「應該要是媒體中心蓋下來」)。 */
    {
        /* = status_bar_bg_main，但那個 static 宣告在本檔更下面；取 tileview 的 parent
           是同一個物件，還少一個宣告順序的依賴。 */
        extern void session_pager_pin_for_panel(lv_obj_t * fixed_parent);
        session_pager_pin_for_panel(lv_obj_get_parent(app_clock_main_status_bar));
    }
    lv_obj_clear_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
    if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
        lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
}

/* 放開:跟錶盤那組同一個判準(看放開當下的實際 scroll,不是累計位移),只是 commit 回
   第 2 欄而不是 HOME 欄。取消的話立刻把面板搬回錶盤那格。 */
void clock_main_session_panel_follow_end(lv_coord_t dy, lv_coord_t vy)
{
    (void)dy;
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    lv_coord_t sy = lv_obj_get_scroll_y(app_clock_main_status_bar);
    bool open_panel = (sy <= LV_VER_RES - LV_VER_RES / 4) || (vy > 6);
    /* 停在原來那一欄 —— 面板在 begin 就搬過去了,這裡只決定停第 0 列還是第 1 列。 */
    int col = (s_panel_parked_col >= 0) ? s_panel_parked_col
                                        : (session_scroll_col(app_clock_main_status_bar) -
                                           SESSION_COL_FIRST);
    if (col < 0) col = 0;
    LOG_W("[sp-pull] follow end col=%d sy=%d vy=%d open=%d", col, (int)sy, (int)vy,
          (int)open_panel);
    lv_obj_set_tile_id(app_clock_main_status_bar, (uint8_t)(SESSION_COL_FIRST + col),
                       open_panel ? 0 : 1, true);
    if (!open_panel)
        session_panel_park_home();
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

/* ---- Notification / control-center finger-follow from the same full-screen face
   swipe catcher ---------------------------------------------------------------- *
   Mirrors clock_main_applist_follow_* on the Y axis: HOME (row 1) already has both
   LV_DIR_TOP and LV_DIR_BOTTOM registered (see the tile add call below), so unlike
   the App List column this direction never needed the mid-drag retreat fix — a
   live reversal already scrolls back cleanly either way. A downward pull (dy>0)
   reveals the message/notification tile (row 0, scroll_y toward 0); an upward pull
   (dy<0) reveals control-center (row 2, scroll_y toward 2*LV_VER_RES). */
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
    lv_coord_t sy = LV_VER_RES - dy; /* dy>0 (down) -> sy<466 toward 0 (notification) */
    if (sy < 0) sy = 0;
    /* 上界只到 home：往上拉沒有頁可去（控制中心已搬進頂部面板），夾住就不會
       出現「跟著手指往上空滑一段再彈回來」。 */
    if (sy > LV_VER_RES) sy = LV_VER_RES;
    lv_obj_scroll_to_y(app_clock_main_status_bar, sy, LV_ANIM_OFF);
}

void clock_main_notify_follow_end(lv_coord_t dy, lv_coord_t vy)
{
    (void)dy;
    if (!app_clock_main_status_bar || !lv_obj_is_valid(app_clock_main_status_bar))
        return;
    /* Commit by the LIVE scroll position, not the raw cumulative dy from the press
       point — unlike the App List (a single one-sided reveal), this axis has two
       opposite targets, so a big enough reversal (drag down past the threshold,
       then swipe back up past it) could cross the OPPOSITE side's raw-dy threshold
       and launch control-center when the user was just trying to cancel back out
       of notifications. Read the actual scroll_y (mirrors how
       instruction_list_reveal_drag_end reads the live translate_x) so only where
       you ACTUALLY let go decides which tile wins. */
    lv_coord_t sy = lv_obj_get_scroll_y(app_clock_main_status_bar);
    /* 2026-08-06: 只剩「往下拉 = 頂部面板」一個去處；控制中心搬進面板，
       錶盤往上滑不再開頁（HOME 已移除 LV_DIR_BOTTOM）。 */
    bool open_notify = (sy <= LV_VER_RES - LV_VER_RES / 4) || (vy > 6);
    lv_obj_set_tile_id(app_clock_main_status_bar, 1, open_notify ? 0 : 1, true);
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
        /* session 欄的變暗（founder 2026-08-10:「需要像錶盤那樣有個慢慢變黑的背景」）。
           **必須放在這個 case 的最前面**:底下 sx > 466 那條 App List／右欄的分支自己
           會 break（本檔約 40 行後），我們這一欄的 scroll_x 是 932，永遠先撞上那個
           早退，寫在 case 結尾的話一次都不會執行。
           用 scroll_y 當唯一訊號:拉下 / 收回 / 放開後的慣性滑行全部吃得到，不必在手勢
           與原生捲動兩邊各補一次。466（session 列）→ 0（面板列）對應 0 → 204。 */
        /* 面板正停在 session 欄 = 我們的下拉/收回進行中,頂部時間的濃度由下面那段
           垂直換算獨佔。否則後面 sx > 466 的 App List 分支會用水平位移算出滿值再設
           一次,把我們的淡出蓋掉(founder 2026-08-10:「上面的時間沒有」)。 */
        bool session_panel_owns_time = (s_panel_parked_col >= 0);
        if (session_panel_owns_time)
        {
            lv_coord_t dim_sy = lv_obj_get_scroll_y(obj);
            if (dim_sy < 0) dim_sy = 0;
            if (dim_sy > LV_VER_RES) dim_sy = LV_VER_RES;
            extern void session_pager_set_dim(lv_opa_t opa);
            session_pager_set_dim((lv_opa_t)((LV_VER_RES - dim_sy) * 204 / LV_VER_RES));
            /* 頂部時間在 lv_layer_top 的全域列上,畫在所有 tile 之上 —— scrim 是 session
               頁的子物件,蓋不到它(founder 2026-08-10:「上面的時間沒有變暗」)。跟錶盤
               路徑一樣另外淡掉它,用同一個 scroll_y 反向換算,兩者才會同步。 */
            set_instruction_list_time_opa((uint8_t)(dim_sy * 255 / LV_VER_RES));
        }
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
            /* HOME's registered tile dir (LV_DIR_TOP|BOTTOM|RIGHT, no LEFT — see the
               tile add call below) is what lv_tileview locks lv_obj_set_scroll_dir to
               for the WHOLE press: LV_EVENT_SCROLL_END only re-applies a tile's dir
               when the indev is no longer pressed (lv_tileview.c tileview_event_cb),
               so mid-drag the missing LEFT makes retreating from a partial App List
               pull impossible — scroll_x can only climb, forcing a full commit on
               release no matter which way you're dragging when you let go. Punch a
               temporary LEFT hole in only while sx is past HOME (mid pull-out) so the
               live drag can retreat; drop it the instant sx is back at HOME so this
               can't also open the old device_pager-vs-@-list conflict (P3) that got
               LEFT permanently removed from HOME's own registration. */
            static bool s_applist_retreat_open = false;
            if (sx > 466 && !s_applist_retreat_open)
            {
                s_applist_retreat_open = true;
                lv_obj_set_scroll_dir(obj, LV_DIR_TOP | LV_DIR_RIGHT |
                                               LV_DIR_LEFT);
            }
            else if (sx <= 466 && s_applist_retreat_open)
            {
                s_applist_retreat_open = false;
                lv_obj_set_scroll_dir(obj, LV_DIR_TOP | LV_DIR_RIGHT);
            }
            if (sx > 466)
            {
                extern void instruction_list_bar_set_blur_amount(uint8_t opa);
                lv_coord_t pull = sx - 466; /* 0..466 */
                lv_coord_t opa = pull * 255 / 466;
                if (opa > 255) opa = 255;
                instruction_list_bar_set_blur_amount((uint8_t)opa);
                if (!session_panel_owns_time)
                    set_instruction_list_time_opa((uint8_t)opa);
                /* Same pull fades in the blurred dial behind the session pager — the SCREEN-LEVEL
                   gaus_dial_bg, exactly as the left action list uses it. It must not be parented
                   to the tile: that travels with the tileview, and the backdrop then drifts
                   sideways with the page instead of sitting still (founder 2026-08-05). Black
                   underlay + blurred image ramp together, so the dial dissolves into the blur
                   rather than the page arriving on a hard cut. */
                if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
                {
                    lv_obj_clear_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_set_style_bg_opa(gaus_dial_bg, (lv_opa_t)opa, 0);
                    if (gaus_dial_img && lv_obj_is_valid(gaus_dial_img))
                        lv_obj_set_style_img_opa(gaus_dial_img, (lv_opa_t)opa, 0);
                }
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
        /* 滑鼠模式下拉面板的黑底跟手漸黑（錶盤路徑由 gaus_dial_bg 負責，
           那層在滑鼠圖層底下看不到，所以另走面板自己那份）。scroll_y 從 466
           (home) 降到 0 (面板全開) → up 就是下拉進度。 */
        {
            lv_coord_t up = 466 - lv_obj_get_scroll_y(obj);
            if (up < 0) up = 0;
            if (up > 466) up = 466;
            lv_top_panel_set_backdrop_opa((uint8_t)((int32_t)up * 204 / 466));
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
            /* 垂直方向唯一的去處是頂部面板，那裡不顯示電量 → 一拉就關掉，
               不要讓它跟著滑一路淡入再於 settle 突然消失。 */
            shady_transparency = abs(scroll_second_y);
            set_instruction_list_battery_opa(LV_OPA_TRANSP);
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
    case LV_EVENT_SCROLL_BEGIN:
    {
        /* 收回面板是 LVGL 原生捲動((2,0) 是 LV_DIR_VER),不經過我們的下拉手勢,所以
           session 頁還在自己的 tile 裡、會跟著捲上來 —— founder 2026-08-10:「往上拉
           回去還是會看到 session 從下面上來」。面板停在 session 欄時(= 面板開著,唯一
           的去處就是收回去),捲動一開始就把頁面釘住,關閉於是跟開啟對稱:面板走,頁面
           留在原地。橫向不必擔心:(2,0) 只允許垂直,開著面板時滑不到別欄。 */
        if (s_panel_parked_col >= 0)
        {
            extern void session_pager_pin_for_panel(lv_obj_t * fixed_parent);
            session_pager_pin_for_panel(lv_obj_get_parent(app_clock_main_status_bar));
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
            /* Landing back at HOME — including a CANCELLED reveal, which settles
               here with active_pos unchanged from before the gesture (1), so this
               must run ahead of the middle_layer_tileview_index early-return below,
               not only in the "moved to a new tile" path further down. Without it:
               1) the full-screen swipe catcher can end up covered and only an edge
                  touch (a different object) would re-foreground it;
               2) worse, gaus_dial_bg — full-screen, CLICKABLE by lv_obj_create's
                  default, un-hidden by clock_main_notify_follow_begin for the
                  finger-follow blur — never gets re-hidden on a cancel (that only
                  happened in the "moved to a new tile" branch further down), so it
                  sits on top intercepting every touch afterward even though nothing
                  LOOKS different, since the scroll position is back at home. */
            extern void clock_main_face_swipe_catcher_foreground(void);
            clock_main_face_swipe_catcher_foreground();
            if (gaus_dial_bg && lv_obj_is_valid(gaus_dial_bg))
                lv_obj_add_flag(gaus_dial_bg, LV_OBJ_FLAG_HIDDEN);
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
            /* 滑鼠模式時 HOME 底下不是錶面而是 hid_mouse 圖層，它自帶底部 bar
               (bottom_swipe_area / skaibar)，全域 bar 再冒出來會疊兩條。 */
            instruction_list_bar_set_visible((active_pos == MAIN_PAGE_TYPE_HOME ||
                                              active_pos == MAIN_PAGE_TYPE_RIGHT) &&
                                             gui_app_is_actived("Main") &&
                                             !lv_top_panel_mouse_mode());
        }
        /* 頂部面板 settle：進來時刷新設備清單 / 媒體頁 / 底部按鈕狀態。
           面板有 1+N 個可能的落點 —— 錶盤上方 (1,0) 與每一台設備自己那欄的上方
           (2+i,0)，全部是同一個面板實體，只是被搬過去。用格子座標判，不用 tile 加入
           順序的索引：補了 8 格之後那個索引已經沒有可讀性。 */
        int col_now = session_scroll_col(obj);
        int row_now = session_scroll_row(obj);
        /* 設備數會隨配對 / 上下線變動,每次 settle 重算可滑的欄數。 */
        session_cols_apply_dirs(session_col_count());
        bool on_session_panel = (row_now == 0 && col_now >= SESSION_COL_FIRST);
        bool on_session_list = (row_now == 1 && col_now >= SESSION_COL_FIRST);
        {
            bool on_panel = (active_pos == MAIN_PAGE_TYPE_UP || on_session_panel);
            lv_top_panel_set_open(on_panel);
            lv_top_panel_set_backdrop_opa(on_panel ? 204 : 0);
            /* 停在別的地方就把面板送回錶盤那格：使用者可能從 session 面板一路滑回
               錶盤，那條路徑不會經過我們的手勢 end，沒有這行錶盤下拉就會是空的。 */
            if (!on_session_panel)
                session_panel_park_home();
            /* 停在第 i 欄 → 那一欄顯示第 i 台的 sessions。換欄會關掉上一欄的對話
               （手錶同時只開一個），所以只在真的換了設備時才呼叫。 */
            if (on_session_list || on_session_panel)
            {
                int dev = col_now - SESSION_COL_FIRST;
                if (dev >= session_col_count())
                    dev = session_col_count() - 1;
                extern void session_pager_set_column(int device_index, lv_obj_t * tile);
                /* 把那一欄的 tile 一起交出去 —— session UI 只有一份,不搬過去的話使用者
                   滑到的就是一格空格子(founder 2026-08-11 實測第二欄全空)。 */
                session_pager_set_column(
                    dev, (dev >= 0 && dev < MAX_SYNCED_DEVICES) ? s_session_tile[dev] : NULL);
            }
            /* 手勢結束了 → session 頁回到自己的 tile。釘住只在拖曳期間成立；留著的話
               它會掛在 tileview 外面,滑到任何一頁都跟著出現。settle 是唯一該解除的
               時機:animation 還在跑時必須維持釘住,否則頁面會在動畫中途彈位。 */
            {
                extern void session_pager_pin_for_panel(lv_obj_t * fixed_parent);
                extern void session_pager_set_dim(lv_opa_t opa);
                session_pager_pin_for_panel(NULL);
                /* 停在 session 面板 = 頁面被完全蓋住,維持全暗；其餘一律清掉,否則
                   離開之後那層黑會留在 session 頁上。 */
                session_pager_set_dim(on_session_panel ? (lv_opa_t)204 : LV_OPA_TRANSP);
                /* 時間跟著 settle 收尾:停在面板 = 全暗(0)。停在別頁不在這裡復原 ——
                   各頁本來就會自己設定(session 列表那頁設 OPA_100),硬塞會蓋掉它們。 */
                if (on_session_panel)
                    set_instruction_list_time_opa(LV_OPA_TRANSP);
            }
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
            /* 2026-08-06: 面板頁不再顯示電量（founder:通知列表上面的電量拿掉），
               頂部那條位置現在是「控制中設備」名稱。 */
            set_instruction_list_battery_opa(LV_OPA_TRANSP);

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

    /* Tool-button grid below the slider. */
    lv_obj_t *cc_bottom = lv_obj_create(control_center_window);
    lv_obj_set_size(cc_bottom, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_opa(cc_bottom, LV_OPA_0, 0);
    lv_obj_set_style_border_width(cc_bottom, 0, 0);
    lv_obj_clear_flag(cc_bottom, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align_to(cc_bottom, bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /* Row 1: calculator / flashlight / mouse */
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

    /* Mouse replaced the recorder here (founder direction 2026-07-02); the
       recorder app stays reachable from the App List. */
    lv_obj_t *mouse_btn = common_image_button(
        cc_bottom, MOUSE_MODE_ICON, 100, 100, mouse_mode_icon_event_cb);
    lv_obj_align(mouse_btn, LV_ALIGN_TOP_RIGHT, -70, 0);

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
    lv_obj_align_to(dnd_mode_btn, mouse_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);
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
            /* 2026-08-06: LV_DIR_BOTTOM 拿掉 — 控制中心搬進頂部面板(從通知列表
               往右滑)，錶盤往上滑不再有頁面。上 tile 現在是整個面板。 */
            pages[i] = lv_tileview_add_tile(app_clock_main_status_bar, 1, i,
                                            LV_DIR_TOP | LV_DIR_RIGHT);
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
    /* session 欄正上方的面板停車位 (2,0)。在迴圈之後才加,所以拿到索引 5,既有的
       MAIN_PAGE_TYPE_*(0..4) 映射完全不動。LV_DIR_VER 只是給原生捲動用;實際下拉由
       session tile 的 catcher 手動驅動 scroll_y,所以 (2,1) 不用開 LV_DIR_TOP —— 也
       不該開,否則使用者在列表上隨便往上滑就會把空面板拉出來。 */
    s_panel_home_tile = pages[MESSAGE_PAGE_INDEX];
    /* 欄 2 的 session tile 就是迴圈裡建好的那格 (2,1);欄 3.. 在這裡補。每一欄都配一個
       正上方的面板停車位。全部先建滿 MAX_SYNCED_DEVICES —— tileview 的格子不能事後
       抽換,而設備數是執行期才知道的;多出來的欄由 settle 邏輯擋住不讓停(見
       session_col_clamp)。 */
    s_session_tile[0] = pages[INSTRUCTION_LIST_PAGE_INDEX];
    for (int c = 0; c < MAX_SYNCED_DEVICES; c++)
    {
        uint8_t col = (uint8_t)(SESSION_COL_FIRST + c);
        if (c > 0)
        {
            s_session_tile[c] =
                lv_tileview_add_tile(app_clock_main_status_bar, col, 1, LV_DIR_HOR);
            lv_obj_set_size(s_session_tile[c], LV_HOR_RES_MAX, LV_VER_RES_MAX);
            lv_obj_set_style_bg_opa(s_session_tile[c], LV_OPA_TRANSP, 0);
            lv_obj_set_scrollbar_mode(s_session_tile[c], LV_SCROLLBAR_MODE_OFF);
        }
        s_panel_park_tile[c] =
            lv_tileview_add_tile(app_clock_main_status_bar, col, 0, LV_DIR_VER);
        lv_obj_set_size(s_panel_park_tile[c], LV_HOR_RES_MAX, LV_VER_RES_MAX);
        lv_obj_set_style_bg_opa(s_panel_park_tile[c], LV_OPA_TRANSP, 0);
        lv_obj_set_scrollbar_mode(s_panel_park_tile[c], LV_SCROLLBAR_MODE_OFF);
    }

    session_cols_apply_dirs(session_col_count()); /* 開機時就把多餘的欄鎖起來 */

    lv_obj_set_tile_id(app_clock_main_status_bar, 1, 1, false);
    lv_obj_add_flag(app_clock_main_status_bar, LV_OBJ_FLAG_HIDDEN);
    /* 2026-08-06: 控制中心不再自己佔一個 tile — 它是頂部面板的最左頁，
       由 lv_top_panel_create 掛進去。(1,2) tile 保留但空著且不可達。 */

    extern lv_obj_t *lv_instruction_list_layout_create(lv_obj_t * parent);
    LOG_I("clock_status_bar: before instruction_list_layout_create");
    lv_instruction_list_layout_create(pages[INSTRUCTION_LIST_PAGE_INDEX]);
    LOG_I("clock_status_bar: after instruction_list_layout_create");

    /* 2026-06-25: the right-swipe tile (col 2 = INSTRUCTION_LIST_PAGE_INDEX) now
       hosts the App List grid. instruction_list_layout_create above floats its
       content on lv_layer_top and only tracks this tile as instruction_list_page
       (read-only, for scroll-snap), so the tile itself is free to render the app
       grid that swipes in from the right edge. The left mixed list keeps using
       the left-edge reveal overlay.
       2026-08-05: that tile now hosts the DESKTOP-SESSION PAGER instead — one
       horizontally-snapping page per desktop chat session. lv_app_list_layout.c is
       kept intact but unmounted (founder: "App List 先不顯示"); re-mounting it is a
       one-line change here. */
    extern lv_obj_t *lv_session_pager_create(lv_obj_t * parent);
    lv_session_pager_create(pages[INSTRUCTION_LIST_PAGE_INDEX]);
    /* 上 tile = 頂部面板（控制中心 ← 通知列表 → 各設備媒體中心 + 固定的頂部
       設備列 / 底部按鈕）。通知列表與控制中心都由面板內部建立。 */
    LOG_I("clock_status_bar: before top_panel_create");
    lv_top_panel_create(pages[MESSAGE_PAGE_INDEX], par);
    LOG_I("clock_status_bar: after top_panel_create");

    /* T4: device_pager 內容放右 tile (2,1)，鏡像左側 instruction_list。
       拉出靠原生 tileview 滑動。 */
    extern lv_obj_t *device_pager_create(lv_obj_t * parent);
    // device_pager_create(pages[MAIN_PAGE_TYPE_RIGHT]); // 2026-06-30: removed; left (0,1) slot will host the instruction list (static tile)

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