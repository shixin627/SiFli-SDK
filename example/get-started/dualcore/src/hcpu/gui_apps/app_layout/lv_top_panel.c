/**
 * @file lv_top_panel.c
 * @brief 錶盤頂部下拉面板 — 控制中心 / 通知列表 / 各設備媒體中心的共同容器。
 *        設計說明見 lv_top_panel.h。
 */
#include <rtthread.h>
#include <string.h>
#include "lvgl.h"
#include "lv_top_panel.h"
#include "hid_mouse.h"
#include "ui_img_helper.h"
#include "watch_global_data.h"
#include "ble_hid.h"
#include "lv_ext_resource_manager.h"

#define DBG_TAG "app.top_panel"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

LV_IMG_DECLARE(mouse_mode_icon);
LV_IMG_DECLARE(img_left_arrow);
LV_IMG_DECLARE(img_right_arrow);

/* 面板內容由狀態列 / 通知列表提供，避免把兩千行搬家 */
extern lv_obj_t *control_center_layout_create(lv_obj_t *parent);
extern lv_obj_t *lv_message_list_layout_create(lv_obj_t *parent);
extern void app_clock_status_bar_return_home(void);
extern void clock_main_panel_reveal(void);
extern void clock_main_status_bar_to_front(void);
extern void display_status_bar_area(uint32_t idx, bool display);
extern bool get_bluetooth_connection_status(void);

/* 水平頁序：0 = 控制中心、1 = 通知列表、2+i = 第 i 台設備的媒體中心 */
#define PAGE_CONTROL 0
#define PAGE_MESSAGE 1
#define PAGE_MEDIA_0 2

/* 頂部設備名列 / 底部按鈕的座標沿用滑鼠 app 媒體頁的既有位置，換頁時視覺不跳 */
#define DEV_DOT_Y 26
#define DEV_LABEL_Y 40
#define DEV_ARROW_DX 117
#define DEV_ARROW_Y 42
#define PANEL_BTN_SIZE 80
#define PANEL_BTN_MARGIN 18

static lv_obj_t *s_pager = NULL;
static lv_obj_t *s_cell_control = NULL;
static lv_obj_t *s_cell_message = NULL;
static lv_obj_t *s_media_tile[MAX_SYNCED_DEVICES];
static lv_obj_t *s_media_page[MAX_SYNCED_DEVICES];
static int s_media_count = 0;

static lv_obj_t *s_dev_strip = NULL;   /* 設備列的觸控攔截條（吃掉 press，不穿透）*/
static lv_obj_t *s_dev_dot = NULL;
static lv_obj_t *s_dev_label = NULL;
static lv_obj_t *s_arrow_left = NULL;
static lv_obj_t *s_arrow_right = NULL;

static lv_obj_t *s_btn = NULL;
static lv_obj_t *s_btn_img = NULL;
static lv_obj_t *s_btn_label = NULL;

static lv_obj_t *s_mouse_layer = NULL;
/* 滑鼠模式下拉面板時的黑色半透明底（蓋在觸控板上、面板之下），讓面板拉出來
   的視覺跟錶盤一致 —— 錶盤那邊是 gaus_dial_bg 提供，但它掛在滑鼠圖層底下、
   在這條路徑上被蓋住，所以另外備一份掛在圖層自己身上。 */
static lv_obj_t *s_backdrop = NULL;
static lv_obj_t *s_layer_parent = NULL;
static bool s_mouse_mode = false;
/* 進滑鼠模式時圖層先建好但**不顯示**，等面板收合動畫跑完才 un-hide。
   原因:滑鼠 UI 一亮，收合動畫的每一幀都得把整套觸控板重畫一次，畫面來不及、
   tick-based 的動畫就跳格 → 看起來是「媒體頁咻一下消失」而不是往上滑走
   （founder 2026-08-06）。離開時因為 hid_mouse_destroy 先把它拆了，動畫是
   蓋在便宜的錶面上跑，所以一直都很順 —— 這個 flag 就是把兩邊拉齊。 */
static bool s_mouse_reveal_pending = false;
static bool s_opened = false;

static void update_device_bar(void);
static void update_bottom_btn(void);
static void rebuild_media_pages(void);

/* -------------------------------------------------------------------------- */
/* 頁碼                                                                        */
/* -------------------------------------------------------------------------- */

/* 用 scroll_x 反推目前頁，而不是比對 tile 指標 — 媒體頁會隨設備數增減重建，
   指標比對在重建當下會短暫落空，scroll_x 永遠有效。 */
static int current_page(void)
{
    if (s_pager == NULL || !lv_obj_is_valid(s_pager))
        return PAGE_MESSAGE;
    lv_coord_t sx = lv_obj_get_scroll_x(s_pager);
    if (sx < 0) sx = 0;
    return (int)((sx + LV_HOR_RES / 2) / LV_HOR_RES);
}

static void goto_page(int page, bool anim)
{
    if (s_pager == NULL || !lv_obj_is_valid(s_pager))
        return;
    int max_page = PAGE_MEDIA_0 + s_media_count - 1;
    if (max_page < PAGE_MESSAGE) max_page = PAGE_MESSAGE;
    if (page < PAGE_CONTROL) page = PAGE_CONTROL;
    if (page > max_page) page = max_page;
    lv_obj_set_tile_id(s_pager, (uint8_t)page, 0, anim);
}

/* -------------------------------------------------------------------------- */
/* 設備列                                                                      */
/* -------------------------------------------------------------------------- */

/* 頁面決定控制目標:媒體頁 = 該台設備；通知列表 / 控制中心 = 當前連線的手機
   （founder 2026-08-06）。所以頂部那條在非媒體頁顯示「手機」+ 藍牙連線狀態燈。*/
static bool page_targets_phone(void)
{
    return current_page() < PAGE_MEDIA_0;
}

/* 箭頭走的軸線:通知列表(手機) → 設備0 → 設備1 → …。控制中心不在這條軸上
   （它是左滑進去的），末端就是最後一台設備。 */
static int last_device_page(void)
{
    return (s_media_count > 0) ? (PAGE_MEDIA_0 + s_media_count - 1) : PAGE_MESSAGE;
}

static void update_device_bar(void)
{
    int cnt = hid_mouse_device_count();
    bool phone = page_targets_phone();
    int idx = phone ? -1 : hid_mouse_active_device_index();
    const char *name = phone ? LV_EXT_STR_GET_BY_KEY(connected_phone, "Phone")
                             : ((idx >= 0) ? hid_mouse_device_name(idx) : NULL);

    if (s_dev_label && lv_obj_is_valid(s_dev_label))
    {
        if (name && name[0])
        {
            lv_label_set_text(s_dev_label, name);
            lv_obj_clear_flag(s_dev_label, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(s_dev_label, LV_OBJ_FLAG_HIDDEN);
        }
    }
    /* 在線燈號:綠=在線 / 紅=斷線（與滑鼠 app 媒體頁同一套語意） */
    if (s_dev_dot && lv_obj_is_valid(s_dev_dot))
    {
        if (name && name[0])
        {
            bool online = phone ? get_bluetooth_connection_status()
                                : (idx >= 0 && hid_mouse_device_online(idx));
            lv_obj_set_style_bg_color(s_dev_dot,
                                      online ? lv_color_hex(0x4CAF50)
                                             : lv_color_hex(0xFF3B30),
                                      0);
            lv_obj_clear_flag(s_dev_dot, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(s_dev_dot, LV_OBJ_FLAG_HIDDEN);
        }
    }
    /* 不循環（founder 2026-08-06）：切到底那一側的箭頭直接消失。左端是通知
       列表(=手機)，所以從設備一路往左按也回得到通知列表。 */
    int page = current_page();
    bool show_left = (page > PAGE_MESSAGE);
    bool show_right = (page < last_device_page());
    if (s_arrow_left && lv_obj_is_valid(s_arrow_left))
    {
        if (show_left) lv_obj_clear_flag(s_arrow_left, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(s_arrow_left, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_arrow_right && lv_obj_is_valid(s_arrow_right))
    {
        if (show_right) lv_obj_clear_flag(s_arrow_right, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(s_arrow_right, LV_OBJ_FLAG_HIDDEN);
    }
    (void)cnt;
}

/* 箭頭 = 沿著「通知列表(手機) → 設備0 → 設備1 → …」這條軸線走一格，**不循環**
   （founder 2026-08-06：切到底就沒有那邊的箭頭，也要切得回通知列表）。
   只負責換頁 —— 控制目標由換頁 settle 統一決定（媒體頁 = 該台、通知列表 = 手機），
   不在這裡另外選台，免得兩條路各自寫一次選台邏輯而不同步。 */
static void arrow_step(int dir)
{
    int page = current_page();
    int target;
    if (page < PAGE_MESSAGE)
    {
        /* 人在控制中心:▶ 先回通知列表；◀ 已經到左端，不動。 */
        if (dir <= 0) return;
        target = PAGE_MESSAGE;
    }
    else
    {
        target = page + dir;
    }
    if (target < PAGE_MESSAGE || target > last_device_page())
        return;
    goto_page(target, true);
    update_device_bar();
}

static void arrow_prev_cb(lv_event_t *e) { (void)e; arrow_step(-1); }
static void arrow_next_cb(lv_event_t *e) { (void)e; arrow_step(+1); }

/* 設備列本身也吃左右滑（founder 2026-08-06：在設備名稱上左右滑想換設備，結果
   press 穿透到下面的通知卡去觸發滑動刪除）。這條 strip 是 CLICKABLE 的，press
   停在它身上不會落到列表；箭頭建在它之後 → z-order 在它之上，點箭頭照常。 */
#define DEV_STRIP_SWIPE_PX 30
static lv_coord_t s_strip_press_x = 0;
static bool s_strip_pressing = false;
static bool s_strip_fired = false;

/* strip 上的滑動走**完整頁面範圍**（含最左邊的控制中心），跟在列表空白處滑
   一樣；箭頭那條軸線刻意不含控制中心，兩者不共用。第一版把這裡也接到
   arrow_step，於是在名稱上往右滑進控制中心整個沒反應。 */
static void strip_step_page(int dir)
{
    int target = current_page() + dir;
    if (target < PAGE_CONTROL || target > last_device_page())
        return;
    goto_page(target, true);
    update_device_bar();
}

static void dev_strip_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (indev == NULL) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    if (code == LV_EVENT_PRESSED)
    {
        s_strip_press_x = p.x;
        s_strip_pressing = true;
        s_strip_fired = false;
    }
    /* 在 PRESSING 就判定、不等 RELEASED：這條 strip 沒有可水平捲動的祖先，
       橫拖時 LVGL 隨時可能因為些微垂直分量把 press 交給外層 tileview 而先送
       PRESS_LOST，等放開才動作等於看運氣（第一版就是這樣沒反應）。順帶手感
       也比較跟得上——拖到門檻就換頁。 */
    else if (code == LV_EVENT_PRESSING)
    {
        if (!s_strip_pressing || s_strip_fired) return;
        lv_coord_t dx = p.x - s_strip_press_x;
        if (dx <= -DEV_STRIP_SWIPE_PX)      { s_strip_fired = true; strip_step_page(+1); }
        else if (dx >= DEV_STRIP_SWIPE_PX)  { s_strip_fired = true; strip_step_page(-1); }
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
    {
        s_strip_pressing = false;
        s_strip_fired = false;
    }
}

/* -------------------------------------------------------------------------- */
/* 滑鼠模式圖層                                                                */
/* -------------------------------------------------------------------------- */

/* 滑鼠模式時「頂部往下拉」的去處：hid_mouse 頂部區判定為下拉後呼叫這裡，
   把主 tileview 亮出來，press 下一 tick 就轉給它原生跟手。 */
static void panel_reveal_cb(void)
{
    clock_main_panel_reveal();
}

static void mouse_layer_ensure(void)
{
    if (s_mouse_layer && lv_obj_is_valid(s_mouse_layer))
        return;
    if (s_layer_parent == NULL || !lv_obj_is_valid(s_layer_parent))
        return;
    s_mouse_layer = lv_obj_create(s_layer_parent);
    lv_obj_remove_style_all(s_mouse_layer);
    lv_obj_set_size(s_mouse_layer, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_pos(s_mouse_layer, 0, 0);
    lv_obj_set_style_bg_color(s_mouse_layer, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_mouse_layer, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_mouse_layer, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_mouse_layer, LV_OBJ_FLAG_HIDDEN);
}

/* 進滑鼠模式拆成兩段(founder 2026-08-06 兩次回報後的最終形)：
     prepare  — 按下按鈕當下:建 UI + 亮圖層。面板往上收的整段期間，底下看到的
                就已經是滑鼠頁(不是錶盤)，黑底隨收合淡出。
     activate — 收合動畫結束才做:hid_mouse_enter_mode() 會打開 IMU / 飛鼠資料流，
                訊息量大到會把 GUI thread 餓住(log 看得到 lvgl_mq full)；動畫吃
                tick，被餓到就跳格 = 「進去時媒體頁咻一下消失」。挪到動畫後就沒
                人跟它搶。
   離開那側本來就是先 destroy 再播動畫，所以一直很順，不必動。 */
static void mouse_layer_prepare(void)
{
    mouse_layer_ensure();
    if (s_mouse_layer == NULL)
        return;
    {
        /* cb 必須在 build_ui 之前裝好：hid_mouse 靠它決定「不要建自己的媒體
           tileview」+「下拉去面板」。 */
        hid_mouse_set_pulldown_cb(panel_reveal_cb);
        if (hid_mouse_ui_host() != s_mouse_layer)
        {
            lv_obj_clean(s_mouse_layer);
            hid_mouse_build_ui(s_mouse_layer);
        }
        /* 建在 build_ui 之後 = 圖層的最後一個子物件 = 壓在觸控板 UI 之上；
           面板本身在 status_bar_bg_main 裡、又在圖層之上，所以夾在中間。 */
        s_backdrop = lv_obj_create(s_mouse_layer);
        lv_obj_remove_style_all(s_backdrop);
        lv_obj_set_size(s_backdrop, LV_HOR_RES_MAX, LV_VER_RES_MAX);
        lv_obj_set_pos(s_backdrop, 0, 0);
        lv_obj_set_style_bg_color(s_backdrop, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(s_backdrop, LV_OPA_TRANSP, 0);
        lv_obj_clear_flag(s_backdrop, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(s_backdrop, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(s_backdrop, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_mouse_layer);
        clock_main_status_bar_to_front(); /* 面板要壓在滑鼠圖層之上 */
        lv_obj_clear_flag(s_mouse_layer, LV_OBJ_FLAG_HIDDEN);
        LOG_I("[top_panel] mouse layer prepared (visible under the panel)");
    }
}

/* 收合動畫結束後才開 mouse mode 本身（見上面 prepare/activate 的分工說明）。 */
static void mouse_layer_activate(void)
{
    if (s_mouse_layer == NULL || !lv_obj_is_valid(s_mouse_layer))
        return;
    hid_mouse_enter_mode();       /* 這裡面已把上/下/左緣 zone 收掉 */
    hid_mouse_set_hosted(true);   /* 面板擁有上下緣 */
    display_status_bar_area(3, false); /* 右緣 zone enter_mode 沒收，補收 */
    /* 控制目標是誰由「按下滑鼠鈕當下停在哪一頁」決定:媒體頁 → relay 到那台
       設備；通知列表/控制中心 → 不開 route，滑鼠直接控當前連線的手機。 */
    ble_hid_mouse_set_app_route(hid_mouse_active_device_index() >= 0);
    LOG_I("[top_panel] mouse mode ON");
}

static void mouse_layer_set(bool on)
{
    if (on == s_mouse_mode)
        return;

    if (on)
    {
        /* 當下就把圖層備好並亮出來(收合期間底下就是滑鼠頁)，只有會餓死 GUI
           thread 的 enter_mode 留到收合完才做。 */
        s_mouse_mode = true;
        s_mouse_reveal_pending = true;
        mouse_layer_prepare();
    }
    else
    {
        s_mouse_mode = false;
        ble_hid_mouse_set_app_route(false);
        hid_mouse_set_hosted(false);
        hid_mouse_destroy();             /* 內含 exit_mode，會還原上/下/左緣 */
        hid_mouse_set_pulldown_cb(NULL);
        display_status_bar_area(3, true);
        s_mouse_reveal_pending = false;
        s_backdrop = NULL; /* 隨 lv_obj_clean 一起消失 */
        if (s_mouse_layer && lv_obj_is_valid(s_mouse_layer))
        {
            lv_obj_clean(s_mouse_layer);
            lv_obj_add_flag(s_mouse_layer, LV_OBJ_FLAG_HIDDEN);
        }
        LOG_I("[top_panel] mouse mode OFF");
    }
    update_bottom_btn();
}

bool lv_top_panel_mouse_mode(void) { return s_mouse_mode; }

/* -------------------------------------------------------------------------- */
/* 底部按鈕                                                                    */
/* -------------------------------------------------------------------------- */

/* 按下底部鈕之後、面板還在往上收的那段期間先不要換外觀 —— 不然使用者會在
   頁面離場前就看到圖示先變（founder 2026-08-06:「他換得太早了」）。收完
   (lv_top_panel_set_open(false)) 才補畫，下次拉下來已經是新樣子。 */
static bool s_btn_defer = false;

static void update_bottom_btn(void)
{
    if (s_btn_defer)
        return;
    if (s_btn == NULL || !lv_obj_is_valid(s_btn))
        return;
    lv_obj_set_style_bg_color(
        s_btn, s_mouse_mode ? lv_color_hex(0xFF3B30) : lv_color_hex(0x333333),
        LV_PART_MAIN);
    if (s_btn_img && lv_obj_is_valid(s_btn_img))
    {
        if (s_mouse_mode) lv_obj_add_flag(s_btn_img, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_clear_flag(s_btn_img, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_btn_label && lv_obj_is_valid(s_btn_label))
    {
        if (s_mouse_mode) lv_obj_clear_flag(s_btn_label, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(s_btn_label, LV_OBJ_FLAG_HIDDEN);
    }
}

/* 進滑鼠模式:先收面板再開圖層 — 收起用主 tileview 的既有動畫，面板整片往上走，
   底下換成滑鼠模式（不是錶盤）。退出:反向，並把頁重設回通知列表，符合
   「回到錶盤後再下拉要看到通知列表」。 */
/* 收合動畫改用 async 起跑（進滑鼠模式那側）。原因:mouse_layer_prepare() 裡的
   hid_mouse_build_ui() 會把 LVGL thread 卡住數百 ms，如果緊接著在同一輪
   lv_timer_handler 內建動畫，LVGL 的 anim timer 這一輪算出來的 elapsed 就是那
   一大塊停頓，動畫第一步直接跳掉大半 = 「離開的動畫又變得超快」。丟到下一輪
   才建，那塊 tick 已經被吃掉，動畫從乾淨的時間基準開始跑。 */
static void collapse_async_cb(void *unused)
{
    (void)unused;
    app_clock_status_bar_return_home();
}

static void bottom_btn_cb(lv_event_t *e)
{
    (void)e;
    s_btn_defer = true; /* 收完再換外觀 */
    if (!s_mouse_mode)
    {
        mouse_layer_set(true);
        lv_async_call(collapse_async_cb, NULL);
    }
    else
    {
        mouse_layer_set(false);
        goto_page(PAGE_MESSAGE, false);
        app_clock_status_bar_return_home();
    }
}

/* -------------------------------------------------------------------------- */
/* 媒體頁                                                                      */
/* -------------------------------------------------------------------------- */

static void rebuild_media_pages(void)
{
    if (s_pager == NULL || !lv_obj_is_valid(s_pager))
        return;
    int n = hid_mouse_device_count();
    if (n > MAX_SYNCED_DEVICES) n = MAX_SYNCED_DEVICES;
    if (n == s_media_count)
        return;

    /* 縮減前先離開媒體區，否則刪到腳下那一頁 */
    if (n < s_media_count && current_page() >= PAGE_MEDIA_0)
        goto_page(PAGE_MESSAGE, false);

    while (s_media_count > n)
    {
        s_media_count--;
        if (s_media_tile[s_media_count] && lv_obj_is_valid(s_media_tile[s_media_count]))
            lv_obj_del(s_media_tile[s_media_count]);
        s_media_tile[s_media_count] = NULL;
        s_media_page[s_media_count] = NULL;
    }
    while (s_media_count < n)
    {
        lv_obj_t *tile = lv_tileview_add_tile(
            s_pager, (uint8_t)(PAGE_MEDIA_0 + s_media_count), 0, LV_DIR_HOR);
        lv_obj_set_size(tile, LV_HOR_RES_MAX, LV_VER_RES_MAX);
        lv_obj_set_style_bg_opa(tile, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(tile, 0, 0);
        lv_obj_set_style_pad_all(tile, 0, 0);
        lv_obj_set_scrollbar_mode(tile, LV_SCROLLBAR_MODE_OFF);
        s_media_tile[s_media_count] = tile;
        s_media_page[s_media_count] = hid_mouse_media_page_create(tile);
        s_media_count++;
    }
    LOG_I("[top_panel] media pages = %d", s_media_count);
}

/* -------------------------------------------------------------------------- */
/* 換頁                                                                        */
/* -------------------------------------------------------------------------- */

static void pager_value_changed_cb(lv_event_t *e)
{
    (void)e;
    int page = current_page();
    if (page >= PAGE_MEDIA_0)
    {
        int dev = page - PAGE_MEDIA_0;
        if (dev >= 0 && dev < s_media_count)
        {
            /* 曲名/播放狀態路由指向這一頁，再把控制目標換成這台（founder:
               滑到哪台就控制哪台）。先 bind 後選台，選台當下的 reset title 才
               會寫在正確的那一頁上。 */
            hid_mouse_media_page_bind(s_media_page[dev]);
            if (hid_mouse_active_device_index() != dev)
            {
                hid_mouse_media_page_reset_title(s_media_page[dev]);
                hid_mouse_set_active_device_index(dev);
            }
        }
    }
    else
    {
        /* 通知列表 / 控制中心 = 控制當前連線的手機 */
        hid_mouse_clear_active_device();
    }
    update_device_bar();
}

/* -------------------------------------------------------------------------- */
/* 對外                                                                        */
/* -------------------------------------------------------------------------- */

/* 面板下拉進度 → 黑底濃度（0 = 全收、204 = LV_OPA_80 全開）。由錶盤狀態列的
   SCROLL / VALUE_CHANGED 餵進來，所以是跟手漸黑，不是拉完才硬切。非滑鼠模式
   時什麼都不做 —— 那條路徑由 gaus_dial_bg 負責。 */
void lv_top_panel_set_backdrop_opa(uint8_t opa)
{
    if (s_backdrop == NULL || !lv_obj_is_valid(s_backdrop))
        return;
    if (!s_mouse_mode) opa = 0;
    if (opa == 0)
    {
        lv_obj_add_flag(s_backdrop, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_clear_flag(s_backdrop, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_opa(s_backdrop, (lv_opa_t)opa, 0);
}

void lv_top_panel_set_hor_enabled(bool enabled)
{
    if (s_pager == NULL || !lv_obj_is_valid(s_pager))
        return;
    lv_obj_set_scroll_dir(s_pager, enabled ? LV_DIR_HOR : LV_DIR_NONE);
}

void lv_top_panel_refresh_devices(void)
{
    /* 只有停在媒體頁時才需要「保證有一台 active」；停在通知列表/控制中心時
       目標本來就該是手機，硬挑一台會把控制目標搶走。 */
    rebuild_media_pages();
    if (!page_targets_phone())
        hid_mouse_ensure_active_device();
    update_device_bar();
}

void lv_top_panel_set_open(bool opened)
{
    s_opened = opened;
    if (!opened)
    {
        /* 面板已經完全收掉（動畫結束的 settle）→ 現在才把底部鈕換成新狀態，
           使用者看不到中途替換。 */
        if (s_mouse_reveal_pending)
        {
            s_mouse_reveal_pending = false;
            mouse_layer_activate();
        }
        if (s_btn_defer)
        {
            s_btn_defer = false;
            update_bottom_btn();
        }
        return;
    }
    s_btn_defer = false;
    lv_top_panel_refresh_devices();
    update_bottom_btn();
    /* 面板重新開啟:把曲名路由重新綁回目前這一頁（hid_mouse_destroy 會把 live
       指標清成 NULL，不重綁的話下拉回來曲名就停在舊值）。 */
    int page = current_page();
    if (page >= PAGE_MEDIA_0 && (page - PAGE_MEDIA_0) < s_media_count)
        hid_mouse_media_page_bind(s_media_page[page - PAGE_MEDIA_0]);
}

void lv_top_panel_deinit(void)
{
    if (s_mouse_mode)
        mouse_layer_set(false);
    if (s_mouse_layer && lv_obj_is_valid(s_mouse_layer))
        lv_obj_del(s_mouse_layer);
    s_mouse_layer = NULL;
    s_mouse_reveal_pending = false;
    s_backdrop = NULL;
    s_layer_parent = NULL;
    s_pager = NULL;
    s_cell_control = NULL;
    s_cell_message = NULL;
    s_dev_strip = NULL;
    s_dev_dot = s_dev_label = s_arrow_left = s_arrow_right = NULL;
    s_strip_pressing = false;
    s_btn = s_btn_img = s_btn_label = NULL;
    for (int i = 0; i < MAX_SYNCED_DEVICES; i++)
    {
        s_media_tile[i] = NULL;
        s_media_page[i] = NULL;
    }
    s_media_count = 0;
    s_opened = false;
}

lv_obj_t *lv_top_panel_create(lv_obj_t *tile, lv_obj_t *layer_parent)
{
    s_layer_parent = layer_parent;
    s_media_count = 0;
    for (int i = 0; i < MAX_SYNCED_DEVICES; i++)
    {
        s_media_tile[i] = NULL;
        s_media_page[i] = NULL;
    }

    /* --- 水平頁容器 ------------------------------------------------------- */
    s_pager = lv_tileview_create(tile);
    lv_obj_set_size(s_pager, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_pos(s_pager, 0, 0);
    lv_obj_set_style_bg_opa(s_pager, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_pager, 0, 0);
    lv_obj_set_style_pad_all(s_pager, 0, 0);
    lv_obj_set_scrollbar_mode(s_pager, LV_SCROLLBAR_MODE_OFF);
    /* 垂直方向要能往上傳給主 tileview（往上滑收面板），水平自己吃 */
    lv_obj_add_flag(s_pager, LV_OBJ_FLAG_SCROLL_CHAIN_VER);

    s_cell_control = lv_tileview_add_tile(s_pager, PAGE_CONTROL, 0, LV_DIR_RIGHT);
    lv_obj_set_size(s_cell_control, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_opa(s_cell_control, LV_OPA_TRANSP, 0);
    lv_obj_set_scrollbar_mode(s_cell_control, LV_SCROLLBAR_MODE_OFF);
    control_center_layout_create(s_cell_control);

    s_cell_message = lv_tileview_add_tile(s_pager, PAGE_MESSAGE, 0, LV_DIR_HOR);
    lv_obj_set_size(s_cell_message, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_opa(s_cell_message, LV_OPA_TRANSP, 0);
    lv_obj_set_scrollbar_mode(s_cell_message, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(s_cell_message, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_message_list_layout_create(s_cell_message);

    rebuild_media_pages();
    lv_obj_set_tile_id(s_pager, PAGE_MESSAGE, 0, false);
    lv_obj_add_event_cb(s_pager, pager_value_changed_cb, LV_EVENT_VALUE_CHANGED,
                        NULL);

    /* --- 固定頂部設備列（不隨頁捲動 → 掛在 tile 上，建在 pager 之後）------ */
    /* 先建攔截條:它要在名稱/箭頭之下、pager 之上 */
    s_dev_strip = lv_obj_create(tile);
    lv_obj_remove_style_all(s_dev_strip);
    lv_obj_set_size(s_dev_strip, LV_HOR_RES_MAX, 80);
    lv_obj_align(s_dev_strip, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(s_dev_strip, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(s_dev_strip, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_dev_strip, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_dev_strip, dev_strip_cb, LV_EVENT_ALL, NULL);

    s_dev_dot = lv_obj_create(tile);
    lv_obj_remove_style_all(s_dev_dot);
    lv_obj_set_size(s_dev_dot, 10, 10);
    lv_obj_align(s_dev_dot, LV_ALIGN_TOP_MID, 0, DEV_DOT_Y);
    lv_obj_set_style_radius(s_dev_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_dev_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_dev_dot, lv_color_hex(0x4CAF50), 0);
    lv_obj_clear_flag(s_dev_dot, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_dev_dot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_dev_dot, LV_OBJ_FLAG_HIDDEN);

    s_dev_label = lv_label_create(tile);
    lv_obj_set_width(s_dev_label, 200);
    lv_label_set_long_mode(s_dev_label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(s_dev_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_dev_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_opa(s_dev_label, LV_OPA_80, 0);
    lv_obj_align(s_dev_label, LV_ALIGN_TOP_MID, 0, DEV_LABEL_Y);
    lv_obj_clear_flag(s_dev_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_dev_label, LV_OBJ_FLAG_SCROLLABLE);

    s_arrow_left = lv_img_create(tile);
    lv_img_set_src(s_arrow_left, &img_left_arrow);
    lv_obj_align(s_arrow_left, LV_ALIGN_TOP_MID, -DEV_ARROW_DX, DEV_ARROW_Y);
    lv_obj_add_flag(s_arrow_left, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(s_arrow_left, 24);
    lv_obj_add_event_cb(s_arrow_left, arrow_prev_cb, LV_EVENT_CLICKED, NULL);

    s_arrow_right = lv_img_create(tile);
    lv_img_set_src(s_arrow_right, &img_right_arrow);
    lv_obj_align(s_arrow_right, LV_ALIGN_TOP_MID, DEV_ARROW_DX, DEV_ARROW_Y);
    lv_obj_add_flag(s_arrow_right, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(s_arrow_right, 24);
    lv_obj_add_event_cb(s_arrow_right, arrow_next_cb, LV_EVENT_CLICKED, NULL);

    /* --- 固定底部按鈕 ---------------------------------------------------- */
    s_btn = lv_obj_create(tile);
    lv_obj_remove_style_all(s_btn);
    lv_obj_set_size(s_btn, PANEL_BTN_SIZE, PANEL_BTN_SIZE);
    lv_obj_align(s_btn, LV_ALIGN_BOTTOM_MID, 0, -PANEL_BTN_MARGIN);
    lv_obj_set_style_radius(s_btn, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_btn, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(s_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_btn, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_flag(s_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_btn, bottom_btn_cb, LV_EVENT_CLICKED, NULL);

    s_btn_img = lv_img_create(s_btn);
    lv_img_set_src(s_btn_img, &mouse_mode_icon);
    lv_obj_center(s_btn_img);
    lv_obj_clear_flag(s_btn_img, LV_OBJ_FLAG_CLICKABLE);

    s_btn_label = lv_label_create(s_btn);
    lv_label_set_text(s_btn_label, "Exit");
    lv_obj_set_style_text_color(s_btn_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(s_btn_label);
    lv_obj_clear_flag(s_btn_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_btn_label, LV_OBJ_FLAG_HIDDEN);

    update_device_bar();
    update_bottom_btn();
    LOG_I("[top_panel] created");
    return s_pager;
}
