/**
 * @file lv_top_panel.c
 * @brief 錶盤頂部下拉面板 — ADR-0020 之後只剩**通知列表**。
 *        控制中心 + App List 在右頁 (2,1)（R47 2026-09-05 從下方 (1,2) 搬去）。
 *        滑鼠模式圖層仍由本檔管理：**錶盤往上拉**到下方停車位 (1,2) settle 時由
 *        lv_top_panel_mouse_enter() 進入（控制目標 = 上次控制的那台）；
 *        退出 = 滑鼠頁頂部下拉，settle 回錶盤（clock 的 settle handler 做）。
 *        每設備一欄的媒體中心已退役 —— 正在播放的媒體改成掛在滑鼠頁頂部的
 *        header（hid_mouse 自建），點它開那一台的媒體控制頁。
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

extern lv_obj_t *lv_message_list_layout_create(lv_obj_t *parent);
extern void app_clock_status_bar_return_home(void);
extern void clock_main_panel_reveal(void);
extern void clock_main_status_bar_to_front(void);
extern void display_status_bar_area(uint32_t idx, bool display);

static lv_obj_t *s_tile = NULL; /* (1,0) — 面板唯一的家（ADR-0020 起不再換欄停放） */

static lv_obj_t *s_mouse_layer = NULL;
/* 滑鼠模式下拉面板時的黑色半透明底（蓋在觸控板上、面板之下）。錶盤那邊由
   gaus_dial_bg 提供，但它在滑鼠圖層底下看不到，所以另備一份掛在圖層上。 */
static lv_obj_t *s_backdrop = NULL;
static lv_obj_t *s_layer_parent = NULL;
static bool s_mouse_mode = false;
/* 進滑鼠模式時圖層先建好但不顯示，等收合動畫跑完才 enter_mode——
   hid_mouse_enter_mode() 開 IMU 資料流會餓死 GUI thread，動畫會跳格
   （founder 2026-08-06）。 */
static bool s_mouse_reveal_pending = false;
static bool s_opened = false;

/* -------------------------------------------------------------------------- */
/* 滑鼠模式圖層                                                                */
/* -------------------------------------------------------------------------- */

/* 滑鼠模式時「頂部往下拉」= 把錶盤跟手拉下來、settle 才退出滑鼠
   (R47:落點從「該欄媒體頁」改成錶盤)。reveal 本身不拆任何東西,直接同步呼叫安全;
   真正的退出由 clock 的 settle handler 做(那時 press 已交棒給 tileview)。 */
static void panel_reveal_cb(void)
{
    extern void clock_main_mouse_pulldown_reveal(void);
    clock_main_mouse_pulldown_reveal();
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

/* 進滑鼠模式拆成兩段（founder 2026-08-06 的最終形）：
     prepare  — 當下建 UI + 亮圖層。
     activate — 動畫結束才做：hid_mouse_enter_mode() 開 IMU / 飛鼠資料流。 */
static void mouse_layer_prepare(void)
{
    mouse_layer_ensure();
    if (s_mouse_layer == NULL)
        return;
    /* cb 必須在 build_ui 之前裝好：hid_mouse 靠它決定「不要建自己的媒體
       tileview」+「下拉去面板」。 */
    hid_mouse_set_pulldown_cb(panel_reveal_cb);
    if (hid_mouse_ui_host() != s_mouse_layer)
    {
        lv_obj_clean(s_mouse_layer);
        hid_mouse_build_ui(s_mouse_layer);
    }
    /* backdrop 建在 build_ui 之後 = 圖層最後一個子物件 = 壓在觸控板 UI 之上；
       面板本身在 status_bar_bg_main 裡、又在圖層之上，夾在中間。 */
    s_backdrop = lv_obj_create(s_mouse_layer);
    lv_obj_remove_style_all(s_backdrop);
    lv_obj_set_size(s_backdrop, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_pos(s_backdrop, 0, 0);
    lv_obj_set_style_bg_color(s_backdrop, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_backdrop, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(s_backdrop, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_backdrop, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_backdrop, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(s_mouse_layer, 0, 0); /* 上一輪下拉可能把它留在位移狀態 */
    lv_obj_move_foreground(s_mouse_layer);
    clock_main_status_bar_to_front(); /* 面板要壓在滑鼠圖層之上 */
    lv_obj_clear_flag(s_mouse_layer, LV_OBJ_FLAG_HIDDEN);
    LOG_I("[top_panel] mouse layer prepared");
}

static void mouse_layer_activate(void)
{
    if (s_mouse_layer == NULL || !lv_obj_is_valid(s_mouse_layer))
        return;
    hid_mouse_enter_mode();       /* 這裡面已把上/下/左緣 zone 收掉 */
    hid_mouse_set_hosted(true);   /* 面板擁有上下緣 */
    display_status_bar_area(3, false); /* 右緣 zone enter_mode 沒收，補收 */
    /* 控制目標由呼叫端在 settle 時選好（hid_mouse_ensure_active_device:上次控制
       的那台 → 預設挑一台）。registry 真的空才 active < 0 = 直控手機、不開 route。 */
    ble_hid_mouse_set_app_route(hid_mouse_active_device_index() >= 0);
    LOG_I("[top_panel] mouse mode ON");
}

static void mouse_layer_set(bool on)
{
    if (on == s_mouse_mode)
        return;

    if (on)
    {
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
            lv_obj_set_pos(s_mouse_layer, 0, 0); /* 位移歸零,下次進來才不是歪的 */
            lv_obj_add_flag(s_mouse_layer, LV_OBJ_FLAG_HIDDEN);
        }
        LOG_I("[top_panel] mouse mode OFF");
    }
}

bool lv_top_panel_mouse_mode(void) { return s_mouse_mode; }

/* heap 輪(2026-08-16):聊天室 overlay 不透明蓋全螢幕時,滑鼠圖層(全螢幕黑底+
   觸控板 UI)整層看不到卻照樣進 EPIC 繪製清單吃合成緩衝(同 R34 gaus_dial_bg 的
   教訓)。聊天室開/關時把圖層整個 hide/show —— mouse_layer_prepare 本來就會
   clear HIDDEN,狀態不會卡死。非滑鼠模式 no-op。 */
void lv_top_panel_mouse_layer_set_covered(bool covered)
{
    if (!s_mouse_mode || s_mouse_layer == NULL || !lv_obj_is_valid(s_mouse_layer))
        return;
    if (covered)
        lv_obj_add_flag(s_mouse_layer, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_clear_flag(s_mouse_layer, LV_OBJ_FLAG_HIDDEN);
    LOG_I("[top_panel] mouse layer %s (chat overlay)", covered ? "covered" : "restored");
}

/* enter_mode 很重（IMU 資料流），settle 那一幀先只把圖層亮出來，下一輪
   lv_timer_handler 再開，settle 的重繪才不會被它卡住。 */
static void mouse_activate_async_cb(void *unused)
{
    (void)unused;
    if (s_mouse_mode && s_mouse_reveal_pending)
    {
        s_mouse_reveal_pending = false;
        mouse_layer_activate();
    }
}

/** R47：錶盤往上拉、settle 在下方停車位 (1,2) 時由 clock 呼叫。
    控制目標（哪一台）由呼叫端在 settle 時就選好。 */
void lv_top_panel_mouse_enter(void)
{
    if (s_mouse_mode)
        return;
    mouse_layer_set(true);
    lv_async_call(mouse_activate_async_cb, NULL);
}

/** 退出滑鼠模式（拆圖層 + 還原邊緣 zone）。落回哪一頁由呼叫端決定。 */
void lv_top_panel_mouse_exit(void)
{
    mouse_layer_set(false);
}

/* -------------------------------------------------------------------------- */
/* 對外                                                                        */
/* -------------------------------------------------------------------------- */

/* R49(founder 2026-09-05:「從上面往下拉沒有看到滑鼠頁面跟著我的手指往下拉」):
   下拉退出的跟手動畫以前只有**錶盤那一格**在動 —— 滑鼠圖層是另一層,靜止不動,
   所以看起來是錶盤蓋上來,不是滑鼠頁被拉走。這裡讓圖層整層跟著位移:錶盤 tileview
   的 scroll 位置 → 圖層的 y。非滑鼠模式 no-op。 */
void lv_top_panel_mouse_layer_set_offset_y(lv_coord_t y)
{
    if (!s_mouse_mode || s_mouse_layer == NULL || !lv_obj_is_valid(s_mouse_layer))
        return;
    if (y < 0) y = 0;
    if (y > LV_VER_RES_MAX) y = LV_VER_RES_MAX;
    if (lv_obj_get_y(s_mouse_layer) == y)
        return;
    lv_obj_set_y(s_mouse_layer, y);
}

/* 面板下拉進度 → 滑鼠模式黑底濃度（錶盤路徑由 gaus_dial_bg 負責）。 */
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

/* ADR-0020 之後面板內沒有水平 pager 了。通知列表的中央卡拖曳仍會呼叫這裡
   （它不知道 pager 沒了），保留成 no-op 讓呼叫端零改動。 */
void lv_top_panel_set_hor_enabled(bool enabled)
{
    (void)enabled;
}

/* 面板不再有設備列 / 媒體頁；R47 之後右側也只有固定一格的控制中心 + App List，
   設備數不再影響任何一格。保留這個入口（device_pager.c 的 E7 同步路徑呼叫它），
   轉發到的 clock_main_media_cols_refresh() 現在是 no-op。 */
void lv_top_panel_refresh_devices(void)
{
    extern void clock_main_media_cols_refresh(void);
    clock_main_media_cols_refresh();
}

/* 面板上已無電量顯示（控制中心搬走了）；保留符號給既有呼叫端。 */
void lv_top_panel_refresh_battery(void)
{
}

void lv_top_panel_set_open(bool opened)
{
    s_opened = opened;
    if (!opened)
    {
        /* 面板完全收掉（動畫結束的 settle）——滑鼠模式的 enter 若還掛著，
           現在補做。 */
        if (s_mouse_reveal_pending)
        {
            s_mouse_reveal_pending = false;
            mouse_layer_activate();
        }
        return;
    }
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
    s_tile = NULL;
    s_opened = false;
}

lv_obj_t *lv_top_panel_create(lv_obj_t *tile, lv_obj_t *layer_parent)
{
    s_layer_parent = layer_parent;
    s_tile = tile;

    /* 通知列表直接鋪滿 tile —— 沒有 pager、沒有設備列。滑鼠模式的退出走
       「滑鼠頁頂部下拉」(clock_main_mouse_pulldown_reveal) 落回錶盤,面板沒有
       Exit 鈕。 */
    lv_message_list_layout_create(tile);

    LOG_I("[top_panel] created (notification-only, ADR-0020)");
    return tile;
}
