/**
 ******************************************************************************
 * @file   hid_mouse.h
 * @brief  Mouse (trackpad / keyboard / media) component API.
 *
 *  T1 part 1 (host decouple): expose the mouse UI as a hostable component so
 *  device_pager can mount it as the per-device control layer WITHOUT going
 *  through gui_app_run(). The existing APP_ID_MOUSE gui_app path is unchanged
 *  — its msg_handler still calls hid_mouse_create(lv_scr_act()) on ONSTART and
 *  hid_mouse_destroy() on ONSTOP, so behavior is byte-identical for the
 *  standalone mouse app.
 ******************************************************************************
 */
#ifndef HID_MOUSE_H
#define HID_MOUSE_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Build the mouse UI (trackpad/keyboard/media + bottom bar + arc
 *        scroll) under @p host and activate it as the control surface.
 * @param host Parent LVGL object to build under (e.g. lv_scr_act() for the
 *             standalone app, or a device_pager tile for the hosted case).
 */
void hid_mouse_create(lv_obj_t *host);

/**
 * @brief Tear down the mouse UI and deactivate the control surface
 *        (mirrors the gui_app ONSTOP cleanup).
 */
void hid_mouse_destroy(void);

/**
 * @brief Split of hid_mouse_create for hosts that want the UI built ONCE and
 *        persisted (device_pager: trackpad visible mid-swipe) but the global
 *        "mouse mode" side effects toggled per page entry/leave:
 *          hid_mouse_build_ui  — pure UI (no global state); call once per host.
 *          hid_mouse_enter_mode — mouse-mode flag + status-bar gesture zones +
 *                                 device-change-bar hit-test + gesture detect.
 *          hid_mouse_exit_mode  — reverse of enter_mode, WITHOUT tearing the UI
 *                                 down (the persisted UI survives for next entry).
 *        hid_mouse_create == build_ui + enter_mode and hid_mouse_destroy still does
 *        exit_mode + full UI teardown, so the standalone APP_ID_MOUSE path is
 *        unchanged.
 */
void hid_mouse_build_ui(lv_obj_t *host);
void hid_mouse_enter_mode(void);
void hid_mouse_exit_mode(void);

/**
 * @brief The host object the (singleton) mouse UI is currently built on, or NULL
 *        if none. device_pager persists its hosted UI and checks this to detect the
 *        standalone APP_ID_MOUSE app having rebuilt the shared globals elsewhere
 *        (host != its tile) — in which case it cleans its tile and rebuilds.
 */
lv_obj_t *hid_mouse_ui_host(void);

/**
 * @brief Fade the trackpad scroll wheel (the gray tick nodes) in from the black
 *        backdrop, by ramping their COLOR (not opacity). For hosts (device_pager)
 *        that build the mouse at settle and want to soften the wheel's
 *        appearance. Call right after hid_mouse_create(). No-op safe if the
 *        wheel isn't built. Hardware-safe (no full-object layer opacity).
 */
void hid_mouse_fade_in_scroll_wheel(void);

/**
 * @brief Register a "return to host" callback (T4 (b)). When set, the mouse is
 *        in hosted mode: the bottom-bar up gesture invokes @p cb (device_pager
 *        slides the instruction layer back) instead of firing a multitask
 *        command. Pass NULL to clear (back to standalone behavior).
 *        The callback is invoked on the LVGL thread (input release handler).
 */
void hid_mouse_set_host_back_cb(void (*cb)(void));

/**
 * @brief Register a "host pull" callback for the hosted (device_pager) case.
 *        When set, dragging UP on the bottom bar is delegated to the host so it
 *        can finger-follow pull its panel back into view, instead of showing the
 *        mouse's own multitask hint. Called on the LVGL thread:
 *          phase 0 = dragging   (up_px = pixels dragged up so far)
 *          phase 1 = released   (host decides commit vs cancel from up_px)
 *        Takes precedence over the back cb. Pass NULL to clear.
 */
void hid_mouse_set_host_pull_cb(void (*cb)(int up_px, int released));

/**
 * @brief Mark the trackpad as hosted by device_pager (true) or standalone
 *        (false). While hosted, the pager owns the top + bottom screen edges
 *        (its device-name strip + bottom input bar), so the mouse page's own
 *        media-center pull-down is suppressed. Set from device_pager_set_active.
 */
void hid_mouse_set_hosted(bool hosted);

/**
 * @brief 由「錶面立起正對臉」姿態偵測跨層觸發：帶出單設備 skaibar 列表
 *        (等同使用者點擊底部 bar)。僅在滑鼠 app 前景由 bloc_motion_tracking
 *        的 set_gravity_position(GRAVITY_POSITION_VERTICAL) 呼叫。
 *        內部發 LVGL msg，實際開 skaibar 在 LVGL thread (open_skaibar_from_pose)。
 */
void hid_mouse_trigger_skaibar_from_pose(void);

/* === 錶盤頂部面板（lv_top_panel.c）用的介面 ==================================
   面板把「媒體中心」與「控制中設備」搬到自己身上，但真相仍在本模組：選台 =
   registry index + commu_send_active_device + app_route。面板只是另一個 UI。 */

/** 目前 registry 內的設備數（0..MAX_SYNCED_DEVICES）。 */
int hid_mouse_device_count(void);
/** 目前控制中的設備 index，未選/不在清單回 -1。 */
int hid_mouse_active_device_index(void);
/** 把第 idx 台設為控制目標（送 active_device + 開 app_route）。 */
void hid_mouse_set_active_device_index(int idx);
/** 切到相鄰設備（dir=-1/+1，循環）。 */
void hid_mouse_switch_active_device(int dir);
/** 第 idx 台的顯示名稱（RAM 內 device_name），越界回 NULL。 */
const char *hid_mouse_device_name(int idx);
/** 第 idx 台是否在線。 */
bool hid_mouse_device_online(int idx);
/** 沒有有效控制目標時挑一台預設（主要→第一個在線→第一台）；registry 空則不動。 */
void hid_mouse_ensure_active_device(void);
/** 把控制目標退回「當前連線的手機」（清 active + 關 app_route，回 BLE HID 直連）。 */
void hid_mouse_clear_active_device(void);

/**
 * @brief 建一頁媒體控制（曲名 + 上/播/下 + 音量）到 @p parent，回傳該頁物件。
 *        每頁自帶 widget 指標；用 hid_mouse_media_page_bind 指定「目前顯示中」
 *        的那一頁，0x19 / 0x46 曲名路由就會寫到它。設備名/箭頭不含在內。
 */
lv_obj_t *hid_mouse_media_page_create(lv_obj_t *parent);
/** 把曲名/播放圖示路由綁到這一頁（切頁時呼叫）。 */
void hid_mouse_media_page_bind(lv_obj_t *page);
/** 把這一頁的曲名還原成 placeholder（換設備、等新 now-playing 時用）。 */
void hid_mouse_media_page_reset_title(lv_obj_t *page);

/**
 * @brief 覆寫「頂部往下拉」的去處。裝了之後滑鼠 UI 不再建自己的媒體 tileview，
 *        下拉改呼叫 @p cb（面板 reveal）。**必須在 hid_mouse_build_ui 之前設定**。
 *        NULL = 回到自有媒體下拉（獨立 APP_ID_MOUSE 路徑）。
 */
void hid_mouse_set_pulldown_cb(void (*cb)(void));

#ifdef __cplusplus
}
#endif

#endif /* HID_MOUSE_H */
