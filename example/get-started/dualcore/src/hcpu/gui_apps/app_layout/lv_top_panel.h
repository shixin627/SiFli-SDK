/**
 ******************************************************************************
 * @file   lv_top_panel.h
 * @brief  錶盤頂部下拉面板（通知列表 / 控制中心 / 各設備媒體中心的共同容器）
 *
 *  2026-08-06 重構（founder）：原本錶盤上方 tile 只有通知列表、控制中心掛在
 *  錶盤下方 tile、媒體中心藏在滑鼠 app 裡。現在三者合成同一個下拉面板：
 *
 *      [控制中心] ← [通知列表] → [設備1媒體] → [設備2媒體] → …
 *                     ↑ 水平捲動；上方設備名列 + 下方按鈕**不動**
 *
 *  面板可以蓋在錶盤上，也可以蓋在滑鼠模式上（下方按鈕負責切換）：
 *    - 未進滑鼠模式：按鈕 = 滑鼠圖 → 收起面板 + 底下換成滑鼠模式
 *    - 已在滑鼠模式：按鈕 = Exit   → 退出滑鼠模式 + 收起面板 + 回錶盤
 *
 *  滑鼠模式**不是**用 gui_app 開 APP_ID_MOUSE，而是把 hid_mouse 的完整 UI
 *  build 在面板自己的全螢幕圖層上（hid_mouse_build_ui），所以飛鼠 / 鍵盤 /
 *  手寫 / skaibar 全都在，只有 app 自帶的那層媒體中心被關掉（面板取代它）。
 ******************************************************************************
 */
#ifndef LV_TOP_PANEL_H
#define LV_TOP_PANEL_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 建立面板。
 * @param tile         錶盤主 tileview 的上方 tile (1,0) — 面板內容掛這裡。
 * @param layer_parent 滑鼠模式圖層的父物件（= 主狀態列的父，讓圖層可以壓在
 *                     錶面之上、面板之下）。
 */
lv_obj_t *lv_top_panel_create(lv_obj_t *tile, lv_obj_t *layer_parent);

/** 主 tileview settle 到 / 離開面板頁時通知（刷新設備列、重建媒體頁）。 */
void lv_top_panel_set_open(bool opened);

/** 目前是否在滑鼠模式（底下不是錶盤）。錶盤側的手勢 gate 用。 */
bool lv_top_panel_mouse_mode(void);

/** 設備清單有變（registry 更新）時重建媒體頁 + 刷新頂部設備名。 */
void lv_top_panel_refresh_devices(void);

/** 開在「當前 active 設備」的媒體頁。沒有已同步設備時退回通知列表。呼叫端負責把面板
    本身叫出來。 */
void lv_top_panel_open_media(void);

/** 開在第 [device_index] 台的媒體頁，並把控制目標換成那台 —— 錶盤右側第 i 欄的
    session 列表往下拉時用，欄與設備索引是同一份 registry 順序。索引越界時退回目前
    控制中的設備。 */
void lv_top_panel_open_media_for(int device_index);

/** 通知列表用：手指壓在「中央那張卡」上時關掉面板水平換頁，放開再開。 */
void lv_top_panel_set_hor_enabled(bool enabled);

/** 面板下拉進度 → 滑鼠模式下的黑色半透明底濃度（0..204）。非滑鼠模式為 no-op。*/
void lv_top_panel_set_backdrop_opa(uint8_t opa);

/** 電量有更新時刷新控制中心那頁頂部的電量顯示。 */
void lv_top_panel_refresh_battery(void);

/** 狀態列 deinit 時一併拆掉滑鼠圖層（面板本體隨 tile 一起被刪）。 */
void lv_top_panel_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* LV_TOP_PANEL_H */
