/**
 ******************************************************************************
 * @file   lv_top_panel.h
 * @brief  錶盤頂部下拉面板 — ADR-0020 之後**只剩通知列表**。
 *
 *  歷史：2026-08-06 曾把控制中心 / 通知 / 各設備媒體中心合成一個水平 pager
 *  面板；ADR-0020（2026-08-11 founder 拍板）把媒體中心搬到錶盤右側欄
 *  （一台一欄）、控制中心搬到下方上拉頁，面板於是回歸單純的通知列表。
 *
 *  滑鼠模式圖層仍由本檔管理：
 *    - 進入 = 右側媒體欄往下拉 settle 在滑鼠停車位 → lv_top_panel_mouse_enter()
 *    - 退出 = 滑鼠模式中下拉面板 → 底部 Exit 鈕（只在滑鼠模式顯示）
 *  滑鼠模式不是 gui_app：hid_mouse 的完整 UI build 在全螢幕圖層上
 *  （hid_mouse_build_ui），飛鼠 / 鍵盤 / 手寫 / skaibar 全在。
 ******************************************************************************
 */
#ifndef LV_TOP_PANEL_H
#define LV_TOP_PANEL_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 建立面板（通知列表 + 滑鼠模式限定的 Exit 鈕）。
 * @param tile         錶盤主 tileview 的上方 tile (1,0)。
 * @param layer_parent 滑鼠模式圖層的父物件。
 */
lv_obj_t *lv_top_panel_create(lv_obj_t *tile, lv_obj_t *layer_parent);

/** 主 tileview settle 到 / 離開面板頁時通知。 */
void lv_top_panel_set_open(bool opened);

/** 目前是否在滑鼠模式（底下不是錶盤）。錶盤側的手勢 gate 用。 */
bool lv_top_panel_mouse_mode(void);

/** 進入滑鼠模式（ADR-0020 R2：媒體欄**往上拉** settle 在下方停車位時由 clock
    呼叫）。控制目標（該台設備 / 手機）由呼叫端先選好。 */
void lv_top_panel_mouse_enter(void);

/** 退出滑鼠模式（拆圖層 + 還原邊緣 zone）。落回哪一頁由呼叫端決定 ——
    滑鼠頁頂部下拉走 clock_main_mouse_pulldown_reveal()。 */
void lv_top_panel_mouse_exit(void);

/** 設備清單有變（registry 更新）→ 轉發給錶盤重算右側媒體欄。 */
void lv_top_panel_refresh_devices(void);

/** 舊 pager 時代的入口，現為 no-op（通知列表中央卡拖曳仍呼叫）。 */
void lv_top_panel_set_hor_enabled(bool enabled);

/** 面板下拉進度 → 滑鼠模式下的黑色半透明底濃度（0..204）。非滑鼠模式 no-op。*/
void lv_top_panel_set_backdrop_opa(uint8_t opa);

/** 面板上已無電量顯示；保留符號給既有呼叫端（no-op）。 */
void lv_top_panel_refresh_battery(void);

/** 狀態列 deinit 時一併拆掉滑鼠圖層（面板本體隨 tile 一起被刪）。 */
void lv_top_panel_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* LV_TOP_PANEL_H */
