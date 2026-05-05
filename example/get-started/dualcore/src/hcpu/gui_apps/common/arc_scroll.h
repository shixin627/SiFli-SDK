/**
 ******************************************************************************
 * @file   arc_scroll.h
 * @brief  右側弧形觸控滾動的可重用模組。
 *
 * 在指定的 parent 上加一個全螢幕透明 overlay（自帶 ADV_HITTEST），只接受
 * 螢幕右側弧帶內的 press。
 *   - 在弧帶內拖動 → 把 cfg.list 滾動對應的 pixel（每滑過 slot_angle_deg
 *     角度 = 一個 slot_height_px 的 scroll），放開後 snap 到 snap_cb 回傳的 obj
 *   - 在弧帶內 tap（無明顯位移 + 200ms 內放開）→ 呼叫 tap_cb 把 press 點交給
 *     使用者邏輯，由使用者決定要把 LV_EVENT_CLICKED 轉發給哪個 obj
 *
 * 同一份邏輯可以掛在多個列表上，每個 instance 獨立持有自己的狀態。
 ******************************************************************************
 */
#ifndef APP_COMMON_ARC_SCROLL_H
#define APP_COMMON_ARC_SCROLL_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct arc_scroll_handle arc_scroll_handle_t;

/* tap_cb：tap（無明顯移動 + 短時間放開）時被叫，使用者根據 press 點決定
 * 要把 click 派給誰；回傳 NULL 代表不派發 */
typedef lv_obj_t *(*arc_scroll_tap_cb_t)(lv_point_t press_point, void *ctx);

/* snap_cb：drag 放開時被叫，回傳要 scroll 到中央的目標 obj；NULL 代表不 snap */
typedef lv_obj_t *(*arc_scroll_snap_cb_t)(void *ctx);

typedef struct
{
    lv_obj_t *parent;               /* overlay 要掛在哪個父物件下 */
    lv_obj_t *list;                 /* arc-drag 要捲動的 scrollable 容器 */
    uint16_t slot_height_px;        /* 兩個相鄰 item 的「位置間距」px（= item_height + spacing），
                                     * 跟 slot_angle_deg 一起決定「滑幾度等於滾一個 item」*/
    uint16_t item_height_px;        /* 單一 item 本身的 px 高度，scroll 邊界 clamp 會用到。
                                     * spacing ≥ 0（item 不重疊）可填 0 → 自動取 slot_height_px；
                                     * spacing < 0（item 重疊，例如 instruction_list）要明確填 */
    uint16_t slot_angle_deg;        /* 手指滑過幾度等於一個 slot */
    uint16_t item_count;            /* 目前 item 數。決定 scroll max 邊界（拖到最後一個就不能再
                                     * 往下）。動態變動時呼叫 arc_scroll_set_item_count 更新 */
    uint16_t band_thickness;        /* 弧帶從螢幕邊往內的 px 厚度；0 → 預設 150 */
    bool lock_ancestors;            /* drag 期間鎖住外層 scrollable 祖先（防 tileview 誤搶）*/
    arc_scroll_tap_cb_t tap_cb;     /* 可為 NULL */
    arc_scroll_snap_cb_t snap_cb;   /* 可為 NULL */
    void *ctx;                      /* 給 cb 用的使用者 context */
} arc_scroll_config_t;

/* 建立一個 arc_scroll instance。handle 會在底下的 overlay obj 被 lv_obj_del
 * 時自動釋放（例如 parent 被 destroy 時），通常不需要手動 destroy。 */
arc_scroll_handle_t *arc_scroll_create(const arc_scroll_config_t *cfg);

/* item 數變動時更新（影響 scroll 上下界 clamp） */
void arc_scroll_set_item_count(arc_scroll_handle_t *handle, uint16_t count);

/* 提早銷毀 — 一般情形 overlay 跟著 parent 一起死，不用呼叫。 */
void arc_scroll_destroy(arc_scroll_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMMON_ARC_SCROLL_H */
