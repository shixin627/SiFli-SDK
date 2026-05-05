/**
 ******************************************************************************
 * @file   arc_scroll.c
 * @brief  右側弧形觸控滾動 — 可重用模組。實作見 arc_scroll.h 開頭的說明。
 ******************************************************************************
 */
#include "arc_scroll.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#define ARC_DEFAULT_THICKNESS 100
#define ARC_HALF_ANGLE_SIN 1.0f /* sin(55°)：弧帶 ±55° 角度範圍 */
#define ARC_TAP_THRESHOLD_SQ (12 * 12)
#define ARC_TAP_DURATION_MS 200
#define ARC_LOCK_MAX_ANCESTORS 8
#define ARC_SCROLL_OVERSHOOT_RESISTANCE 0.4f /* 邊界外 resistance：input * 0.4 = 實際移動 */

struct arc_scroll_handle
{
    arc_scroll_config_t cfg;
    lv_obj_t *overlay;
    lv_obj_t *debug_arc;          /* debug 視覺化：half-transparent band；NULL 為關閉 */
    bool active;
    bool motion_detected;
    float last_theta;
    lv_point_t press_point;
    uint32_t press_tick;
    /* parent-lock 狀態（lock_ancestors=true 才用）*/
    lv_obj_t *locked_ancestors[ARC_LOCK_MAX_ANCESTORS];
    lv_dir_t locked_dirs[ARC_LOCK_MAX_ANCESTORS];
    uint8_t locked_count;
};

static bool point_in_band(arc_scroll_handle_t *h, lv_coord_t x, lv_coord_t y)
{
    float cx = LV_HOR_RES / 2.0f;
    float cy = LV_VER_RES / 2.0f;
    float dx = (float)x - cx;
    float dy = (float)y - cy;
    float dist = sqrtf(dx * dx + dy * dy);
    float outer_r = cx;
    float inner_r = outer_r - (float)h->cfg.band_thickness;
    if (dist < inner_r || dist > outer_r) return false;
    if (dx <= 0) return false; /* 只接受右側 */
    float max_dy = dist * ARC_HALF_ANGLE_SIN;
    if (dy < -max_dy || dy > max_dy) return false;
    return true;
}

static void unlock_ancestors(arc_scroll_handle_t *h)
{
    for (uint8_t i = 0; i < h->locked_count; i++)
    {
        if (h->locked_ancestors[i] != NULL &&
            lv_obj_is_valid(h->locked_ancestors[i]))
        {
            lv_obj_set_scroll_dir(h->locked_ancestors[i], h->locked_dirs[i]);
        }
        h->locked_ancestors[i] = NULL;
    }
    h->locked_count = 0;
}

static void lock_ancestors(arc_scroll_handle_t *h)
{
    /* 防呆：先 unlock 把 stale 狀態清乾淨。如果上次 release 沒跑（obj 中途被
     * destroy 等），count 殘留 > 0，再次 lock 會把 stale 的 LV_DIR_NONE 當成
     * 原值存起來，下次 unlock 寫回 NONE 永遠卡死 */
    unlock_ancestors(h);
    if (!h->cfg.lock_ancestors) return;
    lv_obj_t *p = lv_obj_get_parent(h->overlay);
    while (p != NULL && h->locked_count < ARC_LOCK_MAX_ANCESTORS)
    {
        if (lv_obj_has_flag(p, LV_OBJ_FLAG_SCROLLABLE))
        {
            h->locked_ancestors[h->locked_count] = p;
            h->locked_dirs[h->locked_count] = lv_obj_get_scroll_dir(p);
            lv_obj_set_scroll_dir(p, LV_DIR_NONE);
            h->locked_count++;
        }
        p = lv_obj_get_parent(p);
    }
}

static void hit_test_cb(lv_event_t *e)
{
    arc_scroll_handle_t *h = lv_event_get_user_data(e);
    lv_hit_test_info_t *info = lv_event_get_hit_test_info(e);
    if (info == NULL) return;
    if (info->point == NULL)
    {
        info->res = false;
        return;
    }
    info->res = point_in_band(h, info->point->x, info->point->y);
}

static void pressed_cb(lv_event_t *e)
{
    arc_scroll_handle_t *h = lv_event_get_user_data(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (indev == NULL) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);
    h->press_point = p;
    h->press_tick = lv_tick_get();
    h->motion_detected = false;
    h->active = true;
    h->last_theta = atan2f((float)p.y - LV_VER_RES / 2.0f,
                           (float)p.x - LV_HOR_RES / 2.0f);
    lock_ancestors(h);
}

static void pressing_cb(lv_event_t *e)
{
    arc_scroll_handle_t *h = lv_event_get_user_data(e);
    if (!h->active) return;
    /* drag_cb 模式：可以沒有 list，arc 純粹當輸入用 */
    if (h->cfg.drag_cb == NULL &&
        (h->cfg.list == NULL || !lv_obj_is_valid(h->cfg.list)))
        return;
    lv_indev_t *indev = lv_indev_get_act();
    if (indev == NULL) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    if (!h->motion_detected)
    {
        int32_t mdx = p.x - h->press_point.x;
        int32_t mdy = p.y - h->press_point.y;
        if (mdx * mdx + mdy * mdy > ARC_TAP_THRESHOLD_SQ)
        {
            h->motion_detected = true;
        }
    }

    float theta = atan2f((float)p.y - LV_VER_RES / 2.0f,
                         (float)p.x - LV_HOR_RES / 2.0f);
    float delta = theta - h->last_theta;
    while (delta > M_PI) delta -= 2.0f * M_PI;
    while (delta < -M_PI) delta += 2.0f * M_PI;
    h->last_theta = theta;

    /* 把角度增量換算成 list scroll pixel：每滑 slot_angle_deg = 一個
     * slot_height_px。負號讓拖動方向跟「抓著內容往下拉」直覺一致 */
    float angle_per_slot_rad =
        (float)h->cfg.slot_angle_deg * (float)M_PI / 180.0f;
    float scroll_delta =
        -delta * (float)h->cfg.slot_height_px / angle_per_slot_rad;
    if (scroll_delta > -0.5f && scroll_delta < 0.5f) return;

    /* drag_cb 模式：把 delta 丟給 cb 自己處理，不去動 list */
    if (h->cfg.drag_cb != NULL)
    {
        h->cfg.drag_cb((lv_coord_t)scroll_delta, h->cfg.ctx);
        return;
    }

    if (h->cfg.item_count == 0) return;

    lv_obj_t *list = h->cfg.list;
    lv_coord_t cur_scroll = lv_obj_get_scroll_y(list);
    lv_coord_t target_scroll = cur_scroll + (lv_coord_t)scroll_delta;

    /* 自己算 min/max 邊界並 clamp，讓使用者不能拖過第一個/最後一個 item。
     * 不能直接靠 lv_obj_scroll_to_y 內建 clamp — LVGL 的 scroll_by_bounded 在
     * 某些 layout（CURVE_LIST 改 translate_x、hidden+visible 子物件混合、
     * floating widget 等）會算出比真實可滾範圍小的 scroll_max，使用者拖一半
     * 就被卡住。改成 (1) 用我們知道 item 幾何的公式自己 clamp，(2) 用
     * _lv_obj_scroll_by_raw 直接套位移繞過 LVGL 的錯 clamp */
    lv_coord_t min_scroll, max_scroll;
    if (h->cfg.bounds_cb != NULL)
    {
        h->cfg.bounds_cb(&min_scroll, &max_scroll, h->cfg.ctx);
    }
    else
    {
        lv_coord_t list_y1 = list->coords.y1;
        lv_coord_t pad_top = lv_obj_get_style_pad_top(list, LV_PART_MAIN);
        lv_coord_t item_h = (h->cfg.item_height_px > 0)
                                ? (lv_coord_t)h->cfg.item_height_px
                                : (lv_coord_t)h->cfg.slot_height_px;
        min_scroll = list_y1 + pad_top + item_h / 2 - LV_VER_RES / 2;
        max_scroll = min_scroll +
                     (lv_coord_t)(h->cfg.item_count - 1) *
                         (lv_coord_t)h->cfg.slot_height_px;
    }
    /* 邊界 elastic overshoot：拖到 min/max 外時，「比 over_now 還深」的那段
     * 套 resistance 慢動作，總 overshoot 上限 max_overshoot。放開後
     * released_cb → snap_cb 會把 list 拉回到合法 item。*/
    lv_coord_t max_overshoot = (lv_coord_t)(h->cfg.slot_height_px / 2);
    if (target_scroll < min_scroll)
    {
        lv_coord_t over_now = (cur_scroll < min_scroll) ? (min_scroll - cur_scroll) : 0;
        lv_coord_t over_raw = min_scroll - target_scroll;
        if (over_raw > over_now)
        {
            lv_coord_t additional = (lv_coord_t)((float)(over_raw - over_now) *
                                                  ARC_SCROLL_OVERSHOOT_RESISTANCE);
            lv_coord_t new_over = over_now + additional;
            if (new_over > max_overshoot) new_over = max_overshoot;
            target_scroll = min_scroll - new_over;
        }
    }
    else if (target_scroll > max_scroll)
    {
        lv_coord_t over_now = (cur_scroll > max_scroll) ? (cur_scroll - max_scroll) : 0;
        lv_coord_t over_raw = target_scroll - max_scroll;
        if (over_raw > over_now)
        {
            lv_coord_t additional = (lv_coord_t)((float)(over_raw - over_now) *
                                                  ARC_SCROLL_OVERSHOOT_RESISTANCE);
            lv_coord_t new_over = over_now + additional;
            if (new_over > max_overshoot) new_over = max_overshoot;
            target_scroll = max_scroll + new_over;
        }
    }
    if (target_scroll == cur_scroll) return;

    /* lv_obj_scroll_by_raw 的 dy 跟 user-facing scroll_y 方向相反（dy 是
     * scroll.y 的增量，scroll.y = -scroll_y_user），所以要傳負號 */
    lv_coord_t actual_delta = target_scroll - cur_scroll;
    _lv_obj_scroll_by_raw(list, 0, -actual_delta);
}

static void released_cb(lv_event_t *e)
{
    arc_scroll_handle_t *h = lv_event_get_user_data(e);
    if (!h->active) return;
    h->active = false;
    unlock_ancestors(h);

    uint32_t elapsed = lv_tick_elaps(h->press_tick);
    bool is_tap = !h->motion_detected && elapsed < ARC_TAP_DURATION_MS;

    if (is_tap)
    {
        if (h->cfg.tap_cb != NULL)
        {
            lv_obj_t *target = h->cfg.tap_cb(h->press_point, h->cfg.ctx);
            if (target != NULL && lv_obj_is_valid(target) &&
                !lv_obj_has_flag(target, LV_OBJ_FLAG_HIDDEN))
            {
                lv_event_send(target, LV_EVENT_CLICKED, NULL);
            }
        }
    }
    else
    {
        if (h->cfg.snap_cb != NULL)
        {
            lv_obj_t *target = h->cfg.snap_cb(h->cfg.ctx);
            if (target != NULL && lv_obj_is_valid(target))
            {
                lv_obj_scroll_to_view(target, LV_ANIM_ON);
            }
        }
    }
}

static void delete_cb(lv_event_t *e)
{
    arc_scroll_handle_t *h = lv_event_get_user_data(e);
    if (h == NULL) return;
    /* overlay obj 被刪了，把 handle 也釋放掉 */
    unlock_ancestors(h);
    lv_mem_free(h);
}

arc_scroll_handle_t *arc_scroll_create(const arc_scroll_config_t *cfg)
{
    if (cfg == NULL || cfg->parent == NULL || cfg->list == NULL) return NULL;
    if (cfg->slot_height_px == 0 || cfg->slot_angle_deg == 0) return NULL;

    arc_scroll_handle_t *h = lv_mem_alloc(sizeof(arc_scroll_handle_t));
    if (h == NULL) return NULL;
    memset(h, 0, sizeof(arc_scroll_handle_t));
    h->cfg = *cfg;
    if (h->cfg.band_thickness == 0)
        h->cfg.band_thickness = ARC_DEFAULT_THICKNESS;

    lv_obj_t *overlay = lv_obj_create(cfg->parent);
    if (overlay == NULL)
    {
        lv_mem_free(h);
        return NULL;
    }
    h->overlay = overlay;

    lv_obj_set_size(overlay, LV_HOR_RES, LV_VER_RES);
    lv_obj_align(overlay, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(overlay, 0, 0);
    lv_obj_set_style_pad_all(overlay, 0, 0);
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_CLICKABLE);
    /* 沒這個 flag → lv_obj_hit_test 直接用 bbox 判斷，不會發 LV_EVENT_HIT_TEST，
     * 整個全螢幕 overlay 會吃掉每一個 press */
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_ADV_HITTEST);
    /* 清掉 SCROLL_CHAIN，LVGL 的 find_scroll_obj 走到 overlay 就 break，
     * 不會再往上找到 tileview / 其他 scrollable 祖先去做 scroll，
     * 比 runtime 改 ancestor scroll_dir 穩（後者會被 tileview 自己的
     * SCROLL_END handler 重置，造成弧形拖到一半變成 tileview 翻頁）*/
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_SCROLL_CHAIN_VER);

    lv_obj_add_event_cb(overlay, hit_test_cb, LV_EVENT_HIT_TEST, h);
    lv_obj_add_event_cb(overlay, pressed_cb, LV_EVENT_PRESSED, h);
    lv_obj_add_event_cb(overlay, pressing_cb, LV_EVENT_PRESSING, h);
    lv_obj_add_event_cb(overlay, released_cb, LV_EVENT_RELEASED, h);
    lv_obj_add_event_cb(overlay, released_cb, LV_EVENT_PRESS_LOST, h);
    lv_obj_add_event_cb(overlay, delete_cb, LV_EVENT_DELETE, h);

    return h;
}

void arc_scroll_set_item_count(arc_scroll_handle_t *h, uint16_t count)
{
    if (h != NULL) h->cfg.item_count = count;
}

void arc_scroll_bring_to_front(arc_scroll_handle_t *h)
{
    if (h == NULL) return;
    if (h->overlay != NULL && lv_obj_is_valid(h->overlay))
    {
        lv_obj_move_foreground(h->overlay);
    }
}

void arc_scroll_destroy(arc_scroll_handle_t *h)
{
    if (h == NULL) return;
    if (h->overlay != NULL && lv_obj_is_valid(h->overlay))
    {
        /* delete_cb 會接著 free h */
        lv_obj_del(h->overlay);
    }
    else
    {
        unlock_ancestors(h);
        lv_mem_free(h);
    }
}

void arc_scroll_set_debug_visible(arc_scroll_handle_t *h, bool visible)
{
    if (h == NULL) return;
    if (visible)
    {
        if (h->debug_arc != NULL && lv_obj_is_valid(h->debug_arc)) return;
        if (h->overlay == NULL || !lv_obj_is_valid(h->overlay)) return;

        /* 用 lv_arc 畫一個半透明的圓環 segment，反映實際 hit_test 的幾何：
         *   - 圓心：螢幕中央
         *   - 半徑帶：[outer_r - thickness, outer_r]，outer_r = LV_HOR_RES/2
         *   - 角度：±55°（跟 ARC_HALF_ANGLE_SIN ≈ sin(55°) 對齊）
         *
         * lv_arc 的 size = 整圓直徑、arc_width = band 厚度。
         * arc 角度系：0° 在 3 點鐘、順時針增加。所以「右側 ±55°」=
         * start=305°（11 點上方），end=55°（5 點上方），LVGL 會跨 0° wrap。 */
        lv_obj_t *arc = lv_arc_create(h->overlay);
        lv_obj_remove_style_all(arc); /* 把 theme 預設 knob/border 等清掉 */
        lv_obj_set_size(arc, LV_HOR_RES, LV_VER_RES);
        lv_obj_align(arc, LV_ALIGN_CENTER, 0, 0);
        lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(arc, LV_OBJ_FLAG_SCROLLABLE);

        /* MAIN = bg arc 用來畫整個 band */
        lv_obj_set_style_arc_color(arc, lv_color_hex(0x00FF00), LV_PART_MAIN);
        lv_obj_set_style_arc_opa(arc, LV_OPA_30, LV_PART_MAIN);
        lv_obj_set_style_arc_width(arc, h->cfg.band_thickness, LV_PART_MAIN);
        /* 從 ARC_HALF_ANGLE_SIN 反推實際半角度，確保 debug 視覺跟 hit_test 對齊 */
        float half_deg_f = asinf(ARC_HALF_ANGLE_SIN) * 180.0f / (float)M_PI;
        int half_deg = (int)(half_deg_f + 0.5f);
        int start_deg = 360 - half_deg; /* 例如 ±55° → 305° */
        int end_deg = half_deg;          /* 例如 ±55° → 55° */
        lv_arc_set_bg_angles(arc, (uint16_t)start_deg, (uint16_t)end_deg);

        /* INDICATOR / KNOB 都不顯示 */
        lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
        lv_obj_set_style_pad_all(arc, 0, LV_PART_KNOB);

        h->debug_arc = arc;
    }
    else
    {
        if (h->debug_arc != NULL && lv_obj_is_valid(h->debug_arc))
        {
            lv_obj_del(h->debug_arc);
        }
        h->debug_arc = NULL;
    }
}
