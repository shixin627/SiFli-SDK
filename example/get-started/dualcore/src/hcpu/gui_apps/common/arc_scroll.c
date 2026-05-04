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

#define ARC_DEFAULT_THICKNESS 150
#define ARC_HALF_ANGLE_SIN 0.819f /* sin(55°)：弧帶 ±55° 角度範圍 */
#define ARC_TAP_THRESHOLD_SQ (12 * 12)
#define ARC_TAP_DURATION_MS 200
#define ARC_LOCK_MAX_ANCESTORS 8

struct arc_scroll_handle
{
    arc_scroll_config_t cfg;
    lv_obj_t *overlay;
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
    if (!h->active || h->cfg.list == NULL || !lv_obj_is_valid(h->cfg.list))
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

    if (h->cfg.item_count == 0) return;

    /* 用 _lv_obj_scroll_by_raw 直接動 scroll，繞過 lv_obj_scroll_by_bounded 的
     * 邊界檢查。LVGL 那層 clamp 在某些 layout（含 media_widget 之類的 floating
     * 子物件、ENABLE_CURVE_LIST 修改 children 的 translate_x、隱藏 child 跟可見
     * child 混在一起）會算出比實際可滾範圍小的 scroll_max，使用者拖到一半就被
     * 卡住。raw 版本只更新 scroll.y + 移動 children + 發 SCROLL 事件，不做
     * bounds check；snap_cb 在 RELEASE 時會把 list 拉回最近 item，即使中間飄過
     * 頭最後也會回到合理位置 */
    lv_obj_t *list = h->cfg.list;
    lv_coord_t int_scroll_delta = (lv_coord_t)scroll_delta;
    if (int_scroll_delta == 0) return;
    /* lv_obj_scroll_by_raw 的 dy 跟 user-facing scroll_y 方向相反（dy 是
     * scroll.y 的增量，scroll.y = -scroll_y_user），所以要傳負號 */
    _lv_obj_scroll_by_raw(list, 0, -int_scroll_delta);
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
