/**
 ******************************************************************************
 * @file   device_pager.c
 * @brief  Right-side per-device control page — content builder for the
 *         watch-face tileview's RIGHT tile.
 *
 *  Mirrors the WATCH-FACE reveal mechanism:
 *    - mouse_base  : the real hid_mouse, the always-present BASE (like the clock).
 *    - overlay_tv  : a vertical lv_tileview, HIDDEN by default, with two tiles —
 *        (0,0) HOME : transparent (you see the mouse base through it).
 *        (0,1) LIST : the device's instruction list_items (+ a 3-tile horizontal
 *                     recycler for device paging + a bottom-centre mic/skaibar).
 *    - bar         : a bottom handle shown when the overlay is hidden (on the
 *                    mouse), like the watch-face edge zone.
 *
 *  Flow (native finger-follow throughout; the mouse base never scrolls):
 *    - Enter the device page → overlay shown on the LIST tile (list covers mouse).
 *    - Pull the list DOWN → overlay scrolls to the transparent HOME tile, the
 *      list slides down and you see the mouse through the transparent page.
 *    - Release on HOME → overlay HIDES → the mouse base + the bottom BAR show.
 *    - Touch the BAR → overlay re-shows on HOME → drag UP → the LIST slides back.
 *
 *  The mouse base is hosted on device_pager_set_active() and drill-down-targeted
 *  at the current device. list_items are per-device fake content; real per-device
 *  data + skaibar voice over BLE arrive later.
 ******************************************************************************
 */
#include <rtthread.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "lvgl.h"
#include "ble_device_manager.h"
#include "ble_hid.h"                   /* ble_hid_set_conn_idx (drill-down target) */
#include "ui_helper.h"                 /* get_system_font_size */
#include "lv_ext_resource_manager.h"   /* LV_EXT_FONT_GET font idiom */
#include "hid_mouse.h"                 /* mouse component, hosted as the base */

#define DBG_TAG "device.pager"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

LV_IMG_DECLARE(icon_mic); /* shared mic/voice icon, same as instruction_list */
LV_IMG_DECLARE(message_widget_bg); /* skaibar input pill frame (same as left list) */
LV_IMG_DECLARE(img_flashlight); /* placeholder per-item icon (same look as left list) */

#define TILE_LEFT      0
#define TILE_CENTER    1
#define TILE_RIGHT     2
#define MAX_TILE_ITEMS 8 /* pre-created row pool per tile (Issue 5) */

#define ITEM_SLOT_H    150 /* per-item vertical slot height (px) */
#define BAR_H          44  /* bottom re-summon handle thickness */

typedef struct
{
    char    name[32];
    uint8_t dev_idx;
    uint8_t conn_idx;   /* BLE HID target for drill-down; 0xFF = not connected */
    char    items[MAX_TILE_ITEMS][24];
    uint8_t item_count;
} dev_page_t;

typedef struct
{
    lv_obj_t *tile;
    lv_obj_t *header;
    lv_obj_t *list;
    lv_obj_t *item[MAX_TILE_ITEMS];
    lv_obj_t *item_label[MAX_TILE_ITEMS];
    lv_obj_t *item_icon[MAX_TILE_ITEMS];
} tile_ui_t;

typedef struct
{
    lv_obj_t   *parent;          /* the right tile we build into */
    lv_obj_t   *mouse_base;      /* hid_mouse — the always-present base */
    lv_obj_t   *overlay;         /* tileview overlay (hidden by default) */
    lv_obj_t   *home_tile;       /* (0,0) transparent — see the mouse through it */
    lv_obj_t   *list_tile;       /* (0,1) the instruction list page */
    lv_obj_t   *pager;           /* horizontal device carousel (inside list_tile) */
    tile_ui_t   t[3];            /* physical tiles: left / center / right */
    lv_obj_t   *bar;             /* bottom re-summon handle (on the mouse) */
    lv_obj_t   *empty_label;
    bool        mouse_created;   /* real hid_mouse hosted in mouse_base */
    bool        summoning;       /* bar press shown the overlay; awaiting drag/tap */

    dev_page_t  model[MAX_BONDED_DEVICES];
    int         count;
    int         current;
    bool        recycling;

    /* skaibar voice input → peer-device options */
    lv_obj_t   *mic_bar;         /* bottom mic trigger; hidden while skaibar open */
    lv_obj_t   *skaibar_input;
    lv_obj_t   *skaibar_label;
    bool        skaibar_active;
} device_pager_t;

static device_pager_t *p = NULL;

static void mic_clicked_cb(lv_event_t *e);   /* defined below (skaibar section) */
static void mouse_retarget(void);            /* defined below */
static void skaibar_close(void);             /* defined below (skaibar section) */

/* Hide the skaibar input box as soon as a list scroll begins (mirrors the left
   instruction_list, where scrolling dismisses the AI widget). */
static void scroll_hides_skaibar_cb(lv_event_t *e)
{
    (void)e;
    if (p && p->skaibar_active) skaibar_close();
}

/* Fake per-device items until real per-device instruction sets arrive. */
static void fake_items(dev_page_t *d)
{
    uint8_t n = (uint8_t)(3 + (d->dev_idx % 5));
    if (n > MAX_TILE_ITEMS) n = MAX_TILE_ITEMS;
    d->item_count = n;
    for (uint8_t i = 0; i < n; i++)
        snprintf(d->items[i], sizeof(d->items[0]), "%s #%u", d->name, (unsigned)(i + 1));
}

static void load_model(void)
{
    p->count = 0;
    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    if (db)
    {
        for (int i = 0; i < MAX_BONDED_DEVICES; i++)
        {
            if (db->devices[i].is_valid)
            {
                dev_page_t *d = &p->model[p->count];
                strncpy(d->name, db->devices[i].device_name, sizeof(d->name) - 1);
                d->name[sizeof(d->name) - 1] = '\0';
                d->dev_idx = (uint8_t)i;
                d->conn_idx = db->devices[i].conn_idx;
                fake_items(d);
                p->count++;
            }
        }
    }
    if (p->current >= p->count) p->current = p->count > 0 ? p->count - 1 : 0;
}

/* Only the item nearest the list's vertical centre shows its title; others fade. */
static void emphasize_centered(tile_ui_t *u)
{
    lv_area_t la;
    lv_obj_get_coords(u->list, &la);
    lv_coord_t list_cy = (la.y1 + la.y2) / 2;

    for (int i = 0; i < MAX_TILE_ITEMS; i++)
    {
        if (lv_obj_has_flag(u->item[i], LV_OBJ_FLAG_HIDDEN))
            continue;
        lv_area_t ia;
        lv_obj_get_coords(u->item[i], &ia);
        lv_coord_t item_cy = (ia.y1 + ia.y2) / 2;
        int d = item_cy - list_cy;
        if (d < 0) d = -d;
        lv_opa_t opa = (d >= ITEM_SLOT_H) ? LV_OPA_0
                       : (lv_opa_t)(LV_OPA_COVER -
                                    (uint32_t)d * LV_OPA_COVER / ITEM_SLOT_H);
        lv_obj_set_style_opa(u->item_label[i], opa, 0);
        lv_obj_set_style_opa(u->item_icon[i], opa, 0);
    }
}

static int nearest_item(tile_ui_t *u, lv_coord_t *out_delta)
{
    lv_area_t la;
    lv_obj_get_coords(u->list, &la);
    lv_coord_t list_cy = (la.y1 + la.y2) / 2;
    int best = -1;
    lv_coord_t bestd = LV_COORD_MAX, delta = 0;
    for (int i = 0; i < MAX_TILE_ITEMS; i++)
    {
        if (lv_obj_has_flag(u->item[i], LV_OBJ_FLAG_HIDDEN)) continue;
        lv_area_t ia;
        lv_obj_get_coords(u->item[i], &ia);
        lv_coord_t item_cy = (ia.y1 + ia.y2) / 2;
        lv_coord_t dd = item_cy - list_cy;
        if (dd < 0) dd = -dd;
        if (dd < bestd) { bestd = dd; best = i; delta = list_cy - item_cy; }
    }
    if (out_delta) *out_delta = delta;
    return best;
}

static bool s_recentering = false;
static void center_nearest(tile_ui_t *u, lv_anim_enable_t anim)
{
    lv_coord_t delta = 0;
    if (nearest_item(u, &delta) < 0 || delta == 0) return;
    s_recentering = true;
    lv_obj_scroll_by(u->list, 0, delta, anim);
    s_recentering = false;
}

static void list_scroll_cb(lv_event_t *e)
{
    tile_ui_t *u = (tile_ui_t *)lv_event_get_user_data(e);
    if (u) emphasize_centered(u);
}

static void list_scroll_end_cb(lv_event_t *e)
{
    if (s_recentering) return;
    tile_ui_t *u = (tile_ui_t *)lv_event_get_user_data(e);
    if (!u) return;
    center_nearest(u, LV_ANIM_ON);
    emphasize_centered(u);
}

static void bind_tile(int k)
{
    tile_ui_t  *u = &p->t[k];
    int logical = p->current + (k - TILE_CENTER);

    if (logical < 0 || logical >= p->count)
    {
        lv_label_set_text(u->header, "");
        for (int i = 0; i < MAX_TILE_ITEMS; i++)
            lv_obj_add_flag(u->item[i], LV_OBJ_FLAG_HIDDEN);
        return;
    }
    dev_page_t *d = &p->model[logical];
    lv_label_set_text_fmt(u->header, "%s   %d/%d", d->name, logical + 1, p->count);
    for (int i = 0; i < MAX_TILE_ITEMS; i++)
    {
        if (i < d->item_count)
        {
            lv_label_set_text(u->item_label[i], d->items[i]);
            lv_obj_clear_flag(u->item[i], LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(u->item[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
    lv_obj_update_layout(u->list);
    lv_obj_scroll_to_y(u->list, 0, LV_ANIM_OFF);
    center_nearest(u, LV_ANIM_OFF);
    emphasize_centered(u);
}

static void rebind_all(void) { for (int k = 0; k < 3; k++) bind_tile(k); }

static void snap_to_center(lv_anim_enable_t anim)
{
    lv_obj_scroll_to_view(p->t[TILE_CENTER].tile, anim);
}

static int centered_tile(void)
{
    lv_coord_t w = lv_obj_get_width(p->t[TILE_CENTER].tile);
    if (w <= 0) return TILE_CENTER;
    return (int)((lv_obj_get_scroll_x(p->pager) + w / 2) / w);
}

static void pager_scroll_end_cb(lv_event_t *e)
{
    (void)e;
    if (!p || p->recycling) return;
    int c = centered_tile();
    if (c == TILE_CENTER) return;
    int dir = c - TILE_CENTER;
    int want = p->current + dir;
    p->recycling = true;
    if (want >= 0 && want < p->count)
    {
        p->current = want;
        rebind_all();
        LOG_I("[pager] -> device %d/%d (%s)", p->current + 1, p->count,
              p->model[p->current].name);
        if (p->mouse_created) mouse_retarget();
        snap_to_center(LV_ANIM_OFF);
    }
    else if (want < 0)
    {
        snap_to_center(LV_ANIM_OFF);
        LOG_I("[pager] return to watch face");
        extern void device_pager_set_active(bool on);
        device_pager_set_active(false);
        extern void app_clock_status_bar_return_home(void);
        app_clock_status_bar_return_home();
    }
    else
    {
        snap_to_center(LV_ANIM_OFF);
    }
    p->recycling = false;
}

static void make_tile(tile_ui_t *u, lv_obj_t *parent)
{
    u->tile = lv_obj_create(parent);
    lv_obj_set_size(u->tile, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_radius(u->tile, 0, 0);
    lv_obj_set_style_border_width(u->tile, 0, 0);
    lv_obj_set_style_bg_opa(u->tile, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(u->tile, 0, 0);
    lv_obj_clear_flag(u->tile, LV_OBJ_FLAG_SCROLLABLE);

    u->header = lv_label_create(u->tile);
    lv_obj_set_style_text_font(u->header,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(u->header, lv_color_hex(0x00AAFF), 0);
    lv_obj_align(u->header, LV_ALIGN_TOP_MID, 0, 40);
    lv_label_set_text(u->header, "");

    /* Non-scrollable: vertical drags belong to the overlay tileview. */
    u->list = lv_obj_create(u->tile);
    lv_obj_set_size(u->list, LV_HOR_RES, LV_VER_RES);
    lv_obj_align(u->list, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(u->list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(u->list, 0, 0);
    lv_obj_set_style_pad_left(u->list, 0, 0);
    lv_obj_set_style_pad_right(u->list, 0, 0);
    lv_obj_set_style_pad_top(u->list, (LV_VER_RES - ITEM_SLOT_H) / 2, 0);
    lv_obj_set_style_pad_bottom(u->list, (LV_VER_RES - ITEM_SLOT_H) / 2, 0);
    lv_obj_set_style_pad_row(u->list, 0, 0);
    lv_obj_set_flex_flow(u->list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(u->list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(u->list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(u->list, list_scroll_cb, LV_EVENT_SCROLL, u);
    lv_obj_add_event_cb(u->list, list_scroll_end_cb, LV_EVENT_SCROLL_END, u);

    for (int i = 0; i < MAX_TILE_ITEMS; i++)
    {
        lv_obj_t *it = lv_obj_create(u->list);
        lv_obj_set_size(it, LV_HOR_RES, ITEM_SLOT_H);
        lv_obj_set_style_bg_opa(it, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(it, 0, 0);
        lv_obj_set_style_radius(it, 0, 0);
        lv_obj_set_style_pad_all(it, 0, 0);
        lv_obj_clear_flag(it, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(it, LV_OBJ_FLAG_HIDDEN);

        /* Per-item icon (placeholder img_flashlight) above the label — gives the
           list the same icon+text look as the left instruction_list. */
        lv_obj_t *icon = lv_img_create(it);
        lv_img_set_src(icon, &img_flashlight);  /* 80x80 */
        lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 10);
        lv_obj_clear_flag(icon, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_t *lbl = lv_label_create(it);
        lv_obj_set_style_text_font(lbl,
                                   LV_EXT_FONT_GET(get_system_font_size(1)), 0);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
        lv_obj_set_width(lbl, LV_PCT(80));
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, -12);
        lv_label_set_text(lbl, "");

        u->item[i] = it;
        u->item_label[i] = lbl;
        u->item_icon[i] = icon;
    }
}

static void refresh(void)
{
    if (!p) return;
    load_model();
    if (p->count == 0)
    {
        lv_obj_add_flag(p->pager, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(p->empty_label, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_clear_flag(p->pager, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p->empty_label, LV_OBJ_FLAG_HIDDEN);
    rebind_all();
    snap_to_center(LV_ANIM_OFF);
}

static void dev_mgr_cb(dev_mgr_event_t event, uint8_t device_idx, void *ud)
{
    (void)event; (void)device_idx; (void)ud;
    refresh();
}

/* Public: re-read the device DB and rebind (called when the page is revealed). */
void device_pager_refresh(void)
{
    refresh();
}

/* Point BLE HID at the device currently shown (drill-down). */
static void mouse_retarget(void)
{
    if (!p || p->count == 0) return;
    dev_page_t *d = &p->model[p->current];
    ble_dev_mgr_set_active_device(d->dev_idx);
    if (d->conn_idx != 0xFF)
        ble_hid_set_conn_idx(d->conn_idx);
    LOG_I("[pager] mouse target -> %s (conn_idx=%d)", d->name, d->conn_idx);
}

/* ---- overlay show / hide (watch-face style) -------------------------- */

/* Settled on a tile: HOME(transparent) → hide the overlay, reveal mouse + bar;
   LIST → keep the overlay shown, hide the bar. */
static void overlay_value_changed_cb(lv_event_t *e)
{
    (void)e;
    if (!p) return;
    /* A scroll-settle resolves any in-flight bar summon. */
    p->summoning = false;
    lv_obj_t *act = lv_tileview_get_tile_act(p->overlay);
    if (act == p->home_tile)
    {
        /* Settled on the transparent HOME → reveal the mouse + bar. */
        lv_obj_add_flag(p->overlay, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(p->bar, LV_OBJ_FLAG_HIDDEN);
        LOG_I("[pager] overlay hidden — mouse base + bar");
    }
    else
    {
        /* Settled on the LIST → hide the bar. */
        lv_obj_add_flag(p->bar, LV_OBJ_FLAG_HIDDEN);
        LOG_I("[pager] overlay on LIST — bar hidden");
    }
}

/* The bottom bar mirrors the watch-face edge zone (notification_status_bar_cb).
   On PRESS it shows the overlay parked at the transparent HOME tile — the mouse
   shows through it. Because the bar has PRESS_LOCK cleared, the instant the
   finger drags up off the bar the press transfers to the now-topmost overlay,
   which scrolls under the finger (native tileview finger-follow) and pulls the
   LIST up from below. The mouse underneath stays put (the overlay is the
   hit-test target). A tap that never leaves the bar doesn't transfer, so the
   bar's RELEASED undoes the park (back to mouse + bar). A real drag is resolved
   by the overlay's VALUE_CHANGED instead — RELEASED never fires on the bar
   then, it gets PRESS_LOST when the gesture transfers. */
static void bar_cb(lv_event_t *e)
{
    if (!p) return;
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED)
    {
        p->summoning = true;
        device_pager_refresh();
        lv_obj_set_tile_id(p->overlay, 0, 0, LV_ANIM_OFF); /* park at HOME */
        lv_obj_clear_flag(p->overlay, LV_OBJ_FLAG_HIDDEN);
    }
    else if (code == LV_EVENT_RELEASED)
    {
        if (p->summoning) /* tap, no drag → never transferred to the overlay */
        {
            p->summoning = false;
            lv_obj_add_flag(p->overlay, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(p->bar, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* Public: host / tear down the real mouse base as the device page is
   entered / left. */
void device_pager_set_active(bool on)
{
    if (!p) return;
    if (on)
    {
        if (!p->mouse_created)
        {
            p->mouse_created = true; /* set first: hid_mouse_create can re-enter */
            hid_mouse_create(p->mouse_base);
            mouse_retarget();
            /* hid_mouse_create populates mouse_base and disturbs our stack order;
               re-assert it so the overlay sits above the mouse touch surface and
               the bar sits on top of all (else the bar's presses leak through to
               the trackpad — the bar must be the hit-test target at the bottom). */
            lv_obj_move_foreground(p->overlay);
            lv_obj_move_foreground(p->bar);
            LOG_I("[pager] device page active — mouse base hosted");
        }
        device_pager_refresh();
        /* Enter on the LIST (covering the mouse); pull down to reveal the mouse. */
        lv_obj_clear_flag(p->overlay, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_tile_id(p->overlay, 0, 1, LV_ANIM_OFF);
        lv_obj_add_flag(p->bar, LV_OBJ_FLAG_HIDDEN);
    }
    else if (p->mouse_created)
    {
        hid_mouse_destroy();
        lv_obj_clean(p->mouse_base);
        p->mouse_created = false;
        lv_obj_clear_flag(p->overlay, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_tile_id(p->overlay, 0, 1, LV_ANIM_OFF); /* reset to LIST */
        lv_obj_add_flag(p->bar, LV_OBJ_FLAG_HIDDEN);
        LOG_I("[pager] device page inactive — mouse base torn down");
    }
}

/* ---- skaibar voice input -> peer-device options ---------------------- */
static void skaibar_open(void)
{
    if (!p || !p->skaibar_input) return;
    lv_label_set_text(p->skaibar_label, "聽取中");
    lv_obj_add_flag(p->mic_bar, LV_OBJ_FLAG_HIDDEN);     /* like the left list */
    lv_obj_clear_flag(p->skaibar_input, LV_OBJ_FLAG_HIDDEN);
    p->skaibar_active = true;
    LOG_I("[pager] skaibar opened (mic) -- awaiting transcript (no real ASR)");
}

static void skaibar_close(void)
{
    if (!p || !p->skaibar_input) return;
    lv_obj_add_flag(p->skaibar_input, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(p->mic_bar, LV_OBJ_FLAG_HIDDEN);
    p->skaibar_active = false;
}

static void mic_clicked_cb(lv_event_t *e)
{
    (void)e;
    if (p && p->skaibar_active) skaibar_close();
    else                        skaibar_open();
}

void device_pager_skaibar_say(const char *text)
{
    if (!p) return;
    if (!p->skaibar_active) skaibar_open();
    lv_label_set_text(p->skaibar_label, text ? text : "");
    LOG_I("[pager] skaibar transcript: \"%s\"", text ? text : "");
}

void device_pager_skaibar_options(int n, const char *const opts[])
{
    if (!p || p->count == 0 || n <= 0) return;
    if (n > MAX_TILE_ITEMS) n = MAX_TILE_ITEMS;
    dev_page_t *d = &p->model[p->current];
    d->item_count = (uint8_t)n;
    for (int i = 0; i < n; i++)
    {
        strncpy(d->items[i], opts[i] ? opts[i] : "", sizeof(d->items[0]) - 1);
        d->items[i][sizeof(d->items[0]) - 1] = '\0';
    }
    rebind_all();
    skaibar_close();
    LOG_I("[pager] skaibar options applied (%d) to %s", n, d->name);
}

/* Public: build the page into a tileview tile. Called from
   app_clock_main_status_bar_init. */
lv_obj_t *device_pager_create(lv_obj_t *parent)
{
    if (p) return p->overlay; /* singleton: one right tile */
    p = (device_pager_t *)rt_calloc(1, sizeof(device_pager_t));
    if (!p) { LOG_E("device_pager alloc fail"); return NULL; }
    p->parent = parent;

    /* Mouse base — always-present page beneath everything (the "clock"). */
    p->mouse_base = lv_obj_create(parent);
    lv_obj_set_size(p->mouse_base, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->mouse_base, 0, 0);
    lv_obj_set_style_radius(p->mouse_base, 0, 0);
    lv_obj_set_style_border_width(p->mouse_base, 0, 0);
    lv_obj_set_style_pad_all(p->mouse_base, 0, 0);
    lv_obj_set_style_bg_color(p->mouse_base, lv_color_hex(0x101010), 0);
    lv_obj_clear_flag(p->mouse_base, LV_OBJ_FLAG_SCROLLABLE);

    /* Overlay tileview — HOME(transparent) + LIST. Hidden until shown. */
    p->overlay = lv_tileview_create(parent);
    lv_obj_set_size(p->overlay, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->overlay, 0, 0);
    lv_obj_set_style_bg_opa(p->overlay, LV_OPA_TRANSP, 0);
    lv_obj_set_scrollbar_mode(p->overlay, LV_SCROLLBAR_MODE_OFF);
    /* Don't let an over-long vertical drag chain past the overlay into the
       watch-face tileview behind it (which would navigate to the app menu). */
    lv_obj_clear_flag(p->overlay, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_add_event_cb(p->overlay, overlay_value_changed_cb,
                        LV_EVENT_VALUE_CHANGED, NULL);
    p->home_tile = lv_tileview_add_tile(p->overlay, 0, 0, LV_DIR_BOTTOM);
    p->list_tile = lv_tileview_add_tile(p->overlay, 0, 1, LV_DIR_TOP);
    lv_obj_set_style_bg_opa(p->home_tile, LV_OPA_TRANSP, 0); /* see mouse through */
    lv_obj_set_style_bg_color(p->list_tile, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(p->list_tile, LV_OPA_50, 0); /* 50% — the mouse page shows through behind the list */
    lv_obj_clear_flag(p->list_tile, LV_OBJ_FLAG_SCROLLABLE);

    /* List carousel — horizontal device recycler, fills the list tile. */
    p->pager = lv_obj_create(p->list_tile);
    lv_obj_set_size(p->pager, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->pager, 0, 0);
    lv_obj_set_style_pad_all(p->pager, 0, 0);
    lv_obj_set_style_border_width(p->pager, 0, 0);
    lv_obj_set_style_radius(p->pager, 0, 0);
    lv_obj_set_style_bg_opa(p->pager, LV_OPA_TRANSP, 0);
    lv_obj_set_flex_flow(p->pager, LV_FLEX_FLOW_ROW);
    lv_obj_set_scroll_dir(p->pager, LV_DIR_HOR);
    lv_obj_set_scroll_snap_x(p->pager, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scrollbar_mode(p->pager, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(p->pager, pager_scroll_end_cb, LV_EVENT_SCROLL_END, NULL);
    /* Start scrolling the device list (or pulling the overlay) → hide the
       skaibar, like the left instruction_list dismisses on scroll. */
    lv_obj_add_event_cb(p->pager, scroll_hides_skaibar_cb, LV_EVENT_SCROLL_BEGIN, NULL);
    lv_obj_add_event_cb(p->overlay, scroll_hides_skaibar_cb, LV_EVENT_SCROLL_BEGIN, NULL);

    for (int k = 0; k < 3; k++) make_tile(&p->t[k], p->pager);

    /* Bottom-centre mic / skaibar bar — matches the left instruction_list. */
    p->mic_bar = lv_obj_create(p->list_tile);
    lv_obj_set_size(p->mic_bar, 240, 50);
    lv_obj_align(p->mic_bar, LV_ALIGN_BOTTOM_MID, 0, -75);
    lv_obj_set_style_bg_color(p->mic_bar, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(p->mic_bar, LV_OPA_50, 0);
    lv_obj_set_style_radius(p->mic_bar, 25, 0);
    lv_obj_set_style_border_width(p->mic_bar, 0, 0);
    lv_obj_set_style_pad_all(p->mic_bar, 0, 0);
    lv_obj_clear_flag(p->mic_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(p->mic_bar, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(p->mic_bar, mic_clicked_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *mic_icon = lv_img_create(p->mic_bar);
    lv_img_set_src(mic_icon, &icon_mic);
    lv_obj_center(mic_icon);
    lv_obj_clear_flag(mic_icon, LV_OBJ_FLAG_CLICKABLE);

    /* skaibar input box (shown on mic tap) — mirrors the left instruction_list's
       voice input pill (lv_instruction_list_layout.c): a BOTTOM-aligned container
       framed by the message_widget_bg image. On real hardware LVGL's drawn border
       is too thin, so the image (442x252) supplies the visible border + rounded
       shape. Tap it to dismiss. */
    p->skaibar_input = lv_obj_create(p->list_tile);
    lv_obj_set_size(p->skaibar_input, 442, 252);
    lv_obj_align(p->skaibar_input, LV_ALIGN_BOTTOM_MID, 0, 80);
    lv_obj_set_style_bg_opa(p->skaibar_input, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(p->skaibar_input, 0, 0);
    lv_obj_set_style_pad_all(p->skaibar_input, 0, 0);
    lv_obj_clear_flag(p->skaibar_input, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(p->skaibar_input, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(p->skaibar_input, mic_clicked_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(p->skaibar_input, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *skaibar_frame = lv_img_create(p->skaibar_input);
    lv_img_set_src(skaibar_frame, &message_widget_bg);
    lv_obj_center(skaibar_frame);
    lv_obj_clear_flag(skaibar_frame, LV_OBJ_FLAG_CLICKABLE);
    p->skaibar_label = lv_label_create(p->skaibar_input);
    lv_obj_set_style_text_color(p->skaibar_label, lv_color_white(), 0);
    lv_obj_set_style_text_opa(p->skaibar_label, LV_OPA_80, 0);
    lv_obj_set_style_text_font(p->skaibar_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_align(p->skaibar_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(p->skaibar_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(p->skaibar_label, 360);
    lv_obj_align(p->skaibar_label, LV_ALIGN_TOP_MID, 0, 60);
    lv_label_set_text(p->skaibar_label, "");

    p->empty_label = lv_label_create(p->list_tile);
    lv_label_set_text(p->empty_label, "No devices");
    lv_obj_set_style_text_color(p->empty_label, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_align(p->empty_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(p->empty_label);

    /* Bottom re-summon handle (on the mouse base). PRESS_LOCK cleared so the
       continuing drag transfers to the overlay (watch-face edge-zone style). */
    p->bar = lv_obj_create(parent);
    lv_obj_set_size(p->bar, LV_HOR_RES, BAR_H);
    lv_obj_align(p->bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(p->bar, lv_color_hex(0x202020), 0);
    lv_obj_set_style_bg_opa(p->bar, LV_OPA_70, 0);
    lv_obj_set_style_border_width(p->bar, 0, 0);
    lv_obj_set_style_radius(p->bar, 0, 0);
    lv_obj_set_style_pad_all(p->bar, 0, 0);
    lv_obj_clear_flag(p->bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(p->bar, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_flag(p->bar, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(p->bar, bar_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(p->bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *grab = lv_obj_create(p->bar);
    lv_obj_set_size(grab, 44, 5);
    lv_obj_center(grab);
    lv_obj_set_style_radius(grab, 3, 0);
    lv_obj_set_style_bg_color(grab, lv_color_hex(0x888888), 0);
    lv_obj_set_style_border_width(grab, 0, 0);
    lv_obj_clear_flag(grab, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_set_tile_id(p->overlay, 0, 1, LV_ANIM_OFF); /* start on the LIST */

    ble_dev_mgr_register_callback(dev_mgr_cb, NULL);
    refresh();
    LOG_I("[pager] built (overlay tileview + mouse base, %d devices)", p->count);
    return p->overlay;
}
