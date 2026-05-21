/**
 ******************************************************************************
 * @file   device_pager.c
 * @brief  Right-side per-device control page — content builder for the
 *         watch-face tileview's RIGHT tile.
 *
 *  Drawer = a real vertical lv_tileview (same native finger-follow mechanism the
 *  main face uses — NOT hand-computed offsets):
 *    (0,1) LIST  — the device's instruction list_items (a horizontal 3-tile
 *                  recycler so you can page between devices) + a bottom-centre
 *                  mic/skaibar bar (matches the left instruction_list). Start here.
 *    (0,0) MOUSE — the real hid_mouse, the page above. Swipe DOWN on the list to
 *                  reveal it (the tileview reveals the tile above on a down-swipe).
 *  Swipe up (or the mouse's bottom-bar "back") → back to the list page.
 *
 *  The mouse is created ONLY once the tileview settles on the mouse page and
 *  destroyed when it leaves — so it never processes touches (and never moves)
 *  while you are on the list page or mid-swipe. It is drill-down-targeted at the
 *  current device. list_items are per-device fake content; real per-device data
 *  + skaibar voice over BLE arrive later.
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
#include "hid_mouse.h"                 /* mouse component, hosted in the mouse tile */

#define DBG_TAG "device.pager"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

LV_IMG_DECLARE(icon_mic); /* shared mic/voice icon, same as instruction_list */

#define TILE_LEFT      0
#define TILE_CENTER    1
#define TILE_RIGHT     2
#define MAX_TILE_ITEMS 8 /* pre-created row pool per tile (Issue 5) */

#define ITEM_SLOT_H    150 /* per-item vertical slot height (px) */

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
} tile_ui_t;

typedef struct
{
    lv_obj_t   *parent;          /* the right tile we build into */
    lv_obj_t   *drawer;          /* vertical tileview: list page + mouse page */
    lv_obj_t   *list_tile;       /* (0,1) bottom — the instruction list page */
    lv_obj_t   *mouse_tile;      /* (0,0) top — the hid_mouse page */
    lv_obj_t   *pager;           /* horizontal device carousel (inside list_tile) */
    tile_ui_t   t[3];            /* physical tiles: left / center / right */
    lv_obj_t   *empty_label;
    bool        mouse_created;   /* real hid_mouse hosted in the mouse tile */

    dev_page_t  model[MAX_BONDED_DEVICES];
    int         count;
    int         current;
    bool        recycling;

    /* skaibar voice input → peer-device options */
    lv_obj_t   *skaibar_input;
    lv_obj_t   *skaibar_label;
    bool        skaibar_active;
} device_pager_t;

static device_pager_t *p = NULL;

static void mic_clicked_cb(lv_event_t *e);   /* defined below (skaibar section) */
static void mouse_retarget(void);            /* defined below */

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

    /* The vertical list does NOT capture vertical drags itself — those belong to
       the drawer tileview (list <-> mouse). The centred item is the device's
       instruction; item browsing rides the (future) arc-scroll like the left
       list. So the list is non-scrollable here. */
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

        lv_obj_t *lbl = lv_label_create(it);
        lv_obj_set_style_text_font(lbl,
                                   LV_EXT_FONT_GET(get_system_font_size(1)), 0);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
        lv_obj_set_width(lbl, LV_PCT(80));
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(lbl);
        lv_label_set_text(lbl, "");

        u->item[i] = it;
        u->item_label[i] = lbl;
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

/* Create / tear down the real mouse in the mouse tile. */
static void mouse_layer_create(void)
{
    if (!p || p->mouse_created) return;
    p->mouse_created = true; /* set first: hid_mouse_create can re-enter via layout */
    hid_mouse_create(p->mouse_tile);
    mouse_retarget();
    LOG_I("[pager] mouse page settled — mouse hosted");
}

static void mouse_layer_destroy(void)
{
    if (!p || !p->mouse_created) return;
    hid_mouse_destroy();
    lv_obj_clean(p->mouse_tile);  /* hid_mouse_destroy only NULLs its pointers */
    p->mouse_created = false;
    LOG_I("[pager] left mouse page — mouse torn down");
}

/* Bring the list page back (mouse's bottom-bar "back" gesture, deferred so we
   don't tear the mouse down from inside its own event handler). */
static void back_to_list_async(void *unused)
{
    (void)unused;
    if (p && p->drawer) lv_obj_set_tile_id(p->drawer, 0, 1, LV_ANIM_ON);
}
static void mouse_pull_cb(int up_px, int released)
{
    if (!p || !p->mouse_created) return;
    if (!released) return;
    if (up_px > 60) lv_async_call(back_to_list_async, NULL);
}

/* The tileview settled on a page → create the mouse on the mouse page, drop it
   on the list page. Fired natively by lv_tileview's value-changed. */
static void drawer_value_changed_cb(lv_event_t *e)
{
    (void)e;
    if (!p) return;
    lv_obj_t *act = lv_tileview_get_tile_act(p->drawer);
    if (act == p->mouse_tile) mouse_layer_create();
    else                      mouse_layer_destroy();
}

/* Public: called when the device page (right tile) is entered / left. */
void device_pager_set_active(bool on)
{
    if (!p) return;
    if (on)
    {
        device_pager_refresh();
        /* start on the list page; the mouse is only created once you swipe to it */
        lv_obj_set_tile_id(p->drawer, 0, 1, LV_ANIM_OFF);
    }
    else
    {
        mouse_layer_destroy();
        lv_obj_set_tile_id(p->drawer, 0, 1, LV_ANIM_OFF);
    }
}

/* ---- skaibar voice input -> peer-device options ---------------------- */
static void skaibar_open(void)
{
    if (!p || !p->skaibar_input) return;
    lv_label_set_text(p->skaibar_label, "Listening...");
    lv_obj_clear_flag(p->skaibar_input, LV_OBJ_FLAG_HIDDEN);
    p->skaibar_active = true;
    LOG_I("[pager] skaibar opened (mic) -- awaiting transcript (no real ASR)");
}

static void skaibar_close(void)
{
    if (!p || !p->skaibar_input) return;
    lv_obj_add_flag(p->skaibar_input, LV_OBJ_FLAG_HIDDEN);
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
    if (p) return p->drawer; /* singleton: one right tile */
    p = (device_pager_t *)rt_calloc(1, sizeof(device_pager_t));
    if (!p) { LOG_E("device_pager alloc fail"); return NULL; }
    p->parent = parent;

    /* Vertical tileview drawer — native finger-follow, same as the main face. */
    p->drawer = lv_tileview_create(parent);
    lv_obj_set_size(p->drawer, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->drawer, 0, 0);
    lv_obj_set_style_bg_opa(p->drawer, LV_OPA_TRANSP, 0);
    lv_obj_set_scrollbar_mode(p->drawer, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(p->drawer, drawer_value_changed_cb,
                        LV_EVENT_VALUE_CHANGED, NULL);
    /* Mouse is the TOP tile, list the BOTTOM tile, and we start on the list.
       A swipe DOWN then reveals the mouse (tileview reveals the tile above when
       you swipe down — matching "pull the list down to uncover the mouse"). */
    p->mouse_tile = lv_tileview_add_tile(p->drawer, 0, 0, LV_DIR_BOTTOM);
    p->list_tile  = lv_tileview_add_tile(p->drawer, 0, 1, LV_DIR_TOP);
    lv_obj_set_style_bg_color(p->mouse_tile, lv_color_hex(0x101010), 0);
    lv_obj_set_style_bg_opa(p->mouse_tile, LV_OPA_COVER, 0);

    /* List carousel — horizontal device recycler, fills the list tile. */
    p->pager = lv_obj_create(p->list_tile);
    lv_obj_set_size(p->pager, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->pager, 0, 0);
    lv_obj_set_style_pad_all(p->pager, 0, 0);
    lv_obj_set_style_border_width(p->pager, 0, 0);
    lv_obj_set_style_radius(p->pager, 0, 0);
    lv_obj_set_style_bg_color(p->pager, lv_color_black(), 0);
    lv_obj_set_flex_flow(p->pager, LV_FLEX_FLOW_ROW);
    lv_obj_set_scroll_dir(p->pager, LV_DIR_HOR);
    lv_obj_set_scroll_snap_x(p->pager, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scrollbar_mode(p->pager, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(p->pager, pager_scroll_end_cb, LV_EVENT_SCROLL_END, NULL);

    for (int k = 0; k < 3; k++) make_tile(&p->t[k], p->pager);

    /* Bottom-centre mic / skaibar bar — matches the left instruction_list. */
    lv_obj_t *mic_bar = lv_obj_create(p->list_tile);
    lv_obj_set_size(mic_bar, 240, 50);
    lv_obj_align(mic_bar, LV_ALIGN_BOTTOM_MID, 0, -75);
    lv_obj_set_style_bg_color(mic_bar, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(mic_bar, LV_OPA_50, 0);
    lv_obj_set_style_radius(mic_bar, 25, 0);
    lv_obj_set_style_border_width(mic_bar, 0, 0);
    lv_obj_set_style_pad_all(mic_bar, 0, 0);
    lv_obj_clear_flag(mic_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(mic_bar, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(mic_bar, mic_clicked_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *mic_icon = lv_img_create(mic_bar);
    lv_img_set_src(mic_icon, &icon_mic);
    lv_obj_center(mic_icon);
    lv_obj_clear_flag(mic_icon, LV_OBJ_FLAG_CLICKABLE);

    /* skaibar input box (shown on mic tap). */
    p->skaibar_input = lv_obj_create(p->list_tile);
    lv_obj_set_size(p->skaibar_input, LV_PCT(92), 84);
    lv_obj_align(p->skaibar_input, LV_ALIGN_BOTTOM_MID, 0, -130);
    lv_obj_set_style_bg_color(p->skaibar_input, lv_color_hex(0x1E1E1E), 0);
    lv_obj_set_style_bg_opa(p->skaibar_input, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(p->skaibar_input, 18, 0);
    lv_obj_set_style_border_color(p->skaibar_input, lv_color_hex(0x00AAFF), 0);
    lv_obj_set_style_border_width(p->skaibar_input, 2, 0);
    lv_obj_set_style_pad_all(p->skaibar_input, 10, 0);
    lv_obj_clear_flag(p->skaibar_input, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(p->skaibar_input, LV_OBJ_FLAG_HIDDEN);
    p->skaibar_label = lv_label_create(p->skaibar_input);
    lv_obj_set_style_text_color(p->skaibar_label, lv_color_white(), 0);
    lv_obj_set_style_text_align(p->skaibar_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(p->skaibar_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(p->skaibar_label, LV_PCT(100));
    lv_obj_center(p->skaibar_label);
    lv_label_set_text(p->skaibar_label, "");

    p->empty_label = lv_label_create(p->list_tile);
    lv_label_set_text(p->empty_label, "No devices");
    lv_obj_set_style_text_color(p->empty_label, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_align(p->empty_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(p->empty_label);

    /* The mouse's bottom-bar "back" gesture returns to the list page. */
    hid_mouse_set_host_pull_cb(mouse_pull_cb);

    lv_obj_set_tile_id(p->drawer, 0, 1, LV_ANIM_OFF); /* start on the list */

    ble_dev_mgr_register_callback(dev_mgr_cb, NULL);
    refresh();
    LOG_I("[pager] built (tileview drawer, %d devices)", p->count);
    return p->drawer;
}
