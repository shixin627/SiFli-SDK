/**
 ******************************************************************************
 * @file   device_pager.c
 * @brief  Right-side per-device control page — content builder for the
 *         watch-face tileview's RIGHT tile.
 *
 *  This is NOT a gui_app. device_pager_create(right_tile) builds the page into a
 *  tile of app_clock_main_status_bar. The "pull out from the right" is the native
 *  tileview finger-follow scroll.
 *
 *  Layout (vertical drawer):
 *    - The list_items are the FRONT layer, full screen, with a handle bar at the
 *      TOP of the list. The mouse page is stacked BEHIND it (full screen).
 *    - Grab the top handle and pull DOWN: the whole list slides down (finger-
 *      follow), revealing the mouse beneath. On release it snaps so the handle
 *      bar rests at the BOTTOM of the screen — a pull-tab to bring the list back.
 *    - Grab that bottom bar and pull UP: the list slides back up over the mouse.
 *    - Within the list: vertical drag scrolls the device's items; horizontal
 *      drag pages between devices; drag-right past the first device → watch face.
 *
 *  The real hid_mouse is hosted behind the list when the device page is entered
 *  (device_pager_set_active) and torn down on leave; pulling the list down
 *  reveals it, and it is drill-down-targeted at the current device. list_items
 *  are per-device fake content; real per-device data + skaibar voice flow over
 *  BLE arrive later.
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
#include "hid_mouse.h"                 /* mouse component (hosted later) */

#define DBG_TAG "device.pager"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

LV_IMG_DECLARE(icon_mic); /* shared mic/voice icon, same as instruction_list */

#define TILE_LEFT      0
#define TILE_CENTER    1
#define TILE_RIGHT     2
#define MAX_TILE_ITEMS 8 /* pre-created row pool per tile (Issue 5) */

#define ITEM_SLOT_H    150 /* per-item vertical slot height (px) */

/* Drawer geometry: the handle bar that stays visible when the list is pulled
   down to reveal the mouse. */
#define BAR_H        44
#define MAX_OFFSET   (LV_VER_RES - BAR_H)   /* list pulled down: bar at bottom  */
#define SNAP_OPEN_AT (LV_VER_RES * 30 / 100)/* drag past this → reveal mouse    */

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
    lv_obj_t   *mouse_base;      /* full-screen trackpad page (placeholder) */
    lv_obj_t   *pager;           /* list carousel (the FRONT drawer) */
    tile_ui_t   t[3];            /* physical tiles: left / center / right */
    lv_obj_t   *handle;          /* top-of-list handle bar (drawer pull-tab) */
    lv_obj_t   *empty_label;
    lv_coord_t  offset;          /* how far the list is pulled down (0 = out) */
    bool        dragging;        /* handle drag in progress */
    lv_coord_t  grab_dy;         /* finger offset within the bar at grab time */
    bool        mouse_created;   /* real hid_mouse hosted behind the list */

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
void device_pager_set_active(bool on);       /* defined below (public) */

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
        if (p->mouse_created) mouse_retarget(); /* control the paged-to device */
        snap_to_center(LV_ANIM_OFF);
    }
    else if (want < 0)
    {
        snap_to_center(LV_ANIM_OFF);
        LOG_I("[pager] return to watch face");
        device_pager_set_active(false); /* leaving the device page → drop mouse */
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

    /* Device-name header + mic (skaibar trigger), below the top handle. */
    u->header = lv_label_create(u->tile);
    lv_obj_set_style_text_font(u->header,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(u->header, lv_color_hex(0x00AAFF), 0);
    lv_obj_align(u->header, LV_ALIGN_TOP_MID, 0, BAR_H + 8);
    lv_label_set_text(u->header, "");

    /* Bottom-centre mic / skaibar bar — same geometry as the left
       instruction_list's mic_bar (240x50, BOTTOM_MID -75, radius 25). */
    lv_obj_t *mic = lv_obj_create(u->tile);
    lv_obj_set_size(mic, 240, 50);
    lv_obj_align(mic, LV_ALIGN_BOTTOM_MID, 0, -75);
    lv_obj_set_style_radius(mic, 25, 0);
    lv_obj_set_style_bg_color(mic, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(mic, LV_OPA_50, 0);
    lv_obj_set_style_border_width(mic, 0, 0);
    lv_obj_set_style_pad_all(mic, 0, 0);
    lv_obj_clear_flag(mic, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(mic, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(mic, mic_clicked_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *mic_icon = lv_img_create(mic);
    lv_img_set_src(mic_icon, &icon_mic);
    lv_obj_center(mic_icon);
    lv_obj_clear_flag(mic_icon, LV_OBJ_FLAG_CLICKABLE);

    /* Vertical instruction list: snap-centre scroll, item-centred title. */
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
    lv_obj_set_scroll_dir(u->list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(u->list, LV_SCROLLBAR_MODE_OFF);
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

/* Slide the list (and its top handle) down by `y`. y=0 → list fully out;
   y=MAX_OFFSET → list pulled down with just the handle bar at the bottom. */
static void set_offset(lv_coord_t y)
{
    if (y < 0) y = 0;
    if (y > MAX_OFFSET) y = MAX_OFFSET;
    p->offset = y;
    lv_obj_set_y(p->pager, y);
    lv_obj_set_y(p->handle, y);   /* handle rides the top edge of the list */
}

static void offset_anim_cb(void *var, int32_t v)
{
    (void)var;
    set_offset((lv_coord_t)v);
}

/* Smooth snap to a target offset (release feel, like a tileview settle). */
static void snap_offset(lv_coord_t target)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, p);
    lv_anim_set_exec_cb(&a, offset_anim_cb);
    lv_anim_set_values(&a, p->offset, target);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

static void handle_drag_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t pt;
    lv_indev_get_point(indev, &pt);

    if (code == LV_EVENT_PRESSED)
    {
        lv_anim_del(p, offset_anim_cb); /* cancel any in-flight snap */
        p->dragging = true;
        /* Remember where on the bar the finger landed so the bar tracks the
           finger 1:1 from that point (no jump-to-centre). */
        p->grab_dy = pt.y - p->offset;
    }
    else if (code == LV_EVENT_PRESSING && p->dragging)
    {
        set_offset(pt.y - p->grab_dy); /* exact finger-follow */
    }
    else if ((code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) &&
             p->dragging)
    {
        p->dragging = false;
        /* Snap (animated): pulled down past the threshold → reveal mouse (bar to
           bottom); else → list back out. */
        snap_offset(p->offset >= SNAP_OPEN_AT ? MAX_OFFSET : 0);
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

/* Public: host / tear down the real mouse behind the list as the device page is
   entered / left. The mouse is the page; the list drawer rides on top of it. */
void device_pager_set_active(bool on)
{
    if (!p) return;
    if (on)
    {
        if (!p->mouse_created)
        {
            hid_mouse_create(p->mouse_base);
            p->mouse_created = true;
            mouse_retarget();
            LOG_I("[pager] device page active — mouse hosted");
        }
    }
    else if (p->mouse_created)
    {
        hid_mouse_destroy();
        /* hid_mouse_destroy only NULLs its own pointers; clear the objects it
           built so the host is empty for next time. */
        lv_obj_clean(p->mouse_base);
        p->mouse_created = false;
        lv_anim_del(p, offset_anim_cb);
        set_offset(0); /* reset the drawer: list out for next entry */
        LOG_I("[pager] device page inactive — mouse torn down");
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
    if (p) return p->pager; /* singleton: one right tile */
    p = (device_pager_t *)rt_calloc(1, sizeof(device_pager_t));
    if (!p) { LOG_E("device_pager alloc fail"); return NULL; }
    p->parent = parent;
    p->offset = 0;

    /* Mouse page base — full screen, BEHIND the list. Placeholder for now; the
       real hid_mouse gets hosted here once the drawer interaction is confirmed. */
    p->mouse_base = lv_obj_create(parent);
    lv_obj_set_size(p->mouse_base, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->mouse_base, 0, 0);
    lv_obj_set_style_radius(p->mouse_base, 0, 0);
    lv_obj_set_style_border_width(p->mouse_base, 0, 0);
    lv_obj_set_style_pad_all(p->mouse_base, 0, 0);
    lv_obj_set_style_bg_color(p->mouse_base, lv_color_hex(0x101010), 0);
    lv_obj_clear_flag(p->mouse_base, LV_OBJ_FLAG_SCROLLABLE);
    /* Empty host — the real hid_mouse is built into this on device_pager_set_active(). */

    /* List carousel — the FRONT drawer, full screen, slides down via offset. */
    p->pager = lv_obj_create(parent);
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

    /* Top-of-list handle bar — drawer pull-tab. Rides the list's top edge:
       at the top when the list is out, at the bottom when pulled down. */
    p->handle = lv_obj_create(parent);
    lv_obj_set_size(p->handle, LV_HOR_RES, BAR_H);
    lv_obj_set_pos(p->handle, 0, 0);
    lv_obj_set_style_bg_color(p->handle, lv_color_hex(0x202020), 0);
    lv_obj_set_style_bg_opa(p->handle, LV_OPA_70, 0);
    lv_obj_set_style_border_width(p->handle, 0, 0);
    lv_obj_set_style_radius(p->handle, 0, 0);
    lv_obj_set_style_pad_all(p->handle, 0, 0);
    lv_obj_clear_flag(p->handle, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(p->handle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(p->handle, handle_drag_cb, LV_EVENT_ALL, NULL);
    lv_obj_t *grab = lv_obj_create(p->handle);
    lv_obj_set_size(grab, 44, 5);
    lv_obj_center(grab);
    lv_obj_set_style_radius(grab, 3, 0);
    lv_obj_set_style_bg_color(grab, lv_color_hex(0x888888), 0);
    lv_obj_set_style_border_width(grab, 0, 0);
    lv_obj_clear_flag(grab, LV_OBJ_FLAG_CLICKABLE);

    /* skaibar input box (shown on mic tap). */
    p->skaibar_input = lv_obj_create(parent);
    lv_obj_set_size(p->skaibar_input, LV_PCT(92), 84);
    lv_obj_align(p->skaibar_input, LV_ALIGN_TOP_MID, 0, BAR_H + 60);
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

    p->empty_label = lv_label_create(parent);
    lv_label_set_text(p->empty_label, "No devices");
    lv_obj_set_style_text_color(p->empty_label, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_align(p->empty_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(p->empty_label);

    ble_dev_mgr_register_callback(dev_mgr_cb, NULL);
    refresh();
    LOG_I("[pager] built (drawer model, %d devices)", p->count);
    return p->pager;
}
