/**
 ******************************************************************************
 * @file   device_pager.c
 * @brief  Right-side per-device instruction pager — content builder for the
 *         watch-face tileview's RIGHT tile (mirrors lv_instruction_list_layout
 *         on the LEFT tile).
 *
 *  This is NOT a gui_app. Like lv_instruction_list_layout_create(left_tile),
 *  device_pager_create(right_tile) builds the pager content into a tile of
 *  app_clock_main_status_bar. The "pull out from the right" is then the native
 *  tileview finger-follow scroll — no app launch, no custom entry gesture.
 *
 *  Gesture model (two axes):
 *    - vertical drag   → scroll the current device's instruction list
 *      (centered item shows its title, like the LEFT instruction_list)
 *    - horizontal drag → page between devices (3-tile recycler, Issue 2/5);
 *      dragging right past the first device returns to the watch face.
 *
 *  UI is a faithful VISUAL replica of the left instruction_list (transparent
 *  bg, single big centered white title, bottom mic/skaibar) rather than a
 *  shared instance — the left list is a 4000-line singleton that can't be
 *  instantiated per tile. Design values are copied from
 *  lv_instruction_list_layout.c so both surfaces read the same; the right-edge
 *  arc_scroll affordance + per-device real data are deferred.
 *
 *  Item data is per-device fake content for now; real per-device instruction
 *  sets arrive over the BLE protocol later.
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
#include "hid_mouse.h"                 /* hosted mouse component (down-drag layer) */

#define DBG_TAG "device.pager"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

LV_IMG_DECLARE(icon_mic); /* shared mic/voice icon, same as instruction_list */

#define TILE_LEFT      0
#define TILE_CENTER    1
#define TILE_RIGHT     2
#define MAX_TILE_ITEMS 8 /* pre-created row pool per tile (Issue 5) */

/* Vertical list metrics — echo lv_instruction_list_layout's spacious feel:
   one big centered title, generous slot, only the centered item labelled. */
#define ITEM_SLOT_H    150 /* per-item vertical slot height (px) */

typedef struct
{
    char    name[32];
    uint8_t dev_idx;
    uint8_t conn_idx;   /* BLE HID target for drill-down; 0xFF = not connected */
    char    items[MAX_TILE_ITEMS][24];
    uint8_t item_count;
} dev_page_t;

/* Down-drag mouse drawer: how far past the top handle counts as "open". */
#define MOUSE_OPEN_THRESHOLD (LV_VER_RES * 35 / 100)
#define HANDLE_ZONE_H        54

typedef struct
{
    lv_obj_t *tile;
    lv_obj_t *header;
    lv_obj_t *list;
    lv_obj_t *item[MAX_TILE_ITEMS];
    lv_obj_t *item_label[MAX_TILE_ITEMS];
    lv_obj_t *mic_bar;
} tile_ui_t;

typedef struct
{
    lv_obj_t   *parent;         /* the right tile we build into */
    lv_obj_t   *pager;          /* scroll-snap container (instruction panel) */
    tile_ui_t   t[3];           /* physical tiles: left / center / right */
    lv_obj_t   *empty_label;
    dev_page_t  model[MAX_BONDED_DEVICES];
    int         count;
    int         current;
    bool        recycling;

    /* Down-drag mouse drawer */
    lv_obj_t   *mouse_host;     /* behind the pager; hosts hid_mouse on reveal */
    lv_obj_t   *handle;         /* top pull-down handle (initiates the reveal) */
    bool        mouse_created;  /* hid_mouse_create has run (mouse mode active) */
    bool        dragging;       /* handle drag in progress */
    lv_coord_t  drag_start_y;   /* finger y at drag start */

    /* skaibar voice input → peer-device options */
    lv_obj_t   *skaibar_input;  /* bottom input box (shown on mic tap) */
    lv_obj_t   *skaibar_label;  /* transcript text inside the input box */
    bool        skaibar_active; /* input box open */
} device_pager_t;

static device_pager_t *p = NULL;

static void mic_clicked_cb(lv_event_t *e); /* defined below (skaibar section) */

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

/* Like the left instruction_list, only the item nearest the vertical centre
   shows its title; the rest fade out. Recomputed every scroll frame. */
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
        /* full title in the centre slot, quick fade outside it */
        lv_opa_t opa = (d >= ITEM_SLOT_H) ? LV_OPA_0
                       : (lv_opa_t)(LV_OPA_COVER -
                                    (uint32_t)d * LV_OPA_COVER / ITEM_SLOT_H);
        lv_obj_set_style_opa(u->item_label[i], opa, 0);
    }
}

/* Find the visible item nearest the list's vertical centre. Returns its index
   and the scroll delta that would centre it exactly (out_delta). */
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
        lv_coord_t d = item_cy - list_cy;
        if (d < 0) d = -d;
        if (d < bestd) { bestd = d; best = i; delta = list_cy - item_cy; }
    }
    if (out_delta) *out_delta = delta;
    return best;
}

/* Explicitly centre the nearest item — LVGL's snap-centre lands a few dozen px
   off with this padded-flex layout, so we drive the scroll ourselves. */
static bool s_recentering = false;
static void center_nearest(tile_ui_t *u, lv_anim_enable_t anim)
{
    lv_coord_t delta = 0;
    int idx = nearest_item(u, &delta);
    if (idx < 0 || delta == 0) return;
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
    /* Force layout so coords are valid, then centre the first instruction. */
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
        snap_to_center(LV_ANIM_OFF);
    }
    else if (want < 0)
    {
        /* Dragged right past the first device → leave the pager and return to
           the watch face (the inverse of the right-edge pull-out). Reset the
           carousel to center first so a later pull-out starts clean. */
        snap_to_center(LV_ANIM_OFF);
        LOG_I("[pager] return to watch face");
        extern void app_clock_status_bar_return_home(void);
        app_clock_status_bar_return_home();
    }
    else
    {
        /* Past the last device → no further tile; bounce back to center. */
        snap_to_center(LV_ANIM_OFF);
    }
    p->recycling = false;
}

static void make_tile(tile_ui_t *u, lv_obj_t *parent)
{
    /* Transparent full-screen tile — matches the left list's see-through bg. */
    u->tile = lv_obj_create(parent);
    lv_obj_set_size(u->tile, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_radius(u->tile, 0, 0);
    lv_obj_set_style_border_width(u->tile, 0, 0);
    lv_obj_set_style_bg_opa(u->tile, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(u->tile, 0, 0);
    lv_obj_clear_flag(u->tile, LV_OBJ_FLAG_SCROLLABLE);

    /* Device-name context header (where the left list shows time/weather). */
    u->header = lv_label_create(u->tile);
    lv_obj_set_style_text_font(u->header,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(u->header, lv_color_hex(0x00AAFF), 0);
    lv_obj_align(u->header, LV_ALIGN_TOP_MID, 0, 72); /* below the top handle */
    lv_label_set_text(u->header, "");

    /* Vertical instruction list: snap-centre scroll, item-centred title. */
    u->list = lv_obj_create(u->tile);
    lv_obj_set_size(u->list, LV_HOR_RES, LV_VER_RES);
    lv_obj_align(u->list, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(u->list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(u->list, 0, 0);
    lv_obj_set_style_pad_left(u->list, 0, 0);
    lv_obj_set_style_pad_right(u->list, 0, 0);
    /* pad so the first / last item can sit at the vertical centre */
    lv_obj_set_style_pad_top(u->list, (LV_VER_RES - ITEM_SLOT_H) / 2, 0);
    lv_obj_set_style_pad_bottom(u->list, (LV_VER_RES - ITEM_SLOT_H) / 2, 0);
    lv_obj_set_style_pad_row(u->list, 0, 0);
    lv_obj_set_flex_flow(u->list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(u->list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(u->list, LV_DIR_VER);
    /* No LVGL snap — we centre the nearest item ourselves (see center_nearest),
       because snap-centre lands off with this padded-flex layout. */
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

    /* Bottom mic / skaibar — same style as instruction_list's mic_bar. */
    u->mic_bar = lv_obj_create(u->tile);
    lv_obj_set_size(u->mic_bar, 240, 50);
    lv_obj_align(u->mic_bar, LV_ALIGN_BOTTOM_MID, 0, -75);
    lv_obj_set_style_bg_color(u->mic_bar, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(u->mic_bar, LV_OPA_50, 0);
    lv_obj_set_style_radius(u->mic_bar, 25, 0);
    lv_obj_set_style_border_width(u->mic_bar, 0, 0);
    lv_obj_set_style_pad_all(u->mic_bar, 0, 0);
    lv_obj_clear_flag(u->mic_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(u->mic_bar, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(u->mic_bar, mic_clicked_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *mic_icon = lv_img_create(u->mic_bar);
    lv_img_set_src(mic_icon, &icon_mic);
    lv_obj_align(mic_icon, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(mic_icon, LV_OBJ_FLAG_CLICKABLE);
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

/* Refresh when the bonded-device set changes (dev_add/connect/disconnect). */
static void dev_mgr_cb(dev_mgr_event_t event, uint8_t device_idx, void *ud)
{
    (void)event; (void)device_idx; (void)ud;
    refresh();
}

/* Public: re-read the device DB and rebind. Called when the right tile is
   pulled into view, so the pager always reflects the latest bonded set even
   if the device-manager callback was missed (e.g. the PC-sim fake never
   fires it; on hardware a single global cb slot can be claimed by another
   listener). Mirrors the "re-read DB on page open" pattern of the other
   device-list surfaces. */
void device_pager_refresh(void)
{
    refresh();
}

/* ---- Down-drag mouse drawer ------------------------------------------- */
/* The instruction panel (carousel + top handle) slides down to reveal the
   hosted hid_mouse layer beneath. Triggered only from the dedicated top
   handle, so it never fights the instruction list's vertical scroll. */

static void set_panel_offset(lv_coord_t y)
{
    lv_obj_set_y(p->pager, y);
    lv_obj_set_y(p->handle, y);
}

/* Build hid_mouse into the host behind the panel and point BLE HID at the
   device we're controlling (drill-down). Idempotent. */
static void mouse_layer_begin(void)
{
    if (p->mouse_created) return;
    lv_obj_clear_flag(p->mouse_host, LV_OBJ_FLAG_HIDDEN);
    hid_mouse_create(p->mouse_host);
    p->mouse_created = true;
    if (p->count > 0)
    {
        dev_page_t *d = &p->model[p->current];
        ble_dev_mgr_set_active_device(d->dev_idx);
        if (d->conn_idx != 0xFF)
            ble_hid_set_conn_idx(d->conn_idx);
        LOG_I("[pager] mouse layer -> %s (conn_idx=%d)", d->name, d->conn_idx);
    }
}

static void mouse_layer_end(void)
{
    if (!p->mouse_created) return;
    LOG_I("[pager] mouse layer end");
    hid_mouse_destroy();
    /* hid_mouse_destroy only NULLs its own pointers; the LVGL objects it built
       under the host stay alive (the gui_app path relies on the screen being
       cleared). Clear them here so the host is empty for the next reveal. */
    lv_obj_clean(p->mouse_host);
    p->mouse_created = false;
    lv_obj_add_flag(p->mouse_host, LV_OBJ_FLAG_HIDDEN);
}

static void mouse_open(void)   /* snap fully open: panel off the bottom */
{
    if (p->skaibar_input)
    {
        lv_obj_add_flag(p->skaibar_input, LV_OBJ_FLAG_HIDDEN);
        p->skaibar_active = false;
    }
    mouse_layer_begin();
    set_panel_offset(LV_VER_RES);
}

static void mouse_close(void)  /* snap closed: instruction panel back */
{
    set_panel_offset(0);
    mouse_layer_end();
}

/* Restoring the instruction list tears down the hosted mouse, but the trigger
   fires from inside hid_mouse's own release handler — destroying synchronously
   would free the objects being processed (use-after-free). Defer to the next
   LVGL tick. */
static void mouse_close_async(void *unused)
{
    (void)unused;
    if (p) mouse_close();
}

/* hid_mouse bottom-bar UP gesture, hosted. The mouse delegates the drag here so
   the instruction panel finger-follows back up over the mouse (symmetric with
   the down-drag that revealed it), instead of showing the multitask hint.
     released==0 : dragging — pull the panel up by up_px.
     released==1 : let go — commit (restore list) if pulled past the threshold,
                   else snap back to the mouse. */
static void mouse_pull_cb(int up_px, int released)
{
    if (!p || !p->mouse_created) return;
    /* We don't finger-follow the panel during the drag: the panel comes up from
       the bottom, exactly over the mouse's bottom bar (the gesture owner), which
       would break the press before release. Instead we commit/cancel on release
       — and crucially the mouse's multitask hint is suppressed (this cb is set),
       so the up-drag reads as "return to the list", not "multitask". */
    if (!released) return;
    if (up_px >= MOUSE_OPEN_THRESHOLD)
        lv_async_call(mouse_close_async, NULL); /* commit: restore the list */
    else
        set_panel_offset(LV_VER_RES);           /* cancel: stay in mouse mode */
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
        p->drag_start_y = pt.y;
        p->dragging = true;
    }
    else if (code == LV_EVENT_PRESSING && p->dragging)
    {
        lv_coord_t dy = pt.y - p->drag_start_y;
        if (dy < 0) dy = 0;
        if (dy > LV_VER_RES) dy = LV_VER_RES;
        if (dy > 10) mouse_layer_begin(); /* reveal the real mouse as we drag */
        set_panel_offset(dy);
    }
    else if ((code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) &&
             p->dragging)
    {
        p->dragging = false;
        lv_coord_t dy = pt.y - p->drag_start_y;
        if (dy >= MOUSE_OPEN_THRESHOLD) mouse_open();
        else                            mouse_close();
    }
}

/* ---- skaibar voice input -> peer-device options ---------------------- */
/* Tapping the mic opens the skaibar input box. We deliberately DO NOT start
   the watch's real voice recognition here (it hangs the PC sim); the
   "voice -> text" transcript and the "device returned options" both arrive via
   the pager_say / pager_options MSH commands (fake data). The returned options
   replace the instruction titles shown above for the current device. */

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
    /* Mic = open skaibar input only. No voice-recognition trigger. */
    if (p && p->skaibar_active) skaibar_close();
    else                        skaibar_open();
}

/* Fake "voice -> text": show the transcript in the open input box. */
void device_pager_skaibar_say(const char *text)
{
    if (!p) return;
    if (!p->skaibar_active) skaibar_open();
    lv_label_set_text(p->skaibar_label, text ? text : "");
    LOG_I("[pager] skaibar transcript: \"%s\"", text ? text : "");
}

/* Fake "peer device returned options": replace the current device's instruction
   items with these and refresh the titles shown above, then close the input. */
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

/* Public: build the pager into a tileview tile (mirrors
   lv_instruction_list_layout_create). Called from app_clock_main_status_bar_init. */
lv_obj_t *device_pager_create(lv_obj_t *parent)
{
    if (p) return p->pager; /* singleton: one right tile */
    p = (device_pager_t *)rt_calloc(1, sizeof(device_pager_t));
    if (!p) { LOG_E("device_pager alloc fail"); return NULL; }
    p->parent = parent;

    /* Mouse layer host — created first so it sits BEHIND the instruction panel.
       Empty until a down-drag reveals it (hid_mouse_create populates it). */
    p->mouse_host = lv_obj_create(parent);
    lv_obj_set_size(p->mouse_host, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->mouse_host, 0, 0);
    lv_obj_set_style_pad_all(p->mouse_host, 0, 0);
    lv_obj_set_style_border_width(p->mouse_host, 0, 0);
    lv_obj_set_style_radius(p->mouse_host, 0, 0);
    lv_obj_set_style_bg_color(p->mouse_host, lv_color_black(), 0);
    lv_obj_clear_flag(p->mouse_host, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(p->mouse_host, LV_OBJ_FLAG_HIDDEN);

    /* Instruction panel (the horizontal device carousel). */
    p->pager = lv_obj_create(parent);
    lv_obj_set_size(p->pager, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->pager, 0, 0);
    lv_obj_set_style_pad_all(p->pager, 0, 0);
    lv_obj_set_style_border_width(p->pager, 0, 0);
    lv_obj_set_style_bg_color(p->pager, lv_color_black(), 0);
    lv_obj_set_flex_flow(p->pager, LV_FLEX_FLOW_ROW);
    lv_obj_set_scroll_dir(p->pager, LV_DIR_HOR);
    lv_obj_set_scroll_snap_x(p->pager, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scrollbar_mode(p->pager, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(p->pager, pager_scroll_end_cb, LV_EVENT_SCROLL_END, NULL);

    for (int k = 0; k < 3; k++) make_tile(&p->t[k], p->pager);

    /* Top pull-down handle — the dedicated affordance that reveals the mouse
       layer. Sits in front of the panel; slides down with it during the drag. */
    p->handle = lv_obj_create(parent);
    lv_obj_set_size(p->handle, LV_HOR_RES, HANDLE_ZONE_H);
    lv_obj_set_pos(p->handle, 0, 0);
    lv_obj_set_style_bg_opa(p->handle, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(p->handle, 0, 0);
    lv_obj_set_style_pad_all(p->handle, 0, 0);
    lv_obj_set_style_radius(p->handle, 0, 0);
    lv_obj_clear_flag(p->handle, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(p->handle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(p->handle, handle_drag_cb, LV_EVENT_ALL, NULL);
    lv_obj_t *grab = lv_obj_create(p->handle);
    lv_obj_set_size(grab, 44, 5);
    lv_obj_align(grab, LV_ALIGN_TOP_MID, 0, 42);
    lv_obj_set_style_radius(grab, 3, 0);
    lv_obj_set_style_bg_color(grab, lv_color_hex(0x666666), 0);
    lv_obj_set_style_border_width(grab, 0, 0);
    lv_obj_clear_flag(grab, LV_OBJ_FLAG_CLICKABLE);

    /* skaibar input box — shown on mic tap; displays the (fake) voice
       transcript while the peer device's options are fetched. Hidden until the
       mic is pressed. Sits above the carousel near the bottom. */
    p->skaibar_input = lv_obj_create(parent);
    lv_obj_set_size(p->skaibar_input, LV_PCT(92), 84);
    lv_obj_align(p->skaibar_input, LV_ALIGN_BOTTOM_MID, 0, -110);
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

    /* Bottom-bar UP in mouse mode finger-follows the instruction panel back up
       (no multitask hint) and commits/cancels on release. */
    hid_mouse_set_host_pull_cb(mouse_pull_cb);

    ble_dev_mgr_register_callback(dev_mgr_cb, NULL);
    refresh();
    LOG_I("[pager] built on right tile (%d devices)", p->count);
    return p->pager;
}
