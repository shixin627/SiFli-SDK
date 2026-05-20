/**
 ******************************************************************************
 * @file   device_pager.c
 * @brief  Right-side per-device instruction pager — virtualized scroll-snap
 *         recycler (T3).
 *
 *  Issue 2 (2A): NOT lv_tileview (can't virtualize). A plain lv_obj with
 *  LV_SCROLL_SNAP_CENTER and exactly 3 physical tiles (prev/center/next)
 *  represents N logical devices. On scroll settle we advance the logical
 *  index, re-bind all 3 tiles from model[], and reset scroll to the center
 *  tile instantly — the classic carousel recycler. RAM constant in N.
 *
 *  Issue 5 (5A) + T3 Inc 2: each tile shows a LIGHTWEIGHT per-device
 *  instruction list (header + up to MAX_TILE_ITEMS rows). The rows are
 *  PRE-CREATED once; re-bind only set_text + show/hide + reset scroll — never
 *  create/destroy objects, so paging never hitches. The heavy AI tileview /
 *  skai voice widget is NOT per-tile: it stays single-instance for the center
 *  device (single recognizer, N transcripts) and is wired in T4 / Part 2.
 *
 *  Item data is per-device fake content for now (model[d].items[]); real
 *  per-device instruction sets arrive over the BLE protocol later. Non-phone
 *  filter is also a later refinement.
 *
 *  Build gate: BSP_USING_PC_SIMULATOR (widen to !kReleaseMode for real-hw).
 *  Launch:  goto_app device_pager   (after `dev_add A`, `dev_add B`, ...)
 ******************************************************************************
 */
#include <rtthread.h>
#include <string.h>
#include <stdio.h>
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "ble_device_manager.h"
#include "ble_hid.h"    /* ble_hid_set_conn_idx — HID re-target (Issue 1 / 1A) */
#include "hid_mouse.h"  /* hid_mouse_create/destroy — host the real mouse (T1) */
#ifdef BSP_USING_UI_HANDLER
    #include "ui_img_helper.h"
#endif

#define DBG_TAG "device.pager"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef BSP_USING_PC_SIMULATOR

#define APP_ID_DEVICE_PAGER "device_pager"
#define TILE_LEFT      0
#define TILE_CENTER    1
#define TILE_RIGHT     2
#define MAX_TILE_ITEMS 8   /* pre-created row pool per tile (Issue 5) */

/* Minimal per-device model for T3. skaibar session + mouse persist arrive
   with T4 / Part 2. */
typedef struct
{
    char    name[32];
    uint8_t dev_idx;                       /* index into bonded DB */
    char    items[MAX_TILE_ITEMS][24];     /* instruction labels (fake) */
    uint8_t item_count;
} dev_page_t;

/* One physical tile: header + a fixed pool of row objects, reused on rebind. */
typedef struct
{
    lv_obj_t *tile;
    lv_obj_t *header;
    lv_obj_t *list;
    lv_obj_t *row[MAX_TILE_ITEMS];
    lv_obj_t *row_label[MAX_TILE_ITEMS];
} tile_ui_t;

typedef struct
{
    lv_obj_t   *root;
    lv_obj_t   *pager;          /* scroll-snap container */
    tile_ui_t   t[3];           /* physical tiles: left / center / right */
    lv_obj_t   *empty_label;
    /* T4 inc 1: mouse drill-down */
    lv_obj_t   *mouse_host;     /* hosts the real hid_mouse, behind the pager */
    lv_obj_t   *reveal_strip;   /* top band of the pager: pull-down -> drill in */
    lv_obj_t   *ret_zone;       /* top band over the mouse layer: tap -> return */
    bool        revealed;       /* mouse layer is showing */
    bool        returning;      /* return slide-up anim in flight */
    dev_page_t  model[MAX_BONDED_DEVICES];
    int         count;          /* logical device count */
    int         current;        /* logical index of the centered device */
    bool        recycling;      /* re-entrancy guard for reset-to-center */
} device_pager_t;

static device_pager_t *p = NULL;

/* Fake per-device instruction items so the list visibly differs per device.
   Replaced by real per-device sets (BLE protocol) later. */
static void fake_items(dev_page_t *d)
{
    uint8_t n = (uint8_t)(3 + (d->dev_idx % 5)); /* 3..7 items, varies per device */
    if (n > MAX_TILE_ITEMS) n = MAX_TILE_ITEMS;
    d->item_count = n;
    for (uint8_t i = 0; i < n; i++)
        snprintf(d->items[i], sizeof(d->items[0]), "%s #%u", d->name, (unsigned)(i + 1));
}

/* Pull the (bonded) device set into model[]. Fake-backed on PC sim. */
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
                fake_items(d);
                p->count++;
            }
        }
    }
    if (p->current >= p->count) p->current = p->count > 0 ? p->count - 1 : 0;
}

/* Bind physical tile k (0/1/2) to logical device (current + k - 1).
   Issue 5: set_text + show/hide only — never create/destroy. */
static void bind_tile(int k)
{
    tile_ui_t  *u = &p->t[k];
    int logical = p->current + (k - TILE_CENTER);

    if (logical < 0 || logical >= p->count)
    {
        lv_label_set_text(u->header, "");
        for (int i = 0; i < MAX_TILE_ITEMS; i++)
            lv_obj_add_flag(u->row[i], LV_OBJ_FLAG_HIDDEN);
        return;
    }

    dev_page_t *d = &p->model[logical];
    lv_label_set_text_fmt(u->header, "%s   %d/%d", d->name, logical + 1, p->count);
    for (int i = 0; i < MAX_TILE_ITEMS; i++)
    {
        if (i < d->item_count)
        {
            lv_label_set_text(u->row_label[i], d->items[i]);
            lv_obj_clear_flag(u->row[i], LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(u->row[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
    lv_obj_scroll_to_y(u->list, 0, LV_ANIM_OFF); /* fresh device starts at top */
}

static void rebind_all(void)
{
    for (int k = 0; k < 3; k++) bind_tile(k);
}

static void snap_to_center(lv_anim_enable_t anim)
{
    lv_obj_scroll_to_view(p->t[TILE_CENTER].tile, anim);
}

static int centered_tile(void)
{
    lv_coord_t w = lv_obj_get_width(p->t[TILE_CENTER].tile);
    if (w <= 0) return TILE_CENTER;
    lv_coord_t sx = lv_obj_get_scroll_x(p->pager);
    return (int)((sx + w / 2) / w);
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
    }
    else
    {
        LOG_I("[pager] edge: cannot page %s end of %d devices",
              dir < 0 ? "before" : "past", p->count);
    }
    snap_to_center(LV_ANIM_OFF);
    p->recycling = false;
}

/* ---- T4 inc 1: drill-down to the real hosted mouse ------------------- *
 * Issue 1 / 1A: point the single global HID connection at the current
 * device, then host the real hid_mouse component for it. Smooth reveal
 * animation + bottom-bar-return arbitration are inc 2. */
#define REVEAL_ANIM_MS 280
static void pager_y_anim_cb(void *var, int32_t v) { lv_obj_set_y((lv_obj_t *)var, v); }

/* Slide the instruction pager DOWN off-screen to expose the mouse beneath.
   (T4 inc 2: continuous slide instead of an instant hide/show.) */
static void drill_into_mouse(void)
{
    if (!p || p->revealed || p->returning || p->count == 0) return;

    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    uint8_t conn = 0xFF;
    if (db) conn = db->devices[p->model[p->current].dev_idx].conn_idx;
    ble_hid_set_conn_idx(conn); /* PC: no-op stub; ARM: real HID target */
    LOG_I("[pager] drill -> mouse for %s (dev_idx=%u conn=%u)",
          p->model[p->current].name, p->model[p->current].dev_idx, conn);

    hid_mouse_create(p->mouse_host); /* build the real mouse UI under the host */
    lv_obj_clear_flag(p->mouse_host, LV_OBJ_FLAG_HIDDEN);     /* expose beneath */
    lv_obj_add_flag(p->reveal_strip, LV_OBJ_FLAG_HIDDEN);     /* drill trigger off */
    lv_obj_clear_flag(p->ret_zone, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(p->ret_zone);
    p->revealed = true;

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, p->pager);
    lv_anim_set_exec_cb(&a, pager_y_anim_cb);
    lv_anim_set_values(&a, 0, LV_VER_RES);
    lv_anim_set_time(&a, REVEAL_ANIM_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

/* After the slide-up completes, tear down the hosted mouse and restore the
   instruction layer. */
static void return_anim_done_cb(lv_anim_t *a)
{
    (void)a;
    if (!p) return;
    hid_mouse_destroy();         /* NULLs hid_mouse statics, stops timers */
    lv_obj_clean(p->mouse_host); /* remove the mouse UI tree it built */
    lv_obj_add_flag(p->mouse_host, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p->ret_zone, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(p->reveal_strip, LV_OBJ_FLAG_HIDDEN);
    p->returning = false;
    LOG_I("[pager] return -> instruction list");
}

/* Slide the instruction pager back UP over the mouse; destroy the mouse when
   the slide finishes (keeps it visible beneath during the animation). */
static void return_to_list(void)
{
    if (!p || !p->revealed || p->returning) return;
    p->revealed = false;
    p->returning = true;

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, p->pager);
    lv_anim_set_exec_cb(&a, pager_y_anim_cb);
    lv_anim_set_values(&a, lv_obj_get_y(p->pager), 0);
    lv_anim_set_time(&a, REVEAL_ANIM_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a, return_anim_done_cb);
    lv_anim_start(&a);
}

/* Top reveal strip on the instruction layer: start in the top band + drag
   down (vertical-dominant) -> drill into the mouse. Horizontal page swipes
   live in the pager body, below the strip, so they are unaffected. */
static lv_point_t s_reveal_start;
static bool       s_reveal_lock;
static void reveal_strip_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t pt; lv_indev_get_point(indev, &pt);
    if (code == LV_EVENT_PRESSED) { s_reveal_start = pt; s_reveal_lock = false; }
    else if (code == LV_EVENT_PRESSING && !s_reveal_lock)
    {
        int32_t dy = pt.y - s_reveal_start.y, dx = pt.x - s_reveal_start.x;
        int32_t ady = dy < 0 ? -dy : dy, adx = dx < 0 ? -dx : dx;
        if (ady + adx < 16) return;
        s_reveal_lock = true;
        if (dy > 0 && ady > adx) drill_into_mouse();
    }
}

/* inc 1 return: tap the top band over the mouse. inc 2 replaces this with the
   mouse's own bottom bar (needs the gesture broker to disambiguate). */
static void ret_zone_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) return_to_list();
}

/* Build one tile: header + a vertical list of MAX_TILE_ITEMS pre-created rows. */
static void make_tile(tile_ui_t *u, lv_obj_t *parent, uint32_t bg)
{
    u->tile = lv_obj_create(parent);
    lv_obj_set_size(u->tile, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_radius(u->tile, 0, 0);
    lv_obj_set_style_border_width(u->tile, 0, 0);
    lv_obj_set_style_bg_color(u->tile, lv_color_hex(bg), 0);
    lv_obj_set_style_pad_all(u->tile, 0, 0);
    lv_obj_clear_flag(u->tile, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(u->tile, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(u->tile, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    u->header = lv_label_create(u->tile);
    lv_obj_set_style_text_color(u->header, lv_color_hex(0x00AAFF), 0);
    lv_obj_set_style_pad_top(u->header, 60, 0);
    lv_obj_set_style_pad_bottom(u->header, 10, 0);
    lv_label_set_text(u->header, "");

    /* vertical scrollable list (nested inside the horizontal pager) */
    u->list = lv_obj_create(u->tile);
    lv_obj_set_size(u->list, LV_PCT(86), LV_PCT(64));
    lv_obj_set_style_bg_opa(u->list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(u->list, 0, 0);
    lv_obj_set_style_pad_all(u->list, 0, 0);
    lv_obj_set_style_pad_row(u->list, 8, 0);
    lv_obj_set_flex_flow(u->list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(u->list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(u->list, LV_SCROLLBAR_MODE_OFF);

    for (int i = 0; i < MAX_TILE_ITEMS; i++)
    {
        lv_obj_t *r = lv_obj_create(u->list);
        lv_obj_set_size(r, LV_PCT(100), 56);
        lv_obj_set_style_radius(r, 14, 0);
        lv_obj_set_style_bg_color(r, lv_color_hex(0x2A2A2A), 0);
        lv_obj_set_style_border_width(r, 0, 0);
        lv_obj_set_style_pad_left(r, 16, 0);
        lv_obj_clear_flag(r, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(r, LV_OBJ_FLAG_HIDDEN);

        lv_obj_t *lbl = lv_label_create(r);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 0, 0);
        lv_label_set_text(lbl, "");

        u->row[i] = r;
        u->row_label[i] = lbl;
    }
}

static void build(lv_obj_t *scr)
{
    p->root = lv_obj_create(scr);
    lv_obj_set_size(p->root, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_pad_all(p->root, 0, 0);
    lv_obj_set_style_border_width(p->root, 0, 0);
    lv_obj_set_style_bg_color(p->root, lv_color_black(), 0);
    lv_obj_clear_flag(p->root, LV_OBJ_FLAG_SCROLLABLE);

    /* mouse layer: created before the pager so it sits behind it; the real
       hid_mouse UI is built into it on drill-down (T4 inc 1). */
    p->mouse_host = lv_obj_create(p->root);
    lv_obj_set_size(p->mouse_host, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->mouse_host, 0, 0);
    lv_obj_set_style_pad_all(p->mouse_host, 0, 0);
    lv_obj_set_style_border_width(p->mouse_host, 0, 0);
    lv_obj_set_style_bg_color(p->mouse_host, lv_color_black(), 0);
    lv_obj_clear_flag(p->mouse_host, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(p->mouse_host, LV_OBJ_FLAG_HIDDEN);

    p->pager = lv_obj_create(p->root);
    lv_obj_set_size(p->pager, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->pager, 0, 0); /* known y=0 origin for the reveal slide */
    lv_obj_set_style_pad_all(p->pager, 0, 0);
    lv_obj_set_style_border_width(p->pager, 0, 0);
    lv_obj_set_flex_flow(p->pager, LV_FLEX_FLOW_ROW);
    lv_obj_set_scroll_dir(p->pager, LV_DIR_HOR);
    lv_obj_set_scroll_snap_x(p->pager, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scrollbar_mode(p->pager, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(p->pager, pager_scroll_end_cb, LV_EVENT_SCROLL_END, NULL);

    static const uint32_t bg[3] = {0x121212, 0x161616, 0x121212};
    for (int k = 0; k < 3; k++) make_tile(&p->t[k], p->pager, bg[k]);

    /* top reveal strip (drag down -> drill into mouse) on top of the pager */
    p->reveal_strip = lv_obj_create(p->root);
    lv_obj_set_size(p->reveal_strip, LV_HOR_RES, 40);
    lv_obj_set_pos(p->reveal_strip, 0, 0);
    lv_obj_set_style_bg_opa(p->reveal_strip, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(p->reveal_strip, 0, 0);
    lv_obj_add_flag(p->reveal_strip, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(p->reveal_strip, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(p->reveal_strip, reveal_strip_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(p->reveal_strip, reveal_strip_cb, LV_EVENT_PRESSING, NULL);

    /* return zone over the mouse layer (tap -> back to list); hidden until reveal.
       Faint tint so it's findable in testing; inc 2 swaps this for the mouse's
       own bottom bar via the gesture broker. */
    p->ret_zone = lv_obj_create(p->root);
    lv_obj_set_size(p->ret_zone, LV_HOR_RES, 40);
    lv_obj_set_pos(p->ret_zone, 0, 0);
    lv_obj_set_style_bg_color(p->ret_zone, lv_color_hex(0x00AAFF), 0);
    lv_obj_set_style_bg_opa(p->ret_zone, LV_OPA_30, 0);
    lv_obj_set_style_border_width(p->ret_zone, 0, 0);
    lv_obj_add_flag(p->ret_zone, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(p->ret_zone, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(p->ret_zone, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(p->ret_zone, ret_zone_cb, LV_EVENT_CLICKED, NULL);

    p->empty_label = lv_label_create(p->root);
    lv_label_set_text(p->empty_label, "No devices\n(dev_add A, then re-open)");
    lv_obj_set_style_text_color(p->empty_label, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_align(p->empty_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(p->empty_label);
}

static void refresh(void)
{
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
    LOG_I("[pager] up: %d devices, current=%d (swipe to page)", p->count, p->current + 1);
}

static void on_start(lv_obj_t *scr)
{
    RT_ASSERT(p == NULL);
    p = (device_pager_t *)rt_calloc(1, sizeof(device_pager_t));
    if (!p) { LOG_E("device_pager alloc fail"); return; }
    build(scr);
    refresh();
}

static void on_stop(void)
{
    if (p)
    {
        if (p->pager) lv_anim_del(p->pager, NULL); /* stop in-flight slide anim */
        if (p->revealed || p->returning) hid_mouse_destroy(); /* tear down hosted mouse */
        if (p->root) lv_obj_del(p->root);
        rt_free(p);
        p = NULL;
    }
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART: on_start(lv_scr_act()); break;
    case GUI_APP_MSG_ONRESUME: if (p) refresh(); break;
    case GUI_APP_MSG_ONSTOP:  on_stop(); break;
    default: break;
    }
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_DEVICE_PAGER, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(flashlight), IMG_FLASHLIGHT, APP_ID_DEVICE_PAGER, app_main);

#endif /* BSP_USING_PC_SIMULATOR */
