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
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "lvgl.h"
#include "ble_device_manager.h"
#include "ble_hid.h"                   /* ble_hid_set_conn_idx (drill-down target) */
#include "ui_helper.h"                 /* get_system_font_size */
#include "lv_ext_resource_manager.h"   /* LV_EXT_FONT_GET font idiom */
#include "hid_mouse.h"                 /* mouse component, hosted as the base */
#include "arc_scroll.h"                 /* right-band arc scroll (same as left list) */
#include "ui_handler.h"                 /* lvgl_msg_handler (v2t transcript route) */
#include "watch_system_interact.h"      /* V2T_INTENT_SKAIBAR */
#ifdef BSP_USING_BLOC
#include "bloc_v2t.h"                   /* voice_provider.start_v2t (real ASR) */
#endif

/* Transcript-return router (app_system_interface.c) — on real hardware the v2t
   pipeline calls this with the recognised text; it now branches to the device
   skaibar when this page is the one showing (see device_pager_skaibar_is_open). */
extern void refresh_ai_chat_input_message(char *text);

#define DBG_TAG "device.pager"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

LV_IMG_DECLARE(icon_mic); /* shared mic/voice icon, same as instruction_list */
LV_IMG_DECLARE(message_widget_bg); /* skaibar input pill frame (same as left list) */
LV_IMG_DECLARE(img_flashlight); /* placeholder per-item icon (same look as left list) */

#define TILE_LEFT      0
#define TILE_CENTER    1
#define TILE_RIGHT     2
#define MAX_TILE_ITEMS 10 /* pre-created icon pool per tile */
#define MAX_DEVICES    8  /* online devices the peer (phone) can report */

#define BAR_H          44  /* bottom re-summon handle thickness */

/* Circular (arc) item layout — mirrors the left instruction_list / app_exercise
   apply_circular_layout: icons ride a vertical arc bulging right, the centred
   one zoomed big, neighbours shrink + fade, items past ±90° hidden. Scrolled by
   the right-band arc_scroll (snap_targets give the scroll range; the floating
   icons are positioned each scroll frame from scroll_y). */
#define ARC_RADIUS     175
#define ARC_SLOT_DEG   36.0f       /* angle between adjacent items */
#define ARC_ICON_SIZE  80          /* img_flashlight native size */
#define ARC_SLOT_H     90          /* vertical scroll pitch per item */
#define ARC_ZOOM_MIN   102         /* 0.4x at the edges  (was 128 / 0.5x — shrunk to 80%) */
#define ARC_ZOOM_CTR   256         /* 1.0x at the centre (was 320 / 1.25x — shrunk to 80%) */
#define ARC_OPA_MIN    LV_OPA_40
#define ARC_PULL_MOUSE_PX 55       /* elastic over-pull past the top item → mouse */

typedef struct
{
    uint32_t id;        /* device id reported by the connected peer (phone) */
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
    lv_obj_t *list;                       /* scroll container (driven by arc_scroll) */
    lv_obj_t *snap[MAX_TILE_ITEMS];       /* invisible scroll/snap anchors */
    lv_obj_t *item_icon[MAX_TILE_ITEMS];  /* floating icons, positioned on the arc */
    lv_obj_t *name_label;                 /* single centred label = selected item's name */
    arc_scroll_handle_t *arc;             /* right-band drag → scrolls this list */
    dev_page_t *dev;                      /* device this tile currently shows */
    int       sel;                        /* index currently at the arc centre */
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
    lv_obj_t   *bar;             /* bottom re-summon touch zone (invisible, on the mouse) */
    lv_obj_t   *grabber;         /* visible drawer handle on lv_layer_top — rides up with the reveal */
    lv_obj_t   *empty_label;
    bool        mouse_created;   /* real hid_mouse hosted in mouse_base */
    bool        summoning;       /* bar press shown the overlay; awaiting drag/tap */
    bool        pull_pending;    /* over-pull at the top item → going to mouse */
    bool        host_pulling;    /* bar/header drag is driving a live (bidirectional) reveal */
    lv_coord_t  bar_press_y;     /* y where the drag started (for the reveal delta) */
    lv_coord_t  host_scroll0;    /* overlay scroll_y at press (0 from the mouse bar, 466 from the list header) */
    bool        bar_from_mouse;  /* this drag started on the mouse page (bottom bar), not the list header */
    bool        bar_moved;       /* the finger moved enough to count as a drag, not a tap */

    dev_page_t  model[MAX_DEVICES];
    int         count;
    int         current;
    bool        recycling;
    int         pager_anim_dir;  /* device-switch slide direction, recycled when the anim ends */

    /* skaibar voice input → peer-device options */
    lv_obj_t   *mic_bar;         /* bottom mic trigger; morphs into the skaibar */
    lv_obj_t   *mic_icon;        /* mic glyph (faded during the morph) */
    lv_obj_t   *skaibar_input;
    lv_obj_t   *skaibar_frame;   /* message_widget_bg image (faded during morph) */
    lv_obj_t   *skaibar_label;
    bool        skaibar_active;
} device_pager_t;

static device_pager_t *p = NULL;

static void mic_clicked_cb(lv_event_t *e);   /* defined below (skaibar section) */
static void mouse_retarget(void);            /* defined below */
static void skaibar_close(void);             /* defined below (skaibar section) */
static void skaibar_open(void);              /* defined below (skaibar section) */

/* Hide the skaibar input box as soon as a list scroll begins (mirrors the left
   instruction_list, where scrolling dismisses the AI widget). */
static void scroll_hides_skaibar_cb(lv_event_t *e)
{
    (void)e;
    if (p && p->skaibar_active) skaibar_close();
}

/* Fake online-device list standing in for what the connected peer (phone) will
   later stream in one device at a time (online device + id + its item options).
   NOT sourced from the BLE bonded-device DB. 3 devices, random ids, 3 items each
   (every item uses the flashlight placeholder icon). Seeded once at create. */
static void seed_fake_devices(void)
{
    p->count = 0;
    for (int i = 0; i < 3; i++)
    {
        dev_page_t *d = &p->model[p->count];
        d->id       = (uint32_t)rand();
        snprintf(d->name, sizeof(d->name), "Dev %08lX", (unsigned long)d->id);
        d->dev_idx  = (uint8_t)i;
        d->conn_idx = 0xFF;          /* not a real BLE HID target (fake data) */
        d->item_count = 3;
        for (int j = 0; j < 3; j++)
            snprintf(d->items[j], sizeof(d->items[0]), "Item %d", j + 1);
        p->count++;
    }
    p->current = 0;
}

/* Lay the item icons on a right-bulging vertical arc (same look as the left
   instruction_list / app_exercise): item `sel` sits at the centre (3 o'clock,
   zoomed big), neighbours curve up/down shrinking + fading, items past ±90°
   hide. The single name_label shows the centred item's text. */
/* scroll_y at which item 0 sits at the arc centre. TILE-LOCAL (matches apply_arc,
   which positions the icons assuming the tile is at the viewport origin) — i.e.
   position-INDEPENDENT, so the anchor is correct even when the list_tile is still
   off-screen (e.g. computed mid-reveal while pulling back from the mouse). When the
   list IS on screen its coords.y1 is 0, so this is identical to the old value. */
static int32_t arc_base_scroll(tile_ui_t *u)
{
    const lv_coord_t pt = lv_obj_get_style_pad_top(u->list, LV_PART_MAIN);
    return pt + ARC_ICON_SIZE / 2 - LV_VER_RES / 2;
}

/* Centre name label: slide a freshly-selected item's text in from above, the way
   the left instruction_list's labels scroll in (animate_label_vertical). The label
   rests at LV_ALIGN_LEFT_MID +24; the anim drives its y offset -NAME_SLIDE_PX → 0. */
#define NAME_SLIDE_PX 26
#define NAME_SLIDE_MS 200
static void name_label_set_y(lv_obj_t *obj, lv_coord_t y)
{
    lv_obj_align(obj, LV_ALIGN_LEFT_MID, 24, y);
}

static void apply_arc(tile_ui_t *u)
{
    dev_page_t *d = u->dev;
    int count = d ? d->item_count : 0;
    const int   cx   = LV_HOR_RES / 2;
    const int   cy   = LV_VER_RES / 2;
    const float aps  = ARC_SLOT_DEG * (float)M_PI / 180.0f;
    /* Position icons in TILE-LOCAL coords (tile assumed at the viewport origin
       when shown) — NOT the list's live screen coords. The 3-tile carousel
       binds off-screen neighbours too; using live coords there placed their
       floating icons wrong, so they were misaligned once paged into view. */
    const lv_coord_t pl  = lv_obj_get_style_pad_left(u->list, LV_PART_MAIN);
    const lv_coord_t pt  = lv_obj_get_style_pad_top(u->list, LV_PART_MAIN);

    const int32_t base_scroll = arc_base_scroll(u);
    lv_coord_t scroll_y = lv_obj_get_scroll_y(u->list);
    const int32_t overshoot   = ARC_SLOT_H / 2;
    const int32_t scroll_max  = base_scroll + (count > 0 ? (count - 1) : 0) * ARC_SLOT_H;
    if (scroll_y < base_scroll - overshoot) scroll_y = base_scroll - overshoot;
    if (scroll_y > scroll_max + overshoot)  scroll_y = scroll_max + overshoot;
    const float offset_angle = (float)(scroll_y - base_scroll) / ARC_SLOT_H * aps;

    int   closest = 0;
    float min_abs = (float)M_PI;
    for (int i = 0; i < MAX_TILE_ITEMS; i++)
    {
        if (i >= count) { lv_obj_add_flag(u->item_icon[i], LV_OBJ_FLAG_HIDDEN); continue; }
        float ang = (float)i * aps - offset_angle;
        float aa  = fabsf(ang);
        if (aa < min_abs) { min_abs = aa; closest = i; }
        if (aa > (float)M_PI / 2.0f)
        {
            lv_obj_add_flag(u->item_icon[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(u->item_icon[i], LV_OBJ_FLAG_HIDDEN);
        float c = cosf(aa);
        uint16_t zoom = (uint16_t)(ARC_ZOOM_MIN + (ARC_ZOOM_CTR - ARC_ZOOM_MIN) * c);
        lv_img_set_zoom(u->item_icon[i], zoom);
        int sx = cx + (int)(ARC_RADIUS * cosf(ang));
        int sy = cy + (int)(ARC_RADIUS * sinf(ang));
        lv_coord_t iw = lv_obj_get_width(u->item_icon[i]);
        lv_coord_t ih = lv_obj_get_height(u->item_icon[i]);
        lv_obj_set_pos(u->item_icon[i], sx - pl - iw / 2, sy - pt - ih / 2);
        lv_opa_t opa = (lv_opa_t)(ARC_OPA_MIN + (LV_OPA_COVER - ARC_OPA_MIN) * c);
        lv_obj_set_style_img_opa(u->item_icon[i], opa, 0);
    }
    int prev_sel = u->sel;
    u->sel = closest;
    if (count > 0 && min_abs <= (float)M_PI / 2.0f)
    {
        lv_label_set_text(u->name_label, d->items[closest]);
        if (closest != prev_sel) /* item changed → slide the new text in, matching scroll dir */
        {
            /* mirror the icon's motion: next item (index up) rose from BELOW → enter
               from the bottom; previous item (index down) came from ABOVE → from the top. */
            lv_coord_t from_y = (closest > prev_sel) ? NAME_SLIDE_PX : -NAME_SLIDE_PX;
            lv_anim_del(u->name_label, (lv_anim_exec_xcb_t)name_label_set_y);
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, u->name_label);
            lv_anim_set_values(&a, from_y, 0);
            lv_anim_set_time(&a, NAME_SLIDE_MS);
            lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
            lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)name_label_set_y);
            lv_anim_start(&a);
        }
    }
    else
    {
        lv_anim_del(u->name_label, (lv_anim_exec_xcb_t)name_label_set_y);
        name_label_set_y(u->name_label, 0);
        lv_label_set_text(u->name_label, "");
    }
}

/* arc_scroll snap target: the invisible anchor for the centred item. */
static lv_obj_t *arc_snap_cb(void *ctx)
{
    tile_ui_t *u = (tile_ui_t *)ctx;
    if (!u || u->sel < 0 || u->sel >= MAX_TILE_ITEMS) return NULL;
    return u->snap[u->sel];
}

/* Deferred: pulling the top item down far enough reveals the mouse — drive the
   overlay back to its transparent HOME tile (its VALUE_CHANGED then hides the
   overlay and shows the mouse + bar). Deferred so we don't retile mid-scroll. */
static void pull_to_mouse_async(void *unused)
{
    (void)unused;
    if (!p) return;
    lv_obj_set_tile_id(p->overlay, 0, 0, LV_ANIM_ON);
}

static void list_scroll_cb(lv_event_t *e)
{
    tile_ui_t *u = (tile_ui_t *)lv_event_get_user_data(e);
    if (!u || !u->dev) return;
    /* Scrolling/switching items dismisses the skaibar (mirrors the left list).
       Handled here (not SCROLL_BEGIN) so the arc-band scroll, which fires only
       LV_EVENT_SCROLL via scroll_by_raw, is covered too. */
    if (p->skaibar_active) skaibar_close();
    apply_arc(u);
    /* Over-pull past the first item (elastic overshoot below the top) → mouse. */
    if (u == &p->t[TILE_CENTER] && !p->pull_pending)
    {
        lv_coord_t sy   = lv_obj_get_scroll_y(u->list);
        int32_t    base = arc_base_scroll(u);
        if (sy < base - ARC_PULL_MOUSE_PX)
        {
            p->pull_pending = true;
            lv_async_call(pull_to_mouse_async, NULL);
        }
    }
}

/* On release, settle on the nearest item (centre it). Covers both the native
   centre-drag and the arc-band scroll — neither reliably centres on its own. */
static void list_scroll_end_cb(lv_event_t *e)
{
    tile_ui_t *u = (tile_ui_t *)lv_event_get_user_data(e);
    if (!u || !u->dev || p->pull_pending) return;
    int count = u->dev->item_count;
    if (count <= 0) return;
    int32_t    base = arc_base_scroll(u);
    lv_coord_t sy   = lv_obj_get_scroll_y(u->list);
    int idx = (int)lroundf((float)(sy - base) / (float)ARC_SLOT_H);
    if (idx < 0) idx = 0;
    if (idx >= count) idx = count - 1;
    lv_coord_t target = (lv_coord_t)(base + idx * ARC_SLOT_H);
    if (abs((int)(sy - target)) > 2)            /* not already settled → snap */
        lv_obj_scroll_to_y(u->list, target, LV_ANIM_ON);
}

static void bind_tile(int k)
{
    tile_ui_t  *u = &p->t[k];
    int logical = p->current + (k - TILE_CENTER);

    if (logical < 0 || logical >= p->count)
    {
        u->dev = NULL;
        lv_label_set_text(u->header, "");
        lv_label_set_text(u->name_label, "");
        for (int i = 0; i < MAX_TILE_ITEMS; i++)
        {
            lv_obj_add_flag(u->item_icon[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(u->snap[i], LV_OBJ_FLAG_HIDDEN);
        }
        if (u->arc) arc_scroll_set_item_count(u->arc, 0);
        return;
    }
    dev_page_t *d = &p->model[logical];
    u->dev = d;
    lv_label_set_text_fmt(u->header, "%s   %d/%d", d->name, logical + 1, p->count);
    /* Show one snap anchor per item so the arc_scroll range matches item_count. */
    for (int i = 0; i < MAX_TILE_ITEMS; i++)
    {
        if (i < d->item_count) lv_obj_clear_flag(u->snap[i], LV_OBJ_FLAG_HIDDEN);
        else                   lv_obj_add_flag(u->snap[i], LV_OBJ_FLAG_HIDDEN);
    }
    if (u->arc) arc_scroll_set_item_count(u->arc, d->item_count);
    /* Start at the LAST (bottom-most) item; layout must be current for the math. */
    int last = d->item_count > 0 ? d->item_count - 1 : 0;
    u->sel = last;
    lv_obj_update_layout(u->list);
    lv_obj_scroll_to_y(u->list, arc_base_scroll(u) + last * ARC_SLOT_H, LV_ANIM_OFF);
    apply_arc(u);
}

static void rebind_all(void) { for (int k = 0; k < 3; k++) bind_tile(k); }

static void snap_to_center(lv_anim_enable_t anim)
{
    lv_obj_scroll_to_view(p->t[TILE_CENTER].tile, anim);
}

/* Limit the carousel's scroll directions so that swiping toward a non-existent
   neighbour chains to the outer watch-face tileview instead of dead-scrolling:
   on the FIRST device a right-swipe (toward "previous") chains back to the watch
   face (whole page + mouse finger-follow out); on the LAST device a left-swipe
   (toward "next") chains away too. Middle devices scroll both ways between devices. */
static void update_pager_scroll_dir(void)
{
    if (!p || !p->pager) return;
    lv_dir_t dir = LV_DIR_HOR;
    bool first = (p->current <= 0);
    bool last  = (p->current >= p->count - 1);
    /* LV_DIR_RIGHT allows the left-swipe (→ next device) but blocks the right-swipe
       (→ "previous"), so on the first device the right-swipe chains to the watch
       face. LV_DIR_LEFT is the mirror for the last device. */
    if (first && last)      dir = LV_DIR_RIGHT;         /* single device: only the home chain */
    else if (first)         dir = LV_DIR_RIGHT;         /* block "previous" → chains to watch face */
    else if (last)          dir = LV_DIR_LEFT;          /* block "next" */
    lv_obj_set_scroll_dir(p->pager, dir);
}

static int centered_tile(void)
{
    lv_coord_t w = lv_obj_get_width(p->t[TILE_CENTER].tile);
    if (w <= 0) return TILE_CENTER;
    return (int)((lv_obj_get_scroll_x(p->pager) + w / 2) / w);
}

/* Per-frame scroll setter for the settle animation. */
static void pager_settle_exec_cb(void *obj, int32_t v)
{
    lv_obj_scroll_to_x((lv_obj_t *)obj, v, LV_ANIM_OFF);
}

/* Runs when the slide-to-centre animation finishes: recycle to the new current
   device. rebind_all + snap_to_center happen in the SAME callback (no render in
   between), so the instant re-centre is invisible — the centred device stays put. */
static void pager_settle_ready_cb(lv_anim_t *a)
{
    (void)a;
    if (!p) return;
    /* p->recycling was set true when the slide started, guarding the SCROLL_END
       that fires as the slide lands on the neighbour. */
    int want = p->current + p->pager_anim_dir;
    if (want >= 0 && want < p->count)
    {
        p->current = want;
        rebind_all();
        if (p->mouse_created) mouse_retarget();
        LOG_I("[pager] -> device %d/%d (%s)", p->current + 1, p->count,
              p->model[p->current].name);
    }
    snap_to_center(LV_ANIM_OFF);
    update_pager_scroll_dir();
    p->recycling = false;
}

static void pager_scroll_end_cb(lv_event_t *e)
{
    (void)e;
    if (!p || p->recycling) return;
    int c = centered_tile();
    if (c == TILE_CENTER) return;            /* snapped back — LVGL animated it */
    int dir  = c - TILE_CENTER;
    int want = p->current + dir;
    if (want < 0)                            /* swiped before the first → leave to watch face */
    {
        /* Slide the whole page (the mouse base included) out to the watch face.
           Don't tear the mouse down here — let it ride out with the slide; the main
           tileview's settle (VALUE_CHANGED on the watch-face tile) calls
           device_pager_set_active(false) once the page is gone. */
        snap_to_center(LV_ANIM_OFF);
        extern void app_clock_status_bar_return_home(void);
        app_clock_status_bar_return_home();
        return;
    }
    if (want >= p->count)                    /* past the last → just animate back to centre */
    {
        snap_to_center(LV_ANIM_ON);
        return;
    }
    /* Slide the neighbour fully to centre with an animation, THEN recycle on
       finish. Capture the target scroll_x via a momentary scroll-to-view (no
       render happens between these synchronous calls, so it's invisible).
       Guard re-entry now: a SCROLL_END fires again as the slide lands. */
    p->pager_anim_dir = dir;
    p->recycling = true;
    lv_coord_t cur = lv_obj_get_scroll_x(p->pager);
    lv_obj_scroll_to_view(p->t[c].tile, LV_ANIM_OFF);
    lv_coord_t target = lv_obj_get_scroll_x(p->pager);
    lv_obj_scroll_to_x(p->pager, cur, LV_ANIM_OFF);
    if (target == cur) { pager_settle_ready_cb(NULL); return; } /* already there */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, p->pager);
    lv_anim_set_values(&a, cur, target);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, pager_settle_exec_cb);
    lv_anim_set_ready_cb(&a, pager_settle_ready_cb);
    lv_anim_start(&a);
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

    /* Scroll container — full-screen, vertically scrollable like the left list:
       drag ANYWHERE scrolls the items; the right-band arc_scroll is an extra
       one-handed affordance on the same list. scroll_dir VER means horizontal
       drags chain to the device pager, and dragging down past the first item
       chains to the overlay tileview (pull-to-mouse). */
    u->list = lv_obj_create(u->tile);
    lv_obj_set_size(u->list, LV_HOR_RES, LV_VER_RES);
    lv_obj_align(u->list, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(u->list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(u->list, 0, 0);
    lv_obj_set_style_pad_left(u->list, 0, 0);
    lv_obj_set_style_pad_right(u->list, 0, 0);
    lv_obj_set_style_pad_ver(u->list, LV_VER_RES / 2, 0);
    lv_obj_set_scrollbar_mode(u->list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(u->list, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(u->list, LV_SCROLL_SNAP_CENTER);
    /* Vertical over-pull at the top item should rubber-band (so we can detect it
       and reveal the mouse), NOT chain into the nested pager/overlay. Horizontal
       still chains to the device pager. */
    lv_obj_clear_flag(u->list, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_add_event_cb(u->list, list_scroll_cb, LV_EVENT_SCROLL, u);
    lv_obj_add_event_cb(u->list, list_scroll_end_cb, LV_EVENT_SCROLL_END, u);

    for (int i = 0; i < MAX_TILE_ITEMS; i++)
    {
        /* Invisible snap/scroll anchor (non-floating → defines the scroll range
           + arc_scroll's snap targets). */
        lv_obj_t *sn = lv_obj_create(u->list);
        lv_obj_set_size(sn, ARC_ICON_SIZE, ARC_ICON_SIZE);
        lv_obj_set_pos(sn, 0, i * ARC_SLOT_H);
        lv_obj_set_style_bg_opa(sn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(sn, 0, 0);
        lv_obj_set_style_pad_all(sn, 0, 0);
        lv_obj_clear_flag(sn, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(sn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(sn, LV_OBJ_FLAG_HIDDEN);
        u->snap[i] = sn;

        /* Floating icon (doesn't affect scroll size); positioned on the arc by
           apply_arc(). Pivot + OVERFLOW_VISIBLE so the zoom isn't clipped. */
        lv_obj_t *icon = lv_img_create(u->list);
        lv_img_set_src(icon, &img_flashlight);  /* 80x80 placeholder */
        lv_img_set_pivot(icon, ARC_ICON_SIZE / 2, ARC_ICON_SIZE / 2);
        lv_obj_add_flag(icon, LV_OBJ_FLAG_FLOATING);
        lv_obj_add_flag(icon, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
        lv_obj_clear_flag(icon, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(icon, LV_OBJ_FLAG_HIDDEN);
        u->item_icon[i] = icon;
    }

    /* Single centred name label — shows the arc-centred item's text. */
    u->name_label = lv_label_create(u->tile);
    lv_obj_set_style_text_font(u->name_label,
                               LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(u->name_label, lv_color_white(), 0);
    lv_label_set_long_mode(u->name_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(u->name_label, 200);
    lv_obj_set_style_text_align(u->name_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(u->name_label, LV_ALIGN_LEFT_MID, 24, 0);
    lv_label_set_text(u->name_label, "");
    u->sel = 0;

    /* Right-band arc scroll — drag in the right arc band scrolls the items;
       lock_ancestors keeps the overlay tileview from stealing the drag (this
       page is a tileview child, like the left instruction_list). */
    arc_scroll_config_t cfg = {
        .parent          = u->tile,
        .list            = u->list,
        .slot_height_px  = ARC_SLOT_H,
        .item_height_px  = ARC_ICON_SIZE,
        .slot_angle_deg  = (uint16_t)ARC_SLOT_DEG,
        .item_count      = 0,
        .band_thickness  = 90,
        .lock_ancestors  = true,
        .snap_cb         = arc_snap_cb,
        .ctx             = u,
    };
    u->arc = arc_scroll_create(&cfg);
}

static void refresh(void)
{
    if (!p) return;
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
    update_pager_scroll_dir();
}

/* Public: rebind the (peer-reported) device list and refresh (called on reveal). */
void device_pager_refresh(void)
{
    refresh();
}

/* Drill-down to the device currently shown. The fake list isn't BLE-bonded
   (conn_idx 0xFF), so HID retarget is skipped; the real peer-device target is
   wired when the phone-streamed list replaces the fake seed. */
static void mouse_retarget(void)
{
    if (!p || p->count == 0) return;
    dev_page_t *d = &p->model[p->current];
    if (d->conn_idx != 0xFF)
    {
        ble_dev_mgr_set_active_device(d->dev_idx);
        ble_hid_set_conn_idx(d->conn_idx);
    }
    LOG_I("[pager] target -> %s (id=%08lX)", d->name, (unsigned long)d->id);
}

/* ---- overlay show / hide (watch-face style) -------------------------- */

#define GRABBER_REST_Y (-20)  /* visible handle's resting offset from the screen bottom */

/* As the list is pulled in over the mouse (overlay scrolls HOME→LIST), ride the
   visible grabber up with it. The grabber lives on lv_layer_top so it is never
   covered by the trackpad or the rising list; p->bar (below) still drives the
   actual finger-follow. overlay scroll_y is 0 at the mouse and grows as the list
   rises, so the handle climbs from its resting spot toward the top. */
static void bar_follow_reveal_cb(lv_event_t *e)
{
    (void)e;
    if (!p || !p->grabber || !lv_obj_is_valid(p->grabber)) return;
    lv_coord_t sy = lv_obj_get_scroll_y(p->overlay);
    if (sy < 0) sy = 0;
    lv_obj_align(p->grabber, LV_ALIGN_BOTTOM_MID, 0, GRABBER_REST_Y - sy);
}

/* Settled on a tile: HOME(transparent) → hide the overlay, reveal mouse + bar;
   LIST → keep the overlay shown, hide the bar. */
static void overlay_value_changed_cb(lv_event_t *e)
{
    (void)e;
    if (!p) return;
    if (p->host_pulling) return; /* mid live drag — don't settle yet */
    /* A scroll-settle resolves any in-flight bar summon. */
    p->summoning = false;
    lv_obj_t *act = lv_tileview_get_tile_act(p->overlay);
    if (act == p->home_tile)
    {
        /* Settled on the transparent HOME → reveal the mouse + bar + grabber. */
        lv_obj_add_flag(p->overlay, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(p->bar, LV_OBJ_FLAG_HIDDEN);
        if (p->grabber)
        {
            lv_obj_align(p->grabber, LV_ALIGN_BOTTOM_MID, 0, GRABBER_REST_Y);
            lv_obj_clear_flag(p->grabber, LV_OBJ_FLAG_HIDDEN);
        }
        LOG_I("[pager] overlay hidden — mouse base + bar");
    }
    else
    {
        /* Settled on the LIST → hide the bar + grabber; the mouse stays visible
           behind the (transparent) list. */
        lv_obj_add_flag(p->bar, LV_OBJ_FLAG_HIDDEN);
        if (p->grabber) lv_obj_add_flag(p->grabber, LV_OBJ_FLAG_HIDDEN);
        p->pull_pending = false;
        LOG_I("[pager] overlay on LIST — bar hidden");
    }
}

/* Live drawer drag, shared by the mouse-page bottom bar AND the list-page device-name
   handle. It KEEPS its press (no PRESS_LOCK clear) and drives the overlay reveal
   itself so the list finger-follows BOTH ways. PRESS records where the overlay was
   (0 = parked at the mouse for the bottom bar; 466 = the list for the name handle);
   PRESSING scrolls the overlay by how far the finger moved from the press point
   (toward the list when dragging up, back toward the mouse when dragging down — live);
   RELEASE snaps to whichever tile is nearer. The visible grabber rides along via
   bar_follow_reveal_cb. The overlay's settle handler is suppressed mid-drag. */
static void bar_cb(lv_event_t *e)
{
    if (!p) return;
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (code == LV_EVENT_PRESSED)
    {
        lv_point_t pt;
        if (indev) lv_indev_get_point(indev, &pt); else pt.y = 0;
        p->bar_press_y = pt.y;
        p->host_pulling = true;
        p->bar_moved = false;
        if (lv_obj_has_flag(p->overlay, LV_OBJ_FLAG_HIDDEN))
        {
            /* mouse page: park at HOME and reveal upward from 0 */
            p->bar_from_mouse = true;
            device_pager_refresh();
            lv_obj_set_tile_id(p->overlay, 0, 0, LV_ANIM_OFF);
            lv_obj_scroll_to_y(p->overlay, 0, LV_ANIM_OFF);
            lv_obj_clear_flag(p->overlay, LV_OBJ_FLAG_HIDDEN);
            p->host_scroll0 = 0;
        }
        else
        {
            /* list page (name handle): start from the current reveal (≈ LIST) */
            p->bar_from_mouse = false;
            p->host_scroll0 = lv_obj_get_scroll_y(p->overlay);
        }
        /* Show the drawer handle right away so it rides in/out during the live drag
           (from the list it starts off-screen above and slides down as we retract). */
        if (p->grabber)
        {
            lv_obj_align(p->grabber, LV_ALIGN_BOTTOM_MID, 0,
                         GRABBER_REST_Y - lv_obj_get_scroll_y(p->overlay));
            lv_obj_clear_flag(p->grabber, LV_OBJ_FLAG_HIDDEN);
        }
    }
    else if (code == LV_EVENT_PRESSING)
    {
        if (!p->host_pulling || !indev) return;
        lv_point_t pt;
        lv_indev_get_point(indev, &pt);
        int dy = p->bar_press_y - pt.y;                     /* up = positive */
        if (dy > 10 || dy < -10) p->bar_moved = true;       /* moved enough → a drag, not a tap */
        int sy = p->host_scroll0 + dy;                      /* up → toward list, down → toward mouse */
        if (sy < 0) sy = 0;
        if (sy > LV_VER_RES) sy = LV_VER_RES;
        lv_obj_scroll_to_y(p->overlay, sy, LV_ANIM_OFF);    /* live, both directions */
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
    {
        if (!p->host_pulling) return;
        p->host_pulling = false;
        if (p->bar_from_mouse && !p->bar_moved)
        {
            /* Plain TAP on the mouse-page bar (no drag): jump straight to the device
               list AND open voice input. */
            lv_obj_set_tile_id(p->overlay, 0, 1, LV_ANIM_ON);
            skaibar_open();
            return;
        }
        lv_coord_t sy = lv_obj_get_scroll_y(p->overlay);
        if (sy > LV_VER_RES / 2)
            lv_obj_set_tile_id(p->overlay, 0, 1, LV_ANIM_ON); /* nearer the LIST → commit */
        else
            lv_obj_set_tile_id(p->overlay, 0, 0, LV_ANIM_ON); /* nearer the mouse → snap back */
    }
}

/* Public: host / tear down the real mouse base as the device page is
   entered / left.

   NOTE: the mouse is built HERE (at settle), not during the reveal drag.
   Building hid_mouse mid-drag — even just its UI, even deferred via
   lv_async_call — mutates the object tree enough to abort the main tileview's
   active gesture, so the page snaps fully open instead of finger-following
   (verified in the sim). And pre-building it permanently to dodge that would
   collide with the standalone APP_ID_MOUSE app, which shares hid_mouse's
   file-static singleton. So the mouse appears as the page settles. */
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
            /* The mouse can't be built during the reveal drag (would abort
               finger-follow), so soften its settle appearance: fade the scroll
               wheel's tick nodes in from black via colour (hardware-safe, no
               layer opacity). */
            hid_mouse_fade_in_scroll_wheel();
            LOG_I("[pager] device page active — mouse base hosted");
        }
        device_pager_refresh();
        /* Enter on the LIST (the mouse shows through behind it); pull down to reveal
           the full mouse. */
        p->pull_pending = false;
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
        if (p->grabber) lv_obj_add_flag(p->grabber, LV_OBJ_FLAG_HIDDEN);
        LOG_I("[pager] device page inactive — mouse base torn down");
    }
}

/* ---- skaibar voice input -> peer-device options ---------------------- */
#define SKAIBAR_GROW_MS  220   /* phase 1: mic button grows into the box backdrop */
#define SKAIBAR_FRAME_MS 160   /* phase 2: frame image + transcript fade */
#define MICB_W 100             /* slim home-indicator pill (matches the trackpad's former bottom bar) */
#define MICB_H 16
#define MICB_Y (-20)
#define SKAIB_W 442            /* input-box geometry (matches skaibar_input) */
#define SKAIB_H 252
#define SKAIB_Y (75)           /* open box sits 75px below bottom edge — bottom runs off-screen, only the top half shows */
#define MICB_RADIUS  8         /* pill corner radius (matches the trackpad bar) */
#define SKAIB_RADIUS 80        /* matches message_widget_bg's rounded corners */

/* The morph runs in two SEQUENCED phases driven by two separate 0..255 anims:

     phase 1 (grow): the mic bar grows + slides from the button geometry up to the
       box geometry, its corner radius opens 25→80 and its black fill deepens
       50%→80% so it becomes the box's backdrop (the frame image is too faint to
       be a fill on its own). The mic glyph fades out.
     phase 2 (frame): the frame image + transcript label fade in.

   OPEN runs grow → THEN frame (the border only appears once the button has
   finished enlarging). CLOSE runs frame → THEN grow (the border disappears
   first, then the button shrinks back). The mic bar is NOT hidden at the open
   end — it stays as the box backdrop; on close only skaibar_input is hidden. */
static void skaibar_grow_cb(void *var, int32_t f)
{
    (void)var;
    if (!p) return;
    if (p->mic_bar && lv_obj_is_valid(p->mic_bar))
    {
        lv_obj_set_size(p->mic_bar, MICB_W + (SKAIB_W - MICB_W) * f / 255,
                        MICB_H + (SKAIB_H - MICB_H) * f / 255);
        lv_obj_align(p->mic_bar, LV_ALIGN_BOTTOM_MID, 0,
                     MICB_Y + (SKAIB_Y - MICB_Y) * f / 255);
        lv_obj_set_style_radius(p->mic_bar,
                                MICB_RADIUS + (SKAIB_RADIUS - MICB_RADIUS) * f / 255, 0);
        lv_obj_set_style_bg_opa(p->mic_bar,
                                (lv_opa_t)(LV_OPA_90 + (LV_OPA_80 - LV_OPA_90) * f / 255), 0);
        /* the slim pill's white border fades out as the box's frame image takes over */
        lv_obj_set_style_border_opa(p->mic_bar,
                                    (lv_opa_t)(LV_OPA_50 - LV_OPA_50 * f / 255), 0);
    }
}

static void skaibar_frame_cb(void *var, int32_t f)
{
    (void)var;
    if (!p) return;
    if (p->skaibar_frame && lv_obj_is_valid(p->skaibar_frame))
        lv_obj_set_style_img_opa(p->skaibar_frame, (lv_opa_t)f, 0);
    if (p->skaibar_label && lv_obj_is_valid(p->skaibar_label))
        lv_obj_set_style_text_opa(p->skaibar_label, (lv_opa_t)(LV_OPA_80 * f / 255), 0);
}

/* close, last step: the button has shrunk back — hide the box (button stays). */
static void skaibar_close_done_cb(lv_anim_t *a)
{
    (void)a;
    if (!p) return;
    lv_obj_add_flag(p->skaibar_input, LV_OBJ_FLAG_HIDDEN);
}

/* close, after the frame faded out: shrink the button back down. */
static void skaibar_close_grow_cb(lv_anim_t *a)
{
    (void)a;
    if (!p || !p->skaibar_input) return;
    lv_anim_t g;
    lv_anim_init(&g);
    lv_anim_set_var(&g, p->skaibar_input);
    lv_anim_set_values(&g, 255, 0);
    lv_anim_set_time(&g, SKAIBAR_GROW_MS);
    lv_anim_set_path_cb(&g, lv_anim_path_ease_in);
    lv_anim_set_exec_cb(&g, skaibar_grow_cb);
    lv_anim_set_ready_cb(&g, skaibar_close_done_cb);
    lv_anim_start(&g);
}

/* open, after the button finished enlarging: fade the frame image + label in. */
static void skaibar_open_frame_cb(lv_anim_t *a)
{
    (void)a;
    if (!p || !p->skaibar_input) return;
    lv_anim_t fr;
    lv_anim_init(&fr);
    lv_anim_set_var(&fr, p->skaibar_input);
    lv_anim_set_values(&fr, 0, 255);
    lv_anim_set_time(&fr, SKAIBAR_FRAME_MS);
    lv_anim_set_path_cb(&fr, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&fr, skaibar_frame_cb);
    lv_anim_start(&fr);
}

static void skaibar_open(void)
{
    if (!p || !p->skaibar_input) return;
    lv_anim_del(p->skaibar_input, skaibar_grow_cb);    /* cancel any in-flight morph */
    lv_anim_del(p->skaibar_input, skaibar_frame_cb);
    lv_label_set_text(p->skaibar_label, "聽取中");
    skaibar_grow_cb(NULL, 0);                           /* button state */
    skaibar_frame_cb(NULL, 0);                          /* frame fully hidden */
    lv_obj_clear_flag(p->mic_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(p->skaibar_input, LV_OBJ_FLAG_HIDDEN);
    p->skaibar_active = true;
    /* phase 1: grow the button; phase 2 (frame fade-in) chains off its ready_cb. */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, p->skaibar_input);
    lv_anim_set_values(&a, 0, 255);
    lv_anim_set_time(&a, SKAIBAR_GROW_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, skaibar_grow_cb);
    lv_anim_set_ready_cb(&a, skaibar_open_frame_cb);
    lv_anim_start(&a);
#if defined(BSP_USING_BLOC) && !defined(BSP_USING_PC_SIMULATOR)
    /* Real hardware: same voice pipeline as the left mic — record audio, send to
       the phone for STT, the transcript returns via lvgl_msg_handler →
       refresh_ai_chat_input_message → (device branch) → device_pager_skaibar_say.
       Disabled in the PC sim (hangs); the sim uses pager_say fake injection. */
    lvgl_msg_handler.handle_input_message = refresh_ai_chat_input_message;
    voice_set_pending_v2t_intent(V2T_INTENT_SKAIBAR);
    voice_provider.start_v2t();
    LOG_I("[pager] skaibar opened (mic) -- v2t started");
#else
    LOG_I("[pager] skaibar opened (mic) -- sim/no-BLOC, awaiting pager_say");
#endif
}

static void skaibar_close(void)
{
    if (!p || !p->skaibar_input || !p->skaibar_active) return; /* guard repeats */
    p->skaibar_active = false;
#if defined(BSP_USING_BLOC) && !defined(BSP_USING_PC_SIMULATOR)
    voice_provider.stop_v2t();
#endif
    lv_anim_del(p->skaibar_input, skaibar_grow_cb);    /* cancel any in-flight morph */
    lv_anim_del(p->skaibar_input, skaibar_frame_cb);
    skaibar_grow_cb(NULL, 255);                         /* box-sized backdrop */
    skaibar_frame_cb(NULL, 255);                        /* frame fully shown */
    lv_obj_clear_flag(p->mic_bar, LV_OBJ_FLAG_HIDDEN);
    /* phase 1: fade the frame out; phase 2 (button shrink) chains off ready_cb. */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, p->skaibar_input);
    lv_anim_set_values(&a, 255, 0);
    lv_anim_set_time(&a, SKAIBAR_FRAME_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_exec_cb(&a, skaibar_frame_cb);
    lv_anim_set_ready_cb(&a, skaibar_close_grow_cb);
    lv_anim_start(&a);
}

/* Public: is the device-page voice input box currently showing? The transcript
   router (refresh_ai_chat_input_message) uses this to decide that the recognised
   text belongs here rather than the left instruction_list widget. */
bool device_pager_skaibar_is_open(void)
{
    return p && p->skaibar_active;
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

    /* Mouse base — always-present page beneath everything (the "clock"). Transparent
       so the device page reveals only its elements over the watch-face fade-to-black
       (gaus_dial_bg); no opaque panel slides in with the content. The hosted hid_mouse
       trackpad then sits on that same black backdrop. */
    p->mouse_base = lv_obj_create(parent);
    lv_obj_set_size(p->mouse_base, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->mouse_base, 0, 0);
    lv_obj_set_style_radius(p->mouse_base, 0, 0);
    lv_obj_set_style_border_width(p->mouse_base, 0, 0);
    lv_obj_set_style_pad_all(p->mouse_base, 0, 0);
    lv_obj_set_style_bg_opa(p->mouse_base, LV_OPA_TRANSP, 0);
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
    lv_obj_add_event_cb(p->overlay, bar_follow_reveal_cb, LV_EVENT_SCROLL, NULL);
    p->home_tile = lv_tileview_add_tile(p->overlay, 0, 0, LV_DIR_BOTTOM);
    p->list_tile = lv_tileview_add_tile(p->overlay, 0, 1, LV_DIR_TOP);
    lv_obj_set_style_bg_opa(p->home_tile, LV_OPA_TRANSP, 0); /* see mouse through */
    /* Transparent — the black backdrop is the watch-face gaus_dial_bg, faded to full
       black by the right-tile reveal (app_clock_status_bar), so the whole face darkens
       to black as the page is pulled out. */
    lv_obj_set_style_bg_opa(p->list_tile, LV_OPA_TRANSP, 0);
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

    /* Device-name drag handle: a transparent band over the top (device-name) row,
       above the item lists, so dragging there pulls the WHOLE page up/down (the
       overlay drawer, via bar_cb) instead of scrolling items — same feel as the
       mouse-page bottom bar. Items below it still scroll normally. */
    {
        lv_obj_t *name_drag = lv_obj_create(p->list_tile);
        lv_obj_set_size(name_drag, LV_HOR_RES, 76);
        lv_obj_align(name_drag, LV_ALIGN_TOP_MID, 0, 0);
        lv_obj_set_style_bg_opa(name_drag, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(name_drag, 0, 0);
        lv_obj_set_style_pad_all(name_drag, 0, 0);
        lv_obj_clear_flag(name_drag, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(name_drag, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(name_drag, bar_cb, LV_EVENT_ALL, NULL);
    }

    /* Bottom-centre voice-input bar — a slim home-indicator pill (matches the
       trackpad's former bottom bar): dark fill, thin white border, no icon. A
       tap morphs it into the skaibar input box. */
    p->mic_bar = lv_obj_create(p->list_tile);
    lv_obj_set_size(p->mic_bar, MICB_W, MICB_H);
    lv_obj_align(p->mic_bar, LV_ALIGN_BOTTOM_MID, 0, MICB_Y);
    lv_obj_set_style_bg_color(p->mic_bar, lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_bg_opa(p->mic_bar, LV_OPA_90, 0);
    lv_obj_set_style_border_color(p->mic_bar, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(p->mic_bar, 2, 0);
    lv_obj_set_style_border_opa(p->mic_bar, LV_OPA_50, 0);
    lv_obj_set_style_radius(p->mic_bar, MICB_RADIUS, 0);
    lv_obj_set_style_pad_all(p->mic_bar, 0, 0);
    lv_obj_clear_flag(p->mic_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(p->mic_bar, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(p->mic_bar, mic_clicked_cb, LV_EVENT_CLICKED, NULL);

    /* skaibar input box (shown on mic tap) — mirrors the left instruction_list's
       voice input pill (lv_instruction_list_layout.c): a BOTTOM-aligned container
       framed by the message_widget_bg image. On real hardware LVGL's drawn border
       is too thin, so the image (442x252) supplies the visible border + rounded
       shape. Tap it to dismiss. */
    p->skaibar_input = lv_obj_create(p->list_tile);
    lv_obj_set_size(p->skaibar_input, SKAIB_W, SKAIB_H);
    lv_obj_align(p->skaibar_input, LV_ALIGN_BOTTOM_MID, 0, SKAIB_Y);
    lv_obj_set_style_bg_opa(p->skaibar_input, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(p->skaibar_input, 0, 0);
    lv_obj_set_style_pad_all(p->skaibar_input, 0, 0);
    lv_obj_clear_flag(p->skaibar_input, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(p->skaibar_input, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(p->skaibar_input, mic_clicked_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(p->skaibar_input, LV_OBJ_FLAG_HIDDEN);
    p->skaibar_frame = lv_img_create(p->skaibar_input);
    lv_img_set_src(p->skaibar_frame, &message_widget_bg);
    lv_obj_center(p->skaibar_frame);
    lv_obj_clear_flag(p->skaibar_frame, LV_OBJ_FLAG_CLICKABLE);
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

    /* Bottom re-summon zone (on the mouse base). Invisible by design — like the
       watch-face edge zones — so the trackpad stays clean; a swipe up from the
       bottom still pulls the device list back. PRESS_LOCK cleared so the
       continuing drag transfers to the overlay (watch-face edge-zone style). */
    p->bar = lv_obj_create(parent);
    lv_obj_set_size(p->bar, LV_HOR_RES, BAR_H);
    lv_obj_align(p->bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(p->bar, LV_OPA_TRANSP, 0);   /* no visible dark bar */
    lv_obj_set_style_border_width(p->bar, 0, 0);
    lv_obj_set_style_radius(p->bar, 0, 0);
    lv_obj_set_style_pad_all(p->bar, 0, 0);
    lv_obj_clear_flag(p->bar, LV_OBJ_FLAG_SCROLLABLE);
    /* keep PRESS_LOCK: the bar handles the whole drag itself (live bidirectional
       reveal in bar_cb), it no longer transfers the press to the overlay. */
    lv_obj_add_flag(p->bar, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(p->bar, bar_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(p->bar, LV_OBJ_FLAG_HIDDEN);

    /* Visible drawer handle on the very top layer so neither the hosted trackpad nor
       the rising list can cover it. Non-clickable: touches fall through to p->bar
       (which drives the finger-follow); this grabber is purely the visual that rides
       up with the list (bar_follow_reveal_cb). Shown only in mouse mode. */
    p->grabber = lv_obj_create(lv_layer_top());
    lv_obj_set_size(p->grabber, 100, 16);                       /* match the list's mic_bar pill */
    lv_obj_align(p->grabber, LV_ALIGN_BOTTOM_MID, 0, GRABBER_REST_Y);
    lv_obj_set_style_bg_color(p->grabber, lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_bg_opa(p->grabber, LV_OPA_90, 0);
    lv_obj_set_style_border_color(p->grabber, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(p->grabber, 2, 0);
    lv_obj_set_style_border_opa(p->grabber, LV_OPA_50, 0);
    lv_obj_set_style_radius(p->grabber, 8, 0);
    lv_obj_set_style_pad_all(p->grabber, 0, 0);
    lv_obj_clear_flag(p->grabber, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(p->grabber, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(p->grabber, LV_OBJ_FLAG_HIDDEN);

    lv_obj_set_tile_id(p->overlay, 0, 1, LV_ANIM_OFF); /* start on the LIST */

    seed_fake_devices();   /* peer-reported list will replace this later */
    refresh();
    LOG_I("[pager] built (overlay tileview + mouse base, %d devices)", p->count);
    return p->overlay;
}
