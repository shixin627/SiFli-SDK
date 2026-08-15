/**
 ******************************************************************************
 * @file   device_pager.c
 * @brief  Per-device control page — content builder for the watch-face
 *         tileview's LEFT tile (0,1); home is (1,1), so the page slides
 *         out to the LEFT (finger-left swipe) to return to the watch face.
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
#include "watch_global_data.h"          /* ADR-0008 E7/E8: SkaiWatchSys.device_registry/status/name */
#include "communicate_task.h"           /* ADR-0008 E7: commu_send_active_device (uplink) */
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
LV_IMG_DECLARE(qr_download); /* no-device empty state: prebuilt QR for skaiwalk.com/download (large_ezip resource) */
LV_IMG_DECLARE(img_left_arrow);  /* 9x17  — device-name strip: previous-device arrow */
LV_IMG_DECLARE(img_right_arrow); /* 9x17  — device-name strip: next-device arrow */
LV_IMG_DECLARE(logout);          /* 16x16 — replaces the left arrow on the first device (tap = exit) */
/* Per-action category icons (auto-built from the resource ezip image folder).
   Selected per item by its DEV_ACTION_TYPE; replaces the old img_flashlight
   placeholder. */
LV_IMG_DECLARE(instruction_icon); /* type 0 */
LV_IMG_DECLARE(application_icon); /* type 1 */
LV_IMG_DECLARE(folder_icon);      /* type 2 */
LV_IMG_DECLARE(img_logo);         /* type 3 (ai) */

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
    char    id_str[SYNCED_DEVICE_ID_LEN]; /* ADR-0008 E7: real account device_id (UUID) for active-select routing */
    char    name[32];
    uint8_t dev_idx;
    uint8_t conn_idx;   /* BLE HID target for drill-down; 0xFF = not connected */
    uint8_t status;     /* ADR-0008 E7: 0 off / 1 on / 2 primary (gray the header when off) */
    char    items[MAX_TILE_ITEMS][24];
    uint8_t item_type[MAX_TILE_ITEMS]; /* DEV_ACTION_TYPE per item -> category icon */
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
    lv_obj_t *offline_label;              /* shown instead of the list when the device is offline */
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
    lv_obj_t   *empty_view;      /* no-device empty state: download QR image + hint (shown when count == 0) */
    lv_obj_t   *empty_qr_card;   /* white rounded card that holds the static QR image */
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
    lv_obj_t   *skaibar_clip;    /* 2-row clip window: only the latest 2 wrapped rows show */
    lv_obj_t   *skaibar_label;
    bool        skaibar_active;

    /* mouse-page top device-name strip: a horizontal drag scrubs through devices
       (drag farther / hold and keep moving = step through several at once, so a
       distant device is reached without flicking one-by-one). */
    lv_obj_t   *dev_name_bar;
    lv_obj_t   *dev_name_label;
    lv_obj_t   *dev_left_icon;   /* img_left_arrow (prev) — swaps to `logout` on the first device (tap = exit) */
    lv_obj_t   *dev_right_icon;  /* img_right_arrow (next) */
} device_pager_t;

static device_pager_t *p = NULL;

/* Set while device_pager_refresh() does its programmatic rebind. The rebind
   scrolls the list (lv_obj_scroll_to_y), which fires LV_EVENT_SCROLL ->
   list_scroll_cb, whose default action closes the skaibar. Without this guard a
   LIVE sync refresh (now that it actually fires) would kick the user out of
   voice-input mode. A real finger scroll has this flag clear, so it still
   dismisses the skaibar as before. */
static bool s_suppress_skaibar_dismiss = false;

/* Active-target uplink for the RIGHT page (the selected non-primary device, or
   "" on home). De-dup now lives in commu_send_active_device (SHARED with the
   LEFT instruction_list, which asserts the primary as the target), so a local
   dedup here would shadow it and skip a needed re-assert after switching pages.
   Just forward; the shared dedup only sends on an actual change. */
static void pager_send_active(const char *id)
{
    commu_send_active_device(id ? id : "");
}

static void mic_clicked_cb(lv_event_t *e);   /* defined below (skaibar section) */
static void mouse_retarget(void);            /* defined below */
static void dev_name_bar_update(void);       /* defined below */
static void pager_set_current(int idx);      /* defined below */
static void skaibar_close(void);             /* defined below (skaibar section) */
static void skaibar_open(void);              /* defined below (skaibar section) */
static void skaibar_grow_cb(void *var, int32_t f); /* morph step; defined below */
static bool current_device_offline(void);    /* defined below */
static void sync_offline_page_chrome(void);   /* defined below */


/* ADR-0008 E7: load the device pager model from the account device registry
   (SkaiWatchSys.device_registry), which the phone (primary) streams in over the
   SKAI_LINK BWPS group (communicate_parse_skailink.c) and which E8 persists to
   flash. Replaces the throwaway fake seed. Each device's instruction-list items
   come from its default_actions (the per-device actions synced from the phone).
   conn_idx stays 0xFF: targets are network-routed via the primary (active
   select uplink), not local BLE-HID. Re-callable; E7 calls it via
   device_pager_refresh() / skai_device_ui_refresh() on every sync. */
static void load_devices_from_registry(void)
{
    p->count = 0;
    uint8_t n = SkaiWatchSys.device_registry.count;
    if (n > MAX_DEVICES) n = MAX_DEVICES;
    for (uint8_t i = 0; i < n; i++)
    {
        const T_SYNCED_DEVICE *src =
            (const T_SYNCED_DEVICE *)&SkaiWatchSys.device_registry.devices[i];
        dev_page_t *d = &p->model[p->count];
        memset(d, 0, sizeof(*d));
        strncpy(d->id_str, src->id, sizeof(d->id_str) - 1);
        strncpy(d->name, (const char *)SkaiWatchSys.device_name[i],
                sizeof(d->name) - 1);
        if (d->name[0] == '\0')
            snprintf(d->name, sizeof(d->name), "Device %u", (unsigned)(i + 1));
        d->status   = SkaiWatchSys.device_status[i];
        d->dev_idx  = i;
        d->conn_idx = 0xFF;
        uint8_t ic = src->default_action_count;
        if (ic > MAX_TILE_ITEMS) ic = MAX_TILE_ITEMS;
        for (uint8_t j = 0; j < ic; j++)
        {
            strncpy(d->items[j], src->default_actions[j], sizeof(d->items[0]) - 1);
            d->item_type[j] = src->default_action_types[j]; /* category -> icon */
        }
        d->item_count = ic;
        p->count++;
    }
    if (p->current >= p->count)
        p->current = p->count > 0 ? p->count - 1 : 0;
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
    /* Tell the phone which option is now centred on the active device's tile, so
       it can mirror the highlight. Only for the centre tile (the focused device),
       only on an actual change, and NOT during a programmatic rebind (s_suppress
       is set then) — otherwise a sync/refresh that re-centres would spam it. */
    if (u == &p->t[TILE_CENTER] && count > 0 && closest != prev_sel &&
        !s_suppress_skaibar_dismiss)
    {
        commu_send_option_focus((uint8_t)closest);
        /* Haptic tick on each option switch, matching the left instruction_list
           (motor_pattern_scrolling_app, gated internally by the global motor
           switch). Fires only on a real user-scroll boundary crossing — the
           s_suppress guard already excludes programmatic rebinds/device switches. */
        motor_pattern_scrolling_app();
        LOG_I("[pager] scrolled to option %d", closest);
    }
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


/* Map a device-list item's category (DEV_ACTION_TYPE) to its icon. Unknown
   values fall back to the instruction icon. */
static const lv_img_dsc_t *icon_for_item_type(uint8_t type)
{
    switch (type)
    {
    case DEV_ACTION_TYPE_APPLICATION: return &application_icon;
    case DEV_ACTION_TYPE_FOLDER:      return &folder_icon;
    case DEV_ACTION_TYPE_AI:          return &img_logo;
    case DEV_ACTION_TYPE_INSTRUCTION:
    default:                          return &instruction_icon;
    }
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
    bool offline = (d->status == 0);
    lv_label_set_text_fmt(u->header, "%s   %d/%d", d->name, logical + 1, p->count);
    /* ADR-0008 E7: status-driven header tint (status = device_status[i], synced
       by communicate_parse_skailink): 0 off -> gray, 1 on -> blue, 2 primary ->
       lit accent so the device currently bridging the watch stands out. */
    lv_obj_set_style_text_color(
        u->header,
        offline            ? lv_color_hex(0x666666)   /* off: dimmed     */
        : (d->status == 2) ? lv_color_hex(0x4DE3A0)   /* primary: lit    */
                           : lv_color_hex(0x00AAFF),  /* on: normal blue */
        0);

    /* Offline device: lock the page — hide the action list + its name label,
       show the OFFLINE placeholder, and make the list non-scrollable so nothing
       on the page moves (nor can the trackpad be pulled in). The horizontal
       device swipe still works: it chains through the non-scrollable list to the
       pager, so the user can swipe to another (online) device. */
    if (offline)
    {
        for (int i = 0; i < MAX_TILE_ITEMS; i++)
        {
            lv_obj_add_flag(u->item_icon[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(u->snap[i], LV_OBJ_FLAG_HIDDEN);
        }
        lv_obj_add_flag(u->name_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(u->offline_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(u->list, LV_OBJ_FLAG_SCROLLABLE);
        if (u->arc) arc_scroll_set_item_count(u->arc, 0);
        u->sel = -1;            /* no selectable item → taps/commits are no-ops */
        return;
    }
    /* Online: restore the interactive list (offline placeholder hidden, list
       scrollable again) before binding the items. */
    lv_obj_add_flag(u->offline_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(u->name_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(u->list, LV_OBJ_FLAG_SCROLLABLE);
    /* Show one snap anchor per item so the arc_scroll range matches item_count,
       and point each item icon at its category image (instruction/application/
       folder). The icons are recycled across devices, so the src must be re-bound
       here whenever a device is bound to this tile. */
    for (int i = 0; i < MAX_TILE_ITEMS; i++)
    {
        if (i < d->item_count)
        {
            lv_obj_clear_flag(u->snap[i], LV_OBJ_FLAG_HIDDEN);
            const lv_img_dsc_t *dsc = icon_for_item_type(d->item_type[i]);
            lv_img_set_src(u->item_icon[i], dsc);
            /* re-center the zoom pivot for this icon's actual size (the category
               icons need not all match img_flashlight's 80x80). */
            lv_img_set_pivot(u->item_icon[i], dsc->header.w / 2, dsc->header.h / 2);
        }
        else
        {
            lv_obj_add_flag(u->snap[i], LV_OBJ_FLAG_HIDDEN);
        }
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
   the page lives on the LEFT tile (0,1) with home to its grid-right, so on the
   LAST device a left-swipe (toward "next") chains back to the watch face (whole
   page + mouse finger-follow out). On the FIRST device the right-swipe (toward
   "previous") is blocked too — there is no tile left of (0,1), so it just dead-
   stops. Middle devices scroll both ways between devices. */
static void update_pager_scroll_dir(void)
{
    if (!p || !p->pager) return;
    lv_dir_t dir = LV_DIR_HOR;
    bool first = (p->current <= 0);
    bool last  = (p->current >= p->count - 1);
    /* LV_DIR_LEFT allows the right-swipe (→ previous device) but blocks the
       left-swipe (→ "next"), so on the last device the left-swipe chains to the
       watch face. LV_DIR_RIGHT is the mirror for the first device. */
    if (first && last)      dir = LV_DIR_LEFT;          /* single device: only the home chain */
    else if (first)         dir = LV_DIR_RIGHT;         /* block "previous" (nothing past it) */
    else if (last)          dir = LV_DIR_LEFT;          /* block "next" → chains to watch face */
    lv_obj_set_scroll_dir(p->pager, dir);
}


/* Update the mouse-page device-name strip to the current device (or a no-device
   placeholder). Safe any time; no-ops until the strip is built. */
static void dev_name_bar_update(void)
{
    if (!p || !p->dev_name_label || !lv_obj_is_valid(p->dev_name_label)) return;
    if (p->count > 0 && p->current < p->count)
        lv_label_set_text(p->dev_name_label, p->model[p->current].name);
    else
        lv_label_set_text(p->dev_name_label, "No device");
    lv_obj_center(p->dev_name_label);
    /* Left icon = NEXT-device arrow; hidden on the LAST device (no next device). */
    if (p->dev_left_icon && lv_obj_is_valid(p->dev_left_icon))
    {
        lv_img_set_src(p->dev_left_icon, &img_left_arrow);
        if (p->current < p->count - 1)
            lv_obj_clear_flag(p->dev_left_icon, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(p->dev_left_icon, LV_OBJ_FLAG_HIDDEN);
    }
    /* Right icon = exit-to-watch-face (`logout`) on the LAST device — home is to the
       grid-right, so the EXIT sits on the RIGHT (user request); PREVIOUS-device arrow
       on middle devices; hidden on the FIRST device (no previous, no exit-here). */
    if (p->dev_right_icon && lv_obj_is_valid(p->dev_right_icon))
    {
        if (p->current >= p->count - 1)
        {
            lv_img_set_src(p->dev_right_icon, &logout);
            lv_obj_clear_flag(p->dev_right_icon, LV_OBJ_FLAG_HIDDEN);
        }
        else if (p->current > 0)
        {
            lv_img_set_src(p->dev_right_icon, &img_right_arrow);
            lv_obj_clear_flag(p->dev_right_icon, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(p->dev_right_icon, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* Jump the active device to an absolute index and run the SAME settle work the
   carousel-swipe path does (rebind, uplink active target, retarget the hosted
   mouse, recentre, offline chrome, name-strip). Shared by the mouse-page name
   strip so there is one switch path, not two. */
static void pager_set_current(int idx)
{
    if (!p || p->count == 0) return;
    if (idx < 0) idx = 0;
    if (idx >= p->count) idx = p->count - 1;
    if (idx == p->current) return;
    p->current = idx;
    rebind_all();
    pager_send_active(p->model[p->current].id_str);
    if (p->mouse_created) mouse_retarget();
    snap_to_center(LV_ANIM_OFF);
    update_pager_scroll_dir();
    sync_offline_page_chrome();
    dev_name_bar_update();
    LOG_I("[pager] name-strip -> device %d/%d (%s)", p->current + 1, p->count,
          p->model[p->current].name);
}


/* Is the CURRENT (centred) device offline (status 0)? Its page is locked. */
static bool current_device_offline(void)
{
    return p && p->count > 0 && p->current < p->count &&
           p->model[p->current].status == 0;
}

/* Page-level lock for an offline current device: hide the bottom voice (mic)
   bar so its input can't be opened (the per-tile list is already hidden +
   non-scrollable in bind_tile). Call whenever the current device or its status
   may have changed (refresh / device switch). */
static void sync_offline_page_chrome(void)
{
    if (!p || !p->mic_bar) return;
    if (current_device_offline())
    {
        if (p->skaibar_active) skaibar_close();
        lv_obj_add_flag(p->mic_bar, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_clear_flag(p->mic_bar, LV_OBJ_FLAG_HIDDEN);
    }
}

static void refresh(void)
{
    if (!p) return;
    if (p->count == 0)
    {
        lv_obj_add_flag(p->pager, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(p->empty_view, LV_OBJ_FLAG_HIDDEN);
        /* No device → the trackpad has nothing to control, so the pull-down that
           reveals the mouse base makes no sense. Lock the overlay on the LIST
           tile (the QR empty state): pin it to LIST and clear SCROLLABLE so a
           downward drag can't scroll to the transparent HOME tile / the mouse.
           Re-enabled in the count > 0 path below. */
        lv_obj_set_tile_id(p->overlay, 0, 1, LV_ANIM_OFF);
        lv_obj_clear_flag(p->overlay, LV_OBJ_FLAG_SCROLLABLE);
        return;
    }
    /* Device(s) present → restore the pull-down-to-mouse gesture. */
    lv_obj_add_flag(p->overlay, LV_OBJ_FLAG_SCROLLABLE);
    /* R3 stage 2: the old drag-up device carousel (this overlay's list_tile) is
       superseded by the shared floating list. Keep the overlay HIDDEN whenever
       there are devices, so the pre-merge per-page list can't ride into the mouse
       page on the first pull-in — it is BUILT shown, and set_active only hides
       it at the swipe midpoint, so without this it slides in with the device tile.
       The mouse-page entry shows the trackpad directly; the count == 0 branch above
       keeps its own empty_view / QR path (this only runs when count > 0). */
    lv_obj_add_flag(p->overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(p->pager, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p->empty_view, LV_OBJ_FLAG_HIDDEN);
    rebind_all();
    snap_to_center(LV_ANIM_OFF);
    update_pager_scroll_dir();
    sync_offline_page_chrome();
}

/* R3 unified UI: the device page's own carousel is gone — the shared floating
   instruction list (lv_instruction_list_layout) is the single options surface.
   Mirror the ACTIVE device's options (d->items: streamed from the phone via
   handle_device_actions_batch -> registry -> load_devices_from_registry) into
   that list so it shows THIS device's skaibar options, updating live as the phone
   streams them. Same order the phone sent and app_base_count==0, so the list's
   tap commit (commu_send_option_commit(index) = KEY_ACTION_SELECT) resolves on
   the phone by that same index against its _lastSentDeviceActions -> runs the
   action on the device. open_app=NULL keeps a tap on the SKAIBAR-commit (phone
   relay) path, not a local watch app. LVGL-thread only (callers: device_pager_
   set_active and device_pager_refresh; the E7 path defers to the LVGL thread). */
static void feed_active_device_options_to_list(void)
{
    if (!p || p->count == 0 || p->current >= p->count) return;
    extern void clear_custom_instructions(void);
    extern void add_or_update_custom_instruction(const char *id,
            const char *title, const char *trigger_type, uint32_t interval_sec,
            bool enabled, uint32_t version, const char *open_app);
    extern void refresh_custom_instructions(void);
    extern void instruction_list_save_base(void);
    static uint32_t s_dev_opt_ver = 0;
    dev_page_t *d = &p->model[p->current];
    /* snapshot the watch-face instruction list before we overwrite it with this
       device's options, so leaving the device page can restore it (guarded: only
       the first feed of a visit actually snapshots). */
    instruction_list_save_base();
    /* Device options are 0-based with no pinned Settings entry — drop the
       watch-face Settings pin so they fill from index 0 (restore_base re-pins it
       on the way out). */
    extern void instruction_list_drop_pinned_for_device(void);
    instruction_list_drop_pinned_for_device();
    s_dev_opt_ver++;
    clear_custom_instructions();
    for (uint8_t i = 0; i < d->item_count; i++)
    {
        if (d->items[i][0] == '\0') continue;
        add_or_update_custom_instruction(d->items[i], d->items[i], "once", 0,
                                         true, s_dev_opt_ver, NULL);
    }
    refresh_custom_instructions();
}

/* Public: re-load the (phone-streamed) device list from the registry and
   refresh. Called on reveal AND by skai_device_ui_refresh() whenever E7 syncs a
   new device list / status / per-device actions from the primary. */
void device_pager_refresh(void)
{
    /* P3 麥克風 OOM 修復:standalone 滑鼠 app 進來時錶盤(Main app)被 gui_app_exit 整個拆掉,
       device_pager 的 lv_objs(建在 Main 的 tile 上)隨之釋放,但本檔 static `p` 不會被設 NULL →
       此後 refresh()/load_devices_from_registry 碰 p 的 child 物件是野指標(實測 0x03
       device-actions → ui_refresh → 這裡 → UNALIGNED UsageFault、WDT 重開)。此情境改成只重餵
       滑鼠 skaibar 清單(讀 registry、完全不碰 device_pager UI)。本函式已在 LVGL 執行緒
       (skai_device_ui_refresh 有 defer)。embedded device_pager(Main 活著)走原路、不受影響。 */
    extern bool gui_app_is_actived(char *id);
    if (gui_app_is_actived(APP_ID_MOUSE))
    {
        extern void instruction_list_refeed_single_device(void);
        instruction_list_refeed_single_device();
        return;
    }
    /* Hosted 滑鼠(錶盤圖層 hid_mouse_build_ui,Main 活著、APP_ID_MOUSE 非 active)開著
       單設備搜尋抽屜時,0x03 的即時選項也要重餵到浮層清單 —— refeed 自帶
       s_bar_single_device gate,非單設備模式是 no-op。不 return:device_pager 本體照常刷。 */
    {
        extern bool app_control_get_mouse_mode(void);
        if (app_control_get_mouse_mode())
        {
            extern void instruction_list_refeed_single_device(void);
            instruction_list_refeed_single_device();
        }
    }
    if (p) load_devices_from_registry();
    /* The rebind in refresh() programmatically scrolls the list, which would
       otherwise fire list_scroll_cb -> skaibar_close. When a phone sync arrives
       while the user is mid voice-input, that must NOT close the box; suppress the
       scroll-driven dismiss for the duration of this programmatic rebind only. */
    s_suppress_skaibar_dismiss = true;
    refresh();
    s_suppress_skaibar_dismiss = false;

    /* R3 persist: device gone → free the KEPT (off-page) trackpad UI now, on the
       disconnect sync, rather than lazily on the next device-page visit (heap). Host-
       guarded so we never touch the standalone APP_ID_MOUSE app's own UI; off-page
       only (!mouse_created) — an on-page disconnect is handled by the set_active QR
       path which shows the download QR. */
    if (p && p->count == 0 && !p->mouse_created &&
        hid_mouse_ui_host() == p->mouse_base)
    {
        hid_mouse_destroy();
        lv_obj_clean(p->mouse_base);
    }

    /* R3 persist: build the trackpad UI as soon as a device is present (this sync
       runs on the LVGL thread) so it's already on p->mouse_base BEFORE the first
       swipe — no black settle even the FIRST time. Guard on host==NULL: build only
       when NOBODY owns the singleton UI, so we never clobber an active standalone
       APP_ID_MOUSE app (which sets the host to its own screen). enter_mode stays
       deferred to device_pager_set_active, so this inert off-page UI doesn't put the
       watch into mouse mode. */
    if (p && p->count > 0 && hid_mouse_ui_host() == NULL)
    {
        /* host==NULL can still leave orphaned objects on our tile: if the standalone
           app stole the globals (its build re-pointed them) then quit (destroy NULLs
           them but cleans ITS screen, not ours). Clean before rebuilding so we don't
           stack a second UI on top. No-op on a truly empty tile. */
        lv_obj_clean(p->mouse_base);
        hid_mouse_build_ui(p->mouse_base);
    }

    /* (Re)assert the active target. device_pager_refresh runs on every phone list
       sync (skai_device_ui_refresh) and on page entry, so this is where a freshly
       connected phone learns the watch's current target: the centred device while
       we're actually ON the device page (mouse_created), else "" = no device. This
       fixes "boot straight to the left page and the app still thinks a device is
       active": the connect-time sync now tells it there is none. De-duped. */
    if (p)
    {
        bool on_device_page = p->mouse_created;
        const char *active = (on_device_page && p->count > 0 &&
                              p->current < p->count)
                                 ? p->model[p->current].id_str
                                 : "";
        /* standalone APP_ID_MOUSE app 開著且已選設備時，它擁有 active relay 目標；
           這個 refresh 每次手機 E7 同步都跑，不可把它清回 "" — 否則選完設備約 60s
           (下次同步)就把 relay 目標洗掉、控制中斷。只在那情況跳過清 ""。 */
        extern bool hid_mouse_owns_active_target(void);
        if (!(active[0] == '\0' && hid_mouse_owns_active_target()))
            pager_send_active(active);
        /* R3: keep the shared floating list mirroring THIS device's options as
           the phone streams live skaibar updates — only while we're on the
           device page (else we'd clobber the watch-face instruction list). */
        if (on_device_page)
            feed_active_device_options_to_list();
    }
}

/* ADR-0008 E7: weak hook invoked by communicate_parse_skailink.c after it
   updates SkaiWatchSys.device_registry from a phone-streamed sync — resolves the
   weak symbol so the pager re-renders live.

   communicate_parse_skailink runs on the BLE rx thread (KE_EVT2, 4KB stack).
   device_pager_refresh() creates / lays out LVGL objects, which is too
   stack-heavy AND not thread-safe to run there (LVGL is single-threaded). So
   when called off the LVGL thread, defer to it — same pattern as the
   instruction-list rebuild/reset (see ui_handler + lv_instruction_list_layout).
   This is what fixes the KE_EVT2 stack-overflow LCPU crash on device sync. */
void skai_device_ui_refresh(void)
{
    if (!is_on_lvgl_thread())
    {
        lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_REFRESH_DEVICE_PAGER};
        lvgl_send_msg(msg);
        return;
    }
    device_pager_refresh();
    /* 頂部面板的設備名 / 媒體頁也吃同一份 registry — 一併刷新，否則設備上下線
       只有裝置頁會動、面板停在舊清單。 */
    {
        extern void lv_top_panel_refresh_devices(void);
        lv_top_panel_refresh_devices();
    }
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


/* Mouse-page device-name strip: switch by tapping the side arrow icons, or by
   dragging the strip left/right — the name finger-follows and a release past a
   threshold commits one device in the drag direction (dev_name_bar_cb below). */


/* --- drag-to-switch on the name strip. Finger-follows the name (translate_x) so the
   drag is visible, then commits one device on release (no next-device preview). ----- */
#define NAME_DRAG_SWITCH_PX 40   /* drag at least this far before release → switch one */
static lv_obj_t  *s_name_locked_tv = NULL; /* main tileview we cleared SCROLLABLE on */


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

/* Pin the 2-row transcript window to its newest content: scroll the clip so the
   last two wrapped rows are visible (older rows scroll up out of view; the user
   can pull them back down manually). No-op when the text fits in two rows. */
static void skaibar_scroll_to_bottom(void)
{
    if (!p || !p->skaibar_clip || !lv_obj_is_valid(p->skaibar_clip)) return;
    lv_obj_update_layout(p->skaibar_clip);
    lv_coord_t bottom = lv_obj_get_scroll_bottom(p->skaibar_clip);
    if (bottom > 0)
        lv_obj_scroll_by(p->skaibar_clip, 0, -bottom, LV_ANIM_OFF);
    else
        lv_obj_scroll_to_y(p->skaibar_clip, 0, LV_ANIM_OFF);
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
    lv_label_set_text(p->skaibar_label, LV_EXT_STR_GET_BY_KEY(listening, "Listening"));
    skaibar_scroll_to_bottom();                         /* reset window to top */
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
    /* Clear the re-entry gate directly — the async STOP event can early-return
       (AI processing) and leave voice_recognition_started=true, which would make
       the NEXT skaibar-open's start_v2t short-circuit (no mic, no transcript). */
    set_voice_recognition_started(false);
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

/* Public: is the watch ON the device page hosting the trackpad (= actively
   controlling a remote device)? R3: the shared floating instruction list shows
   THIS device's options and is its controlled surface, so it uses this to avoid
   asserting the local/primary target — which would clear the active remote device
   (active="") mid focus/commit and make the phone fall back to its own launcher
   (the wrong options would then mirror onto the watch). */
bool device_pager_is_on_page(void)
{
    return p && p->mouse_created;
}

static void mic_clicked_cb(lv_event_t *e)
{
    (void)e;
    if (!p) return;
    /* Offline device: the page is locked — no voice input. */
    if (current_device_offline()) return;
    /* Toggle by the box's ACTUAL visibility, not the skaibar_active flag — the
       flag can get stuck (e.g. cleared on page-leave without hiding), which
       would make a tap take the close branch and do nothing instead of opening.
       skaibar_input is HIDDEN when closed. */
    bool box_visible = p->skaibar_input &&
                       !lv_obj_has_flag(p->skaibar_input, LV_OBJ_FLAG_HIDDEN);
    if (box_visible) skaibar_close();
    else             skaibar_open();
}

void device_pager_skaibar_say(const char *text)
{
    if (!p) return;
    if (!p->skaibar_active) skaibar_open();
    lv_label_set_text(p->skaibar_label, text ? text : "");
    skaibar_scroll_to_bottom();   /* keep the latest 2 rows in view */
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


