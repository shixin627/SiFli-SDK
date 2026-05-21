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
#define ARC_ZOOM_MIN   128         /* 0.5x at the edges */
#define ARC_ZOOM_CTR   320         /* ~1.25x at the centre */
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
    lv_obj_t   *bar;             /* bottom re-summon handle (on the mouse) */
    lv_obj_t   *empty_label;
    bool        mouse_created;   /* real hid_mouse hosted in mouse_base */
    bool        summoning;       /* bar press shown the overlay; awaiting drag/tap */
    bool        pull_pending;    /* over-pull at the top item → going to mouse */

    dev_page_t  model[MAX_DEVICES];
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
            strncpy(d->items[j], src->default_actions[j], sizeof(d->items[0]) - 1);
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
/* scroll_y at which item 0 sits at the arc centre (mirrors app_exercise). */
static int32_t arc_base_scroll(tile_ui_t *u)
{
    const lv_coord_t ly1 = u->list->coords.y1;
    const lv_coord_t pt  = lv_obj_get_style_pad_top(u->list, LV_PART_MAIN);
    return ly1 + pt + ARC_ICON_SIZE / 2 - LV_VER_RES / 2;
}

static void apply_arc(tile_ui_t *u)
{
    dev_page_t *d = u->dev;
    int count = d ? d->item_count : 0;
    const int   cx   = LV_HOR_RES / 2;
    const int   cy   = LV_VER_RES / 2;
    const float aps  = ARC_SLOT_DEG * (float)M_PI / 180.0f;
    const lv_coord_t lx1 = u->list->coords.x1;
    const lv_coord_t ly1 = u->list->coords.y1;
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
        lv_obj_set_pos(u->item_icon[i], sx - lx1 - pl - iw / 2, sy - ly1 - pt - ih / 2);
        lv_opa_t opa = (lv_opa_t)(ARC_OPA_MIN + (LV_OPA_COVER - ARC_OPA_MIN) * c);
        lv_obj_set_style_img_opa(u->item_icon[i], opa, 0);
    }
    u->sel = closest;
    if (count > 0 && min_abs <= (float)M_PI / 2.0f)
        lv_label_set_text(u->name_label, d->items[closest]);
    else
        lv_label_set_text(u->name_label, "");
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
    /* Start at the first item; layout must be current for the scroll math. */
    u->sel = 0;
    lv_obj_update_layout(u->list);
    lv_obj_scroll_to_y(u->list, arc_base_scroll(u), LV_ANIM_OFF);
    apply_arc(u);
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
        /* ADR-0008 E7: tell the phone the new active-target device so it routes
           subsequent watch commands / recognized text there (network via primary). */
        commu_send_active_device(p->model[p->current].id_str);
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
}

/* Public: re-load the (phone-streamed) device list from the registry and
   refresh. Called on reveal AND by skai_device_ui_refresh() whenever E7 syncs a
   new device list / status / per-device actions from the primary. */
void device_pager_refresh(void)
{
    if (p) load_devices_from_registry();
    refresh();
}

/* ADR-0008 E7: weak hook invoked by communicate_parse_skailink.c after it
   updates SkaiWatchSys.device_registry from a phone-streamed sync — resolves the
   weak symbol so the pager re-renders live. */
void skai_device_ui_refresh(void)
{
    device_pager_refresh();
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
        /* Settled on the LIST → hide the bar; allow a fresh over-pull. */
        lv_obj_add_flag(p->bar, LV_OBJ_FLAG_HIDDEN);
        p->pull_pending = false;
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
    if (!p || !p->skaibar_input) return;
    lv_obj_add_flag(p->skaibar_input, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(p->mic_bar, LV_OBJ_FLAG_HIDDEN);
    p->skaibar_active = false;
#if defined(BSP_USING_BLOC) && !defined(BSP_USING_PC_SIMULATOR)
    voice_provider.stop_v2t();
#endif
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

    load_devices_from_registry();   /* ADR-0008 E7: real phone-streamed device registry */
    refresh();
    LOG_I("[pager] built (overlay tileview + mouse base, %d devices)", p->count);
    return p->overlay;
}
