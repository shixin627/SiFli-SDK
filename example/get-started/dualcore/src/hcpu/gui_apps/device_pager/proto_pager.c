/**
 ******************************************************************************
 * @file   proto_pager.c
 * @brief  T0 THROWAWAY prototype — gesture-arbitration skeleton for the
 *         right-side per-device pager + pull-down mouse layer.
 *
 *  Goal: validate that the FOUR gesture axes do not fight each other on the
 *  466px round screen BEFORE investing in the hid_mouse component-ization:
 *    - horizontal page swipe (switch device)   -> native LVGL scroll-snap
 *    - top-band pull-down (reveal mouse layer)  -> top 40px reveal strip
 *    - bottom-bar pull-up (return to list)      -> mouse-layer bottom bar
 *    - center single-finger drag (cursor)       -> logged only in this proto
 *
 *  Every gesture decision is LOG_I'd as [proto] INTENT=... so you can watch
 *  for misclassification while dragging in PC sim or on the watch.
 *
 *  This file is intentionally disposable. It is NOT the real implementation.
 *  Build gate: BSP_USING_PC_SIMULATOR (never ships; widen to !kReleaseMode
 *  once kReleaseMode's definition is confirmed, to test feel on real hw).
 *  Launch from FinSH:  goto_app proto_pager
 ******************************************************************************
 */
#include <rtthread.h>
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "custom_trans_anim.h"
#ifdef BSP_USING_UI_HANDLER
    #include "ui_img_helper.h"
#endif

#define DBG_TAG "proto.pager"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef BSP_USING_PC_SIMULATOR

#define APP_ID_PROTO_PAGER "proto_pager"

#define PROTO_DEVICE_COUNT 3
#define TOP_BAND_PX        40   /* reveal strip height; tuning target */
#define ARC_ZONE_PX        60   /* left/right arc width; tuning target */
#define MOVE_THRESH_PX     16   /* lock intent after this much travel */
#define VERT_RATIO_NUM     1    /* |dy| > |dx| for vertical */
#define HORIZ_RATIO_NUM    3    /* |dx|*2 > |dy|*3 i.e. |dx|>|dy|*1.5 for horiz */

typedef enum { ZONE_TOP, ZONE_ARC_L, ZONE_ARC_R, ZONE_CENTER } start_zone_t;
typedef enum { INTENT_NONE, INTENT_REVEAL, INTENT_PAGE, INTENT_ARC, INTENT_CURSOR } intent_t_e;

typedef struct
{
    lv_obj_t *root;
    lv_obj_t *pager;        /* horizontal scroll-snap container (instruction layer) */
    lv_obj_t *mouse_layer;  /* sits behind pager; revealed by sliding pager down */
    lv_obj_t *top_strip;    /* transparent reveal-capture strip, top 40px, on top */
    lv_obj_t *intent_label; /* debug HUD: last classified intent */
    bool revealed;          /* true when mouse layer is showing */
} proto_pager_t;

static proto_pager_t *p = NULL;

/* gesture-broker scratch (throwaway: file statics, single instance) */
static lv_point_t g_start;
static bool       g_locked;
static intent_t_e g_intent;

static const char *intent_name(intent_t_e i)
{
    switch (i)
    {
    case INTENT_REVEAL: return "REVEAL";
    case INTENT_PAGE:   return "PAGE";
    case INTENT_ARC:    return "ARC";
    case INTENT_CURSOR: return "CURSOR";
    default:            return "NONE";
    }
}

static start_zone_t classify_zone(const lv_point_t *pt)
{
    if (pt->y < TOP_BAND_PX)                  return ZONE_TOP;
    if (pt->x < ARC_ZONE_PX)                  return ZONE_ARC_L;
    if (pt->x > (LV_HOR_RES - ARC_ZONE_PX))   return ZONE_ARC_R;
    return ZONE_CENTER;
}

/* Slide the instruction pager down (reveal) / up (return). The mouse layer
   sits behind at y=0, so moving the pager off the bottom exposes it. */
static void pager_y_anim_cb(void *var, int32_t v) { lv_obj_set_y((lv_obj_t *)var, v); }

static void animate_reveal(bool reveal)
{
    if (!p || !p->pager) return;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, p->pager);
    lv_anim_set_exec_cb(&a, pager_y_anim_cb);
    lv_anim_set_values(&a, lv_obj_get_y(p->pager), reveal ? LV_VER_RES : 0);
    lv_anim_set_time(&a, 250);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_start(&a);
    p->revealed = reveal;
    LOG_I("[proto] %s mouse layer", reveal ? "REVEAL ->" : "RETURN <-");
}

/* Central broker on the top reveal strip: classify start-zone + first-16px
   vector, lock the intent, act on it. Only the top band routes here; the
   pager body keeps native horizontal scroll, so a horizontal page swipe in
   the body never reaches this handler. */
static void top_strip_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t pt;
    lv_indev_get_point(indev, &pt);

    if (code == LV_EVENT_PRESSED)
    {
        g_start = pt;
        g_locked = false;
        g_intent = INTENT_NONE;
    }
    else if (code == LV_EVENT_PRESSING && !g_locked)
    {
        int32_t dx = pt.x - g_start.x;
        int32_t dy = pt.y - g_start.y;
        int32_t adx = dx < 0 ? -dx : dx;
        int32_t ady = dy < 0 ? -dy : dy;
        if (adx + ady < MOVE_THRESH_PX) return;   /* not enough travel yet */

        start_zone_t z = classify_zone(&g_start);
        if (z == ZONE_TOP && dy > 0 && ady > adx)
            g_intent = INTENT_REVEAL;
        else if (adx * 2 > ady * 3)               /* |dx| > |dy|*1.5 */
            g_intent = INTENT_PAGE;               /* started in top, but horizontal */
        else
            g_intent = INTENT_CURSOR;

        g_locked = true;
        LOG_I("[proto] INTENT=%s zone=TOP dx=%d dy=%d", intent_name(g_intent), (int)dx, (int)dy);
        if (p->intent_label)
            lv_label_set_text_fmt(p->intent_label, "intent: %s", intent_name(g_intent));

        if (g_intent == INTENT_REVEAL && !p->revealed)
            animate_reveal(true);
        /* INTENT_PAGE from the top band is logged but not forwarded in this
           proto — that forwarding is exactly the real-broker work for Phase 4. */
    }
}

static void bottom_bar_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED && p->revealed)
        animate_reveal(false);
}

static void build_pager(lv_obj_t *root)
{
    /* mouse layer first => lower z, sits behind the pager */
    p->mouse_layer = lv_obj_create(root);
    lv_obj_set_size(p->mouse_layer, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->mouse_layer, 0, 0);
    lv_obj_set_style_bg_color(p->mouse_layer, lv_color_hex(0x102030), 0);
    lv_obj_clear_flag(p->mouse_layer, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *ml_label = lv_label_create(p->mouse_layer);
    lv_label_set_text(ml_label, "MOUSE LAYER\n(trackpad placeholder)");
    lv_obj_set_style_text_color(ml_label, lv_color_white(), 0);
    lv_obj_center(ml_label);

    lv_obj_t *bar = lv_obj_create(p->mouse_layer);
    lv_obj_set_size(bar, 200, 36);
    lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x3A6EA5), 0);
    lv_obj_set_style_radius(bar, 18, 0);
    lv_obj_add_flag(bar, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(bar, bottom_bar_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bar_lbl = lv_label_create(bar);
    lv_label_set_text(bar_lbl, "^ bar -> return");
    lv_obj_set_style_text_color(bar_lbl, lv_color_white(), 0);
    lv_obj_center(bar_lbl);

    /* instruction pager on top: horizontal scroll-snap, 3 fake device tiles */
    p->pager = lv_obj_create(root);
    lv_obj_set_size(p->pager, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->pager, 0, 0);
    lv_obj_set_style_pad_all(p->pager, 0, 0);
    lv_obj_set_style_border_width(p->pager, 0, 0);
    lv_obj_set_flex_flow(p->pager, LV_FLEX_FLOW_ROW);
    lv_obj_set_scroll_dir(p->pager, LV_DIR_HOR);
    lv_obj_set_scroll_snap_x(p->pager, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scrollbar_mode(p->pager, LV_SCROLLBAR_MODE_OFF);

    static const uint32_t tile_colors[PROTO_DEVICE_COUNT] = {0x402020, 0x204020, 0x202040};
    for (int i = 0; i < PROTO_DEVICE_COUNT; i++)
    {
        lv_obj_t *tile = lv_obj_create(p->pager);
        lv_obj_set_size(tile, LV_HOR_RES, LV_VER_RES);
        lv_obj_set_style_bg_color(tile, lv_color_hex(tile_colors[i]), 0);
        lv_obj_set_style_radius(tile, 0, 0);
        lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t *lbl = lv_label_create(tile);
        lv_label_set_text_fmt(lbl, "DEVICE %c\n(instruction_list)\n\n<- swipe to page ->",
                              (char)('A' + i));
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_obj_center(lbl);
    }

    /* top reveal strip on top of everything, only top 40px */
    p->top_strip = lv_obj_create(root);
    lv_obj_set_size(p->top_strip, LV_HOR_RES, TOP_BAND_PX);
    lv_obj_set_pos(p->top_strip, 0, 0);
    lv_obj_set_style_bg_opa(p->top_strip, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(p->top_strip, 0, 0);
    lv_obj_add_flag(p->top_strip, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(p->top_strip, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(p->top_strip, top_strip_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(p->top_strip, top_strip_event_cb, LV_EVENT_PRESSING, NULL);

    /* debug HUD */
    p->intent_label = lv_label_create(root);
    lv_label_set_text(p->intent_label, "intent: NONE");
    lv_obj_set_style_text_color(p->intent_label, lv_color_hex(0xFFFF00), 0);
    lv_obj_align(p->intent_label, LV_ALIGN_TOP_MID, 0, 6);
}

static void on_start(lv_obj_t *scr)
{
    RT_ASSERT(p == NULL);
    p = (proto_pager_t *)rt_calloc(1, sizeof(proto_pager_t));
    if (!p) { LOG_E("proto alloc fail"); return; }

    p->root = lv_obj_create(scr);
    lv_obj_set_size(p->root, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(p->root, 0, 0);
    lv_obj_set_style_pad_all(p->root, 0, 0);
    lv_obj_set_style_border_width(p->root, 0, 0);
    lv_obj_set_style_bg_color(p->root, lv_color_black(), 0);
    lv_obj_clear_flag(p->root, LV_OBJ_FLAG_SCROLLABLE);

    build_pager(p->root);
    cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);
    LOG_I("[proto] pager up: swipe body=PAGE, drag-down top 40px=REVEAL, tap bar=RETURN");
}

static void on_stop(void)
{
    if (p)
    {
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
    case GUI_APP_MSG_ONSTOP:  on_stop(); break;
    default: break;
    }
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_PROTO_PAGER, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(flashlight), IMG_FLASHLIGHT, APP_ID_PROTO_PAGER, app_main, 1);

#endif /* BSP_USING_PC_SIMULATOR */
