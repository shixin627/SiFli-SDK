/**
 ******************************************************************************
 * @file   app_tv_remote.c
 * @author Skaiwalk software development team
 * @brief  TV remote — brand-neutral key surface on the watch.
 *
 * The watch has no IP stack, so nothing here talks to a TV. Every press emits a
 * BRAND-NEUTRAL verb over SKAI_LINK KEY_TV_CONTROL (0x22); the phone owns LAN
 * discovery, pairing and the per-vendor driver (Android TV Remote v2 / LG SSAP /
 * Roku ECP / Samsung Tizen ws). The phone pushes the binding back on KEY_TV_STATE
 * (0x23), which is all this app knows about "which TV".
 *
 * Deliberately does NOT hide buttons per platform: the bound TV can change under
 * the app at any time (the user walks into another room), and a remote whose keys
 * move around is worse than one whose keys occasionally no-op. Unmappable verbs
 * are dropped phone-side with a log.
 ******************************************************************************
 */
/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "custom_trans_anim.h"
#include "communicate_task.h"
#include "watch_system_interact.h"
#include <string.h>

#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
    #include "ui_img_helper.h"
#endif

#define DBG_TAG "app.tv_remote"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_TV_REMOTE

/*********************
 *      DEFINES
 *********************/
/* Skaiwalk UI tokens (references/tokens.md). The watch codebase has no central
   token module yet, so the values are named here and used through these names —
   never as bare literals at the callsite. */
#define TV_COL_BG            lv_color_hex(0x000000) /* systemBg     */
#define TV_COL_SURFACE       lv_color_hex(0x1C1C1E) /* systemGray6  */
#define TV_COL_SURFACE_NEST  lv_color_hex(0x2C2C2E) /* systemGray5  */
#define TV_COL_ACCENT        lv_color_hex(0x0091FF) /* systemBlue   */
#define TV_COL_GREEN         lv_color_hex(0x30D158) /* systemGreen  */
#define TV_COL_ORANGE        lv_color_hex(0xFF9230) /* systemOrange */
#define TV_COL_RED           lv_color_hex(0xFF4245) /* systemRed    */
#define TV_COL_LABEL         lv_color_hex(0xFFFFFF) /* label        */
#define TV_COL_LABEL_2ND     lv_color_hex(0xEBEBF5) /* secondaryLabel base (bluish) */
#define TV_OPA_LABEL_2ND     153                    /* 60% -> secondaryLabel */

#define TV_MOTION_FAST       220 /* motionFast — press feedback */

/* 466x466 round display geometry. Every value below is checked against the
   circle: a widget centred at (cx,cy) with half-size h must satisfy
   (|cx-233|+h)^2 + (|cy-233|+h)^2 <= 233^2 to stay off the clipped rim. */
#define TV_CX                (LV_HOR_RES / 2)
#define TV_CY                (LV_VER_RES / 2)
#define TV_DPAD_R            120 /* outer radius of the ring          */
#define TV_DPAD_OK_R         48  /* centre OK disc (96px >= 44pt hit) */
#define TV_SIDE_BTN_R        28  /* volume discs (56px)               */
#define TV_SIDE_BTN_X        52  /* from each edge, on the centre row */
#define TV_BOTTOM_BTN_R      32  /* back / home / power (64px)        */
#define TV_BOTTOM_BTN_CY     404
#define TV_BOTTOM_BTN_DX     96  /* spacing between bottom buttons    */
#define TV_STATUS_CY         58

/* Long-press auto-repeat: TV menus are lists, holding a direction should walk
   them. LVGL fires LONG_PRESSED_REPEAT at its own period; we forward every
   other one so the phone->TV path isn't flooded on a slow driver. */
#define TV_REPEAT_DIVIDER    2

/*********************
 *   STATIC STATE
 *********************/
typedef struct
{
    lv_obj_t *main_window;
    lv_obj_t *status_dot;
    lv_obj_t *status_label;
    lv_obj_t *dpad;       /* hit target for the whole ring + centre */
    lv_obj_t *ok_disc;    /* visual centre, not clickable           */
    lv_obj_t *arrow[4];   /* up, right, down, left visuals          */
} app_tv_remote_t;

static app_tv_remote_t *p_app = NULL;

/* Last binding pushed by the phone (0x23). Kept OUTSIDE the app struct so a state
   frame that lands while the app is closed is still shown when it reopens —
   otherwise every entry flashes "searching" until the next push. */
static char s_tv_name[40] = "";
static char s_tv_state[16] = "none";
static uint8_t s_repeat_tick = 0;

/* Direction indices for arrow[]; also the wedge order used by dpad_verb_at(). */
enum { TV_DIR_UP = 0, TV_DIR_RIGHT, TV_DIR_DOWN, TV_DIR_LEFT };

static const char *const TV_DIR_VERB[4] = { "up", "right", "down", "left" };

/*********************
 *   SEND HELPERS
 *********************/
/* One place where a press becomes a wire verb: haptic + uplink together, so a
   button that forgets the haptic can't exist. */
static void tv_send(const char *verb)
{
    motor_pattern_tap();
    commu_send_tv_key(verb);
}

/* Repeat path: no haptic (buzzing continuously while held is unpleasant) and
   rate-divided. */
static void tv_send_repeat(const char *verb)
{
    if ((++s_repeat_tick % TV_REPEAT_DIVIDER) != 0) return;
    commu_send_tv_key(verb);
}

/*********************
 *   STATUS LINE
 *********************/
/* Colour is never the sole indicator (Skaiwalk UI §1.4): the dot and the text
   always change together, and the text is the authoritative one. */
static void tv_status_render(void)
{
    if (p_app == NULL || !lv_obj_is_valid(p_app->status_label)) return;

    lv_color_t dot;
    const char *fallback;
    if (strcmp(s_tv_state, "ready") == 0)
    {
        dot = TV_COL_GREEN;
        fallback = LV_EXT_STR_GET_BY_KEY(tv_state_ready, "Connected");
    }
    else if (strcmp(s_tv_state, "pairing") == 0)
    {
        dot = TV_COL_ORANGE;
        fallback = LV_EXT_STR_GET_BY_KEY(tv_state_pairing, "Pair on phone");
    }
    else if (strcmp(s_tv_state, "scanning") == 0)
    {
        dot = TV_COL_ACCENT;
        fallback = LV_EXT_STR_GET_BY_KEY(tv_state_scanning, "Searching");
    }
    else if (strcmp(s_tv_state, "error") == 0)
    {
        dot = TV_COL_RED;
        fallback = LV_EXT_STR_GET_BY_KEY(tv_state_error, "Can't control");
    }
    else
    {
        dot = TV_COL_RED;
        fallback = LV_EXT_STR_GET_BY_KEY(tv_state_none, "No TV found");
    }

    /* Bound TV's name wins the line once we have one — "客廳電視" tells the user
       more than "Connected". The dot still carries the state. */
    if (s_tv_name[0] != '\0' && strcmp(s_tv_state, "ready") == 0)
        lv_label_set_text(p_app->status_label, s_tv_name);
    else
        lv_label_set_text(p_app->status_label, fallback);

    if (lv_obj_is_valid(p_app->status_dot))
        lv_obj_set_style_bg_color(p_app->status_dot, dot, 0);
}

/* Called on the GUI thread from tv_state_apply_pending() (SKAI_LINK 0x23).
   Must tolerate being called with the app closed — see s_tv_* rationale. */
void tv_remote_handle_state(const char *name, const char *platform,
                            const char *state, const char *detail)
{
    rt_strncpy(s_tv_name, name ? name : "", sizeof(s_tv_name) - 1);
    s_tv_name[sizeof(s_tv_name) - 1] = '\0';
    rt_strncpy(s_tv_state, state ? state : "none", sizeof(s_tv_state) - 1);
    s_tv_state[sizeof(s_tv_state) - 1] = '\0';
    LOG_I("tv state: %s / %s / %s%s%s", s_tv_name, platform ? platform : "?",
          s_tv_state, (detail && detail[0]) ? " - " : "",
          (detail && detail[0]) ? detail : "");
    tv_status_render();
}

/*********************
 *   D-PAD
 *********************/
/* Map a press point to a verb. Inside TV_DPAD_OK_R it is OK; outside, the larger
   axis wins, which gives four 90-degree wedges — the shape people expect from a
   physical clickpad, and forgiving of imprecise thumbs on a 466px screen.
   Returns NULL for presses outside the ring (the corners of the bounding box). */
static const char *dpad_verb_at(lv_point_t p, int *out_dir)
{
    lv_area_t a;
    lv_obj_get_coords(p_app->dpad, &a);
    int cx = (a.x1 + a.x2) / 2;
    int cy = (a.y1 + a.y2) / 2;
    int dx = p.x - cx;
    int dy = p.y - cy;
    int r2 = dx * dx + dy * dy;

    if (r2 <= TV_DPAD_OK_R * TV_DPAD_OK_R)
    {
        *out_dir = -1;
        return "ok";
    }
    if (r2 > TV_DPAD_R * TV_DPAD_R)
    {
        *out_dir = -1;
        return NULL; /* bounding-box corner, not the ring */
    }

    int dir;
    if (dx * dx >= dy * dy)
        dir = (dx > 0) ? TV_DIR_RIGHT : TV_DIR_LEFT;
    else
        dir = (dy > 0) ? TV_DIR_DOWN : TV_DIR_UP;
    *out_dir = dir;
    return TV_DIR_VERB[dir];
}

/* Press feedback (Skaiwalk UI §3.1): the pressed element brightens for
   motionFast then returns. No scale animation — on the ring the pressed area is
   a wedge, not an object, so brightening the arrow reads more clearly than
   scaling the whole pad. */
static void dpad_flash_reset_cb(lv_timer_t *t)
{
    if (p_app == NULL) return;
    if (lv_obj_is_valid(p_app->ok_disc))
        lv_obj_set_style_bg_color(p_app->ok_disc, TV_COL_SURFACE_NEST, 0);
    for (int i = 0; i < 4; i++)
        if (lv_obj_is_valid(p_app->arrow[i]))
            lv_obj_set_style_text_color(p_app->arrow[i], TV_COL_LABEL, 0);
}

static void dpad_flash(int dir)
{
    if (p_app == NULL) return;
    if (dir < 0)
    {
        if (lv_obj_is_valid(p_app->ok_disc))
            lv_obj_set_style_bg_color(p_app->ok_disc, TV_COL_ACCENT, 0);
    }
    else if (lv_obj_is_valid(p_app->arrow[dir]))
    {
        lv_obj_set_style_text_color(p_app->arrow[dir], TV_COL_ACCENT, 0);
    }
    lv_timer_t *t = lv_timer_create(dpad_flash_reset_cb, TV_MOTION_FAST, NULL);
    lv_timer_set_repeat_count(t, 1);
}

static void dpad_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED && code != LV_EVENT_LONG_PRESSED_REPEAT) return;
    if (p_app == NULL || !lv_obj_is_valid(p_app->dpad)) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (indev == NULL) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    int dir = -1;
    const char *verb = dpad_verb_at(p, &dir);
    if (verb == NULL) return;

    if (code == LV_EVENT_CLICKED)
    {
        dpad_flash(dir);
        tv_send(verb);
    }
    else if (dir >= 0)
    {
        /* Auto-repeat directions only. Holding OK must NOT repeat — on every one
           of the four platforms OK is "select", and repeating it double-opens
           whatever the user just launched. */
        tv_send_repeat(verb);
    }
}

/*********************
 *   PLAIN BUTTONS
 *********************/
/* user_data is the verb string (static storage), so one callback serves every
   round button. Volume repeats on hold; power/home/back deliberately do not. */
static void btn_event_cb(lv_event_t *e)
{
    const char *verb = (const char *)lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);
    if (verb == NULL) return;

    if (code == LV_EVENT_CLICKED)
    {
        tv_send(verb);
    }
    else if (code == LV_EVENT_LONG_PRESSED_REPEAT)
    {
        if (strcmp(verb, "volumeUp") == 0 || strcmp(verb, "volumeDown") == 0)
            tv_send_repeat(verb);
    }
}

/* Round icon button. `symbol` is an LV_SYMBOL_* glyph; `tint` colours it (power
   is systemRed so the one irreversible-feeling key is distinct by colour AND by
   its own glyph — §1.4). */
static lv_obj_t *tv_round_btn(lv_obj_t *parent, const char *symbol, int r,
                              lv_color_t tint, const char *verb)
{
    lv_obj_t *btn = lv_obj_create(parent);
    lv_obj_set_size(btn, r * 2, r * 2);
    lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(btn, TV_COL_SURFACE, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(btn, 0, 0);
    lv_obj_set_style_pad_all(btn, 0, 0);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
    /* Pressed state: nest colour, no ripple (Skaiwalk UI §3.1 forbids Material
       ripple). LVGL applies this for the press duration on its own. */
    lv_obj_set_style_bg_color(btn, TV_COL_SURFACE_NEST, LV_STATE_PRESSED);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, (void *)verb);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_LONG_PRESSED_REPEAT,
                        (void *)verb);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, symbol);
    lv_obj_set_style_text_color(lbl, tint, 0);
    lv_obj_center(lbl);
    return btn;
}

/*********************
 *   SCREEN
 *********************/
static lv_obj_t *create_tv_remote_screen(lv_obj_t *scr)
{
    lv_obj_t *bg = lv_obj_create(scr);
    lv_obj_set_size(bg, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(bg, TV_COL_BG, 0);
    lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(bg, 0, 0);
    lv_obj_set_style_pad_all(bg, 0, 0);
    lv_obj_set_style_radius(bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_center(bg);

    /* ── Status line ───────────────────────────────────────────────────── */
    lv_obj_t *status_row = lv_obj_create(bg);
    lv_obj_set_size(status_row, 300, 40);
    lv_obj_set_style_bg_opa(status_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(status_row, 0, 0);
    lv_obj_set_style_pad_all(status_row, 0, 0);
    lv_obj_clear_flag(status_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(status_row, LV_ALIGN_TOP_MID, 0, TV_STATUS_CY - 20);

    p_app->status_dot = lv_obj_create(status_row);
    lv_obj_set_size(p_app->status_dot, 12, 12);
    lv_obj_set_style_radius(p_app->status_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(p_app->status_dot, 0, 0);
    lv_obj_set_style_bg_color(p_app->status_dot, TV_COL_RED, 0);
    lv_obj_clear_flag(p_app->status_dot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(p_app->status_dot, LV_ALIGN_LEFT_MID, 60, 0);

    p_app->status_label = lv_label_create(status_row);
    lv_label_set_long_mode(p_app->status_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(p_app->status_label, 200);
    lv_obj_set_style_text_color(p_app->status_label, TV_COL_LABEL_2ND, 0);
    lv_obj_set_style_text_opa(p_app->status_label, TV_OPA_LABEL_2ND, 0);
    lv_obj_set_style_text_font(p_app->status_label,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_align_to(p_app->status_label, p_app->status_dot,
                    LV_ALIGN_OUT_RIGHT_MID, 8 /* s2 */, 0);

    /* ── D-pad ─────────────────────────────────────────────────────────── */
    p_app->dpad = lv_obj_create(bg);
    lv_obj_set_size(p_app->dpad, TV_DPAD_R * 2, TV_DPAD_R * 2);
    lv_obj_set_style_radius(p_app->dpad, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(p_app->dpad, TV_COL_SURFACE, 0);
    lv_obj_set_style_bg_opa(p_app->dpad, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(p_app->dpad, 0, 0);
    lv_obj_set_style_pad_all(p_app->dpad, 0, 0);
    lv_obj_clear_flag(p_app->dpad, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(p_app->dpad, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(p_app->dpad, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(p_app->dpad, dpad_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(p_app->dpad, dpad_event_cb, LV_EVENT_LONG_PRESSED_REPEAT,
                        NULL);

    static const char *const arrow_sym[4] = {
        LV_SYMBOL_UP, LV_SYMBOL_RIGHT, LV_SYMBOL_DOWN, LV_SYMBOL_LEFT
    };
    /* Arrow glyphs sit midway between the OK disc and the ring edge. They are
       decoration only — the hit test is dpad_verb_at(), so a thumb landing
       between two arrows still resolves to a wedge instead of missing. */
    const int arrow_off = (TV_DPAD_OK_R + TV_DPAD_R) / 2;
    const lv_coord_t arrow_xy[4][2] = {
        {  0, -arrow_off }, {  arrow_off, 0 }, { 0, arrow_off }, { -arrow_off, 0 }
    };
    for (int i = 0; i < 4; i++)
    {
        p_app->arrow[i] = lv_label_create(p_app->dpad);
        lv_label_set_text(p_app->arrow[i], arrow_sym[i]);
        lv_obj_set_style_text_color(p_app->arrow[i], TV_COL_LABEL, 0);
        lv_obj_align(p_app->arrow[i], LV_ALIGN_CENTER, arrow_xy[i][0],
                     arrow_xy[i][1]);
        /* Not clickable: the parent owns the hit test. */
        lv_obj_clear_flag(p_app->arrow[i], LV_OBJ_FLAG_CLICKABLE);
    }

    p_app->ok_disc = lv_obj_create(p_app->dpad);
    lv_obj_set_size(p_app->ok_disc, TV_DPAD_OK_R * 2, TV_DPAD_OK_R * 2);
    lv_obj_set_style_radius(p_app->ok_disc, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(p_app->ok_disc, TV_COL_SURFACE_NEST, 0);
    lv_obj_set_style_bg_opa(p_app->ok_disc, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(p_app->ok_disc, 0, 0);
    lv_obj_clear_flag(p_app->ok_disc, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(p_app->ok_disc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_center(p_app->ok_disc);

    lv_obj_t *ok_lbl = lv_label_create(p_app->ok_disc);
    lv_label_set_text(ok_lbl, LV_SYMBOL_OK);
    lv_obj_set_style_text_color(ok_lbl, TV_COL_LABEL, 0);
    lv_obj_center(ok_lbl);

    /* ── Volume, on the centre row where the circle is widest ──────────── */
    lv_obj_t *vol_down = tv_round_btn(bg, LV_SYMBOL_VOLUME_MID, TV_SIDE_BTN_R,
                                      TV_COL_LABEL, "volumeDown");
    lv_obj_align(vol_down, LV_ALIGN_LEFT_MID, TV_SIDE_BTN_X - TV_SIDE_BTN_R, 0);

    lv_obj_t *vol_up = tv_round_btn(bg, LV_SYMBOL_VOLUME_MAX, TV_SIDE_BTN_R,
                                    TV_COL_LABEL, "volumeUp");
    lv_obj_align(vol_up, LV_ALIGN_RIGHT_MID, -(TV_SIDE_BTN_X - TV_SIDE_BTN_R), 0);

    /* ── Bottom row: back / home / power ───────────────────────────────── */
    lv_obj_t *back = tv_round_btn(bg, LV_SYMBOL_LEFT, TV_BOTTOM_BTN_R,
                                  TV_COL_LABEL, "back");
    lv_obj_align(back, LV_ALIGN_TOP_MID, -TV_BOTTOM_BTN_DX,
                 TV_BOTTOM_BTN_CY - TV_BOTTOM_BTN_R);

    lv_obj_t *home = tv_round_btn(bg, LV_SYMBOL_HOME, TV_BOTTOM_BTN_R,
                                  TV_COL_LABEL, "home");
    lv_obj_align(home, LV_ALIGN_TOP_MID, 0, TV_BOTTOM_BTN_CY - TV_BOTTOM_BTN_R);

    lv_obj_t *power = tv_round_btn(bg, LV_SYMBOL_POWER, TV_BOTTOM_BTN_R,
                                   TV_COL_RED, "power");
    lv_obj_align(power, LV_ALIGN_TOP_MID, TV_BOTTOM_BTN_DX,
                 TV_BOTTOM_BTN_CY - TV_BOTTOM_BTN_R);

    return bg;
}

/*********************
 *   LIFECYCLE
 *********************/
static lv_obj_t *on_start(lv_obj_t *scr)
{
    RT_ASSERT(NULL == p_app);
    p_app = (app_tv_remote_t *)lv_mem_alloc(sizeof(app_tv_remote_t));
    if (!p_app)
    {
        LOG_E("Failed to allocate memory for TV remote app");
        return NULL;
    }
    memset(p_app, 0, sizeof(app_tv_remote_t));

    p_app->main_window = create_tv_remote_screen(scr);
    tv_status_render(); /* paint whatever binding we already know about */

    cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);
    return p_app->main_window;
}

static void on_resume(void)
{
    reset_lvgl_msg_handler();
    /* Ask the phone to (re)bind. The Data path has no pull, so without this an
       app opened after a binding change would show a stale line until the phone
       happened to push one. Cheap: one 20-byte L2 frame per app entry. */
    commu_send_tv_key("discover");
    s_repeat_tick = 0;
}

static void on_stop(void)
{
    if (p_app)
    {
        if (p_app->main_window)
        {
            lv_obj_del(p_app->main_window);
            p_app->main_window = NULL;
        }
        lv_mem_free(p_app);
        p_app = NULL;
    }
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
    {
        /* app_run 直接開啟不經 Main 狀態機，左緣右滑返回 bar 仍隱藏，這裡補開
           (see reference: left-swipe back is gated by the Main state machine). */
        extern void display_gesture_detect_objs(uint32_t idx, bool display);
        display_gesture_detect_objs(0, true);
        on_start(lv_scr_act());
        break;
    }
    case GUI_APP_MSG_ONRESUME:
        on_resume();
        break;
    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_TV_REMOTE, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(tv_remote), IMG_ITUNES, APP_ID_TV_REMOTE,
                   app_main, 1);

#else  /* !APP_ID_TV_REMOTE */

/* Keep the 0x23 downlink linkable when the app is compiled out — the parser
   calls this unconditionally. */
void tv_remote_handle_state(const char *name, const char *platform,
                            const char *state, const char *detail)
{
    (void)name; (void)platform; (void)state; (void)detail;
}

#endif /* APP_ID_TV_REMOTE */
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
