/**
 * @file   skaiapp_render.c
 * @brief  SkaiApp model → LVGL page (dark Skaiwalk look, 466px round display).
 *
 * The generated UI is a vertical flex flow inside a scrollable full-screen
 * container with round-screen safe padding; `row` items become nested
 * horizontal flex groups. Buttons dispatch to skaiapp_engine with the item
 * index — the module keeps a back-pointer to the CURRENT model, which the host
 * invalidates via skaiapp_render_detach() before freeing (UAF discipline).
 */
#include <string.h>
#include <stdio.h>
#include <rtthread.h>
#include "lvgl.h"
#include "lv_ext_resource_manager.h"
#include "skaiapp_render.h"
#include "skai/skai_dispatch.h"
#include "skaiapp_engine.h"
#include "ui_img_helper.h"
#include "ui_helper.h"
#include <dfs_posix.h>
#include "skaiapp_store.h"
#include "app_mainmenu.h"      /* gui_set_brightness */
#include "watch_global_data.h" /* SkaiWatchSys.brightness */

#define DBG_TAG "skaiapp.rend"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static const skaiapp_model_t *s_model = NULL;
static skaiapp_render_ctx_t *s_ctx = NULL;
static char s_app_id[SKAIAPP_ID_MAX];
/* Parent of the page — where the full-screen picture view is parented, so it
   outlives neither the app nor a page swap (both go through _detach()). */
static lv_obj_t *s_root = NULL;
static lv_obj_t *s_photo_full = NULL;
static lv_obj_t *s_photo_picker = NULL;
static uint16_t s_brightness_saved = 0;

/* ── tokens ── */

static lv_color_t resolve_color(uint8_t col, uint8_t accent, bool for_text)
{
    switch (col)
    {
    case SKAIAPP_COL_WHITE:  return lv_color_hex(0xFFFFFF);
    case SKAIAPP_COL_GRAY:   return lv_color_hex(0x8E8E93);
    case SKAIAPP_COL_RED:    return lv_color_hex(0xFF3B30);
    case SKAIAPP_COL_ORANGE: return lv_color_hex(0xFF9500);
    case SKAIAPP_COL_YELLOW: return lv_color_hex(0xFFD60A);
    case SKAIAPP_COL_GREEN:  return lv_color_hex(0x30D158);
    case SKAIAPP_COL_BLUE:   return lv_color_hex(0x0A84FF);
    case SKAIAPP_COL_PURPLE: return lv_color_hex(0xBF5AF2);
    case SKAIAPP_COL_ACCENT:
        return resolve_color(accent, SKAIAPP_COL_BLUE, for_text);
    case SKAIAPP_COL_DEFAULT:
    default:
        return for_text ? lv_color_hex(0xFFFFFF)
                        : resolve_color(accent, SKAIAPP_COL_BLUE, false);
    }
}

/* image box per size token: m / l / full. "full" is the widest a picture can
   be without touching the round screen's clipped sides (page pad_hor is 52). */
static const lv_coord_t k_photo_box[3] = { 180, 260, 362 };

/* Full-screen picture view: the box a picture is fitted into, and the backlight
   it is shown at (a barcode has to survive a shop scanner). */
#define SKAIAPP_PHOTO_FULL_BOX        440
#define SKAIAPP_PHOTO_FULL_BRIGHTNESS 100

static const lv_font_t *resolve_font(uint8_t size)
{
    static const int8_t k_rel[4] = { -2, 0, 1, 3 }; /* s m l xl */
    int8_t rel = k_rel[(size < 4) ? size : 1];
    return LV_EXT_FONT_GET(get_system_font_size(rel));
}

/* mirror of k_icons order in skaiapp_pkg.c — v2 swaps in dedicated assets */
const void *skaiapp_render_icon_src(uint8_t icon_enum)
{
    switch (icon_enum)
    {
    case 0:  return IMG_ACTIVITY;   /* water   (closest shipped asset) */
    case 1:  return IMG_ALARM_2;    /* timer   */
    case 2:  return IMG_HEART_RATE; /* heart   */
    case 3:  return IMG_WORKOUT;    /* steps   */
    case 4:  return IMG_CHARGING;   /* battery */
    case 5:  return IMG_ALARM;      /* bell    */
    case 6:  return IMG_LOGO;       /* star    */
    case 7:  return IMG_ACTIVITY;   /* sun     */
    case 8:  return IMG_SLEEP;      /* moon    */
    case 9:  return IMG_NOTE;       /* coffee  */
    case 10: return IMG_NOTE;       /* pill    */
    case 11: return IMG_WORKOUT;    /* run     */
    default: return IMG_LOGO;
    }
}

/* ── bind formatting ── */

static void fmt_bind(const skaiapp_witem_t *it, char *buf, size_t cap)
{
    buf[0] = '\0';
    switch (it->bind)
    {
    /* Every watch-state bind, present and future, goes through here — the
       capability supplies its own value and display format, so a new one needs
       no case of its own (ADR-0019 Phase 2). Missing readings all render as
       "--", so an absent heart rate looks like any other absent value. */
    case SKAIAPP_BIND_CAP:
        skai_cap_render(skai_cap_at(it->bind_idx), buf, (uint32_t)cap);
        break;
    case SKAIAPP_BIND_TIMER:
    {
        uint32_t rem = 0, dur = 0;
        bool running = false;
        if (skaiapp_engine_timer_query(s_app_id, it->bind_idx, &rem, &running, &dur))
        {
            uint32_t s_total = (rem + 999) / 1000;
            if (s_total >= 3600)
            {
                rt_snprintf(buf, cap, "%u:%02u:%02u", (unsigned)(s_total / 3600),
                            (unsigned)((s_total / 60) % 60), (unsigned)(s_total % 60));
            }
            else
            {
                rt_snprintf(buf, cap, "%02u:%02u", (unsigned)(s_total / 60),
                            (unsigned)(s_total % 60));
            }
        }
        break;
    }
    /* A counter: the number the user has been tapping, plus the package's own
       short unit ("3 杯"). The unit is APPENDED, never used as a format — it is
       model-authored text and must never reach a printf conversion. */
    case SKAIAPP_BIND_VAR:
    {
        int32_t v = 0;
        if (skaiapp_engine_var_get(s_app_id, it->bind_idx, &v, NULL, NULL))
        {
            const char *unit = (s_model != NULL && it->bind_idx >= 0 &&
                                it->bind_idx < s_model->n_vars)
                                   ? skaiapp_model_text(s_model,
                                                        s_model->var_unit_off[it->bind_idx])
                                   : "";
            if (unit[0] != '\0')
            {
                rt_snprintf(buf, cap, "%d %s", (int)v, unit);
            }
            else
            {
                rt_snprintf(buf, cap, "%d", (int)v);
            }
        }
        break;
    }
    case SKAIAPP_BIND_REMINDER:
    {
        uint16_t nxt = skaiapp_engine_reminder_next(s_app_id, it->bind_idx);
        if (nxt == 0xFFFF)
        {
            rt_snprintf(buf, cap, "--:--");
        }
        else
        {
            char tmp[16];
            ui_time_format_hhmm(tmp, sizeof(tmp), (uint8_t)(nxt / 60),
                                (uint8_t)(nxt % 60));
            rt_snprintf(buf, cap, "%s", tmp);
        }
        break;
    }
    /* SKAIAPP_BIND_MEMO is NOT handled here — memo text can be up to 200 chars,
       far bigger than this buffer, so the value render path sets the label text
       straight from the strpool via memo_text_of(). */
    default:
        break;
    }
}

/* Memo text pointer straight out of the model strpool (lv_label copies it), so
   long user notes bypass fmt_bind's small stack buffer. "—" when empty. */
static const char *memo_text_of(const skaiapp_witem_t *it)
{
    if (s_model != NULL && it->bind_idx >= 0 && it->bind_idx < s_model->n_memos)
    {
        const char *txt = skaiapp_model_text(s_model, s_model->memo_text_off[it->bind_idx]);
        if (txt[0] != '\0')
        {
            return txt;
        }
    }
    return "—";
}

static int32_t gauge_percent(const skaiapp_witem_t *it)
{
    switch (it->bind)
    {
    case SKAIAPP_BIND_CAP:
    {
        int32_t v;
        if (!skai_cap_value(skai_cap_at(it->bind_idx), &v))
            return 0; /* no reading — an empty gauge, not a wrong one */
        /* max is the package's goal, or the legacy default the parser filled
           in for v0 bind keys. Without one the value is already a percentage
           (battery level, chance of rain). */
        if (it->max > 0)
        {
            v = (int32_t)((int64_t)v * 100 / it->max);
        }
        return (v > 100) ? 100 : ((v < 0) ? 0 : v);
    }
    case SKAIAPP_BIND_VAR:
    {
        int32_t v = 0, mn = 0, mx = 0;
        if (!skaiapp_engine_var_get(s_app_id, it->bind_idx, &v, &mn, &mx))
        {
            return 0;
        }
        /* The widget's own "max" wins when the package states a goal; otherwise
           fill across the counter's declared range. Neither present (an
           unbounded counter) → nothing sensible to fill, so stay empty. */
        int32_t lo = 0, hi = it->max;
        if (hi <= 0)
        {
            if (mx <= mn || mx >= SKAIAPP_VAR_MAX)
            {
                return 0;
            }
            lo = mn;
            hi = mx;
        }
        if (hi <= lo)
        {
            return 0;
        }
        int32_t pct = (int32_t)(((int64_t)(v - lo) * 100) / (hi - lo));
        return (pct > 100) ? 100 : ((pct < 0) ? 0 : pct);
    }
    case SKAIAPP_BIND_TIMER:
    {
        uint32_t rem = 0, dur = 0;
        bool running = false;
        if (skaiapp_engine_timer_query(s_app_id, it->bind_idx, &rem, &running, &dur)
            && dur > 0)
        {
            return (int32_t)((uint64_t)rem * 100 / ((uint64_t)dur * 1000u));
        }
        return 0;
    }
    default:
        return 0;
    }
}

/* ── pictures ── */

/* The file the phone transferred for this slot; "" until the user picks one. */
static const char *photo_src_of(const skaiapp_witem_t *it)
{
    if (s_model != NULL && it->bind == SKAIAPP_BIND_PHOTO && it->bind_idx >= 0 &&
        it->bind_idx < s_model->n_photos)
    {
        return s_model->photo_src[it->bind_idx];
    }
    return "";
}

/* Zoom that fits a `w`x`h` image inside a `box`-wide square (256 = 1:1). Only
   ever shrinks — blowing a small picture up just makes it mushy. */
static uint16_t fit_zoom(lv_coord_t w, lv_coord_t h, lv_coord_t box)
{
    lv_coord_t longest = (w > h) ? w : h;
    if (longest <= 0 || longest <= box)
    {
        return 256;
    }
    return (uint16_t)((256 * box) / longest);
}

static void photo_full_close(void)
{
    if (s_photo_full != NULL)
    {
        lv_obj_del(s_photo_full);
        s_photo_full = NULL;
        /* Put the backlight back where the user had it. */
        gui_set_brightness(s_brightness_saved, false);
    }
}

static void photo_full_click_cb(lv_event_t *e)
{
    (void)e;
    photo_full_close();
}

/* Tap a picture → show it as big as the round screen allows, at full backlight.
   That combination is the whole point for a barcode/QR at a checkout counter:
   a dim, letterboxed code does not scan. Tap anywhere to come back. */
static void photo_click_cb(lv_event_t *e)
{
    if (s_root == NULL || s_model == NULL)
    {
        return;
    }
    const char *path = (const char *)lv_event_get_user_data(e);
    if (path == NULL || path[0] == '\0')
    {
        return;
    }
    photo_full_close();

    s_photo_full = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_photo_full);
    lv_obj_set_size(s_photo_full, LV_PCT(100), LV_PCT(100));
    /* White, not black: a barcode/QR is dark-on-light and scanners want the
       quiet zone around it to be light too. */
    lv_obj_set_style_bg_color(s_photo_full, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(s_photo_full, LV_OPA_COVER, 0);
    lv_obj_add_flag(s_photo_full, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_photo_full, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_photo_full, photo_full_click_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *img = lv_img_create(s_photo_full);
    lv_img_set_src(img, path);
    lv_img_header_t header;
    if (lv_img_decoder_get_info(path, &header) == LV_RES_OK)
    {
        uint16_t zoom = fit_zoom(header.w, header.h, SKAIAPP_PHOTO_FULL_BOX);
        if (zoom != 256)
        {
            /* zoom pivots on the centre and must be allowed to draw outside the
               object's own (unscaled) box — see the LVGL zoom/overflow rule. */
            lv_img_set_pivot(img, header.w / 2, header.h / 2);
            lv_obj_add_flag(img, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
            lv_img_set_zoom(img, zoom);
        }
    }
    lv_obj_center(img);

    s_brightness_saved = SkaiWatchSys.brightness;
    gui_set_brightness(SKAIAPP_PHOTO_FULL_BRIGHTNESS, false);
}

/* ── running one of the user's phone Actions ── */

/* Flash a one-line verdict over the page for ~1.5 s. The watch cannot know
   whether the Action itself succeeded (it runs on the phone, possibly for
   seconds), so the words are about the HANDOFF — "sent" / "no link" — and
   never claim the action ran. */
static void toast(const char *text)
{
    if (s_root == NULL)
    {
        return;
    }
    lv_obj_t *t = lv_obj_create(s_root);
    lv_obj_set_size(t, 260, 72);
    lv_obj_set_style_radius(t, 20, 0);
    lv_obj_set_style_bg_color(t, lv_color_hex(0x2C2C2E), 0);
    lv_obj_set_style_bg_opa(t, LV_OPA_90, 0);
    lv_obj_set_style_border_width(t, 0, 0);
    lv_obj_clear_flag(t, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(t, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(t, LV_ALIGN_CENTER, 0, 120);
    lv_obj_t *l = lv_label_create(t);
    lv_obj_set_style_text_font(l, resolve_font(1), 0);
    lv_obj_set_style_text_color(l, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(l, 220);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    /* Self-deleting: no stored pointer to dangle when the page closes under it
       (lv_obj_del_delayed cancels itself if the object is deleted first). */
    lv_obj_del_delayed(t, 1500);
}

/* The watch does not interpret the Action at all — it echoes the id the package
   carries back up (0x28) and the PHONE runs the user's saved Action. That is
   what keeps this open-ended: anything the phone can be taught to do becomes a
   button here without a firmware change. */
static void run_phone_action(const skaiapp_witem_t *it)
{
    extern bool commu_send_skaiapp_action(const char *app_id, const char *action_id);

    if (s_model == NULL || it->action_off == 0xFFFF)
    {
        return;
    }
    const char *action_id = skaiapp_model_text(s_model, it->action_off);
    if (action_id[0] == '\0')
    {
        return;
    }
    if (commu_send_skaiapp_action(s_app_id, action_id))
    {
        LOG_I("phone.run %s (app=%s)", action_id, s_app_id);
        toast(LV_EXT_STR_GET_BY_KEY(skaiapp_sent_to_phone, "Sent to your phone"));
    }
    else
    {
        LOG_W("phone.run %s FAILED (link down?)", action_id);
        toast(LV_EXT_STR_GET_BY_KEY(skaiapp_phone_not_linked, "Phone not connected"));
    }
}

/* ── choosing a picture, on the watch ── */

/* The watch's own album — the same folder the phone's photo sync fills and the
   Photo app lists. A mini-app's picture is chosen FROM HERE, on the wrist, which
   is why this screen exists at all: the package can name a picture, but only the
   person can say which one. */
#define SKAIAPP_ALBUM_DIR  "/photo"
#define SKAIAPP_PICK_THUMB 120
#define SKAIAPP_PICK_MAX   24

static void photo_picker_close(void)
{
    if (s_photo_picker != NULL)
    {
        lv_obj_del(s_photo_picker);
        s_photo_picker = NULL;
    }
}

static void picker_dismiss_cb(lv_event_t *e)
{
    (void)e;
    photo_picker_close();
}

/* Tap a thumbnail: record it against the slot, drop the picker, and let the host
   app's tick notice the store's new generation and rebuild the page with it.
   The path string is owned by the thumbnail's user data (strdup'd at build time)
   and freed with it. */
static void picker_choose_cb(lv_event_t *e)
{
    const char *path = (const char *)lv_event_get_user_data(e);
    lv_obj_t *cell = lv_event_get_target(e);
    int slot = (int)(intptr_t)lv_obj_get_user_data(cell);
    if (path == NULL || s_app_id[0] == '\0')
    {
        return;
    }
    if (skaiapp_store_set_photo_src(s_app_id, (uint8_t)slot, path) != 0)
    {
        LOG_W("could not record picked photo %s", path);
    }
    photo_picker_close();
}

static void picker_free_path_cb(lv_event_t *e)
{
    char *path = (char *)lv_event_get_user_data(e);
    rt_free(path);
}

/* A picture is a picture: only the album's own image blobs are offered. */
static bool picker_is_image(const char *name)
{
    size_t n = strlen(name);
    return (n > 4) && (strcmp(&name[n - 4], ".bin") == 0);
}

/**
 * Full-screen chooser over the album, for photo slot `slot`.
 *
 * Bounded on purpose: at most SKAIAPP_PICK_MAX thumbnails. Every thumbnail is a
 * decode from NAND on the LVGL thread, and an unbounded album would stall the UI
 * the way the avatar sweep once did — the cap is the same discipline, not a
 * layout choice.
 */
static void photo_picker_open(int slot)
{
    if (s_root == NULL)
    {
        return;
    }
    photo_picker_close();

    s_photo_picker = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_photo_picker);
    lv_obj_set_size(s_photo_picker, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_photo_picker, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_photo_picker, LV_OPA_COVER, 0);
    lv_obj_add_flag(s_photo_picker, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_pad_top(s_photo_picker, 60, 0);
    lv_obj_set_style_pad_bottom(s_photo_picker, 60, 0);
    lv_obj_set_style_pad_hor(s_photo_picker, 40, 0);
    lv_obj_set_flex_flow(s_photo_picker, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(s_photo_picker, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_photo_picker, 10, 0);
    lv_obj_set_style_pad_column(s_photo_picker, 10, 0);
    lv_obj_set_scrollbar_mode(s_photo_picker, LV_SCROLLBAR_MODE_OFF);
    /* Tapping the backdrop (not a thumbnail) leaves without choosing. */
    lv_obj_add_event_cb(s_photo_picker, picker_dismiss_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_photo_picker);
    lv_obj_set_style_text_font(title, resolve_font(1), 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(title, LV_EXT_STR_GET_BY_KEY(skaiapp_choose_photo, "Choose a picture"));

    DIR *dir = opendir(SKAIAPP_ALBUM_DIR);
    if (dir == NULL)
    {
        lv_obj_t *empty = lv_label_create(s_photo_picker);
        lv_obj_set_style_text_font(empty, resolve_font(0), 0);
        lv_obj_set_style_text_color(empty, lv_color_hex(0x8E8E93), 0);
        lv_label_set_long_mode(empty, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(empty, LV_PCT(90));
        lv_obj_set_style_text_align(empty, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(empty,
                          LV_EXT_STR_GET_BY_KEY(skaiapp_album_empty,
                                                "No pictures on the watch yet — send some from your phone"));
        return;
    }
    struct dirent *ent;
    int shown = 0;
    while ((ent = readdir(dir)) != NULL && shown < SKAIAPP_PICK_MAX)
    {
        if (ent->d_type != DT_REG || !picker_is_image(ent->d_name))
        {
            continue;
        }
        char full[SKAIAPP_PATH_MAX];
        if (rt_snprintf(full, sizeof(full), SKAIAPP_ALBUM_DIR "/%s", ent->d_name) >=
            (int)sizeof(full))
        {
            continue; /* a name that cannot be stored is a picture we cannot offer */
        }
        lv_img_header_t header;
        if (lv_img_decoder_get_info(full, &header) != LV_RES_OK)
        {
            continue;
        }
        char *owned = (char *)rt_malloc(strlen(full) + 1);
        if (owned == NULL)
        {
            break;
        }
        strcpy(owned, full);

        lv_obj_t *cell = lv_obj_create(s_photo_picker);
        lv_obj_remove_style_all(cell);
        lv_obj_set_size(cell, SKAIAPP_PICK_THUMB, SKAIAPP_PICK_THUMB);
        lv_obj_set_style_radius(cell, 12, 0);
        lv_obj_set_style_clip_corner(cell, true, 0);
        lv_obj_set_style_bg_color(cell, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_bg_opa(cell, LV_OPA_COVER, 0);
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(cell, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_user_data(cell, (void *)(intptr_t)slot);
        lv_obj_add_event_cb(cell, picker_choose_cb, LV_EVENT_CLICKED, owned);
        lv_obj_add_event_cb(cell, picker_free_path_cb, LV_EVENT_DELETE, owned);

        lv_obj_t *thumb = lv_img_create(cell);
        lv_img_set_src(thumb, full);
        uint16_t zoom = fit_zoom(header.w, header.h, SKAIAPP_PICK_THUMB);
        if (zoom != 256)
        {
            lv_img_set_pivot(thumb, header.w / 2, header.h / 2);
            lv_obj_add_flag(thumb, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
            lv_img_set_zoom(thumb, zoom);
        }
        lv_obj_center(thumb);
        shown++;
    }
    closedir(dir);

    if (shown == 0)
    {
        lv_obj_t *empty = lv_label_create(s_photo_picker);
        lv_obj_set_style_text_font(empty, resolve_font(0), 0);
        lv_obj_set_style_text_color(empty, lv_color_hex(0x8E8E93), 0);
        lv_label_set_long_mode(empty, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(empty, LV_PCT(90));
        lv_obj_set_style_text_align(empty, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(empty,
                          LV_EXT_STR_GET_BY_KEY(skaiapp_album_empty,
                                                "No pictures on the watch yet — send some from your phone"));
    }
}

/* The placeholder is the invitation: tap an empty frame to choose. */
static void photo_pick_cb(lv_event_t *e)
{
    photo_picker_open((int)(intptr_t)lv_event_get_user_data(e));
}

/* A long press on a picture that is already set = choose a different one. */
static void photo_repick_cb(lv_event_t *e)
{
    photo_picker_open((int)(intptr_t)lv_event_get_user_data(e));
}

/* ── button dispatch ── */

static void btn_event_cb(lv_event_t *e)
{
    if (s_model == NULL)
    {
        return;
    }
    uint32_t idx = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
    if (idx >= s_model->n_items)
    {
        return;
    }
    const skaiapp_witem_t *it = &s_model->items[idx];
    switch (it->action)
    {
    case SKAIAPP_ACT_TIMER_START:
        skaiapp_engine_timer_start(s_app_id, it->action_idx);
        break;
    case SKAIAPP_ACT_TIMER_PAUSE:
        skaiapp_engine_timer_pause(s_app_id, it->action_idx);
        break;
    case SKAIAPP_ACT_TIMER_RESET:
        skaiapp_engine_timer_reset(s_app_id, it->action_idx);
        break;
    case SKAIAPP_ACT_REMINDER_TOGGLE:
        skaiapp_engine_reminder_toggle(s_app_id, it->action_idx);
        break;
    case SKAIAPP_ACT_VAR_ADD:
        skaiapp_engine_var_add(s_app_id, it->action_idx, it->action_arg);
        break;
    case SKAIAPP_ACT_VAR_SET:
        skaiapp_engine_var_set(s_app_id, it->action_idx, it->action_arg);
        break;
    case SKAIAPP_ACT_PHONE_RUN:
        run_phone_action(it);
        break;
    default:
        break;
    }
    if (s_model != NULL && s_ctx != NULL)
    {
        skaiapp_render_refresh(s_model, s_ctx);
    }
}

/* 🎤 on a memo: tell the phone which memo to fill (KEY_SKAIAPP_VOICE), then start
   streaming mic audio with the MEMO intent. The phone runs STT and writes the
   transcript back via setMemoText (which re-pushes the package, rebuilding the
   page with the new text). ADR-0037 watch-side voice memo. */
static void memo_voice_cb(lv_event_t *e)
{
    extern bool commu_send_skaiapp_voice(const char *app_id, const char *memo_id);
    /* interact_memo_v2t_input does vad_init() + pending intent + start — the
       vad_init() is essential: the mic-audio-send gate is `vad_inst && ...`, and
       start_voice_recognition alone never creates vad_inst (only vad_init does),
       so without it the mic subscribes but zero audio ever reaches the phone. */
    extern void interact_memo_v2t_input(void);

    if (s_model == NULL)
    {
        return;
    }
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= s_model->n_memos || s_model->memo_id[slot][0] == '\0')
    {
        return;
    }
    if (!commu_send_skaiapp_voice(s_app_id, s_model->memo_id[slot]))
    {
        LOG_W("memo voice: uplink failed (phone not connected?)");
        return;
    }
    interact_memo_v2t_input();
    LOG_I("memo voice started: app=%s memo=%s", s_app_id, s_model->memo_id[slot]);
}

/* ── builders ── */

static lv_obj_t *build_leaf(lv_obj_t *parent, const skaiapp_model_t *m,
                            uint32_t idx)
{
    const skaiapp_witem_t *it = &m->items[idx];
    switch (it->wtype)
    {
    case SKAIAPP_W_VALUE:
        if (it->bind == SKAIAPP_BIND_MEMO)
        {
            /* memo = user note: the wrapping text PLUS a 🎤 button that starts
               on-watch voice fill for this memo (ADR-0037). Returns a column
               container; refresh skips memo (text is static between re-pushes). */
            lv_obj_t *col = lv_obj_create(parent);
            lv_obj_remove_style_all(col);
            lv_obj_set_size(col, LV_PCT(100), LV_SIZE_CONTENT);
            lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                                  LV_FLEX_ALIGN_CENTER);
            lv_obj_set_style_pad_row(col, 8, 0);

            lv_obj_t *lbl = lv_label_create(col);
            lv_obj_set_style_text_font(lbl, resolve_font(it->size), 0);
            lv_obj_set_style_text_color(lbl,
                                        resolve_color(it->color, m->accent, true), 0);
            lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_max_width(lbl, 320, 0);
            lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
            lv_label_set_text(lbl, memo_text_of(it));

            lv_obj_t *mic = lv_btn_create(col);
            lv_obj_set_size(mic, 56, 56);
            lv_obj_set_style_radius(mic, 28, 0);
            lv_obj_set_style_bg_color(mic,
                                      resolve_color(SKAIAPP_COL_ACCENT, m->accent, false), 0);
            lv_obj_set_style_shadow_width(mic, 0, 0);
            lv_obj_add_event_cb(mic, memo_voice_cb, LV_EVENT_CLICKED,
                                (void *)(intptr_t)it->bind_idx);
            lv_obj_t *ml = lv_label_create(mic);
            lv_label_set_text(ml, LV_SYMBOL_AUDIO); /* mic-ish glyph */
            lv_obj_center(ml);
            return col;
        }
        /* fall through to the plain label/value path */
    case SKAIAPP_W_LABEL:
    {
        lv_obj_t *lbl = lv_label_create(parent);
        lv_obj_set_style_text_font(lbl, resolve_font(it->size), 0);
        lv_obj_set_style_text_color(lbl,
                                    resolve_color(it->color, m->accent, true), 0);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_max_width(lbl, 340, 0); /* stay inside the round column */
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        if (it->wtype == SKAIAPP_W_LABEL)
        {
            lv_label_set_text(lbl, skaiapp_model_text(m, it->text_off));
        }
        else
        {
            char buf[48];
            fmt_bind(it, buf, sizeof(buf));
            lv_label_set_text(lbl, buf);
        }
        return lbl;
    }
    case SKAIAPP_W_ICON:
    {
        lv_obj_t *img = lv_img_create(parent);
        lv_img_set_src(img, skaiapp_render_icon_src(it->icon));
        return img;
    }
    case SKAIAPP_W_ARC:
    {
        lv_obj_t *arc = lv_arc_create(parent);
        int sz = (it->size == 0) ? 140 : 200;
        lv_obj_set_size(arc, sz, sz);
        lv_arc_set_rotation(arc, 270);
        lv_arc_set_bg_angles(arc, 0, 360);
        lv_arc_set_range(arc, 0, 100);
        lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
        lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_arc_width(arc, 10, LV_PART_MAIN);
        lv_obj_set_style_arc_width(arc, 10, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(arc, lv_color_hex(0x2C2C2E), LV_PART_MAIN);
        lv_obj_set_style_arc_color(arc,
                                   resolve_color(it->color, m->accent, false),
                                   LV_PART_INDICATOR);
        lv_arc_set_value(arc, gauge_percent(it));
        return arc;
    }
    case SKAIAPP_W_BAR:
    {
        lv_obj_t *bar = lv_bar_create(parent);
        lv_obj_set_size(bar, 300, 12);
        lv_bar_set_range(bar, 0, 100);
        lv_obj_set_style_radius(bar, 6, LV_PART_MAIN);
        lv_obj_set_style_radius(bar, 6, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(bar, lv_color_hex(0x2C2C2E), LV_PART_MAIN);
        lv_obj_set_style_bg_color(bar,
                                  resolve_color(it->color, m->accent, false),
                                  LV_PART_INDICATOR);
        lv_bar_set_value(bar, gauge_percent(it), LV_ANIM_OFF);
        return bar;
    }
    case SKAIAPP_W_BUTTON:
    {
        lv_obj_t *btn = lv_btn_create(parent);
        lv_obj_set_height(btn, 64);
        lv_obj_set_style_radius(btn, 32, 0);
        lv_obj_set_style_pad_hor(btn, 28, 0);
        if (it->ghost)
        {
            lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(btn, 2, 0);
            lv_obj_set_style_border_color(btn, lv_color_hex(0x48484A), 0);
        }
        else
        {
            lv_obj_set_style_bg_color(btn,
                                      resolve_color(SKAIAPP_COL_ACCENT, m->accent, false), 0);
        }
        lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED,
                            (void *)(uintptr_t)idx);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_obj_set_style_text_font(lbl, resolve_font(1), 0);
        lv_label_set_text(lbl, skaiapp_model_text(m, it->text_off));
        lv_obj_center(lbl);
        return btn;
    }
    case SKAIAPP_W_IMAGE:
    {
        const char *path = photo_src_of(it);
        lv_img_header_t header;
        /* No picture yet (or the file went missing) → say so, in the same box
           the picture will occupy, so the page does not reflow when it lands. */
        if (path[0] == '\0' || lv_img_decoder_get_info(path, &header) != LV_RES_OK)
        {
            lv_obj_t *ph = lv_obj_create(parent);
            lv_obj_set_size(ph, k_photo_box[(it->size < 3) ? it->size : 1], 120);
            lv_obj_set_style_radius(ph, 16, 0);
            lv_obj_set_style_bg_color(ph, lv_color_hex(0x1C1C1E), 0);
            lv_obj_set_style_border_width(ph, 2, 0);
            lv_obj_set_style_border_color(ph, lv_color_hex(0x48484A), 0);
            lv_obj_clear_flag(ph, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(ph, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(ph, photo_pick_cb, LV_EVENT_CLICKED,
                                (void *)(intptr_t)it->bind_idx);
            lv_obj_t *hint = lv_label_create(ph);
            lv_obj_set_style_text_font(hint, resolve_font(0), 0);
            lv_obj_set_style_text_color(hint, lv_color_hex(0x8E8E93), 0);
            lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
            lv_obj_set_width(hint, LV_PCT(90));
            lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);
            lv_label_set_text(hint,
                              LV_EXT_STR_GET_BY_KEY(skaiapp_tap_to_pick_photo,
                                                    "Tap to choose a picture"));
            lv_obj_center(hint);
            return ph;
        }
        lv_coord_t box = k_photo_box[(it->size < 3) ? it->size : 1];
        uint16_t zoom = fit_zoom(header.w, header.h, box);
        /* The image sits in a fixed-size container: an lv_img scaled with
           set_zoom keeps its ORIGINAL size for layout, so a flex column would
           reserve the wrong height and the neighbours would overlap it. */
        lv_obj_t *cell = lv_obj_create(parent);
        lv_obj_remove_style_all(cell);
        lv_obj_set_size(cell, (lv_coord_t)((int32_t)header.w * zoom / 256),
                        (lv_coord_t)((int32_t)header.h * zoom / 256));
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(cell, LV_OBJ_FLAG_CLICKABLE);
        /* The path lives in the model, which outlives the page (the host frees
           it only after skaiapp_render_detach()), so passing it as user data
           needs no copy. */
        lv_obj_add_event_cb(cell, photo_click_cb, LV_EVENT_CLICKED, (void *)path);
        /* Long press = change it. Tap stays "show it big", which is what the
           person does at a checkout counter; changing the picture is the rarer
           intent and must not be one accidental tap away. */
        lv_obj_add_event_cb(cell, photo_repick_cb, LV_EVENT_LONG_PRESSED,
                            (void *)(intptr_t)it->bind_idx);

        lv_obj_t *img = lv_img_create(cell);
        lv_img_set_src(img, path);
        if (zoom != 256)
        {
            lv_img_set_pivot(img, header.w / 2, header.h / 2);
            lv_obj_add_flag(img, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
            lv_img_set_zoom(img, zoom);
        }
        lv_obj_center(img);
        return cell;
    }
    case SKAIAPP_W_SPACER:
    {
        lv_obj_t *sp = lv_obj_create(parent);
        lv_obj_remove_style_all(sp);
        lv_obj_set_size(sp, 10, it->spacer_h);
        return sp;
    }
    default:
        return NULL;
    }
}

lv_obj_t *skaiapp_render_page(lv_obj_t *parent, const skaiapp_model_t *m,
                              skaiapp_render_ctx_t *ctx)
{
    s_model = m;
    s_ctx = ctx;
    s_root = parent;
    strncpy(s_app_id, m->id, sizeof(s_app_id) - 1);
    s_app_id[sizeof(s_app_id) - 1] = '\0';
    memset(ctx, 0, sizeof(*ctx));

    lv_obj_t *page = lv_obj_create(parent);
    lv_obj_remove_style_all(page);
    lv_obj_set_size(page, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(page, LV_OPA_COVER, 0);
    lv_obj_set_flex_flow(page, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(page, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    /* Round 466px display: the top/bottom and the four corners are clipped by
       the circle, so content is kept in a narrower centered column with a
       generous top/bottom safe area. The generator is also told to design for
       the circle (compact, centered) — this padding is the backstop. */
    lv_obj_set_style_pad_top(page, 72, 0);
    lv_obj_set_style_pad_bottom(page, 96, 0);
    lv_obj_set_style_pad_hor(page, 52, 0);
    lv_obj_set_style_pad_row(page, 10, 0);
    lv_obj_add_flag(page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(page, LV_SCROLLBAR_MODE_OFF);

    uint32_t i = 0;
    while (i < m->n_items)
    {
        const skaiapp_witem_t *it = &m->items[i];
        if (it->wtype == SKAIAPP_W_ROW)
        {
            lv_obj_t *row = lv_obj_create(page);
            lv_obj_remove_style_all(row);
            lv_obj_set_size(row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                                  LV_FLEX_ALIGN_CENTER);
            lv_obj_set_style_pad_column(row, 14, 0);
            uint32_t nchild = it->row_n;
            i++;
            for (uint32_t c = 0; c < nchild && i < m->n_items; c++, i++)
            {
                ctx->obj[i] = build_leaf(row, m, i);
            }
        }
        else
        {
            ctx->obj[i] = build_leaf(page, m, i);
            i++;
        }
    }
    return page;
}

void skaiapp_render_refresh(const skaiapp_model_t *m, skaiapp_render_ctx_t *ctx)
{
    if (m == NULL || ctx == NULL)
    {
        return;
    }
    for (uint32_t i = 0; i < m->n_items; i++)
    {
        const skaiapp_witem_t *it = &m->items[i];
        lv_obj_t *obj = ctx->obj[i];
        if (obj == NULL)
        {
            continue;
        }
        switch (it->wtype)
        {
        case SKAIAPP_W_VALUE:
        {
            /* memo values are a container (text + 🎤), not a bare label, and
               their text only changes on a re-push (full rebuild) — skip them. */
            if (it->bind != SKAIAPP_BIND_MEMO)
            {
                char buf[48];
                fmt_bind(it, buf, sizeof(buf));
                lv_label_set_text(obj, buf);
            }
            break;
        }
        case SKAIAPP_W_ARC:
            lv_arc_set_value(obj, gauge_percent(it));
            break;
        case SKAIAPP_W_BAR:
            lv_bar_set_value(obj, gauge_percent(it), LV_ANIM_OFF);
            break;
        /* A reminder.toggle button must stay FULLY visible + pressable in both
           states — it is the only control that turns the reminder back on.
           (Earlier this dimmed the button to 50% when off, which on the round
           black screen read as "the button vanished", stranding the user.)
           On/off state is shown by the reminder's own value bind: next-fire
           HH:MM when on, "--:--" when off. So no per-tick button restyle. */
        default:
            break;
        }
    }
}

void skaiapp_render_detach(void)
{
    /* Before the model goes away: the full-screen picture view holds a path
       pointer INTO the model, and it is parented to the app root rather than to
       the page, so nothing else would take it down. */
    photo_full_close();
    photo_picker_close();
    s_root = NULL;
    s_model = NULL;
    s_ctx = NULL;
    s_app_id[0] = '\0';
}
