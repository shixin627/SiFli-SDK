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
#include "skaiapp_engine.h"
#include "ui_img_helper.h"
#include "ui_helper.h"
#include "watch_global_data.h"

#define DBG_TAG "skaiapp.rend"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static const skaiapp_model_t *s_model = NULL;
static skaiapp_render_ctx_t *s_ctx = NULL;
static char s_app_id[SKAIAPP_ID_MAX];

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

/* days-since-1970 → y/m/d (civil algorithm, avoids libc time dependency) */
static void civil_from_days(int32_t z, int *y, int *mo, int *d)
{
    z += 719468;
    int32_t era = (z >= 0 ? z : z - 146096) / 146097;
    uint32_t doe = (uint32_t)(z - era * 146097);
    uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int32_t yy = (int32_t)yoe + era * 400;
    uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    uint32_t mp = (5 * doy + 2) / 153;
    uint32_t dd = doy - (153 * mp + 2) / 5 + 1;
    uint32_t mm = mp + (mp < 10 ? 3 : (uint32_t)-9);
    *y = (int)(yy + (mm <= 2));
    *mo = (int)mm;
    *d = (int)dd;
}

static void fmt_bind(const skaiapp_witem_t *it, char *buf, size_t cap)
{
    uint32_t sec = SkaiWatchSys.SecondCountRTC; /* local wall clock as epoch */
    buf[0] = '\0';
    switch (it->bind)
    {
    case SKAIAPP_BIND_TIME:
    {
        uint8_t h = (uint8_t)((sec / 3600u) % 24u);
        uint8_t mi = (uint8_t)((sec / 60u) % 60u);
        ui_time_format_hhmm(buf, cap, h, mi);
        break;
    }
    case SKAIAPP_BIND_DATE:
    {
        int y, mo, d;
        civil_from_days((int32_t)(sec / 86400u), &y, &mo, &d);
        rt_snprintf(buf, cap, "%02d/%02d", mo, d);
        break;
    }
    case SKAIAPP_BIND_BATTERY:
        rt_snprintf(buf, cap, "%d%%", SkaiWatchSys.battery_level_value);
        break;
    case SKAIAPP_BIND_HR:
    {
        /* same HCPU-resident source the exercise app writes (SkaiWatchSys);
           0 = no recent reading → "--" (HR only samples when subscribed) */
        uint8_t bpm = SkaiWatchSys.heart_rate_bpm;
        if (bpm == 0)
        {
            rt_snprintf(buf, cap, "--");
        }
        else
        {
            rt_snprintf(buf, cap, "%d", bpm);
        }
        break;
    }
    case SKAIAPP_BIND_STEPS:
        rt_snprintf(buf, cap, "%u", (unsigned)SkaiWatchSys.gPedoData.global_steps);
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
    case SKAIAPP_BIND_BATTERY:
        return SkaiWatchSys.battery_level_value;
    case SKAIAPP_BIND_STEPS:
    {
        int32_t max = (it->max > 0) ? it->max : 8000;
        int32_t v = (int32_t)((int64_t)SkaiWatchSys.gPedoData.global_steps * 100 / max);
        return (v > 100) ? 100 : v;
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
    s_model = NULL;
    s_ctx = NULL;
    s_app_id[0] = '\0';
}
