/**
 ******************************************************************************
 * @file   lv_session_pager.c
 * @brief  Desktop-session pager. See lv_session_pager.h.
 ******************************************************************************
 */

#include "lvgl.h"
#include "lv_session_pager.h"
#include "communicate_task.h"
#include "ui_handler.h"
#include "bloc_v2t.h"
#include <cJSON.h>
#include <string.h>

#define DBG_TAG "session_pager"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* Shared mic glyph (resource/images/.../micro_icon.png) — the same asset the chat page
   and the skaibar bar use, so the voice affordance reads identically across surfaces. */
LV_IMG_DECLARE(micro_icon);

/* ── Geometry ──
   The bubble column stops well clear of the bottom band so the shared voice bar floats
   over empty space, and clear of the round display's top corner. */
#define SP_TITLE_Y 20
#define SP_LIST_Y 62
#define SP_LIST_H (LV_VER_RES - SP_LIST_Y - 96)
#define SP_LIST_SIDE_PAD 26
/* AI bubble — identical to lv_chat_page's "theirs" bubble so the two surfaces match
   pixel-for-pixel: systemGray5 fill, pad 11x7, radius 21, wrap at ~72% width. */
#define SP_BUBBLE_BG 0x2C2C2E
#define SP_BUBBLE_RADIUS 21
#define SP_BUBBLE_PAD_H 11
#define SP_BUBBLE_PAD_V 7
#define SP_TEXT_WRAP_PCT 72

/* Type scale — Skaiwalk design tokens §4. Everything here used to inherit LV_FONT_DEFAULT
   (16), which founder read as too small on the watch (2026-08-05: "文字有點小"). */
#define SP_FONT_TITLE 22      /* textLg  — section title */
#define SP_FONT_BUBBLE 19     /* textMd  — subhead; body legibility at arm's length */
#define SP_FONT_TRANSCRIPT 17 /* textBase */

/* Voice ripple — the SAME pulse the skaibar bar's push-to-talk uses
   (lv_instruction_list_layout.c LMIC_RIPPLE_*): a blue ring growing out of the mic
   centre and fading, replayed on a loop while listening. */
#define SP_RIPPLE_COLOR 0x5DA8FF
#define SP_RIPPLE_MIN_D 48
#define SP_RIPPLE_MAX_D 176
#define SP_RIPPLE_PERIOD_MS 1200
#define SP_MIC_Y (-14)

#define SP_V2T_INTENT 2 /* V2T_INTENT_REMOTE_INPUT — same intent the chat room uses */

/* Transcript buffer for the OPEN page only. Deliberately smaller than the chat room's
   16x256: the watch already holds one such buffer in lv_chat_page.c and SRAM here is
   tight (the mouse app has OOM'd opening the mic).
   ponytail: 12x192 is what fits the round screen's readable band — raise both if a
   session's turns start truncating in practice. */
#define SP_MSG_MAX 12
#define SP_MSG_LEN 192

typedef struct
{
    char id[SESSION_ID_LEN];
    char title[SESSION_TITLE_LEN];
    char preview[SESSION_PREVIEW_LEN];
} session_meta_t;

typedef struct
{
    char role[12];
    char text[SP_MSG_LEN];
} sp_msg_t;

/* ── Committed state (LVGL thread only) ── */
static session_meta_t s_sessions[SESSION_PAGER_MAX];
static int s_session_count;
static int s_current; /* index of the centred page */

static lv_obj_t *s_pager;     /* horizontal snap container (owns the pages) */
static lv_obj_t *s_voice_bar; /* shared bottom voice affordance (tile child, NOT a page child) */
static lv_obj_t *s_mic_img;
static lv_obj_t *s_ripple;
static lv_obj_t *s_transcript;                        /* live V2T text, shown while listening */
static lv_obj_t *s_page_lists[SESSION_PAGER_MAX];     /* each page's bubble column */
static bool s_listening;
/* The session the watch currently has OPEN on the phone (convOpen sent, no convClose
   yet), "" when none. Exactly one at a time — see lv_session_pager.h. */
static char s_open_id[SESSION_ID_LEN];
static bool s_visible; /* pager tile on screen (tracked by the visibility timer) */

/* ── Pending, written on the BLE parse thread ──
   Same split as lv_chat_page.c: parse into bounded statics off the 4KB BLE stack, then
   render on the LVGL thread. `s_pending_kind` is published LAST so an LVGL reader never
   observes a half-filled buffer. */
#define SP_PENDING_NONE 0
#define SP_PENDING_LIST 1
#define SP_PENDING_STATE 2
static volatile int s_pending_kind;

static session_meta_t s_pending_sessions[SESSION_PAGER_MAX];
static int s_pending_session_count;

static char s_pending_sid[SESSION_ID_LEN];
static char s_pending_title[SESSION_TITLE_LEN];
static bool s_pending_sending;
static sp_msg_t s_pending_msgs[SP_MSG_MAX];
static int s_pending_msg_count;

static void sp_open_current(void);

/* ── Voice ripple ────────────────────────────────────────────────────────── */

static void sp_ripple_anim_cb(void *var, int32_t v)
{
    lv_obj_t *ring = (lv_obj_t *)var;
    if (ring == NULL || !lv_obj_is_valid(ring))
        return;
    lv_coord_t d = SP_RIPPLE_MIN_D + (lv_coord_t)((SP_RIPPLE_MAX_D - SP_RIPPLE_MIN_D) * v / 256);
    lv_obj_set_size(ring, d, d);
    lv_obj_set_style_radius(ring, d / 2, 0);
    lv_obj_align(ring, LV_ALIGN_CENTER, 0, 0);
    /* Fade out as it grows — full at birth, gone at the outer edge. */
    lv_obj_set_style_border_opa(ring, (lv_opa_t)(255 - v), 0);
}

static void sp_set_listening(bool on)
{
    s_listening = on;
    if (s_ripple != NULL && lv_obj_is_valid(s_ripple))
    {
        lv_anim_del(s_ripple, sp_ripple_anim_cb);
        if (on)
        {
            lv_obj_clear_flag(s_ripple, LV_OBJ_FLAG_HIDDEN);
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, s_ripple);
            lv_anim_set_values(&a, 0, 255);
            lv_anim_set_time(&a, SP_RIPPLE_PERIOD_MS);
            lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
            lv_anim_set_exec_cb(&a, sp_ripple_anim_cb);
            lv_anim_start(&a);
        }
        else
        {
            lv_obj_add_flag(s_ripple, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (s_transcript != NULL && lv_obj_is_valid(s_transcript))
    {
        if (on)
        {
            lv_label_set_text(s_transcript, "聆聽中…");
            lv_obj_clear_flag(s_transcript, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(s_transcript, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* ── Voice input (mic → V2T → commu_send_conv_send) ──
   Mirrors lv_chat_page.c's proven sequence: voice_provider.start_v2t() does the full
   setup AND tells the phone to listen (the bare start_voice_recognition does not — it
   returns an empty transcript), the partial streams back through
   interact_voice_recognition → session_pager_set_transcript, and the finalized text is
   read from get_combined_voice2text() on stop.
   The build-time lint asks new gui_apps code to use skai_* (ADR-0019); there is no
   skai_* voice entry point in this tree, and this is the only path device-verified to
   actually reach the phone's transcriber — so we follow the tested one (repo CLAUDE.md
   R7: pick the more-tested side of a conflict and say why) rather than a nonexistent
   API. Revisit if ADR-0019 grows a voice surface. */

static void sp_stop_and_send(void)
{
    if (!s_listening)
        return;
    sp_set_listening(false);
    voice_provider.auto_stop_listening();
    const char *text = get_combined_voice2text();
    if (text != NULL && text[0] != '\0')
    {
        /* The open conversation IS the centred page — the phone routes convSend by the
           session it last got convOpen for, so no target is repeated here. */
        commu_send_conv_send(text);
        clearVoice2Text();
    }
}

static void sp_mic_toggle(void)
{
    if (s_listening)
    {
        sp_stop_and_send();
        return;
    }
    if (s_open_id[0] == '\0')
    {
        /* Nothing open yet (list still empty, or the open raced the tap) — try again so
           a tap is never silently dropped. */
        sp_open_current();
        if (s_open_id[0] == '\0')
        {
            LOG_W("mic: no open session, ignoring");
            return;
        }
    }
    clearVoice2Text();
    voice_provider.start_v2t();
    sp_set_listening(true);
    LOG_I("mic: listening for session=%s", s_open_id);
}

static void sp_mic_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    sp_mic_toggle();
}

/* Live partial transcript, routed here from interact_voice_recognition
   (app_system_interface.c) while this pager owns the mic. */
void session_pager_set_transcript(const char *text)
{
    if (!s_listening || s_transcript == NULL || !lv_obj_is_valid(s_transcript))
        return;
    bool has = (text != NULL && text[0] != '\0');
    lv_label_set_text(s_transcript, has ? text : "聆聽中…");
}

/* True while the pager is the surface that should receive voice events (the tile is on
   screen and a session is open) — the same gate chat_page_is_open() provides for the
   @-list chat room. */
bool session_pager_is_open(void)
{
    return s_pager != NULL && lv_obj_is_valid(s_pager) && s_visible && s_open_id[0] != '\0';
}

/* Release-gesture / lift-to-talk entry point, mirroring chat_page_start_voice_input. */
void session_pager_start_voice_input(void)
{
    if (!session_pager_is_open())
        return;
    sp_mic_toggle();
}

/* ── Bubbles ─────────────────────────────────────────────────────────────── */

/* One transcript row. Founder rule: ONLY the AI's turns get a bubble frame — the user's
   own words render as bare right-aligned text, so a glance down the page reads as "what
   Skai said" with the user's side as quiet context. */
/* lvsf_get_font_from_size returns NULL when freetype never initialised (no .ttf on the FS —
   see font_partition_dsc.c's is_freetype_safe_to_init), and pinning a NULL font would take
   the draw path down. Keep the inherited default in that case: small text beats no watch. */
static void sp_set_font(lv_obj_t *obj, uint16_t size)
{
    lv_font_t *f = lvsf_get_font_from_size(size);
    if (f != NULL)
        lv_obj_set_style_text_font(obj, f, 0);
}

static void sp_add_bubble(lv_obj_t *list, const char *text, bool from_ai)
{
    lv_obj_t *row = lv_obj_create(list);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *holder = lv_obj_create(row);
    lv_obj_set_width(holder, LV_SIZE_CONTENT);
    lv_obj_set_height(holder, LV_SIZE_CONTENT);
    lv_obj_clear_flag(holder, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(holder, 0, 0);
    if (from_ai)
    {
        lv_obj_set_style_pad_hor(holder, SP_BUBBLE_PAD_H, 0);
        lv_obj_set_style_pad_ver(holder, SP_BUBBLE_PAD_V, 0);
        lv_obj_set_style_radius(holder, SP_BUBBLE_RADIUS, 0);
        lv_obj_set_style_bg_color(holder, lv_color_hex(SP_BUBBLE_BG), 0);
        lv_obj_set_style_bg_opa(holder, LV_OPA_COVER, 0);
    }
    else
    {
        lv_obj_set_style_pad_all(holder, 0, 0);
        lv_obj_set_style_bg_opa(holder, LV_OPA_TRANSP, 0);
    }
    lv_obj_align(holder, from_ai ? LV_ALIGN_TOP_LEFT : LV_ALIGN_TOP_RIGHT, 0, 0);

    lv_obj_t *lbl = lv_label_create(holder);
    lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
    /* BEFORE the measure below — lv_txt_get_size reads the label's current font, so setting
       it afterwards would wrap the text against the wrong metrics. */
    sp_set_font(lbl, SP_FONT_BUBBLE);
    /* LVGL clips (rather than re-wraps) a SIZE_CONTENT label pinned by max_width, so
       measure first and only pin the width when the text actually overflows — the same
       guard lv_chat_page.c needs for long messages. */
    {
        const lv_font_t *fnt = lv_obj_get_style_text_font(lbl, LV_PART_MAIN);
        lv_coord_t lsp = lv_obj_get_style_text_letter_space(lbl, LV_PART_MAIN);
        lv_coord_t lnsp = lv_obj_get_style_text_line_space(lbl, LV_PART_MAIN);
        lv_point_t tsz;
        lv_txt_get_size(&tsz, text, fnt, lsp, lnsp, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
        lv_coord_t cap = (LV_HOR_RES * SP_TEXT_WRAP_PCT) / 100;
        lv_obj_set_width(lbl, tsz.x > cap ? cap : LV_SIZE_CONTENT);
    }
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
    if (!from_ai)
        lv_obj_set_style_text_opa(lbl, LV_OPA_70, 0); /* the user's own words sit back */
}

/* ── Pages ───────────────────────────────────────────────────────────────── */

static void sp_build_page(lv_obj_t *pager, const session_meta_t *s, int idx)
{
    lv_obj_t *page = lv_obj_create(pager);
    lv_obj_remove_style_all(page);
    lv_obj_set_size(page, LV_HOR_RES, LV_VER_RES);
    lv_obj_clear_flag(page, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(page);
    lv_label_set_long_mode(title, LV_LABEL_LONG_DOT);
    lv_obj_set_width(title, LV_HOR_RES - 120);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    sp_set_font(title, SP_FONT_TITLE);
    lv_label_set_text(title, s->title[0] ? s->title : "Session");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, SP_TITLE_Y);

    lv_obj_t *list = lv_obj_create(page);
    lv_obj_set_size(list, LV_HOR_RES - SP_LIST_SIDE_PAD, SP_LIST_H);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, SP_LIST_Y);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF);
    /* The bubble column covers nearly the whole page, so a horizontal drag almost always
       STARTS on it. set_scroll_dir(VER) means the column itself will not move sideways —
       and with the horizontal chain CLEARED that drag dead-ends here instead of reaching
       the pager. Founder 2026-08-05: "只能碰最右邊才能左滑…沒辦法在畫面中央直接左右滑" —
       the only live strip was the 13px of side padding either side of this object. Chain
       horizontally (LVGL then hands the pager exactly the axis this column refuses). */
    lv_obj_add_flag(list, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list, 6, 0);
    lv_obj_set_style_pad_hor(list, 2, 0);
    if (idx >= 0 && idx < SESSION_PAGER_MAX)
        s_page_lists[idx] = list;

    /* The preview stands in until this page becomes the open conversation and real turns
       arrive on KEY_CONV_STATE (which replaces the column wholesale). */
    if (s->preview[0])
        sp_add_bubble(list, s->preview, true);

    lv_obj_scroll_to_y(list, LV_COORD_MAX, LV_ANIM_OFF); /* pin to the newest */
}

/* Send convOpen for the centred page (and convClose for the one being left). The watch
   keeps exactly ONE conversation open, so these always come in pairs. */
static void sp_open_current(void)
{
    if (s_session_count <= 0 || s_current < 0 || s_current >= s_session_count)
        return;
    const session_meta_t *s = &s_sessions[s_current];
    if (s->id[0] == '\0')
        return;
    if (strcmp(s_open_id, s->id) == 0)
        return; /* already open */
    if (s_open_id[0] != '\0')
        commu_send_conv_close();
    commu_send_conv_open(s->title, s->id, (uint8_t)s_current);
    strncpy(s_open_id, s->id, SESSION_ID_LEN - 1);
    s_open_id[SESSION_ID_LEN - 1] = '\0';
    LOG_I("conv open: %s", s_open_id);
}

static void sp_close_current(void)
{
    if (s_open_id[0] == '\0')
        return;
    if (s_listening)
    {
        /* Leaving mid-dictation: drop the partial rather than posting it into a
           conversation the user just walked away from. */
        sp_set_listening(false);
        voice_provider.auto_stop_listening();
        clearVoice2Text();
    }
    commu_send_conv_close();
    LOG_I("conv close: %s", s_open_id);
    s_open_id[0] = '\0';
}

/* Debounced page commit: a fast flick across the pager must not convOpen every page it
   passes. Only the page the finger LEFT the pager on opens. */
#define SP_SETTLE_DEBOUNCE_MS 300
static lv_timer_t *s_settle_timer;

static void sp_settle_timer_cb(lv_timer_t *t)
{
    lv_timer_del(t);
    s_settle_timer = NULL;
    sp_open_current();
}

static void sp_scroll_end_cb(lv_event_t *e)
{
    lv_obj_t *pager = lv_event_get_target(e);
    if (pager == NULL || !lv_obj_is_valid(pager) || s_session_count <= 0)
        return;
    lv_coord_t sx = lv_obj_get_scroll_x(pager);
    int idx = (sx + LV_HOR_RES / 2) / LV_HOR_RES;
    if (idx < 0)
        idx = 0;
    if (idx >= s_session_count)
        idx = s_session_count - 1;
    if (idx == s_current)
        return;
    s_current = idx;
    LOG_I("page settled: %d (%s)", idx, s_sessions[idx].id);
    if (s_settle_timer != NULL)
        lv_timer_del(s_settle_timer);
    s_settle_timer = lv_timer_create(sp_settle_timer_cb, SP_SETTLE_DEBOUNCE_MS, NULL);
}

static void sp_rebuild_pages(void)
{
    if (s_pager == NULL || !lv_obj_is_valid(s_pager))
        return;
    lv_obj_clean(s_pager);
    memset(s_page_lists, 0, sizeof(s_page_lists));
    for (int i = 0; i < s_session_count; i++)
        sp_build_page(s_pager, &s_sessions[i], i);
    s_current = 0;
    lv_obj_scroll_to_x(s_pager, 0, LV_ANIM_OFF);
}

/* ── Visibility ──────────────────────────────────────────────────────────────
   The pager lives in a tileview page, so "am I on screen" is what decides whether a
   conversation should be open at all. LVGL's own clip-aware lv_obj_is_visible() answers
   that without touching app_clock_status_bar.c's tile bookkeeping.
   ponytail: polled at 500ms rather than hooked into the tileview's VALUE_CHANGED —
   one timer, zero coupling. Move to the event if the open/close latency ever shows. */
#define SP_VIS_POLL_MS 500

static void sp_visibility_timer_cb(lv_timer_t *t)
{
    (void)t;
    if (s_pager == NULL || !lv_obj_is_valid(s_pager))
        return;
    bool vis = lv_obj_is_visible(s_pager);
    if (vis == s_visible)
        return;
    s_visible = vis;
    if (vis)
    {
        /* This tile is still INSTRUCTION_LIST_PAGE_INDEX, so app_clock keeps floating the
           instruction bar layer (lv_layer_top) for it — from when this tile WAS the
           instruction list. lv_layer_top draws above every tile, so an opaque page
           background cannot cover it, and it takes the touches too: founder 2026-08-05
           saw "左側的 action 列表疊在右側 tileview 上面" and could only drag the far right
           edge. The pager owns this tile now, so take that layer down on entry; every
           other page asserts its own state on arrival (the documented per-page contract
           on instruction_list_bar_set_visible), so leaving needs nothing here. */
        {
            extern void instruction_list_bar_set_visible(bool visible);
            instruction_list_bar_set_visible(false);
        }
        /* Entering the pager: recover a list we may have missed, then open the page the
           user is looking at. */
        commu_send_conv_list_req();
        sp_open_current();
    }
    else
    {
        sp_close_current();
    }
    LOG_I("pager visible=%d", (int)vis);
}

/* ── BLE parse thread ────────────────────────────────────────────────────── */

/* 0x20 — {"sessions":[{id,title,preview}]}. Bounded copies only; no LVGL calls here. */
void skai_sessions_on_conv_list(const uint8_t *json, uint16_t length)
{
    if (json == NULL || length == 0)
        return;
    cJSON *root = cJSON_ParseWithLength((const char *)json, length);
    if (!cJSON_IsObject(root))
    {
        cJSON_Delete(root);
        return;
    }
    cJSON *arr = cJSON_GetObjectItem(root, "sessions");
    int count = 0;
    if (cJSON_IsArray(arr))
    {
        cJSON *it = NULL;
        cJSON_ArrayForEach(it, arr)
        {
            if (count >= SESSION_PAGER_MAX)
                break;
            cJSON *j_id = cJSON_GetObjectItem(it, "id");
            if (!cJSON_IsString(j_id) || j_id->valuestring[0] == '\0')
                continue; /* a row with no identity can never be opened — drop it */
            memset(&s_pending_sessions[count], 0, sizeof(s_pending_sessions[count]));
            strncpy(s_pending_sessions[count].id, j_id->valuestring, SESSION_ID_LEN - 1);
            cJSON *j_title = cJSON_GetObjectItem(it, "title");
            if (cJSON_IsString(j_title))
                strncpy(s_pending_sessions[count].title, j_title->valuestring,
                        SESSION_TITLE_LEN - 1);
            cJSON *j_prev = cJSON_GetObjectItem(it, "preview");
            if (cJSON_IsString(j_prev))
                strncpy(s_pending_sessions[count].preview, j_prev->valuestring,
                        SESSION_PREVIEW_LEN - 1);
            count++;
        }
    }
    cJSON_Delete(root);
    s_pending_session_count = count;
    s_pending_kind = SP_PENDING_LIST; /* publish LAST */
    LOG_I("conv_list rx: %d sessions", count);

    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_REFRESH_SESSIONS};
    lvgl_send_msg(msg);
}

/* True while a KEY_CONV_STATE belongs to a pager page rather than the @-list chat room.
   Read from the BLE parse thread by handle_conv_state. */
bool skai_sessions_owns_conv(void)
{
    return s_open_id[0] != '\0';
}

/* 0x12 (routed here when the pager owns the conversation) —
   {sid, title, sending, messages:[{role,text}]}. */
void skai_sessions_on_conv_state(const uint8_t *json, uint16_t length)
{
    if (json == NULL || length == 0)
        return;
    cJSON *root = cJSON_ParseWithLength((const char *)json, length);
    if (!cJSON_IsObject(root))
    {
        cJSON_Delete(root);
        return;
    }
    cJSON *j_sid = cJSON_GetObjectItem(root, "sid");
    s_pending_sid[0] = '\0';
    if (cJSON_IsString(j_sid))
    {
        strncpy(s_pending_sid, j_sid->valuestring, SESSION_ID_LEN - 1);
        s_pending_sid[SESSION_ID_LEN - 1] = '\0';
    }
    cJSON *j_title = cJSON_GetObjectItem(root, "title");
    s_pending_title[0] = '\0';
    if (cJSON_IsString(j_title))
    {
        strncpy(s_pending_title, j_title->valuestring, SESSION_TITLE_LEN - 1);
        s_pending_title[SESSION_TITLE_LEN - 1] = '\0';
    }
    s_pending_sending = cJSON_IsTrue(cJSON_GetObjectItem(root, "sending"));

    int count = 0;
    cJSON *j_msgs = cJSON_GetObjectItem(root, "messages");
    if (cJSON_IsArray(j_msgs))
    {
        /* Keep the LAST SP_MSG_MAX valid turns: with a full backlog the NEWEST message
           is the one that matters, and taking the first N would drop what just arrived
           (the same bug lv_chat_page.c had to fix). */
        int valid_total = 0;
        cJSON *m = NULL;
        cJSON_ArrayForEach(m, j_msgs)
        {
            cJSON *jt = cJSON_GetObjectItem(m, "text");
            if (cJSON_IsString(jt) && jt->valuestring[0] != '\0')
                valid_total++;
        }
        int skip = (valid_total > SP_MSG_MAX) ? (valid_total - SP_MSG_MAX) : 0;
        int seen = 0;
        cJSON_ArrayForEach(m, j_msgs)
        {
            cJSON *j_text = cJSON_GetObjectItem(m, "text");
            if (!cJSON_IsString(j_text) || j_text->valuestring[0] == '\0')
                continue;
            if (seen++ < skip)
                continue;
            if (count >= SP_MSG_MAX)
                break;
            memset(&s_pending_msgs[count], 0, sizeof(s_pending_msgs[count]));
            cJSON *j_role = cJSON_GetObjectItem(m, "role");
            if (cJSON_IsString(j_role))
                strncpy(s_pending_msgs[count].role, j_role->valuestring,
                        sizeof(s_pending_msgs[count].role) - 1);
            strncpy(s_pending_msgs[count].text, j_text->valuestring, SP_MSG_LEN - 1);
            count++;
        }
    }
    cJSON_Delete(root);
    s_pending_msg_count = count;
    s_pending_kind = SP_PENDING_STATE; /* publish LAST */
    LOG_I("conv_state rx: sid=%s msgs=%d sending=%d", s_pending_sid, count,
          (int)s_pending_sending);

    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_REFRESH_SESSIONS};
    lvgl_send_msg(msg);
}

/* ── LVGL thread ─────────────────────────────────────────────────────────── */

static void sp_apply_list(void)
{
    int count = s_pending_session_count;
    if (count > SESSION_PAGER_MAX)
        count = SESSION_PAGER_MAX;

    /* A refresh that changes NOTHING must not rebuild. lv_obj_clean() + 8 fresh pages takes
       the pager out of lv_obj_is_visible() for a moment, which the 500ms visibility poll
       reads as leave-then-enter — and every enter re-requests the list, which lands here
       again. Measured on real hardware 2026-08-05: a convListReq/convOpen/convClose cycle
       every ~1.5s, and the UI frozen under the constant destroy-and-rebuild. An identical
       list is the COMMON case (the phone re-pushes on every reconnect and on every list
       request), so this early-out is the loop's off switch, not an optimisation. */
    if (count == s_session_count && count > 0 &&
        memcmp(s_sessions, s_pending_sessions, (size_t)count * sizeof(s_sessions[0])) == 0)
    {
        LOG_D("conv_list unchanged (%d sessions) — no rebuild", count);
        return;
    }

    /* Remember what was open so a list refresh doesn't silently drop the conversation
       out from under the user. */
    char was_open[SESSION_ID_LEN];
    strncpy(was_open, s_open_id, SESSION_ID_LEN - 1);
    was_open[SESSION_ID_LEN - 1] = '\0';

    memcpy(s_sessions, s_pending_sessions, sizeof(s_sessions));
    s_session_count = count;
    sp_rebuild_pages();

    /* Re-centre on the session that was open, if it survived the refresh. */
    if (was_open[0] != '\0')
    {
        for (int i = 0; i < s_session_count; i++)
        {
            if (strcmp(s_sessions[i].id, was_open) == 0)
            {
                s_current = i;
                lv_obj_scroll_to_x(s_pager, i * LV_HOR_RES, LV_ANIM_OFF);
                return; /* still open on the phone — nothing to re-send */
            }
        }
        /* It's gone (session deleted on the desktop): the phone's conversation is stale. */
        s_open_id[0] = '\0';
    }
    if (s_visible)
        sp_open_current();
}

static void sp_apply_state(void)
{
    /* Drop a late frame from the conversation we just left. An older phone sends no sid
       at all, in which case it can only be about what's open. */
    if (s_pending_sid[0] != '\0' && s_open_id[0] != '\0' &&
        strcmp(s_pending_sid, s_open_id) != 0)
    {
        LOG_D("conv_state for %s ignored (open=%s)", s_pending_sid, s_open_id);
        return;
    }
    if (s_current < 0 || s_current >= SESSION_PAGER_MAX)
        return;
    lv_obj_t *list = s_page_lists[s_current];
    if (list == NULL || !lv_obj_is_valid(list))
        return;

    lv_obj_clean(list);
    for (int i = 0; i < s_pending_msg_count; i++)
    {
        const sp_msg_t *m = &s_pending_msgs[i];
        /* The phone folds both vocabularies (user/ai and outgoing/incoming) into this
           field; anything that isn't the user is rendered as the AI side. */
        bool mine = (strcmp(m->role, "user") == 0 || strcmp(m->role, "outgoing") == 0);
        sp_add_bubble(list, m->text, !mine);
    }
    if (s_pending_sending)
        sp_add_bubble(list, "…", true);
    lv_obj_scroll_to_y(list, LV_COORD_MAX, LV_ANIM_OFF);
}

/* Dispatched from ui_handler.c on LVGL_MSG_TYPE_REFRESH_SESSIONS. */
void session_pager_apply_pending(void)
{
    int kind = s_pending_kind;
    s_pending_kind = SP_PENDING_NONE;
    if (s_pager == NULL || !lv_obj_is_valid(s_pager))
        return;
    if (kind == SP_PENDING_LIST)
        sp_apply_list();
    else if (kind == SP_PENDING_STATE)
        sp_apply_state();
}

/* ── Public ──────────────────────────────────────────────────────────────── */

lv_obj_t *lv_session_pager_create(lv_obj_t *parent)
{
    lv_obj_t *pager = lv_obj_create(parent);
    lv_obj_remove_style_all(pager);
    /* remove_style_all leaves the pager fully TRANSPARENT, so nothing ever paints over the
       tile underneath it. Awake that is invisible (the tileview only shows one tile), but
       after a sleep/resume the watch face was still in the framebuffer and showed THROUGH
       this page — founder 2026-08-05: "錶盤左側的頁面疊在右側頁面上". A full-screen tile
       must own its pixels. */
    lv_obj_set_style_bg_color(pager, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(pager, LV_OPA_COVER, 0);
    lv_obj_set_size(pager, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(pager, 0, 0);
    lv_obj_add_flag(pager, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(pager, LV_DIR_HOR);
    lv_obj_set_scroll_snap_x(pager, LV_SCROLL_SNAP_START);
    lv_obj_set_scrollbar_mode(pager, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_flow(pager, LV_FLEX_FLOW_ROW);
    /* Founder rule (2026-08-05): a mid-pager drag belongs to the PAGES, not the tileview.
       Clearing the chain enforced that — but it also swallowed the drag at the ends, so
       there was no way back to the watch face at all ("在螢幕左邊緣右滑…也沒辦法回錶盤";
       the lvsf_gesture edge detector sits UNDER this full-screen object and never sees the
       press). LVGL's chain already encodes the rule we actually want: a child keeps the
       gesture while it can still scroll that way, and only hands over once it cannot. So
       pages 1..N-1 stay the pager's, and a right-drag on page 0 falls through to the
       tileview — which is exactly "swipe back from the first page". */
    lv_obj_add_flag(pager, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_add_event_cb(pager, sp_scroll_end_cb, LV_EVENT_SCROLL_END, NULL);
    s_pager = pager;

    /* Shared voice bar — a TILE child created AFTER the pager, so it floats above the
       pages and does not scroll with them. One mic, one ripple, one transcript for the
       whole pager; it acts on whichever session is centred. */
    lv_obj_t *bar = lv_obj_create(parent);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, LV_HOR_RES, 96);
    lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    s_voice_bar = bar;

    lv_obj_t *ripple = lv_obj_create(bar);
    lv_obj_remove_style_all(ripple);
    lv_obj_set_size(ripple, SP_RIPPLE_MIN_D, SP_RIPPLE_MIN_D);
    lv_obj_set_style_radius(ripple, SP_RIPPLE_MIN_D / 2, 0);
    lv_obj_set_style_bg_opa(ripple, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ripple, 2, 0);
    lv_obj_set_style_border_color(ripple, lv_color_hex(SP_RIPPLE_COLOR), 0);
    lv_obj_align(ripple, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(ripple, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ripple, LV_OBJ_FLAG_CLICKABLE);
    s_ripple = ripple;

    lv_obj_t *mic = lv_img_create(bar);
    lv_img_set_src(mic, &micro_icon);
    /* Zoom with a CENTRE pivot + OVERFLOW_VISIBLE (a set_size would clip the ezip
       bitmap) — the same treatment lv_chat_page.c gives this glyph. */
    lv_img_set_pivot(mic, micro_icon.header.w / 2, micro_icon.header.h / 2);
    lv_obj_add_flag(mic, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_img_set_zoom(mic, 128);
    lv_obj_align(mic, LV_ALIGN_CENTER, 0, SP_MIC_Y);
    lv_obj_add_flag(mic, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(mic, 20);
    lv_obj_add_event_cb(mic, sp_mic_cb, LV_EVENT_CLICKED, NULL);
    s_mic_img = mic;

    lv_obj_t *tr = lv_label_create(bar);
    lv_label_set_long_mode(tr, LV_LABEL_LONG_DOT);
    sp_set_font(tr, SP_FONT_TRANSCRIPT);
    lv_obj_set_width(tr, LV_HOR_RES - 140);
    lv_obj_set_style_text_align(tr, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(tr, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_opa(tr, LV_OPA_80, 0);
    lv_label_set_text(tr, "");
    lv_obj_align(tr, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_flag(tr, LV_OBJ_FLAG_HIDDEN);
    s_transcript = tr;

    s_listening = false;
    s_open_id[0] = '\0';
    s_visible = false;
    lv_timer_create(sp_visibility_timer_cb, SP_VIS_POLL_MS, NULL);

    LOG_I("session pager created (%d sessions)", s_session_count);

#ifdef BSP_USING_PC_SIMULATOR
    /* PC sim has no BLE, so the phone never pushes a session list — stand in with a
       few so the layout can actually be looked at. Never compiled for the watch. */
    if (s_session_count == 0)
    {
        static const char demo_ids[3][SESSION_ID_LEN] = {
            "conv:hermes:demo-1", "conv:hermes:demo-2", "conv:hermes:demo-3"};
        static const char demo_titles[3][SESSION_TITLE_LEN] = {
            "重構 relay 心跳", "週報草稿", "SiFli 編譯錯誤"};
        static const char demo_previews[3][SESSION_PREVIEW_LEN] = {
            "心跳間隔拉到 30 秒之後，斷線重連的中位數從 8.2 秒掉到 2.1 秒。要我把這個值寫進設定檔嗎？",
            "這週三個重點：手錶 session pager、relay 穩定性、Windows 焦點修復。要展開哪一項？",
            "缺的是 lv_session_pager.h 的 include path，SConscript 的 Glob 沒收到新目錄。"};
        lv_session_pager_set_sessions(demo_ids, demo_titles, demo_previews, 3);
        return pager;
    }
#endif

    /* Nothing cached yet — ask the phone for the list. The visibility timer asks again
       on every entry, so a push that arrives while disconnected is never lost for good. */
    commu_send_conv_list_req();
    if (s_session_count > 0)
        sp_rebuild_pages();
    return pager;
}

void lv_session_pager_set_sessions(const char (*ids)[SESSION_ID_LEN],
                                   const char (*titles)[SESSION_TITLE_LEN],
                                   const char (*previews)[SESSION_PREVIEW_LEN],
                                   int count)
{
    if (count < 0)
        count = 0;
    if (count > SESSION_PAGER_MAX)
        count = SESSION_PAGER_MAX;
    memset(s_sessions, 0, sizeof(s_sessions));
    for (int i = 0; i < count; i++)
    {
        if (ids != NULL)
        {
            strncpy(s_sessions[i].id, ids[i], SESSION_ID_LEN - 1);
            s_sessions[i].id[SESSION_ID_LEN - 1] = '\0';
        }
        if (titles != NULL)
        {
            strncpy(s_sessions[i].title, titles[i], SESSION_TITLE_LEN - 1);
            s_sessions[i].title[SESSION_TITLE_LEN - 1] = '\0';
        }
        if (previews != NULL)
        {
            strncpy(s_sessions[i].preview, previews[i], SESSION_PREVIEW_LEN - 1);
            s_sessions[i].preview[SESSION_PREVIEW_LEN - 1] = '\0';
        }
    }
    s_session_count = count;
    LOG_I("sessions set: %d", count);
    sp_rebuild_pages();
}

const char *lv_session_pager_current_id(void)
{
    if (s_session_count <= 0 || s_current < 0 || s_current >= s_session_count)
        return NULL;
    return s_sessions[s_current].id;
}
