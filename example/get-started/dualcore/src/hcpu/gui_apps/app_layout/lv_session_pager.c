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

/* ── Committed state (LVGL thread only) ──
   One list PER DESKTOP. The watch face's right side is heading for one column per device
   (founder 2026-08-10), and even before that UI exists the storage has to be per-device:
   a flat list cannot say which desktop a row belongs to, so a second desktop's push would
   overwrite the first and every convOpen after it would be routed at the wrong machine.
   `s_dev_shown` is the device the single existing column renders; the column split is a
   later round. */
typedef struct
{
    char id[SESSION_ID_LEN];
    char name[SESSION_DEVICE_NAME_LEN];
    session_meta_t items[SESSION_PAGER_MAX];
    int count;
} device_sessions_t;

static device_sessions_t s_devices[SESSION_DEVICE_MAX];
static int s_device_count;
static int s_dev_shown; /* index into s_devices — what the column currently shows */
static int s_current;   /* index into that device's items — the open session */

/* The list for the column the UI is looking at. Never NULL: with nothing stored for that
   device it answers an empty slot carrying the RIGHT id, so the mic can still create a
   session on a desktop that has none yet.

   The column is indexed against the DEVICE REGISTRY (hid_mouse_device_id), not against the
   order lists happened to arrive in. That is what keeps column i, media page i and "the
   device the panel controls" the same machine by construction — the registry is the one
   ordering every other surface already uses (founder 2026-08-10: 右邊一台設備一欄，從上面
   拉下來就是那一台的媒體頁). Matching on name instead would break on rename and duplicates. */
static device_sessions_t *sp_shown(void)
{
    static device_sessions_t scratch;
    extern const char *hid_mouse_device_id(int idx);
    extern int hid_mouse_device_count(void);

    const char *want = hid_mouse_device_id(s_dev_shown);
    if (want != NULL)
    {
        for (int i = 0; i < s_device_count; i++)
        {
            if (strcmp(s_devices[i].id, want) == 0)
                return &s_devices[i];
        }
        /* A phone older than the per-device contract pushes with NO device, which lands in
           the empty-id slot. Show it on the first column rather than an empty state: during
           an OTA window the watch is new and the phone is not, and "my sessions vanished"
           is a far worse failure than showing the one desktop's list on column 0.
           (founder 2026-08-10 hit exactly this — new firmware, old APK.) */
        if (s_dev_shown == 0)
        {
            for (int i = 0; i < s_device_count; i++)
            {
                if (s_devices[i].id[0] == '\0')
                    return &s_devices[i];
            }
        }
        /* Registered device we have no list for yet — answer an empty list that still
           knows WHICH desktop it is, so conv_new/list_req remain addressable. */
        memset(&scratch, 0, sizeof(scratch));
        strncpy(scratch.id, want, SESSION_ID_LEN - 1);
        return &scratch;
    }
    /* No registry (or a phone older than the per-device contract): fall back to the single
       stored list, which is exactly the pre-2026-08-10 behaviour. */
    if (s_device_count > 0)
        return &s_devices[0];
    memset(&scratch, 0, sizeof(scratch));
    return &scratch;
}

/* ── Two layers, one tile (2026-08-10 founder) ──
   LIST is what the tile shows on arrival: the sessions this desktop has, as tappable rows.
   CHAT replaces it for exactly ONE session once a row is tapped, and the left-edge back
   swipe returns. They are siblings, not a pager: the old design paged horizontally across
   sessions, which left no free horizontal axis for a back gesture and opened a conversation
   just by drifting past it. */
static lv_obj_t *s_root;      /* the tile child everything else hangs off */
static lv_obj_t *s_home_tile; /* the tile we were built in — restored to after a pin */
static lv_obj_t *s_dim;       /* black scrim over the page while the panel comes over it */
static lv_obj_t *s_list_view; /* LIST layer — vertical column of session rows */
static lv_obj_t *s_chat_view; /* CHAT layer — one session's transcript (hidden on LIST) */
static lv_obj_t *s_chat_title;
static lv_obj_t *s_chat_list; /* the bubble column inside CHAT */
static bool s_in_chat;
static lv_obj_t *s_voice_bar; /* shared bottom voice affordance (tile child, NOT a layer child) */
static lv_obj_t *s_mic_img;
static lv_obj_t *s_ripple;
static lv_obj_t *s_transcript; /* live V2T text, shown while listening */
static bool s_listening;
/* Set when the mic on the LIST layer asked for a new session. The desktop answers with an
   ordinary list push (see KEY_CONV_NEW), so this is what tells sp_apply_list that the row
   it has never seen before is the one to walk into. */
static bool s_await_new;
static char s_known_ids[SESSION_PAGER_MAX][SESSION_ID_LEN];
static int s_known_count;
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
static char s_pending_dev_id[SESSION_ID_LEN];
static char s_pending_dev_name[SESSION_DEVICE_NAME_LEN];

static char s_pending_sid[SESSION_ID_LEN];
static char s_pending_title[SESSION_TITLE_LEN];
static bool s_pending_sending;
static sp_msg_t s_pending_msgs[SP_MSG_MAX];
static int s_pending_msg_count;

/* What the centred page currently HAS drawn — see the redraw skip in sp_apply_state.
   sp_enter_chat must void this: it clears the bubble column, so "already drawn" would
   otherwise leave the fresh (empty) room unpainted until the text next changes. */
static uint64_t s_drawn_sig;
static int s_drawn_page = -1;

static void sp_enter_chat(int idx);
static void sp_leave_chat(void);
static void sp_rebuild_list(void);

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
    /* LIST layer: there is no conversation to dictate INTO, so the mic means "start a new
       one" (founder 2026-08-10). It only asks — the desktop creates the session and the
       new row arrives in the ordinary list push, which is what walks us into the chat.
       Dictation is a second, deliberate tap once we're there: auto-arming the mic on a
       session the user has not seen yet would post their first words into a conversation
       they never confirmed opened. */
    if (!s_in_chat)
    {
        /* Address the desktop this column belongs to. Without a device the phone would
           have to guess, which is wrong the moment a second desktop is online — the send
           refuses rather than creating a session on the wrong machine. */
        device_sessions_t *d = sp_shown();
        s_await_new = true;
        if (!commu_send_conv_new(d->id))
        {
            s_await_new = false; /* never leave the flag armed on a send that failed */
            LOG_W("mic: conv_new send failed");
            return;
        }
        LOG_I("mic: requested a new session");
        return;
    }
    if (s_open_id[0] == '\0')
    {
        LOG_W("mic: in chat with no open session, ignoring");
        return;
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

/* ── Pinning the page while the panel comes over it ──────────────────────────
   founder 2026-08-10:「session 頁面不該往下離開，應該要是媒體中心蓋下來，跟錶盤還有通知
   列表的互動一樣」。The watch face reads that way because its blurred dial (gaus_dial_bg)
   is a SCREEN-level object that never scrolls — only the face's own contents ride the
   tileview. This page had everything inside the tile, so the tileview scroll carried it
   away instead of letting the panel cover it.
   So for the duration of the pull we re-parent the page (and the voice bar with it — they
   are one surface) OUT of the tile and onto the tileview's own parent, pushed to the
   background so the descending panel draws over it. Restored the moment the gesture ends,
   whichever way it resolves. */
/* 面板蓋下來時,底下的 session 頁要跟著變暗 —— 錶盤是靠 set_clock_main_status_opa 吃
   tileview 的 scroll_y 做到的(founder 2026-08-10:「需要像錶盤那樣有個慢慢變黑的背景」)。
   這裡用同一個訊號源:clock 端在 tileview 的 SCROLL 事件裡換算成 0..204 餵進來,所以
   拉下與收回兩個方向、手勢與慣性滑行全都自動跟上,不必各自補。 */
/* Which registry device this column is showing. Called by the clock when the watch face
   settles on a session column; re-renders only when the device actually changes so a
   settle on the same column never rebuilds the list under the user. */
void session_pager_set_column(int device_index, lv_obj_t *column_tile)
{
    if (device_index < 0)
        device_index = 0;
    /* MOVE THE UI INTO THAT COLUMN'S TILE.
       There is one session UI, built into the first column's tile. Switching columns used to
       only re-resolve WHICH device's rows to draw — so the rows were rebuilt correctly (the
       trace even reported rows=3) but kept rendering in column 0's tile, and the column the
       user had swiped to was an empty grid cell. founder 2026-08-11: 「第二欄還是空的」.
       Re-parenting is what actually puts it on screen. It also keeps s_home_tile honest, so
       the panel pin/unpin restores into the column we are actually on. */
    if (column_tile != NULL && lv_obj_is_valid(column_tile) && column_tile != s_home_tile)
    {
        s_home_tile = column_tile;
        if (s_root != NULL && lv_obj_is_valid(s_root))
        {
            lv_obj_set_parent(s_root, column_tile);
            lv_obj_set_pos(s_root, 0, 0);
        }
        if (s_voice_bar != NULL && lv_obj_is_valid(s_voice_bar))
        {
            lv_obj_set_parent(s_voice_bar, column_tile);
            lv_obj_align(s_voice_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
        }
    }
    if (device_index == s_dev_shown)
        return;
    s_dev_shown = device_index;
    /* Leaving a column abandons its room: the watch holds exactly one open conversation
       and it belongs to the device we just walked away from. */
    sp_leave_chat();
    s_known_count = 0; /* "seen" ids are per-device — otherwise the new column's rows all
                          look old and a conv_new there would never be walked into */
    s_await_new = false;
    sp_rebuild_list();
    LOG_W("session column -> %d id=%s rows=%d (devices stored=%d)", device_index,
          sp_shown()->id, sp_shown()->count, s_device_count);
}

void session_pager_set_dim(lv_opa_t opa)
{
    if (s_dim == NULL || !lv_obj_is_valid(s_dim))
        return;
    lv_obj_set_style_bg_opa(s_dim, opa, 0);
    if (opa == LV_OPA_TRANSP)
        lv_obj_add_flag(s_dim, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_clear_flag(s_dim, LV_OBJ_FLAG_HIDDEN);
}

void session_pager_pin_for_panel(lv_obj_t *fixed_parent)
{
    if (s_root == NULL || !lv_obj_is_valid(s_root))
        return;
    lv_obj_t *dst = (fixed_parent != NULL) ? fixed_parent : s_home_tile;
    if (dst == NULL || !lv_obj_is_valid(dst))
        return;
    if (lv_obj_get_parent(s_root) == dst)
        return;
    lv_obj_set_parent(s_root, dst);
    lv_obj_set_pos(s_root, 0, 0);
    if (s_voice_bar != NULL && lv_obj_is_valid(s_voice_bar))
    {
        lv_obj_set_parent(s_voice_bar, dst);
        lv_obj_align(s_voice_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    }
    if (fixed_parent != NULL)
    {
        /* Below the tileview, so the panel sliding down inside it covers this page. */
        lv_obj_move_background(s_voice_bar);
        lv_obj_move_background(s_root);
    }
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
    return s_root != NULL && lv_obj_is_valid(s_root) && s_visible && s_in_chat &&
           s_open_id[0] != '\0';
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

/* ── LIST layer ──────────────────────────────────────────────────────────── */

#define SP_ROW_H 76
#define SP_ROW_RADIUS 18
#define SP_ROW_BG 0x1C1C1E /* systemGray6 — content layer, no glass (Skaiwalk UI §1.1) */
#define SP_FONT_ROW_TITLE 19
#define SP_FONT_ROW_PREVIEW 16

static void sp_row_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    /* The index rides in user_data rather than being derived from the child order, so a
       row stays bound to its session even if the column is rebuilt around it. */
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    sp_enter_chat(idx);
}

static void sp_add_row(lv_obj_t *parent, const session_meta_t *s, int idx)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, SP_ROW_H);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(row, lv_color_hex(SP_ROW_BG), 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(row, SP_ROW_RADIUS, 0);
    lv_obj_set_style_pad_hor(row, 14, 0);
    lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
    /* The column scrolls vertically and the rows are the only thing in it, so a row must
       hand the vertical axis back up — otherwise a drag that starts on a row (i.e. almost
       every drag) dead-ends and the list cannot be scrolled at all. */
    lv_obj_add_flag(row, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_add_event_cb(row, sp_row_cb, LV_EVENT_CLICKED, (void *)(intptr_t)idx);

    lv_obj_t *title = lv_label_create(row);
    lv_label_set_long_mode(title, LV_LABEL_LONG_DOT);
    lv_obj_set_width(title, LV_HOR_RES - SP_LIST_SIDE_PAD - 48);
    sp_set_font(title, SP_FONT_ROW_TITLE);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(title, s->title[0] ? s->title : "Session");
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 12);

    if (s->preview[0])
    {
        lv_obj_t *prev = lv_label_create(row);
        lv_label_set_long_mode(prev, LV_LABEL_LONG_DOT);
        lv_obj_set_width(prev, LV_HOR_RES - SP_LIST_SIDE_PAD - 48);
        sp_set_font(prev, SP_FONT_ROW_PREVIEW);
        lv_obj_set_style_text_color(prev, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(prev, LV_OPA_60, 0); /* secondaryLabel */
        lv_label_set_text(prev, s->preview);
        lv_obj_align(prev, LV_ALIGN_TOP_LEFT, 0, 40);
    }
}

static void sp_rebuild_list(void)
{
    if (s_list_view == NULL || !lv_obj_is_valid(s_list_view))
        return;
    lv_obj_clean(s_list_view);
    device_sessions_t *d = sp_shown();
    for (int i = 0; i < d->count; i++)
        sp_add_row(s_list_view, &d->items[i], i);
    if (d->count == 0)
    {
        /* Empty state (Skaiwalk UI §4.2): say what the mic below will do rather than
           leaving a blank tile that reads as "broken" or "still loading". */
        lv_obj_t *hint = lv_label_create(s_list_view);
        lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(hint, LV_HOR_RES - SP_LIST_SIDE_PAD - 40);
        sp_set_font(hint, SP_FONT_ROW_PREVIEW);
        lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(hint, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(hint, LV_OPA_60, 0);
        lv_label_set_text(hint, "還沒有對話\n按下方麥克風開新的");
    }
    lv_obj_scroll_to_y(s_list_view, 0, LV_ANIM_OFF);
}

/* ── CHAT layer ──────────────────────────────────────────────────────────── */

static void sp_enter_chat(int idx)
{
    device_sessions_t *d = sp_shown();
    if (idx < 0 || idx >= d->count)
        return;
    const session_meta_t *s = &d->items[idx];
    if (s->id[0] == '\0')
        return;
    s_current = idx;

    if (s_open_id[0] != '\0' && strcmp(s_open_id, s->id) != 0)
        commu_send_conv_close();
    commu_send_conv_open(s->title, s->id, (uint8_t)idx);
    strncpy(s_open_id, s->id, SESSION_ID_LEN - 1);
    s_open_id[SESSION_ID_LEN - 1] = '\0';

    lv_label_set_text(s_chat_title, s->title[0] ? s->title : "Session");
    /* Seed with the preview so the room is never blank while KEY_CONV_STATE is in flight;
       the first real state replaces the column wholesale. */
    lv_obj_clean(s_chat_list);
    s_drawn_sig = 0;
    s_drawn_page = -1;
    if (s->preview[0])
        sp_add_bubble(s_chat_list, s->preview, true);
    lv_obj_scroll_to_y(s_chat_list, LV_COORD_MAX, LV_ANIM_OFF);

    lv_obj_add_flag(s_list_view, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_chat_view, LV_OBJ_FLAG_HIDDEN);
    s_in_chat = true;
    LOG_I("chat enter [%d]: %s", idx, s_open_id);
}

static void sp_leave_chat(void)
{
    if (!s_in_chat)
        return;
    if (s_listening)
    {
        /* Backing out mid-dictation drops the partial — the same rule leaving the tile
           has always applied. */
        sp_set_listening(false);
        voice_provider.auto_stop_listening();
        clearVoice2Text();
    }
    if (s_open_id[0] != '\0')
    {
        commu_send_conv_close();
        LOG_I("chat leave: %s", s_open_id);
        s_open_id[0] = '\0';
    }
    s_in_chat = false;
    lv_obj_add_flag(s_chat_view, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_list_view, LV_OBJ_FLAG_HIDDEN);
}

/* ── Back gesture (CHAT → LIST) ──────────────────────────────────────────────
   Left-edge right-swipe, the same affordance every other watch overlay uses. It is NOT the
   tileview's own edge-back: this tile's horizontal axis still has to take the user home
   from the LIST layer, so the chat room claims the gesture itself and only within the left
   edge band — a right-drag started anywhere else in the room still falls through.

   Why the displacement is accumulated during PRESSING and not read at RELEASE:
   the touch driver fabricates a release at the ORIGIN when its I2C read fails —
       E/drv.i2c    tpread: bus err:0, xfer:0/2, i2c_stat:20, i2c_errcode=4
       E/drv.ft3168 tpread: Error, return Up event, x = 0, y = 0
   captured on the dev watch 2026-08-10 while founder was testing these very gestures. A
   handler that measures `release_point - press_point` then computes a large NEGATIVE
   displacement, so a perfectly good rightward or downward drag fails its threshold and does
   nothing. Taps were unaffected (CLICKED carries no coordinate) — exactly the asymmetry
   founder reported: rows tappable, both drags dead. Tracking the furthest offset seen while
   the finger is still DOWN removes the release coordinate from the decision entirely. */
#define SP_BACK_EDGE_X 60
#define SP_BACK_DX 70

static lv_coord_t s_back_x0, s_back_y0, s_back_dx, s_back_dy;
static bool s_back_armed;

static void sp_chat_back_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (indev == NULL)
        return;
    lv_point_t pt;
    lv_indev_get_point(indev, &pt);

    if (code == LV_EVENT_PRESSED)
    {
        s_back_x0 = pt.x;
        s_back_y0 = pt.y;
        s_back_dx = 0;
        s_back_dy = 0;
        s_back_armed = (pt.x <= SP_BACK_EDGE_X);
        /* LOG_W, not LOG_I: this build's console prints W and E only — a whole round was
           spent reading an empty capture because the traces were LOG_I (2026-08-10). */
        LOG_W("[sp-back] pressed x=%d y=%d armed=%d", (int)pt.x, (int)pt.y,
              (int)s_back_armed);
    }
    else if (code == LV_EVENT_PRESSING)
    {
        lv_coord_t dx = pt.x - s_back_x0;
        lv_coord_t dy = pt.y - s_back_y0;
        if (dx > s_back_dx)
            s_back_dx = dx; /* furthest RIGHTWARD travel, monotonic */
        if (LV_ABS(dy) > LV_ABS(s_back_dy))
            s_back_dy = dy;
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
    {
        LOG_W("[sp-back] release armed=%d dx=%d dy=%d", (int)s_back_armed, (int)s_back_dx,
              (int)s_back_dy);
        if (!s_back_armed)
            return;
        s_back_armed = false;
        if (s_back_dx >= SP_BACK_DX && s_back_dx > LV_ABS(s_back_dy))
            sp_leave_chat();
    }
}

/* ── Pull down → this device's media centre ──────────────────────────────────
   Founder 2026-08-10: 「從上面往下拉他要拉出我那個設備的媒體中心」— the SAME media page the
   watch face's top panel already owns, not a second copy, and it must FOLLOW THE FINGER the
   way pulling it from the dial does (a set_tile_id jump was the first attempt: 「他為什麼是
   瞬間跳轉?」). So this feeds the clock's own three-phase follow every frame.
   Gated to a drag that STARTS in the top band, so the session list keeps its own scroll. */
#define SP_PULL_BAND_Y 70
#define SP_PULL_DY 70
#define SP_PULL_SLOP 12 /* 跟錶盤 FACE_SWIPE_SLOP 同量級：確定是往下拉才接管 */

static lv_coord_t s_pull_x0, s_pull_y0, s_pull_dx, s_pull_dy;
static bool s_pull_armed;
static bool s_pull_following; /* 已經把面板交給手指了（begin 送出過） */

static void sp_pulldown_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (indev == NULL)
        return;
    lv_point_t pt;
    lv_indev_get_point(indev, &pt);

    if (code == LV_EVENT_PRESSED)
    {
        s_pull_x0 = pt.x;
        s_pull_y0 = pt.y;
        s_pull_dx = 0;
        s_pull_dy = 0;
        s_pull_armed = (pt.y <= SP_PULL_BAND_Y);
        LOG_W("[sp-pull] pressed x=%d y=%d armed=%d", (int)pt.x, (int)pt.y,
              (int)s_pull_armed);
    }
    else if (code == LV_EVENT_PRESSING)
    {
        lv_coord_t dx = pt.x - s_pull_x0;
        lv_coord_t dy = pt.y - s_pull_y0;
        if (dy > s_pull_dy)
            s_pull_dy = dy; /* furthest DOWNWARD travel — see the note above */
        if (LV_ABS(dx) > LV_ABS(s_pull_dx))
            s_pull_dx = dx;
        if (!s_pull_following && s_pull_armed && dy > SP_PULL_SLOP && dy > LV_ABS(dx))
        {
            s_pull_following = true;
            /* Hand the room back first: the panel covers this tile, and a conversation
               left open behind it would keep claiming the mic. */
            sp_leave_chat();
            extern void clock_main_session_panel_follow_begin(void);
            clock_main_session_panel_follow_begin();
        }
        if (s_pull_following)
        {
            extern void clock_main_notify_follow_update(lv_coord_t);
            clock_main_notify_follow_update(dy);
        }
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
    {
        LOG_W("[sp-pull] release armed=%d follow=%d dx=%d dy=%d", (int)s_pull_armed,
              (int)s_pull_following, (int)s_pull_dx, (int)s_pull_dy);
        s_pull_armed = false;
        if (s_pull_following)
        {
            s_pull_following = false;
            /* Commit through the clock's session-column end: it reads the LIVE scroll
               position rather than the cumulative dy, so a pull-then-push-back cancels
               instead of opening, and it lands back in THIS column (2) rather than the
               watch face's. Velocity comes from the indev, same as the watch-face path. */
            lv_point_t v;
            lv_indev_get_vect(indev, &v);
            extern void clock_main_session_panel_follow_end(lv_coord_t, lv_coord_t);
            clock_main_session_panel_follow_end(s_pull_dy, v.y);
        }
    }
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
    if (s_root == NULL || !lv_obj_is_valid(s_root))
        return;
    bool vis = lv_obj_is_visible(s_root);
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
        /* Entering the tile: recover a list we may have missed. NOTHING is opened here —
           arriving on this tile now lands on the LIST layer, and a conversation is opened
           only by an explicit row tap. The old build opened whatever page happened to be
           centred, which is why merely sliding past the tile bound a conversation. */
        commu_send_conv_list_req(NULL); /* NULL = every desktop; the watch keeps one list each */
    }
    else
    {
        /* Leaving the tile abandons the room too: the watch holds exactly one open
           conversation and it must not survive off-screen. */
        sp_leave_chat();
    }
    LOG_I("session tile visible=%d", (int)vis);
}

/* ── BLE parse thread ────────────────────────────────────────────────────── */

/* 0x20 — {"device":{id,name},"sessions":[{id,title,preview}]}. One push per desktop.
   Bounded copies only; no LVGL calls here. */
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
    /* Absent "device" = the single unnamed desktop, i.e. what a pre-2026-08-10 phone
       sends. Slot it under the empty id so an old phone still fills the one column. */
    s_pending_dev_id[0] = '\0';
    s_pending_dev_name[0] = '\0';
    {
        cJSON *dev = cJSON_GetObjectItem(root, "device");
        if (cJSON_IsObject(dev))
        {
            cJSON *j_did = cJSON_GetObjectItem(dev, "id");
            if (cJSON_IsString(j_did))
                strncpy(s_pending_dev_id, j_did->valuestring, SESSION_ID_LEN - 1);
            cJSON *j_dname = cJSON_GetObjectItem(dev, "name");
            if (cJSON_IsString(j_dname))
                strncpy(s_pending_dev_name, j_dname->valuestring,
                        SESSION_DEVICE_NAME_LEN - 1);
        }
        else if (cJSON_IsString(dev))
        {
            strncpy(s_pending_dev_id, dev->valuestring, SESSION_ID_LEN - 1);
        }
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
    /* LOG_W: 這台 dev 錶的 console 只印 W/E,per-device 這條路要能在真機上判讀
       「有沒有收到第二台的推播」,靠 LOG_I 等於沒有 trace。 */
    LOG_W("conv_list rx: dev=%s name=%s sessions=%d", s_pending_dev_id, s_pending_dev_name,
          count);

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
    /* Which device slot this push belongs to — upsert, never replace the whole table:
       each desktop pushes its own list independently, so overwriting slot 0 on every
       arrival would make two desktops fight over one column. */
    int slot = -1;
    for (int i = 0; i < s_device_count; i++)
    {
        if (strcmp(s_devices[i].id, s_pending_dev_id) == 0)
        {
            slot = i;
            break;
        }
    }
    if (slot < 0)
    {
        if (s_device_count >= SESSION_DEVICE_MAX)
        {
            LOG_W("conv_list: device table full, dropping %s", s_pending_dev_id);
            return;
        }
        slot = s_device_count++;
        memset(&s_devices[slot], 0, sizeof(s_devices[slot]));
        strncpy(s_devices[slot].id, s_pending_dev_id, SESSION_ID_LEN - 1);
    }
    strncpy(s_devices[slot].name, s_pending_dev_name, SESSION_DEVICE_NAME_LEN - 1);

    device_sessions_t *dev = &s_devices[slot];
    bool unchanged = (count == dev->count && count > 0 &&
                      memcmp(dev->items, s_pending_sessions,
                             (size_t)count * sizeof(dev->items[0])) == 0);
    if (unchanged)
    {
        LOG_D("conv_list unchanged (%d sessions) — no rebuild", count);
        return;
    }
    memcpy(dev->items, s_pending_sessions, sizeof(dev->items));
    dev->count = count;

    /* Only the device the column is showing can change what is on screen. Another
       desktop's push is stored and nothing else — repainting for it would yank the list
       out from under the user. */
    /* Compare IDENTITY, not slot number: the storage slot is arrival order while the column
       is the registry position — a slot index that happens to equal the column would repaint
       the wrong device's list. */
    if (strcmp(dev->id, sp_shown()->id) != 0)
    {
        LOG_W("conv_list stored slot=%d id=%s (column %d shows id=%s)", slot, dev->id,
              s_dev_shown, sp_shown()->id);
        return;
    }
    LOG_W("conv_list painted slot=%d id=%s on column %d", slot, dev->id, s_dev_shown);

    /* Remember what was open so a list refresh doesn't silently drop the conversation
       out from under the user. */
    char was_open[SESSION_ID_LEN];
    strncpy(was_open, s_open_id, SESSION_ID_LEN - 1);
    was_open[SESSION_ID_LEN - 1] = '\0';

    /* Which id is NEW in this push, measured against everything we have ever rendered
       (s_known_ids), not against the immediately-previous list: the desktop may push an
       interim list between our conv_new and the one carrying the created session, and
       diffing against the previous push alone would call an unrelated row "new". */
    int fresh = -1;
    for (int i = 0; i < count && fresh < 0; i++)
    {
        bool seen = false;
        for (int k = 0; k < s_known_count && !seen; k++)
            seen = (strcmp(s_known_ids[k], s_pending_sessions[i].id) == 0);
        if (!seen)
            fresh = i;
    }

    sp_rebuild_list();

    /* Record what we now know about, capped like everything else on this path. */
    s_known_count = 0;
    for (int i = 0; i < count && s_known_count < SESSION_PAGER_MAX; i++)
    {
        strncpy(s_known_ids[s_known_count], dev->items[i].id, SESSION_ID_LEN - 1);
        s_known_ids[s_known_count][SESSION_ID_LEN - 1] = '\0';
        s_known_count++;
    }

    /* The mic asked for a new session and here it is — walk straight in (founder: 開新
       session 之後就在那個聊天室裡). Only when the tile is actually on screen: a list push
       that lands while the user is on the watch face must not yank them into a room. */
    if (s_await_new && fresh >= 0 && s_visible)
    {
        s_await_new = false;
        sp_enter_chat(fresh);
        return;
    }

    /* A room that is open stays open across a refresh, as long as its session survived. */
    if (s_in_chat && was_open[0] != '\0')
    {
        for (int i = 0; i < dev->count; i++)
        {
            if (strcmp(dev->items[i].id, was_open) == 0)
            {
                s_current = i; /* index may have moved; the id is what we are bound to */
                return;
            }
        }
        /* It's gone (session deleted on the desktop): the phone's conversation is stale, so
           drop the room rather than leaving a chat bound to nothing. */
        s_open_id[0] = '\0';
        s_in_chat = false;
        lv_obj_add_flag(s_chat_view, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_list_view, LV_OBJ_FLAG_HIDDEN);
    }
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
    /* State only ever paints the CHAT layer; on the LIST layer there is no open
       conversation for it to belong to. */
    if (!s_in_chat)
        return;
    lv_obj_t *list = s_chat_list;
    if (list == NULL || !lv_obj_is_valid(list))
        return;

    /* Re-render only when the thread actually changed. The phone pushes a folded conv_state
       on EVERY convEvent — including ones that fold to the same thing (reconnect re-pushes,
       a refresh, an event that only moved metadata) — and the render below is a full
       lv_obj_clean + rebuild of every bubble. founder 2026-08-05: 「session 的聊天內容如果
       沒有變化就不要重複刷新」. A 64-bit FNV-1a over what is actually DRAWN (each role+text,
       plus the sending flag that adds the "…" bubble) is far cheaper than the rebuild it
       skips, and streaming still updates every frame because the text differs every frame. */
    {
        uint64_t sig = 1469598103934665603ULL;
#define SP_SIG_BYTE(b)                          \
    do {                                        \
        sig ^= (uint64_t)(uint8_t)(b);          \
        sig *= 1099511628211ULL;                \
    } while (0)
        for (int i = 0; i < s_pending_msg_count; i++)
        {
            const sp_msg_t *m = &s_pending_msgs[i];
            for (const char *p = m->role; *p != '\0'; p++)
                SP_SIG_BYTE(*p);
            SP_SIG_BYTE('\x1f');
            for (const char *p = m->text; *p != '\0'; p++)
                SP_SIG_BYTE(*p);
            SP_SIG_BYTE('\x1e');
        }
        SP_SIG_BYTE(s_pending_sending ? 1 : 0);
#undef SP_SIG_BYTE
        /* Keyed by page too, so switching pages always redraws even if two sessions happen
           to fold to identical text. */
        if (sig == s_drawn_sig && s_current == s_drawn_page)
        {
            LOG_D("conv_state unchanged — no redraw");
            return;
        }
        s_drawn_sig = sig;
        s_drawn_page = s_current;
    }

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
    if (s_root == NULL || !lv_obj_is_valid(s_root))
        return;
    if (kind == SP_PENDING_LIST)
        sp_apply_list();
    else if (kind == SP_PENDING_STATE)
        sp_apply_state();
}

/* ── Public ──────────────────────────────────────────────────────────────── */

lv_obj_t *lv_session_pager_create(lv_obj_t *parent)
{
    /* No backdrop object here. The blurred dial behind this tile is the clock's SCREEN-LEVEL
       gaus_dial_bg — the same one the left action list uses (founder 2026-08-05:
       「底部高斯模糊背景要留在原地,不要跟著頁面滑動左右移動,看左側 action 列表怎麼做到的」).
       A backdrop parented to this TILE travels with the tileview as the tile slides in, which
       is exactly the sideways drift being complained about; the screen-level one never moves
       and only its opacity ramps. app_clock's scroll handler drives it. */
    lv_obj_t *root = lv_obj_create(parent);
    lv_obj_remove_style_all(root);
    /* Transparent by design — the backdrop above owns the pixels. */
    lv_obj_set_size(root, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(root, 0, 0);
    lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    s_root = root;
    s_home_tile = parent; /* where session_pager_pin_for_panel(NULL) puts us back */

    /* ── LIST layer ── */
    lv_obj_t *listv = lv_obj_create(root);
    lv_obj_remove_style_all(listv);
    lv_obj_set_size(listv, LV_HOR_RES - SP_LIST_SIDE_PAD, SP_LIST_H);
    lv_obj_align(listv, LV_ALIGN_TOP_MID, 0, SP_LIST_Y);
    lv_obj_set_scroll_dir(listv, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(listv, LV_SCROLLBAR_MODE_OFF);
    /* Vertical only, and the chain NOT extended horizontally: the tile's horizontal axis
       belongs to the tileview so a right-drag anywhere on the list still goes home. */
    lv_obj_set_flex_flow(listv, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(listv, 8, 0);
    s_list_view = listv;

    /* ── CHAT layer (hidden until a row is tapped) ── */
    lv_obj_t *chat = lv_obj_create(root);
    lv_obj_remove_style_all(chat);
    lv_obj_set_size(chat, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(chat, 0, 0);
    lv_obj_clear_flag(chat, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(chat, LV_OBJ_FLAG_HIDDEN);
    s_chat_view = chat;

    lv_obj_t *ctitle = lv_label_create(chat);
    lv_label_set_long_mode(ctitle, LV_LABEL_LONG_DOT);
    lv_obj_set_width(ctitle, LV_HOR_RES - 120);
    lv_obj_set_style_text_align(ctitle, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(ctitle, lv_color_hex(0xFFFFFF), 0);
    sp_set_font(ctitle, SP_FONT_TITLE);
    lv_label_set_text(ctitle, "");
    lv_obj_align(ctitle, LV_ALIGN_TOP_MID, 0, SP_TITLE_Y);
    s_chat_title = ctitle;

    lv_obj_t *clist = lv_obj_create(chat);
    lv_obj_set_size(clist, LV_HOR_RES - SP_LIST_SIDE_PAD, SP_LIST_H);
    lv_obj_align(clist, LV_ALIGN_TOP_MID, 0, SP_LIST_Y);
    lv_obj_set_style_bg_opa(clist, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(clist, 0, 0);
    lv_obj_set_scroll_dir(clist, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(clist, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_flow(clist, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(clist, 6, 0);
    lv_obj_set_style_pad_hor(clist, 2, 0);
    s_chat_list = clist;

    /* ── Gesture catchers ────────────────────────────────────────────────────
       Both gestures started as event handlers on the CONTAINERS. Neither ever fired the
       way it needed to: LVGL delivers a press to the TOPMOST object under the finger and
       picks the SCROLL target by walking up to the first SCROLLABLE ancestor — straight
       past a clickable non-scrollable catcher. The watch-face tileview therefore claimed
       the right-drag and the chat room exited to the dial (founder 2026-08-10).
       A dedicated catcher is this repo's existing answer (app_clock_main.c's
       face_swipe_catcher). Made SCROLLABLE but refusing the gesture's own axis, with that
       axis's chain cleared, it is a dead end for exactly that axis — the tileview never
       sees the drag, so the gesture stays ours. */

    /* Left-edge strip, CHAT only — created after the bubble column so it sits above it. */
    lv_obj_t *back_edge = lv_obj_create(chat);
    lv_obj_remove_style_all(back_edge);
    lv_obj_set_size(back_edge, SP_BACK_EDGE_X, LV_VER_RES);
    lv_obj_align(back_edge, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_opa(back_edge, LV_OPA_TRANSP, 0);
    lv_obj_add_flag(back_edge, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(back_edge, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(back_edge, LV_DIR_VER);
    lv_obj_clear_flag(back_edge, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_set_scrollbar_mode(back_edge, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(back_edge, sp_chat_back_cb, LV_EVENT_ALL, NULL);

    /* Top band, BOTH layers — a root child created last, so it covers list and chat alike. */
    lv_obj_t *pull_band = lv_obj_create(root);
    lv_obj_remove_style_all(pull_band);
    lv_obj_set_size(pull_band, LV_HOR_RES, SP_PULL_BAND_Y);
    lv_obj_align(pull_band, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(pull_band, LV_OPA_TRANSP, 0);
    lv_obj_add_flag(pull_band, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(pull_band, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(pull_band, LV_DIR_HOR);
    lv_obj_clear_flag(pull_band, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_set_scrollbar_mode(pull_band, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(pull_band, sp_pulldown_cb, LV_EVENT_ALL, NULL);

    /* Scrim, created LAST so it covers the page (and the catchers) — driven by
       session_pager_set_dim. Not clickable: it must never swallow a touch, it is purely
       a visual, and while it is up the panel over it owns the input anyway. */
    lv_obj_t *dim = lv_obj_create(root);
    lv_obj_remove_style_all(dim);
    lv_obj_set_size(dim, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(dim, 0, 0);
    lv_obj_set_style_bg_color(dim, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(dim, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(dim, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(dim, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(dim, LV_OBJ_FLAG_HIDDEN);
    s_dim = dim;

    /* Shared voice bar — a TILE child created AFTER both layers, so it floats above them
       and does not scroll with either. One mic, one ripple, one transcript for the whole
       tile; what a tap MEANS depends on the layer (new session on LIST, dictate in CHAT —
       see sp_mic_toggle). */
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
    s_in_chat = false;
    s_await_new = false;
    s_known_count = 0;
    sp_rebuild_list();
    lv_timer_create(sp_visibility_timer_cb, SP_VIS_POLL_MS, NULL);

    LOG_I("session list created (%d devices)", s_device_count);

#ifdef BSP_USING_PC_SIMULATOR
    /* PC sim has no BLE, so the phone never pushes a session list — stand in with a
       few so the layout can actually be looked at. Never compiled for the watch. */
    if (s_device_count == 0)
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
        return root;
    }
#endif

    /* Nothing cached yet — ask the phone for the list. The visibility timer asks again
       on every entry, so a push that arrives while disconnected is never lost for good. */
    commu_send_conv_list_req(NULL); /* NULL = every desktop; the watch keeps one list each */
    return root;
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
    /* Seeds the FIRST device slot — this entry point predates the per-device split and is
       only used by the PC-sim fixture, which has exactly one imaginary desktop. */
    if (s_device_count == 0)
        s_device_count = 1;
    s_dev_shown = 0;
    device_sessions_t *dev = &s_devices[0];
    memset(dev->items, 0, sizeof(dev->items));
    for (int i = 0; i < count; i++)
    {
        if (ids != NULL)
        {
            strncpy(dev->items[i].id, ids[i], SESSION_ID_LEN - 1);
            dev->items[i].id[SESSION_ID_LEN - 1] = '\0';
        }
        if (titles != NULL)
        {
            strncpy(dev->items[i].title, titles[i], SESSION_TITLE_LEN - 1);
            dev->items[i].title[SESSION_TITLE_LEN - 1] = '\0';
        }
        if (previews != NULL)
        {
            strncpy(dev->items[i].preview, previews[i], SESSION_PREVIEW_LEN - 1);
            dev->items[i].preview[SESSION_PREVIEW_LEN - 1] = '\0';
        }
    }
    dev->count = count;
    LOG_I("sessions set: %d", count);
    sp_rebuild_list();
}

const char *lv_session_pager_current_id(void)
{
    device_sessions_t *d = sp_shown();
    if (d->count <= 0 || s_current < 0 || s_current >= d->count)
        return NULL;
    return d->items[s_current].id;
}
