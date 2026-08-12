/**
 ******************************************************************************
 * @file   lv_session_pager.c
 * @brief  合併 session 列表（ADR-0020 左頁）。See lv_session_pager.h.
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
   The list stops well clear of the bottom band so the shared voice bar floats
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

/* Type scale — Skaiwalk design tokens §4. */
#define SP_FONT_TITLE 22      /* textLg  — section title */
#define SP_FONT_BUBBLE 19     /* textMd  — subhead; body legibility at arm's length */
#define SP_FONT_TRANSCRIPT 17 /* textBase */

/* Voice ripple — the SAME pulse the skaibar bar's push-to-talk uses. */
#define SP_RIPPLE_COLOR 0x5DA8FF
#define SP_RIPPLE_MIN_D 48
#define SP_RIPPLE_MAX_D 176
#define SP_RIPPLE_PERIOD_MS 1200
#define SP_MIC_Y (-14)

#define SP_V2T_INTENT 2 /* V2T_INTENT_REMOTE_INPUT — same intent the chat room uses */

/* Transcript buffer for the OPEN page only. Deliberately smaller than the chat room's
   16x256: SRAM here is tight (the mouse app has OOM'd opening the mic). */
#define SP_MSG_MAX 12
#define SP_MSG_LEN 192

typedef struct
{
    char id[SESSION_ID_LEN];
    char title[SESSION_TITLE_LEN];
    char preview[SESSION_PREVIEW_LEN];
    uint32_t ts; /* epoch 秒(牆鐘偽 UTC,同手錶其他 timestamp);0 = 手機沒給 */
} session_meta_t;

typedef struct
{
    char role[12];
    char text[SP_MSG_LEN];
} sp_msg_t;

/* ── Committed state (LVGL thread only) ──
   Storage stays PER DESKTOP (a flat list cannot say which desktop a row belongs to,
   so convOpen would route at the wrong machine); ADR-0020 merges only the VIEW:
   one list, every device's sessions sorted by ts (absent ts = grouped by device). */
typedef struct
{
    char id[SESSION_ID_LEN];
    char name[SESSION_DEVICE_NAME_LEN];
    session_meta_t items[SESSION_PAGER_MAX];
    int count;
} device_sessions_t;

static device_sessions_t s_devices[SESSION_DEVICE_MAX];
static int s_device_count;
static int s_shown_slot = -1; /* s_devices slot the OPEN conversation belongs to */
static int s_current;         /* index into that slot's items — the open session */

/* ── UI ── */
static lv_obj_t *s_root;      /* the tile child everything else hangs off */
static lv_obj_t *s_list_view; /* ONE merged list: session cards + actions section */
static lv_obj_t *s_chat_view; /* CHAT layer — one session's transcript (hidden on LIST) */
static lv_obj_t *s_chat_title;
static lv_obj_t *s_chat_list; /* the bubble column inside CHAT */
static bool s_in_chat;
static lv_obj_t *s_voice_bar; /* shared bottom voice affordance (tile child) */
static lv_obj_t *s_mic_img;
static lv_obj_t *s_ripple;
static lv_obj_t *s_transcript; /* live V2T text, shown while listening */
static bool s_listening;
/* Set when the mic on the LIST layer asked for a new session on s_await_dev_id.
   The desktop answers with an ordinary list push; the row that device's push carries
   which we have never stored before is the one to walk into. */
static bool s_await_new;
static char s_await_dev_id[SESSION_ID_LEN];
/* The session the watch currently has OPEN on the phone (convOpen sent, no convClose
   yet), "" when none. Exactly one at a time. */
static char s_open_id[SESSION_ID_LEN];
static bool s_visible; /* tile on screen (tracked by the visibility timer) */

/* ── Pending, written on the BLE parse thread ──
   One slot per desktop: two desktops answer ~113 ms apart (measured 2026-08-11) and a
   single shared buffer loses the first push. Keyed by device id. */
#define SP_PENDING_NONE 0
#define SP_PENDING_LIST 1
#define SP_PENDING_STATE 2
static volatile int s_pending_kind;

typedef struct
{
    volatile int ready; /* published LAST, after the payload is fully written */
    char dev_id[SESSION_ID_LEN];
    char dev_name[SESSION_DEVICE_NAME_LEN];
    session_meta_t sessions[SESSION_PAGER_MAX];
    int count;
} pending_list_t;

static pending_list_t s_pending_lists[SESSION_DEVICE_MAX];

/* Working copies the LVGL-thread apply path reads. */
static session_meta_t s_pending_sessions[SESSION_PAGER_MAX];
static int s_pending_session_count;
static char s_pending_dev_id[SESSION_ID_LEN];
static char s_pending_dev_name[SESSION_DEVICE_NAME_LEN];

static char s_pending_sid[SESSION_ID_LEN];
static char s_pending_title[SESSION_TITLE_LEN];
static bool s_pending_sending;
static sp_msg_t s_pending_msgs[SP_MSG_MAX];
static int s_pending_msg_count;

/* What the chat page currently HAS drawn — redraw skip, see sp_apply_state. */
static uint64_t s_drawn_sig;
static int s_drawn_page = -1;

static void sp_enter_chat(int slot, int idx);
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
   Mirrors lv_chat_page.c's proven sequence (the only path device-verified to reach
   the phone's transcriber — repo CLAUDE.md R7). */

static void sp_stop_and_send(void)
{
    if (!s_listening)
        return;
    sp_set_listening(false);
    voice_provider.auto_stop_listening();
    const char *text = get_combined_voice2text();
    if (text != NULL && text[0] != '\0')
    {
        commu_send_conv_send(text);
        clearVoice2Text();
    }
}

/* LIST 層麥克風/滑鼠頁 skaibar 開新 session 的目標:目前控制中的那台(registry
   active),沒有就 registry 第 0 台,再沒有就已存列表的第 0 台。 */
static const char *sp_new_session_target(void)
{
    extern const char *hid_mouse_device_id(int idx);
    extern int hid_mouse_active_device_index(void);
    extern int hid_mouse_device_count(void);
    const char *id = NULL;
    int a = hid_mouse_active_device_index();
    if (a >= 0)
        id = hid_mouse_device_id(a);
    if (id == NULL && hid_mouse_device_count() > 0)
        id = hid_mouse_device_id(0);
    if (id == NULL && s_device_count > 0)
        id = s_devices[0].id;
    return id;
}

/* 對 [dev_id] 開新 session 並武裝 walk-in:清單推回來、看到沒見過的 id 就自動
   走進聊天室(sp_apply_list)。dev_id NULL = 用預設目標。 */
static bool sp_request_new_session(const char *dev_id)
{
    if (dev_id == NULL || dev_id[0] == '\0')
        dev_id = sp_new_session_target();
    if (dev_id == NULL || dev_id[0] == '\0')
    {
        LOG_W("conv_new: no device to create a session on");
        return false;
    }
    s_await_new = true;
    strncpy(s_await_dev_id, dev_id, SESSION_ID_LEN - 1);
    s_await_dev_id[SESSION_ID_LEN - 1] = '\0';
    if (!commu_send_conv_new(dev_id))
    {
        s_await_new = false; /* never leave the flag armed on a failed send */
        LOG_W("conv_new send failed");
        return false;
    }
    LOG_W("conv_new requested on %s", dev_id);
    return true;
}

/** 滑鼠頁底部 skaibar tap(founder 2026-08-11 R6):開「那台設備」的新 session,
    UI 與左頁 session 一樣 —— 呼叫端先把畫面切到左頁,清單推回來就 walk-in。 */
void session_list_open_new_for_device(const char *device_id)
{
    sp_request_new_session(device_id);
}

static void sp_mic_toggle(void)
{
    if (s_listening)
    {
        sp_stop_and_send();
        return;
    }
    /* LIST layer(founder 2026-08-11 R6:「點麥克風也是直接去開新的 session」):
       直接對目標設備發 conv_new,清單推回來自動走進新聊天室。 */
    if (!s_in_chat)
    {
        sp_request_new_session(NULL);
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

/* Live partial transcript, routed here from interact_voice_recognition
   (app_system_interface.c) while this page owns the mic. */
void session_pager_set_transcript(const char *text)
{
    if (!s_listening || s_transcript == NULL || !lv_obj_is_valid(s_transcript))
        return;
    bool has = (text != NULL && text[0] != '\0');
    lv_label_set_text(s_transcript, has ? text : "聆聽中…");
}

bool session_pager_is_open(void)
{
    return s_root != NULL && lv_obj_is_valid(s_root) && s_visible && s_in_chat &&
           s_open_id[0] != '\0';
}

void session_pager_start_voice_input(void)
{
    if (!session_pager_is_open())
        return;
    sp_mic_toggle();
}

/** 左緣入口把頁面拉出來之前呼叫:回到列表頂端(最新的列)。 */
void session_list_reset_scroll(void)
{
    if (s_list_view != NULL && lv_obj_is_valid(s_list_view))
        lv_obj_scroll_to_y(s_list_view, 0, LV_ANIM_OFF);
}

/* ── Fonts / bubbles ─────────────────────────────────────────────────────── */

/* lvsf_get_font_from_size returns NULL when freetype never initialised; keep the
   inherited default in that case — small text beats no watch. */
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
    /* BEFORE the measure below — lv_txt_get_size reads the label's current font. */
    sp_set_font(lbl, SP_FONT_BUBBLE);
    /* LVGL clips (rather than re-wraps) a SIZE_CONTENT label pinned by max_width, so
       measure first and only pin the width when the text actually overflows. */
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

/* ── Merged LIST layer ───────────────────────────────────────────────────── */

/* founder 2026-08-11 R6:「session ui 要跟本來的 actions ui 長得一樣」——
   session 列與 actions 列共用同一種緊湊列樣式(icon + 標題,同高同底色),
   不再是大卡片。preview 捨棄;來源設備名保留為右側小字(R1 決策 #3)。 */
#define SP_ROW_H 68
#define SP_ROW_RADIUS 18
#define SP_ROW_BG 0x1C1C1E /* systemGray6 — content layer, no glass (Skaiwalk UI §1.1) */
#define SP_FONT_ROW_TITLE 19
#define SP_FONT_ROW_DEVICE 13

/* 兩個區段共用的緊湊列。icon_src 可為 NULL(純文字);right_tag 可為 NULL。 */
static lv_obj_t *sp_add_compact_row(lv_obj_t *parent, const void *icon_src,
                                    const char *title_text, const char *right_tag,
                                    lv_event_cb_t cb, void *user_data)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, SP_ROW_H);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(row, lv_color_hex(SP_ROW_BG), 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_60, 0);
    lv_obj_set_style_radius(row, SP_ROW_RADIUS, 0);
    lv_obj_set_style_pad_hor(row, 14, 0);
    lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
    /* Hand the vertical axis back up — otherwise a drag that starts on a row (i.e.
       almost every drag) dead-ends and the list cannot be scrolled at all. */
    lv_obj_add_flag(row, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_add_event_cb(row, cb, LV_EVENT_CLICKED, user_data);

    /* founder 2026-08-11 R7:「之前的 actions 列表那樣,中間是文字、右邊是圖標」
       —— 標題置中,icon 靠右;session 沒有 icon,右側放小字設備名(同一個位置)。 */
    lv_obj_t *lbl = lv_label_create(row);
    lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
    lv_obj_set_width(lbl, LV_HOR_RES - SP_LIST_SIDE_PAD - 150);
    sp_set_font(lbl, SP_FONT_ROW_TITLE);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(lbl, title_text);
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);

    if (icon_src != NULL)
    {
        lv_obj_t *icon = lv_img_create(row);
        lv_img_set_src(icon, icon_src);
        lv_img_set_zoom(icon, 128); /* 80px 源圖 → ~40px */
        lv_obj_align(icon, LV_ALIGN_RIGHT_MID, 8, 0);
        lv_obj_clear_flag(icon, LV_OBJ_FLAG_CLICKABLE);
    }
    else if (right_tag != NULL && right_tag[0])
    {
        lv_obj_t *dev = lv_label_create(row);
        lv_label_set_long_mode(dev, LV_LABEL_LONG_DOT);
        lv_obj_set_width(dev, 70);
        sp_set_font(dev, SP_FONT_ROW_DEVICE);
        lv_obj_set_style_text_align(dev, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_style_text_color(dev, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(dev, LV_OPA_40, 0); /* tertiaryLabel */
        lv_label_set_text(dev, right_tag);
        lv_obj_align(dev, LV_ALIGN_RIGHT_MID, 0, 0);
    }
    return row;
}

static void sp_row_cb(lv_event_t *e)
{
    /* slot/idx ride in user_data so a row stays bound to its session even if the
       list is rebuilt around it. */
    uint32_t packed = (uint32_t)(intptr_t)lv_event_get_user_data(e);
    sp_enter_chat((int)(packed >> 8), (int)(packed & 0xFF));
}

static void sp_add_row(lv_obj_t *parent, int slot, int idx)
{
    const session_meta_t *s = &s_devices[slot].items[idx];
    sp_add_compact_row(parent, NULL, s->title[0] ? s->title : "Session",
                       s_devices[slot].name, sp_row_cb,
                       (void *)(intptr_t)(((uint32_t)slot << 8) | (uint32_t)idx));
}

/* ── Actions 區段(接在 session 列表下方,ADR-0020)──
   資料與執行路徑都在 lv_instruction_list_layout.c(手機推播的 0x65/0x6B 清單);
   這裡只畫列與轉發點擊,浮層那套(bar tap 開浮動清單)原樣保留。 */
static void sp_action_row_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    extern void instruction_list_activate_index(uint8_t i);
    instruction_list_activate_index((uint8_t)(intptr_t)lv_event_get_user_data(e));
}

static void sp_append_actions(lv_obj_t *view)
{
    extern uint8_t return_total_list_count(void);
    extern const char *instruction_list_export_title(uint8_t i);
    extern const void *instruction_list_export_icon(uint8_t i);

    uint8_t n = return_total_list_count();
    if (n == 0)
        return;

    /* 分隔線(不用文字 —— 免去 i18n 兩份 JSON 的成本,線本身已經足夠分段)。 */
    lv_obj_t *divider = lv_obj_create(view);
    lv_obj_remove_style_all(divider);
    lv_obj_set_size(divider, lv_pct(86), 2);
    lv_obj_set_style_bg_color(divider, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(divider, LV_OPA_20, 0);
    lv_obj_clear_flag(divider, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    for (uint8_t i = 0; i < n; i++)
    {
        const char *title = instruction_list_export_title(i);
        if (title == NULL || title[0] == '\0')
            continue;
        /* R6:與 session 列共用同一個 builder —— 兩個區段長得一模一樣。 */
        sp_add_compact_row(view, instruction_list_export_icon(i), title, NULL,
                           sp_action_row_cb, (void *)(intptr_t)i);
    }
}

/* ── ADR-0020 R8:左頁顯示的就是**原本的浮動 actions 清單本體**(founder 附圖:
   黑底、文字置中、右緣圓形圖標輪播、底部麥克風)。session 以 '@' item 塞進那份
   清單,點了走既有 '@' 路徑(conv_open + chat_page);這裡的自有列表/聊天層退役成
   隱藏備援。upsert/移除都有變動才 refresh,避免 refresh→hook→inject 迴圈。 */
static void sp_inject_sessions_into_actions(void)
{
    extern void add_or_update_custom_instruction(const char *id, const char *title,
                                                 const char *trigger_type,
                                                 uint32_t interval_sec, bool enabled,
                                                 uint32_t version, const char *open_app);
    extern void set_instruction_category(const char *id, char cat);
    extern void remove_custom_instruction(const char *id);
    extern void refresh_custom_instructions(void);
    extern uint8_t return_total_list_count(void);
    extern const char *instruction_list_export_id(uint8_t i);
    extern const char *instruction_list_export_title(uint8_t i);

    bool changed = false;

    /* 移除:清單裡 conv: 開頭、但儲存裡已不存在的(桌面刪了 session)。 */
    for (int i = (int)return_total_list_count() - 1; i >= 0; i--)
    {
        const char *iid = instruction_list_export_id((uint8_t)i);
        if (iid == NULL || strncmp(iid, "conv:", 5) != 0)
            continue;
        bool still = false;
        for (int d = 0; d < s_device_count && !still; d++)
            for (int k = 0; k < s_devices[d].count && !still; k++)
                still = (strcmp(s_devices[d].items[k].id, iid) == 0);
        if (!still)
        {
            remove_custom_instruction(iid);
            changed = true;
        }
    }

    /* upsert:按 ts 新→舊(首次注入的順序就是清單順序);title 沒變就不動
       (add_or_update 恆觸發重繪成本)。 */
    struct
    {
        uint8_t slot;
        uint8_t idx;
        uint32_t ts;
    } order[SESSION_DEVICE_MAX * SESSION_PAGER_MAX];
    int cnt = 0;
    for (int d = 0; d < s_device_count; d++)
        for (int k = 0; k < s_devices[d].count; k++)
        {
            order[cnt].slot = (uint8_t)d;
            order[cnt].idx = (uint8_t)k;
            order[cnt].ts = s_devices[d].items[k].ts;
            cnt++;
        }
    for (int i = 1; i < cnt; i++)
    {
        int j = i;
        while (j > 0 && order[j - 1].ts < order[j].ts)
        {
            uint8_t t_slot = order[j].slot, t_idx = order[j].idx;
            uint32_t t_ts = order[j].ts;
            order[j] = order[j - 1];
            order[j - 1].slot = t_slot;
            order[j - 1].idx = t_idx;
            order[j - 1].ts = t_ts;
            j--;
        }
    }
    for (int o = 0; o < cnt; o++)
    {
        const device_sessions_t *dv = &s_devices[order[o].slot];
        const session_meta_t *s = &dv->items[order[o].idx];
        /* R10(founder):顯示標題帶上來源設備 —— 「標題 · 設備名」。輪播項只有
           一個 label,併進標題最直接;點進聊天室的標頭也跟著標明來源。
           LIST_ITEM_TITLE_LEN=64,超長由 add_or_update 的 strncpy 截斷。 */
        char disp[96];
        if (dv->name[0])
            rt_snprintf(disp, sizeof(disp), "%s · %s",
                        s->title[0] ? s->title : "Session", dv->name);
        else
            rt_snprintf(disp, sizeof(disp), "%s", s->title[0] ? s->title : "Session");
        bool same = false;
        uint8_t n = return_total_list_count();
        for (uint8_t i = 0; i < n; i++)
        {
            const char *iid = instruction_list_export_id(i);
            const char *it = instruction_list_export_title(i);
            if (iid && it && strcmp(iid, s->id) == 0 && strncmp(it, disp, 63) == 0)
            {
                same = true;
                break;
            }
        }
        if (!same)
        {
            add_or_update_custom_instruction(s->id, disp, "", 0, false, 0, "");
            changed = true;
        }
        set_instruction_category(s->id, '@'); /* 冪等 */
    }

    if (changed)
    {
        /* R9(founder:「actions 在下面」):session 移到清單前段、actions 在後。 */
        extern void instruction_list_move_conv_items_first(void);
        instruction_list_move_conv_items_first();
        refresh_custom_instructions();
    }
}

/** actions 清單有更新(手機推播 0x65/0x6B 落地)時由 instruction list 呼叫:
    把 session 重新注入(replace-all 會把它們洗掉)。 */
void session_list_actions_changed(void)
{
    sp_inject_sessions_into_actions();
}

/* 合併重建:全設備的 sessions 收成一份,有 ts 就按 ts 由新到舊,沒 ts 的
   (舊 APK)排在有 ts 的後面、按設備順序群聚 —— 穩定插入排序,量級 4x8=32。 */
static void sp_rebuild_list(void)
{
    if (s_list_view == NULL || !lv_obj_is_valid(s_list_view))
        return;
    lv_obj_clean(s_list_view);

    struct
    {
        uint8_t slot;
        uint8_t idx;
        uint32_t ts;
    } order[SESSION_DEVICE_MAX * SESSION_PAGER_MAX];
    int n = 0;
    for (int d = 0; d < s_device_count; d++)
    {
        for (int i = 0; i < s_devices[d].count; i++)
        {
            order[n].slot = (uint8_t)d;
            order[n].idx = (uint8_t)i;
            order[n].ts = s_devices[d].items[i].ts;
            n++;
        }
    }
    for (int i = 1; i < n; i++)
    {
        /* stable insertion, ts 大者在前;ts==0 恆沉底(uint 比較天然成立) */
        int j = i;
        while (j > 0 && order[j - 1].ts < order[j].ts)
        {
            uint8_t ts_slot = order[j].slot, ts_idx = order[j].idx;
            uint32_t ts_ts = order[j].ts;
            order[j] = order[j - 1];
            order[j - 1].slot = ts_slot;
            order[j - 1].idx = ts_idx;
            order[j - 1].ts = ts_ts;
            j--;
        }
    }

    for (int i = 0; i < n; i++)
        sp_add_row(s_list_view, order[i].slot, order[i].idx);

    if (n == 0)
    {
        /* Empty state (Skaiwalk UI §4.2): say what the mic below will do. */
        lv_obj_t *hint = lv_label_create(s_list_view);
        lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(hint, LV_HOR_RES - SP_LIST_SIDE_PAD - 40);
        sp_set_font(hint, SP_FONT_TRANSCRIPT);
        lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(hint, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(hint, LV_OPA_60, 0);
        lv_label_set_text(hint, "還沒有對話\n按下方麥克風開新的");
    }

    sp_append_actions(s_list_view);
}

/* ── CHAT layer ──────────────────────────────────────────────────────────── */

static void sp_enter_chat(int slot, int idx)
{
    if (slot < 0 || slot >= s_device_count)
        return;
    device_sessions_t *d = &s_devices[slot];
    if (idx < 0 || idx >= d->count)
        return;
    const session_meta_t *s = &d->items[idx];
    if (s->id[0] == '\0')
        return;
    s_shown_slot = slot;
    s_current = idx;

    if (s_open_id[0] != '\0' && strcmp(s_open_id, s->id) != 0)
        commu_send_conv_close();
    commu_send_conv_open(s->title, s->id, (uint8_t)idx);
    strncpy(s_open_id, s->id, SESSION_ID_LEN - 1);
    s_open_id[SESSION_ID_LEN - 1] = '\0';

    lv_label_set_text(s_chat_title, s->title[0] ? s->title : "Session");
    /* Seed with the preview so the room is never blank while KEY_CONV_STATE is in
       flight; the first real state replaces the column wholesale. */
    lv_obj_clean(s_chat_list);
    s_drawn_sig = 0;
    s_drawn_page = -1;
    if (s->preview[0])
        sp_add_bubble(s_chat_list, s->preview, true);
    lv_obj_scroll_to_y(s_chat_list, LV_COORD_MAX, LV_ANIM_OFF);

    if (s_list_view != NULL && lv_obj_is_valid(s_list_view))
        lv_obj_add_flag(s_list_view, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_chat_view, LV_OBJ_FLAG_HIDDEN);
    s_in_chat = true;
    LOG_I("chat enter [%d:%d]: %s", slot, idx, s_open_id);
}

static void sp_leave_chat(void)
{
    if (!s_in_chat)
        return;
    if (s_listening)
    {
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
    if (s_list_view != NULL && lv_obj_is_valid(s_list_view))
        lv_obj_clear_flag(s_list_view, LV_OBJ_FLAG_HIDDEN);
}

/* ── Back gesture (CHAT → LIST) ──────────────────────────────────────────────
   Left-edge right-swipe. Displacement is accumulated during PRESSING, never read at
   RELEASE: the touch driver fabricates a (0,0) Up event when its I2C read fails
   (dev-watch captured 2026-08-10), which poisons release-point math. */
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
        /* LOG_W, not LOG_I: this dev watch's console prints W and E only. */
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

/* ── Visibility ──────────────────────────────────────────────────────────────
   The page lives in a tileview tile; polled at 500ms (one timer, zero coupling to
   app_clock_status_bar's bookkeeping). */
#define SP_VIS_POLL_MS 500

static void sp_visibility_timer_cb(lv_timer_t *t)
{
    (void)t;
    if (s_root == NULL || !lv_obj_is_valid(s_root))
        return;
    bool vis = lv_obj_is_visible(s_root);
    /* 手機 AI 輸入框(麥克風開的,活在 lv_layer_top)關閉後把浮層收回去 ——
       留著的話 skaibar pill 會浮在左頁我們自己的麥克風上。500ms 一次、
       idempotent;框開著或浮動清單還在時不動。 */
    if (vis)
    {
        extern bool get_is_open_instruction_list_ai(void);
        extern bool instruction_list_is_visible(void);
        extern void instruction_list_bar_set_visible(bool visible);
        if (!get_is_open_instruction_list_ai() && !instruction_list_is_visible())
            instruction_list_bar_set_visible(false);
    }
    if (vis == s_visible)
        return;
    s_visible = vis;
    if (vis)
    {
        /* Entering the tile: recover a list we may have missed. NOTHING is opened
           here — a conversation is opened only by an explicit row tap. */
        commu_send_conv_list_req(NULL); /* NULL = every desktop */
    }
    else
    {
        /* Leaving the tile abandons the room: the watch holds exactly one open
           conversation and it must not survive off-screen. */
        sp_leave_chat();
    }
    LOG_I("session tile visible=%d", (int)vis);
}

/* ── BLE parse thread ────────────────────────────────────────────────────── */

/* 0x20 — {"device":{id,name},"sessions":[{id,title,preview,ts?}]}. One push per
   desktop. Bounded copies only; no LVGL calls here. "ts" (epoch 秒) 是 ADR-0020
   合併排序用的新欄位 —— 舊 APK 沒有,缺了就是 0(沉底、按設備群聚)。 */
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
    /* Parsed into LOCALS, not shared statics: two desktops answer within ~100 ms on
       this same thread, so anything shared here is a lost push. */
    char dev_id[SESSION_ID_LEN];
    char dev_name[SESSION_DEVICE_NAME_LEN];
    session_meta_t parsed[SESSION_PAGER_MAX];
    memset(parsed, 0, sizeof(parsed));
    dev_id[0] = '\0';
    dev_name[0] = '\0';
    {
        cJSON *dev = cJSON_GetObjectItem(root, "device");
        if (cJSON_IsObject(dev))
        {
            cJSON *j_did = cJSON_GetObjectItem(dev, "id");
            if (cJSON_IsString(j_did))
            {
                strncpy(dev_id, j_did->valuestring, SESSION_ID_LEN - 1);
                dev_id[SESSION_ID_LEN - 1] = '\0';
            }
            cJSON *j_dname = cJSON_GetObjectItem(dev, "name");
            if (cJSON_IsString(j_dname))
            {
                strncpy(dev_name, j_dname->valuestring, SESSION_DEVICE_NAME_LEN - 1);
                dev_name[SESSION_DEVICE_NAME_LEN - 1] = '\0';
            }
        }
        else if (cJSON_IsString(dev))
        {
            strncpy(dev_id, dev->valuestring, SESSION_ID_LEN - 1);
            dev_id[SESSION_ID_LEN - 1] = '\0';
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
            strncpy(parsed[count].id, j_id->valuestring, SESSION_ID_LEN - 1);
            cJSON *j_title = cJSON_GetObjectItem(it, "title");
            if (cJSON_IsString(j_title))
                strncpy(parsed[count].title, j_title->valuestring, SESSION_TITLE_LEN - 1);
            cJSON *j_prev = cJSON_GetObjectItem(it, "preview");
            if (cJSON_IsString(j_prev))
                strncpy(parsed[count].preview, j_prev->valuestring, SESSION_PREVIEW_LEN - 1);
            cJSON *j_ts = cJSON_GetObjectItem(it, "ts");
            if (cJSON_IsNumber(j_ts) && j_ts->valuedouble > 0)
                parsed[count].ts = (uint32_t)j_ts->valuedouble;
            count++;
        }
    }
    cJSON_Delete(root);
    /* Park it in THIS DESKTOP's own slot (see pending_list_t). */
    {
        int slot = -1;
        for (int i = 0; i < SESSION_DEVICE_MAX; i++)
        {
            if (s_pending_lists[i].ready && strcmp(s_pending_lists[i].dev_id, dev_id) == 0)
            {
                slot = i; /* this desktop re-pushed before we drained — replace its own slot */
                break;
            }
        }
        if (slot < 0)
        {
            for (int i = 0; i < SESSION_DEVICE_MAX; i++)
            {
                if (!s_pending_lists[i].ready)
                {
                    slot = i;
                    break;
                }
            }
        }
        if (slot < 0)
        {
            LOG_W("conv_list rx: pending full, dropping %s", dev_id);
            return;
        }
        pending_list_t *p = &s_pending_lists[slot];
        p->ready = 0; /* stop a drain from reading a half-written payload */
        strncpy(p->dev_id, dev_id, SESSION_ID_LEN - 1);
        p->dev_id[SESSION_ID_LEN - 1] = '\0';
        strncpy(p->dev_name, dev_name, SESSION_DEVICE_NAME_LEN - 1);
        p->dev_name[SESSION_DEVICE_NAME_LEN - 1] = '\0';
        memcpy(p->sessions, parsed, sizeof(p->sessions));
        p->count = count;
        p->ready = 1; /* publish LAST */
        LOG_W("conv_list rx: dev=%s name=%s sessions=%d -> pending[%d]", dev_id, dev_name,
              count, slot);
    }

    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_REFRESH_SESSIONS};
    lvgl_send_msg(msg);
}

/* True while a KEY_CONV_STATE belongs to this page rather than the @-list chat room.
   Read from the BLE parse thread by handle_conv_state. */
bool skai_sessions_owns_conv(void)
{
    return s_open_id[0] != '\0';
}

/* 0x12 (routed here when this page owns the conversation) —
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
        /* Keep the LAST SP_MSG_MAX valid turns — with a full backlog the NEWEST
           message is the one that matters. */
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

    /* Which device slot this push belongs to — upsert, never replace the table. */
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
    /* 收到某台的列表 = 那台在線,順手讓錶盤重算右側可滑的媒體欄數。 */
    {
        extern void clock_main_media_cols_refresh(void);
        clock_main_media_cols_refresh();
    }

    device_sessions_t *dev = &s_devices[slot];
    /* An identical list is the COMMON case (the phone re-pushes on every reconnect
       and every list request); this early-out is the rebuild-loop's off switch
       (real-hw measured 2026-08-05), not an optimisation. */
    bool unchanged = (count == dev->count && count > 0 &&
                      memcmp(dev->items, s_pending_sessions,
                             (size_t)count * sizeof(dev->items[0])) == 0);
    if (unchanged)
    {
        LOG_D("conv_list unchanged (%d sessions) — no rebuild", count);
        return;
    }

    /* Which id is NEW in this push — diffed against what this device HAD before the
       overwrite, only consumed when the mic's conv_new targeted this very device. */
    int fresh = -1;
    if (s_await_new && strcmp(dev->id, s_await_dev_id) == 0)
    {
        for (int i = 0; i < count && fresh < 0; i++)
        {
            bool seen = false;
            for (int k = 0; k < dev->count && !seen; k++)
                seen = (strcmp(dev->items[k].id, s_pending_sessions[i].id) == 0);
            if (!seen)
                fresh = i;
        }
    }

    /* Remember what was open so a list refresh doesn't silently drop the room. */
    char was_open[SESSION_ID_LEN];
    strncpy(was_open, s_open_id, SESSION_ID_LEN - 1);
    was_open[SESSION_ID_LEN - 1] = '\0';

    memcpy(dev->items, s_pending_sessions, sizeof(dev->items));
    dev->count = count;

    /* 合併視圖(隱藏備援)重畫 + 把 session 注入浮動 actions 清單(左頁的真身)。 */
    sp_rebuild_list();
    sp_inject_sessions_into_actions();

    /* The mic asked for a new session and here it is — walk straight in (founder:
       開新 session 之後就在那個聊天室裡). Only when the tile is on screen.
       R8:聊天室 = chat_page(與點 '@' item 同一條路),不再用自有 chat 層;
       不設 s_open_id,0x12 才會路由給 chat_page。 */
    if (s_await_new && fresh >= 0 && s_visible)
    {
        s_await_new = false;
        const session_meta_t *ns = &dev->items[fresh];
        commu_send_conv_open(ns->title, ns->id, 0);
        extern void chat_page_open(const char *title, const char *icon_src);
        chat_page_open(ns->title[0] ? ns->title : "Session", NULL);
        return;
    }

    /* A room that is open stays open across a refresh, as long as it survived. */
    if (s_in_chat && was_open[0] != '\0' && slot == s_shown_slot)
    {
        for (int i = 0; i < dev->count; i++)
        {
            if (strcmp(dev->items[i].id, was_open) == 0)
            {
                s_current = i; /* index may have moved; the id is what we're bound to */
                return;
            }
        }
        /* It's gone (deleted on the desktop): drop the room rather than leaving a
           chat bound to nothing. */
        s_open_id[0] = '\0';
        s_in_chat = false;
        lv_obj_add_flag(s_chat_view, LV_OBJ_FLAG_HIDDEN);
        if (s_list_view != NULL && lv_obj_is_valid(s_list_view))
            lv_obj_clear_flag(s_list_view, LV_OBJ_FLAG_HIDDEN);
    }
}

static void sp_apply_state(void)
{
    /* Drop a late frame from the conversation we just left. */
    if (s_pending_sid[0] != '\0' && s_open_id[0] != '\0' &&
        strcmp(s_pending_sid, s_open_id) != 0)
    {
        LOG_D("conv_state for %s ignored (open=%s)", s_pending_sid, s_open_id);
        return;
    }
    if (!s_in_chat)
        return;
    lv_obj_t *list = s_chat_list;
    if (list == NULL || !lv_obj_is_valid(list))
        return;

    /* Re-render only when the thread actually changed (FNV-1a over what is drawn) —
       the phone pushes a folded conv_state on EVERY convEvent. */
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

    /* Drain EVERY desktop that has something waiting — two desktops answer nearly
       together and this callback runs once per LVGL message. */
    for (int i = 0; i < SESSION_DEVICE_MAX; i++)
    {
        if (!s_pending_lists[i].ready)
            continue;
        pending_list_t *p = &s_pending_lists[i];
        strncpy(s_pending_dev_id, p->dev_id, SESSION_ID_LEN - 1);
        s_pending_dev_id[SESSION_ID_LEN - 1] = '\0';
        strncpy(s_pending_dev_name, p->dev_name, SESSION_DEVICE_NAME_LEN - 1);
        s_pending_dev_name[SESSION_DEVICE_NAME_LEN - 1] = '\0';
        memcpy(s_pending_sessions, p->sessions, sizeof(s_pending_sessions));
        s_pending_session_count = p->count;
        p->ready = 0; /* released BEFORE the apply, so a re-push mid-apply is kept */
        sp_apply_list();
    }

    if (kind == SP_PENDING_STATE)
        sp_apply_state();
}

/* ── Public ──────────────────────────────────────────────────────────────── */

lv_obj_t *lv_session_pager_create(lv_obj_t *parent)
{
    /* No backdrop object here: the blurred dial behind this tile is the clock's
       SCREEN-LEVEL gaus_dial_bg (a tile-parented one would slide with the page). */
    lv_obj_t *root = lv_obj_create(parent);
    lv_obj_remove_style_all(root);
    lv_obj_set_size(root, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(root, 0, 0);
    lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    /* NOT clickable — lv_obj_create hands out CLICKABLE by default and a transparent
       full-screen catcher swallows every tap (founder 2026-08-11 hit exactly this). */
    lv_obj_clear_flag(root, LV_OBJ_FLAG_CLICKABLE);
    s_root = root;

    /* ── Merged LIST(session 卡 + actions 區段)── */
    lv_obj_t *view = lv_obj_create(root);
    lv_obj_remove_style_all(view);
    lv_obj_set_size(view, LV_HOR_RES - SP_LIST_SIDE_PAD, SP_LIST_H);
    lv_obj_align(view, LV_ALIGN_TOP_MID, 0, SP_LIST_Y);
    lv_obj_set_scroll_dir(view, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(view, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_flow(view, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(view, 8, 0);
    /* R8:左頁的真身是浮動 actions 清單(session 已注入那份);這份自有列表退役成
       隱藏備援,避免拉頁過程先閃一版舊樣式。 */
    lv_obj_add_flag(view, LV_OBJ_FLAG_HIDDEN);
    s_list_view = view;

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

    /* ── Back-gesture catcher(CHAT only)──
       SCROLLABLE but refusing its own axis, with that axis's chain cleared — a dead
       end for exactly that axis, so the watch-face tileview never sees the drag. */
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

    /* ── Shared voice bar — a TILE child created after both layers, floats above.
       One mic; a tap means "new session" on LIST, "dictate" in CHAT. ── */
    lv_obj_t *bar = lv_obj_create(parent);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, LV_HOR_RES, 96);
    lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    /* R8:麥克風入口改用浮動清單自己的 pill(session-page 模式下 tap=開新
       session);這條自有語音列一併退役隱藏。 */
    lv_obj_add_flag(bar, LV_OBJ_FLAG_HIDDEN);
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
    s_await_dev_id[0] = '\0';
    s_shown_slot = -1;
    sp_rebuild_list();
    lv_timer_create(sp_visibility_timer_cb, SP_VIS_POLL_MS, NULL);

    LOG_I("merged session list created (%d devices)", s_device_count);

#ifdef BSP_USING_PC_SIMULATOR
    /* PC sim has no BLE — stand in with a few sessions so the layout can be looked
       at. Never compiled for the watch. */
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

    /* Nothing cached yet — ask the phone. The visibility timer asks again on every
       entry, so a push that arrives while disconnected is never lost for good. */
    commu_send_conv_list_req(NULL); /* NULL = every desktop */
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
    /* Seeds the FIRST device slot — PC-sim fixture only (one imaginary desktop). */
    if (s_device_count == 0)
        s_device_count = 1;
    device_sessions_t *dev = &s_devices[0];
    memset(dev->items, 0, sizeof(dev->items));
    strncpy(dev->name, "Demo PC", SESSION_DEVICE_NAME_LEN - 1);
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
        dev->items[i].ts = (uint32_t)(1000 + count - i); /* 保持傳入順序 */
    }
    dev->count = count;
    LOG_I("sessions set: %d", count);
    sp_rebuild_list();
}

const char *lv_session_pager_current_id(void)
{
    if (s_shown_slot < 0 || s_shown_slot >= s_device_count)
        return NULL;
    device_sessions_t *d = &s_devices[s_shown_slot];
    if (d->count <= 0 || s_current < 0 || s_current >= d->count)
        return NULL;
    return d->items[s_current].id;
}
