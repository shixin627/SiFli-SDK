/**
 ******************************************************************************
 * @file   lv_session_pager.h
 * @brief  Desktop-session pager — the watch-face RIGHT tile (was the App List).
 *
 * One horizontally-snapping page per desktop chat session (the Hermes
 * conversations the phone lists via AgentSessionBridge/convList). Each page
 * shows that session's recent turns as notification-style bubbles; a single
 * shared voice bar sits at the bottom of the tile (NOT inside the pager, so it
 * doesn't scroll with the pages) and drives the current page's session.
 *
 * Memory: only session METADATA is held per page (title + preview). The live
 * transcript belongs to the CURRENT page only — the watch keeps exactly one
 * conversation open at a time (convOpen on page settle / convClose on leave),
 * so the existing single chat buffer in lv_chat_page.c is reused rather than
 * multiplied by the page count. See docs/adr (session pager).
 ******************************************************************************
 */
#ifndef LV_SESSION_PAGER_H
#define LV_SESSION_PAGER_H

#include "lvgl.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Bounded like every other watch-side list — no heap on the BLE parse path. */
#define SESSION_PAGER_MAX 8
/* One desktop per column on the watch face's right side (founder 2026-08-10). Four is what
   the top panel's media pages already cap at (MAX_SYNCED_DEVICES), so the two stay in step. */
#define SESSION_DEVICE_MAX 4
#define SESSION_DEVICE_NAME_LEN 32
#define SESSION_ID_LEN 64
#define SESSION_TITLE_LEN 48
#define SESSION_PREVIEW_LEN 128

    /** Build the pager into [parent] (the watch-face right tile). Returns the
        scrolling container. */
    lv_obj_t *lv_session_pager_create(lv_obj_t *parent);

    /** Replace the session list and rebuild the pages. [count] is clamped to
        SESSION_PAGER_MAX. Call on the LVGL thread only. */
    void lv_session_pager_set_sessions(const char (*ids)[SESSION_ID_LEN],
                                       const char (*titles)[SESSION_TITLE_LEN],
                                       const char (*previews)[SESSION_PREVIEW_LEN],
                                       int count);

    /** The session id of the page currently centred, or NULL when empty. */
    const char *lv_session_pager_current_id(void);

    /* ── 頂部面板從這一頁下拉時的兩個掛鉤（app_clock_status_bar.c 呼叫）──
       面板是錶盤 tileview 的一格，捲動會把整個 tile 帶走；這兩個函式讓這一頁在面板
       蓋上來時「留在原地並變暗」，跟錶盤的互動一致。 */

    /** 把這一頁（含底部語音列）暫時移出自己的 tile、釘在 [fixed_parent] 上並壓到最底，
        於是 tileview 捲動只帶走面板。傳 NULL 放回原本的 tile。只在拖曳期間成立 —— 留著
        它會掛在 tileview 外面，滑到任何一頁都跟著出現。 */
    void session_pager_pin_for_panel(lv_obj_t *fixed_parent);

    /** 面板下拉進度 → 這一頁上方黑色 scrim 的濃度（0..204）。由 clock 端在 tileview 的
        SCROLL 事件裡換算，所以拉下 / 收回 / 慣性滑行都連續。 */
    void session_pager_set_dim(lv_opa_t opa);

    /** 錶盤停在右側第 [device_index] 欄時呼叫：切換顯示哪一台的 sessions，並把這一份
        session UI 搬進 [column_tile]（UI 只有一份，不搬過去那一欄就是空格子）。 */
    void session_pager_set_column(int device_index, lv_obj_t *column_tile);

    /* The blurred backdrop behind this tile is the clock's screen-level gaus_dial_bg (shared
       with the left action list), ramped by app_clock_status_bar.c's tileview scroll handler.
       The pager owns no backdrop of its own — one parented here would slide with the tile. */

    /* ── Voice, mirroring lv_chat_page.h's chat_page_* trio ──
       The watch has ONE mic; whichever surface is up claims it. Callers on the shared
       voice paths (interact_voice_recognition, the release gesture) must ask this
       FIRST — a pager page and the @-list chat room are never open together. */

    /** True while the pager tile is on screen with a session open — i.e. voice belongs
        to the pager, not to the @-list chat room. */
    bool session_pager_is_open(void);
    /** Live partial transcript from the phone's transcriber. */
    void session_pager_set_transcript(const char *text);
    /** Start/stop dictation for the centred session (release-gesture entry point). */
    void session_pager_start_voice_input(void);

    /* ── Cross-thread plumbing ──
       skai_sessions_* run on the BLE parse thread (bounded copies, no LVGL); the render
       is deferred to session_pager_apply_pending() via LVGL_MSG_TYPE_REFRESH_SESSIONS. */

    /** 0x20 KEY_CONV_LIST — the desktop session list. */
    void skai_sessions_on_conv_list(const uint8_t *json, uint16_t length);
    /** True when a KEY_CONV_STATE belongs to a pager page (a conversation is open here)
        rather than to the @-list chat room. */
    bool skai_sessions_owns_conv(void);
    /** 0x12 KEY_CONV_STATE, routed here when skai_sessions_owns_conv(). */
    void skai_sessions_on_conv_state(const uint8_t *json, uint16_t length);
    /** LVGL-thread render of whatever the two above parsed. */
    void session_pager_apply_pending(void);

#ifdef __cplusplus
}
#endif

#endif /* LV_SESSION_PAGER_H */
