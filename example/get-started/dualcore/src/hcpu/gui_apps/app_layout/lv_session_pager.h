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
