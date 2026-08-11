/**
 ******************************************************************************
 * @file   lv_session_pager.h
 * @brief  合併 session 列表 — ADR-0020 錶盤左頁 (0,1)。
 *
 * 跨設備合併成一份列表（通知卡樣式,每列副標小字標來源設備,有 ts 就按時間
 * 混排）,下方接 actions 列表（資料/執行都在 lv_instruction_list_layout.c,
 * 這裡只畫列與轉發點擊）。點列進聊天室、左緣右滑返回;列表層麥克風 = 對
 * active 設備開新 session (0x24)。
 *
 * 儲存仍是 per-desktop（convOpen 要 route 回正確的機器）,只有呈現層合併。
 * Memory: only session METADATA is held per row; the live transcript belongs to
 * the OPEN conversation only — the watch keeps exactly one open at a time
 * (convOpen on row tap / convClose on leave).
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
/* 桌面設備上限,與媒體欄 / MAX_SYNCED_DEVICES 同步。 */
#define SESSION_DEVICE_MAX 4
#define SESSION_DEVICE_NAME_LEN 32
#define SESSION_ID_LEN 64
#define SESSION_TITLE_LEN 48
#define SESSION_PREVIEW_LEN 128

    /** Build the merged list into [parent] (the watch-face LEFT tile (0,1)). */
    lv_obj_t *lv_session_pager_create(lv_obj_t *parent);

    /** Replace the session list and rebuild (PC-sim fixture). LVGL thread only. */
    void lv_session_pager_set_sessions(const char (*ids)[SESSION_ID_LEN],
                                       const char (*titles)[SESSION_TITLE_LEN],
                                       const char (*previews)[SESSION_PREVIEW_LEN],
                                       int count);

    /** The session id of the open conversation, or NULL when none. */
    const char *lv_session_pager_current_id(void);

    /** 左緣入口把頁面拉出來之前呼叫:回到列表頂端。 */
    void session_list_reset_scroll(void);

    /** actions 清單有更新(手機推播落地)時由 instruction list 呼叫,重畫合併列表。 */
    void session_list_actions_changed(void);

    /* ── Voice, mirroring lv_chat_page.h's chat_page_* trio ── */

    /** True while the tile is on screen with a session open — voice belongs here. */
    bool session_pager_is_open(void);
    /** Live partial transcript from the phone's transcriber. */
    void session_pager_set_transcript(const char *text);
    /** Start/stop dictation for the open session (release-gesture entry point). */
    void session_pager_start_voice_input(void);

    /* ── Cross-thread plumbing ──
       skai_sessions_* run on the BLE parse thread (bounded copies, no LVGL); the
       render is deferred to session_pager_apply_pending(). */

    /** 0x20 KEY_CONV_LIST — one desktop's session list ({id,title,preview,ts?}). */
    void skai_sessions_on_conv_list(const uint8_t *json, uint16_t length);
    /** True when a KEY_CONV_STATE belongs here rather than the @-list chat room. */
    bool skai_sessions_owns_conv(void);
    /** 0x12 KEY_CONV_STATE, routed here when skai_sessions_owns_conv(). */
    void skai_sessions_on_conv_state(const uint8_t *json, uint16_t length);
    /** LVGL-thread render of whatever the two above parsed. */
    void session_pager_apply_pending(void);

#ifdef __cplusplus
}
#endif

#endif /* LV_SESSION_PAGER_H */
