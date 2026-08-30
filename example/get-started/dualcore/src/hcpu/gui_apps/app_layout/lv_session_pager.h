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

/* Bounded like every other watch-side list — no heap on the BLE parse path.
   這是**儲存**上限(每台桌面留幾筆 metadata),不是顯示上限:搜尋要能在使用者看不到的
   那幾筆裡命中,所以池子必須比畫面上的深。 */
/* 2026-08-30 起 = 25:最近 5 個 bot × 各自最新 5 個 session(founder;桌面
   ConversationRelayController.TrimToRecentBots 先修剪好才推)。倉庫本體因此搬進
   PSRAM(見 lv_session_pager.c sp_storage_ensure)—— 25 列 × 268B × 4 台 × 2 份
   (committed+pending)≈54KB,SRAM 塞不下。 */
#define SESSION_PAGER_MAX 25
/* 一台桌面在清單上**同時畫出來**的 session 上限(founder 2026-08-24:「每個設備只會有
   最新的 5 個 session」)。四台桌面 × 8 筆全部注入 actions 清單時,LVGL 每次重建 32 列
   卡片(每列還有右緣圓框/設備名),錶盤左頁明顯卡頓。顯示收到 5、儲存仍留 8:
   - 平常 = 每台最新 5 筆;
   - 有語音搜尋字時 = 每台**比中查詢**的最新 5 筆(池子是那台留著的 8 筆)。 */
#define SESSION_VISIBLE_MAX 5
/* 桌面設備上限,與媒體欄 / MAX_SYNCED_DEVICES 同步。 */
#define SESSION_DEVICE_MAX 4
#define SESSION_DEVICE_NAME_LEN 32
#define SESSION_ID_LEN 64
#define SESSION_TITLE_LEN 48
#define SESSION_PREVIEW_LEN 128
/* Hermes profile(Bot)名 — 0x20 每列選配的 "bot" 欄。 */
#define SESSION_BOT_LEN 24

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

    /** 語音搜尋字變了(instruction list 的 s_text_filter)時由 instruction list 呼叫:
        重挑每台桌面要露出的那 5 筆(有查詢字 = 比中的最新 5 筆)並重新注入。 */
    void session_list_text_filter_changed(void);

    /** 對 [device_id] 開新 session 並武裝 walk-in(清單推回來自動進聊天室)。
        滑鼠頁底部 skaibar tap 用;呼叫端負責把畫面切到左頁。 */
    void session_list_open_new_for_device(const char *device_id);

    /* ── 滑鼠抽屜 bot 聊天室(2026-08-30)──
       都是唯讀查詢,LVGL thread only;id 一律是 "conv:<service>:<sessionId>"。 */

    /** 反查一筆 session:成功時填 title(可 NULL 略過)與 bot(可 NULL)。 */
    bool session_list_lookup(const char *device_id, const char *conv_id,
                             char *out_title, size_t title_len,
                             char *out_bot, size_t bot_len);
    /** 反查一筆 session 的擁有者:掃過**所有**設備,填 device 與 bot(皆可 NULL)。
        錶盤左頁的清單跨設備,點下去時只有 conv id,用這支補出另外兩個查詢要的 device。 */
    bool session_list_owner_of(const char *conv_id,
                               char *out_device, size_t device_len,
                               char *out_bot, size_t bot_len);
    /** [bot] 最新(ts 最大)的 session;bot=="" 或 NULL = 不過濾。 */
    bool session_list_latest_for_bot(const char *device_id, const char *bot,
                                     char *out_id, size_t id_len,
                                     char *out_title, size_t title_len);
    /** [cur_id] 在同 bot session(ts 由新到舊排序)中的鄰居。dir>0 = 更舊,
        dir<0 = 更新。到端點回 false。 */
    bool session_list_neighbor(const char *device_id, const char *bot,
                               const char *cur_id, int dir,
                               char *out_id, size_t id_len,
                               char *out_title, size_t title_len);

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
