/**
 ******************************************************************************
 * @file   lv_chat_page.h
 * @brief  In-watch @-conversation chat room (P5 "run @ chat on the watch").
 *
 * A floating panel on lv_layer_top (above the @-list layer) that MIRRORS the
 * desktop conversation: opened from a left-@ contact tap, the phone routes the
 * open to the owning desktop (km-relay convOpen), the desktop runs it headless
 * and streams turns back, and the phone pushes the folded chat state DOWN to the
 * watch (KEY_CONV_STATE -> skai_chat_on_conv_state). Back-swipe sends conv-close.
 ******************************************************************************
 */
#ifndef LV_CHAT_PAGE_H
#define LV_CHAT_PAGE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* Open the chat room for a tapped @-contact. Builds a fresh full-screen panel on
       lv_layer_top (over the @-list). [icon_src] is the row's service logo (an lv_img_dsc_t* cast to
       const char*, or NULL) shown small before the name in the header. LVGL thread (the tap cb is). */
    void chat_page_open(const char *title, const char *icon_src);

    /* Hermes(AI session)vs messaging(@聯絡人)畫風分流 — 開房前設定(桌面
       ConversationPane 的 IsHermes 同款)。true = Hermes 平鋪風(使用者=全寬玻璃卡、
       AI=平鋪全寬文字);false = iMessage 氣泡(左灰右藍)。 */
    void chat_page_set_style_hermes(bool hermes);

    /* 聊天室內非邊緣的左右滑(2026-08-30,滑鼠抽屜 bot 房用,tileview 手感):
       dir=+1 滑左(切更舊的 session)、-1 滑右(更新)。dry_run=true 只回答
       「這個方向有沒有房」(彈回判定);false 才真的換(送 conv_open + 呼叫
       chat_page_switch_session)。開房後由開房的人註冊;chat_page_close 歸零,
       沒註冊的房(錶盤 '@' 房)行為不變。LVGL thread。 */
    void chat_page_set_session_switcher(bool (*cb)(int dir, bool dry_run));

    /* 就地換房:標題/清空泡泡/回「載入中」,不拆面板(滑入滑出動畫由 chat page
       自己駕駛)。switcher cb 的 dry_run=false 分支呼叫。LVGL thread。 */
    void chat_page_switch_session(const char *title);

    /* Tear the chat panel down (revealing the @-list underneath). Idempotent. LVGL thread. */
    void chat_page_close(void);

    /* True while the chat panel is up. */
    bool chat_page_is_open(void);

    /* DOWNLINK hook (KEY_CONV_STATE) — called on the BLE PARSE thread with the folded
       chat-state JSON {title, sending, messages:[{role,text}]}. Renders the transcript.
       STRONG symbol (resolve_skailink_command references it; a weak ref would be
       dead-stripped -> NULL -> silent no-op, the device_pager ui_refresh lesson). */
    void skai_chat_on_conv_state(const uint8_t *json, uint16_t length);

    /* LVGL-thread render of the pending conv-state (called from ui_handler's
       LVGL_MSG_TYPE_REFRESH_CHAT, which skai_chat_on_conv_state triggers). */
    void chat_page_apply_pending_state(void);

    /* 開新對話時先把使用者剛講的那句話畫成一則已送出的訊息(桌面約 4 秒後才回傳真實
       transcript,那之前房間是空的)。真實 conv_state 回來會整份重畫,不會重複。LVGL thread。 */
    void chat_page_seed_local_message(const char *text);

    /* LVGL-thread update of the live mic transcript box — called from the shared voice router
       (refresh_ai_chat_input_message's chat branch), with the running partial transcript. */
    void chat_page_set_transcript(const char *text);

    /* Global RELEASE gesture hook: starts (or stops+sends, if already recording) the chat mic,
       mirroring chat_mic_btn_cb's tap toggle. No-op if the chat panel isn't open. Called from
       gesture_recognition_task.c BEFORE the is_at_instruction_list() branch — the @-list stays
       open underneath chat, so that check alone can't distinguish the two screens. */
    void chat_page_start_voice_input(void);

#ifdef __cplusplus
}
#endif

#endif /* LV_CHAT_PAGE_H */
