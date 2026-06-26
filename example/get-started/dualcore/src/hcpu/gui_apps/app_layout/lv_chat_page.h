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
       lv_layer_top (over the @-list). Must be called on the LVGL thread (the tap cb is). */
    void chat_page_open(const char *title);

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

#ifdef __cplusplus
}
#endif

#endif /* LV_CHAT_PAGE_H */
