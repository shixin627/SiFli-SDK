/* SPDX-License-Identifier: Apache-2.0 */
#ifndef SKAI_NOTIFICATION_REPLY_H
#define SKAI_NOTIFICATION_REPLY_H
#include <stdbool.h>
#include <stdint.h>
#include "bloc_notification.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef enum {
    NOTIFICATION_REPLY_NONE, NOTIFICATION_REPLY_WAITING, NOTIFICATION_REPLY_SENT,
    NOTIFICATION_REPLY_FAILED, NOTIFICATION_REPLY_UNKNOWN, NOTIFICATION_REPLY_INVALID,
    NOTIFICATION_REPLY_UNSUPPORTED, NOTIFICATION_REPLY_FULL, NOTIFICATION_REPLY_DISMISSED
} notification_reply_state_t;
typedef struct {
    uint32_t (*now_ms)(void);
    bool (*ready)(void);
    bool (*awake)(void);
    bool (*send)(const char *json, uint16_t length);
    bool (*new_id)(char output[33]);
} notification_reply_transport_t;
typedef struct {
    uint32_t input;
    notification_reply_state_t state;
    const char *draft; /* Borrowed until the next GUI operation; copy into the view. */
} notification_reply_view_t;
notification_reply_view_t notification_reply_open(const notification_t *notification);
notification_reply_view_t notification_reply_open_version(const char *instance, uint32_t revision);
bool notification_reply_can_finish(uint32_t input);
bool notification_reply_can_submit(uint32_t input);
void notification_reply_bind_transport(const notification_reply_transport_t *transport);
/* Complete reply operations. Submission never means dispatched. */
void notification_reply_peer(bool connected, bool capable);
void notification_reply_link(bool connected);
bool notification_reply_available(void);
bool notification_reply_pending(const char *instance, uint32_t revision);
uint32_t notification_reply_begin(const notification_t *notification);
uint32_t notification_reply_begin_version(const char *instance, uint32_t revision);
bool notification_reply_listen(uint32_t input);
/* Voice adapter: renew scoped delivery when dictation resumes after an amendment. */
bool notification_reply_prepare_voice(void);
/* Current input only; cancelled/replaced views cannot detach its observer. */
void notification_reply_bind_transcript(uint32_t input, void (*deliver)(uint32_t input, const char *text));
void notification_reply_cancel(uint32_t input);
/* Explicitly clear an unsent/failed draft and invalidate its late voice. */
uint32_t notification_reply_clear(uint32_t input);
notification_reply_state_t notification_reply_submit(uint32_t input, const char *text);
notification_reply_state_t notification_reply_option(const notification_t *notification, const char *text);
notification_reply_state_t notification_reply_option_version(const char *instance, uint32_t revision, const char *text);
notification_reply_state_t notification_reply_state(uint32_t input);
bool notification_reply_query(uint32_t input);
bool notification_reply_dismiss(const notification_t *notification);
bool notification_reply_was_processed(const char *instance, uint32_t revision);
notification_reply_state_t notification_reply_version_state(const char *instance, uint32_t revision);
/* Producer thread: admission only; GUI process publishes accepted results. */
bool notification_reply_receive(const void *json, uint16_t length);
void notification_reply_process(void);
#ifdef __cplusplus
}
#endif
#endif
