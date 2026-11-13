/* Copyright (c) 2026 Skaiwalk Technology
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SKAI_INSTRUCTION_LIST_H
#define SKAI_INSTRUCTION_LIST_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t key;
    const void *payload;
    uint16_t length;
    uint8_t command; /* 0 is the ordinary NOTIFY group. */
} instruction_list_delivery_t;

typedef struct
{
    uint32_t (*now_ms)(void);
    bool (*ready)(void);
    bool (*awake)(void);
    bool (*request)(uint32_t id, uint32_t scope);
} instruction_list_transport_t;

typedef struct
{
    /* GUI thread. Row exports expose the prospective frame during prepare.
       A rejection leaves the visible frame untouched. */
    bool (*prepare)(const char *focused_id);
    void (*publish)(void);
    /* Release unused resources after publication or rejection. */
    void (*finish)(bool committed);
} instruction_list_presentation_t;

/* The real LVGL adapter or a recording/rejecting GUI adapter. Copied on bind. */
void instruction_list_bind_presentation(const instruction_list_presentation_t *presentation);

typedef struct
{
    bool needs_sync;
    bool supports_resend;
    uint32_t active_request;
    uint32_t completed_request;
    uint32_t scope;
    size_t render_reserved_bytes;
    size_t render_peak_bytes;
} instruction_list_status_t;

/* Bind once to the real or recording adapter. NULL restores the real adapter.
 * Status and binding are GUI-thread operations. */
void instruction_list_bind_transport(const instruction_list_transport_t *transport);
instruction_list_status_t instruction_list_get_status(void);
void instruction_list_link_changed(bool connected);
void instruction_list_scope_changed(void);
/* GUI thread: store updates may arrive while searching; presentation waits. */
void instruction_list_sessions_changed(void);
/* Complete GUI operation: select sessions, repair the view and resume updates. */
void instruction_list_set_query(const char *text);
/* Complete GUI operations. An open rejected by the renderer is retried;
 * close cancels that intent. category is 0, '@' or '/'. */
bool instruction_list_open(char category);
bool instruction_list_open_device_drawer(void);
bool instruction_list_open_voice(void);
void instruction_list_close(void);
/* Replace a borrowed device list; close restores the complete previous context.
 * Options are in the existing display/tap order. NULL/0 retains the current
 * fallback until options arrive. Does not open the panel. */
bool instruction_list_set_device_options(const char *device, const char *const *options, uint8_t count);
/* GUI-thread phone-origin refresh. Copies into the same bounded update stream;
 * admission does not mean displayed. A different device context is ignored. */
bool instruction_list_refresh_device_options(const char *device, const char *const *options, uint8_t count);
/* Read-only indices for the retired pager view; selection has one owner. */
int instruction_list_selected_sessions(uint8_t device, uint8_t *indices, uint8_t capacity);
/* The displayed row retains its owner while the session pool changes. */
bool instruction_list_resolve_owner(const char *id, char *device, size_t device_len,
                                    char *bot, size_t bot_len);
const char *instruction_list_device_name_for_row(const char *id);

/* Any producer thread. Copies borrowed payload before returning.
 * true means admitted, never synchronized or displayed. */
bool instruction_list_submit_phone(const instruction_list_delivery_t *delivery);

#ifdef __cplusplus
}
#endif
#endif
