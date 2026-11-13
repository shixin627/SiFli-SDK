/* Copyright (c) 2026 Skaiwalk Technology
 * SPDX-License-Identifier: Apache-2.0
 */
#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <rtdevice.h>
#include "lvgl.h"
#include "communicate_parse_notify.h"
#include "communicate_parse.h"
#include "communicate_protocol.h"
#include "communicate_parse_skailink.h"
#include "instruction_list.h"
#include "lv_left_cards.h"
#include "lv_session_pager.h"
#include "app_mem.h"

extern void apply_pending_instruction_batch(void);
extern void instruction_list_set_text_filter(const char *text);
extern uint8_t return_total_list_count(void);
extern const char *instruction_list_export_id(uint8_t index);

static uint32_t test_now_ms, test_requested_id, test_requested_scope;
static unsigned test_request_count;
static bool test_transport_ready = true, test_transport_awake = true;
static bool test_change_scope_on_send, test_lose_updates_on_send;
static uint32_t test_clock(void) { return test_now_ms; }
static bool test_ready(void) { return test_transport_ready; }
static bool test_awake(void) { return test_transport_awake; }
static bool test_request(uint32_t id, uint32_t scope)
{
    test_request_count++;
    test_requested_id = id;
    test_requested_scope = scope;
    if (test_change_scope_on_send)
    {
        test_change_scope_on_send = false;
        instruction_list_scope_changed();
    }
    if (test_lose_updates_on_send) {
        char update[] = "{\"id\":\"tdd-send-race\",\"title\":\"During send\"}";
        unsigned i;
        char snapshot[128];
        instruction_list_delivery_t delivery = {KEY_INSTRUCTION_LIST_SYNC, snapshot, 0, SKAI_LINK_COMMAND_ID};
        test_lose_updates_on_send = false;
        rt_snprintf(snapshot, sizeof(snapshot),
            "{\"op\":\"snapshot\",\"id\":%lu,\"scope\":%lu,\"items\":[]}",
            (unsigned long)id, (unsigned long)scope);
        delivery.length = strlen(snapshot);
        instruction_list_submit_phone(&delivery); /* Admit the following suffix before overflowing it. */
        for (i = 0; i < 32; i++)
            resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS, (uint8_t *)update, strlen(update));
    }
    return true;
}

static void list_tdd_report(const char *format, ...)
{
    char line[160];
    va_list args;
    va_start(args, format);
    rt_vsnprintf(line, sizeof(line), format, args);
    va_end(args);
#ifdef BSP_USING_PC_SIMULATOR
    rt_kprintf("%s", line);
#else
    /* Production virtual console is not a proof artifact. Emit test outcomes
       on the same physical UART as the hardware self-test protocol. */
    rt_device_t uart = rt_device_find("uart1");
    if (uart != NULL)
        rt_device_write(uart, 0, line, strlen(line));
#endif
}

static bool list_has_id(const char *wanted)
{
    uint8_t i;
    for (i = 0; i < return_total_list_count(); i++)
    {
        const char *id = instruction_list_export_id(i);
        if (id != NULL && strcmp(id, wanted) == 0)
            return true;
    }
    return false;
}

#ifndef BSP_USING_PC_SIMULATOR
static void physical_voice_open_uses_prepared_frame(void);
static uint32_t live_previous_receipt;
static rt_tick_t live_deadline;
static void live_snapshot_check(lv_timer_t *timer)
{
    instruction_list_status_t status;
    apply_pending_instruction_batch();
    status = instruction_list_get_status();
    if (status.supports_resend && !status.needs_sync &&
        status.completed_request != live_previous_receipt)
    {
        rt_uint32_t total = 0, used = 0, peak = 0;
        rt_memory_info(&total, &used, &peak);
        list_tdd_report("LIST_TDD PASS physical_phone_snapshot_round_trip id=%lu\n",
                        (unsigned long)status.completed_request);
        list_tdd_report("LIST_TDD MEMORY system_free=%lu peak_used=%lu\n",
                        (unsigned long)(total - used), (unsigned long)peak);
        lv_timer_del(timer);
        physical_voice_open_uses_prepared_frame();
    }
    else if ((rt_tick_t)(rt_tick_get() - live_deadline) < (rt_tick_t)0x80000000u)
    {
        list_tdd_report("LIST_TDD FAIL physical_phone_snapshot_round_trip supported=%u pending=%u\n",
                        (unsigned)status.supports_resend, (unsigned)status.needs_sync);
        lv_timer_del(timer);
    }
}
#endif

static void invalid_full_list_requires_resync(void)
{
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[{\"id\":\"tdd-safe\",\"title\":\"Safe\"}]";
    char invalid[] = "[{\"id\":\"tdd-missing-title\"}]";
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    caps[1] = 1;
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
#ifndef BSP_USING_PC_SIMULATOR
    live_previous_receipt = instruction_list_get_status().completed_request;
#endif
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)invalid, (uint16_t)strlen(invalid));
    apply_pending_instruction_batch();
    if (!instruction_list_get_status().needs_sync)
    {
        list_tdd_report("LIST_TDD FAIL invalid_full_list_requires_resync\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS invalid_full_list_requires_resync\n");
#ifndef BSP_USING_PC_SIMULATOR
    /* A cold phone/watch reconnect was measured at 80 s. This is an
       integration-test deadline, independent of the production retry cadence. */
    live_deadline = rt_tick_get() + rt_tick_from_millisecond(180000);
    lv_timer_create(live_snapshot_check, 250, NULL);
#endif
}

static bool tree_has_title(lv_obj_t *root, const char *title, bool require_on_screen)
{
    uint32_t i;
    if (root != NULL && lv_obj_get_parent(root) == NULL) lv_obj_update_layout(root);
    if (root == NULL || lv_obj_has_flag(root, LV_OBJ_FLAG_HIDDEN)) return false;
    if (lv_obj_check_type(root, &lv_label_class) && (!require_on_screen || lv_obj_is_visible(root)))
    {
        const char *text = lv_label_get_text(root);
        if (text && strcmp(text, title) == 0) return true;
    }
    for (i = 0; i < lv_obj_get_child_cnt(root); i++)
        if (tree_has_title(lv_obj_get_child(root, i), title, require_on_screen)) return true;
    return false;
}

static bool visible_tree_has_title(lv_obj_t *root, const char *title)
{
    return tree_has_title(root, title, true);
}

static uint32_t visible_expected_receipt;

static bool rejection_saw_candidate, rejection_published, rejection_called;
static bool rejection_was_pending;
static uint32_t rejection_expected_request;
static bool reject_presentation(const char *focused_id)
{
    instruction_list_status_t status = instruction_list_get_status();
    rejection_called = true;
    (void)focused_id;
    rejection_saw_candidate = list_has_id("tdd-rejected-new");
    rejection_was_pending = status.needs_sync && status.active_request == rejection_expected_request;
    return false;
}
static void record_rejected_publication(void) { rejection_published = true; }
static void finish_rejected_presentation(bool committed) { (void)committed; }

#ifndef BSP_USING_PC_SIMULATOR
static void closing_voice_keeps_the_frame_model_until_hidden(void)
{
    extern void animate_open_ai_widget(void), close_ai_widget(void), instruction_list_close_ai_on_leave(void);
    extern bool instruction_list_is_visible(void);
    char seed[] = "[{\"id\":\"tdd-voice-base\",\"title\":\"Voice base\"}]";
    char update[] = "{\"id\":\"tdd-voice-session\",\"title\":\"Voice session\"}";
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)seed, strlen(seed));
    apply_pending_instruction_batch();
    animate_open_ai_widget();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS, (uint8_t *)update, strlen(update));
    apply_pending_instruction_batch();
    bool setup = instruction_list_is_visible() && list_has_id("tdd-voice-session");
    close_ai_widget();
    bool retained = !instruction_list_is_visible() || list_has_id("tdd-voice-session");
    /* Synchronous cleanup: the opening morph must never reach microphone start. */
    instruction_list_close_ai_on_leave();
    instruction_list_close();
    if (!setup || !retained || !list_has_id("tdd-voice-base") || list_has_id("tdd-voice-session")) {
        list_tdd_report("LIST_TDD FAIL closing_voice_keeps_the_frame_model_until_hidden\n"); return;
    }
    list_tdd_report("LIST_TDD PASS closing_voice_keeps_the_frame_model_until_hidden\n");
}

static void physical_voice_open_uses_prepared_frame(void)
{
    extern void instruction_list_hide_now(void), instruction_list_release_ui(void);
    extern void animate_open_ai_widget(void), instruction_list_close_ai_on_leave(void);
    extern bool instruction_list_is_visible(void);
    extern const char *instruction_list_export_title(uint8_t index);
    instruction_list_presentation_t presentation = {reject_presentation, record_rejected_publication, finish_rejected_presentation};
    extern void instruction_list_layout_capture_focus(char *id, size_t capacity);
    char focus[64];
    const char *title = NULL;
    bool opened, called, title_prepared;
    instruction_list_hide_now();
    instruction_list_release_ui();
    rejection_called = rejection_published = false;
    instruction_list_bind_presentation(&presentation);
    animate_open_ai_widget();
    opened = instruction_list_is_visible();
    called = rejection_called;
    instruction_list_bind_presentation(NULL);
    /* Cancel the grow animation synchronously, before its ready callback can
       start the microphone. This test only exercises the first prepared frame. */
    instruction_list_close_ai_on_leave();
    instruction_list_close();
    if (opened || !called || rejection_published) {
        list_tdd_report("LIST_TDD FAIL physical_voice_open_uses_prepared_frame\n"); return;
    }
    animate_open_ai_widget();
    opened = instruction_list_is_visible();
    instruction_list_layout_capture_focus(focus, sizeof(focus));
    for (uint8_t i = 0; focus[0] && i < return_total_list_count(); i++)
    {
        const char *id = instruction_list_export_id(i);
        if (id && strcmp(id, focus) == 0) {
            title = instruction_list_export_title(i);
            break;
        }
    }
    /* The first prepared frame is still sliding in. Check the mounted label
       for the actual focus selected after projecting the live phone list. */
    title_prepared = return_total_list_count() == 0 ||
        (title && tree_has_title(lv_layer_top(), title, false));
    instruction_list_close_ai_on_leave();
    instruction_list_close();
    if (!opened || !title_prepared) {
        list_tdd_report("LIST_TDD FAIL physical_voice_open_did_not_rebuild opened=%u prepared=%u\n",
                        (unsigned)opened, (unsigned)title_prepared);
        return;
    }
    list_tdd_report("LIST_TDD PASS physical_voice_open_uses_prepared_frame\n");
    closing_voice_keeps_the_frame_model_until_hidden();
}
#endif

static bool change_scope_before_publication(const char *focused_id)
{
    (void)focused_id;
    instruction_list_scope_changed();
    return true;
}

#ifndef BSP_USING_PC_SIMULATOR
static void render_pressure_check(lv_timer_t *timer)
{
    extern struct rt_memheap app_image_psram_memheap;
    extern void instruction_list_hide_now(void);
    void *held[192];
    const unsigned sizes[] = {16u * 1024u, 1024u, 64u};
    unsigned n = 0, i;
    rt_uint32_t total, before, after, peak;
    bool retained;
    char replacement[] = "[{\"id\":\"tdd-render-new\",\"title\":\"Render new\",\"sub\":\"New preview\",\"cat\":\"/\"}]";
    lv_timer_del(timer);
    if (!visible_tree_has_title(lv_layer_top(), "Render old") || left_cards_busy())
    {
        instruction_list_hide_now();
        list_tdd_report("LIST_TDD FAIL render_pressure_setup\n");
        return;
    }
    /* Native PSRAM pressure after parser/ingress initialization. No fake
       renderer: the real LVGL presentation must retain the interactive frame. */
    for (i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++)
        while (n < sizeof(held) / sizeof(held[0]))
        {
            void *memory = rt_memheap_alloc(&app_image_psram_memheap, sizes[i]);
            if (!memory) break;
            held[n++] = memory;
        }
    rt_memory_info(&total, &before, &peak);
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)replacement, (uint16_t)strlen(replacement));
    apply_pending_instruction_batch();
    rt_memory_info(&total, &after, &peak);
    retained = list_has_id("tdd-render-old") && !list_has_id("tdd-render-new") &&
               instruction_list_get_status().needs_sync;
    while (n) rt_memheap_free(held[--n]);
    retained = retained && visible_tree_has_title(lv_layer_top(), "Render old") &&
               visible_tree_has_title(lv_layer_top(), "Old preview") && after <= before + 4096u;
    if (!retained)
    {
        instruction_list_hide_now();
        list_tdd_report("LIST_TDD FAIL render_psram_exhaustion_keeps_visible_frame delta=%ld\n", (long)after - (long)before);
        return;
    }
    /* A legacy phone can recover by sending its next ordinary complete list. */
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)replacement, (uint16_t)strlen(replacement));
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-render-new") || instruction_list_get_status().needs_sync ||
        !visible_tree_has_title(lv_layer_top(), "Render new"))
    {
        instruction_list_hide_now();
        list_tdd_report("LIST_TDD FAIL render_pressure_recovery_did_not_resume\n");
        return;
    }
    instruction_list_hide_now();
    list_tdd_report("LIST_TDD PASS render_psram_exhaustion_keeps_visible_frame\n");
    invalid_full_list_requires_resync();
}

static void render_psram_exhaustion_keeps_visible_frame(void)
{
    extern void instruction_list_reveal_drag_begin_ex(bool from_left, char filter);
    extern void instruction_list_reveal_drag_update(lv_coord_t dx);
    extern void instruction_list_reveal_drag_end(lv_coord_t dx, lv_coord_t vx);
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[{\"id\":\"tdd-render-old\",\"title\":\"Render old\",\"sub\":\"Old preview\",\"cat\":\"/\"}]";
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    instruction_list_reveal_drag_begin_ex(true, '/');
    instruction_list_reveal_drag_update(LV_HOR_RES);
    instruction_list_reveal_drag_end(LV_HOR_RES, 0);
    lv_timer_create(render_pressure_check, 1200, NULL);
}
#endif

static lv_obj_t *open_probe_host;
static bool open_probe_exposed_candidate;
static lv_obj_t *find_label_any(lv_obj_t *root, const char *text)
{
    uint32_t i;
    if (lv_obj_check_type(root, &lv_label_class) && strcmp(lv_label_get_text(root), text) == 0) return root;
    for (i = 0; i < lv_obj_get_child_cnt(root); i++)
    {
        lv_obj_t *found = find_label_any(lv_obj_get_child(root, i), text);
        if (found) return found;
    }
    return NULL;
}
static void observe_open_construction(lv_event_t *event)
{
    extern bool instruction_list_is_visible(void);
    lv_obj_t *child = lv_event_get_param(event);
    if (lv_obj_has_flag(open_probe_host, LV_OBJ_FLAG_HIDDEN) && instruction_list_is_visible())
        open_probe_exposed_candidate = true;
    if (child) lv_obj_add_event_cb(child, observe_open_construction, LV_EVENT_CHILD_CREATED, NULL);
}
static void remove_open_probe(lv_obj_t *root)
{
    uint32_t i;
    lv_obj_remove_event_cb(root, observe_open_construction);
    for (i = 0; i < lv_obj_get_child_cnt(root); i++) remove_open_probe(lv_obj_get_child(root, i));
}

static void device_return_keeps_the_current_connection_state(void)
{
    const char *options[] = {"tdd-device-link-only"};
    if (!instruction_list_set_device_options("tdd-device-link", options, 1)) {
        list_tdd_report("LIST_TDD FAIL device_link_context_setup\n"); return;
    }
    instruction_list_link_changed(false);
    apply_pending_instruction_batch();
    instruction_list_close();
    if (list_has_id("tdd-relay") || !list_has_id("tdd-offline") || list_has_id("tdd-device-link-only")) {
        list_tdd_report("LIST_TDD FAIL device_return_keeps_the_current_connection_state\n"); return;
    }
    instruction_list_link_changed(true);
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-relay") || !list_has_id("tdd-offline")) {
        list_tdd_report("LIST_TDD FAIL device_return_keeps_the_reconnect_source\n"); return;
    }
    list_tdd_report("LIST_TDD PASS device_return_keeps_the_current_connection_state\n");
#ifndef BSP_USING_PC_SIMULATOR
    render_psram_exhaustion_keeps_visible_frame();
#else
    invalid_full_list_requires_resync();
#endif
}

static void connection_changes_keep_the_old_frame_on_rejection(void)
{
    instruction_list_presentation_t presentation = {reject_presentation, record_rejected_publication, finish_rejected_presentation};
    char seed[] = "[{\"id\":\"tdd-relay\",\"title\":\"Phone relay\"},{\"id\":\"tdd-offline\",\"title\":\"Offline\",\"openApp\":\"Settings\"}]";
    instruction_list_link_changed(true);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)seed, strlen(seed));
    apply_pending_instruction_batch();
    rejection_called = rejection_published = false;
    instruction_list_bind_presentation(&presentation);
    instruction_list_link_changed(false);
    apply_pending_instruction_batch();
    instruction_list_bind_presentation(NULL);
    if (!rejection_called || rejection_published || !list_has_id("tdd-relay") || !list_has_id("tdd-offline")) {
        list_tdd_report("LIST_TDD FAIL connection_changes_keep_the_old_frame_on_rejection\n"); return;
    }
    apply_pending_instruction_batch();
    if (list_has_id("tdd-relay") || !list_has_id("tdd-offline")) {
        list_tdd_report("LIST_TDD FAIL rejected_disconnect_did_not_resume\n"); return;
    }
    instruction_list_link_changed(true);
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-relay") || !list_has_id("tdd-offline")) {
        list_tdd_report("LIST_TDD FAIL reconnect_did_not_restore_the_list\n"); return;
    }
    list_tdd_report("LIST_TDD PASS connection_changes_keep_the_old_frame_on_rejection\n");
    device_return_keeps_the_current_connection_state();
}

static void single_capacity_row_check(lv_timer_t *timer)
{
    extern void instruction_list_hide_now(void);
    if (timer->user_data == NULL) {
        timer->user_data = (void *)1;
        left_cards_scroll_to(1, false);
        lv_timer_set_period(timer, 180);
        return;
    }
    lv_timer_del(timer);
    if (left_cards_current() != 1 || !visible_tree_has_title(lv_layer_top(), "Capacity 0") ||
        !visible_tree_has_title(lv_layer_top(), "Preview 0")) {
        instruction_list_hide_now();
        list_tdd_report("LIST_TDD FAIL single_update_lost_the_surviving_preview\n"); return;
    }
    instruction_list_hide_now();
    list_tdd_report("LIST_TDD PASS single_updates_keep_surviving_row_state\n");
    connection_changes_keep_the_old_frame_on_rejection();
}
static void single_updates_keep_surviving_row_state(void)
{
    extern int instruction_switch_pos(const char *id);
    char removed[] = "tdd-capacity-6";
    char added[] = "{\"id\":\"Added\",\"title\":\"Added\",\"sub\":\"Added preview\",\"sw\":0,\"cat\":\"/\"}";
    resolve_Notify_command(KEY_DISMISS_SKAI_INSTRUCTION, (uint8_t *)removed, strlen(removed));
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS, (uint8_t *)added, strlen(added));
    apply_pending_instruction_batch();
    if (return_total_list_count() != 30 || !list_has_id("Added") || list_has_id(removed) ||
        instruction_switch_pos("tdd-capacity-0") != 1) {
        list_tdd_report("LIST_TDD FAIL single_updates_keep_surviving_row_state\n"); return;
    }
    instruction_list_open('/');
    lv_timer_create(single_capacity_row_check, 1200, NULL);
}

static char *capacity_payload;
static unsigned capacity_step, capacity_round;
static void capacity_build(bool mixed)
{
    size_t used = 1;
    unsigned i, count = 0;
    capacity_payload[0] = '[';
    if (mixed) {
        used += rt_snprintf(capacity_payload + used, 4096 - used,
            "{\"id\":\"tdd-capacity-new-%u\",\"title\":\"Capacity new\",\"cat\":\"/\",\"sw\":0}", capacity_round);
        count++;
    }
    for (i = 0; i < 30; i++) {
        if (mixed && i == 5) continue;
        used += rt_snprintf(capacity_payload + used, 4096 - used,
            "%s{\"id\":\"tdd-capacity-%u\",\"title\":\"Capacity %u\",\"sub\":\"Preview %u\",\"cat\":\"/\",\"sw\":0}",
            count++ ? "," : "", i, i, i);
    }
    capacity_payload[used++] = ']';
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)capacity_payload, used);
    apply_pending_instruction_batch();
}
static void capacity_dump_labels(lv_obj_t *root)
{
    if (lv_obj_has_flag(root, LV_OBJ_FLAG_HIDDEN)) return;
    if (lv_obj_check_type(root, &lv_label_class)) {
        const char *text = lv_label_get_text(root);
        if (text && (strncmp(text, "Capacity", 8) == 0 || strncmp(text, "Preview", 7) == 0))
            list_tdd_report("LIST_TDD FRAME %s\n", text);
    }
    for (uint32_t i = 0; i < lv_obj_get_child_cnt(root); i++) capacity_dump_labels(lv_obj_get_child(root, i));
}

static void full_capacity_check(lv_timer_t *timer)
{
    extern void instruction_list_hide_now(void);
    extern int instruction_switch_pos(const char *id);
    static const uint8_t positions[] = {29, 0, 15, 29};
    static const char *titles[] = {"Capacity 29", "Capacity 0", "Capacity 15", "Capacity 29"};
    static const char *previews[] = {"Preview 29", "Preview 0", "Preview 15", "Preview 29"};
    if (return_total_list_count() != 30 || instruction_list_get_status().needs_sync) goto fail;
    if (capacity_step < 4 && (left_cards_current() != positions[capacity_step] ||
        !visible_tree_has_title(lv_layer_top(), titles[capacity_step]) ||
        !visible_tree_has_title(lv_layer_top(), previews[capacity_step]))) goto fail;
    if (capacity_step == 0) left_cards_scroll_to(0, false);
    else if (capacity_step == 1) {
        lv_obj_t *label = find_label_any(lv_layer_top(), "Capacity 0");
        if (!label) goto fail;
        lv_event_send(lv_obj_get_parent(label), LV_EVENT_CLICKED, NULL);
        if (instruction_switch_pos("tdd-capacity-0") != 1) goto fail;
        left_cards_scroll_to(15, false);
    }
    else if (capacity_step == 2) left_cards_scroll_to(29, false);
    else if (capacity_step == 3) {
        capacity_build(true); /* Remove row 5 and insert a new row before the retained rows. */
        if (instruction_switch_pos("tdd-capacity-0") != 1) goto fail;
    }
    else if (capacity_step == 4) {
        if (instruction_switch_pos("tdd-capacity-0") != 1 ||
            left_cards_current() != 29 || !visible_tree_has_title(lv_layer_top(), "Capacity 29")) goto fail;
        if (capacity_round < 4) {
            capacity_round++;
            capacity_build(true);
            return;
        }
        left_cards_scroll_to(0, false);
    }
    else {
        instruction_list_status_t status = instruction_list_get_status();
        static const int8_t expected_rows[] = {-1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14,
            15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29};
        unsigned index = capacity_step - 5;
        char title[32], preview[32];
        if (index >= 30 || left_cards_current() != index) goto fail;
        if (expected_rows[index] < 0) rt_strncpy(title, "Capacity new", sizeof(title));
        else {
            rt_snprintf(title, sizeof(title), "Capacity %d", expected_rows[index]);
            rt_snprintf(preview, sizeof(preview), "Preview %d", expected_rows[index]);
            if (!visible_tree_has_title(lv_layer_top(), preview)) goto fail;
        }
        if (!visible_tree_has_title(lv_layer_top(), title) || status.render_reserved_bytes != 256u * 1024u ||
            status.render_peak_bytes > 256u * 1024u) goto fail;
        if (index < 29) {
            left_cards_scroll_to(index + 1, false);
            capacity_step++;
            return;
        }
        list_tdd_report("LIST_TDD MEMORY full_rows=30 render_active=%lu render_peak=%lu\n",
            (unsigned long)status.render_reserved_bytes, (unsigned long)status.render_peak_bytes);
        lv_timer_del(timer);
        instruction_list_hide_now();
        app_cache_free(capacity_payload); capacity_payload = NULL;
        list_tdd_report("LIST_TDD PASS full_capacity_updates_keep_state_and_bound_memory\n");
        single_updates_keep_surviving_row_state();
        return;
    }
    capacity_step++;
    lv_timer_set_period(timer, 180);
    return;
fail:
    lv_timer_del(timer);
    app_cache_free(capacity_payload); capacity_payload = NULL;
    list_tdd_report("LIST_TDD FAIL full_capacity_updates_keep_state_and_bound_memory step=%u round=%u switch=%d cur=%u title18=%u sub18=%u\n",
        capacity_step, capacity_round, instruction_switch_pos("tdd-capacity-0"), (unsigned)left_cards_current(),
        (unsigned)visible_tree_has_title(lv_layer_top(), "Capacity 18"),
        (unsigned)visible_tree_has_title(lv_layer_top(), "Preview 18"));
    extern const char *instruction_list_export_title(uint8_t index);
    list_tdd_report("LIST_TDD FRAME model18=%s peak=%lu\n", instruction_list_export_title(18),
        (unsigned long)instruction_list_get_status().render_peak_bytes);
    capacity_dump_labels(lv_layer_top());
    instruction_list_hide_now();
}
static void full_capacity_updates_keep_state_and_bound_memory(void)
{
    int count = 0;
    const device_sessions_t *pool = session_list_pool(&count);
    char devices[SESSION_DEVICE_MAX][SESSION_ID_LEN], empty[128];
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    if (count > SESSION_DEVICE_MAX) { list_tdd_report("LIST_TDD FAIL full_capacity_pool_setup\n"); return; }
    for (int i = 0; i < count; i++) rt_strncpy(devices[i], pool[i].id, SESSION_ID_LEN);
    for (int i = 0; i < count; i++) {
        rt_snprintf(empty, sizeof(empty), "{\"device\":\"%s\",\"sessions\":[]}", devices[i]);
        skai_sessions_on_conv_list((const uint8_t *)empty, strlen(empty));
        session_pager_apply_pending();
    }
    instruction_list_submit_phone(&capability); apply_pending_instruction_batch();
    capacity_payload = app_cache_alloc(4096, IMAGE_CACHE_PSRAM);
    if (!capacity_payload) { list_tdd_report("LIST_TDD FAIL full_capacity_payload_setup\n"); return; }
    capacity_step = capacity_round = 0;
    capacity_build(false);
    instruction_list_open('/');
    lv_timer_create(full_capacity_check, 1200, NULL);
}

static void destroyed_list_releases_its_frame(void)
{
    extern rt_int32_t instruction_list_deinit(void);
    extern lv_obj_t *lv_instruction_list_layout_create(lv_obj_t *parent);
    const char *options[] = {"tdd-recreated-device-only"};
    extern void instruction_list_skaibar_dismiss_notify_only(void);
    if (!instruction_list_set_device_options("tdd-recreated-device", options, 1)) {
        list_tdd_report("LIST_TDD FAIL recreated_device_context_setup\n"); return;
    }
    instruction_list_status_t before = instruction_list_get_status();
    if (!before.render_reserved_bytes) {
        list_tdd_report("LIST_TDD FAIL frame_lifecycle_setup\n"); return;
    }
    instruction_list_deinit();
    if (instruction_list_get_status().render_reserved_bytes != 0) {
        list_tdd_report("LIST_TDD FAIL destroyed_list_releases_its_frame\n"); return;
    }
    if (!lv_instruction_list_layout_create(lv_scr_act())) {
        list_tdd_report("LIST_TDD FAIL frame_recreate_setup\n"); return;
    }
    list_tdd_report("LIST_TDD PASS destroyed_list_releases_its_frame\n");
    instruction_list_skaibar_dismiss_notify_only();
    if (!list_has_id("tdd-open-current") || list_has_id("tdd-recreated-device-only")) {
        list_tdd_report("LIST_TDD FAIL recreated_device_leave_restores_watch_context\n"); return;
    }
    list_tdd_report("LIST_TDD PASS recreated_device_leave_restores_watch_context\n");
    full_capacity_updates_keep_state_and_bound_memory();
}

static void invalid_version_rejects_the_complete_list(void);
static void escaped_zero_cannot_truncate_an_identity(void)
{
    char seed[] = "[{\"id\":\"tdd-zero-safe\",\"title\":\"Safe\"}]";
    char invalid[] = "[{\"id\":\"tdd-null\\u0000suffix\",\"title\":\"Wrong\"}]";
    char literal[] = "[{\"id\":\"tdd-literal\\\\u0000\",\"title\":\"Literal\"}]";
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)seed, strlen(seed));
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)invalid, strlen(invalid));
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-zero-safe") || list_has_id("tdd-null") || !instruction_list_get_status().needs_sync) {
        list_tdd_report("LIST_TDD FAIL escaped_zero_cannot_truncate_an_identity\n"); return;
    }
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)literal, strlen(literal));
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-literal\\u0000") || instruction_list_get_status().needs_sync) {
        list_tdd_report("LIST_TDD FAIL literal_backslash_zero_must_remain_valid\n"); return;
    }
    list_tdd_report("LIST_TDD PASS escaped_zero_cannot_truncate_an_identity\n");
    invalid_version_rejects_the_complete_list();
}

static void invalid_ingress_packet_requires_recovery(void)
{
    char seed[] = "[{\"id\":\"tdd-wire-safe\",\"title\":\"Safe\"}]";
    char later[] = "{\"id\":\"tdd-wire-later\",\"title\":\"Later\"}";
    static const char embedded_zero[] = "{\"id\":\"tdd-truncated\",\"title\":\"Wrong\"}\0suffix";
    char *oversized = app_cache_alloc(L2_REASSEMBLY_MAX_BYTES + 1u, IMAGE_CACHE_PSRAM);
    struct { const void *payload; uint16_t length; } cases[4] = {
        {embedded_zero, sizeof(embedded_zero) - 1}, {"", 0}, {NULL, 1},
        {oversized, L2_REASSEMBLY_MAX_BYTES + 1u}
    };
    unsigned i;
    if (!oversized) { list_tdd_report("LIST_TDD FAIL invalid_ingress_setup\n"); return; }
    memset(oversized, ' ', L2_REASSEMBLY_MAX_BYTES + 1u);
    for (i = 0; i < 4; i++) {
        instruction_list_delivery_t delivery = {KEY_SKAI_CREATION_INSTRUCTIONS, cases[i].payload, cases[i].length, 0};
        resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)seed, strlen(seed));
        apply_pending_instruction_batch();
        bool admitted = instruction_list_submit_phone(&delivery);
        resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS, (uint8_t *)later, strlen(later));
        apply_pending_instruction_batch();
        if (admitted || !instruction_list_get_status().needs_sync ||
            !list_has_id("tdd-wire-safe") || list_has_id("tdd-truncated") || list_has_id("tdd-wire-later")) {
            app_cache_free(oversized);
            list_tdd_report("LIST_TDD FAIL invalid_ingress_packet_requires_recovery case=%u\n", i); return;
        }
    }
    app_cache_free(oversized);
    list_tdd_report("LIST_TDD PASS invalid_ingress_packet_requires_recovery\n");
    escaped_zero_cannot_truncate_an_identity();
}

static void invalid_increment_requires_a_new_snapshot(void)
{
    char seed[] = "[{\"id\":\"tdd-increment-safe\",\"title\":\"Safe\"}]";
    char invalid[] = "{\"id\":\"tdd-increment-broken\"}";
    char later[] = "{\"id\":\"tdd-after-broken\",\"title\":\"Later\"}";
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)seed, strlen(seed));
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS, (uint8_t *)invalid, strlen(invalid));
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS, (uint8_t *)later, strlen(later));
    apply_pending_instruction_batch();
    if (!instruction_list_get_status().needs_sync || !list_has_id("tdd-increment-safe") || list_has_id("tdd-after-broken")) {
        list_tdd_report("LIST_TDD FAIL invalid_increment_requires_a_new_snapshot\n"); return;
    }
    list_tdd_report("LIST_TDD PASS invalid_increment_requires_a_new_snapshot\n");
    invalid_ingress_packet_requires_recovery();
}

static void search_freezes_device_refreshes_in_delivery_order(void)
{
    const char *keep[] = {"Keep"}, *later[] = {"Later"}, *fresh[] = {"Fresh"};
    char batch[] = "[{\"id\":\"Latest\",\"title\":\"Latest\"}]";
    for (unsigned i = 0; i < 2; i++) {
        if (!instruction_list_set_device_options("tdd-frozen-device", keep, 1)) goto fail;
        instruction_list_set_query("Keep");
        if (i == 1) resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)batch, strlen(batch));
        if (!instruction_list_refresh_device_options("tdd-frozen-device", later, 1)) goto fail;
        if (i == 0) resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)batch, strlen(batch));
        apply_pending_instruction_batch();
        if (!list_has_id("Keep") || list_has_id("Later") || list_has_id("Latest")) goto fail;
        instruction_list_set_query("");
        const char *expected = i == 0 ? "Latest" : "Later";
        if (!list_has_id(expected) || list_has_id("Keep")) goto fail;
    }
    /* A -> B -> A must not revive an update admitted during the first visit. */
    instruction_list_set_device_options("tdd-frozen-device", keep, 1);
    instruction_list_set_query("Keep");
    instruction_list_refresh_device_options("tdd-frozen-device", later, 1);
    instruction_list_set_device_options("tdd-other-device", fresh, 1);
    instruction_list_set_device_options("tdd-frozen-device", fresh, 1);
    instruction_list_set_query("");
    if (!list_has_id("Fresh") || list_has_id("Later")) goto fail;
    instruction_list_close();
    instruction_list_refresh_device_options("tdd-frozen-device", later, 1);
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-bookmark") || list_has_id("Later")) goto fail;
    list_tdd_report("LIST_TDD PASS search_freezes_device_refreshes_in_delivery_order\n");
    invalid_increment_requires_a_new_snapshot();
    return;
fail:
    instruction_list_set_query("");
    instruction_list_close();
    list_tdd_report("LIST_TDD FAIL search_freezes_device_refreshes_in_delivery_order\n");
}

static void device_refresh_keeps_the_same_sync_scope(void)
{
    const char *first[] = {"tdd-device-first"}, *latest[] = {"tdd-device-latest"};
    if (!instruction_list_set_device_options("tdd-same-device", first, 1)) {
        list_tdd_report("LIST_TDD FAIL device_scope_setup\n"); return;
    }
    uint32_t scope = instruction_list_get_status().scope;
    if (!instruction_list_set_device_options("tdd-same-device", latest, 1) ||
        instruction_list_get_status().scope != scope || !list_has_id("tdd-device-latest") ||
        list_has_id("tdd-device-first")) {
        list_tdd_report("LIST_TDD FAIL device_refresh_keeps_the_same_sync_scope\n"); return;
    }
    instruction_list_close();
    list_tdd_report("LIST_TDD PASS device_refresh_keeps_the_same_sync_scope\n");
    search_freezes_device_refreshes_in_delivery_order();
}

static void device_without_options_keeps_the_existing_list(void)
{
    extern bool instruction_list_single_device_active(void);
    if (!instruction_list_set_device_options("tdd-no-options", NULL, 0) ||
        !instruction_list_single_device_active() || !list_has_id("tdd-bookmark")) {
        list_tdd_report("LIST_TDD FAIL device_without_options_keeps_the_existing_list\n"); return;
    }
    instruction_list_close();
    if (instruction_list_single_device_active() || !list_has_id("tdd-bookmark")) {
        list_tdd_report("LIST_TDD FAIL empty_device_close_did_not_restore\n"); return;
    }
    list_tdd_report("LIST_TDD PASS device_without_options_keeps_the_existing_list\n");
    device_refresh_keeps_the_same_sync_scope();
}

static bool visible_drawer_control(lv_obj_t *root)
{
    LV_IMG_DECLARE(erth);
    if (!root || lv_obj_has_flag(root, LV_OBJ_FLAG_HIDDEN)) return false;
    if (lv_obj_check_type(root, &lv_img_class) && lv_img_get_src(root) == &erth && lv_obj_is_visible(root)) return true;
    for (uint32_t i = 0; i < lv_obj_get_child_cnt(root); i++)
        if (visible_drawer_control(lv_obj_get_child(root, i))) return true;
    return false;
}

static void drawer_retry_check(lv_timer_t *timer)
{
    extern bool instruction_list_is_visible(void), instruction_list_single_device_active(void);
    lv_timer_del(timer);
    if (!instruction_list_is_visible() || !list_has_id("Drawer") || !visible_tree_has_title(lv_layer_top(), "Drawer") || !visible_drawer_control(lv_layer_top())) {
        instruction_list_close();
        list_tdd_report("LIST_TDD FAIL rejected_drawer_intent_did_not_resume\n"); return;
    }
    instruction_list_close();
    if (instruction_list_is_visible() || instruction_list_single_device_active() ||
        visible_drawer_control(lv_layer_top()) || !list_has_id("tdd-bookmark")) {
        list_tdd_report("LIST_TDD FAIL drawer_close_did_not_finish_the_context\n"); return;
    }
    list_tdd_report("LIST_TDD PASS rejected_drawer_stays_closed_and_retries\n");
    device_without_options_keeps_the_existing_list();
}
static void rejected_drawer_stays_closed_and_retries(void)
{
    extern void instruction_list_drawer_slide_in(void), instruction_list_release_ui(void);
    extern bool instruction_list_is_visible(void);
    instruction_list_presentation_t presentation = {reject_presentation, record_rejected_publication, finish_rejected_presentation};
    const char *options[] = {"Drawer"};
    if (!instruction_list_set_device_options("tdd-drawer-device", options, 1)) {
        list_tdd_report("LIST_TDD FAIL drawer_context_setup\n"); return;
    }
    instruction_list_release_ui();
    rejection_called = rejection_published = false;
    instruction_list_bind_presentation(&presentation);
    instruction_list_drawer_slide_in();
    bool premature = instruction_list_is_visible();
    instruction_list_bind_presentation(NULL);
    if (premature || !rejection_called || rejection_published) {
        instruction_list_close();
        list_tdd_report("LIST_TDD FAIL rejected_drawer_stays_closed_and_retries\n"); return;
    }
    apply_pending_instruction_batch();
    lv_timer_create(drawer_retry_check, 700, NULL);
}

static void leaving_device_cancels_its_pending_change(void)
{
    extern void instruction_list_bar_device_dismiss(void), instruction_list_skaibar_dismiss_notify_only(void);
    extern void instruction_list_close_ai_on_leave(void);
    extern bool instruction_list_single_device_active(void), instruction_list_is_visible(void);
    void (*leave[])(void) = {instruction_list_bar_device_dismiss, instruction_list_skaibar_dismiss_notify_only, instruction_list_close_ai_on_leave};
    const char *options[] = {"tdd-leave-pending"};
    instruction_list_presentation_t presentation = {reject_presentation, record_rejected_publication, finish_rejected_presentation};
    for (unsigned i = 0; i < 3; i++) {
        instruction_list_bind_presentation(&presentation);
        bool accepted = instruction_list_set_device_options("tdd-leave-target", options, 1);
        instruction_list_open(0);
        instruction_list_bind_presentation(NULL);
        if (accepted) { list_tdd_report("LIST_TDD FAIL pending_leave_setup\n"); return; }
        leave[i]();
        apply_pending_instruction_batch();
        if (instruction_list_is_visible() || instruction_list_single_device_active() ||
            list_has_id("tdd-leave-pending") || !list_has_id("tdd-bookmark")) {
            list_tdd_report("LIST_TDD FAIL leaving_device_cancels_its_pending_change case=%u\n", i); return;
        }
    }
    list_tdd_report("LIST_TDD PASS leaving_device_cancels_its_pending_change\n");
    rejected_drawer_stays_closed_and_retries();
}

static void rejected_device_change_keeps_the_old_context(void)
{
    extern bool instruction_list_single_device_active(void);
    const char *options[] = {"tdd-rejected-device"};
    instruction_list_presentation_t presentation = {reject_presentation, record_rejected_publication, finish_rejected_presentation};
    rejection_published = false;
    instruction_list_bind_presentation(&presentation);
    bool accepted = instruction_list_set_device_options("tdd-rejected-target", options, 1);
    instruction_list_bind_presentation(NULL);
    if (accepted || rejection_published || instruction_list_single_device_active() ||
        !list_has_id("tdd-bookmark") || list_has_id("tdd-rejected-device") ||
        !visible_tree_has_title(lv_layer_top(), "Original")) {
        list_tdd_report("LIST_TDD FAIL rejected_device_change_keeps_the_old_context\n"); return;
    }
    apply_pending_instruction_batch();
    if (!instruction_list_single_device_active() || !list_has_id("tdd-rejected-device") ||
        !visible_tree_has_title(lv_layer_top(), "tdd-rejected-device")) {
        list_tdd_report("LIST_TDD FAIL rejected_device_intent_did_not_resume\n"); return;
    }
    instruction_list_close();
    if (!list_has_id("tdd-bookmark") || instruction_list_single_device_active()) {
        list_tdd_report("LIST_TDD FAIL rejected_device_bookmark_was_corrupted\n"); return;
    }
    list_tdd_report("LIST_TDD PASS rejected_device_change_keeps_the_old_context\n");
    leaving_device_cancels_its_pending_change();
}

static void device_return_check(lv_timer_t *timer)
{
    extern void instruction_list_hide_now(void);
    lv_timer_del(timer);
    if (!visible_tree_has_title(lv_layer_top(), "Original") ||
        !visible_tree_has_title(lv_layer_top(), "Original preview") ||
        visible_tree_has_title(lv_layer_top(), "Foreign preview")) {
        list_tdd_report("LIST_TDD FAIL device_close_restores_the_complete_context\n"); return;
    }
    list_tdd_report("LIST_TDD PASS device_close_restores_the_complete_context\n");
    rejected_device_change_keeps_the_old_context();
}
static void device_close_restores_the_complete_context(void)
{
    extern int instruction_switch_pos(const char *id);
    char seed[] = "[{\"id\":\"tdd-bookmark\",\"title\":\"Original\",\"cat\":\"/\",\"sub\":\"Original preview\",\"sw\":1}]";
    char update[] = "{\"id\":\"tdd-bookmark\",\"title\":\"Remote\",\"sub\":\"Foreign preview\"}";
    const char *options[] = {"tdd-bookmark"};
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)seed, strlen(seed));
    apply_pending_instruction_batch();
    if (!instruction_list_set_device_options("tdd-remote", options, 1)) {
        list_tdd_report("LIST_TDD FAIL device_context_setup\n"); return;
    }
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS, (uint8_t *)update, strlen(update));
    apply_pending_instruction_batch();
    instruction_list_close();
    if (!list_has_id("tdd-bookmark") || instruction_switch_pos("tdd-bookmark") != 1) {
        list_tdd_report("LIST_TDD FAIL device_close_restores_the_complete_context\n"); return;
    }
    instruction_list_open('/');
    lv_timer_create(device_return_check, 1200, NULL);
}

static void rejected_open_retry_check(lv_timer_t *timer)
{
    extern void instruction_list_hide_now(void);
    lv_timer_del(timer);
    if (!visible_tree_has_title(lv_layer_top(), "Open current") || !list_has_id("tdd-open-current"))
    {
        instruction_list_hide_now();
        list_tdd_report("LIST_TDD FAIL rejected_open_intent_did_not_resume\n");
        return;
    }
    instruction_list_hide_now();
    list_tdd_report("LIST_TDD PASS rejected_open_stays_closed_and_retries\n");
    destroyed_list_releases_its_frame();
}

static void rejected_open_stays_closed_and_retries(void)
{
    extern void instruction_list_hide_now(void);
    extern bool instruction_list_is_visible(void);
    instruction_list_presentation_t presentation = {reject_presentation, record_rejected_publication, finish_rejected_presentation};
    char seed[] = "[{\"id\":\"tdd-open-current\",\"title\":\"Open current\",\"cat\":\"/\"}]";
    instruction_list_hide_now();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    rejection_published = false;
    instruction_list_bind_presentation(&presentation);
    instruction_list_open('/'); /* Isolate action rows from the simulator's session pool. */
    instruction_list_bind_presentation(NULL);
    if (instruction_list_is_visible() || rejection_published)
    {
        instruction_list_hide_now();
        list_tdd_report("LIST_TDD FAIL rejected_open_stays_closed_and_retries\n");
        return;
    }
    open_probe_host = find_label_any(lv_layer_top(), "Retained old");
    if (!open_probe_host) { list_tdd_report("LIST_TDD FAIL open_visibility_probe_setup\n"); return; }
    while (lv_obj_get_parent(open_probe_host) &&
           lv_obj_get_parent(lv_obj_get_parent(open_probe_host)) != lv_layer_top())
        open_probe_host = lv_obj_get_parent(open_probe_host);
    open_probe_exposed_candidate = false;
    lv_obj_add_event_cb(open_probe_host, observe_open_construction, LV_EVENT_CHILD_CREATED, NULL);
    apply_pending_instruction_batch();
    remove_open_probe(open_probe_host);
    if (open_probe_exposed_candidate)
    {
        instruction_list_hide_now();
        list_tdd_report("LIST_TDD FAIL opening_does_not_expose_the_hidden_candidate\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS opening_does_not_expose_the_hidden_candidate\n");
    lv_timer_create(rejected_open_retry_check, 1200, NULL);
}

static void rejected_query_keeps_the_view_and_retries(void)
{
    extern const char *instruction_list_text_filter(void);
    instruction_list_presentation_t presentation = {reject_presentation, record_rejected_publication, finish_rejected_presentation};
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[{\"id\":\"tdd-query-keep\",\"title\":\"Keep\"},{\"id\":\"tdd-query-other\",\"title\":\"Other\"}]";
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    rejection_published = false;
    instruction_list_bind_presentation(&presentation);
    instruction_list_set_query("Keep");
    instruction_list_bind_presentation(NULL);
    if (rejection_published || instruction_list_text_filter()[0] ||
        !list_has_id("tdd-query-keep") || !list_has_id("tdd-query-other"))
    {
        instruction_list_set_query("");
        list_tdd_report("LIST_TDD FAIL rejected_query_keeps_the_view_and_retries\n");
        return;
    }
    apply_pending_instruction_batch();
    if (strcmp(instruction_list_text_filter(), "Keep") || !list_has_id("tdd-query-keep") || list_has_id("tdd-query-other"))
    {
        instruction_list_set_query("");
        list_tdd_report("LIST_TDD FAIL rejected_query_intent_did_not_resume\n");
        return;
    }
    instruction_list_set_query("");
    list_tdd_report("LIST_TDD PASS rejected_query_keeps_the_view_and_retries\n");
    rejected_open_stays_closed_and_retries();
}

static void complete_lists_coalesce_while_search_is_frozen(void)
{
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[{\"id\":\"tdd-coalesce-old\",\"title\":\"Safe\"}]";
    char batch[100];
    unsigned i;
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    instruction_list_set_query("Safe");
    for (i = 0; i < 60; i++)
    {
        rt_snprintf(batch, sizeof(batch), "[{\"id\":\"tdd-coalesce-%u\",\"title\":\"Safe latest\"}]", i);
        resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)batch, (uint16_t)strlen(batch));
        apply_pending_instruction_batch();
    }
    if (instruction_list_get_status().needs_sync || !list_has_id("tdd-coalesce-old") || list_has_id("tdd-coalesce-59"))
    {
        instruction_list_set_query("");
        list_tdd_report("LIST_TDD FAIL complete_lists_coalesce_while_search_is_frozen\n");
        return;
    }
    instruction_list_set_query("");
    if (!list_has_id("tdd-coalesce-59") || list_has_id("tdd-coalesce-old") || instruction_list_get_status().needs_sync)
    {
        list_tdd_report("LIST_TDD FAIL coalesced_list_did_not_resume\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS complete_lists_coalesce_while_search_is_frozen\n");
    rejected_query_keeps_the_view_and_retries();
}

static void capability_survives_search_queue_overflow(void)
{
    instruction_list_transport_t transport = {test_clock, test_ready, test_awake, test_request};
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[{\"id\":\"tdd-cap-safe\",\"title\":\"Safe\"}]";
    char remove[] = "tdd-cap-safe";
    unsigned i;
    instruction_list_bind_transport(&transport);
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    instruction_list_set_query("Safe");
    caps[1] = 1;
    instruction_list_submit_phone(&capability);
    for (i = 0; i < 30; i++)
        resolve_Notify_command(KEY_DISMISS_SKAI_INSTRUCTION, (uint8_t *)remove, (uint16_t)strlen(remove));
    apply_pending_instruction_batch();
    if (!instruction_list_get_status().supports_resend || !instruction_list_get_status().needs_sync ||
        !list_has_id("tdd-cap-safe"))
    {
        instruction_list_set_query("");
        instruction_list_bind_transport(NULL);
        list_tdd_report("LIST_TDD FAIL capability_survives_search_queue_overflow\n");
        return;
    }
    instruction_list_set_query("");
    if (instruction_list_get_status().active_request == 0)
    {
        instruction_list_bind_transport(NULL);
        list_tdd_report("LIST_TDD FAIL capability_recovery_did_not_resume\n");
        return;
    }
    instruction_list_bind_transport(NULL);
    list_tdd_report("LIST_TDD PASS capability_survives_search_queue_overflow\n");
    complete_lists_coalesce_while_search_is_frozen();
}

static void scope_change_during_preparation_keeps_the_old_list(void)
{
    instruction_list_transport_t transport = {test_clock, test_ready, test_awake, test_request};
    instruction_list_presentation_t presentation = {
        change_scope_before_publication, record_rejected_publication, finish_rejected_presentation
    };
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char old[] = "[{\"id\":\"tdd-scope-old\",\"title\":\"Old scope\"}]";
    char next[] = "[{\"id\":\"tdd-scope-next\",\"title\":\"Expired scope\"}]";
    instruction_list_bind_transport(&transport);
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)old, (uint16_t)strlen(old));
    apply_pending_instruction_batch();
    rejection_published = false;
    instruction_list_bind_presentation(&presentation);
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)next, (uint16_t)strlen(next));
    apply_pending_instruction_batch();
    instruction_list_bind_presentation(NULL);
    instruction_list_bind_transport(NULL);
    if (rejection_published || !list_has_id("tdd-scope-old") || list_has_id("tdd-scope-next") ||
        !instruction_list_get_status().needs_sync)
    {
        list_tdd_report("LIST_TDD FAIL scope_change_during_preparation_keeps_the_old_list\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS scope_change_during_preparation_keeps_the_old_list\n");
    capability_survives_search_queue_overflow();
}

static void rejected_presentation_check(lv_timer_t *timer)
{
    extern void instruction_list_hide_now(void);
    extern int instruction_switch_pos(const char *id);
    instruction_list_transport_t transport = {test_clock, test_ready, test_awake, test_request};
    instruction_list_presentation_t presentation = {
        reject_presentation, record_rejected_publication, finish_rejected_presentation
    };
    uint8_t caps[2] = {2, 1};
    instruction_list_delivery_t delivery = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char invalid[] = "[{}]";
    char reply[480];
    uint32_t previous_receipt = instruction_list_get_status().completed_request;
    uint32_t requested;
    lv_timer_del(timer);
    if (!visible_tree_has_title(lv_layer_top(), "Retained old") ||
        !visible_tree_has_title(lv_layer_top(), "Old preview"))
    {
        instruction_list_hide_now();
        list_tdd_report("LIST_TDD FAIL rejected_presentation_setup\n");
        return;
    }
    instruction_list_submit_phone(&delivery);
    apply_pending_instruction_batch();
    instruction_list_bind_transport(&transport);
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)invalid, (uint16_t)strlen(invalid));
    apply_pending_instruction_batch();
    requested = instruction_list_get_status().active_request;
    if (requested == 0)
    {
        instruction_list_hide_now();
        instruction_list_bind_transport(NULL);
        list_tdd_report("LIST_TDD FAIL rejected_presentation_request_setup\n");
        return;
    }
    rejection_saw_candidate = rejection_published = false;
    rejection_expected_request = requested;
    instruction_list_bind_presentation(&presentation);
    rt_snprintf(reply, sizeof(reply),
        "{\"op\":\"snapshot\",\"id\":%lu,\"scope\":%lu,\"items\":["
        "{\"id\":\"tdd-retained-old\",\"title\":\"Rejected edit\",\"sub\":\"Rejected preview\",\"sw\":1,\"cat\":\"/\"},"
        "{\"id\":\"tdd-rejected-new\",\"title\":\"Rejected new\",\"sw\":1,\"cat\":\"/\"}]}",
        (unsigned long)requested, (unsigned long)test_requested_scope);
    delivery.key = KEY_INSTRUCTION_LIST_SYNC;
    delivery.command = SKAI_LINK_COMMAND_ID;
    delivery.payload = reply;
    delivery.length = (uint16_t)strlen(reply);
    instruction_list_submit_phone(&delivery);
    apply_pending_instruction_batch();
    instruction_list_bind_presentation(NULL);
    if (!rejection_was_pending)
    {
        instruction_list_hide_now();
        instruction_list_bind_transport(NULL);
        list_tdd_report("LIST_TDD FAIL snapshot_stays_pending_until_publication\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS snapshot_stays_pending_until_publication\n");
    if (!rejection_saw_candidate || rejection_published ||
        list_has_id("tdd-rejected-new") || !list_has_id("tdd-retained-old") ||
        instruction_switch_pos("tdd-retained-old") != 0 || instruction_switch_pos("tdd-rejected-new") != -1 ||
        !instruction_list_get_status().needs_sync ||
        instruction_list_get_status().completed_request != previous_receipt ||
        !visible_tree_has_title(lv_layer_top(), "Retained old"))
    {
        instruction_list_hide_now();
        instruction_list_bind_transport(NULL);
        list_tdd_report("LIST_TDD FAIL rejected_presentation_retains_the_entire_old_model\n");
        return;
    }
    /* Rebuild through a public query intent: this also observes the retained
       preview side table rather than only an already-painted old label. */
    instruction_list_set_query("Retained old");
    if (!visible_tree_has_title(lv_layer_top(), "Old preview") ||
        visible_tree_has_title(lv_layer_top(), "Rejected preview"))
    {
        instruction_list_hide_now();
        instruction_list_bind_transport(NULL);
        list_tdd_report("LIST_TDD FAIL rejected_presentation_leaked_preview_state\n");
        return;
    }
    instruction_list_set_query("");
    instruction_list_hide_now();
    instruction_list_bind_transport(NULL);
    list_tdd_report("LIST_TDD PASS rejected_presentation_retains_the_entire_old_model\n");
    scope_change_during_preparation_keeps_the_old_list();
}

static void rejected_presentation_retains_the_entire_old_model(void)
{
    extern void instruction_list_reveal_drag_begin_ex(bool from_left, char filter);
    extern void instruction_list_reveal_drag_update(lv_coord_t dx);
    extern void instruction_list_reveal_drag_end(lv_coord_t dx, lv_coord_t vx);
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char old[] = "[{\"id\":\"tdd-retained-old\",\"title\":\"Retained old\",\"sub\":\"Old preview\",\"sw\":0,\"cat\":\"/\"}]";
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)old, (uint16_t)strlen(old));
    apply_pending_instruction_batch();
    instruction_list_reveal_drag_begin_ex(true, '/');
    instruction_list_reveal_drag_update(LV_HOR_RES);
    instruction_list_reveal_drag_end(LV_HOR_RES, 0);
    lv_timer_create(rejected_presentation_check, 1200, NULL);
}

static void focus_update_after_settle(lv_timer_t *timer)
{
    extern void instruction_list_hide_now(void);
    const char *focused;
    char reordered[] = "[{\"id\":\"tdd-focus-c\",\"title\":\"Focus C\",\"cat\":\"/\"},{\"id\":\"tdd-focus-a\",\"title\":\"Focus A\",\"cat\":\"/\"},{\"id\":\"tdd-focus-b\",\"title\":\"Focus B\",\"cat\":\"/\"}]";
    lv_timer_del(timer);
    left_cards_scroll_to(1, false);
    focused = instruction_list_export_id(left_cards_current());
    if (!visible_tree_has_title(lv_layer_top(), "Focus B") ||
        focused == NULL || strcmp(focused, "tdd-focus-b") != 0)
    {
        instruction_list_hide_now();
        list_tdd_report("LIST_TDD FAIL focus_preservation_setup\n");
        return;
    }
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)reordered, (uint16_t)strlen(reordered));
    apply_pending_instruction_batch();
    focused = instruction_list_export_id(left_cards_current());
    if (!visible_tree_has_title(lv_layer_top(), "Focus B") ||
        focused == NULL || strcmp(focused, "tdd-focus-b") != 0)
    {
        instruction_list_hide_now();
        list_tdd_report("LIST_TDD FAIL source_updates_keep_the_focused_identity\n");
        return;
    }
    instruction_list_hide_now();
    list_tdd_report("LIST_TDD PASS source_updates_keep_the_focused_identity\n");
    rejected_presentation_retains_the_entire_old_model();
}

static void source_updates_keep_the_focused_identity(void)
{
    extern void instruction_list_reveal_drag_begin_ex(bool from_left, char filter);
    extern void instruction_list_reveal_drag_update(lv_coord_t dx);
    extern void instruction_list_reveal_drag_end(lv_coord_t dx, lv_coord_t vx);
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[{\"id\":\"tdd-focus-a\",\"title\":\"Focus A\",\"cat\":\"/\"},{\"id\":\"tdd-focus-b\",\"title\":\"Focus B\",\"cat\":\"/\"},{\"id\":\"tdd-focus-c\",\"title\":\"Focus C\",\"cat\":\"/\"}]";
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    instruction_list_reveal_drag_begin_ex(true, '/');
    instruction_list_reveal_drag_update(LV_HOR_RES);
    instruction_list_reveal_drag_end(LV_HOR_RES, 0);
    /* Entry deliberately prefers the newest session for eight seconds. Test
       steady browsing after that documented grace, rather than bypassing it. */
    lv_timer_create(focus_update_after_settle, 9000, NULL);
}

static void visible_commit_finish(lv_timer_t *timer)
{
    extern void instruction_list_hide_now(void);
    if (timer) lv_timer_del(timer);
    apply_pending_instruction_batch();
    if (!visible_tree_has_title(lv_layer_top(), "TDD visible new") ||
        !list_has_id("tdd-visible-new") || instruction_list_get_status().needs_sync ||
        instruction_list_get_status().completed_request != visible_expected_receipt)
    {
        instruction_list_hide_now();
        instruction_list_bind_transport(NULL);
        list_tdd_report("LIST_TDD FAIL visible_snapshot_did_not_finish\n");
        return;
    }
    instruction_list_hide_now();
    instruction_list_bind_transport(NULL);
    list_tdd_report("LIST_TDD PASS visible_snapshot_keeps_text_targets_and_receipt_together\n");
    source_updates_keep_the_focused_identity();
}

static void visible_commit_deliver(lv_timer_t *timer)
{
    extern void instruction_list_hide_now(void);
    instruction_list_transport_t transport = {test_clock, test_ready, test_awake, test_request};
    uint8_t caps[2] = {2, 1};
    instruction_list_delivery_t delivery = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char old[] = "[{\"id\":\"tdd-visible-old\",\"title\":\"TDD visible old\",\"cat\":\"/\"}]";
    char invalid[] = "[{}]";
    char reply[220];
    bool new_visible;
    lv_timer_del(timer);
    /* The floating list is on LVGL's top layer, separate from lv_scr_act(). */
    if (!visible_tree_has_title(lv_layer_top(), "TDD visible old"))
    {
        instruction_list_hide_now();
        list_tdd_report("LIST_TDD FAIL visible_snapshot_setup\n");
        return;
    }
    /* Two deliveries in one GUI turn exercise the real rebuild throttle.
       The module may defer publication, but the old targets must remain valid. */
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)old, (uint16_t)strlen(old));
    apply_pending_instruction_batch();
    instruction_list_submit_phone(&delivery);
    apply_pending_instruction_batch();
    instruction_list_bind_transport(&transport);
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)invalid, (uint16_t)strlen(invalid));
    apply_pending_instruction_batch();
    visible_expected_receipt = instruction_list_get_status().active_request;
    if (visible_expected_receipt == 0)
    {
        instruction_list_hide_now();
        instruction_list_bind_transport(NULL);
        list_tdd_report("LIST_TDD FAIL visible_snapshot_request_setup\n");
        return;
    }
    rt_snprintf(reply, sizeof(reply),
        "{\"op\":\"snapshot\",\"id\":%lu,\"scope\":%lu,\"items\":[{\"id\":\"tdd-visible-new\",\"title\":\"TDD visible new\",\"cat\":\"/\"}]}",
        (unsigned long)visible_expected_receipt, (unsigned long)test_requested_scope);
    delivery.key = KEY_INSTRUCTION_LIST_SYNC;
    delivery.command = SKAI_LINK_COMMAND_ID;
    delivery.payload = reply;
    delivery.length = (uint16_t)strlen(reply);
    instruction_list_submit_phone(&delivery);
    apply_pending_instruction_batch();
    new_visible = visible_tree_has_title(lv_layer_top(), "TDD visible new");
    if ((!new_visible && !list_has_id("tdd-visible-old")) ||
        (new_visible && !list_has_id("tdd-visible-new")) ||
        (instruction_list_get_status().completed_request == visible_expected_receipt && !new_visible))
    {
        instruction_list_hide_now();
        instruction_list_bind_transport(NULL);
        list_tdd_report("LIST_TDD FAIL visible_snapshot_keeps_text_targets_and_receipt_together\n");
        return;
    }
    if (new_visible) visible_commit_finish(NULL);
    else lv_timer_create(visible_commit_finish, 700, NULL);
}

static void visible_snapshot_keeps_text_targets_and_receipt_together(void)
{
    extern void instruction_list_reveal_drag_begin_ex(bool from_left, char filter);
    extern void instruction_list_reveal_drag_update(lv_coord_t dx);
    extern void instruction_list_reveal_drag_end(lv_coord_t dx, lv_coord_t vx);
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char old[] = "[{\"id\":\"tdd-visible-old\",\"title\":\"TDD visible old\",\"cat\":\"/\"}]";
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)old, (uint16_t)strlen(old));
    apply_pending_instruction_batch();
    instruction_list_reveal_drag_begin_ex(true, '/');
    instruction_list_reveal_drag_update(LV_HOR_RES);
    instruction_list_reveal_drag_end(LV_HOR_RES, 0);
    /* Let entry animation and any earlier trailing rebuild finish. */
    lv_timer_create(visible_commit_deliver, 1200, NULL);
}

static void removal_outside_the_category_reaches_the_full_list(void)
{
    extern void instruction_list_reveal_drag_begin_ex(bool from_left, char filter);
    extern void instruction_list_hide_now(void);
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[{\"id\":\"tdd-action\",\"title\":\"Action\",\"cat\":\"/\"},{\"id\":\"tdd-hidden-chat\",\"title\":\"Chat\",\"cat\":\"@\"}]";
    char removed[] = "tdd-hidden-chat";
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    instruction_list_reveal_drag_begin_ex(true, '/');
    resolve_Notify_command(KEY_DISMISS_SKAI_INSTRUCTION,
                           (uint8_t *)removed, (uint16_t)strlen(removed));
    apply_pending_instruction_batch();
    instruction_list_hide_now();
    instruction_list_reveal_drag_begin_ex(true, 0);
    if (!list_has_id("tdd-action") || list_has_id("tdd-hidden-chat"))
    {
        instruction_list_hide_now();
        list_tdd_report("LIST_TDD FAIL removal_outside_the_category_reaches_the_full_list\n");
        return;
    }
    instruction_list_hide_now();
    list_tdd_report("LIST_TDD PASS removal_outside_the_category_reaches_the_full_list\n");
    visible_snapshot_keeps_text_targets_and_receipt_together();
}

static void replacement_in_a_category_keeps_deletions(void)
{
    extern void instruction_list_reveal_drag_begin_ex(bool from_left, char filter);
    extern void instruction_list_hide_now(void);
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[{\"id\":\"tdd-old-action\",\"title\":\"Old\",\"cat\":\"/\"},{\"id\":\"tdd-chat\",\"title\":\"Chat\",\"cat\":\"@\"}]";
    char replacement[] = "[{\"id\":\"tdd-new-action\",\"title\":\"New\",\"cat\":\"/\"}]";
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    instruction_list_reveal_drag_begin_ex(true, '/');
    if (!list_has_id("tdd-old-action") || list_has_id("tdd-chat"))
    {
        extern bool instruction_list_is_visible(void);
        list_tdd_report("LIST_TDD FAIL category_replacement_setup old=%u chat=%u visible=%u reserved=%lu pending=%u\n",
            (unsigned)list_has_id("tdd-old-action"), (unsigned)list_has_id("tdd-chat"),
            (unsigned)instruction_list_is_visible(), (unsigned long)instruction_list_get_status().render_reserved_bytes,
            (unsigned)instruction_list_get_status().needs_sync);
        return;
    }
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)replacement, (uint16_t)strlen(replacement));
    apply_pending_instruction_batch();
    if (list_has_id("tdd-old-action") || list_has_id("tdd-chat") ||
        !list_has_id("tdd-new-action"))
    {
        instruction_list_hide_now();
        list_tdd_report("LIST_TDD FAIL replacement_in_a_category_keeps_deletions\n");
        return;
    }
    instruction_list_hide_now();
    list_tdd_report("LIST_TDD PASS replacement_in_a_category_keeps_deletions\n");
    removal_outside_the_category_reaches_the_full_list();
}

static void unrepresentable_identity_keeps_the_old_list(void)
{
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[{\"id\":\"tdd-safe\",\"title\":\"Safe\"}]";
    char invalid[] = "[{\"id\":\"tdd-new\",\"title\":\"New\"},{\"id\":\"0123456789012345678901234567890123456789012345678901234567890123\",\"title\":\"Too long\"}]";
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)invalid, (uint16_t)strlen(invalid));
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-safe") || list_has_id("tdd-new") || !instruction_list_get_status().needs_sync)
    {
        list_tdd_report("LIST_TDD FAIL unrepresentable_identity_keeps_the_old_list\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS unrepresentable_identity_keeps_the_old_list\n");
    replacement_in_a_category_keeps_deletions();
}

static void invalid_optional_fields_reject_the_complete_list(void)
{
    const char *bad[] = {
        "\"enabled\":\"yes\"", "\"trigger\":[]", "\"trigger\":{\"type\":7}",
        "\"trigger\":{\"type\":\"interval\",\"intervalSeconds\":-1}",
        "\"trigger\":{\"type\":\"interval\",\"intervalSeconds\":0.5}",
        "\"cat\":\"unknown\"", "\"sw\":2", "\"openApp\":true",
        "\"svc\":{}", "\"ico\":[]", "\"sub\":17", "\"title\":\"duplicate\""
    };
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[{\"id\":\"tdd-safe\",\"title\":\"Safe\"}]";
    char invalid[360];
    unsigned i;
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    for (i = 0; i < sizeof(bad) / sizeof(bad[0]); i++)
    {
        resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)seed, (uint16_t)strlen(seed));
        apply_pending_instruction_batch();
        rt_snprintf(invalid, sizeof(invalid),
            "[{\"id\":\"tdd-new\",\"title\":\"New\"},{\"id\":\"tdd-bad\",\"title\":\"Bad\",%s}]", bad[i]);
        resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)invalid, (uint16_t)strlen(invalid));
        apply_pending_instruction_batch();
        if (!list_has_id("tdd-safe") || list_has_id("tdd-new") || !instruction_list_get_status().needs_sync)
        {
            list_tdd_report("LIST_TDD FAIL invalid_optional_fields_reject_the_complete_list case=%u\n", i);
            return;
        }
    }
    list_tdd_report("LIST_TDD PASS invalid_optional_fields_reject_the_complete_list\n");
    unrepresentable_identity_keeps_the_old_list();
}

static void invalid_version_rejects_the_complete_list(void)
{
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[{\"id\":\"tdd-safe\",\"title\":\"Safe\"}]";
    char invalid[] = "[{\"id\":\"tdd-new\",\"title\":\"New\"},{\"id\":\"tdd-bad-version\",\"title\":\"Bad\",\"version\":\"new\"}]";
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)invalid, (uint16_t)strlen(invalid));
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-safe") || list_has_id("tdd-new") ||
        !instruction_list_get_status().needs_sync)
    {
        list_tdd_report("LIST_TDD FAIL invalid_version_rejects_the_complete_list\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS invalid_version_rejects_the_complete_list\n");
    invalid_optional_fields_reject_the_complete_list();
}

static void legal_switch_rows_keep_their_state(void)
{
    extern int instruction_switch_pos(const char *id);
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char payload[1200], id[32];
    size_t used = 0;
    unsigned i;
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    payload[used++] = '[';
    for (i = 0; i < 9; i++)
        used += rt_snprintf(payload + used, sizeof(payload) - used,
            "%s{\"id\":\"tdd-switch-%u\",\"title\":\"Switch\",\"sw\":%u}", i ? "," : "", i, i % 2);
    payload[used++] = ']';
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)payload, used);
    apply_pending_instruction_batch();
    for (i = 0; i < 9; i++) {
        rt_snprintf(id, sizeof(id), "tdd-switch-%u", i);
        if (!list_has_id(id) || instruction_switch_pos(id) != (int)(i % 2)) {
            list_tdd_report("LIST_TDD FAIL legal_switch_rows_keep_their_state row=%u\n", i); return;
        }
    }
    list_tdd_report("LIST_TDD PASS legal_switch_rows_keep_their_state\n");
    device_close_restores_the_complete_context();
}

static void duplicate_id_rejects_the_complete_list(void)
{
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[{\"id\":\"tdd-safe\",\"title\":\"Safe\"}]";
    char invalid[] = "[{\"id\":\"tdd-duplicate\",\"title\":\"One\"},{\"id\":\"tdd-duplicate\",\"title\":\"Two\"}]";
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)invalid, (uint16_t)strlen(invalid));
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-safe") || list_has_id("tdd-duplicate") ||
        !instruction_list_get_status().needs_sync)
    {
        list_tdd_report("LIST_TDD FAIL duplicate_id_rejects_the_complete_list\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS duplicate_id_rejects_the_complete_list\n");
    legal_switch_rows_keep_their_state();
}

static void queue_loss_during_send_invalidates_the_request(void)
{
    instruction_list_transport_t transport = {test_clock, test_ready, test_awake, test_request};
    char invalid[] = "[{}]";
    instruction_list_bind_transport(&transport);
    test_lose_updates_on_send = true;
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)invalid, strlen(invalid));
    apply_pending_instruction_batch();
    instruction_list_status_t status = instruction_list_get_status();
    instruction_list_bind_transport(NULL);
    if (test_lose_updates_on_send || !status.needs_sync || status.active_request != 0) {
        list_tdd_report("LIST_TDD FAIL queue_loss_during_send_invalidates_the_request\n"); return;
    }
    list_tdd_report("LIST_TDD PASS queue_loss_during_send_invalidates_the_request\n");
    duplicate_id_rejects_the_complete_list();
}

static void a_scope_change_during_send_invalidates_the_request(void)
{
    instruction_list_transport_t transport = {test_clock, test_ready, test_awake, test_request};
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[]", invalid[] = "[{}]";
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    caps[1] = 1;
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    instruction_list_bind_transport(&transport);
    test_change_scope_on_send = true;
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)invalid, (uint16_t)strlen(invalid));
    apply_pending_instruction_batch();
    instruction_list_bind_transport(NULL);
    if (instruction_list_get_status().active_request != 0 ||
        instruction_list_get_status().scope == test_requested_scope)
    {
        list_tdd_report("LIST_TDD FAIL a_scope_change_during_send_invalidates_the_request\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS a_scope_change_during_send_invalidates_the_request\n");
    queue_loss_during_send_invalidates_the_request();
}

static void queue_loss_invalidates_the_outstanding_request(void)
{
    instruction_list_transport_t transport = {test_clock, test_ready, test_awake, test_request};
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[]", invalid[] = "[{}]";
    unsigned i;
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    caps[1] = 1;
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    instruction_list_bind_transport(&transport);
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)invalid, (uint16_t)strlen(invalid));
    apply_pending_instruction_batch();
    if (instruction_list_get_status().active_request == 0)
    {
        list_tdd_report("LIST_TDD FAIL outstanding_request_setup\n");
        return;
    }
    for (i = 0; i < 24; i++)
        resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                               (uint8_t *)seed, (uint16_t)strlen(seed));
    if (instruction_list_get_status().active_request != 0)
    {
        instruction_list_bind_transport(NULL);
        list_tdd_report("LIST_TDD FAIL queue_loss_invalidates_the_outstanding_request\n");
        return;
    }
    instruction_list_bind_transport(NULL);
    list_tdd_report("LIST_TDD PASS queue_loss_invalidates_the_outstanding_request\n");
    a_scope_change_during_send_invalidates_the_request();
}

static void retry_cadence_pauses_and_resumes(void)
{
    static const uint32_t times[] = {1999, 2000, 6999, 7000, 21999, 22000, 51999, 52000, 81999, 82000};
    static const unsigned expected[] = {1, 2, 2, 3, 3, 4, 4, 5, 5, 6};
    instruction_list_transport_t transport = {test_clock, test_ready, test_awake, test_request};
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[]", invalid[] = "[{}]";
    unsigned i;
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    caps[1] = 1;
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    test_now_ms = 0;
    test_request_count = 0;
    instruction_list_bind_transport(&transport);
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)invalid, (uint16_t)strlen(invalid));
    apply_pending_instruction_batch();
    if (test_request_count != 1) goto fail;
    for (i = 0; i < sizeof(times) / sizeof(times[0]); i++)
    {
        test_now_ms = times[i];
        apply_pending_instruction_batch();
        if (test_request_count != expected[i]) goto fail;
    }
    test_transport_ready = false;
    test_now_ms = 100000;
    apply_pending_instruction_batch();
    if (test_request_count != 6) goto fail;
    test_transport_ready = true;
    apply_pending_instruction_batch();
    if (test_request_count != 7) goto fail;
    test_transport_awake = false;
    test_now_ms = 120000;
    apply_pending_instruction_batch();
    if (test_request_count != 7) goto fail;
    test_transport_awake = true;
    apply_pending_instruction_batch();
    if (test_request_count != 8) goto fail;
    instruction_list_set_query("Waiting");
    test_now_ms = 500000;
    apply_pending_instruction_batch();
    if (test_request_count != 8) goto fail;
    instruction_list_set_query("");
    if (test_request_count != 9) goto fail;
    instruction_list_bind_transport(NULL);
    list_tdd_report("LIST_TDD PASS retry_cadence_pauses_and_resumes\n");
    queue_loss_invalidates_the_outstanding_request();
    return;
fail:
    test_transport_ready = test_transport_awake = true;
    instruction_list_bind_transport(NULL);
    list_tdd_report("LIST_TDD FAIL retry_cadence_pauses_and_resumes requests=%u\n", test_request_count);
}

static void combined_row_capacity_is_checked_before_replacement(void)
{
    extern void skai_sessions_on_conv_list(const uint8_t *json, uint16_t length);
    extern void session_pager_apply_pending(void);
    char seed[] = "[{\"id\":\"tdd-safe\",\"title\":\"Safe\"}]";
    const char sessions[] = "{\"device\":\"tdd-device\",\"sessions\":["
        "{\"id\":\"conv:tdd:one\",\"title\":\"One\"},"
        "{\"id\":\"conv:tdd:two\",\"title\":\"Two\"}]}";
    const char removed[] = "{\"device\":\"tdd-device\",\"sessions\":[]}";
    char *payload = (char *)app_cache_alloc(4096, IMAGE_CACHE_PSRAM);
    size_t used = 0;
    unsigned i;
    if (payload == NULL)
    {
        list_tdd_report("LIST_TDD FAIL combined_capacity_setup\n");
        return;
    }
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    skai_sessions_on_conv_list((const uint8_t *)sessions, sizeof(sessions) - 1);
    session_pager_apply_pending();
    payload[used++] = '[';
    for (i = 0; i < 29; i++)
        used += rt_snprintf(payload + used, 4096 - used,
                            "%s{\"id\":\"tdd-combined-%u\",\"title\":\"Row\"}", i ? "," : "", i);
    payload[used++] = ']';
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)payload, (uint16_t)used);
    app_cache_free(payload);
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-safe") || list_has_id("tdd-combined-0") ||
        !instruction_list_get_status().needs_sync)
    {
        list_tdd_report("LIST_TDD FAIL combined_row_capacity_is_checked_before_replacement\n");
        return;
    }
    skai_sessions_on_conv_list((const uint8_t *)removed, sizeof(removed) - 1);
    session_pager_apply_pending();
    list_tdd_report("LIST_TDD PASS combined_row_capacity_is_checked_before_replacement\n");
    retry_cadence_pauses_and_resumes();
}

static void session_selection_preserves_bot_order_and_search(void)
{
    extern void skai_sessions_on_conv_list(const uint8_t *json, uint16_t length);
    extern void session_pager_apply_pending(void);
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[]";
    const char sessions[] = "{\"device\":\"tdd-device\",\"sessions\":["
        "{\"id\":\"conv:tdd:older\",\"title\":\"Older room\",\"bot\":\"A\",\"ts\":1},"
        "{\"id\":\"conv:tdd:newest\",\"title\":\"Newest room\",\"bot\":\"A\",\"ts\":3},"
        "{\"id\":\"conv:tdd:other\",\"title\":\"Other room\",\"bot\":\"B\",\"ts\":2}]}";
    const char removed[] = "{\"device\":\"tdd-device\",\"sessions\":[]}";
    unsigned i, newest = 255, other = 255;
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    skai_sessions_on_conv_list((const uint8_t *)sessions, sizeof(sessions) - 1);
    session_pager_apply_pending();
    for (i = 0; i < return_total_list_count(); i++)
    {
        const char *id = instruction_list_export_id((uint8_t)i);
        if (id && strcmp(id, "conv:tdd:newest") == 0) newest = i;
        if (id && strcmp(id, "conv:tdd:other") == 0) other = i;
    }
    if (newest == 255 || other >= newest || list_has_id("conv:tdd:older"))
    {
        list_tdd_report("LIST_TDD FAIL session_selection_bot_order\n");
        return;
    }
    instruction_list_set_query("Older room");
    if (!list_has_id("conv:tdd:older") || list_has_id("conv:tdd:newest"))
    {
        list_tdd_report("LIST_TDD FAIL session_selection_search_pool\n");
        return;
    }
    instruction_list_set_query("");
    if (list_has_id("conv:tdd:older") || !list_has_id("conv:tdd:newest"))
    {
        list_tdd_report("LIST_TDD FAIL session_selection_restores_bots\n");
        return;
    }
    skai_sessions_on_conv_list((const uint8_t *)removed, sizeof(removed) - 1);
    session_pager_apply_pending();
    list_tdd_report("LIST_TDD PASS session_selection_preserves_bot_order_and_search\n");
    combined_row_capacity_is_checked_before_replacement();
}

static void oversized_list_keeps_the_committed_list(void)
{
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[{\"id\":\"tdd-safe\",\"title\":\"Safe\"}]";
    char *payload = (char *)app_cache_alloc(4096, IMAGE_CACHE_PSRAM);
    size_t used = 0;
    unsigned i;
    if (payload == NULL)
    {
        list_tdd_report("LIST_TDD FAIL row_capacity_setup\n");
        return;
    }
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    payload[used++] = '[';
    for (i = 0; i < 31; i++)
        used += rt_snprintf(payload + used, 4096 - used,
                            "%s{\"id\":\"tdd-row-%u\",\"title\":\"Row\"}", i ? "," : "", i);
    payload[used++] = ']';
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)payload, (uint16_t)used);
    app_cache_free(payload);
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-safe") || list_has_id("tdd-row-0") ||
        !instruction_list_get_status().needs_sync)
    {
        list_tdd_report("LIST_TDD FAIL oversized_list_keeps_the_committed_list\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS oversized_list_keeps_the_committed_list\n");
    session_selection_preserves_bot_order_and_search();
}

static void full_list_single_update_requires_recovery(void)
{
    extern void skai_sessions_on_conv_list(const uint8_t *json, uint16_t length);
    extern void session_pager_apply_pending(void);
    extern int session_list_device_count(void);
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char *batch = app_cache_alloc(4096, IMAGE_CACHE_PSRAM);
    char extra[] = "{\"id\":\"tdd-extra\",\"title\":\"Extra\"}";
    const char sessions[] = "{\"device\":\"tdd-cap-overflow\",\"sessions\":[{\"id\":\"conv:tdd:overflow\",\"title\":\"Overflow\"}]}";
    const char removed[] = "{\"device\":\"tdd-cap-overflow\",\"sessions\":[]}";
    unsigned actions = 30, i;
    size_t used = 0;
    int d;
    if (!batch) { list_tdd_report("LIST_TDD FAIL single_capacity_setup_alloc\n"); return; }
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    for (d = 0; d < session_list_device_count(); d++)
    {
        uint8_t selected[5];
        actions -= (unsigned)instruction_list_selected_sessions((uint8_t)d, selected, 5);
    }
    batch[used++] = '[';
    for (i = 0; i < actions; i++)
        used += rt_snprintf(batch + used, 4096 - used,
                            "%s{\"id\":\"tdd-full-%u\",\"title\":\"Full\"}", i ? "," : "", i);
    batch[used++] = ']';
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)batch, (uint16_t)used);
    apply_pending_instruction_batch();
    if (return_total_list_count() != 30 || !list_has_id("tdd-full-0"))
    {
        list_tdd_report("LIST_TDD FAIL single_capacity_setup\n");
        app_cache_free(batch);
        return;
    }
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS, (uint8_t *)extra, (uint16_t)strlen(extra));
    apply_pending_instruction_batch();
    if (return_total_list_count() != 30 || !list_has_id("tdd-full-0") ||
        list_has_id("tdd-extra") || !instruction_list_get_status().needs_sync)
    {
        list_tdd_report("LIST_TDD FAIL full_list_single_update_requires_recovery\n");
        app_cache_free(batch);
        return;
    }
    list_tdd_report("LIST_TDD PASS full_list_single_update_requires_recovery\n");
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)batch, (uint16_t)used);
    apply_pending_instruction_batch();
    skai_sessions_on_conv_list((const uint8_t *)sessions, sizeof(sessions) - 1);
    session_pager_apply_pending();
    if (return_total_list_count() != 30 || !list_has_id("tdd-full-0") ||
        list_has_id("conv:tdd:overflow") || !instruction_list_get_status().needs_sync)
    {
        app_cache_free(batch);
        list_tdd_report("LIST_TDD FAIL session_projection_capacity_requires_recovery\n");
        return;
    }
    skai_sessions_on_conv_list((const uint8_t *)removed, sizeof(removed) - 1);
    session_pager_apply_pending();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)batch, (uint16_t)used);
    app_cache_free(batch);
    apply_pending_instruction_batch();
    list_tdd_report("LIST_TDD PASS session_projection_capacity_requires_recovery\n");
    oversized_list_keeps_the_committed_list();
}

static void search_keeps_session_projection_pending(void)
{
    extern bool session_list_owner_of(const char *id, char *device, size_t device_len, char *bot, size_t bot_len);
    extern void skai_sessions_on_conv_list(const uint8_t *json, uint16_t length);
    extern void session_pager_apply_pending(void);
    uint8_t caps[2] = {2, 0};
    instruction_list_delivery_t capability = {KEY_PHONE_OS_VERSION, caps, 2, SET_CONFIG_COMMAND_ID};
    char seed[] = "[]";
    const char sessions[] = "{\"device\":\"tdd-device\",\"sessions\":[{\"id\":\"conv:tdd:keep\",\"title\":\"Safe session\"}]}";
    const char removed[] = "{\"device\":\"tdd-device\",\"sessions\":[]}";
    char owner[64];
    instruction_list_submit_phone(&capability);
    apply_pending_instruction_batch();
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    session_pager_apply_pending(); /* initialize the real session store */
    skai_sessions_on_conv_list((const uint8_t *)sessions, sizeof(sessions) - 1);
    session_pager_apply_pending();
    instruction_list_set_text_filter("Safe session");
    if (!list_has_id("conv:tdd:keep"))
    {
        list_tdd_report("LIST_TDD FAIL session_freeze_setup\n");
        return;
    }
    skai_sessions_on_conv_list((const uint8_t *)removed, sizeof(removed) - 1);
    session_pager_apply_pending();
    if (!list_has_id("conv:tdd:keep"))
    {
        list_tdd_report("LIST_TDD FAIL search_keeps_session_projection_pending\n");
        return;
    }
    if (!session_list_owner_of("conv:tdd:keep", owner, sizeof(owner), NULL, 0) ||
        strcmp(owner, "tdd-device") != 0)
    {
        list_tdd_report("LIST_TDD FAIL frozen_session_keeps_its_device_route\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS frozen_session_keeps_its_device_route\n");
    instruction_list_set_text_filter("");
    apply_pending_instruction_batch();
    if (list_has_id("conv:tdd:keep"))
    {
        list_tdd_report("LIST_TDD FAIL session_updates_resume_after_search\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS search_keeps_session_projection_pending\n");
    full_list_single_update_requires_recovery();
}

static void parsing_budget_failure_keeps_the_list(void)
{
    char seed[] = "[{\"id\":\"tdd-safe\",\"title\":\"Safe\"}]";
    const char prefix[] = "[{\"id\":\"tdd-excess\",\"title\":\"Other\",\"noise\":[";
    char *payload = (char *)app_cache_alloc(8192, IMAGE_CACHE_PSRAM);
    size_t used = sizeof(prefix) - 1;
    unsigned i;
    if (payload == NULL)
    {
        list_tdd_report("LIST_TDD FAIL parsing_budget_setup\n");
        return;
    }
    instruction_list_set_text_filter("");
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    memcpy(payload, prefix, used);
    for (i = 0; i < 3000; i++)
    {
        if (i != 0) payload[used++] = ',';
        payload[used++] = '0';
    }
    memcpy(payload + used, "]}]", 4);
    used += 3;
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)payload, (uint16_t)used);
    app_cache_free(payload);
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-safe") || list_has_id("tdd-excess"))
    {
        list_tdd_report("LIST_TDD FAIL parsing_budget_failure_keeps_the_list\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS parsing_budget_failure_keeps_the_list\n");
    search_keeps_session_projection_pending();
}

static void duplicate_snapshot_keeps_later_updates(void)
{
    instruction_list_transport_t transport = {test_clock, test_ready, test_ready, test_request};
    char invalid[] = "[{}]";
    char delta[] = "{\"id\":\"tdd-after-cut\",\"title\":\"After cut\"}";
    char reply[240];
    instruction_list_delivery_t delivery = {KEY_INSTRUCTION_LIST_SYNC, reply, 0, SKAI_LINK_COMMAND_ID};
    instruction_list_bind_transport(&transport);
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, (uint8_t *)invalid, strlen(invalid));
    apply_pending_instruction_batch();
    uint32_t id = instruction_list_get_status().active_request;
    if (!id) { list_tdd_report("LIST_TDD FAIL duplicate_snapshot_setup\n"); return; }
    rt_snprintf(reply, sizeof(reply),
        "{\"op\":\"snapshot\",\"id\":%lu,\"scope\":%lu,\"items\":[]}",
        (unsigned long)id, (unsigned long)instruction_list_get_status().scope);
    delivery.length = strlen(reply);
    instruction_list_submit_phone(&delivery);
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS, (uint8_t *)delta, strlen(delta));
    instruction_list_submit_phone(&delivery);
    apply_pending_instruction_batch();
    instruction_list_status_t status = instruction_list_get_status();
    instruction_list_bind_transport(NULL);
    if (!list_has_id("tdd-after-cut") || status.needs_sync || status.completed_request != id) {
        list_tdd_report("LIST_TDD FAIL duplicate_snapshot_keeps_later_updates\n"); return;
    }
    list_tdd_report("LIST_TDD PASS duplicate_snapshot_keeps_later_updates\n");
    parsing_budget_failure_keeps_the_list();
}

static void only_latest_snapshot_can_end_recovery(void)
{
    instruction_list_transport_t transport = {test_clock, test_ready, test_ready, test_request};
    instruction_list_delivery_t delivery = {0};
    uint8_t caps[2] = {2, 1};
    uint32_t first, latest;
    char reply[240];
    test_now_ms = test_requested_id = test_requested_scope = 0;
    instruction_list_bind_transport(&transport);
    delivery.command = SET_CONFIG_COMMAND_ID;
    delivery.key = KEY_PHONE_OS_VERSION;
    delivery.payload = caps;
    delivery.length = sizeof(caps);
    instruction_list_submit_phone(&delivery);
    apply_pending_instruction_batch();
    first = test_requested_id;
    test_now_ms = 2000;
    apply_pending_instruction_batch();
    latest = test_requested_id;
    if (first == 0 || latest == first)
    {
        list_tdd_report("LIST_TDD FAIL correlated_snapshot_retry\n");
        return;
    }
    delivery.command = SKAI_LINK_COMMAND_ID;
    delivery.key = KEY_INSTRUCTION_LIST_SYNC;
    delivery.payload = reply;
    rt_snprintf(reply, sizeof(reply),
        "{\"op\":\"snapshot\",\"id\":%lu,\"scope\":%lu,\"items\":[{\"id\":\"tdd-stale\",\"title\":\"Stale\"}]}",
        (unsigned long)first, (unsigned long)test_requested_scope);
    delivery.length = (uint16_t)strlen(reply);
    instruction_list_submit_phone(&delivery);
    apply_pending_instruction_batch();
    if (list_has_id("tdd-stale") || !instruction_list_get_status().needs_sync)
    {
        list_tdd_report("LIST_TDD FAIL stale_snapshot_changed_the_list\n");
        return;
    }
    rt_snprintf(reply, sizeof(reply),
        "{\"op\":\"snapshot\",\"id\":%lu,\"scope\":%lu,\"items\":[{\"id\":\"tdd-fresh\",\"title\":\"Fresh\"}]}",
        (unsigned long)latest, (unsigned long)test_requested_scope);
    delivery.length = (uint16_t)strlen(reply);
    instruction_list_submit_phone(&delivery);
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-fresh") || instruction_list_get_status().needs_sync ||
        instruction_list_get_status().completed_request != latest)
    {
        list_tdd_report("LIST_TDD FAIL latest_snapshot_did_not_commit\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS only_latest_snapshot_can_end_recovery\n");
    instruction_list_bind_transport(NULL);
    duplicate_snapshot_keeps_later_updates();
}

static void ingress_has_a_total_byte_budget(void)
{
    char seed[] = "[{\"id\":\"tdd-safe\",\"title\":\"Safe\"}]";
    const char object[] = "{\"id\":\"tdd-large\",\"title\":\"Safe\"}";
    char *payload = (char *)app_cache_alloc(6001, IMAGE_CACHE_PSRAM);
    instruction_list_delivery_t delivery = {0};
    bool first, second, third;
    if (payload == NULL)
    {
        list_tdd_report("LIST_TDD FAIL byte_budget_setup_allocation\n");
        return;
    }
    memset(payload, ' ', 6000);
    memcpy(payload, object, strlen(object));
    payload[6000] = '\0';
    instruction_list_set_text_filter("");
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    instruction_list_set_text_filter("Safe");
    delivery.key = KEY_SKAI_CREATION_INSTRUCTIONS;
    delivery.payload = payload;
    delivery.length = 6000;
    first = instruction_list_submit_phone(&delivery);
    second = instruction_list_submit_phone(&delivery);
    third = instruction_list_submit_phone(&delivery);
    app_cache_free(payload);
    if (!first || !second || third)
    {
        list_tdd_report("LIST_TDD FAIL ingress_has_a_total_byte_budget admitted=%u%u%u\n",
                        (unsigned)first, (unsigned)second, (unsigned)third);
        return;
    }
    list_tdd_report("LIST_TDD PASS ingress_has_a_total_byte_budget\n");
    instruction_list_set_text_filter("");
    apply_pending_instruction_batch();
    only_latest_snapshot_can_end_recovery();
}

static void queue_loss_keeps_the_committed_list(void)
{
    char seed[] = "[{\"id\":\"tdd-safe\",\"title\":\"Safe\"}]";
    char update[120];
    unsigned i;
    instruction_list_set_text_filter("");
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    instruction_list_set_text_filter("Safe");
    for (i = 0; i < 25; i++)
    {
        rt_snprintf(update, sizeof(update),
                    "{\"id\":\"tdd-increment-%u\",\"title\":\"Safe\"}", i);
        resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS,
                               (uint8_t *)update, (uint16_t)strlen(update));
    }
    instruction_list_set_text_filter("");
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-safe") || list_has_id("tdd-increment-24"))
    {
        list_tdd_report("LIST_TDD FAIL queue_loss_keeps_the_committed_list\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS queue_loss_keeps_the_committed_list\n");
    ingress_has_a_total_byte_budget();
}

static void invalid_batch_preserves_committed_list(void)
{
    char seed[] = "[{\"id\":\"tdd-safe\",\"title\":\"Safe\"}]";
    char invalid[] = "[{\"id\":\"tdd-other\",\"title\":\"Other\"},{\"id\":\"tdd-bad\"}]";
    instruction_list_set_text_filter("");
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)seed, (uint16_t)strlen(seed));
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-safe"))
    {
        list_tdd_report("LIST_TDD FAIL invalid_batch_seed\n");
        return;
    }
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)invalid, (uint16_t)strlen(invalid));
    apply_pending_instruction_batch();
    if (!list_has_id("tdd-safe") || list_has_id("tdd-other") || list_has_id("tdd-bad"))
    {
        list_tdd_report("LIST_TDD FAIL invalid_batch_preserves_committed_list\n");
        return;
    }
    list_tdd_report("LIST_TDD PASS invalid_batch_preserves_committed_list\n");
    queue_loss_keeps_the_committed_list();
}

/* Exercise the actual phone ingress and list interface on the GUI thread.
 * Assertions observe the same committed rows used by the watch UI. */
#ifndef BSP_USING_PC_SIMULATOR
static bool exhausted_psram_keeps_the_model_and_system_heap(void)
{
    extern struct rt_memheap app_image_psram_memheap;
    void *held[128];
    unsigned n = 0;
    rt_uint32_t total, before, after, peak;
    uint8_t old_count = return_total_list_count();
    char payload[] = "[]";
    /* Fixture consumes only PSRAM through the native heap, never SRAM or GPU
       objects. Run once before the module's lazy parser arena is allocated. */
    while (n < sizeof(held) / sizeof(held[0]))
    {
        void *memory = rt_memheap_alloc(&app_image_psram_memheap, 16u * 1024u);
        if (memory == NULL) break;
        held[n++] = memory;
    }
    rt_memory_info(&total, &before, &peak);
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)payload, sizeof(payload) - 1);
    apply_pending_instruction_batch();
    rt_memory_info(&total, &after, &peak);
    while (n) rt_memheap_free(held[--n]);
    if (return_total_list_count() != old_count ||
        !instruction_list_get_status().needs_sync || after > before + 4096u)
    {
        list_tdd_report("LIST_TDD FAIL exhausted_psram_keeps_the_model_and_system_heap delta=%ld\n",
                        (long)after - (long)before);
        return false;
    }
    list_tdd_report("LIST_TDD PASS exhausted_psram_keeps_the_model_and_system_heap\n");
    return true;
}
#endif

static void search_keeps_phone_remove_pending(void *unused)
{
    char batch[] = "[{\"id\":\"tdd-keep\",\"title\":\"Keep\",\"cat\":\"/\"}]";
    char removed[] = "tdd-keep";
    uint8_t before;
    const char *id;
    (void)unused;
    instruction_list_set_text_filter("");
#ifndef BSP_USING_PC_SIMULATOR
    if (!exhausted_psram_keeps_the_model_and_system_heap()) return;
#endif
    resolve_Notify_command(KEY_SKAI_CREATION_INSTRUCTIONS_BATCH,
                           (uint8_t *)batch, (uint16_t)strlen(batch));
    apply_pending_instruction_batch();
    instruction_list_set_text_filter("Keep");
    before = return_total_list_count();
    resolve_Notify_command(KEY_DISMISS_SKAI_INSTRUCTION,
                           (uint8_t *)removed, (uint16_t)strlen(removed));
    apply_pending_instruction_batch();
    id = instruction_list_export_id(0);
    if (before != 1 || return_total_list_count() != 1 ||
        id == NULL || strcmp(id, "tdd-keep") != 0)
    {
        list_tdd_report("LIST_TDD FAIL search_keeps_phone_remove_pending before=%u after=%u\n",
                   (unsigned)before, (unsigned)return_total_list_count());
        return;
    }
    instruction_list_set_text_filter("");
    for (before = 0; before < return_total_list_count(); before++)
    {
        id = instruction_list_export_id(before);
        if (id != NULL && strcmp(id, "tdd-keep") == 0)
        {
            list_tdd_report("LIST_TDD FAIL removal_resumes_after_search\n");
            return;
        }
    }
    list_tdd_report("LIST_TDD PASS search_keeps_phone_remove_pending\n");
    invalid_batch_preserves_committed_list();
}

static int list_tdd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    lv_async_call(search_keeps_phone_remove_pending, NULL);
    return 0;
}
MSH_CMD_EXPORT(list_tdd, list consistency behavior check);

static void list_tdd_autorun(void *unused)
{
    (void)unused;
    rt_thread_mdelay(5000);
#ifndef BSP_USING_PC_SIMULATOR
    {
        extern void watch_system_wakeup(void);
        watch_system_wakeup();
        list_tdd_report("LIST_TDD START physical_watch\n");
    }
#endif
    lv_async_call(search_keeps_phone_remove_pending, NULL);
#ifndef BSP_USING_PC_SIMULATOR
    /* Test images must not depend on an attached phone to wake the GUI.
       Bounded to three minutes; normal images never include this test. */
    {
        unsigned i;
        extern void watch_system_wakeup(void);
        for (i = 0; i < 180; i++)
        {
            rt_thread_mdelay(1000);
            watch_system_wakeup();
        }
    }
#endif
}

static int list_tdd_init(void)
{
#ifdef BSP_USING_PC_SIMULATOR
    if (getenv("SKAI_LIST_TDD") != NULL)
#endif
    {
        rt_thread_t thread = rt_thread_create("list_tdd", list_tdd_autorun,
                                             NULL, 4096, 20, 10);
        if (thread == NULL)
            return -1;
        rt_thread_startup(thread);
    }
    return 0;
}
INIT_APP_EXPORT(list_tdd_init);
