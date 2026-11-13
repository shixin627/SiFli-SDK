/* SPDX-License-Identifier: Apache-2.0 */
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "lvgl.h"
#include "bloc_notification.h"
#include "notification_reply.h"
#include "bloc_v2t.h"
#include "watch_system_interact.h"
#include "gui_app_fwk.h"
#include "ui_handler.h"
#include "instruction_list.h"
#include "communicate_parse.h"
#include "communicate_protocol.h"
#include "lv_left_cards.h"

static void report(const char *line)
{
#ifdef BSP_USING_PC_SIMULATOR
    rt_kprintf("%s\n", line);
#else
    rt_device_t uart = rt_device_find("uart1");
    if (uart) { rt_device_write(uart, 0, line, strlen(line)); rt_device_write(uart, 0, "\n", 1); }
#endif
}
static bool legacy_phone_cannot_start_a_reply(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-tdd-legacy");
    strcpy(notification.reply_instance, "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    notification.reply_revision = 1;
    notification.can_reply = true;
    set_notification(&notification, 0);
    notification_items_amount = 1;
    notification_reply_peer(true, false);
    return !notification_reply_available() && notification_reply_begin(&notification) == 0;
}
static bool unscoped_legacy_intent_cannot_bypass_reply_confirmation(void)
{
    notification_t notification = {0};
    char text[] = "Reply preserved";
    strcpy(notification.id, "reply-tdd-failed-send");
    strcpy(notification.title, "Reply test");
    strcpy(notification.message, "Original notification");
    notification.can_reply = true;
    set_notification(&notification, 0);
    notification_items_amount = 1;
#ifdef BSP_USING_PC_SIMULATOR
    extern unsigned pc_legacy_reply_sends;
    unsigned before = pc_legacy_reply_sends;
#endif
    handle_user_speech_intent(V2T_INTENT_REMOTE_INPUT, text);
#ifdef BSP_USING_PC_SIMULATOR
    if (pc_legacy_reply_sends != before) return false;
#endif
    return notification_center_get_info_count() == 1 &&
        strcmp(get_notification(0)->id, notification.id) == 0;
}
static bool stale_notification_cannot_begin_a_reply(void)
{
    notification_t old = *get_notification(0), current = old;
    notification_reply_peer(true, true);
    strcpy(old.reply_instance, "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    old.reply_revision = 1;
    current = old;
    current.reply_revision = 2;
    set_notification(&current, 0);
    return notification_reply_begin(&old) == 0 && notification_reply_begin(&current) != 0;
}
static bool cancelled_input_cannot_submit_a_late_transcript(void)
{
    uint32_t input = notification_reply_begin(get_notification(0));
    if (!input) return false;
    notification_reply_cancel(input);
    return notification_reply_submit(input, "Late transcript") == NOTIFICATION_REPLY_INVALID &&
        notification_center_get_info_count() == 1;
}
static bool target_change_during_input_rejects_submission(void)
{
    notification_t current = *get_notification(0);
    uint32_t input = notification_reply_begin(&current);
    if (!input) return false;
    current.reply_revision++;
    set_notification(&current, 0);
    return notification_reply_submit(input, "Stale reply") == NOTIFICATION_REPLY_INVALID &&
        notification_center_get_info_count() == 1;
}
static unsigned send_count, new_id_count;
static uint32_t test_now, submitted_input, expired_input;
static char sent_json[3584];
static bool test_ready = true;
static const char *observed_target;
static unsigned observed_dismisses, observed_queries;
static bool ready(void) { return test_ready; }
static uint32_t now_ms(void) { return test_now; }
static bool send_json(const char *json, uint16_t length)
{
    send_count++;
    if (length >= sizeof(sent_json)) return false;
    memcpy(sent_json, json, length); sent_json[length] = 0;
    if (observed_target && strstr(sent_json, observed_target)) {
        if (strstr(sent_json, "\"op\":\"dismiss\"")) observed_dismisses++;
        if (strstr(sent_json, "\"op\":\"query\"")) observed_queries++;
    }
    return true;
}
static bool new_id(char output[33])
{
    rt_snprintf(output, 33, "%032lx", (unsigned long)++new_id_count);
    return true;
}
static bool sixteen_pending_replies_preserve_the_first_draft_when_full(void)
{
    notification_reply_transport_t adapter = {now_ms, ready, ready, send_json, new_id};
    notification_reply_bind_transport(&adapter); notification_reply_peer(true, true);
    unsigned requests[16];
    notification_t notification = {0};
    notification.can_reply = true; notification.reply_revision = 1;
    for (unsigned i = 0; i < 17; i++) {
        rt_snprintf(notification.id, sizeof(notification.id), "full-work-%u", i);
        rt_snprintf(notification.reply_instance, sizeof(notification.reply_instance), "%032lx", (unsigned long)(0x31000000u + i));
        set_notification(&notification, 0); notification_items_amount = 1;
        uint32_t input = notification_reply_begin(&notification);
        unsigned before = send_count;
        notification_reply_state_t state = notification_reply_submit(input, "Held reply");
        if (i < 16) { if (state != NOTIFICATION_REPLY_WAITING) return false; requests[i] = new_id_count; }
        else if (state != NOTIFICATION_REPLY_FULL || send_count != before) return false;
    }
    char instance[33]; rt_snprintf(instance, sizeof(instance), "%032lx", (unsigned long)0x31000000u);
    notification_reply_view_t view = notification_reply_open_version(instance, 1);
    if (view.state != NOTIFICATION_REPLY_WAITING || !view.draft || strcmp(view.draft, "Held reply")) return false;
    for (unsigned i = 0; i < 16; i++) {
        char result[256];
        rt_snprintf(result, sizeof(result), "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"%032lx\",\"v\":1,\"state\":\"sent\"}",
            (unsigned long)requests[i], (unsigned long)(0x31000000u + i));
        if (!notification_reply_receive(result, strlen(result))) return false;
        notification_reply_process();
    }
    notification_reply_cancel(view.input); notification_items_amount = 0;
    return true;
}
static bool accepted_submission_waits_for_phone_confirmation(void)
{
    notification_reply_transport_t adapter = {now_ms, ready, ready, send_json, new_id};
    notification_reply_bind_transport(&adapter);
    uint32_t input = notification_reply_begin(get_notification(0));
    submitted_input = input;
    send_count = 0;
    return input && notification_reply_submit(input, "Reply with \"quotes\"") == NOTIFICATION_REPLY_WAITING &&
        send_count == 1 && strstr(sent_json, "\"op\":\"reply\"") &&
        strstr(sent_json, get_notification(0)->reply_instance) && notification_center_get_info_count() == 1;
}
static bool only_the_matching_phone_result_removes_the_notification(void)
{
    char result[256];
    const notification_t *notification = get_notification(0);
    rt_snprintf(result, sizeof(result),
        "{\"op\":\"result\",\"r\":\"ffffffffffffffffffffffffffffffff\",\"n\":\"%s\",\"v\":%lu,\"state\":\"sent\",\"c\":\"pending\"}",
        notification->reply_instance, (unsigned long)notification->reply_revision);
    notification_reply_receive(result, strlen(result));
    notification_reply_process();
    if (notification_reply_state(submitted_input) != NOTIFICATION_REPLY_WAITING || notification_center_get_info_count() != 1) return false;
    rt_snprintf(result, sizeof(result),
        "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"%s\",\"v\":%lu,\"state\":\"sent\",\"c\":\"pending\"}",
        (unsigned long)new_id_count, notification->reply_instance, (unsigned long)notification->reply_revision);
    notification_reply_receive(result, strlen(result));
    notification_reply_process();
    return notification_reply_state(submitted_input) == NOTIFICATION_REPLY_SENT && notification_center_get_info_count() == 0;
}
static bool lost_result_queries_with_backoff_without_resending_text(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-query");
    strcpy(notification.reply_instance, "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    test_now = 0;
    uint32_t input = notification_reply_begin(&notification);
    if (notification_reply_submit(input, "Only once") != NOTIFICATION_REPLY_WAITING) return false;
    expired_input = input;
    unsigned before = send_count;
    test_now = 1999; notification_reply_process();
    if (send_count != before) return false;
    test_now = 2000; notification_reply_process();
    if (send_count != before + 1 || !strstr(sent_json, "\"op\":\"query\"") || strstr(sent_json, "Only once")) return false;
    test_now = 6999; notification_reply_process();
    if (send_count != before + 1) return false;
    test_now = 7000; notification_reply_process();
    if (send_count != before + 2) return false;
    test_now = 600000; notification_reply_process();
    return send_count == before + 2 && notification_reply_state(input) == NOTIFICATION_REPLY_UNKNOWN;
}
static bool notification_ingress_refreshes_reply_target_without_changing_visible_content(void)
{
    notification_items_amount = 0;
    handle_notification(Notify_others, "{\"id\":\"ingress\",\"title\":\"Same\",\"message\":\"Same\",\"reply\":true,\"ri\":\"cccccccccccccccccccccccccccccccc\",\"rv\":1}");
    const notification_t *current = get_notification(0);
    if (!current || strcmp(current->reply_instance, "cccccccccccccccccccccccccccccccc") || current->reply_revision != 1) return false;
    uint32_t input = notification_reply_begin(current);
    handle_notification(Notify_others, "{\"id\":\"ingress\",\"title\":\"Same\",\"message\":\"Same\",\"reply\":true,\"ri\":\"cccccccccccccccccccccccccccccccc\",\"rv\":2,\"options\":[\"New\"]}");
    current = get_notification(0);
    return notification_items_amount == 1 && current->reply_revision == 2 && current->option_count == 1 &&
        !strcmp(current->options[0], "New") && notification_reply_submit(input, "Old page") == NOTIFICATION_REPLY_INVALID;
}
static bool old_revision_cannot_replace_a_newer_notification(void)
{
    handle_notification(Notify_others, "{\"id\":\"ingress\",\"title\":\"Old\",\"message\":\"Old\",\"reply\":true,\"ri\":\"cccccccccccccccccccccccccccccccc\",\"rv\":1}");
    return notification_items_amount == 1 && get_notification(0)->reply_revision == 2 && !strcmp(get_notification(0)->title, "Same");
}
static bool identical_content_from_different_instances_stays_distinct(void)
{
    handle_notification(Notify_others, "{\"id\":\"another\",\"title\":\"Same\",\"message\":\"Same\",\"reply\":true,\"ri\":\"dddddddddddddddddddddddddddddddd\",\"rv\":1}");
    return notification_items_amount == 2 && !strcmp(get_notification(0)->reply_instance, "dddddddddddddddddddddddddddddddd");
}
static bool option_reply_keeps_notification_and_suppresses_a_second_press(void)
{
    unsigned before = send_count;
    const notification_t *notification = get_notification(0);
    if (notification_reply_option(notification, "Yes") != NOTIFICATION_REPLY_WAITING || send_count != before + 1 || notification_items_amount != 2) return false;
    return notification_reply_option(notification, "No") == NOTIFICATION_REPLY_WAITING && send_count == before + 1;
}
static bool malformed_result_cannot_remove_a_notification(void)
{
    char result[400];
    rt_snprintf(result, sizeof(result),
        "{\"op\":\"result\",\"r\":\"%032lx\",\"r\":\"%032lx\",\"n\":\"dddddddddddddddddddddddddddddddd\",\"v\":1,\"state\":\"sent\"}",
        (unsigned long)new_id_count, (unsigned long)new_id_count);
    if (notification_reply_receive(result, strlen(result))) return false;
    const char *invalid_fields[] = {",\"m\":\"Unexpected reply text\"", ",\"c\":42", ",\"c\":\"invalid_closure\""};
    for (unsigned i = 0; i < sizeof(invalid_fields) / sizeof(invalid_fields[0]); i++) {
        rt_snprintf(result, sizeof(result),
            "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"dddddddddddddddddddddddddddddddd\",\"v\":1,\"state\":\"sent\"%s}",
            (unsigned long)new_id_count, invalid_fields[i]);
        if (notification_reply_receive(result, strlen(result))) return false;
    }
    notification_reply_process();
    return notification_items_amount == 2;
}
static bool new_input_cannot_resend_a_pending_target(void)
{
    return notification_reply_begin(get_notification(0)) == 0 && notification_items_amount == 2;
}
static lv_obj_t *find_text(lv_obj_t *root, const char *text)
{
    if (lv_obj_check_type(root, &lv_label_class) && !strcmp(lv_label_get_text(root), text)) return root;
    for (unsigned i = 0; i < lv_obj_get_child_cnt(root); i++) {
        lv_obj_t *found = find_text(lv_obj_get_child(root, i), text);
        if (found) return found;
    }
    return NULL;
}
static unsigned ui_sends;
static void begin_speech_ui_test(void);
static void begin_card_ui_test(lv_timer_t *timer);
static void begin_reopened_message_test(lv_timer_t *timer);
static void begin_reopened_speech_test(lv_timer_t *timer);
static uint32_t background_input;
static void background_result_test(lv_timer_t *timer)
{
    lv_timer_del(timer);
    bool passed = notification_reply_state(background_input) == NOTIFICATION_REPLY_SENT && notification_items_amount == 0;
    report(passed ? "REPLY_TDD PASS result_completes_after_leaving_the_reply_page" : "REPLY_TDD FAIL result_completes_after_leaving_the_reply_page");
    if (passed) begin_speech_ui_test();
}
static void begin_background_result_test(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-background"); strcpy(notification.reply_instance, "ffffffffffffffffffffffffffffffff");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    background_input = notification_reply_begin(&notification);
    notification_reply_submit(background_input, "Background reply");
    notification_reply_cancel(background_input);
    char result[256];
    rt_snprintf(result, sizeof(result),
        "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"ffffffffffffffffffffffffffffffff\",\"v\":1,\"state\":\"sent\"}",
        (unsigned long)new_id_count);
    notification_reply_receive(result, strlen(result));
    lv_timer_create(background_result_test, 1000, NULL);
}
#ifndef BSP_USING_PC_SIMULATOR
static bool real_identifier_allows_offline_dismiss(void)
{
    notification_reply_link(false);
    notification_reply_bind_transport(NULL);
    notification_t notification = {0};
    strcpy(notification.id, "reply-real-id"); strcpy(notification.title, "Identifier fixture");
    strcpy(notification.reply_instance, "dbdbdbdbdbdbdbdbdbdbdbdbdbdbdbdb"); notification.reply_revision = 1;
    set_notification(&notification, 0); notification_items_amount = 1;
    return notification_reply_dismiss(&notification) && notification_items_amount == 0;
}
#ifdef NOTIFICATION_REPLY_BLE_TEST
static char ble_instance[33];
static uint32_t ble_revision;
static rt_tick_t ble_deadline;
#ifdef NOTIFICATION_REPLY_VOICE_TEST
static uint8_t ble_voice_stage;
static rt_tick_t ble_voice_changed, ble_voice_stopped;
static char ble_voice_text[512];
static bool ble_voice_has_text(const char *text)
{
    for (const unsigned char *p = (const unsigned char *)text; *p; p++)
        if (*p > ' ') return true;
    return false;
}
static void ble_voice_report_visible(const char *text)
{
    static const char digits[] = "0123456789abcdef";
    size_t length = strlen(text);
    char line[144];
    rt_snprintf(line, sizeof(line), "REPLY_VOICE_VISIBLE_BYTES=%u", (unsigned)length);
    report(line);
    for (size_t offset = 0; offset < length; offset += 32) {
        unsigned used = rt_snprintf(line, sizeof(line), "REPLY_VOICE_VISIBLE_HEX offset=%u data=", (unsigned)offset);
        for (size_t i = offset; i < length && i < offset + 32; i++) {
            unsigned byte = (unsigned char)text[i];
            line[used++] = digits[byte >> 4]; line[used++] = digits[byte & 15];
        }
        line[used] = 0; report(line);
    }
}
static bool ble_voice_owns_page(void)
{
    if (!gui_app_is_actived(APP_ID_SPEECH)) return false;
    const char *instance = intent_get_string(gui_app_get_intent(), "reply_instance");
    return instance && !strcmp(instance, ble_instance);
}
static void ble_voice_fail(lv_timer_t *timer, const char *reason)
{
    report(reason);
    if (ble_voice_owns_page()) {
        stop_voice_recognition(V2T_INTENT_NOTHING);
        gui_app_exit(APP_ID_SPEECH);
    }
    lv_timer_del(timer);
}
static void ble_voice_poll(lv_timer_t *timer)
{
    if (!ble_voice_owns_page()) {
        if (ble_voice_stage > 1) ble_voice_fail(timer, "REPLY_BLE FAIL voice_page_left_before_confirmation");
        return;
    }
    if (ble_voice_stage == 1) {
        if (!app_voice_get_listening_status()) return;
        report("REPLY_BLE VOICE_READY microphone_active");
        ble_voice_stage = 2;
    }
    const char *text = get_combined_voice2text();
    if (ble_voice_stage == 2) {
        if (strcmp(ble_voice_text, text)) {
            rt_snprintf(ble_voice_text, sizeof(ble_voice_text), "%s", text);
            ble_voice_changed = rt_tick_get();
        }
        if (!ble_voice_has_text(text) || rt_tick_get() - ble_voice_changed < rt_tick_from_millisecond(2000)) return;
        stop_voice_recognition(V2T_INTENT_NOTHING);
        ble_voice_stopped = rt_tick_get(); ble_voice_stage = 3;
        report("REPLY_BLE VOICE_STOPPED waiting_for_final_text");
        return;
    }
    if (ble_voice_stage != 3 || rt_tick_get() - ble_voice_stopped < rt_tick_from_millisecond(6500)) return;
    /* The ASR stream's finalization deadline is 5 s. Accept its final full
       text only through the existing scoped delivery and visible Speech UI. */
    lv_obj_t *label = find_text(lv_scr_act(), text);
    if (!ble_voice_has_text(text) || !label || !lv_obj_is_visible(label)) {
        ble_voice_fail(timer, "REPLY_BLE FAIL scoped_voice_text_not_visible"); return;
    }
    report("REPLY_BLE PASS scoped_voice_transcript_visible");
    ble_voice_report_visible(text);
    uint32_t input = intent_get_uint32(gui_app_get_intent(), "reply_input", 0);
    if (!notification_reply_can_submit(input) || !lvgl_msg_handler.handle_tap_indicator) {
        ble_voice_fail(timer, "REPLY_BLE FAIL voice_send_unavailable"); return;
    }
    lvgl_msg_handler.handle_tap_indicator(1);
    notification_reply_state_t state = notification_reply_state(input);
    if (state != NOTIFICATION_REPLY_WAITING && state != NOTIFICATION_REPLY_UNKNOWN) {
        ble_voice_fail(timer, "REPLY_BLE FAIL voice_submission_rejected"); return;
    }
    ble_voice_stage = 4;
    report("REPLY_BLE VOICE_SUBMITTED existing_speech_send_handler");
}
#endif
static void ble_fixture(lv_timer_t *timer)
{
    notification_reply_process();
    if (ble_revision && notification_reply_version_state(ble_instance, ble_revision) == NOTIFICATION_REPLY_SENT) {
#ifdef NOTIFICATION_REPLY_VOICE_TEST
        if (ble_voice_stage != 5) {
            report("REPLY_BLE PASS watch_received_dispatch_confirmation"); ble_voice_stage = 5;
        }
        if (ble_voice_owns_page()) {
            if ((rt_tick_t)(rt_tick_get() - ble_deadline) < (rt_tick_t)0x80000000u)
                ble_voice_fail(timer, "REPLY_BLE FAIL confirmed_voice_page_did_not_close");
            return;
        }
        report("REPLY_BLE PASS confirmed_voice_reply_closed_its_page");
#else
        report("REPLY_BLE PASS watch_received_dispatch_confirmation");
#endif
        report("REPLY_BLE DONE"); lv_timer_del(timer); return;
    }
    if ((rt_tick_t)(rt_tick_get() - ble_deadline) < (rt_tick_t)0x80000000u) {
#ifdef NOTIFICATION_REPLY_VOICE_TEST
        ble_voice_fail(timer, "REPLY_BLE FAIL round_trip_timeout"); return;
#else
        report("REPLY_BLE FAIL round_trip_timeout"); lv_timer_del(timer); return;
#endif
    }
#ifdef NOTIFICATION_REPLY_VOICE_TEST
    if (ble_revision) { ble_voice_poll(timer); return; }
#endif
    if (ble_revision || !notification_reply_available()) return;
    for (unsigned i = 0; i < notification_items_amount; i++) {
        const notification_t *notification = get_notification(i);
        if (strncmp(notification->title, "Reply BLE TDD", 13) || !notification->reply_revision ||
            (!strcmp(ble_instance, notification->reply_instance) && ble_revision == notification->reply_revision)) continue;
        char instance[33]; memcpy(instance, notification->reply_instance, 33);
        uint32_t revision = notification->reply_revision;
#ifdef NOTIFICATION_REPLY_VOICE_TEST
        if (strncmp(notification->title, "Reply BLE TDD Voice", 19)) continue;
        memcpy(ble_instance, instance, 33); ble_revision = revision;
        ble_voice_stage = 1;
        notify_provider.navigate_to_reply((notification_t *)notification);
#else
        notification_reply_state_t state = notification_reply_option(notification, "Fixture reply");
        char line[96]; rt_snprintf(line, sizeof(line), "REPLY_BLE submitted state=%u", (unsigned)state); report(line);
        if (state != NOTIFICATION_REPLY_WAITING && state != NOTIFICATION_REPLY_UNKNOWN) {
            report("REPLY_BLE FAIL submission_rejected"); lv_timer_del(timer); return;
        }
        memcpy(ble_instance, instance, 33); ble_revision = revision;
#endif
        return;
    }
}
static void begin_ble_fixture(void *unused)
{
    (void)unused;
    notification_reply_bind_transport(NULL);
#ifdef NOTIFICATION_REPLY_VOICE_TEST
    ble_deadline = rt_tick_get() + rt_tick_from_millisecond(300000);
    report("REPLY_BLE READY voice_real_transport_only");
#else
    ble_deadline = rt_tick_get() + rt_tick_from_millisecond(180000);
    report("REPLY_BLE READY real_transport_only");
#endif
    lv_timer_create(ble_fixture, 250, NULL);
}
#endif
#endif
static void message_confirmation_test(lv_timer_t *timer)
{
    lv_timer_del(timer);
    bool passed = notification_items_amount == 1 && !find_text(lv_scr_act(), "Reply TDD Yes");
    report(passed ? "REPLY_TDD PASS message_confirmation_finishes_its_own_page" : "REPLY_TDD FAIL message_confirmation_finishes_its_own_page");
    gui_app_exit(APP_ID_MESSAGE);
    if (passed) {
        begin_background_result_test();
    }
}
static void message_option_test(lv_timer_t *timer)
{
    lv_timer_del(timer);
    setVoice2Text("Kept draft");
    VOICE_RECOGNITION_PAYLOAD legacy = {0};
    legacy.header = get_speech_coding(); legacy.length = 10; legacy.p_msg_value = (uint8_t *)"Late words";
    interact_voice_recognition(&legacy);
    bool scoped = !strcmp(get_combined_voice2text(), "Kept draft");
    report(scoped ? "REPLY_TDD PASS unscoped_transcript_cannot_change_notification_draft" : "REPLY_TDD FAIL unscoped_transcript_cannot_change_notification_draft");
    if (!scoped) return;
    clearVoice2Text();
    lv_obj_t *label = find_text(lv_scr_act(), "Reply TDD Yes");
    if (label) lv_event_send(lv_obj_get_parent(label), LV_EVENT_CLICKED, NULL);
    bool passed = label && notification_items_amount == 2 && send_count == ui_sends + 1 && strstr(sent_json, "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee");
    report(passed ? "REPLY_TDD PASS message_option_waits_for_confirmation" : "REPLY_TDD FAIL message_option_waits_for_confirmation");
    if (!passed) { gui_app_exit(APP_ID_MESSAGE); return; }
    char result[256];
    rt_snprintf(result, sizeof(result),
        "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee\",\"v\":1,\"state\":\"sent\"}",
        (unsigned long)new_id_count);
    notification_reply_receive(result, strlen(result));
    notification_reply_process();
    lv_timer_create(message_confirmation_test, 1000, NULL);
}
static void begin_message_ui_test(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-ui"); strcpy(notification.title, "Reply UI");
    strcpy(notification.message, "Original UI notification");
    strcpy(notification.reply_instance, "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee");
    notification.reply_revision = 1; notification.can_reply = true;
    notification.option_count = 1; strcpy(notification.options[0], "Reply TDD Yes");
    set_notification(&notification, 1);
    notification_t other = notification;
    strcpy(other.reply_instance, "99999999999999999999999999999999");
    strcpy(other.options[0], "Wrong notification");
    set_notification(&other, 0); notification_items_amount = 2;
    ui_sends = send_count;
    navigate_notification_info(&notification);
    lv_timer_create(message_option_test, 1500, NULL);
}
static void speech_confirmation_test(lv_timer_t *timer)
{
    lv_timer_del(timer);
    bool passed = notification_items_amount == 0 && !gui_app_is_actived(APP_ID_SPEECH);
    report(passed ? "REPLY_TDD PASS speech_confirmation_finishes_its_own_page" : "REPLY_TDD FAIL speech_confirmation_finishes_its_own_page");
    gui_app_exit(APP_ID_SPEECH);
    if (passed) lv_timer_create(begin_reopened_message_test, 500, NULL);
}
static unsigned reopened_request, reopened_sends;
static void reopened_message_completed(lv_timer_t *timer)
{
    lv_timer_del(timer);
    bool passed = gui_app_is_actived(APP_ID_MESSAGE) && notification_items_amount == 0 && send_count == reopened_sends;
    report(passed ? "REPLY_TDD PASS old_confirmation_keeps_reopened_page" : "REPLY_TDD FAIL old_confirmation_keeps_reopened_page");
    gui_app_exit(APP_ID_MESSAGE);
    test_ready = true;
    if (passed) lv_timer_create(begin_reopened_speech_test, 500, NULL);
}
static void reopened_speech_completed(lv_timer_t *timer)
{
    lv_timer_del(timer);
    bool passed = gui_app_is_actived(APP_ID_SPEECH) && notification_items_amount == 0 && send_count == reopened_sends;
    report(passed ? "REPLY_TDD PASS old_confirmation_keeps_reopened_speech" : "REPLY_TDD FAIL old_confirmation_keeps_reopened_speech");
    gui_app_exit(APP_ID_SPEECH); test_ready = true;
    if (passed) lv_timer_create(begin_card_ui_test, 500, NULL);
}
static void reopened_speech_query(lv_timer_t *timer)
{
    lv_timer_del(timer);
    lv_obj_t *label = find_text(lv_scr_act(), "查詢狀態");
    bool visible = label && !lv_obj_has_flag(lv_obj_get_parent(label), LV_OBJ_FLAG_HIDDEN);
    test_ready = true;
    if (visible) lv_event_send(lv_obj_get_parent(label), LV_EVENT_CLICKED, NULL);
    bool passed = visible && send_count == reopened_sends + 1 && strstr(sent_json, "\"op\":\"query\"") && !strstr(sent_json, "Retained speech draft");
    report(passed ? "REPLY_TDD PASS expired_speech_can_query_without_resending" : "REPLY_TDD FAIL expired_speech_can_query_without_resending");
    if (!passed) return;
    reopened_sends = send_count; test_ready = false;
    char result[256];
    rt_snprintf(result, sizeof(result), "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"93939393939393939393939393939393\",\"v\":1,\"state\":\"sent\"}", (unsigned long)reopened_request);
    notification_reply_receive(result, strlen(result)); notification_reply_process();
    lv_timer_create(reopened_speech_completed, 1000, NULL);
}
static void reopened_speech_visible(lv_timer_t *timer)
{
    lv_timer_del(timer);
    bool passed = gui_app_is_actived(APP_ID_SPEECH) && !strcmp(get_combined_voice2text(), "Retained speech draft") &&
        find_text(lv_scr_act(), "Retained speech draft") && send_count == reopened_sends;
    report(passed ? "REPLY_TDD PASS reopened_speech_shows_frozen_draft" : "REPLY_TDD FAIL reopened_speech_shows_frozen_draft");
    if (!passed) return;
    test_now += 600000; notification_reply_process();
    lv_timer_create(reopened_speech_query, 500, NULL);
}
static void begin_reopened_speech_test(lv_timer_t *timer)
{
    lv_timer_del(timer);
    notification_t notification = {0};
    strcpy(notification.id, "reopened-speech"); strcpy(notification.title, "Reopened speech");
    strcpy(notification.reply_instance, "93939393939393939393939393939393");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    uint32_t input = notification_reply_begin(&notification);
    notification_reply_submit(input, "Retained speech draft"); notification_reply_cancel(input);
    reopened_request = new_id_count; reopened_sends = send_count; test_ready = false;
    notify_provider.navigate_to_reply(&notification);
    lv_timer_create(reopened_speech_visible, 1500, NULL);
}
static void reopened_message_query(lv_timer_t *timer)
{
    lv_timer_del(timer);
    lv_obj_t *label = find_text(lv_scr_act(), "查詢狀態");
    bool visible = label && !lv_obj_has_flag(lv_obj_get_parent(label), LV_OBJ_FLAG_HIDDEN);
    test_ready = true;
    if (visible) lv_event_send(lv_obj_get_parent(label), LV_EVENT_CLICKED, NULL);
    bool passed = visible && send_count == reopened_sends + 1 && strstr(sent_json, "\"op\":\"query\"") && !strstr(sent_json, "Retained pending draft");
    report(passed ? "REPLY_TDD PASS expired_message_can_query_without_resending" : "REPLY_TDD FAIL expired_message_can_query_without_resending");
    if (!passed) return;
    reopened_sends = send_count; test_ready = false;
    char result[256];
    rt_snprintf(result, sizeof(result), "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"92929292929292929292929292929292\",\"v\":1,\"state\":\"sent\"}", (unsigned long)reopened_request);
    notification_reply_receive(result, strlen(result)); notification_reply_process();
    lv_timer_create(reopened_message_completed, 1000, NULL);
}
static void reopened_message_visible(lv_timer_t *timer)
{
    lv_timer_del(timer);
    bool passed = gui_app_is_actived(APP_ID_MESSAGE) && !strcmp(get_combined_voice2text(), "Retained pending draft") &&
        find_text(lv_scr_act(), "Retained pending draft") && send_count == reopened_sends;
    report(passed ? "REPLY_TDD PASS reopened_message_shows_frozen_draft" : "REPLY_TDD FAIL reopened_message_shows_frozen_draft");
    if (!passed) return;
    notification_reply_peer(true, true);
    test_now += 600000; notification_reply_process();
    lv_timer_create(reopened_message_query, 500, NULL);
}
static void begin_reopened_message_test(lv_timer_t *timer)
{
    lv_timer_del(timer);
    notification_t notification = {0};
    strcpy(notification.id, "reopened-message"); strcpy(notification.title, "Reopened notification");
    strcpy(notification.reply_instance, "92929292929292929292929292929292");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    uint32_t input = notification_reply_begin(&notification);
    notification_reply_submit(input, "Retained pending draft");
    notification_reply_cancel(input);
    reopened_request = new_id_count; reopened_sends = send_count;
    test_ready = false;
    notification_reply_link(false);
    navigate_notification_info(&notification);
    lv_timer_create(reopened_message_visible, 1500, NULL);
}
#ifndef BSP_USING_PC_SIMULATOR
static void call_removal_finished(lv_timer_t *timer)
{
    lv_timer_del(timer);
    bool passed = !gui_app_is_actived(APP_ID_INCOMING_CALL) && notification_items_amount == 0;
    report(passed ? "REPLY_TDD PASS versioned_call_removal_closes_the_incoming_call_page" : "REPLY_TDD FAIL versioned_call_removal_closes_the_incoming_call_page");
    if (passed) {
        bool identifier = real_identifier_allows_offline_dismiss();
        report(identifier ? "REPLY_TDD PASS real_identifier_allows_offline_dismiss" : "REPLY_TDD FAIL real_identifier_allows_offline_dismiss");
        if (identifier) report("REPLY_TDD DONE");
    }
}
static void call_removal_deliver(lv_timer_t *timer)
{
    lv_timer_del(timer);
    if (!gui_app_is_actived(APP_ID_INCOMING_CALL)) { report("REPLY_TDD FAIL incoming_call_setup"); return; }
    const char packet[] = "{\"op\":\"removed\",\"n\":\"dadadadadadadadadadadadadadadada\",\"v\":1}";
    notification_reply_receive(packet, strlen(packet)); notification_reply_process();
    lv_timer_create(call_removal_finished, 1000, NULL);
}
static void begin_call_removal_test(lv_timer_t *timer)
{
    lv_timer_del(timer);
    notification_t notification = {0};
    strcpy(notification.id, "reply-call-ui"); strcpy(notification.title, "Fixture call");
    strcpy(notification.reply_instance, "dadadadadadadadadadadadadadadada");
    notification.reply_revision = 1; notification.calling = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    extern void incoming_call_set_caller(const char *, const char *, uint8_t);
    incoming_call_set_caller("Fixture call", notification.id, 0);
    gui_app_run(APP_ID_INCOMING_CALL);
    lv_timer_create(call_removal_deliver, 1000, NULL);
}
#endif
static void card_confirmation_test(lv_timer_t *timer)
{
    lv_timer_del(timer);
    bool passed = notification_items_amount == 0 && !find_text(lv_layer_top(), "C-OK");
    report(passed ? "REPLY_TDD PASS card_confirmation_removes_its_options" : "REPLY_TDD FAIL card_confirmation_removes_its_options");
    instruction_list_close();
    if (passed) {
#ifdef BSP_USING_PC_SIMULATOR
        report("REPLY_TDD DONE");
#else
        lv_timer_create(begin_call_removal_test, 500, NULL);
#endif
    }
}
static unsigned card_request;
static void card_pending_test(lv_timer_t *timer)
{
    lv_timer_del(timer);
    lv_obj_t *label = find_text(lv_layer_top(), "C-OK");
    unsigned before = send_count;
    bool disabled = label && lv_obj_has_state(lv_obj_get_parent(label), LV_STATE_DISABLED);
    if (label) lv_event_send(lv_obj_get_parent(label), LV_EVENT_CLICKED, NULL);
    bool passed = disabled && send_count == before && notification_items_amount == 1;
    report(passed ? "REPLY_TDD PASS pending_card_options_are_disabled" : "REPLY_TDD FAIL pending_card_options_are_disabled");
    if (!passed) return;
    char result[256];
    rt_snprintf(result, sizeof(result),
        "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"70707070707070707070707070707070\",\"v\":1,\"state\":\"sent\"}",
        (unsigned long)card_request);
    notification_reply_receive(result, strlen(result)); notification_reply_process();
    lv_timer_create(card_confirmation_test, 1500, NULL);
}
static void card_submission_test(lv_timer_t *timer)
{
    lv_timer_del(timer);
    lv_obj_t *label = find_text(lv_layer_top(), "C-OK");
    unsigned before = send_count;
    if (label) lv_event_send(lv_obj_get_parent(label), LV_EVENT_CLICKED, NULL);
    bool passed = label && notification_items_amount == 1 && send_count == before + 1;
    report(passed ? "REPLY_TDD PASS card_submission_waits_for_phone_confirmation" : "REPLY_TDD FAIL card_submission_waits_for_phone_confirmation");
    if (!passed) return;
    card_request = new_id_count;
    lv_timer_create(card_pending_test, 500, NULL);
}
static void card_capability_lost(lv_timer_t *timer)
{
    lv_timer_del(timer);
    bool passed = !find_text(lv_layer_top(), "C-OK");
    report(passed ? "REPLY_TDD PASS card_options_follow_phone_capability" : "REPLY_TDD FAIL card_options_follow_phone_capability");
    notification_reply_peer(true, true);
    if (passed) lv_timer_create(card_submission_test, 1500, NULL);
}
static void card_capability_start(lv_timer_t *timer)
{
    lv_timer_del(timer);
    if (!find_text(lv_layer_top(), "C-OK")) {
        report("REPLY_TDD FAIL card_capability_setup"); return;
    }
    notification_reply_peer(true, false);
    lv_timer_create(card_capability_lost, 1500, NULL);
}
static void begin_card_ui_test(lv_timer_t *timer)
{
    lv_timer_del(timer);
    /* Mount the real host explicitly, as the list UI integration tests do. */
    extern lv_obj_t *lv_instruction_list_layout_create(lv_obj_t *parent);
    if (!lv_instruction_list_layout_create(lv_scr_act())) { report("REPLY_TDD FAIL card_host_setup"); return; }
    notification_t notification = {0};
    strcpy(notification.id, "reply-card-ui"); strcpy(notification.title, "Card UI"); strcpy(notification.message, "Card source");
    strcpy(notification.reply_instance, "70707070707070707070707070707070");
    notification.reply_revision = 1; notification.can_reply = true;
    notification.option_count = 1; strcpy(notification.options[0], "C-OK");
    set_notification(&notification, 0); notification_items_amount = 1;
    instruction_list_set_query(""); instruction_list_link_changed(true);
    extern void apply_pending_instruction_batch(void);
    apply_pending_instruction_batch();
    char json[] = "[]";
    instruction_list_delivery_t delivery = {KEY_SKAI_CREATION_INSTRUCTIONS_BATCH, json, strlen(json), NOTIFY_COMMAND_ID};
    instruction_list_submit_phone(&delivery);
    apply_pending_instruction_batch();
    instruction_list_open('@'); /* Initializes the borrowed session pool on a cold UI. */
    const char sessions[] = "{\"device\":\"reply-device\",\"sessions\":[{\"id\":\"conv:reply:tdd\",\"title\":\"Reply Bot\",\"preview\":\"Card source\",\"bot\":\"reply-bot\",\"glance\":true,\"ts\":4000000000}]}";
    extern void skai_sessions_on_conv_list(const uint8_t *, uint16_t);
    extern void session_pager_apply_pending(void);
    skai_sessions_on_conv_list((const uint8_t *)sessions, strlen(sessions));
    session_pager_apply_pending();
    instruction_list_open('@');
    extern uint8_t return_total_list_count(void);
    extern const char *instruction_list_export_id(uint8_t index);
    for (uint8_t i = 0; i < return_total_list_count(); i++)
        if (!strcmp(instruction_list_export_id(i), "conv:reply:tdd")) left_cards_scroll_to(i, false);
    lv_timer_create(card_capability_start, 1500, NULL);
}
static void speech_submission_test(lv_timer_t *timer)
{
    lv_timer_del(timer);
    test_ready = true;
    uint32_t input = intent_get_uint32(gui_app_get_intent(), "reply_input", 0);
    if (!notification_reply_listen(input)) { report("REPLY_TDD FAIL clear_speech_setup"); return; }
    unsigned context = new_id_count;
    setVoice2Text("Words to clear");
    if (lvgl_msg_handler.handle_back_event) lvgl_msg_handler.handle_back_event();
    char late[160];
    rt_snprintf(late, sizeof(late), "{\"op\":\"text\",\"c\":\"%032lx\",\"m\":\"Late cleared speech\"}", (unsigned long)context);
    notification_reply_receive(late, strlen(late)); notification_reply_process();
    bool cleared = isTextEmpty() && !find_text(lv_scr_act(), "Visible scoped speech");
    report(cleared ? "REPLY_TDD PASS cleared_speech_rejects_late_transcript" : "REPLY_TDD FAIL cleared_speech_rejects_late_transcript");
    if (!cleared) return;
    setVoice2Text("Speech UI reply");
    unsigned before = send_count;
    if (lvgl_msg_handler.handle_tap_indicator) lvgl_msg_handler.handle_tap_indicator(1);
    bool passed = notification_items_amount == 1 && send_count == before + 1 && !strcmp(get_combined_voice2text(), "Speech UI reply");
    report(passed ? "REPLY_TDD PASS speech_submission_keeps_text_until_confirmation" : "REPLY_TDD FAIL speech_submission_keeps_text_until_confirmation");
    if (!passed) { gui_app_exit(APP_ID_SPEECH); return; }
    char result[256];
    rt_snprintf(result, sizeof(result),
        "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"66666666666666666666666666666666\",\"v\":1,\"state\":\"sent\"}",
        (unsigned long)new_id_count);
    notification_reply_receive(result, strlen(result));
    notification_reply_process();
    lv_timer_create(speech_confirmation_test, 1000, NULL);
}
static void speech_live_text_test(lv_timer_t *timer)
{
    lv_timer_del(timer);
    bool visible = find_text(lv_scr_act(), "Visible scoped speech") != NULL;
    report(visible ? "REPLY_TDD PASS scoped_speech_is_visible_in_its_reply_page" : "REPLY_TDD FAIL scoped_speech_is_visible_in_its_reply_page");
    if (visible) lv_timer_create(speech_submission_test, 500, NULL);
}
static void speech_live_text_start(lv_timer_t *timer)
{
    lv_timer_del(timer);
    test_ready = true;
    uint32_t input = intent_get_uint32(gui_app_get_intent(), "reply_input", 0);
    if (!notification_reply_listen(input)) { report("REPLY_TDD FAIL live_speech_setup"); return; }
    char packet[160];
    rt_snprintf(packet, sizeof(packet), "{\"op\":\"text\",\"c\":\"%032lx\",\"m\":\"Visible scoped speech\"}", (unsigned long)new_id_count);
    notification_reply_receive(packet, strlen(packet)); notification_reply_process();
    lv_timer_create(speech_live_text_test, 500, NULL);
}
static void begin_speech_ui_test(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-speech-ui"); strcpy(notification.title, "Reply speech UI");
    strcpy(notification.reply_instance, "66666666666666666666666666666666");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    test_ready = false; /* Refuse microphone start; this checks UI with fixed text. */
    notify_provider.navigate_to_reply(&notification);
    lv_timer_create(speech_live_text_start, 1500, NULL);
}
static bool blank_or_invalid_utf8_text_is_rejected_before_send(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-text-check"); strcpy(notification.reply_instance, "11111111111111111111111111111111");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    uint32_t input = notification_reply_begin(&notification);
    unsigned before = send_count;
    const char invalid[] = {(char)0xc0, (char)0xaf, 0};
    return notification_reply_submit(input, " \t\n") == NOTIFICATION_REPLY_FAILED &&
        notification_reply_submit(input, invalid) == NOTIFICATION_REPLY_FAILED && send_count == before;
}
static bool reconnect_discards_previous_capability_and_queued_results(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-epoch"); strcpy(notification.reply_instance, "22222222222222222222222222222222");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    uint32_t input = notification_reply_begin(&notification);
    if (notification_reply_submit(input, "Epoch reply") != NOTIFICATION_REPLY_WAITING) return false;
    char result[256];
    rt_snprintf(result, sizeof(result),
        "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"22222222222222222222222222222222\",\"v\":1,\"state\":\"sent\"}",
        (unsigned long)new_id_count);
    notification_reply_receive(result, strlen(result));
    notification_reply_link(false); notification_reply_link(true);
    bool cleared = !notification_reply_available();
    notification_reply_peer(true, true);
    notification_reply_process();
    return cleared && notification_reply_state(input) == NOTIFICATION_REPLY_WAITING && notification_items_amount == 1;
}
static bool unversioned_dismiss_cannot_remove_a_modern_notification(void)
{
    dismiss_notification_from_phone("reply-epoch");
    return notification_items_amount == 1;
}
static bool versioned_removal_only_removes_the_matching_notification(void)
{
    const char *old = "{\"op\":\"removed\",\"n\":\"22222222222222222222222222222222\",\"v\":2}";
    if (!notification_reply_receive(old, strlen(old))) return false;
    notification_reply_process();
    if (notification_items_amount != 1) return false;
    const char *current = "{\"op\":\"removed\",\"n\":\"22222222222222222222222222222222\",\"v\":1}";
    if (!notification_reply_receive(current, strlen(current))) return false;
    notification_reply_process();
    return notification_items_amount == 0;
}
static bool manual_query_after_the_deadline_never_resends_reply_text(void)
{
    unsigned before = send_count;
    if (!notification_reply_query(expired_input) || send_count != before + 1 ||
        !strstr(sent_json, "\"op\":\"query\"") || strstr(sent_json, "Only once")) return false;
    notification_reply_process();
    return send_count == before + 1 && notification_reply_state(expired_input) == NOTIFICATION_REPLY_UNKNOWN;
}
static bool ordinary_dismiss_is_versioned_and_does_not_send_reply_text(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-dismiss"); strcpy(notification.reply_instance, "33333333333333333333333333333333");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    unsigned before = send_count;
    return notification_reply_dismiss(&notification) && notification_items_amount == 0 && send_count == before + 1 &&
        strstr(sent_json, "\"op\":\"dismiss\"") && strstr(sent_json, notification.reply_instance) && !strstr(sent_json, "\"m\":");
}
static bool offline_dismiss_keeps_current_experience_and_sends_after_reconnect(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-offline-dismiss"); strcpy(notification.reply_instance, "44444444444444444444444444444444");
    notification.reply_revision = 1; notification.can_reply = false;
    set_notification(&notification, 0); notification_items_amount = 1;
    notification_reply_link(false);
    unsigned before = send_count;
    bool removed = notification_reply_dismiss(&notification);
    bool deferred = removed && notification_items_amount == 0 && send_count == before;
    notification_reply_peer(true, true);
    notification_reply_process();
    return deferred && send_count > before && strstr(sent_json, "\"op\":\"dismiss\"") && strstr(sent_json, notification.reply_instance);
}
static bool dismissed_revision_cannot_reappear_but_a_new_revision_can(void)
{
    handle_notification(Notify_others, "{\"id\":\"reply-dismiss\",\"title\":\"Dismissed\",\"message\":\"Dismissed\",\"reply\":true,\"ri\":\"33333333333333333333333333333333\",\"rv\":1}");
    if (notification_items_amount) return false;
    handle_notification(Notify_others, "{\"id\":\"reply-dismiss\",\"title\":\"Updated\",\"message\":\"Updated\",\"reply\":true,\"ri\":\"33333333333333333333333333333333\",\"rv\":2}");
    return notification_items_amount == 1 && get_notification(0)->reply_revision == 2;
}
static bool confirmed_dismiss_releases_capacity_for_later_actions(void)
{
    for (unsigned i = 0; i < 20; i++) {
        notification_t notification = {0};
        rt_snprintf(notification.id, sizeof(notification.id), "dismiss-capacity-%u", i);
        rt_snprintf(notification.reply_instance, sizeof(notification.reply_instance), "%032lx", (unsigned long)(0x20000000u + i));
        notification.reply_revision = 1;
        set_notification(&notification, 0); notification_items_amount = 1;
        if (!notification_reply_dismiss(&notification)) return false;
        char result[256];
        rt_snprintf(result, sizeof(result),
            "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"%s\",\"v\":1,\"state\":\"dismissed\"}",
            (unsigned long)new_id_count, notification.reply_instance);
        if (!notification_reply_receive(result, strlen(result))) return false;
        notification_reply_process();
    }
    return notification_items_amount == 0;
}
static unsigned transcript_count;
static uint32_t transcript_input;
static char transcript_text[512];
static void transcript(uint32_t input, const char *text)
{
    transcript_count++; transcript_input = input;
    strncpy(transcript_text, text, sizeof(transcript_text) - 1);
}
static bool late_transcript_cannot_fill_a_new_notification_input(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-voice"); strcpy(notification.reply_instance, "55555555555555555555555555555555");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;

    uint32_t old = notification_reply_begin(&notification);
    if (!notification_reply_listen(old) || !strstr(sent_json, "\"op\":\"input\"")) return false;
    unsigned old_context = new_id_count;
    notification_reply_cancel(old);
    uint32_t current = notification_reply_begin(&notification);
    notification_reply_bind_transcript(current, transcript);
    notification_reply_bind_transcript(old, NULL); /* A late old-page teardown cannot detach this view. */
    if (!notification_reply_listen(current)) return false;
    char result[256];
    rt_snprintf(result, sizeof(result), "{\"op\":\"text\",\"c\":\"%032lx\",\"m\":\"Current text\"}", (unsigned long)new_id_count);
    notification_reply_receive(result, strlen(result));
    rt_snprintf(result, sizeof(result), "{\"op\":\"text\",\"c\":\"%032lx\",\"m\":\"Old text\"}", (unsigned long)old_context);
    notification_reply_receive(result, strlen(result));
    notification_reply_process();
    bool scoped = transcript_count == 1 && transcript_input == current && !strcmp(transcript_text, "Current text");
    notification_reply_bind_transcript(current, NULL);
    notification_reply_cancel(current);
    return scoped;
}
static bool independent_instances_with_colliding_legacy_ids_remain_independent(void)
{
    notification_items_amount = 0;
    handle_notification(Notify_others, "{\"id\":\"collision\",\"title\":\"Same\",\"message\":\"Same\",\"reply\":true,\"ri\":\"77777777777777777777777777777777\",\"rv\":1}");
    handle_notification(Notify_others, "{\"id\":\"collision\",\"title\":\"Same\",\"message\":\"Same\",\"reply\":true,\"ri\":\"88888888888888888888888888888888\",\"rv\":1}");
    if (notification_items_amount != 2) return false;
    return notification_reply_dismiss(get_notification(1)) && notification_items_amount == 1 &&
        !strcmp(get_notification(0)->reply_instance, "88888888888888888888888888888888");
}
static bool completion_remains_visible_after_the_work_slot_is_reused(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-reuse"); strcpy(notification.reply_instance, "abababababababababababababababab");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    uint32_t input = notification_reply_begin(&notification);
    if (notification_reply_submit(input, "Completed reply") != NOTIFICATION_REPLY_WAITING) return false;
    char result[256];
    rt_snprintf(result, sizeof(result),
        "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"%s\",\"v\":1,\"state\":\"sent\"}",
        (unsigned long)new_id_count, notification.reply_instance);
    notification_reply_receive(result, strlen(result)); notification_reply_process();
    strcpy(notification.id, "dismiss-reuse"); strcpy(notification.reply_instance, "acacacacacacacacacacacacacacacac");
    set_notification(&notification, 0); notification_items_amount = 1;
    if (!notification_reply_dismiss(&notification) || notification_reply_state(input) != NOTIFICATION_REPLY_SENT) return false;
    strcpy(notification.id, "external-removal"); strcpy(notification.reply_instance, "adadadadadadadadadadadadadadadad");
    set_notification(&notification, 0); notification_items_amount = 1;
    uint32_t pending = notification_reply_begin(&notification);
    notification_reply_submit(pending, "Still uncertain");
    const char *removed = "{\"op\":\"removed\",\"n\":\"adadadadadadadadadadadadadadadad\",\"v\":1}";
    notification_reply_receive(removed, strlen(removed)); notification_reply_process();
    return notification_reply_state(pending) != NOTIFICATION_REPLY_SENT;
}
static bool reopening_pending_work_keeps_its_draft_without_owning_the_old_completion(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-reopen"); strcpy(notification.reply_instance, "90909090909090909090909090909090");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    uint32_t original = notification_reply_begin(&notification);
    notification_reply_submit(original, "Frozen draft");
    unsigned request = new_id_count, before = send_count;
    notification_reply_cancel(original);
    notification_reply_view_t view = notification_reply_open(&notification);
    if (!view.input || view.input == original || view.state != NOTIFICATION_REPLY_WAITING ||
        !view.draft || strcmp(view.draft, "Frozen draft")) return false;
    if (notification_reply_submit(view.input, "Different text") != NOTIFICATION_REPLY_WAITING || send_count != before) return false;
    char result[256];
    rt_snprintf(result, sizeof(result),
        "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"%s\",\"v\":1,\"state\":\"sent\"}",
        (unsigned long)request, notification.reply_instance);
    notification_reply_receive(result, strlen(result)); notification_reply_process();
    if (notification_reply_state(view.input) != NOTIFICATION_REPLY_SENT || notification_reply_can_finish(view.input)) return false;
    strcpy(notification.id, "reply-current-view"); strcpy(notification.reply_instance, "91919191919191919191919191919191");
    set_notification(&notification, 0); notification_items_amount = 1;
    uint32_t current = notification_reply_begin(&notification);
    notification_reply_submit(current, "Current view");
    rt_snprintf(result, sizeof(result),
        "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"%s\",\"v\":1,\"state\":\"sent\"}",
        (unsigned long)new_id_count, notification.reply_instance);
    notification_reply_receive(result, strlen(result)); notification_reply_process();
    return notification_reply_can_finish(current);
}
static bool removal_before_confirmation_does_not_lose_the_submitting_page(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-removal-order"); strcpy(notification.reply_instance, "94949494949494949494949494949494");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    uint32_t input = notification_reply_begin(&notification);
    notification_reply_submit(input, "Confirmed separately");
    unsigned request = new_id_count;
    const char *removed = "{\"op\":\"removed\",\"n\":\"94949494949494949494949494949494\",\"v\":1}";
    notification_reply_receive(removed, strlen(removed)); notification_reply_process();
    if (notification_items_amount || notification_reply_state(input) == NOTIFICATION_REPLY_SENT || notification_reply_can_finish(input)) return false;
    char result[256];
    rt_snprintf(result, sizeof(result), "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"94949494949494949494949494949494\",\"v\":1,\"state\":\"sent\"}", (unsigned long)request);
    notification_reply_receive(result, strlen(result)); notification_reply_process();
    return notification_reply_can_finish(input);
}
static bool confirmation_before_removal_preserves_submitting_page(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-confirmation-order"); strcpy(notification.reply_instance, "dededededededededededededededede");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    uint32_t input = notification_reply_begin(&notification);
    if (notification_reply_submit(input, "Confirm before removal") != NOTIFICATION_REPLY_WAITING) return false;
    char result[256];
    rt_snprintf(result, sizeof(result),
        "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"dededededededededededededededede\",\"v\":1,\"state\":\"sent\"}",
        (unsigned long)new_id_count);
    notification_reply_receive(result, strlen(result)); notification_reply_process();
    if (!notification_reply_can_finish(input) || notification_items_amount) return false;
    const char *removed = "{\"op\":\"removed\",\"n\":\"dededededededededededededededede\",\"v\":1}";
    notification_reply_receive(removed, strlen(removed)); notification_reply_process();
    return notification_reply_can_finish(input) && notification_reply_state(input) == NOTIFICATION_REPLY_SENT &&
        notification_items_amount == 0;
}
static bool ambiguous_send_keeps_the_request_locked_until_a_phone_result(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-ambiguous"); strcpy(notification.reply_instance, "95959595959595959595959595959595");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    notification_reply_transport_t adapter = {now_ms, ready, ready, NULL, new_id};
    extern bool test_reply_send_ambiguous(const char *, uint16_t);
    adapter.send = test_reply_send_ambiguous;
    notification_reply_bind_transport(&adapter);
    uint32_t input = notification_reply_begin(&notification);
    notification_reply_state_t state = notification_reply_submit(input, "May have left the watch");
    adapter.send = send_json; notification_reply_bind_transport(&adapter);
    if (state != NOTIFICATION_REPLY_UNKNOWN || notification_reply_begin(&notification)) return false;
    notification_reply_view_t view = notification_reply_open(&notification);
    unsigned before = send_count;
    if (!view.input || notification_reply_submit(view.input, "Must not resend") != NOTIFICATION_REPLY_UNKNOWN || send_count != before) return false;
    return notification_reply_query(view.input) && send_count == before + 1 && strstr(sent_json, "\"op\":\"query\"");
}
bool test_reply_send_ambiguous(const char *json, uint16_t length) { send_json(json, length); return false; }
static bool oversized_dictation_cannot_send_the_last_partial_text(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "reply-voice-overflow"); strcpy(notification.reply_instance, "96969696969696969696969696969696");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    uint32_t input = notification_reply_begin(&notification);
    if (!notification_reply_listen(input)) return false;
    setVoice2Text("Last partial");
    char error[160];
    rt_snprintf(error, sizeof(error), "{\"op\":\"input_error\",\"c\":\"%032lx\",\"m\":\"too_long\"}", (unsigned long)new_id_count);
    if (!notification_reply_receive(error, strlen(error))) return false;
    notification_reply_process();
    unsigned before = send_count;
    if (strcmp(get_combined_voice2text(), "Last partial") || notification_reply_submit(input, get_combined_voice2text()) != NOTIFICATION_REPLY_FAILED || send_count != before) return false;
    rt_snprintf(error, sizeof(error), "{\"op\":\"text\",\"c\":\"%032lx\",\"m\":\"New complete text\"}", (unsigned long)new_id_count);
    if (!notification_reply_receive(error, strlen(error))) return false;
    if (notification_reply_submit(input, "Last partial") != NOTIFICATION_REPLY_FAILED || send_count != before) return false;
    notification_reply_process();
    if (strcmp(get_combined_voice2text(), "New complete text")) return false;
    rt_snprintf(error, sizeof(error), "{\"op\":\"input_error\",\"c\":\"%032lx\",\"m\":\"unavailable\"}", (unsigned long)new_id_count);
    if (!notification_reply_receive(error, strlen(error))) return false;
    notification_reply_process();
    if (strcmp(get_combined_voice2text(), "New complete text") || notification_reply_submit(input, get_combined_voice2text()) != NOTIFICATION_REPLY_FAILED || send_count != before) return false;
    return notification_reply_submit(input, "Shortened") == NOTIFICATION_REPLY_WAITING && send_count == before + 1;
}
static bool legacy_dismiss_keeps_an_independent_modern_notification(void)
{
    notification_t legacy = {0}, modern = {0};
    strcpy(legacy.id, "legacy-dismiss-collision"); strcpy(legacy.title, "Old phone");
    modern = legacy; strcpy(modern.reply_instance, "97979797979797979797979797979797"); modern.reply_revision = 1;
    set_notification(&legacy, 0); set_notification(&modern, 1); notification_items_amount = 2;
    notification_reply_peer(true, false);
    bool removed = notification_reply_dismiss(&legacy);
    notification_reply_peer(true, true);
    return removed && notification_items_amount == 1 && !strcmp(get_notification(0)->reply_instance, modern.reply_instance);
}
static bool lost_dismiss_retries_only_the_same_cleanup_operation(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "lost-dismiss"); strcpy(notification.reply_instance, "98989898989898989898989898989898"); notification.reply_revision = 1;
    set_notification(&notification, 0); notification_items_amount = 1;
    observed_target = "98989898989898989898989898989898";
    observed_dismisses = observed_queries = 0;
    bool removed = notification_reply_dismiss(&notification);
    unsigned request = new_id_count;
    test_now += 1999; notification_reply_process();
    bool before_due = observed_dismisses == 1;
    test_now++; notification_reply_process();
    observed_target = NULL;
    return removed && before_due && observed_dismisses == 2 && !observed_queries && new_id_count == request && notification_items_amount == 0;
}
static bool submission_availability_follows_the_owned_target_and_phone_result(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "submission-availability"); strcpy(notification.reply_instance, "99999999999999999999999999999998");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    uint32_t input = notification_reply_begin(&notification);
    if (!notification_reply_can_submit(input)) return false;
    notification_reply_submit(input, "Availability check");
    if (notification_reply_can_submit(input)) return false;
    char result[256];
    rt_snprintf(result, sizeof(result), "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"%s\",\"v\":1,\"state\":\"failed\"}", (unsigned long)new_id_count, notification.reply_instance);
    notification_reply_receive(result, strlen(result)); notification_reply_process();
    if (!notification_reply_can_submit(input)) return false;
    notification.reply_revision++; set_notification(&notification, 0);
    return !notification_reply_can_submit(input) && notification_reply_state(input) == NOTIFICATION_REPLY_INVALID;
}
static bool clearing_a_draft_invalidates_late_voice_and_releases_a_failed_attempt(void)
{
    notification_t notification = {0};
    strcpy(notification.id, "clear-draft"); strcpy(notification.reply_instance, "a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0");
    notification.reply_revision = 1; notification.can_reply = true;
    set_notification(&notification, 0); notification_items_amount = 1;
    uint32_t old = notification_reply_begin(&notification);
    if (!notification_reply_listen(old)) return false;
    unsigned context = new_id_count;
    uint32_t fresh = notification_reply_clear(old);
    if (!fresh || fresh == old || notification_reply_submit(old, "Old draft") != NOTIFICATION_REPLY_INVALID) return false;
    char packet[256];
    rt_snprintf(packet, sizeof(packet), "{\"op\":\"text\",\"c\":\"%032lx\",\"m\":\"Late cleared words\"}", (unsigned long)context);
    if (notification_reply_receive(packet, strlen(packet))) return false;
    if (notification_reply_submit(fresh, "Failed draft") != NOTIFICATION_REPLY_WAITING) return false;
    rt_snprintf(packet, sizeof(packet), "{\"op\":\"result\",\"r\":\"%032lx\",\"n\":\"%s\",\"v\":1,\"state\":\"failed\"}", (unsigned long)new_id_count, notification.reply_instance);
    notification_reply_receive(packet, strlen(packet)); notification_reply_process();
    if (!notification_reply_clear(fresh)) return false;
    notification_reply_view_t view = notification_reply_open(&notification);
    return view.input && notification_reply_can_submit(view.input) && !view.draft;
}
static void run_all(void *unused)
{
    static const struct { const char *name; bool (*run)(void); } cases[] = {
        {"sixteen_pending_replies_preserve_the_first_draft_when_full", sixteen_pending_replies_preserve_the_first_draft_when_full},
        {"unscoped_legacy_intent_cannot_bypass_reply_confirmation", unscoped_legacy_intent_cannot_bypass_reply_confirmation},
        {"legacy_phone_cannot_start_a_reply", legacy_phone_cannot_start_a_reply},
        {"stale_notification_cannot_begin_a_reply", stale_notification_cannot_begin_a_reply},
        {"cancelled_input_cannot_submit_a_late_transcript", cancelled_input_cannot_submit_a_late_transcript},
        {"target_change_during_input_rejects_submission", target_change_during_input_rejects_submission},
        {"accepted_submission_waits_for_phone_confirmation", accepted_submission_waits_for_phone_confirmation},
        {"only_the_matching_phone_result_removes_the_notification", only_the_matching_phone_result_removes_the_notification},
        {"lost_result_queries_with_backoff_without_resending_text", lost_result_queries_with_backoff_without_resending_text},
        {"notification_ingress_refreshes_reply_target_without_changing_visible_content", notification_ingress_refreshes_reply_target_without_changing_visible_content},
        {"old_revision_cannot_replace_a_newer_notification", old_revision_cannot_replace_a_newer_notification},
        {"identical_content_from_different_instances_stays_distinct", identical_content_from_different_instances_stays_distinct},
        {"option_reply_keeps_notification_and_suppresses_a_second_press", option_reply_keeps_notification_and_suppresses_a_second_press},
        {"malformed_result_cannot_remove_a_notification", malformed_result_cannot_remove_a_notification},
        {"new_input_cannot_resend_a_pending_target", new_input_cannot_resend_a_pending_target},
        {"blank_or_invalid_utf8_text_is_rejected_before_send", blank_or_invalid_utf8_text_is_rejected_before_send},
        {"reconnect_discards_previous_capability_and_queued_results", reconnect_discards_previous_capability_and_queued_results},
        {"unversioned_dismiss_cannot_remove_a_modern_notification", unversioned_dismiss_cannot_remove_a_modern_notification},
        {"versioned_removal_only_removes_the_matching_notification", versioned_removal_only_removes_the_matching_notification},
        {"manual_query_after_the_deadline_never_resends_reply_text", manual_query_after_the_deadline_never_resends_reply_text},
        {"ordinary_dismiss_is_versioned_and_does_not_send_reply_text", ordinary_dismiss_is_versioned_and_does_not_send_reply_text},
        {"offline_dismiss_keeps_current_experience_and_sends_after_reconnect", offline_dismiss_keeps_current_experience_and_sends_after_reconnect},
        {"dismissed_revision_cannot_reappear_but_a_new_revision_can", dismissed_revision_cannot_reappear_but_a_new_revision_can},
        {"confirmed_dismiss_releases_capacity_for_later_actions", confirmed_dismiss_releases_capacity_for_later_actions},
        {"late_transcript_cannot_fill_a_new_notification_input", late_transcript_cannot_fill_a_new_notification_input},
        {"independent_instances_with_colliding_legacy_ids_remain_independent", independent_instances_with_colliding_legacy_ids_remain_independent},
        {"completion_remains_visible_after_the_work_slot_is_reused", completion_remains_visible_after_the_work_slot_is_reused},
        {"reopening_pending_work_keeps_its_draft_without_owning_the_old_completion", reopening_pending_work_keeps_its_draft_without_owning_the_old_completion},
        {"removal_before_confirmation_does_not_lose_the_submitting_page", removal_before_confirmation_does_not_lose_the_submitting_page},
        {"confirmation_before_removal_preserves_submitting_page", confirmation_before_removal_preserves_submitting_page},
        {"ambiguous_send_keeps_the_request_locked_until_a_phone_result", ambiguous_send_keeps_the_request_locked_until_a_phone_result},
        {"oversized_dictation_cannot_send_the_last_partial_text", oversized_dictation_cannot_send_the_last_partial_text},
        {"legacy_dismiss_keeps_an_independent_modern_notification", legacy_dismiss_keeps_an_independent_modern_notification},
        {"lost_dismiss_retries_only_the_same_cleanup_operation", lost_dismiss_retries_only_the_same_cleanup_operation},
        {"submission_availability_follows_the_owned_target_and_phone_result", submission_availability_follows_the_owned_target_and_phone_result},
        {"clearing_a_draft_invalidates_late_voice_and_releases_a_failed_attempt", clearing_a_draft_invalidates_late_voice_and_releases_a_failed_attempt},
    };
    char line[160];
    (void)unused;
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        bool passed = cases[i].run();
        rt_snprintf(line, sizeof(line), "REPLY_TDD %s %s", passed ? "PASS" : "FAIL", cases[i].name);
        report(line);
        if (!passed) return;
    }
    begin_message_ui_test();
}
static void autorun(void *unused)
{
    (void)unused;
    rt_thread_mdelay(5000);
#if defined(NOTIFICATION_REPLY_BLE_TEST) && !defined(BSP_USING_PC_SIMULATOR)
    lv_async_call(begin_ble_fixture, NULL);
#else
    lv_async_call(run_all, NULL);
#endif
}
static int reply_tdd_init(void)
{
#ifdef BSP_USING_PC_SIMULATOR
    if (!getenv("SKAI_REPLY_TDD")) return 0;
#endif
    rt_thread_t thread = rt_thread_create("reply_tdd", autorun, NULL, 4096, 20, 10);
    if (!thread) return -1;
    return rt_thread_startup(thread);
}
INIT_APP_EXPORT(reply_tdd_init);
