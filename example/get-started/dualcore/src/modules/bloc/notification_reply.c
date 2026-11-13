/* Copyright (c) 2026 Skaiwalk Technology
 * SPDX-License-Identifier: Apache-2.0
 */
#define _CRT_RAND_S
#include "notification_reply.h"
#include <string.h>
#include <limits.h>
#include <rtthread.h>
#include "app_mem.h"
#include "cJSON.h"
#include "ui_handler.h"
#include "communicate_task.h"
#include "watch_global_data.h"
#include "bloc_peripheral.h"
#include "bloc_v2t.h"
#ifdef BSP_USING_PC_SIMULATOR
#include <stdlib.h>
#else
#include "bf0_hal.h"
#endif

static volatile bool linked, supported;
static volatile uint32_t link_epoch;
static bool receive_ready;
static uint32_t input_counter, active_input, active_revision;
static char active_instance[33], active_voice[33];
static bool voice_enabled, voice_ready, voice_overflow, current_input_submitted;
static uint32_t overflow_text_hash;
static unsigned overflow_text_length;
static bool decode_id(const char *text, uint8_t output[16])
{
    if (!text) return false;
    for (unsigned i = 0; i < 32; i++) {
        unsigned value;
        if (text[i] >= '0' && text[i] <= '9') value = text[i] - '0';
        else if (text[i] >= 'a' && text[i] <= 'f') value = text[i] - 'a' + 10;
        else return false;
        if (i & 1) output[i / 2] |= value;
        else output[i / 2] = (uint8_t)(value << 4);
    }
    return text[32] == 0;
}
static uint32_t text_hash(const char *text) {
    uint32_t hash = 2166136261u;
    for (const unsigned char *p = (const unsigned char *)text; *p; p++) hash = (hash ^ *p) * 16777619u;
    return hash;
}
static void deliver_voice(uint32_t input, const char *text) { (void)input; setVoice2Text((char *)text); }
static void (*deliver_transcript)(uint32_t, const char *) = deliver_voice;
static uint32_t real_now(void) { return (uint32_t)((uint64_t)rt_tick_get() * 1000u / RT_TICK_PER_SECOND); }
static bool real_ready(void) { return SkaiWatchSys.connected_to_phone; }
static bool real_awake(void) { return !is_hcpu_suspend(); }
static bool real_new_id(char output[33])
{
    uint32_t words[4];
#ifdef BSP_USING_PC_SIMULATOR
    for (unsigned i = 0; i < 4; i++) if (rand_s(&words[i])) return false;
#else
    RNG_HandleTypeDef rng = {0};
    rng.Instance = hwp_trng;
    if (HAL_RNG_Init(&rng) != HAL_OK) return false;
    uint32_t seed;
    bool generated = HAL_RNG_Generate(&rng, &seed, 1) == HAL_OK;
    for (unsigned i = 0; generated && i < 4; i++) generated = HAL_RNG_Generate(&rng, &words[i], 0) == HAL_OK;
    HAL_RNG_DeInit(&rng);
    if (!generated) return false;
#endif
    rt_snprintf(output, 33, "%08lx%08lx%08lx%08lx", (unsigned long)words[0],
        (unsigned long)words[1], (unsigned long)words[2], (unsigned long)words[3]);
    return true;
}
static const notification_reply_transport_t real_transport = {real_now, real_ready, real_awake, commu_send_notification_reply, real_new_id};
static notification_reply_transport_t transport = {real_now, real_ready, real_awake, commu_send_notification_reply, real_new_id};
typedef struct {
    uint32_t input, revision, started, next, epoch;
    char instance[33], request[33], text[512];
    uint8_t state;
    uint8_t retry;
    bool used, eligible, automatic, dismiss, submitted;
} reply_work_t;
typedef struct {
    char request[33], instance[33];
    uint32_t revision;
    notification_reply_state_t state;
    bool removed;
} reply_event_t;
typedef struct {
    reply_work_t work[16]; char wire[3200];
    reply_event_t events[8]; unsigned count;
    union { double align; unsigned char bytes[1024]; } json;
    struct { uint8_t instance[16], state; uint32_t revision; } processed[16];
    unsigned processed_next;
    struct { char context[33], text[512]; uint32_t input; bool pending, error; } voice;
} reply_storage_t;
APP_L2_RET_BSS_SECT(notification_replies, ALIGN(8) static reply_storage_t storage);
static struct rt_mutex receive_mutex;
static unsigned json_used;
static lv_timer_t *pump;
static void pump_results(lv_timer_t *timer) { (void)timer; notification_reply_process(); }
static int reply_init(void) {
    memset(&storage, 0, sizeof(storage));
    int result = rt_mutex_init(&receive_mutex, "replyrx", RT_IPC_FLAG_PRIO);
    receive_ready = result == RT_EOK;
    return result;
}
INIT_APP_EXPORT(reply_init);
typedef char reply_storage_is_bounded[(sizeof(storage) + sizeof(transport) + sizeof(active_instance) +
    sizeof(active_voice) + sizeof(receive_mutex) + sizeof(pump) + sizeof(deliver_transcript) + 64u +
    /* Shared notification identities (store, ingress, page) and store lock. */
    (ITEM_AMOUNT_NOTIFICATION + 2u) * 40u + sizeof(struct rt_mutex) + 4u <= 16u * 1024u) ? 1 : -1];

void notification_reply_peer(bool connected, bool capable) {
    notification_reply_link(connected);
    supported = connected && capable;
}
void notification_reply_link(bool connected) {
    if (receive_ready) rt_mutex_take(&receive_mutex, RT_WAITING_FOREVER);
    if (linked != connected) {
        linked = connected; supported = false; link_epoch++;
        storage.count = 0;
        active_voice[0] = 0; storage.voice.pending = false; voice_ready = false;
    }
    if (receive_ready) rt_mutex_release(&receive_mutex);
}
bool notification_reply_available(void) { return linked && supported; }
bool notification_reply_pending(const char *instance, uint32_t revision)
{
    if (!instance || !revision) return false;
    for (unsigned i = 0; i < 16; i++) {
        const reply_work_t *work = &storage.work[i];
        if (work->used && work->revision == revision && !strcmp(work->instance, instance) &&
            (work->state == NOTIFICATION_REPLY_WAITING || work->state == NOTIFICATION_REPLY_UNKNOWN)) return true;
    }
    return false;
}
notification_reply_view_t notification_reply_open(const notification_t *notification)
{ return notification_reply_open_version(notification ? notification->reply_instance : NULL, notification ? notification->reply_revision : 0); }
bool notification_reply_can_finish(uint32_t input)
{ return input && input == active_input && current_input_submitted && notification_reply_state(input) == NOTIFICATION_REPLY_SENT; }
bool notification_reply_was_processed(const char *instance, uint32_t revision)
{ return notification_reply_version_state(instance, revision) != NOTIFICATION_REPLY_NONE; }
notification_reply_state_t notification_reply_version_state(const char *instance, uint32_t revision)
{
    uint8_t identity[16];
    if (!revision || !receive_ready || !decode_id(instance, identity)) return NOTIFICATION_REPLY_NONE;
    notification_reply_state_t state = NOTIFICATION_REPLY_NONE;
    rt_mutex_take(&receive_mutex, RT_WAITING_FOREVER);
    for (unsigned i = 0; i < 16; i++)
        if (storage.processed[i].revision == revision && !memcmp(storage.processed[i].instance, identity, sizeof(identity))) { state = (notification_reply_state_t)storage.processed[i].state; break; }
    rt_mutex_release(&receive_mutex);
    return state;
}
static void remember_processed(const char *instance, uint32_t revision, notification_reply_state_t state)
{
    uint8_t identity[16];
    if (!decode_id(instance, identity)) return;
    rt_mutex_take(&receive_mutex, RT_WAITING_FOREVER);
    for (unsigned i = 0; i < 16; i++) {
        if (storage.processed[i].revision == revision && !memcmp(storage.processed[i].instance, identity, sizeof(identity))) {
            if (state == NOTIFICATION_REPLY_SENT || storage.processed[i].state == NOTIFICATION_REPLY_INVALID)
                storage.processed[i].state = (uint8_t)state;
            rt_mutex_release(&receive_mutex);
            return;
        }
    }
    unsigned index = storage.processed_next++ % 16;
    memcpy(storage.processed[index].instance, identity, sizeof(identity));
    storage.processed[index].revision = revision;
    storage.processed[index].state = (uint8_t)state;
    rt_mutex_release(&receive_mutex);
}
void notification_reply_bind_transport(const notification_reply_transport_t *adapter)
{
    if (adapter) transport = *adapter;
    else transport = real_transport;
}
static bool hexadecimal_id(const char *text)
{
    uint8_t identity[16];
    return decode_id(text, identity);
}
extern bool notification_reply_target_current(const char *instance, uint32_t revision, bool require_reply);
static bool current_target(const char *instance, uint32_t revision)
{
    return hexadecimal_id(instance) && notification_reply_target_current(instance, revision, true);
}
uint32_t notification_reply_begin(const notification_t *notification)
{
    return notification ? notification_reply_begin_version(notification->reply_instance, notification->reply_revision) : 0;
}
bool notification_reply_can_submit(uint32_t input)
{
    if (!input || input != active_input || !notification_reply_available() || !current_target(active_instance, active_revision)) return false;
    notification_reply_state_t state = notification_reply_state(input);
    return state == NOTIFICATION_REPLY_NONE || state == NOTIFICATION_REPLY_FAILED;
}
static uint32_t bind_input(const char *instance, uint32_t revision)
{
    if (input_counter == UINT32_MAX) return 0;
    rt_mutex_take(&receive_mutex, RT_WAITING_FOREVER);
    memcpy(active_instance, instance, sizeof(active_instance));
    active_revision = revision;
    active_input = ++input_counter;
    deliver_transcript = deliver_voice;
    active_voice[0] = 0; storage.voice.pending = false;
    voice_enabled = voice_ready = voice_overflow = current_input_submitted = false;
    rt_mutex_release(&receive_mutex);
    return active_input;
}
notification_reply_view_t notification_reply_open_version(const char *instance, uint32_t revision)
{
    notification_reply_view_t view = {0, NOTIFICATION_REPLY_INVALID, NULL};
    if (!hexadecimal_id(instance) || !revision) return view;
    for (unsigned i = 0; i < 16; i++) {
        reply_work_t *work = &storage.work[i];
        if (!work->used || work->dismiss || work->revision != revision || strcmp(work->instance, instance)) continue;
        view.draft = work->text;
        view.state = work->state;
        if (work->state == NOTIFICATION_REPLY_WAITING || work->state == NOTIFICATION_REPLY_UNKNOWN) {
            view.input = bind_input(instance, revision);
            if (view.input) work->input = view.input;
            return view;
        }
        break;
    }
    view.input = notification_reply_begin_version(instance, revision);
    view.state = view.input ? notification_reply_state(view.input) :
        (notification_reply_available() ? NOTIFICATION_REPLY_INVALID : NOTIFICATION_REPLY_UNSUPPORTED);
    return view;
}
uint32_t notification_reply_begin_version(const char *instance, uint32_t revision)
{
    if (!pump) pump = lv_timer_create(pump_results, 250, NULL);
    if (!pump) return 0;
    if (!notification_reply_available() || input_counter == UINT32_MAX) return 0;
    if (!current_target(instance, revision)) return 0;
    for (unsigned i = 0; i < 16; i++) {
        const reply_work_t *work = &storage.work[i];
        if (work->used && work->revision == revision &&
            !strcmp(work->instance, instance) &&
            (work->state == NOTIFICATION_REPLY_WAITING || work->state == NOTIFICATION_REPLY_UNKNOWN)) return 0;
    }
    uint32_t input = bind_input(instance, revision);
    for (unsigned i = 0; i < 16; i++) {
        reply_work_t *work = &storage.work[i];
        if (work->used && work->state == NOTIFICATION_REPLY_FAILED && work->revision == revision &&
            !strcmp(work->instance, instance)) { work->input = input; break; }
    }
    return input;
}
void notification_reply_cancel(uint32_t input) {
    rt_mutex_take(&receive_mutex, RT_WAITING_FOREVER);
    if (input == active_input) { active_input = 0; active_voice[0] = 0; storage.voice.pending = false; voice_enabled = voice_ready = voice_overflow = current_input_submitted = false; deliver_transcript = deliver_voice; }
    rt_mutex_release(&receive_mutex);
}
bool notification_reply_listen(uint32_t input) {
    if (!input || input != active_input || !current_target(active_instance, active_revision) ||
        notification_reply_pending(active_instance, active_revision) ||
        !notification_reply_available() || !transport.ready || !transport.ready() || !transport.send || !transport.new_id) return false;
    char context[33];
    if (!transport.new_id(context) || !hexadecimal_id(context)) return false;
    int length = rt_snprintf(storage.wire, sizeof(storage.wire), "{\"op\":\"input\",\"c\":\"%s\"}", context);
    if (!transport.send(storage.wire, (uint16_t)length)) return false;
    rt_mutex_take(&receive_mutex, RT_WAITING_FOREVER);
    memcpy(active_voice, context, 33); storage.voice.pending = false;
    voice_enabled = voice_ready = true;
    rt_mutex_release(&receive_mutex);
    return true;
}
uint32_t notification_reply_clear(uint32_t input)
{
    if (!input || input != active_input) return 0;
    if (notification_reply_pending(active_instance, active_revision)) return input;
    char instance[33]; memcpy(instance, active_instance, sizeof(instance));
    uint32_t revision = active_revision;
    for (unsigned i = 0; i < 16; i++) {
        reply_work_t *work = &storage.work[i];
        if (work->used && work->input == input && work->state == NOTIFICATION_REPLY_FAILED)
            memset(work, 0, sizeof(*work));
    }
    void (*deliver)(uint32_t, const char *) = deliver_transcript;
    notification_reply_cancel(input);
    uint32_t fresh = notification_reply_begin_version(instance, revision);
    notification_reply_bind_transcript(fresh, deliver);
    return fresh;
}
bool notification_reply_prepare_voice(void) {
    if (!voice_enabled) return true;
    if (!voice_ready && !notification_reply_listen(active_input)) return false;
    voice_ready = false;
    return true;
}
void notification_reply_bind_transcript(uint32_t input, void (*deliver)(uint32_t input, const char *text)) {
    if (input && input == active_input) deliver_transcript = deliver ? deliver : deliver_voice;
}

static int encode_reply(const reply_work_t *work)
{
    unsigned used = rt_snprintf(storage.wire, sizeof(storage.wire),
        "{\"op\":\"reply\",\"r\":\"%s\",\"n\":\"%s\",\"v\":%lu,\"m\":\"",
        work->request, work->instance, (unsigned long)work->revision);
    static const char hex[] = "0123456789abcdef";
    for (const unsigned char *p = (const unsigned char *)work->text; *p; p++) {
        if (used + 9 >= sizeof(storage.wire)) return -1;
        if (*p == '"' || *p == '\\') { storage.wire[used++] = '\\'; storage.wire[used++] = *p; }
        else if (*p < 32) {
            memcpy(storage.wire + used, "\\u00", 4); used += 4;
            storage.wire[used++] = hex[*p >> 4]; storage.wire[used++] = hex[*p & 15];
        } else storage.wire[used++] = *p;
    }
    storage.wire[used++] = '"'; storage.wire[used++] = '}'; storage.wire[used] = 0;
    return (int)used;
}
static bool valid_text(const char *text)
{
    if (!text) return false;
    size_t length = 0;
    while (length < 512 && text[length]) length++;
    if (!length || length >= 512) return false;
    bool content = false;
    for (size_t i = 0; i < length;) {
        uint32_t cp = (unsigned char)text[i++], minimum = 0;
        unsigned extra = 0;
        if (cp >= 0xc2 && cp <= 0xdf) { cp &= 31; extra = 1; minimum = 0x80; }
        else if (cp >= 0xe0 && cp <= 0xef) { cp &= 15; extra = 2; minimum = 0x800; }
        else if (cp >= 0xf0 && cp <= 0xf4) { cp &= 7; extra = 3; minimum = 0x10000; }
        else if (cp >= 0x80) return false;
        if (i + extra > length) return false;
        while (extra--) {
            unsigned byte = (unsigned char)text[i++];
            if ((byte & 0xc0) != 0x80) return false;
            cp = (cp << 6) | (byte & 63);
        }
        if (cp < minimum || cp > 0x10ffff || (cp >= 0xd800 && cp <= 0xdfff)) return false;
        bool space = (cp >= 9 && cp <= 13) || (cp >= 28 && cp <= 32) || cp == 0xa0 || cp == 0x1680 ||
            (cp >= 0x2000 && cp <= 0x200a) || cp == 0x2028 || cp == 0x2029 || cp == 0x202f || cp == 0x205f || cp == 0x3000;
        if (!space) content = true;
    }
    return content;
}
notification_reply_state_t notification_reply_submit(uint32_t input, const char *text)
{
    if (!input || input != active_input) return NOTIFICATION_REPLY_INVALID;
    if (!current_target(active_instance, active_revision)) return NOTIFICATION_REPLY_INVALID;
    if (!notification_reply_available()) return NOTIFICATION_REPLY_UNSUPPORTED;
    if (!valid_text(text)) return NOTIFICATION_REPLY_FAILED;
    if (voice_overflow && strlen(text) == overflow_text_length && text_hash(text) == overflow_text_hash) return NOTIFICATION_REPLY_FAILED;
    reply_work_t *work = NULL;
    for (unsigned i = 0; i < 16; i++) {
        if (storage.work[i].used && storage.work[i].input == input) { work = &storage.work[i]; break; }
    }
    if (work && (work->state == NOTIFICATION_REPLY_WAITING || work->state == NOTIFICATION_REPLY_UNKNOWN)) return work->state;
    if (!work) for (unsigned i = 0; i < 16; i++) if (!storage.work[i].used) { work = &storage.work[i]; break; }
    if (!work) return NOTIFICATION_REPLY_FULL;
    memset(work, 0, sizeof(*work));
    work->used = true; work->input = input; work->revision = active_revision;
    memcpy(work->instance, active_instance, sizeof(work->instance));
    memcpy(work->text, text, strlen(text) + 1);
    work->state = NOTIFICATION_REPLY_FAILED;
    if (!transport.ready || !transport.ready() || !transport.send || !transport.new_id ||
        !transport.new_id(work->request) || !hexadecimal_id(work->request)) return work->state;
    int length = encode_reply(work);
    if (length < 0) return work->state;
    work->started = transport.now_ms ? transport.now_ms() : 0;
    work->epoch = link_epoch;
    work->next = work->started + 2000;
    work->state = NOTIFICATION_REPLY_WAITING;
    voice_overflow = false;
    current_input_submitted = true;
    work->submitted = true;
    work->automatic = work->eligible = true;
    if (!transport.send(storage.wire, (uint16_t)length)) work->state = NOTIFICATION_REPLY_UNKNOWN;
    rt_mutex_take(&receive_mutex, RT_WAITING_FOREVER);
    active_voice[0] = 0; storage.voice.pending = false; voice_enabled = voice_ready = false;
    rt_mutex_release(&receive_mutex);
    return work->state;
}

notification_reply_state_t notification_reply_state(uint32_t input)
{
    for (unsigned i = 0; i < 16; i++) if (storage.work[i].input == input && input) {
        if (storage.work[i].state == NOTIFICATION_REPLY_FAILED && input == active_input && !current_target(active_instance, active_revision))
            return NOTIFICATION_REPLY_INVALID;
        return (notification_reply_state_t)storage.work[i].state;
    }
    if (input && input == active_input && notification_reply_version_state(active_instance, active_revision) == NOTIFICATION_REPLY_SENT)
        return NOTIFICATION_REPLY_SENT;
    if (input && input == active_input && voice_overflow) return NOTIFICATION_REPLY_FAILED;
    if (input && input == active_input && !current_target(active_instance, active_revision)) return NOTIFICATION_REPLY_INVALID;
    return NOTIFICATION_REPLY_NONE;
}
bool notification_reply_query(uint32_t input)
{
    if (!notification_reply_available() || !transport.ready || !transport.ready() || !transport.send) return false;
    for (unsigned i = 0; i < 16; i++) {
        const reply_work_t *work = &storage.work[i];
        if (!work->used || work->input != input || !hexadecimal_id(work->request)) continue;
        int length = rt_snprintf(storage.wire, sizeof(storage.wire),
            "{\"op\":\"query\",\"r\":\"%s\",\"n\":\"%s\",\"v\":%lu}",
            work->request, work->instance, (unsigned long)work->revision);
        return transport.send(storage.wire, (uint16_t)length);
    }
    return false;
}
extern bool notification_reply_remove_version(const char *instance, uint32_t revision);
extern bool notification_reply_remove_legacy(const notification_t *notification);
bool notification_reply_dismiss(const notification_t *notification)
{
    if (!notification) return false;
    if (!notification->reply_revision) return notification_reply_remove_legacy(notification);
    if (!hexadecimal_id(notification->reply_instance) ||
        !transport.new_id || input_counter == UINT32_MAX) return false;
    if (!notification_reply_target_current(notification->reply_instance, notification->reply_revision, false)) return false;
    reply_work_t *work = NULL;
    for (unsigned i = 0; i < 16; i++) if (!storage.work[i].used) { work = &storage.work[i]; break; }
    if (!work) return false;
    if (!pump) pump = lv_timer_create(pump_results, 250, NULL);
    if (!pump) return false;
    memset(work, 0, sizeof(*work));
    if (!transport.new_id(work->request) || !hexadecimal_id(work->request)) return false;
    memcpy(work->instance, notification->reply_instance, 33);
    work->revision = notification->reply_revision;
    work->input = ++input_counter;
    work->used = work->automatic = work->dismiss = true;
    work->state = NOTIFICATION_REPLY_WAITING;
    remember_processed(work->instance, work->revision, NOTIFICATION_REPLY_DISMISSED);
    bool removed = notification_reply_remove_version(work->instance, work->revision);
    notification_reply_process();
    return removed;
}
notification_reply_state_t notification_reply_option(const notification_t *notification, const char *text)
{
    if (!notification) return NOTIFICATION_REPLY_INVALID;
    return notification_reply_option_version(notification->reply_instance, notification->reply_revision, text);
}
notification_reply_state_t notification_reply_option_version(const char *instance, uint32_t revision, const char *text)
{
    if (!hexadecimal_id(instance) || !revision) return NOTIFICATION_REPLY_INVALID;
    for (unsigned i = 0; i < 16; i++) {
        const reply_work_t *work = &storage.work[i];
        if (work->used && work->revision == revision && !strcmp(work->instance, instance) &&
            (work->state == NOTIFICATION_REPLY_WAITING || work->state == NOTIFICATION_REPLY_UNKNOWN)) return work->state;
    }
    uint32_t input = notification_reply_begin_version(instance, revision);
    return input ? notification_reply_submit(input, text) :
        (notification_reply_available() ? NOTIFICATION_REPLY_INVALID : NOTIFICATION_REPLY_UNSUPPORTED);
}
static void *json_alloc(size_t size)
{
    size = (size + 7u) & ~(size_t)7u;
    if (size > sizeof(storage.json.bytes) - json_used) return NULL;
    void *result = storage.json.bytes + json_used;
    json_used += size;
    return result;
}
static void json_free(void *unused) { (void)unused; }
bool notification_reply_receive(const void *json, uint16_t length)
{
    if (!receive_ready || !json || !length || length > 3584 || memchr(json, 0, length)) return false;
    bool quoted = false, escape = false;
    unsigned depth = 0;
    const char *bytes = json;
    for (unsigned i = 0; i < length; i++) {
        char ch = bytes[i];
        if (quoted) {
            if (escape) { escape = false; continue; }
            if (ch == '\\') {
                if (i + 5 < length && !memcmp(bytes + i, "\\u0000", 6)) return false;
                escape = true;
            } else if (ch == '"') quoted = false;
        } else if (ch == '"') quoted = true;
        else if (ch == '[' || ch == ']' || (ch == '{' && ++depth > 1)) return false;
        else if (ch == '}' && (!depth || --depth)) return false;
    }
    if (quoted || depth) return false;
    rt_mutex_take(&receive_mutex, RT_WAITING_FOREVER);
    json_used = 0;
    const cJSON_Hooks hooks = {json_alloc, json_free};
    const char *end = NULL;
    cJSON *root = cJSON_ParseWithLengthHooks(json, length, &end, false, &hooks);
    bool accepted = false;
    cJSON *op = cJSON_GetObjectItemCaseSensitive(root, "op");
    cJSON *request = cJSON_GetObjectItemCaseSensitive(root, "r");
    cJSON *instance = cJSON_GetObjectItemCaseSensitive(root, "n");
    cJSON *revision = cJSON_GetObjectItemCaseSensitive(root, "v");
    cJSON *state = cJSON_GetObjectItemCaseSensitive(root, "state");
    cJSON *context = cJSON_GetObjectItemCaseSensitive(root, "c");
    cJSON *text = cJSON_GetObjectItemCaseSensitive(root, "m");
    bool fields_valid = true;
    for (cJSON *field = root ? root->child : NULL; field; field = field->next) {
        if (!field->string || (strcmp(field->string, "op") && strcmp(field->string, "r") &&
            strcmp(field->string, "n") && strcmp(field->string, "v") &&
            strcmp(field->string, "state") && strcmp(field->string, "c") && strcmp(field->string, "m"))) fields_valid = false;
        for (cJSON *other = field->next; other; other = other->next)
            if (other->string && field->string && !strcmp(field->string, other->string)) fields_valid = false;
    }
    while (end && end < (const char *)json + length && (*end == ' ' || *end == '\r' || *end == '\n' || *end == '\t')) end++;
    if (fields_valid && cJSON_IsObject(root) && end == (const char *)json + length && cJSON_GetArraySize(root) == 3 &&
        cJSON_IsString(op) && !strcmp(op->valuestring, "text") && cJSON_IsString(context) &&
        hexadecimal_id(context->valuestring) && cJSON_IsString(text) && valid_text(text->valuestring) &&
        active_input && !strcmp(active_voice, context->valuestring)) {
        memcpy(storage.voice.context, context->valuestring, 33);
        memcpy(storage.voice.text, text->valuestring, strlen(text->valuestring) + 1);
        storage.voice.input = active_input; storage.voice.pending = true; storage.voice.error = false; accepted = true;
    } else if (fields_valid && cJSON_IsObject(root) && end == (const char *)json + length && cJSON_GetArraySize(root) == 3 &&
        cJSON_IsString(op) && !strcmp(op->valuestring, "input_error") && cJSON_IsString(context) &&
        hexadecimal_id(context->valuestring) && cJSON_IsString(text) &&
        (!strcmp(text->valuestring, "too_long") || !strcmp(text->valuestring, "unavailable")) &&
        active_input && !strcmp(active_voice, context->valuestring)) {
        memcpy(storage.voice.context, context->valuestring, 33);
        storage.voice.input = active_input; storage.voice.pending = storage.voice.error = true; accepted = true;
    }
    bool target_valid = fields_valid && cJSON_IsObject(root) && end == (const char *)json + length &&
        cJSON_IsString(instance) && hexadecimal_id(instance->valuestring) &&
        cJSON_IsNumber(revision) && revision->valuedouble >= 1 && revision->valuedouble <= UINT32_MAX &&
        revision->valuedouble == (uint32_t)revision->valuedouble && storage.count < 8;
    if (target_valid && cJSON_IsString(op) && !strcmp(op->valuestring, "removed") && cJSON_GetArraySize(root) == 3) {
        reply_event_t *event = &storage.events[storage.count++];
        memset(event, 0, sizeof(*event)); event->removed = true;
        memcpy(event->instance, instance->valuestring, 33);
        event->revision = (uint32_t)revision->valuedouble;
        accepted = true;
    } else if (target_valid &&
        cJSON_IsString(op) && !strcmp(op->valuestring, "result") &&
        !text && ((!context && cJSON_GetArraySize(root) == 5) ||
            (cJSON_GetArraySize(root) == 6 && cJSON_IsString(context) &&
             (!strcmp(context->valuestring, "none") || !strcmp(context->valuestring, "pending") ||
              !strcmp(context->valuestring, "closed") || !strcmp(context->valuestring, "failed")))) &&
        cJSON_IsString(request) && hexadecimal_id(request->valuestring) &&
        cJSON_IsString(instance) && hexadecimal_id(instance->valuestring) &&
        cJSON_IsNumber(revision) && revision->valuedouble >= 1 && revision->valuedouble <= UINT32_MAX &&
        revision->valuedouble == (uint32_t)revision->valuedouble && cJSON_IsString(state) && storage.count < 8) {
        notification_reply_state_t value = NOTIFICATION_REPLY_NONE;
        if (!strcmp(state->valuestring, "sent") || !strcmp(state->valuestring, "already_sent")) value = NOTIFICATION_REPLY_SENT;
        else if (!strcmp(state->valuestring, "failed")) value = NOTIFICATION_REPLY_FAILED;
        else if (!strcmp(state->valuestring, "unknown")) value = NOTIFICATION_REPLY_UNKNOWN;
        else if (!strcmp(state->valuestring, "dismissed")) value = NOTIFICATION_REPLY_DISMISSED;
        if (value != NOTIFICATION_REPLY_NONE) {
            reply_event_t *event = &storage.events[storage.count++];
            memset(event, 0, sizeof(*event));
            memcpy(event->request, request->valuestring, 33);
            memcpy(event->instance, instance->valuestring, 33);
            event->revision = (uint32_t)revision->valuedouble;
            event->state = value;
            accepted = true;
        }
    }
    cJSON_DeleteWithHooks(root, &hooks);
    rt_mutex_release(&receive_mutex);
    if (accepted) {
        lvgl_msg_t message = {.type = LVGL_MSG_TYPE_NOTIFICATION_REPLY};
        lvgl_send_msg(message);
    }
    return accepted;
}
extern bool notification_reply_remove_version(const char *instance, uint32_t revision);
void notification_reply_process(void)
{
    uint32_t voice_input = 0;
    rt_mutex_take(&receive_mutex, RT_WAITING_FOREVER);
    if (storage.voice.pending) {
        if (storage.voice.input == active_input && !strcmp(storage.voice.context, active_voice)) {
            if (storage.voice.error) {
                voice_overflow = true;
                const char *draft = get_combined_voice2text();
                overflow_text_length = strlen(draft);
                overflow_text_hash = text_hash(draft);
            } else {
                voice_overflow = false;
                voice_input = active_input;
                memcpy(storage.wire, storage.voice.text, strlen(storage.voice.text) + 1);
            }
        }
        storage.voice.pending = false;
    }
    rt_mutex_release(&receive_mutex);
    if (voice_input && current_target(active_instance, active_revision) && deliver_transcript) deliver_transcript(voice_input, storage.wire);
    for (;;) {
        reply_event_t event;
        rt_mutex_take(&receive_mutex, RT_WAITING_FOREVER);
        if (!storage.count) { rt_mutex_release(&receive_mutex); break; }
        event = storage.events[0];
        memmove(storage.events, storage.events + 1, --storage.count * sizeof(event));
        rt_mutex_release(&receive_mutex);
        if (event.removed) {
            remember_processed(event.instance, event.revision, NOTIFICATION_REPLY_INVALID);
            if (!strcmp(active_instance, event.instance) && active_revision == event.revision) {
                bool submitted = notification_reply_can_finish(active_input);
                for (unsigned i = 0; i < 16; i++) {
                    const reply_work_t *work = &storage.work[i];
                    if (work->used && work->submitted && !work->dismiss && work->input == active_input &&
                        work->revision == event.revision && !strcmp(work->instance, event.instance)) submitted = true;
                }
                rt_mutex_take(&receive_mutex, RT_WAITING_FOREVER);
                active_voice[0] = 0; storage.voice.pending = false; voice_enabled = voice_ready = false;
                if (!submitted) active_input = 0;
                rt_mutex_release(&receive_mutex);
            }
            for (unsigned i = 0; i < 16; i++) {
                reply_work_t *work = &storage.work[i];
                if (work->used && work->state == NOTIFICATION_REPLY_FAILED &&
                    work->revision == event.revision && !strcmp(work->instance, event.instance))
                    memset(work, 0, sizeof(*work));
            }
            notification_reply_remove_version(event.instance, event.revision);
            continue;
        }
        for (unsigned i = 0; i < 16; i++) {
            reply_work_t *work = &storage.work[i];
            if (!work->used || strcmp(work->request, event.request) ||
                strcmp(work->instance, event.instance) || work->revision != event.revision) continue;
            if (event.state == NOTIFICATION_REPLY_DISMISSED && !work->dismiss) continue;
            work->state = event.state;
            if (event.state == NOTIFICATION_REPLY_SENT || event.state == NOTIFICATION_REPLY_DISMISSED) {
                remember_processed(event.instance, event.revision, event.state);
                work->used = false;
                memset(work->text, 0, sizeof(work->text));
                notification_reply_remove_version(event.instance, event.revision);
            } else if (event.state == NOTIFICATION_REPLY_FAILED) work->automatic = false;
            break;
        }
    }
    const uint32_t now = transport.now_ms ? transport.now_ms() : 0;
    const bool eligible = notification_reply_available() && transport.ready && transport.ready() &&
        transport.awake && transport.awake();
    static const uint32_t delays[] = {5000, 15000, 30000};
    for (unsigned i = 0; i < 16; i++) {
        reply_work_t *work = &storage.work[i];
        if (!work->used || !work->automatic) continue;
        if (work->submitted && (uint32_t)(now - work->started) >= 600000) {
            work->automatic = false;
            work->state = NOTIFICATION_REPLY_UNKNOWN;
            continue;
        }
        bool resumed = eligible && (!work->eligible || work->epoch != link_epoch);
        work->epoch = link_epoch;
        work->eligible = eligible;
        if (eligible && work->dismiss && !work->submitted) {
            int length = rt_snprintf(storage.wire, sizeof(storage.wire),
                "{\"op\":\"dismiss\",\"r\":\"%s\",\"n\":\"%s\",\"v\":%lu}",
                work->request, work->instance, (unsigned long)work->revision);
            work->submitted = true; work->started = now; work->next = now + 2000;
            if (transport.send) transport.send(storage.wire, (uint16_t)length);
            continue;
        }
        if (!eligible || (!resumed && (int32_t)(now - work->next) < 0)) continue;
        int length = rt_snprintf(storage.wire, sizeof(storage.wire),
            "{\"op\":\"%s\",\"r\":\"%s\",\"n\":\"%s\",\"v\":%lu}",
            work->dismiss ? "dismiss" : "query", work->request, work->instance, (unsigned long)work->revision);
        if (transport.send) transport.send(storage.wire, (uint16_t)length);
        work->next = now + delays[work->retry];
        if (work->retry < 2) work->retry++;
    }
}
