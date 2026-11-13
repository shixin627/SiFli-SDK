/* SPDX-License-Identifier: Apache-2.0
 * Instruction-list ingress and update orchestration. The GUI owns application;
 * producers submit complete deliveries without repairing filters or refreshing.
 */
#include <rtthread.h>
#include <string.h>
#include <sys/stat.h>
#include "dfs_posix.h"
#include "cJSON.h"
#include "communicate_parse_notify.h"
#include "communicate_protocol.h"
#include "ui_handler.h"
#include "ui_helper.h"
#include "instruction_list.h"
#include "instruction_list_render_memory.h"
#include "lv_session_pager.h"
#include "app_mem.h"
#include "communicate_parse.h"
#include "communicate_parse_skailink.h"
#include "communicate_task.h"
#include "watch_global_data.h"
#include "watch_system_interact.h"
#include "bloc_peripheral.h"

#define DBG_TAG "instruction.list"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

extern void request_instruction_image(const char *id);
extern void update_instruction_image(const char *id, const char *path);
extern void remove_custom_instruction(const char *id);
extern void clear_custom_instructions(void);
extern void instruction_list_layout_capture_focus(char *id, size_t capacity);
extern void instruction_list_layout_publish(const char *focused_id);
extern bool instruction_list_layout_prepare(const char *focused_id);
extern void instruction_list_layout_publish_prepared(void);
extern void instruction_list_layout_finish(bool committed);
extern bool instruction_list_layout_begin_candidate(void);
extern void instruction_list_layout_finish_candidate(bool accepted);
extern void instruction_list_layout_prepare_view(void);
extern void apply_pending_instruction_batch(void);
static int instruction_list_project_sessions(void);
static const instruction_list_presentation_t s_real_presentation = {
    instruction_list_layout_prepare, instruction_list_layout_publish_prepared, instruction_list_layout_finish
};
static instruction_list_presentation_t s_presentation = {
    instruction_list_layout_prepare, instruction_list_layout_publish_prepared, instruction_list_layout_finish
};

void instruction_list_bind_presentation(const instruction_list_presentation_t *presentation)
{
    s_presentation = presentation ? *presentation : s_real_presentation;
}

typedef enum { LIST_OPEN_BROWSE, LIST_OPEN_DEVICE_DRAWER, LIST_OPEN_VOICE } instruction_open_kind_t;
static instruction_open_kind_t s_open_kind;
static bool s_open_requested, s_open_changing;
static char s_open_category;

void instruction_list_cancel_open(void) { s_open_requested = false; }
static void instruction_list_try_open(void)
{
    extern void instruction_list_layout_open_browse(char category);
    extern bool instruction_list_layout_open_drawer(void), instruction_list_layout_open_voice(void), instruction_list_is_visible(void);
    instruction_open_kind_t wanted = s_open_kind;
    char category = s_open_category;
    bool opened;
    if (!s_open_requested || s_open_changing) return;
    s_open_changing = true;
    if (wanted == LIST_OPEN_DEVICE_DRAWER) opened = instruction_list_layout_open_drawer();
    else if (wanted == LIST_OPEN_VOICE) opened = instruction_list_layout_open_voice();
    else {
        instruction_list_layout_open_browse(category);
        opened = instruction_list_is_visible();
    }
    s_open_changing = false;
    if (opened && s_open_kind == wanted && s_open_category == category) s_open_requested = false;
}
bool instruction_list_open(char category)
{
    s_open_kind = LIST_OPEN_BROWSE;
    s_open_category = category == '@' || category == '/' ? category : 0;
    s_open_requested = true;
    instruction_list_try_open();
    return !s_open_requested;
}
bool instruction_list_open_device_drawer(void)
{
    extern const char *instruction_list_device_target(void);
    const char *device = instruction_list_device_target();
    if (!device || !device[0]) return false;
    s_open_kind = LIST_OPEN_DEVICE_DRAWER;
    s_open_category = 0;
    s_open_requested = true;
    instruction_list_try_open();
    return !s_open_requested;
}

bool instruction_list_open_voice(void)
{
    s_open_kind = LIST_OPEN_VOICE;
    s_open_category = 0;
    s_open_requested = true;
    instruction_list_try_open();
    return !s_open_requested;
}

typedef struct {
    char device[64];
    uint32_t epoch;
    uint8_t count;
    bool replace;
    char options[30][64];
} instruction_device_options_t;
typedef char instruction_device_budget_fits[(sizeof(instruction_device_options_t) <= 2048u) ? 1 : -1];
static instruction_device_options_t *s_device_options;
static bool s_device_pending, s_device_changing;
static uint32_t s_device_revision, s_device_epoch;
static void instruction_list_try_device(void);

const char *instruction_list_device_target(void)
{
    extern const char *instruction_list_layout_device_target(void);
    return s_device_pending ? s_device_options->device : instruction_list_layout_device_target();
}

void instruction_list_cancel_device(void)
{
    s_device_pending = false;
    if (!s_device_changing && s_device_options) {
#ifdef BSP_USING_PC_SIMULATOR
        rt_free(s_device_options);
#else
        app_cache_free(s_device_options);
#endif
        s_device_options = NULL;
    }
}
bool instruction_list_set_device_options(const char *device, const char *const *options, uint8_t count)
{
    unsigned i;
    if (!device || !device[0] || strlen(device) >= 64 || count > 30 || (count && !options)) return false;
    for (i = 0; i < count; i++) if (!options[i] || strlen(options[i]) >= 64) return false;
    bool target_changed = strcmp(instruction_list_device_target(), device) != 0;
    if (!s_device_options) {
#ifdef BSP_USING_PC_SIMULATOR
        s_device_options = rt_malloc(sizeof(*s_device_options));
#else
        s_device_options = app_cache_alloc(sizeof(*s_device_options), IMAGE_CACHE_PSRAM_ONLY);
#endif
    }
    if (!s_device_options) return false;
    rt_strncpy(s_device_options->device, device, sizeof(s_device_options->device));
    for (i = 0; i < count; i++) rt_strncpy(s_device_options->options[i], options[i], 64);
    s_device_options->count = count;
    s_device_options->replace = options != NULL;
    s_device_pending = true;
    s_device_revision++;
    if (target_changed) { s_device_epoch++; instruction_list_scope_changed(); }
    instruction_list_try_device();
    return !s_device_pending;
}

void instruction_list_close(void)
{
    extern void instruction_list_layout_close_now(void);
    s_open_requested = false;
    s_device_epoch++;
    instruction_list_cancel_device();
    instruction_list_layout_close_now();
    extern void instruction_list_restore_base(void);
    instruction_list_restore_base();
    instruction_list_set_query("");
    instruction_list_scope_changed();
}
static char s_pending_query[48];
static bool s_query_pending, s_query_scope_pending, s_query_changing;
static void instruction_list_try_query(void);

void instruction_list_set_query(const char *text)
{
    char next[sizeof(s_pending_query)];
    rt_strncpy(next, text ? text : "", sizeof(next) - 1);
    next[sizeof(next) - 1] = '\0';
    if (!s_query_pending || strcmp(next, s_pending_query) != 0)
        s_query_scope_pending = true;
    memcpy(s_pending_query, next, sizeof(next));
    s_query_pending = true;
    instruction_list_try_query();
    if (!s_query_changing) apply_pending_instruction_batch();
}
/* A separate parser budget: message admission cannot bound DOM expansion.
 * Only this GUI-thread parse uses these hooks; chat's global hooks are untouched. */
#define INSTRUCTION_JSON_BYTES (40u * 1024u)
static uint8_t *s_instruction_json;
static size_t s_instruction_json_used;

static void *CJSON_CDECL instruction_json_alloc(size_t bytes)
{
    size_t aligned = (bytes + 7u) & ~(size_t)7u;
    void *memory;
    if (aligned < bytes || aligned > INSTRUCTION_JSON_BYTES - s_instruction_json_used)
        return NULL;
    memory = s_instruction_json + s_instruction_json_used;
    s_instruction_json_used += aligned;
    return memory;
}

static void CJSON_CDECL instruction_json_free(void *memory) { (void)memory; }
static const cJSON_Hooks s_instruction_json_hooks = {instruction_json_alloc, instruction_json_free};

static cJSON *instruction_json_parse(const char *text)
{
    size_t i, length;
    unsigned depth = 0;
    bool quoted = false, escaped = false;
    const char *end = NULL;
    cJSON *root;
    if (text == NULL)
        return NULL;
    length = strlen(text);
    /* The accepted record schema is shallow. Reject excessive nesting before
       entering the recursive vendor parser, independently of the heap budget. */
    for (i = 0; i < length; i++)
    {
        char ch = text[i];
        if (quoted)
        {
            if (escaped) {
                /* cJSON exposes C strings; an encoded NUL would truncate keys
                   and identities before schema validation can inspect them. */
                if (ch == 'u' && i + 4 < length && memcmp(text + i + 1, "0000", 4) == 0) return NULL;
                escaped = false;
            }
            else if (ch == '\\') escaped = true;
            else if (ch == '"') quoted = false;
        }
        else if (ch == '"') quoted = true;
        else if (ch == '[' || ch == '{') { if (++depth > 8) return NULL; }
        else if (ch == ']' || ch == '}') { if (depth == 0) return NULL; depth--; }
    }
    if (s_instruction_json == NULL)
    {
#ifdef BSP_USING_PC_SIMULATOR
        static union { uint64_t align; uint8_t bytes[INSTRUCTION_JSON_BYTES]; } backing;
        s_instruction_json = backing.bytes;
#else
        void *storage = app_cache_alloc(INSTRUCTION_JSON_BYTES, IMAGE_CACHE_PSRAM_ONLY);
        if (storage != NULL && (((uintptr_t)storage & 0xfc000000u) != 0x60000000u))
        {
            app_cache_free(storage);
            storage = NULL;
        }
        s_instruction_json = (uint8_t *)storage;
#endif
    }
    if (s_instruction_json == NULL)
        return NULL;
    s_instruction_json_used = 0;
    root = cJSON_ParseWithLengthHooks(text, length + 1, &end, true, &s_instruction_json_hooks);
    if (root != NULL && end != text + length)
    {
        cJSON_DeleteWithHooks(root, &s_instruction_json_hooks);
        return NULL;
    }
    return root;
}

static void instruction_json_delete(cJSON *root)
{
    cJSON_DeleteWithHooks(root, &s_instruction_json_hooks);
}

static bool instruction_uint32(const cJSON *value)
{
    return cJSON_IsNumber(value) && value->valuedouble >= 0 &&
           value->valuedouble <= 4294967295.0 &&
           (double)(uint32_t)value->valuedouble == value->valuedouble;
}

static bool instruction_fields_unique(const cJSON *object)
{
    const cJSON *field, *previous;
    for (field = object->child; field != NULL; field = field->next)
        for (previous = object->child; previous != field; previous = previous->next)
            if (strcmp(previous->string, field->string) == 0) return false;
    return true;
}

static bool instruction_object_valid(const cJSON *item)
{
    static const char *strings[] = {"openApp", "svc", "ico", "sub"};
    const cJSON *id = cJSON_GetObjectItemCaseSensitive(item, "id");
    const cJSON *title = cJSON_GetObjectItemCaseSensitive(item, "title");
    const cJSON *value;
    unsigned i;
    if (!cJSON_IsObject(item) || !instruction_fields_unique(item) ||
        !cJSON_IsString(id) || !id->valuestring[0] || strlen(id->valuestring) >= 64u ||
        !cJSON_IsString(title) || !title->valuestring[0]) return false;
    value = cJSON_GetObjectItemCaseSensitive(item, "version");
    if (value && !cJSON_IsNull(value) && !instruction_uint32(value)) return false;
    value = cJSON_GetObjectItemCaseSensitive(item, "enabled");
    if (value && !cJSON_IsNull(value) && !cJSON_IsBool(value)) return false;
    for (i = 0; i < sizeof(strings) / sizeof(strings[0]); i++)
    {
        value = cJSON_GetObjectItemCaseSensitive(item, strings[i]);
        if (value && !cJSON_IsNull(value) && !cJSON_IsString(value)) return false;
    }
    value = cJSON_GetObjectItemCaseSensitive(item, "cat");
    if (value && !cJSON_IsNull(value) &&
        (!cJSON_IsString(value) || (value->valuestring[0] &&
         (value->valuestring[1] || (value->valuestring[0] != '@' && value->valuestring[0] != '/'))))) return false;
    value = cJSON_GetObjectItemCaseSensitive(item, "sw");
    if (value && !cJSON_IsNull(value) &&
        (!cJSON_IsNumber(value) || (value->valuedouble != 0 && value->valuedouble != 1))) return false;
    value = cJSON_GetObjectItemCaseSensitive(item, "trigger");
    if (value && !cJSON_IsNull(value))
    {
        const cJSON *type, *seconds;
        if (!cJSON_IsObject(value) || !instruction_fields_unique(value)) return false;
        type = cJSON_GetObjectItemCaseSensitive(value, "type");
        seconds = cJSON_GetObjectItemCaseSensitive(value, "intervalSeconds");
        if (type && !cJSON_IsNull(type) && !cJSON_IsString(type)) return false;
        if (seconds && !cJSON_IsNull(seconds) && !instruction_uint32(seconds)) return false;
    }
    return true;
}
static bool instruction_batch_valid(const cJSON *batch)
{
    extern uint8_t instruction_list_phone_capacity(void);
    extern bool instruction_list_single_device_active(void);
    const cJSON *item;
    const device_sessions_t *devices;
    int device_count = 0;
    unsigned rows;
    if (!cJSON_IsArray(batch))
        return false;
    rows = (unsigned)cJSON_GetArraySize(batch);
    if (rows > instruction_list_phone_capacity())
        return false;
    cJSON_ArrayForEach(item, batch)
    {
        const cJSON *previous;
        if (!instruction_object_valid(item))
            return false;
        for (previous = batch->child; previous != item; previous = previous->next)
            if (strcmp(cJSON_GetObjectItemCaseSensitive(previous, "id")->valuestring,
                       cJSON_GetObjectItemCaseSensitive(item, "id")->valuestring) == 0)
                return false;
    }
    /* Projection participates in the same 30-row budget. A phone row already
       representing a selected session occupies one slot, not two. */
    devices = session_list_pool(&device_count);
    if (!instruction_list_single_device_active())
    {
        int d;
        for (d = 0; d < device_count; d++)
        {
            uint8_t selected[SESSION_VISIBLE_MAX];
            int i, count = instruction_list_selected_sessions((uint8_t)d, selected, SESSION_VISIBLE_MAX);
            for (i = 0; i < count; i++)
            {
                bool present = false;
                const char *id = devices[d].items[selected[i]].id;
                cJSON_ArrayForEach(item, batch)
                    if (strcmp(cJSON_GetObjectItemCaseSensitive(item, "id")->valuestring, id) == 0)
                        present = true;
                if (!present && ++rows > instruction_list_phone_capacity())
                    return false;
            }
        }
    }
    return true;
}

/* Pull one instruction JSON object into the watch list and ensure the
   image asset exists locally (requesting it from the phone if not). Used
   by both the single-item 0x65 path and the batch 0x6B path; refactored
   out so the two stay byte-for-byte consistent. Returns true if the
   object had a usable id+title pair. */
static bool apply_one_instruction_obj(cJSON *root)
{
    extern bool instruction_list_layout_can_upsert(const char *id);
    if (!instruction_object_valid(root))
        return false;

    cJSON *j_id = cJSON_GetObjectItemCaseSensitive(root, "id");
    cJSON *j_title = cJSON_GetObjectItemCaseSensitive(root, "title");
    cJSON *j_trigger = cJSON_GetObjectItemCaseSensitive(root, "trigger");

    const char *id = cJSON_IsString(j_id) ? j_id->valuestring : "";
    if (!instruction_list_layout_can_upsert(id)) return false;
    const char *title = cJSON_IsString(j_title) ? j_title->valuestring : "";
    const char *trigger_type = "";
    LOG_D("Applying instruction: id=%s, title=%s", id, title);
    uint32_t interval_sec = 0;

    if (j_trigger)
    {
        cJSON *j_type = cJSON_GetObjectItemCaseSensitive(j_trigger, "type");
        if (cJSON_IsString(j_type))
        {
            trigger_type = j_type->valuestring;
            if (strcmp(trigger_type, "interval") == 0)
            {
                cJSON *j_sec = cJSON_GetObjectItemCaseSensitive(j_trigger,
                                                   "intervalSeconds");
                if (cJSON_IsNumber(j_sec))
                    interval_sec = (uint32_t)j_sec->valuedouble;
            }
        }
    }

    uint32_t version = 0;
    cJSON *j_version = cJSON_GetObjectItemCaseSensitive(root, "version");
    if (cJSON_IsNumber(j_version))
        version = (uint32_t)j_version->valuedouble;

    bool enabled = false;
    cJSON *j_enabled = cJSON_GetObjectItemCaseSensitive(root, "enabled");
    if (cJSON_IsBool(j_enabled))
        enabled = cJSON_IsTrue(j_enabled);

    /* openApp: phone marks a "just open this watch app" instruction so the
       tap runs locally with no phone relay (works while disconnected).
       Absent for normal instructions → NULL keeps the relay behaviour. */
    const char *open_app = NULL;
    cJSON *j_open_app = cJSON_GetObjectItemCaseSensitive(root, "openApp");
    if (cJSON_IsString(j_open_app) && j_open_app->valuestring[0] != '\0')
        open_app = j_open_app->valuestring;

    /* cat: '@' = chat option, '/' = action option (cross-device skaibar
       filtered views). Absent → untagged; the watch shows it in the "all"
       view and treats it as an action in the "/" view. */
    char category = 0;
    cJSON *j_cat = cJSON_GetObjectItemCaseSensitive(root, "cat");
    if (cJSON_IsString(j_cat) && j_cat->valuestring[0] != '\0')
        category = j_cat->valuestring[0];

    /* svc: the @-contact's messaging service (e.g. "messenger" / "whatsapp"), so the watch can show
       the service logo on the row's right-side indicator dot. The list push only carries the title
       (not the conv id), so the service rides over the wire. Absent for non-@ rows → no service icon. */
    const char *svc = NULL;
    cJSON *j_svc = cJSON_GetObjectItemCaseSensitive(root, "svc");
    if (cJSON_IsString(j_svc) && j_svc->valuestring[0] != '\0')
        svc = j_svc->valuestring;

    /* ico: the saved Action's TYPE (e.g. "music" / "weather" / "generic"), so the row shows
       the same glyph the phone shows in front of its name. Absent = this row isn't a saved
       Action (a calc / url / memo / ask card in the aggregated skaibar batch) → no glyph. */
    const char *ico = NULL;
    cJSON *j_ico = cJSON_GetObjectItemCaseSensitive(root, "ico");
    if (cJSON_IsString(j_ico) && j_ico->valuestring[0] != '\0')
        ico = j_ico->valuestring;

    add_or_update_custom_instruction(id, title, trigger_type,
                                     interval_sec, enabled, version,
                                     open_app);
    set_instruction_category(id, category);
    extern void set_instruction_service_icon(const char *id, const char *svc);
    set_instruction_service_icon(id, svc);
    /* Last of the three icon sources — fills only a still-empty slot (see its doc). */
    extern void set_instruction_type_icon(const char *id, const char *ico);
    set_instruction_type_icon(id, ico);
    /* sub:這一列的「大概瀏覽」小字(founder 2026-09-30)。缺席 = 清掉:批次是整份取代,
       手機不再送就代表這列現在沒有預覽了。 */
    {
        extern bool set_instruction_sub(const char *id, const char *sub);
        cJSON *j_sub = cJSON_GetObjectItemCaseSensitive(root, "sub");
        set_instruction_sub(id, cJSON_IsString(j_sub) ? j_sub->valuestring : "");
    }
    /* sw:這一列是「開關」Action(0/1 = 第一次出現時的圓點位置)。缺席 = 一般 Action,不畫開關。 */
    {
        extern void set_instruction_switch(const char *id, int sw);
        cJSON *j_sw = cJSON_GetObjectItemCaseSensitive(root, "sw");
        set_instruction_switch(id, cJSON_IsNumber(j_sw) ? (int)j_sw->valuedouble : -1);
    }

    if (id[0] != '\0')
    {
        char id_prefix[64];
        strncpy(id_prefix, id, sizeof(id_prefix) - 1);
        id_prefix[sizeof(id_prefix) - 1] = '\0';
        char *dash = strchr(id_prefix, '-');
        if (dash)
            *dash = '\0';

        char img_path[128];
        rt_snprintf(img_path, sizeof(img_path),
                    "/assets/images/instruction/%s.bin", id_prefix);

        struct stat st;
        if (stat(img_path, &st) == 0)
            update_instruction_image(id, img_path);
        else
            request_instruction_image(id);
    }
    return true;
}

/* ── Single-thread ownership fix (intermittent left-list DACCVIOL, 2026-05-28)
   ──────────────────────────────────────────────────────────────────────────
   list_items[] / list_item_count were mutated on KE_EVT2 (BLE task) by the
   instruction handlers — batch (0x6B clear+apply), single (0x65 upsert),
   dismiss (0x67 remove) — and by update_instruction_image() on the file-receive
   thread, while refresh_custom_instructions() rebuilds the UI from the same
   arrays on the LVGL/app_watc thread. No lock → cross-thread data race; the
   skaibar batch storm during voice makes them overlap → corrupted list_items[]
   → wild-pointer DACCVIOL.

   Fix: NO list_items[] mutation off the LVGL thread. Every mutator heap-copies
   its payload and enqueues an op; the LVGL thread drains the queue (single
   owner), applying each op in order, then rebuilds the UI once. cJSON_Parse
   also moves off KE_EVT2's near-full 4KB stack as a bonus. */
typedef enum
{
    INST_OP_SNAPSHOT,
    INST_OP_DEVICE_OPTIONS,
    INST_OP_BATCH,  /* a = JSON array string  — replace-all   */
    INST_OP_SINGLE, /* a = JSON object string — upsert one    */
    INST_OP_REMOVE, /* a = id string          — remove one    */
    INST_OP_IMAGE,  /* a = id string, b = image path          */
    INST_OP_AVATAR, /* a = bot avatar key (檔案剛落地)        */
} inst_op_kind_t;

typedef struct
{
    inst_op_kind_t kind;
    char *a;
    char *b;
} inst_op_t;

/* A bounded native heap owns pending payloads, including messages popped
 * for processing. Its backing allocation never falls back to system SRAM. */
#define INST_OP_Q_LEN 24
static inst_op_t s_inst_op_q[INST_OP_Q_LEN];
static uint8_t s_inst_op_head = 0; /* next to drain  */
static uint8_t s_inst_op_tail = 0; /* next free slot */
static bool s_pending_gap;
static bool s_recovery_batch_pending;
static bool s_peer_snapshot_capable;
static bool s_linked;
static bool s_link_pending, s_link_changing;
static bool s_was_eligible;
static uint32_t s_request_counter;
static uint32_t s_active_request;
static uint32_t s_completed_request;
static uint32_t s_scope;
static uint32_t s_ingress_generation;
static uint32_t s_retry_at;
static uint8_t s_retry_index;
static bool s_sessions_pending;

void instruction_list_sessions_changed(void)
{
    extern bool instruction_list_filter_active(void);
    s_sessions_pending = true;
    if (instruction_list_filter_active())
        return;
    apply_pending_instruction_batch();
}

static uint32_t list_now_ms(void) { return (uint32_t)(((uint64_t)rt_tick_get() * 1000u) / RT_TICK_PER_SECOND); }
static bool list_ready(void) { return SkaiWatchSys.connected_to_phone; }
static bool list_awake(void) { return !is_hcpu_suspend(); }
static const instruction_list_transport_t s_real_transport = {
    list_now_ms, list_ready, list_awake, commu_send_instruction_list_resync
};
static instruction_list_transport_t s_transport = {
    list_now_ms, list_ready, list_awake, commu_send_instruction_list_resync
};

static void instruction_list_drive_sync(void);

static struct rt_mutex s_ingress_lock;
static struct rt_memheap s_ingress_heap;
static bool s_ingress_ready;
#define INGRESS_BUDGET_BYTES (16u * 1024u)
/* The margin includes outer app-cache block metadata, pointers and flags;
 * the native heap's per-allocation metadata lives inside its backing store. */
#define INGRESS_ARENA_BYTES (INGRESS_BUDGET_BYTES - sizeof(s_inst_op_q) - \
                            sizeof(s_ingress_lock) - sizeof(s_ingress_heap) - 128u)

static bool instruction_ingress_prepare(void)
{
    void *storage;
    if (s_ingress_ready)
        return true;
#ifdef BSP_USING_PC_SIMULATOR
    static union { uint64_t align; uint8_t bytes[INGRESS_ARENA_BYTES]; } backing;
    storage = backing.bytes;
#else
    storage = app_cache_alloc(INGRESS_ARENA_BYTES, IMAGE_CACHE_PSRAM_ONLY);
    if (storage != NULL && (((uintptr_t)storage & 0xfc000000u) != 0x60000000u))
    {
        app_cache_free(storage);
        storage = NULL;
    }
#endif
    if (storage == NULL)
        return false;
    if (rt_memheap_init(&s_ingress_heap, "ilist_rx", storage, INGRESS_ARENA_BYTES) != RT_EOK)
    {
#ifndef BSP_USING_PC_SIMULATOR
        app_cache_free(storage);
#endif
        return false;
    }
    s_ingress_ready = true;
    return true;
}

static int instruction_list_ingress_init(void)
{
    rt_mutex_init(&s_ingress_lock, "ilist_rx", RT_IPC_FLAG_PRIO);
    /* Try again on a later delivery if PSRAM was temporarily unavailable. */
    instruction_ingress_prepare();
    return RT_EOK;
}
INIT_APP_EXPORT(instruction_list_ingress_init);

static void inst_op_free(inst_op_t *op);

static void instruction_ingress_invalidate(void)
{
    s_ingress_generation++;
    while (s_inst_op_head != s_inst_op_tail)
    {
        inst_op_free(&s_inst_op_q[s_inst_op_head]);
        s_inst_op_head = (uint8_t)((s_inst_op_head + 1) % INST_OP_Q_LEN);
    }
    s_inst_op_head = s_inst_op_tail = 0;
    s_pending_gap = true;
    s_recovery_batch_pending = false;
    s_active_request = 0;
}

static bool instruction_list_stage(bool *staged)
{
    if (*staged) return true;
    *staged = instruction_list_layout_begin_candidate();
    if (!*staged)
    {
        rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
        instruction_ingress_invalidate();
        rt_mutex_release(&s_ingress_lock);
    }
    return *staged;
}

/* A query is also a complete presentation transaction. Retain the intent when
   the renderer is busy or out of memory; the GUI drain retries it. */
static void instruction_list_try_query(void)
{
    extern bool instruction_list_layout_set_query(const char *text);
    extern const char *instruction_list_text_filter(void);
    extern void instruction_list_session_inject_begin(void);
    extern void instruction_list_session_inject_end(void);
    char focused_id[64], wanted[sizeof(s_pending_query)];
    uint32_t generation;
    bool presented;
    if (!s_query_pending || s_query_changing) return;
    s_query_changing = true;
    instruction_list_layout_capture_focus(focused_id, sizeof(focused_id));
    if (!instruction_list_layout_begin_candidate())
    {
        s_query_changing = false;
        return;
    }
    if (!instruction_list_layout_set_query(s_pending_query))
    {
        instruction_list_layout_finish_candidate(false);
        s_query_pending = s_query_scope_pending = s_query_changing = false;
        return;
    }
    rt_strncpy(wanted, instruction_list_text_filter(), sizeof(wanted) - 1);
    wanted[sizeof(wanted) - 1] = '\0';
    memcpy(s_pending_query, wanted, sizeof(wanted));
    if (s_query_scope_pending) instruction_list_scope_changed();
    s_query_scope_pending = false;
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    generation = s_ingress_generation;
    rt_mutex_release(&s_ingress_lock);
    instruction_list_session_inject_begin();
    /* Clearing a failed search must be able to expose the last consistent base
       and resume recovery, even when the newer session pool cannot fit yet. */
    int projected = (wanted[0] == '\0' && instruction_list_get_status().needs_sync)
                        ? 0 : instruction_list_project_sessions();
    instruction_list_session_inject_end();
    if (projected < 0)
    {
        instruction_list_layout_finish_candidate(false);
        rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
        instruction_ingress_invalidate();
        rt_mutex_release(&s_ingress_lock);
        s_query_changing = false;
        return;
    }
    instruction_list_layout_prepare_view();
    presented = s_presentation.prepare(focused_id);
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    presented = presented && generation == s_ingress_generation && strcmp(wanted, s_pending_query) == 0;
    if (presented) s_presentation.publish();
    instruction_list_layout_finish_candidate(presented);
    if (presented) s_query_pending = false;
    rt_mutex_release(&s_ingress_lock);
    s_presentation.finish(presented);
    if (presented) session_list_refresh_view();
    s_query_changing = false;
}
static void instruction_list_try_device(void)
{
    extern bool instruction_list_layout_set_device_options(const char *, const char *const *, uint8_t);
    extern void instruction_list_save_base(void);
    extern void instruction_list_session_inject_begin(void);
    extern void instruction_list_session_inject_end(void);
    const char *options[30];
    char focused_id[64];
    uint32_t generation, revision = s_device_revision;
    bool presented;
    unsigned i;
    if (!s_device_pending || s_device_changing || s_query_pending || s_query_changing) return;
    s_device_changing = true;
    if (!instruction_list_layout_begin_candidate()) { s_device_changing = false; return; }
    instruction_list_layout_capture_focus(focused_id, sizeof(focused_id));
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    generation = s_ingress_generation;
    rt_mutex_release(&s_ingress_lock);
    for (i = 0; i < s_device_options->count; i++) options[i] = s_device_options->options[i];
    instruction_list_session_inject_begin();
    presented = instruction_list_layout_set_device_options(s_device_options->device,
        s_device_options->replace ? options : NULL, s_device_options->count);
    instruction_list_session_inject_end();
    instruction_list_layout_prepare_view();
    presented = presented && s_presentation.prepare(focused_id);
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    presented = presented && s_device_pending && revision == s_device_revision && generation == s_ingress_generation;
    if (presented) {
        instruction_list_save_base();
        s_presentation.publish();
    }
    instruction_list_layout_finish_candidate(presented);
    if (presented) s_device_pending = false;
    rt_mutex_release(&s_ingress_lock);
    s_presentation.finish(presented);
    s_device_changing = false;
    if (!s_device_pending) instruction_list_cancel_device();
}

/* Both a programmatic open and the first frame of a finger reveal use this
   transaction. Gesture/animation details remain in the GUI adapter. */
static bool instruction_list_prepare_open_frame(char category, bool choose_entry)
{
    extern bool instruction_list_layout_open_context(char category, bool preparing);
    extern void instruction_list_layout_choose_entry(void);
    extern void instruction_list_session_inject_begin(void);
    extern void instruction_list_session_inject_end(void);
    uint32_t generation;
    int projected;
    bool presented;
    char focus[64];
    instruction_list_layout_capture_focus(focus, sizeof(focus));
    if (s_query_pending || s_query_changing || !instruction_list_layout_begin_candidate()) return false;
    if (instruction_list_layout_open_context(category, true)) instruction_list_scope_changed();
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    generation = s_ingress_generation;
    rt_mutex_release(&s_ingress_lock);
    instruction_list_session_inject_begin();
    projected = instruction_list_get_status().needs_sync ? 0 : instruction_list_project_sessions();
    instruction_list_session_inject_end();
    instruction_list_layout_prepare_view();
    if (choose_entry) instruction_list_layout_choose_entry();
    presented = projected >= 0 && s_presentation.prepare(choose_entry ? NULL : focus);
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    presented = presented && generation == s_ingress_generation;
    if (presented) s_presentation.publish();
    instruction_list_layout_finish_candidate(presented);
    if (presented && projected >= 0) s_sessions_pending = false;
    if (projected < 0) instruction_ingress_invalidate();
    rt_mutex_release(&s_ingress_lock);
    if (projected >= 0) s_presentation.finish(presented);
    instruction_list_layout_open_context(category, false);
    return presented;
}
bool instruction_list_prepare_open(char category)
{
    return instruction_list_prepare_open_frame(category, true);
}
bool instruction_list_prepare_voice_open(char category)
{
    return instruction_list_prepare_open_frame(category, false);
}
void instruction_list_bind_transport(const instruction_list_transport_t *transport)
{
    s_transport = transport ? *transport : s_real_transport;
    s_was_eligible = false;
}

instruction_list_status_t instruction_list_get_status(void)
{
    instruction_list_status_t status;
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    status.needs_sync = s_pending_gap;
    status.supports_resend = s_peer_snapshot_capable;
    status.active_request = s_active_request;
    status.completed_request = s_completed_request;
    status.scope = s_scope;
    rt_mutex_release(&s_ingress_lock);
    status.render_reserved_bytes = instruction_render_reserved();
    status.render_peak_bytes = instruction_render_high_water();
    return status;
}

void instruction_list_scope_changed(void)
{
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    s_scope++;
    s_ingress_generation++;
    s_active_request = 0;
    s_was_eligible = false;
    rt_mutex_release(&s_ingress_lock);
}

void instruction_list_link_changed(bool connected)
{
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    if (s_linked != connected)
    {
        s_linked = connected;
        s_link_pending = true;
        s_peer_snapshot_capable = false;
        instruction_ingress_invalidate();
        instruction_list_scope_changed();
    }
    rt_mutex_release(&s_ingress_lock);
    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_APPLY_INSTRUCTION_BATCH};
    lvgl_send_msg(msg);
}

/* Link visibility is a local intent: do not lose it when rendering fails or
   the phone update stream is frozen. Producers never touch the GUI model. */
static bool instruction_list_try_link(void)
{
    extern void instruction_list_layout_set_phone_connected(bool connected);
    bool connected, presented;
    uint32_t generation;
    char focus[64];
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    if (!s_link_pending) { rt_mutex_release(&s_ingress_lock); return true; }
    if (s_link_changing) { rt_mutex_release(&s_ingress_lock); return false; }
    s_link_changing = true;
    connected = s_linked;
    generation = s_ingress_generation;
    rt_mutex_release(&s_ingress_lock);
    instruction_list_layout_capture_focus(focus, sizeof(focus));
    if (!instruction_list_layout_begin_candidate()) {
        s_link_changing = false;
        return false;
    }
    instruction_list_layout_set_phone_connected(connected);
    instruction_list_layout_prepare_view();
    presented = s_presentation.prepare(focus);
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    presented = presented && generation == s_ingress_generation && connected == s_linked;
    if (presented) s_presentation.publish();
    instruction_list_layout_finish_candidate(presented);
    if (presented) s_link_pending = false;
    rt_mutex_release(&s_ingress_lock);
    s_presentation.finish(presented);
    s_link_changing = false;
    return presented;
}

static char *inst_alloc(size_t size)
{
    char *memory = NULL;
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    if (instruction_ingress_prepare())
        memory = (char *)rt_memheap_alloc(&s_ingress_heap, size);
    if (memory == NULL)
        instruction_ingress_invalidate();
    rt_mutex_release(&s_ingress_lock);
    return memory;
}

static char *inst_strdup(const char *s)
{
    if (!s)
        return RT_NULL;
    size_t n = strlen(s) + 1;
    char *p = inst_alloc(n);
    if (p)
        memcpy(p, s, n);
    return p;
}

static void inst_op_free(inst_op_t *op)
{
    if (op->a)
        rt_memheap_free(op->a);
    if (op->b)
        rt_memheap_free(op->b);
    op->a = op->b = RT_NULL;
}

/* Transfers ownership of heap-allocated a / b (either may be NULL). Safe from
   any thread; the LVGL thread drains via apply_pending_instruction_batch(). */
static bool inst_op_enqueue(inst_op_kind_t kind, char *a, char *b)
{
    inst_op_t rejected;
    rejected.kind = kind;
    rejected.a = a;
    rejected.b = b;
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    if (s_pending_gap && !s_recovery_batch_pending && kind != INST_OP_BATCH &&
        kind != INST_OP_SNAPSHOT)
    {
        rt_mutex_release(&s_ingress_lock);
        inst_op_free(&rejected);
        return false;
    }
    uint8_t next = (uint8_t)((s_inst_op_tail + 1) % INST_OP_Q_LEN);
    if (next == s_inst_op_head)
    {
        /* A suffix after a lost operation is not a valid update stream.
           Keep the committed list, discard this round, await a full base. */
        instruction_ingress_invalidate();
        if (kind != INST_OP_BATCH)
        {
            rt_mutex_release(&s_ingress_lock);
            inst_op_free(&rejected);
            LOG_W("instruction queue lost continuity; full list required");
            return false;
        }
        next = 1;
    }
    s_inst_op_q[s_inst_op_tail].kind = kind;
    s_inst_op_q[s_inst_op_tail].a = a;
    s_inst_op_q[s_inst_op_tail].b = b;
    s_inst_op_tail = next;
    if (s_pending_gap && (kind == INST_OP_SNAPSHOT ||
                         (kind == INST_OP_BATCH && !s_peer_snapshot_capable)))
        s_recovery_batch_pending = true;
    rt_mutex_release(&s_ingress_lock);

    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_APPLY_INSTRUCTION_BATCH};
    lvgl_send_msg(msg);
    return true;
}

bool instruction_list_refresh_device_options(const char *device, const char *const *options, uint8_t count)
{
    instruction_device_options_t *update;
    const char *current = instruction_list_device_target();
    unsigned i;
    if (!device || !device[0] || strlen(device) >= 64 || count > 30 || (count && !options) ||
        strcmp(device, current) != 0) return false;
    for (i = 0; i < count; i++) if (!options[i] || strlen(options[i]) >= 64) return false;
    update = (instruction_device_options_t *)inst_alloc(offsetof(instruction_device_options_t, options) + count * 64u);
    if (!update) return false;
    rt_strncpy(update->device, device, sizeof(update->device));
    update->epoch = s_device_epoch;
    update->count = count;
    update->replace = options != NULL;
    for (i = 0; i < count; i++) rt_strncpy(update->options[i], options[i], 64);
    return inst_op_enqueue(INST_OP_DEVICE_OPTIONS, (char *)update, RT_NULL);
}

bool instruction_list_submit_phone(const instruction_list_delivery_t *delivery)
{
    inst_op_kind_t kind;
    char *copy;
    if (delivery == NULL) return false;
    if (delivery->command == SET_CONFIG_COMMAND_ID && delivery->key == KEY_PHONE_OS_VERSION)
    {
        if (!delivery->payload || delivery->length != 2)
            return false;
        bool capable = (((const uint8_t *)delivery->payload)[1] & 1u) != 0;
        /* Connection control is independent of the frozen, lossy data FIFO. */
        rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
        if (s_peer_snapshot_capable != capable)
        {
            s_peer_snapshot_capable = capable;
            s_ingress_generation++;
            s_active_request = 0;
            s_was_eligible = false;
        }
        rt_mutex_release(&s_ingress_lock);
        lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_APPLY_INSTRUCTION_BATCH};
        lvgl_send_msg(msg);
        return true;
    }
    else if (delivery->command == SKAI_LINK_COMMAND_ID && delivery->key == KEY_INSTRUCTION_LIST_SYNC)
        kind = INST_OP_SNAPSHOT;
    else if (delivery->command == 0 || delivery->command == NOTIFY_COMMAND_ID)
    {
        switch (delivery->key)
        {
        case KEY_SKAI_CREATION_INSTRUCTIONS_BATCH: kind = INST_OP_BATCH; break;
        case KEY_SKAI_CREATION_INSTRUCTIONS: kind = INST_OP_SINGLE; break;
        case KEY_DISMISS_SKAI_INSTRUCTION: kind = INST_OP_REMOVE; break;
        default: return false;
        }
    }
    else
        return false;
    if (!delivery->payload || delivery->length == 0 || delivery->length > L2_REASSEMBLY_MAX_BYTES ||
        memchr(delivery->payload, '\0', delivery->length) != NULL)
    {
        rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
        instruction_ingress_invalidate();
        rt_mutex_release(&s_ingress_lock);
        lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_APPLY_INSTRUCTION_BATCH};
        lvgl_send_msg(msg);
        return false;
    }
    copy = inst_alloc((size_t)delivery->length + 1);
    if (copy == NULL)
        return false;
    memcpy(copy, delivery->payload, delivery->length);
    copy[delivery->length] = '\0';
    return inst_op_enqueue(kind, copy, RT_NULL);
}

/* Exposed for the layout's update_instruction_image(), which is also called on
   the file-receive thread when an icon download completes. */
void instruction_op_enqueue_image(const char *id, const char *path)
{
    char *id_copy = inst_strdup(id);
    char *path_copy = inst_strdup(path);
    if (!id_copy || !path_copy)
    {
        if (id_copy)
            rt_memheap_free(id_copy);
        if (path_copy)
            rt_memheap_free(path_copy);
        LOG_E("instruction image op: alloc failed");
        return;
    }
    inst_op_enqueue(INST_OP_IMAGE, id_copy, path_copy);
}

/* 同上,但給 Bot 頭像:檔案在收檔執行緒落地,列的 img_path 只能在 LVGL 執行緒改。 */
void instruction_op_enqueue_avatar(const char *av)
{
    char *av_copy = inst_strdup(av);
    if (!av_copy)
    {
        LOG_E("bot avatar op: alloc failed");
        return;
    }
    inst_op_enqueue(INST_OP_AVATAR, av_copy, RT_NULL);
}

/* A validated later full list supersedes its prefix without changing the
   frozen presentation. Keep the suffix: it is ordered after that full list. */
static void instruction_list_coalesce_frozen(void)
{
    uint8_t cursor;
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    for (cursor = s_inst_op_head; cursor != s_inst_op_tail;
         cursor = (uint8_t)((cursor + 1) % INST_OP_Q_LEN))
    {
        inst_op_t *op = &s_inst_op_q[cursor];
        cJSON *root;
        bool valid;
        if (op->kind != INST_OP_BATCH || (s_pending_gap && s_peer_snapshot_capable)) continue;
        root = instruction_json_parse(op->a);
        valid = instruction_batch_valid(root);
        instruction_json_delete(root);
        if (!valid) continue;
        while (s_inst_op_head != cursor)
        {
            inst_op_free(&s_inst_op_q[s_inst_op_head]);
            s_inst_op_head = (uint8_t)((s_inst_op_head + 1) % INST_OP_Q_LEN);
        }
    }
    rt_mutex_release(&s_ingress_lock);
}

/* LVGL thread only — dispatched from process_lvgl_message() on
   LVGL_MSG_TYPE_APPLY_INSTRUCTION_BATCH. Drains every queued op (so list_items[]
   is only ever mutated here) then rebuilds the UI once. */
void apply_pending_instruction_batch(void)
{
    extern bool instruction_list_filter_active(void);
    bool applied_any = false;
    bool staged = false, replacement_base = false;
    bool sessions_were_pending = s_sessions_pending;
    char focused_id[64];
    uint32_t completed_candidate = 0, completed_scope = 0;
    uint32_t generation;
    if (!instruction_list_try_link()) return;
    instruction_list_try_query();
    if (s_query_pending || s_query_changing) return;
    instruction_list_try_device();
    if (s_device_pending || s_device_changing) return;
    instruction_list_try_open();
    if (s_open_requested || s_open_changing) return;
    /* Both GUI-message and timer entry points must retain the entire FIFO
       while searching, including removals and image/property updates. */
    if (instruction_list_filter_active())
    {
        instruction_list_coalesce_frozen();
        s_was_eligible = false;
        return;
    }
    instruction_list_layout_capture_focus(focused_id, sizeof(focused_id));
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    generation = s_ingress_generation;
    rt_mutex_release(&s_ingress_lock);
    for (;;)
    {
        inst_op_t op;
        rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
        if (s_inst_op_head == s_inst_op_tail)
        {
            rt_mutex_release(&s_ingress_lock);
            break;
        }
        op = s_inst_op_q[s_inst_op_head];
        s_inst_op_q[s_inst_op_head].a = RT_NULL;
        s_inst_op_q[s_inst_op_head].b = RT_NULL;
        s_inst_op_head = (uint8_t)((s_inst_op_head + 1) % INST_OP_Q_LEN);
        rt_mutex_release(&s_ingress_lock);
        if (s_pending_gap && !replacement_base && op.kind != INST_OP_SNAPSHOT &&
            (op.kind != INST_OP_BATCH || s_peer_snapshot_capable))
        {
            inst_op_free(&op);
            continue;
        }
        switch (op.kind)
        {
        case INST_OP_BATCH:
        case INST_OP_SNAPSHOT:
        {
            cJSON *root = op.a ? instruction_json_parse(op.a) : RT_NULL;
            cJSON *items = root;
            if (op.kind == INST_OP_SNAPSHOT)
            {
                cJSON *operation = cJSON_GetObjectItemCaseSensitive(root, "op");
                cJSON *id = cJSON_GetObjectItemCaseSensitive(root, "id");
                cJSON *scope = cJSON_GetObjectItemCaseSensitive(root, "scope");
                /* The first accepted reply defines this transaction's cut.
                   A duplicate cannot erase deltas admitted after that cut. */
                if (completed_candidate != 0 || !s_pending_gap || !s_peer_snapshot_capable || s_active_request == 0 ||
                    !cJSON_IsString(operation) || strcmp(operation->valuestring, "snapshot") != 0 ||
                    !cJSON_IsNumber(id) || id->valuedouble != (double)s_active_request ||
                    !cJSON_IsNumber(scope) || scope->valuedouble != (double)s_scope)
                {
                    instruction_json_delete(root);
                    break;
                }
                items = cJSON_GetObjectItemCaseSensitive(root, "items");
            }
            if (instruction_batch_valid(items))
            {
                if (!instruction_list_stage(&staged))
                {
                    instruction_json_delete(root);
                    break;
                }
                if (op.kind == INST_OP_SNAPSHOT)
                {
                    completed_candidate = s_active_request;
                    completed_scope = s_scope;
                }
                replacement_base = true;
                applied_any = true;
                int incoming = cJSON_GetArraySize(items);
                int n = 0;
                cJSON *item;
                const char *switch_ids[30];
                unsigned switch_count = 0;
                extern void instruction_list_layout_prepare_switches(const char *const *, unsigned);
                cJSON_ArrayForEach(item, items)
                    if (cJSON_IsNumber(cJSON_GetObjectItemCaseSensitive(item, "sw")))
                        switch_ids[switch_count++] = cJSON_GetObjectItemCaseSensitive(item, "id")->valuestring;
                instruction_list_layout_prepare_switches(switch_ids, switch_count);
                /* Replace-all: clear before appending so phone-side deletions
                   take effect. App items < app_base_count are preserved. */
                clear_custom_instructions();
                cJSON_ArrayForEach(item, items)
                {
                    if (apply_one_instruction_obj(item))
                        n++;
                }
                LOG_I("BATCH instructions: %d of %d item(s) applied", n,
                      incoming);
            }
            else
            {
                rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
                instruction_ingress_invalidate();
                rt_mutex_release(&s_ingress_lock);
                LOG_W("BATCH instructions rejected: invalid complete list");
            }
            if (root)
                instruction_json_delete(root);
            break;
        }
        case INST_OP_SINGLE:
        {
            cJSON *root = op.a ? instruction_json_parse(op.a) : RT_NULL;
            if (instruction_object_valid(root) && instruction_list_stage(&staged) && apply_one_instruction_obj(root))
                applied_any = true;
            else
            {
                rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
                instruction_ingress_invalidate();
                rt_mutex_release(&s_ingress_lock);
            }
            instruction_json_delete(root);
            break;
        }
        case INST_OP_DEVICE_OPTIONS:
        {
            extern bool instruction_list_single_device_active(void);
            extern const char *instruction_list_layout_device_target(void);
            extern bool instruction_list_layout_set_device_options(const char *, const char *const *, uint8_t);
            instruction_device_options_t *update = (instruction_device_options_t *)op.a;
            const char *options[30];
            if (!update || update->epoch != s_device_epoch || !instruction_list_single_device_active() ||
                strcmp(update->device, instruction_list_layout_device_target()) != 0) break;
            if (!instruction_list_stage(&staged)) break;
            for (unsigned i = 0; i < update->count; i++) options[i] = update->options[i];
            if (instruction_list_layout_set_device_options(update->device, update->replace ? options : NULL, update->count))
                applied_any = true;
            break;
        }
        case INST_OP_REMOVE:
            if (!instruction_list_stage(&staged)) break;
            applied_any = true;
            if (op.a)
                remove_custom_instruction(op.a);
            break;
        case INST_OP_IMAGE:
            if (!instruction_list_stage(&staged)) break;
            applied_any = true;
            if (op.a && op.b)
                update_instruction_image(op.a, op.b);
            break;
        case INST_OP_AVATAR:
            if (!instruction_list_stage(&staged)) break;
            applied_any = true;
            if (op.a)
            {
                extern void instruction_list_on_bot_avatar_file(const char *path);
                instruction_list_on_bot_avatar_file(op.a);
            }
            break;
        }
        inst_op_free(&op);
    }

    if (applied_any || (s_sessions_pending && !s_pending_gap))
    {
        if (instruction_list_stage(&staged))
        {
            int projected = instruction_list_project_sessions();
            if (projected < 0)
            {
                s_sessions_pending = true;
                rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
                instruction_ingress_invalidate();
                rt_mutex_release(&s_ingress_lock);
            }
            else
            {
                s_sessions_pending = false;
                applied_any = projected != 0 || applied_any;
            }
        }
    }
    if (applied_any)
    {
        bool presented;
        instruction_list_layout_prepare_view();
        presented = s_presentation.prepare(focused_id);
        rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
        presented = presented && generation == s_ingress_generation;
        if (presented) s_presentation.publish();
        instruction_list_layout_finish_candidate(presented);
        if (presented && replacement_base)
        {
            s_pending_gap = false;
            s_recovery_batch_pending = false;
            s_active_request = 0;
            s_retry_index = 0;
        }
        if (presented && completed_candidate != 0 && completed_scope == s_scope && !s_pending_gap)
            s_completed_request = completed_candidate;
        rt_mutex_release(&s_ingress_lock);
        s_presentation.finish(presented);
        staged = false;
        if (!presented)
        {
            s_sessions_pending = s_sessions_pending || sessions_were_pending;
            rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
            instruction_ingress_invalidate();
            rt_mutex_release(&s_ingress_lock);
            completed_candidate = 0;
        }
    }
    if (staged) instruction_list_layout_finish_candidate(false);
    instruction_list_drive_sync();
}


static void instruction_list_drive_sync(void)
{
    static const uint32_t delays[] = {2000, 5000, 15000, 30000};
    extern bool instruction_list_filter_active(void);
    bool eligible = s_pending_gap && s_peer_snapshot_capable &&
                    s_transport.ready() && s_transport.awake() &&
                    !instruction_list_filter_active();
    uint32_t now;
    uint32_t id;
    uint32_t scope, generation;
    if (!eligible)
    {
        s_was_eligible = false;
        return;
    }
    now = s_transport.now_ms();
    if (s_was_eligible && (int32_t)(now - s_retry_at) < 0)
        return;
    s_was_eligible = true;
    id = ++s_request_counter;
    if (id == 0)
        id = ++s_request_counter;
    rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
    scope = s_scope;
    generation = s_ingress_generation;
    rt_mutex_release(&s_ingress_lock);
    if (s_transport.request(id, scope))
    {
        rt_mutex_take(&s_ingress_lock, RT_WAITING_FOREVER);
        if (scope == s_scope && generation == s_ingress_generation && s_pending_gap)
            s_active_request = id;
        else
            s_was_eligible = false; /* A loss during send needs a fresh request now. */
        rt_mutex_release(&s_ingress_lock);
    }
    s_retry_at = now + delays[s_retry_index];
    if (s_retry_index < 3)
        s_retry_index++;
}

/* 這一列該不該併進「同一個 Bot 已經有代表列」裡 —— sp_visible_for_device 與注入迴圈
   共用同一條判準。

   平常一律併(一列一個 Bot)。搜尋開著時只併「因為 **bot 名**比中而留下來的」那些 ——
   使用者講的是 Bot 的名字,要的是那一個 Bot,不是它底下五間房各一列(而且那五列的標題
   都會是同一個 bot 名,根本分不出來)。因為 **session 標題**比中而留下的照舊一列一間房。 */
static bool sp_row_collapses_by_bot(const session_meta_t *s)
{
    extern bool instruction_list_filter_active(void);
    extern bool instruction_list_title_matches_filter(const char *title);
    if (s->bot[0] == '\0')
        return false; /* 舊桌面沒送 profile:彼此不是「同一個 Bot」,併起來會憑空少房 */
    if (!instruction_list_filter_active())
        return true;
    return instruction_list_title_matches_filter(s->bot) &&
           !instruction_list_title_matches_filter(s->title);
}

/* 每台桌面在清單上要露出哪幾筆 —— founder 2026-08-24:「每個設備只會有最新的 5 個
   session,除非我有搜尋 session,才改成每個設備搜尋到的最新 5 個」。

   為什麼是挑、不是少存:搜尋字是在**手錶本地**比對標題的(instruction list 的
   pack_text_filter),池子只有露出來的那幾筆的話,搜尋永遠只能命中畫面上已經看得到的
   東西 —— 那就不叫搜尋了。所以儲存維持 SESSION_PAGER_MAX 筆/台,這裡只決定哪
   幾筆變成清單列;搜尋開著時同一支函式改用「比中查詢的最新 5 筆」。

   ── 名額是給 BOT 的,不是給 SESSION 的(founder 2026-09-02) ──────────────────
   收斂**必須在取前 N 筆之前**做。原本的順序是先挑「這台最新的 5 個 session」、注入時
   才併成 Bot 列:一個 Bot 被連續談過五次就把五個名額全吃光,收斂完只剩它一列,同一台
   桌面上的其他 Bot 一個都進不來。真機:手機送了 1234 五間 + default 五間,1234 的五間
   都比較新 → 手錶只看得到 1234,default 從頭到尾沒出現過。
   這個順序在 8/24 是對的(那時左頁列的是 session);8/30 改成「一列一個 Bot」之後,
   cap 就套錯層了 —— 要的是「最新 5 個 **Bot**」。

   out[] 收 items[] 的 index,ts 由新到舊(同 ts / 都沒 ts 時取陣列在前者 —— 手機推
   來的順序本來就是新→舊)。回傳筆數。 */
int instruction_list_selected_sessions(uint8_t slot, uint8_t *out, uint8_t max)
{
    /* 空查詢時恆真 = 平常就是「最新 N 筆」。 */
    extern bool instruction_list_title_matches_filter(const char *title);
    int s_device_count = 0;
    const device_sessions_t *s_devices = session_list_pool(&s_device_count);
    const device_sessions_t *dev;
    bool taken[SESSION_PAGER_MAX];
    int n = 0;
    if (s_devices == NULL || slot >= s_device_count || out == NULL || max <= 0)
        return 0;
    dev = &s_devices[slot];
    memset(taken, 0, sizeof(taken));
    while (n < max)
    {
        int best = -1;
        bool dup = false;
        for (int k = 0; k < dev->count && k < SESSION_PAGER_MAX; k++)
        {
            if (taken[k])
                continue;
            /* 標題**或 bot 名**任一比中就留(founder 2026-08-30:「點下面的麥克風輸入
               文字要先查看有沒有輸入文字的 bot 或是 session 存在」)。左頁改成列 Bot 之後
               只比標題就不對了:講一個 Bot 的名字會一個都沒中,於是清單給你「開新對話」——
               明明那個 Bot 就在上面。比中之後 real_matches>0,合成的「開新對話」列本來
               就會被丟掉,所以「先查有沒有、有就別開新的」這半自然成立。 */
            if (!instruction_list_title_matches_filter(dev->items[k].title) &&
                !instruction_list_title_matches_filter(dev->items[k].bot))
                continue;
            if (best < 0 || dev->items[k].ts > dev->items[best].ts)
                best = k;
        }
        if (best < 0)
            break;
        taken[best] = true;
        /* 已經有同 Bot 的代表列了就跳過,而且**不佔名額** —— 這一行就是「名額給 Bot」。
           標成 taken 後繼續掃,所以它底下更舊的那幾間也不會反覆被挑到。 */
        if (sp_row_collapses_by_bot(&dev->items[best]))
        {
            for (int j = 0; j < n && !dup; j++)
                dup = (strncmp(dev->items[out[j]].bot, dev->items[best].bot,
                               SESSION_BOT_LEN) == 0);
        }
        if (dup)
            continue;
        out[n++] = (uint8_t)best;
    }
    return n;
}

static int instruction_list_project_sessions(void)
{
    extern bool instruction_list_layout_set_route(unsigned index, const char *id, const char *device,
                                                  const char *bot, const char *name);
    extern bool instruction_list_layout_finish_routes(unsigned count);
    int s_device_count = 0;
    const device_sessions_t *s_devices = session_list_pool(&s_device_count);
    extern void add_or_update_custom_instruction(const char *id, const char *title,
                                                 const char *trigger_type,
                                                 uint32_t interval_sec, bool enabled,
                                                 uint32_t version, const char *open_app);
    extern void set_instruction_category(const char *id, char cat);
    extern void remove_custom_instruction(const char *id);
    extern uint8_t return_total_list_count(void);
    extern const char *instruction_list_export_id(uint8_t i);
    extern const char *instruction_list_export_title(uint8_t i);

    /* 滑鼠 app 單設備模式(搜尋抽屜/立起面板)佔用共享清單期間**整段跳過**:那份清單是
       「正在控制那一台電腦」面板的鏡像(0x03),把 s_devices 全部設備的 session upsert
       進去=別台的 session 混進單設備搜尋結果(2026-08-15 真機抓到)。抽屜收掉
       restore_base 還原錶盤清單後,下一次 0x20/輪詢會照常補注入,不會漏。 */
    {
        extern bool instruction_list_single_device_active(void);
        if (instruction_list_single_device_active())
            return false;
    }

    bool changed = false;

    /* 這一輪每台桌面要露出的那幾筆(見 sp_visible_for_device)。清單上的 conv 段
       **只由這份集合決定** —— 沒被挑中的(第 6 筆之後、或搜尋沒比中的)就跟被桌面
       刪掉的一樣從清單移除,只是仍留在 s_devices 裡供下一次搜尋比對。 */
    uint8_t vis[SESSION_DEVICE_MAX][SESSION_VISIBLE_MAX];
    int vis_n[SESSION_DEVICE_MAX];
    for (int d = 0; d < SESSION_DEVICE_MAX; d++)
        vis_n[d] = (d < s_device_count) ? instruction_list_selected_sessions((uint8_t)d, vis[d], SESSION_VISIBLE_MAX) : 0;

    /* ── 一列一個 BOT(founder 2026-08-30:「改成顯示前幾個最新的 bot」)──────────
       同一個 Bot 被談過六次就佔掉六列、把別的 Bot 擠出畫面。代表列 = 該 Bot 最新的那個
       session,所以列上的 id 仍是真的 session key,點下去的路由一個位元都不用改;左右滑
       切同 Bot 的其他 session 由 chat page 的 switcher 接手(wf_chat_bind_switcher)。

       收斂本身已經做在 sp_visible_for_device 裡(2026-09-02 移進去的):在那裡做,五個
       名額才是給 BOT 的。留在這裡做等於先讓最吵的 Bot 吃光名額、再把它自己收成一列,
       同一台桌面的其他 Bot 就永遠進不來(真機:1234 五間都比 default 新 → 手錶只
       看得到 1234)。vis[][] 因此已經是「每台最新 N 個 Bot 的代表列」,下面的移除迴圈
       拿它判斷「這輪還在不在」也還是同一份真相。 */

    {
        extern bool instruction_list_layout_sessions_fit(const char *const *ids, unsigned n);
        const char *ids[SESSION_DEVICE_MAX * SESSION_VISIBLE_MAX];
        unsigned n = 0;
        for (int d = 0; d < s_device_count; d++)
            for (int v = 0; v < vis_n[d]; v++) ids[n++] = s_devices[d].items[vis[d][v]].id;
        if (!instruction_list_layout_sessions_fit(ids, n)) return -1;
    }
    /* 移除:清單裡 conv: 開頭、但這一輪沒被挑中的(桌面刪了 session、或落在最新 5 筆
       之外 / 搜尋沒比中)。 */
    for (int i = (int)return_total_list_count() - 1; i >= 0; i--)
    {
        const char *iid = instruction_list_export_id((uint8_t)i);
        if (iid == NULL || strncmp(iid, "conv:", 5) != 0)
            continue;
        bool still = false;
        for (int d = 0; d < s_device_count && !still; d++)
            for (int v = 0; v < vis_n[d] && !still; v++)
                still = (strcmp(s_devices[d].items[vis[d][v]].id, iid) == 0);
        if (!still)
        {
            remove_custom_instruction(iid);
            changed = true;
        }
    }

    /* upsert:按 ts **舊→新**(注入順序 = 清單由上而下的順序,所以最新的落在 session
       段最下面 —— founder 2026-08-12:「希望它是按照時間最新的在最下面」)。
       R81(founder 2026-09-10:「bot 在 actions 最下面,每次進去都在 bot 的位置」):
       conv 段整段搬到 actions **下面**(instruction_list_order_conv_items),所以最新的
       session = 整份清單最後一項 = 進場落點;往上捲是愈舊的 bot,再往上才是 actions。title 沒變就不動(add_or_update
       恆觸發重繪成本)。無 ts 的(舊韌體/桌面沒給)ts=0,排在最上面=最舊,合理。 */
    struct
    {
        uint8_t slot;
        uint8_t idx;
        uint32_t ts;
    } order[SESSION_DEVICE_MAX * SESSION_VISIBLE_MAX];
    int cnt = 0;
    for (int d = 0; d < s_device_count; d++)
        for (int v = 0; v < vis_n[d]; v++)
        {
            uint8_t k = vis[d][v];
            order[cnt].slot = (uint8_t)d;
            order[cnt].idx = k;
            order[cnt].ts = s_devices[d].items[k].ts;
            cnt++;
        }
    for (int i = 1; i < cnt; i++)
    {
        /* stable insertion, ts 小(舊)者在前 → 最新的排在最後 = 清單最下面 */
        int j = i;
        while (j > 0 && order[j - 1].ts > order[j].ts)
        {
            uint8_t t_slot = order[j].slot, t_idx = order[j].idx;
            uint32_t t_ts = order[j].ts;
            order[j] = order[j - 1];
            order[j - 1].slot = t_slot;
            order[j - 1].idx = t_idx;
            order[j - 1].ts = t_ts;
            j--;
        }
    }
    for (int o = 0; o < cnt; o++)
    {
        const session_meta_t *s = &s_devices[order[o].slot].items[order[o].idx];
        const device_sessions_t *device = &s_devices[order[o].slot];
        changed = instruction_list_layout_set_route((unsigned)o, s->id, device->id, s->bot, device->name) || changed;
        /* R11(founder):來源設備不併進標題,改由 instruction list 在標題右下角
           畫小字半透明副標(session_list_device_name_for 反查)。 */
        /* 標題的規則只有一條硬性要求:**注入的標題必須能通過 pack_text_filter 的同一支
           比對**,否則這一列會在打包階段被再篩掉一次,`real_matches` 歸零、清單改給
           「開新對話」——明明東西就在上面(那正是上面那段註解警告的「兩邊語意分岔」)。

           於是:session 標題比中就用 session 標題(使用者搜的是那間房);否則用 bot 名
           (沒搜尋時恆走這條 = 一列一個 Bot;搜尋時走這條的一定是因 bot 名比中而留下的,
           所以 bot 名必然也比得中)。兩種情況都滿足上面那條要求。
           沒有 bot 的舊資料一律退回 session 標題。 */
        extern bool instruction_list_filter_active(void);
        extern bool instruction_list_title_matches_filter(const char *t);
        bool title_hit = instruction_list_filter_active() &&
                         instruction_list_title_matches_filter(s->title);
        const char *title = (!title_hit && s->bot[0])
                                ? s->bot
                                : (s->title[0] ? s->title : "Session");
        bool same = false;
        uint8_t n = return_total_list_count();
        for (uint8_t i = 0; i < n; i++)
        {
            const char *iid = instruction_list_export_id(i);
            const char *it = instruction_list_export_title(i);
            if (iid && it && strcmp(iid, s->id) == 0 && strcmp(it, title) == 0)
            {
                same = true;
                break;
            }
        }
        if (!same)
        {
            add_or_update_custom_instruction(s->id, title, "", 0, false, 0, "");
            changed = true;
        }
        set_instruction_category(s->id, '@'); /* 冪等 */
        /* Bot 頭像(founder 2026-08-31:「右邊的設備名稱改成顯示跟電腦一樣的 bot 頭像」)。
           這裡只交出鍵,檔案在不在、要不要跟手機要,由 instruction list 那邊決定 —— 它才是
           畫那個右緣槽位的人。搜尋時列的是 session 不是 Bot,頭像仍屬於同一個 Bot,照給。 */
        {
            extern void set_instruction_avatar(const char *id, const char *av);
            set_instruction_avatar(s->id, s->av);
        }
        /* 預覽行:代表列(該 Bot 最新的 session)帶的 preview 就是這個 Bot 的「大概瀏覽」。
           手機只在代表列填,其他列 preview 為空;旁表以 id 為鍵,空的只清自己那一格。
           內容變了要重建清單,label 才會補建。 */
        {
            extern bool set_instruction_sub(const char *id, const char *sub);
            extern bool set_instruction_sub_long(const char *id, const char *text);
            if (set_instruction_sub(s->id, s->preview))
                changed = true;
            /* 超過旁表 95B 的那句 headline 另存一份長的,卡片才顯示得完整 */
            if (set_instruction_sub_long(s->id, s->glance ? s->preview : ""))
                changed = true;
        }
    }

    /* R27:每次注入都把 conv 段**強制排成上面算好的 ts 舊→新順序**。只靠 upsert 的呼叫
       順序不夠 —— add_or_update 對已存在的 id 位置原地不動,所以早就在清單裡的 session
       會停在第一次插入時的位置(founder:改了排序但「最新的還是在最上面」)。這支自己回報
       有沒有真的動到順序,沒動就不 refresh,輪詢才不會每 5s 重繪。 */
    {
        extern bool instruction_list_order_conv_items(const char *const *ids, uint8_t n);
        const char *ids[SESSION_DEVICE_MAX * SESSION_VISIBLE_MAX];
        uint8_t idn = 0;
        for (int o = 0; o < cnt && idn < (uint8_t)(sizeof(ids) / sizeof(ids[0])); o++)
            ids[idn++] = s_devices[order[o].slot].items[order[o].idx].id;
        if (instruction_list_order_conv_items(ids, idn))
            changed = true;
    }

    LOG_W("[land] inject changed=%d devs=%d", (int)changed, s_device_count);
    changed = instruction_list_layout_finish_routes((unsigned)cnt) || changed;
    return changed;
}
