/**
 * @file   skaiapp_pkg.h
 * @brief  SkaiApp — AI-generated declarative mini-app packages (SkaiLink ADR-0037).
 *
 * A "SkaiApp" is a single JSON document the phone generates with AI and pushes
 * over BLE (SKAI_LINK keys 0x13-0x15). The firmware stores the raw JSON in
 * /skaiapp/<id>.json, keeps a tiny always-resident schedule record per app
 * (skaiapp_engine), and parses the full render model ONLY while the host app
 * (APP_ID_SKAIAPP) is showing that page — HCPU SRAM is chronically full, so
 * nothing bulky may be static.
 *
 * Contract (single source of truth): SkaiLink/bridge/skaiapp-package.schema.json.
 * The phone validator is strict; this parser is TOLERANT: unknown fields and
 * unknown widget types are skipped (forward compat), higher `skaiapp` major is
 * rejected, and every copy is bounded (external-input overflow rules).
 */
#ifndef SKAIAPP_PKG_H
#define SKAIAPP_PKG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── hard caps (mirror the schema; parser clamps, never trusts) ── */
#define SKAIAPP_MAX_APPS        8
#define SKAIAPP_SCHEMA_MAJOR    0
#define SKAIAPP_ID_MAX          25   /* 24 + NUL */
#define SKAIAPP_NAME_MAX        49   /* 16 CJK chars (48B) + NUL */
#define SKAIAPP_MAX_TOP_ITEMS   12
#define SKAIAPP_MAX_ROW_CHILD   3
#define SKAIAPP_MAX_ITEMS       (SKAIAPP_MAX_TOP_ITEMS * (SKAIAPP_MAX_ROW_CHILD + 1))
#define SKAIAPP_MAX_TIMERS      4
#define SKAIAPP_MAX_REMINDERS   4
#define SKAIAPP_MAX_MEMOS       4
#define SKAIAPP_MAX_DAILY_TIMES 6
#define SKAIAPP_STRPOOL_MAX     4608 /* all texts of one package, truncating pool */
#define SKAIAPP_PKG_MAX_BYTES   8192 /* serialized package cap (schema) */
#define SKAIAPP_NOTIFY_MAX      121  /* 40 CJK chars (120B) + NUL */
#define SKAIAPP_POOL_STR_MAX    640  /* longest single pooled string (memo: 200 CJK) */

/* ── enums (wire strings → small ints at parse time) ── */
typedef enum
{
    SKAIAPP_W_LABEL = 0,
    SKAIAPP_W_VALUE,
    SKAIAPP_W_ICON,
    SKAIAPP_W_ARC,
    SKAIAPP_W_BAR,
    SKAIAPP_W_BUTTON,
    SKAIAPP_W_SPACER,
    SKAIAPP_W_ROW,     /* marker: next `row_n` items are its children */
} skaiapp_wtype_t;

/* Binds are two different things wearing one name, and only one of them was
   ever the problem (ADR-0019 Phase 2):

   - Watch state (time, battery, heart rate, weather, ...) — these used to be
     one enum value each, so every new capability meant editing four files.
     Now they are SKAIAPP_BIND_CAP plus an index into the skai dispatch table,
     resolved by NAME at parse time. Adding a capability touches nothing here.
   - Package-local references (timer/reminder/memo) — these point at objects
     inside the package itself, so the dispatch table has nothing to say about
     them. They keep their own enum values and slot index.

   Changing these values is safe: packages are stored as raw JSON and re-parsed
   on every load, so nothing on a user's watch holds these numbers. */
typedef enum
{
    SKAIAPP_BIND_NONE = 0,
    SKAIAPP_BIND_CAP,      /* bind_idx = skai dispatch table index */
    SKAIAPP_BIND_TIMER,    /* bind_idx = timer slot */
    SKAIAPP_BIND_REMINDER, /* bind_idx = reminder slot */
    SKAIAPP_BIND_MEMO,     /* bind_idx = memo slot → user-authored text */
} skaiapp_bind_t;

typedef enum
{
    SKAIAPP_ACT_NONE = 0,
    SKAIAPP_ACT_TIMER_START,
    SKAIAPP_ACT_TIMER_PAUSE,
    SKAIAPP_ACT_TIMER_RESET,
    SKAIAPP_ACT_REMINDER_TOGGLE,
} skaiapp_action_t;

/* color token index — resolved to lv colors in the renderer */
typedef enum
{
    SKAIAPP_COL_DEFAULT = 0, /* white / accent depending on widget */
    SKAIAPP_COL_WHITE, SKAIAPP_COL_GRAY, SKAIAPP_COL_ACCENT,
    SKAIAPP_COL_RED, SKAIAPP_COL_ORANGE, SKAIAPP_COL_YELLOW,
    SKAIAPP_COL_GREEN, SKAIAPP_COL_BLUE, SKAIAPP_COL_PURPLE,
} skaiapp_color_t;

/* ── render model (transient: rt_malloc'd while the page is open) ── */
typedef struct
{
    uint8_t  wtype;      /* skaiapp_wtype_t */
    uint8_t  size;       /* 0=s 1=m 2=l 3=xl (label/value); icon 0=s 1=m 2=l; arc 0=s 1=l */
    uint8_t  color;      /* skaiapp_color_t */
    uint8_t  bind;       /* skaiapp_bind_t */
    int16_t  bind_idx;   /* dispatch index or timer/reminder slot, -1 = none.
                            int16 not int8: the dispatch table grows with every
                            capability and must not silently hit a ceiling. */
    uint8_t  action;     /* skaiapp_action_t (button) */
    int8_t   action_idx; /* timer/reminder slot the action targets */
    uint8_t  ghost;      /* button style: 0 primary 1 ghost */
    uint8_t  spacer_h;   /* 8/16/24 */
    uint8_t  icon;       /* icon enum index (skaiapp_render maps to assets) */
    uint8_t  row_n;      /* SKAIAPP_W_ROW: child count that follows inline */
    int32_t  max;        /* arc/bar gauge max (steps goal); 0 = per-bind default */
    uint16_t text_off;   /* offset into strpool; 0xFFFF = none */
} skaiapp_witem_t;

typedef struct
{
    char     id[SKAIAPP_ID_MAX];
    char     name[SKAIAPP_NAME_MAX];
    uint8_t  icon;    /* app icon enum */
    uint8_t  accent;  /* skaiapp_color_t for SKAIAPP_COL_ACCENT resolution */
    uint8_t  n_items; /* flattened item count (rows inline their children) */
    uint8_t  n_timers;
    uint8_t  n_reminders;
    uint8_t  n_memos;
    skaiapp_witem_t items[SKAIAPP_MAX_ITEMS];
    /* timers/reminders live in the engine; the model only needs display bits */
    char     timer_label[SKAIAPP_MAX_TIMERS][SKAIAPP_NAME_MAX];
    /* memo text (user-authored, lives in the package) → strpool offset per slot */
    uint16_t memo_text_off[SKAIAPP_MAX_MEMOS];
    /* memo ids (needed to name the memo when the watch voice-fills it) */
    char     memo_id[SKAIAPP_MAX_MEMOS][SKAIAPP_ID_MAX];
    uint16_t strpool_used;
    char     strpool[SKAIAPP_STRPOOL_MAX];
} skaiapp_model_t;

/* NUL-safe pool string accessor ("" when absent) */
const char *skaiapp_model_text(const skaiapp_model_t *m, uint16_t off);

struct skaiapp_eng_seed; /* skaiapp_engine.h */

/**
 * Parse + semantically clamp one package document.
 * `m` is caller-owned (rt_malloc'd). When `seed_out` is non-NULL it receives
 * the engine schedule seed — the caller decides if/when to
 * skaiapp_engine_load() it (install feeds AFTER the file write succeeds; a
 * page-open display re-parse passes NULL so runtime timer state survives).
 * Returns 0 ok, -1 malformed JSON, -2 unsupported major, -3 caps violated.
 */
int skaiapp_pkg_parse(const uint8_t *json, uint32_t len, skaiapp_model_t *m,
                      struct skaiapp_eng_seed *seed_out);

/* Standard CRC-32 (IEEE, reflected — matches java.util.zip.CRC32). */
uint32_t skaiapp_crc32(const uint8_t *data, uint32_t len);

/* Decode base64 into dst; returns decoded byte count or -1. Bounded by dst_cap. */
int skaiapp_b64_decode(const char *src, uint8_t *dst, uint32_t dst_cap);

/* BLE entry points (BLE parse thread) — wired from communicate_parse_skailink.c */
void skaiapp_on_push_chunk(const uint8_t *pValue, uint16_t length);
void skaiapp_on_remove(const uint8_t *pValue, uint16_t length);

/* ack codes (ADR-0037 D3) */
#define SKAIAPP_ACK_OK          0
#define SKAIAPP_ACK_CRC         1
#define SKAIAPP_ACK_PARSE       2
#define SKAIAPP_ACK_UNSUPPORTED 3
#define SKAIAPP_ACK_STORAGE     4
#define SKAIAPP_ACK_LIMIT       5

#ifdef __cplusplus
}
#endif

#endif /* SKAIAPP_PKG_H */
