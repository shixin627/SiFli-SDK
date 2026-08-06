/*
 * Skai SDK — sandboxed JavaScript runtime for external apps (ADR-0019 Phase 3).
 *
 * The point of this layer is the SANDBOX, not the API surface. Wiring
 * capabilities into JS is a few hundred lines; getting the sandbox wrong means
 * a third-party app bricks a watch its owner cannot reflash.
 *
 * Four guarantees, none of them optional:
 *
 *   1. Memory  — a hard byte ceiling per run. Counted by us, not by QuickJS:
 *                its own accounting charges every allocation a flat 8 bytes on
 *                both our toolchains, which bounds allocation COUNT and not
 *                memory. See the ceiling section in skai_js.c.
 *   2. Time    — a wall-clock deadline enforced by the interpreter's own
 *                interrupt hook, so an infinite loop is stopped mid-bytecode.
 *   3. Permission — every capability call is checked against the manifest's
 *                declared list. Undeclared is refused, never "probably fine".
 *   4. Reporting — an app that dies must say WHY. Silent death is the single
 *                most expensive failure mode for an external developer who
 *                cannot attach a debugger.
 *
 * Everything is denied by default: an empty policy can call nothing.
 */
#ifndef SKAI_JS_H
#define SKAI_JS_H

#include <stdbool.h>
#include <stdint.h>

#define SKAI_JS_APP_ID_MAX 25
#define SKAI_JS_KEYID_MAX  16

/* Defaults chosen to be survivable rather than generous. The JS heap lives in
 * PSRAM (QUICKJS_USING_PSRAM); HCPU SRAM would not survive this. */
#define SKAI_JS_DEFAULT_MEM_BYTES   (192 * 1024)
#define SKAI_JS_DEFAULT_WATCHDOG_MS 2000

typedef struct
{
    char app_id[SKAI_JS_APP_ID_MAX];
    /* Publisher key id — the app's log tag and blocklist key. Under
     * self-signing this is the only durable identity an app has. */
    char keyid[SKAI_JS_KEYID_MAX];
    uint32_t mem_limit_bytes;
    uint32_t watchdog_ms;
    /* Capabilities the manifest declared. NULL/0 means the app can call
     * nothing, which is the correct default for untrusted code. */
    const char *const *caps;
    uint16_t n_caps;
} skai_js_policy_t;

/* Why a run ended. Each value is separately reportable on purpose: "it stopped"
 * is not a debuggable answer. */
typedef enum
{
    SKAI_JS_OK = 0,
    SKAI_JS_ERR_EXCEPTION, /* the app threw, or has a syntax error */
    SKAI_JS_ERR_OOM,       /* memory quota exceeded */
    SKAI_JS_ERR_TIMEOUT,   /* watchdog fired — runaway loop */
    SKAI_JS_ERR_DENIED,    /* called a capability its manifest did not declare */
    SKAI_JS_ERR_INTERNAL,  /* runtime could not start */
} skai_js_result_t;

const char *skai_js_result_name(skai_js_result_t r);

/* Evaluate `src` under `policy`. Returns when the script finishes, throws, or
 * is stopped. Never blocks longer than policy->watchdog_ms in script code.
 *
 * With no session open this is self-contained: runtime created, script run,
 * runtime freed. Inside a session (below) it evaluates in the live context. */
skai_js_result_t skai_js_eval(const char *src, uint32_t len,
                              const skai_js_policy_t *policy);

/* ── sessions ──
 * An interactive app needs its context to outlive the first eval, because its
 * click handlers ARE JS closures — free the runtime and every button points at
 * nothing. Opening a session changes what the quotas mean:
 *
 *   - memory becomes a SESSION budget, so an app can creep toward the cap over
 *     minutes instead of blowing it at once;
 *   - the watchdog is RE-ARMED on every entry (eval and each callback), or an
 *     app would trip it simply by staying on screen.
 *
 * Not re-entrant: one app is on screen at a time. */
skai_js_result_t skai_js_open(const skai_js_policy_t *policy);
void             skai_js_close(void);
bool             skai_js_is_open(void);

/* Fill `policy` with the safe defaults; the caller then narrows it. */
void skai_js_policy_init(skai_js_policy_t *policy, const char *app_id,
                         const char *keyid);

/* ── app-scoped log (ADR-0019 debug pipeline hook) ──
 * Tagged by keyid so an app only ever sees its own lines, and bounded so a
 * chatty app cannot exhaust memory. Phase 5 attaches the BLE transport; the
 * ring and the drop counter exist now because retrofitting them is painful. */

/* Tell any open app that a capability namespace changed, so its
 * skai.on_change(...) handler runs. Called from the firmware path that already
 * knows the data moved — the topic is the namespace, e.g. "weather". A no-op
 * when no app is open or none subscribed. */
void skai_js_notify_change(const char *topic);

#define SKAI_JS_LOG_LINES 16
#define SKAI_JS_LOG_LINE  96

/* Oldest-first. Returns the number of lines copied, at most `max`. */
int  skai_js_log_read(const char *keyid, char out[][SKAI_JS_LOG_LINE], int max);

/* Lines discarded because the ring was full. Reported to the developer rather
 * than silently swallowed — otherwise they chase a bug that is not there. */
uint32_t skai_js_log_dropped(void);
void     skai_js_log_reset(void);

#endif /* SKAI_JS_H */
