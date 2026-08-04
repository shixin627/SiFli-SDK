/*
 * Skai SDK — sandboxed QuickJS runtime (ADR-0019 Phase 3).
 *
 * The JS global `skai` is BUILT FROM THE DISPATCH TABLE at startup, not hand
 * written. That is the same rule the declarative renderer follows: a second
 * hand-maintained binding would fall behind the C layer, and the external one
 * always loses that race. Adding a capability makes it appear in JS with no
 * edit here.
 *
 * ponytail: memory ceiling and watchdog are QuickJS features (JS_SetMemoryLimit,
 * JS_SetInterruptHandler), not a custom allocator. The interpreter already
 * accounts every allocation and already polls between bytecodes; reimplementing
 * either would be more code and worse.
 */
#include <string.h>

#include <rtthread.h>
#include <stdarg.h>

#define DBG_TAG "skai.js"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "quickjs.h"

#include "skai/skai_dispatch.h"
#include "skai/skai_export.h"
#include "skai/skai_js.h"
#include "skai/skai_ui.h"

/* Last, and only for its malloc/free/realloc switch: it redirects them to the
 * heap QuickJS itself allocates from (PSRAM memheap when QUICKJS_USING_PSRAM,
 * rt_malloc otherwise). Including it earlier would put that macro in front of
 * the headers above. */
#include "cutils.h"

/* Longest argument list the marshaller handles. Capabilities beyond this are
 * refused at injection time rather than half-bound. Five because ui.align_to
 * takes (id, ref, side, dx, dy) -- anything longer is a sign the capability
 * wants a different shape, not a wider marshaller. */
#define SKAI_JS_MAX_ARGS 5
/* Bound for a string returned through the (char *out, uint32_t cap) idiom. */
#define SKAI_JS_STR_MAX  128

/* ─────────────────────────── app-scoped log ─────────────────────────── */

typedef struct
{
    char keyid[SKAI_JS_KEYID_MAX];
    char text[SKAI_JS_LOG_LINE];
} log_line_t;

static log_line_t s_log[SKAI_JS_LOG_LINES];
static uint8_t    s_log_head;   /* next write slot */
static uint8_t    s_log_used;
static uint32_t   s_log_dropped;

static void app_log(const char *keyid, const char *fmt, ...)
{
    va_list ap;
    log_line_t *l = &s_log[s_log_head];

    if (s_log_used == SKAI_JS_LOG_LINES)
    {
        /* Overwriting an unread line IS a drop; count it so the developer is
         * told rather than left with a silently short log. */
        s_log_dropped++;
    }
    else
    {
        s_log_used++;
    }

    rt_strncpy(l->keyid, keyid ? keyid : "", sizeof(l->keyid) - 1);
    l->keyid[sizeof(l->keyid) - 1] = '\0';

    va_start(ap, fmt);
    rt_vsnprintf(l->text, sizeof(l->text), fmt, ap);
    va_end(ap);

    s_log_head = (uint8_t)((s_log_head + 1) % SKAI_JS_LOG_LINES);
    LOG_I("[%s] %s", l->keyid, l->text);
}

int skai_js_log_read(const char *keyid, char out[][SKAI_JS_LOG_LINE], int max)
{
    int n = 0;
    uint8_t start;

    if (!out || max <= 0)
        return 0;

    start = (uint8_t)((s_log_head + SKAI_JS_LOG_LINES - s_log_used) % SKAI_JS_LOG_LINES);
    for (uint8_t i = 0; i < s_log_used && n < max; i++)
    {
        const log_line_t *l = &s_log[(start + i) % SKAI_JS_LOG_LINES];
        /* An app only ever sees its own lines. */
        if (keyid && strcmp(l->keyid, keyid) != 0)
            continue;
        rt_strncpy(out[n], l->text, SKAI_JS_LOG_LINE - 1);
        out[n][SKAI_JS_LOG_LINE - 1] = '\0';
        n++;
    }
    return n;
}

uint32_t skai_js_log_dropped(void) { return s_log_dropped; }

void skai_js_log_reset(void)
{
    s_log_head = 0;
    s_log_used = 0;
    s_log_dropped = 0;
}

/* ───────────────────────────── sandbox env ──────────────────────────── */

typedef struct
{
    const skai_js_policy_t *pol;
    rt_tick_t deadline;
    bool      timed_out;
    bool      denied;
    bool      oom_hit;   /* the ceiling refused an allocation -- see js_mem_* */
} js_env_t;

static int js_watchdog(JSRuntime *rt, void *opaque)
{
    js_env_t *env = (js_env_t *)opaque;
    (void)rt;
    /* Signed difference so the comparison survives tick wraparound. */
    if ((rt_int32_t)(rt_tick_get() - env->deadline) >= 0)
    {
        env->timed_out = true;
        return 1; /* non-zero interrupts the interpreter mid-bytecode */
    }
    return 0;
}

/* ─────────────────────────── the memory ceiling ─────────────────────────
 *
 * QuickJS's own ceiling does not measure bytes on either of our toolchains,
 * and cannot. js_def_malloc() accounts every block as
 *
 *     malloc_size += js_def_malloc_usable_size(ptr) + MALLOC_OVERHEAD;
 *
 * and js_def_malloc_usable_size() returns 0 for _WIN32 and for the #else
 * fallback armclang takes (external/quickjs/quickjs.c:1696). Every allocation
 * therefore costs the same 8 bytes of quota whatever its real size, so a
 * "192 KB" limit is really "24576 allocations" and an app can hold far more
 * memory than its manifest declared. Measured, not deduced: the simulator's
 * own JS_ComputeMemoryUsage dump reads "8.0 per block".
 *
 * That is a hole in guarantee 1, and it is also what crashed the simulator.
 * With bytes uncounted an app drains the heap QuickJS actually allocates from
 * before the quota notices; allocations then start failing for real, and
 * QuickJS's out-of-memory path is not robust in that state -- it dies building
 * the exception (build_backtrace -> JS_DefineProperty -> find_own_property, on
 * a shape a failed allocation left behind). A ceiling that trips while the
 * heap still has room keeps that path out of trouble entirely.
 *
 * So the accounting is ours: an 8-byte header per block carries the requested
 * size, which is what makes byte accounting possible with no usable-size
 * primitive to ask.
 *
 * ponytail: still not a custom heap. Allocation itself stays the platform's
 * (PSRAM memheap on the watch, rt_malloc on the simulator) via the same
 * cutils.h switch QuickJS uses -- only the counting changed. */

typedef union
{
    size_t size;   /* bytes the caller asked for */
    double align;  /* keeps the block behind it 8-byte aligned, as JSValue needs */
} js_hdr_t;

#define JS_HDR_BYTES  sizeof(js_hdr_t)
#define js_hdr(p)     (((js_hdr_t *)(p)) - 1)
#define js_block(h)   ((void *)((js_hdr_t *)(h) + 1))

/* Once the ceiling has refused the app, QuickJS still has to BUILD the
 * out-of-memory exception, and every allocation on that path must succeed or
 * it corrupts the object it is decorating. This reserve is that headroom. It
 * opens only after the app has already been stopped, and closes again on the
 * next entry (arm_watchdog), so an app cannot spend it. */
static size_t js_mem_cap(JSMallocState *s)
{
    const js_env_t *env = (const js_env_t *)s->opaque;
    size_t cap = s->malloc_limit;

    if (env && env->oom_hit && cap <= ((size_t)-1) / 2)
        cap += cap / 4;
    return cap;
}

static void js_mem_refused(JSMallocState *s)
{
    js_env_t *env = (js_env_t *)s->opaque;

    /* Latched at the source, so classify_failure never has to infer the cause
     * from an exception the interpreter may have had no memory to build. */
    if (env)
        env->oom_hit = true;
}

static void *js_mem_malloc(JSMallocState *s, size_t size)
{
    js_hdr_t *h;
    size_t need = size + JS_HDR_BYTES;

    if (size == 0 || need < size)
        return NULL;

    if (s->malloc_size + need > js_mem_cap(s))
    {
        js_mem_refused(s);
        return NULL;
    }

    h = malloc(need);
    if (!h)
    {
        js_mem_refused(s);
        return NULL;
    }

    h->size = size;
    s->malloc_count++;
    s->malloc_size += need;
    return js_block(h);
}

static void js_mem_free(JSMallocState *s, void *ptr)
{
    js_hdr_t *h;

    if (!ptr)
        return;

    h = js_hdr(ptr);
    s->malloc_count--;
    s->malloc_size -= h->size + JS_HDR_BYTES;
    free(h);
}

static void *js_mem_realloc(JSMallocState *s, void *ptr, size_t size)
{
    js_hdr_t *h;
    size_t old, need;

    if (!ptr)
        return size ? js_mem_malloc(s, size) : NULL;

    h = js_hdr(ptr);
    old = h->size + JS_HDR_BYTES;

    if (size == 0)
    {
        s->malloc_count--;
        s->malloc_size -= old;
        free(h);
        return NULL;
    }

    need = size + JS_HDR_BYTES;
    if (need < size || (s->malloc_size - old) + need > js_mem_cap(s))
    {
        js_mem_refused(s);
        return NULL;
    }

    h = realloc(h, need);
    if (!h)
    {
        js_mem_refused(s);
        return NULL;
    }

    h->size = size;
    s->malloc_size = (s->malloc_size - old) + need;
    return js_block(h);
}

static size_t js_mem_usable_size(const void *ptr)
{
    return ptr ? ((const js_hdr_t *)ptr)[-1].size : 0;
}

static const JSMallocFunctions js_mem_funcs =
{
    js_mem_malloc,
    js_mem_free,
    js_mem_realloc,
    js_mem_usable_size,
};

static bool cap_declared(const skai_js_policy_t *p, const char *name)
{
    if (!p || !p->caps)
        return false; /* deny by default: no manifest, no capabilities */
    for (uint16_t i = 0; i < p->n_caps; i++)
    {
        if (p->caps[i] && strcmp(p->caps[i], name) == 0)
            return true;
    }
    return false;
}

/* ───────────────────────── capability marshalling ───────────────────── */

/* One typedef per shape that actually exists in the dispatch table. A generic
 * varargs cast would be undefined behaviour and would silently corrupt the
 * stack on a mismatch; an explicit set fails to compile instead. The key is
 * "<args>|<ret>", so adding a shape is one line here and one in
 * shape_supported(). */
typedef void     (*fn_v)(void);
typedef bool     (*fn_b)(void);
typedef int32_t  (*fn_i)(void);
typedef uint32_t (*fn_u)(void);
typedef int32_t  (*fn_str)(char *, uint32_t);
typedef bool     (*fn_b_i)(int32_t);
typedef bool     (*fn_b_u)(uint32_t);
typedef void     (*fn_v_us)(uint32_t, const char *);
typedef bool     (*fn_b_s)(const char *);
typedef int32_t  (*fn_i_s)(const char *);
typedef int32_t  (*fn_i_i)(int32_t);
typedef bool     (*fn_b_si)(const char *, int32_t);
typedef bool     (*fn_b_ss)(const char *, const char *);
typedef bool     (*fn_b_is)(int32_t, const char *);
typedef bool     (*fn_b_ii)(int32_t, int32_t);
typedef int32_t  (*fn_i_si)(const char *, int32_t);
typedef int32_t  (*fn_str_s)(const char *, char *, uint32_t);
typedef int32_t  (*fn_str_i)(int32_t, char *, uint32_t);
typedef bool     (*fn_b_iii)(int32_t, int32_t, int32_t);
typedef bool     (*fn_b_isii)(int32_t, const char *, int32_t, int32_t);
typedef bool     (*fn_b_iisii)(int32_t, int32_t, const char *, int32_t, int32_t);
typedef bool     (*fn_b_isi)(int32_t, const char *, int32_t);

/* Every shape the marshaller below implements, as "<args>|<ret>". Checked at
 * injection time so an unsupported capability is simply absent from JS with a
 * loud warning, rather than present and wrong. */
static const char *const k_shapes[] =
{
    "|V", "|B", "|I", "|U", "|S",
    "U|B", "US|V", "S|B", "S|I", "S|S", "I|I", "I|B",
    "SI|B", "SI|I", "SS|B", "IS|B", "II|B", "III|B", "ISII|B", "ISI|B",
    "IISII|B",
    "I|S",
};

static void shape_key(const skai_cap_t *c, char out[12])
{
    rt_snprintf(out, 12, "%s|%c", c->args, c->ret);
}

static bool shape_supported(const skai_cap_t *c)
{
    char key[12];

    if (strlen(c->args) > SKAI_JS_MAX_ARGS)
        return false;
    shape_key(c, key);
    for (size_t i = 0; i < sizeof(k_shapes) / sizeof(k_shapes[0]); i++)
        if (strcmp(key, k_shapes[i]) == 0)
            return true;
    return false;
}

/* Positional argument slot. One tag, one value — the previous "first string
 * wins" scheme could not express (int, string) and would have silently mapped
 * the wrong argument. */
typedef union
{
    int32_t     i;
    uint32_t    u;
    const char *s;
} arg_t;

static JSValue js_skai_call(JSContext *ctx, JSValueConst this_val,
                            int argc, JSValueConst *argv, int magic)
{
    js_env_t *env = (js_env_t *)JS_GetContextOpaque(ctx);
    const skai_cap_t *c = skai_cap_at(magic);
    arg_t a[SKAI_JS_MAX_ARGS];
    bool is_str[SKAI_JS_MAX_ARGS];
    char buf[SKAI_JS_STR_MAX];
    char key[12];
    JSValue ret;
    int n = 0;

    (void)this_val;
    memset(a, 0, sizeof(a));
    memset(is_str, 0, sizeof(is_str));

    if (!c || !env)
        return JS_ThrowInternalError(ctx, "skai: dispatch entry missing");

    /* THE permission gate. Under self-signing this check and the quotas are
     * the only security boundary there is, so it runs before anything else and
     * before any argument is even read. */
    if (!cap_declared(env->pol, c->name))
    {
        env->denied = true;
        app_log(env->pol->keyid, "DENIED %s (not in manifest capabilities)", c->name);
        return JS_ThrowReferenceError(ctx, "skai: capability '%s' not declared", c->name);
    }

    /* Read arguments positionally per the table's tag string. A missing
     * argument becomes 0 / "" rather than reading past argv. */
    for (const char *t = c->args; *t && n < SKAI_JS_MAX_ARGS; t++, n++)
    {
        JSValueConst v = (n < argc) ? argv[n] : JS_UNDEFINED;
        if (*t == 'S')
        {
            const char *cs = JS_IsUndefined(v) ? NULL : JS_ToCString(ctx, v);
            a[n].s = cs ? cs : "";
            is_str[n] = (cs != NULL);
        }
        else if (*t == 'U')
        {
            if (!JS_IsUndefined(v)) JS_ToUint32(ctx, &a[n].u, v);
        }
        else /* 'I' or 'B' */
        {
            if (!JS_IsUndefined(v)) JS_ToInt32(ctx, &a[n].i, v);
        }
    }

    shape_key(c, key);

    if      (!strcmp(key, "|V"))   { ((fn_v)c->fn)();                      ret = JS_UNDEFINED; }
    else if (!strcmp(key, "US|V")) { ((fn_v_us)c->fn)(a[0].u, a[1].s);     ret = JS_UNDEFINED; }
    else if (!strcmp(key, "|B"))   ret = JS_NewBool(ctx, ((fn_b)c->fn)());
    else if (!strcmp(key, "I|B"))  ret = JS_NewBool(ctx, ((fn_b_i)c->fn)(a[0].i));
    else if (!strcmp(key, "U|B"))  ret = JS_NewBool(ctx, ((fn_b_u)c->fn)(a[0].u));
    else if (!strcmp(key, "S|B"))  ret = JS_NewBool(ctx, ((fn_b_s)c->fn)(a[0].s));
    else if (!strcmp(key, "SI|B")) ret = JS_NewBool(ctx, ((fn_b_si)c->fn)(a[0].s, a[1].i));
    else if (!strcmp(key, "SS|B")) ret = JS_NewBool(ctx, ((fn_b_ss)c->fn)(a[0].s, a[1].s));
    else if (!strcmp(key, "IS|B")) ret = JS_NewBool(ctx, ((fn_b_is)c->fn)(a[0].i, a[1].s));
    else if (!strcmp(key, "II|B")) ret = JS_NewBool(ctx, ((fn_b_ii)c->fn)(a[0].i, a[1].i));
    else if (!strcmp(key, "III|B")) ret = JS_NewBool(ctx, ((fn_b_iii)c->fn)(a[0].i, a[1].i, a[2].i));
    else if (!strcmp(key, "ISI|B")) ret = JS_NewBool(ctx, ((fn_b_isi)c->fn)(a[0].i, a[1].s, a[2].i));
    else if (!strcmp(key, "ISII|B")) ret = JS_NewBool(ctx, ((fn_b_isii)c->fn)(a[0].i, a[1].s, a[2].i, a[3].i));
    else if (!strcmp(key, "IISII|B")) ret = JS_NewBool(ctx, ((fn_b_iisii)c->fn)(a[0].i, a[1].i, a[2].s, a[3].i, a[4].i));
    else if (!strcmp(key, "|U"))   ret = JS_NewUint32(ctx, ((fn_u)c->fn)());
    else if (!strcmp(key, "|S"))
    {
        int32_t w = ((fn_str)c->fn)(buf, sizeof(buf));
        ret = (w < 0) ? JS_NULL : JS_NewString(ctx, buf);
    }
    else if (!strcmp(key, "S|S"))
    {
        int32_t w = ((fn_str_s)c->fn)(a[0].s, buf, sizeof(buf));
        ret = (w < 0) ? JS_NULL : JS_NewString(ctx, buf);
    }
    else if (!strcmp(key, "I|S"))
    {
        int32_t w = ((fn_str_i)c->fn)(a[0].i, buf, sizeof(buf));
        ret = (w < 0) ? JS_NULL : JS_NewString(ctx, buf);
    }
    else
    {
        /* Remaining shapes all return an int32, where the C sentinel becomes
         * null in JS: an app should get a value it can test, not a magic
         * number it has to know about. */
        int32_t v;
        if      (!strcmp(key, "|I"))   v = ((fn_i)c->fn)();
        else if (!strcmp(key, "S|I"))  v = ((fn_i_s)c->fn)(a[0].s);
        else if (!strcmp(key, "I|I"))  v = ((fn_i_i)c->fn)(a[0].i);
        else if (!strcmp(key, "SI|I")) v = ((fn_i_si)c->fn)(a[0].s, a[1].i);
        else
        {
            ret = JS_ThrowInternalError(ctx, "skai: unmarshalled shape %s", key);
            goto done;
        }
        ret = (v == SKAI_NO_DATA) ? JS_NULL : JS_NewInt32(ctx, v);
    }

done:
    for (int i = 0; i < n; i++)
        if (is_str[i])
            JS_FreeCString(ctx, a[i].s);
    return ret;
}

/* skai.available(name) — lets an app degrade instead of throwing when a
 * capability is missing on this firmware or absent from its own manifest. */
static JSValue js_skai_available(JSContext *ctx, JSValueConst this_val,
                                 int argc, JSValueConst *argv)
{
    js_env_t *env = (js_env_t *)JS_GetContextOpaque(ctx);
    const char *name;
    bool ok;

    (void)this_val;
    if (argc < 1)
        return JS_NewBool(ctx, 0);
    name = JS_ToCString(ctx, argv[0]);
    if (!name)
        return JS_NewBool(ctx, 0);
    ok = (skai_cap_index(name) >= 0) && cap_declared(env->pol, name);
    JS_FreeCString(ctx, name);
    return JS_NewBool(ctx, ok);
}

static void report_exception(JSContext *ctx, const js_env_t *env);

/* ── click handlers ──
 * Registration lives here rather than in the dispatch table because a callback
 * has no projection in the capability type vocabulary. The capability model is
 * unaffected: on_click grants nothing, it only routes an event to a closure the
 * app already owns. */

static JSContext *s_session_ctx;               /* non-NULL inside a session */
static JSValue    s_handlers[SKAI_UI_SLOTS];
static bool       s_handlers_init;

static void clear_handlers(void)
{
    if (!s_handlers_init)
        return;
    for (int i = 0; i < SKAI_UI_SLOTS; i++)
    {
        if (s_session_ctx)
            JS_FreeValue(s_session_ctx, s_handlers[i]);
        s_handlers[i] = JS_UNDEFINED;
    }
}

/* Every re-entry into an open context goes through here: a click, a data
 * change, and whatever comes next. Two things have to happen every time and
 * are easy to forget one at a time — re-arm the watchdog for THIS callback (a
 * session-long deadline would kill an app just for staying open; no deadline
 * would let a handler hang the LVGL thread, which is the whole screen), and
 * make sure a throwing handler is neither fatal nor silent. */
static void js_reenter(JSValue fn, const char *what, int argc, JSValueConst *argv)
{
    JSContext *ctx = s_session_ctx;
    js_env_t *env;
    JSValue r;

    if (!ctx || JS_IsUndefined(fn))
        return;

    env = (js_env_t *)JS_GetContextOpaque(ctx);
    if (env)
    {
        env->deadline = rt_tick_get()
                        + rt_tick_from_millisecond((rt_int32_t)env->pol->watchdog_ms);
        env->timed_out = false;
    }

    r = JS_Call(ctx, fn, JS_UNDEFINED, argc, argv);
    if (JS_IsException(r))
    {
        /* This is the failure an external developer is most likely to hit and
         * least able to see. */
        if (env && env->timed_out)
            app_log(env->pol->keyid, "WATCHDOG stopped a %s handler", what);
        else
            report_exception(ctx, env);
    }
    JS_FreeValue(ctx, r);
}

/* Called from LVGL when the widget is clicked. */
static void on_click_trampoline(int32_t id, const char *text, void *arg)
{
    JSContext *ctx = s_session_ctx;

    (void)arg;
    if (!ctx || id < 1 || id > SKAI_UI_SLOTS)
        return;

    {
        /* The pressed label is the handler argument, so one keypad handler
           can serve every key. */
        JSValue arg0 = JS_NewString(ctx, text ? text : "");
        js_reenter(s_handlers[id - 1], "click", 1, (JSValueConst *)&arg0);
        JS_FreeValue(ctx, arg0);
    }
}

/* ── change subscriptions ──
 *
 * The topic is the capability's NAMESPACE, taken straight off the name the app
 * passed: on_change("weather.temp") subscribes to "weather". That is why there
 * is no capability-to-topic table to keep in step with anything — a namespace
 * gains change notification the moment some firmware path calls
 * skai_js_notify_change() with its name.
 *
 * Four slots because an app that watches more than a handful of domains is
 * doing something this API is the wrong shape for. */
#define SKAI_JS_SUBS      4
#define SKAI_JS_TOPIC_MAX 16

static JSValue s_subs[SKAI_JS_SUBS];
static char    s_sub_topic[SKAI_JS_SUBS][SKAI_JS_TOPIC_MAX];
static bool    s_subs_init;

static void clear_subs(void)
{
    if (!s_subs_init)
        return;
    for (int i = 0; i < SKAI_JS_SUBS; i++)
    {
        if (s_session_ctx)
            JS_FreeValue(s_session_ctx, s_subs[i]);
        s_subs[i] = JS_UNDEFINED;
        s_sub_topic[i][0] = '\0';
    }
}

void skai_js_notify_change(const char *topic)
{
    if (!s_session_ctx || !s_subs_init || !topic)
        return;
    for (int i = 0; i < SKAI_JS_SUBS; i++)
        if (s_sub_topic[i][0] != '\0' && strcmp(s_sub_topic[i], topic) == 0)
            js_reenter(s_subs[i], "change", 0, NULL);
}

static JSValue js_skai_on_change(JSContext *ctx, JSValueConst this_val,
                                 int argc, JSValueConst *argv)
{
    js_env_t *env = (js_env_t *)JS_GetContextOpaque(ctx);
    const char *cap;
    const char *dot;
    size_t nslen;
    int slot = -1;

    (void)this_val;
    if (argc < 2 || !JS_IsFunction(ctx, argv[1]))
        return JS_ThrowTypeError(ctx, "skai.on_change: (capability, function)");

    cap = JS_ToCString(ctx, argv[0]);
    if (!cap)
        return JS_NewBool(ctx, 0);

    /* Subscribing is reading, so it needs the same declaration reading does —
       otherwise an app could watch a capability it was never granted and infer
       the value from the timing of the callbacks. */
    if (!env || !cap_declared(env->pol, cap))
    {
        if (env)
        {
            env->denied = true;
            app_log(env->pol->keyid, "DENIED on_change %s (not in manifest)", cap);
        }
        JS_FreeCString(ctx, cap);
        return JS_ThrowReferenceError(ctx, "skai: capability not declared");
    }

    dot = strchr(cap, '.');
    nslen = dot ? (size_t)(dot - cap) : strlen(cap);
    if (nslen == 0 || nslen >= SKAI_JS_TOPIC_MAX)
    {
        JS_FreeCString(ctx, cap);
        return JS_NewBool(ctx, 0);
    }

    if (!s_subs_init)
    {
        for (int i = 0; i < SKAI_JS_SUBS; i++)
        {
            s_subs[i] = JS_UNDEFINED;
            s_sub_topic[i][0] = '\0';
        }
        s_subs_init = true;
    }

    /* One handler per topic: a second on_change for the same namespace replaces
       the first rather than fanning out, which keeps the slot budget meaningful
       and the ordering unsurprising. */
    for (int i = 0; i < SKAI_JS_SUBS; i++)
    {
        if (strncmp(s_sub_topic[i], cap, nslen) == 0 && s_sub_topic[i][nslen] == '\0')
        {
            slot = i;
            break;
        }
        if (slot < 0 && s_sub_topic[i][0] == '\0')
            slot = i;
    }
    if (slot < 0)
    {
        JS_FreeCString(ctx, cap);
        return JS_NewBool(ctx, 0);   /* out of slots — refused, not silently dropped */
    }

    memcpy(s_sub_topic[slot], cap, nslen);
    s_sub_topic[slot][nslen] = '\0';
    JS_FreeCString(ctx, cap);

    JS_FreeValue(ctx, s_subs[slot]);
    s_subs[slot] = JS_DupValue(ctx, argv[1]);
    return JS_NewBool(ctx, 1);
}

static JSValue js_skai_on_click(JSContext *ctx, JSValueConst this_val,
                                int argc, JSValueConst *argv)
{
    int32_t id = 0;

    (void)this_val;
    if (argc < 2 || JS_ToInt32(ctx, &id, argv[0]) < 0)
        return JS_NewBool(ctx, 0);
    if (id < 1 || id > SKAI_UI_SLOTS)
        return JS_NewBool(ctx, 0);
    if (!JS_IsFunction(ctx, argv[1]))
        return JS_ThrowTypeError(ctx, "skai.ui.on_click: second argument must be a function");
    if (!skai_ui_on_click(id, on_click_trampoline, NULL))
        return JS_NewBool(ctx, 0); /* unknown widget id — refused, not stored */

    JS_FreeValue(ctx, s_handlers[id - 1]);
    s_handlers[id - 1] = JS_DupValue(ctx, argv[1]);
    return JS_NewBool(ctx, 1);
}

/* Build the `skai` global from the dispatch table. Namespaces come from the
 * capability names, so "weather.temp" lands at skai.weather.temp with no list
 * of namespaces maintained anywhere. */
static void inject_skai(JSContext *ctx)
{
    JSValue global = JS_GetGlobalObject(ctx);
    JSValue root = JS_NewObject(ctx);
    int n = skai_cap_count();

    JS_SetPropertyStr(ctx, root, "available",
                      JS_NewCFunction(ctx, js_skai_available, "available", 1));
    /* Not in the dispatch table for the same reason ui.on_click is not: a
       callback has no projection in the capability type vocabulary
       (ADR-0019 decision 13). It sits on the root rather than in a namespace
       because it is about capabilities in general, not about one domain. */
    JS_SetPropertyStr(ctx, root, "on_change",
                      JS_NewCFunction(ctx, js_skai_on_change, "on_change", 2));

    for (int i = 0; i < n; i++)
    {
        const skai_cap_t *c = skai_cap_at(i);
        const char *dot = strchr(c->name, '.');
        char ns[24];
        JSValue nsobj;
        size_t nslen;

        if (!dot)
            continue;
        if (!shape_supported(c))
        {
            /* Loud, not silent: a capability that exists in C but cannot be
             * projected is a generator/vocabulary gap someone must fix. */
            LOG_W("cap %s has no JS projection (ret '%c', args \"%s\")",
                  c->name, c->ret, c->args);
            continue;
        }

        nslen = (size_t)(dot - c->name);
        if (nslen >= sizeof(ns))
            continue;
        memcpy(ns, c->name, nslen);
        ns[nslen] = '\0';

        nsobj = JS_GetPropertyStr(ctx, root, ns);
        if (JS_IsUndefined(nsobj))
        {
            nsobj = JS_NewObject(ctx);
            JS_SetPropertyStr(ctx, root, ns, JS_DupValue(ctx, nsobj));
        }
        JS_SetPropertyStr(ctx, nsobj, dot + 1,
                          JS_NewCFunctionMagic(ctx, js_skai_call, dot + 1,
                                               (int)strlen(c->args),
                                               JS_CFUNC_generic_magic, i));
        JS_FreeValue(ctx, nsobj);
    }

    /* on_click hangs off the ui namespace the loop above already created. */
    {
        JSValue ui = JS_GetPropertyStr(ctx, root, "ui");
        if (!JS_IsUndefined(ui))
        {
            JS_SetPropertyStr(ctx, ui, "on_click",
                              JS_NewCFunction(ctx, js_skai_on_click, "on_click", 2));
        }
        JS_FreeValue(ctx, ui);
    }

    JS_SetPropertyStr(ctx, global, "skai", root);
    JS_FreeValue(ctx, global);
}

/* ────────────────────────────── the runner ──────────────────────────── */

const char *skai_js_result_name(skai_js_result_t r)
{
    switch (r)
    {
    case SKAI_JS_OK:            return "ok";
    case SKAI_JS_ERR_EXCEPTION: return "exception";
    case SKAI_JS_ERR_OOM:       return "memory-quota";
    case SKAI_JS_ERR_TIMEOUT:   return "watchdog";
    case SKAI_JS_ERR_DENIED:    return "permission-denied";
    default:                    return "internal";
    }
}

void skai_js_policy_init(skai_js_policy_t *p, const char *app_id, const char *keyid)
{
    if (!p)
        return;
    memset(p, 0, sizeof(*p));
    p->mem_limit_bytes = SKAI_JS_DEFAULT_MEM_BYTES;
    p->watchdog_ms = SKAI_JS_DEFAULT_WATCHDOG_MS;
    if (app_id) rt_strncpy(p->app_id, app_id, sizeof(p->app_id) - 1);
    if (keyid)  rt_strncpy(p->keyid, keyid, sizeof(p->keyid) - 1);
}

/* Report the exception with file and line. This is the highest-value debug
 * signal a JS SDK has and QuickJS hands it over for free. */
static void report_exception(JSContext *ctx, const js_env_t *env)
{
    JSValue ex = JS_GetException(ctx);
    const char *msg = JS_ToCString(ctx, ex);
    JSValue stack;

    app_log(env->pol->keyid, "EXCEPTION %s", msg ? msg : "(unprintable)");
    if (msg)
        JS_FreeCString(ctx, msg);

    stack = JS_GetPropertyStr(ctx, ex, "stack");
    if (!JS_IsUndefined(stack))
    {
        const char *st = JS_ToCString(ctx, stack);
        if (st)
        {
            app_log(env->pol->keyid, "  at %s", st);
            JS_FreeCString(ctx, st);
        }
    }
    JS_FreeValue(ctx, stack);
    JS_FreeValue(ctx, ex);
}

/* Classify a failed eval. Order matters: the watchdog and the quota both
 * surface as exceptions, so the specific cause is read first and "exception" is
 * the fallback. Collapsing these into one code is exactly the silent-death
 * problem external developers cannot debug. */
static skai_js_result_t classify_failure(JSContext *ctx, js_env_t *env)
{
    JSRuntime *rt = JS_GetRuntime(ctx);
    skai_js_result_t res;
    JSValue ex;
    const char *msg;
    bool oom;

    if (env->timed_out)
    {
        app_log(env->pol->keyid, "WATCHDOG stopped after %u ms", env->pol->watchdog_ms);
        JS_FreeValue(ctx, JS_GetException(ctx));
        return SKAI_JS_ERR_TIMEOUT;
    }
    if (env->denied)
    {
        JS_FreeValue(ctx, JS_GetException(ctx));
        return SKAI_JS_ERR_DENIED;
    }
    if (env->oom_hit)
    {
        /* The ceiling itself refused an allocation, so the cause is known
         * exactly rather than inferred from an exception QuickJS may not have
         * had the memory to build. */
        app_log(env->pol->keyid, "MEMORY QUOTA %u bytes exceeded",
                env->pol->mem_limit_bytes);
        JS_FreeValue(ctx, JS_GetException(ctx));
        return SKAI_JS_ERR_OOM;
    }

    /* Lift the quota before touching the exception: reading one ALLOCATES,
     * both for the message and for .stack. Restored below, before any app code
     * runs again, so the app never gets to spend this headroom. */
    JS_SetMemoryLimit(rt, (size_t)-1);

    ex = JS_GetException(ctx);
    msg = JS_ToCString(ctx, ex);

    /* Only the text now. An out-of-memory the CEILING caused was already
     * latched by js_mem_refused() and returned above, so this is left for one
     * QuickJS raises without an allocation of ours failing.
     *
     * This used to read `msg == NULL || strstr(...)`, because at a real OOM
     * QuickJS cannot build its InternalError("out of memory") — the object
     * needs allocations there are none of — and an unreadable exception was
     * the only signal available. The latch is a better signal, and dropping
     * the NULL half also drops the case it mislabelled: an app that throws an
     * object whose toString() itself throws is the app's bug, not a memory
     * failure, and is now reported as the exception it is. */
    oom = (msg != NULL) && (strstr(msg, "out of memory") != NULL);
    if (msg)
        JS_FreeCString(ctx, msg);
    JS_Throw(ctx, ex); /* hand it back so report_exception can read it */

    if (oom)
    {
        app_log(env->pol->keyid, "MEMORY QUOTA %u bytes exceeded",
                env->pol->mem_limit_bytes);
        JS_FreeValue(ctx, JS_GetException(ctx));
        res = SKAI_JS_ERR_OOM;
    }
    else
    {
        report_exception(ctx, env);
        res = SKAI_JS_ERR_EXCEPTION;
    }

    /* Back under quota. Matters for a session, where the app keeps running: a
     * single bad allocation must not silently buy it an unlimited heap. */
    JS_SetMemoryLimit(rt, env->pol->mem_limit_bytes);
    return res;
}

/* Session state. Kept in statics rather than on a caller's stack because the
 * context now outlives the call that created it. */
static JSRuntime       *s_session_rt;
static js_env_t         s_session_env;
static skai_js_policy_t s_session_policy;   /* our own copy: the caller's may go */

static void arm_watchdog(js_env_t *env)
{
    env->deadline = rt_tick_get()
                    + rt_tick_from_millisecond((rt_int32_t)env->pol->watchdog_ms);
    env->timed_out = false;
    env->denied = false;
    env->oom_hit = false;   /* closes the out-of-memory reserve again */
}

/* Everything a sandboxed runtime needs, in one place so the session path and
 * the one-shot path cannot drift apart on a quota. */
static JSContext *runtime_start(JSRuntime **out_rt, js_env_t *env)
{
    JSRuntime *rt = JS_NewRuntime2(&js_mem_funcs, env);
    JSContext *ctx;

    if (!rt)
        return NULL;

    JS_SetMemoryLimit(rt, env->pol->mem_limit_bytes);

    /* Cyclic garbage has to be collected before the ceiling is reached or an
     * app that makes a cycle gets a false out-of-memory. QuickJS's default
     * threshold is a flat 256 KB, which sits above the whole quota; it never
     * mattered while the accounting counted blocks. */
    JS_SetGCThreshold(rt, env->pol->mem_limit_bytes / 2);

    /* Deep recursion must hit a wall inside the interpreter rather than
     * overflowing the RTOS thread stack, which would take the watch with it.
     *
     * Two things make this work, and both are easy to get wrong:
     *
     * 1. rtconfig_project.h re-enables CONFIG_STACK_CHECK. The vendored
     *    quickjs.c disables it for RT-Thread builds, which turns
     *    js_check_stack_overflow() into "return FALSE" and makes this call a
     *    silent no-op.
     * 2. The budget is DERIVED from the calling thread, not a constant. The
     *    guard trips when the stack pointer falls this far below where it was
     *    at JS_NewRuntime(); a constant larger than the host thread's stack
     *    means the native stack is already gone by the time the check fires. */
    {
        rt_thread_t self = rt_thread_self();
        size_t budget = 2048;
        if (self && self->stack_size > 4096)
            budget = (size_t)self->stack_size / 2;
        JS_SetMaxStackSize(rt, budget);
    }

    arm_watchdog(env);
    JS_SetInterruptHandler(rt, js_watchdog, env);

    ctx = JS_NewContext(rt);
    if (!ctx)
    {
        JS_FreeRuntime(rt);
        return NULL;
    }
    JS_SetContextOpaque(ctx, env);
    inject_skai(ctx);

    *out_rt = rt;
    return ctx;
}

skai_js_result_t skai_js_open(const skai_js_policy_t *policy)
{
    if (!policy)
        return SKAI_JS_ERR_INTERNAL;
    skai_js_close();

    s_session_policy = *policy;
    memset(&s_session_env, 0, sizeof(s_session_env));
    s_session_env.pol = &s_session_policy;

    s_session_ctx = runtime_start(&s_session_rt, &s_session_env);
    if (!s_session_ctx)
        return SKAI_JS_ERR_INTERNAL;

    for (int i = 0; i < SKAI_UI_SLOTS; i++)
        s_handlers[i] = JS_UNDEFINED;
    s_handlers_init = true;
    return SKAI_JS_OK;
}

void skai_js_close(void)
{
    if (!s_session_ctx)
        return;
    clear_handlers();
    clear_subs();
    s_handlers_init = false;
    JS_FreeContext(s_session_ctx);
    JS_FreeRuntime(s_session_rt);
    s_session_ctx = NULL;
    s_session_rt = NULL;
}

bool skai_js_is_open(void) { return s_session_ctx != NULL; }

skai_js_result_t skai_js_eval(const char *src, uint32_t len,
                              const skai_js_policy_t *policy)
{
    JSRuntime *rt;
    JSContext *ctx;
    js_env_t env;
    JSValue val;
    skai_js_result_t res = SKAI_JS_OK;

    if (!src || !policy)
        return SKAI_JS_ERR_INTERNAL;

    /* Inside a session, evaluate in the live context so anything the script
     * defines is still there when a click handler fires. */
    if (s_session_ctx)
    {
        arm_watchdog(&s_session_env);
        val = JS_Eval(s_session_ctx, src, len,
                      s_session_policy.app_id[0] ? s_session_policy.app_id : "<app>",
                      JS_EVAL_TYPE_GLOBAL);
        if (JS_IsException(val))
            res = classify_failure(s_session_ctx, &s_session_env);
        JS_FreeValue(s_session_ctx, val);
        return res;
    }

    /* One-shot: create, run, free. Same quotas as a session, applied through
       the same helper so the two paths cannot drift apart. */
    memset(&env, 0, sizeof(env));
    env.pol = policy;

    ctx = runtime_start(&rt, &env);
    if (!ctx)
        return SKAI_JS_ERR_INTERNAL;

    val = JS_Eval(ctx, src, len, policy->app_id[0] ? policy->app_id : "<app>",
                  JS_EVAL_TYPE_GLOBAL);
    if (JS_IsException(val))
        res = classify_failure(ctx, &env);
    JS_FreeValue(ctx, val);

    JS_FreeContext(ctx);
    JS_FreeRuntime(rt);
    return res;
}

/* gui_app_fwk asks for a script-app enumerator whenever PKG_USING_QUICKJS is
 * defined. The vendored one scans "/" for loose .js files; Skai apps are
 * installed packages under /skaiapp with a verified manifest, so an unvetted
 * filesystem scan is exactly the discovery path we do not want. Returning NULL
 * keeps script apps out of the builtin list — they launch through their host. */
#include "gui_app_fwk.h"
const builtin_app_desc_t *gui_qjs_app_list_get_next(const builtin_app_desc_t *ptr_app)
{
    (void)ptr_app;
    return RT_NULL;
}

void gui_qjs_watch_face_register(void) { }
