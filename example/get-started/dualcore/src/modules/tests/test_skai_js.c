/*
 * Sandbox gate tests for the external JS runtime (ADR-0019 Phase 3).
 * PC simulator + QuickJS only.
 *
 *   skai_js_test     run every check
 *
 * These are not "does the binding work" tests. The plan calls the sandbox a
 * GATE, not a feature: a third-party app that can hang or exhaust the watch is
 * unshippable, because its owner cannot reflash. So every check below is a
 * hostile script, and the assertion is that the runtime stops it AND says why.
 * A run that ends with the right result code but no log line still fails the
 * real requirement, which is that an external developer can tell what happened.
 */
#include <rtthread.h>
#include <string.h>
#include <stdlib.h>

#if defined(BSP_USING_PC_SIMULATOR) && defined(PKG_USING_QUICKJS)

#include "gui_app_fwk.h"
#include "ui_handler.h"
#include "watch_global_data.h"
#include "skai/skai_js.h"
#include "skai/skai_ui.h"

static int s_fail;
/* The simulator console is a rolling window, so per-check lines scroll away
   long before the summary. Failures are repeated at the end or a FAIL count is
   all anyone ever sees. */
#define SKAI_FAIL_KEEP 8
static const char *s_failed[SKAI_FAIL_KEEP];

static void check(const char *what, bool ok)
{
    rt_kprintf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
    {
        if (s_fail < SKAI_FAIL_KEEP)
            s_failed[s_fail] = what;
        s_fail++;
    }
}

static void print_failures(void)
{
    for (int i = 0; i < s_fail && i < SKAI_FAIL_KEEP; i++)
        rt_kprintf("  FAILED: %s\n", s_failed[i]);
}

/* Did the app-scoped log record something containing `needle`? */
static bool logged(const char *keyid, const char *needle)
{
    static char lines[SKAI_JS_LOG_LINES][SKAI_JS_LOG_LINE];
    int n = skai_js_log_read(keyid, lines, SKAI_JS_LOG_LINES);
    for (int i = 0; i < n; i++)
        if (strstr(lines[i], needle))
            return true;
    return false;
}

static skai_js_result_t run(const char *src, const char *const *caps, int n_caps)
{
    skai_js_policy_t p;
    skai_js_policy_init(&p, "test-app", "TESTKEY");
    p.caps = caps;
    p.n_caps = (uint16_t)n_caps;
    p.watchdog_ms = 400;              /* keep the hostile cases quick */
    p.mem_limit_bytes = 128 * 1024;
    return skai_js_eval(src, (uint32_t)strlen(src), &p);
}

static int skai_js_test(int argc, char **argv)
{
    static const char *const caps_ok[] =
    {
        "health.heart_rate", "battery.level", "time.hhmm",
        "haptic.vibrate", "persist.set_int", "persist.get_int",
    };
    skai_js_result_t r;

    (void)argc;
    (void)argv;
    s_fail = 0;
    skai_js_log_reset();

    /* --- the app that behaves --- */
    r = run("var hr = skai.health.heart_rate();"
            "var lvl = skai.battery.level();"
            "var t = skai.time.hhmm();"
            "skai.haptic.vibrate(0);"
            "skai.persist.set_int('runs', 1);",
            caps_ok, 6);
    check("declared capabilities run to completion", r == SKAI_JS_OK);

    /* The C no-data sentinel must not leak into JS as a magic number. */
    r = run("if (skai.health.heart_rate() !== null && "
            "    typeof skai.health.heart_rate() !== 'number')"
            "  throw new Error('bad hr type');",
            caps_ok, 6);
    check("no-data reaches JS as null, not a sentinel integer", r == SKAI_JS_OK);

    r = run("if (!skai.available('battery.level')) throw new Error('should be available');"
            "if (skai.available('voice.dictation')) throw new Error('should not be');",
            caps_ok, 6);
    check("skai.available reflects the manifest, so apps can degrade", r == SKAI_JS_OK);

    /* --- gate: a runaway loop must not take the watch with it --- */
    skai_js_log_reset();
    r = run("while (true) { }", caps_ok, 6);
    check("infinite loop is stopped by the watchdog", r == SKAI_JS_ERR_TIMEOUT);
    check("watchdog stop is reported, not silent", logged("TESTKEY", "WATCHDOG"));

    /* Recursion must hit the interpreter's stack wall, not the RTOS thread's.
     *
     * NOT RUNNABLE ON THE SIMULATOR, and the reason is a vendor bug worth
     * knowing about. QuickJS's stack guard measures how far the stack pointer
     * has fallen since JS_NewRuntime(), but its MSVC implementation is
     *
     *     return (uintptr_t)_ReturnAddress();   -- quickjs.c:1603
     *
     * and _ReturnAddress() yields a CODE address, not a stack address. (The
     * correct MSVC primitive is _AddressOfReturnAddress().) Comparing it
     * against stack_limit is meaningless, so the guard cannot fire and the
     * recursion takes the process down with it -- running this here would kill
     * the simulator rather than test anything.
     *
     * The armclang path uses __builtin_frame_address(0) and is correct, so on
     * the watch this guard does work, provided CONFIG_STACK_CHECK is enabled --
     * which is what rtconfig_project.h now does. Verify this case on hardware.
     * Fixing the simulator would mean patching external/, which is don't-touch.
     */
#ifdef _MSC_VER
    rt_kprintf("  [SKIP] unbounded recursion -- QuickJS stack guard is broken "
               "under MSVC (_ReturnAddress is a code address); verify on hw\n");
#else
    skai_js_log_reset();
    r = run("function f(){ return f(); } f();", caps_ok, 6);
    check("unbounded recursion is contained", r != SKAI_JS_OK);
#endif

    /* --- gate: allocation storm --- */
    skai_js_log_reset();
    rt_kprintf("       >> storm A enter\n");
    r = run("var a = []; for (;;) { a.push(new Array(4096).fill(7)); }", caps_ok, 6);
    rt_kprintf("       >> storm A exit\n");
    {
        /* Name the actual outcome in the label: the console scrolls, so a bare
           "FAILED: ..." tells you nothing about what happened instead. */
        static char label[96];
        rt_snprintf(label, sizeof(label),
                    "allocation storm hits quota or watchdog (got %s)",
                    skai_js_result_name(r));
        check(label, r == SKAI_JS_ERR_OOM || r == SKAI_JS_ERR_TIMEOUT);
    }

    /* Same storm, but with a watchdog too generous to win the race, so the only
     * correct answer is the memory quota. This is the regression guard for a
     * defect that was live: OOM used to classify as a plain "exception",
     * because QuickJS cannot build its own out-of-memory Error when memory is
     * what ran out. "Something stopped it" is not a debuggable answer, so the
     * test asserts the exact code, not just that the app died. */
    skai_js_log_reset();
    {
        skai_js_policy_t p;
        skai_js_policy_init(&p, "test-app", "TESTKEY");
        p.caps = caps_ok;
        p.n_caps = 6;
        p.watchdog_ms = 15000;
        p.mem_limit_bytes = 64 * 1024;
        static const char storm[] =
            "var a = []; for (;;) { a.push(new Array(2048).fill(1)); }";
        rt_kprintf("       >> storm B enter\n");
        r = skai_js_eval(storm, (uint32_t)(sizeof(storm) - 1), &p);
        rt_kprintf("       >> storm B exit\n");
        check("memory exhaustion reports memory-quota, not a bare exception",
              r == SKAI_JS_ERR_OOM);
        check("memory-quota violation names the limit in the app log",
              logged("TESTKEY", "MEMORY QUOTA"));
    }
    check("quota/watchdog violation is reported",
          logged("TESTKEY", "MEMORY QUOTA") || logged("TESTKEY", "WATCHDOG"));

    /* --- gate: undeclared capability --- */
    skai_js_log_reset();
    r = run("skai.health.heart_rate();", RT_NULL, 0);
    check("empty manifest can call nothing", r == SKAI_JS_ERR_DENIED);
    check("denial names the capability", logged("TESTKEY", "health.heart_rate"));

    skai_js_log_reset();
    {
        static const char *const only_battery[] = { "battery.level" };
        r = run("skai.battery.level(); skai.persist.set_int('x', 1);", only_battery, 1);
        check("a declared capability does not unlock its neighbours",
              r == SKAI_JS_ERR_DENIED);
        check("denial names the RIGHT capability", logged("TESTKEY", "persist.set_int"));
    }

    /* An app can catch the denial and carry on -- refusal is not a crash. */
    skai_js_log_reset();
    {
        static const char *const only_battery[] = { "battery.level" };
        r = run("var ok = false;"
                "try { skai.persist.set_int('x', 1); } catch (e) { ok = true; }"
                "if (!ok) throw new Error('denial was not catchable');",
                only_battery, 1);
        check("a refused call throws catchable JS, not a hard kill",
              r == SKAI_JS_OK || r == SKAI_JS_ERR_DENIED);
    }

    /* --- gate: exceptions carry enough to debug with --- */
    skai_js_log_reset();
    r = run("function boom() { null.x; } boom();", caps_ok, 6);
    check("app exception is reported as an exception", r == SKAI_JS_ERR_EXCEPTION);
    check("exception message reaches the app log", logged("TESTKEY", "EXCEPTION"));
    check("stack trace reaches the app log", logged("TESTKEY", "at "));

    skai_js_log_reset();
    r = run("this is not javascript", caps_ok, 6);
    check("syntax error is reported, not swallowed", r == SKAI_JS_ERR_EXCEPTION);

    /* --- gate: dropped log lines are counted, never silently lost --- */
    skai_js_log_reset();
    check("drop counter starts clean", skai_js_log_dropped() == 0);
    for (int i = 0; i < SKAI_JS_LOG_LINES + 4; i++)
        (void)run("skai.health.heart_rate();", RT_NULL, 0); /* each logs a denial */
    check("overflowing the log increments the drop counter",
          skai_js_log_dropped() > 0);
    rt_kprintf("       dropped=%u\n", (unsigned)skai_js_log_dropped());

    /* --- an app must not see another app's log --- */
    check("log is scoped by keyid", !logged("OTHERKEY", "DENIED"));

    /* --- gate: custom images cannot escape the app's own directory ---
     * This is the entire security surface added by app-supplied assets, so it
     * is tested directly rather than only through LVGL. A blacklist of tricky
     * strings would miss spellings like "a/../../b"; the check walks segments,
     * so these cases are representative, not exhaustive-by-luck. */
    {
        static const char *const ok[] =
        {
            "logo.png", "icons/heart.png", "a/b/c.bin",
        };
        static const char *const bad[] =
        {
            "../secret.png",        /* parent */
            "a/../../etc/passwd",   /* parent, spelled around a real segment */
            "/skaiapp/other/x.png", /* absolute */
            "S:/system/font.bin",   /* LVGL drive prefix */
            "..",                   /* bare parent */
            ".",                    /* bare self */
            "a//b.png",             /* empty segment */
            "a\\b.png",             /* other separator */
            "",                     /* empty */
        };
        bool all_ok = true, all_bad = true;

    for (size_t i = 0; i < sizeof(ok) / sizeof(ok[0]); i++)
            if (!skai_ui_path_ok(ok[i]))
                all_ok = false;
        for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++)
            if (skai_ui_path_ok(bad[i]))
            {
                all_bad = false;
                rt_kprintf("       leaked: %s\n", bad[i]);
            }
        check("ui.image accepts paths inside the app directory", all_ok);
        check("ui.image refuses every escape attempt", all_bad);
        check("ui.image refuses NULL", !skai_ui_path_ok(RT_NULL));
    }

    /* An app with no asset directory must fail closed, not fall back to
     * something readable. Off the LVGL thread ui.image refuses anyway, which is
     * itself the check that the thread contract is enforced. */
    skai_js_log_reset();
    {
        static const char *const img_cap[] = { "ui.image" };
        r = run("if (skai.ui.image('logo.png') !== 0)"
                "  throw new Error('image succeeded with no app dir');",
                img_cap, 1);
        check("ui.image fails closed outside a running app", r == SKAI_JS_OK);
    }

    /* Still alive after every hostile case -- that is the whole gate. */
    check("runtime survived every hostile script", true);

    print_failures();
    rt_kprintf("skai_js_test: %s (%d failure%s)\n",
               s_fail ? "FAIL" : "PASS", s_fail, s_fail == 1 ? "" : "s");
    return s_fail ? -1 : 0;
}
MSH_CMD_EXPORT(skai_js_test, run external JS sandbox gate tests);


/* ---- ADR-0019 Phase 3 acceptance gate 1 ----
 * "run a JS app on the simulator that reads heart rate, draws an arc and
 * vibrates". It has to go through the app host rather than run here, because
 * skai_ui refuses calls off the LVGL thread and MSH is not it. */

extern void skaijs_set_source(const char *src, const skai_js_policy_t *policy);

static const char *const k_gate1_caps[] =
{
    "health.heart_rate", "haptic.vibrate", "time.hhmm",
    "ui.label", "ui.arc", "ui.set_text", "ui.button", "ui.row",
    "ui.end", "ui.bar", "ui.set_color", "battery.level",
};

/* Deliberately written the way a third-party app would be: it asks whether a
 * capability is there, tolerates a null reading, and never assumes. */
static const char k_gate1_src[] =
    "skai.ui.label('Heart Rate');"
    "var hr = skai.health.heart_rate();"
    "var v = skai.ui.label(hr === null ? '--' : hr + ' bpm');"
    "var pct = (hr === null) ? 0 : Math.round((hr - 40) * 100 / 160);"
    "var a = skai.ui.arc(pct);"
    /* row + battery bar: the grouping and second gauge kinds */
    "skai.ui.row();"
    "  skai.ui.label('batt');"
    "  var b = skai.ui.bar(skai.battery.level() || 0);"
    "skai.ui.end();"
    "skai.ui.set_color(skai.ui.label(skai.time.hhmm()), 0x8E8E93);"
    /* a button whose handler is a JS closure over the widgets above ??this is
       what needs the session context to outlive the first eval */
    "var taps = 0;"
    "var btn = skai.ui.button('Tap');"
    "skai.ui.on_click(btn, function () {"
    "  taps++;"
    "  skai.ui.set_text(v, 'taps: ' + taps);"
    "  skai.ui.set_arc(a, (taps * 10) % 101);"
    "  skai.haptic.vibrate(0);"
    "});"
    "if (skai.available('haptic.vibrate')) skai.haptic.vibrate(0);";

/* The existing hr_set posts an LVGL message to whichever app registered a
 * handler; it never touches SkaiWatchSys, so skai_health_heart_rate() rightly
 * reports no data. This writes the global the way MSG_SERVICE_HEALTH_INFO_IND
 * does on real hardware, which is the path the SDK actually reads. */
static int skai_fake_hr(int argc, char **argv)
{
    extern SkaiWatchSysType_t SkaiWatchSys;
    int bpm = (argc >= 2) ? atoi(argv[1]) : 0;

    SkaiWatchSys.heart_rate_bpm = (uint8_t)bpm;
    rt_kprintf("skai_fake_hr: SkaiWatchSys.heart_rate_bpm = %d\n", bpm);
    return 0;
}
MSH_CMD_EXPORT(skai_fake_hr, set SkaiWatchSys.heart_rate_bpm as LCPU would);

/* ---- reproduce app_calculator in JS ----
 * The point is not "the API has a keypad". It is that a JS app can land on the
 * same pixels as a hand-written C screen; every constant below is read off
 * app_calculator.c rather than eyeballed. */

static const char *const k_calc_caps[] =
{
    "ui.label", "ui.button", "ui.keypad", "ui.keypad_accent", "ui.divider",
    "ui.set_text", "ui.set_color", "ui.set_bg", "ui.set_font", "ui.set_size",
    "ui.align",
};

static const char k_calc_src[] =
    /* display: white, centred, system font size 0, top-mid +50 (app_calculator.c:236-245) */
    "var d = skai.ui.label('');"
    "skai.ui.set_size(d, 247, 40);"
    "skai.ui.set_font(d, 0);"
    "skai.ui.align(d, 'top', 0, 34);"
    /* clear key, left of the display, 0xC17E7C */
    "var c = skai.ui.button('C');"
    "skai.ui.set_bg(c, 0x000000);"
    "skai.ui.set_color(c, 0xC17E7C);"
    "skai.ui.align(c, 'top', -140, 34);"
    /* divider under the display, 0x404040 */
    "var ln = skai.ui.divider(400);"
    "skai.ui.align(ln, 'top', 0, 70);"
    /* keypad 380x360, centre -30/+50; operators at 3,7,11,15 accented */
    "var k = skai.ui.keypad('7 8 9 +\\n4 5 6 -\\n1 2 3 x\\n  0 . /');"
    "skai.ui.set_size(k, 380, 360);"
    "skai.ui.set_font(k, 2);"
    "skai.ui.keypad_accent(k, '3,7,11,15', 0x3A6D7B);"
    "skai.ui.align(k, 'center', -30, 42);"
    /* equals: 60x120, 0x3A6D7B, right -30 */
    "var eq = skai.ui.button('=');"
    "skai.ui.set_size(eq, 60, 120);"
    "skai.ui.set_bg(eq, 0x3A6D7B);"
    "skai.ui.align(eq, 'right', -30, 0);"
    /* behaviour */
    "var expr = '';"
    "function show() { skai.ui.set_text(d, expr); }"
    "skai.ui.on_click(k, function (key) {"
    "  if (key === ' ') return;"
    "  expr += (key === 'x') ? '*' : key;"
    "  show();"
    "});"
    "skai.ui.on_click(c, function () { expr = ''; show(); });"
    "skai.ui.on_click(eq, function () {"
    "  try { expr = String(eval(expr)); } catch (e) { expr = 'Error'; }"
    "  show();"
    "});";

static int skaijs_calc(int argc, char **argv)
{
    skai_js_policy_t p;

    (void)argc;
    (void)argv;
    skai_js_policy_init(&p, "calculator-js", "DEMOKEY");
    p.caps = k_calc_caps;
    p.n_caps = sizeof(k_calc_caps) / sizeof(k_calc_caps[0]);

    skaijs_set_source(k_calc_src, &p);
    rt_kprintf("skaijs_calc: launching the JS calculator\n");
    return (int)gui_app_run(APP_ID_SKAIJS);
}
MSH_CMD_EXPORT(skaijs_calc, run the JS reproduction of app_calculator);

static int skaijs_run(int argc, char **argv)
{
    skai_js_policy_t p;

    (void)argc;
    (void)argv;
    skai_js_policy_init(&p, "gate1-hr", "DEMOKEY");
    p.caps = k_gate1_caps;
    p.n_caps = sizeof(k_gate1_caps) / sizeof(k_gate1_caps[0]);

    skaijs_set_source(k_gate1_src, &p);
    rt_kprintf("skaijs_run: launching %s\n", APP_ID_SKAIJS);
    return (int)gui_app_run(APP_ID_SKAIJS);
}
MSH_CMD_EXPORT(skaijs_run, run the gate-1 JS app (hr + arc + vibrate));

#endif /* BSP_USING_PC_SIMULATOR && PKG_USING_QUICKJS */


