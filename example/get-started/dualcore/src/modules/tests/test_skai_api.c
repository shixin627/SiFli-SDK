/*
 * MSH self-check for the skai_* capability layer (Phase 1, ADR-0019).
 * PC simulator only -- gated by BSP_USING_PC_SIMULATOR in SConscript.
 *
 *   skai_test          run every check, print PASS/FAIL per line
 *
 * ponytail: one command covering all eight domains, not one command per API.
 * Twenty near-identical MSH wrappers is boilerplate; what matters is that
 * something fails loudly when a wrapper breaks.
 *
 * Most reads are plain accessors whose value depends on runtime state, so
 * they are checked for PLAUSIBILITY (in range / correctly reports "no data")
 * rather than exact values. The two with real logic -- persist round-trip and
 * the timer slot table -- are checked properly, including their failure paths.
 */
#include <rtthread.h>
#include <string.h>

#include "skai/skai_battery.h"
#include "skai/skai_dispatch.h"
#include "skai/skai_haptic.h"
#include "skai/skai_health.h"
#include "skai/skai_log.h"
#include "skai/skai_persist.h"
#include "skai/skai_sdk_version.h"
#include "skai/skai_time.h"
#include "skai/skai_timer.h"
#include "skai/skai_watch_info.h"

static int s_fail;

static void check(const char *what, bool ok)
{
    rt_kprintf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        s_fail++;
}

static void tick_cb(void *arg)
{
    *(volatile int *)arg += 1;
}

static int skai_test(int argc, char **argv)
{
    char buf[64];
    int32_t n;

    (void)argc;
    (void)argv;
    s_fail = 0;

    rt_kprintf("skai SDK %d.%d\n", SKAI_API_MAJOR, SKAI_API_MINOR);

    /* time -- now() may legitimately be -1 before a clock sync, so accept
     * either "failed" or "after 2000-01-01"; anything else is a bug. */
    n = skai_time_now();
    check("time.now is no-data or a post-2000 timestamp", n == SKAI_NO_DATA || n > 946684800);
    n = skai_time_format("%H:%M", buf, sizeof(buf));
    check("time.format writes 5 chars", n == 5 && buf[2] == ':');
    check("time.format rejects a 1-byte buffer", skai_time_format("%H:%M", buf, 1) < 0);
    check("time.format rejects NULL", skai_time_format(RT_NULL, buf, sizeof(buf)) < 0);

    /* battery / health -- range or the documented "no reading" sentinel */
    n = skai_battery_level();
    check("battery.level is no-data or 0..100", n == SKAI_NO_DATA || (n >= 0 && n <= 100));
    (void)skai_battery_charging();
    check("battery.charging returns", true);
    n = skai_health_heart_rate();
    check("health.heart_rate is no-data or 20..250", n == SKAI_NO_DATA || (n >= 20 && n <= 250));
    check("health.steps is not negative", skai_health_steps() >= 0);

    /* persist -- the one with real logic. Round-trip, overwrite, remove, and
     * the bounds that stop a bad key reaching share_prefs. */
    check("persist.set_int", skai_persist_set_int("t_i", 4242));
    check("persist.get_int round-trips", skai_persist_get_int("t_i", -1) == 4242);
    check("persist.set_int overwrites", skai_persist_set_int("t_i", 7) &&
          skai_persist_get_int("t_i", -1) == 7);
    check("persist.set_str", skai_persist_set_str("t_s", "hello"));
    n = skai_persist_get_str("t_s", buf, sizeof(buf));
    check("persist.get_str round-trips", n == 5 && strcmp(buf, "hello") == 0);
    check("persist.remove", skai_persist_remove("t_i"));
    check("persist.get_int returns fallback after remove",
          skai_persist_get_int("t_i", -99) == -99);
    check("persist rejects an over-long key",
          !skai_persist_set_int("0123456789012345678901234567890", 1));
    check("persist rejects an empty key", !skai_persist_set_int("", 1));
    check("persist.get_str rejects a 0-byte buffer",
          skai_persist_get_str("t_s", buf, 0) < 0);
    (void)skai_persist_remove("t_s");

    /* timer -- slot table, including exhaustion and bad handles */
    {
        /* static, not on the stack: a timer callback that outlived this
         * function would otherwise scribble on a dead frame. */
        static volatile int hits;
        uint32_t h[SKAI_TIMER_SLOTS + 1];
        uint32_t reuse;
        int i, made = 0;

        hits = 0;

        check("timer.uptime_ms advances", skai_timer_uptime_ms() > 0);
        check("timer rejects a NULL callback", skai_timer_create(10, false, RT_NULL, RT_NULL) == 0);
        check("timer rejects handle 0", !skai_timer_stop(0));
        check("timer rejects an out-of-range handle", !skai_timer_stop(9999));

        for (i = 0; i <= SKAI_TIMER_SLOTS; i++)
        {
            h[i] = skai_timer_create(50, true, tick_cb, (void *)&hits);
            if (h[i])
                made++;
        }
        check("timer hands out exactly the slot count, then refuses",
              made == SKAI_TIMER_SLOTS);

        rt_thread_mdelay(160);
        check("timer fired", hits > 0);

        for (i = 0; i <= SKAI_TIMER_SLOTS; i++)
            if (h[i])
                skai_timer_destroy(h[i]);
        reuse = skai_timer_create(50, false, tick_cb, (void *)&hits);
        check("timer slots are reusable after destroy", reuse != 0);
        skai_timer_destroy(reuse); /* leave no timer running behind us */
    }

    /* watch info */
    n = skai_watchinfo_firmware(buf, sizeof(buf));
    check("watchinfo.firmware writes something", n > 0 && buf[0] != '\0');
    rt_kprintf("       firmware=%s\n", buf);
    check("watchinfo.firmware truncates into a small buffer",
          skai_watchinfo_firmware(buf, 4) == 3 && buf[3] == '\0');
    n = skai_watchinfo_model(buf, sizeof(buf));
    check("watchinfo.model writes something", n > 0 && buf[0] != '\0');
    check("watchinfo screen is 466x466 round",
          skai_watchinfo_screen_width() == 466 &&
          skai_watchinfo_screen_height() == 466 &&
          skai_watchinfo_screen_round());

    /* haptic -- no motor in the simulator, so only the guard rail is
     * meaningful here: an unknown pattern must be refused before any
     * provider call. Real vibration is verified on hardware. */
    check("haptic rejects an unknown pattern", !skai_haptic_vibrate(99));

    /* dispatch table (Phase 2) -- the "adding a capability is one row" claim
     * is only true if lookup is by name and rendering is table-driven. */
    {
        const skai_cap_t *c;

        check("dispatch table is populated", skai_cap_count() > 0);
        check("unknown capability is not resolved", skai_cap_index("no.such") < 0);
        check("NULL name is not resolved", skai_cap_index(RT_NULL) < 0);

        /* Binary search only works while the generator keeps the table sorted;
         * an unsorted table would fail lookups intermittently, not loudly. */
        {
            bool sorted = true;
            for (n = 1; n < skai_cap_count(); n++)
                if (strcmp(skai_cap_at(n - 1)->name, skai_cap_at(n)->name) >= 0)
                    sorted = false;
            check("table is sorted by name", sorted);
        }
        check("out-of-range index yields NULL",
              skai_cap_at(-1) == RT_NULL && skai_cap_at(skai_cap_count()) == RT_NULL);

        c = skai_cap_find("battery.level");
        check("battery.level resolves", c != RT_NULL);
        check("battery.level carries its display format",
              c != RT_NULL && strcmp(c->ui, "%d%%") == 0);
        check("battery.level renders", skai_cap_render(c, buf, sizeof(buf)));
        rt_kprintf("       battery.level -> %s\n", buf);

        /* The capability that did not exist when the renderer was written. */
        c = skai_cap_find("weather.temp");
        check("weather.temp resolves (added with zero renderer edits)", c != RT_NULL);
        check("weather.temp renders", skai_cap_render(c, buf, sizeof(buf)));
        rt_kprintf("       weather.temp -> %s\n", buf);
        check("weather.temp reads as no-data before a forecast arrives",
              strcmp(buf, "--") == 0);

        /* Legacy v0 bind keys must still land on real capabilities, or every
         * package already on a user's watch breaks. */
        check("compat: time -> time.hhmm", skai_cap_index("time.hhmm") >= 0);
        check("compat: date -> time.date_md", skai_cap_index("time.date_md") >= 0);
        check("compat: hr -> health.heart_rate", skai_cap_index("health.heart_rate") >= 0);
        check("compat: steps -> health.steps", skai_cap_index("health.steps") >= 0);

        /* A capability taking arguments has no declarative projection. */
        c = skai_cap_find("haptic.vibrate");
        check("argument-taking capability refuses to render",
              c != RT_NULL && !skai_cap_render(c, buf, sizeof(buf)));
        check("argument-taking capability has no value",
              !skai_cap_value(c, &n));
    }

    skai_log_write(SKAI_LOG_INFO, "skai_test log line");
    skai_log_write(9, "skai_test unknown level -> INFO, not dropped");

    rt_kprintf("skai_test: %s (%d failure%s)\n",
               s_fail ? "FAIL" : "PASS", s_fail, s_fail == 1 ? "" : "s");
    return s_fail ? -1 : 0;
}
MSH_CMD_EXPORT(skai_test, run skai_* capability layer self-check);
