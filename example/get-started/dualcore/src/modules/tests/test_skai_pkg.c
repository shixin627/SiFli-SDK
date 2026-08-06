/*
 * Install-path and signature gates for signed app packages (ADR-0019 §2.5).
 * PC simulator only.
 *
 *   skai_pkg_test              run every check
 *   skai_pkg_ls                list what is installed
 *   skai_pkg_add <dir>         install /pkgsrc/<dir> by hand
 *   skai_pkg_run <keyid> <id>  launch an installed app
 *
 * Like the sandbox suite, these are hostile inputs rather than API smoke tests.
 * A verifier that accepts a package it should refuse is worth less than no
 * verifier at all, because the permission model is the only security boundary
 * self-signing leaves standing — so every case here is a package that was
 * signed correctly and then broken in exactly one way.
 *
 * Fixtures come from tools/sdk/sign_pkg.js and live in the simulator's
 * /pkgsrc; project/hcpu/_mkfixtures.cmd regenerates them.
 */
#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#if defined(BSP_USING_PC_SIMULATOR) && defined(PKG_USING_QUICKJS)

#include <dfs_posix.h>

#include "skai/skai_pkg.h"

static int s_fail;
static int s_ran;
/* Quiet mode exists for the harness, not for people: _dev_test reads the
   simulator's VISIBLE console buffer, so 27 per-check lines push the summary —
   the only line that carries the verdict — off the top of the window. */
static bool s_quiet;
#define SKAI_FAIL_KEEP 8
static const char *s_failed[SKAI_FAIL_KEEP];

static void check(const char *what, bool ok)
{
    s_ran++;
    if (!s_quiet || !ok)
        rt_kprintf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
    {
        if (s_fail < SKAI_FAIL_KEEP)
            s_failed[s_fail] = what;
        s_fail++;
    }
}

/* Fixture loader. Both blobs are read whole; the manifest is bounded by the
   schema and the payloads here are tiny. */
static char    s_manifest[SKAI_PKG_MANIFEST_MAX];
static uint8_t s_payload[SKAI_PKG_JS_SRC_MAX];
static uint32_t s_mlen, s_plen;

static bool fixture_load(const char *name)
{
    char path[96];
    int fd, n;

    rt_snprintf(path, sizeof(path), "/pkgsrc/%s/manifest.json", name);
    fd = skai_fopen(path, O_RDONLY, 0);
    if (fd < 0)
    {
        rt_kprintf("       missing fixture %s — run _mkfixtures.cmd\n", path);
        return false;
    }
    n = skai_fread(fd, s_manifest, sizeof(s_manifest));
    skai_fclose(fd);
    if (n <= 0)
        return false;
    s_mlen = (uint32_t)n;

    rt_snprintf(path, sizeof(path), "/pkgsrc/%s/payload", name);
    fd = skai_fopen(path, O_RDONLY, 0);
    if (fd < 0)
        return false;
    n = skai_fread(fd, s_payload, sizeof(s_payload));
    skai_fclose(fd);
    if (n < 0)
        return false;
    s_plen = (uint32_t)n;
    return true;
}

static skai_pkg_info_t s_info;

/* Install a fixture and name the outcome in the label: a bare "FAILED: bad
   signature is refused" does not say what happened instead. */
static void expect_install(const char *fixture, skai_pkg_result_t want,
                           const char *what)
{
    static char label[112];
    skai_pkg_result_t got;

    if (!fixture_load(fixture))
    {
        check(what, false);
        return;
    }
    got = skai_pkg_install(s_manifest, s_mlen, s_payload, s_plen, &s_info);
    rt_snprintf(label, sizeof(label), "%s (got %s)", what,
                skai_pkg_result_name(got));
    check(label, got == want);
}

static void wipe_installed(void)
{
    /* static, not automatic: skai_pkg_info_t is ~1.4 KB and the finsh thread's
       stack is not sized for several of them plus mbedtls's ECDSA frame. */
    static skai_pkg_info_t info;

    /* Removing shifts the indices, so always take slot 0. */
    while (skai_pkg_count() > 0 && skai_pkg_at(0, &info))
        if (skai_pkg_remove(info.keyid, info.app_id) != SKAI_PKG_OK)
            break;
}

static int skai_pkg_test(int argc, char **argv)
{
    /* Zeroed: a keyid is exactly 11 chars in a 12-byte field, so a copy of
       sizeof-1 writes no terminator of its own. */
    char keyid_good[SKAI_PKG_KEYID_MAX] = { 0 };
    static char bl[256];
    int32_t seq;

    s_quiet = (argc >= 2 && strcmp(argv[1], "-q") == 0);
    s_fail = 0;
    s_ran = 0;
    wipe_installed();

    /* The blocklist lives in the KVDB and survives a reboot, which is the whole
       point of it — so a run that stopped half way through leaves a publisher
       revoked and every later run fails at the first check for the wrong
       reason. Start from empty rather than trusting the previous run. */
    seq = skai_pkg_blocklist_seq();
    rt_snprintf(bl, sizeof(bl),
                "{\"skai_blocklist\":1,\"seq\":%d,\"revoked\":[]}", (int)(seq + 1));
    skai_pkg_blocklist_apply(bl, (uint32_t)strlen(bl));

    /* --- the package that is exactly right --- */
    expect_install("good", SKAI_PKG_OK, "a correctly signed package installs");
    rt_strncpy(keyid_good, s_info.keyid, sizeof(keyid_good) - 1);
    check("install is visible to the launcher", skai_pkg_count() == 1);

    /* --- gate: every single-property break is refused, and named --- */
    expect_install("t_payload", SKAI_PKG_ERR_DIGEST,
                   "payload edited after signing is refused");
    expect_install("t_sig", SKAI_PKG_ERR_SIGNATURE,
                   "corrupted signature is refused");
    expect_install("t_keyid", SKAI_PKG_ERR_KEYID,
                   "keyid not derived from the pubkey is refused");
    expect_install("future", SKAI_PKG_ERR_VERSION,
                   "app needing a newer firmware is refused at install");

    /* The one that matters most: capabilities are INSIDE the digest, so
       widening them after signing breaks the signature. If this ever passes,
       self-signing has no security model left -- an attacker could take any
       app and grant it whatever it likes. */
    expect_install("t_caps", SKAI_PKG_ERR_SIGNATURE,
                   "capability list widened after signing is refused");

    check("refused packages left nothing installed", skai_pkg_count() == 1);

    /* --- update vs impostor: this is what (keyid, app_id) buys --- */
    expect_install("update", SKAI_PKG_OK, "same publisher can update its app");
    check("an update replaces rather than duplicates", skai_pkg_count() == 1);
    {
        static skai_pkg_info_t info;
        check("the update is the version now installed",
              skai_pkg_at(0, &info) && strcmp(info.version, "1.0.1") == 0);
    }

    expect_install("impostor", SKAI_PKG_OK,
                   "another publisher may claim the same app id");
    check("the impostor did NOT overwrite the original", skai_pkg_count() == 2);
    {
        /* Whichever slot the original landed in, it must still be the update. */
        static skai_pkg_info_t a, b;
        bool ok = skai_pkg_at(0, &a) && skai_pkg_at(1, &b);
        const skai_pkg_info_t *orig = (strcmp(a.keyid, keyid_good) == 0) ? &a : &b;
        check("the original app is untouched by the impostor",
              ok && strcmp(orig->keyid, keyid_good) == 0
              && strcmp(orig->version, "1.0.1") == 0);
    }

    /* --- gate: revocation stops both install and launch --- */
    seq = skai_pkg_blocklist_seq();   /* moved on by the clear above */
    rt_snprintf(bl, sizeof(bl),
                "{\"skai_blocklist\":1,\"seq\":%d,"
                "\"revoked\":[{\"keyid\":\"%s\",\"reason\":\"malware\"}]}",
                (int)(seq + 1), keyid_good);
    check("blocklist push is accepted",
          skai_pkg_blocklist_apply(bl, (uint32_t)strlen(bl)) == SKAI_PKG_OK);
    check("the revoked publisher reads as blocked", skai_pkg_blocked(keyid_good));
    expect_install("update", SKAI_PKG_ERR_BLOCKED,
                   "a revoked publisher cannot install");
    check("a revoked app cannot launch either",
          skai_pkg_launch(keyid_good, "hello") == SKAI_PKG_ERR_BLOCKED);

    /* A stale list must not be able to un-revoke a key by replay. */
    rt_snprintf(bl, sizeof(bl),
                "{\"skai_blocklist\":1,\"seq\":%d,\"revoked\":[]}", (int)seq);
    skai_pkg_blocklist_apply(bl, (uint32_t)strlen(bl));
    check("replaying an older list does not un-revoke",
          skai_pkg_blocked(keyid_good));

    /* Clear it again so the next run starts from a known state. */
    rt_snprintf(bl, sizeof(bl),
                "{\"skai_blocklist\":1,\"seq\":%d,\"revoked\":[]}", (int)(seq + 2));
    skai_pkg_blocklist_apply(bl, (uint32_t)strlen(bl));
    check("clearing the list restores the publisher",
          !skai_pkg_blocked(keyid_good));

    /* --- garbage in --- */
    check("a legacy bare package is not mistaken for a signed one",
          skai_pkg_verify("{\"skaiapp\":1}", 13, s_payload, s_plen, &s_info)
          == SKAI_PKG_ERR_PARSE);
    check("truncated JSON is refused",
          skai_pkg_verify("{\"skai\":\">=1.0\"", 15, s_payload, s_plen, &s_info)
          == SKAI_PKG_ERR_PARSE);
    check("an empty manifest is refused",
          skai_pkg_verify("", 0, s_payload, s_plen, &s_info) == SKAI_PKG_ERR_PARSE);

    /* --- an installed app actually runs --- */
    check("removing an installed app works",
          skai_pkg_remove(keyid_good, "hello") == SKAI_PKG_OK);
    check("removal is visible to the launcher", skai_pkg_count() == 1);
    check("removing something that is not there says so",
          skai_pkg_remove(keyid_good, "hello") == SKAI_PKG_ERR_NOTFOUND);

    wipe_installed();
    check("the store empties cleanly", skai_pkg_count() == 0);

    for (int i = 0; i < s_fail && i < SKAI_FAIL_KEEP; i++)
        rt_kprintf("  FAILED: %s\n", s_failed[i]);
    /* The check COUNT is part of the verdict. The simulator console is a
       rolling window, so a run that stopped half way through looks exactly like
       a clean one if the summary only reports failures — which is how a broken
       suite once passed unnoticed in this project. */
    rt_kprintf("skai_pkg_test: %s (%d failure%s, %d checks)\n",
               s_fail ? "FAIL" : "PASS", s_fail, s_fail == 1 ? "" : "s", s_ran);
    return s_fail ? -1 : 0;
}
MSH_CMD_EXPORT(skai_pkg_test, run signed-package install and verify gates);

static int skai_pkg_ls(int argc, char **argv)
{
    int n = skai_pkg_count();

    (void)argc;
    (void)argv;
    rt_kprintf("%d installed\n", n);
    for (int i = 0; i < n; i++)
    {
        skai_pkg_info_t info;
        if (skai_pkg_at(i, &info))
            rt_kprintf("  %s/%s  %s v%s  %s\n", info.keyid, info.app_id,
                       info.name, info.version, info.is_js ? "js" : "declarative");
    }
    return 0;
}
MSH_CMD_EXPORT(skai_pkg_ls, list installed signed packages);

static int skai_pkg_add(int argc, char **argv)
{
    skai_pkg_result_t r;

    if (argc < 2)
    {
        rt_kprintf("usage: skai_pkg_add <fixture-dir under /pkgsrc>\n");
        return -1;
    }
    if (!fixture_load(argv[1]))
        return -1;
    r = skai_pkg_install(s_manifest, s_mlen, s_payload, s_plen, &s_info);
    rt_kprintf("skai_pkg_add %s: %s\n", argv[1], skai_pkg_result_name(r));
    return (r == SKAI_PKG_OK) ? 0 : -1;
}
MSH_CMD_EXPORT(skai_pkg_add, install a fixture package: skai_pkg_add <dir>);

/* Push a whole weather sync through the REAL ingest path — bloc_weather parses
   each record, pushes it onto the forecast list and calls notify_weather(),
   which is what fires skai.on_change('weather.*') in any open JS app.
   Injecting JSON rather than calling notify directly is the point: it proves
   the whole chain, not just the last link.

   FOUR records, in the phone's own order: current, +3h, +6h, +9h. Pushing
   fewer is what made an earlier fixture lie — weather_push_front() fills from
   index 0, so a short batch leaves the LAST slot (which is "now") empty, and
   both apps then correctly render a blank current reading that looks like a
   bug in whichever one you were suspecting. */
static int fake_weather(int argc, char **argv)
{
    extern void handle_weather(char *json);
    static const char *const k_cond[4] = { "Clear", "Clouds", "Rain", "Thunderstorm" };
    /* `fake_weather <temp> odd` pushes the descriptions the four above can never
       produce, because those four are exactly condition_token()'s happy path and
       so every branch off it was dead to this suite:

         Snow      IS in condition_token's map but has NO icon asset, so
                   ui.icon("weather.snow") returns 0. C draws weather_thunder.
         Drizzle   outside the map, yet C's daily rain gate matches it by strstr
                   while C's icon lookup does not.
         Fog       outside the map entirely — the plain unknown-word path.

       A snow day drawing the wrong glyph survived several review rounds purely
       because nothing could push the word "Snow" at it. */
    static const char *const k_odd[4] = { "Snow", "Drizzle", "Fog", "Clear" };
    static char json[224];
    int temp = (argc >= 2) ? atoi(argv[1]) : 20;
    const char *const *cond = (argc >= 3 && argv[2][0] == 'o') ? k_odd : k_cond;

    for (int i = 0; i < 4; i++)
    {
        /* Field names read off bloc_weather.c's parse_weather(); it takes a
           flat object, not the nested provider format. Temperature climbs by
           slot so a wrongly-ordered forecast row is visible at a glance. */
        /* `date` is epoch seconds, 3 hours apart like the phone's aligned
           slots. Note it changes nothing HERE: parse_weather's timestamp block
           is inside #ifndef BSP_USING_PC_SIMULATOR, so on the simulator every
           record keeps a zero time and hour_time() formats it as 12:00 AM.
           Sent correctly anyway, because this fixture is also the shape a
           hardware test needs. */
        rt_snprintf(json, sizeof(json),
                    "{\"temperature\":%d,\"description\":\"%s\","
                    "\"precipitationProbability\":%d,\"isDailySummary\":false,"
                    "\"location\":\"Taipei\",\"date\":%u}",
                    temp + i, cond[i], 10 * (i + 1),
                    (unsigned)(time(RT_NULL) + i * 3 * 3600));
        handle_weather(json);
    }
    /* Then the five daily summaries, ascending, exactly as the phone sends
       them after the hourly block. isDailySummary routes them to the week
       list, which is what the second page reads. */
    for (int d = 0; d < 5; d++)
    {
        rt_snprintf(json, sizeof(json),
                    "{\"temperature\":%d,\"maxTemperature\":%d,"
                    "\"minTemperature\":%d,\"description\":\"%s\","
                    "\"precipitationProbability\":%d,\"isDailySummary\":true,"
                    "\"location\":\"Taipei\",\"date\":%u}",
                    temp + d, temp + 4 + d, temp - 3 + d, cond[d % 4],
                    20 + 5 * d, (unsigned)(time(RT_NULL) + d * 86400));
        handle_weather(json);
    }

    rt_kprintf("fake_weather: now %d C %s, then %d/%d/%d, +5 daily\n",
               temp, cond[0], temp + 1, temp + 2, temp + 3);
    return 0;
}
MSH_CMD_EXPORT(fake_weather, inject a 4-record weather sync: fake_weather [tempC] [odd]);

static int skai_pkg_run(int argc, char **argv)
{
    static skai_pkg_info_t info;
    const char *keyid = NULL, *app_id = NULL;
    skai_pkg_result_t r;

    if (argc == 3)
    {
        keyid = argv[1];
        app_id = argv[2];
    }
    else if (argc == 2)
    {
        /* Keyids are regenerated whenever the fixtures are, so a test script
           cannot hardcode one. With a single publisher of that app id the pair
           is unambiguous; with two, say so rather than guess. */
        int n = skai_pkg_count(), hits = 0;
        static char found[SKAI_PKG_KEYID_MAX];

        for (int i = 0; i < n; i++)
            if (skai_pkg_at(i, &info) && strcmp(info.app_id, argv[1]) == 0)
            {
                rt_strncpy(found, info.keyid, sizeof(found) - 1);
                found[sizeof(found) - 1] = '\0';
                hits++;
            }
        if (hits != 1)
        {
            rt_kprintf("skai_pkg_run: %d packages named '%s'; give the keyid\n",
                       hits, argv[1]);
            return -1;
        }
        keyid = found;
        app_id = argv[1];
    }
    else
    {
        rt_kprintf("usage: skai_pkg_run [<keyid>] <app_id>\n");
        return -1;
    }

    r = skai_pkg_launch(keyid, app_id);
    rt_kprintf("skai_pkg_run %s/%s: %s\n", keyid, app_id,
               skai_pkg_result_name(r));
    return (r == SKAI_PKG_OK) ? 0 : -1;
}
MSH_CMD_EXPORT(skai_pkg_run, launch an installed package: skai_pkg_run [keyid] <id>);

#endif /* BSP_USING_PC_SIMULATOR && PKG_USING_QUICKJS */
