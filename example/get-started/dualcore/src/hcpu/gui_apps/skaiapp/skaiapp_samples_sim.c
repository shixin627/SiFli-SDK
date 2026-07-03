/**
 * @file   skaiapp_samples_sim.c
 * @brief  PC-sim-only sample seeder: `skaiapp_seed <1|2|3>` installs an
 *         embedded package through the REAL install path (parse→FS→engine),
 *         so the renderer/engine iterate on sim without a phone.
 *
 * ASCII-only texts on purpose: the sim has no CJK glyphs (FreeType fonts live
 * on the watch NAND) and MSVC source encoding is safest kept ASCII. The
 * canonical CJK samples live in SkaiLink/bridge/skaiapp-samples/.
 */
#include <rtthread.h>

#if defined(BSP_USING_PC_SIMULATOR)
#include <string.h>
#include "skaiapp_store.h"

static const char *const k_samples[4] =
{
    /* 1 — water reminder */
    "{\"skaiapp\":0,\"id\":\"water-reminder\",\"name\":\"Water\",\"icon\":\"water\","
    "\"accent\":\"blue\",\"page\":{\"items\":["
    "{\"w\":\"spacer\",\"h\":24},"
    "{\"w\":\"icon\",\"name\":\"water\",\"size\":\"l\"},"
    "{\"w\":\"label\",\"text\":\"Water Reminder\",\"size\":\"l\"},"
    "{\"w\":\"label\",\"text\":\"next\",\"size\":\"s\",\"color\":\"gray\"},"
    "{\"w\":\"value\",\"bind\":\"reminder:drink\",\"size\":\"xl\",\"color\":\"blue\"},"
    "{\"w\":\"spacer\",\"h\":16},"
    "{\"w\":\"button\",\"text\":\"On / Off\",\"action\":\"reminder.toggle:drink\",\"style\":\"primary\"}]},"
    "\"reminders\":[{\"id\":\"drink\",\"kind\":\"interval\",\"every_min\":60,"
    "\"window\":{\"start\":\"09:00\",\"end\":\"21:00\"},\"message\":\"Time to drink water\","
    "\"vibrate\":true,\"enabled\":true}]}",

    /* 2 — pomodoro */
    "{\"skaiapp\":0,\"id\":\"pomodoro\",\"name\":\"Pomodoro\",\"icon\":\"timer\","
    "\"accent\":\"red\",\"page\":{\"items\":["
    "{\"w\":\"spacer\",\"h\":16},"
    "{\"w\":\"label\",\"text\":\"Pomodoro\",\"size\":\"m\",\"color\":\"gray\"},"
    "{\"w\":\"arc\",\"bind\":\"timer:work\",\"size\":\"l\",\"color\":\"red\"},"
    "{\"w\":\"value\",\"bind\":\"timer:work\",\"size\":\"xl\"},"
    "{\"w\":\"row\",\"items\":[{\"w\":\"label\",\"text\":\"break\",\"size\":\"s\",\"color\":\"gray\"},"
    "{\"w\":\"value\",\"bind\":\"timer:break\",\"size\":\"s\",\"color\":\"green\"}]},"
    "{\"w\":\"row\",\"items\":[{\"w\":\"button\",\"text\":\"Start\",\"action\":\"timer.start:work\",\"style\":\"primary\"},"
    "{\"w\":\"button\",\"text\":\"Reset\",\"action\":\"timer.reset:work\",\"style\":\"ghost\"}]}]},"
    "\"timers\":["
    "{\"id\":\"work\",\"kind\":\"countdown\",\"duration_s\":1500,\"label\":\"Focus\","
    "\"on_fire\":{\"notify\":\"Focus done, take a break\",\"vibrate\":true,\"next\":\"break\",\"autostart_next\":true}},"
    "{\"id\":\"break\",\"kind\":\"countdown\",\"duration_s\":300,\"label\":\"Break\","
    "\"on_fire\":{\"notify\":\"Break over, focus again\",\"vibrate\":true,\"next\":\"work\",\"autostart_next\":true}}]}",

    /* 3 — health glance */
    "{\"skaiapp\":0,\"id\":\"health-glance\",\"name\":\"Health\",\"icon\":\"heart\","
    "\"accent\":\"green\",\"page\":{\"items\":["
    "{\"w\":\"spacer\",\"h\":16},"
    "{\"w\":\"value\",\"bind\":\"time\",\"size\":\"xl\"},"
    "{\"w\":\"value\",\"bind\":\"date\",\"size\":\"s\",\"color\":\"gray\"},"
    "{\"w\":\"spacer\",\"h\":16},"
    "{\"w\":\"row\",\"items\":[{\"w\":\"icon\",\"name\":\"heart\",\"size\":\"s\"},"
    "{\"w\":\"value\",\"bind\":\"hr\",\"size\":\"l\",\"color\":\"red\"}]},"
    "{\"w\":\"row\",\"items\":[{\"w\":\"icon\",\"name\":\"steps\",\"size\":\"s\"},"
    "{\"w\":\"value\",\"bind\":\"steps\",\"size\":\"l\",\"color\":\"green\"}]},"
    "{\"w\":\"bar\",\"bind\":\"steps\",\"max\":8000,\"color\":\"green\"},"
    "{\"w\":\"row\",\"items\":[{\"w\":\"icon\",\"name\":\"battery\",\"size\":\"s\"},"
    "{\"w\":\"value\",\"bind\":\"battery\",\"size\":\"m\",\"color\":\"gray\"}]}]}}",

    /* 4 — voice memo + timer (the user's "放我語音輸入的文字 + 計時器" case) */
    "{\"skaiapp\":0,\"id\":\"note-timer\",\"name\":\"Note\",\"icon\":\"star\","
    "\"accent\":\"orange\",\"page\":{\"items\":["
    "{\"w\":\"spacer\",\"h\":16},"
    "{\"w\":\"label\",\"text\":\"Note\",\"size\":\"m\",\"color\":\"gray\"},"
    "{\"w\":\"value\",\"bind\":\"memo:note\",\"size\":\"l\"},"
    "{\"w\":\"spacer\",\"h\":16},"
    "{\"w\":\"value\",\"bind\":\"timer:t\",\"size\":\"xl\",\"color\":\"orange\"},"
    "{\"w\":\"row\",\"items\":[{\"w\":\"button\",\"text\":\"Start\",\"action\":\"timer.start:t\",\"style\":\"primary\"},"
    "{\"w\":\"button\",\"text\":\"Reset\",\"action\":\"timer.reset:t\",\"style\":\"ghost\"}]}]},"
    "\"memos\":[{\"id\":\"note\",\"text\":\"Buy milk and eggs\"}],"
    "\"timers\":[{\"id\":\"t\",\"kind\":\"countdown\",\"duration_s\":180,\"label\":\"Timer\"}]}",
};

int skaiapp_sim_seed(int which)
{
    if (which < 1 || which > 4)
    {
        return -1;
    }
    const char *json = k_samples[which - 1];
    return skaiapp_store_install((const uint8_t *)json,
                                 (uint32_t)strlen(json), NULL);
}

/* Install the samples once if the store is empty (called from the host app's
   on_start on PC sim, since FINSH stdin is unreliable headless). */
void skaiapp_sim_seed_all_if_empty(void)
{
    if (skaiapp_store_count() > 0)
    {
        return;
    }
    for (int i = 1; i <= 4; i++)
    {
        int code = skaiapp_sim_seed(i);
        rt_kprintf("[sim] seed sample %d -> ack=%d (0=ok)\n", i, code);
    }
}

#ifdef FINSH_USING_MSH
#include <finsh.h>
static void skaiapp_seed(int argc, char **argv)
{
    int which = (argc >= 2) ? atoi(argv[1]) : 0;
    if (which < 1 || which > 3)
    {
        rt_kprintf("usage: skaiapp_seed <1|2|3>  (1=water 2=pomodoro 3=health)\n");
        return;
    }
    rt_kprintf("seed sample %d -> ack=%d (0=ok)\n", which, skaiapp_sim_seed(which));
}
MSH_CMD_EXPORT(skaiapp_seed, install an embedded skaiapp sample (PC sim));
#endif /* FINSH_USING_MSH */
#endif /* BSP_USING_PC_SIMULATOR */
