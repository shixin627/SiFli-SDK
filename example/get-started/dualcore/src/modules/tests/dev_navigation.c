/*
 * SPDX-FileCopyrightText: 2026 SiFli / project contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Programmatic app navigation for the PC simulator — skip the swipe / tap
 * dance and jump straight into a specific gui_apps/<app>.
 *
 *   list_apps          list every BUILTIN_APP_EXPORT'd app's id
 *   goto_app <id>      launch (resume) app by id, e.g. "goto_app calculator"
 *   app_status         dump the active app's id (and state)
 *
 * The framework already exports `app_run`, `app_exit`, `app_goback`,
 * `list_app` (running apps) and `app_cleanup`. goto_app is a more
 * discoverable alias for app_run; list_apps (plural) covers the full
 * builtin table — distinct from list_app (singular = running only).
 *
 * Threading: gui_app_run is mailbox-based so it's safe from the FinSH
 * thread — the gui_app scheduler thread picks the message up.
 *
 * Build gate: BSP_USING_PC_SIMULATOR (see tests/SConscript).
 */
#include <rtthread.h>
#include <string.h>
#include <stdint.h>

#include "gui_app_fwk.h"
/* gui_app_int.h holds gui_runing_app_t internals (state/flag/tick_cnt). It
 * sits in middleware/app_fwk which is already on CPPPATH via that group. */
#include "gui_app_int.h"

#define DBG_TAG "dev.nav"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static int list_apps(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    const builtin_app_desc_t *p = gui_builtin_app_list_open();
    int n = 0;
    rt_kprintf("builtin apps:\n");
    while (p != NULL)
    {
        rt_kprintf("  %s\n", p->id);
        n++;
        p = gui_builtin_app_list_get_next(p);
    }
    gui_builtin_app_list_close(NULL);
    rt_kprintf("(%d apps)\n", n);
    return 0;
}
MSH_CMD_EXPORT(list_apps, list_apps - print every registered builtin app id);

static int goto_app(int argc, char *argv[])
{
    if (argc < 2)
    {
        rt_kprintf("usage: goto_app <app_id>   (try list_apps)\n");
        return -1;
    }
    rt_err_t r = gui_app_run(argv[1]);
    if (r == RT_EOK)
        rt_kprintf("goto_app: launched '%s'\n", argv[1]);
    else
        rt_kprintf("goto_app: gui_app_run('%s') failed (%d)\n", argv[1], (int)r);
    return (int)r;
}
MSH_CMD_EXPORT(goto_app, goto_app id - launch GUI app by id);

/* gui_app_get_actived() asserts the calling thread is the gui scheduler;
 * called from FinSH that assert fires and the sim crashes. Skip it and read
 * the active pointer directly — it's a single-pointer dereference, worst
 * case we get a stale snapshot. */
extern gui_runing_app_t *app_schedule_get_active(void);

static int app_status(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    gui_runing_app_t *cur = app_schedule_get_active();
    if (!cur)
    {
        rt_kprintf("app_status: no active app\n");
        return 0;
    }
    rt_kprintf("app_status: id='%s' state=%u flag=0x%02x ticks=%u\n",
               cur->id, cur->state, cur->flag, cur->tick_cnt);
    return 0;
}
MSH_CMD_EXPORT(app_status, app_status - dump currently-active app);
