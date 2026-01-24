/*
 * Copyright (c) 2018-2024, Skaiwalk Development Team
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-16     jack         first version
 */

#include <rtthread.h>
#include <stdlib.h>
#ifdef BSP_SHARE_PREFS
#include "share_prefs.h"

#define DBG_TAG "utest.sf"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/// @brief test share_prefs integer
/// @param argc
/// @param argv
/// @return
int utest_share_prefs_int(int argc, char **argv)
{
    rt_err_t res = RT_EOK;
    if (argc < 1)
    {
        LOG_E("Invalid amount of parameter");
        return 0;
    }

    /* Open an preference*/
    share_prefs_t *pref = share_prefs_open("utest", SHAREPREFS_MODE_PRIVATE);
    LOG_D("open share_prefs: prfs_name = %s, mode = %d", pref->prfs_name, pref->mode);
    if (strcmp(argv[1], "-read") == 0) // utest_share_prefs_int -read
    {
        int32_t value = 0;
        value = share_prefs_get_int(pref, "integer", -1);
        LOG_D("read integer value: %d", value);
    }
    else if (strcmp(argv[1], "-write") == 0) // utest_share_prefs_int -write 100
    {
        int32_t value = atoi(argv[2]);
        LOG_D("writing integer value: %d", value);
        res = share_prefs_set_int(pref, "integer", value);
        if (res != RT_EOK)
        {
            LOG_E("write integer value failed");
        }
    }
    else
    {
        LOG_D("Invalid intent parameter");
    }

    res = share_prefs_close(pref);
    return res;
}
MSH_CMD_EXPORT(utest_share_prefs_int, test share_prefs integer);

/// @brief test share_prefs string
/// @param argc
/// @param argv
/// @return
int utest_share_prefs_string(int argc, char **argv)
{
    rt_err_t res = RT_EOK;
    if (argc < 1)
    {
        LOG_E("Invalid amount of parameter");
        return 0;
    }

    /* Open an preference*/
    share_prefs_t *pref = share_prefs_open("utest", SHAREPREFS_MODE_PRIVATE);
    LOG_D("open share_prefs: prfs_name = %s, mode = %d", pref->prfs_name, pref->mode);
    if (strcmp(argv[1], "-read") == 0) // utest_share_prefs_string -read
    {
        char value[32] = {0};
        share_prefs_get_string(pref, "string", value, sizeof(value));
        LOG_D("read string value: %s", value);
    }
    else if (strcmp(argv[1], "-write") == 0) // utest_share_prefs_string -write "hello"
    {
        LOG_D("writing string value: %s", argv[2]);
        res = share_prefs_set_string(pref, "string", argv[2]);
        if (res != RT_EOK)
        {
            LOG_E("write string value failed");
        }
    }
    else
    {
        LOG_E("Invalid intent parameter");
    }

    res = share_prefs_close(pref);
    return res;
}
MSH_CMD_EXPORT(utest_share_prefs_string, test share_prefs string);

#endif /* BSP_SHARE_PREFS */