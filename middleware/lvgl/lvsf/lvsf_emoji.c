/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "lv_ext_resource_manager.h"

#if defined (EMOJI_SUPPORT)

#define EMOJI_RES_PATH    "/assets/emoji/"
#define EMOJI_RES_SUFFIX  ".bin"

static char emoji_file_name[60];

void *lv_get_emoji_by_unicode(uint32_t u_letter)
{
#define GET_EMOJI_INFO(_id) \
    case 0x##_id: \
    { \
        sprintf(emoji_file_name, "%semoji_%s%s", EMOJI_RES_PATH, #_id, EMOJI_RES_SUFFIX); \
        return emoji_file_name; \
    }

    switch (u_letter)
    {
#include "emoji_info.h"
    default:
        return NULL;
    }
}

#else

void *lv_get_emoji_by_unicode(uint32_t u_letter)
{
    return NULL;
}

#endif

