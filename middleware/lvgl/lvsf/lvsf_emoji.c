/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "lv_ext_resource_manager.h"

#if defined (EMOJI_SUPPORT)

#include "lvgl.h"

#ifndef EMOJI_CACHE_MAX
#define EMOJI_CACHE_MAX 60
#endif

#define EMOJI_RES_PATH    "/assets/emoji/"
#define EMOJI_RES_SUFFIX  ".bin"

static char emoji_file_name[60];

typedef struct emoji_cache_entry {
    struct emoji_cache_entry *next;
    uint32_t unicode;
    uint32_t access_cnt;
    lv_img_dsc_t img_dsc;
    uint8_t *data;
} emoji_cache_entry_t;

static emoji_cache_entry_t *emoji_cache_head = NULL;
static uint32_t emoji_cache_count = 0;
static uint32_t emoji_access_counter = 0;

static const char *emoji_unicode_to_path(uint32_t u_letter)
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

static emoji_cache_entry_t *emoji_cache_find(uint32_t unicode)
{
    emoji_cache_entry_t *entry = emoji_cache_head;
    while (entry)
    {
        if (entry->unicode == unicode)
        {
            entry->access_cnt = ++emoji_access_counter;
            return entry;
        }
        entry = entry->next;
    }
    return NULL;
}

static void emoji_cache_evict_lru(void)
{
    if (!emoji_cache_head)
        return;

    /* Find LRU entry and its predecessor */
    emoji_cache_entry_t *lru = emoji_cache_head;
    emoji_cache_entry_t *lru_prev = NULL;
    emoji_cache_entry_t *prev = NULL;
    emoji_cache_entry_t *entry = emoji_cache_head;

    while (entry)
    {
        if (entry->access_cnt < lru->access_cnt)
        {
            lru = entry;
            lru_prev = prev;
        }
        prev = entry;
        entry = entry->next;
    }

    /* Remove from list */
    if (lru_prev)
        lru_prev->next = lru->next;
    else
        emoji_cache_head = lru->next;

    lv_mem_free(lru->data);
    lv_mem_free(lru);
    emoji_cache_count--;
}

static emoji_cache_entry_t *emoji_cache_load(uint32_t unicode)
{
    const char *path = emoji_unicode_to_path(unicode);
    if (path == NULL)
        return NULL;

    lv_fs_file_t f;
    if (lv_fs_open(&f, path, LV_FS_MODE_RD) != LV_FS_RES_OK)
        return NULL;

    /* Read header (4 bytes) */
    lv_img_header_t header;
    uint32_t br;
    if (lv_fs_read(&f, &header, sizeof(header), &br) != LV_FS_RES_OK || br != sizeof(header))
    {
        lv_fs_close(&f);
        return NULL;
    }

    /* Get file size to calculate data size */
    uint32_t file_size = 0;
    lv_fs_seek(&f, 0, LV_FS_SEEK_END);
    lv_fs_tell(&f, &file_size);
    uint32_t data_size = file_size - sizeof(header);
    lv_fs_seek(&f, sizeof(header), LV_FS_SEEK_SET);

    uint8_t *buf = lv_mem_alloc(data_size);
    if (buf == NULL)
    {
        lv_fs_close(&f);
        return NULL;
    }

    if (lv_fs_read(&f, buf, data_size, &br) != LV_FS_RES_OK || br != data_size)
    {
        lv_mem_free(buf);
        lv_fs_close(&f);
        return NULL;
    }
    lv_fs_close(&f);

    /* Evict if at capacity */
    if (emoji_cache_count >= EMOJI_CACHE_MAX)
        emoji_cache_evict_lru();

    /* Allocate new entry */
    emoji_cache_entry_t *entry = lv_mem_alloc(sizeof(emoji_cache_entry_t));
    if (entry == NULL)
    {
        lv_mem_free(buf);
        return NULL;
    }

    entry->unicode = unicode;
    entry->data = buf;
    entry->access_cnt = ++emoji_access_counter;
    entry->img_dsc.header = header;
    entry->img_dsc.data_size = data_size;
    entry->img_dsc.data = buf;

    /* Insert at head */
    entry->next = emoji_cache_head;
    emoji_cache_head = entry;
    emoji_cache_count++;

    return entry;
}

void *lv_get_emoji_by_unicode(uint32_t u_letter)
{
    /* Check cache first */
    emoji_cache_entry_t *entry = emoji_cache_find(u_letter);
    if (entry)
        return &entry->img_dsc;

    /* Cache miss: load from flash into RAM */
    entry = emoji_cache_load(u_letter);
    if (entry)
        return &entry->img_dsc;

    return NULL;
}

#else

void *lv_get_emoji_by_unicode(uint32_t u_letter)
{
    return NULL;
}

#endif
