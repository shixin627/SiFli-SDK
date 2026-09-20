/**
 * @file   ui_fs_img.c
 * @brief  Images that live on the filesystem, with a built-in fallback.
 *
 * Some logos moved out of the `main` image into /assets/icons/ (see the note in
 * ui_img_helper.h). A file can legitimately be absent — a watch the phone has
 * not synced yet — and LVGL draws nothing at all for a missing file, so every
 * such source goes through ui_fs_img_or() before it reaches lv_img_set_src().
 *
 * The answer is cached per source pointer: these paths are string literals, so
 * the pointer is a stable key, and a stat() on NAND for every redraw of the
 * notification list would be wasted work. A MISS is only trusted for a few
 * seconds, so icons the phone copies down show up without a reboot.
 */
#include <string.h>
#include <rtthread.h>
#include "lvgl.h"
#include "ui_img_helper.h"

#if defined(RT_USING_DFS)
#include <dfs_posix.h>
#endif

#define FS_IMG_CACHE_SLOTS 40
#define FS_IMG_MISS_TTL_MS 5000

typedef struct
{
    const void *src;
    rt_tick_t   checked;
    bool        present;
} fs_img_slot_t;

static fs_img_slot_t s_slots[FS_IMG_CACHE_SLOTS];

static bool file_present(const char *path)
{
#if defined(RT_USING_DFS)
    struct stat st;
    return stat(path, &st) == 0 && st.st_size > (off_t)sizeof(lv_img_header_t);
#else
    (void)path;
    return false;
#endif
}

const void *ui_fs_img_or(const void *src, const void *fallback)
{
    if (src == NULL)
    {
        return fallback;
    }
    if (lv_img_src_get_type(src) != LV_IMG_SRC_FILE)
    {
        return src; /* built-in descriptor or symbol: nothing to check */
    }

    rt_tick_t now = rt_tick_get();
    fs_img_slot_t *free_slot = NULL;
    for (int i = 0; i < FS_IMG_CACHE_SLOTS; i++)
    {
        fs_img_slot_t *s = &s_slots[i];
        if (s->src == src)
        {
            if (s->present)
            {
                return src;
            }
            if (now - s->checked < rt_tick_from_millisecond(FS_IMG_MISS_TTL_MS))
            {
                return fallback;
            }
            s->present = file_present((const char *)src);
            s->checked = now;
            return s->present ? src : fallback;
        }
        if (s->src == NULL && free_slot == NULL)
        {
            free_slot = s;
        }
    }

    bool present = file_present((const char *)src);
    if (free_slot != NULL)
    {
        free_slot->src = src;
        free_slot->present = present;
        free_slot->checked = now;
    }
    return present ? src : fallback;
}
