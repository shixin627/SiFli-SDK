/*
 * lvsf font layer for LVGL v9 -- see lvsf_font.h for why this exists.
 *
 * Built on LVGL's tiny_ttf rather than FreeType, following
 * example/multimedia/lvgl/watch_v9: that is the SDK's own v9 watch and it
 * enables LV_USE_TINY_TTF with no FreeType at all. tiny_ttf reads the .ttf
 * itself, so there are no ft_* platform hooks to supply and nothing drags in
 * FT_Stroker_*, which LVGL's lv_freetype_outline.c would have needed.
 *
 * ponytail: a fixed-size cache, not a font manager. The watch asks for seven
 * distinct sizes and asks for them repeatedly, so an array of "sizes we have
 * opened" is enough; the v8 side ran a registry with names, ordering and
 * priorities that nothing in this project ever called.
 */
#include "lvsf_font.h"

#include <rtthread.h>

#define DBG_TAG "lvsf.font.v9"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#if LV_USE_TINY_TTF

/* NOT the v8 face. tiny55_full.ttf is preprocessed for SiFli's closed font
   engine: its outlines live in a private FTFG table (86% of the file) and the
   standard glyf table is a 4-byte stub, so anything but lvgl_extensions --
   which has no v9 build -- renders it blank. DroidSansFallback.ttf is a normal
   TrueType and is what example/multimedia/lvgl/watch_v9 uses. */
#define LVSF_FONT_PATH "/assets/fonts/DroidSansFallback.ttf"

/* Seven FONT_* sizes today. One spare so adding a size does not silently start
   evicting. */
#define LVSF_FONT_CACHE_MAX 8

typedef struct
{
    uint16_t   size;
    lv_font_t *font;
} font_slot_t;

static font_slot_t s_cache[LVSF_FONT_CACHE_MAX];
static uint8_t     s_used;

/* Mirrors the v8 fallback ladder, so a missing .ttf degrades the same way:
   ASCII still renders, non-ASCII becomes tofu, the watch boots. */
static const lv_font_t *builtin_for(uint16_t size)
{
    if (FONT_BIGL <= size)     return lv_theme_get_font_large(NULL);
    if (FONT_SUBTITLE <= size) return lv_theme_get_font_normal(NULL);
    return lv_theme_get_font_small(NULL);
}

const lv_font_t *LV_EXT_FONT_GET(uint16_t size)
{
    uint8_t i;

    lvsf_convert_font_size(size);

    for (i = 0; i < s_used; i++)
    {
        if (s_cache[i].size == size)
        {
            return s_cache[i].font;
        }
    }

    if (s_used >= LVSF_FONT_CACHE_MAX)
    {
        LOG_W("font cache full (%d); size %d falls back to built-in",
              LVSF_FONT_CACHE_MAX, (int)size);
        return builtin_for(size);
    }

    lv_font_t *font = lv_tiny_ttf_create_file(LVSF_FONT_PATH, (int32_t)size);
    if (!font)
    {
        /* No .ttf on the filesystem yet (first boot before file sync), or the
           face failed to parse. Not fatal -- see builtin_for(). */
        LOG_E("lv_tiny_ttf_create_file(%s, %d) failed", LVSF_FONT_PATH, (int)size);
        return builtin_for(size);
    }

    s_cache[s_used].size = size;
    s_cache[s_used].font = font;
    s_used++;
    return font;
}

void lv_ext_font_reset(void)
{
    uint8_t i;

    for (i = 0; i < s_used; i++)
    {
        lv_tiny_ttf_destroy(s_cache[i].font);
        s_cache[i].font = NULL;
        s_cache[i].size = 0;
    }
    s_used = 0;
}

#else /* !LV_USE_TINY_TTF */

const lv_font_t *LV_EXT_FONT_GET(uint16_t size)
{
    lvsf_convert_font_size(size);
    if (FONT_BIGL <= size)     return lv_theme_get_font_large(NULL);
    if (FONT_SUBTITLE <= size) return lv_theme_get_font_normal(NULL);
    return lv_theme_get_font_small(NULL);
}

void lv_ext_font_reset(void)
{
}

#endif /* LV_USE_TINY_TTF */
