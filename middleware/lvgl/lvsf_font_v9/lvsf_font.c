/*
 * lvsf font layer for LVGL v9 -- see lvsf_font.h for why this exists.
 *
 * ponytail: a fixed-size cache, not a font manager. The watch asks for seven
 * distinct sizes and asks for them repeatedly, so an array indexed by nothing
 * more than "which sizes have we opened" is enough; the v8 side ran a
 * registry with names, ordering and priorities that nothing in this project
 * ever called.
 */
#include "lvsf_font.h"

#include <rtthread.h>

#define DBG_TAG "lvsf.font.v9"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#if LV_USE_FREETYPE

/* Same face the v8 build registered; see resource/fonts/SConscript, which
   generates the descriptor pointing here. */
#define LVSF_FONT_PATH "/assets/fonts/tiny55_full.ttf"

/* Seven FONT_* sizes today. One spare so adding a size does not silently
   start evicting. */
#define LVSF_FONT_CACHE_MAX 8

typedef struct
{
    uint16_t   size;
    lv_font_t *font;
} font_slot_t;

static font_slot_t s_cache[LVSF_FONT_CACHE_MAX];
static uint8_t     s_used;
static bool        s_ft_ready;
static bool        s_ft_failed;

static void ft_init_once(void)
{
    if (s_ft_ready || s_ft_failed)
    {
        return;
    }

    /* Glyph cache count, not a byte budget. 256 covers the Latin set plus the
       CJK actually on screen at once; misses re-render rather than fail. */
    if (lv_freetype_init(256) != LV_RESULT_OK)
    {
        LOG_E("lv_freetype_init failed; falling back to built-in fonts");
        s_ft_failed = true;
        return;
    }
    s_ft_ready = true;
}

/* The v8 fallback ladder, kept so a missing .ttf degrades the same way:
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

    ft_init_once();
    if (!s_ft_ready)
    {
        return builtin_for(size);
    }

    if (s_used >= LVSF_FONT_CACHE_MAX)
    {
        LOG_W("font cache full (%d); size %d falls back to built-in",
              LVSF_FONT_CACHE_MAX, (int)size);
        return builtin_for(size);
    }

    lv_font_t *font = lv_freetype_font_create(LVSF_FONT_PATH,
                                              LV_FREETYPE_FONT_RENDER_MODE_BITMAP,
                                              size,
                                              LV_FREETYPE_FONT_STYLE_NORMAL);
    if (!font)
    {
        LOG_E("lv_freetype_font_create(%s, %d) failed", LVSF_FONT_PATH, (int)size);
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
        lv_freetype_font_delete(s_cache[i].font);
        s_cache[i].font = NULL;
        s_cache[i].size = 0;
    }
    s_used = 0;
}

#else /* !LV_USE_FREETYPE */

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

#endif /* LV_USE_FREETYPE */
