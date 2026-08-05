/*
 * lvsf font layer for LVGL v9.
 *
 * v8 got this from middleware/lvgl/lvsf/lvsf_font.h, which is built on the
 * closed lvsf font manager. There is no v9 port of that manager, but LVGL v9
 * ships FreeType itself (LV_USE_FREETYPE), so this rebuilds the same contract
 * -- the FONT_* size table and LV_EXT_FONT_GET() -- directly on top of it.
 *
 * Only what the watch actually calls is here. The v8 header also exposed
 * bitmap-font helpers, per-object font overrides and a font-name registry that
 * no caller in this project uses; those are left out rather than stubbed.
 */
#ifndef LVSF_FONT_V9_H
#define LVSF_FONT_V9_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Point sizes, kept identical to the v8 table for this panel so text metrics
   do not shift during the port. LV_HOR_RES_MAX is 466 here. */
#ifndef FT_SIZE_SELF_DEFINED
typedef enum
{
#if LV_HOR_RES_MAX > 350
    FONT_SMALL      = 20,
    FONT_NORMAL     = 24,
    FONT_SUBTITLE   = 28,
    FONT_TITLE      = 36,
    FONT_BIGL       = 40,
    FONT_HUGE       = 64,
    FONT_SUPER      = 90,
#else
    FONT_SMALL      = 12,
    FONT_NORMAL     = 16,
    FONT_SUBTITLE   = 20,
    FONT_TITLE      = 24,
    FONT_BIGL       = 28,
    FONT_HUGE       = 56,
    FONT_SUPER      = 72,
#endif
} FONT_SIZES;
#else
    #include "ft_size_custom_reg.h"
#endif

/* Index form, used by lvsf-style controls that pass a slot rather than a size. */
typedef enum
{
    LVSF_FONT_SMALL = 0,
    LVSF_FONT_NORMAL,
    LVSF_FONT_SUBTITLE,
    LVSF_FONT_TITLE,
    LVSF_FONT_BIG,
    LVSF_FONT_HUGE,
    LVSF_FONT_SUPER,
} LVSF_FONT_SIZES;

typedef LVSF_FONT_SIZES lvsf_font_size_t;

#define lvsf_convert_font_size(size)                            \
  switch(size)                                                  \
  {                                                             \
    case LVSF_FONT_SMALL:     size = FONT_SMALL;      break;    \
    case LVSF_FONT_NORMAL:    size = FONT_NORMAL;     break;    \
    case LVSF_FONT_SUBTITLE:  size = FONT_SUBTITLE;   break;    \
    case LVSF_FONT_TITLE:     size = FONT_TITLE;      break;    \
    case LVSF_FONT_BIG:       size = FONT_BIGL;       break;    \
    case LVSF_FONT_HUGE:      size = FONT_HUGE;       break;    \
    case LVSF_FONT_SUPER:     size = FONT_SUPER;      break;    \
  }

/* Font for `size`, which may be a FONT_* point size or an LVSF_FONT_* slot.
   Never returns NULL: if FreeType is unavailable (no .ttf on the filesystem
   yet, FS corrupt) it falls back to the LVGL built-in fonts so the watch still
   boots and renders ASCII, which is what the v8 path did. */
const lv_font_t *LV_EXT_FONT_GET(uint16_t size);

/* Drop every cached face. Call from the LVGL thread only. */
void lv_ext_font_reset(void);

#define lv_ext_get_font_size(size) size

#ifdef __cplusplus
}
#endif

#endif /* LVSF_FONT_V9_H */
