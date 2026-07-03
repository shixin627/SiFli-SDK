/**
 * @file   skaiapp_render.h
 * @brief  SkaiApp declarative model → LVGL page builder + live-bind refresher.
 *         LVGL thread only.
 */
#ifndef SKAIAPP_RENDER_H
#define SKAIAPP_RENDER_H

#include "lvgl.h"
#include "skaiapp_pkg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    lv_obj_t *obj[SKAIAPP_MAX_ITEMS]; /* bind targets; NULL when static */
} skaiapp_render_ctx_t;

/* Build the page under `parent`. `m` must stay alive until the returned
   container is deleted (button callbacks read it) — the host guarantees
   delete-page-then-free-model ordering and calls skaiapp_render_detach()
   in between. */
lv_obj_t *skaiapp_render_page(lv_obj_t *parent, const skaiapp_model_t *m,
                              skaiapp_render_ctx_t *ctx);

/* Refresh every bound widget (page tick, ~500 ms while visible). */
void skaiapp_render_refresh(const skaiapp_model_t *m, skaiapp_render_ctx_t *ctx);

/* Invalidate the module's model back-pointer before freeing the model. */
void skaiapp_render_detach(void);

/* Shared icon mapping (launcher rows reuse it). 0xFF/unknown → fallback. */
const void *skaiapp_render_icon_src(uint8_t icon_enum);

#ifdef __cplusplus
}
#endif

#endif /* SKAIAPP_RENDER_H */
