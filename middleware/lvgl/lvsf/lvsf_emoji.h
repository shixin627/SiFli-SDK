/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVSF_EMOJI_H
#define LVSF_EMOJI_H

#include "lvgl.h"

/**
 * Process an emoji codepoint during glyph description (width query).
 * Handles compound emoji sequences (ZWJ, variation selectors, flags, keycaps).
 *
 * @param letter       current unicode codepoint
 * @param letter_next  next unicode codepoint (for look-ahead)
 * @return  > 0: emoji resolved, call lv_emoji_get_pending() for the image
 *          = 0: not an emoji, proceed with normal font rendering
 *          < 0: codepoint is part of an ongoing compound sequence (zero-width)
 */
int lv_emoji_process_glyph(uint32_t letter, uint32_t letter_next);

/**
 * Get the emoji image resolved by the most recent lv_emoji_process_glyph() call
 * that returned > 0.
 */
void *lv_emoji_get_pending(void);

/**
 * Get the emoji image to draw for the current draw_letter call.
 *
 * @param letter  the unicode codepoint being drawn
 * @return  valid pointer: draw this emoji image
 *          EMOJI_SKIP_DRAW: skip rendering (part of compound sequence)
 *          NULL: not an emoji, proceed with normal font rendering
 */
void *lv_emoji_get_draw(uint32_t letter);

#define EMOJI_SKIP_DRAW ((void *)(uintptr_t)1)

/* Legacy API kept for compatibility */
void *lv_get_emoji_by_unicode(uint32_t u_letter);

#endif /* LVSF_EMOJI_H */
