/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "lv_ext_resource_manager.h"

#if defined (EMOJI_SUPPORT)

#include "lvgl.h"
#include "lvsf_emoji.h"
#include <string.h>
#include <stdio.h>

#ifndef EMOJI_CACHE_MAX
#define EMOJI_CACHE_MAX 60
#endif

#define EMOJI_SEQ_MAX       10
#define EMOJI_RES_PATH      "/assets/emoji/"
#define EMOJI_RES_SUFFIX    ".bin"

/* ── Cache (dynamic linked list, LRU eviction) ────────────────────── */

typedef struct emoji_cache_entry {
    struct emoji_cache_entry *next;
    char path[80];
    uint32_t access_cnt;
    lv_img_dsc_t img_dsc;
    uint8_t *data;
} emoji_cache_entry_t;

static emoji_cache_entry_t *emoji_cache_head = NULL;
static uint32_t emoji_cache_count = 0;
static uint32_t emoji_access_counter = 0;

static emoji_cache_entry_t *cache_find(const char *path)
{
    emoji_cache_entry_t *e = emoji_cache_head;
    while (e)
    {
        if (strcmp(e->path, path) == 0)
        {
            e->access_cnt = ++emoji_access_counter;
            return e;
        }
        e = e->next;
    }
    return NULL;
}

static void cache_evict_lru(void)
{
    if (!emoji_cache_head) return;

    emoji_cache_entry_t *lru = emoji_cache_head, *lru_prev = NULL;
    emoji_cache_entry_t *prev = NULL, *e = emoji_cache_head;
    while (e)
    {
        if (e->access_cnt < lru->access_cnt)
        {
            lru = e;
            lru_prev = prev;
        }
        prev = e;
        e = e->next;
    }
    if (lru_prev) lru_prev->next = lru->next;
    else          emoji_cache_head = lru->next;

    lv_mem_free(lru->data);
    lv_mem_free(lru);
    emoji_cache_count--;
}

static emoji_cache_entry_t *cache_load(const char *path)
{
    lv_fs_file_t f;
    if (lv_fs_open(&f, path, LV_FS_MODE_RD) != LV_FS_RES_OK)
        return NULL;

    lv_img_header_t header;
    uint32_t br;
    if (lv_fs_read(&f, &header, sizeof(header), &br) != LV_FS_RES_OK || br != sizeof(header))
    {
        lv_fs_close(&f);
        return NULL;
    }

    uint32_t file_size = 0;
    lv_fs_seek(&f, 0, LV_FS_SEEK_END);
    lv_fs_tell(&f, &file_size);
    uint32_t data_size = file_size - sizeof(header);
    lv_fs_seek(&f, sizeof(header), LV_FS_SEEK_SET);

    uint8_t *buf = lv_mem_alloc(data_size);
    if (!buf) { lv_fs_close(&f); return NULL; }

    if (lv_fs_read(&f, buf, data_size, &br) != LV_FS_RES_OK || br != data_size)
    {
        lv_mem_free(buf);
        lv_fs_close(&f);
        return NULL;
    }
    lv_fs_close(&f);

    if (emoji_cache_count >= EMOJI_CACHE_MAX)
        cache_evict_lru();

    emoji_cache_entry_t *entry = lv_mem_alloc(sizeof(emoji_cache_entry_t));
    if (!entry) { lv_mem_free(buf); return NULL; }

    strncpy(entry->path, path, sizeof(entry->path) - 1);
    entry->path[sizeof(entry->path) - 1] = '\0';
    entry->data = buf;
    entry->access_cnt = ++emoji_access_counter;
    entry->img_dsc.header = header;
    entry->img_dsc.data_size = data_size;
    entry->img_dsc.data = buf;

    entry->next = emoji_cache_head;
    emoji_cache_head = entry;
    emoji_cache_count++;
    return entry;
}

/* Get a cached (or freshly loaded) emoji by file path.
   Returns pointer to lv_img_dsc_t or NULL. */
static void *emoji_get_by_path(const char *path)
{
    emoji_cache_entry_t *e = cache_find(path);
    if (e) return &e->img_dsc;
    e = cache_load(path);
    if (e) return &e->img_dsc;
    return NULL;
}

/* ── Path builder ─────────────────────────────────────────────────── */

static char path_buf[80];

/* Build path from a single unicode codepoint using the switch table */
static const char *single_to_path(uint32_t u)
{
#define GET_EMOJI_INFO(_id) \
    case 0x##_id: \
        sprintf(path_buf, "%semoji_%s%s", EMOJI_RES_PATH, #_id, EMOJI_RES_SUFFIX); \
        return path_buf;

    switch (u)
    {
#include "emoji_info.h"
    default:
        return NULL;
    }
#undef GET_EMOJI_INFO
}

/* Build path from a sequence of codepoints, e.g. "emoji_1f468_200d_1f469.bin" */
static const char *seq_to_path(const uint32_t *seq, int len)
{
    char *p = path_buf;
    int remain = sizeof(path_buf);
    int n;

    n = snprintf(p, remain, "%semoji", EMOJI_RES_PATH);
    p += n; remain -= n;

    for (int i = 0; i < len && remain > 12; i++)
    {
        n = snprintf(p, remain, "_%x", seq[i]);
        p += n; remain -= n;
    }
    snprintf(p, remain, "%s", EMOJI_RES_SUFFIX);
    return path_buf;
}

/* ── Sequence state machine ───────────────────────────────────────── */

static uint32_t seq_buf[EMOJI_SEQ_MAX];
static int      seq_len = 0;
static bool     seq_active = false;

/* Resolved emoji from the most recent completed sequence */
static void *pending_emoji = NULL;

/* Draw-phase tracking — trigger-based (robust across multiple labels / re-renders).
   Instead of a fragile counter, we record WHICH unicode codepoint should trigger
   the draw and whether the most recent width query was a sequence member. */
static bool     last_glyph_is_seq_member = false;  /* set by process_glyph */
static uint32_t draw_trigger_letter = 0;            /* unicode that triggers the draw */
static void    *draw_trigger_emoji  = NULL;         /* image to draw on that trigger */

static bool is_emoji_codepoint(uint32_t cp)
{
    /* Emoji & symbols block (U+1F000 – U+1FFFF) */
    if (cp >= 0x1F000 && cp <= 0x1FFFF) return true;
    /* Regional indicators for flags */
    if (cp >= 0x1F1E6 && cp <= 0x1F1FF) return true;
    /* Misc symbols & dingbats */
    if (cp >= 0x2600 && cp <= 0x27BF) return true;
    /* Misc technical (e.g. ⌚⌛) */
    if (cp >= 0x2300 && cp <= 0x23FF) return true;
    /* Arrows / geometric supplement */
    if (cp >= 0x2B05 && cp <= 0x2B55) return true;
    /* ©, ® */
    if (cp == 0x00A9 || cp == 0x00AE) return true;
    /* Keycap bases: only # * 0-9, NOT $%&'()+,-./ */
    if (cp == 0x0023 || cp == 0x002A) return true;
    if (cp >= 0x0030 && cp <= 0x0039) return true;
    /* Specific misc symbols used as emoji */
    if (cp == 0x203C || cp == 0x2049) return true;
    if (cp >= 0x2194 && cp <= 0x21AA) return true;
    if (cp >= 0x25AA && cp <= 0x25FE) return true;
    if (cp == 0x2122 || cp == 0x2139) return true;
    if (cp >= 0x2934 && cp <= 0x2935) return true;
    if (cp == 0x3030 || cp == 0x303D) return true;
    if (cp == 0x3297 || cp == 0x3299) return true;
    return false;
}

/* Check if the sequence should continue accumulating */
static bool seq_should_continue(uint32_t letter, uint32_t letter_next)
{
    /* next is a joiner / variation selector / keycap combiner */
    if (letter_next == 0x200D || letter_next == 0xFE0F || letter_next == 0x20E3)
        return true;
    /* current is ZWJ → always expect one more codepoint after it */
    if (letter == 0x200D)
        return true;
    /* regional indicator pair (flags) */
    if (letter >= 0x1F1E6 && letter <= 0x1F1FF &&
        letter_next >= 0x1F1E6 && letter_next <= 0x1F1FF)
        return true;
    /* skin tone modifier follows */
    if (letter_next >= 0x1F3FB && letter_next <= 0x1F3FF)
        return true;
    return false;
}

static void *resolve_sequence(void)
{
    /* Try full compound path first */
    const char *path = seq_to_path(seq_buf, seq_len);
    void *img = emoji_get_by_path(path);
    if (img) return img;

    /* Fallback: try single codepoint for the first non-joiner in sequence */
    for (int i = 0; i < seq_len; i++)
    {
        if (seq_buf[i] == 0x200D || seq_buf[i] == 0xFE0F || seq_buf[i] == 0x20E3)
            continue;
        path = single_to_path(seq_buf[i]);
        if (path)
        {
            img = emoji_get_by_path(path);
            if (img) return img;
        }
        break; /* only try the first real codepoint */
    }
    return NULL;
}

int lv_emoji_process_glyph(uint32_t letter, uint32_t letter_next)
{
    /* Joiners / variation selector / keycap combiner — always zero-width.
       If a sequence is active, append and check whether it should be resolved
       here (e.g. ❤️ = 2764 FE0F ends on FE0F).  Without this, seq_active would
       leak across the end of the sequence and sweep the next unrelated letter
       into resolve_sequence(), causing failed fs lookups (lag) and mis-drawn
       emojis on the wrong character. */
    if (letter == 0xFE0F || letter == 0x200D || letter == 0x20E3)
    {
        if (seq_active)
        {
            if (seq_len < EMOJI_SEQ_MAX)
                seq_buf[seq_len++] = letter;

            if (seq_should_continue(letter, letter_next))
            {
                last_glyph_is_seq_member = true;
                return -1;
            }

            pending_emoji = resolve_sequence();
            seq_active = false;
            seq_len = 0;
            if (pending_emoji)
            {
                draw_trigger_letter = letter;
                draw_trigger_emoji  = pending_emoji;
                last_glyph_is_seq_member = false;
                return 1;
            }
            last_glyph_is_seq_member = false;
            return 0;
        }
        last_glyph_is_seq_member = true;
        return -1; /* stray joiner outside any sequence — zero width */
    }

    /* Skin tone modifier in an active sequence */
    if (seq_active && letter >= 0x1F3FB && letter <= 0x1F3FF)
    {
        if (seq_len < EMOJI_SEQ_MAX)
            seq_buf[seq_len++] = letter;

        if (seq_should_continue(letter, letter_next))
        {
            last_glyph_is_seq_member = true;
            return -1;
        }

        /* Sequence ends here */
        pending_emoji = resolve_sequence();
        seq_active = false;
        seq_len = 0;
        if (pending_emoji)
        {
            draw_trigger_letter = letter;
            draw_trigger_emoji  = pending_emoji;
            last_glyph_is_seq_member = false;
            return 1;
        }
        last_glyph_is_seq_member = false;
        return 0;
    }

    /* Continuation of an active compound sequence */
    if (seq_active)
    {
        if (seq_len < EMOJI_SEQ_MAX)
            seq_buf[seq_len++] = letter;

        if (seq_should_continue(letter, letter_next))
        {
            last_glyph_is_seq_member = true;
            return -1; /* more to come */
        }

        /* Sequence ends — resolve */
        pending_emoji = resolve_sequence();
        seq_active = false;
        seq_len = 0;
        if (pending_emoji)
        {
            draw_trigger_letter = letter;
            draw_trigger_emoji  = pending_emoji;
            last_glyph_is_seq_member = false;
            return 1;
        }
        last_glyph_is_seq_member = false;
        return 0;
    }

    /* Not in a sequence — check if this codepoint starts one */
    if (!is_emoji_codepoint(letter))
    {
        last_glyph_is_seq_member = false;
        return 0; /* not an emoji */
    }

    if (seq_should_continue(letter, letter_next))
    {
        /* Start a new compound sequence */
        seq_buf[0] = letter;
        seq_len = 1;
        seq_active = true;
        last_glyph_is_seq_member = true;
        return -1;
    }

    /* Standalone single-codepoint emoji */
    const char *path = single_to_path(letter);
    if (path)
    {
        pending_emoji = emoji_get_by_path(path);
        if (pending_emoji)
        {
            draw_trigger_letter = letter;
            draw_trigger_emoji  = pending_emoji;
            last_glyph_is_seq_member = false;
            return 1;
        }
    }
    last_glyph_is_seq_member = false;
    return 0;
}

void *lv_emoji_get_pending(void)
{
    return pending_emoji;
}

void *lv_emoji_get_draw(uint32_t letter)
{
    /* The most recent width-query told us this codepoint is a mid-sequence
       member (joiner, variation selector, or compound-start).  Skip drawing
       so draw_letter does NOT fall through to get_glyph_dsc(letter, '\0')
       which would re-enter the state machine and corrupt it. */
    if (last_glyph_is_seq_member)
        return EMOJI_SKIP_DRAW;

    /* Check if this is the codepoint that should trigger the emoji draw.
       Matching by unicode makes this robust across re-renders: if a stale
       trigger is left from a previous label, a different letter simply won't
       match and gets normal rendering. */
    if (draw_trigger_emoji && letter == draw_trigger_letter)
    {
        void *img = draw_trigger_emoji;
        draw_trigger_emoji  = NULL;
        draw_trigger_letter = 0;
        return img;
    }

    return NULL;
}

/* Legacy API — simple single-codepoint lookup */
void *lv_get_emoji_by_unicode(uint32_t u_letter)
{
    const char *path = single_to_path(u_letter);
    if (path) return emoji_get_by_path(path);
    return NULL;
}

#else /* !EMOJI_SUPPORT */

int  lv_emoji_process_glyph(uint32_t letter, uint32_t letter_next) { return 0; }
void *lv_emoji_get_pending(void) { return NULL; }
void *lv_emoji_get_draw(uint32_t letter) { return NULL; }
void *lv_get_emoji_by_unicode(uint32_t u_letter) { return NULL; }

#endif
