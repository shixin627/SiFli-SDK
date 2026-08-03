/*
 * Skai SDK — dispatch table instance and lookup (ADR-0019 Phase 2).
 *
 * The table body is generated; this file only gives it storage and the three
 * operations callers need. If you are here to add a capability, you are in the
 * wrong file — annotate the declaration in includes/skai/ and re-run
 * tools/sdk/gen_dispatch.py.
 */
#include <string.h>

#include <rtthread.h>

#define DBG_TAG "skai.disp"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "skai/skai_dispatch.h"
#include "skai/skai_export.h"
/* Every capability domain, generated — so adding one never means remembering
   to edit this file. */
#include "generated/skai_all.h"

static const skai_cap_t s_caps[] =
{
#define SKAI_DISPATCH(idx, cap, fn, tier, thread, ret, args, ui) \
    { cap, (skai_fn_t)(fn), tier, thread, ret, args, ui },
#include "generated/skai_dispatch_table.inc"
#undef SKAI_DISPATCH
};

int skai_cap_count(void)
{
    return SKAI_DISPATCH_COUNT;
}

const skai_cap_t *skai_cap_at(int index)
{
    if (index < 0 || index >= SKAI_DISPATCH_COUNT)
        return RT_NULL;
    return &s_caps[index];
}

int skai_cap_index(const char *name)
{
    int lo = 0, hi = SKAI_DISPATCH_COUNT - 1;

    if (!name)
        return -1;
    /* The generator sorts by name, so binary search is free. */
    while (lo <= hi)
    {
        int mid = lo + (hi - lo) / 2;
        int cmp = strcmp(name, s_caps[mid].name);
        if (cmp == 0)
            return mid;
        if (cmp < 0)
            hi = mid - 1;
        else
            lo = mid + 1;
    }
    return -1;
}

const skai_cap_t *skai_cap_find(const char *name)
{
    int i = skai_cap_index(name);
    return (i < 0) ? RT_NULL : &s_caps[i];
}

bool skai_cap_value(const skai_cap_t *c, int32_t *out)
{
    if (!c || !out || c->args[0] != '\0')
        return false;

    switch (c->ret)
    {
    case 'I':
    {
        int32_t v = ((int32_t (*)(void))c->fn)();
        if (v == SKAI_NO_DATA)
            return false;
        *out = v;
        return true;
    }
    case 'U':
        *out = (int32_t)((uint32_t (*)(void))c->fn)();
        return true;
    case 'B':
        *out = ((bool (*)(void))c->fn)() ? 1 : 0;
        return true;
    default:
        return false;
    }
}

bool skai_cap_render(const skai_cap_t *c, char *out, uint32_t cap)
{
    if (!out || cap == 0)
        return false;
    out[0] = '\0';
    if (!c || c->args[0] != '\0')
        return false;

    if (c->ret == 'S')
    {
        /* ponytail: the ui format is ignored for string capabilities — they
         * render themselves. Add a temp-buffer path here if one ever needs a
         * suffix; nothing does today. */
        return ((int32_t (*)(char *, uint32_t))c->fn)(out, cap) >= 0;
    }

    {
        int32_t v;
        if (!skai_cap_value(c, &v))
        {
            /* Every bind degrades identically, so a missing heart rate looks
             * the same as a missing anything else. */
            rt_snprintf(out, cap, "--");
            return true;
        }
        rt_snprintf(out, cap, c->ui, (int)v);
        return true;
    }
}
