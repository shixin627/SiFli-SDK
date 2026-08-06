/*
 * Skai SDK — runtime capability lookup (ADR-0019 Phase 2).
 *
 * The dispatch table is generated from SKAI_EXPORT() annotations. This is how
 * the declarative (Tier 0) renderer reaches it: bind by NAME, not by an enum
 * the renderer has to grow. Adding a capability is one annotated declaration;
 * nothing here changes.
 *
 * Only no-argument capabilities are reachable this way, because a declarative
 * bind is a string with nowhere to put arguments. Capabilities that take
 * parameters exist for the C and (Phase 3) scripting callers.
 */
#ifndef SKAI_DISPATCH_H
#define SKAI_DISPATCH_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*skai_fn_t)(void);

typedef struct
{
    const char *name;    /* capability name, e.g. "battery.level" */
    skai_fn_t   fn;
    uint8_t     tier;    /* SKAI_T1 / SKAI_T2 / ... — Phase 3 permission gate */
    uint8_t     thread;  /* SKAI_THREAD_* */
    char        ret;     /* 'V' 'B' 'I' 'U' 'S' */
    const char *args;    /* one tag per argument; "" = no arguments */
    const char *ui;      /* display format for declarative rendering */
} skai_cap_t;

/* Table order is the generator's (sorted by name), so these indices are stable
 * within a firmware build but NOT across builds — never persist one. */
int                skai_cap_count(void);
const skai_cap_t  *skai_cap_at(int index);
int                skai_cap_index(const char *name);   /* -1 when absent */
const skai_cap_t  *skai_cap_find(const char *name);    /* NULL when absent */

/* Render a no-argument capability into `out` using its ui format. Numeric
 * capabilities that report "no data" (negative, the SDK-wide sentinel) render
 * as "--" so every bind degrades the same way. Returns false and empties `out`
 * when the capability is unusable here (takes arguments, or returns void). */
bool skai_cap_render(const skai_cap_t *c, char *out, uint32_t cap);

/* Numeric value of a no-argument numeric capability, for gauges. False when it
 * is not numeric or has no reading. */
bool skai_cap_value(const skai_cap_t *c, int32_t *out);

#endif /* SKAI_DISPATCH_H */
