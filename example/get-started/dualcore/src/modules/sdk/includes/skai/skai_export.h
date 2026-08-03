/*
 * Skai SDK — capability annotation (ADR-0019).
 *
 * SKAI_EXPORT() expands to nothing. It exists purely so
 * tools/sdk/gen_dispatch.py can find exported capabilities by scanning
 * headers. The dispatch table, the phone-side capability registry and the
 * developer .d.ts are all GENERATED from these annotations — that is what
 * keeps the external API from drifting behind the internal C layer.
 *
 * Usage: annotation line, then a plain C declaration.
 *
 *     SKAI_EXPORT("battery.level", SKAI_T1, SKAI_THREAD_ANY)
 *     int32_t skai_battery_level(void);
 *
 * The generator derives the signature from the declaration itself, so the
 * annotation only carries what C cannot express: the capability name, the
 * permission tier, and the thread contract.
 *
 * Declarations are restricted to a fixed type vocabulary (see the generator).
 * Anything outside it is a hard error, not a silent skip — an exported
 * capability that the generator cannot project is worse than one that does
 * not exist.
 */
#ifndef SKAI_EXPORT_H
#define SKAI_EXPORT_H

#include <stdint.h>

/* "No reading available" for numeric capabilities. Deliberately NOT "any
 * negative value": temperature, altitude and deltas are legitimately negative,
 * and a convention that forbids negative data would be an invisible landmine
 * on every future capability. Renderers show this as "--".
 * Distinct from the negative byte counts returned by the bounded-string
 * functions, which are error returns, not data. */
#define SKAI_NO_DATA  INT32_MIN

/* ponytail: a no-op macro is the whole mechanism. A real registration macro
 * (linker section, ctor) would cost SRAM on a chronically full HCPU and buy
 * nothing — the table is known at build time. Revisit only if capabilities
 * ever need to register at runtime. */
#define SKAI_EXPORT(...)

/* Permission tiers — see ADR-0019 §tiers. */
#define SKAI_T1      1  /* auto-granted */
#define SKAI_T2      2  /* per-item user consent at install */
#define SKAI_T2_MIC  3  /* T2 + system in-use indicator + foreground-only */
#define SKAI_T3      4  /* never granted to self-signed apps */

/* Thread contract — the single most common source of bugs across the
 * LVGL / engine / BLE-parse thread split, and the one thing a refactorable C
 * layer still cannot fix after the fact. */
#define SKAI_THREAD_ANY   1  /* safe from any thread */
#define SKAI_THREAD_LVGL  2  /* must be called on the LVGL thread */
#define SKAI_THREAD_APP   3  /* must be called on the owning app's thread */

#endif /* SKAI_EXPORT_H */
