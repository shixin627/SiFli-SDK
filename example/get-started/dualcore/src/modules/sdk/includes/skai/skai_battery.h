/*
 * Skai SDK — battery. Phase 0: contract only, no implementation.
 * Backend (Phase 1): SkaiWatchSys.battery_level_value / .charger_status.
 * Note bloc_battery is NOT linked into HCPU — do not reach for
 * battery_get_charge_state() here.
 */
#ifndef SKAI_BATTERY_H
#define SKAI_BATTERY_H

#include <stdbool.h>
#include <stdint.h>
#include "skai/skai_export.h"

/* State of charge, 0..100. SKAI_NO_DATA before the first reading lands.
 * The "%d%%" is the declarative renderer's display format — it lives in the
 * dispatch table so units travel with the capability instead of in a second
 * table the renderer would have to keep in sync. */
SKAI_EXPORT("battery.level", SKAI_T1, SKAI_THREAD_ANY, "%d%%")
int32_t skai_battery_level(void);

SKAI_EXPORT("battery.charging", SKAI_T1, SKAI_THREAD_ANY)
bool skai_battery_charging(void);

#endif /* SKAI_BATTERY_H */
