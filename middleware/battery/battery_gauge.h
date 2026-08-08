/*
 * SPDX-FileCopyrightText: 2026 Skaiwalk Technology
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * State-based battery gauge.
 *
 * WHY THIS EXISTS (and why battery_calculator.c is still here untouched):
 * the legacy calculator computes `percent = lookup(table, terminal_voltage)`,
 * i.e. the displayed SOC is a pure function of the instantaneous voltage, with
 * a rate limiter bolted on afterwards to hide the jumps. That has three
 * failure modes we hit in the field:
 *
 *   1. The charge and discharge tables are not on the same SOC scale (at 30%
 *      the discharge table says 3550 mV and the charging table says 3974 mV --
 *      424 mV apart, which no plausible charge-current * internal-resistance
 *      can explain). Crossing tables on plug-in is therefore a step change of
 *      tens of percent in either direction.
 *   2. The rate limiter is skipped entirely while `last_percent == 0`, which is
 *      the state after every reset -- so a reboot on the charger published the
 *      raw charging-table value (~10% at a real ~3.9 V).
 *   3. The limiter counts *samples*, not time, while the sample interval varies
 *      between 10 s, 60 s and 5 min. The effective discharge rate therefore
 *      varies ~30x with no relation to reality.
 *
 * This gauge fixes the class, not the instances, by making SOC a *state
 * variable* rather than a lookup result:
 *
 *   - SOC only ever changes by an amount proportional to elapsed time.
 *     A charge-state transition changes the *rate*, never the *value*, so
 *     plugging in cannot move the number no matter what the tables say.
 *   - One table is used (the discharge/OCV one), always. The terminal voltage
 *     is corrected to an open-circuit estimate before lookup, so the same
 *     table is valid in both directions. The charging table is left in place
 *     but is not consulted -- see BATTERY_GAUGE_USE_LEGACY below to go back.
 *   - While charging, SOC advances on a time integral (the charge current is
 *     roughly constant, so SOC is roughly linear in time) with a slow pull
 *     toward the OCV estimate so a mis-calibrated rate self-corrects instead
 *     of running away. Charge-done from the charger IC snaps to 100%.
 *   - The gauge is seeded once, from the OCV estimate, which is at its most
 *     accurate right after a cold boot (the cell has been resting). The HCPU
 *     may override that with a persisted value via battery_gauge_seed(), which
 *     matters only when we boot while already on the charger.
 *
 * All calibration knobs live at the top of battery_gauge.c and are reachable
 * at runtime through the `batgauge` MSH command.
 */

#ifndef _BATTERY_GAUGE_H_
#define _BATTERY_GAUGE_H_

#include <stdint.h>
#include <stdbool.h>
#include "battery_calculator.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Reset the gauge to the unseeded state (test / diagnostics only).
 */
void battery_gauge_reset(void);

/**
 * @brief Install the OCV curve used for both charge and discharge.
 *
 * @param table      curve table, sorted by descending percent.
 * @param table_size number of entries.
 */
void battery_gauge_set_curve(const battery_lookup_point_t *table,
                             uint32_t table_size);

/**
 * @brief Seed the gauge with a known-good SOC, e.g. one persisted by the HCPU
 *        across a reset.
 *
 * Ignored once the gauge has already been seeded, so a late-arriving seed can
 * never yank a gauge that is already tracking. Takes effect only if it arrives
 * before the first update, which is the case the seed exists for (booting on
 * the charger, where no trustworthy OCV reading is available).
 *
 * @param percent 1..100. 0 is rejected -- it is indistinguishable from "no
 *                stored value" in the persistence layer.
 * @return true if the seed was accepted.
 */
bool battery_gauge_seed(uint8_t percent);

/**
 * @brief Advance the gauge.
 *
 * @param voltage_01mv terminal voltage in 0.1 mV units.
 * @param charging     true if the charger reports current flowing in.
 * @param charge_full  true if the charger reports charge termination.
 *
 * @return displayed battery percentage, 0..100.
 */
uint8_t battery_gauge_update(uint32_t voltage_01mv, bool charging,
                             bool charge_full);

/**
 * @brief Current displayed percentage without advancing the gauge.
 */
uint8_t battery_gauge_get(void);

/**
 * @brief True once the gauge holds a real estimate (as opposed to nothing).
 */
bool battery_gauge_is_seeded(void);

/**
 * @brief Open-circuit voltage estimate for a terminal reading, in 0.1 mV.
 *        Exposed for diagnostics.
 */
uint32_t battery_gauge_ocv_estimate(uint32_t voltage_01mv, bool charging);

/**
 * @brief Minutes for a 0->100% charge, used by the charging time integral.
 *        Calibration knob; see battery_gauge.c for the default.
 */
void battery_gauge_set_charge_minutes(uint32_t minutes);
uint32_t battery_gauge_get_charge_minutes(void);

/**
 * @brief Dump internal state to the log. Diagnostics only.
 */
void battery_gauge_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* _BATTERY_GAUGE_H_ */
