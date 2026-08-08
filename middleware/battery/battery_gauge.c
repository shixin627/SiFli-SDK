/*
 * SPDX-FileCopyrightText: 2026 Skaiwalk Technology
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * State-based battery gauge. Rationale and design contract: battery_gauge.h.
 */

#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>

#include "battery_gauge.h"

#define DBG_TAG "bat.gauge"
#include <rtdbg.h>

/* ------------------------------------------------------------------------ */
/* Calibration knobs                                                         */
/* ------------------------------------------------------------------------ */

/* Minutes for a 0 -> 100% charge. The charging SOC is a time integral because
 * the charger runs constant-current for most of the range, so SOC is roughly
 * linear in time -- far more trustworthy than reading a voltage that is offset
 * by an unknown IR term. This is a bench-measurable number: charge from flat to
 * termination and time it. Until that measurement exists the slow OCV pull
 * below keeps a wrong value from running away. */
#define GAUGE_CHARGE_FULL_MINUTES_DEFAULT   100u

/* Charge-current IR rise, in 0.1 mV. Terminal voltage while charging sits this
 * far ABOVE the open-circuit voltage; we subtract it before the curve lookup.
 * 60 mV = ~300 mA (AW32001 CC level 1) into ~200 mOhm of cell impedance, which
 * matches the small step observed on the bench when the cable is plugged in.
 * Measure properly by logging the voltage across a plug-in event with the
 * screen off: the instantaneous step IS this number. */
#define GAUGE_CHARGE_IR_01MV                600u

/* Load IR drop while discharging, in 0.1 mV. Deliberately 0: the discharge
 * curve in battery_table.c was measured under a real load, so an average load
 * sag is already baked into the table. Only raise this if the table is ever
 * re-measured as a true open-circuit (rested) curve. */
#define GAUGE_DISCHARGE_IR_01MV             0u

/* How fast the charging time integral is allowed to be corrected toward the
 * OCV estimate, in percent per hour. Small on purpose: it is a slow leash on a
 * mis-calibrated charge rate, not a tracking loop. */
#define GAUGE_CHARGE_CORRECT_PCT_PER_HOUR   10u

/* Maximum rate at which the displayed SOC may fall while discharging, in
 * percent per hour. This is the whole discharge model: track the OCV estimate,
 * but never faster than a real cell can actually drain. It replaces the legacy
 * "-1% per sample" limiter, which meant something different at every sample
 * interval. 40%/h = a 2.5 h full discharge, comfortably faster than any real
 * usage, so it only ever bites on load sag and glitches. */
#define GAUGE_DISCHARGE_DOWN_PCT_PER_HOUR   40u

/* Maximum rate at which the displayed SOC may RISE while discharging.
 * 0 by product decision: a percentage that climbs while unplugged reads as a
 * bug even when it is a correct recovery from an earlier under-read. The cost
 * is that an unplug leaves any accumulated under-estimate in place until the
 * next full charge (charge termination snaps to 100% and re-anchors). Raise to
 * ~2 to trade that for a slow self-heal. */
#define GAUGE_DISCHARGE_UP_PCT_PER_HOUR     0u

/* Largest gap between two updates the gauge will integrate over. Bounds the
 * damage from a stuck timer or a tick discontinuity; a genuine hour-long gap
 * (deep sleep) still integrates correctly up to this limit. */
#define GAUGE_MAX_DT_MS                     3600000u

/* Internal resolution: centi-percent, 0..10000. Integer throughout -- this runs
 * on the LCPU, which has no FPU. */
#define GAUGE_SOC_MAX                       10000

/* ------------------------------------------------------------------------ */

static const battery_lookup_point_t *s_curve = RT_NULL;
static uint32_t s_curve_size = 0;

static int32_t  s_soc_centi = 0;        /* 0..10000 */
static bool     s_seeded = false;
static bool     s_have_tick = false;
static uint32_t s_last_tick_ms = 0;
static uint32_t s_charge_residual_ms = 0;
static bool     s_last_charging = false;
static bool     s_charge_done = false;  /* termination seen this charge session */
static uint32_t s_charge_minutes = GAUGE_CHARGE_FULL_MINUTES_DEFAULT;

/* Last inputs, kept for the diagnostic dump only. */
static uint32_t s_last_voltage_01mv = 0;
static uint32_t s_last_ocv_01mv = 0;
static uint32_t s_last_ocv_soc = 0;

void battery_gauge_set_curve(const battery_lookup_point_t *table,
                             uint32_t table_size)
{
    s_curve = table;
    s_curve_size = table_size;
}

void battery_gauge_reset(void)
{
    s_soc_centi = 0;
    s_seeded = false;
    s_have_tick = false;
    s_charge_residual_ms = 0;
    LOG_I("gauge reset");
}

bool battery_gauge_seed(uint8_t percent)
{
    if (s_seeded)
    {
        LOG_D("seed %d%% ignored, gauge already tracking at %d%%",
              percent, s_soc_centi / 100);
        return false;
    }
    if (percent == 0 || percent > 100)
    {
        /* 0 is what the persistence layer returns when nothing was stored. */
        return false;
    }
    s_soc_centi = (int32_t)percent * 100;
    s_seeded = true;
    LOG_I("gauge seeded from persisted value: %d%%", percent);
    return true;
}

uint32_t battery_gauge_ocv_estimate(uint32_t voltage_01mv, bool charging)
{
    if (charging)
    {
        /* Terminal sits above OCV while current flows in. */
        return (voltage_01mv > GAUGE_CHARGE_IR_01MV)
                   ? (voltage_01mv - GAUGE_CHARGE_IR_01MV)
                   : 0;
    }
    return voltage_01mv + GAUGE_DISCHARGE_IR_01MV;
}

void battery_gauge_set_charge_minutes(uint32_t minutes)
{
    if (minutes >= 10 && minutes <= 600)
    {
        s_charge_minutes = minutes;
        LOG_I("charge full time set to %d min", minutes);
    }
}

uint32_t battery_gauge_get_charge_minutes(void)
{
    return s_charge_minutes;
}

/* Elapsed milliseconds since the previous update, clamped. Returns 0 on the
 * first call, when no interval is defined yet. */
static uint32_t gauge_elapsed_ms(void)
{
    uint32_t now = (uint32_t)rt_tick_get_millisecond();

    if (!s_have_tick)
    {
        s_have_tick = true;
        s_last_tick_ms = now;
        return 0;
    }

    uint32_t dt = now - s_last_tick_ms;  /* unsigned: wraps correctly */
    s_last_tick_ms = now;

    if (dt > GAUGE_MAX_DT_MS)
    {
        LOG_W("update gap %u ms clamped to %u ms", dt, GAUGE_MAX_DT_MS);
        dt = GAUGE_MAX_DT_MS;
    }
    return dt;
}

/* Move s_soc_centi toward `target_centi`, by at most `pct_per_hour` worth of
 * `dt_ms`. Direction-agnostic; the caller picks the limit per direction. */
static void gauge_correct_toward(int32_t target_centi, uint32_t pct_per_hour,
                                 uint32_t dt_ms)
{
    if (pct_per_hour == 0 || dt_ms == 0)
    {
        return;
    }

    /* centi-percent budget = pct_per_hour * 100 * dt_ms / 3600000 */
    int32_t budget = (int32_t)((pct_per_hour * dt_ms) / 36000u);
    if (budget <= 0)
    {
        return;
    }

    int32_t delta = target_centi - s_soc_centi;
    if (delta > budget)
    {
        delta = budget;
    }
    else if (delta < -budget)
    {
        delta = -budget;
    }
    s_soc_centi += delta;
}

uint8_t battery_gauge_update(uint32_t voltage_01mv, bool charging,
                             bool charge_full)
{
    if (s_curve == RT_NULL || s_curve_size == 0)
    {
        LOG_E("no curve installed");
        return (uint8_t)(s_soc_centi / 100);
    }

    uint32_t ocv = battery_gauge_ocv_estimate(voltage_01mv, charging);
    int32_t ocv_soc_centi =
        (int32_t)battery_percent_from_curve_table(s_curve, s_curve_size, ocv) * 100;

    s_last_voltage_01mv = voltage_01mv;
    s_last_ocv_01mv = ocv;
    s_last_ocv_soc = (uint32_t)(ocv_soc_centi / 100);

    uint32_t dt_ms = gauge_elapsed_ms();

    /* First real sample: adopt the OCV estimate outright. Right after a cold
     * boot the cell has been resting, which is exactly when an OCV lookup is at
     * its most accurate -- so this is the correct anchor, not a fallback. If
     * we booted onto the charger the HCPU may already have seeded us with the
     * persisted value, in which case s_seeded is set and we skip this. */
    if (!s_seeded)
    {
        s_soc_centi = ocv_soc_centi;
        s_seeded = true;
        s_last_charging = charging;
        LOG_I("gauge anchored at %d%% (%d mV, ocv %d mV, charging=%d)",
              s_soc_centi / 100, voltage_01mv / 10, ocv / 10, charging);
        return (uint8_t)(s_soc_centi / 100);
    }

    /* A charge-state transition changes the RATE below, never the VALUE. This
     * one line is what makes plug-in and unplug invisible to the user, and it
     * holds regardless of how badly the curves disagree. */
    if (charging != s_last_charging)
    {
        LOG_I("charge state %d -> %d at %d%% (value held, rate switched)",
              s_last_charging, charging, s_soc_centi / 100);
        s_last_charging = charging;
        s_charge_residual_ms = 0;
        if (!charging)
        {
            s_charge_done = false;   /* new charge session next time */
        }
    }

    if (charging)
    {
        /* Once termination has been reported, stay at 100% for the rest of the
         * session. The charger drops in and out of "done" as it top-balances,
         * and both the time integral and the OCV pull would otherwise walk the
         * display back down to 99% while the watch is still sitting on the
         * dock -- which reads as a broken charger. */
        if (charge_full)
        {
            s_charge_done = true;
        }

        if (s_charge_done)
        {
            /* The charger IC is the authority on termination. */
            if (s_soc_centi != GAUGE_SOC_MAX)
            {
                LOG_I("charge termination reported, snapping %d%% -> 100%%",
                      s_soc_centi / 100);
            }
            s_soc_centi = GAUGE_SOC_MAX;
        }
        else
        {
            /* Time integral. ms per centi-percent = minutes * 60000 / 10000. */
            uint32_t ms_per_centi = s_charge_minutes * 6u;
            uint32_t total = s_charge_residual_ms + dt_ms;
            s_soc_centi += (int32_t)(total / ms_per_centi);
            s_charge_residual_ms = total % ms_per_centi;

            /* Slow leash so a mis-calibrated charge time cannot run away. */
            gauge_correct_toward(ocv_soc_centi,
                                 GAUGE_CHARGE_CORRECT_PCT_PER_HOUR, dt_ms);

            /* Do not show 100% on a guess. Either the charger says it is done,
             * or the cell is genuinely at the top of the curve. Without this
             * the time integral would claim 100% while the charger is still
             * pushing current, and the user would unplug early. */
            int32_t ceiling = (ocv_soc_centi >= GAUGE_SOC_MAX)
                                  ? GAUGE_SOC_MAX
                                  : (GAUGE_SOC_MAX - 100);
            if (s_soc_centi > ceiling)
            {
                s_soc_centi = ceiling;
            }
        }
    }
    else
    {
        /* Discharge: track the OCV estimate, rate-limited in both directions.
         * The down limit rejects load sag (which an OCV gauge must ignore); the
         * up limit is 0 by product decision -- see the knob comment. */
        if (ocv_soc_centi < s_soc_centi)
        {
            gauge_correct_toward(ocv_soc_centi,
                                 GAUGE_DISCHARGE_DOWN_PCT_PER_HOUR, dt_ms);
        }
        else
        {
            gauge_correct_toward(ocv_soc_centi,
                                 GAUGE_DISCHARGE_UP_PCT_PER_HOUR, dt_ms);
        }
    }

    if (s_soc_centi < 0)
    {
        s_soc_centi = 0;
    }
    else if (s_soc_centi > GAUGE_SOC_MAX)
    {
        s_soc_centi = GAUGE_SOC_MAX;
    }

    LOG_D("v=%d mV ocv=%d mV ocv_soc=%d%% soc=%d.%02d%% chg=%d full=%d dt=%u",
          voltage_01mv / 10, ocv / 10, ocv_soc_centi / 100,
          s_soc_centi / 100, s_soc_centi % 100, charging, charge_full, dt_ms);

    return (uint8_t)(s_soc_centi / 100);
}

uint8_t battery_gauge_get(void)
{
    return (uint8_t)(s_soc_centi / 100);
}

bool battery_gauge_is_seeded(void)
{
    return s_seeded;
}

void battery_gauge_dump(void)
{
    rt_kprintf("[batgauge] seeded=%d soc=%d.%02d%% charging=%d\n",
               s_seeded, s_soc_centi / 100, s_soc_centi % 100, s_last_charging);
    rt_kprintf("[batgauge] last v=%d mV -> ocv=%d mV -> curve=%d%%\n",
               s_last_voltage_01mv / 10, s_last_ocv_01mv / 10, s_last_ocv_soc);
    rt_kprintf("[batgauge] charge_full_minutes=%d chg_ir=%d mV dis_ir=%d mV\n",
               s_charge_minutes, GAUGE_CHARGE_IR_01MV / 10,
               GAUGE_DISCHARGE_IR_01MV / 10);
    rt_kprintf("[batgauge] rates: chg_correct=%d%%/h dis_down=%d%%/h dis_up=%d%%/h\n",
               GAUGE_CHARGE_CORRECT_PCT_PER_HOUR,
               GAUGE_DISCHARGE_DOWN_PCT_PER_HOUR,
               GAUGE_DISCHARGE_UP_PCT_PER_HOUR);
}
