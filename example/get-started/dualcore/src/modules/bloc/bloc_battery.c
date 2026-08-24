/**
 ******************************************************************************
 * @file   bloc_battery.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include "watch_sys_service.h"
#include "bloc_battery.h"
#include "bloc_peripheral.h"
#include "battery_calculator.h"
#include "battery_gauge.h"
#include "charge.h"

/* Gauge selection. 1 = state-based gauge (battery_gauge.c): SOC is a state
 * variable advanced by elapsed time, one OCV curve for both directions.
 * 0 = legacy battery_calculator.c: SOC is a lookup of the instantaneous
 * voltage against two disagreeing curves. Both implementations, and both
 * curve tables, are kept in the tree -- flip this one line to go back. */
#ifndef BATTERY_USE_STATE_GAUGE
    #define BATTERY_USE_STATE_GAUGE 1
#endif

#define DBG_TAG "bloc.battery"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include "rtdbg.h"

#ifdef BSP_USING_ADC
    #define ADC_DEV_NAME "bat1"
    #define ADC_DEV_CHANNEL 5

    #ifdef USING_BATTERY_ADC_LOW_ACCURACY
        #define BATTERY_VOLTAGE_RATIO_NUM 100
        #define BATTERY_VOLTAGE_RATIO_DEN 100
    #endif

    #ifdef USING_BATTERY_ADC_HIGH_ACCURACY
        // R1=470K, R2=1M, ratio = (R1+R2)/R2 = 1.47
        #define BATTERY_VOLTAGE_RATIO_NUM 147
        #define BATTERY_VOLTAGE_RATIO_DEN 100
    #endif

/* ADC glitch guard. The GPADC occasionally returns an implausible reading
 * (field reports: ~1.9 V while the cell is actually ~3.7 V — about half).
 * The root cause is an intermittent analog-side fault we cannot reproduce on
 * demand, so guard at the reported value with two rules:
 *   1. reject anything outside the cell's physical operating window, and
 *   2. reject any sudden drop of >0.5 V from the last good value — a real cell
 *      cannot fall that fast between samples, so such a step is a glitch (or a
 *      transient load sag, which an OCV-based gauge should ignore anyway).
 * On either condition we FREEZE: keep reporting the last good value and do not
 * advance the baseline, until a reading recovers (back in range AND within the
 * drop threshold). There is deliberately NO consecutive-read tolerance — a
 * sustained glitch must never leak through, because the discharge curve is
 * monotonic-down and a single bad sample latches the displayed % low until a
 * reboot/charge (the "shows 4% while the cell is 3.9 V" field bug). Trade-off,
 * accepted by product: a permanently failed ADC freezes the voltage at the
 * last good value until reboot; we log loudly on freeze and recovery so the
 * condition is never silent. */
#define BATTERY_VOLTAGE_MIN_PLAUSIBLE_01MV  30000   /* 3.0 V (0.1 mV units) */
#define BATTERY_VOLTAGE_MAX_PLAUSIBLE_01MV  45000   /* 4.5 V (0.1 mV units) */
#define BATTERY_VOLTAGE_GLITCH_DROP_01MV     5000   /* 0.5 V sudden drop = glitch */

static battery_calculator_t s_calculator;
static bool s_calculator_initialized = false;

static uint32_t read_battery_voltage(void)
{
    rt_device_t adc_dev = rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        LOG_E("Can't find adc device %s", ADC_DEV_NAME);
        return 0;
    }

    rt_err_t r = rt_adc_enable((rt_adc_device_t)adc_dev, ADC_DEV_CHANNEL);
    if (r != RT_EOK)
    {
        LOG_E("Enable adc channel %d fail", ADC_DEV_CHANNEL);
        return 0;
    }

    uint32_t value = rt_adc_read((rt_adc_device_t)adc_dev, ADC_DEV_CHANNEL);
    rt_adc_disable((rt_adc_device_t)adc_dev, ADC_DEV_CHANNEL);

    if (value == 0)
    {
        LOG_E("ADC read returned 0");
        return 0;
    }

    // value is in 0.1mV, apply voltage divider ratio using integer math
    uint32_t battery_voltage = (value * BATTERY_VOLTAGE_RATIO_NUM) / BATTERY_VOLTAGE_RATIO_DEN;

    LOG_D("ADC raw: %d (0.1mV), battery: %d (0.1mV)", value, battery_voltage);

    /* Reject glitch readings (see note above the thresholds). */
    static uint32_t s_last_valid_voltage = 0;   /* 0.1 mV; 0 = no valid sample yet */
    static bool     s_frozen = false;

    bool out_of_range = (battery_voltage < BATTERY_VOLTAGE_MIN_PLAUSIBLE_01MV) ||
                        (battery_voltage > BATTERY_VOLTAGE_MAX_PLAUSIBLE_01MV);

    if (s_last_valid_voltage == 0)
    {
        /* No baseline yet: only a physically plausible sample may seed it, so a
         * power-on glitch can't anchor the drop comparison to a bad value. */
        if (out_of_range)
        {
            LOG_W("First battery voltage %d (0.1mV), raw %d out of range; not seeding baseline",
                  battery_voltage, value);
            return 0;   /* caller treats 0 as "no sample" and skips */
        }
        s_last_valid_voltage = battery_voltage;
        return battery_voltage;
    }

    bool sudden_drop = (battery_voltage + BATTERY_VOLTAGE_GLITCH_DROP_01MV) < s_last_valid_voltage;

    if (out_of_range || sudden_drop)
    {
        if (!s_frozen)
        {
            s_frozen = true;
            LOG_W("Battery voltage glitch %d->%d (0.1mV, raw %d); freezing at last good until recovery",
                  s_last_valid_voltage, battery_voltage, value);
        }
        return s_last_valid_voltage;   /* hold last good; do not advance baseline */
    }

    if (s_frozen)
    {
        s_frozen = false;
        LOG_W("Battery voltage recovered to %d (0.1mV); resuming updates", battery_voltage);
    }
    s_last_valid_voltage = battery_voltage;

    return battery_voltage;
}

static battery_calculator_config_t s_calculator_config;

void bloc_battery_init(void)
{
    extern const battery_lookup_point_t discharge_curve_table[];
    extern const battery_lookup_point_t charging_curve_table[];
    extern const uint32_t discharge_curve_table_size;
    extern const uint32_t charging_curve_table_size;

    s_calculator_config.charging_table = charging_curve_table;
    s_calculator_config.charging_table_size = charging_curve_table_size;
    s_calculator_config.discharging_table = discharge_curve_table;
    s_calculator_config.discharging_table_size = discharge_curve_table_size;
    s_calculator_config.charge_filter_threshold = 200;      // 20mV in 0.1mV units
    s_calculator_config.discharge_filter_threshold = 200;   // 20mV in 0.1mV units
    s_calculator_config.filter_count = 3;
    s_calculator_config.secondary_filter_enabled = true;
    s_calculator_config.secondary_filter_weight_pre = 80;
    s_calculator_config.secondary_filter_weight_cur = 20;

    int ret = battery_calculator_init(&s_calculator, &s_calculator_config);
    if (ret != BATTERY_CALC_SUCCESS)
    {
        LOG_E("Battery calculator init failed: %d", ret);
        return;
    }
    s_calculator_initialized = true;
    LOG_I("Battery calculator initialized");

#if BATTERY_USE_STATE_GAUGE
    /* The state gauge uses the discharge curve as its OCV table in BOTH
     * directions, correcting the terminal voltage to open-circuit before the
     * lookup. charging_curve_table stays installed on the legacy calculator
     * above so switching back needs no other change. */
    battery_gauge_set_curve(discharge_curve_table, discharge_curve_table_size);
    LOG_I("Battery state gauge active (OCV curve: %d points)",
          discharge_curve_table_size);
#endif
}

#endif // BSP_USING_ADC

/// ************** Charging Detect ************** ///
extern void main_send_read_charge_status_event(void);
extern void main_send_read_voltage_event(void);
extern void main_send_read_voltage_poll_event(void);

/* During charging, voltage rises slowly (0..100% over 1-2 hours = <0.05%
 * per second). The EMA filter (80/20) + 20 mV hysteresis in
 * battery_calculator already absorb high-frequency noise, so polling every
 * second is wasteful -- 10 s is invisible to the user but cuts ADC enable
 * / I2C charger reads ~10x. The plug-in event still triggers an immediate
 * read (read_charge_status -> main_send_read_voltage_event) so the UI
 * shows the current % the instant the cable is connected. */
#define CHARGING_VOLTAGE_READ_INTERVAL_MS 10000

static rt_timer_t s_charging_voltage_timer = RT_NULL;

static void charging_voltage_timer_callback(void *parameter)
{
    /* Re-poll charge status as a safety net: the unplug IRQ can be dropped
     * by the 150-tick debounce in aw32001_input_handle when the connector
     * bounces, leaving the UI stuck on "charging" forever. Polling here
     * means staleness is bounded to one timer interval. */
    main_send_read_charge_status_event();
    main_send_read_voltage_event();
}

static void charging_voltage_timer_start(void)
{
    if (s_charging_voltage_timer == RT_NULL)
    {
        s_charging_voltage_timer = rt_timer_create(
            "chg_vol",
            charging_voltage_timer_callback,
            RT_NULL,
            rt_tick_from_millisecond(CHARGING_VOLTAGE_READ_INTERVAL_MS),
            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    }
    if (s_charging_voltage_timer)
    {
        rt_timer_start(s_charging_voltage_timer);
        LOG_D("Charging voltage timer started (interval: %dms)", CHARGING_VOLTAGE_READ_INTERVAL_MS);
    }
}

static void charging_voltage_timer_stop(void)
{
    if (s_charging_voltage_timer)
    {
        rt_timer_stop(s_charging_voltage_timer);
        LOG_D("Charging voltage timer stopped");
    }
}

void bloc_battery_read_charge_status(void)
{
    main_send_read_charge_status_event();
}

void bloc_battery_read_voltage(void)
{
    main_send_read_voltage_event();
}

/* On wake-from-sleep the GPADC analog front-end (LDO / reference) has not
 * settled yet; sampling in the first instant occasionally yields an
 * implausibly low reading that then gets cached and "sticks" (the phone shows
 * it, unchanged on refresh) until the next sample — which, while discharging,
 * may not come for a long time. Defer the wake-up sample by a short settle
 * delay so the first post-wake reading is trustworthy. read_battery_voltage's
 * range guard still backstops any glitch that slips through. */
#define WAKEUP_VOLTAGE_SETTLE_MS 300

static rt_timer_t s_wakeup_voltage_timer = RT_NULL;

static void wakeup_voltage_timer_callback(void *parameter)
{
    main_send_read_voltage_event();
}

void bloc_battery_read_voltage_after_settle(void)
{
    if (s_wakeup_voltage_timer == RT_NULL)
    {
        s_wakeup_voltage_timer = rt_timer_create(
            "bat_wake",
            wakeup_voltage_timer_callback,
            RT_NULL,
            rt_tick_from_millisecond(WAKEUP_VOLTAGE_SETTLE_MS),
            RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    }
    if (s_wakeup_voltage_timer)
    {
        rt_timer_start(s_wakeup_voltage_timer);
    }
}

BatteryChargeState battery_charge_state = {0};

BatteryChargeState *battery_get_charge_state(void)
{
    return &battery_charge_state;
}

/* Debounce the charge IC's own opinion before anyone downstream sees it.
 *
 * AW32001's SYS_STATUS charging bits are not a clean "is the cable in" signal.
 * When the die heats up the IC folds back and suspends charging, reporting
 * NO_CHARGING while the cable is still plugged in, then resumes once it cools
 * -- and every one of those blips used to travel the whole chain: the charging
 * screen pops (gui_app_run(APP_ID_BATTERY)) and is dismissed again seconds
 * later, over and over, for the whole charge.
 *
 * So a new value from the IC must HOLD for CHARGE_STATE_DEBOUNCE_MS before we
 * publish it. While a candidate is being timed we publish nothing and arm a
 * one-shot timer to re-read (reads are otherwise event-driven at 10 s while
 * charging / 5 min otherwise, which is far too coarse to measure a 1 s hold).
 * A candidate that flips back to the currently published value inside the
 * window simply never becomes a transition -- which is exactly the flicker we
 * are killing.
 *
 * Note this deliberately does NOT gate the *republishing* of an unchanged
 * value: the resync-after-LCPU-restart behaviour described below depends on
 * every read reporting. Only genuine transitions are held. */
#define CHARGE_STATE_DEBOUNCE_MS 1000

static bool      s_charge_state_valid = false;  /* has anything been published yet */
static bool      s_charge_published   = false;  /* last value the rest of the system saw */
static bool      s_charge_candidate   = false;  /* raw IC value currently being timed */
static rt_tick_t s_charge_candidate_tick = 0;   /* when the candidate first appeared */
static rt_timer_t s_charge_debounce_timer = RT_NULL;

static void charge_debounce_timer_callback(void *parameter)
{
    main_send_read_charge_status_event();
}

/* Re-read once the hold window is up. The small margin keeps the re-read on
 * the far side of the window so it settles in one extra pass, not two. */
static void charge_debounce_arm(void)
{
    if (s_charge_debounce_timer == RT_NULL)
    {
        s_charge_debounce_timer = rt_timer_create(
            "chg_deb",
            charge_debounce_timer_callback,
            RT_NULL,
            rt_tick_from_millisecond(CHARGE_STATE_DEBOUNCE_MS / 4 + 20),
            RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    }
    if (s_charge_debounce_timer)
    {
        rt_timer_stop(s_charge_debounce_timer);
        rt_timer_start(s_charge_debounce_timer);
    }
}

static void charge_debounce_disarm(void)
{
    if (s_charge_debounce_timer)
        rt_timer_stop(s_charge_debounce_timer);
}

static void read_charge_status(void)
{
    battery_charger_status_t status = battery_get_charging_status();
    bool charging = (status == BATTERY_CHARGER_STATUS_CHARGING);

    LOG_D("[%s] Is charging? %d", __func__, charging);

    if (!s_charge_state_valid)
    {
        /* First read since boot: there is no previous value to debounce
         * against, and holding it back would leave the UI blank. */
        s_charge_state_valid = true;
        s_charge_published   = charging;
        s_charge_candidate   = charging;
        s_charge_candidate_tick = rt_tick_get();
    }
    else if (charging != s_charge_candidate)
    {
        /* The IC just moved. Start timing the new value. */
        s_charge_candidate = charging;
        s_charge_candidate_tick = rt_tick_get();

        if (charging != s_charge_published)
        {
            LOG_I("charge status %d, holding before publish", (int)charging);
            charge_debounce_arm();
            return;
        }
        /* It bounced straight back to what is already published -- the
         * transition never happened as far as the UI is concerned. */
        LOG_I("charge status bounced back to %d", (int)charging);
    }
    else if (charging != s_charge_published)
    {
        /* Same value as the previous read; has it held long enough? */
        rt_tick_t held = (rt_tick_t)(rt_tick_get() - s_charge_candidate_tick);
        if (held < rt_tick_from_millisecond(CHARGE_STATE_DEBOUNCE_MS))
        {
            charge_debounce_arm();
            return;
        }
        LOG_I("charge status settled at %d", (int)charging);
        s_charge_published = charging;
    }

    charge_debounce_disarm();

    /* Report on every read, not just on a local transition. Gating here on our
     * own last-seen value made the two cores unrecoverable once they diverged:
     * an LCPU-only restart clears is_charging while the HCPU keeps rendering
     * "charging", and the next read then matches our (reset) state, so no edge
     * is ever produced to correct the display. Dedup now lives on the HCPU
     * side, keyed on the value the UI actually renders. */
    if (watch_sys_sync.charge_status_callback)
        watch_sys_sync.charge_status_callback(
            charging ? (battery_charge_state.charge_percent == 100 ? 2 : 1)
                     : 0);
    battery_charge_state.is_charging = charging;
    battery_charge_state.is_plugged = charging;

    if (charging)
    {
        // read voltage immediately and start periodic 1s timer
        main_send_read_voltage_event();
        charging_voltage_timer_start();
    }
    else
    {
        charging_voltage_timer_stop();
    }
}

#ifdef BSP_USING_ADC
/* Set once the gauge has been computed at least once from a real sample, so the
 * voltage-only refresh has a valid percentage to reuse instead of a stale 0%. */
static bool s_percent_valid = false;

/* Report (voltage, percentage) up to the HCPU. The phone shows the raw mV; the
 * watch UI shows the percentage. */
static void report_battery(uint32_t battery_voltage, uint8_t percentage)
{
    uint32_t battery_mv = battery_voltage / 10;   // 0.1mV -> mV
    uint32_t data = (battery_mv << 16) | (percentage & 0xFFFF);
    battery_charge_state.charge_percent = percentage;
    if (watch_sys_sync.notify_battery_voltage)
        watch_sys_sync.notify_battery_voltage(data);
}
#endif

static void check_battery_voltage(void)
{
#ifdef BSP_USING_ADC
    if (!s_calculator_initialized)
    {
        LOG_E("Battery calculator not initialized");
        return;
    }

    uint32_t battery_voltage = read_battery_voltage();
    if (battery_voltage == 0)
        return;

#if BATTERY_USE_STATE_GAUGE
    /* Use the debounced state, not a fresh raw read: the gauge switches its
     * whole model between charge and discharge, so feeding it the IC's
     * thermal-throttle blips would make it flip direction every few seconds.
     * Before the first charge read has landed there is nothing debounced yet,
     * so fall back to the raw value. */
    bool charging =
        s_charge_state_valid
            ? s_charge_published
            : (battery_get_charging_status() == BATTERY_CHARGER_STATUS_CHARGING);
    uint8_t full = 0;
    /* Only trust a successful read; an I2C hiccup must not be read as
     * "termination reported", which would snap the gauge to 100%. */
    if (rt_charge_get_full_status(&full) != RT_CHARGE_EOK)
        full = 0;

    uint8_t percentage =
        battery_gauge_update(battery_voltage, charging, full != 0);
#else
    uint8_t percentage = battery_calculator_get_percent(&s_calculator, battery_voltage);
#endif
    s_percent_valid = true;

    LOG_D("[%s] %d mV, display=%d%%, charging=%d",
          __func__, battery_voltage / 10, percentage,
          battery_charge_state.is_charging);

    report_battery(battery_voltage, percentage);
#endif
}

/* Periodic-poll refresh. With BATTERY_USE_STATE_GAUGE this is a full gauge
 * update (see the note inside); the description below applies to the legacy
 * calculator path only.
 *
 * Legacy: voltage-only refresh for the periodic discharge poll. It keeps the
 * reported mV fresh on the phone (so it never freezes) but deliberately does
 * NOT move the percentage gauge. The discharge curve is OCV-based, so a sample taken
 * while the watch is under load (AI / BLE / haptics) reads low and -- via the
 * "discharge can only decrease" rule in battery_calculator -- would latch the
 * gauge down with no way back up until reboot. Recomputing the gauge every poll
 * is exactly what made the displayed % drift down on its own; instead the gauge
 * is recomputed only on the trustworthy triggers (wake-after-settle / app
 * request / charging), as it was before the poll existed. */
static void refresh_battery_voltage_only(void)
{
#ifdef BSP_USING_ADC
#if BATTERY_USE_STATE_GAUGE
    /* The reason this function skipped the gauge no longer applies. The state
     * gauge is rate-limited per unit of TIME and rejects load sag by design, so
     * a poll taken while the watch is busy can no longer latch the percentage
     * down -- and the gauge actively needs these regular ticks, since its whole
     * model is "SOC advances with elapsed time". Feeding it the slow poll is
     * what makes the discharge rate independent of how often the user happens
     * to wake the watch. */
    check_battery_voltage();
    return;
#else
    /* Bootstrap: if the watch sat idle past the first poll before any wake-up,
     * compute the gauge once here so we never report a stale 0%. */
    if (!s_percent_valid)
    {
        check_battery_voltage();
        return;
    }

    uint32_t battery_voltage = read_battery_voltage();
    if (battery_voltage == 0)
        return;

    LOG_D("[%s] %d mV, keep display=%d%% (voltage-only)",
          __func__, battery_voltage / 10, battery_charge_state.charge_percent);

    report_battery(battery_voltage, battery_charge_state.charge_percent);
#endif /* BATTERY_USE_STATE_GAUGE */
#endif /* BSP_USING_ADC */
}

void bloc_battery_handle_charging_event(void)
{
    read_charge_status();
}

void bloc_battery_handle_voltage_event(void)
{
    check_battery_voltage();
}

void bloc_battery_handle_voltage_poll_event(void)
{
    refresh_battery_voltage_only();
}

/* Periodic battery sampling so the reported voltage keeps refreshing while
 * discharging. Without this, discharging only samples on wake-up or an
 * explicit app request, so a stale (or once-glitched) value can sit frozen on
 * the phone for a long time. Charging keeps its faster 10 s timer; this slow
 * timer is a floor that also covers the discharge case. SOFT_TIMER: it fires
 * while the screen is off at this multi-minute cadence (same mechanism the
 * background HR-curve sampler relies on, hr_service.c) and only pauses in the
 * very deepest CPU-clock-stopped sleep — a state in which BLE is typically
 * down and the phone would not receive an update anyway. Started at init so it
 * does not depend on read_charge_status ever running (that only fires on a
 * charge-state *change*). */
#define BATTERY_VOLTAGE_POLL_INTERVAL_MS 300000   /* 5 min */

static void battery_voltage_poll_callback(void *parameter)
{
    /* Voltage-only: refresh the reported mV without moving the percentage gauge
     * (see refresh_battery_voltage_only for why). */
    main_send_read_voltage_poll_event();
    /* Re-check the charge status as well. While discharging nothing else ever
     * re-reads it -- the 10 s safety-net timer only runs *while charging* --
     * so a missed unplug edge stays wrong indefinitely. This bounds it to one
     * poll interval even if the watch is never woken. */
    main_send_read_charge_status_event();
}

static int bloc_battery_voltage_poll_init(void)
{
    rt_timer_t timer = rt_timer_create(
        "bat_poll",
        battery_voltage_poll_callback,
        RT_NULL,
        rt_tick_from_millisecond(BATTERY_VOLTAGE_POLL_INTERVAL_MS),
        RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (timer)
        rt_timer_start(timer);
    return 0;
}
INIT_APP_EXPORT(bloc_battery_voltage_poll_init);

/* Seed the gauge with a SOC the HCPU persisted across a reset. Only matters
 * when we boot while already on the charger: with the cable out, the first
 * reading is a rested open-circuit voltage and the gauge anchors itself
 * accurately without help. Arriving late is harmless -- battery_gauge_seed
 * refuses to move a gauge that is already tracking. */
void bloc_battery_seed_soc(uint8_t percent)
{
#if BATTERY_USE_STATE_GAUGE
    if (battery_gauge_seed(percent))
    {
        /* Publish immediately so the UI shows the restored value rather than
         * sitting on 0% until the next sample. */
        battery_charge_state.charge_percent = percent;
    }
#else
    (void)percent;
#endif
}

#if defined(RT_USING_FINSH) && BATTERY_USE_STATE_GAUGE
static int batgauge(int argc, char **argv)
{
    if (argc >= 3 && rt_strcmp(argv[1], "minutes") == 0)
    {
        battery_gauge_set_charge_minutes((uint32_t)atoi(argv[2]));
    }
    else if (argc >= 3 && rt_strcmp(argv[1], "seed") == 0)
    {
        battery_gauge_reset();
        bloc_battery_seed_soc((uint8_t)atoi(argv[2]));
    }
    else if (argc >= 2 && rt_strcmp(argv[1], "sample") == 0)
    {
        check_battery_voltage();
    }
    else if (argc >= 2 && rt_strcmp(argv[1], "help") == 0)
    {
        rt_kprintf("batgauge                 - dump gauge state\n");
        rt_kprintf("batgauge sample          - take a reading now\n");
        rt_kprintf("batgauge minutes <n>     - set 0->100%% charge time\n");
        rt_kprintf("batgauge seed <pct>      - reset and re-anchor at pct\n");
        return 0;
    }

    battery_gauge_dump();
    rt_kprintf("[batgauge] charging=%d plugged=%d reported=%d%%\n",
               battery_charge_state.is_charging,
               battery_charge_state.is_plugged,
               battery_charge_state.charge_percent);
    return 0;
}
MSH_CMD_EXPORT(batgauge, battery gauge state and calibration);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF *
 * FILE****/
