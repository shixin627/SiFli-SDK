/**
 ******************************************************************************
 * @file   wear_detect.c
 * @author Skaiwalk software development team
 * @brief  Wear detection algorithm combining IMU and PPG sensor signals.
 *
 *         Detection principle (continuous, multi-indicator fusion):
 *
 *         Four indicators are evaluated every EVAL_PERIOD_MS:
 *
 *         1. PPG freshness: if PPG sensor is off (no new samples),
 *            fall back to IMU-only re-trigger to wake PPG up.
 *
 *         2. PPG DC Level (contact detection):
 *            - DC < threshold → nothing touching → NOT wearing.
 *
 *         3. Perfusion Index (PI = AC / DC) + variability:
 *            - PI > threshold AND varying → heartbeat → WEARING.
 *            - PI > threshold BUT constant → noise → NOT wearing.
 *
 *         4. IMU Variance (supplementary):
 *            - Used to disambiguate when PI is low but DC is high.
 *            - Also used as sole re-trigger when PPG data is stale.
 *
 *         State transitions use hysteresis counters to prevent oscillation.
 ******************************************************************************
 */
/* Temporary PPG stall diagnostic instrumentation. Remove once done. */
#ifndef PPG_RACE_DEBUG
    #define PPG_RACE_DEBUG 1
#endif

#include <rtthread.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "bsp_board.h"
#include "wear_detect.h"
#include "bloc_peripheral.h"
#include "bloc_battery.h"
#include "watch_sys_service.h"

#define DBG_TAG "wear_detect"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

/* -------------------- Configuration -------------------- */

/* Evaluation period in milliseconds (continuous, not event-driven) */
#define EVAL_PERIOD_MS          1500

/* PPG DC threshold: below this → nothing touching the sensor (suspended in air).
 * REAL measured data (this sensor, HRV probe): air ~40k, WRIST ~43-44k (skin
 * absorbs light, so a worn wrist reads LOWER than a reflective table ~50k!).
 * The old 48000 assumption was wrong and rejected a worn wrist (DC 44k<45k).
 * Threshold now sits just under the wrist value; a table (high DC) is rejected
 * later by the pulse check, not here. ⚠ narrow margin -- needs per-unit calib. */
#define PPG_DC_LOW_THD          42000

/* Perfusion Index threshold (AC_pp / DC_mean).
 * Asymmetric (Schmitt trigger) to prevent oscillation on static surfaces:
 *   - PI_THD_TO_ON:   stricter bar for OFF→ON  (real heartbeat is clearly > this).
 *   - PI_THD_KEEP_ON: looser bar to maintain ON state.
 * Measured data: wearing PI spikes > 0.001, table PI occasionally spikes to
 * ~0.0005-0.0008 from noise. */
#define PI_THD_TO_ON            0.0010f
#define PI_THD_KEEP_ON          0.0004f

/* IMU variance threshold (m/s^2)^2 for supplementary motion check */
#define IMU_VARIANCE_THD        0.03f

/* Hysteresis: votes needed to change state.
 * OFF→ON is WINDOWED, not consecutive: HYSTERESIS_ON votes within the last
 * 8 evaluations (~12 s). Measured 2026-06 (worn wrist + charging cable):
 * PI hovers 0.0005-0.0014, crossing the 0.0010 bar on only ~30% of evals,
 * so requiring 3 *consecutive* crossings left the watch stuck OFF for 20+
 * minutes. A table never crosses the bar at all (noise tops ~0.0008), so
 * windowed density keeps the same false-positive rejection.
 * ON→OFF stays consecutive: DC collapse votes are deterministic. */
#define HYSTERESIS_ON           3   /* OFF→ON:  3 ON votes in last 8 evals */
#define HYSTERESIS_OFF          3   /* ON→OFF:  3 consecutive OFF (~4.5s) */

/* PI variability check: real heartbeat causes PI to fluctuate across
 * evaluations.  Constant PI (noise/static surface) should be rejected.
 * Track the last PI_HISTORY_LEN evaluations and require the range
 * (max - min) to exceed PI_RANGE_THD before accepting PI as heartbeat. */
#define PI_HISTORY_LEN          5
#define PI_RANGE_THD            0.0003f

/* Upper bound on PI variability for OFF->ON. A real heartbeat's PI range is
 * modest; a motion artefact (e.g. setting the watch down on a table) makes PI
 * swing wildly and can fake a pulse. Reject OFF->ON when range is implausibly
 * large. Measured: worn ~0.03-0.05, table set-down artefact ~0.27. */
#define PI_RANGE_MAX            0.15f

/* PPG freshness: if no new PPG sample arrives within this many
 * milliseconds, consider PPG data stale (sensor likely powered off). */
#define PPG_STALE_MS            5000

/* PPG settle time: after PPG sensor restarts, the first few seconds
 * produce wildly inaccurate readings (huge PI spikes).  Ignore PPG
 * data during this settle period. */
#define PPG_SETTLE_MS           6000

/* When PPG is stale and device is OFF-wrist, use IMU motion to
 * re-trigger ON (which causes system to restart PPG sensor).
 * Require N consecutive IMU-motion evaluations before voting ON. */
#define IMU_RETRIGGER_COUNT     2
#define IMU_RETRIGGER_THD       0.05f

/* [DC-led] After motion while off-wrist, hold PPG powered for this long so the
 * pulse can confirm a real wrist even if the user then stays still. If no wrist
 * is confirmed within the window, PPG is powered back down until next motion. */
#define PROBE_WINDOW_MS         (3 * 60 * 1000)

/* Contact-break re-confirmation: taking the watch off reads as a DC collapse
 * WITH motion, but if it lands on a table within the OFF hysteresis (~4.5s)
 * the table's DC can match skin (measured 2026-06-10: table 46.9k vs wrist
 * 46.2k) and DC-held would latch ON forever. After such a break, suspend
 * DC-held and demand a live pulse (same windowed gate as OFF→ON); no pulse
 * after this many *usable* evaluations → vote OFF.
 *
 * The countdown ticks ONLY on evaluations where a pulse could actually be
 * seen: PPG fresh AND wrist still. It freezes while PPG is duty-cycled off
 * by bg_hr (sleep: 60s on / 120s off; awake: ~40s per ~15min — wall-clock
 * grace would expire inside a gap and force OFF even on a perfect wrist)
 * and while moving (motion artefacts block the pulse gate; a moving surface
 * is not a table). 60 clean looks with no pulse = not a wrist.
 * Sleep-time DC dips without motion never arm this path, so the
 * motionless-sleep DC-held guarantee is preserved. */
#define RECONFIRM_LIVE_EVALS    60

/* PPG ring buffer size: at ~25 Hz, 75 samples = 3 seconds */
#define PPG_WINDOW_SIZE         75

/* IMU ring buffer size: at ~25 Hz, 75 samples = 3 seconds */
#define IMU_WINDOW_SIZE         75

/* -------------------- Internal state -------------------- */

typedef struct
{
    /* PPG ring buffer for PI calculation */
    uint32_t ppg_buf[PPG_WINDOW_SIZE];
    uint16_t ppg_idx;
    uint16_t ppg_count;

    /* IMU ring buffer (acceleration magnitude) */
    float acce_mag[IMU_WINDOW_SIZE];
    uint16_t imu_idx;
    uint16_t imu_count;

    /* PI history for variability check */
    float pi_history[PI_HISTORY_LEN];
    uint8_t pi_hist_idx;
    uint8_t pi_hist_count;

    /* PPG freshness tracking */
    uint32_t last_ppg_ms;       /* timestamp of last PPG sample */
    bool ppg_ever_received;     /* true after first PPG sample */

    /* PPG settle: timestamp when PPG resumed after a stale gap */
    uint32_t ppg_restart_ms;
    bool ppg_settling;          /* true during settle period after restart */

    /* IMU re-trigger counter (for stale PPG + OFF state) */
    uint8_t imu_retrigger_cnt;

    /* Timing for periodic evaluation */
    uint32_t last_eval_ms;

    /* Hysteresis state */
    uint8_t on_vote_window; /* bitmask of last 8 eval votes, bit set = ON vote */
    int8_t off_counter;     /* counts consecutive OFF evaluations */

    /* Contact-break re-confirmation (take-off detection) */
    bool contact_break_pending;     /* DC collapsed while moving; DC-held suspended */
    uint8_t reconfirm_evals_left;   /* usable (fresh-PPG, still) evals left to re-confirm */

    /* Diagnostic uplink: last computed metrics (events fire outside the
       eval scope) + sample-rate limiter */
    float last_dc;
    float last_pi;
    float last_pi_range;
    float last_imu_var;
    uint32_t last_diag_sample_ms;

    /* Current output */
    wear_status_t status;
    bool initialized;
} wear_detect_ctx_t;

static wear_detect_ctx_t ctx;

/* [DC-led] Motion-triggered PPG probe window. PPG power lives in hr_service;
 * wear_detect asks it to power up (probe) and back down when nothing is worn. */
extern void hr_set_power(uint8_t arg);
static bool s_probe_active = false;
static uint32_t s_probe_until_ms = 0;

/* Diagnostic override (settings toggle "佩戴偵測"): when false, the contact
 * algorithm is bypassed and the watch is forced WORN unless on the charger. */
static bool s_detect_enabled = true;

/* -------------------- Helpers -------------------- */

static uint8_t on_votes_in_window(uint8_t w)
{
    uint8_t n = 0;
    while (w)
    {
        n += w & 1u;
        w >>= 1;
    }
    return n;
}

static float compute_imu_variance(const float *buf, uint16_t len)
{
    float sum = 0.0f;
    float sum_sq = 0.0f;

    for (uint16_t i = 0; i < len; i++)
    {
        sum += buf[i];
        sum_sq += buf[i] * buf[i];
    }

    float mean = sum / (float)len;
    return (sum_sq / (float)len) - (mean * mean);
}

/**
 * @brief Compute PPG DC mean, AC peak-to-peak, and Perfusion Index
 */
static void compute_ppg_metrics(float *dc_mean, float *ac_pp, float *pi)
{
    uint16_t len = (ctx.ppg_count < PPG_WINDOW_SIZE)
                       ? ctx.ppg_count : PPG_WINDOW_SIZE;

    if (len == 0)
    {
        *dc_mean = 0.0f;
        *ac_pp = 0.0f;
        *pi = 0.0f;
        return;
    }

    uint32_t ppg_min = UINT32_MAX;
    uint32_t ppg_max = 0;
    uint64_t ppg_sum = 0;

    for (uint16_t i = 0; i < len; i++)
    {
        uint32_t v = ctx.ppg_buf[i];
        ppg_sum += v;
        if (v < ppg_min) ppg_min = v;
        if (v > ppg_max) ppg_max = v;
    }

    *dc_mean = (float)ppg_sum / (float)len;
    *ac_pp = (float)(ppg_max - ppg_min);

    if (*dc_mean > 1.0f)
        *pi = *ac_pp / *dc_mean;
    else
        *pi = 0.0f;
}

static bool is_ppg_stale(uint32_t now)
{
    if (!ctx.ppg_ever_received)
        return true;
    return (now - ctx.last_ppg_ms) > PPG_STALE_MS;
}

static void notify_wear_status(bool wearing)
{
    if (watch_sys_sync.soft_adt_status_callback)
    {
        watch_sys_sync.soft_adt_status_callback(wearing);
    }
}

/* -------------------- Diagnostic uplink --------------------
 * Cable-less units have no serial console; these records (forwarded to the
 * phone as a daily CSV) are the only window into nightly wear-detect
 * internals for per-unit threshold analysis. ~1 record/min + transitions:
 * negligible BLE / cross-core load. */

static uint16_t diag_clamp_u16(float v)
{
    if (v < 0.0f) return 0;
    if (v > 65535.0f) return 65535;
    return (uint16_t)v;
}

static void diag_emit(uint8_t evt, float dc, float pi, float pi_range,
                      float imu_var)
{
    if (!watch_sys_sync.notify_wear_diag)
        return;

    watch_sys_wear_diag_t rec;
    rec.ts = (uint32_t)time(RT_NULL);
    rec.evt = evt;
    rec.status = (ctx.status == WEAR_STATUS_WEARING) ? 1 : 0;
    rec.dc_q4 = diag_clamp_u16(dc * 0.25f);
    rec.pi_e6 = diag_clamp_u16(pi * 1e6f);
    rec.pi_range_e6 = diag_clamp_u16(pi_range * 1e6f);
    rec.imu_var_e4 = diag_clamp_u16(imu_var * 1e4f);
    watch_sys_sync.notify_wear_diag(&rec);
}

/* Emit with the last metrics computed by evaluate_once (for events that fire
 * where dc/pi are out of scope, e.g. state transitions in try_evaluate). */
static void diag_emit_last(uint8_t evt)
{
    diag_emit(evt, ctx.last_dc, ctx.last_pi, ctx.last_pi_range,
              ctx.last_imu_var);
}

static void set_status(wear_status_t new_status)
{
    if (ctx.status == new_status)
        return;

    ctx.status = new_status;
    ctx.contact_break_pending = false;

    if (new_status == WEAR_STATUS_WEARING)
    {
        s_probe_active = false; /* probe confirmed a wrist; hand PPG to hr_service/bg_hr */
        LOG_I("Wear detected: ON WRIST");
        notify_wear_status(true);
        diag_emit_last(WEAR_DIAG_EVT_ON);
    }
    else
    {
        LOG_I("Wear detected: OFF WRIST");
        notify_wear_status(false);
        diag_emit_last(WEAR_DIAG_EVT_OFF);
    }
}

/* -------------------- Core evaluation -------------------- */

/**
 * @brief Evaluate using only IMU when PPG data is stale.
 *        Used to re-trigger ON state so system restarts PPG sensor.
 * @return  1 = vote ON,  0 = no change
 */
static int evaluate_imu_only(void)
{
    uint16_t imu_len = (ctx.imu_count < IMU_WINDOW_SIZE)
                           ? ctx.imu_count : IMU_WINDOW_SIZE;
    if (imu_len == 0)
        return 0;

    float imu_var = compute_imu_variance(ctx.acce_mag, imu_len);

    if (imu_var >= IMU_RETRIGGER_THD)
    {
        /* Motion while off-wrist + PPG asleep: power PPG up and (re)start the
         * probe window, so the next few minutes of PPG can confirm a real wrist
         * even if the user then stays still. We never vote ON from IMU alone --
         * only a live PPG pulse (in evaluate_once) confirms wear. */
        if (!s_probe_active)
        {
            hr_set_power(1);
            /* Ignore the first few seconds after powering PPG up: the signal is
             * unstable and PI spikes during warm-up (this fakes a pulse on a
             * table). Arm the settle gate so evaluate_once drops those reads. */
            ctx.ppg_settling = true;
            ctx.ppg_restart_ms = rt_tick_get_millisecond();
            LOG_I("Wear: motion (var=%.4f) -> open PPG probe window (settling %ums)",
                  imu_var, PPG_SETTLE_MS);
            ctx.last_imu_var = imu_var;
            diag_emit_last(WEAR_DIAG_EVT_PROBE_OPEN);
        }
        s_probe_active = true;
        s_probe_until_ms = rt_tick_get_millisecond() + PROBE_WINDOW_MS;
    }
    else
    {
        LOG_D("Eval: off-wrist, IMU_var=%.5f (no motion)", imu_var);
    }

    return 0; /* PPG pulse confirms wear, not IMU */
}

/**
 * @brief Evaluate all indicators and determine wear/not-wear vote.
 * @return  1 = vote ON,  -1 = vote OFF,  0 = uncertain
 */
static int evaluate_once(uint32_t now)
{
#if (CUSTOMER_BOARD_VER == BOARD_VER_29)
    /* --- Board v29: charging always means OFF WRIST --- */
    if (battery_get_charge_state()->is_charging)
    {
        LOG_I("Eval: charging (board v29) -> force OFF");
        return -1;
    }
#endif

    /* --- Check PPG freshness first --- */
    if (is_ppg_stale(now))
    {
        /* Off-wrist + PPG asleep: motion opens a PPG probe window (below). If a
         * probe window already expired without confirming a wrist (e.g. PPG
         * never produced data), power PPG back down here too. */
        if (ctx.status == WEAR_STATUS_NOT_WEARING)
        {
            if (s_probe_active && now >= s_probe_until_ms)
            {
                hr_set_power(0);
                s_probe_active = false;
                ctx.on_vote_window = 0;
                LOG_I("Wear: probe expired (no PPG data) -> close PPG, wait for motion");
                diag_emit_last(WEAR_DIAG_EVT_PROBE_EXPIRE);
                return 0;
            }
            return evaluate_imu_only();
        }
        /* If currently ON but PPG went stale, don't change state yet */
        return 0;
    }

    /* Reset IMU retrigger counter since PPG is active */
    ctx.imu_retrigger_cnt = 0;

    /* --- Check PPG settle period after sensor restart --- */
    if (ctx.ppg_settling)
    {
        if (now - ctx.ppg_restart_ms < PPG_SETTLE_MS)
        {
            LOG_D("Eval: PPG settling (%u ms remaining)",
                  PPG_SETTLE_MS - (now - ctx.ppg_restart_ms));
            return 0; /* don't vote during settle */
        }
        ctx.ppg_settling = false;
        /* Clear PPG buffer and PI history to use only post-settle data */
        ctx.ppg_count = 0;
        ctx.ppg_idx = 0;
        ctx.pi_hist_count = 0;
        ctx.pi_hist_idx = 0;
        ctx.on_vote_window = 0;
        LOG_I("PPG settle complete, cleared buffers");
        return 0; /* wait for fresh data next eval */
    }

    float dc_mean, ac_pp, pi;
    compute_ppg_metrics(&dc_mean, &ac_pp, &pi);

    /* Record PI into history ring buffer */
    ctx.pi_history[ctx.pi_hist_idx] = pi;
    ctx.pi_hist_idx = (ctx.pi_hist_idx + 1) % PI_HISTORY_LEN;
    if (ctx.pi_hist_count < PI_HISTORY_LEN)
        ctx.pi_hist_count++;

    /* Compute PI variability (range = max - min over recent history) */
    float pi_range = 0.0f;
    if (ctx.pi_hist_count >= 2)
    {
        float pi_min = ctx.pi_history[0];
        float pi_max = ctx.pi_history[0];
        for (uint8_t i = 1; i < ctx.pi_hist_count; i++)
        {
            if (ctx.pi_history[i] < pi_min) pi_min = ctx.pi_history[i];
            if (ctx.pi_history[i] > pi_max) pi_max = ctx.pi_history[i];
        }
        pi_range = pi_max - pi_min;
    }

    /* Keep the latest metrics for out-of-scope diagnostic events, and emit a
     * rate-limited snapshot so the nightly CSV shows the analog levels even
     * when no event fires (the whole point for cable-less units). */
    ctx.last_dc = dc_mean;
    ctx.last_pi = pi;
    ctx.last_pi_range = pi_range;
    if (now - ctx.last_diag_sample_ms >= 60000)
    {
        ctx.last_diag_sample_ms = now;
        uint16_t imu_len = (ctx.imu_count < IMU_WINDOW_SIZE)
                               ? ctx.imu_count : IMU_WINDOW_SIZE;
        ctx.last_imu_var = (imu_len > 0)
                               ? compute_imu_variance(ctx.acce_mag, imu_len)
                               : 0.0f;
        diag_emit(WEAR_DIAG_EVT_SAMPLE, dc_mean, pi, pi_range,
                  ctx.last_imu_var);
    }

    /* === DC-led contact detection (industry standard: reflectance DC == skin
     * contact; off-body == DC collapse). The accelerometer is deliberately NOT
     * used to decide wear -- it only marks PPG reliability -- so a motionless
     * worn wrist during sleep stays ON instead of being mistaken for off-wrist
     * (root cause A2). PI variability is used ONLY to confirm OFF->ON, never to
     * drop an already-worn state (root cause A1). === */
    if (dc_mean < (float)PPG_DC_LOW_THD)
    {
        /* Worn + DC collapse + motion = take-off signature. Arm the
         * re-confirmation gate in case the watch lands on a skin-like DC
         * surface before the OFF hysteresis completes. A motionless dip
         * (loose strap during sleep) deliberately does NOT arm it. */
        if (ctx.status == WEAR_STATUS_WEARING && !ctx.contact_break_pending)
        {
            uint16_t imu_len = (ctx.imu_count < IMU_WINDOW_SIZE)
                                   ? ctx.imu_count : IMU_WINDOW_SIZE;
            float imu_var = (imu_len > 0)
                                ? compute_imu_variance(ctx.acce_mag, imu_len) : 0.0f;
            if (imu_var >= IMU_VARIANCE_THD)
            {
                /* counter first, flag last: a racing reader that sees the
                 * flag must also see a valid countdown */
                ctx.reconfirm_evals_left = RECONFIRM_LIVE_EVALS;
                ctx.contact_break_pending = true;
                LOG_I("Wear: contact break + motion (var=%.3f) -> require pulse re-confirm within %u live evals",
                      imu_var, RECONFIRM_LIVE_EVALS);
                ctx.last_imu_var = imu_var;
                diag_emit(WEAR_DIAG_EVT_BREAK_ARM, dc_mean, pi, pi_range,
                          imu_var);
            }
        }
        LOG_I("Eval: DC=%.0f (< %u) -> contact lost -> OFF", dc_mean, PPG_DC_LOW_THD);
        return -1;
    }

    /* DC is in the contact range. */
    if (ctx.status == WEAR_STATUS_WEARING)
    {
        if (!ctx.contact_break_pending)
        {
            /* Already worn: hold ON as long as DC stays high. Ignore PI/IMU so a
             * flat, low-variability sleep pulse never flips us OFF. Only the DC
             * collapse above takes us off-wrist. */
            LOG_I("Eval: DC=%.0f, PI=%.5f (worn; DC held) -> ON", dc_mean, pi);
            return 1;
        }

        /* Contact broke while moving: DC alone no longer proves a wrist (the
         * sensor may now rest on a table whose DC matches skin). Fall through
         * to the live-pulse gate; try_evaluate lifts the suspension once the
         * vote window re-confirms. Once the countdown of usable evals is
         * spent without a pulse, vote OFF. */
        if (ctx.reconfirm_evals_left == 0)
        {
            LOG_I("Eval: contact break unconfirmed (no pulse in %u live evals) -> OFF",
                  RECONFIRM_LIVE_EVALS);
            diag_emit(WEAR_DIAG_EVT_BREAK_TIMEOUT, dc_mean, pi, pi_range,
                      ctx.last_imu_var);
            return -1;
        }

        /* Tick the countdown only while still: motion artefacts block the
         * pulse gate anyway, and a moving surface is not a table. Stale
         * evals return earlier, so PPG-off gaps freeze the countdown.
         * (bg_hr burst restarts during WEARING skip the settle gate, so a
         * few warm-up evals do burn ticks — ~4-9 per restart, budgeted in
         * RECONFIRM_LIVE_EVALS.) */
        {
            uint16_t imu_len = (ctx.imu_count < IMU_WINDOW_SIZE)
                                   ? ctx.imu_count : IMU_WINDOW_SIZE;
            float imu_var = (imu_len > 0)
                                ? compute_imu_variance(ctx.acce_mag, imu_len) : 0.0f;
            if (imu_var < IMU_VARIANCE_THD)
                ctx.reconfirm_evals_left--;
        }
    }

    /* Currently OFF. The PPG running now was opened by a motion probe window;
     * if that window expired without a live pulse, power PPG back down and wait
     * for the next motion (saves LED current when nothing is worn). */
    if (s_probe_active && now >= s_probe_until_ms)
    {
        hr_set_power(0);
        s_probe_active = false;
        ctx.ppg_count = 0;
        ctx.ppg_idx = 0;
        ctx.on_vote_window = 0;
        LOG_I("Wear: probe window expired, no wrist -> close PPG, wait for motion");
        diag_emit_last(WEAR_DIAG_EVT_PROBE_EXPIRE);
        return 0;
    }

    /* Require a *live* pulse (PI above the ON bar AND varying) to confirm a real
     * wrist, so a watch resting on a table (high DC, no pulse) is not worn. */
    if (pi >= PI_THD_TO_ON && pi_range <= PI_RANGE_MAX &&
        (ctx.pi_hist_count < PI_HISTORY_LEN || pi_range >= PI_RANGE_THD))
    {
        LOG_I("Eval: DC=%.0f, AC_pp=%.0f, PI=%.5f, range=%.5f -> live wrist -> ON",
              dc_mean, ac_pp, pi, pi_range);
        return 1;
    }
    if (pi >= PI_THD_TO_ON && pi_range > PI_RANGE_MAX)
    {
        LOG_I("Eval: DC=%.0f, PI=%.5f, range=%.5f (too wild -> motion artefact, not a pulse) -> hold",
              dc_mean, pi, pi_range);
        return 0;
    }

    LOG_I("Eval: DC=%.0f, PI=%.5f, range=%.5f (high DC, no live pulse yet) -> hold",
          dc_mean, pi, pi_range);
    return 0; /* high DC but pulse unconfirmed -> wait, do NOT flip OFF */
}

/**
 * @brief Run periodic evaluation with hysteresis
 */
static void try_evaluate(void)
{
    uint32_t now = rt_tick_get_millisecond();
    if (now - ctx.last_eval_ms < EVAL_PERIOD_MS)
        return;
    ctx.last_eval_ms = now;

    /* Diagnostic override: wear detection disabled by the user. Force WORN
     * unless on the charger, so HR/sleep run regardless of the contact
     * algorithm. set_status() no-ops when unchanged and drives the UI
     * indicator + bg_hr gating + sleep on transition. */
    if (!s_detect_enabled)
    {
        bool on_charger = battery_get_charge_state()->is_plugged;
        set_status(on_charger ? WEAR_STATUS_NOT_WEARING : WEAR_STATUS_WEARING);
        return;
    }

    /* Need minimum data before evaluating (PPG or IMU) */
    if (ctx.ppg_count < 20 && ctx.imu_count < 20)
        return;

    /* [DC-led] Broadcast the initial wear state once. Boot now defaults to
     * NOT_WEARING and may legitimately stay there (e.g. resting on a table),
     * which is no state *change* -> set_status() never fires -> the UI's
     * "not worn" indicator would never be told at boot. Push it once here. */
    static bool initial_notified = false;
    if (!initial_notified)
    {
        initial_notified = true;
        notify_wear_status(ctx.status == WEAR_STATUS_WEARING);
    }

#ifdef PPG_RACE_DEBUG
    uint32_t eval_t0 = rt_tick_get_millisecond();
    LOG_I("[WE-EVAL] enter ts=%u", (unsigned)eval_t0);
#endif
    int vote = evaluate_once(now);
#ifdef PPG_RACE_DEBUG
    {
        uint32_t eval_dt = rt_tick_get_millisecond() - eval_t0;
        if (eval_dt > 20)
        {
            LOG_I("[WE-EVAL] exit took=%ums vote=%d (slow!)",
                  (unsigned)eval_dt, vote);
        }
    }
#endif

    /* Slide the ON-vote window every evaluation (bit set = ON vote). ON votes
     * age out after 8 evals instead of being erased by a single hold, so a
     * marginal pulse that crosses the PI bar intermittently can still
     * accumulate HYSTERESIS_ON votes (see comment at HYSTERESIS_ON). */
    ctx.on_vote_window = (uint8_t)((ctx.on_vote_window << 1) | ((vote > 0) ? 1u : 0u));

    if (vote > 0)
    {
        /* Vote ON */
        ctx.off_counter = 0;

        uint8_t on_votes = on_votes_in_window(ctx.on_vote_window);
        if (ctx.status != WEAR_STATUS_WEARING && on_votes >= HYSTERESIS_ON)
        {
            /* Reset PI history and PPG buffer on state change so fresh
             * PPG data will be evaluated after sensor restarts. */
            ctx.pi_hist_count = 0;
            ctx.pi_hist_idx = 0;
            set_status(WEAR_STATUS_WEARING);
        }
        else if (ctx.status == WEAR_STATUS_WEARING && ctx.contact_break_pending &&
                 on_votes >= HYSTERESIS_ON)
        {
            ctx.contact_break_pending = false;
            LOG_I("Wear: pulse re-confirmed after contact break -> resume DC-held");
            diag_emit_last(WEAR_DIAG_EVT_BREAK_CONFIRM);
        }
    }
    else if (vote < 0)
    {
        /* Vote OFF: contact lost invalidates any accumulated ON votes */
        ctx.on_vote_window = 0;
        ctx.off_counter++;
        if (ctx.off_counter > HYSTERESIS_OFF)
            ctx.off_counter = HYSTERESIS_OFF;

        if (ctx.status != WEAR_STATUS_NOT_WEARING && ctx.off_counter >= HYSTERESIS_OFF)
        {
            set_status(WEAR_STATUS_NOT_WEARING);
        }
    }
    else
    {
        /* Uncertain: decay OFF counter; ON votes age out of the window */
        if (ctx.off_counter > 0) ctx.off_counter--;
    }
}

/* -------------------- Public API -------------------- */

void wear_detect_init(void)
{
    memset(&ctx, 0, sizeof(ctx));
    ctx.status = WEAR_STATUS_NOT_WEARING; /* [DC-led] start OFF; require DC + a live pulse to confirm worn, so booting/resting on a table is never latched as worn */
    ctx.last_eval_ms = rt_tick_get_millisecond();
    ctx.initialized = true;
    LOG_I("Wear detection initialized (PI-based, eval every %u ms)", EVAL_PERIOD_MS);
}

void wear_detect_feed_imu(Vector3 *acce, float sample_rate)
{
    if (!ctx.initialized || !acce)
        return;

    /* Store magnitude of acceleration vector */
    float mag = sqrtf(acce->x * acce->x +
                      acce->y * acce->y +
                      acce->z * acce->z);
    ctx.acce_mag[ctx.imu_idx] = mag;
    ctx.imu_idx = (ctx.imu_idx + 1) % IMU_WINDOW_SIZE;
    if (ctx.imu_count < IMU_WINDOW_SIZE)
        ctx.imu_count++;

    /* Try periodic evaluation */
    try_evaluate();
}

void wear_detect_feed_ppg(uint32_t ppg_raw, uint32_t ppg_raw2)
{
    if (!ctx.initialized)
        return;

    /* Update PPG freshness timestamp */
    ctx.last_ppg_ms = rt_tick_get_millisecond();
    ctx.ppg_ever_received = true;

    /* Store PPG sample into ring buffer */
    ctx.ppg_buf[ctx.ppg_idx] = ppg_raw;
    ctx.ppg_idx = (ctx.ppg_idx + 1) % PPG_WINDOW_SIZE;
    if (ctx.ppg_count < PPG_WINDOW_SIZE)
        ctx.ppg_count++;

    /* Try periodic evaluation */
    try_evaluate();
}

wear_status_t wear_detect_get_status(void)
{
    return ctx.status;
}

bool wear_detect_is_wearing(void)
{
    return ctx.status == WEAR_STATUS_WEARING;
}

void wear_detect_set_enabled(bool enabled)
{
    if (s_detect_enabled == enabled)
        return;
    s_detect_enabled = enabled;
    LOG_I("Wear detection %s", enabled ? "ENABLED (normal)"
                                       : "DISABLED (force worn unless charging)");
    /* Re-evaluate promptly on the next IMU/PPG feed instead of waiting out
     * the throttle, so the override takes effect within one eval period. */
    ctx.last_eval_ms = 0;
}
