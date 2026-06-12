/**
 ******************************************************************************
 * @file   wear_detect.c
 * @author Skaiwalk software development team
 * @brief  Wear detection — DC-session-led (v2, industry-standard shape).
 *
 *         Principle (matches mainstream wearables): the PULSE is only ever
 *         required to confirm the FIRST wear of a session ("cold entry").
 *         After that, staying worn and re-becoming worn are decided purely
 *         by CONTACT evidence — the PPG DC level measured against a
 *         per-session learned baseline — because deep-sleep perfusion is
 *         too weak to demand a pulse (overnight CSV 2026-06-11: 82% of the
 *         night falsely OFF, 5.5 h stuck, 48 probes all failing the pulse
 *         gate while DC sat solidly in the worn band).
 *
 *         State machine:
 *         - COLD OFF→ON: windowed pulse gate (PI ≥ threshold + variability,
 *           3-of-8 evals). Seeds worn_dc_base = current DC.
 *         - WARM OFF→ON: DC back inside [0.88, 1.15] × worn_dc_base for
 *           3-of-8 evals → ON, no pulse. This is what makes every wrong OFF
 *           cheap (seconds) instead of catastrophic (hours).
 *         - ON hold: DC above max(0.80 × base, DC_ABS_FLOOR). Baseline EMA
 *           adapts slowly while in-band (per-unit / per-fit calibration —
 *           measured worn DC differs >4k between our two units).
 *         - ON→OFF: sustained sub-threshold DC — fast (~12 s) when the dip
 *           came with motion (take-off signature), slow (~30 s) when still
 *           (protects sleep dips). Charger (is_plugged) blocks WARM entry
 *           and freezes the EMA, so a charging cradle can't be learned or
 *           warm-recovered into.
 *
 *         Known accepted trade-off: a surface whose DC coincides with the
 *         learned worn band can warm-recover to a false ON (single green
 *         channel cannot separate skin from such a surface without a pulse;
 *         mainstream solves this with IR-ratio or capacitive hardware we
 *         don't have). Product priority is sleep continuity > table
 *         rejection; sessions are bounded by charging and the settings
 *         toggle exists as an escape hatch. See ADR 0015.
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

/* Absolute DC floor: below this, nothing skin-like touches the sensor.
 * Overnight CSV (sleep unit, 2026-06-11): off-wrist surfaces read 30.5k-35.5k,
 * worn sleep dips bottom out at 36.4k, worn band 39k-44k. The floor sits in
 * the 35.5k-36.4k gap. It is only a safety net / cold-entry sanity check —
 * the working ON/OFF decisions use the per-session learned baseline below. */
#define DC_ABS_FLOOR            36000

/* Per-session DC baseline (learned, replaces any fixed worn threshold —
 * measured worn DC differs >4k between units and with strap fit):
 * - Seeded by the first pulse-confirmed ON of a session.
 * - EMA-adapts while ON and in-band (alpha 1/64 per 1.5s eval ≈ 96s to 63%).
 * - WORN_BAND: DC within [LO,HI]×base counts as wrist contact (hold + warm
 *   re-entry). Night data: worn dips reach 0.90×base, so LO=0.88 keeps them.
 * - OFF_THR: DC below OFF_THR×base (clamped up to DC_ABS_FLOOR) votes OFF. */
#define WORN_BAND_LO            0.88f
#define WORN_BAND_HI            1.15f
#define OFF_THR_FACTOR          0.80f
/* Asymmetric EMA: learn DOWN (loosening fit) at 1/64 per eval, UP at only
 * 1/256 — a pressed-sensor episode (DC 45k+) must not inflate the base and
 * drag off_thr above the 36.4k sleep-dip floor margin. */
#define BASE_EMA_SHIFT          6       /* down: base += (dc-base)/64  */
#define BASE_EMA_UP_SHIFT       8       /* up:   base += (dc-base)/256 */

/* Perfusion Index threshold (AC_pp / DC_mean) — COLD ENTRY ONLY. A live
 * pulse is required just once per session, to prove the first contact is a
 * wrist and not a table. It is never required to stay ON or to re-enter ON
 * (deep-sleep perfusion cannot deliver it; see file header).
 * Measured: wearing PI spikes > 0.001, table noise tops ~0.0005-0.0008. */
#define PI_THD_TO_ON            0.0010f

/* IMU variance threshold (m/s^2)^2: "the wrist is moving" marker, used to
 * pick the fast OFF hysteresis on take-off-shaped DC drops. */
#define IMU_VARIANCE_THD        0.03f

/* ON-vote accumulation: HYSTERESIS_ON votes within the last 8 evaluations
 * (~12 s window, not consecutive — a marginal signal that crosses its bar
 * intermittently still accumulates; measured crossing rate can be ~30%). */
#define HYSTERESIS_ON           3

/* OFF hysteresis (consecutive sub-threshold evals):
 * - with motion in the streak (take-off signature): fast, ~12 s.
 * - still (loose strap / sleep dips): slow, ~30 s — a sleeping wrist's DC
 *   dip is transient, a removed watch's is not. Wrong OFFs are cheap now
 *   anyway: warm re-entry restores ON within seconds of DC returning. */
#define OFF_EVALS_MOTION        8
#define OFF_EVALS_STILL         20

/* PI variability check (cold entry): real heartbeat fluctuates; constant PI
 * (noise/static surface) is rejected, wildly swinging PI (set-down motion
 * artefact, measured ~0.27) is rejected by PI_RANGE_MAX. */
#define PI_HISTORY_LEN          5
#define PI_RANGE_THD            0.0003f
#define PI_RANGE_MAX            0.15f

/* PPG freshness: if no new PPG sample arrives within this many
 * milliseconds, consider PPG data stale (sensor likely powered off). */
#define PPG_STALE_MS            5000

/* PPG settle time: after PPG sensor restarts, the first few seconds
 * produce wildly inaccurate readings (huge PI spikes).  Ignore PPG
 * data during this settle period. */
#define PPG_SETTLE_MS           6000

/* When PPG is stale and device is OFF-wrist, IMU motion powers PPG up for a
 * probe window so contact (warm) or pulse (cold) can confirm a wrist. */
#define IMU_RETRIGGER_THD       0.05f

/* [DC-led] After motion while off-wrist, hold PPG powered for this long so
 * the wrist can confirm even if the user then stays still. If nothing
 * confirms within the window, PPG is powered back down until next motion. */
#define PROBE_WINDOW_MS         (3 * 60 * 1000)

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

    /* Timing for periodic evaluation */
    uint32_t last_eval_ms;

    /* Hysteresis state */
    uint8_t on_vote_window; /* bitmask of last 8 eval votes, bit set = ON vote */
    int8_t off_counter;     /* counts consecutive OFF evaluations */
    bool off_streak_motion; /* motion seen during the current OFF-vote streak
                               (take-off signature -> fast hysteresis) */

    /* Per-session worn-DC baseline. 0 = invalid (cold: no wear confirmed
     * since boot / since charging while off-wrist ended the session).
     * (Re)seeded by EVERY pulse-confirmed ON; EMA-adapted while the current
     * ON stretch is pulse-anchored; survives OFF periods so warm re-entry
     * works at night. */
    float worn_dc_base;
    bool last_vote_pulse;   /* the most recent +1 vote came from the pulse gate */
    bool on_anchor_pulse;   /* current ON stretch was entered via pulse (EMA gate) */

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

    /* Charging while OFF ends the wear session: the baseline must not
     * survive onto the charger (a base poisoned by a band-coincident surface
     * dies here — this implements the "sessions are bounded by charging"
     * contract in the file header). Runs before the stale gate so it works
     * with PPG powered down on the cradle. */
    if (ctx.status == WEAR_STATUS_NOT_WEARING && ctx.worn_dc_base > 0.0f &&
        battery_get_charge_state()->is_plugged)
    {
        ctx.worn_dc_base = 0.0f;
        LOG_I("Wear: charger while off-wrist -> session baseline cleared");
    }

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

    /* === DC-session-led decision (see file header). Pulse is consulted only
     * for cold entry; everything else is contact evidence vs the learned
     * per-session baseline. === */
    bool plugged = battery_get_charge_state()->is_plugged;

    if (ctx.status == WEAR_STATUS_WEARING)
    {
        /* OFF threshold: relative collapse below the worn baseline, never
         * below the absolute floor (protects against a stale/低 baseline). */
        float off_thr = (float)DC_ABS_FLOOR;
        if (ctx.worn_dc_base > 0.0f &&
            ctx.worn_dc_base * OFF_THR_FACTOR > off_thr)
        {
            off_thr = ctx.worn_dc_base * OFF_THR_FACTOR;
        }

        if (dc_mean < off_thr)
        {
            /* Sub-threshold: vote OFF. Motion during the streak marks a
             * take-off signature -> try_evaluate uses the fast hysteresis. */
            uint16_t imu_len = (ctx.imu_count < IMU_WINDOW_SIZE)
                                   ? ctx.imu_count : IMU_WINDOW_SIZE;
            float imu_var = (imu_len > 0)
                                ? compute_imu_variance(ctx.acce_mag, imu_len) : 0.0f;
            if (imu_var >= IMU_VARIANCE_THD)
                ctx.off_streak_motion = true;
            ctx.last_imu_var = imu_var;
            LOG_I("Eval: DC=%.0f (< %.0f) -> contact lost -> OFF vote (%s)",
                  dc_mean, off_thr,
                  ctx.off_streak_motion ? "moving" : "still");
            return -1;
        }

        /* Contact holds: stay ON regardless of PI (sleep perfusion may be
         * flat). Adapt the baseline slowly, but ONLY while this ON stretch
         * is pulse-anchored — a warm re-entry might be a band-coincident
         * surface, and learning from it would drag the band onto the
         * surface (review finding: morning table walked base 43.5k→39.1k).
         * Frozen while charging (charger couples noise into the ADC). */
        if (!plugged && ctx.on_anchor_pulse && ctx.worn_dc_base > 0.0f &&
            dc_mean >= ctx.worn_dc_base * WORN_BAND_LO &&
            dc_mean <= ctx.worn_dc_base * WORN_BAND_HI)
        {
            float delta = dc_mean - ctx.worn_dc_base;
            ctx.worn_dc_base += delta / (float)(1 << ((delta > 0.0f)
                                                          ? BASE_EMA_UP_SHIFT
                                                          : BASE_EMA_SHIFT));
        }
        LOG_I("Eval: DC=%.0f (base=%.0f) -> worn, contact held -> ON",
              dc_mean, ctx.worn_dc_base);
        return 1;
    }

    /* Currently OFF. The PPG running now was opened by a motion probe window;
     * if that window expired without confirming a wrist, power PPG back down
     * and wait for the next motion (saves LED current when nothing is worn). */
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

    /* WARM re-entry: this session already proved a wrist once (pulse-seeded
     * baseline). DC back inside the worn band IS the wrist returning — no
     * pulse demanded, deep sleep cannot deliver one. Blocked while charging
     * so a cradle is never warm-recovered into. The DC_ABS_FLOOR clamp keeps
     * the entry set a subset of the hold set (entry below the hold
     * threshold would oscillate ON/OFF deterministically). */
    if (!plugged && ctx.worn_dc_base > 0.0f &&
        dc_mean >= (float)DC_ABS_FLOOR &&
        dc_mean >= ctx.worn_dc_base * WORN_BAND_LO &&
        dc_mean <= ctx.worn_dc_base * WORN_BAND_HI)
    {
        LOG_I("Eval: DC=%.0f in worn band [%.0f..%.0f] -> warm re-entry -> ON",
              dc_mean, ctx.worn_dc_base * WORN_BAND_LO,
              ctx.worn_dc_base * WORN_BAND_HI);
        ctx.last_vote_pulse = false;
        return 1;
    }

    /* COLD entry: no session evidence (or DC outside the learned band) —
     * require a live pulse so a table is never confirmed as a wrist. */
    if (dc_mean >= (float)DC_ABS_FLOOR &&
        pi >= PI_THD_TO_ON && pi_range <= PI_RANGE_MAX &&
        (ctx.pi_hist_count < PI_HISTORY_LEN || pi_range >= PI_RANGE_THD))
    {
        LOG_I("Eval: DC=%.0f, AC_pp=%.0f, PI=%.5f, range=%.5f -> live wrist -> ON",
              dc_mean, ac_pp, pi, pi_range);
        ctx.last_vote_pulse = true;
        return 1;
    }
    if (pi >= PI_THD_TO_ON && pi_range > PI_RANGE_MAX)
    {
        LOG_I("Eval: DC=%.0f, PI=%.5f, range=%.5f (too wild -> motion artefact, not a pulse) -> hold",
              dc_mean, pi, pi_range);
        return 0;
    }

    LOG_I("Eval: DC=%.0f, PI=%.5f, range=%.5f (no contact evidence yet) -> hold",
          dc_mean, pi, pi_range);
    return 0; /* unconfirmed -> wait */
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
        ctx.off_streak_motion = false;

        if (ctx.status != WEAR_STATUS_WEARING &&
            on_votes_in_window(ctx.on_vote_window) >= HYSTERESIS_ON)
        {
            if (ctx.last_vote_pulse)
            {
                /* Pulse is the highest-trust evidence: ALWAYS (re)seed the
                 * baseline from it. This corrects a base poisoned by an
                 * artefact-inflated earlier seed (review: put-on artefact
                 * seeded 56.9k and locked the real 43.4k wrist out), and
                 * keeps entry ⊆ hold monotonic (off_thr becomes 0.8×this
                 * DC, so a fresh pulse confirm can never oscillate). */
                ctx.worn_dc_base = ctx.last_dc;
                LOG_I("Wear: session baseline (re)seeded at DC=%.0f",
                      ctx.worn_dc_base);
            }
            /* Warm re-entry keeps the existing pulse-seeded baseline; the
             * anchor flag gates EMA learning to pulse-proven stretches. */
            ctx.on_anchor_pulse = ctx.last_vote_pulse;
            /* Reset PI history on state change so fresh PPG data will be
             * evaluated after sensor restarts. */
            ctx.pi_hist_count = 0;
            ctx.pi_hist_idx = 0;
            set_status(WEAR_STATUS_WEARING);
        }
    }
    else if (vote < 0)
    {
        /* Vote OFF: contact lost invalidates any accumulated ON votes.
         * Dual-speed hysteresis: a take-off (motion in the streak) confirms
         * fast; a still dip (loose strap during sleep) must persist ~30 s —
         * and even a wrong OFF is recovered in seconds by warm re-entry. */
        ctx.on_vote_window = 0;
        ctx.off_counter++;
        if (ctx.off_counter > OFF_EVALS_STILL)
            ctx.off_counter = OFF_EVALS_STILL;

        int8_t needed = ctx.off_streak_motion ? OFF_EVALS_MOTION
                                              : OFF_EVALS_STILL;
        if (ctx.status != WEAR_STATUS_NOT_WEARING && ctx.off_counter >= needed)
        {
            set_status(WEAR_STATUS_NOT_WEARING);
            ctx.off_streak_motion = false;
        }
    }
    else
    {
        /* Uncertain: decay OFF counter; ON votes age out of the window */
        if (ctx.off_counter > 0) ctx.off_counter--;
        if (ctx.off_counter == 0)
            ctx.off_streak_motion = false;
    }
}

/* -------------------- Public API -------------------- */

void wear_detect_init(void)
{
    memset(&ctx, 0, sizeof(ctx));
    ctx.status = WEAR_STATUS_NOT_WEARING; /* start OFF (cold): the first wear of a session must be pulse-confirmed, so booting/resting on a table is never latched as worn */
    ctx.last_eval_ms = rt_tick_get_millisecond();
    ctx.initialized = true;
    LOG_I("Wear detection initialized (DC-session-led v2, eval every %u ms)", EVAL_PERIOD_MS);
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
    /* Votes/counters frozen during bypass are stale evidence — clear them
     * so re-enabling starts from a clean slate, and re-evaluate promptly on
     * the next feed instead of waiting out the throttle. */
    ctx.on_vote_window = 0;
    ctx.off_counter = 0;
    ctx.off_streak_motion = false;
    ctx.last_eval_ms = 0;
}
