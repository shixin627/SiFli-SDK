/**
 ******************************************************************************
 * @file   sleep_service.c
 * @author Skaiwalk software development team
 * @brief  LCPU driver for the sleep_fusion algorithm. Samples IMU + HR
 *         once per second, aggregates per-minute features, and ships
 *         stage transitions to HCPU via the existing watch_sys_sync RPC.
 *
 *  Data sources:
 *    Accel — bmi270_accel_read() over I2C, direct chip read. Works in
 *      every power mode (active, low-power, DARK/FIFO-only) because the
 *      data registers are always available once the chip is awake. We
 *      do not rely on hand_tracking / the AHRS path because both are
 *      gated off during screen-off, which is exactly when we need to
 *      detect sleep.
 *    HR — hr_service_get_latest_bpm() returns the last BPM the LCPU HR
 *      service has accepted. PPG is usually power-gated overnight on the
 *      watch boards so HR samples will be sparse to absent during the
 *      sleep window; the classifier falls back to Light staging in that
 *      case.
 *    Steps — bmi270_hw_step_counter_read() delta per minute, used as a
 *      strong "wake" signal that overrides the activity score.
 *
 *  The RT timer is a SOFT_TIMER so PM can keep LCPU in LIGHT sleep
 *  between ticks.
 ******************************************************************************
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <time.h>
#include <string.h>
#include "board.h"
#include "rtconfig.h"

#include "sleep_fusion.h"
#include "watch_sys_service.h"
#ifdef BSP_USING_HR_SVC
#include "hr_service.h"      /* hr_service_get_latest_bpm */
#endif

#ifdef BSP_USING_WEAR_DETECT
#include "wear_detect.h"
#endif

#ifdef ACC_USING_BMI270
#include "bmi270_driver.h"
#endif

/* Sleep status enum on the wire (health_algo.h). Mirrored here so this
   module compiles standalone without the legacy vendor health lib. */
#define WIRE_TSLEEP        0x01
#define WIRE_TDEEP_SLEEP   0x02
#define WIRE_TGET_UP       0x03
#define WIRE_TNOT_WEAR     0x04
#define WIRE_TREM_SLEEP    0x05  /* new — extends T_SLEEP_STATUS for staging */

#define DBG_TAG "DS.SLEEP"
#define DBG_LVL DBG_INFO
#include "rtdbg.h"

#define SLEEP_SAMPLE_PERIOD_MS 1000          /* 1 Hz IMU/HR snapshot */
#define SLEEP_MINUTE_TICKS     60            /* samples per evaluation window */
#define SLEEP_DEFAULT_RESTING_HR 65          /* fallback if we never see HR */

/* How long to reuse a background PPG-burst HR window. Must span one full sampler
   cycle (BG_HR_PERIOD_MS) plus slack, else the minutes between bursts get no HR
   features at all and Light/Deep/REM degrade for lack of input rather than lack
   of accuracy. Sized for the 10-min / 3-min-burst night cadence: a window is
   published at burst end and the next arrives 10 min later, so 11 min covers
   every minute. Keep in sync with BG_HR_PERIOD_MS (hr_service.c).
   NOTE: a "fresh" minute now occurs once per 10 min instead of once per 3, so
   the wake-veto's SF_WAKE_HR_CONSEC_ASLEEP=3 needs ~30 min of sustained high HR
   to rule a sleeper awake (was ~9 min) — deliberately left untuned; ADR-0018
   validated that constant against replay, so re-tune only on new night data. */
#define SLEEP_HR_WINDOW_MAX_AGE_MS (11 * 60 * 1000)

/* Rest-candidate dense-HR gate (Phase 1 of the missed-night fix; ADR-0017).
   Failure mode being fixed: HR sampling density used to follow the sleep
   VERDICT. A night whose sparse (~15 min) readings ran falsely high (loose
   band / poor optical contact reads of 80-119 bpm slip under the 120 artefact
   ceiling) re-armed sleep_fusion's 12-min wake-veto hold on every burst,
   leaving only a ~3-min clean window per 15-min cycle — sleep fragmented to
   ~2 min per cycle all night, and the misdetected "awake" state kept sampling
   sparse: a locked feedback loop. Fix: density now also follows cheap DIRECT
   evidence — a still wrist inside the overnight window goes dense regardless
   of the verdict. Dense readings let the veto self-heal (one clean
   sub-threshold reading clears hold + run at once, sleep_fusion.c), so the
   algorithm itself stays untouched. */
#define SLEEP_REST_SCORE_THRESH   400u /* mirror SF_SLEEP_SCORE_THRESH        */
#define SLEEP_REST_ENTER_MIN      10   /* still minutes in a row to go dense  */
#define SLEEP_REST_EXIT_MIN       3    /* active minutes in a row to drop out */
#define SLEEP_REST_WINDOW_START_H 21   /* overnight window: 21:00 ..          */
#define SLEEP_REST_WINDOW_END_H   11   /* .. 10:59 local wall-clock           */

/* TEMP accel diagnostic (per-second raw accel + delta + per-minute SLPMIN to the
   HCPU log / COM12 via notify_debug_log). DISABLED for overnight collection: the
   per-second debug_log woke HCPU every tick (battery drain over a full night).
   The per-minute data now rides the sleep_diag CSV path (KEY_SLEEP_DIAG) to the
   phone instead. Re-#define ONLY for a cabled daytime still-wrist experiment. */
/* #define SLEEP_ACCEL_DIAG */

typedef struct
{
    rt_timer_t timer;
    bool       started;

    /* Per-minute accumulators. Reset every SLEEP_MINUTE_TICKS samples. */
    uint32_t accel_activity_sum;     /* sum of inter-sample |Δaccel|     */
    uint16_t accel_sample_count;
    uint32_t hr_sum;
    uint32_t hr_sum_sq;
    uint16_t hr_sample_count;
    uint16_t ticks_this_minute;

    /* Previous raw accel sample for inter-sample delta. */
    int16_t  prev_ax;
    int16_t  prev_ay;
    int16_t  prev_az;
    bool     prev_accel_valid;

    /* Cross-minute state. */
    uint32_t last_step_count;
    bool     last_step_count_valid;
    uint8_t  last_emitted_wire_mode;  /* 0 = nothing emitted yet */
    int16_t  last_local_day;          /* for midnight reset; -1 sentinel */

    /* Rest-candidate dense-HR gate (see SLEEP_REST_*). */
    uint8_t  rest_still_run;   /* consecutive minutes score < thresh  */
    uint8_t  rest_active_run;  /* consecutive minutes score >= thresh */
    bool     rest_candidate;   /* dense bursts requested while awake  */

    /* Burst-freshness tracking for the HR window feed: ages only grow
       between bursts, so age going DOWN since the previous minute-eval
       marks a NEW burst (drives input.hr_is_fresh). */
    uint32_t prev_hr_win_age_ms;
    bool     prev_hr_win_age_valid;

    /* Clock-jump guard for the midnight reset: previous minute-eval's utc, to
       tell an external time-set (phone re-sync after reboot) from a real minute. */
    uint32_t prev_eval_utc;
    bool     prev_eval_utc_valid;
} sleep_service_env_t;

static sleep_service_env_t s_env;

/* ------------------------------------------------------------------
 *  Helpers
 * ------------------------------------------------------------------ */

static uint8_t prv_stage_to_wire(sleep_fusion_stage_t stage)
{
    switch (stage)
    {
    case SLEEP_FUSION_STAGE_AWAKE:    return WIRE_TGET_UP;
    case SLEEP_FUSION_STAGE_LIGHT:    return WIRE_TSLEEP;
    case SLEEP_FUSION_STAGE_DEEP:     return WIRE_TDEEP_SLEEP;
    case SLEEP_FUSION_STAGE_REM:      return WIRE_TREM_SLEEP;
    case SLEEP_FUSION_STAGE_NOT_WORN: return WIRE_TNOT_WEAR;
    default:                          return WIRE_TGET_UP;
    }
}

static uint32_t prv_accel_delta_activity(void)
{
    /* Raw accel read via I2C — survives DARK / hand_tracking-off modes.
       BMI270's accel block is always alive once we've enabled the step
       counter at boot; reading the X/Y/Z data registers does not change
       the power state. Sensitivity at ±2g is ~16384 LSB/g; we work
       directly in LSB to stay integer-only. */
#ifdef ACC_USING_BMI270
    int16_t ax = 0, ay = 0, az = 0;
    if (bmi270_accel_read(&ax, &ay, &az) != 0)
    {
        return 0;
    }
    if (!s_env.prev_accel_valid)
    {
        s_env.prev_ax = ax;
        s_env.prev_ay = ay;
        s_env.prev_az = az;
        s_env.prev_accel_valid = true;
        return 0;
    }
    int32_t dx = (int32_t)ax - (int32_t)s_env.prev_ax;
    int32_t dy = (int32_t)ay - (int32_t)s_env.prev_ay;
    int32_t dz = (int32_t)az - (int32_t)s_env.prev_az;
    s_env.prev_ax = ax;
    s_env.prev_ay = ay;
    s_env.prev_az = az;

    /* L1 norm of the delta — cheaper than L2 and monotonically related
       enough for the Cole-Kripke threshold. Scale down by 1024 so a 1g
       single-axis jerk (~16384 LSB) lands at ~16, and a minute of typical
       walking (lots of 0.5-1g changes per second) sums to a few hundred. */
    uint32_t adx = (uint32_t)(dx >= 0 ? dx : -dx);
    uint32_t ady = (uint32_t)(dy >= 0 ? dy : -dy);
    uint32_t adz = (uint32_t)(dz >= 0 ? dz : -dz);
    uint32_t mag = (adx + ady + adz) >> 10;
    return mag;
#else
    return 0;
#endif
}

static bool prv_is_worn(void)
{
#ifdef BSP_USING_WEAR_DETECT
    return wear_detect_is_wearing();
#else
    return true;
#endif
}

static uint16_t prv_get_step_delta_and_advance(void)
{
#ifdef ACC_USING_BMI270
    uint32_t now = 0;
    if (bmi270_hw_step_counter_read(&now) != 0)
    {
        return 0; /* chip not ready — treat as no steps */
    }
    if (!s_env.last_step_count_valid)
    {
        s_env.last_step_count = now;
        s_env.last_step_count_valid = true;
        return 0;
    }
    uint32_t delta = (now >= s_env.last_step_count) ? (now - s_env.last_step_count) : 0;
    s_env.last_step_count = now;
    if (delta > 0xFFFFu)
    {
        delta = 0xFFFFu;
    }
    return (uint16_t)delta;
#else
    return 0;
#endif
}

static void prv_sample_once(void)
{
    /* Activity: inter-sample |Δaccel|, integer LSB. Read direct from
       BMI270 so it works in screen-off / hand-tracking-off modes. */
    uint32_t accel_d = prv_accel_delta_activity();
    s_env.accel_activity_sum += accel_d;
    s_env.accel_sample_count++;
#ifdef SLEEP_ACCEL_DIAG
    if (watch_sys_sync.notify_debug_log)
    {
        char dbg[96];
        rt_snprintf(dbg, sizeof(dbg), "[SLPACC] ax=%d ay=%d az=%d d=%u sum=%u",
                    (int)s_env.prev_ax, (int)s_env.prev_ay, (int)s_env.prev_az,
                    (unsigned)accel_d, (unsigned)s_env.accel_activity_sum);
        watch_sys_sync.notify_debug_log(dbg);
    }
#endif

    /* HR: latest BPM from hr_service. 0 means PPG not running or no
       valid pulse. Note: PPG is gated off during sleep on most boards to
       save power, so HR samples will mostly be missing during night —
       the algorithm degrades to Light-only staging in that case. */
#ifdef BSP_USING_HR_SVC
    uint8_t hr = hr_service_get_latest_bpm();
    if (hr > 0 && hr < 240)
    {
        s_env.hr_sum += hr;
        s_env.hr_sum_sq += (uint32_t)hr * (uint32_t)hr;
        s_env.hr_sample_count++;
    }
#endif
}

static uint8_t prv_compute_hr_std(uint32_t sum, uint32_t sum_sq, uint16_t n)
{
    if (n < 2)
    {
        return 0;
    }
    /* Variance = (sum_sq / n) - (sum/n)^2. Compute with care to avoid
       overflow on uint32. n is at most 60. */
    uint32_t mean = sum / n;
    uint32_t mean_sq = mean * mean;
    uint32_t mean_of_sq = sum_sq / n;
    if (mean_of_sq <= mean_sq)
    {
        return 0;
    }
    uint32_t var = mean_of_sq - mean_sq;
    if (var < 2)
    {
        return (uint8_t)var;
    }
    /* Integer sqrt via Heron's method. Converges in O(log log var) iters;
       loop on r decreasing is the stop condition. var here is at most
       ~uint16 range (variance of an 8-bit HR series capped at 60 samples)
       so 8 iterations is far more than enough. */
    uint32_t r = var >> 1;
    if (r == 0)
    {
        r = 1;
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        uint32_t next = (r + var / r) >> 1;
        if (next >= r)
        {
            break;
        }
        r = next;
    }
    if (r > 0xFFu)
    {
        r = 0xFFu;
    }
    return (uint8_t)r;
}

static uint8_t prv_resting_hr_estimate(void)
{
    /* For now we just use the configured fallback. A future enhancement
       can pull from hr_service's learned RHR (env->env.rhr). Exposing
       it requires a new hr_service public getter — out of scope for
       MVP. */
    return SLEEP_DEFAULT_RESTING_HR;
}

static void prv_emit_stage(const sleep_fusion_output_t *out, uint32_t utc_now)
{
    uint8_t wire_mode = prv_stage_to_wire(out->stage);
    if (wire_mode == s_env.last_emitted_wire_mode)
    {
        return;
    }
    s_env.last_emitted_wire_mode = wire_mode;
    if (watch_sys_sync.notify_sleep_state)
    {
        watch_sys_sync.notify_sleep_state(wire_mode, utc_now);
    }
    LOG_I("sleep stage -> wire=%u utc=%u score=%u baseline=%u total=%u D=%u R=%u L=%u",
          (unsigned)wire_mode, (unsigned)utc_now,
          (unsigned)out->last_cole_kripke_score,
          (unsigned)out->last_hr_baseline_bpm,
          (unsigned)out->total_sleep_min,
          (unsigned)out->deep_min,
          (unsigned)out->rem_min,
          (unsigned)out->light_min);
}

static void prv_minute_eval(uint32_t utc_now)
{
    /* Build the per-minute feature vector. */
    sleep_fusion_minute_input_t input = {0};

    /* Activity: sum of inter-sample |Δaccel| (LSB >> 10) over the minute.
       Approximate ranges with ±2g full-scale (16384 LSB/g):
         dead still       0 - 50
         restless sleep   50 - 400
         tossing / turning 400 - 2000
         walking / awake  > 2000 */
    input.activity_count = s_env.accel_activity_sum;
    input.step_count = prv_get_step_delta_and_advance();

    if (s_env.hr_sample_count > 0)
    {
        /* Foreground path: the Exercise app is measuring, so hr_service has
           accepted live per-second BPM this minute. Use our own aggregation.
           Live data every minute => every minute is a fresh observation. */
        input.hr_mean_bpm = (uint8_t)(s_env.hr_sum / s_env.hr_sample_count);
        input.hr_std_bpm = prv_compute_hr_std(
            s_env.hr_sum, s_env.hr_sum_sq, s_env.hr_sample_count);
        input.hr_is_fresh = true;
    }
    else
    {
        input.hr_mean_bpm = 0;
        input.hr_std_bpm = 0;
#ifdef BSP_USING_HR_SVC
        /* Background path (the usual overnight case): PPG is power-gated, so
           hr_service_get_latest_bpm() is stale and we saw no per-second HR.
           Fall back to the periodic PPG-burst window — a value + std (HRV
           proxy) from the burst (the daily-HR-curve burst, reused here for
           free). Hold it until it goes stale so a coarse Light/Deep split is
           staged across the gaps between bursts rather than collapsing to
           Light — but mark ONLY the first minute after a new burst as FRESH:
           the wake-veto must see each burst once, not once per held minute
           (sleep_fusion.h hr_is_fresh). */
        uint8_t w_mean = 0, w_std = 0;
        uint32_t w_age_ms = 0;
        if (hr_service_get_hr_window(&w_mean, &w_std, &w_age_ms))
        {
            bool new_burst = !s_env.prev_hr_win_age_valid ||
                             (w_age_ms < s_env.prev_hr_win_age_ms);
            s_env.prev_hr_win_age_ms = w_age_ms;
            s_env.prev_hr_win_age_valid = true;
            if (w_age_ms <= SLEEP_HR_WINDOW_MAX_AGE_MS && w_mean > 0)
            {
                input.hr_mean_bpm = w_mean;
                input.hr_std_bpm = w_std;
                input.hr_is_fresh = new_burst;
            }
        }
#endif
    }

    input.is_worn = prv_is_worn();

    /* NB: resting HR is now learned ONLINE inside sleep_fusion from the per-
       minute HR stream (it only needs the seed passed at init). We no longer
       re-seed it every minute — that would just overwrite the seed with the
       same constant. If a real per-user RHR ever lands in hr_service, push it
       once here via sleep_fusion_set_resting_hr(). */

    /* Local-midnight reset of the daily aggregates — guarded against clock-jump
       FALSE triggers. The RTC is "wall-clock as UTC" and the phone re-syncs time
       on every BLE reconnect; a reboot-then-resync can jump the clock across a day
       boundary mid-sleep and fire a false midnight reset, zeroing the night's total
       (a root cause of the 462→60 loss). Guards:
       (a) ignore implausible dates (dirty RTC after VBAT loss, year < 2024);
       (b) if utc jumped far from the previous minute-eval (external time-set, not a
           real minute elapsing), only RE-BASELINE the day — do NOT reset. */
    {
        time_t t = (time_t)utc_now;
        struct tm *lt = localtime(&t);
        bool date_ok = (lt != NULL) && ((lt->tm_year + 1900) >= 2024);
        bool clock_jumped = s_env.prev_eval_utc_valid &&
            ((utc_now > s_env.prev_eval_utc)
                 ? (utc_now - s_env.prev_eval_utc > 600u)
                 : (s_env.prev_eval_utc - utc_now > 600u));
        if (lt && date_ok)
        {
            int16_t today = (int16_t)lt->tm_yday;
            if (s_env.last_local_day < 0 || clock_jumped)
            {
                /* First eval, or an external clock-set — adopt the new day as
                   baseline WITHOUT zeroing today's accumulator. */
                s_env.last_local_day = today;
            }
            else if (today != s_env.last_local_day)
            {
                sleep_fusion_midnight_reset();
                s_env.last_local_day = today;
                LOG_I("sleep: midnight reset (yday %d)", (int)today);
            }
        }
        s_env.prev_eval_utc = utc_now;
        s_env.prev_eval_utc_valid = true;
    }

    const sleep_fusion_output_t *out = sleep_fusion_update(utc_now, &input);

    /* Sleep/wake above is accel-only. When in a sleep stage, drive dense PPG
       bursts (two-stage gate) so HR can stage Deep/REM; revert on wake/not-worn.
       NOTE: wrist-wake suppression was intentionally removed -- gating HW
       wrist-wake on this accel-only "asleep" signal could lock lift-to-wake on a
       false positive, with no reliable accel-driven release while the screen is
       dark (regressed lift-to-wake; reverted to screen-state-only gating). */
    bool asleep = (out->stage == SLEEP_FUSION_STAGE_LIGHT ||
                   out->stage == SLEEP_FUSION_STAGE_DEEP ||
                   out->stage == SLEEP_FUSION_STAGE_REM);

    /* Rest-candidate: a still wrist inside the overnight window also drives
       dense bursts, independent of the verdict (see SLEEP_REST_* block).
       Exit needs SLEEP_REST_EXIT_MIN active minutes so one mid-night toss
       doesn't drop density; the tracker keeps counting while asleep too
       (harmless — OR'd below — and keeps density across brief wake blips). */
    {
        bool in_window = false;
        time_t t = (time_t)utc_now;
        struct tm *lt = localtime(&t);
        if (lt)
        {
            in_window = (lt->tm_hour >= SLEEP_REST_WINDOW_START_H ||
                         lt->tm_hour < SLEEP_REST_WINDOW_END_H);
        }
        if (!in_window || !input.is_worn)
        {
            s_env.rest_still_run = 0;
            s_env.rest_active_run = 0;
            s_env.rest_candidate = false;
        }
        else if (out->last_cole_kripke_score < SLEEP_REST_SCORE_THRESH)
        {
            s_env.rest_active_run = 0;
            if (s_env.rest_still_run < 0xFF)
            {
                s_env.rest_still_run++;
            }
            if (s_env.rest_still_run >= SLEEP_REST_ENTER_MIN)
            {
                s_env.rest_candidate = true;
            }
        }
        else
        {
            s_env.rest_still_run = 0;
            if (s_env.rest_active_run < 0xFF)
            {
                s_env.rest_active_run++;
            }
            if (s_env.rest_active_run >= SLEEP_REST_EXIT_MIN)
            {
                s_env.rest_candidate = false;
            }
        }
    }
#ifdef BSP_USING_HR_SVC
    hr_service_set_sleep_active(asleep || s_env.rest_candidate);
#endif

    /* Forensic once-a-minute line (LCPU console): enough to attribute any
       future missed night — activity vs score, veto state, learned RHR,
       dense-gate state — without reflashing a diagnostic build. */
    LOG_I("[SLPDIAG] act=%u st=%u hr=%u fr=%u sd=%u sc=%u stg=%u veto=%u rhr=%u base=%u rest=%u tot=%u",
          (unsigned)input.activity_count, (unsigned)input.step_count,
          (unsigned)input.hr_mean_bpm, (unsigned)input.hr_is_fresh,
          (unsigned)input.hr_std_bpm,
          (unsigned)out->last_cole_kripke_score, (unsigned)out->stage,
          (unsigned)out->hr_wake_veto_active, (unsigned)out->learned_rhr_bpm,
          (unsigned)out->last_hr_baseline_bpm, (unsigned)s_env.rest_candidate,
          (unsigned)out->total_sleep_min);

#ifdef SLEEP_ACCEL_DIAG
    if (watch_sys_sync.notify_debug_log)
    {
        char dbg[120];
        rt_snprintf(dbg, sizeof(dbg),
                    "[SLPMIN] act=%u sc=%u stg=%u veto=%u rhr=%u base=%u rest=%u tot=%u",
                    (unsigned)input.activity_count, (unsigned)out->last_cole_kripke_score,
                    (unsigned)out->stage, (unsigned)out->hr_wake_veto_active,
                    (unsigned)out->learned_rhr_bpm, (unsigned)out->last_hr_baseline_bpm,
                    (unsigned)s_env.rest_candidate, (unsigned)out->total_sleep_min);
        watch_sys_sync.notify_debug_log(dbg);
    }
#endif

    /* Per-minute sleep diagnostic to phone (KEY_SLEEP_DIAG) — nightly CSV for
       offline SQI-threshold + missed-night attribution. hr_std = SQI candidate.
       Not gated by SLEEP_ACCEL_DIAG: this is the load-bearing collection path. */
    if (watch_sys_sync.notify_sleep_diag)
    {
        uint16_t pi_now = 0;
        uint16_t frame_pct_now = 0;
        uint16_t rate_info_now = 0;
#ifdef BSP_USING_HR_SVC
        extern uint16_t hr_service_get_last_pi_e3(void);
        extern uint16_t hr_service_get_last_frame_pct(void);
        extern uint16_t hr_service_get_last_rate_info(void);
        pi_now = hr_service_get_last_pi_e3();
        frame_pct_now = hr_service_get_last_frame_pct();
        rate_info_now = hr_service_get_last_rate_info();
#endif
        watch_sys_sleep_diag_t drec = {
            .ts     = utc_now,
            .score  = (uint16_t)out->last_cole_kripke_score,
            .hr     = input.hr_mean_bpm,
            .hr_std = input.hr_std_bpm,
            .stage  = (uint8_t)out->stage,
            .veto   = (uint8_t)(out->hr_wake_veto_active ? 1 : 0),
            .rhr    = out->learned_rhr_bpm,
            .worn   = (uint8_t)(input.is_worn ? 1 : 0),
            .rest   = (uint8_t)(s_env.rest_candidate ? 1 : 0),
            .fresh  = (uint8_t)(input.hr_is_fresh ? 1 : 0),
            .total  = out->total_sleep_min,
            .deep   = out->deep_min,
            .rem    = out->rem_min,
            .light  = out->light_min,
            .pi_e3  = pi_now,
            .frame_pct = frame_pct_now,
            .rate_info = rate_info_now,
        };
        watch_sys_sync.notify_sleep_diag(&drec);
    }

    prv_emit_stage(out, utc_now);

    /* Reset per-minute accumulators. */
    s_env.accel_activity_sum = 0;
    s_env.accel_sample_count = 0;
    s_env.hr_sum = 0;
    s_env.hr_sum_sq = 0;
    s_env.hr_sample_count = 0;
    s_env.ticks_this_minute = 0;
}

static void prv_timer_cb(void *param)
{
    (void)param;

    /* Off-wrist: suspend sleep detection entirely until the watch is worn
       again. wear_detect re-arms to WEARING on IMU motion, which resumes this
       automatically next tick. Keep the soft timer ticking (negligible) but do
       no sampling or staging, release the dense-HR sleep gate so HR stays
       powered down, and drop any partial-minute data + re-seed the accel/step
       baselines so the first minute after re-wear starts clean. */
    if (!prv_is_worn())
    {
#ifdef BSP_USING_HR_SVC
        hr_service_set_sleep_active(false);
#endif
        s_env.accel_activity_sum = 0;
        s_env.accel_sample_count = 0;
        s_env.hr_sum = 0;
        s_env.hr_sum_sq = 0;
        s_env.hr_sample_count = 0;
        s_env.ticks_this_minute = 0;
        s_env.prev_accel_valid = false;
        s_env.last_step_count_valid = false;
        s_env.rest_still_run = 0;
        s_env.rest_active_run = 0;
        s_env.rest_candidate = false;
        return;
    }

    prv_sample_once();
    s_env.ticks_this_minute++;
    if (s_env.ticks_this_minute >= SLEEP_MINUTE_TICKS)
    {
        uint32_t utc_now = (uint32_t)time(NULL);
        prv_minute_eval(utc_now);
    }
}

/* ------------------------------------------------------------------
 *  Public init
 * ------------------------------------------------------------------ */

int sleep_service_register(void)
{
    if (s_env.started)
    {
        return 0;
    }

    memset(&s_env, 0, sizeof(s_env));
    s_env.last_local_day = -1;

    sleep_fusion_init(prv_resting_hr_estimate());

    s_env.timer = rt_timer_create("sleep_fuse", prv_timer_cb, NULL,
                                  rt_tick_from_millisecond(SLEEP_SAMPLE_PERIOD_MS),
                                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (!s_env.timer)
    {
        LOG_E("sleep_service: timer create failed");
        return 1;
    }
    rt_timer_start(s_env.timer);
    s_env.started = true;
    LOG_I("sleep_service: armed (1 Hz sampler, %u s eval, resting_hr=%u)",
          (unsigned)(SLEEP_MINUTE_TICKS), (unsigned)prv_resting_hr_estimate());
    return 0;
}

INIT_COMPONENT_EXPORT(sleep_service_register);

/* ------------------------------------------------------------------
 *  MSH debug commands — wrapped in !kReleaseMode per project convention
 * ------------------------------------------------------------------ */
#if !defined(kReleaseMode) || (kReleaseMode == 0)

static int msh_sleep_status(int argc, char **argv)
{
    (void)argc; (void)argv;
    const sleep_fusion_output_t *out = sleep_fusion_current();
    rt_kprintf("sleep_fusion: stage=%u in_stage=%u total=%u D=%u R=%u L=%u WASO=%u\n",
               (unsigned)out->stage,
               (unsigned)out->consecutive_minutes_in_stage,
               (unsigned)out->total_sleep_min,
               (unsigned)out->deep_min,
               (unsigned)out->rem_min,
               (unsigned)out->light_min,
               (unsigned)out->awake_after_onset_min);
    rt_kprintf("  ck_score=%u hr_baseline=%u learned_rhr=%u hr_wake_veto=%u onset_utc=%u last_wake_utc=%u\n",
               (unsigned)out->last_cole_kripke_score,
               (unsigned)out->last_hr_baseline_bpm,
               (unsigned)out->learned_rhr_bpm,
               (unsigned)out->hr_wake_veto_active,
               (unsigned)out->sleep_onset_utc,
               (unsigned)out->last_wake_utc);
    rt_kprintf("  rest_cand=%u still_run=%u active_run=%u\n",
               (unsigned)s_env.rest_candidate,
               (unsigned)s_env.rest_still_run,
               (unsigned)s_env.rest_active_run);
    return 0;
}
MSH_CMD_EXPORT(msh_sleep_status, dump sleep_fusion state);

static int msh_sleep_reset(int argc, char **argv)
{
    (void)argc; (void)argv;
    sleep_fusion_reset();
    s_env.last_emitted_wire_mode = 0;
    s_env.last_local_day = -1;
    rt_kprintf("sleep_fusion: hard reset\n");
    return 0;
}
MSH_CMD_EXPORT(msh_sleep_reset, hard reset sleep_fusion state);

#endif /* !kReleaseMode */
