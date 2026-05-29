/**
 ******************************************************************************
 * @file   sleep_fusion.c
 * @author Skaiwalk software development team
 * @brief  Sleep stage classifier — implementation. Pure C, integer math.
 *
 *  Two-pass per minute:
 *    (1) Cole-Kripke weighted activity score over a 7-minute causal window
 *        decides asleep / awake. Hysteresis on top: 3 consecutive sleep
 *        minutes to enter sleep, 2 consecutive wake minutes to exit. This
 *        suppresses single-epoch flips that confuse a naive cutoff.
 *        A still wrist alone is NOT enough: if HR is available and sits
 *        more than a fixed offset above a SELF-LEARNED resting baseline
 *        (sedentary wake — a train, a desk, reading) the minute votes wake
 *        regardless of how low the activity score is. Without this veto,
 *        smooth low-motion transit reads as hours of phantom sleep. The
 *        baseline is learned online (adopt lows fast, leak up slowly, freeze
 *        while elevated) so it fits the wearer and can't drift up to a long
 *        sedentary HR; the veto is held a few minutes past the last elevated
 *        reading to bridge the sparse gaps between background PPG bursts.
 *    (2) When asleep, HR features distinguish Deep / Light / REM:
 *          Deep — activity ~0  AND  HR < resting * 0.95
 *          REM  — low activity AND  HR near resting AND HR std > 3 bpm
 *          Light — everything else inside a sleep session
 *
 *  Fixed-point: all internal scores are uint32. Cole-Kripke weights are
 *  scaled ×1024 (10-bit fraction). No floating point — keeps the LCPU
 *  microlib footprint flat and lets the unit tests stay deterministic.
 ******************************************************************************
 */
#include "sleep_fusion.h"
#include <string.h>

/* ------------------------------------------------------------------
 *  Tunables — empirical, mirrored from Cole-Kripke 1992 / Sadeh 1994
 *  with adjustments for a 7-min causal window. Re-tune with overnight
 *  PSG comparison if you ever get one.
 * ------------------------------------------------------------------ */

#define SF_WINDOW_MIN          7  /* causal weighted activity window */
#define SF_HR_HISTORY_MIN     10  /* rolling HR samples for baseline */

/* Cole-Kripke style weights, oldest -> newest, ×1024.
   Tuned so a stable resting wrist (activity ~200) scores < threshold
   and a single jerk (activity ~3000) lands above. */
static const uint16_t SF_CK_WEIGHTS_Q10[SF_WINDOW_MIN] = {
    40,   /* t-6 */
    60,   /* t-5 */
    100,  /* t-4 */
    160,  /* t-3 */
    260,  /* t-2 */
    420,  /* t-1 */
    1024  /* t-0 (now) */
};

/* Score below this => sleep candidate. The activity_count input is the
   per-minute sum of inter-sample raw-accel deltas (LSB >> 10) as fed by
   sleep_service.c. Empirical ranges:
     dead still           < 50  per minute
     light tossing       50-400
     significant motion  400+
   Threshold 400 is conservative; raise to be more lenient about calling
   sleep, lower to be stricter. */
#define SF_SLEEP_SCORE_THRESH    400u

/* Any nonzero step count in the minute hard-forces wake. */
#define SF_STEPS_FORCE_WAKE      1u

/* HR wake-veto. A still wrist with HR well above the *learned* resting
   baseline is sedentary wake (a train, a desk, reading), not sleep — when HR
   is present and this elevated, the minute votes wake no matter how low the
   activity score is. The threshold is an ABSOLUTE bpm offset above a learned
   resting estimate (see below), validated on the Walch 2019 PSG set: a fixed
   percent of a fixed resting HR either kills real sleep for higher-HR
   sleepers (too tight) or misses the train (too loose), whereas
   learned-resting + ~20 bpm separates clear daytime elevation from real sleep
   without sacrificing sleep sensitivity. */
#define SF_WAKE_HR_OFFSET_BPM    20   /* HR > learned_rhr + 20 => awake */

/* Online resting-HR learner. The firmware has no per-user resting HR yet
   (the seed is a fixed fallback), and real sleeping HR varies far too much
   between people for a fixed anchor to work. So we learn it: adopt new lows
   immediately (a fraction of the gap — robust to a single artefact low), and
   leak the estimate upward only SLOWLY while HR sits mildly above it. Crucial
   bit: once HR is elevated past the veto threshold the upward leak FREEZES,
   so a long sedentary-but-awake stretch (a 2 h train) can never drag the
   baseline up to its own HR and silently disarm the veto. Resting HR is thus
   learned from calm periods only. */
#define SF_RHR_DOWN_SHIFT        3    /* est -= (est - hr) >> 3  (~1/8 gap) */
#define SF_RHR_LEAK_PERIOD_MIN   4    /* minutes mildly-above per +1 bpm up */
#define SF_RHR_MIN               40u  /* clamp — implausibly low resting    */
#define SF_RHR_MAX              110u  /* clamp — implausibly high resting   */

/* Background PPG is sparse while awake (one ~15-min curve burst, held a few
   minutes), so most awake minutes carry no HR at all. Hold the wake-veto this
   many minutes to bridge those HR-ABSENT gaps so a still wrist cannot creep
   into "sleep" between bursts. The hold counts down only while HR is missing;
   a present, non-elevated reading clears it immediately (positive sleep
   evidence), so a lone sleep-time spike can't snowball into false wake. */
#define SF_WAKE_HR_HOLD_MIN      12u

/* Hysteresis — minutes of agreement to enter / exit sleep. */
#define SF_ENTER_SLEEP_MIN       3
#define SF_EXIT_SLEEP_MIN        2

/* Stage thresholds (when already asleep). Activity is the minute total. */
#define SF_DEEP_ACTIVITY_MAX     50u
#define SF_REM_ACTIVITY_MAX      400u
#define SF_REM_HR_STD_MIN        3u   /* bpm */

/* HR deltas relative to resting HR, in percent. */
#define SF_DEEP_HR_DROP_PCT      5    /* HR < resting * 0.95 */
#define SF_REM_HR_NEAR_PCT       8    /* |HR - resting| < 8% */

/* Bounds on returned aggregate counters (clamp at uint16 max). */
#define SF_MIN_CLAMP(x) ((x) > 0xFFFFu ? 0xFFFFu : (x))

/* ------------------------------------------------------------------
 *  Internal state
 * ------------------------------------------------------------------ */

typedef struct
{
    uint8_t resting_hr_bpm; /* configured seed / enable (0 disables HR) */
    uint8_t learned_rhr_bpm; /* online resting-HR estimate (seeded from above) */
    uint8_t rhr_leak_acc;    /* minutes counted toward the slow upward leak */

    /* Activity ring buffer (last SF_WINDOW_MIN minutes). Index points
       to the slot that will be overwritten next — i.e. "oldest". */
    uint32_t activity_hist[SF_WINDOW_MIN];
    uint8_t  activity_hist_idx;
    uint8_t  activity_hist_filled; /* min(SF_WINDOW_MIN, minutes seen) */

    /* HR rolling history (last SF_HR_HISTORY_MIN valid samples). */
    uint8_t hr_hist[SF_HR_HISTORY_MIN];
    uint8_t hr_hist_idx;
    uint8_t hr_hist_filled;

    /* Hysteresis counters. */
    uint8_t consec_sleep_candidate; /* minutes voting sleep in a row */
    uint8_t consec_wake_candidate;  /* minutes voting wake in a row */

    /* HR wake-veto hold. Set to SF_WAKE_HR_HOLD_MIN on an elevated-HR
       minute, decremented otherwise; while nonzero the minute votes wake.
       Bridges the HR-absent gaps between sparse background PPG bursts. */
    uint8_t wake_hr_hold_min;

    /* Output snapshot. */
    sleep_fusion_output_t out;
} sleep_fusion_state_t;

static sleep_fusion_state_t s_sf;

/* ------------------------------------------------------------------
 *  Helpers
 * ------------------------------------------------------------------ */

static void prv_push_activity(uint32_t activity)
{
    s_sf.activity_hist[s_sf.activity_hist_idx] = activity;
    s_sf.activity_hist_idx = (uint8_t)((s_sf.activity_hist_idx + 1) % SF_WINDOW_MIN);
    if (s_sf.activity_hist_filled < SF_WINDOW_MIN)
    {
        s_sf.activity_hist_filled++;
    }
}

static void prv_push_hr(uint8_t hr_bpm)
{
    if (hr_bpm == 0)
    {
        return; /* skip invalid */
    }
    s_sf.hr_hist[s_sf.hr_hist_idx] = hr_bpm;
    s_sf.hr_hist_idx = (uint8_t)((s_sf.hr_hist_idx + 1) % SF_HR_HISTORY_MIN);
    if (s_sf.hr_hist_filled < SF_HR_HISTORY_MIN)
    {
        s_sf.hr_hist_filled++;
    }
}

/* Weighted Cole-Kripke style activity score over the causal window.
   Returns Q0 (regular integer) — weights are ×1024 internally and we
   shift back at the end. */
static uint32_t prv_cole_kripke_score(void)
{
    if (s_sf.activity_hist_filled == 0)
    {
        return 0;
    }

    /* Walk newest -> oldest. activity_hist_idx points at oldest slot
       (next-to-overwrite). Newest = (idx - 1) mod N. */
    uint32_t score_q10 = 0;
    uint8_t n = s_sf.activity_hist_filled;
    uint8_t walk_idx = (uint8_t)((s_sf.activity_hist_idx + SF_WINDOW_MIN - 1) % SF_WINDOW_MIN);
    /* Newest-to-oldest pairs with weight index SF_WINDOW_MIN-1 down to 0. */
    uint8_t w_idx = SF_WINDOW_MIN - 1;
    for (uint8_t k = 0; k < n; k++)
    {
        uint32_t a = s_sf.activity_hist[walk_idx];
        score_q10 += a * (uint32_t)SF_CK_WEIGHTS_Q10[w_idx];
        walk_idx = (uint8_t)((walk_idx + SF_WINDOW_MIN - 1) % SF_WINDOW_MIN);
        if (w_idx == 0)
        {
            break;
        }
        w_idx--;
    }
    return score_q10 >> 10;
}

/* Median of the HR history, used as a baseline. We pick median over
   mean because it is robust to single-sample HR spikes (motion artefact
   from the PPG sensor on a moving wrist). */
static uint8_t prv_hr_baseline(void)
{
    if (s_sf.hr_hist_filled == 0)
    {
        return s_sf.resting_hr_bpm; /* fall back to configured baseline */
    }
    /* Tiny n — insertion sort into a scratch buffer is fine. */
    uint8_t tmp[SF_HR_HISTORY_MIN];
    uint8_t n = s_sf.hr_hist_filled;
    for (uint8_t i = 0; i < n; i++)
    {
        uint8_t v = s_sf.hr_hist[i];
        uint8_t j = i;
        while (j > 0 && tmp[j - 1] > v)
        {
            tmp[j] = tmp[j - 1];
            j--;
        }
        tmp[j] = v;
    }
    return tmp[n / 2];
}

/* Online resting-HR learner + "is this minute's HR elevated?" test, the core
   of the wake-veto. Returns true when HR is present and sits more than
   SF_WAKE_HR_OFFSET_BPM above the learned resting estimate (=> sedentary
   wake). Side effect: updates learned_rhr_bpm — adopt new lows fast, leak up
   slowly, and FREEZE the upward leak while elevated so a long awake-but-still
   stretch (a train) cannot pull the baseline up to itself and disarm the
   veto. No-op returning false when HR is absent or HR staging is disabled. */
static bool prv_hr_elevated_and_learn(uint8_t hr_bpm)
{
    if (hr_bpm == 0 || s_sf.resting_hr_bpm == 0)
    {
        return false; /* no HR this minute, or HR veto disabled */
    }

    uint8_t rhr = s_sf.learned_rhr_bpm;
    /* Decide elevation against the established baseline (pre-update). */
    bool elevated = ((uint32_t)hr_bpm > (uint32_t)rhr + SF_WAKE_HR_OFFSET_BPM);

    if (hr_bpm < rhr)
    {
        /* New low — adopt a fraction of the gap (robust to artefact lows). */
        uint8_t step = (uint8_t)((rhr - hr_bpm) >> SF_RHR_DOWN_SHIFT);
        if (step == 0)
        {
            step = 1;
        }
        rhr = (uint8_t)(rhr - step);
        s_sf.rhr_leak_acc = 0;
    }
    else if (!elevated)
    {
        /* Mildly above — leak the estimate up slowly. */
        if (++s_sf.rhr_leak_acc >= SF_RHR_LEAK_PERIOD_MIN)
        {
            rhr++;
            s_sf.rhr_leak_acc = 0;
        }
    }
    else
    {
        /* Elevated (veto firing) — freeze the upward leak. */
        s_sf.rhr_leak_acc = 0;
    }

    if (rhr < SF_RHR_MIN)
    {
        rhr = (uint8_t)SF_RHR_MIN;
    }
    if (rhr > SF_RHR_MAX)
    {
        rhr = (uint8_t)SF_RHR_MAX;
    }
    s_sf.learned_rhr_bpm = rhr;

    return elevated;
}

/* Classify the in-sleep stage given current minute's accel + HR features
   and the rolling HR baseline. */
static sleep_fusion_stage_t prv_classify_sleep_stage(
    const sleep_fusion_minute_input_t *in, uint8_t hr_baseline)
{
    /* If no HR signal: best we can do is "Light" — accel alone cannot
       distinguish Deep / REM reliably. */
    if (in->hr_mean_bpm == 0 || hr_baseline == 0)
    {
        return SLEEP_FUSION_STAGE_LIGHT;
    }

    /* Compute percent deviation from baseline (signed, in percent).
       Avoid floating point: pct = (hr - baseline) * 100 / baseline. */
    int32_t hr_delta_pct =
        ((int32_t)in->hr_mean_bpm - (int32_t)hr_baseline) * 100 / (int32_t)hr_baseline;

    /* Deep: minimal motion + HR meaningfully below baseline. */
    if (in->activity_count <= SF_DEEP_ACTIVITY_MAX &&
        hr_delta_pct <= -(int32_t)SF_DEEP_HR_DROP_PCT)
    {
        return SLEEP_FUSION_STAGE_DEEP;
    }

    /* REM: low motion, HR near baseline, HR variability elevated. */
    int32_t abs_pct = hr_delta_pct < 0 ? -hr_delta_pct : hr_delta_pct;
    if (in->activity_count <= SF_REM_ACTIVITY_MAX &&
        abs_pct <= (int32_t)SF_REM_HR_NEAR_PCT &&
        in->hr_std_bpm >= SF_REM_HR_STD_MIN)
    {
        return SLEEP_FUSION_STAGE_REM;
    }

    return SLEEP_FUSION_STAGE_LIGHT;
}

static void prv_apply_stage_transition(uint32_t utc_sec,
                                       sleep_fusion_stage_t prev,
                                       sleep_fusion_stage_t next)
{
    if (prev == next)
    {
        s_sf.out.consecutive_minutes_in_stage++;
        s_sf.out.stage_changed = false;
        return;
    }
    s_sf.out.stage = next;
    s_sf.out.stage_changed = true;
    s_sf.out.consecutive_minutes_in_stage = 1;

    bool prev_was_sleep = (prev == SLEEP_FUSION_STAGE_LIGHT ||
                           prev == SLEEP_FUSION_STAGE_DEEP ||
                           prev == SLEEP_FUSION_STAGE_REM);
    bool next_is_sleep = (next == SLEEP_FUSION_STAGE_LIGHT ||
                          next == SLEEP_FUSION_STAGE_DEEP ||
                          next == SLEEP_FUSION_STAGE_REM);

    if (!prev_was_sleep && next_is_sleep)
    {
        /* Sleep onset — record only the FIRST onset of the day. */
        if (s_sf.out.sleep_onset_utc == 0)
        {
            s_sf.out.sleep_onset_utc = utc_sec;
        }
    }
    if (prev_was_sleep && !next_is_sleep)
    {
        s_sf.out.last_wake_utc = utc_sec;
    }
}

static void prv_update_daily_counters(sleep_fusion_stage_t stage,
                                      bool had_sleep_onset)
{
    switch (stage)
    {
    case SLEEP_FUSION_STAGE_LIGHT:
        s_sf.out.light_min = (uint16_t)SF_MIN_CLAMP((uint32_t)s_sf.out.light_min + 1);
        s_sf.out.total_sleep_min = (uint16_t)SF_MIN_CLAMP((uint32_t)s_sf.out.total_sleep_min + 1);
        break;
    case SLEEP_FUSION_STAGE_DEEP:
        s_sf.out.deep_min = (uint16_t)SF_MIN_CLAMP((uint32_t)s_sf.out.deep_min + 1);
        s_sf.out.total_sleep_min = (uint16_t)SF_MIN_CLAMP((uint32_t)s_sf.out.total_sleep_min + 1);
        break;
    case SLEEP_FUSION_STAGE_REM:
        s_sf.out.rem_min = (uint16_t)SF_MIN_CLAMP((uint32_t)s_sf.out.rem_min + 1);
        s_sf.out.total_sleep_min = (uint16_t)SF_MIN_CLAMP((uint32_t)s_sf.out.total_sleep_min + 1);
        break;
    case SLEEP_FUSION_STAGE_AWAKE:
        /* Only count WASO (Wake After Sleep Onset). */
        if (had_sleep_onset)
        {
            s_sf.out.awake_after_onset_min = (uint16_t)SF_MIN_CLAMP(
                (uint32_t)s_sf.out.awake_after_onset_min + 1);
        }
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------
 *  Public API
 * ------------------------------------------------------------------ */

void sleep_fusion_init(uint8_t resting_hr_bpm)
{
    memset(&s_sf, 0, sizeof(s_sf));
    s_sf.resting_hr_bpm = resting_hr_bpm;
    s_sf.learned_rhr_bpm = resting_hr_bpm; /* seed the online estimate */
    s_sf.out.stage = SLEEP_FUSION_STAGE_AWAKE;
}

void sleep_fusion_set_resting_hr(uint8_t resting_hr_bpm)
{
    s_sf.resting_hr_bpm = resting_hr_bpm;
    /* Seed the learned estimate the first time HR is enabled; after that the
       online learner owns it — don't clobber what it has adapted to. */
    if (s_sf.learned_rhr_bpm == 0)
    {
        s_sf.learned_rhr_bpm = resting_hr_bpm;
    }
}

void sleep_fusion_midnight_reset(void)
{
    /* Keep recent history and current stage — only zero daily counters. */
    s_sf.out.total_sleep_min = 0;
    s_sf.out.deep_min = 0;
    s_sf.out.rem_min = 0;
    s_sf.out.light_min = 0;
    s_sf.out.awake_after_onset_min = 0;
    s_sf.out.sleep_onset_utc = 0;
    s_sf.out.last_wake_utc = 0;
}

void sleep_fusion_reset(void)
{
    uint8_t saved_resting = s_sf.resting_hr_bpm;
    memset(&s_sf, 0, sizeof(s_sf));
    s_sf.resting_hr_bpm = saved_resting;
    s_sf.learned_rhr_bpm = saved_resting; /* re-seed the online estimate */
    s_sf.out.stage = SLEEP_FUSION_STAGE_AWAKE;
}

const sleep_fusion_output_t *sleep_fusion_update(
    uint32_t utc_sec, const sleep_fusion_minute_input_t *input)
{
    sleep_fusion_stage_t prev_stage = s_sf.out.stage;

    /* Not worn short-circuits everything: emit NOT_WORN, clear hysteresis
       so we don't roll into sleep the instant the watch is put back on. */
    if (!input->is_worn)
    {
        s_sf.consec_sleep_candidate = 0;
        s_sf.consec_wake_candidate = 0;
        s_sf.wake_hr_hold_min = 0;
        prv_apply_stage_transition(utc_sec, prev_stage, SLEEP_FUSION_STAGE_NOT_WORN);
        s_sf.out.last_cole_kripke_score = 0;
        s_sf.out.last_hr_baseline_bpm = 0;
        s_sf.out.learned_rhr_bpm = s_sf.learned_rhr_bpm;
        s_sf.out.hr_wake_veto_active = false;
        return &s_sf.out;
    }

    /* Push current minute into history first so the score includes it. */
    prv_push_activity(input->activity_count);
    prv_push_hr(input->hr_mean_bpm);

    uint32_t score = prv_cole_kripke_score();
    uint8_t baseline = prv_hr_baseline();
    s_sf.out.last_cole_kripke_score = score;
    s_sf.out.last_hr_baseline_bpm = baseline;

    /* HR wake-veto: positive evidence of wake from a pulse sitting well above
       the learned resting baseline on a still wrist. prv_hr_elevated_and_learn
       also advances the online resting-HR estimate (and freezes it while
       elevated, so a long sedentary stretch can't disarm the veto). Only fires
       when HR is actually present, so the overnight HR-absent case is
       untouched and degrades to accel-only exactly as before. The hold carries
       the veto across the sparse gaps between background PPG bursts. */
    bool hr_says_awake = prv_hr_elevated_and_learn(input->hr_mean_bpm);
    if (hr_says_awake)
    {
        s_sf.wake_hr_hold_min = SF_WAKE_HR_HOLD_MIN;
    }
    else if (input->hr_mean_bpm > 0)
    {
        /* HR present and NOT elevated — positive, sleep-consistent evidence.
           Clear the veto at once. The hold exists only to bridge HR-ABSENT
           gaps; without this, a single sleep-time HR spike would arm the hold
           and stretch into ~12 min of false wake (it cost real sleep on the
           denser-HR PSG validation set). A real elevation re-arms every minute
           it persists, so sustained wake is unaffected. */
        s_sf.wake_hr_hold_min = 0;
    }
    else if (s_sf.wake_hr_hold_min > 0)
    {
        s_sf.wake_hr_hold_min--; /* HR absent — bridge the gap */
    }
    s_sf.out.hr_wake_veto_active = (s_sf.wake_hr_hold_min > 0);
    s_sf.out.learned_rhr_bpm = s_sf.learned_rhr_bpm;

    /* Sleep-vs-wake vote for this minute. A still wrist (low score, no
       steps) only counts as sleep when the HR veto is not active. */
    bool vote_sleep = (score < SF_SLEEP_SCORE_THRESH) &&
                      (input->step_count < SF_STEPS_FORCE_WAKE) &&
                      (s_sf.wake_hr_hold_min == 0);

    if (vote_sleep)
    {
        s_sf.consec_sleep_candidate =
            (s_sf.consec_sleep_candidate < 0xFF) ? (uint8_t)(s_sf.consec_sleep_candidate + 1) : 0xFF;
        s_sf.consec_wake_candidate = 0;
    }
    else
    {
        s_sf.consec_wake_candidate =
            (s_sf.consec_wake_candidate < 0xFF) ? (uint8_t)(s_sf.consec_wake_candidate + 1) : 0xFF;
        s_sf.consec_sleep_candidate = 0;
    }

    bool currently_in_sleep_state = (prev_stage == SLEEP_FUSION_STAGE_LIGHT ||
                                     prev_stage == SLEEP_FUSION_STAGE_DEEP ||
                                     prev_stage == SLEEP_FUSION_STAGE_REM);

    sleep_fusion_stage_t next_stage = prev_stage;
    if (!currently_in_sleep_state)
    {
        /* Currently awake / not-worn / fresh start. Need ENTER_SLEEP_MIN
           sleep-voting minutes in a row to flip into sleep. */
        if (s_sf.consec_sleep_candidate >= SF_ENTER_SLEEP_MIN)
        {
            next_stage = prv_classify_sleep_stage(input, baseline);
        }
        else
        {
            next_stage = SLEEP_FUSION_STAGE_AWAKE;
        }
    }
    else
    {
        /* Currently asleep. Wake voting needs EXIT_SLEEP_MIN in a row. */
        if (s_sf.consec_wake_candidate >= SF_EXIT_SLEEP_MIN)
        {
            next_stage = SLEEP_FUSION_STAGE_AWAKE;
        }
        else
        {
            /* Stay asleep; re-classify which sleep stage. */
            next_stage = prv_classify_sleep_stage(input, baseline);
        }
    }

    bool had_onset_before_this_minute = (s_sf.out.sleep_onset_utc != 0);
    prv_apply_stage_transition(utc_sec, prev_stage, next_stage);
    prv_update_daily_counters(next_stage, had_onset_before_this_minute ||
                                              (next_stage != SLEEP_FUSION_STAGE_AWAKE));
    return &s_sf.out;
}

const sleep_fusion_output_t *sleep_fusion_current(void)
{
    return &s_sf.out;
}
