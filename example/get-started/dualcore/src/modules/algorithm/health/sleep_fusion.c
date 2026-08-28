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
 *        A still wrist alone is NOT enough: if HR sits more than a fixed offset
 *        above a SELF-LEARNED resting baseline for SEVERAL consecutive readings
 *        (sedentary wake — a train, a desk, reading) the minute votes wake
 *        regardless of how low the activity score is. A LONE elevated burst —
 *        a brief REM/arousal spike or a PPG artefact — does NOT veto, and an
 *        implausibly high burst is dropped outright, so the veto no longer
 *        clips normal REM or fires on garbage HR. The baseline is learned
 *        online (adopt lows fast, leak up slowly, freeze while elevated) so it
 *        fits the wearer and can't drift up to a long sedentary HR; the veto is
 *        held a few minutes to bridge the sparse gaps between PPG bursts.
 *        Steps never veto on their own (a wrist pedometer false-counts at rest)
 *        — they force wake only when the activity score also shows motion.
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
/* 2026-08-26: was 400, chosen against SYNTHETIC accelerometer traces. Six real
   days (2026-08-20..25, ~6.8k minutes of sleep_diag) say the assumed bands do
   not exist on a wrist: the whole-day p90 of this score is 63-73 and the
   all-day maximum is 557-659, so 400 was never reachable and every minute of
   quiet desk work voted sleep. Re-derived from those days by sweep. */
#define SF_SLEEP_SCORE_THRESH    120u

/* Step threshold. A wrist-worn pedometer false-counts at rest (breathing,
   heartbeat, micro-movement), so a bare step count is NOT trusted on its own:
   steps force wake only when the activity score ALSO shows motion (see the vote
   logic). This keeps real walking awake while stopping phantom overnight steps
   from blocking sleep onset (they arrive with score ~0). */
#define SF_STEPS_FORCE_WAKE      1u
/* Steps only force wake when the minute's activity score is at least this —
   i.e. corroborated by real accelerometer motion, not a still-wrist phantom
   step. Sits above a still wrist (~0) and below the sleep threshold (400). */
#define SF_STEP_CORROB_SCORE     100u

/* HR wake-veto. A still wrist with HR well above the *learned* resting
   baseline is sedentary wake (a train, a desk, reading), not sleep — when HR
   is present and this elevated, the minute votes wake no matter how low the
   activity score is. The threshold is an ABSOLUTE bpm offset above a learned
   resting estimate (see below), validated on the Walch 2019 PSG set: a fixed
   percent of a fixed resting HR either kills real sleep for higher-HR
   sleepers (too tight) or misses the train (too loose), whereas
   learned-resting + ~20 bpm separates clear daytime elevation from real sleep
   without sacrificing sleep sensitivity. */
/* 2026-08-26: 20 -> 12, re-derived on six real days against the STABLE
   reference below. With the drifting learner a +20 offset was unreachable in
   daytime (reference leaked to 63-65 => threshold 83-85 vs a real daytime HR
   of 60-84); against a stable ~49 reference, +12 separates quiet wake from
   sleep with 1.6% daytime false-sleep and 80% night sensitivity. */
#define SF_WAKE_HR_OFFSET_BPM    12   /* HR > rhr_reference + 12 => awake */

/* Implausibility ceiling. A still or sleeping wrist physiologically sits well
   under this; sleeping HR is ~40-70 bpm and even REM/arousals rarely pass ~90.
   A burst above this is a PPG motion artefact (poor optical lock), not a real
   pulse, so it is dropped from the wake-veto, the resting-HR learner, and the
   staging baseline — a garbage-high burst (e.g. an overnight 126 bpm spike on
   a motionless wrist) can then neither fire the veto nor corrupt the baseline. */
#define SF_HR_ARTEFACT_MAX      120u

/* Sustained-elevation requirement for the wake-veto, counted in DISTINCT PPG
   bursts (input.hr_is_fresh), NOT minutes. The burst window is held ~4 min
   for staging, so counting minutes let ONE polluted burst self-satisfy any
   "consecutive" requirement via its own held repeats and keep the veto armed
   all night (2026-07-24: watch scored 12 min against ~8 h slept; replaying
   the night's phone-stored HR through this state machine attributed the loss
   to the veto, with the accel score exonerated by the rest-candidate gate
   staying engaged all night). Asymmetric thresholds:
     AWAKE  = 2 bursts — 2026-08-26: was 1. One burst is not evidence about a
              physiological state, it is one optical measurement, and this PPG
              alternates real pulses with poor-contact garbage all night. With
              1, a single garbage burst every other cycle blocked sleep onset
              for good (the distilled `polluted night` case, a real lost night
              on 2026-07-24). Requiring two CONSECUTIVE elevated bursts to
              agree is what separates a state from noise. Cost, measured on
              2026-08-20..25: daytime false-sleep 1.6% -> 5.7%, night
              sensitivity unchanged at 80.4%, and the price is bounded — one
              onset window (~15 min) of a sedentary awake stretch may be
              miscounted before the second burst arrives. Losing 15 minutes of
              a train ride beats losing a night.
     ASLEEP = 3 bursts — once asleep the veto's onset job is done. Kicking an
              established sleeper out is reserved for SUSTAINED elevation
              (~9 min at the dense 3-min cadence); lone/double garbage-high
              bursts (poor optical contact) no longer fragment the night, and
              the slip-into-sleep backstop still fires within ~3 bursts. */
#define SF_WAKE_HR_CONSEC_AWAKE   2u
#define SF_WAKE_HR_CONSEC_ASLEEP  2u

/* Online resting-HR learner. The firmware has no per-user resting HR yet
   (the seed is a fixed fallback), and real sleeping HR varies far too much
   between people for a fixed anchor to work. So we learn it: adopt new lows
   immediately (a fraction of the gap — robust to a single artefact low), and
   leak the estimate upward only SLOWLY while HR sits mildly above it. Crucial
   bit: once HR is elevated past the veto threshold the upward leak FREEZES,
   so a long sedentary-but-awake stretch (a 2 h train) can never drag the
   baseline up to its own HR and silently disarm the veto. Resting HR is thus
   learned from calm periods only. */
/* Stable cross-day resting reference — replaces the online learner as the
   veto's anchor. The learner cannot be that anchor: it is re-seeded to a fixed
   SLEEP_DEFAULT_RESTING_HR on every service start, and the watch restarts 2-3
   times a day (2026-08-24 10:35 and 17:51 caught in sleep_diag as a one-step
   52 -> 65 jump). A 65 seed puts the wake threshold at 85 bpm, which a real
   daytime HR of 60-84 never reaches, so the veto stayed disarmed for the whole
   day and every quiet minute voted sleep.

   Instead: a 2-bpm-resolution histogram per day over a trailing 3-day window,
   read out at the 5th percentile. Measured on 2026-08-20..25 this sits at
   48-51 every single day while the learner swung 49 (night) to 65 (day) — the
   swing was the bug. 3 * 64 * 2 = 384 bytes, no flash, no clock. */
#define SF_RHR_HIST_DAYS         3
#define SF_RHR_HIST_BINS        64    /* 2 bpm per bin, base SF_RHR_HIST_BASE */
#define SF_RHR_HIST_BASE        30u   /* bin 0 = 30-31 bpm                    */
#define SF_RHR_HIST_PCT          5u   /* percentile read out as the reference */
#define SF_RHR_HIST_MIN_N       60u   /* below this, fall back to the learner */

#define SF_RHR_DOWN_SHIFT        3    /* est -= (est - hr) >> 3  (~1/8 gap) */
#define SF_RHR_LEAK_PERIOD_MIN   4    /* minutes mildly-above per +1 bpm up */
#define SF_RHR_MIN               40u  /* clamp — implausibly low resting    */
#define SF_RHR_MAX              110u  /* clamp — implausibly high resting   */

/* (2026-08-26) The HR-absent gaps between sparse bursts used to be bridged by
   a countdown hold, SF_WAKE_HR_HOLD_MIN. It is gone: the veto is now latched
   (@ref wake_hr_latched), which bridges those gaps without ever forgetting. */

/* Hysteresis — minutes of agreement to enter / exit sleep. 2026-08-26: 3/2 was
   twitchy enough that a couple of quiet minutes started a "sleep session" in
   the middle of the afternoon. Sleep onset is not a three-minute event. */
#define SF_ENTER_SLEEP_MIN       15
#define SF_EXIT_SLEEP_MIN        5

/* A session shorter than this is not sleep — it is a few quiet minutes. Its
   minutes are held provisionally and only committed to the daily counters once
   the session survives this long, so afternoon stillness cannot inflate the
   day's total. */
#define SF_MIN_SESSION_MIN       30u

/* Stage thresholds (when already asleep). Activity is the minute total. */
#define SF_DEEP_ACTIVITY_MAX     50u
#define SF_REM_ACTIVITY_MAX      400u
#define SF_REM_HR_STD_MIN        3u   /* bpm */

/* HR deltas relative to resting HR, in percent. */
#define SF_DEEP_HR_DROP_PCT      5    /* HR < resting * 0.95 */
#define SF_REM_HR_NEAR_PCT       8    /* |HR - resting| < 8% */

/* HRV (RMSSD) gate separating Deep from REM. RMSSD rises in deep sleep (vagal
   tone) and is lower in REM (sympathetic) — the OPPOSITE of hr_std, which
   tracks REM's erratic HR. Compared against the night's learned RMSSD baseline
   (prv_rmssd_baseline). Applied ONLY when RR intervals are fed
   (input.hr_rmssd_ms > 0); otherwise staging is unchanged. Literature starting
   point (Walch 2019) — tune on real nights. */
#define SF_DEEP_RMSSD_RISE_PCT  20u   /* Deep: RMSSD >= baseline * 1.20 */

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
    bool    rhr_seeded;      /* learner has adopted a real HR, not the fallback */

    /* Trailing-window HR histogram feeding the stable reference. Slot
       rhr_hist_day is today; midnight rotates. */
    uint16_t rhr_hist[SF_RHR_HIST_DAYS][SF_RHR_HIST_BINS];
    uint8_t  rhr_hist_day;

    /* Activity ring buffer (last SF_WINDOW_MIN minutes). Index points
       to the slot that will be overwritten next — i.e. "oldest". */
    uint32_t activity_hist[SF_WINDOW_MIN];
    uint8_t  activity_hist_idx;
    uint8_t  activity_hist_filled; /* min(SF_WINDOW_MIN, minutes seen) */

    /* HR rolling history (last SF_HR_HISTORY_MIN valid samples). */
    uint8_t hr_hist[SF_HR_HISTORY_MIN];
    uint8_t hr_hist_idx;
    uint8_t hr_hist_filled;

    /* RMSSD (HRV) rolling history — same window as HR. Stays empty until RR
       intervals are fed via input.hr_rmssd_ms (0 = HRV disabled). */
    uint8_t rmssd_hist[SF_HR_HISTORY_MIN];
    uint8_t rmssd_hist_idx;
    uint8_t rmssd_hist_filled;

    /* Hysteresis counters. */
    uint8_t consec_sleep_candidate; /* minutes voting sleep in a row */
    uint8_t consec_wake_candidate;  /* minutes voting wake in a row */

    /* HR wake-veto, LATCHED. 2026-08-26: this used to be a countdown
       (SF_WAKE_HR_HOLD_MIN, 12 min) that expired between bursts, and sleep
       leaked in through every expiry — replaying six real days, switching the
       countdown for a latch is what took daytime false-sleep from 16% to 1.6%,
       more than any threshold change. A burst's verdict is the best estimate
       of the wrist's state until the NEXT burst contradicts it; a timer that
       forgets it is discarding evidence, not aging it. Cleared only by a
       present, non-elevated reading. */
    bool    wake_hr_latched;

    /* Consecutive elevated-HR readings seen, for the sustained-elevation gate.
       Incremented on each elevated reading, reset by any present non-elevated
       (sleep-consistent) reading. The veto arms only at SF_WAKE_HR_CONSEC_MIN. */
    uint8_t hr_elev_consec;

    /* Provisional session accounting — see SF_MIN_SESSION_MIN. */
    uint16_t session_min;          /* minutes in the current sleep session */
    uint16_t pend_total, pend_deep, pend_rem, pend_light;

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

static void prv_push_rmssd(uint8_t rmssd_ms)
{
    if (rmssd_ms == 0)
    {
        return; /* skip invalid / HRV unavailable this minute */
    }
    s_sf.rmssd_hist[s_sf.rmssd_hist_idx] = rmssd_ms;
    s_sf.rmssd_hist_idx = (uint8_t)((s_sf.rmssd_hist_idx + 1) % SF_HR_HISTORY_MIN);
    if (s_sf.rmssd_hist_filled < SF_HR_HISTORY_MIN)
    {
        s_sf.rmssd_hist_filled++;
    }
}

/* Median of recent RMSSD samples — the night's RMSSD baseline. Returns 0 when
   no RMSSD has been seen yet, which makes the classifier ignore RMSSD entirely
   (graceful fall-back to HR-std-only staging). Median for the same artefact
   robustness as prv_hr_baseline. */
static uint8_t prv_rmssd_baseline(void)
{
    if (s_sf.rmssd_hist_filled == 0)
    {
        return 0;
    }
    uint8_t tmp[SF_HR_HISTORY_MIN];
    uint8_t n = s_sf.rmssd_hist_filled;
    for (uint8_t i = 0; i < n; i++)
    {
        uint8_t v = s_sf.rmssd_hist[i];
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
/* Feed one plausible HR reading into today's histogram. */
static void prv_rhr_hist_push(uint8_t hr_bpm)
{
    if (hr_bpm < SF_RHR_HIST_BASE)
    {
        return;
    }
    uint8_t bin = (uint8_t)((hr_bpm - SF_RHR_HIST_BASE) >> 1);
    if (bin >= SF_RHR_HIST_BINS)
    {
        return;
    }
    uint16_t *c = &s_sf.rhr_hist[s_sf.rhr_hist_day][bin];
    if (*c < 0xFFFFu)
    {
        (*c)++;
    }
}

/* The stable reference: SF_RHR_HIST_PCT-th percentile over the whole trailing
   window. Returns 0 while the window is too thin to mean anything, and the
   caller then falls back to the online learner. */
static uint8_t prv_rhr_reference(void)
{
    uint32_t total = 0;
    for (uint8_t d = 0; d < SF_RHR_HIST_DAYS; d++)
    {
        for (uint8_t b = 0; b < SF_RHR_HIST_BINS; b++)
        {
            total += s_sf.rhr_hist[d][b];
        }
    }
    if (total < SF_RHR_HIST_MIN_N)
    {
        return 0;
    }

    uint32_t want = (total * SF_RHR_HIST_PCT) / 100u;
    uint32_t seen = 0;
    for (uint8_t b = 0; b < SF_RHR_HIST_BINS; b++)
    {
        for (uint8_t d = 0; d < SF_RHR_HIST_DAYS; d++)
        {
            seen += s_sf.rhr_hist[d][b];
        }
        if (seen > want)
        {
            return (uint8_t)(SF_RHR_HIST_BASE + ((uint32_t)b << 1));
        }
    }
    return 0;
}

static bool prv_hr_elevated_and_learn(uint8_t hr_bpm)
{
    if (hr_bpm == 0 || s_sf.resting_hr_bpm == 0)
    {
        return false; /* no HR this minute, or HR veto disabled */
    }

    /* First real reading after a (re)start: anchor the learner just below it
       instead of the fixed SLEEP_DEFAULT_RESTING_HR. HR at rest sits at the
       resting level and above it while active, so "first reading minus a
       margin" errs toward calling the wrist AWAKE — the safe direction. The
       old fixed 65 erred the other way and cost whole days. */
    if (!s_sf.rhr_seeded)
    {
        /* Exactly at the elevation boundary, not below it: seeding lower
           would make the very first reading "elevated", and the learner
           FREEZES its upward leak while elevated — so a too-low seed latches
           the veto on for good. Neutral is the only safe seed. */
        uint8_t seed = (hr_bpm > SF_RHR_MIN + SF_WAKE_HR_OFFSET_BPM)
                           ? (uint8_t)(hr_bpm - SF_WAKE_HR_OFFSET_BPM)
                           : (uint8_t)SF_RHR_MIN;
        s_sf.learned_rhr_bpm = seed;
        s_sf.rhr_seeded = true;
        s_sf.rhr_leak_acc = 0;
    }

    uint8_t rhr = s_sf.learned_rhr_bpm;
    /* Elevation is judged against the STABLE reference whenever the trailing
       window has enough data; the learner is only the warm-up fallback. */
    uint8_t ref = prv_rhr_reference();
    if (ref == 0)
    {
        ref = rhr;
    }
    bool elevated = ((uint32_t)hr_bpm > (uint32_t)ref + SF_WAKE_HR_OFFSET_BPM);

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
    int32_t abs_pct = hr_delta_pct < 0 ? -hr_delta_pct : hr_delta_pct;

    /* HRV (RMSSD) evidence — used ONLY when present. RMSSD peaks in deep sleep
       (vagal tone) and drops in REM, so it CONFIRMS Deep and argues AGAINST
       REM. Compare to the night's learned RMSSD baseline. When RMSSD is
       unavailable (rmssd==0 or no baseline yet) both rmssd terms below are
       neutral and staging is identical to the HR-std-only version. */
    uint8_t rmssd_base = prv_rmssd_baseline();
    bool rmssd_ok   = (in->hr_rmssd_ms > 0 && rmssd_base > 0);
    bool rmssd_high = rmssd_ok &&
        ((uint32_t)in->hr_rmssd_ms * 100u >=
         (uint32_t)rmssd_base * (100u + SF_DEEP_RMSSD_RISE_PCT));

    /* Deep: minimal motion + HR meaningfully below baseline + (when HRV is
       available) RMSSD elevated above the night's baseline. */
    if (in->activity_count <= SF_DEEP_ACTIVITY_MAX &&
        hr_delta_pct <= -(int32_t)SF_DEEP_HR_DROP_PCT &&
        (!rmssd_ok || rmssd_high))
    {
        return SLEEP_FUSION_STAGE_DEEP;
    }

    /* REM: low motion, HR near baseline, HR variability (erratic) elevated,
       and — when HRV is available — RMSSD NOT in the deep-sleep range. */
    if (in->activity_count <= SF_REM_ACTIVITY_MAX &&
        abs_pct <= (int32_t)SF_REM_HR_NEAR_PCT &&
        in->hr_std_bpm >= SF_REM_HR_STD_MIN &&
        (!rmssd_ok || !rmssd_high))
    {
        return SLEEP_FUSION_STAGE_REM;
    }

    return SLEEP_FUSION_STAGE_LIGHT;
}

static void prv_commit_session(void);
static void prv_drop_session(void);

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
        /* Session over. Anything still provisional never reached
           SF_MIN_SESSION_MIN, so it was not sleep. */
        prv_drop_session();
    }
}

/* Commit the provisional minutes held for the current session, then keep
   counting straight into the daily totals. @ref SF_MIN_SESSION_MIN. */
static void prv_commit_session(void)
{
    s_sf.out.total_sleep_min = (uint16_t)SF_MIN_CLAMP((uint32_t)s_sf.out.total_sleep_min + s_sf.pend_total);
    s_sf.out.deep_min  = (uint16_t)SF_MIN_CLAMP((uint32_t)s_sf.out.deep_min  + s_sf.pend_deep);
    s_sf.out.rem_min   = (uint16_t)SF_MIN_CLAMP((uint32_t)s_sf.out.rem_min   + s_sf.pend_rem);
    s_sf.out.light_min = (uint16_t)SF_MIN_CLAMP((uint32_t)s_sf.out.light_min + s_sf.pend_light);
    s_sf.pend_total = s_sf.pend_deep = s_sf.pend_rem = s_sf.pend_light = 0;
}

/* Discard a session that never reached SF_MIN_SESSION_MIN — it was a quiet
   stretch, not sleep. */
static void prv_drop_session(void)
{
    s_sf.pend_total = s_sf.pend_deep = s_sf.pend_rem = s_sf.pend_light = 0;
    s_sf.session_min = 0;
}

static void prv_update_daily_counters(sleep_fusion_stage_t stage,
                                      bool had_sleep_onset)
{
    /* 2026-08-26: these used to increment on ANY non-AWAKE stage with no
       session gate at all, so a day spent quietly at a desk was counted minute
       by minute as sleep — 2026-08-21 reported 1164 min (19.4 h). A minute
       only counts inside a session, and only once that session has proven
       itself SF_MIN_SESSION_MIN long. */
    bool provisional = (s_sf.session_min < SF_MIN_SESSION_MIN);
    uint16_t *bucket = NULL;

    switch (stage)
    {
    case SLEEP_FUSION_STAGE_LIGHT:
        bucket = provisional ? &s_sf.pend_light : &s_sf.out.light_min;
        break;
    case SLEEP_FUSION_STAGE_DEEP:
        bucket = provisional ? &s_sf.pend_deep : &s_sf.out.deep_min;
        break;
    case SLEEP_FUSION_STAGE_REM:
        bucket = provisional ? &s_sf.pend_rem : &s_sf.out.rem_min;
        break;
    case SLEEP_FUSION_STAGE_AWAKE:
        /* Only count WASO (Wake After Sleep Onset). */
        if (had_sleep_onset)
        {
            s_sf.out.awake_after_onset_min = (uint16_t)SF_MIN_CLAMP(
                (uint32_t)s_sf.out.awake_after_onset_min + 1);
        }
        return;
    default:
        return;
    }

    /* Inside a session. */
    s_sf.session_min = (uint16_t)SF_MIN_CLAMP((uint32_t)s_sf.session_min + 1);
    if (*bucket < 0xFFFFu)
    {
        (*bucket)++;
    }
    if (provisional)
    {
        s_sf.pend_total = (uint16_t)SF_MIN_CLAMP((uint32_t)s_sf.pend_total + 1);
        if (s_sf.session_min >= SF_MIN_SESSION_MIN)
        {
            prv_commit_session();   /* it lasted — the held minutes are real */
        }
    }
    else
    {
        s_sf.out.total_sleep_min =
            (uint16_t)SF_MIN_CLAMP((uint32_t)s_sf.out.total_sleep_min + 1);
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
    /* Rotate the resting-HR histogram onto a fresh day slot, discarding the
       oldest of the SF_RHR_HIST_DAYS. The reference itself must NOT be reset —
       it is the one thing in here that has to survive the day boundary. */
    s_sf.rhr_hist_day = (uint8_t)((s_sf.rhr_hist_day + 1u) % SF_RHR_HIST_DAYS);
    memset(s_sf.rhr_hist[s_sf.rhr_hist_day], 0,
           sizeof(s_sf.rhr_hist[s_sf.rhr_hist_day]));

    /* Keep recent history and current stage — only zero daily counters. */
    prv_drop_session();
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
    s_sf.rhr_seeded = false;              /* first real HR still re-anchors it */
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
        s_sf.wake_hr_latched = false;
        s_sf.hr_elev_consec = 0;
        prv_apply_stage_transition(utc_sec, prev_stage, SLEEP_FUSION_STAGE_NOT_WORN);
        s_sf.out.last_cole_kripke_score = 0;
        s_sf.out.last_hr_baseline_bpm = 0;
        s_sf.out.learned_rhr_bpm = s_sf.learned_rhr_bpm;
        s_sf.out.hr_wake_veto_active = false;
        return &s_sf.out;
    }

    /* Sanitize HR once: drop an implausibly high burst as a PPG motion artefact
       so it can neither fire the wake-veto nor corrupt the learned baseline.
       Sleeping HR never reaches SF_HR_ARTEFACT_MAX on a still wrist. */
    uint8_t hr_clean = (input->hr_mean_bpm > SF_HR_ARTEFACT_MAX)
                           ? 0u : input->hr_mean_bpm;

    /* Push current minute into history first so the score includes it. */
    prv_push_activity(input->activity_count);
    prv_push_hr(hr_clean);
    /* Feed the stable-reference histogram on EVERY minute that carries a
       plausible HR, not only on fresh bursts. Fresh minutes arrive roughly
       once per 10 min, so a fresh-only histogram would need ~7 h of wear to
       clear SF_RHR_HIST_MIN_N — and the whole point of the reference is to be
       usable soon after a restart. Held repeats are the same observation
       re-stated, which for a percentile over three days is just weighting. */
    if (hr_clean != 0)
    {
        prv_rhr_hist_push(hr_clean);
    }
    prv_push_rmssd(input->hr_rmssd_ms); /* no-op until RR plumbing feeds it */

    uint32_t score = prv_cole_kripke_score();
    uint8_t baseline = prv_hr_baseline();
    s_sf.out.last_cole_kripke_score = score;
    s_sf.out.last_hr_baseline_bpm = baseline;

    /* HR wake-veto: positive evidence of wake from a pulse sitting well above
       the learned resting baseline on a still wrist. Counters and the online
       resting-HR learner advance ONLY on FRESH readings (a new burst / live
       foreground minute): a held window repeat is the same observation again —
       letting it count made one polluted burst triple-vote and shatter real
       nights, and made the learner adopt each burst 3×. The hold carries the
       veto across the truly HR-absent gaps between bursts. */
    if (hr_clean != 0 && input->hr_is_fresh)
    {
        bool hr_elevated = prv_hr_elevated_and_learn(hr_clean);
        if (hr_elevated)
        {
            /* Arm threshold is asymmetric (see SF_WAKE_HR_CONSEC_*): instant
               while awake (train/desk), sustained-only once asleep so garbage
               bursts can't kick an established sleeper out. */
            bool asleep_now = (prev_stage == SLEEP_FUSION_STAGE_LIGHT ||
                               prev_stage == SLEEP_FUSION_STAGE_DEEP ||
                               prev_stage == SLEEP_FUSION_STAGE_REM);
            uint8_t arm_at = asleep_now ? SF_WAKE_HR_CONSEC_ASLEEP
                                        : SF_WAKE_HR_CONSEC_AWAKE;
            if (s_sf.hr_elev_consec < 0xFF)
            {
                s_sf.hr_elev_consec++;
            }
            if (s_sf.hr_elev_consec >= arm_at)
            {
                s_sf.wake_hr_latched = true;
            }
        }
        else
        {
            /* Fresh and NOT elevated — positive, sleep-consistent evidence.
               Clear the veto and the elevation run at once. */
            s_sf.hr_elev_consec = 0;
            s_sf.wake_hr_latched = false;
        }
    }
    /* HR absent, or a held window repeat: the latch is left exactly as it is.
       Nothing new was measured, so nothing changed — only the next fresh burst
       may move it. (This is the latch: the countdown that used to live here
       expired mid-gap and let sleep in, which was the single biggest source of
       daytime false-sleep in the 2026-08-20..25 replay.) */
    s_sf.out.hr_wake_veto_active = s_sf.wake_hr_latched;
    s_sf.out.learned_rhr_bpm = s_sf.learned_rhr_bpm;

    /* Sleep-vs-wake vote for this minute. A still wrist counts as sleep unless
       the HR veto is active OR there is CORROBORATED ambulation — steps backed
       by real accelerometer motion. A bare pedometer count is untrusted: it
       false-fires at rest, which was blocking sleep onset all night. */
    bool steps_with_motion = (input->step_count >= SF_STEPS_FORCE_WAKE) &&
                             (score >= SF_STEP_CORROB_SCORE);
    bool vote_sleep = (score < SF_SLEEP_SCORE_THRESH) &&
                      !steps_with_motion &&
                      !s_sf.wake_hr_latched;

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
