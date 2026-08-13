#include "hr_autocorr.h"
#include <string.h>
#include <stdbool.h>

/* Correlation values are Q15 (32768 == 1.0); lags are Q8. Integer throughout —
   the LCPU's float support is not worth depending on for ~11k operations that
   run once a second. */
#define Q15                 32768
#define THR_Q15             11469   /* 0.35 — below this the window is noise   */
#define TROUGH_RISE_Q15      4915   /* 0.15 — how far the correlation must climb
                                       back after its first turning point for
                                       that turning point to be believed       */
#define TOL_Q15              1966   /* 0.06 — peaks within this of the best are
                                       "tied", and among ties the SHORTEST lag
                                       wins. See the header for why.           */
#define MIN_OVERLAP            40   /* samples that must overlap at a lag      */
#define SCALE_MAX            1023   /* detrended |x| ceiling: 1023^2 * 249 fits
                                       int32 with room to spare               */
#define MAX_PEAKS              24   /* lag range is 44 wide; peaks alternate   */

/* NLMS motion compensation. 16 taps per axis is ~0.64 s of accelerometer at
   25 Hz — long enough to cover the phase lag between a wrist movement and the
   optical artefact it produces, short enough that 48 weights converge inside a
   single 10 s window. */
#define NLMS_TAPS       16
#define NLMS_AXES        3
#define NLMS_W          (NLMS_TAPS * NLMS_AXES)
#define NLMS_MU_Q15   11469         /* 0.35 */
#define NLMS_GATE_ACT   120         /* mean |d(accel)|/sample summed over axes,
                                       below which the filter does not run */
#define NLMS_P_MIN       64         /* floor on reference power; stops a still
                                       wrist (p ~ 0) from producing a division
                                       blow-up and a garbage weight update */

static uint32_t s_ring[HR_AUTOCORR_WIN];
static int16_t  s_acc[NLMS_AXES][HR_AUTOCORR_WIN];
static uint16_t s_head;             /* next write index                        */
static uint16_t s_count;            /* saturates at HR_AUTOCORR_WIN            */

/* Detrended, scaled working copy. Separate from the ring so feeding can carry
   on (from the FIFO hook) while an estimate is being computed. */
static int16_t s_work[HR_AUTOCORR_WIN];
/* Guards hr_autocorr_last_window: without it a dump taken before any estimate
   would ship a window of stale zeros, which reads as a real flat capture in the
   offline suite rather than as "no data". */
static bool s_work_valid = false;

static void stage_reset(void);      /* defined with the staging state below */
static void track_reset(void);      /* defined with the tracker at the end  */
static void track_age_burst(void);  /* ditto                                */

void hr_autocorr_reset(void)
{
    s_head = 0;
    s_count = 0;
    s_work_valid = false;
    /* The accelerometer staging goes too. A reset means the sensor restarted,
       so comparing the next batch against one from before the restart would
       call a genuine fresh batch stale — or, worse, call a frozen ring fresh
       because the restart happened to change it once. */
    stage_reset();
    /* The TRACKER deliberately survives. This is called at every burst start to
       drop the correlation window — samples from before the LED powered up are
       not a heartbeat — but a heart rate does not reset when the LED goes off.
       Ten minutes ago is excellent prior information for now.
     *
     * Clearing it here cost real readings: the tracker needs TRACK_WARMUP
     * consistent values before it may act, so wiping it every ~10 minutes left
     * the first few windows of EVERY burst unprotected. That is exactly where
     * a 30 bpm window on 2026-08-11 (wrist at 56) and a 32 and a 106 on
     * 2026-08-10 slipped through — all of them at warm < 3.
     *
     * Staleness is handled instead by counting bursts that produce nothing:
     * a watch taken off runs bursts that all refuse, and the baseline expires
     * on its own. */
    track_age_burst();
}

static void push_sample(uint32_t ppg, int16_t ax, int16_t ay, int16_t az)
{
    s_ring[s_head] = ppg;
    s_acc[0][s_head] = ax;
    s_acc[1][s_head] = ay;
    s_acc[2][s_head] = az;
    s_head = (uint16_t)((s_head + 1u) % HR_AUTOCORR_WIN);
    if (s_count < HR_AUTOCORR_WIN) s_count++;
}

void hr_autocorr_feed(uint8_t n, const uint32_t *raw)
{
    if (raw == NULL) return;
    for (uint8_t i = 0; i < n; i++) push_sample(raw[i], 0, 0, 0);
}

/* One PPG batch's worth of accelerometer, handed over by the vendor's own accel
   callback just before the frame hooks for that batch fire. GH30X_FIFO_LEN is
   the vendor's batch ceiling; 32 covers it with room to spare and the pushes are
   bounds-checked anyway, because this is fed from a vendor callback whose count
   we do not control. */
#define ACC_STAGE_MAX 32
static int16_t  s_stage[ACC_STAGE_MAX][NLMS_AXES];
static uint8_t  s_stage_n;          /* samples staged for this batch           */
static uint8_t  s_stage_rd;         /* next one to pair with a PPG frame       */

/* Staleness detection.
 *
 * The accelerometer ring this reads is filled by the IMU data-ready stream, and
 * that stream is not guaranteed to be running: the sleep service deliberately
 * bypasses it in favour of a direct register read, with the comment "survives
 * DARK / hand_tracking-off modes", and this project has an open report of the
 * IMU subscription stopping intermittently while the UI carries on normally.
 *
 * A stopped stream does NOT read as zero. The vendor callback returns the newest
 * N entries of a ring nobody is writing, so every batch is the same frozen copy
 * of whatever the wrist last did — a real movement pattern, with real structure
 * and a large sample-to-sample delta. It would sail through the activity gate
 * and NLMS would subtract an arm swing that stopped happening hours ago.
 *
 * A batch identical to its predecessor is the signature, and on a live 25 Hz
 * 16-bit three-axis sensor it does not otherwise occur. Batches of one sample
 * are exempt: a single triple can repeat by quantisation alone on a still wrist,
 * which is not evidence of anything. */
static int16_t  s_prev_batch[ACC_STAGE_MAX][NLMS_AXES];
static uint8_t  s_prev_batch_n;
static bool     s_batch_stale;
static bool     s_batch_judged;     /* staleness decided once per batch        */
static uint8_t  s_stale_bits[(HR_AUTOCORR_WIN + 7) / 8];

static void mark_stale(uint16_t slot, bool stale)
{
    uint8_t mask = (uint8_t)(1u << (slot & 7u));
    if (stale) s_stale_bits[slot >> 3] |= mask;
    else       s_stale_bits[slot >> 3] &= (uint8_t)~mask;
}

static void judge_batch(void)
{
    s_batch_judged = true;
    s_batch_stale = false;
    if (s_stage_n >= 2 && s_stage_n == s_prev_batch_n)
    {
        s_batch_stale = true;
        for (uint8_t i = 0; i < s_stage_n && s_batch_stale; i++)
            for (int k = 0; k < NLMS_AXES; k++)
                if (s_stage[i][k] != s_prev_batch[i][k]) { s_batch_stale = false; break; }
    }
    for (uint8_t i = 0; i < s_stage_n; i++)
        for (int k = 0; k < NLMS_AXES; k++) s_prev_batch[i][k] = s_stage[i][k];
    s_prev_batch_n = s_stage_n;
}

static void stage_reset(void)
{
    s_stage_n = 0;
    s_stage_rd = 0;
    s_prev_batch_n = 0;
    s_batch_stale = false;
    s_batch_judged = false;
    memset(s_stale_bits, 0, sizeof(s_stale_bits));
}

void hr_autocorr_forget(void)
{
    hr_autocorr_reset();
    track_reset();
}

void hr_autocorr_stage_begin(void)
{
    s_stage_n = 0;
    s_stage_rd = 0;
    s_batch_judged = false;
}

void hr_autocorr_stage_push(int16_t ax, int16_t ay, int16_t az)
{
    if (s_stage_n >= ACC_STAGE_MAX) return;
    s_stage[s_stage_n][0] = ax;
    s_stage[s_stage_n][1] = ay;
    s_stage[s_stage_n][2] = az;
    s_stage_n++;
}

void hr_autocorr_feed_frame(uint32_t ppg)
{
    int16_t ax = 0, ay = 0, az = 0;
    /* Judged here rather than in stage_begin because the batch is only complete
       once the last stage_push has landed, and the driver interleaves neither
       call with the other: it fills the whole batch, then delivers the frames. */
    if (!s_batch_judged) judge_batch();
    /* Every sample carries the staleness of the batch it came from, so the
       window can be judged on what is actually IN it rather than on whatever
       the most recent batch happened to be. */
    mark_stale(s_head, s_stage_n > 0 ? s_batch_stale : true);
    if (s_stage_n > 0)
    {
        /* Hold the last staged sample if the batch delivered fewer accel
           samples than PPG frames. Repeating one sample flattens that stretch of
           the reference, which costs cancellation; inventing a rolling value
           would put structure into the reference that the wrist never made, and
           a reference with invented structure is exactly how an adaptive filter
           learns to subtract the heartbeat. */
        uint8_t i = (s_stage_rd < s_stage_n) ? s_stage_rd : (uint8_t)(s_stage_n - 1);
        ax = s_stage[i][0];
        ay = s_stage[i][1];
        az = s_stage[i][2];
        if (s_stage_rd < s_stage_n) s_stage_rd++;
    }
    push_sample(ppg, ax, ay, az);
}

bool hr_autocorr_accel_stale(void)
{
    for (unsigned i = 0; i < sizeof(s_stale_bits); i++)
        if (s_stale_bits[i]) return true;
    return false;
}

uint32_t hr_autocorr_accel_act(void)
{
    const uint16_t base = s_head;
    uint32_t total = 0;
    for (int k = 0; k < NLMS_AXES; k++)
    {
        for (uint16_t i = 1; i < HR_AUTOCORR_WIN; i++)
        {
            int32_t d = (int32_t)s_acc[k][(base + i) % HR_AUTOCORR_WIN]
                      - (int32_t)s_acc[k][(base + i - 1) % HR_AUTOCORR_WIN];
            total += (uint32_t)(d < 0 ? -d : d);
        }
    }
    return total / (HR_AUTOCORR_WIN - 1);
}

uint16_t hr_autocorr_fill(void)
{
    return s_count;
}

uint16_t hr_autocorr_last_window(int8_t *out, uint16_t max)
{
    if (out == NULL || !s_work_valid) return 0;
    uint16_t n = (max < HR_AUTOCORR_WIN) ? max : HR_AUTOCORR_WIN;
    for (uint16_t i = 0; i < n; i++)
    {
        /* s_work is already normalised to |x| <= SCALE_MAX (1023); >>3 lands it
           in int8 with the shape intact. */
        int32_t v = (int32_t)s_work[i] >> 3;
        if (v > 127) v = 127;
        if (v < -128) v = -128;
        out[i] = (int8_t)v;
    }
    return n;
}

/* Integer sqrt, Newton. Only ever called on correlation energies. */
static uint32_t isqrt32(uint32_t v)
{
    if (v == 0) return 0;
    uint32_t x = v, y = (x + 1u) / 2u;
    while (y < x) { x = y; y = (x + v / x) / 2u; }
    return x;
}

/**
 * Remove DC and linear drift, then scale into int16.
 *
 * A moving-average high-pass would be the obvious choice but its first null
 * sits at 60/window_seconds bpm — right on top of a resting heart rate — so it
 * would attenuate the very component being measured. A least-squares line
 * removes baseline wander without touching the pulse band.
 *
 * @return false if the window is flat (no AC at all).
 */
static int detrend_into_work(void)
{
    const int n = HR_AUTOCORR_WIN;
    /* Oldest sample first: s_head is the next write slot, so it is also the
       oldest entry once the ring has wrapped. */
    const uint16_t base = s_head;

    int64_t sum = 0, sum_ix = 0;
    for (int i = 0; i < n; i++)
    {
        int64_t v = (int64_t)s_ring[(base + i) % HR_AUTOCORR_WIN];
        sum += v;
        sum_ix += v * i;
    }
    /* Least-squares slope/intercept, kept in Q16 so the fit stays exact enough
       over 256 points without floating point. */
    const int64_t si  = (int64_t)n * (n - 1) / 2;
    const int64_t sii = (int64_t)(n - 1) * n * (2 * n - 1) / 6;
    const int64_t den = (int64_t)n * sii - si * si;
    if (den == 0) return 0;
    const int64_t b_q16 = (((int64_t)n * sum_ix - si * sum) << 16) / den;
    const int64_t a_q16 = ((sum << 16) - b_q16 * si) / n;

    /* Two passes over the residual: the scale shift cannot be chosen until the
       largest residual is known, and storing unscaled first would clip int16. */
    int32_t peak = 0;
    for (int i = 0; i < n; i++)
    {
        int64_t fit = (a_q16 + b_q16 * i) >> 16;
        int32_t d = (int32_t)((int64_t)s_ring[(base + i) % HR_AUTOCORR_WIN] - fit);
        int32_t ad = (d < 0) ? -d : d;
        if (ad > peak) peak = ad;
    }
    if (peak == 0) return 0;

    int shift = 0;
    while ((peak >> shift) > SCALE_MAX) shift++;

    for (int i = 0; i < n; i++)
    {
        int64_t fit = (a_q16 + b_q16 * i) >> 16;
        int32_t d = (int32_t)((int64_t)s_ring[(base + i) % HR_AUTOCORR_WIN] - fit);
        s_work[i] = (int16_t)(d >> shift);
    }
    s_work_valid = true;
    return 1;
}

/**
 * Normalised cross-correlation at one lag: sum(x[i]*x[i+l]) divided by the
 * geometric mean of the two overlapping segments' energies.
 *
 * The obvious alternative — dividing by (n-lag) — amplifies noise at long lags,
 * which inflated the 2x/3x peaks and made an earlier draft report 110 bpm as
 * 36.7. Normalising by both segments' own energy has no lag-dependent bias.
 */
static int32_t ncc_q15(int lag)
{
    const int m = HR_AUTOCORR_WIN - lag;
    if (m < MIN_OVERLAP) return 0;

    int32_t num = 0, ea = 0, eb = 0;
    for (int i = 0; i < m; i++)
    {
        int32_t a = s_work[i];
        int32_t b = s_work[i + lag];
        num += a * b;
        ea  += a * a;
        eb  += b * b;
    }
    if (ea <= 0 || eb <= 0) return 0;

    uint32_t sa = isqrt32((uint32_t)ea);
    uint32_t sb = isqrt32((uint32_t)eb);
    if (sa == 0 || sb == 0) return 0;

    int64_t r = ((int64_t)num * Q15) / ((int64_t)sa * (int64_t)sb);
    if (r >  Q15) r =  Q15;
    if (r < -Q15) r = -Q15;
    return (int32_t)r;
}

/* Detrend one accelerometer axis into `out`, scaled like s_work so the NLMS
   arithmetic stays inside int32. Returns 0 when the axis is flat — a still wrist
   or an axis the movement did not touch, which simply contributes nothing. */
static int detrend_axis(int axis, int16_t *out)
{
    const int n = HR_AUTOCORR_WIN;
    const uint16_t base = s_head;
    int64_t sum = 0, sum_ix = 0;
    for (int i = 0; i < n; i++)
    {
        int64_t v = (int64_t)s_acc[axis][(base + i) % HR_AUTOCORR_WIN];
        sum += v;
        sum_ix += v * i;
    }
    const int64_t si  = (int64_t)n * (n - 1) / 2;
    const int64_t sii = (int64_t)(n - 1) * n * (2 * n - 1) / 6;
    const int64_t den = (int64_t)n * sii - si * si;
    if (den == 0) return 0;
    const int64_t b_q16 = (((int64_t)n * sum_ix - si * sum) << 16) / den;
    const int64_t a_q16 = ((sum << 16) - b_q16 * si) / n;

    int32_t peak = 0;
    for (int i = 0; i < n; i++)
    {
        int64_t fit = (a_q16 + b_q16 * i) >> 16;
        int32_t d = (int32_t)((int64_t)s_acc[axis][(base + i) % HR_AUTOCORR_WIN] - fit);
        int32_t ad = (d < 0) ? -d : d;
        if (ad > peak) peak = ad;
    }
    if (peak == 0) return 0;
    int shift = 0;
    while ((peak >> shift) > SCALE_MAX) shift++;
    for (int i = 0; i < n; i++)
    {
        int64_t fit = (a_q16 + b_q16 * i) >> 16;
        int32_t d = (int32_t)((int64_t)s_acc[axis][(base + i) % HR_AUTOCORR_WIN] - fit);
        out[i] = (int16_t)(d >> shift);
    }
    return 1;
}

/**
 * Subtract the accelerometer-predictable part of s_work, in place.
 *
 * e[n] = ppg[n] - w·r[n];  w += mu·e·r / (|r|² + eps), with r the last
 * NLMS_TAPS samples of each axis. Normalising by the reference power is what
 * lets one step size cover both a motionless wrist and vigorous movement.
 *
 * Weights are Q20: the transfer function is small (a ±1023 accel producing a
 * ±1023 optical artefact means |w| ~ 1), so Q20 keeps four bits of headroom and
 * still resolves 1e-6 per tap. The per-tap divide is deliberate over hoisting a
 * shared gain — 48 divides at 25 Hz is ~1200/s, invisible next to the
 * autocorrelation, and hoisting needs another fixed-point scale to get right.
 *
 * A still wrist leaves the reference with no structure, so the weights stay
 * near zero and the residual is the input — which is why this can run
 * unconditionally instead of behind a motion gate whose threshold nobody has
 * data for yet.
 */
static void nlms_cancel(void)
{
    static int16_t ref[NLMS_AXES][HR_AUTOCORR_WIN];
    static int32_t w[NLMS_W];

    /* No motion, no artefact, no filter. The artefact's amplitude scales with
       wrist movement, so below the gate there is nothing to subtract and 48
       adaptive weights can only do harm — measured harm: a sleeping wrist whose
       accelerometer carries nothing but breathing (0.25 Hz) drove the filter
       into reporting 30 bpm for a 60 bpm heart, an exact halving. The gate
       removes that failure at every breathing amplitude tested up to 800 LSB
       while leaving all eight motion cases correct.

       A high-pass on the reference was tried first and rejected with evidence:
       it fixed the mild breathing cases but still halved at 250 LSB, and it cost
       two of the motion cases outright (110 and 95 bpm both went to no-answer) —
       giving up the entire reason the filter exists to fix half of one failure.

       The threshold is mean |first difference| summed over the three axes, which
       weights the fast band the artefact lives in and all but ignores breathing.
       120 sits 4x above the worst still window measured and 13x below the
       weakest motion case: a wide gap, but a SYNTHETIC one, which is why the
       measured value is logged on every estimate — the first night of real wrist
       data is what moves this number, not another simulation. */
    if (hr_autocorr_accel_stale()) return;
    if (hr_autocorr_accel_act() < NLMS_GATE_ACT) return;

    int live = 0;
    for (int k = 0; k < NLMS_AXES; k++)
    {
        if (detrend_axis(k, ref[k])) live = 1;
        else memset(ref[k], 0, sizeof(ref[k]));
    }
    if (!live) return;                       /* no accelerometer signal at all */

    memset(w, 0, sizeof(w));                 /* per-window: weights must not
                                                carry across a burst boundary */
    /* static, not a local: this runs from a soft-timer callback on the LCPU,
       whose thread stack is not observable from the HCPU console, and this
       project has already lost a thread to a 192-byte-class stack overflow.
       Not reentrant, but nothing calls it concurrently — one timer, one core. */
    static int32_t r[NLMS_W];

    for (int n = 0; n < HR_AUTOCORR_WIN; n++)
    {
        int32_t p = 0;
        int j = 0;
        for (int k = 0; k < NLMS_AXES; k++)
            for (int t = 0; t < NLMS_TAPS; t++, j++)
            {
                int32_t v = (n - t >= 0) ? ref[k][n - t] : 0;
                r[j] = v;
                p += v * v;
            }
        if (p < NLMS_P_MIN) p = NLMS_P_MIN;

        int64_t y = 0;
        for (j = 0; j < NLMS_W; j++) y += (int64_t)w[j] * r[j];
        int32_t e = (int32_t)(s_work[n] - (y >> 20));

        for (j = 0; j < NLMS_W; j++)
        {
            /* delta_q20 = mu * e * r / p, carried at Q20 */
            int64_t num = (int64_t)NLMS_MU_Q15 * e * r[j] * 32;
            w[j] += (int32_t)(num / p);
        }
        int32_t clamped = e;
        if (clamped >  SCALE_MAX) clamped =  SCALE_MAX;
        if (clamped < -SCALE_MAX) clamped = -SCALE_MAX;
        s_work[n] = (int16_t)clamped;
    }
}


/* ---------------------------------------------------------------- spectral
 *
 * The autocorrelation refuses whenever the beat-to-beat interval wanders enough
 * that ten seconds of pulse stop lining up with themselves. On the first night
 * of real wrist windows that was 22 of 43 — and 15 of those still carried a
 * value agreeing with their neighbours to within a few bpm. The information is
 * present; what the autocorrelation requires (alignment across the whole
 * window) is what a real wrist fails to supply, and a frequency estimate does
 * not require it.
 *
 * This is deliberately NOT the primary estimator, and must never be promoted to
 * one. A spectrum returns a dominant bin for ANY input, a pure decay included —
 * on its own that is precisely the never-refuses behaviour that had the vendor
 * library reporting 108 bpm from a watch lying on a desk. It runs only where the
 * autocorrelation has already declined, and behind two gates of its own.
 *
 * Bounded by measurement rather than by argument: across all 43 real windows the
 * full chain emits 36 values spanning 42-84 bpm and NOTHING at or above 100,
 * against the 40-151 those same windows actually published. The residual failure
 * mode is "mildly uncertain inside a physiological range", which is a different
 * animal from "fabricated 151".
 *
 * Cost: 93 bins x 256 samples of int64 recurrence, roughly 4x the
 * autocorrelation, and only on the seconds where the autocorrelation refused.
 */
#define SPEC_BPM_LO      30
#define SPEC_BPM_STEP     2
#define SPEC_BINS        93
#define SPEC_SHARP_MIN  200         /* 0.200 in per-mille                      */
#define SPEC_RATIO_MIN  130         /* 1.30 in hundredths                      */

static const int16_t SPEC_COEF_Q13[93] = {
     16255,  16237,  16218,  16198,  16177,  16155,  16131,  16107,
     16081,  16054,  16026,  15997,  15967,  15935,  15903,  15869,
     15835,  15799,  15762,  15724,  15685,  15645,  15603,  15561,
     15517,  15473,  15427,  15380,  15332,  15283,  15233,  15182,
     15130,  15077,  15023,  14968,  14911,  14854,  14795,  14736,
     14675,  14614,  14551,  14488,  14423,  14357,  14291,  14223,
     14155,  14085,  14014,  13943,  13870,  13797,  13722,  13647,
     13570,  13493,  13414,  13335,  13255,  13174,  13092,  13009,
     12925,  12840,  12754,  12668,  12580,  12492,  12403,  12312,
     12221,  12130,  12037,  11943,  11849,  11754,  11658,  11561,
     11463,  11365,  11266,  11165,  11065,  10963,  10861,  10758,
     10654,  10549,  10444,  10337,  10231,
};

static const int16_t HANN_Q15[256] = {
         0,      5,     20,     45,     80,    124,    179,    243,
       317,    401,    495,    598,    711,    833,    965,   1106,
      1257,   1416,   1585,   1763,   1949,   2145,   2349,   2561,
      2782,   3011,   3249,   3494,   3747,   4008,   4276,   4552,
      4834,   5124,   5421,   5724,   6034,   6350,   6672,   7000,
      7334,   7673,   8018,   8367,   8722,   9081,   9444,   9812,
     10184,  10559,  10938,  11321,  11706,  12094,  12485,  12879,
     13274,  13671,  14070,  14470,  14872,  15274,  15677,  16081,
     16484,  16888,  17291,  17694,  18096,  18497,  18897,  19295,
     19691,  20085,  20477,  20867,  21254,  21638,  22019,  22396,
     22770,  23139,  23505,  23866,  24223,  24575,  24922,  25264,
     25601,  25932,  26257,  26576,  26889,  27195,  27495,  27789,
     28075,  28354,  28626,  28891,  29148,  29397,  29638,  29871,
     30096,  30313,  30521,  30721,  30912,  31094,  31267,  31432,
     31587,  31732,  31869,  31996,  32114,  32222,  32320,  32409,
     32488,  32557,  32617,  32666,  32706,  32736,  32756,  32766,
     32766,  32756,  32736,  32706,  32666,  32617,  32557,  32488,
     32409,  32320,  32222,  32114,  31996,  31869,  31732,  31587,
     31432,  31267,  31094,  30912,  30721,  30521,  30313,  30096,
     29871,  29638,  29397,  29148,  28891,  28626,  28354,  28075,
     27789,  27495,  27195,  26889,  26576,  26257,  25932,  25601,
     25264,  24922,  24575,  24223,  23866,  23505,  23139,  22770,
     22396,  22019,  21638,  21254,  20867,  20477,  20085,  19691,
     19295,  18897,  18497,  18096,  17694,  17291,  16888,  16484,
     16081,  15677,  15274,  14872,  14470,  14070,  13671,  13274,
     12879,  12485,  12094,  11706,  11321,  10938,  10559,  10184,
      9812,   9444,   9081,   8722,   8367,   8018,   7673,   7334,
      7000,   6672,   6350,   6034,   5724,   5421,   5124,   4834,
      4552,   4276,   4008,   3747,   3494,   3249,   3011,   2782,
      2561,   2349,   2145,   1949,   1763,   1585,   1416,   1257,
      1106,    965,    833,    711,    598,    495,    401,    317,
       243,    179,    124,     80,     45,     20,      5,      0,
};

static uint32_t isqrt64(uint64_t v)
{
    uint64_t r = 0, bit = 1ULL << 62;
    while (bit > v) bit >>= 2;
    while (bit)
    {
        if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return (uint32_t)r;
}

/* Goertzel magnitude of s_work at one bin, Hann-windowed. */
static uint32_t spec_mag(int bin)
{
    const int32_t c = SPEC_COEF_Q13[bin];
    int64_t s1 = 0, s2 = 0;
    for (int i = 0; i < HR_AUTOCORR_WIN; i++)
    {
        int32_t xw = (int32_t)(((int32_t)s_work[i] * HANN_Q15[i]) >> 15);
        int64_t s = (int64_t)xw + ((c * s1) >> 13) - s2;
        s2 = s1;
        s1 = s;
    }
    int64_t p = s1 * s1 + s2 * s2 - ((c * s1 * s2) >> 13);
    if (p < 0) p = 0;
    return isqrt64((uint64_t)p);
}

static uint8_t spectral_estimate(uint8_t *conf_out)
{
    static uint32_t mag[SPEC_BINS];
    uint64_t total = 0;
    int pk = 0;
    for (int i = 0; i < SPEC_BINS; i++)
    {
        mag[i] = spec_mag(i);
        total += mag[i];
        if (mag[i] > mag[pk]) pk = i;
    }
    if (total == 0) return 0;

    int32_t pk_bpm = SPEC_BPM_LO + pk * SPEC_BPM_STEP;
    int32_t lo = (pk_bpm * 92) / 100, hi = (pk_bpm * 108) / 100;

    uint64_t skirt = 0;
    uint32_t comp = 0;
    for (int i = 0; i < SPEC_BINS; i++)
    {
        int32_t b = SPEC_BPM_LO + i * SPEC_BPM_STEP;
        if (b >= lo && b <= hi) skirt += mag[i];
        else if (mag[i] > comp) comp = mag[i];
    }
    /* Sharp: a real pulse is a line, wideband noise is a hump. Ratio: the peak
       must also beat everything outside its own skirt, which is what stops a
       two-humped spectrum from being read as a confident single rate. */
    uint32_t sharp = (uint32_t)((skirt * 1000) / total);
    uint32_t ratio = (comp > 0) ? (uint32_t)(((uint64_t)mag[pk] * 100) / comp) : 9900;
    if (sharp < SPEC_SHARP_MIN || ratio < SPEC_RATIO_MIN) return 0;

    /* Capped below what the autocorrelation reports. This is the weaker of the
       two estimators and downstream must be able to prefer the stronger one
       without any extra plumbing. */
    uint32_t conf = sharp / 5;
    if (conf > 60) conf = 60;
    if (conf_out) *conf_out = (uint8_t)conf;
    return (uint8_t)pk_bpm;
}

static uint8_t estimate_raw(uint8_t *conf_out)
{
    if (conf_out) *conf_out = 0;
    if (s_count < HR_AUTOCORR_WIN) return 0;
    if (!detrend_into_work()) return 0;
    nlms_cancel();

    int32_t r[HR_AUTOCORR_LAG_MAX + 2];
    for (int l = 2; l <= HR_AUTOCORR_LAG_MAX + 1; l++)
        r[l] = ncc_q15(l);

    /* Find where the zero-lag main lobe ends, and refuse the window if it never
       does. This is the single biggest source of wrong readings and it went
       unseen for four nights because nothing here could tell a periodic signal
       from a smooth one.
     *
     * Correlation starts at 1.0 by definition and decays. On a signal with a
     * real pulse it decays THROUGH a trough and comes back up at the period —
     * a captured window that read correctly at 55 bpm dips to -0.49 at lag 14
     * before peaking at 0.76 at lag 27. On a window with no pulse in it, it just
     * decays: one that this code called 151 bpm falls monotonically from 0.59 at
     * lag 7 to -0.14 at lag 50, with no peak anywhere. The old search took the
     * highest local maximum of that decay and reported it with confidence 59,
     * because confidence was peak HEIGHT, and the shoulder of the main lobe is
     * genuinely high — 0.84 on one of the worst windows. No confidence threshold
     * could ever have separated those, which is why raising it never helped.
     *
     * Measured on 43 real captured windows (2026-08-08, first night the capture
     * reached the phone): 21 were wrong. Searching only past the trough leaves 2
     * wrong and 22 refused. Refusing is the right answer for those — the window
     * genuinely has no measurable pulse, and a gap in the curve is worth far
     * more than a fabricated 151.
     *
     * The FIRST version of this test used an absolute level (r <= 0.30). It
     * shipped, and it halved 38 of the NEXT night's 59 windows: when the
     * half-period dip happens not to reach 0.30, the search skips straight past
     * the fundamental and lands on the peak at TWICE the period — 76 bpm read as
     * 38, 66 as 33, 58 as 30, over and over, all night. A turning point does not
     * care how deep the dip is, and depth was never the thing worth testing. The
     * question was always "does this curve come back up at all", and the
     * monotone decay is exactly the curve that never does.
     *
     * The 0.15 rise separates a real turning point from noise wobble on the way
     * down. Over both nights (102 real windows): absolute-0.30 scores 54 correct
     * / 39 wrong; turning-point-0.15 scores 88 correct / 2 wrong. */
    int trough = 0;
    for (int l = 3; l < HR_AUTOCORR_LAG_MAX; l++)
    {
        if (r[l] > r[l - 1] || r[l] >= r[l + 1]) continue;   /* not a minimum */
        int32_t top = r[l + 1];
        for (int m = l + 2; m <= HR_AUTOCORR_LAG_MAX; m++)
            if (r[m] > top) top = r[m];
        if (top - r[l] >= TROUGH_RISE_Q15) { trough = l; break; }
    }
    if (trough == 0) return spectral_estimate(conf_out);

    int lag_lo = (trough > HR_AUTOCORR_LAG_MIN) ? trough : HR_AUTOCORR_LAG_MIN;

    /* Collect local maxima above threshold, with the parabolic vertex of each.
       Interpolating BEFORE comparing matters: a lag of 13.6 samples sits between
       grid points while its 2x/3x multiples can land nearer integers, so raw
       r[] favours the multiples purely from discretisation. */
    int32_t peak_lag_q8[MAX_PEAKS];
    int32_t peak_h[MAX_PEAKS];
    int npk = 0;

    for (int l = lag_lo; l <= HR_AUTOCORR_LAG_MAX && npk < MAX_PEAKS; l++)
    {
        if (r[l] <= THR_Q15) continue;
        if (r[l] < r[l - 1] || r[l] < r[l + 1]) continue;

        int32_t a = r[l - 1], b = r[l], c = r[l + 1];
        int32_t d_q8 = 0, h = b;
        int32_t den = a - 2 * b + c;
        if (den != 0)
        {
            d_q8 = ((a - c) * 128) / den;              /* (a-c)/(2*den) in Q8 */
            if (d_q8 >  128) d_q8 =  128;              /* vertex must stay in */
            if (d_q8 < -128) d_q8 = -128;              /* this bin            */
            h = b - (((a - c) * d_q8) >> 10);          /* b - 0.25*(a-c)*d    */
        }
        peak_lag_q8[npk] = (l << 8) + d_q8;
        peak_h[npk] = h;
        npk++;
    }
    if (npk == 0) return spectral_estimate(conf_out);

    int32_t hmax = peak_h[0];
    for (int i = 1; i < npk; i++) if (peak_h[i] > hmax) hmax = peak_h[i];

    /* Peaks are collected in ascending lag, so the first one within tolerance of
       the best IS the shortest tied lag = the fundamental. */
    for (int i = 0; i < npk; i++)
    {
        if (peak_h[i] < hmax - TOL_Q15) continue;

        int32_t lag_q8 = peak_lag_q8[i];
        if (lag_q8 <= 0) return 0;
        /* bpm = 60*FS/lag = (60*FS<<8)/lag_q8 */
        int32_t bpm = (60 * HR_AUTOCORR_FS * 256) / lag_q8;
        if (bpm < 30 || bpm > 220) return spectral_estimate(conf_out);

        int32_t conf = (peak_h[i] * 100) / Q15;
        if (conf < 0) conf = 0;
        if (conf > 100) conf = 100;
        if (conf_out) *conf_out = (uint8_t)conf;
        return (uint8_t)bpm;
    }
    return 0;
}


/* ------------------------------------------------------------------ tracking
 *
 * The octave ambiguity cannot be resolved inside a single window, and that is a
 * property of the data, not a shortcoming of one method. All three independent
 * estimators tried on this material fail on it, in different directions:
 *
 *   autocorrelation — correlates equally at T, 2T, 3T
 *   spectrum        — on a weak pulse the sub-harmonic outranks the fundamental
 *   pulse counting  — counts the dicrotic notch as a beat and doubles the rate
 *
 * Continuity resolves it. A heart cannot go 76 -> 34 -> 94 in twenty minutes at
 * rest; an octave error can, and did, all of one night. Carrying a state
 * estimate across windows instead of judging each one alone is what commercial
 * trackers do, and it is the piece this estimator was missing.
 *
 * Deliberately minimal. It only ever substitutes exactly double or exactly half,
 * and only when that lands near an already-established baseline. A genuine
 * change — waking, standing, exercise — is neither, so it passes through
 * untouched. This must never grow into a plausibility clamp: suppressing
 * "unusual" readings would hide the very events an HR curve exists to show.
 *
 * Measured over 102 real windows from two nights: it alters 5 of them and takes
 * the octave-sized step count from 3 to 0, tightening the range from 30-130 to
 * 35-88. It corrects both directions (130 -> 65 and 34 -> 68).
 */
#define TRACK_NEAR_PCT   25         /* within this of baseline: leave alone     */
#define TRACK_SNAP_PCT   20         /* an octave shift must land this close     */
#define TRACK_WARMUP      3         /* consistent readings before it may act    */
#define TRACK_MISS_MAX   40         /* refusals that expire the baseline        */

static uint16_t s_track_base_q4;    /* Q4 so the /4 EMA does not quantise away  */
static uint8_t  s_track_warm;
static uint8_t  s_track_miss;

#define TRACK_HIST_N     16         /* median window for the baseline           */
#define TRACK_RUN_N       4         /* consecutive readings on a new level that
                                       re-establish it instead of being fought  */
#define TRACK_DRY_MAX     6         /* ~1 hour of bursts before the baseline is
                                       considered too old to trust              */

/* Two properties matter more than the octave correction itself, and both were
 * learned by breaking them.
 *
 * BASELINE IS A MEDIAN, NOT AN EMA. An EMA has no memory of how MANY readings
 * disagreed — six in a row move it most of the way. On 2026-08-13 04:14 the
 * watch reported 39 bpm on a window whose spectrum AND autocorrelation both say
 * 76-79, because the baseline had been dragged to ~41 while the wrist sat at 56.
 * That bound is derivable from the output alone: for 39 to be chosen the
 * baseline had to lie between 32.5 and 48.75.
 *
 * A SUSTAINED CHANGE IS NOT AN OCTAVE ERROR. An octave slip is isolated — the
 * estimator flips for one window and comes back. A real change persists and
 * keeps going. Without this distinction the tracker pinned exercise to rest:
 * fed a genuine 60 -> 120 it returned 60 for every reading, indefinitely,
 * because 120/2 lands exactly on the resting baseline and each suppressed value
 * fed the history that justified suppressing the next. The symmetric half is
 * just as necessary — without it the END of a workout is DOUBLED back up to the
 * exercising rate. */
static uint8_t  s_track_hist[TRACK_HIST_N];
static uint8_t  s_track_n;          /* entries used                            */
static uint8_t  s_track_w;          /* next write index (ring)                 */
static uint8_t  s_track_warm;
static uint8_t  s_track_miss;
static uint8_t  s_track_dry;
static uint8_t  s_track_run;        /* consecutive readings off the baseline   */
static int8_t   s_track_dir;        /* which side they are on                  */

static void track_reset(void)
{
    s_track_n = 0;
    s_track_w = 0;
    s_track_warm = 0;
    s_track_miss = 0;
    s_track_dry = 0;
    s_track_run = 0;
    s_track_dir = 0;
}

static int32_t track_base(void)
{
    if (s_track_n == 0) return 0;
    uint8_t tmp[TRACK_HIST_N];
    memcpy(tmp, s_track_hist, s_track_n);
    for (uint8_t i = 1; i < s_track_n; i++)      /* insertion sort, n <= 16 */
    {
        uint8_t v = tmp[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = v;
    }
    return tmp[s_track_n / 2];
}

static void track_push(uint8_t v)
{
    s_track_hist[s_track_w] = v;
    s_track_w = (uint8_t)((s_track_w + 1) % TRACK_HIST_N);
    if (s_track_n < TRACK_HIST_N) s_track_n++;
}

/* Called once per burst boundary. The baseline survives a burst, but not an
   indefinite run of bursts that produce nothing — that is what a watch sitting
   on a desk looks like, and its owner's resting rate from an hour ago is no
   longer a safe prior for whoever picks it up next. */
static void track_age_burst(void)
{
    if (s_track_n == 0) return;
    if (s_track_dry < 255) s_track_dry++;
    if (s_track_dry > TRACK_DRY_MAX) track_reset();
}

static uint8_t track_apply(uint8_t bpm)
{
    if (bpm == 0)
    {
        if (s_track_miss < 255) s_track_miss++;
        if (s_track_miss > TRACK_MISS_MAX) track_reset();
        return 0;
    }
    s_track_miss = 0;
    s_track_dry = 0;

    int32_t out = bpm;
    int32_t base = track_base();

    int8_t dir = 0;
    if (base > 0)
    {
        if (out * 100 > base * (100 + TRACK_NEAR_PCT))      dir =  1;
        else if (out * 100 < base * (100 - TRACK_NEAR_PCT)) dir = -1;
    }
    if (dir != 0 && dir == s_track_dir) { if (s_track_run < 255) s_track_run++; }
    else                                  s_track_run = (dir != 0) ? 1 : 0;
    s_track_dir = dir;

    if (base > 0 && s_track_warm >= TRACK_WARMUP && dir != 0)
    {
        int32_t cand[2] = { out * 2, out / 2 };
        for (int i = 0; i < 2; i++)
        {
            if (cand[i] < 30 || cand[i] > 220) continue;
            int32_t cd = cand[i] > base ? cand[i] - base : base - cand[i];
            if (cd * 100 > base * TRACK_SNAP_PCT) continue;
            if (s_track_run >= TRACK_RUN_N) break;  /* sustained = real change */
            out = cand[i];
            break;
        }
    }

    if (base == 0) s_track_warm = 1;
    else
    {
        int32_t d = out > base ? out - base : base - out;
        s_track_warm = (d * 100 <= base * TRACK_NEAR_PCT)
                     ? (uint8_t)(s_track_warm < 255 ? s_track_warm + 1 : 255)
                     : 1;
    }

    if (s_track_run >= TRACK_RUN_N)
    {
        s_track_n = 0; s_track_w = 0;           /* re-establish, do not fight */
        track_push((uint8_t)out);
        s_track_warm = TRACK_WARMUP;
        s_track_run = 0; s_track_dir = 0;
    }
    else track_push((uint8_t)out);

    return (uint8_t)out;
}

uint8_t hr_autocorr_estimate(uint8_t *conf_out)
{
    return track_apply(estimate_raw(conf_out));
}
