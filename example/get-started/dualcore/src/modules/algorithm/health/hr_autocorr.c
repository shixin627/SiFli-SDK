#include "hr_autocorr.h"
#include <string.h>
#include <stdbool.h>

/* Correlation values are Q15 (32768 == 1.0); lags are Q8. Integer throughout —
   the LCPU's float support is not worth depending on for ~11k operations that
   run once a second. */
#define Q15                 32768
#define THR_Q15             11469   /* 0.35 — below this the window is noise   */
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

void hr_autocorr_reset(void)
{
    s_head = 0;
    s_count = 0;
    s_work_valid = false;
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

void hr_autocorr_feed_frame(uint32_t ppg, int16_t ax, int16_t ay, int16_t az)
{
    push_sample(ppg, ax, ay, az);
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

uint8_t hr_autocorr_estimate(uint8_t *conf_out)
{
    if (conf_out) *conf_out = 0;
    if (s_count < HR_AUTOCORR_WIN) return 0;
    if (!detrend_into_work()) return 0;
    nlms_cancel();

    int32_t r[HR_AUTOCORR_LAG_MAX + 2];
    for (int l = HR_AUTOCORR_LAG_MIN - 1; l <= HR_AUTOCORR_LAG_MAX + 1; l++)
        r[l] = ncc_q15(l);

    /* Collect local maxima above threshold, with the parabolic vertex of each.
       Interpolating BEFORE comparing matters: a lag of 13.6 samples sits between
       grid points while its 2x/3x multiples can land nearer integers, so raw
       r[] favours the multiples purely from discretisation. */
    int32_t peak_lag_q8[MAX_PEAKS];
    int32_t peak_h[MAX_PEAKS];
    int npk = 0;

    for (int l = HR_AUTOCORR_LAG_MIN; l <= HR_AUTOCORR_LAG_MAX && npk < MAX_PEAKS; l++)
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
    if (npk == 0) return 0;

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
        if (bpm < 30 || bpm > 220) return 0;

        int32_t conf = (peak_h[i] * 100) / Q15;
        if (conf < 0) conf = 0;
        if (conf > 100) conf = 100;
        if (conf_out) *conf_out = (uint8_t)conf;
        return (uint8_t)bpm;
    }
    return 0;
}
