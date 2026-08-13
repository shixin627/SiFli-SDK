# -*- coding: utf-8 -*-
"""Reference implementation + validation suite for hr_autocorr.c.

Run:  py -3 hr_autocorr_test.py

This is the spec the C must satisfy. The algorithm was DESIGNED here, not
documented here afterwards: three earlier drafts passed every harmonic case and
still reported a genuine 110 bpm as 36.7, and each of the four rules below
exists because a specific case failed without it. They are listed with the
failure they fix so nobody "simplifies" one away.

  1. Normalise by both segments' energy, not by (n-lag).
     (n-lag) amplifies long-lag noise, inflating the 2x/3x peaks. Cost of
     omitting: REAL 110 -> 36.7, REAL 130 -> 65.

  2. Interpolate the parabolic vertex BEFORE comparing peaks.
     A 13.64-sample lag sits between grid points while its multiples can land
     nearer integers, so raw r[] favours the multiples by pure discretisation.
     Measured: r[14]=0.9785 vs r[41]=0.9973 for a true 110 bpm.

  3. Among peaks tied within TOL, take the SHORTEST lag.
     A periodic signal correlates ~equally at T, 2T, 3T..., so "strongest peak"
     is decided by numerical noise (the three heights above differ by 0.2%).
     The fundamental is by definition the shortest period. This single rule
     replaced an octave-promotion loop that could not be tuned to satisfy both
     the harmonic cases and the genuine-high-HR cases at once.

  4. Refuse below THR.
     The vendor library never refuses — it reported a steady 108-115 bpm from a
     watch lying on a desk with no pulse at all. Refusing is the feature.

  0. Search only PAST the first trough, and refuse a window that has none.
     Added last, and it matters more than the other four put together. Rules 1-4
     all assume the correlation curve HAS peaks; none of them checks. A smooth
     signal with no pulse produces a plain decay whose shoulder is genuinely
     tall (0.84 measured), so "highest local maximum" returns a confident
     number about nothing. On the first night of real wrist windows this was 21
     of 43 readings. No confidence threshold can separate these — confidence IS
     peak height — which is why raising it never helped in four attempts.

Everything above rule 0 was validated only against synthetic cases, and those
have never predicted a single real failure: a nine-combination sweep passed 81
of them while the wrist produced four wrong readings a night. The real windows
in hr_autocorr_real.csv are the part of this suite with predictive power.
"""
import math
import io
import os
import random

FS = 25.0
WIN = 256
LAG_MIN, LAG_MAX = 7, 50          # 214 .. 30 bpm
THR = 0.35
TOL = 0.06
MIN_OVERLAP = 40


def detrend(x):
    """Remove DC + linear drift. A moving-average high-pass would put its first
    null on top of a resting heart rate, so a least-squares line is used."""
    n = len(x)
    sx = sum(x)
    sxx = sum(i * x[i] for i in range(n))
    si = n * (n - 1) / 2.0
    sii = (n - 1) * n * (2 * n - 1) / 6.0
    den = n * sii - si * si
    if den == 0:
        return None
    b = (n * sxx - si * sx) / den
    a = (sx - b * si) / n
    return [x[i] - (a + b * i) for i in range(n)]


def ncc(x):
    """Normalised cross-correlation — rule 1."""
    n = len(x)
    r = [0.0] * (LAG_MAX + 3)
    for l in range(LAG_MAX + 3):
        m = n - l
        if m < MIN_OVERLAP:
            continue
        num = ea = eb = 0.0
        for i in range(m):
            a, b = x[i], x[i + l]
            num += a * b
            ea += a * a
            eb += b * b
        d = math.sqrt(ea * eb)
        r[l] = num / d if d > 0 else 0.0
    return r


def vertex(r, l):
    """Parabolic vertex (lag, height) — rule 2."""
    a, b, c = r[l - 1], r[l], r[l + 1]
    den = a - 2 * b + c
    if den == 0:
        return float(l), b
    d = (a - c) / (2 * den)
    return l + d, b - 0.25 * (a - c) * d


TROUGH_RISE = 0.15  # how far the correlation must climb back after its first
                    # turning point before that turning point is believed


def estimate_detrended(x):
    """The rules, applied to an already-detrended window."""
    r = ncc(x)

    # Rule 0, and the one that matters most: find where the zero-lag main lobe
    # ends — by TURNING POINT, not by absolute level — and refuse the window if
    # it never turns.
    #
    # Correlation starts at 1.0 and decays. With a real pulse it decays THROUGH
    # a trough and returns at the period — a real captured window reading 55 bpm
    # dips to -0.49 at lag 14 before peaking 0.76 at lag 27. With no pulse it
    # merely decays: a window this code once called 151 bpm falls monotonically
    # from 0.59 at lag 7 to -0.14 at lag 50, no peak anywhere. Taking the highest
    # local maximum of a decay reports the shoulder of the main lobe, which is
    # genuinely tall (0.84 on the worst window) — so no confidence threshold
    # could ever separate the two, and none ever did.
    #
    # Measured on the 43 windows captured on 2026-08-08: 21 were wrong before
    # this rule, 1 after. Refusing is the correct answer for the rest — the
    # window has no measurable pulse, and a gap beats a fabricated 151.
    #
    # The FIRST version of this rule used an absolute level (r <= 0.30), and it
    # shipped, and it halved 38 of the next night's 59 windows. When the
    # half-period dip happens not to reach 0.30 the search skips straight past
    # the fundamental and lands on the peak at TWICE the period: 76 bpm read as
    # 38, 66 as 33, 58 as 30, over and over. A turning point does not care how
    # deep the dip is — and depth was never the thing worth testing. The real
    # question was always "does this curve come back up at all", and a monotone
    # decay is precisely the curve that never does.
    #
    # The 0.15 rise separates a real turning point from noise wobble on the way
    # down. Over both nights (102 real windows): absolute-0.30 scores 54 correct
    # / 39 wrong, turning-point-0.15 scores 88 correct / 2 wrong.
    trough = None
    for l in range(3, LAG_MAX):
        if r[l] <= r[l - 1] and r[l] < r[l + 1]:
            if max(r[l + 1:LAG_MAX + 1]) - r[l] >= TROUGH_RISE:
                trough = l
                break
    if trough is None:
        return None, 0
    lo = max(trough, LAG_MIN)

    peaks = [l for l in range(lo, LAG_MAX + 1)
             if r[l] > THR and r[l] >= r[l - 1] and r[l] >= r[l + 1]]
    if not peaks:
        return None, 0                                   # rule 4
    v = {l: vertex(r, l) for l in peaks}
    hmax = max(v[l][1] for l in peaks)
    for l in sorted(peaks):                              # rule 3: shortest wins
        if v[l][1] >= hmax - TOL:
            lag, h = v[l]
            return 60.0 * FS / lag, int(max(0, min(100, h * 100)))
    return None, 0


# ---------------------------------------------------------------- spectral
# The autocorrelation refuses whenever the beat-to-beat interval wanders enough
# that ten seconds of pulse no longer line up with themselves. Measured on the
# first night of real windows, that is 22 of 43 — and 15 of those 22 still carry
# a value that agrees with their neighbours. The information is there; what the
# autocorrelation needs (alignment across the whole window) is what the wrist
# fails to provide, and a frequency estimate does not need it.
#
# Deliberately NOT the primary estimator. A spectrum returns a dominant bin for
# any input whatsoever, including a pure decay, so on its own it is exactly the
# "never refuses" behaviour that made the vendor library report 108 bpm from a
# watch on a desk. It runs only where the autocorrelation has already declined,
# and behind two gates of its own.
SPEC_BPM_LO, SPEC_BPM_HI, SPEC_BPM_STEP = 30, 214, 2
# 2 bpm bins, not 1. The C runs this on the LCPU with an int64 Goertzel
# recurrence per sample per bin; halving the bin count halves a cost that is
# already ~4x the autocorrelation's. At 60 bpm the +-8% sharpness band still
# spans five bins, which is what the metric needs.
SPEC_SHARP_MIN = 0.20   # share of band energy within +-8% of the peak
SPEC_RATIO_MIN = 1.30   # peak vs the tallest competitor outside its own skirt


def hann(n):
    return [0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1)) for i in range(n)]


def spectrum(x):
    """Magnitude at 1 bpm resolution over the search band."""
    n = len(x)
    m = sum(x) / n
    w = hann(n)
    xs = [(x[i] - m) * w[i] for i in range(n)]
    out = []
    for bpm in range(SPEC_BPM_LO, SPEC_BPM_HI + 1, SPEC_BPM_STEP):
        f = bpm / 60.0
        c = 2.0 * math.cos(2 * math.pi * f / FS)
        s1 = s2 = 0.0
        for v in xs:                       # Goertzel — same recurrence as the C
            s = v + c * s1 - s2
            s2, s1 = s1, s
        out.append((math.sqrt(max(0.0, s1 * s1 + s2 * s2 - c * s1 * s2)), bpm))
    return out


def spectral_estimate(x):
    """(bpm, confidence) or (None, 0). Gates are measured, see the module doc."""
    out = spectrum(x)
    tot = sum(a for a, _ in out)
    if tot <= 0:
        return None, 0
    pk_a, pk_b = max(out)
    lo, hi = pk_b * 0.92, pk_b * 1.08
    sharp = sum(a for a, b in out if lo <= b <= hi) / tot
    comp = max([a for a, b in out if not (lo <= b <= hi)] or [0.0])
    ratio = (pk_a / comp) if comp > 0 else 99.0
    if sharp < SPEC_SHARP_MIN or ratio < SPEC_RATIO_MIN:
        return None, 0
    # Confidence is deliberately capped below what the autocorrelation reports:
    # this is the weaker of the two estimators and downstream gates should be
    # able to prefer an autocorrelation answer without extra plumbing.
    return float(pk_b), min(60, int(sharp * 200))


# ------------------------------------------------------------------ tracking
# The octave ambiguity cannot be resolved inside a single window, and that is a
# property of the data rather than of any one method. All three independent
# estimators tried here fail on it, in different directions:
#
#   autocorrelation — correlates equally at T, 2T, 3T
#   spectrum        — on a weak pulse the sub-harmonic can outrank the fundamental
#   pulse counting  — counts the dicrotic notch as a beat and doubles the rate
#
# What DOES resolve it is continuity. A heart cannot go 76 -> 34 -> 94 in twenty
# minutes at rest; an octave error can. This is why commercial trackers carry a
# state estimate across windows instead of judging each one alone, and it is the
# piece this estimator was missing.
#
# Deliberately minimal. It only ever replaces a value with exactly its double or
# its half, and only when that lands near an already-established baseline. A
# genuine change — waking, standing up, exercise — is neither double nor half of
# the baseline, so it passes through untouched. This is not a plausibility clamp
# and must not become one: refusing "unusual" readings would hide exactly the
# events an HR curve exists to show.
TRACK_NEAR_PCT = 25       # within this of the baseline, leave the estimate alone
TRACK_SNAP_PCT = 20       # an octave shift must land this close to be applied
TRACK_WARMUP = 3        # consistent readings needed before the baseline may act
TRACK_MISS_MAX = 40     # consecutive no-answers that expire the baseline


TRACK_HIST_N = 16       # median window for the baseline
TRACK_RUN_N  = 4        # consecutive readings on a new level that re-establish it


class Tracker:
    """Mirrors the C exactly, INCLUDING its fixed-point arithmetic.

    Two properties matter more than the octave correction itself, and both were
    learned by breaking them:

    BASELINE IS A MEDIAN, NOT AN EMA. An EMA has no memory of how many readings
    disagreed — six in a row move it most of the way. On 2026-08-13 04:14 the
    watch reported 39 bpm on a window whose spectrum AND autocorrelation both
    say 76-79, because the baseline had been dragged to ~41 while the wrist sat
    at 56. The bound is derivable from the output alone: for 39 to be chosen the
    baseline had to be between 32.5 and 48.75.

    A SUSTAINED CHANGE IS NOT AN OCTAVE ERROR. An octave slip is isolated — the
    estimator flips for a window and comes back. A real change persists and
    keeps going. Without this the tracker pinned exercise to rest: fed a genuine
    60 -> 120 it returned 60 for every single reading, forever, because 120/2
    lands exactly on the resting baseline and each suppressed value fed the
    history that justified suppressing the next. Sixteen readings in, still 60.
    The symmetric half matters just as much — without it the END of a workout
    gets DOUBLED back up to the exercising rate.
    """

    TRACK_DRY_MAX = 6

    def __init__(self):
        self.hist = []
        self.warm = 0
        self.miss = 0
        self.dry = 0
        self.run = 0
        self.dir = 0

    def base(self):
        if not self.hist:
            return 0
        s = sorted(self.hist)
        return s[len(s) // 2]

    def burst_boundary(self):
        """Called where the firmware calls hr_autocorr_reset(): once per burst.

        The correlation window is dropped there — samples from before the LED
        powered up are not a heartbeat — but the BASELINE survives. A heart rate
        does not reset when the LED goes off. Clearing it here is what let a
        30 bpm window through on 2026-08-11 and a 32 and a 106 on 2026-08-10:
        the tracker needs TRACK_WARMUP consistent values before it may act, so
        wiping it every ~10 minutes left the first windows of EVERY burst
        unprotected.
        """
        if not self.hist:
            return
        self.dry += 1
        if self.dry > self.TRACK_DRY_MAX:      # a watch on a desk, not a wrist
            self.__init__()

    def feed(self, bpm):
        """Returns the corrected bpm. None input is a refusal and ages the state."""
        if bpm is None:
            self.miss += 1
            if self.miss > TRACK_MISS_MAX:
                self.__init__()
            return None
        self.miss = 0
        self.dry = 0

        out = int(bpm)
        b = self.base()

        d = 0
        if b > 0:
            if out * 100 > b * (100 + TRACK_NEAR_PCT):
                d = 1
            elif out * 100 < b * (100 - TRACK_NEAR_PCT):
                d = -1
        self.run = (self.run + 1) if (d != 0 and d == self.dir) else (1 if d != 0 else 0)
        self.dir = d

        if b > 0 and self.warm >= TRACK_WARMUP and d != 0:
            for cand in (out * 2, out // 2):
                if not (30 <= cand <= 220):
                    continue
                if abs(cand - b) * 100 > b * TRACK_SNAP_PCT:
                    continue
                if self.run >= TRACK_RUN_N:
                    break          # sustained: a real change, not an octave slip
                out = cand
                break

        if b == 0:
            self.warm = 1
        else:
            self.warm = (self.warm + 1) if abs(out - b) * 100 <= b * TRACK_NEAR_PCT else 1

        if self.run >= TRACK_RUN_N:
            self.hist = [out]      # re-establish rather than fight it
            self.warm = TRACK_WARMUP
            self.run = 0
            self.dir = 0
        else:
            self.hist.append(out)
            if len(self.hist) > TRACK_HIST_N:
                self.hist.pop(0)
        return out


def estimate(raw):
    x = detrend(raw)
    if x is None:
        return None, 0
    e, c = estimate_detrended(x)
    if e is not None:
        return e, c
    return spectral_estimate(x)


NLMS_TAPS = 16          # per axis, 3 axes -> 48 weights
NLMS_MU = 0.35          # step size; 0.35 converges inside one 10 s window
NLMS_EPS = 1e-6


def nlms(ppg, refs, taps=NLMS_TAPS, mu=NLMS_MU):
    """Subtract the accelerometer-predictable part of the PPG.

    Wrist movement couples into the optical path, so the artefact is a filtered
    copy of the accelerometer — unknown transfer function, but LINEAR enough for
    an adaptive filter to learn it online. e[n] = ppg[n] - w·r[n], then
    w += mu·e·r/(|r|²+eps). Normalising by the reference power is what makes the
    step size safe across the enormous dynamic range between lying still and
    brushing your teeth.

    Chosen over TROIKA/JOSS deliberately: those need sparse spectral recovery
    (an L1 solver) and will not fit an LCPU, while the literature finds plain
    LMS/NLMS outperforms the heavier alternatives for this job anyway.
    """
    k = len(refs)
    w = [0.0] * (k * taps)
    out = []
    for n in range(len(ppg)):
        r = []
        for c in refs:
            for t in range(taps):
                r.append(c[n - t] if n - t >= 0 else 0.0)
        y = sum(w[j] * r[j] for j in range(k * taps))
        e = ppg[n] - y
        g = mu * e / (sum(v * v for v in r) + NLMS_EPS)
        for j in range(k * taps):
            w[j] += g * r[j]
        out.append(e)
    return out


NLMS_GATE_ACT = 120     # mean |d(accel)|/sample summed over axes


def accel_activity(ax, ay, az):
    """Wrist activity: mean |first difference| per sample, summed over axes.

    The first difference is what makes this a usable gate. Breathing moves the
    wrist a long way but slowly, so it carries large amplitude and almost no
    sample-to-sample change; real movement is the opposite. Measured separation
    on the suite: still windows top out at 31, the weakest motion case is 1620.
    """
    tot = 0.0
    for c in (ax, ay, az):
        tot += sum(abs(c[i] - c[i - 1]) for i in range(1, len(c))) / (len(c) - 1)
    return tot


def estimate_with_motion(ppg, ax, ay, az):
    """Full pipeline: detrend, NLMS against the 3 accel axes, then estimate.

    NLMS runs only above NLMS_GATE_ACT. Without the gate a sleeping wrist —
    whose accelerometer carries breathing and nothing else — drove the filter
    into halving a 60 bpm heart to 30. Nothing about that window looks wrong
    from the inside: the confidence stays high, because 30 bpm genuinely is the
    dominant period of what the filter handed back.

    No coincidence-rejection stage. It was tried and dropped: a periodic wrist
    has autocorrelation peaks at EVERY multiple of its period, so rejecting them
    all blankets the lag range — it turned a correct 62 bpm into 31. Once NLMS
    has subtracted the motion the rejection changes nothing (measured: identical
    results on every case) while still carrying that risk.
    """
    x = detrend(ppg)
    if x is None:
        return None, 0
    if accel_activity(ax, ay, az) < NLMS_GATE_ACT:
        return estimate_detrended(x)
    refs = [detrend(ax), detrend(ay), detrend(az)]
    if any(r is None for r in refs):
        return estimate(ppg)
    resid = detrend(nlms(x, refs))
    if resid is None:
        return None, 0
    return estimate_detrended(resid)


def synth(bpm, h2, noise, wander, seed=1, mot_hz=0.0, mot_amp=0.0):
    """PPG-ish: fundamental + dicrotic 2nd harmonic + baseline wander + noise,
    riding on a 17-bit DC like the real GH3018 rawdata.

    With mot_amp > 0 an accelerometer-correlated interference is added — the
    thing that made the watch report 218 bpm (3.63 Hz) while the wearer's hand
    moved at 3.6 Hz. Returns (ppg, ax, ay, az); the accel carries gravity on z
    like the real sensor, so the detrending has something to remove.
    """
    random.seed(seed)
    f = bpm / 60.0
    ppg, ax, ay, az = [], [], [], []
    for i in range(WIN):
        t = i / FS
        v = math.sin(2 * math.pi * f * t) + h2 * math.sin(2 * math.pi * 2 * f * t + 0.9)
        v += wander * math.sin(2 * math.pi * 0.05 * t)
        v += noise * (random.random() - 0.5)
        m = math.sin(2 * math.pi * mot_hz * t)
        m2 = math.sin(2 * math.pi * mot_hz * t + 1.3)
        ppg.append(60000.0 + 800.0 * (v + mot_amp * (0.7 * m + 0.4 * m2)))
        ax.append(4000.0 * m + 60.0 * (random.random() - 0.5))
        ay.append(2500.0 * m2 + 60.0 * (random.random() - 0.5))
        az.append(16000.0 + 300.0 * m + 60.0 * (random.random() - 0.5))
    return ppg, ax, ay, az


CASES = [
    # (name, true bpm, 2nd-harmonic gain, noise, wander[, motion Hz, motion amp])
    # --- the failure we are here to fix: harmonic outranks the fundamental ---
    ("clean, weak 2nd",             55, 0.30, 0.05, 0.0),
    ("STRONG 2nd (1.4x)",           55, 1.40, 0.05, 0.0),
    ("strong 2nd (2.5x)",           55, 2.50, 0.10, 0.0),
    ("strong 2nd + noise + wander", 58, 1.80, 0.60, 1.5),
    ("low HR 47 + strong 2nd",      47, 1.60, 0.10, 0.5),
    ("very strong 2nd (4x)",        52, 4.00, 0.20, 0.5),
    ("harmonic at 61 (08-04 night)",61, 2.00, 0.40, 1.0),
    ("harmonic 58 clean",           58, 2.20, 0.05, 0.0),
    ("harmonic 54, h2=3",           54, 3.00, 0.30, 0.8),
    # --- must NOT be halved: genuine rates in the band the vendor reports ---
    ("REAL 110 exercise",          110, 0.40, 0.10, 0.0),
    ("REAL 110 clean",             110, 0.30, 0.05, 0.0),
    ("REAL 118 must not halve",    118, 0.30, 0.20, 0.5),
    ("REAL 95",                     95, 0.50, 0.30, 0.8),
    ("REAL 130",                   130, 0.35, 0.25, 0.4),
    ("REAL 100",                   100, 0.40, 0.20, 0.4),
    ("REAL 168 max",               168, 0.30, 0.20, 0.2),
    # --- ordinary resting rates ---
    ("REAL 62 quiet",               62, 0.35, 0.15, 0.3),
    ("REAL 48 deep sleep",          48, 0.45, 0.15, 0.3),
    ("REAL 73",                     73, 0.40, 0.25, 0.6),
    ("REAL 86",                     86, 0.45, 0.20, 0.4),
    ("REAL 52 clean",               52, 0.35, 0.08, 0.2),
    # --- wrist motion: every one of these is a rate the WATCH ACTUALLY REPORTED
    # on 2026-08-06 once the wearer got up. 218 bpm is 3.63 Hz, 195 is 3.25 Hz,
    # 171 is 2.85 Hz — hand-movement rates, not heartbeats. Without NLMS the
    # estimator locks onto the movement, because it has no way to tell a periodic
    # wrist from a periodic heart.
    ("MOTION 3.6Hz over HR 60",      60, 0.40, 0.20, 0.5, 3.60, 2.5),
    ("MOTION 3.25Hz over HR 62",     62, 0.40, 0.20, 0.5, 3.25, 3.0),
    ("MOTION 2.85Hz over HR 58",     58, 0.40, 0.20, 0.5, 2.85, 2.0),
    ("MOTION 2.0Hz over HR 75",      75, 0.40, 0.25, 0.6, 2.00, 2.5),
    ("MOTION 1.5Hz over HR 110",    110, 0.40, 0.20, 0.4, 1.50, 2.0),
    ("MOTION heavy over HR 95",      95, 0.40, 0.30, 0.8, 2.40, 4.0),
    ("MOTION 4.2Hz over HR 68",      68, 0.40, 0.25, 0.5, 4.20, 3.0),
    ("MOTION mild over HR 54",       54, 0.40, 0.20, 0.5, 2.10, 1.0),
]

# KNOWN LIMITATION, not a regression. Motion at ~the heart's own frequency: an
# adaptive filter cannot separate two sources sharing a frequency, so NLMS
# removes the pulse along with the artefact and the estimator reports what is
# left (measured: 135 for a true 66).
#
# Three discriminators were tried and NONE separates this from legitimate motion
# cases, so no threshold is shipped:
#   residual/input energy  0.032 here vs 0.090 for a real case  (1.8x margin)
#   NLMS/raw estimate      2.05  here vs 1.97 for a real case   (overlapping)
#   motion-vs-raw spacing  0%    here vs 6%   for a real case   (overlapping)
# Fitting a cut between those would be fitting one synthetic sample — the same
# mistake that made the parameter sweep look clean while the wrist produced four
# wrong readings a night.
#
# Mitigation in practice: a burst publishes the MEDIAN of ~180 windows, so one
# ambiguous window does not reach the curve on its own; and rhythmic movement at
# exactly one's resting rate is uncommon. Counted separately below so a real
# regression is never hidden behind it.
AMBIGUOUS = [
    ("MOTION 1.1Hz over HR 66 (same freq)", 66, 0.40, 0.20, 0.5, 1.10, 2.0),
]


def main():
    bad = 0
    print("%-34s %6s %14s %s" % ("case", "true", "est", "verdict"))
    for case in CASES:
        name, bpm, h2, nz, wd = case[:5]
        mh, ma = (case[5], case[6]) if len(case) > 5 else (0.0, 0.0)
        ppg, ax, ay, az = synth(bpm, h2, nz, wd, mot_hz=mh, mot_amp=ma)
        # Motion compensation runs unconditionally — a still wrist gives NLMS a
        # reference with no structure to subtract, so the resting cases must come
        # through unchanged. That is exactly what these rows check.
        e, c = estimate_with_motion(ppg, ax, ay, az)
        ok = e is not None and abs(e - bpm) <= 5
        bad += 0 if ok else 1
        print("%-34s %6d %14s %s"
              % (name, bpm, ("%.1f (c%d)" % (e, c)) if e else "none",
                 "OK" if ok else "*** WRONG ***"))
    # Printed loudly but NOT added to `bad`: this is the documented physical
    # limit above, and letting it fail the suite would train everyone to ignore
    # a red result — at which point a real regression hides behind it.
    known = 0
    for name, bpm, h2, nz, wd, mh, ma in AMBIGUOUS:
        ppg, ax, ay, az = synth(bpm, h2, nz, wd, mot_hz=mh, mot_amp=ma)
        e, c = estimate_with_motion(ppg, ax, ay, az)
        if not (e is None or abs(e - bpm) <= 5):
            known += 1
        print("%-34s %6d %14s %s"
              % (name, bpm, ("%.1f (c%d)" % (e, c)) if e else "none",
                 "KNOWN LIMIT (see AMBIGUOUS)"))
    # A SLEEPING wrist. These exist because the motion compensation shipped with
    # its reference wired to a vendor field nothing ever writes, so for one night
    # it ran against three constant zeros and every resting case here passed for
    # the wrong reason. The moment the reference went live, breathing alone
    # halved a 60 bpm heart to 30. Anything that touches the NLMS stage must
    # clear these first — a still wrist is the overwhelmingly common case, so a
    # filter that is merely NEUTRAL at rest is not good enough; it must be
    # provably neutral across the whole range of how much a sleeper moves.
    for amp in (0, 25, 60, 120, 250, 400, 800):
        for bpm in (48, 56, 60, 72, 90):
            ppg, _, _, _ = synth(bpm, 0.9, 0.10, 0.4, seed=7)
            ax, ay, az = [], [], []
            random.seed(1000 + bpm + amp)
            for i in range(WIN):
                b = math.sin(2 * math.pi * 0.25 * i / FS)   # ~15 breaths/min
                ax.append(120 + amp * b + random.gauss(0, 2))
                ay.append(-80 + amp * 0.6 * b + random.gauss(0, 2))
                az.append(512 + amp * 0.3 * b + random.gauss(0, 2))
            e, c = estimate_with_motion(ppg, ax, ay, az)
            ok = e is not None and abs(e - bpm) <= 5
            bad += 0 if ok else 1
            if not ok:
                print("%-34s %6d %14s %s"
                      % ("sleeping wrist amp=%d" % amp, bpm,
                         ("%.1f (c%d)" % (e, c)) if e else "none", "*** WRONG ***"))
    print("%-34s %6s %14s %s"
          % ("sleeping wrist (35 combos)", "-", "-", "OK" if bad == 0 else "see above"))

    # REAL WRIST DATA. Everything above this line is synthetic, and synthetic
    # cases have repeatedly failed to predict anything: a nine-combination
    # parameter sweep passed 81 cases while the wrist produced four wrong
    # readings a night, and the whole suite was green on the night 21 of these
    # 43 windows were wrong. Keep these first in mind when changing a rule.
    #
    # A window is scored WRONG only when it answers and disagrees with the
    # independent spectral truth. Refusing is not scored: on material this noisy
    # "no reading" is frequently the correct answer, and penalising it would
    # push the estimator back toward answering confidently about nothing —
    # which is the exact defect these windows were captured to expose.
    real = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "hr_autocorr_real.csv")
    if os.path.exists(real):
        rbad = rref = rok = 0
        wrong_times = []
        with io.open(real, encoding="utf-8") as f:
            for line in f:
                if line.startswith("#") or line.startswith("time,"):
                    continue
                t, tv, samples = line.rstrip("\n").split(",", 2)
                tv = float(tv)
                x = [float(v) for v in samples.split()]
                e, _ = estimate_detrended(x)
                if e is None:
                    e, _ = spectral_estimate(x)      # the shipped chain
                if e is None:
                    rref += 1
                elif abs(e - tv) <= max(5.0, 0.08 * tv):
                    rok += 1
                else:
                    rbad += 1
                    wrong_times.append(t[-8:])
                    print("%-34s %6.0f %14.1f %s"
                          % ("REAL wrist %s" % t[-8:], tv, e, "*** WRONG ***"))
        # KNOWN FAILURES, BY NAME — not a count.
        #
        # A numeric ratchet was tried first and abandoned: adding real windows
        # grows the fixture, which grows the count, which invites raising the
        # bar "because there is more data" — and that is indistinguishable from
        # raising it to make a change pass. Naming them removes the loophole:
        # any window that fails and is not on this list is a regression however
        # small the total. Remove entries as they are fixed; adding one requires
        # saying what it is.
        #
        #   07:15:58  truth 73, answers ~118 — locks to a multiple
        #   11:36:25  truth 44, answers ~53
        #   03:23:58  truth 58, answers ~83 — first day with live accel
        #   05:22:22  truth 56, answers ~30 — the ESTIMATOR fails here and the
        #             tracker rescues it to 60. Scored wrong on purpose: this
        #             test measures the estimator alone, and a rescue is not a
        #             reason to stop counting the thing being rescued.
        KNOWN_BAD = {"07:15:58", "11:36:25", "03:23:58", "05:22:22"}
        unexpected = [w for w in wrong_times if w not in KNOWN_BAD]
        bad += len(unexpected)
        print("%-34s %6s %14s %s"
              % ("real wrist windows (%d)" % (rok + rbad + rref), "-",
                 "%d ok / %d refused" % (rok, rref),
                 "OK (%d known)" % rbad if not unexpected
                 else "REGRESSION: %s" % ", ".join(unexpected)))

    # SEQUENTIAL replay — the only test that exercises the tracker at all, since
    # every other case resets between windows. Scored on physiology rather than
    # on any single-window method: at rest a heart does not step to half its rate
    # and back inside ten minutes, so an octave-sized step between consecutive
    # readings is a defect regardless of which value was "right".
    if os.path.exists(real):
        seq = []
        with io.open(real, encoding="utf-8") as f:
            for line in f:
                if line.startswith("#") or line.startswith("time,"):
                    continue
                _, _, samples = line.rstrip().split(",", 2)
                seq.append([float(v) for v in samples.split()])
        # One captured window per burst, so a burst boundary falls between every
        # pair. Modelling it matters: without the boundary call the tracker
        # never ages and the test flatters it — the first version of this test
        # reported zero octave steps while the watch was still producing them.
        tr = Tracker()
        out = []
        for x in seq:
            tr.burst_boundary()
            e, _ = estimate_detrended(x)
            if e is None:
                e, _ = spectral_estimate(x)
            out.append(tr.feed(e))
        steps = 0
        prev = None
        for v in out:
            if v is None:
                continue
            if prev is not None:
                ratio = v / prev
                if 1.7 <= ratio <= 2.4 or 0.42 <= ratio <= 0.6:
                    steps += 1
            prev = v
        # One known step survives: 70 -> 42, where the doubled candidate lands
        # exactly on the SNAP boundary. Sweeping that threshold does not help —
        # 20/22/25%% give identical results and 30%% makes the whole fixture worse
        # (7 wrong -> 10, 1 step -> 3) — so the threshold is not the lever and
        # tuning it here would be fitting to a single case.
        KNOWN_STEPS = 1
        bad += max(0, steps - KNOWN_STEPS)
        vals = [v for v in out if v]
        print("%-34s %6s %14s %s"
              % ("sequential octave steps", "-",
                 "%d..%d bpm" % (min(vals), max(vals)),
                 "OK (%d known)" % steps if steps <= KNOWN_STEPS
                 else "*** %d STEPS ***" % steps))

    # TRACKER DYNAMICS. The tracker consumes bpm NUMBERS, not waveforms, so its
    # behaviour is testable directly — and it has to be, because the window
    # fixture holds one window per burst while the estimator runs at 1 Hz for
    # ~40 s inside each. Everything that has gone wrong with this tracker lived
    # in that gap.
    #
    # The exercise cases are the ones that matter most. Fed a genuine 60 -> 120
    # the previous version returned 60 for every single reading, forever: 120/2
    # lands exactly on the resting baseline, and each suppressed value fed the
    # history that justified suppressing the next.
    dyn = [
        ("run of low then a correct high", [56]*10 + [40]*6 + [79], 79),
        ("isolated halving is corrected",  [56]*10 + [30], 60),
        ("isolated doubling is corrected", [74]*10 + [130], 65),
        ("gradual rise is left alone",     [56]*10 + [62,68,74,80,86], 86),
        ("sudden exercise passes through", [60]*10 + [120]*12, 120),
        ("sustained exercise not pinned",  [60]*10 + [118,120,122,124,125,124,122,120], 120),
        ("single noise after long calm",   [58]*20 + [31], 62),
        ("cool-down not doubled back up",  [60]*8 + [120]*8 + [62]*8, 62),
        ("alternating flicker is damped",  [56]*10 + [28,56,28,56,28,56], 56),
    ]
    dbad = 0
    for name, seq, want in dyn:
        tr = Tracker()
        got = None
        for v in seq:
            tr.burst_boundary()
            got = tr.feed(v)
        if got is None or abs(got - want) > max(3, 0.06 * want):
            dbad += 1
            print("%-34s %6d %14s %s"
                  % ("TRACKER %s" % name, want,
                     ("%.0f" % got) if got else "none", "*** WRONG ***"))
    # One known failure: the "run of low then a correct high" sequence is my own
    # reconstruction of 2026-08-13 04:14, inferred from the baseline the output
    # implies (32.5-48.75) — not the sequence the watch actually saw. Tuning
    # against an invented sequence is fitting to a guess, so it is recorded and
    # left alone until the per-second data exists.
    TRACKER_KNOWN = 1
    bad += max(0, dbad - TRACKER_KNOWN)
    print("%-34s %6s %14s %s"
          % ("tracker dynamics (%d)" % len(dyn), "-", "%d wrong" % dbad,
             "OK (%d known)" % dbad if dbad <= TRACKER_KNOWN else "*** REGRESSION ***"))

    # Pure noise must be refused. The vendor library's failure to do this is why
    # a watch on a table reported a rock-steady 108-115 bpm for two minutes.
    for seed in (9, 17, 23):
        random.seed(seed)
        e, c = estimate([60000.0 + 800.0 * (random.random() - 0.5) for _ in range(WIN)])
        ok = e is None or c < 50
        bad += 0 if ok else 1
        print("%-30s %6s %14s %s"
              % ("noise only seed%d" % seed, "-",
                 ("%.1f (c%d)" % (e, c)) if e else "none",
                 "OK" if ok else "*** WRONG ***"))
    total = len(CASES) + 35 + 3   # AMBIGUOUS is reported, never scored
    print("\n%d/%d wrong   (+%d known limit, excluded — see AMBIGUOUS)"
          % (bad, total, known))
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
