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
"""
import math
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


def estimate(raw):
    x = detrend(raw)
    if x is None:
        return None, 0
    r = ncc(x)
    peaks = [l for l in range(LAG_MIN, LAG_MAX + 1)
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


def synth(bpm, h2, noise, wander, seed=1):
    """PPG-ish: fundamental + dicrotic 2nd harmonic + baseline wander + noise,
    riding on a 17-bit DC like the real GH3018 rawdata."""
    random.seed(seed)
    f = bpm / 60.0
    out = []
    for i in range(WIN):
        t = i / FS
        v = math.sin(2 * math.pi * f * t) + h2 * math.sin(2 * math.pi * 2 * f * t + 0.9)
        v += wander * math.sin(2 * math.pi * 0.05 * t)
        v += noise * (random.random() - 0.5)
        out.append(60000.0 + 800.0 * v)
    return out


CASES = [
    # (name, true bpm, 2nd-harmonic gain, noise, wander)
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
]


def main():
    bad = 0
    print("%-30s %6s %14s %s" % ("case", "true", "est", "verdict"))
    for name, bpm, h2, nz, wd in CASES:
        e, c = estimate(synth(bpm, h2, nz, wd))
        ok = e is not None and abs(e - bpm) <= 5
        bad += 0 if ok else 1
        print("%-30s %6d %14s %s"
              % (name, bpm, ("%.1f (c%d)" % (e, c)) if e else "none",
                 "OK" if ok else "*** WRONG ***"))
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
    total = len(CASES) + 3
    print("\n%d/%d wrong" % (bad, total))
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
