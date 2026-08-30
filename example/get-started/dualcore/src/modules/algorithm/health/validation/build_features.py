# -*- coding: utf-8 -*-
"""
Cache per-minute features for the staging-ceiling experiment.

Question this exists to answer: our shipped stager scores kappa 0.109 against
PSG, which is WORSE than a constant "LIGHT" prediction. Is that because the
rules are bad, or because the features our watch can produce simply do not
carry the Light/Deep/REM distinction? Those two have opposite consequences —
one is a rewrite, the other means dropping the feature from the product.

To answer it we need an upper bound: fit the strongest model we can on exactly
the signals the watch has, and score it on held-out SUBJECTS. Anything that
model cannot reach, no hand-written rule will reach either.

Everything here is restricted to what the device actually produces:
  * activity_count — 1 Hz accel, (|dx|+|dy|+|dz|)>>10 summed per minute
  * step_count
  * hr_mean / hr_std — DECIMATED to the shipped 3-min-in-10 cadence and held
    between bursts (validated against a real night: fresh lands every 10 min,
    each value is reused for 10 minutes, 2026-08-30)
No RR intervals, no continuous HR, no 100 Hz. If the ceiling is high, the
signal is there and the rules are the problem.

Writes features.npz.  Run: python build_features.py [data_dir]
"""
import os
import sys
import glob
import math
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import validate_physionet as V   # noqa: E402
import validate_staging as S     # noqa: E402

CLASS_ID = {S.WAKE: 0, S.LIGHT: 1, S.DEEP: 2, S.REM: 3}


def _roll(a, k, fn):
    """Centred rolling stat over +-k minutes, edge-clamped."""
    n = len(a)
    out = np.zeros(n)
    for i in range(n):
        lo, hi = max(0, i - k), min(n, i + k + 1)
        out[i] = fn(a[lo:hi])
    return out


def subject_features(mins):
    """minute-dict -> (X, y, feature_names). Order is by minute index."""
    ms = sorted(mins)
    act = np.array([mins[m]["activity"] for m in ms], dtype=float)
    stp = np.array([mins[m]["steps"] for m in ms], dtype=float)
    hr = np.array([mins[m]["hr_mean"] for m in ms], dtype=float)
    hrs = np.array([mins[m]["hr_std"] for m in ms], dtype=float)
    y = np.array([CLASS_ID[mins[m]["truth"]] for m in ms], dtype=int)
    n = len(ms)

    # --- the stable resting reference the firmware now keeps (rhr_ref_bpm):
    #     5th percentile of the night's own HR. This is the anchor the old
    #     rules lacked — they compared HR to its own 10-sample median.
    valid = hr[hr > 0]
    rhr = float(np.percentile(valid, 5)) if valid.size else 60.0

    lact = np.log1p(act)
    feats = {
        "act": act,
        "log_act": lact,
        "act_roll5": _roll(lact, 5, np.mean),
        "act_roll15": _roll(lact, 15, np.mean),
        "act_roll30": _roll(lact, 30, np.mean),
        "act_still30": _roll(act, 30, lambda w: float((w == 0).mean())),
        "steps": stp,
        "steps_roll15": _roll(stp, 15, np.sum),

        "hr": hr,
        "hr_std_burst": hrs,
        # HR relative to the stable reference — absolute, not self-referential
        "hr_over_rhr": np.where(hr > 0, hr - rhr, 0.0),
        "hr_ratio_rhr": np.where(hr > 0, hr / max(rhr, 1.0), 1.0),

        # multi-MINUTE HR variability: REM is erratic across minutes, deep is
        # smooth. The old rule only had within-burst std, which at 30% duty is
        # a handful of samples.
        "hr_roll_std15": _roll(hr, 15, lambda w: float(np.std(w[w > 0])) if (w > 0).any() else 0.0),
        "hr_roll_std30": _roll(hr, 30, lambda w: float(np.std(w[w > 0])) if (w > 0).any() else 0.0),
        "hr_roll_std60": _roll(hr, 60, lambda w: float(np.std(w[w > 0])) if (w > 0).any() else 0.0),
        "hr_roll_mean30": _roll(hr, 30, lambda w: float(np.mean(w[w > 0])) if (w > 0).any() else 0.0),
        "hr_range30": _roll(hr, 30, lambda w: float(np.ptp(w[w > 0])) if (w > 0).any() else 0.0),
        # deviation of the local mean from the night's floor
        "hr_local_over_rhr": _roll(hr, 30, lambda w: float(np.mean(w[w > 0])) if (w > 0).any() else 0.0) - rhr,
    }
    # HR slope over a few lags (trend, not just level)
    for lag in (5, 15, 30):
        d = np.zeros(n)
        d[lag:] = hr[lag:] - hr[:-lag]
        d[hr == 0] = 0.0
        feats[f"hr_d{lag}"] = d

    # --- TIME. Deep sleep front-loads the night, REM back-loads it. The
    #     shipped rules use no temporal information at all.
    idx = np.arange(n, dtype=float)
    feats["t_frac"] = idx / max(n - 1, 1)          # position in the recording
    feats["t_min"] = idx                            # minutes since start
    # elapsed since the first plausible sleep onset (quiet + HR near floor),
    # computed causally so it is implementable on-watch
    quiet = (feats["act_roll15"] < np.percentile(feats["act_roll15"], 40))
    low = (hr > 0) & (hr < rhr + 12)
    onset = n
    run = 0
    for i in range(n):
        run = run + 1 if (quiet[i] and low[i]) else 0
        if run >= 15:
            onset = i - 14
            break
    since = np.maximum(idx - onset, 0.0)
    feats["t_since_onset"] = since
    feats["t_since_onset_frac"] = since / max(n - onset, 1)

    names = sorted(feats)
    X = np.column_stack([feats[k] for k in names])
    return X, y, names


def main():
    data_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "data")
    lfs = sorted(glob.glob(os.path.join(data_dir, "*_labeled_sleep.txt")))
    Xs, ys, gs, names = [], [], [], None
    shipped = []
    for lf in lfs:
        sid = os.path.basename(lf).split("_")[0]
        base = os.path.join(data_dir, sid)
        try:
            accel = V.load_pairs(base + "_acceleration.txt", 4)
            hr = V.load_pairs(base + "_heartrate.txt", 2)
            steps = V.load_pairs(base + "_steps.txt", 2)
            labels = V.load_pairs(lf, 2)
        except FileNotFoundError:
            continue
        mins = S.build(accel, hr, steps, labels, duty_cycle=True)
        if len(mins) < 60:
            continue
        X, y, names = subject_features(mins)
        pred = S.run_shipped(mins)
        shipped.append(np.array([CLASS_ID[pred[m]] for m in sorted(mins)], dtype=int))
        Xs.append(X)
        ys.append(y)
        gs.append(np.full(len(y), int(sid)))
        print(f"  {sid}  {len(y)} min", flush=True)
    X = np.vstack(Xs)
    y = np.concatenate(ys)
    g = np.concatenate(gs)
    sh = np.concatenate(shipped)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "features.npz")
    np.savez_compressed(out, X=X, y=y, g=g, shipped=sh, names=np.array(names))
    print(f"\n{X.shape[0]} minutes, {X.shape[1]} features, "
          f"{len(set(g.tolist()))} subjects -> {out}")


if __name__ == "__main__":
    main()
