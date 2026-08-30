# -*- coding: utf-8 -*-
"""
Validate sleep_fusion's STAGE classifier (Light/Deep/REM) against PSG.

Companion to validate_physionet.py, which scores sleep-vs-wake only: it folds
PSG labels 1/2/3/5 into a single "asleep" class, so it can say nothing about
whether the Light/Deep/REM split means anything. This script keeps the stages.

Two things it does that the sleep/wake harness does not, both of which make the
result HONEST for our device rather than flattering:

  1. HR DUTY CYCLE. The dataset carries a heart rate roughly every few seconds.
     Our watch measures 3 minutes out of every 10 and then holds the burst's
     value until the next burst (@ref hr_service.c, the 10-min/3-min cadence
     fixed 2026-08-16). Feeding a continuous HR series into a stager that will
     only ever see 30% duty cycle would measure a device we do not ship. So HR
     is decimated to the real cadence, including hr_is_fresh, before replay.

  2. STAGE-LEVEL SCORING. 4-class (Wake/Light/Deep/REM) and the 3-class
     (Wake/NREM/REM) target that the wearable literature actually reports,
     plus REM-specific sensitivity/precision, plus the trivial baselines that
     make a kappa interpretable.

Ground truth: Walch et al. 2019 PhysioNet sleep-accel 1.0.0 (ODC-BY), the same
dataset sleep_fusion.h already cites. PSG stage per 30 s epoch:
    0 = Wake,  1 = N1,  2 = N2,  3 = N3,  4 = (legacy N4),  5 = REM,  -1 = unscored

Run:  python validate_staging.py [data_dir]
"""
import os
import sys
import math
import glob

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import sleep_fusion_test as sf  # noqa: E402
import validate_physionet as V  # noqa: E402  (reuse the loaders)

G_TO_LSB = 16384
SHIFT = 10

WAKE, LIGHT, DEEP, REM = "WAKE", "LIGHT", "DEEP", "REM"
CLASSES = [WAKE, LIGHT, DEEP, REM]

# PSG label -> our class vocabulary.
PSG_TO_CLASS = {0: WAKE, 1: LIGHT, 2: LIGHT, 3: DEEP, 4: DEEP, 5: REM}

# Our firmware stage -> the same vocabulary.
FW_TO_CLASS = {
    sf.Stage.AWAKE: WAKE,
    sf.Stage.LIGHT: LIGHT,
    sf.Stage.DEEP: DEEP,
    sf.Stage.REM: REM,
    sf.Stage.NOT_WORN: WAKE,   # never produced here (is_worn is forced True)
}

# --- the device's real HR cadence -------------------------------------------
BURST_PERIOD_MIN = 10   # a burst starts every 10 minutes
BURST_LEN_MIN = 3       # and runs for 3 of them


def build(accel, hr, steps, labels, duty_cycle=True):
    """minute -> dict(activity, steps, hr_mean, hr_std, hr_fresh, truth).

    Only minutes carrying a scored PSG label are returned.
    """
    # --- accel -> 1 Hz -> per-minute activity_count, exactly like the firmware
    sec_read, sec_best = {}, {}
    for (t, x, y, z) in accel:
        s = int(round(t))
        d = abs(t - s)
        if s not in sec_best or d < sec_best[s]:
            sec_best[s], sec_read[s] = d, (x, y, z)
    act = {}
    prev_s = prev = None
    for s in sorted(sec_read):
        x, y, z = sec_read[s]
        if prev_s is not None and s - prev_s == 1:
            mag = (abs(int(round((x - prev[0]) * G_TO_LSB)))
                   + abs(int(round((y - prev[1]) * G_TO_LSB)))
                   + abs(int(round((z - prev[2]) * G_TO_LSB)))) >> SHIFT
            m = V.minute_index(s)
            act[m] = act.get(m, 0) + mag
        prev_s, prev = s, (x, y, z)

    # --- raw HR samples per minute
    hr_by_min = {}
    for (t, bpm) in hr:
        if 0 < bpm < 240:
            hr_by_min.setdefault(V.minute_index(t), []).append(bpm)

    st_by_min = {}
    for (t, n) in steps:
        if n > 0:
            m = V.minute_index(t)
            st_by_min[m] = st_by_min.get(m, 0) + int(n)

    # --- PSG stage per minute: majority over the epochs inside the minute
    truth = {}
    lbl = {}
    for (t, stage) in labels:
        lbl.setdefault(V.minute_index(t), []).append(int(stage))
    for m, stages in lbl.items():
        scored = [s for s in stages if s in PSG_TO_CLASS]
        if not scored:
            continue
        counts = {}
        for s in scored:
            c = PSG_TO_CLASS[s]
            counts[c] = counts.get(c, 0) + 1
        # deterministic tie-break by the CLASSES order
        truth[m] = max(CLASSES, key=lambda c: (counts.get(c, 0), -CLASSES.index(c)))

    if not truth:
        return {}
    lo, hi = min(truth), max(truth)

    # --- HR seen through the device's duty cycle ----------------------------
    # A burst covers minutes [10k, 10k+BURST_LEN); its value lands at the end of
    # the burst and is HELD until the next burst lands. fresh only on the minute
    # it lands, mirroring hr_is_fresh.
    hr_view = {}          # minute -> (mean, std, fresh)
    if duty_cycle:
        held = (0, 0)
        landed_at = None
        for m in range(lo, hi + 1):
            k, off = divmod(m - lo, BURST_PERIOD_MIN)
            if off == BURST_LEN_MIN - 1:
                samples = []
                for j in range(BURST_LEN_MIN):
                    samples += hr_by_min.get(lo + k * BURST_PERIOD_MIN + j, [])
                if samples:
                    mean = sum(samples) / len(samples)
                    var = sum((v - mean) ** 2 for v in samples) / len(samples)
                    held = (int(round(mean)), int(round(math.sqrt(var))))
                    landed_at = m
                # a burst that produced nothing leaves the previous value held,
                # which is what the firmware does too (bg_hr_win_mean untouched)
            hr_view[m] = (held[0], held[1], m == landed_at)
    else:
        for m in range(lo, hi + 1):
            s = hr_by_min.get(m, [])
            if s:
                mean = sum(s) / len(s)
                var = sum((v - mean) ** 2 for v in s) / len(s)
                hr_view[m] = (int(round(mean)), int(round(math.sqrt(var))), True)
            else:
                hr_view[m] = (0, 0, False)

    out = {}
    for m in sorted(truth):
        hm, hs, fr = hr_view.get(m, (0, 0, False))
        out[m] = {
            "activity": act.get(m, 0),
            "steps": min(st_by_min.get(m, 0), 0xFFFF),
            "hr_mean": min(hm, 255),
            "hr_std": min(hs, 255),
            "hr_fresh": fr,
            "truth": truth[m],
        }
    return out


def run_shipped(minutes):
    """Replay through the shipped algorithm (the 1:1 C mirror). -> minute->class"""
    st = sf.State()
    pred = {}
    for m in sorted(minutes):
        d = minutes[m]
        sf.update(st, m * 60, sf.MinuteInput(
            activity_count=d["activity"], step_count=d["steps"],
            hr_mean_bpm=d["hr_mean"], hr_std_bpm=d["hr_std"],
            hr_is_fresh=d["hr_fresh"], is_worn=True))
        pred[m] = FW_TO_CLASS[st.stage]
    return pred


# ---------------------------------------------------------------- scoring ---
def confusion(minutes, pred, classes=CLASSES):
    cm = {t: {p: 0 for p in classes} for t in classes}
    for m in minutes:
        cm[minutes[m]["truth"]][pred[m]] += 1
    return cm


def collapse3(cm):
    """4-class -> Wake / NREM / REM (the target the literature reports)."""
    def g(c):
        return "NREM" if c in (LIGHT, DEEP) else c
    cl = [WAKE, "NREM", REM]
    out = {t: {p: 0 for p in cl} for t in cl}
    for t in cm:
        for p in cm[t]:
            out[g(t)][g(p)] += cm[t][p]
    return out


def kappa(cm):
    cls = list(cm)
    n = sum(cm[t][p] for t in cls for p in cls)
    if not n:
        return 0.0
    po = sum(cm[c][c] for c in cls) / n
    pe = sum((sum(cm[c].values()) / n) * (sum(cm[t][c] for t in cls) / n)
             for c in cls)
    return (po - pe) / (1 - pe) if pe < 1 else 0.0


def acc(cm):
    cls = list(cm)
    n = sum(cm[t][p] for t in cls for p in cls)
    return (sum(cm[c][c] for c in cls) / n) if n else 0.0


def per_class(cm, c):
    cls = list(cm)
    tp = cm[c][c]
    fn = sum(cm[c][p] for p in cls) - tp
    fp = sum(cm[t][c] for t in cls) - tp
    sens = tp / (tp + fn) if (tp + fn) else 0.0
    prec = tp / (tp + fp) if (tp + fp) else 0.0
    f1 = 2 * sens * prec / (sens + prec) if (sens + prec) else 0.0
    return sens, prec, f1, tp + fn


def show(cm, title):
    cls = list(cm)
    n = sum(cm[t][p] for t in cls for p in cls)
    print(f"\n  {title}   n={n}  acc={acc(cm)*100:.1f}%  kappa={kappa(cm):.3f}")
    w = max(6, max(len(c) for c in cls) + 1)
    print("    truth\\pred " + "".join(f"{c:>{w}}" for c in cls) + f"{'':>4}share")
    for t in cls:
        row = sum(cm[t].values())
        share = f"   {100 * row / n:5.1f}%" if n else ""
        print(f"    {t:<10}" + "".join(f"{cm[t][p]:>{w}}" for p in cls) + share)
    for c in cls:
        s, p, f1, sup = per_class(cm, c)
        print(f"    {c:<10} sens={s*100:5.1f}%  prec={p*100:5.1f}%  F1={f1:.3f}  support={sup}")


def main():
    data_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "data")
    lfs = sorted(glob.glob(os.path.join(data_dir, "*_labeled_sleep.txt")))
    if not lfs:
        print(f"No data in {data_dir}")
        return
    print(f"Subjects: {len(lfs)}   data: {data_dir}")
    print(f"HR decimated to the shipped cadence: {BURST_LEN_MIN} min on / "
          f"{BURST_PERIOD_MIN} min period, value held between bursts.\n")

    pooled = {t: {p: 0 for p in CLASSES} for t in CLASSES}
    pooled_full = {t: {p: 0 for p in CLASSES} for t in CLASSES}
    nsub = 0
    print(f"{'subject':>9} {'min':>5} {'hrcov':>6} {'REM%':>6} {'DEEP%':>6}"
          f" {'acc4':>6} {'k4':>6} {'k3':>6} {'REMsens':>8} {'REMprec':>8}")
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
        mins = build(accel, hr, steps, labels, duty_cycle=True)
        if not mins:
            continue
        nsub += 1
        pred = run_shipped(mins)
        cm = confusion(mins, pred)
        cm3 = collapse3(cm)
        n = len(mins)
        hrcov = sum(1 for d in mins.values() if d["hr_mean"] > 0) / n
        remp = sum(1 for d in mins.values() if d["truth"] == REM) / n
        deepp = sum(1 for d in mins.values() if d["truth"] == DEEP) / n
        rs, rp, _, _ = per_class(cm, REM)
        print(f"{sid:>9} {n:>5} {hrcov*100:5.0f}% {remp*100:5.1f}% {deepp*100:5.1f}%"
              f" {acc(cm)*100:5.1f}% {kappa(cm):6.3f} {kappa(cm3):6.3f}"
              f" {rs*100:7.1f}% {rp*100:7.1f}%")
        for t in CLASSES:
            for p in CLASSES:
                pooled[t][p] += cm[t][p]

        # full-rate HR, for reference only: how much the duty cycle costs
        mf = build(accel, hr, steps, labels, duty_cycle=False)
        cf = confusion(mf, run_shipped(mf))
        for t in CLASSES:
            for p in CLASSES:
                pooled_full[t][p] += cf[t][p]

    print("\n" + "=" * 96)
    print(f"POOLED over {nsub} subjects — SHIPPED ALGORITHM, real 30% HR duty cycle")
    show(pooled, "4-class (Wake/Light/Deep/REM)")
    show(collapse3(pooled), "3-class (Wake/NREM/REM)")

    print("\n" + "-" * 96)
    print("Reference: same algorithm fed CONTINUOUS HR (not our device) —")
    print("the gap is what the 10-min/3-min cadence costs, not an algorithm change.")
    show(pooled_full, "4-class, continuous HR")

    # --- trivial baselines: a kappa is meaningless without them -------------
    print("\n" + "-" * 96)
    n = sum(pooled[t][p] for t in CLASSES for p in CLASSES)
    prior = {c: sum(pooled[c].values()) / n for c in CLASSES}
    print("Trivial baselines (what you get for free):")
    print("  class prevalence: " + "  ".join(f"{c}={prior[c]*100:.1f}%" for c in CLASSES))
    maj = max(CLASSES, key=lambda c: prior[c])
    print(f"  always-'{maj}'          acc={prior[maj]*100:.1f}%  kappa=0.000")
    never = {t: {p: 0 for p in CLASSES} for t in CLASSES}
    for t in CLASSES:
        never[t][WAKE if t == WAKE else LIGHT] = sum(pooled[t].values())
    print(f"  perfect sleep/wake, all sleep called LIGHT   "
          f"acc={acc(never)*100:.1f}%  kappa={kappa(never):.3f}")
    print("\n  => the shipped stager is only worth keeping if it beats these.")


if __name__ == "__main__":
    main()
