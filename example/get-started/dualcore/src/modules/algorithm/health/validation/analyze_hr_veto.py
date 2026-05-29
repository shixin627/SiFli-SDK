"""
Diagnostics on top of validate_physionet.py, run over every subject in the
data dir:
  (1) HR separation — percentiles of per-minute HR for true-WAKE vs true-SLEEP
      minutes. Shows how little daylight there is between the two on in-bed PSG
      data (the hard case) — which is why the veto must be conservative.
  (2) Offset sweep — replay each subject through the SHIPPED firmware algorithm
      (self-learning resting baseline, internal) at a range of veto offsets and
      report the sleep-sensitivity / wake-specificity trade-off, to justify the
      chosen SF_WAKE_HR_OFFSET_BPM.

Run:  python analyze_hr_veto.py [data_dir]
"""
import os
import sys
import glob

import validate_physionet as V


def pct(sorted_vals, p):
    if not sorted_vals:
        return 0
    i = max(0, min(len(sorted_vals) - 1, int(round(p / 100.0 * (len(sorted_vals) - 1)))))
    return sorted_vals[i]


def load_subject_minutes(data_dir):
    out = {}
    for lf in sorted(glob.glob(os.path.join(data_dir, "*_labeled_sleep.txt"))):
        sid = os.path.basename(lf).split("_")[0]
        base = os.path.join(data_dir, sid)
        try:
            accel = V.load_pairs(base + "_acceleration.txt", 4)
            hr = V.load_pairs(base + "_heartrate.txt", 2)
            steps = V.load_pairs(base + "_steps.txt", 2)
            labels = V.load_pairs(lf, 2)
        except FileNotFoundError:
            continue
        m = V.build_minutes(accel, hr, steps, labels)
        if m:
            out[sid] = m
    return out


def main():
    data_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "data")
    subs = load_subject_minutes(data_dir)
    print(f"Loaded {len(subs)} subjects\n")

    # ---- (1) HR separation: wake vs sleep ----
    print("=== HR separation (per-minute mean HR), true-WAKE vs true-SLEEP ===")
    print(f"{'subject':>9} | {'wake p25/50/75':>16} | {'sleep p25/50/75':>16} | overlap")
    pool_wake, pool_sleep = [], []
    for sid, mins in subs.items():
        wake = sorted(d["hr_mean"] for d in mins.values()
                      if not d["true_sleep"] and d["hr_mean"] > 0)
        sleep = sorted(d["hr_mean"] for d in mins.values()
                       if d["true_sleep"] and d["hr_mean"] > 0)
        pool_wake += wake
        pool_sleep += sleep
        wake_med = pct(wake, 50) if wake else 0
        sleep_above = (sum(1 for v in sleep if v > wake_med) / len(sleep)) if sleep else 0
        print(f"{sid:>9} | {pct(wake,25):>4}/{pct(wake,50):>3}/{pct(wake,75):>3}     "
              f" | {pct(sleep,25):>4}/{pct(sleep,50):>3}/{pct(sleep,75):>3}     "
              f" | {sleep_above*100:4.0f}% sleep>wakeMed")
    pw, psl = sorted(pool_wake), sorted(pool_sleep)
    print(f"{'POOLED':>9} | {pct(pw,25):>4}/{pct(pw,50):>3}/{pct(pw,75):>3}     "
          f" | {pct(psl,25):>4}/{pct(psl,50):>3}/{pct(psl,75):>3}")
    print("  Dataset WAKE is in-bed (calm, HR near sleep) — the HARD case for an"
          "\n  HR veto. A train ride is daytime upright wake, HR far higher — easy.\n")

    # ---- (2) offset sweep through the shipped self-learning algorithm ----
    print("=== Veto offset sweep (shipped firmware, self-learned baseline) ===")
    print(f"{'veto':>14}  {'sleepSens':>9} {'wakeSpec':>8} {'kappa':>6} "
          f"{'FP(w->s)':>8} {'FN(missed sleep)':>16}")

    def pooled(offset):
        agg = [0, 0, 0, 0]
        for sid, mins in subs.items():
            preds, _ = V.run_algo(mins, offset)
            c = V.confusion(mins, preds)
            for i in range(4):
                agg[i] += c[i]
        return agg

    base = pooled(100000)  # veto off
    acc, sens, spec, k = V.metrics(*base)
    print(f"{'OFF (orig)':>14}  {sens*100:8.1f}% {spec*100:7.1f}% {k:6.2f} "
          f"{base[2]:8d} {base[3]:16d}")
    for off in [12, 15, 18, 20, 22, 25, 30]:
        agg = pooled(off)
        acc, sens, spec, k = V.metrics(*agg)
        star = "  <- shipped" if off == V.sf.SF_WAKE_HR_OFFSET_BPM else ""
        print(f"{('rhr+'+str(off)):>14}  {sens*100:8.1f}% {spec*100:7.1f}% {k:6.2f} "
              f"{agg[2]:8d} {agg[3]:16d}{star}")


if __name__ == "__main__":
    main()
