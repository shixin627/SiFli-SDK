"""
Validate sleep_fusion against the Walch et al. 2019 PhysioNet dataset
("Motion and heart rate from a wrist-worn wearable and labeled sleep from
polysomnography", sleep-accel 1.0.0, ODC-BY). This is the very dataset our
sleep_fusion.c header cites (Walch 2019), so it is the right ground truth:
wrist accel (g) + HR (bpm) + steps + PSG-scored sleep stages per 30 s epoch.

What it does, per subject:
  * Replays the data minute-by-minute through the EXACT firmware algorithm
    (imported from sleep_fusion_test.py, the 1:1 C port).
  * Reconstructs the firmware's per-minute features faithfully:
      - activity_count: decimate accel to 1 Hz (one read per second, like
        bmi270_accel_read at 1 Hz), |Δ| between consecutive seconds in LSB
        (g*16384, ±2g), >>10, summed over the minute. Matches sleep_service.c.
      - step_count: real steps from the dataset's step bins.
      - hr_mean/hr_std: from HR samples in the minute (0 if none).
  * Scores predicted sleep/wake against PSG (0=wake; 1/2/3/5=sleep; -1 unscored
    excluded) WITH and WITHOUT the HR wake-veto (toggled via the margin knob),
    so we can measure exactly what the fix changes.

Run:  python validate_physionet.py [data_dir]
Data: data/<id>_{acceleration,heartrate,steps,labeled_sleep}.txt
"""
import os
import sys
import math
import glob

# Import the 1:1 C port. Guarded __main__ in that file means importing it does
# not run the synthetic scenarios.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import sleep_fusion_test as sf  # noqa: E402

G_TO_LSB = 16384         # ±2g full scale, 16-bit signed => 16384 LSB/g
SHIFT = 10               # firmware does (|dx|+|dy|+|dz|) >> 10 per second
SLEEP_LABELS = {1, 2, 3, 5}
WAKE_LABEL = 0


def _split(line):
    line = line.strip()
    if not line:
        return None
    parts = line.replace(",", " ").split()
    return parts


def load_pairs(path, ncols):
    """Load whitespace/comma-separated rows -> list of float tuples."""
    out = []
    with open(path) as f:
        for line in f:
            p = _split(line)
            if not p or len(p) < ncols:
                continue
            try:
                out.append(tuple(float(x) for x in p[:ncols]))
            except ValueError:
                continue
    return out


def minute_index(t_sec):
    return int(math.floor(t_sec / 60.0))


def build_minutes(accel, hr, steps, labels):
    """Bin every signal into absolute 1-minute buckets keyed by floor(sec/60).

    Returns dict: minute -> {activity, steps, hr_mean, hr_std, true_sleep}
    Only minutes that carry a valid (scored) PSG label are returned.
    """
    # --- accel: decimate to 1 Hz (one representative read per integer second),
    #     then sum |Δ|>>10 across the minute, exactly like sleep_service.c. ---
    sec_read = {}  # integer second -> (x,y,z) in g, nearest sample to the second
    sec_best = {}  # integer second -> |frac distance| of the chosen sample
    for (t, x, y, z) in accel:
        s = int(round(t))
        d = abs(t - s)
        if s not in sec_best or d < sec_best[s]:
            sec_best[s] = d
            sec_read[s] = (x, y, z)

    activity_by_min = {}
    prev_s = None
    prev_xyz = None
    for s in sorted(sec_read):
        x, y, z = sec_read[s]
        if prev_s is not None and s - prev_s == 1:
            dx = int(round((x - prev_xyz[0]) * G_TO_LSB))
            dy = int(round((y - prev_xyz[1]) * G_TO_LSB))
            dz = int(round((z - prev_xyz[2]) * G_TO_LSB))
            mag = (abs(dx) + abs(dy) + abs(dz)) >> SHIFT
            m = minute_index(s)
            activity_by_min[m] = activity_by_min.get(m, 0) + mag
        prev_s = s
        prev_xyz = (x, y, z)

    # --- HR: per-minute mean + std from samples in the minute ---
    hr_by_min = {}
    for (t, bpm) in hr:
        if bpm <= 0 or bpm >= 240:
            continue
        hr_by_min.setdefault(minute_index(t), []).append(bpm)

    # --- steps: sum the step bins whose start falls in the minute ---
    steps_by_min = {}
    for (t, n) in steps:
        if n <= 0:
            continue
        steps_by_min[minute_index(t)] = steps_by_min.get(minute_index(t), 0) + int(n)

    # --- labels: PSG epoch(s) in the minute -> majority sleep/wake ---
    lbl_by_min = {}
    for (t, stage) in labels:
        st = int(stage)
        m = minute_index(t)
        lbl_by_min.setdefault(m, []).append(st)

    minutes = {}
    for m, stages in lbl_by_min.items():
        scored = [s for s in stages if s >= 0]  # drop -1 unscored
        if not scored:
            continue
        n_sleep = sum(1 for s in scored if s in SLEEP_LABELS)
        n_wake = sum(1 for s in scored if s == WAKE_LABEL)
        if n_sleep == 0 and n_wake == 0:
            continue
        true_sleep = n_sleep >= n_wake
        hrs = hr_by_min.get(m, [])
        if hrs:
            mean = sum(hrs) / len(hrs)
            var = sum((v - mean) ** 2 for v in hrs) / len(hrs)
            hr_mean = int(round(mean))
            hr_std = int(round(math.sqrt(var)))
        else:
            hr_mean = hr_std = 0
        minutes[m] = {
            "activity": activity_by_min.get(m, 0),
            "steps": min(steps_by_min.get(m, 0), 0xFFFF),
            "hr_mean": min(hr_mean, 255),
            "hr_std": min(hr_std, 255),
            "true_sleep": true_sleep,
        }
    return minutes


def estimate_resting_hr(minutes):
    """Robust resting-HR proxy: 10th percentile of per-minute mean HR."""
    hrs = sorted(d["hr_mean"] for d in minutes.values() if d["hr_mean"] > 0)
    if not hrs:
        return sf.State().resting_hr  # fall back to the firmware default (65)
    return hrs[max(0, int(0.10 * len(hrs)) - 1)]


def run_algo(minutes, offset_bpm):
    """Replay the minutes through the firmware algorithm exactly as shipped:
    seed resting HR = 65 (the firmware default) and let sleep_fusion learn the
    real resting baseline online. offset_bpm = a very large value disables the
    HR wake-veto (=> original accel-only behaviour)."""
    saved = sf.SF_WAKE_HR_OFFSET_BPM
    sf.SF_WAKE_HR_OFFSET_BPM = offset_bpm
    try:
        st = sf.State()  # seed 65, self-learns the baseline
        preds = {}
        for m in sorted(minutes):
            d = minutes[m]
            inp = sf.MinuteInput(
                activity_count=d["activity"], step_count=d["steps"],
                hr_mean_bpm=d["hr_mean"], hr_std_bpm=d["hr_std"], is_worn=True)
            sf.update(st, m * 60, inp)
            preds[m] = st.stage in (sf.Stage.LIGHT, sf.Stage.DEEP, sf.Stage.REM)
        return preds, st.learned_rhr
    finally:
        sf.SF_WAKE_HR_OFFSET_BPM = saved


def confusion(minutes, preds):
    """Return (TP, TN, FP, FN) where positive = SLEEP."""
    tp = tn = fp = fn = 0
    for m in minutes:
        t = minutes[m]["true_sleep"]
        p = preds[m]
        if t and p:
            tp += 1
        elif (not t) and (not p):
            tn += 1
        elif (not t) and p:
            fp += 1   # wake scored as sleep  <-- the bug's signature
        else:
            fn += 1
    return tp, tn, fp, fn


def kappa(tp, tn, fp, fn):
    n = tp + tn + fp + fn
    if n == 0:
        return 0.0
    po = (tp + tn) / n
    p_sleep = (tp + fn) * (tp + fp) / (n * n)
    p_wake = (tn + fp) * (tn + fn) / (n * n)
    pe = p_sleep + p_wake
    return (po - pe) / (1 - pe) if pe < 1 else 0.0


def metrics(tp, tn, fp, fn):
    n = tp + tn + fp + fn
    acc = (tp + tn) / n if n else 0
    sens = tp / (tp + fn) if (tp + fn) else 0   # sleep detection
    spec = tn / (tn + fp) if (tn + fp) else 0   # wake detection
    return acc, sens, spec, kappa(tp, tn, fp, fn)


def fmt_row(label, tp, tn, fp, fn):
    acc, sens, spec, k = metrics(tp, tn, fp, fn)
    return (f"{label:<22} acc={acc*100:5.1f}%  sleepSens={sens*100:5.1f}%  "
            f"wakeSpec={spec*100:5.1f}%  kappa={k:5.2f}  "
            f"FP(wake->sleep)={fp:4d}  FN={fn:4d}")


def main():
    data_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "data")
    label_files = sorted(glob.glob(os.path.join(data_dir, "*_labeled_sleep.txt")))
    if not label_files:
        print(f"No data in {data_dir}")
        return

    agg = {"on": [0, 0, 0, 0], "off": [0, 0, 0, 0]}
    OFFSET = sf.SF_WAKE_HR_OFFSET_BPM
    print(f"Subjects: {len(label_files)}  (data: {data_dir})")
    print(f"Firmware as shipped: seed rHR=65, online learner, veto = HR > "
          f"learned_rhr + {OFFSET} bpm\n")
    print(f"{'subject':>9} {'min':>5} {'hrcov':>6} {'lrnRHR':>6} {'trueTST':>7} "
          f"{'mode':>9}  metrics")

    for lf in label_files:
        sid = os.path.basename(lf).split("_")[0]
        base = os.path.join(data_dir, sid)
        try:
            accel = load_pairs(base + "_acceleration.txt", 4)
            hr = load_pairs(base + "_heartrate.txt", 2)
            steps = load_pairs(base + "_steps.txt", 2)
            labels = load_pairs(lf, 2)
        except FileNotFoundError:
            continue
        minutes = build_minutes(accel, hr, steps, labels)
        if not minutes:
            continue
        n = len(minutes)
        hrcov = sum(1 for d in minutes.values() if d["hr_mean"] > 0) / n
        true_tst = sum(1 for d in minutes.values() if d["true_sleep"])

        preds_off, _ = run_algo(minutes, 100000)   # veto disabled = original
        preds_on, lrn = run_algo(minutes, OFFSET)  # veto, self-learned rHR

        c_off = confusion(minutes, preds_off)
        c_on = confusion(minutes, preds_on)
        for i in range(4):
            agg["off"][i] += c_off[i]
            agg["on"][i] += c_on[i]

        head = f"{sid:>9} {n:>5} {hrcov*100:5.0f}% {lrn:>6} {true_tst:>7} "
        print(head + f"{'OFF(orig)':>9}  " + fmt_row("", *c_off))
        print(" " * len(head) + f"{'ON(veto)':>9}  " + fmt_row("", *c_on))

    print("\n" + "=" * 100)
    print("POOLED (all minutes, all subjects):")
    print("  " + fmt_row("OFF  veto disabled (original accel-only)", *agg["off"]))
    print("  " + fmt_row(f"ON   veto = learned_rhr + {OFFSET} bpm", *agg["on"]))

    fp_off, fp_on = agg["off"][2], agg["on"][2]
    fn_off, fn_on = agg["off"][3], agg["on"][3]
    if fp_off:
        print(f"\n  wake-misread-as-sleep minutes (the bug's signature):")
        print(f"    original : {fp_off}")
        print(f"    + veto   : {fp_on}   ({100*(fp_off-fp_on)/fp_off:.0f}% fewer)")
        print(f"  sleep minutes lost to the veto (sensitivity cost): "
              f"{fn_on - fn_off}")


if __name__ == "__main__":
    main()
