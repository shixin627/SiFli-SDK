"""
Synthetic stress test for the ONE failure mode the PhysioNet data cannot show
(it is all in-bed, at night): a long, sustained, sedentary-but-awake stretch —
a train ride — where the self-learning resting baseline could drift up to the
elevated HR and silently switch the veto off after ~15 min.

Scenario fed through the SHIPPED firmware algorithm (validate_physionet.run_algo,
which seeds rHR=65 and learns online): 30 min sleep (baseline converges low) ->
3 min walking (steps force wake) -> 2 h train (still wrist, no steps, elevated
HR). With the freeze-on-elevated rule the learned baseline must NOT drift, so
the veto must hold for the WHOLE ride.
"""
import validate_physionet as V


def build_scenario(train_hr, train_min=120):
    mins = {}
    m = 0
    for _ in range(30):   # real sleep — baseline converges toward ~58
        mins[m] = dict(activity=20, steps=0, hr_mean=58, hr_std=1, true_sleep=True)
        m += 1
    for _ in range(3):    # walk to the platform — steps force wake
        mins[m] = dict(activity=3000, steps=40, hr_mean=82, hr_std=3, true_sleep=False)
        m += 1
    t0 = m
    for _ in range(train_min):  # train: still, no steps, elevated HR
        mins[m] = dict(activity=30, steps=0, hr_mean=train_hr, hr_std=2, true_sleep=False)
        m += 1
    return mins, t0, t0 + train_min


print("Freeze-on-elevated: a 2 h train must NOT be logged as sleep, and the")
print("learned resting baseline must NOT drift up to the train HR.\n")
print(f"shipped offset = rhr + {V.sf.SF_WAKE_HR_OFFSET_BPM} bpm\n")
print(f"{'trainHR':>7} {'falseSleepMin':>14} {'learnedRHR@end':>15}  result")

fails = 0
for train_hr in (88, 95, 105):   # clearly-elevated daytime wake
    mins, t0, t1 = build_scenario(train_hr)
    preds, lrn_end = V.run_algo(mins, V.sf.SF_WAKE_HR_OFFSET_BPM)
    false_sleep = sum(1 for m in range(t0, t1) if preds[m])
    drift = lrn_end > 70   # baseline should stay near the ~58 sleep level
    ok = (false_sleep <= 3) and not drift
    fails += 0 if ok else 1
    print(f"{train_hr:>7} {false_sleep:>14d} {lrn_end:>15d}  "
          f"{'ok' if ok else 'FAIL'}")

# Sanity: a barely-elevated 'train' (HR just at threshold) is the inherent grey
# zone and MAY read as sleep — that is not a drift failure, just ambiguity.
print(f"\n{'ALL CLEAR — veto holds, no baseline drift' if fails == 0 else str(fails)+' FAILURES'}")
