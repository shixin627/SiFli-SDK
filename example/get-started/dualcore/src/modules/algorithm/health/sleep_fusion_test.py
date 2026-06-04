"""
Reference-port of sleep_fusion.c in Python. Mirrors the algorithm 1:1 so
we can sanity-check the state machine against synthetic minute traces
without firing up the watch. Not part of the firmware build — just a
correctness aid for the in-tree C implementation. Run with `python
sleep_fusion_test.py`.
"""

from dataclasses import dataclass, field
from enum import IntEnum

# Tunables — kept in sync with sleep_fusion.c
SF_WINDOW_MIN = 7
SF_HR_HISTORY_MIN = 10
SF_CK_WEIGHTS_Q10 = [40, 60, 100, 160, 260, 420, 1024]
SF_SLEEP_SCORE_THRESH = 400
SF_STEPS_FORCE_WAKE = 1
SF_STEP_CORROB_SCORE = 100     # steps only force wake if score >= this (real motion)
SF_WAKE_HR_OFFSET_BPM = 20      # HR > learned_rhr + 20 => awake
SF_HR_ARTEFACT_MAX = 120       # bursts above this are PPG artefact -> dropped
SF_WAKE_HR_CONSEC_MIN = 2      # consecutive elevated readings needed to arm veto
SF_RHR_DOWN_SHIFT = 3          # est -= (est - hr) >> 3  (~1/8 of the gap)
SF_RHR_LEAK_PERIOD_MIN = 4     # minutes mildly-above per +1 bpm upward leak
SF_RHR_MIN = 40
SF_RHR_MAX = 110
SF_WAKE_HR_HOLD_MIN = 12
SF_ENTER_SLEEP_MIN = 3
SF_EXIT_SLEEP_MIN = 2
SF_DEEP_ACTIVITY_MAX = 50
SF_REM_ACTIVITY_MAX = 400
SF_REM_HR_STD_MIN = 3
SF_DEEP_HR_DROP_PCT = 5
SF_REM_HR_NEAR_PCT = 8


class Stage(IntEnum):
    AWAKE = 0
    LIGHT = 1
    DEEP = 2
    REM = 3
    NOT_WORN = 4


@dataclass
class MinuteInput:
    activity_count: int = 0
    step_count: int = 0
    hr_mean_bpm: int = 0
    hr_std_bpm: int = 0
    is_worn: bool = True


@dataclass
class State:
    resting_hr: int = 65          # configured seed / enable
    learned_rhr: int = 65         # online resting-HR estimate (seeded from above)
    rhr_leak_acc: int = 0
    activity_hist: list = field(default_factory=list)
    hr_hist: list = field(default_factory=list)
    consec_sleep: int = 0
    consec_wake: int = 0
    wake_hr_hold: int = 0
    hr_elev_consec: int = 0
    stage: Stage = Stage.AWAKE
    in_stage: int = 0
    total: int = 0
    deep: int = 0
    rem: int = 0
    light: int = 0
    waso: int = 0
    onset_utc: int = 0
    last_wake_utc: int = 0
    ck_score: int = 0
    hr_baseline: int = 0

    def __post_init__(self):
        self.learned_rhr = self.resting_hr  # mirror C init: seed from resting_hr


def hr_elevated_and_learn(state, hr):
    """Mirror of prv_hr_elevated_and_learn in sleep_fusion.c: returns True when
    HR sits > SF_WAKE_HR_OFFSET_BPM above the learned resting estimate, and
    advances that estimate (adopt lows fast, leak up slowly, freeze while
    elevated)."""
    if hr <= 0 or state.resting_hr <= 0:
        return False
    rhr = state.learned_rhr
    elevated = hr > rhr + SF_WAKE_HR_OFFSET_BPM
    if hr < rhr:
        step = (rhr - hr) >> SF_RHR_DOWN_SHIFT
        rhr -= step if step else 1
        state.rhr_leak_acc = 0
    elif not elevated:
        state.rhr_leak_acc += 1
        if state.rhr_leak_acc >= SF_RHR_LEAK_PERIOD_MIN:
            rhr += 1
            state.rhr_leak_acc = 0
    else:
        state.rhr_leak_acc = 0
    state.learned_rhr = max(SF_RHR_MIN, min(SF_RHR_MAX, rhr))
    return elevated


def cole_kripke(hist):
    """Pairing: newest gets highest weight (W[-1]), older samples step down.

    hist is ordered [oldest, ..., newest]; weights ordered [40, 60, ..., 1024].
    For n samples, pair hist[-1] with W[-1], hist[-2] with W[-2], etc.
    Stop after min(n, len(weights)) pairs."""
    if not hist:
        return 0
    n = min(len(hist), len(SF_CK_WEIGHTS_Q10))
    s = 0
    for k in range(n):
        s += hist[-1 - k] * SF_CK_WEIGHTS_Q10[-1 - k]
    return s >> 10


def hr_baseline(hr_hist, resting):
    if not hr_hist:
        return resting
    return sorted(hr_hist)[len(hr_hist) // 2]


def classify(inp, baseline):
    if inp.hr_mean_bpm == 0 or baseline == 0:
        return Stage.LIGHT
    pct = (inp.hr_mean_bpm - baseline) * 100 // baseline
    if inp.activity_count <= SF_DEEP_ACTIVITY_MAX and pct <= -SF_DEEP_HR_DROP_PCT:
        return Stage.DEEP
    if (inp.activity_count <= SF_REM_ACTIVITY_MAX
            and abs(pct) <= SF_REM_HR_NEAR_PCT
            and inp.hr_std_bpm >= SF_REM_HR_STD_MIN):
        return Stage.REM
    return Stage.LIGHT


def update(state, utc, inp):
    prev = state.stage
    if not inp.is_worn:
        state.consec_sleep = state.consec_wake = 0
        state.wake_hr_hold = 0
        state.hr_elev_consec = 0
        state.stage = Stage.NOT_WORN
        state.in_stage = 1 if prev != Stage.NOT_WORN else state.in_stage + 1
        state.ck_score = 0
        state.hr_baseline = 0
        return

    # Drop an implausibly high burst as a PPG artefact (mirror hr_clean in C).
    hr_clean = 0 if inp.hr_mean_bpm > SF_HR_ARTEFACT_MAX else inp.hr_mean_bpm

    state.activity_hist.append(inp.activity_count)
    if len(state.activity_hist) > SF_WINDOW_MIN:
        state.activity_hist.pop(0)
    if hr_clean:
        state.hr_hist.append(hr_clean)
        if len(state.hr_hist) > SF_HR_HISTORY_MIN:
            state.hr_hist.pop(0)

    state.ck_score = cole_kripke(state.activity_hist)
    state.hr_baseline = hr_baseline(state.hr_hist, state.resting_hr)

    # HR wake-veto: pulse > offset above the self-learned resting baseline on
    # a still wrist => sedentary wake, not sleep. Gated on HR being present,
    # held a few minutes to bridge sparse background-PPG gaps.
    hr_elevated = hr_elevated_and_learn(state, hr_clean)
    if hr_clean == 0:
        # HR absent / artefact-rejected — bridge the gap; leave the run intact.
        if state.wake_hr_hold > 0:
            state.wake_hr_hold -= 1
    elif hr_elevated:
        # Sustained elevation only: a lone spike (REM/arousal/artefact) must not
        # veto. Arm the hold once SF_WAKE_HR_CONSEC_MIN elevated readings stack.
        state.hr_elev_consec = min(0xFF, state.hr_elev_consec + 1)
        if state.hr_elev_consec >= SF_WAKE_HR_CONSEC_MIN:
            state.wake_hr_hold = SF_WAKE_HR_HOLD_MIN
    else:
        # present, non-elevated HR clears both the run and the veto
        state.hr_elev_consec = 0
        state.wake_hr_hold = 0

    # Steps force wake only when corroborated by accelerometer motion; a bare
    # pedometer count (false-fires at rest) no longer blocks sleep onset.
    steps_with_motion = (inp.step_count >= SF_STEPS_FORCE_WAKE
                         and state.ck_score >= SF_STEP_CORROB_SCORE)
    vote_sleep = (state.ck_score < SF_SLEEP_SCORE_THRESH
                  and not steps_with_motion
                  and state.wake_hr_hold == 0)
    if vote_sleep:
        state.consec_sleep += 1
        state.consec_wake = 0
    else:
        state.consec_wake += 1
        state.consec_sleep = 0

    is_sleep = state.stage in (Stage.LIGHT, Stage.DEEP, Stage.REM)
    if not is_sleep:
        if state.consec_sleep >= SF_ENTER_SLEEP_MIN:
            nxt = classify(inp, state.hr_baseline)
        else:
            nxt = Stage.AWAKE
    else:
        if state.consec_wake >= SF_EXIT_SLEEP_MIN:
            nxt = Stage.AWAKE
        else:
            nxt = classify(inp, state.hr_baseline)

    if nxt == prev:
        state.in_stage += 1
    else:
        prev_was_sleep = prev in (Stage.LIGHT, Stage.DEEP, Stage.REM)
        nxt_is_sleep = nxt in (Stage.LIGHT, Stage.DEEP, Stage.REM)
        if not prev_was_sleep and nxt_is_sleep and state.onset_utc == 0:
            state.onset_utc = utc
        if prev_was_sleep and not nxt_is_sleep:
            state.last_wake_utc = utc
        state.stage = nxt
        state.in_stage = 1

    had_onset = state.onset_utc != 0
    if nxt == Stage.LIGHT:
        state.light += 1; state.total += 1
    elif nxt == Stage.DEEP:
        state.deep += 1; state.total += 1
    elif nxt == Stage.REM:
        state.rem += 1; state.total += 1
    elif nxt == Stage.AWAKE and had_onset:
        state.waso += 1


# -------- Scenarios ----------------------------------------------------

def trace(name, inputs):
    s = State()
    print(f"\n=== {name} ===")
    print(f"{'min':>4} {'act':>5} {'step':>4} {'hr':>3} {'std':>3} {'worn':>4} "
          f"{'ck':>4} {'rhr':>3} {'hold':>4} {'stage':>8} {'in':>3} D={'':>2} R={'':>2} L={'':>2}")
    for i, inp in enumerate(inputs, start=1):
        update(s, i * 60, inp)
        print(f"{i:>4} {inp.activity_count:>5} {inp.step_count:>4} "
              f"{inp.hr_mean_bpm:>3} {inp.hr_std_bpm:>3} {int(inp.is_worn):>4} "
              f"{s.ck_score:>4} {s.hr_baseline:>3} {s.wake_hr_hold:>4} {s.stage.name:>8} "
              f"{s.in_stage:>3} D={s.deep:>2} R={s.rem:>2} L={s.light:>2} "
              f"WASO={s.waso}")
    return s


# 1) Going to sleep: 10 min low activity + dropping HR -> should enter Deep
sleep_in = [
    MinuteInput(activity_count=200, hr_mean_bpm=72, hr_std_bpm=2) for _ in range(2)
] + [
    MinuteInput(activity_count=30, hr_mean_bpm=60, hr_std_bpm=1) for _ in range(10)
]

# 2) Wake up: 5 min Deep, then steps trigger -> AWAKE within 2 min
wake_up = [
    MinuteInput(activity_count=20, hr_mean_bpm=58, hr_std_bpm=1) for _ in range(5)
] + [
    MinuteInput(activity_count=3000, step_count=30, hr_mean_bpm=75) for _ in range(3)
]

# 3) REM-like: low motion, HR near resting, HR variability
rem = [
    MinuteInput(activity_count=20, hr_mean_bpm=60, hr_std_bpm=1) for _ in range(4)
] + [
    MinuteInput(activity_count=100, hr_mean_bpm=66, hr_std_bpm=5) for _ in range(6)
]

# 4) Not worn -> should NOT_WORN immediately
not_worn = [
    MinuteInput(activity_count=30, hr_mean_bpm=60, is_worn=False) for _ in range(3)
] + [
    MinuteInput(activity_count=30, hr_mean_bpm=60, is_worn=True) for _ in range(5)
]

# 5) Train ride: dead-still wrist, no steps, but HR ~88 (well above resting
#    65). The bug this fixes — accel alone reads it as sleep. With the HR
#    wake-veto it must stay AWAKE for the whole ride. HR is sparse while awake
#    (~one reading per 15 min), so the hold has to bridge the gaps: 4 min of
#    HR present, then 11 min absent, repeated.
train = []
for _ in range(8):  # ~2 h ride
    train += [MinuteInput(activity_count=30, hr_mean_bpm=88, hr_std_bpm=2) for _ in range(4)]
    train += [MinuteInput(activity_count=30, hr_mean_bpm=0) for _ in range(11)]

# 6) Backstop: somehow we slipped into Light during a long HR-absent gap
#    (e.g. PPG missed several bursts). The moment elevated HR returns — which
#    is exactly what the dense burst that "asleep" turns on provides — we must
#    exit to AWAKE within SF_EXIT_SLEEP_MIN.
slip_then_rescue = [
    MinuteInput(activity_count=30, hr_mean_bpm=0) for _ in range(20)  # enter Light, no HR
] + [
    MinuteInput(activity_count=30, hr_mean_bpm=88, hr_std_bpm=2) for _ in range(5)  # HR rescues
]

# 7) Phantom overnight steps: dead-still wrist (score ~0), low sleeping HR, but
#    the pedometer false-counts 1 step every minute. THIS is the reported bug —
#    the old `any step => wake` gate kept the whole night AWAKE. With step
#    corroboration (steps must be backed by real accel motion) it must sleep.
phantom_steps = [
    MinuteInput(activity_count=20, step_count=1, hr_mean_bpm=58, hr_std_bpm=1)
    for _ in range(12)
]

# 8) Lone HR spike during sleep — a brief REM/arousal bump or PPG artefact. One
#    elevated burst in an otherwise calm night must NOT wake the sleeper; only
#    SUSTAINED elevation does. (Old single-burst veto clipped real REM here.)
lone_spike = [
    MinuteInput(activity_count=20, hr_mean_bpm=58, hr_std_bpm=1) for _ in range(6)
] + [
    MinuteInput(activity_count=20, hr_mean_bpm=88, hr_std_bpm=2)  # single elevated minute
] + [
    MinuteInput(activity_count=20, hr_mean_bpm=58, hr_std_bpm=1) for _ in range(6)
]

# 9) Artefact-high burst: a still wrist reading 126 bpm is physiologically
#    impossible — it's PPG motion artefact, must be dropped, and must neither
#    veto sleep nor pollute the learned baseline.
artefact = [
    MinuteInput(activity_count=20, hr_mean_bpm=58, hr_std_bpm=1) for _ in range(6)
] + [
    MinuteInput(activity_count=20, hr_mean_bpm=126, hr_std_bpm=2) for _ in range(4)
]


def check(name, cond):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}")
    if not cond:
        raise SystemExit(f"ASSERTION FAILED: {name}")


# Guarded so this file can be imported as a module (e.g. by the PhysioNet
# validation harness) without running the synthetic scenarios on import.
if __name__ == "__main__":
    trace("Falling asleep + Deep stage", sleep_in)
    trace("Wake-up forced by walking", wake_up)
    trace("REM-like burst", rem)
    trace("Not worn -> reset", not_worn)
    s_train = trace("Train ride (still + elevated HR)", train)
    s_rescue = trace("Slip into sleep then HR rescue", slip_then_rescue)
    s_phantom = trace("Phantom overnight steps (still wrist)", phantom_steps)
    s_spike = trace("Lone HR spike during sleep", lone_spike)
    s_artefact = trace("Artefact-high burst (126 bpm, still)", artefact)

    # -------- Assertions ----------------------------------------------
    print("\n=== Assertions ===")

    # No-regression: the canonical fall-asleep trace must still detect sleep.
    s_sleep = State()
    for i, inp in enumerate(sleep_in, start=1):
        update(s_sleep, i * 60, inp)
    check("fall-asleep still detects sleep (total > 0)", s_sleep.total > 0)
    check("fall-asleep reaches Deep", s_sleep.deep > 0)

    # The fix: a train ride must never be scored as sleep.
    check("train ride stays AWAKE", s_train.stage == Stage.AWAKE)
    check("train ride accumulates zero sleep", s_train.total == 0)

    # Backstop: a false sleep is corrected once HR returns. Re-run the rescue
    # trace step by step so we can measure the wake latency directly: HR turns
    # elevated at minute `hr_return` (the 21st input here); assert we are back
    # to AWAKE within SF_EXIT_SLEEP_MIN minutes of that.
    hr_return = 20  # 0-based index of the first elevated-HR minute (21st input)
    s_r = State()
    wake_minute = None
    for i, inp in enumerate(slip_then_rescue):
        update(s_r, (i + 1) * 60, inp)
        if i >= hr_return and s_r.stage == Stage.AWAKE and wake_minute is None:
            wake_minute = i
    check("HR rescue exits to AWAKE", s_rescue.stage == Stage.AWAKE)
    # It did briefly sleep during the gap (proves the rescue, not a no-op)...
    check("rescue trace did enter sleep first", s_rescue.total > 0)
    # ...and recovered promptly once HR returned.
    check("HR rescue woke after HR returned", wake_minute is not None)
    check("HR rescue is prompt (<= SF_EXIT_SLEEP_MIN min after HR returns)",
          wake_minute is not None and (wake_minute - hr_return) <= SF_EXIT_SLEEP_MIN)

    # The reported bug: phantom pedometer steps on a still wrist must NOT block
    # sleep onset. A bare step count is no longer trusted without accel motion.
    check("phantom steps still fall asleep (total > 0)", s_phantom.total > 0)

    # A single elevated burst (REM/arousal spike or artefact) must not wake the
    # sleeper — only sustained elevation vetoes.
    check("lone HR spike does not wake the sleeper",
          s_spike.stage in (Stage.LIGHT, Stage.DEEP, Stage.REM))

    # An artefact-high burst is dropped: it neither wakes the sleeper nor drags
    # the learned baseline up toward the garbage value.
    check("artefact-high burst does not wake the sleeper",
          s_artefact.stage in (Stage.LIGHT, Stage.DEEP, Stage.REM))
    check("artefact-high burst left baseline near real sleeping HR",
          s_artefact.hr_baseline <= 70)

    print("\nAll assertions passed.")
