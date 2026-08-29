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
SF_SLEEP_SCORE_THRESH = 120     # 2026-08-26: 400 was unreachable on real days
SF_STEPS_FORCE_WAKE = 1
SF_STEP_CORROB_SCORE = 100     # steps only force wake if score >= this (real motion)
SF_WAKE_HR_OFFSET_BPM = 12      # HR > STABLE rhr reference + 12 => awake
SF_HR_ARTEFACT_MAX = 120       # bursts above this are PPG artefact -> dropped
SF_WAKE_HR_CONSEC_AWAKE = 2    # fresh elevated BURSTS to arm veto while awake
SF_WAKE_HR_CONSEC_ASLEEP = 2   # ... while already asleep (artefact immunity)
SF_RHR_DOWN_SHIFT = 3          # est -= (est - hr) >> 3  (~1/8 of the gap)
SF_RHR_LEAK_PERIOD_MIN = 4     # minutes mildly-above per +1 bpm upward leak
SF_RHR_MIN = 40
SF_RHR_MAX = 110
# 2026-08-26: the countdown hold is GONE — the veto is latched. It used to
# expire between bursts and sleep leaked in through every expiry; on six real
# days (2026-08-20..25) replacing it with a latch took daytime false-sleep from
# 16% to 1.6%, more than any threshold change did.
SF_RHR_HIST_DAYS = 3
SF_RHR_HIST_BINS = 64
SF_RHR_HIST_BASE = 30
SF_RHR_HIST_PCT = 5
SF_RHR_HIST_MIN_N = 60
# 參考可以快速下修、只能緩慢上修。實測(2026-08-29,真值 03:30-09:45):沒有這個
# 約束時,重啟後的直方圖視窗裡只有清醒資料,p5 給出 70(真實靜息 50),門檻拉到 82,
# 三小時 70-92 的清醒心率全部溜過去。「沒有睡眠的視窗」不能拉高靜息心率。
SF_RHR_UP_LEAK_MIN = 60
SF_MIN_SESSION_MIN = 30
SF_ENTER_SLEEP_MIN = 15
SF_EXIT_SLEEP_MIN = 5
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
    # True = NEW burst / live foreground minute; False = held window repeat
    # (staging-only — veto-neutral). Mirrors sleep_fusion.h hr_is_fresh.
    hr_is_fresh: bool = True
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
    wake_hr_latched: bool = False
    rhr_seeded: bool = False
    rhr_hist: list = field(default_factory=lambda: [[0]*SF_RHR_HIST_BINS
                                                    for _ in range(SF_RHR_HIST_DAYS)])
    rhr_hist_day: int = 0
    rhr_ref_bpm: int = 0        # veto 實際比對的參考;0 = 還沒建立
    rhr_up_acc: int = 0
    session_min: int = 0
    pend: dict = field(default_factory=lambda: dict(total=0, deep=0, rem=0, light=0))
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


def rhr_hist_push(state, hr):
    """今日直方圖:2 bpm 一格,base 30。"""
    if hr < SF_RHR_HIST_BASE:
        return
    b = (hr - SF_RHR_HIST_BASE) >> 1
    if b < SF_RHR_HIST_BINS:
        state.rhr_hist[state.rhr_hist_day][b] += 1


def rhr_hist_p5(state):
    """滾動 3 日窗的第 5 百分位 —— 這是「證據」,不是參考。資料太少回 0。

    這取代了會漂的 learned_rhr 當 veto 的錨點。實測 2026-08-20..25 六天,
    這個值每天都是 48-51,而 learned_rhr 在同幾天裡從 49(夜)盪到 65(日)
    —— 那個擺盪就是「白天整天被判成睡著」的直接原因。
    """
    total = sum(sum(d) for d in state.rhr_hist)
    if total < SF_RHR_HIST_MIN_N:
        return 0
    want = total * SF_RHR_HIST_PCT // 100
    seen = 0
    for b in range(SF_RHR_HIST_BINS):
        seen += sum(state.rhr_hist[d][b] for d in range(SF_RHR_HIST_DAYS))
        if seen > want:
            return SF_RHR_HIST_BASE + (b << 1)
    return 0


def rhr_ref_step(state):
    """把這一分鐘的直方圖證據併進參考:下修快、上修慢。回傳有沒有變動。"""
    p5 = rhr_hist_p5(state)
    if not p5:
        return False
    if state.rhr_ref_bpm == 0 or p5 < state.rhr_ref_bpm:
        state.rhr_ref_bpm = p5          # 新低永遠是更好的證據
        state.rhr_up_acc = 0
        return True
    if p5 > state.rhr_ref_bpm:
        state.rhr_up_acc += 1
        if state.rhr_up_acc >= SF_RHR_UP_LEAK_MIN:
            state.rhr_ref_bpm += 1
            state.rhr_up_acc = 0
            return True
        return False
    state.rhr_up_acc = 0
    return False


def rhr_reference(state):
    return state.rhr_ref_bpm


def hr_elevated_and_learn(state, hr):
    """Mirror of prv_hr_elevated_and_learn in sleep_fusion.c: returns True when
    HR sits > SF_WAKE_HR_OFFSET_BPM above the learned resting estimate, and
    advances that estimate (adopt lows fast, leak up slowly, freeze while
    elevated)."""
    if hr <= 0 or state.resting_hr <= 0:
        return False
    rhr = state.learned_rhr
    if not state.rhr_seeded:
        # 剛好落在門檻上,不能更低:更低的話第一筆就算「偏高」,而學習器
        # 在偏高時凍結上漂 —— 種子太低會把 veto 永久卡住。
        state.learned_rhr = max(SF_RHR_MIN, hr - SF_WAKE_HR_OFFSET_BPM)
        state.rhr_seeded = True
        state.rhr_leak_acc = 0
        rhr = state.learned_rhr
    ref = rhr_reference(state) or rhr
    elevated = hr > ref + SF_WAKE_HR_OFFSET_BPM
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


def commit_session(state):
    for k in ('deep', 'rem', 'light', 'total'):
        setattr(state, k, getattr(state, k) + state.pend[k])
        state.pend[k] = 0


def drop_session(state):
    for k in state.pend:
        state.pend[k] = 0
    state.session_min = 0


def update(state, utc, inp):
    prev = state.stage
    if not inp.is_worn:
        state.consec_sleep = state.consec_wake = 0
        state.wake_hr_latched = False
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
        # 每分鐘都餵直方圖(不只 fresh),否則 warm-up 要 7 小時才夠 60 筆
        rhr_hist_push(state, hr_clean)
        rhr_ref_step(state)

    state.ck_score = cole_kripke(state.activity_hist)
    state.hr_baseline = hr_baseline(state.hr_hist, state.resting_hr)

    # HR wake-veto: pulse > offset above the self-learned resting baseline on
    # a still wrist => sedentary wake, not sleep. Counters + learner advance
    # ONLY on FRESH readings (distinct bursts); a held window repeat is the
    # same observation again — veto-neutral (no count, no clear, no decay).
    # Arm threshold is asymmetric: instant while awake (train/desk), sustained
    # (3 bursts) once asleep so garbage bursts can't kick a sleeper out.
    if hr_clean != 0 and inp.hr_is_fresh:
        elevated = hr_elevated_and_learn(state, hr_clean)
        if elevated:
            asleep_now = state.stage in (Stage.LIGHT, Stage.DEEP, Stage.REM)
            arm_at = SF_WAKE_HR_CONSEC_ASLEEP if asleep_now else SF_WAKE_HR_CONSEC_AWAKE
            state.hr_elev_consec = min(0xFF, state.hr_elev_consec + 1)
            if state.hr_elev_consec >= arm_at:
                state.wake_hr_latched = True
        else:
            # fresh, non-elevated HR clears both the run and the veto
            state.hr_elev_consec = 0
            state.wake_hr_latched = False
    # HR absent, or a held repeat: the latch stays exactly as it is. Nothing new
    # was measured, so nothing changed — only the next fresh burst may move it.

    # Steps force wake only when corroborated by accelerometer motion; a bare
    # pedometer count (false-fires at rest) no longer blocks sleep onset.
    steps_with_motion = (inp.step_count >= SF_STEPS_FORCE_WAKE
                         and state.ck_score >= SF_STEP_CORROB_SCORE)
    vote_sleep = (state.ck_score < SF_SLEEP_SCORE_THRESH
                  and not steps_with_motion
                  and not state.wake_hr_latched)
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
            drop_session(state)      # 沒撐到 SF_MIN_SESSION_MIN 的不是睡眠
        state.stage = nxt
        state.in_stage = 1

    had_onset = state.onset_utc != 0
    key = {Stage.LIGHT: 'light', Stage.DEEP: 'deep', Stage.REM: 'rem'}.get(nxt)
    if key is None:
        if nxt == Stage.AWAKE and had_onset:
            state.waso += 1
        return
    # 2026-08-26:每日累加只在 session 內、而且要等 session 撐過
    # SF_MIN_SESSION_MIN 才落帳。舊版對任何非 AWAKE 分鐘無條件累加,
    # 白天靜坐一天就記成睡了一天(2026-08-21 報 1164 分 = 19.4 小時)。
    provisional = state.session_min < SF_MIN_SESSION_MIN   # 先判,再自增(對齊 C)
    state.session_min += 1
    if provisional:
        state.pend[key] += 1
        state.pend['total'] += 1
        if state.session_min >= SF_MIN_SESSION_MIN:
            commit_session(state)      # 撐過去了,暫記的分鐘是真的
    else:
        setattr(state, key, getattr(state, key) + 1)
        state.total += 1


# -------- Scenarios ----------------------------------------------------

def trace(name, inputs):
    s = State()
    print(f"\n=== {name} ===")
    print(f"{'min':>4} {'act':>5} {'step':>4} {'hr':>3} {'std':>3} {'worn':>4} "
          f"{'ck':>4} {'rhr':>3} {'lat':>4} {'stage':>8} {'in':>3} D={'':>2} R={'':>2} L={'':>2}")
    for i, inp in enumerate(inputs, start=1):
        update(s, i * 60, inp)
        print(f"{i:>4} {inp.activity_count:>5} {inp.step_count:>4} "
              f"{inp.hr_mean_bpm:>3} {inp.hr_std_bpm:>3} {int(inp.is_worn):>4} "
              f"{s.ck_score:>4} {s.hr_baseline:>3} {int(s.wake_hr_latched):>4} {s.stage.name:>8} "
              f"{s.in_stage:>3} D={s.deep:>2} R={s.rem:>2} L={s.light:>2} "
              f"WASO={s.waso}")
    return s


# 1) Going to sleep: 10 min low activity + dropping HR -> should enter Deep
sleep_in = [
    MinuteInput(activity_count=200, hr_mean_bpm=72, hr_std_bpm=2) for _ in range(2)
] + [
    # 入睡後心率真的往下掉(60 -> 52),這才是深睡的樣子。舊 fixture 是
    # 60 bpm 一路平,只因為 hr_hist 中位數還沒收斂才「進到 Deep」——
    # 分期的 baseline 是最近 10 筆 HR 的中位數,平的訊號一收斂就永遠
    # 到不了 Deep。這一輪只修睡/醒,分期的自我參照沒動,見檔頭註解。
    MinuteInput(activity_count=30, hr_mean_bpm=(60 if i < 25 else 52),
                hr_std_bpm=1, hr_is_fresh=(i % 10 == 0)) for i in range(60)
]

# 2) Wake up: 5 min Deep, then steps trigger -> AWAKE within 2 min
wake_up = [
    MinuteInput(activity_count=20, hr_mean_bpm=58, hr_std_bpm=1,
                hr_is_fresh=(i % 10 == 0)) for i in range(50)
] + [
    MinuteInput(activity_count=3000, step_count=30, hr_mean_bpm=75) for _ in range(3)
]

# 3) REM-like: low motion, HR near resting, HR variability
rem = [
    MinuteInput(activity_count=20, hr_mean_bpm=60, hr_std_bpm=1,
                hr_is_fresh=(i % 10 == 0)) for i in range(40)
] + [
    MinuteInput(activity_count=100, hr_mean_bpm=66, hr_std_bpm=5,
                hr_is_fresh=(i % 10 == 0)) for i in range(30)
]

# 4) Not worn -> should NOT_WORN immediately
not_worn = [
    MinuteInput(activity_count=30, hr_mean_bpm=60, is_worn=False) for _ in range(3)
] + [
    MinuteInput(activity_count=30, hr_mean_bpm=60, is_worn=True) for _ in range(5)
]

def dense_bursts(n, hr, act=20, std=2):
    """n × 3-min background bursts as sleep_service feeds them: one FRESH
    minute (a new burst) + 2 held window repeats."""
    out = []
    for _ in range(n):
        out.append(MinuteInput(activity_count=act, hr_mean_bpm=hr,
                               hr_std_bpm=std, hr_is_fresh=True))
        out += [MinuteInput(activity_count=act, hr_mean_bpm=hr,
                            hr_std_bpm=std, hr_is_fresh=False)] * 2
    return out


# 5) Train ride: dead-still wrist, no steps, but HR ~88 (well above resting
#    65). The bug this fixes — accel alone reads it as sleep. With the HR
#    wake-veto it must stay AWAKE for the whole ride. Awake sampling is
#    sparse (~one burst per 15 min): 1 fresh minute + 3 held repeats + 11
#    absent, repeated. The single fresh elevated burst must arm instantly
#    (SF_WAKE_HR_CONSEC_AWAKE=1) and the hold must survive to the next burst
#    WITHOUT decaying on the held minutes, or sleep leaks in between.
#    PRECONDITION (2026-08-26): the veto now measures elevation against the
#    STABLE reference, which is this wearer's own 5th-percentile HR over the
#    trailing three days. So the watch must have seen the wearer at rest at
#    least once. That is true of every real day and false only for the first
#    hour after a factory-fresh boot — an honest limit, and a far better one
#    than the fixed 65 bpm this used to assume for everyone (that assumption is
#    what put the wearer's HR threshold at 85 bpm and called six real days
#    "asleep" from morning to night). The warm-up below is that rest period.
# 暖身期 = 一段真的睡眠(讓穩定參考看過這個人的靜息)+ 起床走到車站
# (讓狀態機在上車前確實回到 AWAKE,否則測的就不是「上車後被誤判入睡」)。
TRAIN_WARMUP_MIN = 90 + 15
train = [MinuteInput(activity_count=20, hr_mean_bpm=52, hr_std_bpm=1,
                     hr_is_fresh=(i % 10 == 0)) for i in range(90)]
train += [MinuteInput(activity_count=3000, step_count=60, hr_mean_bpm=95,
                      hr_std_bpm=3, hr_is_fresh=(i % 3 == 0)) for i in range(15)]
for _ in range(12):  # ~2 h ride,10 分鐘一個 burst(現行節奏)
    train += [MinuteInput(activity_count=30, hr_mean_bpm=88, hr_std_bpm=2,
                          hr_is_fresh=True)]
    train += [MinuteInput(activity_count=30, hr_mean_bpm=88, hr_std_bpm=2,
                          hr_is_fresh=False) for _ in range(3)]
    train += [MinuteInput(activity_count=30, hr_mean_bpm=0) for _ in range(6)]

# 6) Backstop: somehow we slipped into Light during a long HR-absent gap
#    (e.g. PPG missed several bursts). SUSTAINED elevated HR from the dense
#    bursts that "asleep" turns on must rescue us to AWAKE — within
#    SF_WAKE_HR_CONSEC_ASLEEP bursts (3-min cadence) + exit hysteresis.
# 同 train:穩定參考要先看過這個人的靜息,才有東西可以比。
RESCUE_WARMUP_MIN = 90
slip_then_rescue = [
    MinuteInput(activity_count=20, hr_mean_bpm=52, hr_std_bpm=1,
                hr_is_fresh=(i % 10 == 0)) for i in range(RESCUE_WARMUP_MIN)
] + [
    MinuteInput(activity_count=30, hr_mean_bpm=0) for _ in range(50)  # HR 消失的空窗
] + dense_bursts(8, 88, act=30)  # 持續偏高 -> 把人救回 AWAKE

# 7) Phantom overnight steps: dead-still wrist (score ~0), low sleeping HR, but
#    the pedometer false-counts 1 step every minute. THIS is the reported bug —
#    the old `any step => wake` gate kept the whole night AWAKE. With step
#    corroboration (steps must be backed by real accel motion) it must sleep.
phantom_steps = [
    MinuteInput(activity_count=20, step_count=1, hr_mean_bpm=58, hr_std_bpm=1,
                hr_is_fresh=(i % 10 == 0))
    for i in range(60)
]

# 8) Lone HR spike during sleep — a brief REM/arousal bump or a garbage-high
#    burst under the artefact ceiling. One elevated BURST (fresh + its held
#    repeats) in an otherwise calm night must NOT wake the sleeper; only
#    SUSTAINED elevation (SF_WAKE_HR_CONSEC_ASLEEP distinct bursts) does.
#    Held-minute counting used to let this single burst triple-vote itself
#    past the old consec gate — the 2026-07-24 missed-night mechanism.
lone_spike = (dense_bursts(6, 58, std=1)
              + dense_bursts(1, 100)         # single polluted burst
              + dense_bursts(6, 58, std=1))

# 9) Artefact-high burst: a still wrist reading 126 bpm is physiologically
#    impossible — it's PPG motion artefact, must be dropped, and must neither
#    veto sleep nor pollute the learned baseline.
artefact = [
    MinuteInput(activity_count=20, hr_mean_bpm=58, hr_std_bpm=1,
                hr_is_fresh=(i % 10 == 0)) for i in range(60)
] + [
    MinuteInput(activity_count=20, hr_mean_bpm=126, hr_std_bpm=2,
                hr_is_fresh=(i % 3 == 0)) for i in range(12)
]

# 10) Polluted night — THE 2026-07-24 missed night, distilled: dead-still
#     wrist, dense 3-min bursts alternating true sleeping HR (58) with
#     poor-contact garbage (98, under the 120 artefact ceiling). Burst-level
#     asymmetric arming must keep the sleeper asleep: alternation never
#     reaches SF_WAKE_HR_CONSEC_ASLEEP, so the night stays contiguous. Under
#     minute-level counting this exact trace collapsed to fragments.
polluted_night = []
for _ in range(20):  # 2 h alternating clean/garbage bursts
    polluted_night += dense_bursts(1, 58, std=1)
    polluted_night += dense_bursts(1, 98)


# 11) 安靜的清醒白天 —— 2026-08-26 這一輪修的就是這個。手腕整天幾乎不動
#     (辦公桌前,活動量有一半的分鐘是 0),心率是白天水準 70。舊版把這種
#     一天整個判成睡著:2026-08-21 的 daily_sleep 報 1164 分 = 19.4 小時。
#     先給一段真實的夜(建立穩定參考),再接 8 小時的安靜白天。
quiet_awake_day = [
    MinuteInput(activity_count=20, hr_mean_bpm=52, hr_std_bpm=1,
                hr_is_fresh=(i % 10 == 0)) for i in range(120)
] + [
    MinuteInput(activity_count=(0 if i % 2 else 25), hr_mean_bpm=70, hr_std_bpm=3,
                hr_is_fresh=(i % 10 == 0)) for i in range(480)
]
QUIET_NIGHT_MIN = 120


# 12) 沒有睡眠的視窗不能拉高靜息參考 —— 2026-08-29 的主回歸。
#     先給一夜(參考落到 ~50),再給兩小時純清醒(75 bpm)。舊行為:直方圖 p5
#     直接跟隨,參考被拉到 70+,醒來門檻變 82,整段清醒判成睡著。
ref_night_then_awake = [
    MinuteInput(activity_count=20, hr_mean_bpm=50, hr_std_bpm=1,
                hr_is_fresh=(i % 10 == 0)) for i in range(180)
] + [
    MinuteInput(activity_count=40, hr_mean_bpm=75, hr_std_bpm=2,
                hr_is_fresh=(i % 10 == 0)) for i in range(120)
]
REF_NIGHT_MIN = 180


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
    s_night = trace("Polluted night (alternating clean/garbage bursts)", polluted_night)

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
    # 只看「上車之後」有沒有多記到睡眠 —— 暖身期本來就是真的在睡。
    s_t = State()
    warm_total = 0
    for i, inp in enumerate(train):
        update(s_t, (i + 1) * 60, inp)
        if i == TRAIN_WARMUP_MIN - 1:
            warm_total = s_t.total + s_t.pend['total']
    ride_sleep = (s_t.total + s_t.pend['total']) - warm_total
    # SF_WAKE_HR_CONSEC_AWAKE=2 的代價是有界的:veto 要等第二個 burst 才武裝,
    # 所以最多漏掉一個入睡窗(SF_ENTER_SLEEP_MIN)。用一個 burst 就武裝可以做到
    # 零,但那樣交替出現的爛 burst 會永久擋住入睡 —— 見 polluted night。
    check(f"train ride sleep bounded (<= {SF_ENTER_SLEEP_MIN + SF_EXIT_SLEEP_MIN} min)",
          ride_sleep <= SF_ENTER_SLEEP_MIN + SF_EXIT_SLEEP_MIN)

    # Backstop: a false sleep is corrected once HR returns. Re-run the rescue
    # trace step by step so we can measure the wake latency directly: HR turns
    # elevated at minute `hr_return` (the 21st input here); assert we are back
    # to AWAKE within SF_EXIT_SLEEP_MIN minutes of that.
    hr_return = RESCUE_WARMUP_MIN + 50  # 0-based index of the first elevated-HR minute
    s_r = State()
    wake_minute = None
    slept_at_all = False
    for i, inp in enumerate(slip_then_rescue):
        update(s_r, (i + 1) * 60, inp)
        if s_r.stage in (Stage.LIGHT, Stage.DEEP, Stage.REM):
            slept_at_all = True
        if i >= hr_return and s_r.stage == Stage.AWAKE and wake_minute is None:
            wake_minute = i
    check("HR rescue exits to AWAKE", s_rescue.stage == Stage.AWAKE)
    # It did sleep during the gap (proves the rescue, not a no-op). Check the
    # STAGE, not the committed total: a session shorter than SF_MIN_SESSION_MIN
    # is deliberately discarded from the daily counters, so a rescued false
    # sleep is *supposed* to leave total == 0.
    check("rescue trace did enter sleep first", slept_at_all)
    # ...and recovered once HR returned. Burst-level asleep arming makes the
    # rescue take up to SF_WAKE_HR_CONSEC_ASLEEP bursts (3-min cadence) plus
    # exit hysteresis — the deliberate price for not letting lone garbage
    # bursts shatter real sleep (the 2026-07-24 missed night).
    rescue_bound = 3 * SF_WAKE_HR_CONSEC_ASLEEP + SF_EXIT_SLEEP_MIN
    check("HR rescue woke after HR returned", wake_minute is not None)
    check(f"HR rescue is prompt (<= {rescue_bound} min after HR returns)",
          wake_minute is not None and (wake_minute - hr_return) <= rescue_bound)

    # The reported bug: phantom pedometer steps on a still wrist must NOT block
    # sleep onset. A bare step count is no longer trusted without accel motion.
    check("phantom steps still fall asleep (total > 0)", s_phantom.total > 0)

    # 沒有睡眠的視窗不能拉高參考。
    s_ref = State()
    ref_after_night = 0
    for i, inp in enumerate(ref_night_then_awake):
        update(s_ref, (i + 1) * 60, inp)
        if i == REF_NIGHT_MIN - 1:
            ref_after_night = s_ref.rhr_ref_bpm
    check(f"一夜之後參考落在靜息附近 (got {ref_after_night})",
          48 <= ref_after_night <= 54)
    drift = s_ref.rhr_ref_bpm - ref_after_night
    check(f"兩小時純清醒最多把參考推高 2 bpm (got +{drift})", drift <= 2)

    # 這一輪的主回歸:安靜的清醒白天不能被記成睡眠。
    s_q = State()
    night_total = 0
    for i, inp in enumerate(quiet_awake_day):
        update(s_q, (i + 1) * 60, inp)
        if i == QUIET_NIGHT_MIN - 1:
            night_total = s_q.total + s_q.pend['total']
    day_sleep = (s_q.total + s_q.pend['total']) - night_total
    check("quiet awake day: the night itself is still detected",
          night_total >= 60)
    check(f"quiet awake day: <= 30 of 480 daytime minutes counted as sleep "
          f"(got {day_sleep})", day_sleep <= 30)

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

    # THE missed-night regression (2026-07-24): a dead-still night whose
    # bursts alternate clean / garbage-high must stay contiguous sleep.
    # Under minute-level veto counting this exact trace collapsed to
    # fragments (watch scored 12 min against ~8 h in bed).
    check("polluted night stays asleep at the end",
          s_night.stage in (Stage.LIGHT, Stage.DEEP, Stage.REM))
    check("polluted night accumulates >= 100 of 120 min",
          s_night.total >= 100)

    print("\nAll assertions passed.")
