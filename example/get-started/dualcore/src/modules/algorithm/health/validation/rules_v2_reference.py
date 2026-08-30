# -*- coding: utf-8 -*-
"""
rules_v2 -- hand-written sleep-stage rules, no machine learning.
Reference implementation of exactly what the firmware would do, kept here so the
C port can be diffed against something runnable.

Scored with GroupKFold(5) by SUBJECT, out-of-fold, on features.npz (13404 min,
31 subjects):

    shipped firmware        acc4 0.436  k4 0.109 | acc3 0.590  k3 0.225
    always-LIGHT            acc4 0.550  k4 0.000 | acc3 0.681  k3 0.000
    oracle-wake + LIGHT     acc4 0.659  k4 0.315 | acc3 0.790  k3 0.449
    THIS (rules_v2)         acc4 0.536  k4 0.282 | acc3 0.648  k3 0.322
        REM  sens 0.526 prec 0.405
        DEEP sens 0.427 prec 0.477
    (HistGradientBoosting on the same 25 features, same protocol: k4 0.296 --
     i.e. these rules reach ~95% of the ML ceiling on kappa.)

CAUSAL: every quantity uses only the current and past minutes. Verified
empirically -- re-running on the truncated prefix X[0..i] reproduces the label
at minute i for 200/200 sampled (subject, minute) pairs. Safe to emit live.

Constants are the 5-fold-stable quantiles of the training subjects; they were
never dialled by hand against held-out data.
"""
import numpy as np

WAKE = 0; LIGHT = 1; DEEP = 2; REM = 3

# ---- constants (quantiles of the training folds; fold spread in comments) ----
WAKE_A5   = 1.45    # 1.39 .. 1.57
WAKE_A15  = 0.90    # 0.83 .. 0.99
WAKE_HOVR = 9.0     # 9 .. 10  bpm
D_TSL     = 60.0    # 58.8 .. 61.0  min
D_BURST   = 1.0     # 1.0 .. 1.0   bpm
D_HSD15   = 0.50    # 0.49 .. 0.50 bpm
D_A15     = 0.06    # 0.046 .. 0.073
R_TSL     = 334.0   # 330 .. 339   min
R_BURST   = 2.0     # 2.0 .. 2.0   bpm
R_HD30    = 0.0     # 0 .. 0       bpm
R_HOVR    = 4.0     # 4 .. 4       bpm
RHR_WARMUP   = 20   # valid HR samples before the running floor is trusted
RHR_FALLBACK = 60.0
ONSET_RUN    = 10   # consecutive non-wake minutes that latch sleep onset


class Stager:
    """One instance per night. Feed it one minute at a time; it returns a stage."""

    def __init__(self):
        self.t = 0
        self.la = []        # log1p(activity), trailing
        self.hr = []        # held HR, trailing (0 = never seen)
        self.hr_seen = []   # every valid HR so far tonight -> running floor
        self.quiet_run = 0
        self.onset = -1

    @staticmethod
    def _mean_tail(a, k):
        w = a[-k:]
        return float(np.mean(w)) if w else 0.0

    def step(self, activity, hr, hr_sd):
        """activity = per-minute sum of (|dx|+|dy|+|dz|)>>10 at 1 Hz
           hr       = latest duty-cycled HR, HELD between bursts (0 if none yet)
           hr_sd    = within-burst SD of that HR reading, held the same way"""
        hr_sd = float(hr_sd)
        self.la.append(float(np.log1p(activity)))
        self.hr.append(float(hr))
        if hr > 0:
            self.hr_seen.append(float(hr))

        # --- trailing activity ---
        a5 = self._mean_tail(self.la, 5)
        a15 = self._mean_tail(self.la, 15)

        # --- trailing HR variability over the last 15 minutes ---
        w = [v for v in self.hr[-15:] if v > 0]
        hsd15 = float(np.std(w)) if len(w) >= 2 else 0.0

        # --- resting floor: 5th percentile of everything seen SO FAR tonight ---
        rhr = (float(np.percentile(self.hr_seen, 5))
               if len(self.hr_seen) >= RHR_WARMUP else RHR_FALLBACK)
        hovr = (self.hr[-1] - rhr) if self.hr[-1] > 0 else 0.0

        # --- 30-minute HR trend ---
        hd30 = 0.0
        if self.t >= 30 and self.hr[-1] > 0:
            hd30 = self.hr[-1] - self.hr[-31]

        # --- 1. wake gate --------------------------------------------------
        awake = (a5 > WAKE_A5) or (a15 > WAKE_A15 and hovr > WAKE_HOVR)

        # --- 2. sleep-onset latch (never un-latches) ------------------------
        self.quiet_run = 0 if awake else self.quiet_run + 1
        if self.onset < 0 and self.quiet_run >= ONSET_RUN:
            self.onset = self.t - (ONSET_RUN - 1)
        tsl = float(self.t - self.onset) if self.onset >= 0 else 0.0

        # --- 3. four votes each for DEEP and REM ----------------------------
        dv = (int(tsl   <= D_TSL)    + int(hr_sd <= D_BURST)
              + int(hsd15 <= D_HSD15) + int(a15   <= D_A15))
        rv = (int(tsl   >= R_TSL)    + int(hr_sd >= R_BURST)
              + int(hd30  >= R_HD30)  + int(hovr  >= R_HOVR))

        # --- 4. decide ------------------------------------------------------
        if awake:
            stage = WAKE
        elif dv >= 3 and dv >= rv:
            stage = DEEP
        elif rv >= 3 and rv > dv:
            stage = REM
        else:
            stage = LIGHT

        self.t += 1
        return stage
