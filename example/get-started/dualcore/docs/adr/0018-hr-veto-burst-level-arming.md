# ADR-0018 — Sleep wake-veto arms per PPG burst, asymmetric awake/asleep; window feeds converged median

- **Status**: accepted (2026-07-24)
- **Agent**: Claude (autonomous loop), replay-driven; founder-verified E2E pending one night
- **Trigger**: Second missed night in a row DESPITE ADR-0017's rest-candidate
  dense sampling working as designed (whole night dense, zero power-save gaps
  on the phone curve) — watch scored 12 min against ~8 h in bed (2026-07-24).

## Diagnosis (replay, not speculation)

Phone keeps every burst's HR (`skailink_health.db` / `hr_curve`), so the night
was replayed offline through the reference mirror (`sleep_fusion_test.py`):

1. **Accel exonerated**: the rest-candidate gate (enter: score<400 for 10 min,
   exit: 3 min ≥400) stayed engaged all night — the activity score was below
   the sleep threshold essentially the whole night.
2. **Veto convicted by elimination + mechanism**:
   - Replaying the real HR with a dead-still accel profile under the OLD
     semantics yields 490 min — HR pollution alone fragments (WASO 268) but
     cannot zero a night. The observed 12 min requires the veto to have seen a
     dirtier stream than the phone stored, and it did, via two defects:
   - **Held-repeat self-arming**: `sleep_service` holds a burst's HR window up
     to 4 min for staging, so ONE polluted burst fed the veto 2-3 "consecutive"
     elevated minutes — defeating `SF_WAKE_HR_CONSEC_MIN=2` by itself, arming
     the 12-min hold, and triple-stepping the resting-HR learner.
   - **Warm-up contamination**: the window published the burst MEAN including
     the pre-convergence warm-up prefix (dynamic warm-up accepts reads as soon
     as the algo seq moves), running tens of bpm above the converged median the
     phone stores — on poor-contact bursts nearly every window read "elevated".

## Decision

Three coupled changes (sleep_fusion.c/.h, sleep_service.c, hr_service.c, and
the 1:1 python mirror):

1. **F1 — feed the converged value**: `bg_hr_win_mean` now publishes the
   burst's final rolling median (`bg_hr_burst_best`, identical to the phone
   curve point), not the warm-up-contaminated mean. std stays sum-based
   (staging only).
2. **F2 — burst-level veto counting**: new input `hr_is_fresh` marks the first
   minute of each new burst (age-decrease detection in sleep_service; live
   foreground minutes are always fresh). The veto counters and the resting-HR
   learner advance ONLY on fresh minutes. Held repeats are veto-neutral: no
   count, no clear, **no hold decay** (a held value is not missing data;
   decaying on it opened a ~3 min/cycle sleep leak since hold 12 < sparse
   cycle 15). Truly HR-absent minutes still decay the hold.
3. **F3 — asymmetric arm threshold**: `SF_WAKE_HR_CONSEC_AWAKE=1` (a train /
   desk must veto from the very first elevated burst — zero-leak with the
   no-decay-on-held rule), `SF_WAKE_HR_CONSEC_ASLEEP=3` (kicking an
   established sleeper out requires ~9 min of sustained elevation at dense
   cadence; lone/double garbage bursts no longer fragment the night).

## Evidence

Replay matrix on three real nights (phone DB, still-wrist profile, fresh
flags from actual burst timestamps) + synthetic guards:

| metric                    | old (held,2/2) | new (fresh,1/3) |
|---------------------------|---------------|-----------------|
| missed dense 7/23-24      | 490 min       | **582 min**     |
| good night 7/20-21        | 377 min       | **510 min**     |
| missed sparse 7/22-23     | 444 min       | **602 min**     |
| train false sleep         | 0             | **0**           |
| lone-spike stays asleep   | yes           | yes             |
| slip-rescue latency       | 2 min         | 7 min (≤11 bound) |

(2/4 asleep-arming scored higher on nights but leaked 44 min of train false
sleep — rejected: the sedentary-wake guarantee is the veto's reason to exist.)

Full mirror suite: 14/14 assertions green, including the new distilled
`polluted_night` regression (alternating clean/garbage bursts must stay ≥100
of 120 min asleep — the exact 2026-07-24 mechanism).

## Consequences

- Slip-into-sleep rescue slows from ~2 min to ≤3 bursts + hysteresis (~7-11
  min) — deliberate trade for artefact immunity.
- The resting-HR learner sees each burst once (was 2-3×), so it adapts ~3×
  slower in wall-clock terms overnight; direction unchanged.
- `[SLPDIAG]` gains `fr=` (freshness) for one-look attribution of any future
  miss.
- Root PPG contact quality on bad nights is NOT fixed here — the veto is now
  robust to it. If a future night still under-scores with `veto=0` in
  SLPDIAG, suspect staging/accel, not this path.
