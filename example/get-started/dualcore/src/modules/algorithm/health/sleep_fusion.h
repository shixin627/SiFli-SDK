/**
 ******************************************************************************
 * @file   sleep_fusion.h
 * @author Skaiwalk software development team
 * @brief  Sleep stage classifier fusing wrist accel (IMU) and PPG-derived
 *         heart rate. Pure C — no RTOS / hardware dependencies, safe to
 *         build for PC sim and for the watch LCPU.
 *
 *  Algorithm: Cole-Kripke style weighted activity score for wake/sleep
 *  decision (with an HR wake-veto so a still-but-awake wrist on a train or
 *  at a desk is not scored as sleep — HR more than a fixed offset above a
 *  self-learned resting baseline votes wake), followed by HR-based staging
 *  into Wake / Light / Deep / REM. Validated against the Walch 2019 PSG set.
 *  References:
 *    Cole RJ et al. 1992  — actigraphy sleep/wake scoring
 *    Walch O et al. 2019  — accel + PPG sleep staging (Stanford)
 *    Sundararajan K 2023  — lightweight wearable sleep staging
 ******************************************************************************
 */
#ifndef SLEEP_FUSION_H
#define SLEEP_FUSION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* Public stage enum. The wire format (T_SLEEP_STATUS in health_algo.h)
       uses different numeric values; map at the IPC boundary. */
    typedef enum
    {
        SLEEP_FUSION_STAGE_AWAKE = 0,
        SLEEP_FUSION_STAGE_LIGHT,
        SLEEP_FUSION_STAGE_DEEP,
        SLEEP_FUSION_STAGE_REM,
        SLEEP_FUSION_STAGE_NOT_WORN,
        SLEEP_FUSION_STAGE_COUNT,
    } sleep_fusion_stage_t;

    /* Per-minute feature inputs. The caller (sleep_service on LCPU)
       aggregates raw sensor samples into these summary statistics. */
    typedef struct
    {
        /* Sum of |accel_magnitude - 1g| scaled by 100 over the minute.
           Empirical ranges on a wrist-worn IMU:
             rest / still ........ <    200
             gentle motion ....... 200 - 1500
             walking ............. > 3000                         */
        uint32_t activity_count;

        /* Steps in the minute (from BMI270 HW step counter delta).
           Any nonzero value strongly suggests wake.               */
        uint16_t step_count;

        /* Mean HR in BPM over the minute. 0 means HR was unavailable
           (sensor off / poor signal). The classifier degrades to
           accel-only when HR is missing.                          */
        uint8_t hr_mean_bpm;

        /* HR std-dev in BPM. Proxy for HRV — elevated during REM. */
        uint8_t hr_std_bpm;

        /* Beat-to-beat RMSSD in ms (true HRV) over the minute, or 0 when no
           RR intervals were available this minute. Used to separate Deep
           (high RMSSD) from REM (lower RMSSD). The classifier IGNORES it when
           0, so it is safe to leave unset until the PPG RR plumbing lands —
           staging then behaves exactly as the HR-std-only version. */
        uint8_t hr_rmssd_ms;

        /* True when hr_mean_bpm comes from a NEW PPG burst (or live
           foreground HR) this minute; false when it is a HELD repeat of the
           last burst's window (kept so staging keeps HR across the 3-min
           burst gaps). The wake-veto counts DISTINCT bursts, not minutes:
           only fresh minutes advance/clear its counters (see
           SF_WAKE_HR_CONSEC_*) — a held repeat is the same observation
           again, not new evidence.                                */
        bool hr_is_fresh;

        /* From wear_detect.c. When false the classifier emits
           NOT_WORN regardless of accel/HR.                        */
        bool is_worn;
    } sleep_fusion_minute_input_t;

    /* Output snapshot. Returned by sleep_fusion_update() each minute.
       Daily aggregates reset at sleep_fusion_midnight_reset(). */
    typedef struct
    {
        sleep_fusion_stage_t stage;
        bool stage_changed; /* true on the minute the stage changed */
        uint16_t consecutive_minutes_in_stage;

        /* Daily aggregates (since last midnight reset). */
        uint16_t total_sleep_min;     /* Light + Deep + REM */
        uint16_t deep_min;
        uint16_t rem_min;
        uint16_t light_min;
        uint16_t awake_after_onset_min; /* WASO */

        /* Session bounds. 0 if no sleep session detected yet today. */
        uint32_t sleep_onset_utc;
        uint32_t last_wake_utc;

        /* Diagnostics — useful for logging and tuning. */
        uint32_t last_cole_kripke_score;
        uint8_t  last_hr_baseline_bpm;
        uint8_t  learned_rhr_bpm;       /* online resting-HR estimate, veto anchor */
        bool     hr_wake_veto_active;   /* HR held this minute out of sleep */
    } sleep_fusion_output_t;

    /* Initialize internal state. resting_hr_bpm is used as the HR baseline
       for stage classification (typical 60-70). 0 disables HR-based
       staging — the classifier will only emit Awake/Light when asleep. */
    void sleep_fusion_init(uint8_t resting_hr_bpm);

    /* Update the resting HR baseline (e.g. from user prefs or learned
       overnight minimum). Safe to call at any time. */
    void sleep_fusion_set_resting_hr(uint8_t resting_hr_bpm);

    /* Reset daily aggregates. Call at local midnight. Does NOT clear
       the recent history window — the algorithm keeps running across
       the midnight boundary so a sleep session spanning midnight stays
       contiguous on the stage side, only the daily counters reset. */
    void sleep_fusion_midnight_reset(void);

    /* Hard reset — clear everything including history and current stage.
       Use on boot / sensor failure / user opt-out. */
    void sleep_fusion_reset(void);

    /* Advance the algorithm by one minute. Pass utc_sec = current UTC
       second (used only for session bounds reporting; the algorithm
       itself is wall-clock-agnostic). Returns a pointer to internal
       state that is valid until the next call. */
    const sleep_fusion_output_t *sleep_fusion_update(
        uint32_t utc_sec,
        const sleep_fusion_minute_input_t *input);

    /* Read-only access to the most recent output without advancing. */
    const sleep_fusion_output_t *sleep_fusion_current(void);

#ifdef __cplusplus
}
#endif

#endif /* SLEEP_FUSION_H */
