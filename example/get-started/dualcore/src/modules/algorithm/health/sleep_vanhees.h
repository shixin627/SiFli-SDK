/**
 ******************************************************************************
 * @file   sleep_vanhees.h
 * @author Skaiwalk software development team
 * @brief  Accelerometer-only sleep-PERIOD detector based on the van Hees
 *         arm-angle "sustained inactivity bout" (SIB) heuristic. No PPG / HR.
 *         Pure C, no RTOS / hardware dependencies, safe to build for PC sim
 *         and for the watch LCPU.
 *
 *  Algorithm (van Hees et al. 2015, PLOS ONE; the open HDCZA used in GGIR):
 *    - Per 5 s epoch, arm elevation angle = atan2(z, sqrt(x^2+y^2)) in deg.
 *      The angle is a *ratio* of axes, so raw accel LSB can be fed directly
 *      -- no need to convert to g or m/s^2.
 *    - When the absolute angle change between successive epochs stays below
 *      ANGLE_DELTA_THRESH for at least ONSET_MIN minutes, the wrist is in a
 *      Sustained Inactivity Bout (SIB) = asleep. Sustained movement exits it.
 *
 *  This detects a sleep PERIOD (asleep vs awake), not sleep STAGES. There is
 *  no Light/Deep/REM split (that needs HR/HRV). Wear state gates the output.
 *
 *  Reference: van Hees VT et al. 2015 "A Novel, Open Access Method to Assess
 *  Sleep Duration Using a Wrist-Worn Accelerometer", PLOS ONE 10(11).
 ******************************************************************************
 */
#ifndef SLEEP_VANHEES_H
#define SLEEP_VANHEES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* Public stage. Maps to the T_SLEEP_STATUS wire enum at the IPC boundary
       (sleep_service.c). Only three states -- no Deep/REM. */
    typedef enum
    {
        SLEEP_VH_STAGE_AWAKE = 0,
        SLEEP_VH_STAGE_ASLEEP,   /* inside a sustained inactivity bout */
        SLEEP_VH_STAGE_NOT_WORN,
        SLEEP_VH_STAGE_COUNT,
    } sleep_vh_stage_t;

    /* Output snapshot. Valid until the next sleep_vanhees_update_1hz(). */
    typedef struct
    {
        sleep_vh_stage_t stage;
        bool stage_changed;             /* true on the epoch the stage flipped */
        uint16_t consecutive_minutes_in_stage;

        /* Daily aggregates (since last midnight reset). The phone fills the
           20-byte KEY_RETURN_SLEEP_DATA payload from these. */
        uint16_t total_sleep_min;       /* minutes spent ASLEEP today          */
        uint16_t awake_after_onset_min; /* WASO: awake minutes after 1st onset */

        /* Sleep-period bounds. 0 if no sleep session detected yet today.
           sleep_onset_utc is back-dated to the true start of the bout
           (trip time minus the ONSET_MIN qualifying window). */
        uint32_t sleep_onset_utc;
        uint32_t last_wake_utc;

        /* Diagnostics -- useful for logging / tuning. */
        int16_t last_angle_deg;         /* most recent 5 s epoch arm angle     */
        int16_t last_angle_delta_deg;   /* |angle - prev epoch angle|          */
    } sleep_vanhees_output_t;

    /* Initialize internal state. Call once at boot / on enable. */
    void sleep_vanhees_init(void);

    /* Reset daily aggregates (call at local midnight). Keeps the running
       angle history + current stage so a session spanning midnight stays
       contiguous; only the daily counters reset. */
    void sleep_vanhees_midnight_reset(void);

    /* Hard reset -- clear everything incl. history and current stage. */
    void sleep_vanhees_reset(void);

    /* Feed one raw accelerometer sample at ~1 Hz. ax/ay/az are raw signed
       LSB straight from the sensor (any full-scale; the angle is scale-free).
       utc_sec is the current UTC second (used only for period reporting).
       is_worn comes from wear_detect; false forces NOT_WORN.

       The detector buffers EPOCH_SEC samples into one 5 s epoch internally
       and only re-evaluates the stage on epoch boundaries. Returns a pointer
       to internal state valid until the next call. */
    const sleep_vanhees_output_t *sleep_vanhees_update_1hz(
        uint32_t utc_sec, int16_t ax, int16_t ay, int16_t az, bool is_worn);

    /* Read-only access to the most recent output without advancing. */
    const sleep_vanhees_output_t *sleep_vanhees_current(void);

#ifdef __cplusplus
}
#endif

#endif /* SLEEP_VANHEES_H */
