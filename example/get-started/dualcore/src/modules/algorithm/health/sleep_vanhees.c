/**
 ******************************************************************************
 * @file   sleep_vanhees.c
 * @author Skaiwalk software development team
 * @brief  Accelerometer-only sleep-period detector (van Hees SIB) -- impl.
 *
 *  Per 5 s epoch we average the raw accel, compute the arm elevation angle
 *  atan2(z, sqrt(x^2+y^2)) in degrees, and look at how much it moved vs the
 *  previous epoch. A run of epochs whose angle barely changes (< threshold)
 *  for >= ONSET_MIN minutes is a Sustained Inactivity Bout => asleep.
 *  Sustained movement (a few epochs above threshold) ends the bout => awake.
 *
 *  Float math (atan2f/sqrtf) runs once per 5 s, so the CPU cost is trivial;
 *  the M33 LCPU has an FPU. If microlib footprint ever matters this can be
 *  refactored to a fixed-point atan2 -- the rest of the state machine is
 *  already integer.
 ******************************************************************************
 */
#include "sleep_vanhees.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------
 *  Tunables -- empirical, per van Hees 2015 / GGIR HDCZA defaults.
 * ------------------------------------------------------------------ */

#define SVH_EPOCH_SEC            5     /* one angle sample per 5 s          */
#define SVH_SAMPLES_PER_EPOCH    SVH_EPOCH_SEC  /* fed at 1 Hz              */

/* Angle change (deg) between successive epochs below which the wrist is
   considered "not moving". GGIR HDCZA default is 5 degrees. */
#define SVH_ANGLE_DELTA_THRESH   5.0f

/* Minutes of continuous low-movement needed to declare sleep onset. */
#define SVH_ONSET_MIN            5
#define SVH_ONSET_EPOCHS         (SVH_ONSET_MIN * 60 / SVH_EPOCH_SEC)  /* 60 */

/* Consecutive moved epochs needed to exit sleep. Small hysteresis so a
   single roll-over (one epoch above threshold) does not wake the bout. */
#define SVH_WAKE_MOVE_EPOCHS     3     /* ~15 s of sustained movement       */

#define SVH_DEG_PER_RAD          57.2957795f
#define SVH_MIN_CLAMP(x)         ((x) > 0xFFFFu ? 0xFFFFu : (x))

/* ------------------------------------------------------------------
 *  Internal state
 * ------------------------------------------------------------------ */

typedef struct
{
    /* 5 s epoch accumulation (raw LSB summed over SVH_SAMPLES_PER_EPOCH). */
    int32_t  sum_x, sum_y, sum_z;
    uint8_t  sample_count;

    /* Previous epoch angle (deg) for the delta. */
    float    prev_angle_deg;
    bool     prev_angle_valid;

    /* Bout state machine counters (in epochs). */
    uint16_t still_epochs; /* consecutive low-movement epochs               */
    uint8_t  move_epochs;  /* consecutive above-threshold epochs            */

    /* Accumulators for daily aggregates, in seconds (derived to minutes). */
    uint32_t asleep_sec;
    uint32_t waso_sec;
    uint32_t stage_sec;    /* seconds in the current stage                  */

    sleep_vanhees_output_t out;
} sleep_vanhees_state_t;

static sleep_vanhees_state_t s_vh;

/* ------------------------------------------------------------------
 *  Helpers
 * ------------------------------------------------------------------ */

static void prv_enter_stage(uint32_t utc_sec, sleep_vh_stage_t next)
{
    sleep_vh_stage_t prev = s_vh.out.stage;
    if (prev == next)
    {
        s_vh.out.stage_changed = false;
        s_vh.stage_sec += SVH_EPOCH_SEC;
        s_vh.out.consecutive_minutes_in_stage = (uint16_t)(s_vh.stage_sec / 60u);
        return;
    }

    bool prev_sleep = (prev == SLEEP_VH_STAGE_ASLEEP);
    bool next_sleep = (next == SLEEP_VH_STAGE_ASLEEP);

    if (!prev_sleep && next_sleep)
    {
        /* Sleep onset. Back-date to the true start of the bout (the
           qualifying ONSET window happened before this epoch). Record only
           the FIRST onset of the day so the reported period is stable. */
        if (s_vh.out.sleep_onset_utc == 0)
        {
            uint32_t back = (uint32_t)SVH_ONSET_MIN * 60u;
            s_vh.out.sleep_onset_utc = (utc_sec > back) ? (utc_sec - back) : utc_sec;
        }
    }
    if (prev_sleep && !next_sleep)
    {
        s_vh.out.last_wake_utc = utc_sec;
    }

    s_vh.out.stage = next;
    s_vh.out.stage_changed = true;
    s_vh.stage_sec = SVH_EPOCH_SEC;
    s_vh.out.consecutive_minutes_in_stage = 0;
}

/* Evaluate one closed 5 s epoch given its mean accel (raw LSB). */
static void prv_eval_epoch(uint32_t utc_sec, float mx, float my, float mz, bool is_worn)
{
    if (!is_worn)
    {
        s_vh.still_epochs = 0;
        s_vh.move_epochs = 0;
        s_vh.prev_angle_valid = false;
        s_vh.out.last_angle_deg = 0;
        s_vh.out.last_angle_delta_deg = 0;
        prv_enter_stage(utc_sec, SLEEP_VH_STAGE_NOT_WORN);
        return;
    }

    float angle = atan2f(mz, sqrtf(mx * mx + my * my)) * SVH_DEG_PER_RAD;
    s_vh.out.last_angle_deg = (int16_t)lrintf(angle);

    float delta = 0.0f;
    if (s_vh.prev_angle_valid)
    {
        delta = fabsf(angle - s_vh.prev_angle_deg);
    }
    s_vh.prev_angle_deg = angle;
    s_vh.prev_angle_valid = true;
    s_vh.out.last_angle_delta_deg = (int16_t)lrintf(delta);

    bool moved = (delta >= SVH_ANGLE_DELTA_THRESH);
    if (moved)
    {
        s_vh.move_epochs = (s_vh.move_epochs < 0xFF) ? (uint8_t)(s_vh.move_epochs + 1) : 0xFF;
        s_vh.still_epochs = 0;
    }
    else
    {
        s_vh.still_epochs = (s_vh.still_epochs < 0xFFFF) ? (uint16_t)(s_vh.still_epochs + 1) : 0xFFFF;
        s_vh.move_epochs = 0;
    }

    sleep_vh_stage_t prev = s_vh.out.stage;
    sleep_vh_stage_t next = prev;

    if (prev == SLEEP_VH_STAGE_ASLEEP)
    {
        /* Stay asleep until sustained movement. */
        if (s_vh.move_epochs >= SVH_WAKE_MOVE_EPOCHS)
        {
            next = SLEEP_VH_STAGE_AWAKE;
        }
    }
    else
    {
        /* Awake / not-worn / fresh: need ONSET_EPOCHS of stillness in a row. */
        if (s_vh.still_epochs >= SVH_ONSET_EPOCHS)
        {
            next = SLEEP_VH_STAGE_ASLEEP;
        }
        else
        {
            next = SLEEP_VH_STAGE_AWAKE;
        }
    }

    prv_enter_stage(utc_sec, next);

    /* Daily counters (count this epoch's 5 s into the right bucket). */
    if (next == SLEEP_VH_STAGE_ASLEEP)
    {
        s_vh.asleep_sec += SVH_EPOCH_SEC;
    }
    else if (next == SLEEP_VH_STAGE_AWAKE && s_vh.out.sleep_onset_utc != 0)
    {
        s_vh.waso_sec += SVH_EPOCH_SEC;
    }
    s_vh.out.total_sleep_min = (uint16_t)SVH_MIN_CLAMP(s_vh.asleep_sec / 60u);
    s_vh.out.awake_after_onset_min = (uint16_t)SVH_MIN_CLAMP(s_vh.waso_sec / 60u);
}

/* ------------------------------------------------------------------
 *  Public API
 * ------------------------------------------------------------------ */

void sleep_vanhees_init(void)
{
    memset(&s_vh, 0, sizeof(s_vh));
    s_vh.out.stage = SLEEP_VH_STAGE_AWAKE;
}

void sleep_vanhees_midnight_reset(void)
{
    s_vh.asleep_sec = 0;
    s_vh.waso_sec = 0;
    s_vh.out.total_sleep_min = 0;
    s_vh.out.awake_after_onset_min = 0;
    s_vh.out.sleep_onset_utc = 0;
    s_vh.out.last_wake_utc = 0;
}

void sleep_vanhees_reset(void)
{
    memset(&s_vh, 0, sizeof(s_vh));
    s_vh.out.stage = SLEEP_VH_STAGE_AWAKE;
}

const sleep_vanhees_output_t *sleep_vanhees_update_1hz(
    uint32_t utc_sec, int16_t ax, int16_t ay, int16_t az, bool is_worn)
{
    /* Not worn short-circuits: flush epoch accumulation, emit NOT_WORN now
       (don't wait for the 5 s boundary so we react promptly to take-off). */
    if (!is_worn)
    {
        s_vh.sum_x = s_vh.sum_y = s_vh.sum_z = 0;
        s_vh.sample_count = 0;
        prv_eval_epoch(utc_sec, 0.0f, 0.0f, 0.0f, false);
        return &s_vh.out;
    }

    s_vh.sum_x += ax;
    s_vh.sum_y += ay;
    s_vh.sum_z += az;
    s_vh.sample_count++;

    if (s_vh.sample_count >= SVH_SAMPLES_PER_EPOCH)
    {
        float n = (float)s_vh.sample_count;
        float mx = (float)s_vh.sum_x / n;
        float my = (float)s_vh.sum_y / n;
        float mz = (float)s_vh.sum_z / n;
        s_vh.sum_x = s_vh.sum_y = s_vh.sum_z = 0;
        s_vh.sample_count = 0;
        prv_eval_epoch(utc_sec, mx, my, mz, true);
    }
    else
    {
        s_vh.out.stage_changed = false;
    }
    return &s_vh.out;
}

const sleep_vanhees_output_t *sleep_vanhees_current(void)
{
    return &s_vh.out;
}
