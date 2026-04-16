/**
 ******************************************************************************
 * @file   wear_detect.c
 * @author Skaiwalk software development team
 * @brief  Wear detection algorithm combining IMU and PPG sensor signals.
 *
 *         Detection principle (event-driven):
 *         1. Monitor PPG for large changes (delta > threshold). When detected,
 *            start a 3-second detection window.
 *         2. During the window, check PPG level:
 *            - PPG < 50000: nothing touching the sensor → NOT wearing.
 *            - PPG >= 50000: something is touching. Check IMU variance:
 *              - IMU has motion → WEARING (human wrist moves).
 *              - IMU no motion  → NOT wearing (static object).
 ******************************************************************************
 */
#include <rtthread.h>
#include <math.h>
#include <string.h>
#include "wear_detect.h"
#include "bloc_peripheral.h"
#include "watch_sys_service.h"

#define DBG_TAG "wear_detect"
#define DBG_LVL DBG_INFO
#include "rtdbg.h"

/* -------------------- Configuration -------------------- */

/* PPG threshold: above this value means something is touching the sensor */
#define PPG_CONTACT_THD         70000

/* PPG delta threshold to trigger a detection window.
 * A sudden change larger than this indicates put-on or take-off event. */
#define PPG_DELTA_THD           800

/* Detection window duration in milliseconds */
#define DETECT_WINDOW_MS        3000

/* IMU ring buffer size: at 25 Hz, 75 samples = 3 seconds */
#define IMU_WINDOW_SIZE         75

/* IMU variance threshold (m/s^2)^2.
 * On-wrist micro-motion typically produces variance > 0.03.
 * Static surface is < 0.01. */
#define IMU_VARIANCE_THD        0.03f

/* IMU variance threshold to trigger re-detection when OFF wrist.
 * Should be higher than IMU_VARIANCE_THD to avoid noise triggers. */
#define IMU_RETRIGGER_THD       0.05f

/* How many IMU samples between each re-trigger check (at 25Hz, 25 = 1s) */
#define IMU_RETRIGGER_PERIOD    25

/* Moving average length for PPG to smooth noise before delta check */
#define PPG_AVG_LEN             5

/* -------------------- Internal state -------------------- */

typedef struct
{
    /* IMU ring buffer */
    float acce_mag[IMU_WINDOW_SIZE];
    uint16_t imu_idx;
    uint16_t imu_count;

    /* PPG moving average */
    uint32_t ppg_history[PPG_AVG_LEN];
    uint16_t ppg_idx;
    uint16_t ppg_count;
    uint32_t ppg_avg;           /* current moving average */
    uint32_t ppg_prev_avg;      /* previous moving average (for delta) */

    /* Detection window */
    bool detecting;             /* true while inside a 3s window */
    uint32_t detect_start_ms;   /* timestamp when window started */

    /* Accumulated PPG samples during detection window */
    uint32_t detect_ppg_sum;
    uint16_t detect_ppg_count;

    /* IMU re-trigger counter (for OFF state) */
    uint16_t imu_retrigger_counter;

    /* Current output */
    wear_status_t status;
    bool initialized;
} wear_detect_ctx_t;

static wear_detect_ctx_t ctx;

/* -------------------- Helpers -------------------- */

static float compute_variance(const float *buf, uint16_t len)
{
    float sum = 0.0f;
    float sum_sq = 0.0f;

    for (uint16_t i = 0; i < len; i++)
    {
        sum += buf[i];
        sum_sq += buf[i] * buf[i];
    }

    float mean = sum / (float)len;
    return (sum_sq / (float)len) - (mean * mean);
}

static void notify_wear_status(bool wearing)
{
    if (watch_sys_sync.soft_adt_status_callback)
    {
        watch_sys_sync.soft_adt_status_callback(wearing);
    }
}

static void set_status(wear_status_t new_status)
{
    if (ctx.status == new_status)
        return;

    ctx.status = new_status;

    if (new_status == WEAR_STATUS_WEARING)
    {
        LOG_I("Wear detected: ON WRIST");
        notify_wear_status(true);
    }
    else
    {
        LOG_I("Wear detected: OFF WRIST");
        notify_wear_status(false);
    }
}

/* -------------------- Detection window evaluation -------------------- */

static void wear_detect_finish_window(void)
{
    ctx.detecting = false;

    /* Average PPG during the 3s window */
    uint32_t avg_ppg = 0;
    if (ctx.detect_ppg_count > 0)
    {
        avg_ppg = ctx.detect_ppg_sum / ctx.detect_ppg_count;
    }

    // LOG_D("Window done: avg_ppg=%u, imu_count=%u", avg_ppg, ctx.imu_count);

    // if (avg_ppg < PPG_CONTACT_THD)
    // {
    //     /* PPG low → nothing touching sensor → not wearing */
    //     LOG_D("PPG < %u → no contact", PPG_CONTACT_THD);
    //     set_status(WEAR_STATUS_NOT_WEARING);
    // }
    // else
    {
        /* PPG high → something touching. Check IMU for motion. */
        uint16_t len = (ctx.imu_count < IMU_WINDOW_SIZE)
                           ? ctx.imu_count : IMU_WINDOW_SIZE;
        if (len > 0)
        {
            float var = compute_variance(ctx.acce_mag, len);
            // LOG_D("PPG >= %u, IMU variance=%.6f", PPG_CONTACT_THD, var);

            if (var >= IMU_VARIANCE_THD)
            {
                set_status(WEAR_STATUS_WEARING);
            }
            else
            {
                set_status(WEAR_STATUS_NOT_WEARING);
            }
        }
        else
        {
            /* No IMU data yet, can't decide */
            LOG_D("No IMU data during window, keeping current status");
        }
    }
}

/* -------------------- Public API -------------------- */

void wear_detect_init(void)
{
    memset(&ctx, 0, sizeof(ctx));
    ctx.status = WEAR_STATUS_WEARING; /* default: assume wearing at boot */
    ctx.initialized = true;
    LOG_I("Wear detection initialized");
}

void wear_detect_feed_imu(Vector3 *acce, float sample_rate)
{
    if (!ctx.initialized || !acce)
        return;

    /* Store magnitude of acceleration vector */
    float mag = sqrtf(acce->x * acce->x +
                      acce->y * acce->y +
                      acce->z * acce->z);
    ctx.acce_mag[ctx.imu_idx] = mag;
    ctx.imu_idx = (ctx.imu_idx + 1) % IMU_WINDOW_SIZE;

    if (ctx.imu_count < IMU_WINDOW_SIZE)
        ctx.imu_count++;

    /* Check if detection window has expired */
    if (ctx.detecting)
    {
        uint32_t now = rt_tick_get_millisecond();
        if (now - ctx.detect_start_ms >= DETECT_WINDOW_MS)
        {
            wear_detect_finish_window();
        }
    }

    /* When OFF wrist and not detecting, periodically check IMU for large motion.
     * If motion detected → start a detection window to re-evaluate. */
    if (ctx.status == WEAR_STATUS_NOT_WEARING && !ctx.detecting)
    {
        ctx.imu_retrigger_counter++;
        if (ctx.imu_retrigger_counter >= IMU_RETRIGGER_PERIOD)
        {
            ctx.imu_retrigger_counter = 0;
            uint16_t len = (ctx.imu_count < IMU_WINDOW_SIZE)
                               ? ctx.imu_count : IMU_WINDOW_SIZE;
            if (len >= IMU_RETRIGGER_PERIOD)
            {
                float var = compute_variance(ctx.acce_mag, len);
                if (var >= IMU_RETRIGGER_THD)
                {
                    LOG_I("IMU motion while OFF (var=%.4f), start detect window", var);
                    ctx.detecting = true;
                    ctx.detect_start_ms = rt_tick_get_millisecond();
                    ctx.detect_ppg_sum = 0;
                    ctx.detect_ppg_count = 0;
                    ctx.imu_idx = 0;
                    ctx.imu_count = 0;
                }
            }
        }
    }
}

void wear_detect_feed_ppg(uint32_t ppg_raw, uint32_t ppg_raw2)
{
    if (!ctx.initialized)
        return;

    /* Update moving average */
    ctx.ppg_history[ctx.ppg_idx] = ppg_raw;
    ctx.ppg_idx = (ctx.ppg_idx + 1) % PPG_AVG_LEN;
    if (ctx.ppg_count < PPG_AVG_LEN)
        ctx.ppg_count++;

    uint32_t sum = 0;
    for (uint16_t i = 0; i < ctx.ppg_count; i++)
        sum += ctx.ppg_history[i];
    uint32_t new_avg = sum / ctx.ppg_count;

    /* Accumulate PPG during detection window */
    if (ctx.detecting)
    {
        ctx.detect_ppg_sum += ppg_raw;
        ctx.detect_ppg_count++;

        /* If PPG is still changing drastically → immediately mark as not wearing,
         * then postpone the window to re-evaluate once stable. */
        uint32_t win_delta = (new_avg > ctx.ppg_prev_avg)
                                 ? (new_avg - ctx.ppg_prev_avg)
                                 : (ctx.ppg_prev_avg - new_avg);
        LOG_D("During window: PPG raw=%u, avg=%u, prev_avg=%u, delta=%u",
              ppg_raw, new_avg, ctx.ppg_prev_avg, win_delta);
        if (win_delta >= PPG_DELTA_THD)
        {
            LOG_D("PPG still unstable (delta=%u), not wearing", win_delta);
            set_status(WEAR_STATUS_NOT_WEARING);
            ctx.detect_start_ms = rt_tick_get_millisecond();
            ctx.detect_ppg_sum = ppg_raw;
            ctx.detect_ppg_count = 1;
            ctx.imu_idx = 0;
            ctx.imu_count = 0;
        }
    }

    /* Check for large PPG change → trigger detection window */
    if (ctx.ppg_count >= PPG_AVG_LEN && !ctx.detecting)
    {
        uint32_t delta = (new_avg > ctx.ppg_prev_avg)
                             ? (new_avg - ctx.ppg_prev_avg)
                             : (ctx.ppg_prev_avg - new_avg);

        // LOG_D("PPG feed: raw=%u, avg=%u, prev_avg=%u, delta=%u", ppg_raw, new_avg,
        //       ctx.ppg_prev_avg, delta);
        if (delta >= PPG_DELTA_THD)
        {
            LOG_I("PPG delta=%u (avg %u->%u), start 3s detect window",
                  delta, ctx.ppg_prev_avg, new_avg);

            ctx.detecting = true;
            ctx.detect_start_ms = rt_tick_get_millisecond();
            ctx.detect_ppg_sum = ppg_raw;
            ctx.detect_ppg_count = 1;

            /* Reset IMU buffer so we only measure motion during this window */
            ctx.imu_idx = 0;
            ctx.imu_count = 0;
        }
    }

    ctx.ppg_prev_avg = ctx.ppg_avg;
    ctx.ppg_avg = new_avg;
}

wear_status_t wear_detect_get_status(void)
{
    return ctx.status;
}

bool wear_detect_is_wearing(void)
{
    return ctx.status == WEAR_STATUS_WEARING;
}
