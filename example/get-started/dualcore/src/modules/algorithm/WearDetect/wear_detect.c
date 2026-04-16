/**
 ******************************************************************************
 * @file   wear_detect.c
 * @author Skaiwalk software development team
 * @brief  Wear detection algorithm combining IMU and PPG sensor signals.
 *
 *         Detection principle (continuous, multi-indicator fusion):
 *
 *         Four indicators are evaluated every EVAL_PERIOD_MS:
 *
 *         1. PPG freshness: if PPG sensor is off (no new samples),
 *            fall back to IMU-only re-trigger to wake PPG up.
 *
 *         2. PPG DC Level (contact detection):
 *            - DC < threshold → nothing touching → NOT wearing.
 *
 *         3. Perfusion Index (PI = AC / DC) + variability:
 *            - PI > threshold AND varying → heartbeat → WEARING.
 *            - PI > threshold BUT constant → noise → NOT wearing.
 *
 *         4. IMU Variance (supplementary):
 *            - Used to disambiguate when PI is low but DC is high.
 *            - Also used as sole re-trigger when PPG data is stale.
 *
 *         State transitions use hysteresis counters to prevent oscillation.
 ******************************************************************************
 */
#include <rtthread.h>
#include <math.h>
#include <string.h>
#include "wear_detect.h"
#include "bloc_peripheral.h"
#include "watch_sys_service.h"

#define DBG_TAG "wear_detect"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

/* -------------------- Configuration -------------------- */

/* Evaluation period in milliseconds (continuous, not event-driven) */
#define EVAL_PERIOD_MS          3000

/* PPG DC threshold: below this → nothing touching the sensor (suspended in air).
 * Measured data: air DC ~= 40000-41000, contact (wrist/table) DC >= 48000.
 * Set between these ranges for immediate OFF detection. */
#define PPG_DC_LOW_THD          45000

/* Perfusion Index threshold (AC_pp / DC_mean).
 * Measured data: wearing PI >= 0.00052, off-wrist PI ~= 0.00029.
 * Set between these two ranges. */
#define PI_THD                  0.0004f

/* IMU variance threshold (m/s^2)^2 for supplementary motion check */
#define IMU_VARIANCE_THD        0.03f

/* Hysteresis: consecutive evaluations needed to change state */
#define HYSTERESIS_ON           2   /* OFF→ON:  2 consecutive ON  (~6s) */
#define HYSTERESIS_OFF          4   /* ON→OFF:  4 consecutive OFF (~12s) */

/* PI variability check: real heartbeat causes PI to fluctuate across
 * evaluations.  Constant PI (noise/static surface) should be rejected.
 * Track the last PI_HISTORY_LEN evaluations and require the range
 * (max - min) to exceed PI_RANGE_THD before accepting PI as heartbeat. */
#define PI_HISTORY_LEN          5
#define PI_RANGE_THD            0.0003f

/* PPG freshness: if no new PPG sample arrives within this many
 * milliseconds, consider PPG data stale (sensor likely powered off). */
#define PPG_STALE_MS            5000

/* PPG settle time: after PPG sensor restarts, the first few seconds
 * produce wildly inaccurate readings (huge PI spikes).  Ignore PPG
 * data during this settle period. */
#define PPG_SETTLE_MS           6000

/* When PPG is stale and device is OFF-wrist, use IMU motion to
 * re-trigger ON (which causes system to restart PPG sensor).
 * Require N consecutive IMU-motion evaluations before voting ON. */
#define IMU_RETRIGGER_COUNT     2
#define IMU_RETRIGGER_THD       0.05f

/* PPG ring buffer size: at ~25 Hz, 75 samples = 3 seconds */
#define PPG_WINDOW_SIZE         75

/* IMU ring buffer size: at ~25 Hz, 75 samples = 3 seconds */
#define IMU_WINDOW_SIZE         75

/* -------------------- Internal state -------------------- */

typedef struct
{
    /* PPG ring buffer for PI calculation */
    uint32_t ppg_buf[PPG_WINDOW_SIZE];
    uint16_t ppg_idx;
    uint16_t ppg_count;

    /* IMU ring buffer (acceleration magnitude) */
    float acce_mag[IMU_WINDOW_SIZE];
    uint16_t imu_idx;
    uint16_t imu_count;

    /* PI history for variability check */
    float pi_history[PI_HISTORY_LEN];
    uint8_t pi_hist_idx;
    uint8_t pi_hist_count;

    /* PPG freshness tracking */
    uint32_t last_ppg_ms;       /* timestamp of last PPG sample */
    bool ppg_ever_received;     /* true after first PPG sample */

    /* PPG settle: timestamp when PPG resumed after a stale gap */
    uint32_t ppg_restart_ms;
    bool ppg_settling;          /* true during settle period after restart */

    /* IMU re-trigger counter (for stale PPG + OFF state) */
    uint8_t imu_retrigger_cnt;

    /* Timing for periodic evaluation */
    uint32_t last_eval_ms;

    /* Hysteresis counters */
    int8_t on_counter;      /* counts consecutive ON  evaluations */
    int8_t off_counter;     /* counts consecutive OFF evaluations */

    /* Current output */
    wear_status_t status;
    bool initialized;
} wear_detect_ctx_t;

static wear_detect_ctx_t ctx;

/* -------------------- Helpers -------------------- */

static float compute_imu_variance(const float *buf, uint16_t len)
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

/**
 * @brief Compute PPG DC mean, AC peak-to-peak, and Perfusion Index
 */
static void compute_ppg_metrics(float *dc_mean, float *ac_pp, float *pi)
{
    uint16_t len = (ctx.ppg_count < PPG_WINDOW_SIZE)
                       ? ctx.ppg_count : PPG_WINDOW_SIZE;

    if (len == 0)
    {
        *dc_mean = 0.0f;
        *ac_pp = 0.0f;
        *pi = 0.0f;
        return;
    }

    uint32_t ppg_min = UINT32_MAX;
    uint32_t ppg_max = 0;
    uint64_t ppg_sum = 0;

    for (uint16_t i = 0; i < len; i++)
    {
        uint32_t v = ctx.ppg_buf[i];
        ppg_sum += v;
        if (v < ppg_min) ppg_min = v;
        if (v > ppg_max) ppg_max = v;
    }

    *dc_mean = (float)ppg_sum / (float)len;
    *ac_pp = (float)(ppg_max - ppg_min);

    if (*dc_mean > 1.0f)
        *pi = *ac_pp / *dc_mean;
    else
        *pi = 0.0f;
}

static bool is_ppg_stale(uint32_t now)
{
    if (!ctx.ppg_ever_received)
        return true;
    return (now - ctx.last_ppg_ms) > PPG_STALE_MS;
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

/* -------------------- Core evaluation -------------------- */

/**
 * @brief Evaluate using only IMU when PPG data is stale.
 *        Used to re-trigger ON state so system restarts PPG sensor.
 * @return  1 = vote ON,  0 = no change
 */
static int evaluate_imu_only(void)
{
    uint16_t imu_len = (ctx.imu_count < IMU_WINDOW_SIZE)
                           ? ctx.imu_count : IMU_WINDOW_SIZE;
    if (imu_len == 0)
        return 0;

    float imu_var = compute_imu_variance(ctx.acce_mag, imu_len);

    if (imu_var >= IMU_RETRIGGER_THD)
    {
        ctx.imu_retrigger_cnt++;
        LOG_I("Eval: PPG stale, IMU_var=%.5f (motion %u/%u)",
              imu_var, ctx.imu_retrigger_cnt, IMU_RETRIGGER_COUNT);

        if (ctx.imu_retrigger_cnt >= IMU_RETRIGGER_COUNT)
        {
            ctx.imu_retrigger_cnt = 0;
            return 1; /* vote ON to trigger PPG restart */
        }
    }
    else
    {
        ctx.imu_retrigger_cnt = 0;
        LOG_D("Eval: PPG stale, IMU_var=%.5f (no motion)", imu_var);
    }

    return 0;
}

/**
 * @brief Evaluate all indicators and determine wear/not-wear vote.
 * @return  1 = vote ON,  -1 = vote OFF,  0 = uncertain
 */
static int evaluate_once(uint32_t now)
{
    /* --- Check PPG freshness first --- */
    if (is_ppg_stale(now))
    {
        /* PPG sensor is off. Only IMU can re-trigger. */
        if (ctx.status == WEAR_STATUS_NOT_WEARING)
        {
            return evaluate_imu_only();
        }
        /* If currently ON but PPG went stale, don't change state yet */
        return 0;
    }

    /* Reset IMU retrigger counter since PPG is active */
    ctx.imu_retrigger_cnt = 0;

    /* --- Check PPG settle period after sensor restart --- */
    if (ctx.ppg_settling)
    {
        if (now - ctx.ppg_restart_ms < PPG_SETTLE_MS)
        {
            LOG_D("Eval: PPG settling (%u ms remaining)",
                  PPG_SETTLE_MS - (now - ctx.ppg_restart_ms));
            return 0; /* don't vote during settle */
        }
        ctx.ppg_settling = false;
        /* Clear PPG buffer and PI history to use only post-settle data */
        ctx.ppg_count = 0;
        ctx.ppg_idx = 0;
        ctx.pi_hist_count = 0;
        ctx.pi_hist_idx = 0;
        LOG_I("PPG settle complete, cleared buffers");
        return 0; /* wait for fresh data next eval */
    }

    float dc_mean, ac_pp, pi;
    compute_ppg_metrics(&dc_mean, &ac_pp, &pi);

    /* Record PI into history ring buffer */
    ctx.pi_history[ctx.pi_hist_idx] = pi;
    ctx.pi_hist_idx = (ctx.pi_hist_idx + 1) % PI_HISTORY_LEN;
    if (ctx.pi_hist_count < PI_HISTORY_LEN)
        ctx.pi_hist_count++;

    /* Compute PI variability (range = max - min over recent history) */
    float pi_range = 0.0f;
    if (ctx.pi_hist_count >= 2)
    {
        float pi_min = ctx.pi_history[0];
        float pi_max = ctx.pi_history[0];
        for (uint8_t i = 1; i < ctx.pi_hist_count; i++)
        {
            if (ctx.pi_history[i] < pi_min) pi_min = ctx.pi_history[i];
            if (ctx.pi_history[i] > pi_max) pi_max = ctx.pi_history[i];
        }
        pi_range = pi_max - pi_min;
    }

    /* --- Indicator 1: DC level (contact) --- */
    if (dc_mean < (float)PPG_DC_LOW_THD)
    {
        LOG_I("Eval: DC=%.0f (< %u) -> no contact -> OFF", dc_mean, PPG_DC_LOW_THD);
        return -1;
    }

    /* --- Indicator 2: Perfusion Index (heartbeat) --- */
    if (pi >= PI_THD)
    {
        /* Check PI variability: real heartbeat causes PI to fluctuate.
         * Constant PI (noise/static surface) should be rejected. */
        if (ctx.pi_hist_count >= PI_HISTORY_LEN && pi_range < PI_RANGE_THD)
        {
            LOG_I("Eval: DC=%.0f, AC_pp=%.0f, PI=%.5f, pi_range=%.5f (constant -> noise) -> OFF",
                  dc_mean, ac_pp, pi, pi_range);
            return -1;
        }

        LOG_I("Eval: DC=%.0f, AC_pp=%.0f, PI=%.5f, pi_range=%.5f -> ON",
              dc_mean, ac_pp, pi, pi_range);
        return 1;
    }

    /* --- Indicator 3: IMU variance (supplementary) --- */
    uint16_t imu_len = (ctx.imu_count < IMU_WINDOW_SIZE)
                           ? ctx.imu_count : IMU_WINDOW_SIZE;
    if (imu_len > 0)
    {
        float imu_var = compute_imu_variance(ctx.acce_mag, imu_len);
        LOG_I("Eval: DC=%.0f, PI=%.5f (low), pi_range=%.5f, IMU_var=%.5f",
              dc_mean, pi, pi_range, imu_var);

        if (imu_var >= IMU_VARIANCE_THD)
        {
            return 0; /* uncertain */
        }
        else
        {
            return -1; /* static object → OFF */
        }
    }

    return 0; /* not enough data */
}

/**
 * @brief Run periodic evaluation with hysteresis
 */
static void try_evaluate(void)
{
    uint32_t now = rt_tick_get_millisecond();
    if (now - ctx.last_eval_ms < EVAL_PERIOD_MS)
        return;
    ctx.last_eval_ms = now;

    /* Need minimum data before evaluating (PPG or IMU) */
    if (ctx.ppg_count < 20 && ctx.imu_count < 20)
        return;

    int vote = evaluate_once(now);

    if (vote > 0)
    {
        /* Vote ON */
        ctx.off_counter = 0;
        ctx.on_counter++;
        if (ctx.on_counter > HYSTERESIS_ON)
            ctx.on_counter = HYSTERESIS_ON;

        if (ctx.status != WEAR_STATUS_WEARING && ctx.on_counter >= HYSTERESIS_ON)
        {
            /* Reset PI history and PPG buffer on state change so fresh
             * PPG data will be evaluated after sensor restarts. */
            ctx.pi_hist_count = 0;
            ctx.pi_hist_idx = 0;
            set_status(WEAR_STATUS_WEARING);
        }
    }
    else if (vote < 0)
    {
        /* Vote OFF */
        ctx.on_counter = 0;
        ctx.off_counter++;
        if (ctx.off_counter > HYSTERESIS_OFF)
            ctx.off_counter = HYSTERESIS_OFF;

        if (ctx.status != WEAR_STATUS_NOT_WEARING && ctx.off_counter >= HYSTERESIS_OFF)
        {
            set_status(WEAR_STATUS_NOT_WEARING);
        }
    }
    else
    {
        /* Uncertain: decay both counters slowly */
        if (ctx.on_counter > 0) ctx.on_counter--;
        if (ctx.off_counter > 0) ctx.off_counter--;
    }
}

/* -------------------- Public API -------------------- */

void wear_detect_init(void)
{
    memset(&ctx, 0, sizeof(ctx));
    ctx.status = WEAR_STATUS_WEARING; /* default: assume wearing at boot */
    ctx.last_eval_ms = rt_tick_get_millisecond();
    ctx.initialized = true;
    LOG_I("Wear detection initialized (PI-based, eval every %u ms)", EVAL_PERIOD_MS);
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

    /* Try periodic evaluation */
    try_evaluate();
}

void wear_detect_feed_ppg(uint32_t ppg_raw, uint32_t ppg_raw2)
{
    if (!ctx.initialized)
        return;

    /* Update PPG freshness timestamp */
    ctx.last_ppg_ms = rt_tick_get_millisecond();
    ctx.ppg_ever_received = true;

    /* Store PPG sample into ring buffer */
    ctx.ppg_buf[ctx.ppg_idx] = ppg_raw;
    ctx.ppg_idx = (ctx.ppg_idx + 1) % PPG_WINDOW_SIZE;
    if (ctx.ppg_count < PPG_WINDOW_SIZE)
        ctx.ppg_count++;

    /* Try periodic evaluation */
    try_evaluate();
}

wear_status_t wear_detect_get_status(void)
{
    return ctx.status;
}

bool wear_detect_is_wearing(void)
{
    return ctx.status == WEAR_STATUS_WEARING;
}
