/**
 ******************************************************************************
 * @file   sleep_vanhees_service.c
 * @author Skaiwalk software development team
 * @brief  LCPU driver for the accel-only van Hees sleep-period detector.
 *         Samples the BMI270 accelerometer at 1 Hz, feeds raw x/y/z to
 *         sleep_vanhees (which does its own 5 s epoching + SIB state machine),
 *         and ships stage transitions to HCPU via the existing watch_sys_sync
 *         RPC -> phone (KEY_RETURN_SLEEP_DATA). No PPG / HR involved, so it
 *         works overnight regardless of the PPG power state.
 *
 *  SOFT timer so PM can keep LCPU in LIGHT sleep between ticks; the accel
 *  data registers are always available (we read them directly, like
 *  sleep_service.c, so this survives screen-off / hand-tracking-off modes).
 ******************************************************************************
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <time.h>
#include <string.h>
#include "board.h"
#include "rtconfig.h"

#include "sleep_vanhees.h"
#include "watch_sys_service.h"

#ifdef BSP_USING_WEAR_DETECT
#include "wear_detect.h"
#endif

#ifdef ACC_USING_BMI270
#include "bmi270_driver.h"
#endif

/* Sleep status enum on the wire (health_algo.h). Mirrored here so this module
   compiles standalone. van Hees has no Deep/REM, so only three are used. */
#define WIRE_TSLEEP        0x01  /* mapped from ASLEEP   (shown as Light)     */
#define WIRE_TGET_UP       0x03  /* mapped from AWAKE                         */
#define WIRE_TNOT_WEAR     0x04  /* mapped from NOT_WORN                      */

#define DBG_TAG "DS.SLEEPVH"
#define DBG_LVL DBG_INFO
#include "rtdbg.h"

#define SLEEP_SAMPLE_PERIOD_MS 1000   /* 1 Hz accel snapshot (van Hees epochs at 5 s) */

typedef struct
{
    rt_timer_t timer;
    bool       started;
    uint8_t    last_emitted_wire_mode; /* 0 = nothing emitted yet */
    int16_t    last_local_day;         /* for midnight reset; -1 sentinel */
} sleep_vh_env_t;

static sleep_vh_env_t s_env;

static uint8_t prv_stage_to_wire(sleep_vh_stage_t stage)
{
    switch (stage)
    {
    case SLEEP_VH_STAGE_ASLEEP:   return WIRE_TSLEEP;
    case SLEEP_VH_STAGE_NOT_WORN: return WIRE_TNOT_WEAR;
    case SLEEP_VH_STAGE_AWAKE:
    default:                      return WIRE_TGET_UP;
    }
}

static bool prv_read_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
#ifdef ACC_USING_BMI270
    if (bmi270_accel_read(ax, ay, az) != 0)
    {
        return false;
    }
    return true;
#else
    (void)ax; (void)ay; (void)az;
    return false;
#endif
}

static bool prv_is_worn(void)
{
#ifdef BSP_USING_WEAR_DETECT
    return wear_detect_is_wearing();
#else
    return true;
#endif
}

static void prv_emit_stage(const sleep_vanhees_output_t *out, uint32_t utc_now)
{
    uint8_t wire_mode = prv_stage_to_wire(out->stage);
    if (wire_mode == s_env.last_emitted_wire_mode)
    {
        return;
    }
    s_env.last_emitted_wire_mode = wire_mode;
    if (watch_sys_sync.notify_sleep_state)
    {
        watch_sys_sync.notify_sleep_state(wire_mode, utc_now);
    }
    LOG_I("sleep(vanhees) -> wire=%u utc=%u angle=%d d=%d total=%u onset=%u wake=%u WASO=%u",
          (unsigned)wire_mode, (unsigned)utc_now,
          (int)out->last_angle_deg, (int)out->last_angle_delta_deg,
          (unsigned)out->total_sleep_min,
          (unsigned)out->sleep_onset_utc, (unsigned)out->last_wake_utc,
          (unsigned)out->awake_after_onset_min);
}

static void prv_timer_cb(void *param)
{
    (void)param;

    int16_t ax = 0, ay = 0, az = 0;
    bool worn = prv_is_worn();
    bool valid = prv_read_accel(&ax, &ay, &az);
    /* If the accel read failed, treat this tick as not-worn so we don't feed
       a bogus (0,0,0) angle into the detector. */
    if (!valid)
    {
        worn = false;
    }

    uint32_t utc_now = (uint32_t)time(NULL);

    /* Local-midnight reset of the daily aggregates (mirror sleep_service.c). */
    {
        time_t t = (time_t)utc_now;
        struct tm *lt = localtime(&t);
        if (lt)
        {
            int16_t today = (int16_t)lt->tm_yday;
            if (s_env.last_local_day < 0)
            {
                s_env.last_local_day = today;
            }
            else if (today != s_env.last_local_day)
            {
                sleep_vanhees_midnight_reset();
                s_env.last_local_day = today;
                LOG_I("sleep(vanhees): midnight reset (yday %d)", (int)today);
            }
        }
    }

    const sleep_vanhees_output_t *out =
        sleep_vanhees_update_1hz(utc_now, ax, ay, az, worn);
    if (out->stage_changed)
    {
        prv_emit_stage(out, utc_now);
    }
}

int sleep_vanhees_service_register(void)
{
    if (s_env.started)
    {
        return 0;
    }

    memset(&s_env, 0, sizeof(s_env));
    s_env.last_local_day = -1;

    sleep_vanhees_init();

    s_env.timer = rt_timer_create("slp_vh", prv_timer_cb, NULL,
                                  rt_tick_from_millisecond(SLEEP_SAMPLE_PERIOD_MS),
                                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (!s_env.timer)
    {
        LOG_E("sleep_vanhees: timer create failed");
        return 1;
    }
    rt_timer_start(s_env.timer);
    s_env.started = true;
    LOG_I("sleep_vanhees: armed (1 Hz sampler, accel-only SIB detector)");
    return 0;
}

INIT_COMPONENT_EXPORT(sleep_vanhees_service_register);

/* ------------------------------------------------------------------
 *  MSH debug commands -- wrapped in !kReleaseMode per project convention
 * ------------------------------------------------------------------ */
#if !defined(kReleaseMode) || (kReleaseMode == 0)

static int msh_sleepvh_status(int argc, char **argv)
{
    (void)argc; (void)argv;
    const sleep_vanhees_output_t *out = sleep_vanhees_current();
    rt_kprintf("sleep_vanhees: stage=%u in_stage_min=%u total=%u WASO=%u\n",
               (unsigned)out->stage,
               (unsigned)out->consecutive_minutes_in_stage,
               (unsigned)out->total_sleep_min,
               (unsigned)out->awake_after_onset_min);
    rt_kprintf("  angle=%d delta=%d onset_utc=%u last_wake_utc=%u\n",
               (int)out->last_angle_deg, (int)out->last_angle_delta_deg,
               (unsigned)out->sleep_onset_utc, (unsigned)out->last_wake_utc);
    return 0;
}
MSH_CMD_EXPORT(msh_sleepvh_status, dump van Hees sleep detector state);

static int msh_sleepvh_reset(int argc, char **argv)
{
    (void)argc; (void)argv;
    sleep_vanhees_reset();
    s_env.last_emitted_wire_mode = 0;
    s_env.last_local_day = -1;
    rt_kprintf("sleep_vanhees: hard reset\n");
    return 0;
}
MSH_CMD_EXPORT(msh_sleepvh_reset, hard reset van Hees sleep detector);

#endif /* !kReleaseMode */
