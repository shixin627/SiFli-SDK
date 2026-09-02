/**
 ******************************************************************************
 * @file   wear_detect.c
 * @author Skaiwalk software development team
 * @brief  Wear detection — DC-session-led (v2, industry-standard shape).
 *
 *         Principle (matches mainstream wearables): the PULSE is only ever
 *         required to confirm the FIRST wear of a session ("cold entry").
 *         Staying worn is decided by CONTACT evidence — the PPG DC level
 *         against a per-session learned baseline. RE-becoming worn needs both
 *         contact and a minimum pulse.
 *
 *         2026-08-30: that last clause is new, and it reverses the reasoning
 *         below. Warm re-entry used to take DC alone because "deep-sleep
 *         perfusion is too weak to demand a pulse" (overnight CSV 2026-06-11:
 *         82% of the night falsely OFF, 5.5 h stuck, 48 probes all failing the
 *         pulse gate while DC sat in the worn band). That diagnosis was right
 *         about the symptom and wrong about the cause: the gate was 0.0010
 *         while a real sleeping wrist's median perfusion is 0.00083. The
 *         threshold was above its own signal. With it moved to 0.0004 (see
 *         PI_THD_TO_ON) a pulse is affordable everywhere, and the exemption
 *         that made a table indistinguishable from a wrist is no longer the
 *         price of sleeping through the night.
 *
 *         State machine:
 *         - COLD OFF→ON: windowed pulse gate (PI ≥ threshold + variability,
 *           3-of-8 evals). Seeds worn_dc_base = current DC.
 *         - WARM OFF→ON: DC back inside [0.88, 1.15] × worn_dc_base AND a
 *           minimum pulse, for 3-of-8 evals → ON. Still what makes a wrong OFF
 *           cheap (seconds) instead of catastrophic (hours) — 98% of
 *           genuinely-worn samples clear the pulse floor — while no longer
 *           re-admitting a desk.
 *         - ON hold: DC above max(0.85 × base, DC_ABS_FLOOR). Baseline EMA
 *           adapts slowly while in-band (per-unit / per-fit calibration —
 *           measured worn DC differs >4k between our two units).
 *         - ON→OFF: sustained sub-threshold DC — fast (~12 s) when the dip
 *           came with motion (take-off signature), slow (~30 s) when still
 *           (protects sleep dips). Charger (is_plugged) blocks WARM entry
 *           and freezes the EMA, so a charging cradle can't be learned or
 *           warm-recovered into.
 *
 *         That trade-off used to be accepted and documented here as "a surface
 *         whose DC coincides with the learned worn band can warm-recover to a
 *         false ON". It was not theoretical: on the bench unit the detector
 *         flipped ON/OFF 25-52 times a day (2026-08-29: 52 ON, 50 OFF) with a
 *         median PI of 0.000387 at every ON, and a watch left on a desk was
 *         held "worn" for hours while the HR pipeline published ~150 bpm of
 *         noise into it. For a wearer that is a watch on the nightstand
 *         inventing a night's sleep. Requiring the pulse here is what ends it.
 *         Sessions are still bounded by charging and the settings
 *         toggle exists as an escape hatch. See ADR 0015.
 ******************************************************************************
 */
/* Temporary PPG stall diagnostic instrumentation. Remove once done. */
#ifndef PPG_RACE_DEBUG
    #define PPG_RACE_DEBUG 1
#endif

#include <rtthread.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "bsp_board.h"
#include "wear_detect.h"
#include "bloc_peripheral.h"
#include "bloc_battery.h"
#include "watch_sys_service.h"

#define DBG_TAG "wear_detect"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

/* -------------------- Configuration -------------------- */

/* Evaluation period in milliseconds (continuous, not event-driven) */
#define EVAL_PERIOD_MS          1500

/* Absolute DC floor: below this, nothing skin-like touches the sensor.
 * Overnight CSV (sleep unit, 2026-06-11): off-wrist surfaces read 30.5k-35.5k,
 * worn sleep dips bottom out at 36.4k, worn band 39k-44k. The floor sits in
 * the 35.5k-36.4k gap. It is only a safety net / cold-entry sanity check —
 * the working ON/OFF decisions use the per-session learned baseline below. */
#define DC_ABS_FLOOR            36000

/* Per-session DC baseline (learned, replaces any fixed worn threshold —
 * measured worn DC differs >4k between units and with strap fit):
 * - Seeded by the first pulse-confirmed ON of a session.
 * - EMA-adapts while ON and in-band (alpha 1/64 per 1.5s eval ≈ 96s to 63%).
 * - WORN_BAND: DC within [LO,HI]×base counts as wrist contact (hold + warm
 *   re-entry). Night data: worn dips reach 0.90×base, so LO=0.88 keeps them.
 * - OFF_THR: DC below OFF_THR×base (clamped up to DC_ABS_FLOOR) votes OFF. */
#define WORN_BAND_LO            0.88f
#define WORN_BAND_HI            1.15f
/* 2026-08-31: was 0.80f. Measured on the wearer's own watch across three
 * conditions in one day, with the truth known for each (worn 00:00-09:40,
 * then on a desk until 11:56, then back on):
 *
 *                       n    DC median      vs worn
 *   on the wrist      247       47144
 *   desk, PPG in air   34       38122        -19.1%
 *   desk, PPG on wood  24       39724        -15.7%
 *
 * Taking the watch off drops DC by 16-19%. The exit test needed a drop of
 * more than 20%. It never fired: across 2 h 16 min on a desk the detector
 * emitted ZERO OFF events and held "worn" for 100% of samples. This is not a
 * threshold that is slightly off — a 0.80 factor asks for a drop this sensor
 * does not produce when the watch comes off a wrist.
 *
 * Per-sample, over those same three windows:
 *   DC >= 0.80*base   worn 98%   air 100%   wood 100%   <- cannot separate
 *   DC >= 0.85*base   worn 98%   air  24%   wood  33%
 * Worn is unchanged at 98% because a wrist rarely dips near 0.85 (worn p10 is
 * 45976 = 0.975*base); the 2% that do are brief and cannot outlast the exit
 * hysteresis. 0.82 through 0.90 all score the same, so this sits on a plateau,
 * not a tuned point.
 *
 * NOT fixed by this: the residual 24-33% is desk samples whose DC spikes into
 * the worn range (up to 48840, above the wrist's own median). DC alone stops
 * here. A PI floor was measured too and rejected — see the note below.
 *
 * ⚠ One wearer, one watch, one day, two surfaces. */
#define OFF_THR_FACTOR          0.85f

/* Why there is no PI test in the exit path, having measured one:
 * the two off-wrist conditions sit on OPPOSITE sides of the worn PI median
 * (0.001095) — PPG in air reads 0.003216 (3x high, ambient/noise amplified by
 * the lower DC), PPG on wood reads 0.000352 (3x low). No single-sided PI gate
 * catches both, and a two-sided band is worse: 29% of genuinely-worn samples
 * read above 0.0025 (motion artefact), 21% of them pinned at the u16 ceiling,
 * so an upper bound would reject real wear outright. Adding only the lower
 * bound (PI >= PI_THD_TO_ON) buys 3-12 points of desk rejection for 4 points
 * of real wear (98% -> 94%); not worth it against a single constant. */
/* Asymmetric EMA: learn DOWN (loosening fit) at 1/64 per eval, UP at only
 * 1/256 — a pressed-sensor episode (DC 45k+) must not inflate the base and
 * drag off_thr above the 36.4k sleep-dip floor margin. */
#define BASE_EMA_SHIFT          6       /* down: base += (dc-base)/64  */
#define BASE_EMA_UP_SHIFT       8       /* up:   base += (dc-base)/256 */

/* Perfusion Index threshold (AC_pp / DC_mean) — ENTRY ONLY (cold and, since
 * 2026-08-30, warm too). A live pulse proves the contact is a wrist and not a
 * table. It is deliberately NOT part of the exit test — see OFF_THR_FACTOR.
 *
 * 2026-08-30: was 0.0010f, which sat ABOVE the signal it was gating. Measured
 * on wear_diag against two windows whose truth is certain — the wearer's own
 * reported sleep (2026-08-30 04:00-11:20, 191 samples) and the bench watch
 * lying on a desk while being reflashed (2026-08-29 15:00 - 08-30 04:00, 325):
 *
 *              worn wrist        desk
 *   DC median     47356         42904     <- 11% apart; the +-12/15% band
 *                                            cannot separate these at all
 *   PI median   0.000834      0.000281    <- this can
 *
 * A real sleeping wrist's median perfusion is 0.00083. The gate was 0.00100.
 * So the pulse test passed on only 36% of genuinely-worn samples, and the old
 * comment's "deep-sleep perfusion cannot deliver it" was true — of that
 * threshold, not of the signal. Sweeping the threshold against those two
 * windows: 0.0004 accepts 98% of worn samples and rejects 81% of desk ones
 * (separation 79%), against 36%/85% (separation 21%) at 0.0010.
 *
 * This also explains ADR-0015's "48 probe windows, none cleared PI 0.0010":
 * not weak physiology, a threshold set above the median of what it measures.
 *
 * Because the gate now sits where the signal is, warm re-entry can demand a
 * pulse too — which is what actually stops a desk from being re-entered. */
#define PI_THD_TO_ON            0.0004f

/* IMU variance threshold (m/s^2)^2: "the wrist is moving" marker, used to
 * pick the fast OFF hysteresis on take-off-shaped DC drops. */
#define IMU_VARIANCE_THD        0.03f

/* ON-vote accumulation: HYSTERESIS_ON votes within the last 8 evaluations
 * (~12 s window, not consecutive — a marginal signal that crosses its bar
 * intermittently still accumulates; measured crossing rate can be ~30%). */
#define HYSTERESIS_ON           3

/* OFF hysteresis (consecutive sub-threshold evals):
 * - with motion in the streak (take-off signature): fast, ~12 s.
 * - still (loose strap / sleep dips): slow, ~30 s — a sleeping wrist's DC
 *   dip is transient, a removed watch's is not. Wrong OFFs are cheap now
 *   anyway: warm re-entry restores ON within seconds of DC returning. */
#define OFF_EVALS_MOTION        8
#define OFF_EVALS_STILL         20

/* PI variability check (cold entry): real heartbeat fluctuates; constant PI
 * (noise/static surface) is rejected, wildly swinging PI (set-down motion
 * artefact, measured ~0.27) is rejected by PI_RANGE_MAX. */
#define PI_HISTORY_LEN          5
#define PI_RANGE_THD            0.0003f

/* === Off-wrist by STILLNESS of the perfusion signal ==========================
 * Second exit path, added 2026-09-01 after the DC route was measured dead.
 *
 * Ground truth that day (founder): worn 00:00-09:54, then the same watch on a
 * desk with the PPG facing open air until 10:39, then flat on the table.
 *
 *   pi_range median   wrist 0.00104 | desk air 0.00017 | desk flat 0.00007
 *
 * DC cannot see either of those: flat on the table dc/base is 1.11, i.e. ABOVE
 * the learned baseline, so `dc < base * OFF_THR_FACTOR` can never fire at any
 * factor. PI LEVEL cannot either -- it is non-monotonic, the wrist (0.0016)
 * sits BETWEEN desk-air (0.0032) and desk-flat (0.00033), so no single-sided
 * threshold covers both surfaces. Only the VARIABILITY separates them, and
 * being a ratio it is immune to the AGC gain steps that move DC by 29% while
 * the watch never leaves the wrist.
 *
 * Measured on that day, at 0.00025 with a 5-minute run: 0/198 false exits over
 * ten hours of real wear INCLUDING A FULL NIGHT ASLEEP, while catching 64% of
 * flat-on-table and 50% of facing-air minutes.
 *
 * KNOWN UNTESTED CASE: off the wrist but MOVING (in a bag, in a pocket).
 * Vibration would keep pi_range up and this test would not fire. That failure
 * is a MISS, never a false exit -- it leaves us exactly where we are today, so
 * the risk of shipping it is bounded. Do not "fix" it by lowering the run
 * length without new data; the run length is what buys the 0/198. */
#define PI_RANGE_FLAT_THD       0.00025f
/* u16 wire saturation of pi/pi_range (65535/1e6). At or above this the reading
 * is invalid, not "high perfusion" -- 21% of genuinely-worn samples pin here.
 * Invalid readings must RESET the streak, never extend it. */
#define PI_SATURATED            0.0655f
/* Consecutive one-minute checks required. Sampled at 60 s -- the same cadence
 * as the wear_diag CSV the threshold was fitted on -- NOT once per
 * EVAL_PERIOD_MS (1.5 s), which would be a 7.5-second test and would have no
 * relation to the evidence. */
#define FLAT_CHECK_MS           60000u
#define FLAT_CHECKS_TO_OFF      5

/* Same evidence, opposite direction: warm re-entry must also see the perfusion
 * signal MOVING. Measured 2026-09-01 (pi_range medians): wrist 0.00104,
 * desk-in-air 0.00017, desk-flat 0.00007.
 *
 * Without this the desk walks straight back in. The 2026-08-30 pulse gate on
 * this branch (pi >= PI_THD_TO_ON) does not stop it, because PI LEVEL is
 * non-monotonic: a PPG facing open air reads 0.0032, EIGHT TIMES the 0.0004
 * gate and higher than a real wrist's 0.0016. Observed on the bench the same
 * day: exit fired correctly at 12:31:16, and by 12:38 the watch was reporting
 * worn again while still lying on the desk — dc/base was 1.068, inside the
 * [0.88, 1.15] band, and pi was 0.0032, so every existing condition passed.
 *
 * Set below the worn p5 (0.00019) so a quiet sleeping wrist still gets back in:
 * this branch re-evaluates every EVAL_PERIOD_MS, so it only needs ONE sample
 * over the line, not a majority. */
/* Threshold scan on the labelled day (blocked desk-air / blocked desk-flat /
 * admitted wrist):  0.00015 -> 50%/96%/98% | 0.00025 -> 88%/96%/89% |
 * 0.00040 -> 96%/96%/70%.
 *
 * A SINGLE sample over the line is not enough: this branch re-evaluates every
 * EVAL_PERIOD_MS (1.5 s), so even a 4% pass rate lets the desk back in about
 * every 37 seconds. A threshold cannot beat unlimited retries, so entry demands
 * a RUN, exactly like the exit does.
 *
 * The run is short (3) because entry latency already costs up to 7.5 minutes
 * (IMU only reaches us during an HR burst, @ref bmi270_driver.c), and a wrist
 * admitted at 70% per evaluation clears three in a row within seconds.
 *
 * The three evaluations are NOT independent: PI history spans 5 samples over
 * ~7.5 s while evaluation runs every 1.5 s, so consecutive evaluations share
 * most of their window. Read "3 in a row" as "the range stayed up for ~4.5 s",
 * NOT as 0.04^3 -- do not quote a probability for it. */
#define PI_RANGE_ALIVE_THD      0.00040f
/* N-of-M, not N-consecutive. 2026-09-02 on the release watch, flat on a desk:
 * exit fired correctly at 09:55:26, and 36 s later a warm re-entry took it
 * straight back — pi_range had spiked to 0.0014 for a few evals while the AGC
 * hunted between gain steps (dc 40412 -> 49184 -> 40380, pi/pi_range pinned at
 * the u16 ceiling). Three consecutive evals span ~4.5 s; an AGC hunt lasts
 * longer than that. Demanding 6 of the last 10 (~15 s) outlasts the transient,
 * while a wrist admitted at 70% per eval expects 7/10 and clears it. */
#define ALIVE_WINDOW_EVALS      10
#define ALIVE_EVALS_TO_ON       6
/* After an OFF verdict, warm re-entry is refused for this long. The OFF verdict
 * asks hr_set_power(0), but that is VETOED while a bg_hr burst is in flight, so
 * the PPG keeps feeding us and the very next evals can re-enter on the post-
 * verdict transient. Cold entry (motion -> probe) is NOT affected by this, so a
 * real re-wear still comes back promptly. */
#define WARM_REENTRY_COOLDOWN_MS 60000u
#define PI_RANGE_MAX            0.15f

/* PPG freshness: if no new PPG sample arrives within this many
 * milliseconds, consider PPG data stale (sensor likely powered off). */
#define PPG_STALE_MS            5000

/* If PPG stays stale this long WHILE WORN, the sensor has likely wedged
 * (not just a normal gap) -- power-cycle it. Must sit ABOVE the awake bg_hr
 * duty cycle (a burst every ~15 min with the LED deliberately off in
 * between), or every normal inter-burst gap gets misread as a wedge and the
 * LED blinks in a restart loop. */
#define PPG_STALE_RESTART_MS    (20 * 60 * 1000)

/* PPG settle time: after PPG sensor restarts, the first few seconds
 * produce wildly inaccurate readings (huge PI spikes).  Ignore PPG
 * data during this settle period. */
#define PPG_SETTLE_MS           6000

/* When PPG is stale and device is OFF-wrist, IMU motion powers PPG up for a
 * probe window so contact (warm) or pulse (cold) can confirm a wrist. */
#define IMU_RETRIGGER_THD       0.05f

/* [DC-led] After motion while off-wrist, hold PPG powered for this long so
 * the wrist can confirm even if the user then stays still. If nothing
 * confirms within the window, PPG is powered back down until next motion. */
#define PROBE_WINDOW_MS         (3 * 60 * 1000)

/* While OFF with no motion, still open a probe at this interval so a
 * wrongly-voted OFF on a resting wrist can warm re-enter (see the fallback
 * comment in evaluate_imu_only). The fallback window is much shorter than
 * the motion one: settle (6s) + a few evals is enough to warm re-enter a
 * still wrist, and a watch resting on a desk must not strobe its LED 30% of
 * the time (3min/10min) forever. */
#define PROBE_FALLBACK_MS       (10 * 60 * 1000)
#define PROBE_FALLBACK_WINDOW_MS (30 * 1000)

/* Detection-disabled bypass: hold "on charger" for this long after the
 * charge IC last reported charging. A topping-off battery cycles
 * charging<->full every few seconds to minutes; without this latch the
 * bypass flips WEARING<->NOT_WEARING on every cycle and the PPG LED
 * blinks on the cradle all morning (seen in wear_diag 2026-07-02). */
#define PLUGGED_HOLD_MS         (90 * 1000)

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

    /* Timestamp (0 = not tracking) when PPG first went stale while worn.
     * Reset the instant a fresh PPG sample arrives; if it grows past
     * PPG_STALE_RESTART_MS the sensor is power-cycled (see evaluate_once). */
    uint32_t on_stale_since_ms;

    /* Timing for periodic evaluation */
    uint32_t last_eval_ms;

    /* Hysteresis state */
    uint8_t on_vote_window; /* bitmask of last 8 eval votes, bit set = ON vote */
    int8_t off_counter;     /* counts consecutive OFF evaluations */
    bool off_streak_motion; /* motion seen during the current OFF-vote streak
                               (take-off signature -> fast hysteresis) */

    /* Per-session worn-DC baseline. 0 = invalid (cold: no wear confirmed
     * since boot / since charging while off-wrist ended the session).
     * (Re)seeded by EVERY pulse-confirmed ON; EMA-adapted while the current
     * ON stretch is pulse-anchored; survives OFF periods so warm re-entry
     * works at night. */
    float worn_dc_base;
    bool last_vote_pulse;   /* the most recent +1 vote came from the pulse gate */
    bool on_anchor_pulse;   /* current ON stretch was entered via pulse (EMA gate) */

    /* Diagnostic uplink: last computed metrics (events fire outside the
       eval scope) + sample-rate limiter */
    float last_dc;
    float last_pi;
    float last_pi_range;
    float last_imu_var;
    uint32_t last_diag_sample_ms;

    /* Perfusion-stillness exit (@ref PI_RANGE_FLAT_THD): one check a minute. */
    uint32_t last_flat_check_ms;
    uint8_t  flat_streak;
    /* Perfusion-movement entry (@ref PI_RANGE_ALIVE_THD): N-of-M ring. */
    uint16_t alive_ring;         /* bit i = eval i (of last ALIVE_WINDOW_EVALS) was alive */
    uint8_t  alive_ring_n;
    uint32_t last_off_verdict_ms;

    /* Current output */
    wear_status_t status;
    bool initialized;
} wear_detect_ctx_t;

static wear_detect_ctx_t ctx;

/* [DC-led] Motion-triggered PPG probe window. PPG power lives in hr_service;
 * wear_detect asks it to power up (probe) and back down when nothing is worn. */
extern void hr_set_power(uint8_t arg);
static bool s_probe_active = false;
static uint32_t s_probe_until_ms = 0;
static uint32_t s_last_probe_open_ms = 0;

/* Diagnostic override (settings toggle "佩戴偵測"): when false, the contact
 * algorithm is bypassed and the watch is forced WORN unless on the charger. */
static bool s_detect_enabled = true;

/* -------------------- Helpers -------------------- */

static uint8_t on_votes_in_window(uint8_t w)
{
    uint8_t n = 0;
    while (w)
    {
        n += w & 1u;
        w >>= 1;
    }
    return n;
}

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

/* -------------------- Diagnostic uplink --------------------
 * Cable-less units have no serial console; these records (forwarded to the
 * phone as a daily CSV) are the only window into nightly wear-detect
 * internals for per-unit threshold analysis. ~1 record/min + transitions:
 * negligible BLE / cross-core load. */

static uint16_t diag_clamp_u16(float v)
{
    if (v < 0.0f) return 0;
    if (v > 65535.0f) return 65535;
    return (uint16_t)v;
}

static void diag_emit(uint8_t evt, float dc, float pi, float pi_range,
                      float imu_var)
{
    if (!watch_sys_sync.notify_wear_diag)
        return;

    watch_sys_wear_diag_t rec;
    rec.ts = (uint32_t)time(RT_NULL);
    rec.evt = evt;
    rec.status = (ctx.status == WEAR_STATUS_WEARING) ? 1 : 0;
    rec.dc_q4 = diag_clamp_u16(dc * 0.25f);
    rec.pi_e6 = diag_clamp_u16(pi * 1e6f);
    rec.pi_range_e6 = diag_clamp_u16(pi_range * 1e6f);
    rec.imu_var_e4 = diag_clamp_u16(imu_var * 1e4f);
    /* The baseline every ON/OFF decision is actually measured against. Same
       /4 scaling as dc_q4 so a reader can compare them without a conversion. */
    rec.base_q4 = diag_clamp_u16(ctx.worn_dc_base * 0.25f);
    watch_sys_sync.notify_wear_diag(&rec);
}

/* Emit with the last metrics computed by evaluate_once (for events that fire
 * where dc/pi are out of scope, e.g. state transitions in try_evaluate). */
static void diag_emit_last(uint8_t evt)
{
    diag_emit(evt, ctx.last_dc, ctx.last_pi, ctx.last_pi_range,
              ctx.last_imu_var);
}

static void set_status(wear_status_t new_status)
{
    if (ctx.status == new_status)
        return;

    ctx.status = new_status;
    /* A streak only means something within one stretch of a single state. */
    ctx.flat_streak = 0;
    ctx.alive_ring = 0;
    ctx.alive_ring_n = 0;

    if (new_status == WEAR_STATUS_WEARING)
    {
        s_probe_active = false; /* probe confirmed a wrist; hand PPG to hr_service/bg_hr */
        LOG_I("Wear detected: ON WRIST");
        notify_wear_status(true);
        diag_emit_last(WEAR_DIAG_EVT_ON);
    }
    else
    {
        LOG_I("Wear detected: OFF WRIST");
        ctx.last_off_verdict_ms = rt_tick_get_millisecond();
        /* The PPG may keep running past this verdict (power-down is vetoed
         * mid-burst), and when it does cycle it warms up again. Either way the
         * next PPG_SETTLE_MS of readings must not be trusted for re-entry — the
         * settle gate was only ever armed on the probe paths, which left this
         * transition unguarded. */
        ctx.ppg_settling = true;
        ctx.ppg_restart_ms = ctx.last_off_verdict_ms;
        notify_wear_status(false);
        diag_emit_last(WEAR_DIAG_EVT_OFF);
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

    /* Fallback re-probe: a wrongly-voted OFF on a STILL wrist (sleep dip /
     * loose strap) is otherwise a deadlock -- warm re-entry needs PPG data
     * but PPG is powered down, and a resting wrist never crosses the motion
     * threshold. Without this, only the sleep/wake path's forced power-up
     * ever recovers. Blocked while plugged: a charging watch is off-wrist by
     * contract, don't waste LED current probing the cradle. */
    bool fallback_probe =
        !s_probe_active &&
        !battery_get_charge_state()->is_plugged &&
        (rt_tick_get_millisecond() - s_last_probe_open_ms >= PROBE_FALLBACK_MS);

    if (imu_var >= IMU_RETRIGGER_THD || fallback_probe)
    {
        /* Motion while off-wrist + PPG asleep: power PPG up and (re)start the
         * probe window, so the next few minutes of PPG can confirm a real wrist
         * even if the user then stays still. We never vote ON from IMU alone --
         * only a live PPG pulse (in evaluate_once) confirms wear. */
        if (!s_probe_active)
        {
            hr_set_power(1);
            /* Ignore the first few seconds after powering PPG up: the signal is
             * unstable and PI spikes during warm-up (this fakes a pulse on a
             * table). Arm the settle gate so evaluate_once drops those reads. */
            ctx.ppg_settling = true;
            ctx.ppg_restart_ms = rt_tick_get_millisecond();
            s_last_probe_open_ms = rt_tick_get_millisecond();
            LOG_I("Wear: %s (var=%.4f) -> open PPG probe window (settling %ums)",
                  fallback_probe ? "fallback re-probe" : "motion",
                  imu_var, PPG_SETTLE_MS);
            ctx.last_imu_var = imu_var;
            diag_emit_last(WEAR_DIAG_EVT_PROBE_OPEN);
        }
        s_probe_active = true;
        s_probe_until_ms = rt_tick_get_millisecond() +
                           ((imu_var >= IMU_RETRIGGER_THD) ? PROBE_WINDOW_MS
                                                           : PROBE_FALLBACK_WINDOW_MS);
    }
    else
    {
        LOG_D("Eval: off-wrist, IMU_var=%.5f (no motion)", imu_var);
    }

    return 0; /* PPG pulse confirms wear, not IMU */
}

/**
 * @brief Evaluate all indicators and determine wear/not-wear vote.
 * @return  1 = vote ON,  -1 = vote OFF,  0 = uncertain
 */
static int evaluate_once(uint32_t now)
{
#if (CUSTOMER_BOARD_VER == BOARD_VER_29)
    /* --- Board v29: charging always means OFF WRIST --- */
    if (battery_get_charge_state()->is_charging)
    {
        LOG_I("Eval: charging (board v29) -> force OFF");
        return -1;
    }
#endif

    /* Charging while OFF ends the wear session: the baseline must not
     * survive onto the charger (a base poisoned by a band-coincident surface
     * dies here — this implements the "sessions are bounded by charging"
     * contract in the file header). Runs before the stale gate so it works
     * with PPG powered down on the cradle. */
    if (ctx.status == WEAR_STATUS_NOT_WEARING && ctx.worn_dc_base > 0.0f &&
        battery_get_charge_state()->is_plugged)
    {
        ctx.worn_dc_base = 0.0f;
        LOG_I("Wear: charger while off-wrist -> session baseline cleared");
    }

    /* --- Check PPG freshness first --- */
    if (is_ppg_stale(now))
    {
        /* Off-wrist + PPG asleep: motion opens a PPG probe window (below). If a
         * probe window already expired without confirming a wrist (e.g. PPG
         * never produced data), power PPG back down here too. */
        if (ctx.status == WEAR_STATUS_NOT_WEARING)
        {
            if (s_probe_active && now >= s_probe_until_ms)
            {
                hr_set_power(0);
                s_probe_active = false;
                ctx.on_vote_window = 0;
                LOG_I("Wear: probe expired (no PPG data) -> close PPG, wait for motion");
                diag_emit_last(WEAR_DIAG_EVT_PROBE_EXPIRE);
                return 0;
            }
            return evaluate_imu_only();
        }

        /* Currently ON but PPG went stale: give it a grace period (a normal
         * gap self-heals within PPG_STALE_MS..PPG_STALE_RESTART_MS as new
         * samples keep landing, which resets on_stale_since_ms in
         * wear_detect_feed_ppg). If it stays stale past the grace period the
         * sensor has likely wedged -- nothing else in this file ever calls
         * hr_set_power(1) while WEARING, so without this the watch would
         * stay dark until the next sleep/wake cycle re-initializes the
         * state machine and stumbles into evaluate_imu_only(). */
        if (ctx.on_stale_since_ms == 0)
        {
            ctx.on_stale_since_ms = now;
        }
        else if (now - ctx.on_stale_since_ms >= PPG_STALE_RESTART_MS)
        {
            LOG_W("Wear: PPG stale %ums while worn -> power-cycle sensor",
                  now - ctx.on_stale_since_ms);
            hr_set_power(0);
            hr_set_power(1);
            ctx.ppg_settling = true;
            ctx.ppg_restart_ms = now;
            ctx.on_stale_since_ms = 0;
        }
        return 0;
    }

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
        ctx.on_vote_window = 0;
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

    /* Keep the latest metrics for out-of-scope diagnostic events, and emit a
     * rate-limited snapshot so the nightly CSV shows the analog levels even
     * when no event fires (the whole point for cable-less units). */
    ctx.last_dc = dc_mean;
    ctx.last_pi = pi;
    ctx.last_pi_range = pi_range;
    if (now - ctx.last_diag_sample_ms >= 60000)
    {
        ctx.last_diag_sample_ms = now;
        uint16_t imu_len = (ctx.imu_count < IMU_WINDOW_SIZE)
                               ? ctx.imu_count : IMU_WINDOW_SIZE;
        ctx.last_imu_var = (imu_len > 0)
                               ? compute_imu_variance(ctx.acce_mag, imu_len)
                               : 0.0f;
        diag_emit(WEAR_DIAG_EVT_SAMPLE, dc_mean, pi, pi_range,
                  ctx.last_imu_var);
    }

    /* === DC-session-led decision (see file header). Pulse is consulted only
     * for cold entry; everything else is contact evidence vs the learned
     * per-session baseline. === */
    bool plugged = battery_get_charge_state()->is_plugged;

    if (ctx.status == WEAR_STATUS_WEARING)
    {
        /* OFF threshold: relative collapse below the worn baseline, never
         * below the absolute floor (protects against a stale/低 baseline). */
        float off_thr = (float)DC_ABS_FLOOR;
        if (ctx.worn_dc_base > 0.0f &&
            ctx.worn_dc_base * OFF_THR_FACTOR > off_thr)
        {
            off_thr = ctx.worn_dc_base * OFF_THR_FACTOR;
        }

        if (dc_mean < off_thr)
        {
            /* Sub-threshold: vote OFF. Motion during the streak marks a
             * take-off signature -> try_evaluate uses the fast hysteresis. */
            uint16_t imu_len = (ctx.imu_count < IMU_WINDOW_SIZE)
                                   ? ctx.imu_count : IMU_WINDOW_SIZE;
            float imu_var = (imu_len > 0)
                                ? compute_imu_variance(ctx.acce_mag, imu_len) : 0.0f;
            if (imu_var >= IMU_VARIANCE_THD)
                ctx.off_streak_motion = true;
            ctx.last_imu_var = imu_var;
            LOG_I("Eval: DC=%.0f (< %.0f) -> contact lost -> OFF vote (%s)",
                  dc_mean, off_thr,
                  ctx.off_streak_motion ? "moving" : "still");
            return -1;
        }

        /* DC says contact. Second opinion: has the perfusion signal been
         * DEAD FLAT for five straight minutes? A wrist is never that quiet --
         * even asleep the pulse and posture move it. A table is. Checked once
         * a minute so the run length means minutes, not evaluations. */
        if (now - ctx.last_flat_check_ms >= FLAT_CHECK_MS)
        {
            ctx.last_flat_check_ms = now;
            if (pi >= PI_SATURATED || pi_range >= PI_SATURATED)
            {
                /* HOLD, do not reset. A saturated read is an AGC gain step, and
                 * on a desk those come often enough that resetting here meant the
                 * exit streak never completed (2026-09-02: six flat minutes wiped
                 * by one saturated sample at 10:04:52). Two nights of real wear
                 * (n=460, 25% saturated) never reached a streak of 4 under this
                 * policy either, so holding costs nothing on the wrist. */
            }
            else if (pi_range > 0.0f && pi_range < PI_RANGE_FLAT_THD)
            {
                if (ctx.flat_streak < 0xFFu) ctx.flat_streak++;
            }
            else
            {
                ctx.flat_streak = 0;
            }

            if (ctx.flat_streak >= FLAT_CHECKS_TO_OFF)
            {
                LOG_W("Eval: PI flat %.6f < %.6f for %u min -> off-wrist -> OFF vote",
                      pi_range, PI_RANGE_FLAT_THD, (unsigned)ctx.flat_streak);
                ctx.flat_streak = 0;
                return -1;
            }
        }

        /* Contact holds: stay ON regardless of PI (sleep perfusion may be
         * flat). Adapt the baseline slowly, but ONLY while this ON stretch
         * is pulse-anchored — a warm re-entry might be a band-coincident
         * surface, and learning from it would drag the band onto the
         * surface (review finding: morning table walked base 43.5k→39.1k).
         * Frozen while charging (charger couples noise into the ADC). */
        if (!plugged && ctx.on_anchor_pulse && ctx.worn_dc_base > 0.0f &&
            dc_mean >= ctx.worn_dc_base * WORN_BAND_LO &&
            dc_mean <= ctx.worn_dc_base * WORN_BAND_HI)
        {
            float delta = dc_mean - ctx.worn_dc_base;
            ctx.worn_dc_base += delta / (float)(1 << ((delta > 0.0f)
                                                          ? BASE_EMA_UP_SHIFT
                                                          : BASE_EMA_SHIFT));
        }
        LOG_I("Eval: DC=%.0f (base=%.0f) -> worn, contact held -> ON",
              dc_mean, ctx.worn_dc_base);
        return 1;
    }

    /* Currently OFF. The PPG running now was opened by a motion probe window;
     * if that window expired without confirming a wrist, power PPG back down
     * and wait for the next motion (saves LED current when nothing is worn). */
    if (s_probe_active && now >= s_probe_until_ms)
    {
        hr_set_power(0);
        s_probe_active = false;
        ctx.ppg_count = 0;
        ctx.ppg_idx = 0;
        ctx.on_vote_window = 0;
        LOG_I("Wear: probe window expired, no wrist -> close PPG, wait for motion");
        diag_emit_last(WEAR_DIAG_EVT_PROBE_EXPIRE);
        return 0;
    }

    /* Track how long the perfusion signal has been MOVING. A saturated reading
     * IS genuine high variability here, so unlike the exit test it extends the
     * streak instead of resetting it. */
    ctx.alive_ring = (uint16_t)((ctx.alive_ring << 1) |
                                (pi_range >= PI_RANGE_ALIVE_THD ? 1u : 0u));
    ctx.alive_ring &= (uint16_t)((1u << ALIVE_WINDOW_EVALS) - 1u);
    if (ctx.alive_ring_n < ALIVE_WINDOW_EVALS) ctx.alive_ring_n++;
    uint8_t alive_count = 0;
    for (uint16_t b = ctx.alive_ring; b; b &= (uint16_t)(b - 1u)) alive_count++;
    bool warm_cooldown = (now - ctx.last_off_verdict_ms) < WARM_REENTRY_COOLDOWN_MS;

    /* WARM re-entry: this session already proved a wrist once (pulse-seeded
     * baseline), so DC back inside the worn band is evidence the wrist has
     * returned. Blocked while charging so a cradle is never warm-recovered
     * into. The DC_ABS_FLOOR clamp keeps the entry set a subset of the hold
     * set (entry below the hold threshold would oscillate ON/OFF).
     *
     * 2026-08-30: a minimum pulse is now demanded here too. This branch used
     * to take DC alone, and DC alone cannot tell a desk from a wrist — measured
     * medians are 42904 vs 47356, 11% apart, well inside the +-12/15% band. So
     * a watch put down on a table kept being re-entered as "worn": on the bench
     * unit the detector flipped ON/OFF 25-52 times a day (2026-08-29: 52 ON,
     * 50 OFF), every ON a warm re-entry with a median PI of 0.000387 — far
     * below any pulse. Meanwhile the genuinely-worn night showed 0.0% false
     * OFF and not one flip, so the failure this guards against was the
     * false-POSITIVE, not the false-negative we had assumed.
     *
     * Replayed per-sample over those two windows, the two changes are only
     * useful together — neither alone moves anything:
     *   shipped (PI 0.0010, no pulse here)   worn 100%   desk 93%   sep  7%
     *   threshold 0.0004 only                worn 100%   desk 95%   sep  5%
     *   pulse here only (still 0.0010)       worn  36%   desk 15%   sep 21%
     *   both                                 worn  98%   desk 19%   sep 79%
     * The threshold alone does nothing because this branch bypassed the pulse
     * test entirely; the pulse alone destroys real wear because 0.0010 is
     * above a sleeping wrist's perfusion.
     *
     * ⚠ Caveat on that replay: wear_diag emits SAMPLE at most once a minute
     * while evaluation runs every EVAL_PERIOD_MS, so this is a per-sample
     * comparison of the decision function, NOT a full state-machine replay —
     * the vote windows and hysteresis are not reproduced. */
    if (!plugged && ctx.worn_dc_base > 0.0f &&
        dc_mean >= (float)DC_ABS_FLOOR &&
        pi >= PI_THD_TO_ON &&
        !warm_cooldown &&
        ctx.alive_ring_n >= ALIVE_WINDOW_EVALS &&
        alive_count >= ALIVE_EVALS_TO_ON &&
        dc_mean >= ctx.worn_dc_base * WORN_BAND_LO &&
        dc_mean <= ctx.worn_dc_base * WORN_BAND_HI)
    {
        LOG_I("Eval: DC=%.0f in worn band [%.0f..%.0f], pi_range=%.6f alive"
              " -> warm re-entry -> ON",
              dc_mean, ctx.worn_dc_base * WORN_BAND_LO,
              ctx.worn_dc_base * WORN_BAND_HI, pi_range);
        ctx.last_vote_pulse = false;
        return 1;
    }

    /* COLD entry: no session evidence (or DC outside the learned band) —
     * require a live pulse so a table is never confirmed as a wrist. */
    if (dc_mean >= (float)DC_ABS_FLOOR &&
        pi >= PI_THD_TO_ON && pi_range <= PI_RANGE_MAX &&
        (ctx.pi_hist_count < PI_HISTORY_LEN || pi_range >= PI_RANGE_THD))
    {
        LOG_I("Eval: DC=%.0f, AC_pp=%.0f, PI=%.5f, range=%.5f -> live wrist -> ON",
              dc_mean, ac_pp, pi, pi_range);
        ctx.last_vote_pulse = true;
        return 1;
    }
    if (pi >= PI_THD_TO_ON && pi_range > PI_RANGE_MAX)
    {
        LOG_I("Eval: DC=%.0f, PI=%.5f, range=%.5f (too wild -> motion artefact, not a pulse) -> hold",
              dc_mean, pi, pi_range);
        return 0;
    }

    LOG_I("Eval: DC=%.0f, PI=%.5f, range=%.5f (no contact evidence yet) -> hold",
          dc_mean, pi, pi_range);
    return 0; /* unconfirmed -> wait */
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

    /* Diagnostic override: wear detection disabled by the user. Force WORN
     * unless on the charger, so HR/sleep run regardless of the contact
     * algorithm. set_status() no-ops when unchanged and drives the UI
     * indicator + bg_hr gating + sleep on transition. */
    if (!s_detect_enabled)
    {
        /* Latch "plugged": is_plugged mirrors the charge IC's charging bit
         * (bloc_battery read_charge_status), which cycles while topping off.
         * Raw use here means a status flip + LED power flip per cycle. */
        static uint32_t last_plugged_ms = 0;
        if (battery_get_charge_state()->is_plugged)
            last_plugged_ms = now;
        bool on_charger = (last_plugged_ms != 0) &&
                          (now - last_plugged_ms < PLUGGED_HOLD_MS);
        set_status(on_charger ? WEAR_STATUS_NOT_WEARING : WEAR_STATUS_WEARING);
        return;
    }

    /* Need minimum data before evaluating (PPG or IMU) */
    if (ctx.ppg_count < 20 && ctx.imu_count < 20)
        return;

    /* [DC-led] Broadcast the initial wear state once. Boot now defaults to
     * NOT_WEARING and may legitimately stay there (e.g. resting on a table),
     * which is no state *change* -> set_status() never fires -> the UI's
     * "not worn" indicator would never be told at boot. Push it once here. */
    static bool initial_notified = false;
    if (!initial_notified)
    {
        initial_notified = true;
        notify_wear_status(ctx.status == WEAR_STATUS_WEARING);
    }

#ifdef PPG_RACE_DEBUG
    uint32_t eval_t0 = rt_tick_get_millisecond();
    LOG_I("[WE-EVAL] enter ts=%u", (unsigned)eval_t0);
#endif
    int vote = evaluate_once(now);
#ifdef PPG_RACE_DEBUG
    {
        uint32_t eval_dt = rt_tick_get_millisecond() - eval_t0;
        if (eval_dt > 20)
        {
            LOG_I("[WE-EVAL] exit took=%ums vote=%d (slow!)",
                  (unsigned)eval_dt, vote);
        }
    }
#endif

    /* Slide the ON-vote window every evaluation (bit set = ON vote). ON votes
     * age out after 8 evals instead of being erased by a single hold, so a
     * marginal pulse that crosses the PI bar intermittently can still
     * accumulate HYSTERESIS_ON votes (see comment at HYSTERESIS_ON). */
    ctx.on_vote_window = (uint8_t)((ctx.on_vote_window << 1) | ((vote > 0) ? 1u : 0u));

    if (vote > 0)
    {
        /* Vote ON */
        ctx.off_counter = 0;
        ctx.off_streak_motion = false;

        if (ctx.status != WEAR_STATUS_WEARING &&
            on_votes_in_window(ctx.on_vote_window) >= HYSTERESIS_ON)
        {
            if (ctx.last_vote_pulse)
            {
                /* Pulse is the highest-trust evidence: ALWAYS (re)seed the
                 * baseline from it. This corrects a base poisoned by an
                 * artefact-inflated earlier seed (review: put-on artefact
                 * seeded 56.9k and locked the real 43.4k wrist out), and
                 * keeps entry ⊆ hold monotonic (off_thr becomes 0.8×this
                 * DC, so a fresh pulse confirm can never oscillate). */
                ctx.worn_dc_base = ctx.last_dc;
                LOG_I("Wear: session baseline (re)seeded at DC=%.0f",
                      ctx.worn_dc_base);
            }
            /* Warm re-entry keeps the existing pulse-seeded baseline; the
             * anchor flag gates EMA learning to pulse-proven stretches. */
            ctx.on_anchor_pulse = ctx.last_vote_pulse;
            /* Reset PI history on state change so fresh PPG data will be
             * evaluated after sensor restarts. */
            ctx.pi_hist_count = 0;
            ctx.pi_hist_idx = 0;
            set_status(WEAR_STATUS_WEARING);
        }
    }
    else if (vote < 0)
    {
        /* Vote OFF: contact lost invalidates any accumulated ON votes.
         * Dual-speed hysteresis: a take-off (motion in the streak) confirms
         * fast; a still dip (loose strap during sleep) must persist ~30 s —
         * and even a wrong OFF is recovered in seconds by warm re-entry. */
        ctx.on_vote_window = 0;
        ctx.off_counter++;
        if (ctx.off_counter > OFF_EVALS_STILL)
            ctx.off_counter = OFF_EVALS_STILL;

        int8_t needed = ctx.off_streak_motion ? OFF_EVALS_MOTION
                                              : OFF_EVALS_STILL;
        if (ctx.status != WEAR_STATUS_NOT_WEARING && ctx.off_counter >= needed)
        {
            set_status(WEAR_STATUS_NOT_WEARING);
            ctx.off_streak_motion = false;
        }
    }
    else
    {
        /* Uncertain: decay OFF counter; ON votes age out of the window */
        if (ctx.off_counter > 0) ctx.off_counter--;
        if (ctx.off_counter == 0)
            ctx.off_streak_motion = false;
    }
}

/* -------------------- Public API -------------------- */

void wear_detect_init(void)
{
    memset(&ctx, 0, sizeof(ctx));
    ctx.status = WEAR_STATUS_NOT_WEARING; /* start OFF (cold): the first wear of a session must be pulse-confirmed, so booting/resting on a table is never latched as worn */
    ctx.last_eval_ms = rt_tick_get_millisecond();
    ctx.initialized = true;
    LOG_I("Wear detection initialized (DC-session-led v2, eval every %u ms)", EVAL_PERIOD_MS);
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
    ctx.on_stale_since_ms = 0;

    /* Store PPG sample into ring buffer */
    ctx.ppg_buf[ctx.ppg_idx] = ppg_raw;
    ctx.ppg_idx = (ctx.ppg_idx + 1) % PPG_WINDOW_SIZE;
    if (ctx.ppg_count < PPG_WINDOW_SIZE)
        ctx.ppg_count++;

    /* Try periodic evaluation */
    try_evaluate();
}

bool wear_detect_is_wearing(void)
{
    return ctx.status == WEAR_STATUS_WEARING;
}

void wear_detect_set_enabled(bool enabled)
{
    if (s_detect_enabled == enabled)
        return;
    s_detect_enabled = enabled;
    LOG_I("Wear detection %s", enabled ? "ENABLED (normal)"
                                       : "DISABLED (force worn unless charging)");
    /* Votes/counters frozen during bypass are stale evidence — clear them
     * so re-enabling starts from a clean slate, and re-evaluate promptly on
     * the next feed instead of waiting out the throttle. */
    ctx.on_vote_window = 0;
    ctx.off_counter = 0;
    ctx.off_streak_motion = false;
    ctx.last_eval_ms = 0;
}
