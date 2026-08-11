#ifndef __HR_AUTOCORR_H__
#define __HR_AUTOCORR_H__

#include <stdint.h>
#include <stdbool.h>    /* the watch build gets this transitively from RT-Thread;
                           a host harness including this header alone does not */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * In-tree resting heart-rate estimator: normalised autocorrelation of the raw
 * green PPG. Pure C, integer only, no RTOS or libm dependency.
 *
 * WHY THIS EXISTS. The Goodix HBA library picks the main peak of a spectrum
 * (goodix_hba.h documents hba_snr as "频谱主峰能量处于总能量" and exposes a
 * frequency-domain Wiener filter). At rest a PPG's dicrotic notch can make the
 * 2nd harmonic outrank the fundamental, and the picker then reports exactly
 * double. Measured against a reference watch on 2026-08-02/03: five clean hours
 * at 52-67 bpm, then a 34-minute plateau at 107-124 while the reference never
 * exceeded 84, with the wrist accelerometer reading zero throughout. Every
 * external explanation was eliminated (lost frames, decimation divider,
 * timebase, cold start, wear loss, body motion) and all six vendor
 * quality/scene fields read 0 all night, so there is no in-band way to know
 * when to distrust that library's answer.
 *
 * Autocorrelation is not chosen for elegance: it removes the failure mode
 * structurally. The catch is that a periodic signal correlates ~equally at T,
 * 2T, 3T..., so "strongest peak" is decided by numerical noise — that is what
 * made three earlier drafts of this report a genuine 110 bpm as 37. The
 * fundamental is BY DEFINITION the shortest period, hence the tie-break below.
 * A half-period only ties when consecutive half-cycles are genuinely alike; a
 * dicrotic notch makes them differ, so the harmonic falls outside the tolerance
 * and is rejected without any special-casing.
 *
 * Scope: resting/sleep only. Motion-artefact rejection is the hard part of PPG
 * and the vendor library is better at it — this does not attempt it, and the
 * accelerometer says the failing case has no motion to reject anyway.
 *
 * The reference implementation and its 24 validation cases live in
 * hr_autocorr_test.py; the C must agree with it case for case.
 */

#define HR_AUTOCORR_FS       25   /* Hz — GH30X_FUNCTION_HR sample rate       */
#define HR_AUTOCORR_WIN      256  /* 10.24 s — needs >=2 cycles at 30 bpm     */
#define HR_AUTOCORR_LAG_MIN  7    /* 60*25/7  = 214 bpm ceiling               */
#define HR_AUTOCORR_LAG_MAX  50   /* 60*25/50 =  30 bpm floor                 */

/** Drop the window. Call whenever the sensor restarts — stale samples from
 *  before a power cycle would be correlated against fresh ones. */
void hr_autocorr_reset(void);

/** Drop EVERYTHING, tracker baseline included.
 *
 * hr_autocorr_reset() deliberately keeps the baseline — it runs at every burst
 * boundary and a heart rate does not reset when the LED goes off. This is for
 * the cases where the prior genuinely stops applying: the watch coming off a
 * wrist, or a test that needs each window judged on its own. */
void     hr_autocorr_forget(void);

/** Append raw channel-0 PPG samples (17-bit, straight from the FIFO hook).
 *  Accelerometer is recorded as zero for these, which disables motion
 *  compensation for the window — use hr_autocorr_feed_frame where the aligned
 *  accel is available. */
void hr_autocorr_feed(uint8_t n, const uint32_t *raw);

/**
 * Stage the accelerometer batch that belongs to the PPG batch about to arrive.
 *
 * Call ONE begin per batch, then one push per sample, from
 * gsensor_drv_get_fifo_data() — the vendor's own accel callback, which returns
 * exactly as many samples as this PPG batch has frames.
 *
 * THIS IS NOT WHERE THE ACCEL USED TO COME FROM, and the difference is the
 * whole reason motion compensation did nothing for a night. The first version
 * read STGh30xFrameInfo::pusGsensordata, which points at
 * g_psGh30xFrameGsensorData[3] in gh30x_example_process.c — a .bss array that
 * NOTHING in the entire SDK ever writes (confirmed in the LCPU link map: the
 * three frame-info structs refer to it, no object writes it, and it is marked
 * Zero). So the reference signal was three permanent zeros, every axis read
 * flat, and the NLMS stage returned immediately without ever adapting once.
 *
 * The lesson is cheap to state and was expensive to learn: a field being
 * present in a vendor struct is not evidence that the vendor fills it.
 */
void hr_autocorr_stage_begin(void);
void hr_autocorr_stage_push(int16_t ax, int16_t ay, int16_t az);

/**
 * Append ONE frame of PPG. The accelerometer sample is taken from the batch
 * staged above, in order, so each PPG frame pairs with the accel sample the
 * vendor driver considers contemporaneous.
 *
 * Motion is the failure this exists for. On 2026-08-06 the watch read correctly
 * all night — 53-72 bpm against a reference watch's 48-90 — and then, once the
 * wearer got up, reported 218, 195, 171 and 165 bpm. Converted to frequency
 * those are 3.63, 3.25, 2.85 and 2.75 Hz: hand-movement rates. Wrist motion
 * couples into the optical path, and an estimator that simply finds the
 * strongest period cannot tell a periodic wrist from a periodic heart.
 *
 * The accelerometer can: the artefact is a filtered copy of it. See the NLMS
 * stage in hr_autocorr.c.
 */
void hr_autocorr_feed_frame(uint32_t ppg);

/**
 * Wrist activity over the current window: mean |first difference| per sample,
 * summed over the three axes. Two jobs, one number.
 *
 * It gates the motion compensation (see NLMS_GATE_ACT), and it is the liveness
 * proof for the reference itself. A dead accelerometer feed and a motionless
 * wrist both present a flat reference, so the filter cannot tell them apart and
 * stays silent either way — which is exactly how a feed of three constant zeros
 * survived a full night of wrist data unnoticed. A value that is visibly small
 * at 3 a.m. and visibly large while walking makes that failure loud.
 */
uint32_t hr_autocorr_accel_act(void);

/**
 * True when any sample in the current window came from an accelerometer batch
 * byte-identical to its predecessor — the signature of a stopped IMU stream.
 *
 * A stopped stream is more dangerous than a dead one. The vendor callback hands
 * back the newest N entries of a ring nobody is writing, so every batch is the
 * same frozen copy of the wrist's last real movement: large deltas, genuine
 * structure, straight through the activity gate. The compensation would then
 * subtract an arm swing that stopped happening hours ago. Motion compensation
 * refuses to run while this is set.
 */
bool hr_autocorr_accel_stale(void);

/**
 * Estimate from the current window.
 *
 * @param conf_out  0..100, the interpolated correlation height. Optional.
 * @return bpm, or 0 when the window is not yet full or no peak clears the
 *         threshold. Zero means "no answer" and must NOT be treated as a
 *         reading — refusing is the whole point of having a confidence at all,
 *         since the vendor library never refuses.
 */
uint8_t hr_autocorr_estimate(uint8_t *conf_out);

/** Samples currently buffered (0..HR_AUTOCORR_WIN). Diagnostics. */
uint16_t hr_autocorr_fill(void);

/**
 * Copy the detrended window the LAST hr_autocorr_estimate() ran on, rescaled to
 * int8, oldest sample first. Returns the count written (0 if no estimate has run).
 *
 * This exists because the synthetic suite cannot reproduce the field failures.
 * On 2026-08-05 the estimator left four isolated outliers (171/144/101/38 bpm),
 * all at the extremes of the lag range — yet sweeping the tie tolerance and the
 * accept threshold over nine combinations left all 81 synthetic cases passing.
 * Uniform noise plus a sine wander is evidently not what this sensor produces,
 * so the parameters cannot be designed against imagination: the failing window
 * itself has to come back off the wrist and into the offline suite.
 *
 * int8 rather than the int16 the estimator uses: three bits of amplitude buys a
 * single BLE frame instead of a reassembled pair, and period estimation does not
 * need them — the algorithm already normalises amplitude away.
 */
uint16_t hr_autocorr_last_window(int8_t *out, uint16_t max);

#ifdef __cplusplus
}
#endif

#endif /* __HR_AUTOCORR_H__ */
