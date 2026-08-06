#ifndef __HR_AUTOCORR_H__
#define __HR_AUTOCORR_H__

#include <stdint.h>

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

/** Append raw channel-0 PPG samples (17-bit, straight from the FIFO hook).
 *  Accelerometer is recorded as zero for these, which disables motion
 *  compensation for the window — use hr_autocorr_feed_frame where the aligned
 *  accel is available. */
void hr_autocorr_feed(uint8_t n, const uint32_t *raw);

/**
 * Append ONE frame of PPG together with the accelerometer sample the vendor
 * driver has already time-aligned to it (STGh30xFrameInfo::pusGsensordata).
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
void hr_autocorr_feed_frame(uint32_t ppg, int16_t ax, int16_t ay, int16_t az);

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
