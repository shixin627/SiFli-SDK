/**
 ******************************************************************************
 * @file   wear_detect.h
 * @author Skaiwalk software development team
 * @brief  Wear detection algorithm using IMU and PPG sensor data.
 *         Determines whether the watch is being worn on the user's wrist
 *         by analyzing accelerometer variance and PPG signal quality.
 ******************************************************************************
 */
#ifndef WEAR_DETECT_H
#define WEAR_DETECT_H

#include <stdbool.h>
#include <stdint.h>
#include "watch_global_data.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        WEAR_STATUS_NOT_WEARING = 0,
        WEAR_STATUS_WEARING = 1,
        WEAR_STATUS_UNCERTAIN = 2,
    } wear_status_t;

    /**
     * @brief Initialize wear detection module
     */
    void wear_detect_init(void);

    /**
     * @brief Feed IMU accelerometer data into wear detection
     * @param acce Accelerometer data (x, y, z) in m/s^2
     * @param sample_rate Current sampling rate in Hz
     */
    void wear_detect_feed_imu(Vector3 *acce, float sample_rate);

    /**
     * @brief Feed PPG raw data into wear detection
     * @param ppg_raw Raw PPG signal value (channel 0)
     * @param ppg_raw2 Raw PPG signal value (channel 1)
     */
    void wear_detect_feed_ppg(uint32_t ppg_raw, uint32_t ppg_raw2);

    /**
     * @brief Check if user is currently wearing the watch
     * @return true if wearing, false otherwise
     */
    bool wear_detect_is_wearing(void);

    /**
     * @brief Enable/disable the wear-detection algorithm (diagnostic override).
     *        When disabled, the watch is forced WORN unless it is on the
     *        charger — lets HR/sleep run regardless of the contact algorithm
     *        while per-unit thresholds are being diagnosed. Default: enabled.
     * @param enabled true = normal detection, false = force worn (unless charging)
     */
    void wear_detect_set_enabled(bool enabled);

    /**
     * @brief Tell the detector the PPG sensor was just (re)opened or changed
     *        mode (hr_service power/mode paths). Arms the warm-up settle so the
     *        driver's start-up spikes and the old/new DC discontinuity in the
     *        sample ring are never scored as a pulse. Restarts the settle on every
     *        call (each open is a full sensor re-init); only a duplicate call
     *        within 300 ms is ignored.
     */
    void wear_detect_on_ppg_restart(void);

#ifdef __cplusplus
}
#endif

#endif /* WEAR_DETECT_H */
