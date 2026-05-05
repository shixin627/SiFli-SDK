/**
 ******************************************************************************
 * @file   alarm_client.h
 * @author Skaiwalk software development team
 * @brief  Public API for the alarm data-service client.
 ******************************************************************************
 */
#ifndef __ALARM_CLIENT_H__
#define __ALARM_CLIENT_H__

#include <stdint.h>
#include <stdbool.h>
#include "watch_global_data.h"     /* T_ALARM */
#include "alarm_manager_service.h" /* alarm_contxt_t */

#ifdef __cplusplus
extern "C"
{
#endif

    /** Subscribe to the "alarmmgr" data service. Idempotent. */
    int subscribe_alarm_client(void);

    /** Unsubscribe and release the client handle. */
    void unsubscribe_alarm_client(void);

    /**
     * Replace the watch's alarm list with @p alarms (full sync from the phone).
     * Lazy-subscribes if needed. Existing alarms are cleared first; @p num is
     * capped by the alarm manager's own limit.
     */
    void apply_alarms_from_ble(const T_ALARM *alarms, uint8_t num);

    /**
     * Mirror a watch-side mutation into SkaiWatchSys.alarms[] so the
     * BLE-reply path (commu_send_alarm_settings on phone pull) and the
     * boot-restore path (WatchPrefs.read_alarms) stay consistent with the
     * alarm_manager_service state. Persists to flash via WatchPrefs.write_alarms.
     *   - op = 0 : add at end
     *   - op = 1 : replace slot @p idx
     *   - op = 2 : delete slot @p idx (shifts remaining alarms down)
     */
    void watch_alarm_local_sync(int op, int32_t idx,
                                const alarm_contxt_t *ctx);

    /**
     * Returns the index of the currently-ringing alarm, or -1 if none.
     * Polled by app_alarm.c so it can flip to the ringing UI when an alarm
     * fires while the app is open (foreground).
     */
    int32_t bloc_alarm_get_ringing_idx(void);

    /**
     * Stop the currently ringing alarm.
     * @param snooze_5min  true → schedule a one-shot alarm 5 minutes from now.
     */
    void bloc_alarm_stop_ringing(bool snooze_5min);

#ifdef __cplusplus
}
#endif

#endif /* __ALARM_CLIENT_H__ */
