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
#include "watch_global_data.h" /* T_ALARM */

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
     * Snapshot the current alarm_manager_service list and push it to the phone
     * (KEY_RETURN_ALARM_SETTINGS). Call after any watch-side mutation
     * (toggle / add / edit / delete) so the phone UI stays in sync without
     * having to poll. Safe to call back-to-back — concurrent invocations
     * coalesce into a single follow-up pass.
     */
    void bloc_alarm_push_to_phone(void);

#ifdef __cplusplus
}
#endif

#endif /* __ALARM_CLIENT_H__ */
