#ifndef __APP_INCOMING_CALL_H__
#define __APP_INCOMING_CALL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

void incoming_call_set_caller(const char *title, const char *id, uint8_t type);
void incoming_call_close_if_active(const char *id);

/* iOS ANCS integration: when the call is sourced from an ANCS notification,
 * pass the notification UID so the accept/hangup buttons can fall back to
 * ANCS perform-action when HFP is not bonded. Pass 0 to clear. */
void incoming_call_set_ancs_uid(uint32_t noti_uid);

/* Force-close the incoming call UI regardless of caller id. Used when the
 * phone signals end-of-call via a new event (e.g. iOS missed-call category). */
void incoming_call_force_close(void);

#ifdef __cplusplus
}
#endif

#endif
