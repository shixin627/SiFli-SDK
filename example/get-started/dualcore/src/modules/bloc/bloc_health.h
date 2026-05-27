/**
 ******************************************************************************
 * @file   bloc_health.h
 * @author Skaiwalk software development team
 * @brief  Persist health data (sleep / heart-rate curve) to per-day JSON files
 *         under /health, so it survives BLE disconnection and can be synced to
 *         the phone later (store-and-forward, mirroring the exercise writer).
 ******************************************************************************
 */

#ifndef __BLOC_HEALTH_H__
#define __BLOC_HEALTH_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "watch_global_data.h" /* watch_sys_sleep_state_t */

/* IMPORTANT: both functions take watch_storage_api_lock() and touch flash, so
   they MUST be called off the BLE/RPC threads (KE_EVT2 4KB stack → STKOF).
   Route via a worker thread; do not call directly from communicate_* or the
   data_service indication handlers.

   File shapes consumed by the phone (delete_after_sync stays FALSE; the phone
   upserts idempotently by day, the watch ages files out by retention):
     /health/sleep_YYYYMMDD.json : {ts_utc,stage,total_min,deep_min,rem_min,
                                    light_min,waso_min,hr,rhr}   (overwritten)
     /health/hr_YYYYMMDD.json    : {samples:[{ts,bpm},...]}       (appended) */
void store_sleep_data(const watch_sys_sleep_state_t *state);
void store_hr_sample(uint32_t timestamp_utc, uint8_t bpm);

/* Thread-safe enqueue: copies the record and hands it to the health worker
   thread for persistence. SAFE to call from the BLE/RPC threads (non-blocking
   rt_mq_send; the worker does the flash + cJSON work off-thread). Use THESE,
   not the store_* functions, from communicate_* / data_service handlers. */
void health_store_sleep_async(const watch_sys_sleep_state_t *state);
void health_store_hr_async(uint32_t timestamp_utc, uint8_t bpm);

/* Delete /health files older than the retention window so flash doesn't grow
   unbounded. Takes watch_storage_api_lock(); call from a normal thread, not a
   BLE/RPC handler. */
void health_cleanup_old_files(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLOC_HEALTH_H__ */
