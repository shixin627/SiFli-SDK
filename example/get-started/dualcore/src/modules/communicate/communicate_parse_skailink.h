/**
 * @file   communicate_parse_skailink.h
 * @brief  ADR-0008 § E7: SKAI_LINK (0x20) device-sync command group.
 *
 * Phone (primary) → watch: the account device list + per-device status + each
 * device's actions, with incremental deltas. Watch → phone: the active-target
 * selection. Subkeys live in their own space under SKAI_LINK_COMMAND_ID (0x20),
 * which the firmware reserved but did not previously handle.
 *
 * Cross-device protocol — keep in lockstep with the dart side:
 *   SkaiLink/lib/shared/watch/communicate/communicate_protocol.dart
 *   (skaiLinkCommandId 0x20 + matching subkeys).
 */
#ifndef COMMUNICATE_PARSE_SKAILINK_H
#define COMMUNICATE_PARSE_SKAILINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        KEY_DEVICE_LIST_BATCH = 0x01,    /* phone→watch: full list [{id,name,platform,status}] */
        KEY_DEVICE_STATUS_DELTA = 0x02,  /* phone→watch: {id,status} one device's status */
        KEY_DEVICE_ACTIONS_BATCH = 0x03, /* phone→watch: {device_id,items:[...]} per-device actions */
        KEY_DEVICE_REMOVED = 0x04,       /* phone→watch: {id} device removed (logout) */
        KEY_ACTIVE_SELECT = 0x05,        /* watch→phone (UPLINK): {device_id} active target */
    } SKAI_LINK_KEY;

    /* Dispatched from communicate_parse.c for cmd_id == SKAI_LINK_COMMAND_ID.
       pValue/length are the reassembled L2 payload (UTF-8 JSON for 0x01-0x04). */
    void resolve_skailink_command(uint8_t key, uint8_t *pValue, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATE_PARSE_SKAILINK_H */
