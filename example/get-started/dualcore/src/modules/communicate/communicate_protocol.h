/**
*********************************************************************************************************
*               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
*********************************************************************************************************
* @file         communicate_protocol.c
* @brief       斯凱沃克通信協議
* @details   communicate_protocol implementation
* @author
* @date      2014-12-29
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef __COMMUNICATE_PROTOCOL_H__
#define __COMMUNICATE_PROTOCOL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"
#include "communicate_update_image.h"

#define GLOBAL_RECEIVE_BUFFER_SIZE 244
#define GLOBAL_RESPONSE_BUFFER_SIZE 244
/**************************************************************************
 * Note:
 * big_endian in communication protocol
 * without considering Byte padding
 * use constant shift
 ***************************************************************************/

/*Mobile Command Heading */
#define MOBILE_HEADER_VOICE_RECOGNITION_RESULT (0xA1)
#define MOBILE_HEADER_NOTIFICATION (0x4E)
/*Watch Command Heading */
#define WATCH_COMMAND_HEADER (0xC0)
#define WATCH_COMMAND_HEADER_AUDIO (0xA0)        // --- Audio Command Heading
#define WATCH_COMMAND_HEADER_NOTIFICATION (0x4E) // --- Notification Command Heading [N]
#define WATCH_DATA_HEADER_ACTIVITY (0xAC)        // Activity Data Heading [Ac]
#define WATCH_DATA_HEADER_GSENSOR (0x47)         // Gsensor Data Heading [G]

#define L2_HEADER_SIZE (2)       /*L2 header length*/
#define L2_HEADER_VERSION (0x00) /*L2 header version*/
#define L2_KEY_SIZE (1)
#define L2_PAYLOAD_HEADER_SIZE (3) /*L2 payload header*/

#define L2_FIRST_VALUE_POS (L2_HEADER_SIZE + L2_PAYLOAD_HEADER_SIZE)

/* L2 fragmentation flag, packed into the unused high bits of byte[3] of the
   L2 header. The legacy length decode `((b3<<8)|b4) & 0x1FF` strips bits 1..7
   of byte[3], so an old peer transparently ignores the flag. */
#define L2_MORE_FRAGMENTS_BIT (0x80)
/* Hard cap on the reassembled L2 payload size (across all fragments of one
   logical command). Anything bigger gets dropped with an error log. */
#define L2_REASSEMBLY_MAX_BYTES (4096)
/* ATT NTF/WRT carries its own 3-byte header, so the largest L2 frame we can
   put on the wire is mtu - 3. */
#define L2_ATT_OVERHEAD (3)
#define BLE_DEFAULT_MTU (23)
/* Upper bound for one BLE TX packet, used to size the per-fragment scratch
   buffer in skaiwatch_ble_notify. BLE 5.0 caps ATT MTU at 517, so an ATT
   NTF payload is at most 514 bytes; we round up a touch for headroom. */
#define L2_TX_PKT_BUF_SIZE (520)
/* Bluetooth Core Spec (Vol 3 Part F §3.2.9): a GATT attribute value is at
   most 512 octets, so a single notify/write payload tops out at 512 even
   when the negotiated ATT_MTU is larger. Long Write/Read can exceed this
   but isn't used here. */
#define BLE_MAX_ATTR_VALUE_LEN (512)
/* The max_len declared on our BLE TX/RX characteristics. Phone-side writes
   and our own notify payloads are both bounded by this — exceeding it gets
   GATT_INVALID_ATTRIBUTE_LENGTH on the receiver. Used both in the GATT
   service declaration (main.c) and the L2 send path. Phone code keeps its
   own copy (`watchCharValueMaxLen` in dart) that must match. */
#define BLE_APP_CHAR_MAX_LEN (512)

    typedef enum
    {
        NONE = 0x00,
        IOS = 0x01,
        ANDROID = 0x02,
    } PHONE_OS_VERSION;

    /******************* Function definition **********************************/
    extern void skaiwatch_ble_set_performance(bool status);
    extern void ble_station_entry(void *parameter);
    extern void communicate_protocol_init(void);
    /* Send a payload as one logical L2 command. The function builds the L2
       header internally and splits the payload across multiple BLE writes if
       it exceeds the negotiated MTU; the caller does not need to allocate a
       buffer big enough for the whole frame. Prefer this over
       skaiwatch_ble_notify for new callers, especially when payload sizes can
       grow large (e.g. sensor batches). */
    extern bool skaiwatch_ble_send_l2(uint8_t cmd_id, uint8_t key,
                                       const uint8_t *payload, uint16_t payload_len);
    /* Legacy entry point: caller pre-builds the full L2 frame in `buf`. Kept
       wire-identical to old code on the fast path; auto-fragments only when
       the pre-built frame doesn't fit in one BLE write. */
    extern bool skaiwatch_ble_notify(uint8_t *buf, uint16_t length);
    extern bool skaiwatch_ble_audio_send(uint8_t *buf, uint16_t length);

    extern void L1_receive_data_without_crc_check(uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PROTOCOL_H__
