/*********************************************************************************************************
 *               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_protocol.c
 * @brief    Communicate protocol implementation for Skaiwalk watch system.
 *********************************************************************************************************
 */

#include <rtthread.h>
#include "string.h"
#include "communicate_protocol.h"
#include "watch_global_data.h"
#include "communicate_parse.h"

#define DBG_TAG "communicate.protocol"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

extern uint16_t skaiwalk_ble_app_notify(uint8_t *p_data, uint16_t data_len);

/* Effective ATT NTF/WRT payload ceiling: negotiated MTU (clamped to spec
   minimum) minus the 3-byte ATT header. */
static inline uint16_t l2_max_att_payload(void)
{
    uint16_t mtu = SkaiWatchSys.watch_mtu;
    if (mtu < BLE_DEFAULT_MTU) mtu = BLE_DEFAULT_MTU;
    return (uint16_t)(mtu - L2_ATT_OVERHEAD);
}

/* The original L1 layer (magic 0xAB + length + CRC16 + seq + ACK retry) was
   never wired up to the phone — phone packets arrive as raw L2 frames, so
   only the stripped-down dispatch below remains. */
void L1_receive_data_without_crc_check(uint8_t *data, uint16_t length)
{
    L2_frame_resolve(data, length);
}

/* Build L2 frames for one logical command and write them to BLE, splitting
   across multiple ATT writes if the payload doesn't fit in mtu - 3 - 5 bytes.
   Each non-final fragment sets L2_MORE_FRAGMENTS_BIT in byte[3] of the L2
   header; the receiver reassembles by (cmd, key) until a fragment without the
   bit arrives. The caller never has to allocate a buffer big enough for the
   whole frame — only the per-fragment scratch (~MTU bytes) is used here. */
bool skaiwatch_ble_send_l2(uint8_t cmd_id, uint8_t key,
                           const uint8_t *payload, uint16_t payload_len)
{
    uint16_t max_pkt = l2_max_att_payload();
    if (max_pkt <= L2_FIRST_VALUE_POS)
    {
        LOG_E("MTU too small for L2 framing");
        return false;
    }
    /* Cap by our notify characteristic's declared max_len. Sending more than
       the GATT attribute size gets the receiver's BLE stack to reply with
       GATT_INVALID_ATTRIBUTE_LENGTH (Android) or silently truncate (some
       other stacks). BLE_APP_CHAR_MAX_LEN is in turn bounded by the BT-spec
       ceiling of 512 (BLE_MAX_ATTR_VALUE_LEN). */
    if (max_pkt > BLE_APP_CHAR_MAX_LEN) max_pkt = BLE_APP_CHAR_MAX_LEN;
    if (max_pkt > L2_TX_PKT_BUF_SIZE) max_pkt = L2_TX_PKT_BUF_SIZE;

    uint16_t frag_payload_max = max_pkt - L2_FIRST_VALUE_POS;
    uint8_t frag_buf[L2_TX_PKT_BUF_SIZE];

    uint16_t offset = 0;
    /* do/while so a zero-length payload still produces one empty L2 frame. */
    do
    {
        uint16_t remain = (uint16_t)(payload_len - offset);
        uint16_t chunk  = (remain > frag_payload_max) ? frag_payload_max : remain;
        bool     more   = (offset + chunk) < payload_len;

        frag_buf[0] = cmd_id;
        frag_buf[1] = L2_HEADER_VERSION;
        frag_buf[2] = key;
        frag_buf[3] = (uint8_t)((more ? L2_MORE_FRAGMENTS_BIT : 0) | ((chunk >> 8) & 0x01));
        frag_buf[4] = (uint8_t)(chunk & 0xFF);
        if (chunk > 0 && payload != NULL)
        {
            memcpy(frag_buf + L2_FIRST_VALUE_POS, payload + offset, chunk);
        }

        if (skaiwalk_ble_app_notify(frag_buf, chunk + L2_FIRST_VALUE_POS) == 0)
        {
            LOG_E("L2 send failed at %u/%u (cmd=0x%02x key=0x%02x)",
                  offset, payload_len, cmd_id, key);
            return false;
        }
        offset += chunk;
    } while (offset < payload_len);

    return true;
}

/* Legacy wrapper. Fast path: if the pre-built frame fits in one BLE write,
   send it as-is (zero copy, wire-identical to the original implementation).
   Otherwise unpack (cmd, key) + payload and hand off to skaiwatch_ble_send_l2
   for fragmentation. */
bool skaiwatch_ble_notify(uint8_t *buf, uint16_t length)
{
    if (buf == NULL || length < L2_FIRST_VALUE_POS)
    {
        LOG_E("skaiwatch_ble_notify: malformed frame, length=%u", length);
        return false;
    }

    if (length <= l2_max_att_payload())
    {
        return skaiwalk_ble_app_notify(buf, length) > 0;
    }

    return skaiwatch_ble_send_l2(buf[0], buf[2],
                                  buf + L2_FIRST_VALUE_POS,
                                  length - L2_FIRST_VALUE_POS);
}
