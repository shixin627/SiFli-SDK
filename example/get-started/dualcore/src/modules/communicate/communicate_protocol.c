/*********************************************************************************************************
 *               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_protocol.c
 * @brief    斯凱沃克通信協議
 * @details
 * @author
 * @date
 * @version  v0.1
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

/******************* Private variables **********************************/
#define USING_L2_RESOLVE_TASK 0

#if USING_L2_RESOLVE_TASK
#define TASK_STACK_SIZE 512 * 6
#define TASK_PRIORITY 5
#define TASK_TICK 10
typedef struct
{
    uint8_t *buf;
    uint16_t len;
} l1_l2_msg_t;
static rt_mq_t l1_to_l2_mq = RT_NULL;

#define L1_L2_POOL_SIZE 4
static uint8_t l1_rx_pool[L1_L2_POOL_SIZE][GLOBAL_RECEIVE_BUFFER_SIZE];
static uint8_t l1_rx_pool_used[L1_L2_POOL_SIZE] = {0};

static uint8_t *alloc_l1_rx_buf(void);
static void free_l1_rx_buf(uint8_t *buf);

static rt_thread_t communicate_parse_thread = RT_NULL;
#endif

extern uint16_t skaiwalk_ble_app_notify(uint8_t *p_data, uint16_t data_len);

/* L1 layer (magic 0xAB + length + CRC16 + seq_id + ACK retry) was never
   wired to the phone bridge — phone packets arrive as raw L2 frames. The
   former L1_send / L1_send_ack / L1_receive_data / L1_crc_check stack and
   the matching extern decls in protocol.h have been removed. BLE bytes
   now flow phone → L1_receive_data_without_crc_check → L2_frame_resolve. */

void L1_receive_data_without_crc_check(uint8_t *data, uint16_t length)
{
#if USING_L2_RESOLVE_TASK
    uint8_t *buf = alloc_l1_rx_buf();
    if (!buf)
    {
        LOG_E("No free L1 RX buffer! Drop packet.");
        return;
    }
    memcpy(buf, data, length);
    l1_l2_msg_t msg = {.buf = buf, .len = length};
    if (rt_mq_send(l1_to_l2_mq, &msg, sizeof(msg)) != RT_EOK)
    {
        LOG_E("l1_to_l2_mq full, drop packet");
        free_l1_rx_buf(buf);
    }
#else
    L2_frame_resolve(data, length);
#endif
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
    uint16_t mtu = SkaiWatchSys.watch_mtu;
    if (mtu < BLE_DEFAULT_MTU) mtu = BLE_DEFAULT_MTU;
    uint16_t max_pkt = mtu - L2_ATT_OVERHEAD;
    if (max_pkt <= L2_FIRST_VALUE_POS)
    {
        LOG_E("MTU %u too small for L2 framing", mtu);
        return false;
    }
    /* Cap by our notify characteristic's declared max_len. Sending more
       than the GATT attribute size gets the receiver's BLE stack to reply
       with GATT_INVALID_ATTRIBUTE_LENGTH (Android) or silently truncate
       (some other stacks). BLE_APP_CHAR_MAX_LEN is also bounded by the
       BT-spec ceiling of 512 (BLE_MAX_ATTR_VALUE_LEN). */
    if (max_pkt > BLE_APP_CHAR_MAX_LEN) max_pkt = BLE_APP_CHAR_MAX_LEN;
    /* Defensive: also keep within our scratch buffer. */
    if (max_pkt > L2_TX_PKT_BUF_SIZE) max_pkt = L2_TX_PKT_BUF_SIZE;

    uint16_t frag_payload_max = max_pkt - L2_FIRST_VALUE_POS;
    uint8_t frag_buf[L2_TX_PKT_BUF_SIZE];

    uint16_t offset = 0;
    /* do/while so a zero-length payload still produces one empty L2 frame,
       matching the legacy behavior of wire-format-equivalent commands. */
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

    uint16_t mtu = SkaiWatchSys.watch_mtu;
    if (mtu < BLE_DEFAULT_MTU) mtu = BLE_DEFAULT_MTU;
    if (length <= (uint16_t)(mtu - L2_ATT_OVERHEAD))
    {
        return skaiwalk_ble_app_notify(buf, length) > 0;
    }

    return skaiwatch_ble_send_l2(buf[0], buf[2],
                                  buf + L2_FIRST_VALUE_POS,
                                  length - L2_FIRST_VALUE_POS);
}

#if USING_L2_RESOLVE_TASK
static uint8_t *alloc_l1_rx_buf(void)
{
    for (int i = 0; i < L1_L2_POOL_SIZE; i++)
    {
        if (!l1_rx_pool_used[i])
        {
            l1_rx_pool_used[i] = 1;
            return l1_rx_pool[i];
        }
    }
    return NULL;
}

static void free_l1_rx_buf(uint8_t *buf)
{
    for (int i = 0; i < L1_L2_POOL_SIZE; i++)
    {
        if (l1_rx_pool[i] == buf)
        {
            l1_rx_pool_used[i] = 0;
            return;
        }
    }
}

static void communicate_parse_thread_entry(void *parameter)
{
    l1_l2_msg_t msg;
    while (1)
    {
        if (rt_mq_recv(l1_to_l2_mq, &msg, sizeof(msg), RT_WAITING_FOREVER) == RT_EOK)
        {
            L2_frame_resolve(msg.buf, msg.len);
            free_l1_rx_buf(msg.buf);
        }
    }
}

static int communicate_parse_thread_init(void)
{
    if (l1_to_l2_mq == RT_NULL)
    {
        l1_to_l2_mq = rt_mq_create("l1l2mq", sizeof(l1_l2_msg_t), L1_L2_POOL_SIZE, RT_IPC_FLAG_FIFO);
        if (l1_to_l2_mq == RT_NULL)
        {
            LOG_E("l1_to_l2_mq create failed!");
            return -1;
        }
    }
    communicate_parse_thread = rt_thread_create("l2resolve",
                                                communicate_parse_thread_entry, RT_NULL,
                                                TASK_STACK_SIZE, TASK_PRIORITY, TASK_TICK);
    if (communicate_parse_thread != RT_NULL)
    {
        rt_thread_startup(communicate_parse_thread);
    }
    return 0;
}
INIT_APP_EXPORT(communicate_parse_thread_init);
#endif