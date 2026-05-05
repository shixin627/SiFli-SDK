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

bool skaiwatch_ble_notify(uint8_t *buf, uint16_t length)
{
    // if (SkaiWatchSys.watch_mtu < 233) // 233 is the max length of ble packet
    // {
    //     LOG_E("watch_mtu:%d is too small", SkaiWatchSys.watch_mtu);
    //     return false;
    // }
    // uint16_t max_len_once = SkaiWatchSys.watch_mtu - 3;
    // if (length > max_len_once)
    // {
    //     LOG_E("send data length:%d is too long", length);
    //     return false;
    // }
    uint16_t ret = skaiwalk_ble_app_notify(buf, length);
    return ret > 0;
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