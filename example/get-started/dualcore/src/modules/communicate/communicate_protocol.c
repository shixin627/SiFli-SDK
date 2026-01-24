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
#define DBG_LVL DBG_LOG
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

#define USE_L1_CRC_CHECK 0
#define MAX_RESEND_COUNT 3

extern uint16_t skaiwalk_ble_app_notify(uint8_t *p_data, uint16_t data_len);


/* Packet receive parameter */
static uint8_t received_buffer[GLOBAL_RECEIVE_BUFFER_SIZE];
static uint32_t package_len = 0;

#if USE_L1_CRC_CHECK
static uint32_t package_offset = 0;
/* L1 layer message parameter */
static uint16_t L1_sequence_id = 0;

static rt_mq_t l1recv_queue_handle; //!< Event queue handle

/**
 * @brief  CRC check L1 reveived packet
 * @param   crc_value: crc_value
 * @param   pData: data pointer to be checked
 * @param   length:  data length
 * @retval  error code
 */
static bool L1_crc_check(uint16_t crc_value, uint8_t *pData, uint16_t length)
{
    // uint16_t crc = btxfcs(0x0000, pData, length);
    // if (crc == crc_value)
    // {
    //     return true;
    // }
    return false;
}

bool L1_send(uint8_t *buf, uint16_t length)
{
    uint16_t record_len = length;
    /*fill header*/
    buf[L1_HEADER_MAGIC_POS] = L1_HEADER_MAGIC;                            /* Magic */
    buf[L1_HEADER_PROTOCOL_VERSION_POS] = L1_HEADER_VERSION;               /* protocol version */
    buf[L1_PAYLOAD_LENGTH_HIGH_BYTE_POS] = (length - L1_HEADER_SIZE) >> 8; /* length high byte */
    buf[L1_PAYLOAD_LENGTH_LOW_BYTE_POS] = length - L1_HEADER_SIZE;         /* length low byte */
    /*cal crc*/
    uint16_t crc16_ret = 0; // TODO: btxfcs(0, buf + L1_HEADER_SIZE, length - L1_HEADER_SIZE);
    buf[L1_HEADER_CRC16_HIGH_BYTE_POS] = crc16_ret >> 8;
    buf[L1_HEADER_CRC16_LOW_BYTE_POS] = crc16_ret;

    L1_sequence_id++;

    /* sequence id */
    buf[L1_HEADER_SEQ_ID_HIGH_BYTE_POS] = L1_sequence_id >> 8;
    buf[L1_HEADER_SEQ_ID_LOW_BYTE_POS] = L1_sequence_id;

    LOG_I("sequence id:%d", L1_sequence_id);

    uint8_t send_offset = 0;
    while (length)
    {
        if (length >= SkaiWatchSys.watch_mtu - 3)
        {
            skaiwalk_ble_app_notify(buf + send_offset, SkaiWatchSys.watch_mtu - 3);
            send_offset += SkaiWatchSys.watch_mtu - 3;
            length = length - (SkaiWatchSys.watch_mtu - 3);
        }
        else
        {
            skaiwalk_ble_app_notify(buf + send_offset, length);
            length = 0;
            send_offset = 0;
        }
    }

    uint8_t event;
    static uint32_t retry_count = 0;
    if (rt_mq_recv(l1recv_queue_handle, &event, sizeof(event), RT_TICK_PER_SECOND * 3) == RT_EOK)
    {
        LOG_I("receive L1 send ACK success!");
        retry_count = 0;
        return true;
    }
    else
    {
        retry_count++;
        LOG_I("receive L1 send ACK time out! do retry count = %d", retry_count);
        if (retry_count < MAX_RESEND_COUNT)
        {
            L1_send(buf, record_len);
        }
        else
        {
            LOG_E("retry fail!!! retry count= %d", retry_count);
            retry_count = 0;
        }
        return false;
    }
}

/**
 * @brief  Do action to received ACK packet
 *         if check fail should start to resend procedure
 * @param  sequence_id: L1 layer sequence id
 * @param  crc_check: ACK packet crc check result ,CRC_SUCCESS or CRC_FAIL
 * @retval void
 */
static void ack_package_handle(uint16_t sequence_id, bool err_flag)
{
    if (err_flag == false)
    {
        uint8_t event = sequence_id;
        if (rt_mq_send(l1recv_queue_handle, &event, sizeof(event)) != RT_EOK)
        {
            LOG_E("send_msg_to_l1send_task_fail");
        }
    }
    else
    {
        /*do nothing only operation like ACK timeout*/
        LOG_I("get the wrong ack of host");
    }
}

/**
 * @brief   response to L1 reveived packet
 * @param   sequence_id:   L1 sequence_id
 * @param   check_success: crc check result
 * @retval  error code
 */
void L1_send_ack(uint16_t sequence_id, bool check_success)
{
    uint8_t ack_package_buffer[8] = {0};
    L1_version_value_t version_ack;

    version_ack.version_def.version = L2_HEADER_VERSION;
    version_ack.version_def.ack_flag = 1;
    version_ack.version_def.err_flag = (check_success ? 0 : 1);
    version_ack.version_def.reserve = 0;

    ack_package_buffer[0] = L1_HEADER_MAGIC;
    ack_package_buffer[1] = version_ack.value;
    ack_package_buffer[2] = 0; // length
    ack_package_buffer[3] = 0; // length
    ack_package_buffer[4] = 0; // crc16
    ack_package_buffer[5] = 0; // crc16
    ack_package_buffer[6] = (sequence_id >> 8) & 0xFF;
    ack_package_buffer[7] = sequence_id & 0xFF;
    skaiwalk_ble_app_notify(ack_package_buffer, L1_HEADER_SIZE);
}

/**
 * @brief   deliver L1 received data to upper layer
 * @param   pData: pointer to L1 received data
 * @param   length:  data length
 * @retval  void
 */
void L1_receive_data(uint8_t *data, uint16_t length)
{
    static bool receiving = false;
    L1_version_value_t inner_version;

    if ((data[0] == L1_HEADER_MAGIC) && (!receiving))
    {
        package_len = (((uint16_t)(data[2])) << 8) + data[3] + 8;
        receiving = true;
    }
    else if (!receiving)
    {
        // magic code error
        LOG_E("Magic code error!");
        return;
    }

    if (package_offset + length <= GLOBAL_RECEIVE_BUFFER_SIZE)
    {
        memcpy(received_buffer + package_offset, data, length);
        package_offset += length;
    }
    else
    {
        // CMD length error
        LOG_E("CMD length larger than buffer size!");
        package_offset = 0;
        package_len = 0;
        receiving = false;
        return;
    }

    if (package_offset == package_len)
    {
        package_offset = 0;
        package_len = 0;
        receiving = false;
        // data package
        inner_version.value = received_buffer[L1_HEADER_PROTOCOL_VERSION_POS];
        uint16_t crc16_value = (received_buffer[L1_HEADER_CRC16_HIGH_BYTE_POS] << 8 | received_buffer[L1_HEADER_CRC16_LOW_BYTE_POS]);
        uint16_t L2_frame_length = received_buffer[L1_PAYLOAD_LENGTH_LOW_BYTE_POS] | (received_buffer[L1_PAYLOAD_LENGTH_HIGH_BYTE_POS] << 8);
        uint16_t seq_id = received_buffer[L1_HEADER_SEQ_ID_LOW_BYTE_POS] | (received_buffer[L1_HEADER_SEQ_ID_HIGH_BYTE_POS] << 8);

        LOG_I("ack error flag:%d, ack or data flag:%d, version:%d, seq_id:%d\n",
              inner_version.version_def.err_flag, inner_version.version_def.ack_flag,
              inner_version.version_def.version, seq_id);

        if (inner_version.version_def.ack_flag == RESPONSE_PACKAGE)
        {
            LOG_I("receive a ack package");
            // restart receive state machine
            ack_package_handle(seq_id, inner_version.version_def.err_flag);
            return;
        }

        if (L1_crc_check(crc16_value, received_buffer + L1_HEADER_SIZE, L2_frame_length) == true)
        {
            LOG_I("receive data package & send response");
            /* send response */
            L1_send_ack(seq_id, true);
            /*throw data to upper layer*/
            L2_frame_resolve(received_buffer + L1_HEADER_SIZE, L2_frame_length);
        }
        else
        {
            // send response
            L1_send_ack(seq_id, false);
            // schedule error handler
            LOG_I("received data crc check error");
        }
    }
    else if (package_offset > package_len) // Received data length larger than the expected length
    {
        LOG_I("CMD length larger than the expected length");
        package_offset = 0;
        package_len = 0;
        receiving = false;
    }
    else if (package_offset < package_len) // Received data length less than the expected length
    {
    }
}
#endif

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