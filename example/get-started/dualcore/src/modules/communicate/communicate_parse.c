/*********************************************************************************************************
 *               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_parse.c
 * @brief
 * @details
 * @author
 * @date
 * @version  v0.1
 *********************************************************************************************************
 */
#include <rtthread.h>
#include "string.h"
#include "communicate_parse.h"
#include "communicate_parse_skailink.h"
#include "communicate_protocol.h"

#define DBG_TAG "communicate.parse"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

/* L2 fragment reassembly state. The phone may split a single logical command
   across multiple BLE writes when the payload exceeds the negotiated MTU; each
   non-final fragment carries L2_MORE_FRAGMENTS_BIT in byte[3] of the header.
   Single-thread BLE rx callback so no locking needed. */
static uint8_t  s_l2_reass_buf[L2_REASSEMBLY_MAX_BYTES];
static uint16_t s_l2_reass_len = 0;
static uint8_t  s_l2_reass_cmd = 0;
static uint8_t  s_l2_reass_key = 0;
static bool     s_l2_reass_active = false;

static inline void l2_reass_reset(void)
{
    s_l2_reass_len = 0;
    s_l2_reass_active = false;
}

static void dispatch_l2_command(uint8_t cmd_id, uint8_t key, uint8_t *payload, uint16_t length)
{
    switch ((WRISTBAND_COMMUNICATE_COMMAND)cmd_id)
    {
    case BOND_COMMAND_ID:
        resolve_private_bond_command(key, payload, length);
        break;
    case SET_CONFIG_COMMAND_ID:
        LOG_D("SET_CONFIG_COMMAND, KEY = 0x%x", key);
        resolve_settings_config_command(key, payload, length);
        break;
    case HEALTH_DATA_COMMAND_ID:
        LOG_D("HEALTH_DATA_COMMAND, KEY = 0x%x", key);
        resolve_HealthData_command(key, payload, length);
        break;
    case NOTIFY_COMMAND_ID:
        LOG_D("NOTIFY_COMMAND_ID, KEY = 0x%x", key);
        resolve_Notify_command(key, payload, length);
        break;
    case CONTROL_COMMAND_ID:
        LOG_D("CONTROL_COMMAND, KEY = 0x%x", key);
        resolve_Control_command(key, payload, length);
        break;
    case SKAI_LINK_COMMAND_ID:
        LOG_D("SKAI_LINK_COMMAND, KEY = 0x%x", key);
        resolve_skailink_command(key, payload, length);
        break;
    default:
        LOG_E("Unknown command id 0x%02x", cmd_id);
        break;
    }
}

/**
 * @brief   resolve received data from remote APP
 * @param   pData: received frame pointer (raw BLE write payload)
 * @param   length: frame length in bytes
 * @retval  true on success, false on malformed input
 */
bool L2_frame_resolve(uint8_t *pData, uint16_t length)
{
    if ((pData == NULL) || (length < L2_FIRST_VALUE_POS))
    {
        return false;
    }

    uint8_t  command_id = pData[0];
    uint8_t  first_key  = pData[2];
    bool     more_frag  = (pData[3] & L2_MORE_FRAGMENTS_BIT) != 0;
    uint16_t value_len  = (((pData[3] << 8) | pData[4]) & 0x1FF);

    if ((uint32_t)L2_FIRST_VALUE_POS + value_len > length)
    {
        LOG_E("L2 frame truncated: have %u, need %u", length,
              L2_FIRST_VALUE_POS + value_len);
        l2_reass_reset();
        return false;
    }

    uint8_t *frag_payload = pData + L2_FIRST_VALUE_POS;

    /* Single-fragment frame: dispatch directly without touching the chain
       state UNLESS this frame is the final fragment of the in-progress chain
       (same cmd+key). This is critical during OTA — a stream of single-frame
       KEY_UPDATE_WATCH_* chunks must not clobber an unrelated fragmented
       phone→watch command (e.g., a long notification text) that happens to
       be in mid-reassembly. */
    if (!more_frag)
    {
        bool is_chain_tail = s_l2_reass_active &&
                             s_l2_reass_cmd == command_id &&
                             s_l2_reass_key == first_key;
        if (!is_chain_tail)
        {
            dispatch_l2_command(command_id, first_key, frag_payload, value_len);
            return true;
        }
        /* Fall through to append + dispatch. */
    }

    /* From here on we know this is either a continuation fragment or the
       final tail of an existing chain. A continuation that doesn't match
       the active chain replaces it (real interleaving of two fragmented
       chains; rare — last-writer-wins, with a warning so we notice). */
    if (!s_l2_reass_active || s_l2_reass_cmd != command_id || s_l2_reass_key != first_key)
    {
        if (s_l2_reass_active)
        {
            LOG_W("L2 reass: chain (0x%02x:0x%02x) interrupted by (0x%02x:0x%02x), drop %u bytes",
                  s_l2_reass_cmd, s_l2_reass_key, command_id, first_key, s_l2_reass_len);
        }
        s_l2_reass_cmd = command_id;
        s_l2_reass_key = first_key;
        s_l2_reass_len = 0;
        s_l2_reass_active = true;
    }

    if ((uint32_t)s_l2_reass_len + value_len > L2_REASSEMBLY_MAX_BYTES)
    {
        LOG_E("L2 reass overflow (%u + %u > %u), drop chain",
              s_l2_reass_len, value_len, L2_REASSEMBLY_MAX_BYTES);
        l2_reass_reset();
        return false;
    }
    memcpy(s_l2_reass_buf + s_l2_reass_len, frag_payload, value_len);
    s_l2_reass_len += value_len;

    if (more_frag)
    {
        return true; /* wait for the next fragment */
    }

    /* Last fragment: dispatch the concatenated payload, then clear state. */
    dispatch_l2_command(s_l2_reass_cmd, s_l2_reass_key, s_l2_reass_buf, s_l2_reass_len);
    l2_reass_reset();
    return true;
}
