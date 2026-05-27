/*********************************************************************************************************
 *               Copyright(c) 2024, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_parse_factory.c
 * @brief    Resolves FACTORY_TEST_COMMAND_ID payloads. The factory test app is
 *           hidden from the watch UI and only reachable via this command (sent
 *           from the phone development page).
 *********************************************************************************************************
 */
#include <rtthread.h>
#include "communicate_parse.h"
#include "communicate_parse_factory.h"
#include "gui_app_fwk.h"
#include "gui_app_fwk2.h"
#include "string.h"

#define DBG_TAG "commu.parse.factory"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

/**
 * @brief   resolve Factory Test command received from remote APP
 * @param   key: L2 key
 * @param   pValue: received value pointer
 * @param   length: value length
 */
void resolve_factory_test_command(uint8_t key, const uint8_t *pValue,
                                  uint16_t length)
{
    (void)pValue;
    (void)length;

    switch (key)
    {
    case KEY_OPEN_TEST_APP:
        LOG_I("Factory test: launch test app");
        /* Safe from the BLE rx thread: gui_app_run() only posts
           GUI_APP_MSG_RUN_APP to the GUI task mailbox. */
        gui_app_run("test");
        break;

    default:
        LOG_W("Unknown factory test key 0x%02x", key);
        break;
    }
}
