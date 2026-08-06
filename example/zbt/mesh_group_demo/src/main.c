/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 * Copyright (c) 2026-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>

#include <zephyr/settings/settings.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>


#include "../../../middleware/bluetooth/zephyr_bt/host/id.h"
#include "../../../middleware/bluetooth/zephyr_bt/mesh/net.h"
#include "../../../middleware/bluetooth/zephyr_bt/mesh/access.h"  // For bt_mesh_primary_addr()
#include "../../../external/FlashDB/inc/flashdb.h"
#include "../../../middleware/bluetooth/include/bf0_sibles_nvds.h"

#include "board.h"

#define OP_ONOFF_GET       BT_MESH_MODEL_OP_2(0x82, 0x01)
#define OP_ONOFF_SET       BT_MESH_MODEL_OP_2(0x82, 0x02)
#define OP_ONOFF_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x03)
#define OP_ONOFF_STATUS    BT_MESH_MODEL_OP_2(0x82, 0x04)

static void attention_on(const struct bt_mesh_model *mod)
{
    board_led_set(true);
}

static void attention_off(const struct bt_mesh_model *mod)
{
    board_led_set(false);
}

static const struct bt_mesh_health_srv_cb health_cb =
{
    .attn_on = attention_on,
    .attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv =
{
    .cb = &health_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static const char *const onoff_str[] = { "off", "on" };

static struct
{
    bool val;
    uint8_t tid;
    uint16_t src;
    uint32_t transition_time;
    struct k_work_delayable work;
} onoff;

/* OnOff messages' transition time and remaining time fields are encoded as an
 * 8 bit value with a 6 bit step field and a 2 bit resolution field.
 * The resolution field maps to:
 * 0: 100 ms
 * 1: 1 s
 * 2: 10 s
 * 3: 20 min
 */
static const uint32_t time_res[] =
{
    100,
    MSEC_PER_SEC,
    10 * MSEC_PER_SEC,
    10 * 60 * MSEC_PER_SEC,
};

static inline int32_t model_time_decode(uint8_t val)
{
    uint8_t resolution = (val >> 6) & BIT_MASK(2);
    uint8_t steps = val & BIT_MASK(6);

    if (steps == 0x3f)
    {
        return SYS_FOREVER_MS;
    }

    return steps * time_res[resolution];
}

static inline uint8_t model_time_encode(int32_t ms)
{
    if (ms == SYS_FOREVER_MS)
    {
        return 0x3f;
    }

    for (int i = 0; i < ARRAY_SIZE(time_res); i++)
    {
        if (ms >= BIT_MASK(6) * time_res[i])
        {
            continue;
        }

        uint8_t steps = DIV_ROUND_UP(ms, time_res[i]);

        return steps | (i << 6);
    }

    return 0x3f;
}

static int onoff_status_send(const struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx)
{
    uint32_t remaining;

    BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_STATUS, 3);
    bt_mesh_model_msg_init(&buf, OP_ONOFF_STATUS);

    remaining = k_ticks_to_ms_floor32(
                    k_work_delayable_remaining_get(&onoff.work)) +
                onoff.transition_time;

    /* Check using remaining time instead of "work pending" to make the
     * onoff status send the right value on instant transitions. As the
     * work item is executed in a lower priority than the mesh message
     * handler, the work will be pending even on instant transitions.
     */
    if (remaining)
    {
        net_buf_simple_add_u8(&buf, !onoff.val);
        net_buf_simple_add_u8(&buf, onoff.val);
        net_buf_simple_add_u8(&buf, model_time_encode(remaining));
    }
    else
    {
        net_buf_simple_add_u8(&buf, onoff.val);
    }

    return bt_mesh_model_send(model, ctx, &buf, NULL, NULL);
}

static void onoff_timeout(struct k_work *work)
{
    if (onoff.transition_time)
    {
        /* Start transition.
         *
         * The LED should be on as long as the transition is in
         * progress, regardless of the target value, according to the
         * Bluetooth Mesh Model specification, section 3.1.1.
         */
        board_led_set(true);

        k_work_reschedule(&onoff.work, K_MSEC(onoff.transition_time));
        onoff.transition_time = 0;
        return;
    }

    board_led_set(onoff.val);
}

/* Generic OnOff Server message handlers */

static int gen_onoff_get(const struct bt_mesh_model *model,
                         struct bt_mesh_msg_ctx *ctx,
                         struct net_buf_simple *buf)
{
    onoff_status_send(model, ctx);
    return 0;
}

static int gen_onoff_set_unack(const struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf)
{
    uint8_t val = net_buf_simple_pull_u8(buf);
    uint8_t tid = net_buf_simple_pull_u8(buf);
    int32_t trans = 0;
    int32_t delay = 0;

    if (buf->len)
    {
        trans = model_time_decode(net_buf_simple_pull_u8(buf));
        delay = net_buf_simple_pull_u8(buf) * 5;
    }

    /* Only perform change if the message wasn't a duplicate and the
     * value is different.
     */
    if (tid == onoff.tid && ctx->addr == onoff.src)
    {
        /* Duplicate */
        return 0;
    }

    if (val == onoff.val)
    {
        /* No change */
        return 0;
    }

    printk("set: %s delay: %d ms time: %d ms\n", onoff_str[val], delay,
           trans);

    if (val == 0)
    {
        // TODO: green light off
    }
    else if (val == 1)
    {
        // TODO: green light on
    }

    onoff.tid = tid;
    onoff.src = ctx->addr;
    onoff.val = val;
    onoff.transition_time = trans;

    /* Schedule the next action to happen on the delay, and keep
     * transition time stored, so it can be applied in the timeout.
     */
    k_work_reschedule(&onoff.work, K_MSEC(delay));

    return 0;
}

static int gen_onoff_set(const struct bt_mesh_model *model,
                         struct bt_mesh_msg_ctx *ctx,
                         struct net_buf_simple *buf)
{
    (void)gen_onoff_set_unack(model, ctx, buf);
    onoff_status_send(model, ctx);

    return 0;
}

static const struct bt_mesh_model_op gen_onoff_srv_op[] =
{
    { OP_ONOFF_GET,       BT_MESH_LEN_EXACT(0), gen_onoff_get },
    { OP_ONOFF_SET,       BT_MESH_LEN_MIN(2),   gen_onoff_set },
    { OP_ONOFF_SET_UNACK, BT_MESH_LEN_MIN(2),   gen_onoff_set_unack },
    BT_MESH_MODEL_OP_END,
};

/* Generic OnOff Client */

static int gen_onoff_status(const struct bt_mesh_model *model,
                            struct bt_mesh_msg_ctx *ctx,
                            struct net_buf_simple *buf)
{
    uint8_t present = net_buf_simple_pull_u8(buf);

    if (buf->len)
    {
        uint8_t target = net_buf_simple_pull_u8(buf);
        int32_t remaining_time =
            model_time_decode(net_buf_simple_pull_u8(buf));

        printk("OnOff status: %s -> %s: (%d ms)\n", onoff_str[present],
               onoff_str[target], remaining_time);
        return 0;
    }

    printk("OnOff status: %s\n", onoff_str[present]);

    return 0;
}

static const struct bt_mesh_model_op gen_onoff_cli_op[] =
{
    {OP_ONOFF_STATUS, BT_MESH_LEN_MIN(1), gen_onoff_status},
    BT_MESH_MODEL_OP_END,
};

/* This application only needs one element to contain its models */
/* Note: keys and groups arrays are automatically allocated by BT_MESH_MODEL macro */
/* based on CONFIG_BT_MESH_MODEL_KEY_COUNT and CONFIG_BT_MESH_MODEL_GROUP_COUNT */
/* IMPORTANT: models array MUST NOT be const, as mesh settings restore modifies keys/groups at runtime */
static struct bt_mesh_model models[] =
{
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op, NULL,
                  NULL),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op, NULL,
                  NULL),
};

static const struct bt_mesh_elem elements[] =
{
    BT_MESH_ELEM(0, models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp =
{
    .cid = BT_COMP_ID_LF,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

/* Provisioning */

static void prov_input_complete(void)
{
    printk("Provisioning input complete\n");
}

static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
    printk("OOB Number: %u (action: 0x%02x)\n", number, action);

    board_output_number(action, number);

    return 0;
}

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
    printk("=== Provisioning SUCCESS ===\n");
    printk("NetIdx: 0x%04x, Addr: 0x%04x\n", net_idx, addr);

    // TODO: red ligth on

    /* Force immediate storage of provisioning data */
    if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
        printk("Storing provisioning data...\n");

        extern void bt_mesh_settings_store_pending();
        // Trigger immediate storage of all pending Mesh settings
        bt_mesh_settings_store_pending();

        /* Wait for NVDS flush work to complete (500ms delay in settings_sifli.c) */
        k_sleep(K_MSEC(600));

        printk("Provisioning data stored\n");
    }

    board_prov_complete();
}

static void prov_reset(void)
{
    printk("=== Provisioning Reset ===\n");
    printk("Node has been reset and is now unprovisioned\n");

    /* CRITICAL: Clear Flash storage to prevent state restoration after reboot */
    if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
        printk("Clearing Mesh settings from FlashDB...\n");

        int deleted_count = 0;
        struct fdb_kv_iterator itr_obj;
        fdb_kvdb_t kvdb;

        /* Get FlashDB instance */
        kvdb = sifli_nvds_get_ble_kvdb();

        if (kvdb)
        {
            /* Iterate and delete all bt/mesh/* keys */
            bool continue_iterating = true;

            while (continue_iterating)
            {
                struct fdb_kv_iterator *itr = &itr_obj;
                memset(itr, 0, sizeof(struct fdb_kv_iterator));

                continue_iterating = false;

                while (fdb_kv_iterate(kvdb, itr))
                {
                    const char *key_name = itr->curr_kv.name;

                    /* Check if this is a Mesh-related key */
                    if (strncmp(key_name, "bt/mesh", 7) == 0)
                    {
                        int ret = fdb_kv_del(kvdb, key_name);
                        if (ret == FDB_NO_ERR)
                        {
                            printk("  Deleted: %s\n", key_name);
                            deleted_count++;
                            /* After deletion, restart iteration from beginning */
                            continue_iterating = true;
                            break;
                        }
                        else
                        {
                            printk("  Failed to delete: %s (err=%d)\n", key_name, ret);
                        }
                    }
                }
            }

            printk("✓ Deleted %d Mesh keys from FlashDB\n", deleted_count);
        }
        else
        {
            printk("⚠ Could not access FlashDB, settings may persist!\n");
        }
    }

    bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
    drv_reboot();
}

static uint8_t dev_uuid[16];

/* Forward declaration for bt_ready function */
static void bt_ready(int err);

/* Provision structure without static init - will be filled in bt_ready */
static struct bt_mesh_prov prov =
{
    .output_size = 4,
    .output_actions = BT_MESH_DISPLAY_NUMBER,
    .output_number = output_number,
    .input_complete = prov_input_complete,
    .complete = prov_complete,
    .reset = prov_reset,
};

/** Send an OnOff Set message from the Generic OnOff Client to all nodes. */
static int gen_onoff_send(bool val)
{
    struct bt_mesh_msg_ctx ctx =
    {
        .app_idx = models[3].keys[0], /* Use the bound key */
        .addr = BT_MESH_ADDR_ALL_NODES,
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };
    static uint8_t tid;

    if (ctx.app_idx == BT_MESH_KEY_UNUSED)
    {
        printk("The Generic OnOff Client must be bound to a key before "
               "sending.\n");
        return -ENOENT;
    }

    BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_SET_UNACK, 2);
    bt_mesh_model_msg_init(&buf, OP_ONOFF_SET_UNACK);
    net_buf_simple_add_u8(&buf, val);
    net_buf_simple_add_u8(&buf, tid++);

    printk("Sending OnOff Set: %s\n", onoff_str[val]);

    return bt_mesh_model_send(&models[3], &ctx, &buf, NULL, NULL);
}

static void mesh_btn()
{
    /* When using external provisioner, just print current status */
    if (bt_mesh_is_provisioned())
    {
        printk("Already provisioned, sending OnOff toggle\n");
        (void)gen_onoff_send(!onoff.val);
    }
    else
    {
        printk("Waiting for provisioner...\n");
    }
}

MSH_CMD_EXPORT(mesh_btn, enter button for Mesh example);

/* Debug command to check subscription status */
static void mesh_sub_check(uint8_t argc, char **argv)
{
    if (!bt_mesh_is_provisioned())
    {
        printk("Node is not provisioned\n");
        return;
    }

    printk("\n=== Subscription Status Check ===\n");
    printk("Node unicast address: 0x%04x\n\n", bt_mesh_primary_addr());

    /* Find and check Generic OnOff Server model */
    const struct bt_mesh_model *srv_model = NULL;
    int srv_idx = -1;
    for (int i = 0; i < ARRAY_SIZE(models); i++)
    {
        if (models[i].id == BT_MESH_MODEL_ID_GEN_ONOFF_SRV)
        {
            srv_model = &models[i];
            srv_idx = i;
            break;
        }
    }

    if (srv_model)
    {
        printk("Generic OnOff Server model (idx=%d):\n", srv_idx);
        printk("  Model ID: 0x%04x\n", srv_model->id);
        printk("  Groups count: %d\n", srv_model->groups_cnt);
        printk("  Key count: %d\n", srv_model->keys_cnt);

        /* Print bound AppKeys */
        uint16_t *keys = (uint16_t *)srv_model->keys;
        for (int j = 0; j < srv_model->keys_cnt && j < 16; j++)
        {
            printk("    AppKey[%d]: 0x%04x\n", j, keys[j]);
        }

        /* Print subscribed group addresses */
        printk("\n  Subscribed groups:\n");
        for (int j = 0; j < srv_model->groups_cnt; j++)
        {
            uint16_t *groups = (uint16_t *)srv_model->groups;
            if (groups[j] != 0 && groups[j] != 0xFFFF)
            {
                printk("    Group[%d]: 0x%04x\n", j, groups[j]);
            }
        }

        printk("\nExpected group addresses:\n");
        printk("  Total Group (0xC001)\n");
        printk("  Row Group (0xC1XX)\n");
    }
    else
    {
        printk("Generic OnOff Server model not found!\n");
    }

    /* Also check Generic OnOff Client model */
    const struct bt_mesh_model *cli_model = NULL;
    for (int i = 0; i < ARRAY_SIZE(models); i++)
    {
        if (models[i].id == BT_MESH_MODEL_ID_GEN_ONOFF_CLI)
        {
            cli_model = &models[i];
            printk("\nGeneric OnOff Client model (idx=%d):\n", i);
            printk("  Model ID: 0x%04x\n", cli_model->id);
            printk("  Groups count: %d\n", cli_model->groups_cnt);
            printk("  Key count: %d\n", cli_model->keys_cnt);

            uint16_t *keys = (uint16_t *)cli_model->keys;
            for (int j = 0; j < cli_model->keys_cnt && j < 16; j++)
            {
                printk("    AppKey[%d]: 0x%04x\n", j, keys[j]);
            }
            break;
        }
    }

    printk("\n==============================\n");
    printk("\nTest instructions:\n");
    printk("  1. Send mesh_send_onoff 0xFFFF from provisioner\n");
    printk("  2. Send mesh_send_onoff 0xC001 (total group)\n");
    printk("  3. Send mesh_send_onoff 0xC1XX (row group)\n");
}
MSH_CMD_EXPORT(mesh_sub_check, check node subscription status);

/* Debug command to manually trigger settings store */
static void mesh_store_now(uint8_t argc, char **argv)
{
    if (!bt_mesh_is_provisioned())
    {
        printk("Node is not provisioned\n");
        return;
    }

    printk("\n=== Manual Settings Store ===\n");

    /* Find Generic OnOff Server model */
    const struct bt_mesh_model *srv_model = NULL;
    for (int i = 0; i < ARRAY_SIZE(models); i++)
    {
        if (models[i].id == BT_MESH_MODEL_ID_GEN_ONOFF_SRV)
        {
            srv_model = &models[i];
            break;
        }
    }

    if (srv_model)
    {
        printk("Triggering subscription store for Generic OnOff Server...\n");
        bt_mesh_model_sub_store(srv_model);

        /* Wait and force immediate store */
        k_sleep(K_MSEC(100));
        extern void bt_mesh_settings_store_pending(void);
        bt_mesh_settings_store_pending();
        printk("Settings store triggered\n");
    }
}
MSH_CMD_EXPORT(mesh_store_now, manually trigger settings store);

/* Debug command to manually trigger beacon sending */
// static void mesh_debug_beacon(uint8_t argc, char **argv)
// {
//     printk("=== Manual Beacon Test ===\n");
//     printk("Is provisioned: %d\n", bt_mesh_is_provisioned());
//     printk("Is prov active: %d\n", bt_mesh_prov_active());

//     /* Force beacon enable */
//     bt_mesh_beacon_enable();
//     printk("Beacon enabled manually\n");
// }

// MSH_CMD_EXPORT(mesh_debug_beacon, manually trigger mesh beacon for debugging);

static void bt_ready(int err)
{
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    /* Read Bluetooth address and fill dev_uuid */
    bt_addr_le_t addr;
    if (bt_id_read_public_addr(&addr) == 1)
    {
        /* Fill dev_uuid with company pattern + Bluetooth address */
        dev_uuid[0] = 0xdd;
        dev_uuid[1] = 0xd6;

        /* Copy 6-byte Bluetooth address to UUID (big-endian) */
        dev_uuid[2] = addr.a.val[5];
        dev_uuid[3] = addr.a.val[4];
        dev_uuid[4] = addr.a.val[3];
        dev_uuid[5] = addr.a.val[2];
        dev_uuid[6] = addr.a.val[1];
        dev_uuid[7] = addr.a.val[0];
        /* Fill remaining bytes with a pattern */
        dev_uuid[8] = 0x00;
        dev_uuid[9] = 0x00;
        dev_uuid[10] = 0x00;
        dev_uuid[11] = 0x00;
        dev_uuid[12] = 0x00;
        dev_uuid[13] = 0x00;
        dev_uuid[14] = 0x00;
        dev_uuid[15] = 0x00;

        printk("Using BT address for UUID: %02x%02x:%02x%02x:%02x%02x%02x%02x%02x%02x\n",
               dev_uuid[0], dev_uuid[1],
               dev_uuid[2], dev_uuid[3], dev_uuid[4], dev_uuid[5],
               dev_uuid[6], dev_uuid[7], dev_uuid[8], dev_uuid[9]);
    }
    else
    {
        /* Fallback: generate random UUID */
        bt_rand(dev_uuid, sizeof(dev_uuid));
        dev_uuid[0] = 0xdd;
        dev_uuid[1] = 0xd6;
        printk("Using random UUID (fallback)\n");
    }

    /* Update prov structure with UUID */
    prov.uuid = dev_uuid;

    err = bt_mesh_init(&prov, &comp);
    if (err)
    {
        printk("Initializing mesh failed (err %d)\n", err);
        return;
    }

    printk("Mesh initialized\n");

    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        printk("Loading stored settings...\n");
        int ret = settings_load();
        printk("settings_load() returned %d\n", ret);
    }

    /* Check provisioning status after loading settings */
    if (bt_mesh_is_provisioned())
    {
        printk("\n=== Node Restored from Flash ===\n");
        printk("✓ Already provisioned with address: 0x%04x\n", bt_mesh_primary_addr());

        /* Print subscription information */
        printk("\n--- Subscription Status Check (after restore) ---\n");

        /* Find the Generic OnOff Server model and check its subscriptions */
        const struct bt_mesh_model *srv_model = NULL;
        for (int i = 0; i < ARRAY_SIZE(models); i++)
        {
            if (models[i].id == BT_MESH_MODEL_ID_GEN_ONOFF_SRV)
            {
                srv_model = &models[i];
                printk("Found Generic OnOff Server at model[%d]\n", i);
                break;
            }
        }

        if (srv_model)
        {
            /* Get element and model indices for debugging */
            const struct bt_mesh_elem *elem = bt_mesh_model_elem(srv_model);
            printk("  Model Elem_idx: %d\n", srv_model->rt->elem_idx);
            printk("  Groups count (max): %d\n", srv_model->groups_cnt);
            printk("  Key count (max): %d\n", srv_model->keys_cnt);

            /* Print bound AppKeys */
            uint16_t *keys = (uint16_t *)srv_model->keys;
            printk("  Bound AppKeys:\n");
            for (int j = 0; j < srv_model->keys_cnt && j < 4; j++)
            {
                printk("    [%d]: 0x%04x\n", j, keys[j]);
            }

            /* Print subscribed group addresses - THIS IS THE KEY CHECK */
            uint16_t *groups = (uint16_t *)srv_model->groups;
            printk("  Subscribed group addresses:\n");
            for (int j = 0; j < srv_model->groups_cnt; j++)
            {
                printk("    [%d]: 0x%04x\n", j, groups[j]);
            }

            /* Check if subscriptions are restored */
            bool found_c001 = false, found_row = false;
            for (int j = 0; j < srv_model->groups_cnt; j++)
            {
                if (groups[j] == 0xC001) found_c001 = true;
                if ((groups[j] & 0xFF00) == 0xC100) found_row = true;
            }

            printk("\n  Subscription restore status:\n");
            if (found_c001)
            {
                printk("    ✓ Total Group (0xC001): SUBSCRIBED\n");
            }
            else
            {
                printk("    ✗ Total Group (0xC001): NOT SUBSCRIBED!\n");
            }
            if (found_row)
            {
                printk("    ✓ Row Group (0xC1XX): SUBSCRIBED\n");
            }
            else
            {
                printk("    ✗ Row Group (0xC1XX): NOT SUBSCRIBED!\n");
            }
        }
        else
        {
            printk("  Generic OnOff Server model not found!\n");
        }

        printk("\n✓ Node is ready to receive commands\n");
        printk("==============================\n\n");

        // TODO: red ligth on
    }
    else
    {
        printk("\n=== Unprovisioned Node ===\n");
        printk("Waiting for provisioner to discover and configure this node...\n");

        /* Enable provisioning bearers */
        printk("Enabling provisioning bearers: ADV + GATT\n");
        err = bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
        if (err)
        {
            printk("Failed to enable provisioning (err %d)\n", err);
            return;
        }

        printk("Node is broadcasting unprovisioned beacon\n");
        printk("==============================\n\n");
    }
}

static void mesh_reset(uint8_t argc, char **argv)
{
    bool auto_reboot = false;

    /* Check if auto-reboot is requested */
    if (argc > 1 && strcmp(argv[1], "reboot") == 0)
    {
        auto_reboot = true;
    }

    printk("\n=== Resetting Mesh Configuration ===\n");
    if (auto_reboot)
    {
        printk("Auto-reboot mode enabled\n");
    }

    /* Step 1: Clear CDB (Configuration Database) */
    if (atomic_test_bit(bt_mesh_cdb.flags, BT_MESH_CDB_VALID))
    {
        printk("Clearing CDB...\n");
        bt_mesh_cdb_clear();
        printk("✓ CDB cleared\n");
    }
    else
    {
        printk("CDB is not valid, skipping clear\n");
    }

    /* Step 2: Clear Bluetooth settings from storage */
    if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
        printk("Clearing stored settings...\n");

        /* Delete all bt/mesh/* keys from FlashDB */
        int deleted_count = 0;
        struct fdb_kv_iterator itr_obj;
        fdb_kv_iterator_t itr = &itr_obj;
        fdb_kvdb_t kvdb;

        /* Get FlashDB instance through the NVDS adapter */
        kvdb = sifli_nvds_get_ble_kvdb();

        if (kvdb)
        {
            memset(itr, 0, sizeof(struct fdb_kv_iterator));
            itr->iterated_cnt = 0;

            /* Iterate and delete all bt/mesh/* keys */
            while (fdb_kv_iterate(kvdb, itr))
            {
                const char *key_name = itr->curr_kv.name;
                if (strncmp(key_name, "bt/mesh", 7) == 0)
                {
                    int ret = fdb_kv_del(kvdb, key_name);
                    if (ret == FDB_NO_ERR)
                    {
                        printk("  Deleted: %s\n", key_name);
                        deleted_count++;
                    }
                    else
                    {
                        printk("  Failed to delete: %s (err=%d)\n", key_name, ret);
                    }
                }
            }
            printk("✓ Deleted %d Mesh keys from FlashDB\n", deleted_count);
        }
        else
        {
            printk("⚠ Could not access FlashDB, settings may persist\n");
        }

        /* Also clear bt/keys and other BT settings if needed */
        printk("Note: Bluetooth pairing data (bt/keys/*) is preserved\n");
    }

    /* Step 3: Reset Mesh subsystem state */
    printk("Resetting Mesh subsystem state...\n");

    /* Clear provisioned flag (BT_MESH_VALID indicates device has been provisioned) */
    atomic_clear_bit(bt_mesh.flags, BT_MESH_VALID);
    printk("✓ Cleared provisioned flag\n");

    /* Step 4: Reboot or suggest reboot */
    if (auto_reboot)
    {
        printk("\n=== Auto-rebooting in 2 seconds ===\n");
        k_sleep(K_SECONDS(2));

        /* Trigger system reset */
        //extern void sfbl_reboot_to_bootloader(void);
        //sfbl_reboot_to_bootloader();

        /* If the above doesn't work, try NVIC reset */
#include <cmsis_gcc.h>
        NVIC_SystemReset();
    }
    else
    {
        printk("\n=== Reset Complete ===\n");
        printk("All Mesh configuration has been cleared.\n");
        printk("Please reboot the device to reinitialize Mesh.\n");
        printk("After reboot, the device will start as a new unprovisioned provisioner.\n");
        printk("\nUsage:\n");
        printk("  mesh_reset          - Reset without auto-reboot\n");
        printk("  mesh_reset reboot   - Reset and auto-reboot\n");
        printk("======================\n\n");
    }
}
MSH_CMD_EXPORT(mesh_reset, reset Mesh configuration and clear all stored data);

int main(void)
{
    static struct k_work button_work;
    int err = -1;

    printk("Initializing...\n");

    /* Generate a unique device UUID */
#if defined(CONFIG_HWINFO)
    err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
#endif

    if (err < 0)
    {
        /* Fallback: use random bytes for UUID */
        bt_rand(dev_uuid, sizeof(dev_uuid));
        /* Set first two bytes to identifiable pattern */
        dev_uuid[0] = 0xdd;
        dev_uuid[1] = 0xd6;

        printk("Using random UUID (first 4 bytes): %02x%02x%02x%02x...\n",
               dev_uuid[0], dev_uuid[1], dev_uuid[2], dev_uuid[3]);
    }
    else
    {
        printk("Using hardware UUID\n");
    }

    k_work_init(&button_work, mesh_btn);

    err = board_init(&button_work);
    if (err)
    {
        printk("Board init failed (err: %d)\n", err);
        return 0;
    }

    k_work_init_delayable(&onoff.work, onoff_timeout);

    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(bt_ready);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
    }

    return 0;
}






