/*
 * Copyright (c) 2019 Tobias Svehagen
 * Copyright (c) 2026-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Bluetooth Mesh Provisioner for 96-node Lighting Control System
 *
 * This application implements a Mesh provisioner for controlling 96 lighting nodes
 * organized in 4 groups × 3 rows × 8 columns configuration.
 *
 * Features:
 * - Targeted provisioning by group and row
 * - Automatic subscription configuration (group + row addresses)
 * - Unicast, group, and broadcast control
 * - Persistent storage with power-on recovery
 * - Node reset and network management
 *
 * Address Planning:
 * - Provisioner: 0x0001
 * - Nodes: 0x0100-0x015F (96 nodes)
 * - Group Total: 0xC001-0xC004 (4 groups)
 * - Group Row: 0xC111-0xC143 (12 rows)
 * - All Nodes: 0xFFFF
 *
 * Quick Start:
 * 1. Provision a row: mesh_start_target 0 0  (Group A, Row 1)
 * 2. Control lights:  mesh_send_onoff 0xFFFF 1  (All ON)
 * 3. List nodes:      mesh_list_nodes
 * 4. Help:            mesh_btn
 */

#include <stdlib.h>
#include <zephyr/sys/printk.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include "../../../middleware/bluetooth/zephyr_bt/mesh/net.h"
#include "../../../middleware/bluetooth/zephyr_bt/mesh/access.h"
#include "../../../middleware/bluetooth/zephyr_bt/mesh/subnet.h"
#include "../../../external/FlashDB/inc/flashdb.h"
#include "../../../middleware/bluetooth/include/bf0_sibles_nvds.h"

#define BTN_COUNT 1

/* OnOff message opcodes */
#define OP_ONOFF_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x03)

/* Mesh Address Planning Constants */
#define PROVISIONER_ADDR        0x0001
#define GROUP_ALL_NODES         0xFFFF
#define GROUP_A_TOTAL           0xC001
#define GROUP_B_TOTAL           0xC002
#define GROUP_C_TOTAL           0xC003
#define GROUP_D_TOTAL           0xC004

/* Group address calculation macros */
#define GROUP_ROW_BASE          0xC100
/*
 * Row address formula: 0xC100 + (group+1)*16 + (row+1)
 * Examples:
 *   Group A (0) Row 1 (0): 0xC100 + 16 + 1 = 0xC111
 *   Group A (0) Row 2 (1): 0xC100 + 16 + 2 = 0xC112
 *   Group B (1) Row 1 (0): 0xC100 + 32 + 1 = 0xC121
 *   Group D (3) Row 3 (2): 0xC100 + 64 + 3 = 0xC143
 */
#define GET_GROUP_ROW_ADDR(group, row)  (GROUP_ROW_BASE + (((group) + 1) << 4) + ((row) + 1))

/* Node address pool configuration */
#define NODES_PER_ROW         8   /* Each row has 8 nodes */
#define ROWS_PER_GROUP        3   /* Each group has 3 rows */
#define GROUPS_TOTAL          4   /* Total 4 groups (A-D) */
#define NODES_PER_GROUP       (NODES_PER_ROW * ROWS_PER_GROUP)  /* 24 nodes per group */
#define TOTAL_NODES           (GROUPS_TOTAL * NODES_PER_GROUP)  /* 96 nodes total */

/* Unicast address allocation:
 * Group A Row 1: 0x0100-0x0107
 * Group A Row 2: 0x0108-0x010F
 * Group A Row 3: 0x0110-0x0117
 * Group B Row 1: 0x0118-0x011F
 * ... and so on
 */
#define UNICAST_ADDR_BASE     0x0100
#define GET_NODE_ADDR(group, row, col) \
    (UNICAST_ADDR_BASE + ((group) * NODES_PER_GROUP) + ((row) * NODES_PER_ROW) + (col))

/* Address assignment tracking for targeted provisioning */
static struct
{
    uint16_t next_addr;  /* Next unicast address to assign */
} addr_pool =
{
    .next_addr = UNICAST_ADDR_BASE
};

/*
 * Address mapping table: maps actual assigned addresses to logical positions
 * This allows displaying the expected logical address (e.g., 0x0100)
 * even when Zephyr assigns a different address (e.g., 0x0002)
 */
#define MAX_MAPPED_NODES 96
static struct
{
    uint16_t actual_addr;      /* Actual address assigned by Zephyr */
    uint16_t logical_addr;     /* Expected logical address from our plan */
    uint8_t group;             /* Group index (0-3 for A-D) */
    uint8_t row;               /* Row index (0-2) */
    uint8_t col;               /* Column index (0-7) */
    bool valid;                /* Entry is in use */
    bool configuring;          /* Node is currently being configured */
} addr_mapping[MAX_MAPPED_NODES];

/* Find logical address for a given actual address */
static uint16_t get_logical_address(uint16_t actual_addr)
{
    for (int i = 0; i < MAX_MAPPED_NODES; i++)
    {
        if (addr_mapping[i].valid && addr_mapping[i].actual_addr == actual_addr)
        {
            return addr_mapping[i].logical_addr;
        }
    }
    return 0;  /* Not found */
}

/* Get position info for a given actual address */
static bool get_node_position(uint16_t actual_addr, uint8_t *group, uint8_t *row, uint8_t *col)
{
    for (int i = 0; i < MAX_MAPPED_NODES; i++)
    {
        if (addr_mapping[i].valid && addr_mapping[i].actual_addr == actual_addr)
        {
            if (group) *group = addr_mapping[i].group;
            if (row) *row = addr_mapping[i].row;
            if (col) *col = addr_mapping[i].col;
            return true;
        }
    }
    return false;
}

// /* Register address mapping for a newly provisioned node */
// static void register_address_mapping(uint16_t actual_addr, uint8_t group, uint8_t row, uint8_t col)
// {
//     uint16_t logical_addr = GET_NODE_ADDR(group, row, col);

//     /* Find empty slot or update existing */
//     for (int i = 0; i < MAX_MAPPED_NODES; i++) {
//         if (!addr_mapping[i].valid) {
//             addr_mapping[i].actual_addr = actual_addr;
//             addr_mapping[i].logical_addr = logical_addr;
//             addr_mapping[i].group = group;
//             addr_mapping[i].row = row;
//             addr_mapping[i].col = col;
//             addr_mapping[i].valid = true;

//             printk("Address mapping registered: actual=0x%04x -> logical=0x%04x (Group %c Row %d Col %d)\n",
//                    actual_addr, logical_addr, 'A' + group, row + 1, col + 1);
//             return;
//         }
//     }

//     printk("Warning: Address mapping table full!\n");
// }

/* ============================================================================
 * NVDS Storage for Node Location Persistence
 * ============================================================================
 */
#define NVDS_MESH_NODE_LOC_MAGIC 0x4E4C4D48  /* "NLMH" - NVDS Mesh Node Location Header */
#define NVDS_MESH_NODE_LOC_VERSION 1

typedef struct
{
    uint32_t magic;           /* Magic number for validation */
    uint8_t version;          /* Structure version */
    uint8_t node_count;       /* Number of valid nodes */
    uint8_t reserved[2];      /* Reserved for alignment */
    /* Node location data follows (variable length) */
    /* Format per node: uint16_t actual_addr, uint16_t logical_addr, uint8_t group, uint8_t row, uint8_t col */
} nvds_mesh_node_loc_header_t;

#define NVDS_NODE_LOC_ENTRY_SIZE 7  /* actual_addr(2) + logical_addr(2) + group(1) + row(1) + col(1) = 7 bytes per entry */

/* Maximum payload size for NVDS (accounting for header) */
#define NVDS_MESH_LOC_MAX_SIZE (SIFLI_NVDS_KEY_LEN_MESH)

/* Save node location table to NVDS */
static int mesh_location_save(void)
{
    nvds_mesh_node_loc_header_t header;
    uint8_t *buffer;
    uint8_t *p;
    int node_count = 0;
    uint8_t status;

    /* Count valid nodes */
    for (int i = 0; i < MAX_MAPPED_NODES; i++)
    {
        if (addr_mapping[i].valid)
        {
            node_count++;
        }
    }

    /* Allocate buffer */
    uint16_t buf_size = sizeof(nvds_mesh_node_loc_header_t) + (node_count * NVDS_NODE_LOC_ENTRY_SIZE);
    if (buf_size > NVDS_MESH_LOC_MAX_SIZE)
    {
        printk("Warning: Node location data too large (%d bytes)\n", buf_size);
        buf_size = NVDS_MESH_LOC_MAX_SIZE;
    }

    buffer = (uint8_t *)k_malloc(buf_size);
    if (!buffer)
    {
        printk("Failed to allocate buffer for node location save\n");
        return -1;
    }

    p = buffer;

    /* Build header */
    header.magic = NVDS_MESH_NODE_LOC_MAGIC;
    header.version = NVDS_MESH_NODE_LOC_VERSION;
    header.node_count = node_count;
    header.reserved[0] = 0;
    header.reserved[1] = 0;

    /* Copy header to buffer */
    memcpy(p, &header, sizeof(nvds_mesh_node_loc_header_t));
    p += sizeof(nvds_mesh_node_loc_header_t);

    /* Copy node location data */
    for (int i = 0; i < MAX_MAPPED_NODES && node_count > 0; i++)
    {
        if (addr_mapping[i].valid)
        {
            /* Pack: actual_addr(2) + logical_addr(2) + group(1) + row(1) + col(1) */
            *(uint16_t *)p = addr_mapping[i].actual_addr;
            p += 2;
            *(uint16_t *)p = addr_mapping[i].logical_addr;
            p += 2;
            *p++ = addr_mapping[i].group;
            *p++ = addr_mapping[i].row;
            *p++ = addr_mapping[i].col;

            node_count--;
        }
    }

    /* Write to NVDS */
    uint16_t len = (uint16_t)(p - buffer);
    status = sifli_nvds_write(SIFLI_NVDS_TYPE_MESH, len, buffer);

    k_free(buffer);

    if (status != NVDS_OK)
    {
        printk("Failed to save node location to NVDS (status=%d)\n", status);
        return -1;
    }

    printk("Saved %d node locations to NVDS\n", header.node_count);
    return 0;
}

/* Load node location table from NVDS */
static int mesh_location_load(void)
{
    uint8_t *buffer;
    uint8_t *p;
    nvds_mesh_node_loc_header_t *header;
    uint16_t len = NVDS_MESH_LOC_MAX_SIZE;
    uint8_t status;

    /* Allocate buffer */
    buffer = (uint8_t *)k_malloc(len);
    if (!buffer)
    {
        printk("Failed to allocate buffer for node location load\n");
        return -1;
    }

    /* Read from NVDS */
    status = sifli_nvds_read(SIFLI_NVDS_TYPE_MESH, &len, buffer);

    if (status != NVDS_OK)
    {
        k_free(buffer);
        if (status == NVDS_TAG_NOT_DEFINED)
        {
            printk("No saved node location data in NVDS (first boot or cleared)\n");
        }
        else
        {
            printk("Failed to read node location from NVDS (status=%d)\n", status);
        }
        return -1;
    }

    if (len < sizeof(nvds_mesh_node_loc_header_t))
    {
        printk("Invalid NVDS data length: %d bytes\n", len);
        k_free(buffer);
        return -1;
    }

    header = (nvds_mesh_node_loc_header_t *)buffer;

    /* Validate header */
    if (header->magic != NVDS_MESH_NODE_LOC_MAGIC)
    {
        printk("Invalid NVDS magic: 0x%08x (expected 0x%08x)\n",
               header->magic, NVDS_MESH_NODE_LOC_MAGIC);
        k_free(buffer);
        return -1;
    }

    if (header->version != NVDS_MESH_NODE_LOC_VERSION)
    {
        printk("NVDS version mismatch: %d (expected %d)\n",
               header->version, NVDS_MESH_NODE_LOC_VERSION);
        k_free(buffer);
        return -1;
    }

    /* Clear current mapping table */
    memset(addr_mapping, 0, sizeof(addr_mapping));

    /* Parse node location entries */
    p = buffer + sizeof(nvds_mesh_node_loc_header_t);
    int data_len = len - sizeof(nvds_mesh_node_loc_header_t);
    int entry_count = 0;

    while (data_len >= NVDS_NODE_LOC_ENTRY_SIZE && entry_count < header->node_count &&
            entry_count < MAX_MAPPED_NODES)
    {
        /* Find empty slot in mapping table */
        for (int i = 0; i < MAX_MAPPED_NODES; i++)
        {
            if (!addr_mapping[i].valid)
            {
                /* Unpack: actual_addr(2) + logical_addr(2) + group(1) + row(1) + col(1) */
                addr_mapping[i].actual_addr = *(uint16_t *)p;
                p += 2;
                addr_mapping[i].logical_addr = *(uint16_t *)p;
                p += 2;
                addr_mapping[i].group = *p++;
                addr_mapping[i].row = *p++;
                addr_mapping[i].col = *p++;

                addr_mapping[i].valid = true;
                addr_mapping[i].configuring = false;

                entry_count++;
                data_len -= NVDS_NODE_LOC_ENTRY_SIZE;
                break;
            }
        }
    }

    k_free(buffer);

    printk("Loaded %d node locations from NVDS\n", entry_count);

    /* Debug: Print loaded locations */
    for (int i = 0; i < MAX_MAPPED_NODES; i++)
    {
        if (addr_mapping[i].valid)
        {
            printk("  [Recovery] actual=0x%04x logical=0x%04x Group %c Row %d Col %d\n",
                   addr_mapping[i].actual_addr,
                   addr_mapping[i].logical_addr,
                   'A' + addr_mapping[i].group,
                   addr_mapping[i].row + 1,
                   addr_mapping[i].col + 1);
        }
    }

    return entry_count;
}

/* Update register_address_mapping to also save to NVDS */
static void register_address_mapping(uint16_t actual_addr, uint8_t group, uint8_t row, uint8_t col)
{
    uint16_t logical_addr = GET_NODE_ADDR(group, row, col);
    bool found = false;

    /* Find existing or empty slot */
    for (int i = 0; i < MAX_MAPPED_NODES; i++)
    {
        if (!addr_mapping[i].valid)
        {
            addr_mapping[i].actual_addr = actual_addr;
            addr_mapping[i].logical_addr = logical_addr;
            addr_mapping[i].group = group;
            addr_mapping[i].row = row;
            addr_mapping[i].col = col;
            addr_mapping[i].valid = true;

            printk("Address mapping registered: actual=0x%04x -> logical=0x%04x (Group %c Row %d Col %d)\n",
                   actual_addr, logical_addr, 'A' + group, row + 1, col + 1);

            /* Save to NVDS */
            mesh_location_save();
            return;
        }
    }

    printk("Warning: Address mapping table full!\n");
}

/* Remove a node from location storage and save to NVDS */
static void mesh_location_remove(uint16_t actual_addr)
{
    for (int i = 0; i < MAX_MAPPED_NODES; i++)
    {
        if (addr_mapping[i].valid && addr_mapping[i].actual_addr == actual_addr)
        {
            printk("Removing location for node 0x%04x (Group %c Row %d Col %d)\n",
                   actual_addr, 'A' + addr_mapping[i].group,
                   addr_mapping[i].row + 1, addr_mapping[i].col + 1);

            addr_mapping[i].valid = false;
            addr_mapping[i].actual_addr = 0;
            addr_mapping[i].logical_addr = 0;
            addr_mapping[i].group = 0;
            addr_mapping[i].row = 0;
            addr_mapping[i].col = 0;
            addr_mapping[i].configuring = false;

            /* Save updated table to NVDS */
            mesh_location_save();
            return;
        }
    }

    printk("Node 0x%04x not found in location table\n", actual_addr);
}

/* Clear all location data from NVDS */
static void mesh_location_clear(void)
{
    /* Clear local table */
    memset(addr_mapping, 0, sizeof(addr_mapping));

    /* Write empty header to NVDS to clear data */
    nvds_mesh_node_loc_header_t header;
    uint8_t buffer[sizeof(nvds_mesh_node_loc_header_t)];

    header.magic = NVDS_MESH_NODE_LOC_MAGIC;
    header.version = NVDS_MESH_NODE_LOC_VERSION;
    header.node_count = 0;
    header.reserved[0] = 0;
    header.reserved[1] = 0;

    memcpy(buffer, &header, sizeof(header));

    uint8_t status = sifli_nvds_write(SIFLI_NVDS_TYPE_MESH, sizeof(header), buffer);

    if (status == NVDS_OK)
    {
        printk("Cleared node location data from NVDS\n");
    }
    else
    {
        printk("Failed to clear node location from NVDS (status=%d)\n", status);
    }
}

static const uint16_t net_idx;
static const uint16_t app_idx;
static uint16_t self_addr = PROVISIONER_ADDR, node_addr;
static const uint8_t dev_uuid[16] = { 0xdd, 0xdd };
static uint8_t node_uuid[16];

K_SEM_DEFINE(sem_unprov_beacon, 0, 1);
K_SEM_DEFINE(sem_node_added, 0, 1);
K_SEM_DEFINE(sem_button_pressed, 0, 1);
K_SEM_DEFINE(configure_node_sem, 1, 1);  /* Binary semaphore for configure_node sync */

/* Helper for finding node by UUID during timeout cleanup */
static struct bt_mesh_cdb_node *node_by_uuid = NULL;
static uint8_t find_node_by_uuid(struct bt_mesh_cdb_node *node, void *data)
{
    if (memcmp(node->uuid, node_uuid, 16) == 0)
    {
        node_by_uuid = node;
        return BT_MESH_CDB_ITER_STOP;
    }
    return BT_MESH_CDB_ITER_CONTINUE;
}

/* Remove timed-out node from CDB */
static void remove_timed_out_node(void)
{
    node_by_uuid = NULL;
    bt_mesh_cdb_node_foreach(find_node_by_uuid, NULL);
    if (node_by_uuid != NULL)
    {
        char uuid_str[33];
        bin2hex(node_by_uuid->uuid, 16, uuid_str, sizeof(uuid_str));
        printk("Removing timed-out node %s from CDB (addr: 0x%04x)\n",
               uuid_str, node_by_uuid->addr);
        bt_mesh_cdb_node_del(node_by_uuid, true);
    }
    node_by_uuid = NULL;
}

/* Provisioning control flags */
static bool provisioning_enabled = false;
static bool provisioning_thread_running = false;
static struct k_thread provisioning_thread_data;
static K_THREAD_STACK_DEFINE(provisioning_stack, 4096);

/* Synchronization: prevents concurrent configuration operations
 * This is critical because bt_mesh_cfg_cli uses synchronous operations
 * and concurrent calls cause "Another synchronous operation pending" errors
 */
static bool self_configuring = false;

/* Targeted provisioning state */
static struct
{
    bool active;
    uint8_t target_group;   /* 0-3 for A-D */
    uint8_t target_row;     /* 0-2 for Row 1-3 */
    uint8_t provisioned_count;
    uint8_t max_nodes;
    bool waiting_for_config;  /* Flag to keep state during configuration */
} targeted_prov =
{
    .active = false,
    .max_nodes = NODES_PER_ROW,
    .waiting_for_config = false
};

static struct bt_mesh_cfg_cli cfg_cli =
{
};

static void health_current_status(struct bt_mesh_health_cli *cli, uint16_t addr,
                                  uint8_t test_id, uint16_t cid, uint8_t *faults,
                                  size_t fault_count)
{
    size_t i;

    printk("Health Current Status from 0x%04x\n", addr);

    if (!fault_count)
    {
        printk("Health Test ID 0x%02x Company ID 0x%04x: no faults\n",
               test_id, cid);
        return;
    }

    printk("Health Test ID 0x%02x Company ID 0x%04x Fault Count %zu:\n",
           test_id, cid, fault_count);

    for (i = 0; i < fault_count; i++)
    {
        printk("\t0x%02x\n", faults[i]);
    }
}

static struct bt_mesh_health_cli health_cli =
{
    .current_status = health_current_status,
};

/* Generic OnOff Client model - no callbacks needed for sending commands */
static const struct bt_mesh_model_op gen_onoff_cli_op[] =
{
    BT_MESH_MODEL_OP_END,
};

static const struct bt_mesh_model root_models[] =
{
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_CFG_CLI(&cfg_cli),
    BT_MESH_MODEL_HEALTH_CLI(&health_cli),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op, NULL, NULL),
};

static const struct bt_mesh_elem elements[] =
{
    BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp mesh_comp =
{
    .cid = BT_COMP_ID_LF,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

static void setup_cdb(void)
{
    struct bt_mesh_cdb_app_key *key;
    uint8_t app_key[16];
    int err;

    key = bt_mesh_cdb_app_key_alloc(net_idx, app_idx);
    if (key == NULL)
    {
        printk("Failed to allocate app-key 0x%04x\n", app_idx);
        return;
    }

    bt_rand(app_key, 16);

    err = bt_mesh_cdb_app_key_import(key, 0, app_key);
    if (err)
    {
        printk("Failed to import appkey into cdb. Err:%d\n", err);
        return;
    }

    if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
        bt_mesh_cdb_app_key_store(key);
    }
}

static void configure_self(struct bt_mesh_cdb_node *self)
{
    struct bt_mesh_cdb_app_key *key;
    uint8_t app_key[16];
    uint8_t status = 0;
    int err;

    printk("Configuring self...\n");

    /* Set synchronization flag to prevent provisioning thread from
     * configuring other nodes while we're configuring self.
     * This prevents "Another synchronous operation pending" errors.
     */
    self_configuring = true;

    key = bt_mesh_cdb_app_key_get(app_idx);
    if (key == NULL)
    {
        printk("No app-key 0x%04x\n", app_idx);
        goto cleanup;
    }

    err = bt_mesh_cdb_app_key_export(key, 0, app_key);
    if (err)
    {
        printk("Failed to export appkey from cdb. Err:%d\n", err);
        goto cleanup;
    }

    /* Add Application Key */
    err = bt_mesh_cfg_cli_app_key_add(self->net_idx, self->addr, self->net_idx, app_idx,
                                      app_key, &status);
    if (err || status)
    {
        printk("Failed to add app-key (err %d, status %d)\n", err,
               status);
        goto cleanup;
    }

    /* Bind Health Client model */
    err = bt_mesh_cfg_cli_mod_app_bind(self->net_idx, self->addr, self->addr, app_idx,
                                       BT_MESH_MODEL_ID_HEALTH_CLI, &status);
    if (err || status)
    {
        printk("Failed to bind Health app-key (err %d, status %d)\n", err,
               status);
        goto cleanup;
    }

    /* Bind Generic OnOff Client model */
    err = bt_mesh_cfg_cli_mod_app_bind(self->net_idx, self->addr, self->addr, app_idx,
                                       BT_MESH_MODEL_ID_GEN_ONOFF_CLI, &status);
    if (err || status)
    {
        printk("Failed to bind OnOff app-key (err %d, status %d)\n", err,
               status);
        goto cleanup;
    }

    /* CRITICAL: Also bind the local OnOff Client model to app_idx
     * This is needed for the provisioner to send messages using bt_mesh_model_send()
     * bt_mesh_cfg_cli_mod_app_bind only binds remote models, not local ones.
     */
    struct bt_mesh_model *local_onoff_cli = (struct bt_mesh_model *)&root_models[3];
    if (local_onoff_cli->keys_cnt > 0)
    {
        local_onoff_cli->keys[0] = app_idx;
        printk("Local OnOff Client bound to app_idx 0x%04x\n", app_idx);
    }

    atomic_set_bit(self->flags, BT_MESH_CDB_NODE_CONFIGURED);

    if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
        bt_mesh_cdb_node_store(self);
    }

    printk("Configuration complete\n");

cleanup:
    /* Clear synchronization flag to allow provisioning thread to continue */
    self_configuring = false;
}

static void configure_node(struct bt_mesh_cdb_node *node)
{
    NET_BUF_SIMPLE_DEFINE(buf, BT_MESH_RX_SDU_MAX);
    struct bt_mesh_comp_p0_elem elem;
    struct bt_mesh_cdb_app_key *key;
    uint8_t app_key[16];
    struct bt_mesh_comp_p0 comp;
    uint8_t status;
    int err, elem_addr;
    int retry_count = 0;
    const int max_retries = 5;

    /* Take semaphore to prevent concurrent configure_node calls.
     * This can happen when:
     * 1. node_added callback triggers configure_node
     * 2. Timeout triggers configure_node from provisioning thread
     * Only one should proceed.
     */
    if (k_sem_take(&configure_node_sem, K_NO_WAIT) != 0)
    {
        printk("Skipping configure_node 0x%04x (already in progress)\n", node->addr);
        return;
    }

    printk("Configuring node 0x%04x...\n", node->addr);

    /* Wait for node to be ready after provisioning */
    k_sleep(K_MSEC(500));

    key = bt_mesh_cdb_app_key_get(app_idx);
    if (key == NULL)
    {
        printk("No app-key 0x%04x\n", app_idx);
        goto cleanup;
    }

    err = bt_mesh_cdb_app_key_export(key, 0, app_key);
    if (err)
    {
        printk("Failed to export appkey from cdb. Err:%d\n", err);
        goto cleanup;
    }

    /* Add Application Key with retry */
    while (retry_count < max_retries)
    {
        err = bt_mesh_cfg_cli_app_key_add(net_idx, node->addr, net_idx, app_idx, app_key, &status);
        if (!err && !status)
        {
            break;  /* Success */
        }

        retry_count++;
        if (retry_count < max_retries)
        {
            printk("Add app-key failed (err %d status %d), retrying (%d/%d)...\n",
                   err, status, retry_count, max_retries);
            k_sleep(K_SECONDS(2));  /* Wait before retry */
        }
    }

    if (err || status)
    {
        printk("Failed to add app-key after %d retries (err %d status %d)\n",
               max_retries, err, status);
        goto cleanup;
    }

    /* Get the node's composition data and bind all models to the appkey */
    retry_count = 0;
    while (retry_count < max_retries)
    {
        err = bt_mesh_cfg_cli_comp_data_get(net_idx, node->addr, 0, &status, &buf);
        if (!err && !status)
        {
            break;  /* Success */
        }

        retry_count++;
        if (retry_count < max_retries)
        {
            printk("Get composition data failed (err %d status %d), retrying (%d/%d)...\n",
                   err, status, retry_count, max_retries);
            k_sleep(K_SECONDS(2));
        }
    }

    if (err || status)
    {
        printk("Failed to get Composition data after %d retries (err %d, status: %d)\n",
               max_retries, err, status);
        goto cleanup;
    }

    err = bt_mesh_comp_p0_get(&comp, &buf);
    if (err)
    {
        printk("Unable to parse composition data (err: %d)\n", err);
        goto cleanup;
    }

    elem_addr = node->addr;
    while (bt_mesh_comp_p0_elem_pull(&comp, &elem))
    {
        printk("Element @ 0x%04x: %u + %u models\n", elem_addr,
               elem.nsig, elem.nvnd);
        for (int i = 0; i < elem.nsig; i++)
        {
            uint16_t id = bt_mesh_comp_p0_elem_mod(&elem, i);

            /* Skip DevKey-based foundation models (they use DevKey, not AppKey):
             * 0x0000 = Config Server
             * 0x0001 = Config Client
             * 0x0002 = Health Server
             * 0x0003 = Health Client
             */
            if (id <= BT_MESH_MODEL_ID_HEALTH_CLI)
            {
                printk("Skipping DevKey model 0x%03x:%04x (no AppKey binding needed)\n",
                       elem_addr, id);
                continue;
            }

            printk("Binding AppKey to model 0x%03x:%04x\n",
                   elem_addr, id);

            /* Retry binding for each model */
            int bind_retry = 0;
            while (bind_retry < 3)
            {
                err = bt_mesh_cfg_cli_mod_app_bind(net_idx, node->addr, elem_addr, app_idx,
                                                   id, &status);
                if (!err && !status)
                {
                    break;  /* Success */
                }

                bind_retry++;
                if (bind_retry < 3)
                {
                    printk("  Bind failed (err: %d, status: %d), retrying...\n", err, status);
                    k_sleep(K_MSEC(500));
                }
            }

            if (err || status)
            {
                printk("  Failed to bind after retries (err: %d, status: %d)\n", err, status);
            }
        }

        for (int i = 0; i < elem.nvnd; i++)
        {
            struct bt_mesh_mod_id_vnd id =
                bt_mesh_comp_p0_elem_mod_vnd(&elem, i);

            printk("Binding AppKey to model 0x%03x:%04x:%04x\n",
                   elem_addr, id.company, id.id);

            err = bt_mesh_cfg_cli_mod_app_bind_vnd(net_idx, node->addr, elem_addr,
                                                   app_idx, id.id, id.company, &status);
            if (err || status)
            {
                printk("Failed (err: %d, status: %d)\n", err,
                       status);
            }
        }

        elem_addr++;
    }

    /* Configure subscriptions for targeted provisioning */
    if (targeted_prov.active)
    {
        uint16_t group_total_addr;
        uint16_t group_row_addr;

        /* Calculate group addresses based on target */
        group_total_addr = GROUP_A_TOTAL + targeted_prov.target_group;
        group_row_addr = GET_GROUP_ROW_ADDR(targeted_prov.target_group, targeted_prov.target_row);

        printk("Configuring subscriptions for node:\n");
        printk("  Total Group Address: 0x%04x\n", group_total_addr);
        printk("  Row Group Address: 0x%04x\n", group_row_addr);

        /* Wait for settings store to complete before sending config */
        k_sleep(K_MSEC(800));

        /* Subscribe to total group address with retry */
        int retry = 0;
        do
        {
            k_sleep(K_MSEC(200));
            err = bt_mesh_cfg_cli_mod_sub_add(net_idx, node->addr, node->addr,
                                              group_total_addr,
                                              BT_MESH_MODEL_ID_GEN_ONOFF_SRV, &status);
            if (err)
            {
                if (err == -16 || err == -116)    /* EAGAIN: sync operation pending */
                {
                    printk("Total group: sync pending, retrying (%d)...\n", retry + 1);
                    k_sleep(K_MSEC(500));
                }
            }
            retry++;
        }
        while ((err == -16 || err == -116) && retry < 5);

        if (err || status)
        {
            printk("Failed to subscribe to total group (err: %d, status: %d)\n", err, status);
        }
        else
        {
            printk("✓ Subscribed to total group 0x%04x\n", group_total_addr);
        }

        /* Wait for node to process previous request */
        k_sleep(K_MSEC(1000));

        /* Subscribe to row group address with retry */
        retry = 0;
        do
        {
            k_sleep(K_MSEC(300));
            err = bt_mesh_cfg_cli_mod_sub_add(net_idx, node->addr, node->addr,
                                              group_row_addr,
                                              BT_MESH_MODEL_ID_GEN_ONOFF_SRV, &status);
            if (err)
            {
                if (err == -16 || err == -116)    /* EAGAIN or ETIMEDOUT: retry */
                {
                    printk("Row group: retrying (%d)...\n", retry + 1);
                    k_sleep(K_MSEC(500));
                }
            }
            retry++;
        }
        while ((err == -16 || err == -116) && retry < 5);

        if (err || status)
        {
            printk("Failed to subscribe to row group (err: %d, status: %d)\n", err, status);
        }
        else
        {
            printk("✓ Subscribed to row group 0x%04x\n", group_row_addr);
        }

        /* CRITICAL: Wait for node to store subscriptions to flash */
        /* BT_MESH_STORE_TIMEOUT is 2 seconds, so we need to wait at least 2.5s */
        printk("Waiting for node to store subscriptions to flash...\n");
        k_sleep(K_MSEC(2500));
        printk("Node storage complete\n");
    }

    atomic_set_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED);

    /* Clear the configuring flag */
    for (int i = 0; i < MAX_MAPPED_NODES; i++)
    {
        if (addr_mapping[i].valid && addr_mapping[i].actual_addr == node->addr)
        {
            addr_mapping[i].configuring = false;
            printk("Configuration flag cleared for node 0x%04x\n", node->addr);
            break;
        }
    }

    if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
        bt_mesh_cdb_node_store(node);
    }

    printk("Configuration complete\n");

cleanup:
    /* Release semaphore to allow next configure_node call */
    k_sem_give(&configure_node_sem);
}

static void unprovisioned_beacon(uint8_t uuid[16],
                                 bt_mesh_prov_oob_info_t oob_info,
                                 uint32_t *uri_hash)
{
    /* Only process beacon if provisioning is enabled */
    if (!provisioning_enabled)
    {
        return;
    }

    memcpy(node_uuid, uuid, 16);
    k_sem_give(&sem_unprov_beacon);
}

static void node_added(uint16_t idx, uint8_t uuid[16], uint16_t addr, uint8_t num_elem)
{
    node_addr = addr;

    /* Update targeted provisioning counter */
    if (targeted_prov.active)
    {
        targeted_prov.provisioned_count++;
        printk("Targeted provisioning: %d/%d nodes provisioned\n",
               targeted_prov.provisioned_count, targeted_prov.max_nodes);

        /* Register address mapping for this node */
        uint8_t col = targeted_prov.provisioned_count - 1;  /* 0-based column index */
        register_address_mapping(addr,
                                 targeted_prov.target_group,
                                 targeted_prov.target_row,
                                 col);

        /* Mark that we're waiting for configuration to complete */
        targeted_prov.waiting_for_config = true;

        /* Check if row is fully provisioned */
        if (targeted_prov.provisioned_count >= targeted_prov.max_nodes)
        {
            printk("✓ All nodes provisioned! Waiting for configuration...\n");
            /* Don't stop yet - wait for check_unconfigured to finish */
        }
    }

    k_sem_give(&sem_node_added);
}

/* Callback to reassign unicast address for a node */
static int reassign_node_address(struct bt_mesh_cdb_node *node, uint16_t new_addr)
{
    struct bt_mesh_cdb_node *existing;
    int err;

    /* Check if new address is already in use */
    existing = bt_mesh_cdb_node_get(new_addr);
    if (existing && existing != node)
    {
        printk("Error: Address 0x%04x already in use by another node\n", new_addr);
        return -EADDRINUSE;
    }

    /* Update the node's address in CDB */
    node->addr = new_addr;

    /* Store the updated node information */
    if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
        bt_mesh_cdb_node_store(node);
    }

    printk("✓ Node address reassigned: 0x%04x -> 0x%04x\n", node_addr, new_addr);
    return 0;
}

static const struct bt_mesh_prov prov =
{
    .uuid = dev_uuid,
    .unprovisioned_beacon = unprovisioned_beacon,
    .node_added = node_added,
};

static int bt_ready(void)
{
    uint8_t net_key[16], dev_key[16];
    int err;

    err = bt_mesh_init(&prov, &mesh_comp);
    if (err)
    {
        printk("Initializing mesh failed (err %d)\n", err);
        return err;
    }

    printk("Mesh initialized\n");

    if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
        printk("Loading stored settings...\n");
        int ret = settings_load();
        printk("settings_load() returned %d\n", ret);
        if (ret != 0)
        {
            printk("WARNING: Settings load failed! Check FlashDB initialization\n");
        }
    }

    /* Check if we are already provisioned (from stored settings) */
    if (bt_mesh_is_provisioned())
    {
        printk("✓ Already provisioned from stored settings\n");

        /* Check if CDB exists and is valid */
        if (atomic_test_bit(bt_mesh_cdb.flags, BT_MESH_CDB_VALID))
        {
            printk("✓ CDB is valid, loading stored nodes...\n");

            /* Re-configure self (bind AppKey to OnOff Client) after reboot restore */
            struct bt_mesh_cdb_node *self_node = bt_mesh_cdb_node_get(self_addr);
            if (self_node)
            {
                printk("Re-configuring self...\n");

                /* Debug: Check network key status */
                struct bt_mesh_subnet *subnet = bt_mesh_subnet_get(self_node->net_idx);
                if (subnet)
                {
                    printk("  ✓ Network key found (net_idx: 0x%04x)\n", self_node->net_idx);
                }
                else
                {
                    printk("  ✗ Network key NOT found (net_idx: 0x%04x)\n", self_node->net_idx);
                }

                /* CRITICAL: OnOff Client needs AppKey binding to send messages!
                 * Config Client uses DevKey for config messages, but
                 * Generic OnOff Client uses AppKey for application messages.
                 */
                printk("  Binding AppKey to OnOff Client...\n");
                configure_self(self_node);
            }
            else
            {
                printk("  ⚠ Self node not found in CDB\n");
            }

            /* Count and display loaded nodes */
            int node_count = 0;
            for (int i = 0; i < ARRAY_SIZE(bt_mesh_cdb.nodes); i++)
            {
                if (bt_mesh_cdb.nodes[i].addr != BT_MESH_ADDR_UNASSIGNED)
                {
                    node_count++;
                    char uuid_hex_str[32 + 1];
                    bin2hex(bt_mesh_cdb.nodes[i].uuid, 16, uuid_hex_str, sizeof(uuid_hex_str));
                    printk("  [%d] Addr: 0x%04x, UUID: %s, Configured: %s\n",
                           node_count,
                           bt_mesh_cdb.nodes[i].addr,
                           uuid_hex_str,
                           atomic_test_bit(bt_mesh_cdb.nodes[i].flags, BT_MESH_CDB_NODE_CONFIGURED) ? "Yes" : "No");
                }
            }
            printk("Total nodes loaded: %d\n", node_count);

            /* Load node location data from NVDS for persistent recovery */
            printk("\n=== Loading Node Location Data from NVDS ===\n");
            int loc_count = mesh_location_load();
            if (loc_count > 0)
            {
                printk("✓ Node location data restored from NVDS\n");
            }
            else if (loc_count == 0)
            {
                printk("  No saved location data (fresh network)\n");
            }
            else
            {
                printk("  Failed to load location data from NVDS\n");
            }
            printk("=============================================\n");
        }
        else
        {
            printk("⚠ CDB is not valid, creating new CDB...\n");
            bt_rand(net_key, 16);
            err = bt_mesh_cdb_create(net_key);
            if (err)
            {
                printk("Failed to create CDB (err %d)\n", err);
                return err;
            }
            setup_cdb();
        }
    }
    else
    {
        printk("Not provisioned yet, initializing as new provisioner...\n");

        /* First time: create new CDB and provision self */
        bt_rand(net_key, 16);
        err = bt_mesh_cdb_create(net_key);
        if (err)
        {
            printk("Failed to create CDB (err %d)\n", err);
            return err;
        }
        printk("Created new CDB\n");
        setup_cdb();

        bt_rand(dev_key, 16);
        err = bt_mesh_provision(net_key, BT_MESH_NET_PRIMARY, 0, 0, self_addr, dev_key);
        if (err)
        {
            printk("Provisioning failed (err %d)\n", err);
            return err;
        }
        printk("Provisioning completed\n");

        /* CRITICAL: Configure self after provisioning!
         * This binds AppKey to the OnOff Client model so we can send messages.
         */
        struct bt_mesh_cdb_node *self_node = bt_mesh_cdb_node_get(self_addr);
        if (self_node)
        {
            printk("Configuring self (bind AppKey to OnOff Client)...\n");
            configure_self(self_node);
        }
        else
        {
            printk("Warning: Self node not found in CDB after provisioning\n");
        }

        /* Force immediate storage and wait for completion */
        if (IS_ENABLED(CONFIG_BT_SETTINGS))
        {
            printk("Storing provisioner CDB and network info...\n");
            k_sleep(K_MSEC(600));  // Wait for async storage to complete
            printk("Provisioner data stored\n");
        }
    }

    return 0;
}

/* Check if a node is unconfigured and configure it */
static uint8_t check_unconfigured(struct bt_mesh_cdb_node *node, void *data)
{
    /* CRITICAL: Skip if we're currently configuring self.
     * This prevents "Another synchronous operation pending" errors
     * when configure_self is running in the main thread while
     * provisioning thread tries to configure other nodes.
     */
    if (self_configuring)
    {
        return BT_MESH_CDB_ITER_CONTINUE;
    }

    if (!atomic_test_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED))
    {
        /* In targeted provisioning mode, only configure nodes that were
         * successfully added through node_added callback.
         * If node_added timed out, the node won't be in addr_mapping,
         * so we skip configuration to avoid errors.
         */
        if (targeted_prov.active)
        {
            bool found_in_mapping = false;
            for (int i = 0; i < MAX_MAPPED_NODES; i++)
            {
                if (addr_mapping[i].valid && addr_mapping[i].actual_addr == node->addr)
                {
                    found_in_mapping = true;
                    break;
                }
            }
            if (!found_in_mapping)
            {
                /* Node not in mapping table - likely add node timeout, skip */
                return BT_MESH_CDB_ITER_CONTINUE;
            }
        }

        /* Skip if this node is already being configured */
        for (int i = 0; i < MAX_MAPPED_NODES; i++)
        {
            if (addr_mapping[i].valid && addr_mapping[i].actual_addr == node->addr)
            {
                if (addr_mapping[i].configuring)
                {
                    /* Already configuring this node, skip */
                    return BT_MESH_CDB_ITER_CONTINUE;
                }

                /* Mark as configuring to prevent duplicate configuration attempts */
                addr_mapping[i].configuring = true;
                printk("Starting configuration for node 0x%04x\n", node->addr);
                break;
            }
        }

        if (node->addr == self_addr)
        {
            configure_self(node);
        }
        else
        {
            configure_node(node);
        }
    }

    return BT_MESH_CDB_ITER_CONTINUE;
}

/* Better approach: Check completion based on provisioned count vs configured count */
static bool check_targeted_provisioning_complete(void)
{
    if (!targeted_prov.active || !targeted_prov.waiting_for_config)
    {
        return false;
    }

    /*
     * Strategy: We should continue provisioning until we have successfully
     * provisioned AND configured max_nodes nodes for the CURRENT TARGET ROW.
     *
     * Don't stop just because some nodes are configured - keep scanning for more!
     */

    /* Count how many nodes we've actually provisioned for CURRENT TARGET ROW */
    int mapped_count = 0;
    int configured_count = 0;

    for (int i = 0; i < MAX_MAPPED_NODES; i++)
    {
        /* Only count nodes belonging to the current target group and row */
        if (addr_mapping[i].valid &&
                addr_mapping[i].group == targeted_prov.target_group &&
                addr_mapping[i].row == targeted_prov.target_row)
        {
            mapped_count++;

            /* Check if this mapped node is configured */
            struct bt_mesh_cdb_node *node = bt_mesh_cdb_node_get(addr_mapping[i].actual_addr);
            if (node && atomic_test_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED))
            {
                configured_count++;
            }
        }
    }

    printk("Checking completion [Group %c Row %d]: %d mapped, %d configured / %d expected\n",
           'A' + targeted_prov.target_group, targeted_prov.target_row + 1,
           mapped_count, configured_count, targeted_prov.max_nodes);

    /* Only complete when we have provisioned AND configured all expected nodes for current row */
    return (mapped_count >= targeted_prov.max_nodes && configured_count >= targeted_prov.max_nodes);
}

/* Callback wrapper for compatibility with foreach if needed, but direct call is preferred now */
static uint8_t check_all_configured(struct bt_mesh_cdb_node *node, void *data)
{
    /* This function is now largely obsolete as check_targeted_provisioning_complete
     * does the precise checking based on the mapping table.
     * Kept for potential other uses or legacy compatibility if called elsewhere. */
    return BT_MESH_CDB_ITER_CONTINUE;
}

/* Callback for listing nodes with detailed information */
static uint8_t list_nodes_cb(struct bt_mesh_cdb_node *node, void *data)
{
    char uuid_hex_str[32 + 1];

    bin2hex(node->uuid, 16, uuid_hex_str, sizeof(uuid_hex_str));

    /* Try to determine group/row from address */
    uint16_t addr = node->addr;
    const char *location = "Unknown";
    char location_buf[48];  /* Increased size for mapping info */

    if (addr == PROVISIONER_ADDR)
    {
        location = "Provisioner";
    }
    else
    {
        /* Check if we have a mapping for this actual address */
        uint8_t group, row, col;
        if (get_node_position(addr, &group, &row, &col))
        {
            /* Found in mapping table - show logical address */
            uint16_t logical_addr = GET_NODE_ADDR(group, row, col);
            snprintf(location_buf, sizeof(location_buf),
                     "Group %c Row %d Col %d (0x%04x)",
                     'A' + group, row + 1, col + 1, logical_addr);
            location = location_buf;
        }
        else if (addr >= UNICAST_ADDR_BASE && addr < (UNICAST_ADDR_BASE + TOTAL_NODES))
        {
            /* In our address pool but not mapped yet */
            uint16_t offset = addr - UNICAST_ADDR_BASE;
            group = offset / NODES_PER_GROUP;
            row = (offset % NODES_PER_GROUP) / NODES_PER_ROW;
            col = offset % NODES_PER_ROW;

            snprintf(location_buf, sizeof(location_buf), "Group %c Row %d Col %d",
                     'A' + group, row + 1, col + 1);
            location = location_buf;
        }
        else if (addr > 0 && addr < UNICAST_ADDR_BASE)
        {
            /* Auto-assigned address outside our pool and not mapped */
            snprintf(location_buf, sizeof(location_buf), "Auto-assigned (0x%04x)", addr);
            location = location_buf;
        }
    }

    printk("0x%04x   %s   %-35s   %s\n",
           node->addr,
           uuid_hex_str,
           location,
           atomic_test_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED) ? "Yes" : "No");

    return BT_MESH_CDB_ITER_CONTINUE;
}

/* Command: List all configured nodes */
static void mesh_list_nodes(uint8_t argc, char **argv)
{
    printk("\n=== Configured Nodes ===\n");
    printk("Addr     UUID                                 Location                    Configured\n");
    printk("-------- ------------------------------------ --------------------------- ----------\n");

    bt_mesh_cdb_node_foreach(list_nodes_cb, NULL);

    printk("========================\n\n");
}
MSH_CMD_EXPORT(mesh_list_nodes, list all mesh nodes in CDB);

/* Command: Check storage status */
static void mesh_check_storage(uint8_t argc, char **argv)
{
    printk("\n=== Storage Status Check ===\n");

    /* Check if CDB is valid */
    if (atomic_test_bit(bt_mesh_cdb.flags, BT_MESH_CDB_VALID))
    {
        printk("✓ CDB is valid\n");
        printk("  IV Index: 0x%08x\n", bt_mesh_cdb.iv_index);
        printk("  Lowest Available Addr: %d\n", bt_mesh_cdb.lowest_avail_addr);

        /* Count subnets */
        int subnet_count = 0;
        for (int i = 0; i < ARRAY_SIZE(bt_mesh_cdb.subnets); i++)
        {
            if (bt_mesh_cdb.subnets[i].net_idx != BT_MESH_KEY_UNUSED)
            {
                subnet_count++;
            }
        }
        printk("  Subnets: %d\n", subnet_count);

        /* Count nodes */
        int node_count = 0;
        int configured_count = 0;
        for (int i = 0; i < ARRAY_SIZE(bt_mesh_cdb.nodes); i++)
        {
            if (bt_mesh_cdb.nodes[i].addr != BT_MESH_ADDR_UNASSIGNED)
            {
                node_count++;
                if (atomic_test_bit(bt_mesh_cdb.nodes[i].flags, BT_MESH_CDB_NODE_CONFIGURED))
                {
                    configured_count++;
                }
            }
        }
        printk("  Nodes: %d (Configured: %d)\n", node_count, configured_count);

        /* Count app keys */
        int app_key_count = 0;
        for (int i = 0; i < ARRAY_SIZE(bt_mesh_cdb.app_keys); i++)
        {
            if (bt_mesh_cdb.app_keys[i].net_idx != BT_MESH_KEY_UNUSED)
            {
                app_key_count++;
            }
        }
        printk("  App Keys: %d\n", app_key_count);
    }
    else
    {
        printk("✗ CDB is NOT valid (no stored data)\n");
    }

    /* Check if self is provisioned */
    if (bt_mesh_is_provisioned())
    {
        printk("✓ Self is provisioned (Addr: 0x%04x)\n", bt_mesh_primary_addr());
        printk("✓ Self is provisioned (Addr:)\n");
    }
    else
    {
        printk("✗ Self is NOT provisioned\n");
    }

    printk("============================\n\n");
}
MSH_CMD_EXPORT(mesh_check_storage, check mesh storage status);

static void button_pressed(uint32_t pins)
{
    k_sem_give(&sem_button_pressed);
}

/* Provisioning thread function */
static void provisioning_thread(void *p1, void *p2, void *p3)
{
    char uuid_hex_str[32 + 1];
    int err;

    printk("Provisioning thread started\n");
    provisioning_thread_running = true;

    /* Address pool is already initialized in mesh_start_target */
    if (targeted_prov.active)
    {
        printk("Using address pool starting from 0x%04x\n", addr_pool.next_addr);
    }

    while (provisioning_enabled)
    {
        /* Check if targeted provisioning is complete before starting new scan */
        if (targeted_prov.active && targeted_prov.waiting_for_config)
        {
            if (check_targeted_provisioning_complete())
            {
                printk("All targeted nodes configured! Stopping provisioning.\n");
                break;
            }
        }

        k_sem_reset(&sem_unprov_beacon);
        k_sem_reset(&sem_node_added);
        bt_mesh_cdb_node_foreach(check_unconfigured, NULL);

        if (targeted_prov.active)
        {
            printk("Scanning for unprovisioned devices (Group %c Row %d, max %d nodes)...\n",
                   'A' + targeted_prov.target_group,
                   targeted_prov.target_row + 1,
                   targeted_prov.max_nodes);
        }
        else
        {
            printk("Scanning for unprovisioned devices...\n");
        }

        err = k_sem_take(&sem_unprov_beacon, K_SECONDS(10));
        if (err == -EAGAIN || !provisioning_enabled)
        {
            if (!provisioning_enabled)
            {
                break;
            }

            /* Check if targeted provisioning is complete on each timeout */
            if (targeted_prov.active && targeted_prov.waiting_for_config)
            {
                if (check_targeted_provisioning_complete())
                {
                    printk("All targeted nodes configured! Stopping provisioning.\n");
                    break;
                }
            }

            continue;
        }

        bin2hex(node_uuid, 16, uuid_hex_str, sizeof(uuid_hex_str));

        /* In targeted mode, skip button press and auto-provision */
        if (!targeted_prov.active)
        {
            k_sem_reset(&sem_button_pressed);
            printk("Device %s detected, press button 1 to provision.\n", uuid_hex_str);
            err = k_sem_take(&sem_button_pressed, K_SECONDS(30));
            if (err == -EAGAIN || !provisioning_enabled)
            {
                if (!provisioning_enabled)
                {
                    printk("Provisioning stopped by user\n");
                    break;
                }
                printk("Timed out, button 1 wasn't pressed in time.\n");
                continue;
            }
        }
        else
        {
            printk("Device %s detected (auto-provisioning)...\n", uuid_hex_str);
        }

        /* CRITICAL: In targeted mode, check if we've already reached max_nodes */
        if (targeted_prov.active && targeted_prov.provisioned_count >= targeted_prov.max_nodes)
        {
            printk("Max nodes (%d) already reached, ignoring device %s\n",
                   targeted_prov.max_nodes, uuid_hex_str);
            continue;
        }

        printk("Provisioning %s\n", uuid_hex_str);
        err = bt_mesh_provision_adv(node_uuid, net_idx, 0, 0);
        if (err < 0)
        {
            printk("Provisioning failed (err %d)\n", err);
            continue;
        }

        printk("Waiting for node to be added...\n");
        err = k_sem_take(&sem_node_added, K_SECONDS(30));
        if (err == -EAGAIN)
        {
            printk("Timeout waiting for node to be added\n");
            /* Remove the timed-out node from CDB.
             * The node was added to CDB by prov_node_added callback
             * but node_added callback was never called due to timeout.
             */
            remove_timed_out_node();
            continue;
        }

        printk("Added node 0x%04x\n", node_addr);

        /* In targeted mode, reassign address if needed */
        if (targeted_prov.active)
        {
            uint16_t expected_addr = addr_pool.next_addr;

            if (node_addr != expected_addr)
            {
                printk("Note: Node got address 0x%04x, expected 0x%04x\n",
                       node_addr, expected_addr);
                printk("This is normal - Zephyr Mesh auto-assigns addresses.\n");
                printk("The node will still work correctly with group subscriptions.\n");
            }

            /* Move to next address in pool */
            addr_pool.next_addr++;
        }

        /* Check if we should stop in targeted mode */
        if (targeted_prov.active && !provisioning_enabled)
        {
            printk("Target row fully provisioned, exiting provisioning thread\n");
            break;
        }
    }

    provisioning_thread_running = false;

    /* Reset targeted provisioning state only if not waiting for config */
    if (targeted_prov.active && !targeted_prov.waiting_for_config)
    {
        printk("Targeted provisioning completed: %d nodes provisioned\n",
               targeted_prov.provisioned_count);
        targeted_prov.active = false;
        targeted_prov.provisioned_count = 0;
    }

    printk("Provisioning thread exited\n");
}

/* Command: Start targeted provisioning for a specific row */
static void mesh_start_target(uint8_t argc, char **argv)
{
    uint8_t group, row;

    if (argc < 3)
    {
        printk("Usage: mesh_start_target <group> <row>\n");
        printk("  group: 0-3 (A-D)\n");
        printk("  row:   0-2 (Row 1-3)\n");
        printk("\nExample:\n");
        printk("  mesh_start_target 0 0  - Start provisioning Group A Row 1\n");
        printk("  mesh_start_target 2 1  - Start provisioning Group C Row 2\n");
        return;
    }

    if (provisioning_enabled)
    {
        printk("Provisioning is already enabled. Stop it first with 'mesh_prov_stop'\n");
        return;
    }

    group = strtol(argv[1], NULL, 0);
    row = strtol(argv[2], NULL, 0);

    if (group >= GROUPS_TOTAL || row >= ROWS_PER_GROUP)
    {
        printk("Error: Invalid group (%d) or row (%d)\n", group, row);
        printk("Valid range: group 0-3, row 0-2\n");
        return;
    }

    /* Setup targeted provisioning state */
    targeted_prov.active = true;
    targeted_prov.target_group = group;
    targeted_prov.target_row = row;
    targeted_prov.provisioned_count = 0;
    targeted_prov.max_nodes = NODES_PER_ROW;
    targeted_prov.waiting_for_config = false;  /* Reset waiting flag for new session */

    /* Clear address mapping table for new provisioning session */
    memset(addr_mapping, 0, sizeof(addr_mapping));
    printk("Address mapping table cleared for new session\n");

    /* Load existing node locations from NVDS to preserve previously provisioned nodes */
    int loaded_count = mesh_location_load();
    if (loaded_count > 0)
    {
        printk("Preserved %d existing node locations from NVDS\n", loaded_count);
    }

    /* Reset address pool */
    addr_pool.next_addr = GET_NODE_ADDR(group, row, 0);
    printk("Address pool initialized: starting from 0x%04x\n", addr_pool.next_addr);

    printk("\n=== Starting Targeted Provisioning ===\n");
    printk("Target: Group %c Row %d\n", 'A' + group, row + 1);
    printk("Expected nodes: %d\n", NODES_PER_ROW);
    printk("Address pool: 0x%04x - 0x%04x\n",
           GET_NODE_ADDR(group, row, 0),
           GET_NODE_ADDR(group, row, NODES_PER_ROW - 1));
    printk("Group subscriptions:\n");
    printk("  Total Group: 0x%04x\n", GROUP_A_TOTAL + group);
    printk("  Row Group:   0x%04x\n", GET_GROUP_ROW_ADDR(group, row));
    printk("=====================================\n\n");

    provisioning_enabled = true;

    /* Create and start provisioning thread */
    k_thread_create(&provisioning_thread_data, provisioning_stack,
                    K_THREAD_STACK_SIZEOF(provisioning_stack),
                    provisioning_thread, NULL, NULL, NULL,
                    K_PRIO_COOP(7), 0, K_FOREVER);
    k_thread_name_set(&provisioning_thread_data, "Mesh Prov");

    /* Start the thread */
    k_thread_start(&provisioning_thread_data);
}
MSH_CMD_EXPORT(mesh_start_target, start targeted provisioning for specific group and row);

/* Command: Start general provisioning (legacy mode) */
static void mesh_prov_start(uint8_t argc, char **argv)
{
    if (provisioning_enabled)
    {
        printk("Provisioning is already enabled\n");
        return;
    }

    printk("Starting general provisioning mode...\n");

    /* Reset targeted provisioning state */
    targeted_prov.active = false;
    targeted_prov.provisioned_count = 0;

    provisioning_enabled = true;

    /* Create and start provisioning thread */
    k_thread_create(&provisioning_thread_data, provisioning_stack,
                    K_THREAD_STACK_SIZEOF(provisioning_stack),
                    provisioning_thread, NULL, NULL, NULL,
                    K_PRIO_COOP(7), 0, K_FOREVER);
    k_thread_name_set(&provisioning_thread_data, "Mesh Prov");

    /* Start the thread */
    k_thread_start(&provisioning_thread_data);

    printk("General provisioning started. Use 'mesh_prov_stop' to stop.\n");
}
MSH_CMD_EXPORT(mesh_prov_start, start general mesh provisioning mode);

/* Command: Stop provisioning */
static void mesh_prov_stop(uint8_t argc, char **argv)
{
    if (!provisioning_enabled)
    {
        printk("Provisioning is not enabled\n");
        return;
    }

    printk("Stopping provisioning mode...\n");
    provisioning_enabled = false;

    /* Wake up the thread if it's blocked on semaphores */
    k_sem_give(&sem_unprov_beacon);
    k_sem_give(&sem_button_pressed);
    k_sem_give(&sem_node_added);

    /* Wait for thread to exit with timeout (5 seconds) */
    int wait_count = 0;
    while (provisioning_thread_running && wait_count < 50)
    {
        k_sleep(K_MSEC(100));
        wait_count++;
    }

    if (provisioning_thread_running)
    {
        printk("Warning: Provisioning thread did not exit in time\n");
        printk("Forcing thread termination...\n");
        /* As a last resort, abort the thread */
        k_thread_abort(&provisioning_thread_data);
        provisioning_thread_running = false;
    }
    else
    {
        printk("Provisioning stopped\n");
    }
}
MSH_CMD_EXPORT(mesh_prov_stop, stop mesh provisioning mode);

/* Command: Button press simulation for provisioning confirmation */
static void mesh_btn(uint8_t argc, char **argv)
{
    if (argc > 1)
    {
        uint8_t sel = strtol(argv[1], NULL, 0);
        if (sel > BTN_COUNT || sel == 0)
            printk("Usage: mesh_btn [1]");
        else
            button_pressed(sel);
    }
    else
    {
        printk("\n=== Mesh Provisioner Commands ===\n");
        printk("\n--- Provisioning Commands ---\n");
        printk("  mesh_start_target <g> <r> - Start targeted provisioning\n");
        printk("                              g: 0-3 (A-D), r: 0-2 (Row 1-3)\n");
        printk("  mesh_prov_start           - Start general provisioning mode\n");
        printk("  mesh_prov_stop            - Stop provisioning mode\n");
        printk("  mesh_btn 1                - Confirm provisioning (general mode)\n");
        printk("\n--- Control Commands ---\n");
        printk("  mesh_send_onoff <addr> <0|1> - Send OnOff command\n");
        printk("    Special addresses:\n");
        printk("      0xFFFF   - All nodes\n");
        printk("      0xC001-4 - Group A-D total\n");
        printk("      0xC111+  - Group rows\n");
        printk("  mesh_test_node <addr>     - Test if a node is online\n");
        printk("\n--- Management Commands ---\n");
        printk("  mesh_list_nodes           - List configured nodes\n");
        printk("  mesh_check_storage        - Check storage status\n");
        printk("  mesh_node_reset <addr>    - Reset and unprovision a node\n");
        printk("  mesh_reset [reboot]       - Reset provisioner configuration\n");
        printk("==============================\n\n");
    }
}
MSH_CMD_EXPORT(mesh_btn, mesh provisioner control commands);

/* Command: Send OnOff command to a node or group */
static void mesh_send_onoff(uint8_t argc, char **argv)
{
    uint16_t addr;
    bool on_off;
    struct bt_mesh_msg_ctx ctx;
    BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_SET_UNACK, 2);
    static uint8_t tid = 0;
    int err;

    if (argc < 3)
    {
        printk("Usage: mesh_send_onoff <addr> <0|1>\n");
        printk("  addr: destination address in hex (e.g., 0x0002, 0xC001, 0xFFFF)\n");
        printk("  0|1:  off or on\n");
        printk("\nSpecial addresses:\n");
        printk("  0xFFFF - All nodes\n");
        printk("  0xC001-0xC004 - Group A-D total\n");
        printk("  0xC111-0xC143 - Specific group rows\n");
        return;
    }

    addr = strtol(argv[1], NULL, 16);
    on_off = strtol(argv[2], NULL, 0) ? true : false;

    /* Determine address type for display */
    const char *addr_type = "Unicast";
    if (addr == GROUP_ALL_NODES)
    {
        addr_type = "All Nodes";
    }
    else if (addr >= GROUP_A_TOTAL && addr <= GROUP_D_TOTAL)
    {
        addr_type = "Group Total";
    }
    else if (addr >= GROUP_ROW_BASE && addr < (GROUP_ROW_BASE + 0x100))
    {
        addr_type = "Group Row";
    }

    printk("Sending OnOff %s to 0x%04x (%s)\n",
           on_off ? "ON" : "OFF", addr, addr_type);

    /* Setup context - use the app_idx that was configured during provisioning */
    ctx.app_idx = app_idx;
    ctx.addr = addr;
    ctx.send_ttl = BT_MESH_TTL_DEFAULT;

    /* Build message - use SET_UNACK to avoid ACK storm */
    bt_mesh_model_msg_init(&buf, OP_ONOFF_SET_UNACK);
    net_buf_simple_add_u8(&buf, on_off ? 0x01 : 0x00);
    net_buf_simple_add_u8(&buf, tid++);

    /* Send message using the Generic OnOff Client model (root_models[3]) */
    err = bt_mesh_model_send(&root_models[3], &ctx, &buf, NULL, NULL);
    if (err)
    {
        printk("Failed to send OnOff (err %d)\n", err);
        if (err == -ENOENT)
        {
            printk("Hint: Make sure the OnOff client model is bound to an app key\n");
        }
        return;
    }

    printk("OnOff sent successfully\n");
}
MSH_CMD_EXPORT(mesh_send_onoff, send OnOff command to mesh node or group);

/* Command: Test if a node is reachable by sending a simple message */
static void mesh_test_node(uint8_t argc, char **argv)
{
    uint16_t addr;
    int err;

    if (argc < 2)
    {
        printk("Usage: mesh_test_node <addr>\n");
        printk("  addr: target node address in hex (e.g., 0x0002)\n");
        printk("\nThis command tests if a node is online and responsive.\n");
        return;
    }

    addr = strtol(argv[1], NULL, 16);

    printk("\n=== Testing Node Connectivity ===\n");
    printk("Target node: 0x%04x\n", addr);

    /* Check if node exists in CDB */
    struct bt_mesh_cdb_node *node = bt_mesh_cdb_node_get(addr);
    if (!node)
    {
        printk("✗ Node not found in CDB\n");
        printk("==================================\n\n");
        return;
    }

    printk("✓ Node found in CDB (NetIdx: 0x%04x)\n", node->net_idx);

    /* Check DevKey status */
    printk("  DevKey: ");
    for (int i = 0; i < 16; i++)
    {
        printk("%02x", node->dev_key.key[i]);
    }
    printk("\n");

    /* Check subnet status */
    struct bt_mesh_subnet *subnet = bt_mesh_subnet_get(node->net_idx);
    if (subnet)
    {
        printk("  ✓ Subnet found (net_idx: 0x%04x)\n", node->net_idx);
    }
    else
    {
        printk("  ✗ Subnet NOT found (net_idx: 0x%04x)\n", node->net_idx);
    }

    /* First test: Try sending a simple OnOff message as a connectivity test */
    printk("\n--- Test 1: OnOff Client (AppKey) ---\n");

    struct bt_mesh_msg_ctx ctx =
    {
        .app_idx = app_idx,
        .addr = addr,
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };

    if (ctx.app_idx == BT_MESH_KEY_UNUSED)
    {
        printk("✗ AppKey not bound to OnOff Client\n");
    }
    else
    {
        BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_SET_UNACK, 2);
        bt_mesh_model_msg_init(&buf, OP_ONOFF_SET_UNACK);
        net_buf_simple_add_u8(&buf, 0x01);  // ON
        net_buf_simple_add_u8(&buf, 0xFF);  // tid

        err = bt_mesh_model_send(&root_models[3], &ctx, &buf, NULL, NULL);
        if (err)
        {
            printk("✗ OnOff send failed (err %d)\n", err);
        }
        else
        {
            printk("✓ OnOff message sent (AppKey)\n");
        }
    }

    /* Second test: Try Config Client (DevKey) */
    printk("\n--- Test 2: Config Client (DevKey) ---\n");
    printk("Sending Composition Data Get request...\n");

    /* Retry mechanism for Config Client */
    const int max_retries = 3;
    int success = 0;

    for (int retry = 0; retry < max_retries; retry++)
    {
        NET_BUF_SIMPLE_DEFINE(buf, 64);
        uint8_t status = 0xFF;

        if (retry > 0)
        {
            printk("Retry %d/%d...\n", retry + 1, max_retries);
            k_sleep(K_MSEC(500));
        }

        err = bt_mesh_cfg_cli_comp_data_get(node->net_idx, addr, 0, &status, &buf);

        if (!err && status == 0x00)
        {
            printk("✓ Node is ONLINE and responsive!\n");
            printk("  Composition data received successfully\n");

            /* Parse and display basic info */
            struct bt_mesh_comp_p0 comp;
            err = bt_mesh_comp_p0_get(&comp, &buf);
            if (!err)
            {
                printk("  Company ID: 0x%04x\n", comp.cid);
                printk("  Product ID: 0x%04x\n", comp.pid);
                printk("  Version ID: 0x%04x\n", comp.vid);
                printk("  CRPL: 0x%04x\n", comp.crpl);
                printk("  Features: 0x%04x\n", comp.feat);
            }
            success = 1;
            break;
        }
        else if (err == -ETIMEDOUT)
        {
            printk("  Timeout (attempt %d/%d)\n", retry + 1, max_retries);
        }
        else
        {
            printk("  Error: err=%d, status=0x%02x\n", err, status);
        }
    }

    if (!success)
    {
        printk("\n✗ Config Client failed after %d retries\n", max_retries);
        printk("  Possible reasons:\n");
        printk("  - Node is powered off or out of range\n");
        printk("  - GATT bearer connection not established\n");
        printk("  - Device Key not properly restored after reboot\n");
        printk("\n  Suggested actions:\n");
        printk("  1. Verify node is powered on\n");
        printk("  2. Check if node is in range\n");
        printk("  3. Try: mesh_send_onoff 0x0002 1 (OnOff test)\n");
    }

    printk("==================================\n\n");
}
MSH_CMD_EXPORT(mesh_test_node, test if a mesh node is online and responsive);

/* Command: Send Node Reset command to all nodes in a group/row or single node */
static void mesh_node_reset(uint8_t argc, char **argv)
{
    uint16_t addr;
    bool status = false;
    int err;
    int retry_count = 0;
    const int max_retries = 3;

    if (argc < 2)
    {
        printk("Usage: mesh_node_reset <addr>\n");
        printk("  addr: target node address in hex (e.g., 0x0002)\n");
        printk("\nWARNING: This will remove the node from the mesh network!\n");
        printk("The node will need to be re-provisioned to join again.\n");
        printk("\nTo reset entire network, use: mesh_reset reboot\n");
        return;
    }

    addr = strtol(argv[1], NULL, 16);

    printk("\n=== Sending Node Reset Command ===\n");
    printk("Target node: 0x%04x\n", addr);
    printk("This will unprovision the node and clear all its mesh configuration.\n");

    /* First, check if node exists in CDB */
    struct bt_mesh_cdb_node *node = bt_mesh_cdb_node_get(addr);
    if (!node)
    {
        printk("✗ Error: Node 0x%04x not found in CDB\n", addr);
        printk("Please verify the node address using 'mesh_list_nodes'\n");
        printk("==================================\n\n");
        return;
    }

    printk("✓ Node found in CDB\n");
    printk("  NetIdx: 0x%04x\n", node->net_idx);
    printk("  Num Elements: %d\n", node->num_elem);

    /* Try to send Node Reset command with retries */
    while (retry_count < max_retries)
    {
        printk("\nAttempt %d/%d: Sending Node Reset command...\n",
               retry_count + 1, max_retries);

        /* Send Node Reset command using Config Client */
        err = bt_mesh_cfg_cli_node_reset(node->net_idx, addr, &status);

        if (!err && status)
        {
            /* Success */
            printk("\n✓ Node Reset command succeeded!\n");
            printk("✓ Node 0x%04x has been reset and unprovisioned\n", addr);

            /* Remove the node from CDB */
            printk("Removing node from CDB...\n");
            bt_mesh_cdb_node_del(node, true);
            printk("✓ Node removed from CDB\n");

            /* Update NVDS: Remove node location data */
            printk("Updating NVDS node location data...\n");
            mesh_location_remove(addr);

            printk("==================================\n\n");
            return;
        }

        /* Failed - analyze error */
        retry_count++;
        if (err == -ETIMEDOUT)
        {
            printk("✗ Timeout (err %d) - Node did not respond\n", err);
            if (retry_count < max_retries)
            {
                printk("  Possible causes:\n");
                printk("  1. Node is offline or out of range\n");
                printk("  2. Node's AppKey binding is incorrect\n");
                printk("  3. Network congestion\n");
                printk("  Retrying in 2 seconds...\n");
                k_sleep(K_SECONDS(2));
            }
        }
        else
        {
            printk("✗ Failed to send Node Reset (err %d)\n", err);
            printk("  This indicates a local error (not a timeout)\n");
            break;  /* Don't retry on non-timeout errors */
        }
    }

    /* All retries failed */
    printk("\n✗ Node Reset failed after %d attempts\n", max_retries);
    printk("Troubleshooting suggestions:\n");
    printk("1. Verify node is online: mesh_send_onoff 0x%04x 1\n", addr);
    printk("2. Check node configuration: mesh_check_storage\n");
    printk("3. Ensure AppKey is bound to Config Server on node\n");
    printk("==================================\n\n");
}
MSH_CMD_EXPORT(mesh_node_reset, send Node Reset command to unprovision a mesh node);

/* Command: Reset entire mesh network by sending Node Reset to all nodes */
static void mesh_reset_all_nodes(uint8_t argc, char **argv)
{
    int reset_count = 0;
    int fail_count = 0;
    uint16_t self_addr = bt_mesh_primary_addr();

    printk("\n=== WARNING: Resetting ALL Mesh Nodes ===\n");
    printk("This will unprovision all nodes in the network!\n");
    printk("All nodes will need to be re-provisioned.\n");
    printk("Proceeding in 3 seconds...\n");
    k_sleep(K_SECONDS(3));

    /*
     * IMPORTANT: Reset remote nodes FIRST, then reset local provisioner LAST.
     * If we reset the local node first, it loses its provisioned state and
     * cannot send Config messages to other nodes.
     */

    /* Step 1: Reset all REMOTE nodes (skip self) */
    printk("\n--- Step 1: Resetting remote nodes ---\n");
    for (int i = 0; i < ARRAY_SIZE(bt_mesh_cdb.nodes); i++)
    {
        struct bt_mesh_cdb_node *node = &bt_mesh_cdb.nodes[i];

        if (node->addr == BT_MESH_ADDR_UNASSIGNED)
        {
            continue;
        }

        /* Skip the provisioner itself - will reset last */
        if (node->addr == self_addr)
        {
            printk("Skipping local node 0x%04x (will reset last)\n", node->addr);
            continue;
        }

        printk("Resetting node 0x%04x...", node->addr);

        /* Retry mechanism for robustness */
        bool reset_success = false;
        int max_retries = 3;

        for (int retry = 0; retry < max_retries; retry++)
        {
            bool status = false;
            int err = bt_mesh_cfg_cli_node_reset(node->net_idx, node->addr, &status);

            if (!err && status)
            {
                reset_success = true;
                break;
            }

            if (retry < max_retries - 1)
            {
                printk(" timeout (retry %d/%d)...", retry + 1, max_retries - 1);
                k_sleep(K_MSEC(1500));  /* Wait 1.5s before retry */
            }
        }

        if (reset_success)
        {
            printk(" OK\n");
            reset_count++;
            bt_mesh_cdb_node_del(node, true);
        }
        else
        {
            printk(" FAILED after %d retries\n", max_retries);
            fail_count++;
        }

        /* Small delay between resets */
        k_sleep(K_MSEC(100));
    }

    printk("\nRemote nodes reset summary:\n");
    printk("  Successfully reset: %d nodes\n", reset_count);
    printk("  Failed: %d nodes\n", fail_count);

    /* Step 2: Reset the local provisioner LAST */
    printk("\n--- Step 2: Clearing Flash storage ---\n");

    /* CRITICAL: Delete all bt/mesh/* keys from FlashDB BEFORE calling bt_mesh_reset() */
    if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
        printk("Deleting stored Mesh settings from FlashDB...\n");

        int deleted_count = 0;
        struct fdb_kv_iterator itr_obj;
        fdb_kvdb_t kvdb;

        /* Get FlashDB instance */
        kvdb = sifli_nvds_get_ble_kvdb();

        if (kvdb)
        {
            /* We need to iterate and delete all bt/mesh/* keys */
            /* Note: Must be careful not to invalidate iterator while deleting */
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
            printk("⚠ Could not access FlashDB\n");
        }

        /* Clear NVDS node location data */
        printk("Clearing NVDS node location data...\n");
        mesh_location_clear();
    }

    printk("\n--- Step 3: Resetting local provisioner ---\n");
    printk("Resetting local node 0x%04x...", self_addr);

    /* Now clear CDB and local state using standard API */
    bt_mesh_reset();
    printk(" OK\n");

    printk("\n=== Reset Complete ===\n");
    printk("All nodes have been reset.\n");
    printk("Provisioner will reboot automatically...\n");
    printk("======================\n\n");

    /* Reboot to ensure clean state */
    drv_reboot();
}
MSH_CMD_EXPORT(mesh_reset_all_nodes, reset all mesh nodes and clear provisioner database);

/* Command: Reset Mesh configuration and clear all stored data */
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

        /* Clear NVDS node location data */
        printk("Clearing NVDS node location data...\n");
        mesh_location_clear();
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
    int err;

    printk("Initializing...\n");

    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    printk("Bluetooth initialized\n");
    bt_ready();

    printk("\n=== Mesh Provisioner Ready ===\n");
    printk("Provisioning is DISABLED by default\n");
    printk("\nQuick Start:\n");
    printk("  mesh_start_target 0 0   - Provision Group A Row 1\n");
    printk("  mesh_send_onoff 0xFFFF 1 - Turn on all lights\n");
    printk("  mesh_list_nodes          - View configured nodes\n");
    printk("  mesh_btn                 - Show all commands\n");
    printk("==============================\n\n");

    /* Main thread just keeps running, provisioning is handled by separate thread */
    while (1)
    {
        k_sleep(K_SECONDS(60));

        /* Periodically check for unconfigured nodes */
        bt_mesh_cdb_node_foreach(check_unconfigured, NULL);

        /* Check if targeted provisioning is complete using precise check */
        if (targeted_prov.waiting_for_config)
        {
            if (check_targeted_provisioning_complete())
            {
                printk("✓ All targeted nodes configured! Stopping provisioning.\n");
                targeted_prov.waiting_for_config = false;
                targeted_prov.active = false;
                targeted_prov.provisioned_count = 0;
                provisioning_enabled = false;
            }
        }
    }
}

