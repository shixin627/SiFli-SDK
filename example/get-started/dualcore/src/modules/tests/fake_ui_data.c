/*
 * SPDX-FileCopyrightText: 2026 SiFli / project contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * PC simulator fake data sources + MSH controls.
 *
 * Three things live here:
 *
 *   1. gesture_back / back        — post LVGL_MSG_TYPE_BACK_EVENT so the
 *                                   left-edge swipe-back gesture can be
 *                                   triggered from tshell (no hardware key).
 *
 *   2. notif_inject / clear / list — fill the notification list (bloc_notification
 *                                    is built on PC; we just call its API and
 *                                    fire LVGL_MSG_TYPE_NOTIFICATION to refresh).
 *
 *   3. dev_add / connect / disconnect / clear / list
 *      Real implementations of ble_dev_mgr_* that back an in-memory bonded
 *      devices DB. ble_device_manager.c is excluded on PC (pulls BLE host),
 *      and pc_link_stubs.c used to return NULL — UI screen saw no devices.
 *      These take over the same symbols.
 *
 * Build gate: BSP_USING_PC_SIMULATOR (see tests/SConscript). Symbols removed
 * from pc_link_stubs.c + _syms_clean.txt so a future _genstub.py run does
 * not regenerate dueling stubs.
 */
#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "ble_device_manager.h"
#include "bloc_notification.h"
#include "ui_handler.h"
#include "watch_global_data.h"  /* ADR-0008 E7: SkaiWatchSys.device_registry seed (sim) */

#define DBG_TAG "fake_ui"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* ===================================================================== */
/*  Fake BLE device manager                                              */
/* ===================================================================== */

static bonded_devices_db_t fake_db;
static dev_mgr_event_cb_t  fake_cb_fn;
static void               *fake_cb_ud;

static void recount_db(void)
{
    fake_db.count = 0;
    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
        if (fake_db.devices[i].is_valid) fake_db.count++;
}

static void fire_cb(dev_mgr_event_t evt, uint8_t idx)
{
    /* The registered callback (app_setting_device_list.c:dev_mgr_event_cb)
     * calls refresh_device_list() which does lv_obj_clean / lv_obj_create
     * directly — those are NOT thread-safe. Our MSH commands run on the
     * FinSH thread; calling the cb here crashes the LVGL renderer.
     *
     * Skip firing. The device list page re-reads the DB on every page open
     * (line 451 / 459 in app_setting_device_list.c), so the workflow is:
     *   1. dev_add / dev_clear / etc. from tshell
     *   2. swipe to settings → device list page (or back-and-forward to re-open)
     *   3. the page init reads our fake DB fresh
     */
    (void)evt;
    (void)idx;
    (void)fake_cb_fn;
    (void)fake_cb_ud;
}

const bonded_devices_db_t *ble_dev_mgr_get_database(void)
{
    return &fake_db;
}

int ble_dev_mgr_register_callback(dev_mgr_event_cb_t cb, void *user_data)
{
    fake_cb_fn = cb;
    fake_cb_ud = user_data;
    return 0;
}

int ble_dev_mgr_get_connected_count(void)
{
    int n = 0;
    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
        if (fake_db.devices[i].is_valid && fake_db.devices[i].conn_idx != 0xFF) n++;
    return n;
}

int ble_dev_mgr_get_active_device(void)
{
    return fake_db.active_device_idx;
}

int ble_dev_mgr_clear_all(void)
{
    memset(&fake_db, 0, sizeof(fake_db));
    fire_cb(DEV_MGR_EVT_DATABASE_UPDATED, 0);
    return 0;
}

int ble_dev_mgr_set_active_device(uint8_t idx)
{
    if (idx >= MAX_BONDED_DEVICES || !fake_db.devices[idx].is_valid) return -1;
    fake_db.active_device_idx = idx;
    fire_cb(DEV_MGR_EVT_ACTIVE_DEVICE_CHANGED, idx);
    return 0;
}

int ble_dev_mgr_remove_device(uint8_t idx)
{
    if (idx >= MAX_BONDED_DEVICES) return -1;
    fake_db.devices[idx].is_valid = 0;
    recount_db();
    fire_cb(DEV_MGR_EVT_DEVICE_REMOVED, idx);
    return 0;
}

int ble_dev_mgr_connect_device(uint8_t idx)
{
    if (idx >= MAX_BONDED_DEVICES || !fake_db.devices[idx].is_valid) return -1;
    fake_db.devices[idx].conn_idx = idx;  /* any non-0xFF means connected */
    fire_cb(DEV_MGR_EVT_DEVICE_CONNECTED, idx);
    return 0;
}

int ble_dev_mgr_disconnect_device(uint8_t idx)
{
    if (idx >= MAX_BONDED_DEVICES) return -1;
    fake_db.devices[idx].conn_idx = 0xFF;
    fire_cb(DEV_MGR_EVT_DEVICE_DISCONNECTED, idx);
    return 0;
}

int ble_dev_mgr_switch_to_next_device(void)
{
    int start = (fake_db.active_device_idx + 1) % MAX_BONDED_DEVICES;
    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        int idx = (start + i) % MAX_BONDED_DEVICES;
        if (fake_db.devices[idx].is_valid)
            return ble_dev_mgr_set_active_device((uint8_t)idx);
    }
    return -1;
}

int ble_dev_mgr_add_device(const uint8_t *mac_addr, uint8_t addr_type,
                           const char *device_name, ble_device_type_t device_type)
{
    /* Replace existing same-MAC entry first; else fill the first empty slot. */
    int idx = -1;
    if (mac_addr)
    {
        for (int i = 0; i < MAX_BONDED_DEVICES; i++)
        {
            if (fake_db.devices[i].is_valid &&
                memcmp(fake_db.devices[i].mac_addr, mac_addr, 6) == 0)
            {
                idx = i;
                break;
            }
        }
    }
    if (idx < 0)
    {
        for (int i = 0; i < MAX_BONDED_DEVICES; i++)
            if (!fake_db.devices[i].is_valid) { idx = i; break; }
    }
    if (idx < 0) return -1;

    bonded_device_t *d = &fake_db.devices[idx];
    if (mac_addr) memcpy(d->mac_addr, mac_addr, 6);
    else          memset(d->mac_addr, 0xAB, 6);
    d->addr_type = addr_type;
    memset(d->device_name, 0, sizeof(d->device_name));
    if (device_name)
        rt_strncpy(d->device_name, device_name, DEVICE_NAME_MAX_LEN - 1);
    d->device_type         = (uint8_t)device_type;
    d->last_connected_time = (uint32_t)(rt_tick_get() / RT_TICK_PER_SECOND);
    d->is_valid            = 1;
    d->conn_idx            = 0xFF;  /* added but not yet connected */
    recount_db();
    fire_cb(DEV_MGR_EVT_DEVICE_ADDED, (uint8_t)idx);
    return idx;
}

/* ===================================================================== */
/*  MSH commands — back gesture                                          */
/* ===================================================================== */

static int gesture_back(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    lvgl_msg_t msg = { .type = LVGL_MSG_TYPE_BACK_EVENT };
    lvgl_send_msg(msg);
    rt_kprintf("back event sent\n");
    return 0;
}
MSH_CMD_EXPORT(gesture_back, gesture_back - inject back gesture);
MSH_CMD_EXPORT_ALIAS(gesture_back, back, back - alias for gesture_back);

/* ===================================================================== */
/*  MSH commands — notification                                          */
/* ===================================================================== */

static int notif_inject(int argc, char *argv[])
{
    if (argc < 3)
    {
        rt_kprintf("usage: notif_inject <title> <message> [type]\n");
        return -1;
    }

    /* Append at slot == notification_items_amount; clamp at the ring's tail.
     * get_notification(i) returns _notification_list[items_amount - i - 1]
     * (newest first), so this slot becomes get_notification(0) after the
     * count bumps — matching how the UI iterates the list. */
    int slot = notification_items_amount;
    if (slot >= ITEM_AMOUNT_NOTIFICATION) slot = ITEM_AMOUNT_NOTIFICATION - 1;

    notification_t notif;
    memset(&notif, 0, sizeof(notif));
    notif.index    = (uint8_t)(slot + 1);  /* 1-based per set_notification */
    notif.state    = true;
    notif.sec_time = (uint32_t)(rt_tick_get() / RT_TICK_PER_SECOND);
    notif.type     = (argc >= 4) ? (uint16_t)atoi(argv[3]) : 0;
    rt_snprintf(notif.id,     NOTIFICATION_ID_LEN,     "fake_%u_%d",
                (unsigned)rt_tick_get(), slot);
    rt_strncpy(notif.title,   argv[1], NOTIFICATION_TITLE_LEN - 1);
    rt_strncpy(notif.message, argv[2], NOTIFICATION_MESSAGE_LEN - 1);

    set_notification(notif, slot);
    if (notification_items_amount < ITEM_AMOUNT_NOTIFICATION)
        notification_items_amount++;

    /* PC simulator caveat: we cannot trigger an LVGL refresh after injecting.
     *
     * The dispatch path of LVGL_MSG_TYPE_NOTIFICATION calls three handlers
     * in sequence (ui_handler.c:279) — handle_notification → refresh_notification_list
     * → create_msg_indicator_dots, handle_new_notification → refresh_new_message_widget,
     * and handle_dial_header_new_notification — and ALL of them eventually hit
     * lv_img_set_src(..., icon_list[type]). On PC the underlying icon resource
     * decode fails (image cache / ROM3_IMG section mapping differs from MCU)
     * and the LVGL renderer crashes. Nulling individual handlers didn't help —
     * the indicator-dot rebuild in handle_notification also touches icon_list.
     *
     * Workaround for fake notifications:
     *   - notif_inject only stashes data into _notification_list[] + items_amount.
     *   - The watchface drawer ("No notifications") is built ONCE at boot when
     *     count was 0, so it sticks around even after injection.
     *   - To verify data lives on, use `notif_list` from tshell.
     *   - To exercise downstream UI that *reads* this data on its own resume
     *     (notification detail screen, message app deeper pages), navigate
     *     there from the launcher — those pages re-read on open.
     *
     * A proper fix needs PC-stub icon resources or a guard in the icon set
     * path; both are larger work than this MSH shim. */
    rt_kprintf("notif_inject slot=%d count=%u title='%s'\n",
               slot, notification_items_amount, argv[1]);
    return 0;
}
MSH_CMD_EXPORT(notif_inject, notif_inject title msg [type] - add fake notification);

static int notif_clear(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    notification_t empty;
    memset(&empty, 0, sizeof(empty));
    for (int i = 0; i < ITEM_AMOUNT_NOTIFICATION; i++)
        set_notification(empty, i);
    notification_items_amount = 0;
    /* Same crash caveat as notif_inject — no live refresh. */
    rt_kprintf("notifications cleared\n");
    return 0;
}
MSH_CMD_EXPORT(notif_clear, notif_clear - clear all notifications);

static int notif_list(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    rt_kprintf("notif count=%lu\n",
               (unsigned long)notification_center_get_info_count());
    for (int i = 0; i < ITEM_AMOUNT_NOTIFICATION; i++)
    {
        notification_t *n = get_notification(i);
        if (n && n->state)
            rt_kprintf("  [%d] type=%u title='%s' msg='%s'\n",
                       i, n->type, n->title, n->message);
    }
    return 0;
}
MSH_CMD_EXPORT(notif_list, notif_list - dump notification list);

/* Mark a known id as dismissed, without touching _notification_list[] (we
 * cannot reach remove_notification from outside the unit, and going through
 * notify_provider.notification_refresh would trip the eZIP icon crash). To
 * verify the production dedup, use notif_check_dismissed on the same id. */
static int notif_dismiss(int argc, char *argv[])
{
    if (argc < 2) { rt_kprintf("usage: notif_dismiss <id>\n"); return -1; }
    bloc_notification_mark_dismissed(argv[1]);
    rt_kprintf("notif_dismiss id=%s -> marked\n", argv[1]);
    return 0;
}
MSH_CMD_EXPORT(notif_dismiss, notif_dismiss id - mark id as dismissed (ring only));

static int notif_check_dismissed(int argc, char *argv[])
{
    if (argc < 2) { rt_kprintf("usage: notif_check_dismissed <id>\n"); return -1; }
    bool d = bloc_notification_is_dismissed(argv[1]);
    rt_kprintf("notif_check_dismissed id=%s -> %s\n",
               argv[1], d ? "DISMISSED" : "not dismissed");
    return 0;
}
MSH_CMD_EXPORT(notif_check_dismissed, notif_check_dismissed id - is id in the dismissed ring?);

static int notif_dismiss_clear(int argc, char *argv[])
{
    (void)argc; (void)argv;
    bloc_notification_clear_dismissed();
    rt_kprintf("dismissed ring cleared (in-memory only -- flush with notif_dismiss_save)\n");
    return 0;
}
MSH_CMD_EXPORT(notif_dismiss_clear, notif_dismiss_clear - reset the dismissed ring);

static int notif_dismiss_save(int argc, char *argv[])
{
    (void)argc; (void)argv;
    bloc_notification_save_dismissed_to_flash();
    rt_kprintf("flush attempted (see log for result)\n");
    return 0;
}
MSH_CMD_EXPORT(notif_dismiss_save, notif_dismiss_save - manually flush ring to flash);

/* ===================================================================== */
/*  MSH commands — fake device list                                      */
/* ===================================================================== */

static int dev_add(int argc, char *argv[])
{
    if (argc < 2)
    {
        rt_kprintf("usage: dev_add <name> [type] [connected:0|1]\n"
                   "  type: 0=phone 1=computer 2=tablet 3=other\n");
        return -1;
    }
    ble_device_type_t type =
        (argc >= 3) ? (ble_device_type_t)atoi(argv[2]) : DEVICE_TYPE_PHONE;
    int connect = (argc >= 4) ? atoi(argv[3]) : 1;

    /* Derive a stable-ish MAC from the name so repeated dev_add of the same
     * name updates the same slot instead of filling new ones. */
    uint8_t mac[6] = { 0x12, 0x34, 0x56, 0, 0, 0 };
    uint32_t h = 0;
    for (const char *p = argv[1]; *p; p++) h = h * 31u + (uint32_t)(uint8_t)*p;
    mac[3] = (uint8_t)(h >> 16);
    mac[4] = (uint8_t)(h >> 8);
    mac[5] = (uint8_t)(h);

    int idx = ble_dev_mgr_add_device(mac, 0, argv[1], type);
    if (idx < 0)
    {
        rt_kprintf("dev_add: db full (max %d)\n", MAX_BONDED_DEVICES);
        return -1;
    }
    if (connect) ble_dev_mgr_connect_device((uint8_t)idx);
    rt_kprintf("dev_add idx=%d name='%s' type=%d conn=%d\n",
               idx, argv[1], (int)type, connect);
    return 0;
}
MSH_CMD_EXPORT(dev_add, dev_add name [type] [conn] - add fake bonded device);

static int dev_connect(int argc, char *argv[])
{
    if (argc < 2) { rt_kprintf("usage: dev_connect <idx>\n"); return -1; }
    int idx = atoi(argv[1]);
    if (ble_dev_mgr_connect_device((uint8_t)idx) != 0)
    {
        rt_kprintf("dev_connect: invalid idx %d\n", idx);
        return -1;
    }
    rt_kprintf("dev[%d] connected\n", idx);
    return 0;
}
MSH_CMD_EXPORT(dev_connect, dev_connect idx - mark fake device connected);

static int dev_disconnect(int argc, char *argv[])
{
    if (argc < 2) { rt_kprintf("usage: dev_disconnect <idx>\n"); return -1; }
    int idx = atoi(argv[1]);
    if (ble_dev_mgr_disconnect_device((uint8_t)idx) != 0)
    {
        rt_kprintf("dev_disconnect: invalid idx %d\n", idx);
        return -1;
    }
    rt_kprintf("dev[%d] disconnected\n", idx);
    return 0;
}
MSH_CMD_EXPORT(dev_disconnect, dev_disconnect idx - mark device disconnected);

static int dev_clear(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    ble_dev_mgr_clear_all();
    rt_kprintf("all fake devices cleared\n");
    return 0;
}
MSH_CMD_EXPORT(dev_clear, dev_clear - remove all fake devices);

static int dev_list(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    rt_kprintf("dev_list count=%u active=%u\n",
               db->count, db->active_device_idx);
    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        const bonded_device_t *d = &db->devices[i];
        if (d->is_valid)
            rt_kprintf("  [%d] type=%u conn=%s name='%s'\n",
                       i, d->device_type,
                       (d->conn_idx != 0xFF) ? "Y" : "N",
                       d->device_name);
    }
    return 0;
}
MSH_CMD_EXPORT(dev_list, dev_list - dump fake bonded device list);

/* ===================================================================== */
/*  MSH commands — battery / heart rate / charge status                  */
/* ===================================================================== */
/*
 * These pump LVGL_MSG_TYPE_BATTERY_LEVEL / VOLTAGE / CHARGE_STATUS / HR
 * straight into the UI message queue. The handlers (registered by clock
 * status bar + battery widget) redraw on the LVGL thread. Unlike the
 * notification path these handlers do not touch icon_list[type], so the
 * eZIP-decoder crash does not apply.
 */

static int battery_set(int argc, char *argv[])
{
    if (argc < 2)
    {
        rt_kprintf("usage: battery_set <0..100>\n");
        return -1;
    }
    int pct = atoi(argv[1]);
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    lvgl_msg_t msg = { 0 };
    msg.type = LVGL_MSG_TYPE_BATTERY_LEVEL;
    msg.data.battery_level = (uint8_t)pct;
    lvgl_send_msg(msg);
    rt_kprintf("battery_set %d%%\n", pct);
    return 0;
}
MSH_CMD_EXPORT(battery_set, battery_set 0..100 - fake battery percentage);

static int battery_volt_set(int argc, char *argv[])
{
    if (argc < 2)
    {
        rt_kprintf("usage: battery_volt_set <millivolts>\n");
        return -1;
    }
    int mv = atoi(argv[1]);
    if (mv < 0)     mv = 0;
    if (mv > 65535) mv = 65535;
    lvgl_msg_t msg = { 0 };
    msg.type = LVGL_MSG_TYPE_BATTERY_VOLTAGE;
    msg.data.battery_voltage = (uint16_t)mv;
    lvgl_send_msg(msg);
    rt_kprintf("battery_volt_set %d mV\n", mv);
    return 0;
}
MSH_CMD_EXPORT(battery_volt_set, battery_volt_set mv - fake battery voltage);

static int charge_set(int argc, char *argv[])
{
    if (argc < 2)
    {
        rt_kprintf("usage: charge_set <0|1>\n");
        return -1;
    }
    bool on = atoi(argv[1]) != 0;
    lvgl_msg_t msg = { 0 };
    msg.type = LVGL_MSG_TYPE_CHARGE_STATUS;
    msg.data.charge_status = on;
    lvgl_send_msg(msg);
    rt_kprintf("charge_set %d\n", (int)on);
    return 0;
}
MSH_CMD_EXPORT(charge_set, charge_set 0|1 - fake charging status);

static int hr_set(int argc, char *argv[])
{
    if (argc < 2)
    {
        rt_kprintf("usage: hr_set <bpm>\n");
        return -1;
    }
    int bpm = atoi(argv[1]);
    lvgl_msg_t msg = { 0 };
    msg.type = LVGL_MSG_TYPE_HR;
    msg.data.hr = bpm;
    lvgl_send_msg(msg);
    rt_kprintf("hr_set %d bpm\n", bpm);
    return 0;
}
MSH_CMD_EXPORT(hr_set, hr_set bpm - fake heart rate value);

/* ===================================================================== */
/*  MSH commands — instruction-list AI widget (PC sim only)              */
/*                                                                       */
/*  On real hardware the AI widget opens via wrist-raise / mic-bar tap   */
/*  with a live BLE connection. PC sim has no BT, so tap_on_ai_widget    */
/*  short-circuits at the connection check. These helpers let us drive   */
/*  the widget directly to verify layout / state behaviour.              */
/* ===================================================================== */

extern void set_is_open_instruction_list_ai(bool open);
extern void open_skai_widget_ai(bool open);
extern void refresh_ai_chat_input_message(char *text);

static int sim_skai_open(int argc, char *argv[])
{
    (void)argc; (void)argv;
    set_is_open_instruction_list_ai(true);
    open_skai_widget_ai(true);
    rt_kprintf("sim_skai_open: AI widget marked open\n");
    return 0;
}
MSH_CMD_EXPORT(sim_skai_open, sim_skai_open - force open instruction-list AI widget);

/* Open the floating mixed list in browse mode (no input box, all categories) —
   the same surface the left-edge reveal pulls in, but driven programmatically so
   the PC sim (no BLE gesture path) can screenshot it. */
static int sim_open_list(int argc, char *argv[])
{
    (void)argc; (void)argv;
    extern void instruction_list_open_browse(void);
    instruction_list_open_browse();
    rt_kprintf("sim_open_list: browse list opened\n");
    return 0;
}
MSH_CMD_EXPORT(sim_open_list, sim_open_list - open the mixed browse list (sim));

/* Persistent buffer — LVGL_MSG_TYPE_INPUT_MESSAGE carries the pointer, not a
   copy, so it must outlive the dispatch. Static storage is fine since this
   command is single-threaded from FinSH. */
static char sim_voice_text_buf[256];

static int sim_voice_say(int argc, char *argv[])
{
    if (argc < 2) {
        rt_kprintf("usage: sim_voice_say <text>...\n");
        return -1;
    }
    sim_voice_text_buf[0] = '\0';
    for (int i = 1; i < argc; i++) {
        if (i > 1) {
            strncat(sim_voice_text_buf, " ",
                       sizeof(sim_voice_text_buf) - strlen(sim_voice_text_buf) - 1);
        }
        strncat(sim_voice_text_buf, argv[i],
                   sizeof(sim_voice_text_buf) - strlen(sim_voice_text_buf) - 1);
    }
    /* The handler is normally wired by start_voice_recognition; we set it
       directly because we are bypassing v2t in PC sim. */
    lvgl_msg_handler.handle_input_message = refresh_ai_chat_input_message;
    lvgl_msg_t msg = { 0 };
    msg.type = LVGL_MSG_TYPE_INPUT_MESSAGE;
    msg.data.message = sim_voice_text_buf;
    lvgl_send_msg(msg);
    rt_kprintf("sim_voice_say sent: '%s'\n", sim_voice_text_buf);
    return 0;
}
MSH_CMD_EXPORT(sim_voice_say, sim_voice_say <text>... - inject voice transcript into skai widget);

/* ----- device_pager skaibar (right-side per-device) ------------------- */
/* Fake the "voice -> text" transcript into the device_pager skaibar input box
   (which the mic tap opens). No real ASR runs. */
static char pager_say_buf[128];
static int pager_say(int argc, char *argv[])
{
    if (argc < 2) { rt_kprintf("usage: pager_say <text>...\n"); return -1; }
    pager_say_buf[0] = '\0';
    for (int i = 1; i < argc; i++)
    {
        if (i > 1)
            strncat(pager_say_buf, " ",
                    sizeof(pager_say_buf) - strlen(pager_say_buf) - 1);
        strncat(pager_say_buf, argv[i],
                sizeof(pager_say_buf) - strlen(pager_say_buf) - 1);
    }
    extern void device_pager_skaibar_say(const char *text);
    device_pager_skaibar_say(pager_say_buf);
    rt_kprintf("pager_say: '%s'\n", pager_say_buf);
    return 0;
}
MSH_CMD_EXPORT(pager_say, pager_say <text>... - fake voice transcript into device_pager skaibar);

/* Fake the peer device returning skaibar options; they replace the instruction
   titles shown for the current device page. */
static int pager_options(int argc, char *argv[])
{
    if (argc < 2)
    {
        rt_kprintf("usage: pager_options <opt1> [opt2] ...\n");
        return -1;
    }
    extern void device_pager_skaibar_options(int n, const char *const opts[]);
    device_pager_skaibar_options(argc - 1, (const char *const *)&argv[1]);
    rt_kprintf("pager_options: applied %d option(s)\n", argc - 1);
    return 0;
}
MSH_CMD_EXPORT(pager_options, pager_options <opt1> [opt2]... - fake peer-device skaibar options);

/* Seed the account device registry (SkaiWatchSys.device_registry) so device_pager
   has devices to show in the PC sim — no phone streams the real registry here, so
   it is otherwise always empty (the QR empty state). Builds N devices (default 3)
   with ASCII placeholder names (the sim has no CJK glyphs) + a few fake actions
   each; the LAST device is left offline (status 0) so the offline branch is
   exercisable. Ends with device_pager_refresh() to re-render. */
static int pager_seed(int argc, char *argv[])
{
    int n = (argc > 1) ? atoi(argv[1]) : 3;
    if (n < 1) n = 1;
    if (n > MAX_SYNCED_DEVICES) n = MAX_SYNCED_DEVICES;

    static const char *const names[MAX_SYNCED_DEVICES] = {
        "Mac", "iPhone", "PC", "iPad", "Linux Box", "Work PC", "Tablet", "Server"
    };
    static const char *const acts[] = { "Open file", "Play music", "Lock screen" };
    const int nacts = (int)(sizeof(acts) / sizeof(acts[0]));

    T_DEVICE_REGISTRY *reg = &SkaiWatchSys.device_registry;
    memset(reg, 0, sizeof(*reg));
    reg->count = (uint8_t)n;
    for (int i = 0; i < n; i++)
    {
        T_SYNCED_DEVICE *d = &reg->devices[i];
        rt_snprintf(d->id, sizeof(d->id), "fake-device-%04d", i + 1);
        uint8_t ac = (uint8_t)nacts;
        if (ac > MAX_DEFAULT_ACTIONS) ac = MAX_DEFAULT_ACTIONS;
        for (uint8_t j = 0; j < ac; j++)
        {
            rt_strncpy(d->default_actions[j], acts[j],
                       sizeof(d->default_actions[0]) - 1);
            d->default_action_types[j] = (uint8_t)(j % 4); /* spread the category icons */
        }
        d->default_action_count = ac;

        rt_strncpy(SkaiWatchSys.device_name[i], names[i], SYNCED_DEVICE_NAME_LEN - 1);
        SkaiWatchSys.device_status[i] = (i == n - 1) ? 0 : 1; /* last device offline */
    }

    extern void device_pager_refresh(void);
    device_pager_refresh();
    rt_kprintf("pager_seed: %d fake device(s); last offline\n", n);
    return 0;
}
MSH_CMD_EXPORT(pager_seed, pager_seed [N] - seed N fake devices into device_pager (sim));
