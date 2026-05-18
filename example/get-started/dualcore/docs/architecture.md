# Dualcore Watch Architecture

How HCPU and LCPU split the work, the BLOC provider pattern, and the
BLE protocol with the phone-side counterpart in `C:\work\SkaiLink`.

---

## HCPU vs LCPU split

| 面向 | HCPU | LCPU |
|---|---|---|
| GUI | LVGL + 所有 gui_apps | 無 |
| BLE | Host + ANCS + 私有協定 | Controller |
| AI | TFLite Micro (gesture, activity) | wear-detect, AHRS |
| 感測器 | client/* 訂閱 | 直接驅動 BMI270 / GH3018 |
| 馬達 | bloc_peripheral provider call | PWM4 driver |
| 檔案系統 | DFS + ELMFAT | 無 |

**Cross-CPU channel**: HCPU `client/*` ↔ LCPU `service/*` via RT-Thread
`data_service` (`CONFIG_BSP_USING_DATA_SVC`) + `RT_USING_HWMAILBOX`.

---

## BLOC Provider pattern (canonical)

Each `bloc_*.c` exposes a global struct of function pointers, registered
at `INIT_APP_EXPORT`. Lets the GUI layer call into business logic without
needing to know the impl — and lets the impl swap by build config (real
vs PC sim fake).

```c
// bloc_control.h
typedef struct {
    void (*ble_hid_mouse_move)(int8_t dx, int8_t dy);
    bool (*bt_speaker_get_status)(void);
} ControlProvider;
extern ControlProvider control_provider;

// bloc_control.c
static int bloc_control_provider_register(void) {
    control_provider.ble_hid_mouse_move = ble_hid_mouse_move_impl;
    return 0;
}
INIT_APP_EXPORT(bloc_control_provider_register);
```

Main providers and their domains:

| Provider | Purpose |
|---|---|
| `peripheral_provider` | sensor subscribe, motor, charger, audio |
| `control_provider` | BLE HID, BT speaker, OS-level control |
| `setting_provider` | persisted user settings, power save mode |
| `notify_provider` | notification ring (RAM) + dismissed ring (flash) |
| `voice_provider` | bloc_v2t (voice → text) interface |
| `skaiwalk_provider` | AI chat, skai widget |
| `bloc_file_system` | DFS mount + file ops |
| `watch_sys_sync` | sync `SkaiWatchSys` global state |

---

## Communication protocol (`modules/communicate/`)

### Frame layout

- **L1**: 8-byte header `(magic 0xAB, ver, len, CRC16, seq)` + payload
- **L2**: `command_id(1) + ver(1) + key(1) + len(2) + value`

### Top-level command IDs

`communicate_parse.h`:

| Hex | Name | Direction |
|---|---|---|
| 0x01 | FIRMWARE_UPDATE_CMD_ID | phone → watch (OTA) |
| 0x02 | SET_CONFIG_COMMAND_ID | bidirectional |
| 0x03 | BOND_COMMAND_ID | bond flow |
| 0x04 | NOTIFY_COMMAND_ID | phone → watch (notifs, status) |
| 0x05 | HEALTH_DATA_COMMAND_ID | watch → phone (HR, steps) |
| 0x06 | FACTORY_TEST_COMMAND_ID | factory line |
| 0x07 | CONTROL_COMMAND_ID | bidirectional |
| 0x0a | BLUETOOTH_LOG_COMMAND_ID | watch → phone |
| 0x0b | WEATHER_INFORMATION_ID | phone → watch |
| 0x20 | SKAI_LINK_COMMAND_ID | AI chat layer |
| 0xFE | TEST_FLASH_READ_WRITE | dev only |
| 0xFF | TEST_COMMAND_ID | dev only |

### Dispatch

`L2_frame_resolve` → `resolve_<bond/setting/health/notify/control/SkaiLink>_command`(each `parse_*.c` switch on key).

### Send

Unified through `commu_send_*` in `communicate_task.c`, declared in
`communicate_task.h`. All sends guarded by `commu_can_send()` (connected
+ BLE DFU not running).

### Adding / removing a key

Must do **all** in one commit:

1. `communicate_parse_<group>.h` — add enum (**hex value stable, don't change**)
2. `communicate_parse_<group>.c` — add case
3. If outbound: add `commu_send_*` function + caller
4. Phone side `C:\work\SkaiLink\lib\shared\watch\communicate\communicate_protocol.dart` — confirm matching key. Watch sending key that dart doesn't recognize = silently dropped.

---

## Code-level conventions

| Rule | Why |
|---|---|
| `kReleaseMode` gate | Dev-only feature (MSH cmd, PPG raw, gesture tuning) wrap `#if !kReleaseMode`. Release auto-strips |
| `#ifdef APP_ID_*` gate | App not enabled on current board → references must `#ifdef` wrap (else compile error) |
| `MSH_CMD_EXPORT(name, "help")` | New shell command. Most under `#if !kReleaseMode` |
| `lvgl_send_msg({.type=LVGL_MSG_TYPE_*, .data=...})` | Cross-thread UI ops. Handler in `ui_handler.c` |
| `peripheral_provider.subscribe_*` or `interact_sensor_subscription({.thread_safe=1})` | Cross-CPU sensor subscribe |

---

## Common tasks quickref

- **新手機指令**: dart 加 key → watch `parse_*.h` 加 enum (照 hex 順序) → `parse_*.c` 加 case → 視需要加 `commu_send_*` 回應
- **新 GUI app**: `hcpu/gui_apps/<name>/`(參考 `flashlight/`)→ board.h 定義 `APP_ID_*` → 加進 `lv_app_list_layout.c` / `lv_instruction_list_layout.c` enum
- **新馬達 pattern**: `watch_system_interact.c` 加 `motor_pattern_*`,header extern,caller 直接 call
- **新感測器訂閱**: `peripheral_provider.subscribe_*`(同步)或 `interact_sensor_subscription({.thread_safe=1})`(thread-safe)
- **跨 thread 操作 UI**: `lvgl_send_msg({.type=LVGL_MSG_TYPE_*, .data=...})`
