# PC Simulator Test Framework

> Build gate: `BSP_USING_PC_SIMULATOR=y`. Production build doesn't link
> any of this in. Designed so an AI agent (or human) can drive the watch
> UI end-to-end through MSH commands without touching real hardware.

整套東西在 `src/modules/tests/` + `middleware/lvgl/lv_win32/lv_touch_sim.{c,h}`.

---

## Build & launch

```cmd
REM PC sim build (MSVC, ~30s incremental)
project\hcpu\_pc_build.cmd -j8
REM → build_pc_hcpu/main.exe (~9.6 MB)
```

Output binary uses MSVC + LVGL win32 backend, embeds RT-Thread + most of
the firmware modules. Real BLE / sensors / flash are stubbed (see
`pc_link_stubs.c` and `main_pc.c`).

---

## Dev iteration loop

`_dev_test.cmd` wraps "rebuild → launch sim → send tshell cmd → capture
console / screenshot" into one shot. The PS1 underneath handles
ExecutionPolicy.

```cmd
project\hcpu\_dev_test.cmd                          REM rebuild + relaunch
project\hcpu\_dev_test.cmd -nobuild                 REM just relaunch
project\hcpu\_dev_test.cmd -cmd "list_apps"         REM rebuild + 1 cmd
project\hcpu\_dev_test.cmd -script my.txt           REM batch (newline separated)
project\hcpu\_dev_test.cmd -nobuild -screenshot foo.png
project\hcpu\_dev_test.cmd -snapshot all            REM nav each builtin app + capture each
```

Existing fixture scripts under `project/hcpu/`:
- `_smoke_test.txt` — full MSH sanity sweep
- `_persist_a.txt` + `_persist_b.txt` — dismissed-ids cross-restart persistence
- `_bug3_test.txt` / `_bug3_flash_test.txt` — dedup ring + flash write
- `_ls_assets.txt` / `_ls_fonts.txt` — `/assets/*` mount sanity

---

## MSH command groups

PC sim ships these on top of RT-Thread's built-in `help` / `ps` / `free` /
`list_thread` / etc.

### Touch injection (`test_touch_sim.c` + `middleware/.../lv_touch_sim.c`)
Independent LVGL pointer indev, coexists with real mouse:

| Cmd | Effect |
|---|---|
| `touch_tap x y` | one tap |
| `touch_press x y` / `touch_release` | sustained touch |
| `touch_move x y` | move while pressed |
| `touch_swipe x1 y1 x2 y2 [ms]` | auto-stepped, LVGL 32 ms sampling |
| `touch_status` | print current sim touch state |

### Programmatic navigation (`dev_navigation.c`)
Skip swipe/tap, jump straight to an app:

| Cmd | Effect |
|---|---|
| `list_apps` | list all `BUILTIN_APP_EXPORT` registered ids — **plural**, sim only. Real hw is singular `list_app` (RT-Thread framework built-in) which lists *running* apps. |
| `goto_app <id>` | `gui_app_run(id)` wrapper, mailbox-safe, FinSH thread OK |
| `app_status` | print current active app (avoids `gui_app_get_actived` thread assert) |
| `app_run` / `app_exit` / `app_goback` / `app_cleanup` | framework built-ins |

### Fake UI data (`fake_ui_data.c`)
Inject the "external world" that's missing on PC (no BLE, no charger, no PPG):

| Cmd | Effect |
|---|---|
| `back` / `gesture_back` | trigger `LVGL_MSG_TYPE_BACK_EVENT` |
| `dev_add <name> [type] [conn]` | fake bonded device. Type: 0=phone 1=computer 2=tablet 3=other |
| `dev_connect <idx>` / `dev_disconnect <idx>` / `dev_clear` / `dev_list` | manage fake device list |
| `notif_inject <title> <msg> [type]` / `notif_clear` / `notif_list` | fake notifications (injected to `_notification_list[]`, UI reads at page open) |
| `notif_dismiss <id>` / `notif_check_dismissed <id>` / `notif_dismiss_clear` / `notif_dismiss_save` | dismissed-ring (32 slots × 48B, RAM + flash, OTA-verify flushed) |
| `battery_set 0..100` / `battery_volt_set <mv>` / `charge_set 0\|1` / `hr_set <bpm>` | fake sensor values via `LVGL_MSG_TYPE_*` |

### OTA rollback (B3 state machine, `boot_rollback.c`)
RTC backup register stubbed by static fake on PC; state transitions still testable:

| Cmd | Effect |
|---|---|
| `boot_meta_dump` | print raw 32-bit + decoded `{state, count, wdt}` |
| `boot_meta_set <state> [count] [wdt]` | force meta for testing |
| `boot_meta_simulate_brick` | set PENDING + count past threshold → next "reboot" should revert |

---

## Known limitations

### Notification drawer doesn't live-refresh
`LVGL_MSG_TYPE_NOTIFICATION` handler chain ends at
`lv_img_set_src(icon, icon_list[notification->type])`. PC sim icon
resource (`resource/images/common/ezip/*.png` → eZIP compressed
`.tmp.c`) declares `LV_IMG_CF_TRUE_COLOR_ALPHA` but bytes are eZIP-encoded
— LVGL default decoder reads wrong, crash. `notif_inject` therefore only
injects data, doesn't emit the refresh msg.

**Fix path**: write variable-based eZIP decoder in
`middleware/lvgl/lv_win32/img_dec.c` (existing pattern is file-based) and
register it.

### Device list callback can't fire from FinSH
`app_setting_device_list.c:dev_mgr_event_cb` does LVGL ops directly. Called
from FinSH thread → crash. `fake_ui_data.c:fire_cb` is therefore no-op.
Workaround: open settings → bluetooth → devices, page open re-reads DB.

### `HAL_Get/Set_backup` is a macro no-op
On WIN32 these are `#define HAL_Set_backup(idx,value)` /
`#define HAL_Get_backup(idx) 0` (`drivers/Include/bf0_hal_hlp.h:279-281`).
B3 boot_rollback module uses a static fake variable in `boot_rollback.c`
on `#ifdef WIN32` to keep the state machine exercisable.

### `bloc_battery` not linked into HCPU build
HCPU `proj.conf` doesn't enable `BSP_USING_BLOC_BATTERY` — bloc_battery.c
only compiles into LCPU. HCPU-side battery reads must go through
`SkaiWatchSys.battery_level_value` / `charger_status`
(`watch_global_data.h`), NOT `battery_get_charge_state()`.

---

## Pre-existing PC sim patches

Local modifications kept across upstream merges:

- **`main_pc.c`** provides `start_ble_rssi_checker` / `stop_ble_rssi_checker`
  no-op stubs. `main.c` is excluded from PC build entirely, but
  `watch_system_interact.c` still extern-references those symbols.
- **`pc_link_stubs.c`** removes 8 `ble_dev_mgr_*` stubs (real fake impls
  live in `fake_ui_data.c` to back the device-list UI). Symbols also
  removed from `_syms_clean.txt` so `_genstub.py` regen won't re-add them.
- **`pc_link_stubs.c`** also drops `start/stop_ble_rssi_checker` dupe
  introduced by the `andrew_v28.46` merge — was causing LNK2005.
- **`external/FlashDB/src/fdb_file.c:290 + 304`** — LIBC mode's
  `fread/fwrite(buf, size, 1, fp) != size` is upstream-buggy (the 3rd
  arg is *count*, return is *elements read*, so any >1-byte op misreports
  `FDB_READ_ERR`). Patched to `!= 1`. Only affects PC sim
  (`PKG_FDB_USING_FILE_LIBC_MODE`); real hw uses fal mode. **After
  upstream merge: verify both lines still read `!= 1`.**

---

## Adding a new fake fixture

1. Find the corresponding `LVGL_MSG_TYPE_*` enum (`modules/model/ui_handler.h`)
2. Read `ui_handler.c::process_lvgl_message` switch and confirm:
   - which `lvgl_msg_handler.handle_*` runs
   - whether that handler touches `icon_list` / `lv_img_set_src` / un-init
     widgets (any → crash on PC)
3. **Safe**: add to `fake_ui_data.c` — `MSH_CMD_EXPORT` + `lvgl_send_msg({...})`
4. **Unsafe**: inject data only (don't send msg), let the UI read at next
   page open. Or stub the dangerous handler before sending.

---

## What PC sim CANNOT verify

Use real hw for these:

- Real BLE pairing / connection / ANCS — no radio
- Real motor / haptic feel
- Real ADC battery — use `battery_set` / `charge_set` fixtures
- Real IMU / PPG / sensor data
- OTA flash partitions (B3 backup-copy is gated `#ifdef BSP_USING_PC_SIMULATOR` returns true without doing the copy)
- Power consumption
- HID over BLE host-side behavior (iOS / Android / Windows / macOS variability)
- eZIP-decoder-dependent UI refresh (notification list live update — see above)

For real-hw log inspection + MSH driving, use `tools/dev_console/`.
