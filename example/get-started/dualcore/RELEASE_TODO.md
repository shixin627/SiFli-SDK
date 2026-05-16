# Dualcore Watch — Release TODO

> Self-contained handoff doc. The receiving AI starts here cold.
> All file paths are relative to repo root unless noted.

---

## 0. Read these first (5 min)

| File | Why |
|---|---|
| [example/get-started/dualcore/CLAUDE.md](CLAUDE.md) | Project structure, BLOC pattern, build commands, **the "PC 模擬器測試框架" section is critical** |
| [example/get-started/dualcore/CHANGELOG.md](CHANGELOG.md) | What was done recently (2026-05-15, 2026-05-16 entries especially) |
| [example/get-started/dualcore/project/hcpu/proj.conf](project/hcpu/proj.conf) | Real-watch HCPU build config |
| [example/get-started/dualcore/project/hcpu/pc_hcpu/proj.conf](project/hcpu/pc_hcpu/proj.conf) | PC sim HCPU build config |
| [customer/boards/eh-lb56xu/bsp_board.h](../../../customer/boards/eh-lb56xu/bsp_board.h) | Current active board config (BOARD_VER_28) |

**Conventions you MUST follow:**
- Code style is Clean Code, *not* dogmatic — see "Clean Code 精神" section in `CLAUDE.md`
- Never edit `external/` deps unless explicitly told
- For any file with Chinese / GBK encoding (vendor code, some legacy), use `sed` not Edit/Write to preserve byte-level encoding
- Conventional commit prefix: `feat:` / `fix:` / `chore:` / `refactor:` / `docs:`
- Include `Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>` trailer
- **NEVER push to remote without explicit user permission**
- **NEVER `git rebase -i`, `git reset --hard`, `--no-verify`** unless the user explicitly asked

---

## 1. PC simulator test framework — use this for everything you can

A full MSH-driven test framework lives under `project/hcpu/` + `src/modules/tests/`. **Use it before reaching for real hardware.**

### One-shot dev loop
```cmd
project\hcpu\_dev_test.cmd                                rebuild + relaunch sim
project\hcpu\_dev_test.cmd -nobuild                       just relaunch (skip scons)
project\hcpu\_dev_test.cmd -nobuild -cmd "list_apps"      send one MSH command
project\hcpu\_dev_test.cmd -script my_test.txt            run batched cmds (newline-separated)
project\hcpu\_dev_test.cmd -nobuild -screenshot foo.png   capture sim window
project\hcpu\_dev_test.cmd -snapshot all                  navigate every app, snapshot each
```

### MSH command inventory (all gated by `BSP_USING_PC_SIMULATOR`)

| Group | Commands |
|---|---|
| Touch | `touch_tap x y` / `touch_press x y` / `touch_release` / `touch_move x y` / `touch_swipe x1 y1 x2 y2 [ms]` / `touch_status` |
| Navigation | `list_apps` / `goto_app <id>` / `app_status` (framework also has `app_run` / `app_exit` / `app_goback`) |
| Back | `back` / `gesture_back` |
| Fake devices | `dev_add <name> [type] [conn]` / `dev_connect <idx>` / `dev_disconnect <idx>` / `dev_clear` / `dev_list` |
| Fake notifications | `notif_inject <title> <msg> [type]` / `notif_clear` / `notif_list` |
| Dismissed ring | `notif_dismiss <id>` / `notif_check_dismissed <id>` / `notif_dismiss_clear` / `notif_dismiss_save` |
| Fake sensors | `battery_set 0..100` / `battery_volt_set <mv>` / `charge_set 0|1` / `hr_set <bpm>` |

### Existing test fixture scripts (under `project/hcpu/`)
- `_smoke_test.txt` — full MSH sanity sweep
- `_persist_a.txt` + `_persist_b.txt` — dismissed-ids cross-restart persistence
- `_bug3_test.txt` / `_bug3_flash_test.txt` — dedup ring + flash write
- `_ls_assets.txt` / `_ls_fonts.txt` — `/assets/*` mount sanity

### What PC sim CANNOT test
- Real BLE pairing / connection / ANCS — no radio
- Real motor / haptic feel
- Real ADC battery — use `battery_set` / `charge_set` fixtures instead
- Real IMU / PPG / sensor data
- OTA flash partitions
- Power consumption
- HID over BLE host-side behavior (iOS / Android / Windows / macOS variability)
- eZIP-decoder-dependent UI refresh (notification list live update) — see B4 below

---

## Phase 0 — Ship blockers (must complete before "release" build)

### B1: Switch `kReleaseMode` from 0 → 1 and run full regression

**Why blocker**: current builds are debug. Release mode strips MSH dev cmds + extra LOG + some asserts + dev-only features. Shipping with `kReleaseMode=0` means leaking debug surface + possibly unintended behavior.

**Files**
- [customer/boards/eh-lb56xu/bsp_board.h:73](../../../customer/boards/eh-lb56xu/bsp_board.h#L73) — flip to 1
- Grep for `#if !kReleaseMode` and `#if kReleaseMode` to find every gated code path
- Pay attention to `gui_apps/test/`, `modules/tests/`, MSH commands like `aw32001`, `utest_*`, etc.

**Approach**
1. Flip `kReleaseMode` to 1
2. Try a watch build:
   `cmd /c project\hcpu\_watch_build.cmd -j8`
3. Fix any compile errors (likely missing `extern` or `#ifdef` mismatch)
4. **Run PC sim smoke** (note: PC sim still has `kReleaseMode=0` paths — those are mostly fine if PC config differs)
5. Try a clean watch flash + boot. Verify:
   - No MSH dev cmds in tshell (they should be stripped)
   - Existing UI flows still work end-to-end
   - No assert fires on common paths

**Acceptance**
- Watch build clean (no errors, warnings count similar to before)
- Boot to watchface
- Swipe through all visible apps without crash
- BLE pair + receive one notification + dismiss it

**Don't**
- Don't strip MSH cmds manually — they should auto-disappear via `#if !kReleaseMode`. If they don't, that's a bug in the cmd registration not in `kReleaseMode`.

---

### B2: Heart rate decision — re-enable, document removal, or fix LCPU memory

**Why blocker**: commit `aa3138f02` "暫時關閉心率因為記憶體不夠" set SPO₂ / soft-ADT / protocol / AGC to 0 in [customer/peripherals/sensor/gh3018/gh30x_example_config.h:52-56](../../../customer/peripherals/sensor/gh3018/gh30x_example_config.h#L52). The commit message says "temporary" but it's been in main. If product still advertises HR, this is a mismatch users will complain about.

**Three options (need user / product decision before coding)**
1. **Fix LCPU memory bloat** — `_watch_build.cmd` shows LCPU is ~75 KB bigger under GCC than Keil microlib due to newlib `localtime`/`mktime` in `hr_service.c` + `alarm_manager_service.c`. Either (a) `USE_MICROLIB=True` in `project/lcpu/rtconfig.py` to use nano.specs (unverified side-effects), or (b) rewrite LCPU to avoid `localtime`. Then re-enable the gh3018 features.
2. **Ship without HR** — remove HR app from launcher (`gui_apps/main/lv_app_list_layout.c` / `lv_instruction_list_layout.c`), update product docs.
3. **Ship with HR crippled** (current state) — document as "HR coming soon" in release notes.

**Files to read before deciding**
- [customer/peripherals/sensor/gh3018/gh3018.c:107,116](../../../customer/peripherals/sensor/gh3018/gh3018.c#L107)
- [customer/peripherals/sensor/gh3018/gh30x_example_config.h:52-56](../../../customer/peripherals/sensor/gh3018/gh30x_example_config.h#L52)
- [src/modules/algorithm/health/](src/modules/algorithm/health/)

**Acceptance** depends on which option you pick. Option 1 needs HR algorithm regression on real PPG.

---

### B3: OTA rollback — dual-partition + bootloader rollback flag

**Why blocker**: currently if a new image bricks on boot, WDT reset → boots back into same brick → infinite loop. Field-undebuggable.

**Scope is LARGE** — this is bootloader work, not just app code.

**Investigation order**
1. Read `customer/bootloader/` (or wherever `bootloader.bin` is built — look in `project/hcpu/build_sf32lb56w-watch_hcpu/`) to understand current boot flow
2. Find FAL partitions for `download` vs main image — see `Register*download to mtd device` in boot log
3. Design boot flag (e.g. "tried-new-image" + "boot-succeeded" timestamp). If watchdog reset within N seconds of first boot of new image, revert flag → next boot uses old image
4. The OTA verify path at [communicate_update_image.c:614](src/modules/communicate/communicate_update_image.c#L614) needs to set "tried-new-image"

**Acceptance**
- Brick a test image (e.g. infinite assert at boot)
- Flash via OTA
- Watch should reboot 2-3 times then fall back to old image
- BLE pair + functional verify on rolled-back image

**Don't**
- Don't attempt this without reading bootloader source first. This task can take days; budget accordingly.

**PC sim test**: limited. Can simulate the flag logic in isolation but not the actual NAND swap.

---

### B4: Verify notification-storm boot crash fix on real hardware

**Why blocker**: commit `ed1acbff1` "嘗試修復開機收到通知直接死當問題(待驗証)" added a 200ms debounce timer in [lv_message_list_layout.c:1801+](src/hcpu/gui_apps/app_layout/lv_message_list_layout.c#L1801). Author marked **待驗證** = "needs verification". If unfixed, boot with cached ANCS dump → FreeType cache thrash → heap corruption → crash.

**Verification approach (real hw)**
1. Pair watch with phone that has 10+ pending notifications
2. Reboot watch (BLE will reconnect, phone will dump cached notifications in <1s burst)
3. Watch should boot cleanly, show all notifications, no crash
4. Repeat 5x with different notification counts (5, 10, 20, 50)

**Investigation hooks if it crashes**
- Look at `app_ft_m` (FreeType cache, 600 KB)
- `lv_freetype_clean_cache` race with EPIC blits
- Check if the debounce window (200 ms) is wide enough — bump to 500 ms or 1 s if needed
- Read the comment block above the debounce timer for context

**PC sim test**: doesn't apply (PC has no FreeType / EPIC behavior; notification UI live refresh is broken on PC for unrelated reasons — eZIP decoder)

---

### B5: HID Mouse cross-OS verification

**Why blocker**: commit `84d39c0f1` "feat:更新滑鼠ui(功能未確認)" — author flagged 896-line addition / 574-line deletion as unverified.

**Files**
- [src/hcpu/gui_apps/hid_mouse/hid_mouse.c](src/hcpu/gui_apps/hid_mouse/hid_mouse.c)
- [src/modules/model/ble_hid.c](src/modules/model/ble_hid.c) (HID descriptor)

**Test matrix (real hw)**
- iOS 17+ — pair as BLE mouse, verify cursor movement, scroll, tap
- Android 13+ — same
- Windows 11 — same
- macOS 14+ — same

Each: 5 min of normal use without disconnect.

**Acceptance**: 4/4 OSes show cursor movement + tap correctly. No reconnect storm.

**PC sim test**: PC can launch the app (`goto_app mouse`) and verify UI renders. Real HID descriptor delivery is BLE-only.

---

## Phase 1 — RC blockers (before internal testing)

### R1: OTA min-battery gate

**Easy fix**: 2-line guard in [communicate_update_image.c:135-141](src/modules/communicate/communicate_update_image.c#L135) `mark_ota_started()`.

```c
// Refuse to start OTA below 15% — half-flash with empty battery bricks the unit.
if (SkaiWatchSys.battery_level < 15 && !SkaiWatchSys.is_charging) {
    notify_update_status_to_client(STATUS_LOW_BATTERY);  // or equivalent
    return;
}
```

Check what status codes the phone-side dart layer expects; coordinate.

**PC sim test**:
```cmd
project\hcpu\_dev_test.cmd -nobuild -cmd "battery_set 5"
# trigger DFU start... need a fake DFU trigger MSH cmd or skip on PC
```

---

### R2: Orphan command IDs — 17 declared keys with no handler

**Files**
- [src/modules/communicate/communicate_parse_notify.{h,c}](src/modules/communicate/communicate_parse_notify.h) — 13 NOTIFY orphans
- [src/modules/communicate/communicate_parse_control.{h,c}](src/modules/communicate/communicate_parse_control.h) — 4 CONTROL orphans
- [src/modules/communicate/communicate_parse_setting.{h,c}](src/modules/communicate/communicate_parse_setting.h) — 7 SETTING orphans (most are response-only, audit each)

**Approach**
1. For each orphan key: check if phone (dart side at `C:\work\SkaiLink\lib\shared\watch\communicate\communicate_protocol.dart` per CLAUDE.md) actually sends it
2. If phone sends: implement handler or explicit `LOG_W("unhandled key 0x%02x")` + drop
3. If phone never sends (response-only key declared for symmetry): document with `/* response-only; no handler needed */` comment

**Highest priority orphans** (likely actually used):
- `KEY_BATTERY_CHARGE_STATUS` (0x06, NOTIFY) — needed by phone status UI?
- `KEY_REMOTE_INPUT` (0x20, NOTIFY)
- `KEY_REQUEST_CALENDAR` (0x2c, NOTIFY)
- `KEY_REQUEST_WEATHER` (0x2b, NOTIFY)
- `KEY_RETURN_ALARM_SETTINGS` (0x1a, SETTING)
- `KEY_PHONE_MEDIA_CONTROL` (0x05, CONTROL)
- `KEY_TAKE_PHOTO` (0x04, CONTROL)
- `KEY_FIND_PHONE` (0x02, CONTROL — phone→watch direction)

**Acceptance**: every declared L2 key either has a case in the dispatcher OR a "// response-only" comment.

**PC sim test**: write MSH cmd to inject raw L2 frames (need new helper).

---

### R8: Factory test routine

**Why needed**: production line cannot manually verify LED / motor / sensor / BLE pair on every unit.

**Approach**: piggy-back on existing `gui_apps/test/app_test.c` (`DEV-ONLY` per app maturity table). Build a "mfg mode" entry point that auto-runs:
1. LED blink (each LED, 1s)
2. Motor pulse (each pattern, 1s)
3. IMU read (verify non-zero accel + gyro)
4. PPG read (verify sensor responds to finger)
5. ADC battery read (verify in [3.0V, 4.5V])
6. Touch screen 4-corner tap
7. BLE advertise + wait 30s for test rig to connect
8. Report pass/fail via UART or BLE

**Entry trigger** — usually a magic key combo at boot, or a build-time `MFG_BUILD` flag.

**Acceptance**: production line runs `MFG_BUILD=1` firmware, gets pass/fail report in <60s per unit.

**PC sim test**: can verify the UI flow + state machine. Real sensor calls fail / no-op on PC.

---

## Phase 2 — Production risks (can ship + patch in 1.1.52)

### R3: Lost commands during BLE disconnect
**File**: [src/modules/communicate/communicate_task.c](src/modules/communicate/communicate_task.c)
**Fix**: add bounded ring queue for `commu_send_health_data` etc.; flush on reconnect.
**Acceptance**: disconnect for 5min → reconnect → queued data arrives at phone.

### R4: Bond ACK before state-commit race
**File**: [src/modules/communicate/communicate_parse_bond.c:72-73](src/modules/communicate/communicate_parse_bond.c#L72)
**Fix**: reorder so `interact_bonded()` runs before `commu_send_bond_success()`, or add rollback on failure.

### R5: iOS pairing TODO
**File**: [src/modules/communicate/communicate_parse_setting.c:54](src/modules/communicate/communicate_parse_setting.c#L54)
**Fix**: implement `le_bond_pair()` call for iOS or remove the TODO with explanation.

### R6: Bonded device overflow UI
**File**: [src/modules/model/ble_device_manager.c:199-204](src/modules/model/ble_device_manager.c#L199)
**Fix**: when `ble_dev_mgr_add_device` returns -2, surface a "Device list full — remove one in settings" UI prompt. Also consider LRU eviction.
**PC sim test**: `dev_add` x9 — verify UI feedback.

### R7: No time-sync on reconnect
**Files**: [src/hcpu/main.c](src/hcpu/main.c) (look for `on_ble_connected` or `gap_state_change`), [src/modules/communicate/communicate_parse_setting.c:59-83](src/modules/communicate/communicate_parse_setting.c#L59)
**Fix**: on reconnect, send `commu_send_time_request` (or whatever the read-time command is).

### R9: Version not in binary header
**Files**: [src/modules/model/watch_global_data.h:14-17](src/modules/model/watch_global_data.h#L14), `package_watch_firmware.py`
**Fix**: write version into a fixed offset in `main.bin` header (or `ftab.bin`) so phone can validate offline.

### R10: Notification storm debounce review
Tied to **B4** — verify the 200 ms window. If B4 verification still shows crashes, bump to 500 ms or rebuild the notification refresh path entirely.

---

## Phase 3 — Polish (queue for 1.1.52)

| ID | Task | File |
|---|---|---|
| P1 | Typo `"cancel": "Canel"` → `"Cancel"` | [src/hcpu/resource/strings/en_us.json:27](src/hcpu/resource/strings/en_us.json#L27) |
| P2 | Implement `// TODO: start power down procedure` | [src/hcpu/gui_apps/watch_demo.c:665](src/hcpu/gui_apps/watch_demo.c#L665) |
| P3 | Delete `#if 0` dead code blocks | [src/hcpu/gui_apps/message/app_message.c:740](src/hcpu/gui_apps/message/app_message.c#L740), [src/hcpu/gui_apps/app_layout/lv_instruction_list_layout.c:3518](src/hcpu/gui_apps/app_layout/lv_instruction_list_layout.c#L3518), [src/modules/bloc/bloc_control.c:262,365,397](src/modules/bloc/bloc_control.c#L262) |
| P4 | Board TODOs (heap calc, flash5, pinmux) | [customer/boards/eh-lb56xu/bsp_board.h:36-38](../../../customer/boards/eh-lb56xu/bsp_board.h#L36), [bsp_init.c:89](../../../customer/boards/eh-lb56xu/bsp_init.c#L89), [bsp_pinmux.c:189,202](../../../customer/boards/eh-lb56xu/bsp_pinmux.c#L189) |
| P5 | ELM filesystem boot warning ("no space to register") | DFS slot table or remove unused elm filesystem registration |
| P6 | `dfs_win32_ops` missing `mkdir` op (PC sim only) | [rtos/rtthread/bsp/sifli/drivers_pc/dfs_win32.c:525](../../../rtos/rtthread/bsp/sifli/drivers_pc/dfs_win32.c#L525) |
| P7 | Battery EMA filter feels sluggish (80/20 → 60/40?) | [src/modules/bloc/bloc_battery.c:87-88](src/modules/bloc/bloc_battery.c#L87) — only if user complains |

---

## Phase 4 — v1.2 backlog (post-release)

| ID | Task | Status |
|---|---|---|
| G1 | Finish `camera` app (needs HW module) | WIP |
| G1 | Finish `hid_mouse` (incl. B5 fixes) | WIP |
| G1 | Register `skai` app with `BUILTIN_APP_EXPORT` + finish AI flow | WIP, no entry point currently |
| G2 | Native iOS ANCS support (currently all notifications via phone-app L2) | Gap |
| G3 | Implement reverse `KEY_FIND_PHONE` etc. | Gap |
| G4 | Add app-layer CRC to BLE protocol | Gap (relies on ATT CRC currently) |
| — | OTA per-component version manifest (HCPU/LCPU/bootloader compat matrix) | Gap |

---

## Investigation tasks (deferred from earlier)

### Battery accuracy ("不準")

User reported battery % display is inaccurate. 5 candidate causes ranked in 2026-05-15 conversation; full analysis at top of CHANGELOG.md.

**Recommended first step**: run on real hw, compare `get_cc_volt` (aw32001 charger IC direct read) vs UI display:
```
> get_cc_volt
# returns charger IC mV reading
# then check UI battery display
```

If they disagree by constant factor → divider ratio wrong (`bloc_battery.c:29-33`).
If they agree but UI % is wrong → curve mismatch (`customer/boards/eh-lb56xu/battery_table.c` or similar).

**Files**
- [src/modules/bloc/bloc_battery.c](src/modules/bloc/bloc_battery.c)
- `customer/peripherals/charger/aw32001/aw32001.c`
- `customer/boards/eh-lb56xu/battery_table.c` (verify the lookup table exists and matches actual cell)

### Notification UI live refresh on PC sim

Currently broken because PC has no eZIP decoder for the icon resources. See CHANGELOG 2026-05-16 entry. Low priority — only affects PC sim test iteration speed; real hw works.

**Fix path** (if you want to tackle it): write a `lv_img_decoder` for `eZIP_RGBARGB565A` format and register it in `middleware/lvgl/lv_win32/img_dec.c`. Pattern: existing file-based decoder there.

---

## Files you MUST NOT touch

These are in working tree but NOT this work's scope — they belong to other in-progress work:

- `customer/boards/eh-lb56xu/bsp_board.h` (someone is editing board config)
- `example/get-started/dualcore/project/lcpu/proj.conf`
- `example/get-started/dualcore/src/modules/bloc/bloc_system_perception.c`
- `example/get-started/dualcore/src/modules/communicate/communicate_parse_notify.h` (and `task.c/h`)
- `example/get-started/dualcore/src/hcpu/resource/fonts/freetype/font_partition_dsc.c` (auto-generated)
- Build artefacts in `drivers/hal/`, `project/hcpu/vc140.pdb`, `_pc_build.log`

**Exception**: B1 (`kReleaseMode`) requires editing `bsp_board.h`. Coordinate with whoever else is working on that file before committing — `git log --oneline -- customer/boards/eh-lb56xu/bsp_board.h | head -10` to see recent authors.

---

## Build commands cheat-sheet

```cmd
REM PC sim (fast iteration)
project\hcpu\_pc_build.cmd -j8

REM Watch firmware (Keil, production)
project\hcpu\_watch_build.cmd -j8
REM → project/hcpu/build_sf32lb56w-watch_hcpu/main.bin

REM Generate Keil .uvprojx for IDE debug
project\hcpu\_watch_mdk5.cmd -j8

REM Package release artefacts
python project\hcpu\package_watch_firmware.py
REM → watchOS/ tree
```

Build wrappers auto-detect repo root (no longer hardcoded to `C:\work\SiFli-SDK`).

---

## Reporting back

When done with a task, commit with conventional format:
```
git commit <files> -m "fix(scope): subject

  Body explaining why.

  Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
"
```

Don't push. Don't open PRs. Let the user do that.

Update this file as you go — strike completed items, add notes if scope changed.
