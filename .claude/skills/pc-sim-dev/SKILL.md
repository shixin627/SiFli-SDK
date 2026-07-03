---
name: pc-sim-dev
description: >
  PC simulator (Win32 LVGL sim) development workflow for the dualcore watch
  firmware. Use when the user wants to develop, test, or debug a feature
  using the PC simulator instead of flashing to the watch — phrases like
  "用模擬器開發 X", "在 PC sim 試 X", "幫我跑模擬器看 X", "model 在 sim
  上加..." or any time user wants to iterate on UI/logic without hardware.
  TRIGGERS: developing a new gui_app on PC, fixing a sim-only crash, adding
  a NULL guard for a stub fn pointer, regenerating pc_link_stubs.c after
  enabling a new module, or just "let me see this on sim".
---

# PC Simulator Development for Dualcore Watch

The dualcore watch firmware can build a **Win32 LVGL simulator** that boots
through RT-Thread and renders the watch UI in a 466×466 window — without
hardware. This skill captures the full workflow: build, run, drive input,
read logs, fix common stub-related crashes.

## Build & run loop

The simulator project is the same `example/get-started/dualcore` — just
build with `--board=pc` instead of an ARM board.

**Build (from project hcpu dir):**
```
cd example/get-started/dualcore/project/hcpu
scons --board=pc -j8
```

The wrapper `_pc_build.cmd` in that dir replicates ConEmu's env init for
running outside ConEmu (used by Bash/PowerShell tool):
```
cmd.exe /c "C:\work\SiFli-SDK\example\get-started\dualcore\project\hcpu\_pc_build.cmd -j8"
```
It writes full output to `_pc_build.log` (already in `.gitignore` semantics).

**Run:**
```
cd build_pc_hcpu
./main.exe
```
The .exe stays running with an LVGL window. Console output (RT-Thread
logs) is teed to `pc_console.log` next to the .exe — `tail -f`-able even
across crashes (flushed on every newline by `uart_console.c`).

**Stop:** `Stop-Process -Name "main" -Force` or close the window.

## File layout — where things live

**SDK-level fixes** (these are all already committed; don't undo):
- `msvc_setup.bat` — VS2022 14.16.27023 + Win SDK 22621 paths
- `rtos/rtthread/include/rtdef.h` — `<stdint.h>` on MSVC branch
- `rtos/rtthread/bsp/sifli/drivers_pc/board.h` — `flash_map.h` → `custom_mem_map.h`
- `middleware/lvgl/littlevgl2rtt.c` — MSVC weak-symbol fallback
- `middleware/data_bus/public/data_service_main.c` — skip HWMAILBOX on PC
- `middleware/simulator/platform.c` — `<shell.h>` only with `RT_USING_FINSH`
- `middleware/lvgl/lvsf/lvsf_gesture.c` — has `DBG_TAG`/`DBG_LVL` for LOG_*

**PC board files** (`customer/boards/pc/`):
- `SConscript` — adds `pc/hcpu/` to CPPPATH so headers below resolve
- `hcpu/bsp_board.h` — board constants stub (CUSTOMER_BOARD_VER, IMU_*, GPIO pins as -1)
- `hcpu/custom_mem_map.h` — KVDB region offset stubs (PC's ptab.json is empty)
- `hcpu/drv_touch.h`, `hcpu/drv_usart.h` — minimal stubs for ARM driver headers
- `hcpu/unistd.h` — POSIX→MSVC mapping with **function-like macros only**
  (must NOT be `#define read _read` — that breaks RT-Thread section pragmas
  in `INIT_*_EXPORT`)

**Project-level** (`example/get-started/dualcore/project/hcpu/`):
- `pc_hcpu/proj.conf` — Kconfig overrides for PC (BLE/BT/TFLite/EZIPA/PM off,
  COMMUNICATE on, FlashDB LIBC mode, `APP_TRANS_ANIMATION_OVERWRITE` etc.).
  Also turns FINSH/MSH **on** so tshell drives main.exe via stdin —
  `CONFIG_RT_USING_FINSH=y` + `CONFIG_FINSH_USING_MSH=y` (the dualcore
  `proj.conf` shared base has FINSH off; the PC override flips it on).
- `rtconfig_project.h` — MSVC defines: `RT_HEAP_SIZE`, `SOC_BF0_HCPU`,
  `BSP_USING_COMMUNICATE`, warning suppression
- `SConstruct` — skips LCPU child project on PC (`if not GetDepend('BSP_USING_PC_SIMULATOR')`)
- `SConscript` — would skip `src/modules`, but it's currently kept on
- `_pc_build.cmd` — env-init wrapper to run scons outside ConEmu
- `_pc_swipe.ps1` — DPI-aware mouse scripting (see Input below)
- `_send_to_main.py` — Win32 keystroke injection helper (AttachConsole +
  WriteConsoleInputW). Status: writes events successfully and main.exe
  consumes them per `GetNumberOfConsoleInputEvents`, but FINSH on the
  other end doesn't always echo/process — use only as a starting point,
  prefer typing into the main.exe console window directly for now.
- `_genstub.py` + `_syms_clean.txt` — regenerate `pc_link_stubs.c`

**Project sources** (`example/get-started/dualcore/src/hcpu/`):
- `main_pc.c` — provides `app_main()` (called from `simulator/application.c`).
  `int main()` is provided by `simulator/startup.c`.
- `pc_link_stubs.c` — auto-generated stubs for ~130 symbols normally
  provided by ARM-only modules (BLE stack, voice/skai/gesture apps, IPC).
  **Always read the file before editing — symbols come and go as modules
  enable/disable.**
- `gui_apps/SConscript` — `pc_skip` tuple lists apps excluded from PC
  build (gesture, speech, skai, recorder, camera, incoming_call,
  exercise, media, hid_mouse, game)

## Input automation: `_pc_swipe.ps1`

The watch sim window receives synthetic mouse events from PowerShell.
PowerShell process must be DPI-aware (script handles this with
`SetProcessDpiAwareness(2)` at start) — without it, `SetCursorPos` coords
get DPI-scaled and miss the window.

```powershell
& _pc_swipe.ps1 -dir up           # finger up (bottom→top)
& _pc_swipe.ps1 -dir down
& _pc_swipe.ps1 -dir left         # finger left  (right→left)
& _pc_swipe.ps1 -dir right        # finger right (left→right)
& _pc_swipe.ps1 -dir right -edge  # hug very-left edge (lvsf_gesture back-bar)
& _pc_swipe.ps1 -tap -x 50 -y 50  # tap at client% (0..100)
& _pc_swipe.ps1 -tap -px 250 -py 250  # tap at LVGL pixel (0..LV_HOR_RES-1)
```

Behavior reference (PC sim):
- **Right swipe** from home → instruction list
- **Left swipe** from instruction list → home (tile-view scroll, NOT lvsf_gesture)
- **Tap (50, 8)** → status bar (notification_status_bar_cb area: 1)
- Up/down swipes from home **may not** open launcher in this UI — control
  center is reached via status-bar tap, not swipe gesture

## The fix loop for crashes

**1. Run, observe crash:**
```
./main.exe   # let it idle, do interaction, watch for silent exit
```
Exit code 139 (POSIX) / silent close (Windows) = segfault.

**2. Read the tail of pc_console.log:**
```
tail -30 build_pc_hcpu/pc_console.log
```

**3. Identify the last log before crash:**
Any line right before the gap — usually a "DBG ... before X" probe or a
LOG_I from a callback — points to the crashing function.

**4. Common crash patterns:**

| Symptom | Fix |
|---|---|
| `xxx_provider.method()` segfault | NULL-guard the fn pointer: `if (provider.method) provider.method();` |
| `myLancher[app_index_X].pagetileview` deref | NULL-guard or early-return — apps in `pc_skip` never set their pagetileview |
| `watch_sensor_motion_data->...` deref | NULL-guard — IMU callback never registered on PC |
| `lv_obj_*(some_widget_builder_result)` crash | The widget builder is a stub returning NULL; change stub to `return parent ? lv_obj_create(parent) : NULL;` so subsequent LVGL ops don't NULL-deref |
| `lv_obj_del(p_app->media_widget)` on first call | NULL-guard with `lv_obj_is_valid` (just-zero'd struct) |
| Compile error: `__attribute__((weak))` / `((used))` | `#ifdef _MSC_VER` skip the attribute or wrap with no-op equivalent |
| Compile error: `bf0_ble_*.h` / `bf0_sibles*.h` not found | Wrap include in `#ifdef RT_USING_BLUETOOTH` and any code referencing BLE types in same guard |
| Compile error: `unistd.h` not found | Already mapped via `customer/boards/pc/hcpu/unistd.h` — should resolve. If new file uses `read()/write()` directly, it's already aliased to `_read/_write` |
| LNK2019 `_setVoice2Text` / `_app_voice_set_voice2text_intent` after enabling FINSH | These were dead-stripped before FINSH; once `MSH_CMD_EXPORT` macros expand, `watch_system_interact` MSH handlers reference them. Stubs added to `pc_link_stubs.c` — keep them. |

**5. Add probes when stuck:**
Drop `LOG_I("dbg before X");` / `LOG_I("dbg after X");` around suspect
calls, rebuild, rerun, read log. **After fixing, downgrade probes to
`LOG_D`** (per user preference) — see also feedback memory.

## Adding a new feature on PC sim

When user says "幫我用模擬器開發 <feature>":

**1. Plan: figure out what the feature touches:**
- Pure UI? (new gui_app or LVGL widget) → easiest
- Needs BLE / sensor / IPC? → must be mocked at the BLOC provider level

**2. If new gui_app:**
- Create `src/hcpu/gui_apps/<name>/{app_<name>.c, SConscript}`
- Register with `BUILTIN_APP_EXPORT(LV_EXT_STR_ID(<name>), IMG, APP_ID_<NAME>, app_main)`
- Add `APP_ID_<NAME>` define to `customer/boards/pc/hcpu/board.conf` or per board
- If the app needs BLE/sensor/HAL deps → add to `pc_skip` tuple in
  `gui_apps/SConscript`. Otherwise it'll auto-build.
- Add icon to `gui_apps/<name>/app_<name>_icon.c` etc.

**3. If touching BLOC provider:**
- Real impl in `src/modules/bloc/bloc_<name>.c` — check it doesn't
  `#include` BLE/HAL headers (or wrap with `#ifdef RT_USING_BLUETOOTH`)
- All call sites: `if (provider.fn) provider.fn(args);` — provider may
  not be registered on PC

**4. If new global pointer (sensor data, BLE state, etc.):**
- Add NULL-guard at access sites (PC won't have the registration callback)
- OR add a default-init in `main_pc.c` if appropriate

**5. After build, before iterating:**
- If link errors: regenerate stubs:
  ```
  cd example/get-started/dualcore/project/hcpu
  # extract symbols from latest _pc_build.log:
  grep "LNK2001\|LNK2019" _pc_build.log | python -c "import re, sys; ..." > _syms_clean.txt
  python _genstub.py
  ```
  Then add the 6-ish "unfound" symbols manually at the end of
  `pc_link_stubs.c` (genstub only finds simple declarations — globals
  and structs need manual stubbing).

**6. Run + verify:**
```
./build_pc_hcpu/main.exe
# new terminal:
./_pc_swipe.ps1 -dir <your gesture>
tail -f build_pc_hcpu/pc_console.log
```

## What's known broken / not testable on PC

- BLE/BT — all stubs, no actual radio. `commu_send_*` no-op.
- TFLite Micro — disabled (zero-size array isn't C-valid on MSVC)
- EZIPA images — disabled (uses HW EPIC HAL)
- Audio (recorder, microphone, speaker, voice recognition) — stubbed
- IMU sensor data, PPG, magnetometer — `watch_sensor_motion_data` NULL
- Cross-CPU services (alarm, watch_sys, hr) — `BSP_USING_DATA_SVC=y` but
  HWMAILBOX disabled, RPC payloads silently dropped
- TFT-only image formats (no_ezip variants)

If user tries to develop something that depends on the above, recommend
either:
- Mock at the provider/callback level
- Test on the ARM build instead

## Git milestones already on the branch

Recent commits (relative to PC sim work):
- `feat: PC simulator build for dualcore (boots to message_list)`
- `feat: PC sim watch face renders (466x466, stable LVGL idle)`
- `feat: PC sim — slide to instruction list works, console tees to file`
- `fix: PC sim — actually enable lvsf_gesture (Kconfig override + dup stub)`
- `chore: PC sim swipe script — full 4-direction + DPI-aware`

Don't push to origin without user explicit consent.

## Quick reference: common one-shot diagnostic loop

```bash
# 1. Build
cd C:/work/SiFli-SDK/example/get-started/dualcore/project/hcpu
cmd.exe /c "_pc_build.cmd -j8" > /dev/null 2>&1
tail -3 _pc_build.log   # last 3 lines: "scons: done building targets." = OK

# 2. Run in background
cd build_pc_hcpu && rm -f pc_console.log && ./main.exe   # via Bash run_in_background

# 3. Drive input (separate PowerShell call)
& _pc_swipe.ps1 -dir right

# 4. Read log
tail -30 build_pc_hcpu/pc_console.log

# 5. Check still alive
Get-Process -Name "main"
```

Iterate steps 3-5 as needed. Step 1 only when source changes.
