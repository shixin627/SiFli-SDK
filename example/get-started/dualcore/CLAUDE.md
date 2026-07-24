# Skaiwalk Dualcore Watch

SiFli SF32LB56W watch firmware on RT-Thread. HCPU runs LVGL GUI + BLE host + AI inference; LCPU drives sensors / motor / charger. Phone-side counterpart lives in `C:\work\SkaiLink`.

**Read `/AGENTS.md` first** for product mission, canonical vocabulary, hard constraints (if present at repo root).

This file is the **project-specific execution overlay** — commands, file structure, in-house conventions. Cross-project rules (UI, PM, i18n discipline, Clean Code philosophy) live in `~/.claude/skills/` (sk-AI).

## Skill router

| Need | Where |
|---|---|
| Sprint workflow (think / plan / build / qa / ship) | sk-AI/claude-skills/ |
| Code style / Clean Code 精神 | sk-AI `/build` skill + ETHOS |
| Embedded conventions (this project) | sections below |

## Autonomy

Claude 已授權執行此 repo 內所有讀寫 / build / sim 操作。**例外要先確認**:flash 實機(雖然 `echo 13 | uart_download.bat` 可 auto-run,改變硬體狀態)、push 共享 branch、發 PR、改 `external/` 任何檔。

- 改完跑對應 `### Smoke` 必綠
- 改 GBK / ISO-8859 檔案 → 用 `sed` 不用 Edit/Write(見 § Domain gotchas)
- Don't-touch 清單(`§ Don't-touch` 段)內檔案 — 動到要 `git log -- <file>` 確認沒踩 in-flight work

## Spec Update Protocol

Spec docs forthcoming. 在那之前,non-trivial decisions → ADR `docs/adr/NNNN-{slug}.md`。

跨設備協定(L2 key 加減)變更必須**同 commit** 改:
- `src/modules/communicate/communicate_parse_<group>.h` (enum, hex 值不要動)
- `src/modules/communicate/communicate_parse_<group>.c` (case)
- 若會送出 → 加 `commu_send_*` in `communicate_task.c`
- Phone dart spec at `C:\work\SkaiLink\lib\shared\watch\communicate\communicate_protocol.dart`

## Commands

> sk-AI skill `/build` `/qa` 讀本 section 取指令。**保留 subsection header 名**(`### Lint` 等)。
>
> 全部從 `project/hcpu/` 下執行 — wrappers 用 `%~dp0` 自動偵測 repo root。

### Lint

GCC compile warnings serve as lint surface. Production 是 Keil 所以用 watch build 當 ground truth:

```cmd
project\hcpu\_watch_build.cmd -j8
REM grep error 摘要:
REM grep -E " error:|undefined reference|cannot find|scons:.*\*\*\*" _watch_build.log
```

### Test

PC sim MSH-driven framework + real-hw integration probes:

```cmd
project\hcpu\_dev_test.cmd -script _smoke_test.txt           REM PC sim batch
python tools\dev_console\_probe.py                            REM real-hw F1-F8 cycle
python tools\dev_console\_test_tui.py                         REM TUI pilot, headless
```

### Build

```cmd
REM PC sim (MSVC, fastest iteration)
project\hcpu\_pc_build.cmd -j8           REM → build_pc_hcpu/main.exe (~9.6 MB)

REM Watch firmware (Keil = production)
project\hcpu\_watch_build.cmd -j8        REM → main.bin (~2.45 MB) + bootloader.bin (~32 KB)

REM Keil .uvprojx for MDK IDE debug
project\hcpu\_watch_mdk5.cmd -j8

REM Flash to watch via UART (non-interactive — pipe COM port number):
echo 13 | cmd /c project\hcpu\build_sf32lb56-watch_hcpu\uart_download.bat
REM 13 = COM13 = CH342 channel B = boot ROM download port
```

### Smoke

```cmd
project\hcpu\_pc_build.cmd -j8                                       REM ~30s incremental
project\hcpu\_dev_test.cmd -nobuild -cmd "list_apps"                 REM boot sim + 1 cmd
```

### Run

```cmd
REM PC sim (rebuild + launch + ready for MSH cmds via tshell)
project\hcpu\_dev_test.cmd

REM Real-hw log monitor (textual TUI, auto-reconnect across reboots)
tools\dev_console\_watch_console.cmd     REM defaults COM14 @ 1M baud
```

## Structure

```
example/get-started/dualcore/
├── project/{hcpu,lcpu}/<board>_<cpu>/    板型 build configs (proj.conf 共用 + 板專屬)
├── src/hcpu/                              GUI + BLE host
│   ├── main.c · main_pc.c · log_file_backend.c · pc_link_stubs.c
│   ├── gui_apps/                          LVGL apps (SConscript 自動掃)
│   └── resource/                          fonts / images / Lottie / .arb
├── src/lcpu/main.c                        sensor / motor / charger driver entry
└── src/modules/
    ├── algorithm/  AI + DSP (TFLite Micro, Mahony, sleep_fusion, WearDetect)
    ├── bloc/       Business Logic Component (provider struct pattern, § below)
    ├── client/     HCPU→LCPU RPC via data_service
    ├── communicate/ 私有 BLE 協定 (L1+L2)
    ├── model/      跨 module interface (ui_handler, ble_*, watch_global_data)
    ├── service/    HCPU services (alarm_manager, hr, watch_system)
    ├── system/     boot_rollback (OTA trial-mode, silent rollback infra)
    ├── tests/      MSH 測試命令 (#if BSP_USING_PC_SIMULATOR)
    └── util/       trig / list / math_fixed / platform / time
```

HCPU vs LCPU 職責表 + cross-CPU channel + BLOC pattern + L1/L2 protocol + code-level conventions(`kReleaseMode` / `#ifdef APP_ID_*` / `MSH_CMD_EXPORT` / `lvgl_send_msg` / sensor subscribe)→ [`docs/architecture.md`](docs/architecture.md).

## PC simulator test framework

Build gate `BSP_USING_PC_SIMULATOR=y`,production build 不會 link。整套在 `src/modules/tests/` + `middleware/lvgl/lv_win32/lv_touch_sim.{c,h}`.

**Full docs**: [`docs/pc-sim.md`](docs/pc-sim.md) — workflow / MSH cmd table / known limits / fake fixture pattern / pre-existing patches.

Quickref:
- `_pc_build.cmd -j8` → `build_pc_hcpu/main.exe` (~9.6 MB)
- `_dev_test.cmd [-nobuild] [-cmd "..."] [-script foo.txt] [-screenshot foo.png] [-snapshot all]`
- MSH cmd groups:touch / navigation / fake devices / fake notifs / fake sensors / rollback (B3 `boot_meta_*`)
- ⚠️ `list_apps`(plural,PC sim)≠ `list_app`(singular,real hw built-in,列 running app)
- ⚠️ HCPU 端讀電池走 `SkaiWatchSys.battery_level_value` / `charger_status`,不是 `battery_get_charge_state()`(bloc_battery 沒 link 進 HCPU)

## Real-hw dev iteration

`tools/dev_console/` — Textual TUI (Python 3.11),auto-reconnect 跨 reboot、F1-F8 hotkeys、pattern alerts(REVERTING / Hard Fault / WDT / mark_valid,OTA rollback 測試用)、自動存 `_logs/watch_*.log`.

**Full docs**: [`../../../tools/dev_console/README.md`](../../../tools/dev_console/README.md)

Quickref:
- `tools\dev_console\_watch_console.cmd` — COM14 @ 1M baud default
- Hotkeys F1-F8:`list_app / version / list_data_service / dserv_stat / cpu / help / free / ps`(都在 real hw 驗證過)
- **Port mapping**:**COM14** = CH342 ch A = uart1 = firmware log(dev_console 用)。**COM13** = CH342 ch B = boot ROM download(`uart_download.bat` 用,running firmware 不 log 在這)

## Platform

- Boards: `eh-lb52x` / `ec-lb56x` / `sf32lb56-watch`(current — 注意**無 w**;刷 `sf32lb56w-watch` 的 build 會全靜默無法正常啟動)/ `sf32lb56w-watch` / `eh-lb56xu`(BOARD_VER_28)/ `eh-lb58x`
- Bootloader: `example/boot_loader/project/sf32lb56x_v2/`(watch 用這個,不是 `sf32lb56x`)
- Production toolchain: **Keil + armclang + microlib**(GCC 只供驗證,LCPU 多 ~75 KB)

Toolchain split / env hardcode / ConEmu 設定 / wrapper path detection / `.cmd` ASCII rule → [`docs/build-system.md`](docs/build-system.md).

## Build gotchas + Don't-touch

GBK 編碼檔(用 sed)/ 多板 `BSP_USING_BOARD_*` 重複 link / Vendor armclang-only lib / GCC 14 嚴格化 / `external/FlashDB` 修法 / pre-existing PC sim patches / current in-flight files
→ [`docs/build-system.md`](docs/build-system.md).

**動任何 `external/` / `customer/peripherals/sensor/` / `tools/build/building.py` 之前必讀。** 詳細修復史見 [`CHANGELOG.md`](CHANGELOG.md)。

## Testing

Two tiers:
1. **PC sim** via `_dev_test.cmd` — MSH-driven,fixtures: `_smoke_test.txt` / `_persist_*.txt` / `_bug3_*.txt` / `_ls_*.txt`
2. **Real hw** via `tools/dev_console/` + flash via `echo 13 | uart_download.bat`

No CI yet。Hardware-in-loop 是 manual。

## Git

Branch: `main` recently,`ota` for in-flight silent OTA work。
- **NEVER push without explicit user permission**
- **NEVER `git rebase -i` / `git reset --hard` / `--no-verify`** unless user asks
- Conventional prefix: `feat: / fix: / chore: / refactor: / docs:`
- Include `Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>` trailer
