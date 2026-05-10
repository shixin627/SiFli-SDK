# Skaiwalk Dualcore Watch — Project Guide

SiFli SF32 系列雙核心智慧手錶韌體。HCPU 跑 LVGL GUI、BLE host、AI 推論;LCPU 常駐感測器、馬達、LED、充電。

## 目錄結構

```
example/get-started/dualcore/
├── project/                          # 編譯設定
│   ├── hcpu/proj.conf                # HCPU 開啟的 CONFIG_*
│   ├── hcpu/<board>_hcpu/            # 各板型 (eh-lb52x、ec-lb56x、eh-lb58x ...)
│   └── lcpu/proj.conf                # LCPU 開啟的 CONFIG_*
└── src/
    ├── hcpu/
    │   ├── main.c                    # HCPU entry
    │   ├── log_file_backend.c        # ulog → /file
    │   ├── app_bt/                   # Classic BT/HFP/ANCS app layer
    │   ├── resource/                 # 字型、圖片、Lottie、JSON 翻譯
    │   └── gui_apps/                 # 所有 LVGL apps (見下方)
    ├── lcpu/main.c                   # LCPU entry
    └── modules/
        ├── algorithm/                # AI/訊號處理 (TFLite Micro、Mahony、Kraepelin、WebRTC VAD)
        ├── bloc/                     # Business Logic Component — provider struct (see pattern below)
        ├── client/                   # HCPU→LCPU RPC clients (sensor、heart rate、alarm、watch_sys)
        ├── communicate/              # 與手機 App 的私有 BLE 協定
        ├── model/                    # 跨 GUI 與週邊的對外介面 (watch_system_interact、ui_handler、ble_hid、ble_device_manager)
        ├── service/                  # alarm_manager、telephone、weather (HCPU 端)
        ├── tests/                    # MSH 測試命令
        └── util/                     # list/hash/crc32/math 共用工具
```

## HCPU vs LCPU 職責

| 面向 | HCPU | LCPU |
|---|---|---|
| GUI | LVGL + 所有 gui_apps | 無 |
| BLE | Host + ANCS + 私有協定 | Controller |
| AI | TFLite Micro 手勢/活動分類 | 抬腕、配戴偵測、AHRS |
| 感測器 | 透過 `client/*` 訂閱 | 直接驅動 BMI270、GH3018 |
| 馬達/LED | 呼叫 `bloc_peripheral` provider | 真正驅動 PWM4 / SK6812 |
| 檔案系統 | DFS + ELMFAT | 無 |

**核心通道**:HCPU `client/*` ↔ LCPU `service/*`,透過 RT-Thread `data_service` (`CONFIG_BSP_USING_DATA_SVC`) + `RT_USING_HWMAILBOX`。

## BLOC Provider 模式 (重要慣例)

每個 `bloc_*.c` 暴露一個全域 struct of function pointers,在 `INIT_APP_EXPORT` 時註冊實作:

```c
// bloc_control.h
typedef struct {
    void (*ble_hid_mouse_move)(int8_t dx, int8_t dy);
    bool (*bt_speaker_get_status)(void);
    // ...
} ControlProvider;
extern ControlProvider control_provider;

// bloc_control.c
static int bloc_control_provider_register(void) {
    control_provider.ble_hid_mouse_move = ble_hid_mouse_move_impl;
    // ...
    return 0;
}
INIT_APP_EXPORT(bloc_control_provider_register);
```

主要 provider:`peripheral_provider`、`control_provider`、`setting_provider`、`notify_provider`、`voice_provider`、`skaiwalk_provider`、`bloc_file_system`、`watch_sys_sync`。

## 通訊協定 (`modules/communicate/`)

- **L1 frame**: 8-byte header (magic 0xAB、ver、len、CRC16、seq) + payload
- **L2 frame**: command_id(1) + ver(1) + key(1) + len(2) + value
- 與手機端 dart spec 嚴格對齊 (`C:\work\SkaiLink\lib\shared\watch\communicate\communicate_protocol.dart`)

**Top-level command IDs** (`communicate_parse.h`):
`BOND_COMMAND_ID(0x03)` / `SET_CONFIG_COMMAND_ID(0x02)` / `NOTIFY_COMMAND_ID(0x04)` / `HEALTH_DATA_COMMAND_ID(0x05)` / `CONTROL_COMMAND_ID(0x07)` / `BLUETOOTH_LOG_COMMAND_ID(0x0a)` / `WEATHER_INFORMATION_ID(0x0b)` / `SKAI_LINK_COMMAND_ID(0x20)` / `FIRMWARE_UPDATE_CMD_ID(0x01)` / `FACTORY_TEST_COMMAND_ID(0x06)` / `TEST_FLASH_READ_WRITE(0xFE)` / `TEST_COMMAND_ID(0xFF)`

**Dispatch**:`L2_frame_resolve` → `resolve_<bond/setting/health/notify/control/SkaiLink>_command`(各 parse_*.c switch on key)。

**送資料**:統一走 `commu_send_*` 函式 (`communicate_task.c`),宣告在 `communicate_task.h`。所有送資料前會被 `commu_can_send()` 守門 (連線中且 BLE DFU 未跑)。

**新增/刪除 key 的流程**:
1. 改 `communicate_parse_<group>.h` 的 enum (**hex 值不要動**)
2. 對應 `parse_*.c` 加/移 case
3. 若會送出,加/移 `commu_send_*` 函式 + 對應 caller
4. 確認手機端 dart 也有對應 key

## watch_system_interact dispatcher

`modules/model/watch_system_interact.c` 是中央 dispatcher,以 `INTERACT_Type` enum 路由到馬達/LED/設定/電源/感測器:

```c
watch_system_interact(INTERACT_RGB_LED_OPEN_WRITE, &brightness);
watch_system_interact(WATCH_SLEEP, NULL);
```

入口會做 `is_ble_dfu_thread_running()` 守門。對於同模組內的單純呼叫,可直接用對外公開函式(例如 motor_pattern_alarm())跳過 dispatcher。

## GUI Apps (`hcpu/gui_apps/`)

每個 app 一個目錄,含 `app_<name>.c`、`SConscript`,透過 `BUILTIN_APP_EXPORT(LV_EXT_STR_ID(name), IMG, APP_ID_NAME, app_main)` 註冊。`gui_apps/SConscript` 自動掃描所有有 SConscript 的子目錄。

**目前 apps**:
`main` (錶盤+launcher) / `app_layout` (instruction list、app grid) / `clock` / `setting` / `interact` (QR/找錶/低電量/配對) / `incoming_call` / `message` / `speech` / `skai` (AI 對話) / `weather` / `calendar` / `alarm` / `timer` / `media` / `recorder` / `camera` / `flashlight` / `qrcode` / `calculator` / `barometer` / `battery` / `heartrate` / `activity` / `exercise` / `gesture` / `hid_mouse` / `hid_touchscreen` / `photo` / `file_browser` / `mem` / `loader` / `widgets` / `common` / `utils` / `test` / `game`

**APP_ID** 字串定義散落在各 board 的 `board.h` / `Kconfig` (透過 `#ifdef APP_ID_*` 條件編譯)。`app_id_*` enum 在 `modules/model/ui_handler.h`。

## 重要慣例

- **`kReleaseMode`**:dev-only feature 包在 `#if !kReleaseMode`(MSH debug 命令、PPG raw collection、gesture threshold tuning 等)。Release build 自動排除
- **`#ifdef APP_ID_*`**:某 app 沒在當前 board 開啟,相關引用就要用 `#ifdef` 包起來避免編譯錯誤
- **MSH 命令**:`MSH_CMD_EXPORT(name, "help")` 註冊 shell 命令,大多在 `#if !kReleaseMode` 下
- **新增送資料函式**:加在 `communicate_task.c`,**檢查 dart 端有對應 key**,否則資料會被手機忽略

## 常見任務速查

- **加新的手機指令**:dart 加 key → watch parse_*.h 加 enum (照 hex 順序) → parse_*.c 加 case → 視需要加 commu_send_* 回應
- **加新的 GUI app**:在 `hcpu/gui_apps/` 開目錄,參考 `flashlight/` 結構,在 board.h 定義 `APP_ID_*`,加進 `lv_app_list_layout.c` / `lv_instruction_list_layout.c` 的 enum
- **加新的馬達**:在 `watch_system_interact.c` 加 `motor_pattern_*`,header 對外 extern,直接呼叫即可
- **加感測器訂閱**:走 `peripheral_provider.subscribe_*` 或 `watch_system_interact(WATCH_SENSOR_SUBSCRIBE, ...)`(後者執行緒安全)
- **跨 thread 操作 UI**:`lvgl_send_msg({.type = LVGL_MSG_TYPE_*, .data = ...})`,handler 在 `ui_handler.c`

## 構建

依板型分目錄:
```
project/hcpu/<board>_hcpu/  # 例如 eh-lb52x_hcpu
project/lcpu/<board>_lcpu/
```

板型決定 `board.h` 中的 `APP_ID_*` 啟用與否、感測器型號、PWM/I2C 接腳。改板型不要動 `proj.conf` 共用設定,改 `<board>_hcpu/` 內專屬設定。

### 工具鏈與 wrapper

`set_env.bat <toolchain>` 設置編譯環境。**沒帶參數預設是 GCC**(set_env.bat 內的 `if "%1"==""    goto :SET_GCC` ),不是 Keil。

ConEmu 之外要編譯,用 `project/hcpu/` 下的 wrapper:

| Wrapper | 板子 | 工具鏈 | 用途 |
|---|---|---|---|
| `_pc_build.cmd` | `pc` | MSVC | PC simulator(LVGL Win32) |
| `_watch_build.cmd` | `sf32lb56w-watch` | GCC | 手錶 ARM build(已驗證 GCC 通過) |

兩個 wrapper 都 `set ENV_VER=1.1.4` 自己 override(免依賴 ConEmu 的 CmdInit.cmd)。SDK env 升級時要同步更新。

**驗收輸出**:
- PC sim:`build_pc_hcpu/main.exe` ~9.6 MB
- 手錶 GCC:`build_sf32lb56w-watch_hcpu/main.bin` ~2.4 MB,`bootloader.bin` ~35 KB,`lcpu.elf` ~2.3 MB
- 手錶 Keil:同上但 `.axf` / `.bin`,可能有 L6304W 警告(已修復為 0)

### 編譯踩雷紀錄(2026-05 upstream/main 合併 + env 1.1.4 升級時整理)

#### 1. SDK env 版本檢查(`set_env.bat` v1.1.4 起)
SDK 跟 env 版號綁定。env 太舊會被 set_env.bat 擋。升級:下載 https://downloads.sifli.com/tools/env/env_latest.zip 解壓覆蓋 `C:\dev\env_latest`(先關 ConEmu)。詳細記錄在 backup 分支 `backup/main-before-upstream-merge-2026-05-09`。

#### 2. 多板共用 `BSP_USING_BOARD_EH_LB563XXX` → eh-lb56xu 重複連結
`customer/boards/SConscript` 會掃所有子目錄並執行各 SConscript。`sf32lb56-watch` / `sf32lb56w-watch` / `eh-lb563` 三個板子的 SConscript 都對 `BSP_USING_BOARD_EH_LB563XXX` 做 `if GetDepend(...)` 然後 `SConscript('../eh-lb56xu/script/SConscript')`,結果 eh-lb56xu 的 .o 被加進 link 三次。

- **Keil**:L6304W 重複輸入警告,但仍能完成 link
- **GCC**:`multiple definition of 'BSP_GetFlash2DIV'` 等硬錯誤,build 失敗

**修復**:三個 SConscript 加 active-board guard:
```python
this_board = os.path.basename(cwd)
active_board = (GetBoardName() or '').rstrip()
for suffix in ('_hcpu', '_lcpu', '_acpu'):
    if active_board.endswith(suffix):
        active_board = active_board[:-len(suffix)]
        break
if GetDepend('BSP_USING_BOARD_EH_LB563XXX') and active_board == this_board:
    group = group + SConscript('../eh-lb56xu/script/SConscript')
```
這個 pattern 對任何「多板共用 flag」的 vendor SConscript 都適用。

#### 3. GCC 14 預設嚴格化
`arm_gcc_14.2.1` 把幾類預設從 warning 升成 error。在 `tools/build/building.py` GCC 區塊 (~L2926) 加上:
```python
rtconfig.CFLAGS += ' -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=builtin-declaration-mismatch -Wno-error=int-conversion'
```
讓這幾類回到警告(維持與舊 GCC / Keil 一致的容忍度),否則大量 legacy code 過不了。

#### 4. Goodix gh3018 sensor lib 只有 armclang 版
`customer/peripherals/sensor/gh3018/Default_KEIL5_M33_hard-fp_fshort.lib` 和
`customer/peripherals/sensor/gh30x_algo/Basic/lib/cortexM33_armcland-mdk531_*.lib` 都是 armclang 編譯,GCC 無法 link。

**修復**(模式參考 `customer/peripherals/sensor/sc7r30/SConscript`):
- 兩個 SConscript 改 `if rtconfig.CROSS_TOOL == "gcc":` 不加 `LIBS=...`
- 新增 `customer/peripherals/sensor/gh3018/gh30x_gcc_stubs.c` 提供所有 lib 函式的 no-op stub(goodix_mem_init / goodix_hba_* / goodix_spo2_* / goodix_hrv_* / NADT_* / Gh30xHBDVersionGet / Gh30xRawdata24BitTo32Bit)
- Keil build 用 SrcRemove 排除 stub 檔避免 symbol clash

**副作用**:GCC 版本的 LCPU 沒有真正的 PPG/HR/SPO2 算法。要 production 用 Keil,GCC 只適合驗證編譯不破。

#### 5. Goodix `GS32` typedef 不一致
五個 header / .c 各自 typedef `GS32`,有的是 `int`、有的是 `long int`。在 ARM 上 `int32_t = long int`,所以 `int` 版的 `GS32` 跟 `goodix_mem.h` 的 `int32_t goodix_mem_init(...)` 衝突。GCC 14 直接 `error: conflicting types`。

**修復**:全部統一成 `long int` / `unsigned long int`。涉及檔案:
- `customer/peripherals/sensor/gh3018/gh30x_example_common.h`
- `customer/peripherals/sensor/gh3018/gh30x_hbd_ctrl.c`
- `customer/peripherals/sensor/gh30x_algo/Basic/inc/goodix_type.h`
- `customer/peripherals/sensor/gh30x_algo/Basic/inc/iot_sys_def.h`
- `customer/peripherals/sensor/gh30x_algo_demo/call/inc/goodix_algo.h`

外加 `customer/peripherals/sensor/gh3018/gh30x_algo_hook.h` 把 `extern GU32 goodix_hrv_*` 改成 `extern goodix_hrv_ret goodix_hrv_*`(對齊 `goodix_sys_hrv.h`)。

#### 6. **重要:Goodix / 其他中文註解檔的編碼問題**
`customer/peripherals/sensor/gh3018/gh30x_algo_hook.h`、`iot_sys_def.h` 等檔是 **GBK/ISO-8859 + CRLF** 不是 UTF-8。**用 Edit/Write 工具寫會把編碼轉成 UTF-8** → 中文註解的 byte 序列改變 → git diff 變成 100+ 行(每行只是 byte 不同),encoding 也壞掉。

**正解**:這類檔用 `sed -i 's|old|new|' file` 做 byte-level 替換,保留原編碼。例:
```bash
sed -i 's|^typedef int GS32;|typedef long int GS32;|' \
    customer/peripherals/sensor/gh3018/gh30x_example_common.h
```
編輯前用 `file <path>` 確認編碼,看到 `ISO-8859 text` 或 `Unicode text, UTF-8` 含中文就走 sed。

#### 7. PC simulator 後續修
- `external/FlashDB/inc/fdb_def.h`:FlashDB 2.1.1 的 `FILE *cur_file` 缺 stdio.h(`FDB_USING_FILE_LIBC_MODE` 條件下補)
- `middleware/lvgl/lv_ext_resouce/lv_ext_resource_manager.h`:upstream 把這檔從 `lvsf/` 抽出後沒再 include `lvsf.h`,33 個 gui_apps 的 `LV_EXT_FONT_GET` / `LVSF_FONT_TITLE` 都會失效。我補回 `#include "lvsf/lvsf.h"`
- `bloc_skaiwalk.c`:`save_user_and_ai_chat` / `clear_chat_history` / `read_user_and_ai_chat` / `get_recent_chat_history` 用 POSIX dirent (DIR / opendir / readdir / mkdir / open),MSVC 沒有。整段用 `#ifdef _MSC_VER` 包起來給 stub
- `bloc_setting.c`:`LVSF_FONT_TITLE` 找不到 → 加 `#include "lvsf/lvsf_font.h"`

#### 8. `gesture_detect.c` 缺宣告 + pointer cast(GCC 14 升級才浮現)
- 加 `#include "hr_service.h"` 取得 `is_ppg_service_ready`
- 加 `extern void hal_gsensor_drv_int1_handler(void);`
- `rt_ringbuffer_get/put` 對 `AccelRawData *` 加 `(rt_uint8_t *)` cast

### 跑編譯的標準流程(下次給我用)

```bash
# 1. PC sim
cd /c/work/SiFli-SDK
cmd.exe /c "C:\\work\\SiFli-SDK\\example\\get-started\\dualcore\\project\\hcpu\\_pc_build.cmd -j8" 2>&1 | tail -30

# 2. 手錶 GCC
cmd.exe /c "C:\\work\\SiFli-SDK\\example\\get-started\\dualcore\\project\\hcpu\\_watch_build.cmd -j8" 2>&1 | tail -30
# 看 "scons: done building targets." 即成功

# 失敗時抓 error 摘要(避免被 warning 洗版):
grep -E "( error:|undefined reference|cannot find|scons:.*\\*\\*\\*)" \
    example/get-started/dualcore/project/hcpu/_watch_build.log
```

`_watch_build.cmd` 預設用 GCC。要切 Keil 改 wrapper 裡 `set_env.bat keil`(行 16)。Keil 結果在 `.axf` / `.map` 檔,GCC 結果在 `.elf` / `.bin` / `.map` 檔。

### Don't-touch 清單(這些是 SDK / vendor 介面,改了會破壞下次 upstream merge)

- `tools/build/building.py` 的 `-Wno-error=` 那行 — 上游若補了相同 flag 要保留並避免重複
- `customer/boards/{sf32lb56-watch,sf32lb56w-watch,eh-lb563}/SConscript` 的 active-board guard — 上游若做了類似結構性修復可以接受 upstream 版本
- `customer/peripherals/sensor/gh3018/SConscript` + `gh30x_gcc_stubs.c` — 上游若提供 GCC 版 .a 就刪 stub
- Goodix `GS32` typedef 統一 — 上游若統一就 follow 上游
