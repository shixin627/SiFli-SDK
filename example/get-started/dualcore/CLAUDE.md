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

### 跑編譯

```bash
# PC sim (MSVC)
cmd.exe /c "C:\\work\\SiFli-SDK\\example\\get-started\\dualcore\\project\\hcpu\\_pc_build.cmd -j8"
# → build_pc_hcpu/main.exe ~9.6 MB

# 手錶 (Keil,production)
cmd.exe /c "C:\\work\\SiFli-SDK\\example\\get-started\\dualcore\\project\\hcpu\\_watch_build.cmd -j8"
# → build_sf32lb56w-watch_hcpu/main.bin ~2.4 MB,lcpu IROM1+IROM2 ~159 KB,bootloader.bin ~35 KB
# Keil 是 production 工具鏈:armclang + microlib,LCPU 比 GCC 小 ~75 KB

# 產 Keil .uvprojx (要 Keil env,wrapper 內已 hardcode)
cmd.exe /c "C:\\work\\SiFli-SDK\\example\\get-started\\dualcore\\project\\hcpu\\_watch_mdk5.cmd -j8"
# → project.uvprojx 可直接用 Keil MDK 5 開

# 失敗時抓 error 摘要(警告很多會洗版):
grep -E "( error:|undefined reference|cannot find|scons:.*\\*\\*\\*)" \
    example/get-started/dualcore/project/hcpu/_watch_build.log
```

成功訊號:`scons: done building targets.`

### Wrapper 設計

`_pc_build.cmd` / `_watch_build.cmd` / `_watch_mdk5.cmd` 是 ConEmu 之外的編譯入口,自帶 env 初始化(因為 ConEmu 的 `CmdInit.cmd` 在 cmd.exe `/c` 模式下不會跑)。三個 wrapper 都 `set ENV_VER=...` 自己 override env 版號檢查 -- SDK env 升級時要同步改 wrapper 裡的版號。

**Production = Keil**。`_watch_build.cmd` 和 `_watch_mdk5.cmd` 都 hardcode `set_env.bat keil`。GCC 通過編譯但只供驗證 -- newlib full vs microlib 差距讓 GCC 版本 LCPU 多 ~75 KB,主要是 `hr_service.c` / `alarm_manager_service.c` 的 `localtime`/`mktime` 拖進 newlib 時區 DB 和 printf-float 家族。要走 GCC production 需要先 enable `USE_MICROLIB=True` 在 lcpu/rtconfig.py 啟用 nano.specs(未驗證副作用)或改寫 LCPU 不要用 `localtime`。

### ConEmu 預設 toolchain = Keil(env 升級後要重做)

ConEmu 開啟時 `SifliUser.cmd` 會從當前目錄往上找 `set_env.bat` 自動呼叫,**預設沒帶 toolchain 參數 = GCC**。改 ConEmu 一開就是 Keil env(在 ConEmu 直接打 `scons --board=sf32lb56w-watch -j8` 就跑 Keil):

編輯 `C:\dev\env_latest\tools\ConEmu\ConEmu\SifliUser.cmd`,兩處 `@call %first_file%` / `@call %current_path%\%target_file%` 加上 ` keil` 參數(echo 行同步)。

> ⚠️ 這是 env 工具內的檔,**env 升級後會被覆蓋**,要重做。env 升級流程已在 [CHANGELOG.md](CHANGELOG.md) 2026-05-09 那段;升級後記得重 patch SifliUser.cmd。

> ⚠️ wrapper / `.cmd` 檔案內容**只用 ASCII**。Em-dash (`—`) 或全形標點會讓 cmd.exe 把它的 UTF-8 byte 各自當成 command 解析,出現 `'M' 不是內部或外部命令` 之類的錯誤。寫註解一律用 `--` / `:`、不用破折號。`file <path>` 結果應該是 `DOS batch file, ASCII text`,出現 `Unicode text` 就有問題。

切換工具鏈:改 wrapper 內 `call C:\work\SiFli-SDK\set_env.bat <toolchain>` 那一行(`gcc` / `keil`)。

> ⚠️ `set_env.bat` 不帶參數**預設是 GCC**(`if "%1"==""    goto :SET_GCC`),不是 Keil。在 ConEmu 直接打 `scons --board=...` 用的是 GCC。

#### 產 Keil `.uvprojx` 不限 env(從 2026-05 起)

`tools/build/keil.py` 已改成檢視 template 實際結構(`tree.find('TargetArm')` 是否存在),不再單看 `rtconfig.PLATFORM`。本專案 `project/hcpu/template.uvprojx` 是 armclang 結構(`<TargetArmAds>`),所以在 gcc 或 keil env 都能跑 `scons --target=mdk5`,產出的 .uvprojx 都是 armclang 結構給 Keil MDK 開。

`_watch_mdk5.cmd` wrapper 還是 hardcode `set_env.bat keil` 因為 scons 跑 `--target=mdk5` 時也會順便 build 一次,Keil env 跑 armclang build 較合適(不會撞 GCC 14 嚴格化 / Goodix lib 等 GCC 才有的問題)。

### 編譯規則

#### 多板共用 `BSP_USING_BOARD_*` flag → 加 active-board guard

`customer/boards/SConscript` 會掃所有子目錄並執行各板 SConscript。如果 N 個板子 SConscript 都對同一個 `BSP_USING_BOARD_*` flag 做 `if GetDepend(...)` 然後拉同一個 vendor 子模組,vendor 的 .o 會被加進 link N 次。Keil 只發 L6304W 警告,**GCC ld 會 multiple definition 失敗**。

修法 pattern(已套在 `sf32lb56-watch`、`sf32lb56w-watch`、`eh-lb563` 三個共用 `BSP_USING_BOARD_EH_LB563XXX` 的板子):

```python
this_board = os.path.basename(cwd)
active_board = (GetBoardName() or '').rstrip()
for suffix in ('_hcpu', '_lcpu', '_acpu'):
    if active_board.endswith(suffix):
        active_board = active_board[:-len(suffix)]
        break
# 只有當前 active 的板子才拉 vendor 模組
if GetDepend('BSP_USING_BOARD_XXX') and active_board == this_board:
    group = group + SConscript('../<vendor>/script/SConscript')
```

#### Vendor lib 只有 armclang 版 → 在 SConscript toolchain-gate + 補 GCC stub

如果 `customer/peripherals/sensor/<name>/*.lib` 是 armclang 編譯(`KEIL5_M33` / `armcland-mdk531` / `cortexM33_keil` 字樣),GCC 鏈無法 link。先例:`gh3018` 的 SConscript:

```python
if rtconfig.CROSS_TOOL == "gcc":
    SrcRemove(src, '<no-stub-needed-for-gcc>.c')  # 反過來:keil 才 SrcRemove gcc stub
    group = DefineGroup(..., CPPPATH = path)      # 不加 LIBS
else:
    SrcRemove(src, '<vendor>_gcc_stubs.c')
    lib = ['<armclang_lib_name>']
    group = DefineGroup(..., LIBS = lib, CPPPATH = path, LIBPATH = path)
```

GCC stub 檔內提供所有 lib 函式的 no-op 實作 — 編譯通過但功能不可用,適合 dev / CI,production 仍要 Keil。

#### ⚠️ GBK / ISO-8859 中文註解檔不能用 Edit/Write

vendor / legacy 檔案常見編碼:**GBK / ISO-8859 + CRLF**,不是 UTF-8。Edit/Write 工具會強制 UTF-8 + LF → 中文註解的 byte 序列整批變動 → git diff 從 3 行變成 100+ 行,而且檔案編碼壞掉(下次別人讀就是亂碼)。

**規則**:編輯前先 `file <path>` 看編碼。出現 `ISO-8859 text` 或 `Unicode text, UTF-8` **而且**裡面有中文,改用 `sed -i 's|old|new|' file` 做 byte-level 替換。

```bash
file customer/peripherals/sensor/gh3018/gh30x_algo_hook.h
# C source, ISO-8859 text, with CRLF line terminators  ← 別 Edit,用 sed

sed -i 's|^extern GU32 goodix_hrv_init|extern goodix_hrv_ret goodix_hrv_init|' \
    customer/peripherals/sensor/gh3018/gh30x_algo_hook.h
```

驗證:`git diff --shortstat <file>` 應該只有實際改動的行數,不會幾百行。

#### GCC 14 對 legacy code 太嚴

`arm_gcc_14.2.1` 把 `-Wimplicit-function-declaration`、`-Wincompatible-pointer-types`、`-Wbuiltin-declaration-mismatch`、`-Wint-conversion` 升成 default error。`tools/build/building.py` GCC 區塊(~L2926)已經加 `-Wno-error=...` 把它們降回警告,維持與舊 GCC / Keil 一致的容忍度。新加的 SDK / vendor 程式碼若觸發新類型的 GCC 14 error,看是要 fix code 還是擴充這串 flag。

### Don't-touch 清單(下次 upstream merge 要小心)

以下是為了讓 GCC + 新 env 編得過所做的本地修改。upstream merge 時若衝突,通常本地版本要**保留**,但若上游做了結構性等價修復(例如統一 GS32 typedef、提供 GCC lib),可以接受上游版本並刪本地補丁:

- `tools/build/building.py` 的 `-Wno-error=` 那行
- `customer/boards/{sf32lb56-watch,sf32lb56w-watch,eh-lb563}/SConscript` 的 active-board guard
- `customer/peripherals/sensor/gh3018/SConscript` + `gh30x_gcc_stubs.c` + `gh30x_algo/Basic/lib/SConscript` 的 GCC 分支
- Goodix `GS32` typedef 跨檔統一(5 檔)

詳細的修復歷史看 [CHANGELOG.md](CHANGELOG.md)。
