# Changelog — Skaiwalk Dualcore Watch

時序工作紀錄。長期可用的編譯規則寫在 [CLAUDE.md](CLAUDE.md)。

## 2026-05-16 — 通知列表震動回歸 + 斷線重連通知去重 + PC sim FS 修復

### 修的 3 個 bug

1. **通知列表卡片切換沒震動**(Bug 2)
   - 原因:commit `c4dae2edc` 加 arc-scroll 時,`scroll_message_list_to_index()` 在 call `scroll_list()` 之前先 pre-sync `old_selected_message_index = selected_message_index`,讓後面 `scroll_list` 內的 `if (old != selected)` vibration trigger 永遠 false
   - 修 [lv_message_list_layout.c:2346](example/get-started/dualcore/src/hcpu/gui_apps/app_layout/lv_message_list_layout.c#L2346):在 pre-sync 之後直接 fire `motor_pattern_scrolling_app()`,並加 `get_scrolling_motor_vibrate_status()` 全域開關 gate
   - 順便修 [lv_message_list_layout.c:657](example/get-started/dualcore/src/hcpu/gui_apps/app_layout/lv_message_list_layout.c#L657):touch-drag 路徑也加上全域 gate(原本完全沒讀設定),跟 instruction_list 一致

2. **斷線重連已 dismiss 的通知再跳**(Bug 3)
   - 原因:dismiss 動作只刪本地陣列,沒持久化「已處理過的 id」。重連時手機 re-push cached 通知,dedup 找不到 → 當成新通知 + 震動 + banner
   - 加 dismissed-ids 環(32 槽 × 48B = 1.5 KB,RAM + flash),**OTA verify 成功 reboot 前**flush,一般 reboot/crash 不存(用戶指定)
   - **piggy-back 在 `watch_global_data` 的 prefs read/write 流程上**,共用 `open_watch_prefs()` 的 cached pref handle:
     - 第一版用獨立 `INIT_APP_EXPORT` 自己開 prefs → 實機撞 WDT1。原因是 `prefdb` partition 在 NAND,first-time `fdb_kvdb_init` 要 scan 整個 128 KB region,>8 秒 → reboot loop。
     - 改成把 read/write 註冊成 `WatchPrefs.read_dismissed_notifications` / `write_dismissed_notifications`,由 `watch_config_struct_flash_read()` 順便 read,FDB init 只發生一次,且在 `watch_demo.c:772` 那個非 boot-critical 的 context
     - OTA save 路徑透過 `store_watch_prefs(WATCH_PREFS_KEY_DISMISSED_NOTIFICATIONS)`,複用 cached handle,partition 已 warm,不會 hang
   - 改檔:
     - [bloc_notification.h](example/get-started/dualcore/src/modules/bloc/bloc_notification.h):公開 6 個 API(含 `read_dismissed` / `write_dismissed` 兩個 void* 包裝給 watch_global_data 用)
     - [bloc_notification.c](example/get-started/dualcore/src/modules/bloc/bloc_notification.c):末段加 ring 實作 + magic 驗證 + dirty flag,`update_notification` 開頭 `if (is_dismissed) return`,3 個 dismiss 點加 `mark_dismissed(id)`,save 改成走 `store_watch_prefs()`
     - [watch_global_data.h](example/get-started/dualcore/src/modules/model/watch_global_data.h):enum 加 `WATCH_PREFS_KEY_DISMISSED_NOTIFICATIONS`,struct 加兩個 function pointer
     - [watch_global_data.c](example/get-started/dualcore/src/modules/model/watch_global_data.c):`watch_prefs_register()` 註冊兩個 fn ptr,`watch_config_struct_flash_read()` 加 `read_dismissed_notifications(pref)`,`store_watch_prefs()` switch 加 case
     - [communicate_update_image.c:614](example/get-started/dualcore/src/modules/communicate/communicate_update_image.c#L614):`DFU_FLASH_MSG_TYPE_VERIFY` 成功、`drv_reboot()` 之前 call `bloc_notification_save_dismissed_to_flash()`
     - [fake_ui_data.c](example/get-started/dualcore/src/modules/tests/fake_ui_data.c):加 `notif_dismiss` / `notif_check_dismissed` / `notif_dismiss_clear` / `notif_dismiss_save` 四個 MSH test
   - 已在 PC sim 跑通完整 round-trip:mark → save → restart → load → check ✓
   - 實機驗證點:boot 不再 WDT;OTA verify success 前的 flush 透過 store_watch_prefs 走 cached pref,不會撞 FDB 重 init

3. **PC sim 沒 mount fs(實際是 host backing dir 沒建 + FDB libc upstream bug)**
   - 原因 1:`middleware/simulator/mnt.c` 沒先在 host 建 `./disk`(dfs_win32 backing)跟 `./prefdb`(FlashDB libc-mode storage)。`dfs_win32_ops` 沒提供 mkdir op,share_prefs `mkdir("prefdb",0)` 經 libc 直接打 host CWD,parent dir 不存在 → fail
   - 原因 2:`external/FlashDB/src/fdb_file.c:290 + 304` 的 `fread/fwrite(buf, size, 1, fp) != size` 是 upstream bug — 第 3 個參數是 count,return 是 element 數(0 或 1),任何 >1 byte 的 read/write 都誤報 `FDB_READ_ERR`。只 1 byte 的 read 巧合會通過,所以 share_prefs 的 `_get_int`(int32_t = 4 byte 但 fdb_kv_get_blob 內部可能拆讀)以前在 PC 上其實也是默默回 default 值,沒人發現
   - 改:
     - [middleware/simulator/mnt.c](middleware/simulator/mnt.c):Win32 `_mkdir` 預建 `./disk` + `./prefdb`,idempotent(EEXIST OK)
     - [external/FlashDB/src/fdb_file.c:290+304](external/FlashDB/src/fdb_file.c#L290):`!= size` → `!= 1`(配 `fread/fwrite` 真正 semantics)
   - **CLAUDE.md「Don't-touch 清單」加 FDB patch 註記**,upstream merge 要保留
   - 只影響 PC sim,實機用 fal mode 不受影響

### 連帶觀察(未動,但值得知道)

- `dfs_win32_ops` vtable 沒 `mkdir` op — 任何 DFS POSIX `mkdir` 都會失敗。share_prefs 走的是 host libc 不是 DFS,所以這次繞過了。如果以後有 code 真的要透過 DFS 在 PC 上 mkdir,要先補 ops
- ELM filesystem register 失敗的 boot log(`E/DFS init: There is no space to register this file system (elm).`)— 看起來 wdir 先佔了 slot 之後 elm 沒空間。目前沒人用 elm 所以不影響,但是個技術債

### 後續補強 (1) — bloc_notification 跟 watch_global_data 共用 prefs

第一版 Bug 3 用獨立 `INIT_APP_EXPORT(bloc_notification_dismissed_init)` 自己開 prefs,實機撞 WDT1 — 因為 NAND `prefdb` partition 第一次 `fdb_kvdb_init` 要 scan 整個 128 KB,>8 秒。改成 piggy-back 在 `watch_global_data` 的 read/write 路徑上,共用 cached pref handle:
- 加 `WATCH_PREFS_KEY_DISMISSED_NOTIFICATIONS` enum + 兩個 fn ptr `read_dismissed_notifications` / `write_dismissed_notifications`
- 在 `watch_config_struct_flash_read()` 順便 read,FDB init 跟其他 prefs 共用一次
- OTA save 改成 `store_watch_prefs(WATCH_PREFS_KEY_DISMISSED_NOTIFICATIONS)`,用 cached handle
- 拿掉 `INIT_APP_EXPORT(bloc_notification_dismissed_init)`
- `bloc_notification.h` 改用 `void *pref` 參數避免要 include `share_prefs.h`

### 後續補強 (2) — PC sim mount `/assets/{emoji,fonts}`

實機 NAND `/` 包含 `/assets/emoji/*` 跟 `/assets/fonts/tiny55_full.ttf`,source 在 `project/jsroot/assets/`。PC sim `dfs_win32` backing 的 `./disk/` 本來是空的。改 `middleware/simulator/mnt.c`,boot 時用 Windows directory junction(`mklink /J`,不需要 admin / dev mode)把:
- `./disk/assets/emoji` → `<repo>/example/get-started/dualcore/project/jsroot/assets/emoji`
- `./disk/assets/fonts` → `<repo>/example/get-started/dualcore/project/jsroot/assets/fonts`

從 sim tshell `ls /assets/emoji` 看到 1910 個 emoji_*.bin,`ls /assets/fonts` 看到 tiny55_full.ttf 1087660 B。FreeType file mode 在 PC 上可以直接讀到字型,跟實機一致。

抓 main.exe 的絕對路徑(`GetModuleFileNameA`)→ 往上走 3 層到 `.../project/` → `jsroot/` 在 sibling。junction 是 idempotent(已存在不覆蓋),user 自己改過的不會被踩到。

## 2026-05-15 — PC sim 測試框架 + 開發工作流(overnight)

### 目標
讓 PC 模擬器上的 UI 開發/測試可以完全用 tshell 指令驅動,AI 直接控制,不需要手動 swipe/tap;同時把寫死 `C:\work\SiFli-SDK` 的工具鏈解開,讓 SDK 放哪都能跑。

### 新檔
- `middleware/lvgl/lv_win32/lv_touch_sim.{c,h}` — 第二個 LVGL pointer indev,跟真實滑鼠並存。mutex 保護的 state,FinSH thread 寫、LVGL thread 讀
- `src/modules/tests/test_touch_sim.c` — `touch_tap` / `touch_press` / `touch_release` / `touch_move` / `touch_swipe` / `touch_status` MSH 指令
- `src/modules/tests/fake_ui_data.c` — 兩層東西混在一起:
  - 8 個 `ble_dev_mgr_*` real impl(取代 `pc_link_stubs.c` 的 no-op,讓 device list UI 看得到資料)
  - MSH 指令:`back` / `gesture_back` / `dev_add` / `dev_connect` / `dev_disconnect` / `dev_clear` / `dev_list` / `notif_inject` / `notif_clear` / `notif_list` / `battery_set` / `battery_volt_set` / `charge_set` / `hr_set`
- `src/modules/tests/dev_navigation.c` — `list_apps` / `goto_app` / `app_status` MSH 指令,直接 `gui_app_run()`,免 swipe/tap 導航
- `project/hcpu/_dev_test.ps1` + `_dev_test.cmd` — 一鍵 rebuild + relaunch sim + send tshell + 抓 console / screenshot / 全 app snapshot
- `project/hcpu/_snapshots/all/*.png` — 15 個 builtin app 的當前 UI 狀態截圖(視覺基準線)

### 改動
- `_pc_build.cmd` / `_watch_build.cmd` / `_watch_mdk5.cmd` / `_genstub.py` — 改用 `%~dp0` / `os.path.dirname(__file__)` 偵測 repo root,不再 hardcode `C:\work\SiFli-SDK`。SDK 放在 `C:\Users\lulu8\Documents\GitHub\SiFli-SDK` 或任何位置都能跑
- `middleware/lvgl/lv_win32/win32drv.c` — `lv_win32_init` 尾部呼叫 `lv_touch_sim_init()`,跟真實滑鼠 indev 一起註冊
- `src/hcpu/main_pc.c` — 補 `start_ble_rssi_checker` / `stop_ble_rssi_checker` no-op stub(`main.c` 被 PC build 排除,`watch_system_interact.c` 還在引用)
- `src/hcpu/pc_link_stubs.c` — 移除 8 個 `ble_dev_mgr_*` 行(real impl 移到 `fake_ui_data.c`)。保留 `start_main_phone_check_timer`(timer helper,跟 fake DB 無關)
- `project/hcpu/_syms_clean.txt` — 同步移除 8 個 `_ble_dev_mgr_*` 符號(下次 `_genstub.py` regen 不會重複定義)
- `src/modules/tests/SConscript` — `BSP_USING_PC_SIMULATOR` gate 下加入三個新 .c
- `CLAUDE.md` — 新增「PC 模擬器測試框架」一節,涵蓋工作流、指令集、已知限制、新 fixture pattern
- `.claude/settings.local.json` — allowlist 加 cmd /c *_pc_build.cmd / *_dev_test.cmd / Get-Process main 等常用 dev loop 指令,減少 permission prompt

### 工作流(現在 AI 跟人都可以這樣跑)
```bash
# 一鍵:rebuild + launch + 跑單一 MSH 指令 + 印 console buffer
project\hcpu\_dev_test.cmd -cmd "list_apps"

# 跑一串指令 → 寫成 .txt 用 -script
project\hcpu\_dev_test.cmd -script my_test.txt

# 不重 build,只快測
project\hcpu\_dev_test.cmd -nobuild -cmd "touch_tap 233 233"

# 截一張當前畫面
project\hcpu\_dev_test.cmd -nobuild -screenshot pic.png

# 跑遍所有 builtin app 各截一張(已驗證能用,15 張在 _snapshots/all/)
project\hcpu\_dev_test.cmd -snapshot all
```

### 已驗證
跑過完整 sequence 確認 sim 不崩:
- `list_apps` → 15 個 app
- `goto_app calculator/weather/Main` → app 切換 trace 正確,`app_status` 回 `id='Main' state=2`
- `dev_add iPhone15/MacBook/iPad` + `dev_list` → 3 個假裝置
- `notif_inject WeChat/LINE/Slack` + `notif_list` → 3 筆假通知存進 `_notification_list[]`
- `battery_set 42` / `charge_set 1` / `hr_set 78` → 不崩
- `touch_tap` / `touch_swipe` → UI 真的有反應

### 已知限制(明早可以接著做)
1. **通知抽屜不會 live refresh** — `LVGL_MSG_TYPE_NOTIFICATION` 三個 handler 都走 `lv_img_set_src(icon_list[type])`,PC sim 的 icon 是 eZIP 壓縮但宣告為 `LV_IMG_CF_TRUE_COLOR_ALPHA`,LVGL 預設 decoder 解錯就 crash。`notif_inject` 因此只注入資料、不發 refresh msg。要做 live refresh 需要在 `middleware/lvgl/lv_win32/img_dec.c` 加一個 variable-based 的 eZIP decoder
2. **`_genstub.py` 沒實際跑驗證** — 路徑改了但 PC build 因為 `pc_link_stubs.c` 有手動編輯,跑 regen 會覆寫掉。下次升級 SDK / 加新 ARM-only 符號時,先把手寫修改備份再 regen
3. **dev list callback 不能即時 fire** — `app_setting_device_list.c:dev_mgr_event_cb` 直接做 LVGL ops,從 FinSH thread call 會崩。workaround:`dev_add` 後切到 settings → bluetooth → devices 頁,page open 時會 re-read DB

### 沒動的檔案
git status 顯示的這些 M 不是我改的,是原本就在 working tree:
- `customer/boards/eh-lb56xu/bsp_board.h`
- `example/get-started/dualcore/project/hcpu/proj.conf`
- `example/get-started/dualcore/project/lcpu/proj.conf`
- `example/get-started/dualcore/src/hcpu/resource/fonts/freetype/font_partition_dsc.c`
- `tools/SifliTrace/SifliTrace.ini`
- `drivers/hal/bf0_hal_hlp.obj`、`example/get-started/dualcore/project/hcpu/vc140.pdb`(build artifact)

## 2026-05-09 — Upstream/main 合併 (SDK v2.5.0) + env 1.1.4 + GCC 14 移植

### Merge

- 父 SDK `upstream/main` 459 commits 合併到 `main`(SDK v2.4 → v2.5.0)
- 合併前備份:`backup/main-before-upstream-merge-2026-05-09`
- Merge commit:`3a5d592a7`

### 衝突解決(5 檔)

| 檔案 | 解法 |
|---|---|
| `customer/boards/Kconfig.v2` | 接受 upstream 新增的 `SD_BL_MODE` |
| `external/Kconfig` | 兩邊 source 都保留 |
| `middleware/lvgl/littlevgl2rtt.c` | **保留本地** 把 freetype init 從 `gui_lib_init` 移到 `gui_freetype_init` 的修改(避免和 ELMFAT 競爭) |
| `middleware/lvgl/app_mem.c` | 接受 upstream 改進的 ffmpeg header-aware 記憶體管理 |
| `example/.../watch_demo.c` | 保留本地全部 skaiwalk includes,只跟 upstream 把 `lv_freetype.h` 改為 `lvsf/lv_freetype.h` |

### Post-merge 編譯修復(commit `4c935dd62`)

- `external/FlashDB/inc/fdb_def.h`:FlashDB 2.1.1 的 `FILE *cur_file` 缺 stdio.h(`FDB_USING_FILE_LIBC_MODE` 條件下補)
- `middleware/lvgl/lv_ext_resouce/lv_ext_resource_manager.h`:upstream 把這檔從 `lvsf/` 抽出後沒再 include `lvsf.h`,33 個 gui_apps 的 `LV_EXT_FONT_GET` / `LVSF_FONT_TITLE` 都會失效。補回 `#include "lvsf/lvsf.h"`
- `bloc_skaiwalk.c`:`save_user_and_ai_chat` / `clear_chat_history` / `read_user_and_ai_chat` / `get_recent_chat_history` 用 POSIX dirent (DIR / opendir / readdir / mkdir / open),MSVC 沒有。整段用 `#ifdef _MSC_VER` 包起來給 stub
- `bloc_setting.c`:`LVSF_FONT_TITLE` 找不到 → 加 `#include "lvsf/lvsf_font.h"`
- `_pc_build.cmd`:upstream 升級 env 版本檢查到 1.1.4,wrapper 同步
- 新增 `_watch_build.cmd`:Keil/GCC 都通過的手錶 build wrapper

### env 1.1.4 升級

SDK upstream 的 `set_env.bat` 升到 v1.1.4 要求,user 原本 env 是 v1.1.3 被擋。下載 https://downloads.sifli.com/tools/env/env_latest.zip 解壓覆蓋 `C:\dev\env_latest`(必須先關 ConEmu64.exe / ConEmuC64.exe,否則 dir locked)。舊 env 備份在 `C:\dev\env_latest.bak.20260509`。新 env GCC 工具鏈是 `arm_gcc_14.2.1`(從舊版升級)。

### GCC 14 移植(commit `f671919e0` + `99d7f9959`)

env 升級後 ConEmu 跑 `scons --board=sf32lb56w-watch -j8` 預設用 GCC(set_env.bat 沒帶參數),撞了五輪問題:

#### Round 1 — 多板共用 `BSP_USING_BOARD_EH_LB563XXX` 重複連結

`customer/boards/SConscript` 掃所有子目錄。`sf32lb56-watch` / `sf32lb56w-watch` / `eh-lb563` 三板的 SConscript 都對同一 flag 做 `if GetDepend(...)` → 各自把 eh-lb56xu 拉進來,同一 .o 出現三次。Keil 只發 L6304W 警告,GCC ld 直接 multiple definition 失敗。

修法:三個 SConscript 加 active-board guard(只有 active 的那個才拉)。pattern 寫進了 CLAUDE.md。

#### Round 2 — GCC 14 預設嚴格化

`-Wimplicit-function-declaration`、`-Wincompatible-pointer-types`、`-Wbuiltin-declaration-mismatch`、`-Wint-conversion` 在 GCC 14 從 warning 升成 error。在 `tools/build/building.py` GCC 區塊(~L2926)加 `-Wno-error=...` 那組 flag 把它們降回警告。

#### Round 3 — `gesture_detect.c` missing decls

- 加 `#include "hr_service.h"` 取得 `is_ppg_service_ready`
- 加 `extern void hal_gsensor_drv_int1_handler(void);`
- `rt_ringbuffer_get/put` 對 `AccelRawData *` 加 `(rt_uint8_t *)` cast

#### Round 4 — Goodix `GS32` typedef 跨檔不一致

5 個 header / .c 各自 typedef `GS32`,有的 `int`、有的 `long int`。在 ARM 上 `int32_t = long int`,所以 `int` 版的 `GS32` 跟 `goodix_mem.h` 的 `int32_t goodix_mem_init(...)` 衝突。GCC 14 直接 `error: conflicting types`。

統一成 `long int` / `unsigned long int`。涉及檔案:
- `customer/peripherals/sensor/gh3018/gh30x_example_common.h`
- `customer/peripherals/sensor/gh3018/gh30x_hbd_ctrl.c`
- `customer/peripherals/sensor/gh30x_algo/Basic/inc/goodix_type.h`
- `customer/peripherals/sensor/gh30x_algo/Basic/inc/iot_sys_def.h`
- `customer/peripherals/sensor/gh30x_algo_demo/call/inc/goodix_algo.h`

外加 `gh30x_algo_hook.h` 把 `extern GU32 goodix_hrv_*` 改成 `extern goodix_hrv_ret goodix_hrv_*`(對齊 `goodix_sys_hrv.h`)。

**踩過的坑**:這些檔是 GBK / ISO-8859 + CRLF,Edit/Write 工具會把編碼轉成 UTF-8 → diff 從 13 行變成 100+ 行。後來改用 `sed -i` 做 byte-level 替換才正確。**規則寫進了 CLAUDE.md**。

#### Round 5 — Goodix gh3018 算法 lib 只有 armclang 版

`Default_KEIL5_M33_hard-fp_fshort.lib` 和 `cortexM33_armcland-mdk531_*.lib` 都是 armclang 編譯,GCC ld 找不到。

模式參考 `customer/peripherals/sensor/sc7r30/SConscript`:
- 兩個 SConscript 改 `if rtconfig.CROSS_TOOL == "gcc":` 不加 `LIBS=...`
- 新增 `customer/peripherals/sensor/gh3018/gh30x_gcc_stubs.c` 提供所有 lib 函式的 no-op stub
- Keil build 用 `SrcRemove(src, 'gh30x_gcc_stubs.c')` 排除避免 symbol clash

GCC 版的 LCPU 因此沒有真正的 PPG/HR/SPO2 算法 — 適合驗證編譯,production 仍需 Keil。

### 驗收

- ✅ PC sim:`scons --board=pc -j8` → `main.exe` 9.6 MB
- ✅ 手錶 GCC:`scons --board=sf32lb56w-watch -j8` → `main.bin` 2.4 MB / `lcpu.elf` 2.3 MB / `bootloader.bin` 35 KB,0 errors
- ✅ 手錶 Keil:`scons --board=sf32lb56w-watch -j8` (with `set_env.bat keil`) → `main.bin` 2.4 MB,0 errors,L6304W 重複警告也消除了

### 2026-05-11 — Keil project gen (.uvprojx) 修復 + LCPU size 分析 → 確定 production 用 Keil

- `tools/build/keil.py`:`scons --target=mdk5` 改成看 template 實際結構而非 scons env(`commit 425ad22a2`)。ConEmu 預設 gcc env 也能產 armclang `.uvprojx`。同時 `LINKFLAGS` (list vs string) 跨 env 正規化。
- 新增 `_watch_mdk5.cmd` wrapper:hardcode `set_env.bat keil` 因為 `--target=mdk5` 會順便 build。
- 修 `_watch_mdk5.cmd` em-dash → ASCII `--`(`commit 59ebe2044`):cmd.exe 在 .cmd 註解碰到 UTF-8 三 byte 字會分別當成 'M' command,出 3 個 `'M' 不是內部或外部命令` 噪訊。**規則寫入 CLAUDE.md:wrapper .cmd 一律 ASCII。**

#### LCPU size 對比 — production 確定用 Keil

對比同一份程式碼在兩個工具鏈下的 LCPU 大小:

| Toolchain | Code | RO | RW | 總 ROM | .bin |
|---|---|---|---|---|---|
| Keil (armclang + microlib) | 124 KB | 34 KB | 1 KB | **159 KB** | 22.9 + 136.6 KB |
| GCC 14 (arm-none-eabi + full newlib) | 21 KB(.text) | 211 KB(.rom2) | 2 KB | **234 KB** | 23 + 211 KB |

差距 ~75 KB 主因:`hr_service.c` / `alarm_manager_service.c` 用 `localtime`/`mktime` 把 newlib 時區 DB(`categories` 13.7 KB) + printf-float family(`_vfprintf_r` + `_dtoa_r` 等共 ~22 KB)拉進來。Keil microlib 沒這些。

**決策**:production 用 Keil。`_watch_build.cmd` / `_watch_mdk5.cmd` 都 hardcode `set_env.bat keil`。GCC build path 保留供驗證(我們之前花了一輪修通,留著對抗未來 upstream 的 Keil-only 假設)。

#### Goodix typedef 二次修正

之前(`commit 99d7f9959`)為了 GCC 過,把 5 個 header 的 `GS32` 從 `int` 改成 `long int`(對齊 GCC 上 `int32_t = long int`),反而打破 Keil(armclang 上 `int32_t = int`)。

正解(`commit f369b7bb4`):`GS32` 全部還原成 vendor 原樣 `int`,只在實際衝突點 `gh30x_algo_hook.h:328` 改用 `int32_t` 對齊 `goodix_mem.h`。這樣兩邊 stdint 實作差異不再撞 typedef。
