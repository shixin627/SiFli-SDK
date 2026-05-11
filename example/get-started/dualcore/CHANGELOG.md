# Changelog — Skaiwalk Dualcore Watch

時序工作紀錄。長期可用的編譯規則寫在 [CLAUDE.md](CLAUDE.md)。

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
