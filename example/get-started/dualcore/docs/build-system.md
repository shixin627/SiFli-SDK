# Build System Gotchas

Build-side rules every agent should know before editing files. Most of
these were learned the hard way during GCC 14 / env 1.1.4 / `andrew_v28.46`
upstream merges — see `CHANGELOG.md` for the full history.

---

## Toolchain split

| Build | Toolchain | Where |
|---|---|---|
| PC sim | MSVC | `_pc_build.cmd` |
| Watch (production) | **Keil + armclang + microlib** | `_watch_build.cmd` |
| Watch (verify only) | GCC (arm_gcc_14.2.1) | `set_env.bat gcc` via ConEmu |

**Production must be Keil.** GCC version's LCPU is ~75 KB bigger
(newlib full vs microlib) — `hr_service.c` / `alarm_manager_service.c`'s
`localtime`/`mktime` pull in the timezone DB and printf-float family.

To go GCC-production would require:
- Enable `USE_MICROLIB=True` in `project/lcpu/rtconfig.py` (nano.specs, side effects unverified)
- OR rewrite LCPU to not use `localtime`

### env hardcoded

`C:\dev\env_latest`, `ENV_VER=1.1.4`. All three wrappers
(`_pc_build.cmd` / `_watch_build.cmd` / `_watch_mdk5.cmd`) bake the
version. Upgrading env = bump all three.

### Wrapper path detection

`_pc_build.cmd` / `_watch_build.cmd` / `_watch_mdk5.cmd` / `_genstub.py`
use `%~dp0` / `os.path.dirname(__file__)` — SDK can live anywhere, no
hardcoded `C:\work\SiFli-SDK`.

### `.cmd` file content rule

**ASCII only.** Em-dash (`—`) or 全形標點 in `.cmd` files → cmd.exe parses
each UTF-8 byte as separate token → `'M' 不是內部或外部命令`-style errors.
Use `--` / `:` in comments. Verify: `file <path>` should say
`DOS batch file, ASCII text`, not `Unicode text`.

### ConEmu default toolchain

`SifliUser.cmd` auto-calls `set_env.bat` with no args → **default is GCC**,
not Keil (`set_env.bat`: `if "%1"=="" goto :SET_GCC`).

To make ConEmu default to Keil: edit
`C:\dev\env_latest\tools\ConEmu\ConEmu\SifliUser.cmd`, both `@call`
sites add ` keil` arg. **env upgrade overwrites this — re-patch after upgrade.**

---

## Gotcha 1: GBK / ISO-8859 中文檔不能用 Edit/Write

Vendor / legacy 檔常見 GBK + CRLF。Edit/Write 強制 UTF-8 + LF →
中文 byte 序列整批變動 → diff 從 3 行變 100+ 行,而且檔案編碼壞掉(下次讀亂碼)。

**Rule**: 編輯前先檢查編碼:

```bash
file customer/peripherals/sensor/gh3018/gh30x_algo_hook.h
# ISO-8859 / GBK + 含中文 → 改用 sed
sed -i 's|old|new|' <file>
git diff --shortstat <file>     # 應該只動實際改的行數
```

---

## Gotcha 2: 多板共用 `BSP_USING_BOARD_*` flag

`customer/boards/SConscript` 掃所有子目錄。N 個板子的 SConscript 同時
`if GetDepend(BSP_USING_BOARD_XXX)` 拉同一個 vendor → vendor `.o` 出現
N 次。Keil 只發 L6304W warning,**GCC ld 直接 multiple definition fail**.

修法 pattern(已套在 `sf32lb56-watch` / `sf32lb56w-watch` / `eh-lb563` 三個共用 `BSP_USING_BOARD_EH_LB563XXX` 的板子):

```python
this_board = os.path.basename(cwd)
active_board = (GetBoardName() or '').rstrip()
for suffix in ('_hcpu', '_lcpu', '_acpu'):
    if active_board.endswith(suffix):
        active_board = active_board[:-len(suffix)]
        break
# Only active board pulls in vendor module
if GetDepend('BSP_USING_BOARD_XXX') and active_board == this_board:
    group = group + SConscript('../<vendor>/script/SConscript')
```

---

## Gotcha 3: Vendor lib 只有 armclang 版

如果 `customer/peripherals/sensor/<name>/*.lib` 字樣有 `KEIL5_M33` /
`armcland-mdk531` / `cortexM33_keil` → 是 armclang 編譯,GCC 鏈 link 不了。

修法(已套在 `gh3018`):

```python
if rtconfig.CROSS_TOOL == "gcc":
    SrcRemove(src, '<no-stub-needed-for-gcc>.c')
    group = DefineGroup(..., CPPPATH=path)        # 不加 LIBS
else:
    SrcRemove(src, '<vendor>_gcc_stubs.c')
    group = DefineGroup(..., LIBS=['<armclang_lib>'], CPPPATH=path, LIBPATH=path)
```

GCC stub 檔提供所有 lib 函式的 no-op 實作 — 編譯通過但功能不可用。Dev / CI
用 stub,production 必須 Keil。

---

## Gotcha 4: GCC 14 對 legacy code 太嚴

`arm_gcc_14.2.1` 把這些升 default error:
- `-Wimplicit-function-declaration`
- `-Wincompatible-pointer-types`
- `-Wbuiltin-declaration-mismatch`
- `-Wint-conversion`

`tools/build/building.py` GCC 區塊(~L2926)加 `-Wno-error=...` 降回 warning。
新代碼觸發新 GCC 14 error → 選擇 fix code 或擴 flag list。

---

## Don't-touch (upstream merge keep local)

以下是為了 GCC + 新 env / PC sim 編譯所做的本地修改。upstream 衝突時
通常**保留本地**(但若上游做了結構性等價修復可接受上游版):

| File | What | Why local |
|---|---|---|
| `tools/build/building.py` `-Wno-error=` 行 | GCC 14 warning downgrades | Upstream 沒處理 |
| `customer/boards/{sf32lb56-watch,sf32lb56w-watch,eh-lb563}/SConscript` | active-board guard | Multi-board flag 重複 link 問題 |
| `customer/peripherals/sensor/gh3018/SConscript` + `gh30x_gcc_stubs.c` + `gh30x_algo/Basic/lib/SConscript` | GCC 分支 | Vendor lib armclang-only |
| Goodix `GS32` typedef 跨檔統一(5 檔) | int vs long int 衝突 | GCC 14 嚴格化 |
| `external/FlashDB/src/fdb_file.c:290 + 304` | `!= size` → `!= 1` | Upstream bug: `fread` 第 3 參數是 count,return 是 element 數 |

`external/FlashDB` 修法只影響 PC sim(`PKG_FDB_USING_FILE_LIBC_MODE`),
實機用 fal mode 沒事。**upstream merge 後必須確認這 2 行還是 `!= 1`**。

---

## Pre-existing PC sim patches

| Patch | What | Why |
|---|---|---|
| `main_pc.c` 提供 `start/stop_ble_rssi_checker` no-op | main.c 整個被 PC build 排除,watch_system_interact.c 還 extern 引用 | Linker 需要這些符號 |
| `pc_link_stubs.c` 移除 8 個 `ble_dev_mgr_*` | Real fake impl 移到 `fake_ui_data.c` | UI device list 要看得到資料 |
| `_syms_clean.txt` 同步移除 `_ble_dev_mgr_*` 8 個符號 | 避免 `_genstub.py` regen 重複定義 | 跟上面成對 |
| `pc_link_stubs.c` 移除 `start/stop_ble_rssi_checker` | `andrew_v28.46` merge 帶來的 dupe → LNK2005 | main_pc.c 已提供,pc_link_stubs.c 不該再有 |

詳細修復歷史見 [`../CHANGELOG.md`](../CHANGELOG.md)。

---

## Current in-flight (NOT this work's scope)

Working tree 內但屬其他 in-flight work。動之前用 `git log -- <file>`
確認沒踩到別人:

- `customer/boards/eh-lb56xu/bsp_board.h`(board config 變動)
- `project/lcpu/proj.conf`
- `src/modules/bloc/bloc_system_perception.c`
- `src/modules/communicate/communicate_parse_notify.h`(and `task.c`/`task.h`)
- `src/hcpu/resource/fonts/freetype/font_partition_dsc.c`(auto-generated)
