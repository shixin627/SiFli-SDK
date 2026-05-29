# 發布版本編譯手冊

> 給團隊成員：如何把手錶韌體編成「正式發布版」並打包，不必再手動記一堆開關。
>
> 所有指令都在 **`example/get-started/dualcore/project/hcpu/`** 這個資料夾下執行。

---

## 1. 這份工具解決什麼問題

以前每次發版，要手動去四個檔案改 7~8 個散落各處的值（版號、板號、release 旗標、FINSH、虛擬串口、心率/IMU 電源腳位…）。漏改一個就會編出「半開發半發布」的韌體 —— 實際上已經發生過好幾次（lcpu 的電源腳位常被忘記切回來）。

現在用兩支腳本把它自動化：

| 腳本 | 用途 |
|---|---|
| `make_release.bat` | **一鍵發布**：切發布參數 → 編譯雙核 → 打包 → 更新 info.json |
| `set_build_mode.py` | 只切「開發 ↔ 發布」參數（被上面那支呼叫，也可單獨用） |

---

## 2. 前置需求

- 已能正常編譯手錶韌體（即 `_watch_build.cmd` 平常跑得起來，Keil 環境 OK）。
- Python 3（SiFli env 內建那套就可以）。
- 在 `project/hcpu/` 目錄下執行。

---

## 3. 最快路線：一鍵發布

在 `project/hcpu/` 下，**直接雙擊或執行**：

```cmd
make_release.bat
```

它會依序做 5 件事，過程中問你 3 個問題：

| 步驟 | 動作 | 會問你 |
|---|---|---|
| 1 | 切換成 **發布(release) 參數** + 版號 +1 | — |
| 2 | 編譯 hcpu + lcpu（Keil 正式版） | — |
| 3 | （可選）開 notepad 編 `info.json` 的發布介紹 | `Edit release notes? (y/N)` |
| 4 | 打包 bin 到 `watchOS\sys\` | `Include watchface? (y/N)` |
| 5 | 把版號 + 檔案清單寫進 `info.json` | — |

> **先編譯、再碰 watchOS**：所有會動到 `info.json` / `watchOS\` 的步驟（3~5）都排在「編譯成功之後」。編譯失敗就直接停在第 2 步，不會留下半寫的發布介紹或過時的 bin。版號 +1（第 1 步）必須在編譯前，因為版號會被編進韌體。

### 那 3 個問題怎麼回答

1. **`Edit release notes (info.json description) now? (y/N)`**
   - 按 `y` → 跳出 notepad，改 `"description"` 欄位（這就是「發布介紹」，會顯示給 App / OTA），存檔後關掉視窗，腳本才會繼續。
   - 按 `N`（或直接 Enter）→ 沿用現有介紹。

2. **`Include watchface folder in package? (y/N)`**
   - 要不要把錶面（jsroot）一起打包進去。不確定就按 `N`。

3. 編完會印 **`RELEASE BUILD DONE`**，產物在 `watchOS\sys\` + `info.json`。

### 編譯失敗會怎樣

腳本會掃 `_watch_build.log`，若發現錯誤會停下來印 **`BUILD FAILED`**，並提醒你「目前仍是發布參數狀態」。修好後可重跑，或先切回開發（見第 5 節）。

---

## 4. 發布版到底改了什麼

`make_release.bat` 第 1 步（= `set_build_mode.py release`）會把這些值在「開發 / 發布」之間切換：

| 檔案 | 設定 | 開發 (dev) | 發布 (release) | 為什麼 |
|---|---|---|---|---|
| `customer/boards/eh-lb56xu/bsp_board.h` | `kReleaseMode` | `0` | `1` | 關掉手勢指示器等 debug UI、降 log 等級 |
| | `CUSTOMER_BOARD_VER` | `BOARD_VER_28` | `BOARD_VER_29` | 量產板腳位 |
| `src/modules/model/watch_global_data.h` | `VERSION_REVISION` | 不動 | **+1**（僅 dev→release 時） | 版號遞增 |
| `project/hcpu/proj.conf` | `RT_USING_FINSH` | 開 | 關 | 拿掉 shell |
| | `BSP_USING_VIRTUAL_CONSOLE` | 關 | 開 | console 走虛擬串口 |
| | `BSP_PM_DEBUG` | 開 | 關 | 拿掉每次睡/醒的 log |
| | `RT_USING_MEMTRACE` | 開 | 關 | 拿掉每次配置記憶體的追蹤開銷 |
| `project/lcpu/proj.conf` | `GH3018_POW_PIN` | `161` | `0` | 量產板沒有心率電源開關腳位 |
| | `BMI270_POW_PIN` | `118` | `0` | 量產板沒有 IMU 電源開關腳位 |
| | `RT_USING_MEMTRACE` | 開 | 關 | 同上 |

> **info.json 的版號是自動的** —— 由 `update_info.py` 直接讀 `watch_global_data.h` 寫入，你不用手改。唯一要手動的是 `description`（發布介紹）。

> **`BT_FINSH` 看似 debug 但不能關**：它的名字像是「藍牙 shell」，但 `middleware/bluetooth/service/bt/bt_finsh/` 同時是 HFP 免持通話（接聽 / 掛斷，`bt_hfp_hf_answer_call_send` 等）的唯一實作來源，`app_incoming_call.c` 會連結到它。關掉會導致發布版 link 失敗（`Undefined symbol bt_hfp_hf_answer_call_send`）。所以 dev / release 兩邊都保持 `=y`，腳本不碰它。

---

## 5. 發布完，切回開發

發布版編完、打包好之後，**記得切回開發參數**，否則接下來的開發 build 會少了 shell / log / 心率電源：

```cmd
python set_build_mode.py dev
```

這會把上表所有值切回 dev（包含修正 lcpu 電源腳位 161 / 118）。**版號不會退回**（已遞增的版號是消耗掉的，正確行為）。

---

## 6. 進階：單獨使用 set_build_mode.py

如果你想分步驟做、或只想檢查目前狀態：

```cmd
REM 看目前是 開發 還是 發布,以及版號 (不改任何檔)
python set_build_mode.py status

REM 預覽切到發布會改什麼,但「不」真的寫檔
python set_build_mode.py release --dry-run

REM 真的切到發布參數
python set_build_mode.py release

REM 切回開發參數
python set_build_mode.py dev
```

`status` 輸出範例：

```
Current build profile: DEV
  kReleaseMode       = 0
  CUSTOMER_BOARD_VER = BOARD_VER_28
  version            = 1.1.59
```

**建議**：第一次用、或剛 pull 完 main 之後，先跑一次 `release --dry-run` 看看要改的東西對不對，再正式跑。

---

## 7. 版號規則

- 版號 = `VERSION_MAJOR.VERSION_MINOR.VERSION_REVISION`（定義在 `watch_global_data.h`）。
- 每次 **dev → release** 轉換時，腳本自動把 `VERSION_REVISION` **+1**。
- 同一個 release 狀態重跑 `release` **不會**再加（避免重複跳號）。
- 要改 major / minor，或想指定特定版號：先手動編輯 `watch_global_data.h`，再跑腳本。

---

## 8. 疑難排解

| 症狀 | 原因 / 處理 |
|---|---|
| `make_release.bat` 印 `BUILD FAILED` | 編譯錯誤。看 `_watch_build.log`。修好後重跑；想先回開發就 `python set_build_mode.py dev`。 |
| 版號跳了好幾號 | 多跑了幾次 `release`（每次 dev→release 都會 +1）。檢查 `status` 確認目前版號，必要時手動改 `watch_global_data.h`。 |
| 開發時心率/IMU 沒反應 | 可能停在發布參數（POW_PIN=0）。跑 `python set_build_mode.py dev` 切回（會還原 161 / 118）。 |
| `info.json` 的中文介紹變亂碼 | 用 notepad（腳本內建那個流程）或 VS Code 以 **UTF-8** 存檔，不要用會轉成其他編碼的編輯器。 |
| 想確認沒有改錯 | `python set_build_mode.py status` + `git diff` 檢查實際變更。 |
| 板號 28 / 29 是不是反了？ | 沒反：**開發=28、發布=29**。歷史上每個真正的發布 commit（`kReleaseMode=1`）都是 `BOARD_VER_29`；`andrew_v28.x` 分支名只是版本標籤，不是板號。 |

---

## 9. 注意事項

- 這兩支腳本只動 **eh-lb56xu** 這塊板的 `bsp_board.h`（這就是量產 `scons --board=sf32lb56w-watch` 實際吃的板）。PC 模擬器和 `sf32lb56-watch_base` 那兩份 header 有自己的板號，腳本**不會**碰。
- 剛 pull 完 main，建議先 `status` + `release --dry-run` 確認腳本仍對得上最新的 proj.conf 結構。
- 發布產物（`watchOS\sys\` 的 bin + `info.json`）才是要拿去 OTA / 燒錄的東西。
- Git：要不要把「發布參數狀態」連同 bin 一起 commit，依團隊慣例。**不要在沒人同意下 push 共享分支。**

---

*工具位置：`example/get-started/dualcore/project/hcpu/{make_release.bat, set_build_mode.py}`*
