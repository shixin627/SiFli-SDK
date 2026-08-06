# Skai SDK — 開放式接口規畫（v0 draft）

> 目標：讓第三方 / AI 生成的小程式能取用手錶「全部」能力，而不是今天寫死在兩個 enum 裡的 14 種。
> 對標：[Pebble Foundation C API](https://developer.repebble.com/docs/c/Foundation/)（同樣跑思澈 SF32LB 系列）。
> 狀態：**規畫稿，尚未動任何 production code。**

---

## 0. 現況盤點：為什麼今天「不能自訂」

能力其實都在，缺的是**邊界**。

| 層 | 現況 | 問題 |
|---|---|---|
| 能力實作 | `src/modules/bloc/` 15 個 BLOC、`client/` 4 個跨核 RPC、`service/` 6 個服務、`model/` 跨模組介面 | 簽章各自為政、無版本、無 thread 契約、無權限概念 |
| 小程式層 | `gui_apps/skaiapp/` — 宣告式 JSON 包（BLE 推送，`/skaiapp/<id>.json`，上限 8 支 × 8 KB） | **能力被 enum 寫死**：8 種 widget、9 種 bind、5 種 action |
| 新增一個能力要動 | `skaiapp_pkg.h` enum → `skaiapp_pkg.c` parser → `skaiapp_render.c` renderer → `skaiapp_engine.c` runtime | 4 個檔、每次都要 firmware 改版 → 這就是「開放不了」的根因 |

**核心診斷**：`skaiapp` 是**模板系統**，不是 SDK。Pebble Foundation 是**穩定 ABI + 事件服務模型**。差別不在功能多寡，在於「新增能力」的邊際成本 —— 今天是 O(改 4 個檔 + 發版)，目標是 O(表格加一列)。

**既有資產（超乎預期的好）**：
- `external/quickjs/` 已 vendor 完整 QuickJS **含 LVGL v8 binding**（`lvgl_v8_qjs.c` / `lv_qjs_generated.c` / `lvgl_app_qjs.c`），Kconfig 有 `QUICKJS_USING_PSRAM`（預設 512 KB heap）。**目前在 `proj.conf` 被註解未啟用。**
- PC 模擬器（`_pc_build.cmd` / `_dev_test.cmd`）已成熟 — 這是 Pebble 生態當年最大的護城河（CloudPebble + emulator），我們已經有一半。
- 8 MB PSRAM + ~87 MB FS 分割區 — 空間上完全撐得起腳本 runtime。
- BLE 私有協定 L1/L2（`communicate/`）已能推包、ACK、分塊 — AppMessage 等價物的傳輸層現成。

---

## 1. 硬體參數表

格式對齊 Pebble 的硬體規格揭露方式（Pebble Time 2 欄位取自 [Zephyr PT2 board doc](https://docs.zephyrproject.org/latest/boards/coredevices/pt2/doc/index.html) 與 [CNX Software](https://www.cnx-software.com/2025/05/14/sifli-sf32lb52j-big-little-arm-cortex-m33-bluetooth-mcu-powers-the-core-time-2-smartwatch/)）。

### 1.1 主規格

| 項目 | Skaiwalk Watch (本機) | Pebble Time 2 | 來源 / 備註 |
|---|---|---|---|
| **SoC** | SiFli **SF32LB56x** | SiFli SF32LB52JUD6 | 規格書 `DS5601-SF32LB56x-CN V1.9.2` |
| **架構** | 大小核 Cortex-M33 STAR-MC1，**合計 1378 CoreMark** | 同系列 | `HCPU`=GUI/BLE host/AI、`LCPU`=sensor/motor/charger |
| **HCPU（大核）** | **240 MHz**，370 DMIPS / 984 CoreMark，Cache 32K(2w)+16K(4w)，**FPU + MPU** | — | 功耗效率 <34 µA/MHz @3.3V |
| **LCPU（小核）** | **96 MHz**，148 DMIPS / 394 CoreMark，Cache 16K(2w)+8K(4w)，**FPU + MPU** | — | 功耗效率 <13.5 µA/MHz @3.3V |
| **GPU** | **ePicasso™ 2.0** 2D/2.5D，四圖層 alpha 混疊，硬體旋轉/縮放/鏡像，最大 1024×1024 | SF32LB LCDC | `CONFIG_BSP_USING_EPIC=y` |
| **NPU** | TinyML 神經網路矩陣加速器，**1.92 GOPS，>10 TOPS/W** | 同系列具備，未使用 | `CONFIG_BSP_USING_CMSIS_NN=y` + TFLite Micro |
| **SRAM** | **960 KB** = HCPU **800 KB**（含 128 KB Retention）+ LCPU **160 KB**（全 Retention） | 511 KiB | 規格書 p4。`mem_map.h` 只把 LCPU 的 96 KB 開給應用，其餘為 patch / ROM RAM。**HCPU 800 KB 是本案最硬的天花板**（見 §5） |
| **PSRAM** | **8 MB**（app_exec 2.5 MB + PSRAM_DATA 5.5 MB） | 16 MB (SoC 內建) | `ptab.json` + `PSRAM_BL_SIZE: 8` |
| **Flash** | **960 MB SPI NAND** @ MPI3 mode 1 | GD25Q256E 256 Mb (32 MB) QSPI **NOR** | `msize` 單位為 MB（`customer/boards/common/flash.c:880`）；`BSP_USING_SPI_NAND=y`。**分割表只用 ~106 MB，尚有 ~854 MB 未配置** |

### 1.2 顯示 / 輸入

| 項目 | Skaiwalk Watch | Pebble Time 2 |
|---|---|---|
| **面板** | 1.43" **AMOLED**（DO0143FMST08 / TT151AMC60C） | 1.5" JDI LPM015M135A **MiP 記憶像素**，64 色 |
| **解析度** | **466 × 466**（圓形），461 DPI | 228 × 200（方形） |
| **驅動 IC** | CO5300，QAD-SPI 介面 | SF32LB LCDC |
| **背光** | PWM4 ch4 @ GPIO 122 | AW2016 RGB 背光驅動 |
| **觸控** | 電容觸控 @ I2C1，IRQ GPIO 50 | CST816D |
| **實體鍵** | **1 顆**（KEY1 @ GPIO 128, active-high） | **4 顆** |
| **手勢** | 抬腕喚醒、air-mouse、手勢辨識（TFLite） | — |

> ⚠️ **對 SDK 的直接影響**：Pebble app 全部假設「4 鍵 + 方形 + 低色深」。我們是「1 鍵 + 圓形 466² + 全彩 AMOLED」。**不能照抄 Pebble 的 `ButtonId` / window stack 語意** —— 導航必須以觸控手勢為主，見 §2.4。

### 1.3 感測 / 電源 / 音訊

| 項目 | Skaiwalk Watch | Pebble Time 2 |
|---|---|---|
| **IMU** | **BMI270** 六軸（加速度 + 陀螺），掛 LCPU | LSM6DSOW + LIS2DW12 |
| **心率** | **GH3018** PPG（匯頂），掛 LCPU | GH3026 |
| **磁力計** | **無** | MMC5603NJ |
| **環境光** | **無** | W1160 |
| **馬達** | LRA（`bloc_motor` 驅動，pattern 化） | AW86225CSR |
| **電池 / 充電** | ADC1 量測，充電狀態走 LCPU → `SkaiWatchSys.charger_status` | nPM1300 PMIC |
| **麥克風** | 內建 CODEC 錄音（AUDPRC RX0 DMA） | 雙 PDM MIC |
| **喇叭** | 內建 CODEC + PA | AW8155BFCR |
| **音訊 codec** | Opus、MP3 (libhelix)、WebRTC (AEC/NS) | — |
| **藍牙** | 雙模 BLE 5.x + **BR/EDR HFP-HF**（可接聽通話）、ANCS、AMS、HID | BLE (mailbox HCI) |
| **RTC** | 片上 RTC | 片上 RTC |

> **本機獨有能力（Pebble 沒有，是差異化 SDK 賣點）**：語音轉文字 `bloc_v2t`、AI 手勢辨識、air-mouse 指標、HFP 通話音訊、Opus 錄音、TFLite 端側推論、WebRTC 前處理。

### 1.4 Flash 分割表（`ptab.json`）

| 區域 | 位址 | 大小 | 用途 |
|---|---|---|---|
| `main` (app_img) | flash3 `0x64000000` +0 | 2.5 MB | HCPU 韌體，載入至 PSRAM `0x60000000` 執行 |
| `dfu` | PSRAM +0 | 512 KB | OTA |
| `FS_REGION` | flash3 +`0x280000` | **87.5 MB** | 檔案系統（小程式包、資源、log 都在這） |
| `KVDB_DFU` / `KVDB_BLE` / `KVDB_PREFDB` | flash3 +`0x5A00000` | 16 KB × 3 | FlashDB 鍵值庫 |
| `BLE_OTA_REGION` | flash3 +`0x5A20000` | 16 MB | OTA 暫存 |
| **未配置** | flash3 +`0x6A20000` 以後 | **~854 MB** | 目前完全沒用到 |

> 分割表只映射到 ~106 MB / 960 MB。**儲存空間對第三方 app store 而言是非問題** —— 今天限制「8 支 × 8 KB」純粹是軟體策略。對照 Pebble Time 2 的 32 MB NOR，我們有 **30 倍**空間。

### 1.5 SoC 加速器與安全（規格書 p4–p5，對 SDK 設計有直接影響）

| 區塊 | 能力 | 對 SDK 的意義 |
|---|---|---|
| **安全** | AES / HASH / CRC / TRNG 硬體加速器、**Secure Boot**、**1024-bit eFuse 存 Root of Trust + UID**、**PSA Certified Level 1** | **第三方 app 套件簽章的基礎設施已內建**。開放外部開發者時，套件簽章驗證幾乎零額外成本 —— 見 §2.5 |
| **eZip™ 2.0** | 硬體無損圖形解壓，支援無損動畫 eZip-A，可與 ePicasso 直連免中間緩衝 | `skai_res` 的資源格式直接用 eZip，省 PSRAM 頻寬（專案已啟用 `LV_USE_EZIP`） |
| **DSP 加速** | HCPU 有 FFT 加速器 + FIR 濾波加速器；**兩核各一個 CORDIC 三角函數協處理器** | `skai_math` 可提供硬體加速三角/FFT，比 Pebble 的軟體 `sin_lookup` 強一個量級 |
| **音訊** | 24-bit DAC（SNR 109 dB）/ 24-bit ADC（SNR 99 dB）、2×PDM MIC、I2S、**取樣率轉換 + EQ 加速器**、支援 **BLE Audio** | `skai_audio` 值得完整開放 |
| **儲存介面** | 4×MPI，支援 QSPI-NOR / SPI-NAND / QPI-OPI-PSRAM；2×SD/SDIO/eMMC | 現用 MPI1=PSRAM、MPI3=SPI-NAND |
| **低功耗** | RTC 休眠 600 nA、腳位喚醒 300 nA | `skai_wakeup` 的排程喚醒成本極低 |

---

## 2. SDK 架構設計

### 2.1 核心決策：**兩種受眾，兩種契約，一個真相源**

```
        外部開發者                          內部開發者
  （第三方 / AI 生成小程式）              （Skaiwalk 韌體工程師）
            │                                    │
            ▼                                    │
┌───────────────────────────────┐                │
│  外部腳本 API（JS）             │                │
│  凍結 · 版本化 · 權限 · 配額 · 簽章 │  ← 真正要凍結的邊界
└───────────────────────────────┘                │
            ▲                                    │
            │ 自動投影（非手寫）                    │
┌───────────────────────────────┐                │
│  skai_dispatch — 能力派發表      │  ← 單一真相源     │
└───────────────────────────────┘                │
            ▲                                    │
            │ 由標註產生                           ▼
┌────────────────────────────────────────────────────┐
│  skai_api — 內部 C 能力層（API，非 ABI；可自由重構）      │
├────────────────────────────────────────────────────┤
│     現有 bloc_* / *_client / service / model（不動）    │
└────────────────────────────────────────────────────┘
```

**根本差異只有一條，其他全是推論：**

> **內部人員能刷機，外部人員只能裝 app。**

| | 內部 C API | 外部腳本 API |
|---|---|---|
| 受眾 | 自家韌體工程師 | 第三方 / AI 生成小程式 |
| 語言 | C | JS（見 §2.7） |
| **能不能刷機** | **能** | **不能，只能裝 app** |
| 因此：更新單位 | 整包韌體，API 與呼叫端**同一個 commit 一起變** | 只有 app 換，**韌體是他控制不了的變數** |
| 因此：相容性負擔 | **API 不是 ABI** —— 可自由重構，改壞了編譯就會報錯 | **必須凍結** —— 舊 app 在新韌體上不能壞，且錯誤只會在使用者手上出現 |
| 因此：沙箱 / 權限 / 配額 | 不需要（改壞了自己刷回去） | **必須**（裝壞了使用者救不回來） |
| 因此：簽章 | 不需要 | **必須**（見 §2.5） |

**這條差異還推出一個容易漏掉的需求 —— 版本協商。**
外部 app 存在錶上、跨 OTA 存活，所以會出現「**新韌體跑舊 app**」和「**舊韌體跑新 app**」兩種歪斜，而且都在使用者手上發生：

- app manifest 必須宣告 `"skai": ">=1.2"`，韌體版本不足 → **安裝時就拒絕並回報**，不是執行到一半才炸
- runtime 提供 `skai.available("health.hr")` 讓 app 自己降級（能力可能因機型或權限而缺）
- 新增能力只准「加」，不准改語意 —— 這是凍結的實際含義

Pebble 早期在這件事上吃過虧（app 版本與 firmware 能力沒協商，導致大量 app 在 firmware 升級後靜默失效）。這三點成本很低，但**必須在 Phase 0 就進 schema**。

> **這個切分直接讓 Phase 0/1 變便宜**：原本我為整層 C API 設計的 opaque handle + 版本化 struct + 語意永不變更，只有**外部那一側**需要。內部 C 層只要有一致的命名、thread 契約、邊界檢查即可。

**為什麼一定要中間那層 `skai_dispatch`？**
這是「開放 vs 不開放」的分水嶺。今天新增一個能力要改 4 個檔；有派發表之後，新增能力 = **在表裡加一列**，宣告式 JSON binding 和 JS global **同時自動獲得**。

**更重要的是：它防止兩套 API 分岔。** 若內部 C 與外部腳本 API 各自手維護，外部那套必然落後（這是所有雙層 SDK 的通病）。因此：

> **外部腳本 API 必須是內部 C API 的「自動投影」，不是手寫的第二套。**
> 派發表由標註過的 header 產生（`tools/sdk/gen_dispatch.py` 掃 `SKAI_EXPORT(...)` 巨集），手維護的表必然腐化。

**同時要防的第二種腐化**：內部工程師會走捷徑直接呼叫 `bloc_*`，繞過 `skai_api` —— 那層就慢慢變成沒人用的裝飾。建議訂一條可執行的規則：

> 新的 `gui_apps/` 一律只准呼叫 `skai_*`；`gui_apps/` 內出現 `bloc_` / `*_client` 直接呼叫 → build 警告（可用 SConscript 加一支 grep 檢查，成本極低）。既有 app 不追溯。

### 2.2 Pebble Foundation → Skai 對照表

| Pebble 模組 | Skai 命名空間 | 後端（已存在） | 工作量 |
|---|---|---|---|
| App / event loop | `skai_app` | `gui_app_framework`, `ui_handler` | 包裝 |
| Event Service · TickTimer | `skai_tick` | RTC + lv_timer | 包裝 |
| Event Service · Accelerometer | `skai_accel` | `imu_client`, `acce_service` | 包裝 |
| Event Service · BatteryState | `skai_battery` | `SkaiWatchSys.battery_level_value` | 包裝 |
| Event Service · Health | `skai_health` | `bloc_health`, `hr_service`, `sleep_service`, `bloc_exercise` | 包裝 |
| Event Service · Connection | `skai_connection` | `ble_device_manager` | 包裝 |
| Event Service · AppFocus | `skai_focus` | `ui_handler` | 包裝 |
| Event Service · Backlight | `skai_backlight` | LCD PWM + `disp_refr_governor` | 包裝（**無環境光感測器 → 只有手動亮度，不提供 auto-brightness**） |
| Event Service · Touch | `skai_touch` | `touch_state_manager`, `gesture_handler` | 包裝 |
| **Event Service · Compass** | — | **無磁力計 → 不實作** | ❌ |
| Timer | `skai_timer` | `skaiapp_engine` + rt_timer | 泛化 |
| Wakeup | `skai_wakeup` | `alarm_manager_service` | 泛化 |
| Wall Time | `skai_time` | RTC + `watch_global_data.time` | 包裝 |
| Storage (`persist_*`) | `skai_persist` | FlashDB KVDB + `bloc_setting` | 薄新寫 |
| Resources | `skai_res` | LVGL ezip/freetype + FS | 新寫 |
| AppMessage / AppComm | `skai_msg` | `communicate` L1/L2 SKAI_LINK | 包裝 |
| DataLogging | `skai_datalog` | `bloc_filesystem` + communicate | 新寫 |
| Dictation | `skai_dictation` | **`bloc_v2t`（我們已有，Pebble 靠手機）** | 包裝 |
| Dictionary | `skai_dict` | — | 小新寫 |
| Logging | `skai_log` | ulog | 包裝 |
| Math | `skai_math` | `util/math_fixed`, `trig` + **CORDIC / FFT / FIR 硬體加速器** | 包裝（可超越 Pebble 的軟體實作） |
| Memory Management | `skai_heap` | `app_mem` / rt_malloc pool | 包裝 |
| Platform / WatchInfo | `skai_watch_info` | board defines + version | 薄新寫 |
| Launch / Exit Reason | `skai_launch` | gui_app framework | 薄新寫 |
| App Glance | `skai_glance` | `ui_handler` app list | 新寫 |
| AppWorker | `skai_worker` | `skaiapp_engine` 背景執行緒 | 泛化 |
| i18n | `skai_i18n` | `.arb` 資源 | 包裝 |

**Skai 專屬擴充（Pebble 沒有）**：

| 命名空間 | 後端 | 說明 |
|---|---|---|
| `skai_voice` | `bloc_v2t`, speech app | 語音輸入 / 意圖 |
| `skai_ai` | TFLite Micro | 端側推論、自訂模型 |
| `skai_motion` | `gesture_handler`, air-mouse, `bloc_motion_tracking` | 手勢 / 指標 |
| `skai_haptic` | `bloc_motor` | 震動 pattern（比 Pebble vibes 豐富） |
| `skai_notify` | `bloc_notification` + ANCS/AMS | 通知讀取與互動 |
| `skai_audio` | recorder / Opus / media | 錄音、播放 |
| `skai_weather` / `skai_calendar` | `bloc_weather`, `bloc_calendar` | 資料源 |

### 2.3 兩套規則（**不要混用**——這是本案最容易做錯的地方）

#### A. 內部 C API —— 輕規則（因為能刷機）

只要求「一致」與「安全」，**不要求凍結**。過度設計這層會拖垮進度且沒有收益。

1. **命名一致**：`skai_<域>_<動作>()`，域名與派發表的 namespace 一一對應。
2. **Thread 契約寫在 header**：每個函式標明可從哪個 thread 呼叫、callback 在哪個 thread 交付。**這是目前最容易出事的地方**（LVGL thread vs engine thread vs BLE parse thread），且 C 層再怎麼可重構也修不掉這種 bug。
3. **全部有界**：取字串/緩衝的 API 一律帶 `cap` 參數（延續 `skaiapp_pkg.c` 現有紀律）。
4. **可自由重構**：改簽章、換 struct、拿掉函式都可以 —— 呼叫端同 commit 一起改，編譯器會抓。**不做 opaque handle、不做版本化 struct、不留 compat shim。**

#### B. 外部腳本 API —— 重規則（因為只能裝 app）

1. **只加不改**：新增能力只准加；既有能力的**語意永不變更**。破壞性變更 → bump major，舊 major 保留 compat 投影。
2. **版本協商**：manifest 宣告 `"skai": ">=1.2"`，安裝時檢查；runtime 提供 `skai.available(cap)` 供 app 降級。
3. **權限宣告**：manifest 列 `"capabilities": ["health.hr", "voice.dictation"]`，runtime 拒絕未宣告的呼叫。心率 / 麥克風 / 檔案 / 通訊類必須 opt-in，安裝時對使用者揭露。
4. **配額**：每支 app 有記憶體上限、執行時間 watchdog、儲存配額。**外部程式碼寫壞不能拖垮錶**（使用者無法刷機自救）。
5. **簽章**：見 §2.5。
6. **輸入永不信任**：外部 app 傳進來的一切都當作敵意輸入處理。

> **判斷準則**：問「這個東西改了，會不會讓某支已經裝在使用者錶上的 app 壞掉？」
> 會 → B 規則。不會 → A 規則，別過度設計。

### 2.4 導航模型：不能照抄 Pebble

Pebble 的 `Window` stack + 4 鍵（Up/Select/Down/Back）語意在 1 鍵圓形錶上無效。建議：

- **Tier 0** 維持單頁滾動（現況），零成本。
- **Tier 1** 提供 `skai_window` 但語意重定義：`back` = 右滑手勢、`select` = 點擊、上下 = 觸控滾動、單一實體鍵 = 回錶面。
- 圓形螢幕的 layout helper（`skai_layout_arc` / 安全區）要內建，否則第三方一定畫出被切角的 UI。UI 規範對齊 `C:\skaiwalk\.claude\skills\Skaiwalk_UI\SKILL.md`。

### 2.5 套件簽章 —— **開發者自簽模式（已定案）**

#### 自簽改變的是簽章的「用途」，不是「要不要簽」

封閉商店模式下，簽章 = **授權**（Skaiwalk 審過了）。自簽模式下簽章**不再代表任何安全保證**，它只剩三個用途：

| 用途 | 說明 |
|---|---|
| **完整性** | 套件傳輸中沒被竄改（現有 CRC-32 只防損毀，不防竄改） |
| **作者連續性（TOFU）** | app 的**更新**必須由當初那把私鑰簽 —— 防止別人冒名推更新劫持已安裝的 app |
| **可歸因** | 出事後能鎖定並封鎖那把金鑰 |

> **最重要的推論：權限模型從「加分項」變成唯一的安全邊界。**
> 既然任何人都能簽，「這支 app 被允許做什麼」就是**僅存的防線**。§2.3-B 的權限與配額不再是設計品味問題，是安全需求。

#### 技術選型（全部是現成路徑）

- **簽章演算法：ECDSA P-256**，用已 vendored 的 `external/mbedtls`（`ecdsa.c` / `ecp.c` / `pk.c`）。Kconfig 的 `PKG_SIFLI_MBEDTLS_BOOT` 表示 **bootloader 的 secure boot 已經在跑這條驗簽路徑**，不是新路。
- **雜湊：SHA-256**，走 SoC 的 HASH 硬體加速器。
- **驗簽只在安裝時做一次**，240 MHz M33 上是毫秒級，不影響執行期。
- ⚠️ **eFuse RoT 在這個模式下用不到** —— 更正我先前的寫法。eFuse 的信任根是給 **Secure Boot（韌體）**用的；自簽 app 是自我為根，公鑰直接放在 manifest 裡。硬體 HASH 加速器仍然會用到。

#### 必須在 Phase 0 一起定的四件事

1. **manifest 帶公鑰 + 簽章欄位**；`skai_pkg_verify()` 用 manifest 內的公鑰驗自己。
2. **TOFU 儲存**：首次安裝 app 時把 `keyid = SHA-256(pubkey)[0:8]` 存進 KVDB。更新時 keyid 不符 → **拒絕**（發佈者換人了），要換必須使用者手動移除重裝。
3. **app id 以 `(keyid, app_id)` 為鍵**，不是 `app_id` 單獨為鍵 —— 否則兩個開發者宣告同一個 id 會互相覆蓋。顯示名稱撞名是 UX 問題，不是安全問題，另外處理。
4. **撤銷清單（blocklist）**：手機端可下推被封鎖的 keyid。**現在設計進 schema 幾乎免費，事後補要動已發佈的格式。** 開放自簽而沒有撤銷手段，等於出事後無法止血。

#### 因為使用者才是承擔風險的人 —— 安裝時的權限揭露 UI

自簽模式把風險判斷交給使用者，前提是**使用者真的看得到自己在同意什麼**。這是原本計畫沒有的 UI 工作項：安裝流程需列出該 app 宣告的權限、發佈者 keyid、以及「此 app 未經 Skaiwalk 審核」的明示。逐項同意，不要做成單一「全部接受」按鈕。

#### 建議：即使開放自簽，能力仍分三層

自簽模式不代表所有能力都得對外開放。建議：

| 層 | 內容 | 授權方式 |
|---|---|---|
| **T1** | UI/繪圖、自身沙箱儲存、計時器、時間、電量、步數 | 自動授予 |
| **T2** | 心率 / 睡眠等健康資料、通知讀取、檔案系統、天氣 / 日曆 | **安裝時逐項使用者同意** |
| **T2-mic** | **原始麥克風音訊、語音轉文字** | **開放**，但強制系統級使用中指示器 + 前景限定（見 §2.6） |
| **T3** | 系統設定寫入、AI 模型載入 | **永不對自簽 app 開放**（已定案） |
| **T4** | BLE central（連第三方週邊） | **本輪不開，但保留，見 §2.9** |

T3 的理由是可被用來劫持裝置（系統設定）或有模型安全問題（AI 模型載入）。日後要放行時再放，反向收回會破壞相容性。

### 2.9 BLE 開放性分析（T4）

#### 先拆詞 ——「原始 BLE」其實是四種能力，風險差很多

| # | 能力 | 判定 |
|---|---|---|
| 1 | **app ↔ 自己的手機端 app 訊息** | **這不該叫 raw BLE。** 走現有 L1/L2、手機仲介、可觀測 → 就是 `skai_msg`，已在 Phase 4 |
| 2 | **手錶當 central 連第三方週邊**（心跳帶、踏頻器、智慧鎖） | **真正在討論的就是這個** |
| 3 | 手錶當 peripheral 開自訂 GATT service | 不開（會讓 app 對外開通道，且與手機連線爭資源） |
| 4 | 原始 HCI / L2CAP | 不開 |

#### 現況：能「連」，不能「找」

`ble_device_manager.c:482` 有 `ble_gap_create_connection()`，且有完整的多裝置管理與 bond 儲存（`bonded_devices_db_t`）。但**專案內沒有任何掃描程式碼** —— `ble_dev_mgr_add_device()` 直接吃 MAC 位址。

> 所以缺的那塊正是 **discovery**。而 discovery 恰好是風險幾乎全部集中的地方。

#### 開放的好處（真實存在，不是場面話）

1. **這是 Pebble 給不了的能力類別。** 運動週邊、IoT 控制、醫療配件 —— 這類 app 沒有 BLE central 就**根本不可能存在**。
2. **開啟的是你自己不會做的 app**，不與內建功能競爭，卻讓錶更黏。
3. **脫離手機的使用情境** —— 運動時不帶手機直連心跳帶，這正是專用手錶的價值主張。
4. **平台開放性的招牌能力。** 要外部開發者認真看待這個平台，這是分水嶺級的能力。

#### 代價（依嚴重度排序）

1. **手機連線是生命線，而射頻時間是共用的。**
   通知、同步、OTA、以及你剛決定的除錯管線**全部**跑在手機連線上。第三方 app 掃描太兇或連線參數設差就會拖垮它 —— 而使用者的心智模型會是「**這隻錶壞了**」，不是「那支 app 爛」。連線不穩正是智慧錶使用者最不能容忍的失效模式。

2. **這是唯一逃出沙箱觀測範圍的能力 —— 最關鍵的一點。**
   整套安全模型是「自簽 + 權限閘 + 使用者同意 + blocklist」，但 **blocklist 與稽核都要看得到才有用**。BLE central 給 app 一條**繞過手機的頻外通道**：
   - T2 取得的健康資料可直接經 BLE 送到攻擊者的裝置，手機完全不知情
   - §2.8 的日誌管線看不到
   - 不像麥克風，**沒有自然的指示器可用**
   → **T2 健康資料 + BLE central 同時開，危險程度高於兩者各自單獨開。這是組合風險，不是加法。**

3. **掃描是位置側通道。** 附近裝置的 MAC 清單可反推位置。Android / iOS 都把 BLE 掃描綁在位置權限之後，正是這個原因。

4. **電池，而且無法用指示器解決。** 掃描是手錶最耗電的動作之一。麥克風可以「開著就亮燈」，掃描不行 —— 總不能一直亮。

5. **連線槽與 bond 槽有限。** `KVDB_BLE_REGION` 只有 16 KB。一支 app 佔滿會排擠其他 app，極端情況排擠手機。

6. **互通性支援成本落在你身上。** BLE 週邊互通惡名昭彰，「我的感測器連不上」的客訴會進到 Skaiwalk，不是 app 開發者那裡。

#### 關鍵解方：**系統仲介的裝置選擇器**（不是給 app 掃描器）

不要把掃描器交給 app，而是提供「使用者已選定的那一台裝置的 handle」。Web Bluetooth 與 Android CompanionDeviceManager 都是這個做法。一個 UI 畫面同時解掉三個問題：

| 問題 | 為何被解掉 |
|---|---|
| 位置側通道 | app **永遠拿不到**附近裝置清單，只拿到使用者選的那一台 |
| 電池 | 掃描只在系統選擇器開著時發生，且有時限 |
| 使用者知情 | 選擇器**本身就是**同意流程，不需要另做 |

**而且它和現有程式碼形狀吻合** —— 既有流程是「用 MAC 加裝置 → 用 index 連」，選擇器產出的正好是 MAC。缺的 discovery 由系統做，app 拿不到原始結果。

#### 若要開放，四個前置條件

1. **系統仲介裝置選擇器**（上述）
2. **手機連線優先權在協定層強制** —— 保留連線槽，app 連線在任何情況下先被砍，不能靠政策約束
3. **manifest 宣告 GATT service UUID 白名單**，連線時強制
4. **射頻時間歸屬統計** —— 讓使用者看得到「這支 app 用掉多少電」

#### 建議：**本輪不開，排 Phase 6**

不是因為它不好，是因為：

- **Phase 3 的 gate 是沙箱**，而 BLE central 正好是唯一逃出沙箱觀測的能力 —— 混在同一階段會讓驗收焦點糊掉
- 讓它安全的四件事各自不大，但合起來是一個**獨立的工作塊**，硬塞進 Phase 3 會兩件事都做不好
- `skai_msg`（能力 #1）已涵蓋多數 app 的實際需求，且早就在計畫內
- **晚開放相容，晚收回不相容** —— 方向性站在「先不開」這邊

> 這是「可以，之後做，成本是這四件事」，不是「不行」。

> ℹ️ **一個需要納入設計的事實**：心率 / 睡眠屬健康資料。在自簽模式下，任何開發者的 app 只要使用者按下同意就能讀取。若產品要進歐盟 / 英國市場，GDPR 將健康資料列為特種個資，要求**明確且針對特定用途**的同意 —— 這是上面「逐項同意、不要做成單一全部接受」的實際依據，也是 T2 分層的理由。這不改變自簽的決定，只是決定同意 UI 怎麼做。

### 2.6 麥克風開放的前提：**系統級使用中指示器**

開放原始音訊與 v2t 給自簽 app，唯一能讓它站得住的機制就是「麥克風一開，使用者一定看得到」。這是 iOS / Android 隱私指示器的同一套邏輯。

#### 鐵則：指示器必須由**系統**畫，不能由 **app** 畫

app 自己畫的指示器對惡意 app 完全無效 —— 不畫就好了。所以：

| 要求 | 做法 |
|---|---|
| **app 蓋不掉** | 畫在 **`lv_layer_sys()`**（`external/lvgl_v8/src/core/lv_disp.h:200`）。它在 `lv_layer_top()` **之上**，而 app 內容與現有全域輸入條（`s_global_bar_layer`）都在 top 層。**專案目前完全沒用過 sys layer，等於是留給這件事的空位。** |
| **app 關不掉** | **不提供任何隱藏它的 API。** 這點要寫進 §2.3-B 規則。 |
| **不可能「開麥但沒指示」** | 顯示條件綁**實際的音訊擷取狀態**（capture open 的 refcount），不是綁 app 有沒有禮貌地宣告。app 走哪條路徑開麥都逃不掉。 |

#### 視覺不用重做，要做的是「升層」

現有的 mic bar（`lv_instruction_list_layout.c` 的 `s_global_bar_layer`，通知回覆頁下方那顆圓形 icon）已經有「麥克風開啟 + 是否有人聲」的呈現，人聲判定走 `BSP_USING_WEBRTC_VAD`（`bloc_v2t` 的 `vad_inst`）。

**所以工作項不是設計新視覺，是把它從 app 層元件升級成綁定 capture refcount 的系統層指示器**，讓所有路徑（內建 app、外部 JS app、背景服務）共用同一個。視覺規格與擺放位置依 `C:\skaiwalk\.claude\skills\Skaiwalk_UI\SKILL.md`（466 圓形螢幕的安全區與一致性由該規範定，不在本文件決定）。

#### 兩個配套限制（否則指示器會被繞過）

1. **自簽 app 的麥克風限前景使用** —— 螢幕熄滅或 app 退到背景即自動停止擷取。
   理由很直接：**指示器只在螢幕亮著時看得到**，允許背景錄音等於允許「有麥克風、沒指示」的狀態存在，整個機制就破了。內建 app（如通話、語音喚醒）不受此限，因為那是能刷機那一側的信任範圍。
2. **v2t 用量配額** —— 語音轉文字可能走手機/雲端，有實際成本。開放給外部 app 就必須有每 app 配額，否則一支寫壞的 app 能把使用者的額度或電池吃光。

### 2.7 外部語言選擇：**JavaScript（QuickJS）**

建議直接定 JS，理由由強到弱：

1. **已經在樹裡** —— `external/quickjs/` 含完整 runtime + **LVGL v8 binding**（`lv_qjs_generated.c`）+ `QUICKJS_USING_PSRAM`。省掉數週移植。
2. **沙箱需求天然吻合 §2.3-B** —— QuickJS 原生支援 `JS_SetInterruptHandler`（執行時間 watchdog）與自訂 allocator（記憶體配額）。配額不用自己造。
3. **AI 生成品質** —— 你們的 app 有相當比例由 AI 生成，JS 是所有語言模型覆蓋最好的目標語言，遠勝 Lua / 自創 DSL。
4. **外部開發者門檻最低** —— 不需要學專屬語言，也不需要裝交叉編譯工具鏈（他們本來就不能刷機，也就不該需要 toolchain）。

**順帶免費拿到的 DX**：既然外部 API 是派發表的自動投影，同一個產生器多輸出一份 **`.d.ts` 型別定義**，外部開發者在 VS Code 就有自動完成與型別檢查。這是 Pebble 當年沒有的東西，成本幾乎為零。

**宣告式 JSON（現有 SkaiApp）不廢除**，理由有二：
- 已推送到使用者錶上的包不能壞（見 §2.3-B 規則 1）；
- 它仍是「AI 一句話生一個簡單 app」最快的路徑，複雜的才落到 JS。

兩者**都從同一張派發表投影**，所以能力永遠一致，不會出現「JSON 有、JS 沒有」這種分岔。

> 若要否決此選擇，唯一值得比較的替代是 Lua（體積更小），但它沒 vendored、沒 LVGL binding、AI 生成品質差一截 —— 除非有 SRAM 上的硬理由，否則不建議。

### 2.8 除錯管線：手錶 → 手機 App → 電腦 App

#### 既有資產與那個關鍵的卡點

`modules/model/ble_ulog_backend.c` 已經把 ulog 接到 BLE（`wristband_ble_log.h` 的 `ble_log_output(level, buf, len)`，256 B 緩衝，D/I/W/E 四級）。但它整段包在 **`#if !kReleaseMode`** 裡 —— **只有 debug build 有**。

而外部開發者**不能刷機**，拿到的永遠是 release 韌體。所以：

> **除錯管線必須在 release 韌體上就能運作，否則對外部開發者等於不存在。**

但也不能把系統 ulog 在 release 直接打開 —— 系統日誌含使用者資料（通知內容、健康數值），而且耗電耗頻寬。

#### 解法：**app 範圍的日誌流，不是系統日誌流**

| | 內部開發者 | 外部開發者 |
|---|---|---|
| 管道 | UART console（`tools/dev_console/`，COM14 @1M） | **BLE → 手機 App → 電腦 App** |
| 內容 | 完整系統 ulog | **只有自己那支 app 的日誌**（依 `keyid` 標記過濾） |
| 韌體 | debug build（他們能刷） | **release 韌體即可** |
| 前提 | 硬體治具 | 已配對的手機 |

這條分界和 §2.1 是同一條 —— **能刷機 vs 只能裝 app**。內部那條完全不動，外部這條是新增的。

- `skai.log()` 從 JS 寫入 → 以該 app 的 `keyid` 標記 → 進獨立 ring buffer
- 系統 ulog 在 release 維持關閉，**不動現況**
- **除錯 session 要使用者明示開啟**（手機 App 上對特定 app 開啟「除錯模式」），並**自動逾期關閉**。否則量產錶會持續外送資料
- 一支 app 只看得到自己的日誌

#### 傳輸架構：電腦**透過手機**連，不直連手錶

```
手錶 ──BLE（現有 L1/L2 私有協定）──> 手機 App ──WebSocket / USB──> 電腦 App
```

理由：桌面端 BLE 各作業系統行為不一致且難維護；手機**已經配對好**，不需要新的配對流程與新的安全面；只要維護一條傳輸路徑而不是兩條；手機還能在電腦沒連線時先緩衝日誌。

#### 三件對 JS SDK 價值最高、成本卻極低的事

1. **QuickJS 例外自動回傳** —— 未捕捉的例外帶行號與 stack trace 直接送到開發者主控台。這是 JS SDK 最高價值的除錯訊號，從 QuickJS 拿幾乎免費。
2. **配額違規要有結構化事件** —— 記憶體超限 / watchdog 觸發 / 權限被拒，三者必須明確回報，**不能只是靜默死掉**。這三種是外部開發者最容易卡住又查不出原因的情況。
3. **丟包要誠實回報** —— BLE 緩衝只有 256 B，日誌量大時必然要丟。必須回報「已丟棄 N 行」而不是安靜地少幾行（否則開發者會追一個根本不存在的 bug）。

#### 分期

**hook 要在 Phase 3 跟 runtime 一起設計**（keyid 標記、session 閘、結構化錯誤事件）—— 這些retrofit 很痛。手機 / 電腦端的 UI 排 Phase 5。

---

## 3. 可實行方案（分階段）

工期為單人全職粗估，含測試不含美術資源。

### Phase 0 — 凍結**外部**邊界（~1–1.5 週）✅ 2026-08-03 完成（除權限揭露 UI 設計稿）
不寫功能，只寫契約。**注意：要凍結的只有外部那側**（§2.3-B），內部 C 層不需要這道工。

- **外部 manifest schema**（延伸 `skaiapp-package.schema.json`），一次把五件事定齊，之後改不動：
  - `"skai": ">=1.2"` 版本協商欄位
  - `"capabilities": [...]` 權限宣告（含 §2.5 的 T1/T2/T3 分層）
  - **公鑰 + ECDSA P-256 簽章欄位**（§2.5）
  - **`keyid` 作為儲存主鍵的一部分** —— `(keyid, app_id)`，自簽模式下 app_id 不唯一
  - **撤銷清單格式** —— 手機端下推被封鎖的 keyid
- `skai_sdk_version.h`：`SKAI_API_MAJOR/MINOR`
- 兩套規則落成 ADR：`docs/adr/00XX-skai-two-tier.md`（§2.1 + §2.3 + §2.5 自簽決策）
- `SKAI_EXPORT()` 標註巨集 + `tools/sdk/gen_dispatch.py` 骨架（同時輸出派發表與 `.d.ts`）
- **安裝權限揭露 UI 設計稿**（§2.5）—— 自簽模式下使用者是承擔風險的人，這不是可延後的裝飾

**兩個 de-risk 探針 —— ✅ 2026-08-03 實測完成，結果推翻了本文原本的頭號風險**

1. **~~量測 HCPU SRAM 餘量~~ → SRAM 不是瓶頸。** 靜態 SRAM 成本只有 **0.8 KB**（`fromelf --text -z` 實測），而 HCPU runtime heap 有 **357 KB**（image 結束於 `0x2006AA38`，heap 到 `HCPU_RAM_DATA_END` `0x200C3C00`）。`skaiapp_pkg.h` 那句「chronically full」講的是執行期壓力，不是這個預算。
2. **✅ LVGL binding 相符，不用補 shim。** 專案 LVGL = **v8.3.1**，`external/quickjs/lvgl/` 是 v8 binding。它引用的 119 個 `lv_*` 有 54 個不在 LVGL core，但全部有著落：40 個在 `middleware/lvgl/lvsf/`（SiFli 擴充 widget，**本韌體已經 link 進去** —— `main.map` 內 4648 處引用），14 個定義在 binding 自己裡面。9 個 TU 全部編譯通過。
3. **🔴 真正的瓶頸是 flash，不是 SRAM。** 開 `PKG_USING_QUICKJS` + `QUICKJS_LVGL` + `QUICKJS_USING_PSRAM` **編得過但 link 不過**：`L6407E, aggregate size 0x17be8` —— **差 95 KB**。QuickJS + binding 要 **312.7 KB** code+RO（`quickjs.o` 241 KB + `libunicode.o` 41 KB 是大宗），而 2.5 MB `main` 分割區只剩 **264 KB**（`ER_IROM1: Size 0x23dff4 / Max 0x280000`），其餘約 46 KB 是 QuickJS 拖進來的 libc/libm。

> **這改變了 Phase 3 的形狀**：原本設想的「JS heap 放 PSRAM」不夠，還要解決 image 塞不下。三條路（見 ADR-0019）：擴 `main` 分割區（`ptab.json` 只映射了 960 MB 中的 ~106 MB，但 `main` 從 PSRAM `app_exec` 執行，要一起長）、拿掉 `libunicode`（要動 `external/` 的 Kconfig，屬 don't-touch）、或從現有 image 省出 95 KB。**擴分割區是唯一能持續的解，但它動 ptab + bootloader，不能走 OTA。**
- **驗收**：選 3 個垂直切片（`skai_time` / `skai_battery` / `skai_haptic`）走完 C header →（產生）派發表 →（投影）宣告式 binding，證明「加一列」的 pattern 成立。

### Phase 1 — 內部 C 能力層（~2 週，比原估省 ~1 週）✅ 2026-08-03 完成
把最常用的一批包成 `skai_*`：`skai_time`、`skai_battery`、`skai_health`、`skai_log`、`skai_persist`、`skai_timer`、`skai_haptic`、`skai_watch_info`。

**因為是 API 不是 ABI，這階段刻意求快** —— 不做 opaque handle、不做版本化 struct、不留 compat shim；設計錯了後面重構，編譯器會抓。省下來的時間投到 Phase 0 的外部 schema 與 Phase 3 的沙箱，那兩處錯了才是真的改不掉。

- 每個 API 一支 PC sim MSH 測試指令（沿用 `src/modules/tests/` 既有框架）
- 同步加上 §2.1 的 build 檢查：`gui_apps/` 直接呼叫 `bloc_*` → 警告
- **驗收**：`gen_dispatch.py` 產出的表涵蓋全部；PC sim 上跑得動。

### Phase 2 — 解除 Tier 0 的 enum 枷鎖（~2 週）✅ 2026-08-03 完成 —— **使用者提出的原始問題到此解決**
`skaiapp` 的 `bind` / `action` 從 enum 改為查派發表的字串鍵。
- 舊包相容：現有 9 個 bind enum 值映射到新鍵名（compat table），**已推送到使用者錶上的包不能壞**
- 手機端 schema 同步更新（`SkaiLink/bridge/skaiapp-package.schema.json`）
- **驗收**：新增一個能力（例如 `weather.temp`）只改派發表一列，AI 生成的包立刻能用。**這一步達成即已解決使用者提出的原始問題。**

### Phase 3 — 外部 JS runtime（~4–5.5 週）— **已定案在範圍內**

> ⚠️ **估時已上修。** 原估 3–4 週是在加入麥克風系統指示器（§2.6）與除錯管線 hook（§2.8）**之前**的數字。這兩項各約 0.5–1 週：指示器的視覺雖現成，但 capture refcount 要穿過所有音訊路徑、加上前景策略與 v2t 配額；除錯 hook 則要動日誌路由與 QuickJS 例外橋接。不把它們算進去會低估。

既然外部接口是程式語言，這階段是必要的，不是選配。**本階段的重點是沙箱，不是把 API 接通** —— 接通很快，沙箱做不好會讓使用者的錶被第三方 app 弄壞而且救不回來。

- 開 `PKG_USING_QUICKJS` + `QUICKJS_LVGL` + `QUICKJS_USING_PSRAM`（**JS heap 必須在 PSRAM**，HCPU 800 KB SRAM 絕對不夠）
- 先確認 `external/quickjs/lvgl/` binding 對得上本專案 LVGL 版本（SConscript 寫 v7/v8）
- **沙箱四件套**（對應 §2.3-B）：
  - 記憶體配額 → QuickJS 自訂 allocator
  - 執行時間 watchdog → `JS_SetInterruptHandler`
  - 權限閘 → 派發表查 manifest 宣告，未宣告直接拒絕
  - ✅ 安裝時驗簽 → SHA-256 + ECDSA P-256（mbedtls）+ TOFU keyid 比對（§2.5）—— 模擬器上 27 項 gate 全綠（`skai_pkg.c`，設計見 ADR-0019 §14）。TOFU 由 `/skaiapp/<keyid>/<app_id>/` 的路徑本身保證，不另存對照表。**BLE 兩段式傳輸尚未接**（簽名包是 manifest + payload 兩個 blob，現行 0x13/0x14 只傳一個，改協定要與手機端同 commit）
- **麥克風系統級指示器**（§2.6）—— 把現有 mic bar 升到 `lv_layer_sys()`，綁 capture refcount，加前景限定與 v2t 配額。**這是開放 T2-mic 的前置條件，不能晚於能力開放。**
- **除錯管線 hook**（§2.8）—— keyid 標記的 app 日誌 ring buffer、除錯 session 閘、QuickJS 例外轉結構化事件、配額違規事件、丟包計數。**hook 現在做，UI 排 Phase 5；retrofit 很痛。**
- 把 `skai_*` 由派發表**自動注入** JS global（不是手寫 binding，見 §2.1）
- ⚠️ `external/` 依 CLAUDE.md 屬 don't-touch，膠水碼寫在 `src/modules/sdk/` 而非改 vendor 檔
- **驗收（gate，非加分項）**：
  1. PC sim 上跑一支 JS app 讀心率 + 畫 arc + 震動
  2. 跑一支故意寫壞的 app（無限迴圈 / 狂配記憶體 / 呼叫未宣告能力），**錶必須活著**
  3. **任何路徑開麥克風，指示器都出現；且沒有任何 API 能隱藏它**
  4. 例外與配額違規都能在日誌流看到，含丟包計數

### Phase 4 — 事件服務 + 手機橋接（~2 週）
`skai_accel` / `skai_touch` / `skai_connection` / `skai_focus` 的 subscribe/unsubscribe，以及 `skai_msg` 接到現有 L1/L2 協定。

### Phase 5 — 開發者體驗（持續）
- **模擬器**：現有 PC sim 加 SDK app 載入路徑 → 這是最大槓桿，Pebble 生態就是靠 emulator 起來的
- **手機 App 除錯面板**（§2.8）：對單一 app 開啟／關閉除錯 session、即時日誌檢視、離線緩衝
- **電腦 App**：透過手機中繼（WebSocket / USB）接收日誌；**不做桌面直連 BLE**
- CLI 打包工具（產生金鑰對、簽章、打包）
- API 文件 + `.d.ts` 型別定義（都從標註 header 生成，格式對齊 Pebble docs）

### Phase 6 — BLE central（保留，非本輪）
四個前置條件見 §2.9：系統仲介裝置選擇器、手機連線優先權協定層強制、GATT UUID 白名單、射頻時間歸屬。**先決條件是 Phase 3 的沙箱驗收已通過** —— 沙箱都還沒證明有效之前，不該加一條逃出觀測範圍的通道。

### 明確不做（本輪）
- **Tier 2 原生 relocatable app（.elf/.skx）**：技術上可行 —— **兩核都有 MPU**（規格書 p4），隔離不是障礙。真正的成本在 ELF loader、ABI 完全凍結、以及**對外發佈與維護 toolchain**。QuickJS 已覆蓋 ~90% 需求，等生態有量再評估。
- **CompassService**：無磁力計。
- **相容 Pebble app 二進位**：不同螢幕、不同輸入、不同 ABI，不可行也不划算。對標的是 **API 設計哲學**，不是二進位相容。

---

## 4. 時程與中途檢查點

**已定案：一路做到 Phase 3**（完整外部生態）。

| 階段 | 估時 | 累計 |
|---|---|---|
| Phase 0 凍結外部邊界 + 2 個 de-risk 探針 | 1–1.5 週 | 1.5 週 |
| Phase 1 內部 C 能力層 | 2 週 | 3.5 週 |
| Phase 2 解除 enum 枷鎖 | 2 週 | 5.5 週 |
| Phase 3 外部 JS runtime + 沙箱 | 4–5.5 週 | **9–11 週** |

單人全職、含測試、不含美術資源。Phase 4（事件服務 + 手機橋接，~2 週）與 Phase 5（開發者體驗，持續）在此之後。

### 第 5.5 週的檢查點（Phase 2 結束）

不再是「要不要繼續投」的關卡，但仍是最有價值的量化時刻：

**新增一個對外能力的成本，從「改 4 個檔 + 發版」降到多少？**

這個數字直接預測 Phase 3 之後的維運成本。若 Phase 2 結束時它還沒明顯下降，代表派發表的自動投影沒真正做到位（§2.1），**那是進 Phase 3 之前要先修的**——因為 Phase 3 會把這個成本乘上外部開發者的數量。

⚠️ **外部 manifest schema 在 Phase 0 就要一次定齊** —— 簽章 / 公鑰 / keyid 主鍵 / 權限 / 版本協商 / 撤銷清單六個欄位。這是全案唯一「發佈後回不去」的東西。

---

## 5. 風險與未決問題

| 項目 | 說明 |
|---|---|
| 🔴 **`main` 分割區塞不下 QuickJS** | **Phase 0 實測**：QuickJS + LVGL binding 需 312.7 KB code+RO，2.5 MB `main` 只剩 264 KB → link 差 **95 KB**。**這才是最可能讓 Phase 3 失敗的因素**（原本以為是 SRAM）。解法動 `ptab.json` + bootloader，**不能走 OTA**，要早於 Phase 3 決定。 |
| ~~**HCPU SRAM 吃緊**~~ | **Phase 0 實測排除**：QuickJS 靜態 SRAM 只吃 0.8 KB，heap 有 357 KB。JS heap 在 PSRAM 的紀律不變，但這不再是 gating 風險。 |
| ~~**LVGL 版本對齊**~~ | **Phase 0 實測排除**：專案 v8.3.1，binding 是 v8，缺的符號全在已 link 的 `middleware/lvgl/lvsf/`。不用 shim。 |
| **`external/` 不可動** | QuickJS 啟用不能改 vendor 檔，膠水碼另放。 |
| **Thread 契約** | LVGL thread / engine thread / BLE parse thread 三者混用是現有慣性，SDK 必須把規則寫死並在 debug build 加 assert。 |
| **手機端同步** | Tier 0 改造牽動 `SkaiLink` schema 與 AI 生成 prompt，需同 sprint 排。 |
| **外部 schema 一次定生死** | 簽章 / 公鑰 / keyid 主鍵 / 權限 / 版本協商 / 撤銷清單六個欄位必須在 Phase 0 一次定齊（§2.3-B、§2.5）。這是全案唯一「做錯就回不去」的地方 —— 因為外部開發者不能刷機，格式一旦發佈就永久背著。 |
| **自簽模式下沒有第二道防線** | 選了 (b) 之後，權限模型 + 沙箱是**僅存的**安全邊界（§2.5）。Phase 3 的沙箱驗收（惡意 app 測試）不是加分項，是 gate —— **不過就不能對外開放**。 |
| **沒有撤銷手段 = 出事無法止血** | 自簽模式下任何人可發佈，blocklist 格式必須在 Phase 0 進 schema。來源可以晚點決定，格式不行。 |
| **麥克風指示器是承重牆** | 開放 mic/v2t 給自簽 app 之後，指示器是使用者**唯一**能察覺被錄音的方式。任何讓它失效的路徑（app 蓋得掉、背景擷取、綁 API 而非綁硬體狀態）都會讓整個開放決策站不住。畫在 `lv_layer_sys()` + 綁 capture refcount 是不可妥協的兩點。 |
| **除錯管線在 release 開太大** | 外部除錯必須在 release 韌體可用，但**不能因此把系統 ulog 打開** —— 系統日誌含通知內容與健康數值。只走 app 範圍日誌 + 使用者明示開啟 + 自動逾期（§2.8）。這條做鬆了是隱私事故。 |
| **兩套 API 分岔** | 若外部腳本 API 變成手寫的第二套，必然落後於內部 C 層。**必須靠 `gen_dispatch.py` 自動投影**（§2.1），不是靠紀律。 |
| **內部繞過 `skai_api`** | 工程師走捷徑直接呼叫 `bloc_*` → C 層變裝飾品。用 build 警告擋（§2.1），不要只寫在文件裡。 |

> §1 硬體參數已於 2026-08-03 依 `DS5601-SF32LB56x-CN V1.9.2` 規格書與 BOM 核實，無待確認項。

---

## 6. 決策記錄

### 已定案

| 決策 | 內容 | 影響 |
|---|---|---|
| **雙層 SDK** | 內部 = C；外部 = 小程式腳本語言。分界理由：**內部能刷機，外部只能裝 app** | 凍結負擔全部落在外部側；內部 C 層是 API 非 ABI，可自由重構 → Phase 1 省約 1 週 |
| **開放對象含外部開發者** | 是 | 簽章、權限、配額、版本協商全部必要，且**必須在 Phase 0 進 schema** |
| **外部語言 = JS (QuickJS)** | §2.6 | Phase 3 確定在範圍內；`.d.ts` 型別定義免費附贈 |
| **宣告式 JSON 保留** | 不廢除 | 舊包相容 + AI 快速生成路徑；與 JS 同源投影 |
| **Tier 2 原生 app 不做** | 本輪 | 阻力是 toolchain 發佈與維護，非技術隔離（兩核都有 MPU） |
| **簽章 = 開發者自簽 (b)** | 使用者自負風險，無官方審核 | 簽章降級為「完整性 + 作者連續性 + 可歸因」；**權限模型與沙箱成為唯一安全邊界**；新增 TOFU、keyid 主鍵、撤銷清單、安裝同意 UI 四個工作項（§2.5） |

| **麥克風 / v2t 對外開放** | 開放，但綁系統級使用中指示器 | 指示器必須畫在 `lv_layer_sys()`、綁 capture refcount、無 API 可隱藏；自簽 app 麥克風**限前景**；v2t 需用量配額（§2.6） |
| **除錯管線** | 手錶 → 手機 App → 電腦 App；**電腦不直連 BLE** | app 範圍日誌（非系統日誌）、release 韌體即可用、需使用者開啟且自動逾期；hook 排 Phase 3、UI 排 Phase 5（§2.8） |

| **系統設定寫入 / AI 模型載入** | **永不對自簽 app 開放** | T3，不再重議 |
| **BLE central** | 本輪不開，排 Phase 6 | 需先完成四個前置條件，且**必須在 Phase 3 沙箱驗收通過之後**（§2.9） |

| **時程：一路做到 Phase 3** | 不設中途投資關卡 | ~9–11 週；Phase 0 的兩個 de-risk 探針因此成為必要前置（§3 Phase 0） |

### 仍待決定

1. **撤銷清單誰維護**：手機端下推 keyid blocklist 的來源是什麼？（Skaiwalk 後台名單 / 使用者自行封鎖 / 兩者）**格式要在 Phase 0 定，但來源可以晚點決定。**
2. **背景麥克風是否真的全禁**：§2.6 建議自簽 app 一律前景限定。若有正當的背景錄音情境（例如睡眠鼾聲偵測類的第三方 app），需另設機制（如 AOD 常駐指示器），成本明顯較高 —— 現在說比之後補便宜。
