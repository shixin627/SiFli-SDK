# Dualcore Watch Hardware

`sf32lb56w-watch` 板的硬體驅動規格 — 主晶片、顯示、感測、電源、記憶體、
匯流排。Ground truth 來源:`customer/boards/sf32lb56w-watch/`(board.conf /
ptab.json)、`project/hcpu/build_sf32lb56w-watch_hcpu/rtconfig.h`、各
`customer/peripherals/*` driver、`src/lcpu/main.c`。

> 板型切換見 [`CLAUDE.md` § Platform](../CLAUDE.md)。HCPU/LCPU 職責分工見
> [`architecture.md`](architecture.md)。

---

## 快速總表(BOM-style)

| 子系統 | 元件 | 介面 | 驅動 / 設定 |
|---|---|---|---|
| SoC | SF32LB56W(SF32LB56X) | — | dualcore HCPU + LCPU,RT-Thread |
| Display | DO0143FMST08 panel / **CO5300** IC | QAD-SPI | `peripherals/co5300/co5300.c` |
| Touch | TT151AMC60C panel / **CST816** IC | I2C1 @ 0x15 | `peripherals/cst816/cst816.c` |
| IMU | **BMI270**(Bosch) | I2C1, INT1=GPIO122 | `peripherals/sensor/BMI270-Sensor-API/` |
| PPG / HR | **GH3018**(Goodix) | I2C1 | `peripherals/sensor/gh3018/` |
| 馬達 | 震動馬達 | PWM4 | LCPU `bloc_peripheral` |
| 按鍵 | KEY1 | GPIO128 active-high | button lib(雙核) |
| LED | LED1 | GPIO41 | board.conf |
| 電池 | VBAT 量測 | ADC1 | LCPU `charge.c` + `bloc_battery` |
| 音訊 | SF32LB56W 內部 codec | AUDPRC + DMA | `BSP_ENABLE_AUD_CODEC` |
| PSRAM | MPI2 / QSPI2 | base 0x60800000 | `BSP_USING_PSRAM2` |
| NAND | SPI NAND(W25N01G 級) | MPI3 / QSPI3, base 0x64000000 | `BSP_USING_SPI_NAND` |
| BLE | HCPU host + LCPU controller | — | ANCS + 私有 L1/L2 協定 |

---

## SoC / 平台

| 項目 | 規格 |
|---|---|
| 主晶片 | **SF32LB56W**(SF32LB56X 家族) |
| 雙核分工 | **HCPU** = LVGL GUI + BLE host + AI 推論;**LCPU** = sensor / motor / charger + BLE controller |
| 圖形加速 | **EPIC** 2.5D accelerator(`BSP_USING_EPIC`) |
| 看門狗 | WDT,10s timeout(`CONFIG_WDT_TIMEOUT=10`) |

---

## 顯示(Display)

| 項目 | 規格 |
|---|---|
| 面板 | **DO0143FMST08**(1.43" 圓形 AMOLED 等級) |
| 驅動 IC | **CO5300** |
| 解析度 | **466 × 466**(`LCD_HOR/VER_RES_MAX=466`、`CO5300_LCD_PIXEL_*=466`) |
| 色彩 | RGB565（16-bit） |
| 介面 | **QAD-SPI**（Quad-SPI，`LCDC_INTF_SPI_DCX_4DATA`） |
| VSYNC | 停用（`# CONFIG_LCD_CO5300_VSYNC_ENABLE is not set`） |

設定:`board.conf` → `CONFIG_LCD_USING_DO0143FMST08_LCD_USING_TT151AMC60C=y`。
驅動:`customer/peripherals/co5300/co5300.c`。

---

## 觸控(Touch）

| 項目 | 規格 |
|---|---|
| 面板字串 | TT151AMC60C |
| 控制 IC | **CST816**（Hynitron 矽創） |
| 介面 | **I2C1**，7-bit 位址 **0x15** |

設定:`CONFIG_BSP_USING_TOUCHD=y`、`CONFIG_BSP_USING_TOUCH_STATE_MANAGER=y`。
驅動:`customer/peripherals/cst816/cst816.c`。

---

## 慣性感測 IMU

| 項目 | 規格 |
|---|---|
| 型號 | **BMI270**（Bosch），accel + gyro |
| 介面 | I2C1 |
| 中斷 | **INT1 = PB26 = GPIO 122**（`src/lcpu/main.c` 確認） |
| 特性 | HW step counter、HW wrist-wake（`BMI2_WRIST_WEAR_WAKE_UP`） |

韌體預設用**硬體抬腕**（`USE_BMI270_HW_WRIST_WAKE=1`,`src/lcpu/main.c`):
晶片內部偵測抬腕後才觸發 INT1,軟體狀態機關閉(無 back-gesture / 手腕旋轉);
另有 SW 模式(gyro_x 閾值)可切換重編 LCPU。step counter 由 LCPU 每 3s
輪詢(`bmi270_hw_step_counter_read`)。驅動:`customer/peripherals/sensor/BMI270-Sensor-API/`。

---

## 心率 / PPG

| 項目 | 規格 |
|---|---|
| 型號 | **GH3018**（Goodix 匯頂） |
| 類型 | 光學 PPG（HR / SpO2-capable via `gh30x_algo`） |
| 介面 | I2C1,電源腳 `GH3018_POW_PIN` |

驅動:`customer/peripherals/sensor/gh3018/`。

---

## 馬達 / 按鍵 / LED

| 項目 | 規格 |
|---|---|
| 震動馬達 | **PWM4** 驅動(LCPU `bloc_peripheral` provider → `watch_system_interact` 的 `motor_pattern_*`) |
| 側鍵 KEY1 | **GPIO 128**,active-high(HCPU + LCPU,button lib) |
| LED1 | **GPIO 41** |

設定:`CONFIG_BSP_USING_PWM4`、`CONFIG_BSP_USING_KEY1` + `CONFIG_BSP_KEY1_PIN=128`、
`CONFIG_BSP_USING_LED1` + `CONFIG_BSP_LED1_PIN=41`。

---

## 電源 / 電池

| 項目 | 規格 |
|---|---|
| 電壓量測 | **VBAT via ADC1**（`CONFIG_BSP_USING_VBAT` + `CONFIG_BSP_USING_ADC1`） |
| 充電 | LCPU `charge.c` + `bloc_battery`;charge-status / voltage 事件回報 HCPU |
| HCPU 讀電池 | `SkaiWatchSys.battery_level_value` / `charger_status`（**非** `battery_get_charge_state()`,bloc_battery 沒 link 進 HCPU） |
| PMIC | **停用**（board.conf 註解 `# CONFIG_PMIC_CTRL_ENABLE`） |

無獨立 charger / fuel-gauge IC driver,以 VBAT ADC + charge-detect 為主。

---

## 音訊

| 項目 | 規格 |
|---|---|
| Codec | **SF32LB56W 內部 codec**（`AUDIO_MIC_USING_CODEC` + `AUDIO_SPEAKER_USING_CODEC` + `BSP_ENABLE_AUD_CODEC`） |
| 處理鏈 | AUDPRC（TX0 / RX0 DMA）+ `audio_processor`（`BSP_USING_AUDIO_PROCESSOR`） |
| 藍牙音訊 | HCI path（`AUDIO_PATH_USING_HCI`，HFP-HF） |
| Codec 套件 | libhelix（MP3）、Opus、WebRTC VAD |
| 外部 PA | **AW8155 停用**（board.conf 註解 `# CONFIG_PA_USING_AW8155`） |

> 註:SDK 內含 `peripherals/audio/da7212/` driver,但本板**未啟用** DA7212,
> 走 SF32LB56W 片上 codec。

---

## 記憶體

| 項目 | 規格 |
|---|---|
| PSRAM | **MPI2 / QSPI2**（MODE 3），`BSP_USING_PSRAM2`，base **0x60800000** |
| └ HCPU 分割 | main exec 2.5MB（`0x280000`）+ PSRAM_DATA 5.5MB（`0x580000`）= 8MB |
| NAND Flash | **MPI3 / QSPI3**（MODE 1），**SPI NAND**（`BSP_USING_SPI_NAND` / `BSP_USING_NAND_FLASH3`），base **0x64000000**，**128MB**（W25N01G 級） |
| └ Flash 分割 | code 2.5MB / **FS_REGION ~87.5MB**（DFS ELMFAT）/ KVDB dfu·ble·prefdb（各 16KB）/ **BLE OTA 16MB** |

> 命名遺留:FAL 分割表（`custom_mem_map.h`）巨集沿用 `NOR_FLASH3_DEV_NAME`,
> 但 flash3 實體是 **SPI NAND**(`rtconfig.h` 為 `BSP_USING_SPI_NAND=1`)。
> 分割定義見 `customer/boards/sf32lb56w-watch/ptab.json`。

---

## 匯流排 / 計時 / 無線

| 項目 | 規格 |
|---|---|
| I2C1 | 感測器(BMI270 / GH3018)+ 觸控(CST816)共用匯流排 |
| UART1（DMA RX） | HCPU log / console — COM14 @ 1M baud |
| UART4（DMA RX） | LCPU console |
| Timers | GPTIM4 / BTIM1 / LPTIM1（HCPU）、LPTIM2（LCPU） |
| RTC | on-chip RTC + alarm（`RT_USING_RTC` + `RT_USING_ALARM`） |
| ADC1 | VBAT 電壓 |
| BLE | HCPU = host（GAP/GATT + ANCS + 私有 L1/L2 協定 + HID + DFU/OTA）；LCPU = controller |

> Port 對應(`CLAUDE.md`):COM14 = CH342 ch A = uart1 = firmware log;
> COM13 = CH342 ch B = boot ROM download(`uart_download.bat`)。

---

## ⚠️ 待確認

下列項目尚未鎖定確切值,**未填猜測值**:

- **GH3018 / BMI270 的 I2C 7-bit 位址**:driver 透過 `gh3018_get_dev_addr()` /
  BMI2 API 動態取得,板級 `#define` 未鎖定。BMI270 業界標準為 0x68 / 0x69
  (SDO 選擇),本板實際值需從 sensor 初始化路徑確認。
- **`GH3018_POW_PIN`、CST816 的 RST / INT GPIO**:定義在 driver / board.h 層,未鎖定。
- **PSRAM 實體總容量**:`CONFIG_BSP_QSPI2_MEM_SIZE=16`,分割表只用到 base
  0x60800000 之後 8MB;`16` 的單位(MB / Mbit)與實體顆粒容量待確認。
