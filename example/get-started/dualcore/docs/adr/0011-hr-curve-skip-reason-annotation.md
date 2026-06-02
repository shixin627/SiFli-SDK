# ADR-0011: Annotate heart-rate-curve gaps with the reason HR was not recorded

- agent: Claude (Opus 4.8)
- model: claude-opus-4-8
- trigger: founder report 2026-06-02 — 看睡眠時段的心率圖「為什麼這麼斷」。
  追到背景採樣器每個 tick 會因多種原因放棄產生一個 HR 點,但「為什麼沒記錄」
  這個原因目前完全沒有上報，只能在接線 console 推測。手錶**不能接線看 log**，
  資料只能在手機 App 看 → 原因必須透過 BLE 上報並畫在心率圖上。
- status: **Proposed** — 協定契約待 founder 確認；階段 0(韌體 instrumentation)
  已落地未驗證，階段 A(韌體上報)/ 階段 B(手機顯示)待實作。
- scope: 跨兩 repo。watch firmware(SiFli SF32LB56W）+ phone（SkaiLink Flutter）。
  新增一個**單向（watch→phone）** L2 notify key。
- relates: 沿用 Skaiwalk 共用 ADR 編號（SkaiLink 0005–0009，watch 0010）。
  時間戳慣例延續既有「手錶本地牆鐘偽裝 UTC + 手機 `watchEpochToLocal` 重解讀」
  約定（與 HR-curve 0x10、心率/睡眠時間戳一致）。

## Context

心率圖（`activity_history_page.dart` 的 `_HrCurveChart`）上的點來自背景採樣器
`hr_service.c` 的 `bg_hr_*`，經 `notify_hr_sample → 0x10 (keyHeartCurveSample)`
上報。採樣是**間歇**的（醒著 ~15 min / 睡著 ~3 min 一個 burst），且每個 tick 會
因下列原因放棄產生一個點：未配戴、PPG 鎖不到、充電中、感測器未就緒等。圖把離散點
連線，缺口超過 30 min 就斷線（`_gapBreakMinutes`），於是睡眠段看起來「很斷」。

「斷在哪」手機已經知道（就是缺口）；「**為什麼斷**」只有韌體那層知道，且目前不上報。

## Decisions

### D1 — 在韌體量化每個 tick 的放棄原因（ACCEPTED，階段 0，已落地未驗證）

`hr_service.c` 的 `bg_hr_*` 路徑加 skip-reason 分類與計數（`bg_hr_note()` +
`BGHR_*` enum + `bg_hr_diag` MSH 命令）。這是純 instrumentation，不改採樣控制流。
*Caveat:* `bg_hr_diag` 走 LCPU console，**手錶不能接線**故只供 bench；正式輸出走 D2/D3。

### D2 — 新增 health-group key `0x11` KEY_HEART_CURVE_SKIP，payload 對稱 0x10（PROPOSED）

在 health command group 緊接 `KEY_HEART_CURVE_SAMPLE = 0x10` 新增
`KEY_HEART_CURVE_SKIP = 0x11`（`communicate_parse_health.h`）。**不用** NotifyKey 的
`0x40`：實作時發現韌體 `commu_send_heart_rate_series` 已佔用 `0x40`
（`KEY_HEART_RATE_SENSOR_SAMPLE`，送 PPG raw series），手機調查誤報為空。`0x11` 與
`0x10` 同 group、同持久化路徑、手機端同處解析，更乾淨。

**Wire 格式（5 bytes，little-endian，與 0x10 對稱）:**

| offset | type      | 欄位        | 說明 |
|--------|-----------|-------------|------|
| 0      | uint32 LE | `ts`        | 5-min bucket 起始 epoch。**手錶本地牆鐘當 UTC**（與 0x10 同），手機用 `watchEpochToLocal` 重解讀對齊 X 軸 |
| 4      | uint8     | `reason`    | 主導原因碼（下表） |

**Reason 碼（穩定 wire 契約，與韌體內部 `BGHR_*` enum 順序解耦，不可重排）:**

每碼一個顏色（founder 指定「不同原因不同顏色」），實際色票階段 B 依 Skaiwalk_UI token：

| 碼 | 名稱          | 中文標籤   | 顏色語意 (Skaiwalk_UI) | 對應內部 BGHR_* |
|----|---------------|-----------|------------------------|-----------------|
| 0  | `OK`          | （不送）  | — 畫心率線本身          | BGHR_OK（保留，永不上報） |
| 1  | `NOT_WORN`    | 未配戴    | 中性灰 neutral          | BGHR_NOT_WORN |
| 2  | `NO_LOCK`     | 訊號不穩  | 琥珀 warning            | BGHR_NO_LOCK（含演算法暖機未鎖、訊號品質不足） |
| 3  | `CHARGING`    | 充電中    | 綠 charge               | BGHR_CHARGING |
| 4  | `NOT_READY`   | 暖機中    | 藍 info                 | BGHR_NOT_READY（含感測器未初始化／已關閉） |
| 5  | `SENSOR_FAULT`| 感測器異常 | 紅 danger               | BGHR_SENSOR_FAULT（整 burst 每次 read 都失敗＝I2C/HW） |
| 6  | `POWER_SAVE`  | 省電間隔  | 藍灰 muted（最淡）      | BGHR_THROTTLE（白天正常 ~15 min 節流，非故障） |
| 7  | `OTHER`       | 其他      | 深灰                    | BGHR_BUSY / NO_TIMER / FWD_ZERO |

### D3 — bucket 語意：只在「整段無 HR 點」時送（PROPOSED）

韌體以**對齊牆鐘的 5 分鐘 bucket**累積。每次 `bg_hr_period_cb`（base tick 3 min）
偵測 `floor(now/300)` 是否翻新；翻新時 flush 上一個 bucket：

- bucket 內**曾成功送出任一 HR 點**（`notify_hr_sample` 成功）→ **不送 skip**
  （圖上已有點，無缺口可標）。
- bucket 內**完全沒有 HR 點** → 送一筆 `0x11 (bucket_start_ts, dominant_reason)`，
  `dominant_reason` = 該 bucket 內計數最高的非 OK 原因。

這把資料量壓到「每個空白 bucket 一筆 5 bytes」，語意精準等於圖上一段空白＋一個原因。
上報節奏 ≈ 每 5 分鐘（founder 選定），無新 timer（搭現有 3 min tick 偵測 bucket 翻新；
最差延遲一個 tick flush，可接受）。

### D4 — 手機端：心率圖空白疊原因標註（PROPOSED）

0x11 → `HrSkipEvent` model → Drift 新表 `HrSkipEvents(dayEpoch, bucketTs, reasonId)`
→ `ExerciseHistoryService` → `_HrCurveChart` 用 `_minutesOfDay(bucketTs)` 對齊，
在斷線缺口以淡色塊 + 中文標籤呈現（fl_chart `extraLinesData` 或 overlay painter）。
標籤文案走 i18n（繁中台灣用語）；不可 strcmp 寫死。

### D5（修正 2026-06-02）— skip event 走既有 store-and-forward，與 HR 點對稱（PROPOSED）

**先前「韌體不做離線緩存」的假設錯誤。** 實際上 HR 點已有成熟 store-and-forward：
`bloc_health` 即時送 `0x10` 之外，同步寫 `/health/hr_YYYYMMDD.json`（`{samples:[{ts,bpm}]}`，
append），離線也存，重連後經檔案同步 idempotent 補送（見 `watch_system_client.c:309`、
`bloc_health.h`）。

睡眠（本功能最重要情境）整夜常離線，所以 skip event **必須對稱持久化**，否則重連後
HR 點補上、skip 原因丟失 → 空白處沒標註。設計：

- **持久化（可靠路徑）**：bucket flush 時若無 HR 點，把原因寫進 `/health/hr_*.json` —
  最自然是同一串 samples 用 `{ts, bpm:0, reason:N}`（`bpm>0`＝心率點，`bpm==0 && reason`＝
  該時刻沒量到＋原因）。複用既有 hr writer，手機檔案同步解析時順帶讀 `reason`。
- **即時（加速路徑）**：連線時另送 `0x11 (ts, reason)`，讓當下就看得到，不必等檔案同步。
- **「未連線」殘餘情境**：只剩錶關機／沒電／存檔失敗才真正整段無資料；手機端可選擇性
  依自身連線歷史補標「未連線」（斜線紋），優先級低。

### Failure-mode 普查（2026-06-02）

全韌體掃過 PPG 驅動 / hr_service 全路徑 / wear_detect / 電源管理 / 睡眠 / BLE 離線 /
WDT 七面向，結論：
- **採納為新 wire 碼**：`SENSOR_FAULT`（整 burst 每次 read 都失敗＝I2C/HW，已落地階段 0）。
- **併入既有碼**：演算法暖機未鎖→`NO_LOCK`；device closed→`NOT_READY`（`close_hr_service`
  會清 `is_ready`）；sample timer 建立失敗→`OTHER`。
- **改走持久化**：BLE 離線→既有 store-and-forward 補送（修正後 D5）；僅關機／沒電／
  存檔失敗才真空白，手機端可選擇性標「未連線」。
- **THROTTLE 獨立**：白天省電節流獨立為 `POWER_SAVE`（自己的顏色），不混入故障類。

## Open questions（待 founder 確認）

1. ~~Reason 碼表 / THROTTLE 是否獨立~~ → **已決議 (2026-06-02)**：原因經普查補齊為
   7 碼，`THROTTLE` 獨立成 `POWER_SAVE`，每碼一色（D2 表）。
2. **視覺**：缺口要「色塊橫跨整段 + 標籤」還是「缺口中央一個小 chip」？（階段 B 走 Skaiwalk_UI 定）
3. **保留期**：skip events 跟 HR curve 同樣保留 N 天即可？

## Files（實作清單）

**韌體階段 A:** `hr_service.c`（bucket flush）· `watch_sys_service.h`（MSG id +
`watch_sys_hr_skip_t`）· `watch_system_service.c`（`notify_hr_skip`）·
`watch_system_client.c`（HCPU 轉發）· `communicate_task.c` + `communicate_parse_health.h`（送 0x11）。

**手機階段 B:** `communicate_protocol.dart`（0x11 解析）· `models/hr_skip_event.dart`（新）·
`watch_protocol_service.dart`（callback）· `watch_foreground_service.dart`（轉發）·
`bluetooth_connection_provider.dart`（case）· `app_database.dart`（新表）·
`exercise_history_service.dart`（存取）· `activity_history_page.dart`（標註）。
