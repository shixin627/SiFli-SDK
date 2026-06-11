# 0014 — 夜間配戴診斷 BLE 上報(KEY_WEAR_DIAG 0x12)

日期:2026-06-11
狀態:Accepted
前置:ADR 0012(滑動窗投票)、ADR 0013(接觸中斷重確認)
相關檔案:`src/modules/algorithm/WearDetect/wear_detect.c`、
`src/modules/communicate/*`、`src/modules/service/watch_*`、
`src/modules/client/watch_system_client.c`、
SkaiLink `lib/shared/watch/`(同步 commit)

## 問題

ADR 0012/0013 修復後,睡眠機(日常配戴那隻)整夜實測(2026-06-10→11)
仍出現 00:30~04:00 約 3.5 小時「未配戴」整段掉線 — 深睡殘餘風險實證
發作。要對症調參需要夜間 DC/PI 實測分布,但:

1. 睡眠機**沒有任何串口**,永遠不能接線看 log,韌體只能 OTA。
2. PPG 感測器 per-unit 差異大(`wear_detect.c` 自註 narrow margin /
   needs per-unit calib),bench 機白天量的數據不能套用。
3. 沒有數據,調 `PPG_DC_LOW_THD` / `PI_THD_TO_ON` 都是盲調。

## 決策

新增 watch→phone 單向診斷上報,讓無線材的錶把 wear_detect 內部狀態
送到手機存成每日 CSV:

- **Wire**:HEALTH group(0x05)新 key `KEY_WEAR_DIAG = 0x12`,
  每筆 14 bytes LE:`{ts:u32, evt:u8, status:u8, dc/4:u16, PI×1e6:u16,
  PIrange×1e6:u16, IMUvar×1e4:u16}`(clamp 至 u16 上限)。
- **事件碼(凍結,勿重排)**:1=轉ON、2=轉OFF、3=評估快照(60 秒節流)、
  4=probe 開、5=probe 過期、6=接觸中斷 arm、7=中斷後重確認成功、
  8=重確認逾時。
- **鏈路**(完全照 0x11 KEY_HEART_CURVE_SKIP 模板):LCPU `wear_detect.c`
  `diag_emit()` → `watch_sys_sync.notify_wear_diag` →
  `push_msg_to_hcpu(MSG_SERVICE_WEAR_DIAG_IND)`(data_service)→ HCPU
  `watch_system_client.c` case → `commu_send_wear_diag` → BLE。
- **手機端**(SkaiLink 同步 commit):HealthKey `keyWearDiag(0x12)` →
  `_handleHealthData` 解析 → foreground isolate 直接 append
  `docs/Skaiwalk/wear_diag_YYYYMMDD.csv`(整夜存活的 isolate,不依賴
  主 isolate);開發者頁加「分享診斷 CSV」鈕走 SharePlus。

## 取捨

- **Live push only,不做 store-and-forward**:夜間手機在床頭、BLE 整夜
  連著(2026-06-10 整夜 skip 標註不斷線可證);斷線掉幾筆診斷可接受。
  若實測常掉,再仿 `health_store_hr_skip_async` 補持久化。
- **手機端用 CSV 不進 drift DB**:避免 schema bump + codegen;診斷數據
  是給工程師離線分析的,不是產品資料。
- **量級**:~1 筆/分 + 事件,整夜 ≈ 500 筆 ≈ 7KB BLE;LCPU 端零緩衝
  (即時推,照小 struct 慣例)。

## 使用方式

1. OTA 新韌體到睡眠機,app 更新到對應版。
2. 正常戴著睡(不用接線)。
3. 隔天:設定頁 7 連點解鎖開發者頁 → 配戴診斷 → 分享診斷 CSV →
   傳給工程師。
4. 分析後對症調 per-unit 門檻(DC/PI/重確認參數)。
