# 0017 — 睡眠感知背景心率採樣:靜止即密採(rest-candidate),密度不再跟隨睡眠判定

日期:2026-07-23
狀態:Accepted
相關檔案:`src/modules/service/sleep_service.c`、`src/modules/service/hr_service.c`、`src/modules/service/hr_service.h`

## 問題

使用者實際睡眠約 01:30–09:00,手機睡眠卡只記到 ~1.5h;同夜心率曲線在睡眠時段
呈 50↔122 鋸齒、鋪滿「省電間隔」灰帶。7/17、7/18、7/22 同樣漏判,7/19、7/21
正常 — **間歇性**,同一隻錶同一套演算法。

Code 對讀(sleep_fusion.c × hr_service.c)定位出四常數的算術鎖死:

- 清醒節流:每 15 min 才 burst 一次(`BG_HR_AWAKE_SKIP=5` × 3 min tick)
- HR wake-veto hold 12 min(`SF_WAKE_HR_HOLD_MIN`,特意設計來跨 HR 缺口橋接
  「火車/桌前靜坐」情境)
- `hr_elev_consec` 跨 HR 缺口**不歸零**(sleep_fusion.c:588-591)
- 入睡遲滯 3 min / 出睡 2 min(`SF_ENTER_SLEEP_MIN` / `SF_EXIT_SLEEP_MIN`)

夜間讀數若持續虛高(80–119 bpm — 錶帶鬆/光學貼合差,低於 120 artefact 天花板
`SF_HR_ARTEFACT_MAX` 所以不被丟棄),每個 15 min 循環變成:讀數分鐘投醒 + hold
蓋掉後續 11 分鐘 → 只剩 3 分鐘乾淨窗,恰好翻入睡即被下一筆虛高讀數踢出(出睡
只要 2 分鐘)→ **每循環僅記 ~2 min 睡眠,整夜 60–90 min**,與觀測吻合。而判定
「清醒」又反過來維持稀疏採樣 → 自我維持的死結。分岔點在入夜頭幾筆讀數品質,
解釋了間歇性(頭幾筆乾淨 → veto 從未武裝 → 正常入睡進密採 → 整夜穩定)。

## 決策

**採樣密度政策與睡眠判定解耦** — 密度改由便宜的直接證據(accel 靜止)決定:

- sleep_service 新增 rest-candidate:夜窗(21:00–10:59 本地牆鐘)內
  Cole-Kripke score < 400 連續 10 min → `hr_service_set_sleep_active(asleep
  || rest_candidate)` 進密採(60 s burst / 3 min);score ≥ 400 連續 3 min、
  離窗、或斷戴即退出。常數 `SLEEP_REST_*`。
- **sleep_fusion.c 零改動**:veto 架構本有「一筆乾淨低讀數立即清 hold+run」的
  自癒性(sleep_fusion.c:613-619)。密度恢復(3 min/筆)後,壞讀數活不過下一
  筆乾淨讀數;火車/桌前防線反而因證據更密而更強。
- 常駐法醫 log:`[SLPDIAG]` 每分鐘一行(LCPU console),
  act/steps/hr/std/score/stage/veto/rhr/baseline/rest/total —
  未來任何漏判夜不需重刷診斷版即可定罪。
- 電池:判對的夜晚**本來就**整晚密採(33% PPG duty vs 節流 4.4%),新增成本
  僅(a)誤判夜(現在的省電是 bug 副產物)與(b)夜窗內醒著但靜止的時段。

## 捨棄的替代方案

- **veto 參數微調**(縮 hold / consec 跨缺口歸零 / 門檻 floor):單獨做會重開
  火車/桌前漏洞,且不解決稀疏數據本質。Phase 2 備案,僅在 Phase 1 後仍有漏判
  夜時依 SLPDIAG 定罪後動。
- **HR 下降正向助判入睡**(2-of-3 投票):演算法行為大改,需跑 Walch 2019 PSG
  回歸;Phase 1 不必要。
- **vendor confidence 品質閘**:ADR-0016 已定案 valid_level 恆 0,不重議。
- **「躺平」姿態條件**:睡姿手腕朝向高度可變,重力向量在腕上是弱證據,徒增
  tuning 面;靜止 + 夜窗已足。

## 後果

- 夜窗外午睡仍走稀疏採樣,鎖死風險殘留(known gap;依真機 SLPDIAG 決定是否
  擴窗或全天化)。
- 手機端不需任何改動:密採後省電灰帶自然消失、心率曲線自然變密變平滑。
- LCPU uart 未接線的 dock 讀不到 SLPDIAG(2026-07-23 本機 COM11=刷/COM12=HCPU
  log,COM7 非 LCPU);HCPU COM 仍可見 stage 轉換(`[Sleep] mode=`)與
  `store_sleep_data`,手機睡眠卡為主要 E2E 證據。
- 驗證:2026-07-23 build 綠 + 刷機成功,開機後 63 s 首次 minute-eval 正常
  (COM12 `[Sleep] mode=3 … rhr=65`);整夜 E2E 待當晚實際配戴。
