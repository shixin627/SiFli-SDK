# 0016 — HR 品質閘停用:Goodix 信心欄位實測恆為 0

日期:2026-06-16
狀態:Accepted(品質閘停用;真修法待定)
相關檔案:`src/modules/service/hr_service.c`、
`customer/peripherals/sensor/gh30x_algo_demo/call/src/gh30x_demo_algo_call_hr.c`

## 背景

睡眠心率曲線呈 50↔130 鋸齒(實測睡眠段 PI 55% 飽和、僅 35% 可用
訊號)。為仿 Apple「低信心不出值」,在 bg_hr 加品質閘
(`BGHR_MIN_QLEVEL`,commit ab0ad0507),用 Goodix 每拍 `valid_level`
(0/1/2)當門檻,並把每 burst 的品質統計經 wear-diag 上報手機 CSV
(evt=9,commit 212abc6a3)以便從實機數據調門檻。

## 觀測(決定性)

睡眠機戴一夜(2026-06-15→16)的 CSV,**234 個 bg_hr burst**:

- `valid_level` = 0、`valid_score` = 0,**零變異**(234/234)
- 門檻 `BGHR_MIN_QLEVEL = 1`(丟 level 0)→ 每個 burst acc = 0(全丟)
- 後果:整夜 HR 全被擋 → 手機顯示整段「訊號不穩」、睡眠分期因無
  HR window 幾乎掛掉(睡眠僅 0.1 小時)

**這是 ad-hoc 引入的迴歸。** 已將 `BGHR_MIN_QLEVEL` 改 0(停用閘,
全收)止血。

## 根因

品質**有接線**但欄位是死的:
`gh30x_demo_algo_call_hr.c:150` 呼叫
`gh3018_set_hr_quality(stResult.valid_score, stResult.valid_level)`,
而 `goodix_hba_update`(閉源 .lib)對本訊號把這兩個欄位填 0 ——
即使 `hba_out_flag == 1`(有鎖到 BPM)。同一結構另有 `hba_confi`
(置信度)、`hba_snr`(信噪比)兩個欄位**整合層沒讀**,疑為這版
algo 實際填的信心來源。

## 決策

1. **品質閘停用**(`BGHR_MIN_QLEVEL = 0`),恢復 HR/睡眠。
2. 真修法**待量測再定**,不再盲改(本次教訓):
   - **路徑 A**:bench 機(有串口)加暫時 log 同時印
     `valid_score / valid_level / hba_confi / hba_snr`,確認哪個欄位
     有真實數據;若 `hba_confi` 有料 → 把閘改讀它(改
     gh30x_demo_algo_call_hr.c 一行 + hr_service)。
   - **路徑 B**:不依賴 vendor 信心,改用 confidence-free 統計濾波
     (burst 間中位數跳變離群拒絕)。
   - **路徑 C**:接受現狀,主力改善靠物理貼合(睡眠繫緊錶帶)——
     55% PI 飽和是接觸/灌流限制,濾波無法無中生有。
3. evt=9 診斷上報暫留以驗證 revert(acc 應回到滿值),調完一併移除。

## 教訓

盲訂門檻 + 直接燒睡眠機 = 賠一夜資料。應「先量再閘」:這次正因為
有 evt=9 診斷上報,才能一夜定位 valid_level 恆 0,否則無串口的睡眠機
根本看不到。下次任何依賴 vendor 數值的閘,先用診斷管線確認該數值
有真實分布再啟用。
