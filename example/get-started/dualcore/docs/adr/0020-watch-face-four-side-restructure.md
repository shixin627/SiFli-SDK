# ADR-0020 — 錶盤四周大改：右=各設備媒體中心、左=合併 session、下=控制中心+App List

- **狀態**：草稿（founder 2026-08-11 口述目標，尚未實作）
- **agent**：Claude Opus 5
- **trigger**：founder「UI 我要再次做大幅度調整」。前一輪（ADR-0047 / 2026-08-10~11）才把右側做成「一台設備一欄的 session 列表」並在兩台桌面上驗過；本 ADR **取代**該配置。

## 目標配置

| 方位 | 改成 | 目前是什麼 |
|---|---|---|
| **右側**（一欄一頁） | **各設備的媒體中心**，一台一欄；**手機也是其中一頁** | 各設備的 session 列表（一台一欄） |
| 右側每一頁**往下拉** | **那台設備的滑鼠頁面** | 那台設備的媒體中心 |
| **左側** | **合併後的 session 列表**（跨設備合成一份），用**現在通知列表的 UI**；列表**下面接原本的 actions 列表** | 指令混合清單 reveal 浮層 |
| **下方**（往上拉） | **控制中心 + App List 合併頁**。控制中心只留：亮度、QR code、勿擾、找手機；**開發版另加 gesture** | 空（控制中心 2026-08-06 已搬進頂部面板） |
| 手勢操作 | **全部保留** | — |

## 這會動到哪幾層（動工前的依賴清單）

前一輪的教訓是「改結構時，舊邏輯依賴的前提被換掉，每一處單獨看都正確」——五個回歸全部屬於這類。所以先列清單再動：

1. **tileview 格子配置**（`app_clock_status_bar.c`）：右側欄位語意從 session→媒體；左側從 reveal 浮層→實體頁；下方要重新長出一列。`active_pos` 是 tile 加入順序索引，**任何新增格子都會讓既有索引判斷失效**（已被咬過一次）。
2. **頂部面板**（`lv_top_panel.c`）：媒體頁搬走之後，面板剩什麼？（推測只剩通知列表 —— **待 founder 確認**）；控制中心搬到下方後，面板內的左右 pager 結構要重寫。
3. **媒體頁實體**：目前由 `hid_mouse_media_page_create()` 建在面板的 pager 裡，並用 `hid_mouse_media_page_bind()` 綁定曲名路由。搬到 tileview 欄位後，一台一頁常駐 vs 只綁當前頁，要重新決定。
4. **滑鼠頁面**：目前是 `hid_mouse` 的完整 UI 圖層（`hid_mouse_build_ui`），由面板底部按鈕切換。改成「每個媒體頁往下拉」= 每台設備一個入口。
5. **session 列表**：從 per-device 多份**合併成一份**——`lv_session_pager.c` 的 per-device 儲存（`s_devices[]`）保留（資料仍需知道每列屬於哪台，才 route 得回正確桌面），但**呈現層合併**，且改用通知列表的卡片 UI（`lv_top_panel` 的通知那頁）。
6. **actions 列表**：目前在 `lv_instruction_list_layout.c`，是掛在 `lv_layer_top` 的全域浮層。要接到 session 列表下方 = 從浮層改成同一個捲動容器的一段。
7. **視覺狀態機**（`app_clock_status_bar.c` 的 `LV_EVENT_SCROLL`）：**建議重寫**。它是為水平換頁寫的，垂直軸是硬接的第二條軸；`gaus_dial_bg` 目前有 5–6 條路徑在寫濃度，靠執行順序與 early-break 決勝負。2026-08-11「下拉時看不到高斯模糊」未解 bug 就活在這裡。

## 待確認（founder）

1. **頂部下拉還剩什麼？** 媒體中心搬到右側、控制中心搬到下方之後，推測只剩通知列表 —— 對嗎？
2. **左側 session 列表點進去還是聊天室嗎？** 麥克風（開新 session，0x24 已實作）留在哪一層？
3. **合併後的列表要不要標示來源設備？** 不標的話使用者無法分辨同名對話屬於哪台；標的話用什麼形式（副標／icon／分組）。
4. **右側「手機也是其中一頁」**：手機的媒體中心內容從哪來？（目前媒體頁是 per-desktop 的 `hid_mouse` 設備索引，手機不在那個 registry 裡）
5. **App List 與控制中心怎麼合併**：上下分區、還是控制中心當第一列？

## 保留的既有結論（別重蹈）

- **catcher 要當該軸的捲動死路**（SCROLLABLE + 另一軸 scroll_dir + 清該軸 chain）；只設 CLICKABLE 擋不住 tileview。
- **手勢位移在 `PRESSING` 期間累計**，不可讀放開座標（ft3168 I2C 失敗會捏造 `(0,0)` Up event）。
- **`tile->dir` 不能在 `VALUE_CHANGED` 裡改**：lv_tileview 在發事件前已把 dir 複製走，改了要**當場**對 tileview 重設 scroll_dir。
- **此錶 console 只印 W/E**，臨時 trace 用 `LOG_W`。
- **`_watch_build.cmd` 的 exit code 不可信**，gate 看 `scons: done building targets.` + `main.bin` mtime。
- 共享視覺狀態（如 `gaus_dial_bg`）**先量再改**，不要推理它當下的值。
