# 手勢檢測應用 (Gesture Detection App)

## 概述

這是一個基於app_timer框架的手勢檢測應用，用於顯示當前手勢狀態和統計點擊次數。

## 功能特性

- **中心圓圈顯示**: 應用中心有一個灰色圓圈，用於顯示手勢狀態
- **手勢狀態指示**: 
  - 默認狀態：灰色圓圈
  - 檢測到點擊手勢：圓圈變為淺藍色
  - 檢測到放開手勢：圓圈變回灰色
- **點擊計數**: 在圓圈下方顯示點擊手勢觸發的累計次數
- **重置功能**: 提供重置按鈕來清零計數器

## 技術實現

### 架構設計
- 基於GUI應用框架 (GUI_APP_FRAMEWORK)
- 使用LVGL圖形庫進行UI渲染
- 集成手勢檢測系統 (BSP_USING_GESTURE_HANDLER)

### 核心組件
1. **UI組件結構** (`gesture_ui_t`)
   - `gesture_circle`: 中心手勢顯示圓圈
   - `status_label`: 狀態標籤
   - `count_label`: 計數標籤
   - `reset_button`: 重置按鈕

2. **數據上下文** (`app_gesture_data_ctx_t`)
   - `tap_count`: 點擊計數
   - `is_pressed`: 當前按壓狀態
   - `gesture_timer`: 手勢檢測定時器

### 手勢事件處理
- 通過UI處理器 (`lvgl_msg_handler`) 接收手勢事件
- 支持 `handle_tap_event` 和 `handle_tap_indicator` 事件
- 使用定時器進行手勢狀態檢測和更新

## 編譯配置

### 必需配置
```c
#define GUI_APP_FRAMEWORK
#define BSP_USING_UI_HANDLER
#define BSP_USING_GESTURE_HANDLER
#define APP_ID_GESTURE "gesture"
```

### 文件結構
```
gesture/
├── app_gesture.c      # 主應用文件
├── app_gesture.h      # 頭文件
├── SConscript         # 編譯腳本
└── README.md          # 說明文檔
```

## 使用方法

1. **啟動應用**: 從應用列表中選擇"Gesture"應用
2. **手勢檢測**: 進行點擊手勢，觀察圓圈顏色變化
3. **查看計數**: 觀察下方的點擊次數統計
4. **重置計數**: 點擊"Reset"按鈕清零計數器

## 開發說明

### 添加新功能
1. 在 `gesture_ui_t` 結構中添加新的UI組件
2. 在 `create_gesture_screen` 函數中創建UI元素
3. 在相應的事件處理函數中添加邏輯

### 修改手勢檢測
1. 調整 `gesture_timer_cb` 中的檢測邏輯
2. 修改定時器週期 (當前為50ms)
3. 更新手勢事件處理函數

### 自定義UI樣式
- 修改顏色定義 (GESTURE_COLOR_*)
- 調整圓圈大小 (GESTURE_CIRCLE_SIZE)
- 更改字體和佈局

## 注意事項

1. 確保手勢檢測系統正常工作
2. 應用需要正確的權限來訪問手勢事件
3. UI更新在主線程中進行，避免阻塞
4. 定時器使用軟定時器，避免影響系統性能

## 版本信息

- 版本: 1.0.0
- 作者: Skaiwalk Technology
- 更新日期: 2025.06.19