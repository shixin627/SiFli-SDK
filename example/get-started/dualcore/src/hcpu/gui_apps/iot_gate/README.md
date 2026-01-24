# IOT Gate Control App

## 概述
這是一個用於控制IOT鐵門的智能手錶應用程式，模仿 `app_media.c` 的結構設計。

## 功能特點

### 三個主要控制按鈕
1. **開啟按鈕** (左側) - 使用 `img_media_next` 圖標
   - 功能：開啟鐵門
   - 對應音樂控制的「下一首」

2. **暫停按鈕** (中間) - 使用 `img_media_pause` 圖標
   - 功能：暫停鐵門動作
   - 對應音樂控制的「暫停」

3. **關閉按鈕** (右側) - 使用 `img_media_previous` 圖標
   - 功能：關閉鐵門
   - 對應音樂控制的「上一首」

## UI 設計

### 應用程式界面 (App UI)
- 全屏黑色背景
- 三個按鈕以水平排列顯示
- 中間按鈕默認有白色陰影突出顯示
- 頂部顯示鐵門狀態標籤
- 支援手勢控制選擇按鈕

### Widget 界面
- 410x250 像素的黑色圓角矩形
- 三個按鈕水平排列
- 頂部顯示狀態文字
- 支援選擇背景動畫

## 按鈕狀態

### 鐵門狀態 (gate_state_t)
- `GATE_STATE_CLOSED` (0) - 鐵門已關閉
- `GATE_STATE_PAUSED` (1) - 鐵門暫停
- `GATE_STATE_OPENED` (2) - 鐵門已開啟

## 交互功能

### Y 軸拖拽控制
應用支援使用 Y 軸位置來選擇不同的按鈕，類似 media app 的控制方式：

- **Y 軸範圍 0~155**：選擇關閉按鈕（右側）
  - 手勢向右拖拽，選擇圖標會移動到右側
  - 映射到 0~-12 的偏移量，提供流暢的視覺反饋

- **Y 軸範圍 155~311**：選擇暫停按鈕（中間）
  - 手勢在中間區域，選擇圖標位於中央
  - 映射到 12~-12 的偏移量

- **Y 軸範圍 311~466**：選擇開啟按鈕（左側）
  - 手勢向左拖拽，選擇圖標會移動到左側
  - 映射到 12~0 的偏移量

### 手勢控制
- 支援手勢滑動選擇按鈕
- 選中時提供震動反饋（切換按鈕時震動）
- 點擊執行對應的鐵門控制命令

### 視覺反饋
- 按鈕選中時顯示半透明背景（OPA_50）
- 點擊後顯示高亮效果（OPA_30，300ms後自動消失）
- 選擇背景跟隨手勢移動，並有 75ms 的流暢動畫
- 支援邊界縮放效果，靠近邊界時偏移量會減小
- 狀態標籤實時更新鐵門狀態

## API 函數

### 應用程式生命週期
```c
void iot_gate_on_start(lv_obj_t *scr);  // 啟動應用
void iot_gate_on_resume(void);          // 恢復應用
void iot_gate_on_pause(void);           // 暫停應用
void iot_gate_on_stop(void);            // 停止應用
```

### Widget 控制
```c
lv_obj_t *lv_iot_gate_widget_builder(lv_obj_t *parent);  // 創建 widget
void clear_iot_gate_widget(void);                        // 清除 widget
void iot_gate_widget_start(void);                        // 啟動 widget
void iot_gate_widget_stop(void);                         // 停止 widget
```

### 事件處理
```c
void iot_gate_widget_handle_tap_event(void);           // 處理點擊事件
void iot_gate_widget_handle_press_event(uint8_t press); // 處理按壓事件
void iot_gate_trigger_drag_by_py(int p_y);             // Y軸拖拽控制
void iot_gate_widget_tap_event_cb(void);               // Widget點擊回調
```

### 按鈕創建函數
```c
lv_obj_t *iot_gate_close_btn_create(lv_obj_t *parent);  // 創建關閉按鈕
lv_obj_t *iot_gate_pause_btn_create(lv_obj_t *parent);  // 創建暫停按鈕
lv_obj_t *iot_gate_open_btn_create(lv_obj_t *parent);   // 創建開啟按鈕
```

### Widget 狀態控制
```c
void reset_iot_gate_widget(void);                      // 重置widget選擇狀態
void selection_iot_gate_widget(uint8_t index);         // 設置widget按鈕選擇
void reset_widget_btn_bg(void);                        // 重置背景透明度
```

## 配置要求

### 必需的宏定義
- `APP_ID_IOT_GATE` - 應用程式 ID
- `BSP_USING_MODEL_WATCH_SYS_INTERACT` - 系統交互支援（震動反饋）
- `BSP_USING_UI_HANDLER` - UI 處理器支援

### 依賴的圖標資源
- `img_media_previous` - 關閉按鈕
- `img_media_pause` - 暫停按鈕
- `img_media_next` - 開啟按鈕
- `control_selection_bg` - 選擇背景
- `img_logo` - 應用圖標

## 實現注意事項

### TODO 項目
1. **IOT 通信實現**
   - 在 `send_gate_command()` 函數中實現與 IOT 鐵門的通信
   - 可使用 BLE、WiFi 或其他通信協議
   - 需要處理連接狀態和錯誤情況

2. **狀態同步**
   - 實現鐵門實際狀態的回饋機制
   - 更新 UI 顯示真實的鐵門狀態

3. **安全機制**
   - 添加鐵門控制的安全驗證
   - 實現超時保護機制

## 文件結構
```
gui_apps/iot_gate/
├── app_iot_gate.c       # 主要實現文件
├── app_iot_gate.h       # 頭文件
└── README.md            # 說明文檔
```

## 使用示例

### 在主菜單中註冊應用
```c
BUILTIN_APP_EXPORT(
    LV_EXT_STR_ID(iot_gate),     // 應用名稱
    LV_EXT_IMG_GET(img_logo),     // 應用圖標
    APP_ID_IOT_GATE,              // 應用 ID
    app_main                       // 入口函數
);
```

### 啟動應用
```c
gui_app_run(APP_ID_IOT_GATE);
```

### Y 軸控制使用範例
```c
// 在手勢處理函數中調用
void handle_gesture(int y_position) {
    // y_position 範圍 0~466
    iot_gate_trigger_drag_by_py(y_position);
}

// 點擊確認選擇
void handle_tap() {
    iot_gate_widget_tap_event_cb();  // 根據當前選擇執行對應命令
}
```

### 與 LVGL 消息處理器集成
```c
// 在應用啟動時設置處理器
lvgl_msg_handler.handle_widgets_control = button_selection;
lvgl_msg_handler.handle_tap_indicator = iot_gate_widget_handle_press_event;

// 在應用停止時清理
lvgl_msg_handler.handle_widgets_control = NULL;
lvgl_msg_handler.handle_tap_indicator = NULL;
```

## 授權
Copyright (c) 2024 - 2025, Skaiwalk Technology
All rights reserved.
