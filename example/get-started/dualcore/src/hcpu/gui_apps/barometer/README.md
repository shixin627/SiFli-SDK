# Barometer App (氣壓計應用)

## 概述 (Overview)

這是一個專門讀取氣壓計數據的應用程式，透過 `ui_datac_subscribe` 直接訂閱 BARO 服務來獲取氣壓和海拔高度數據。

This is a barometer reading application that subscribes to the BARO service directly using `ui_datac_subscribe` to obtain air pressure and altitude data.

## 功能特點 (Features)

1. **實時氣壓監測** - 顯示當前氣壓值（單位：hPa）
2. **氣壓範圍追蹤** - 記錄並顯示最大和最小氣壓值
3. **海拔高度計算** - 基於氣壓計算當前海拔高度（單位：米）
4. **海拔範圍追蹤** - 記錄並顯示最大和最小海拔高度
5. **直接服務訂閱** - 使用 `ui_datac_subscribe` 直接訂閱服務，不通過 client

## 技術實現 (Technical Implementation)

### 服務訂閱 (Service Subscription)

```c
// 打開數據客戶端
app_baro_data_ctx.data_handle = datac_open();

// 直接訂閱 BARO 服務
ui_datac_subscribe(app_baro_data_ctx.data_handle, "BARO", baro_data_callback, 0);
```

### 回調處理 (Callback Handling)

應用程式處理以下消息類型：
- `MSG_SERVICE_SUBSCRIBE_RSP` - 訂閱確認
- `MSG_SERVICE_DATA_NTF_IND` - 實時氣壓數據通知
- `MSG_SERVICE_BARO_RANGE_RSP` - 氣壓範圍響應（最大/最小值）
- `MSG_SERVICE_ALTITUDE_RANGE_RSP` - 海拔範圍響應（當前/最大/最小值）

### 數據請求 (Data Requests)

訂閱成功後，應用會發送以下請求：

```c
// 請求氣壓範圍數據
msg_payload = data_service_init_msg(&msg, MSG_SERVICE_BARO_RANGE_REQ, 1);
msg_payload[0] = BAROS_BARO_RANGE_LEN;
datac_send_msg(app_baro_data_ctx.data_handle, &msg);

// 請求海拔範圍數據
msg_payload = data_service_init_msg(&msg, MSG_SERVICE_ALTITUDE_RANGE_REQ, 1);
msg_payload[0] = BAROS_ALTI_RANGE_LEN;
datac_send_msg(app_baro_data_ctx.data_handle, &msg);
```

## UI 界面 (User Interface)

界面分為兩個主要部分：

### 1. 氣壓顯示區域
- 當前氣壓（藍色）
- 最大氣壓（紅色）
- 最小氣壓（綠色）

### 2. 海拔顯示區域
- 當前海拔（紫色）
- 最大海拔（橙色）
- 最小海拔（青綠色）

## 生命週期管理 (Lifecycle Management)

- **on_start**: 初始化 UI 界面
- **on_resume**: 訂閱氣壓計服務，開始接收數據
- **on_pause**: 取消訂閱服務，停止接收數據
- **on_stop**: 清理資源，釋放記憶體

## 參考文件 (References)

- 服務定義: `example/get-started/dualcore/src/modules/service/baro_service.c`
- 參考範例: `example/get-started/dualcore/src/hcpu/gui_apps/heartrate/heart_rate.c`
- 訂閱接口: `middleware/include/ui_datasrv_subscriber.h`

## 編譯配置 (Build Configuration)

需要定義 `APP_ID_BAROMETER` 宏來啟用此應用程式。

## 依賴項 (Dependencies)

- RT-Thread RTOS
- LVGL GUI 庫
- Data Service Framework
- Barometer Service (baro_service)
- 氣壓感測器硬體 (SPL06 或 BMP280)
