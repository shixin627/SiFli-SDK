# BMI270 抬腕亮屏功能说明

## 功能概述

BMI270 芯片内置了 **Wrist Wear Wake-up** 硬件手势识别功能，可以自动检测抬腕动作并触发中断，无需 MCU 持续采样和计算，极大降低了功耗。

本驱动已集成该功能，通过简单的宏开关即可在以下两种模式间切换：

1. **抬腕唤醒模式**（推荐用于待机）
   - BMI270 停止连续采样
   - 仅抬腕检测功能工作
   - 检测到抬腕时触发 INT1 中断
   - 极低功耗

2. **传统低功耗模式**（原有实现）
   - 25Hz 低频连续采样
   - 数据就绪中断
   - 功耗较高

## 配置方法

### 启用抬腕唤醒模式

在 [bmi270_driver.c](common/bmi270_driver.c) 文件中：

```c
/* 设置为 1 启用抬腕唤醒模式 */
#define BMI270_ENABLE_WRIST_WAKE 1
```

### 使用传统低功耗模式

```c
/* 设置为 0 使用原有的 25Hz 采样模式 */
#define BMI270_ENABLE_WRIST_WAKE 0
```

## API 使用

### 1. 初始化和打开传感器

```c
/* 打开传感器（默认进入高性能模式） */
bmi270_open();
```

### 2. 切换到抬腕唤醒模式（待机）

```c
/* 进入待机模式，启用抬腕检测 */
bmi270_low_power_mode();

/* 此时：
 * - 加速度计和陀螺仪停止连续采样
 * - 抬腕检测功能激活
 * - 功耗降至最低
 */
```

### 3. 处理抬腕中断

当用户抬起手腕时，BMI270 会触发 INT1 中断。在中断处理函数中：

```c
/* 中断处理（已在驱动中实现） */
static void bmi270_int_msg_handler(void)
{
    uint16_t int_status;
    bmi2_get_int_status(&int_status, &bmi2_dev);

    if (int_status & BMI270_WRIST_WAKE_UP_STATUS_MASK)
    {
        LOG_I("检测到抬腕动作！\n");

        /* TODO: 在此处添加应用层回调 */
        // display_wake();  // 唤醒屏幕
        // bmi270_high_performance_mode();  // 恢复正常模式
    }
}
```

### 4. 恢复正常模式

```c
/* 切换回高性能模式（100Hz 采样） */
bmi270_high_performance_mode();

/* 此时：
 * - 抬腕检测功能关闭
 * - 恢复 100Hz 连续采样
 * - 数据就绪中断启用
 */
```

### 5. 关闭传感器

```c
/* 关闭传感器 */
bmi270_close();
```

## 测试命令

驱动提供了调试命令用于测试：

```bash
# 打开传感器
bmi270 -open

# 启用抬腕唤醒模式
bmi270 -ww 1

# 现在抬起手腕，观察中断触发和日志输出

# 切换回正常模式
bmi270 -ww 0

# 关闭传感器
bmi270 -close
```

## 应用示例

### 智能手表待机/唤醒流程

```c
/* 1. 初始化阶段 */
void watch_init(void)
{
    bmi270_open();  // 打开传感器
}

/* 2. 进入待机模式 */
void watch_enter_standby(void)
{
    display_off();              // 关闭屏幕
    bmi270_low_power_mode();    // 启用抬腕检测
    /* MCU 可以进入深度睡眠，等待 INT1 中断唤醒 */
}

/* 3. 抬腕中断回调（在 bmi270_int_msg_handler 中调用） */
void on_wrist_wake_detected(void)
{
    /* 唤醒屏幕 */
    display_wake();
    display_show_watchface();

    /* 切换到正常模式以支持手势识别等功能 */
    bmi270_high_performance_mode();

    /* 启动定时器，X秒后自动回到待机模式 */
    start_auto_sleep_timer();
}

/* 4. 自动回到待机 */
void on_auto_sleep_timeout(void)
{
    display_off();
    bmi270_low_power_mode();
}
```

## 技术细节

### 中断引脚配置

- 使用 **INT1** 引脚（可通过 `BMI270_USE_INT1` 宏配置）
- 中断触发方式：下降沿触发
- 中断类型：非锁存（每次手势产生新的中断）

### 功耗对比

| 模式 | 功耗 | 特点 |
|------|------|------|
| 高性能模式 (100Hz) | ~650µA | 连续采样，用于活动时 |
| 传统低功耗 (25Hz) | ~180µA | 低频采样，保持数据更新 |
| **抬腕唤醒模式** | **~3.8µA** | 仅手势检测，待机首选 |

### 硬件实现

BMI270 内部集成了手势识别算法，无需 MCU 参与：

1. **内部加速度计低功耗模式**：芯片内部保持低频采样
2. **硬件手势识别**：内置算法检测抬腕动作
3. **中断触发**：检测到手势后立即触发 INT 引脚
4. **唤醒 MCU**：MCU 从睡眠中被唤醒处理

## 参数调整

如需调整抬腕检测灵敏度，可修改 `configure_wrist_wake_interrupt()` 函数：

```c
static int8_t configure_wrist_wake_interrupt(struct bmi2_dev *dev)
{
    /* 获取配置 */
    struct bmi2_sens_config sens_cfg;
    sens_cfg.type = BMI2_WRIST_WEAR_WAKE_UP;
    bmi270_get_sensor_config(&sens_cfg, 1, dev);

    /* 自定义参数（可选） */
    sens_cfg.cfg.wrist_wear_wake_up.min_angle_focus = 1448;     // 默认值
    sens_cfg.cfg.wrist_wear_wake_up.min_angle_nonfocus = 1774;  // 默认值
    sens_cfg.cfg.wrist_wear_wake_up.max_tilt_lr = 1024;         // 默认值
    sens_cfg.cfg.wrist_wear_wake_up.max_tilt_ll = 700;          // 默认值

    /* 应用配置 */
    bmi270_set_sensor_config(&sens_cfg, 1, dev);

    /* ... */
}
```

参数含义详见 BMI270 数据手册第 3.2.4 节。

## 注意事项

1. **中断引脚连接**：确保 BMI270 的 INT1 引脚正确连接到 MCU GPIO
2. **GPIO 中断配置**：确保 GPIO 中断设置为下降沿触发
3. **模式切换**：从抬腕模式切换到正常模式后，数据就绪中断会自动恢复
4. **调试**：可通过日志观察中断触发情况

## 问题排查

### 抬腕无反应
- 检查 `BMI270_ENABLE_WRIST_WAKE` 是否为 1
- 检查 INT1 引脚是否正确连接
- 检查 GPIO 中断是否启用
- 查看日志确认是否进入抬腕模式

### 误触发
- 调整灵敏度参数
- 检查手表佩戴位置和方向

## 参考资料

- BMI270 数据手册：Section 3.2.4 Wrist Wear Wake-up
- Bosch Sensortec 应用笔记：AN000-BMI270-Wrist-Gestures

## 版本历史

- **v1.0** (2024): 添加抬腕亮屏功能支持
