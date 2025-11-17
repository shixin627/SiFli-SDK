# 电池电量计算

电池电量计算模块用于根据电池电压值计算当前电池电量百分比。考虑了充电和放电两种状态，使用不同的电池曲线表，并实现了多级滤波算法来提高电量计算的准确性和稳定性。

## 功能特性

- **双曲线支持**：分别支持充电和放电两种状态的电池电压曲线
- **多级滤波算法**：
  - 一级滤波：基于阈值的跳变过滤
  - 二级滤波：加权平均平滑处理（可选）
- **充放电状态检测**：自动识别当前充电状态并切换相应曲线
- **线性插值计算**：在曲线表中进行精确的百分比计算

## 核心结构体

### battery_calculator_config_t

电池计算器配置结构体：

```c
typedef struct {
    const battery_lookup_point_t *charging_table;      // 充电曲线表
    uint32_t charging_table_size;                      // 充电曲线表大小
    const battery_lookup_point_t *discharging_table;   // 放电曲线表
    uint32_t discharging_table_size;                   // 放电曲线表大小
    uint32_t charge_filter_threshold;                  // 充电滤波阈值 (mV)
    uint32_t discharge_filter_threshold;               // 放电滤波阈值 (mV)
    uint8_t filter_count;                              // 滤波计数阈值
    bool secondary_filter_enabled;                     // 是否启用二级滤波
    uint8_t secondary_filter_weight_pre;               // 二级滤波前电压权重 (0-100)
    uint8_t secondary_filter_weight_cur;               // 二级滤波当前电压权重 (0-100)
} battery_calculator_config_t;
```

## API 函数

### 1. battery_calculator_init

```c
void battery_calculator_init(battery_calculator_t *calc, 
                            const battery_calculator_config_t *config);
```

**功能**：初始化电池计算器

**参数**：
- `calc`：电池计算器实例指针
- `config`：配置参数指针

**示例**：
```c
battery_calculator_t battery_calc;
battery_calculator_config_t calc_config = {
    .charging_table = charging_curve_table,
    .charging_table_size = charging_curve_table_size,
    .discharging_table = discharge_curve_table,
    .discharging_table_size = discharge_curve_table_size,
    .charge_filter_threshold = 50,      // 充电滤波阈值
    .discharge_filter_threshold = 30,   // 放电滤波阈值
    .filter_count = 3,                   // 滤波计数阈值
    .secondary_filter_enabled = true,    // 启用二级滤波
    .secondary_filter_weight_pre = 90,   // 二级滤波前电压权重
    .secondary_filter_weight_cur = 10    // 二级滤波当前电压权重
};

battery_calculator_init(&battery_calc, &calc_config);
```

### 2. battery_calculator_get_percent

```c
uint8_t battery_calculator_get_percent(battery_calculator_t *calc, 
                                      uint32_t voltage_mv);
```

**功能**：计算电池电量百分比

**参数**：
- `calc`：电池计算器实例指针
- `voltage_mv`：当前电池电压值（毫伏）

**返回值**：电池电量百分比（0-100）

**示例**：
```c
uint32_t voltage = 3800;  // 3.8V
uint8_t percentage = battery_calculator_get_percent(&battery_calc, voltage);
rt_kprintf("Battery: %d%%\n", percentage);
```

## 使用示例

### 应用层完整示例代码

```c
#include "battery_calculator.h"

void battery_monitor_task(void *parameter)
{
    // 1. 初始化电池计算器
    battery_calculator_t battery_calc;
    battery_calculator_config_t calc_config = {
        .charging_table = charging_curve_table,
        .charging_table_size = charging_curve_table_size,
        .discharging_table = discharge_curve_table,
        .discharging_table_size = sizeof(discharge_curve_table)/sizeof(battery_lookup_point_t),
        .charge_filter_threshold = 50,      // 充电时50mV阈值
        .discharge_filter_threshold = 30,   // 放电时30mV阈值
        .filter_count = 3,                  // 需要3次确认
        .secondary_filter_enabled = true,   // 启用二级滤波
        .secondary_filter_weight_pre = 90,  // 前电压权重90%
        .secondary_filter_weight_cur = 10   // 当前电压权重10%
    };
    
    battery_calculator_init(&battery_calc, &calc_config);
    
    while (1)
    {
        // 2. 读取ADC电压值
        rt_device_t battery_device = rt_device_find("bat1");
        rt_adc_cmd_read_arg_t read_arg;
        read_arg.channel = 7;
        
        rt_adc_enable((rt_adc_device_t)battery_device, read_arg.channel);
        rt_thread_mdelay(300);
        rt_uint32_t voltage = rt_adc_read((rt_adc_device_t)battery_device, read_arg.channel);
        rt_adc_disable((rt_adc_device_t)battery_device, read_arg.channel);
        
        // 3. 计算电量百分比
        uint8_t percentage = battery_calculator_get_percent(&battery_calc, voltage);   

        rt_thread_mdelay(1000);
    }
}
```

## 配置参数说明

### 滤波阈值设置

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `charge_filter_threshold` | 50mV | 充电状态下的电压跳变阈值 |
| `discharge_filter_threshold` | 30mV | 放电状态下的电压跳变阈值 |
| `filter_count` | 3 | 需要连续N次超过阈值才更新 |

### 二级滤波参数

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `secondary_filter_enabled` | true | 是否启用二级滤波 |
| `secondary_filter_weight_pre` | 90 | 历史电压权重（0-100） |
| `secondary_filter_weight_cur` | 10 | 当前电压权重（0-100） |

**注意**：`weight_pre + weight_cur = 100`

## 电池曲线表配置

**重要说明**：这里提供的默认电池曲线表仅作为参考示例，不一定适配当前使用的实际电池。为确保电量计算的准确性，**强烈建议从电池厂家获取匹配的电池电压-电量曲线数据**，并根据实际曲线配置相应的查找表。

电池曲线表定义在 `battery_table.c` 中，格式如下：

```c
const battery_lookup_point_t chargeing_curve_table[] = {
    {4200, 100},  // 4.2V -> 100%
    {4100, 90},
    {4000, 80},
    // ... 更多点
    {3300, 0}     // 3.3V -> 0%
};
```

**配置要点**：
1. 电压值从高到低排列
2. 百分比从100到0对应
3. 充电和放电曲线分别定义
4. 根据实际电池特性调整曲线点

## 注意事项

1. **初始化顺序**：必须先调用 `battery_calculator_init()` 再使用 `battery_calculator_get_percent()`
2. **电压单位**：所有电压值使用毫伏（mV）为单位
3. **曲线表要求**：曲线表必须按电压从高到低排序
4. **充电状态检测**：模块内部通过 `CHARGE_DETECT_PIN` 自动检测充电状态
5. **滤波参数调整**：根据实际电池特性和应用场景调整滤波参数

## 常见问题

**Q: 电量跳变太快怎么办？**  
A: 增大滤波阈值或增加 `filter_count` 值，或启用二级滤波并增大历史权重。

**Q: 电量显示不准确？**  
A: 检查电池曲线表是否符合实际电池特性，必要时重新校准曲线点。

**Q: 如何禁用二级滤波？**  
A: 设置 `secondary_filter_enabled = false`。


