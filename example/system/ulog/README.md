# 日志使用示例

源码路径：example/system/ulog

## 支持的平台

- sf32lb52-lcd系列
- sf32lb56-lcd系列
- sf32lb58-lcd系列

## 编译和烧录

切换到例程 project 目录，运行 scons 命令执行编译：
```
scons --board=sf32lb52-lchspi-ulp -j32
```

## 概述

本例程演示如何使用 RT-Thread ulog 日志组件，主要功能包括：

- **LOG_X API 使用**：演示 LOG_D、LOG_I、LOG_W、LOG_E、LOG_RAW 五种日志级别的使用方法
- **ulog_hexdump 使用**：演示十六进制数据打印功能
- **动态日志过滤**：通过 ulog_lvl、ulog_tag、ulog_kw 命令动态调整日志输出
- **日志级别原理**：帮助理解编译时级别和运行时过滤的区别

## 代码中使用 ulog 打印日志

### 定义日志标签和级别

在代码中使用 ulog 前，必须先定义 `LOG_TAG` 和 `LOG_LVL`：

```c
#define LOG_TAG              "example"         // 该模块对应的标签
#define LOG_LVL              LOG_LVL_DBG       // 该模块的日志输出级别(DBG=7)
#include <ulog.h>                             // 必须在 LOG_TAG 与 LOG_LVL 下面
```

### LOG_X API 级别说明

日志级别定义在 `ulog_def.h` 文件中：

| LOG_X 宏 | 级别值 | 含义 | 颜色 |
|-----------|--------|------|------|
| LOG_E | 3 | ERROR (错误) | 红色 |
| LOG_W | 4 | WARNING (警告) | 黄色 |
| LOG_I | 6 | INFO (信息) | 绿色 |
| LOG_D | 7 | DEBUG (调试) | 蓝色 |
| LOG_RAW | 无 | 原始输出，绕过级别过滤 | 无 |


### 使用测试示例

```c
int ulog_test(int argc, char **argv)
{
    static uint8_t buf[128];
    int i = 0;

    for (i = 0; i < sizeof(buf); i++)
    {
        buf[i] = i;
    }

    /* output different level log by LOG_X API */
    LOG_D("LOG_D: RT-Thread is an open source IoT operating system from China.\n");
    LOG_I("LOG_I: RT-Thread is an open source IoT operating system from China.\n");
    LOG_W("LOG_W: RT-Thread is an open source IoT operating system from China.\n");
    LOG_E("LOG_E: RT-Thread is an open source IoT operating system from China.\n");
    LOG_RAW("LOG_RAW: RT-Thread is an open source IoT operating system from China.\n");
    ulog_hexdump("buf_dump_test", 16, buf, sizeof(buf));
    rt_thread_mdelay(10);

    return 0;
}
MSH_CMD_EXPORT(ulog_test, ulog test.);
```

## ulog 配置选项说明

在编译界面输入 `sdk.py menuconfig` 进入，在 `RT-Thread Components → Utilities → Enable ulog` 下配置：

| 配置项 | 说明 |
|--------|------|
| `Enable ulog` | 使能 ulog |
| `The static output log level` | 编译时静态日志输出级别。比设定级别低的 LOG_X 将不会被编译到 ROM 中 |
| `Enable ISR log` | 使能中断 ISR 日志，即在 ISR 中也可以使用日志输出 API |
| `Enable assert check` | 使能断言检查 |
| `The log's max width` | 日志的最大长度 |
| `Enable async output mode` | 使能异步日志输出模式 |
| `log format` | 日志格式配置（颜色、时间、线程信息等） |
| `Enable console backend` | 使能控制台作为后端 |
| `Enable file backend` | ulog 的文件后端 |
| `Enable runtime log filter` | 使能运行时日志过滤器（动态过滤） |
| `Enable syslog format log and API` | 启用 syslog 格式日志 |

### 日志格式配置 (log format)

在 `RT-Thread Components → Utilities → Enable ulog → log format` 下配置：

| 配置项 | 说明 |
|--------|------|
| `Enable float number support` | 浮点型数字的支持 |
| `Enable color log` | 带颜色的日志 |
| `Enable time information` | 时间信息 |
| `Enable timestamp format for time` | 包括时间戳 |
| `Enable level information` | 级别信息 |
| `Enable tag information` | 标签信息 |
| `Enable thread information` | 线程信息 |

## 动态过滤配置注意事项

### syslog 模式与普通模式的区别

> [!IMPORTANT]
> `Enable syslog format log and API` 和 `Enable runtime log filter` 是**两个独立**的配置：
>
> - `ULOG_USING_SYSLOG` 关闭时，`ULOG_USING_FILTER` **默认也关闭**（需要显式开启）
> - `ULOG_USING_SYSLOG` 开启时，会**自动选中** `ULOG_USING_FILTER`

### syslog 开启 vs 关闭的区别

| 特性 | syslog 开启 | syslog 关闭 |
|------|-------------|-------------|
| 日志格式 | `<7>01-01 00:00:01 example tshell: LOG_D: ...` | `D/example (0): LOG_D: ...` |
| 级别表示 | `<优先级>` 格式 | `D/`, `I/`, `W/`, `E/` 前缀 |
| `ulog_lvl` 设置值 | 需要设 **255** 才能全部显示 | 设 **7** 即可全部显示 |
| 过滤方式 | 位掩码 (`1 << level`) | 大小比较 (`level > filter`) |

### 本例程配置

当前例程 `proj.conf` 配置：

```conf
CONFIG_RT_USING_ULOG=y
CONFIG_ULOG_USING_ISR_LOG=y
CONFIG_ULOG_OUTPUT_FLOAT=y
CONFIG_ULOG_TIME_USING_TIMESTAMP=y
CONFIG_ULOG_OUTPUT_THREAD_NAME=y
CONFIG_ULOG_USING_SYSLOG=y
```

其中 `CONFIG_ULOG_USING_SYSLOG=y` 会自动选中 `CONFIG_ULOG_USING_FILTER=y`。

## 示例输出

编译烧录后，串口输出如下：

```
01-01 00:00:01 D/example tshell: LOG_D: RT-Thread is an open source IoT operating system from China.
01-01 00:00:01 I/example tshell: LOG_I: RT-Thread is an open source IoT operating system from China.
01-01 00:00:01 W/example tshell: LOG_W: RT-Thread is an open source IoT operating system from China.
01-01 00:00:01 E/example tshell: LOG_E: RT-Thread is an open source IoT operating system from China.
LOG_RAW: RT-Thread is an open source IoT operating system from China.
D/HEX buf_dump_test: 0000-0010: 00 01 02 03 04 05 06 07  08 09 0A 0B 0C 0D 0E 0F    ................
```

## 动态过滤测试

### 动态过滤命令概述

ulog 支持三种运行时动态过滤方式：

| 命令 | 功能 | 说明 |
|------|------|------|
| `ulog_tag [tag]` | 按标签过滤 | tag 为空时取消标签过滤 |
| `ulog_lvl <level>` | 按级别过滤 | level 取值见下文 |
| `ulog_kw [keyword]` | 按关键词过滤 | keyword 为空时取消关键词过滤 |
| `ulog_filter` | 查看当前过滤设置 | 显示所有过滤条件 |

**过滤规则**：以上过滤条件是**同时生效**的，只有同时满足所有过滤条件的日志才会输出。

### 运行测试命令

```bash
ulog_test
```

### 按标签全局过滤

命令格式：`ulog_tag [tag]`，tag 为空时取消标签过滤。

该过滤方式可以对所有日志执行按标签过滤，只有包含标签信息的日志才允许输出。

**标签 (TAG)** 是日志输出时表示来源模块/组件的字符串，定义在代码的 `LOG_TAG` 宏中。

**示例**：有 `wifi.driver`、`wifi.mgnt`、`audio.driver` 3种标签的日志，当设定过滤标签为 `wifi` 时，只有标签为 `wifi.driver` 及 `wifi.mgnt` 的日志会输出，标签为 `audio.driver` 的日志将被过滤掉。

过滤原理：ulog_tag 使用**前缀匹配**，设置 `wifi` 会匹配所有以 `wifi` 开头的标签。

```bash
# 设置标签过滤，只显示包含"wifi"标签的日志
ulog_tag wifi

# 取消标签过滤，显示所有标签的日志
ulog_tag
```

### 按关键词全局过滤

命令格式：`ulog_kw [keyword]`，keyword 为空时取消关键词过滤。

该过滤方式可以对所有日志执行按关键词过滤，包含关键词信息的日志才允许输出。

```bash
# 设置关键词过滤，只显示包含"ERROR"关键词的日志
ulog_kw ERROR

# 取消关键词过滤
ulog_kw
```

### 查看过滤器信息

在设定完过滤参数后，输入 `ulog_filter` 命令可以查看当前过滤信息：

```bash
ulog_filter
```

输出示例：
```
current filter info:
level: 248
tag: wifi
keyword: ERROR
```

### syslog 开启时的等级设置

当前例程 `CONFIG_ULOG_USING_SYSLOG=y` 已开启，使用**位掩码**方式，`ulog_lvl` 参数是掩码值：

> | 命令 | 掩码值 | 二进制 | 控制的级别 |
> |------|--------|--------|------------|
> | `ulog_lvl 128` | 128 | 10000000 | 控制 LEVEL 7 (DEBUG) |
> | `ulog_lvl 64` | 64 | 01000000 | 控制 LEVEL 6 (INFO) |
> | `ulog_lvl 32` | 32 | 00100000 | 控制 LEVEL 5 (NOTICE) |
> | `ulog_lvl 16` | 16 | 00010000 | 控制 LEVEL 4 (WARNING) |
> | `ulog_lvl 8` | 8 | 00001000 | 控制 LEVEL 3 (ERROR) |

**位与级别对应关系**（syslog.h定义）：
- bit0=EMERG(0), bit1=ALERT(1), bit2=CRIT(2), bit3=ERR(3), bit4=WARNING(4), bit5=NOTICE(5), bit6=INFO(6), bit7=DEBUG(7)

**过滤逻辑**：`LOG_MASK(LOG_PRI(level)) & ulog.filter.level`，只有当日志级别的对应位为1时才显示。

**LOG_RAW** 不经过级别过滤，总是输出。

### 测试步骤

1. **查看当前过滤设置**
```bash
ulog_filter
```

2. **设置为全部显示 (syslog模式需要255)**
```bash
ulog_lvl 255
ulog_test
```

3. **只显示 ERROR (8)**
```bash
ulog_lvl 8
ulog_test
```

4. **只显示 WARNING 及以下 (24)**
```bash
ulog_lvl 24
ulog_test
```

5. **只显示 INFO 及以下 (120)**
```bash
ulog_lvl 120
ulog_test
```

6. **只显示 DEBUG 及以下 (248)**
```bash
ulog_lvl 248
ulog_test
```

7. **恢复全部显示 (255)**
```bash
ulog_lvl 255
ulog_test
```

### 完整交互日志

以下是一次完整的动态过滤测试过程（LOG_RAW 不受级别控制，始终输出）：

```
msh />ulog_lvl 255
msh />ulog_test
01-01 00:00:01 D/example tshell: LOG_D: RT-Thread is an open source IoT operating system from China.
01-01 00:00:01 I/example tshell: LOG_I: RT-Thread is an open source IoT operating system from China.
01-01 00:00:01 W/example tshell: LOG_W: RT-Thread is an open source IoT operating system from China.
01-01 00:00:01 E/example tshell: LOG_E: RT-Thread is an open source IoT operating system from China.
LOG_RAW: RT-Thread is an open source IoT operating system from China.
D/HEX buf_dump_test: ...

msh />ulog_lvl 8
msh />ulog_test
01-01 00:00:10 E/example tshell: LOG_E: RT-Thread is an open source IoT operating system from China.
LOG_RAW: RT-Thread is an open source IoT operating system from China.

msh />ulog_lvl 24
msh />ulog_test
01-01 00:00:20 W/example tshell: LOG_W: RT-Thread is an open source IoT operating system from China.
01-01 00:00:20 E/example tshell: LOG_E: RT-Thread is an open source IoT operating system from China.
LOG_RAW: RT-Thread is an open source IoT operating system from China.

msh />ulog_lvl 120
msh />ulog_test
01-01 00:00:30 I/example tshell: LOG_I: RT-Thread is an open source IoT operating system from China.
01-01 00:00:30 W/example tshell: LOG_W: RT-Thread is an open source IoT operating system from China.
01-01 00:00:30 E/example tshell: LOG_E: RT-Thread is an open source IoT operating system from China.
LOG_RAW: RT-Thread is an open source IoT operating system from China.

msh />ulog_lvl 248
msh />ulog_test
01-01 00:00:40 D/example tshell: LOG_D: RT-Thread is an open source IoT operating system from China.
01-01 00:00:40 I/example tshell: LOG_I: RT-Thread is an open source IoT operating system from China.
01-01 00:00:40 W/example tshell: LOG_W: RT-Thread is an open source IoT operating system from China.
01-01 00:00:40 E/example tshell: LOG_E: RT-Thread is an open source IoT operating system from China.
LOG_RAW: RT-Thread is an open source IoT operating system from China.
D/HEX buf_dump_test: ...

msh />ulog_lvl 255
msh />ulog_test
01-01 00:01:00 D/example tshell: LOG_D: RT-Thread is an open source IoT operating system from China.
01-01 00:01:00 I/example tshell: LOG_I: RT-Thread is an open source IoT operating system from China.
01-01 00:01:00 W/example tshell: LOG_W: RT-Thread is an open source IoT operating system from China.
01-01 00:01:00 E/example tshell: LOG_E: RT-Thread is an open source IoT operating system from China.
LOG_RAW: RT-Thread is an open source IoT operating system from China.
D/HEX buf_dump_test: ...
```

## hexdump 输出使用

`ulog_hexdump` 用于以十六进制格式输出数据：

```c
uint8_t buf[128];
for (int i = 0; i < sizeof(buf); i++)
{
    buf[i] = i;
}
ulog_hexdump("buf_dump_test", 16, buf, sizeof(buf));
```

输出示例：
```
D/HEX buf_dump_test: 0000-0010: 00 01 02 03 04 05 06 07  08 09 0A 0B 0C 0D 0E 0F    ................
                         0010-0020: 10 11 12 13 14 15 16 17  18 19 1A 1B 1C 1D 1E 1F    ................
```


`ulog_hexdump` 使用 **DEBUG 级别 (level=7)**，因此当运行时过滤级别低于 7 时，hexdump 输出**不会显示**。只有设置 `ulog_lvl 255` (syslog模式) 或 `ulog_lvl 7` (非syslog模式) 时才能看到 hexdump 输出。

## 在中断ISR中使用

ulog 支持在中断 ISR 中输出日志，但默认没有开启。使用时在 menuconfig 中打开 `Enable ISR log` 选项即可，API 与线程中使用方式一致。

## 参考文档

- [RT-Thread ulog 组件文档](https://www.rt-thread.org/document/programming/#ulog)
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)

## 更新记录

| 版本 | 日期 | 发布说明 |
|:---|:---|:---|
| 0.0.1 | 1/2025 | 初始版本 |
| 0.0.2 | 5/2025 | 完善动态过滤配置说明，修正 syslog 模式等级设置 |
