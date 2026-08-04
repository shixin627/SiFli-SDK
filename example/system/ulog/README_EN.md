# Log Usage Example

Source code path: `example/system/ulog`

## Supported Platforms

- sf32lb52-lcd series
- sf32lb56-lcd series
- sf32lb58-lcd series

## Building and Flashing

Switch to the project directory and run the scons command to build:
```
scons --board=sf32lb52-lchspi-ulp -j32
```

## Overview

This example demonstrates how to use the RT-Thread ulog component, with the following main features:

- **LOG_X API Usage**: Demonstrates how to use five log levels: LOG_D, LOG_I, LOG_W, LOG_E, LOG_RAW
- **ulog_hexdump Usage**: Demonstrates the hexadecimal data printing feature
- **Dynamic Log Filtering**: Dynamically adjust log output via ulog_lvl, ulog_tag, ulog_kw commands
- **Log Level Principles**: Helps understand the difference between compile-time level and runtime filtering

## Using ulog in Code

### Define Log Tag and Level

Before using ulog in code, you must define `LOG_TAG` and `LOG_LVL`:

```c
#define LOG_TAG              "example"         // Tag for this module
#define LOG_LVL              LOG_LVL_DBG       // Log output level for this module (DBG=7)
#include <ulog.h>                             // Must be below LOG_TAG and LOG_LVL
```

### LOG_X API Level Description

Log levels are defined in `ulog_def.h`:

| LOG_X Macro | Level Value | Meaning | Color |
|-------------|-------------|---------|-------|
| LOG_E | 3 | ERROR | Red |
| LOG_W | 4 | WARNING | Yellow |
| LOG_I | 6 | INFO | Green |
| LOG_D | 7 | DEBUG | Blue |
| LOG_RAW | N/A | Raw output, bypasses level filtering | None |


### Usage Example

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

## ulog Configuration Options

Enter `sdk.py menuconfig` from the build configuration menu, and configure under `RT-Thread Components → Utilities → Enable ulog`:

| Configuration Option | Description |
|---------------------|-------------|
| `Enable ulog` | Enable ulog |
| `The static output log level` | Compile-time static log output level. LOG_X lower than this level will not be compiled into ROM |
| `Enable ISR log` | Enable ISR log - allows log output in ISR |
| `Enable assert check` | Enable assert check |
| `The log's max width` | Maximum length of log |
| `Enable async output mode` | Enable async output mode |
| `log format` | Log format configuration (color, time, thread info, etc.) |
| `Enable console backend` | Enable console as backend |
| `Enable file backend` | File backend for ulog |
| `Enable runtime log filter` | Enable runtime log filter (dynamic filtering) |
| `Enable syslog format log and API` | Enable syslog format log |

### Log Format Configuration

Configure under `RT-Thread Components → Utilities → Enable ulog → log format`:

| Configuration Option | Description |
|---------------------|-------------|
| `Enable float number support` | Support for floating-point numbers |
| `Enable color log` | Colored log output |
| `Enable time information` | Time information |
| `Enable timestamp format for time` | Include timestamp |
| `Enable level information` | Level information |
| `Enable tag information` | Tag information |
| `Enable thread information` | Thread information |

## Dynamic Filtering Configuration Notes

### Difference Between syslog Mode and Normal Mode

> [!IMPORTANT]
> `Enable syslog format log and API` and `Enable runtime log filter` are **two independent** configurations:
>
> - When `ULOG_USING_SYSLOG` is OFF, `ULOG_USING_FILTER` is **OFF by default** (needs to be explicitly enabled)
> - When `ULOG_USING_SYSLOG` is ON, it **automatically selects** `ULOG_USING_FILTER`

### syslog ON vs OFF

| Feature | syslog ON | syslog OFF |
|---------|-----------|------------|
| Log format | `<7>01-01 00:00:01 example tshell: LOG_D: ...` | `D/example (0): LOG_D: ...` |
| Level notation | `<priority>` format | `D/`, `I/`, `W/`, `E/` prefix |
| `ulog_lvl` value | Need to set **255** to show all | Set **7** to show all |
| Filtering method | Bitmask (`1 << level`) | Comparison (`level > filter`) |

### This Example Configuration

Current example `proj.conf`:

```conf
CONFIG_RT_USING_ULOG=y
CONFIG_ULOG_USING_ISR_LOG=y
CONFIG_ULOG_OUTPUT_FLOAT=y
CONFIG_ULOG_TIME_USING_TIMESTAMP=y
CONFIG_ULOG_OUTPUT_THREAD_NAME=y
CONFIG_ULOG_USING_SYSLOG=y
```

`CONFIG_ULOG_USING_SYSLOG=y` automatically selects `CONFIG_ULOG_USING_FILTER=y`.

## Example Output

After building and flashing, the serial output is as follows:

```
01-01 00:00:01 D/example tshell: LOG_D: RT-Thread is an open source IoT operating system from China.
01-01 00:00:01 I/example tshell: LOG_I: RT-Thread is an open source IoT operating system from China.
01-01 00:00:01 W/example tshell: LOG_W: RT-Thread is an open source IoT operating system from China.
01-01 00:00:01 E/example tshell: LOG_E: RT-Thread is an open source IoT operating system from China.
LOG_RAW: RT-Thread is an open source IoT operating system from China.
D/HEX buf_dump_test: 0000-0010: 00 01 02 03 04 05 06 07  08 09 0A 0B 0C 0D 0E 0F    ................
```

## Dynamic Filtering Test

### Dynamic Filtering Commands Overview

ulog supports three runtime dynamic filtering methods:

| Command | Function | Description |
|---------|----------|-------------|
| `ulog_tag [tag]` | Filter by tag | Empty tag to cancel tag filtering |
| `ulog_lvl <level>` | Filter by level | Level value see below |
| `ulog_kw [keyword]` | Filter by keyword | Empty keyword to cancel keyword filtering |
| `ulog_filter` | View current filter settings | Display all filter conditions |

**Filtering Rule**: The above filter conditions are **simultaneously active**, and only logs that meet all filter conditions will be output.

### Running Test Commands

```bash
ulog_test
```

### Global Filter by Tag

Command format: `ulog_tag [tag]`. Empty tag to cancel tag filtering.

This filtering method filters all logs by tag. Only logs containing tag information are allowed to be output.

**Tag** is a string representing the source module/component of the log, defined in the code's `LOG_TAG` macro.

**Example**: With logs of 3 tags: `wifi.driver`, `wifi.mgnt`, `audio.driver`, when tag filter is set to `wifi`, only logs with tags `wifi.driver` and `wifi.mgnt` will be output, while logs with tag `audio.driver` will be filtered out.

Filtering principle: ulog_tag uses **prefix matching**, setting `wifi` will match all tags starting with `wifi`.

```bash
# Set tag filter, only show logs with "wifi" tag
ulog_tag wifi

# Cancel tag filter, show logs of all tags
ulog_tag
```

### Global Filter by Keyword

Command format: `ulog_kw [keyword]`. Empty keyword to cancel keyword filtering.

This filtering method filters all logs by keyword. Only logs containing keyword information are allowed to be output.

```bash
# Set keyword filter, only show logs containing "ERROR"
ulog_kw ERROR

# Cancel keyword filter
ulog_kw
```

### View Filter Information

After setting filter parameters, input `ulog_filter` command to view current filter information:

```bash
ulog_filter
```

Output example:
```
current filter info:
level: 248
tag: wifi
keyword: ERROR
```

### Level Settings When syslog is Enabled

Current example `CONFIG_ULOG_USING_SYSLOG=y` is enabled, using **bitmask** method, `ulog_lvl` parameter is the mask value:

> | Command | Mask Value | Binary | Controlled Level |
> |---------|------------|--------|------------------|
> | `ulog_lvl 128` | 128 | 10000000 | Controls LEVEL 7 (DEBUG) |
> | `ulog_lvl 64` | 64 | 01000000 | Controls LEVEL 6 (INFO) |
> | `ulog_lvl 32` | 32 | 00100000 | Controls LEVEL 5 (NOTICE) |
> | `ulog_lvl 16` | 16 | 00010000 | Controls LEVEL 4 (WARNING) |
> | `ulog_lvl 8` | 8 | 00001000 | Controls LEVEL 3 (ERROR) |

**Bit to Level Mapping** (defined in syslog.h):
- bit0=EMERG(0), bit1=ALERT(1), bit2=CRIT(2), bit3=ERR(3), bit4=WARNING(4), bit5=NOTICE(5), bit6=INFO(6), bit7=DEBUG(7)

**Filtering Logic**: `LOG_MASK(LOG_PRI(level)) & ulog.filter.level`. Only when the corresponding bit of the log level is 1 will it be displayed.

**LOG_RAW** does not go through level filtering and is always output.

### Test Steps

1. **View Current Filter Settings**
```bash
ulog_filter
```

2. **Set to Show All (syslog mode requires 255)**
```bash
ulog_lvl 255
ulog_test
```

3. **Show Only ERROR (8)**
```bash
ulog_lvl 8
ulog_test
```

4. **Show Only WARNING and Below (24)**
```bash
ulog_lvl 24
ulog_test
```

5. **Show Only INFO and Below (120)**
```bash
ulog_lvl 120
ulog_test
```

6. **Show Only DEBUG and Below (248)**
```bash
ulog_lvl 248
ulog_test
```

7. **Restore Show All (255)**
```bash
ulog_lvl 255
ulog_test
```

### Complete Interaction Log

The following is a complete dynamic filtering test process (LOG_RAW is not affected by level control and is always output):

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

## hexdump Output Usage

`ulog_hexdump` is used to output data in hexadecimal format:

```c
uint8_t buf[128];
for (int i = 0; i < sizeof(buf); i++)
{
    buf[i] = i;
}
ulog_hexdump("buf_dump_test", 16, buf, sizeof(buf));
```

Output example:
```
D/HEX buf_dump_test: 0000-0010: 00 01 02 03 04 05 06 07  08 09 0A 0B 0C 0D 0E 0F    ................
                         0010-0020: 10 11 12 13 14 15 16 17  18 19 1A 1B 1C 1D 1E 1F    ................
```

`ulog_hexdump` uses **DEBUG level (level=7)**, so when the runtime filter level is below 7, hexdump output **will not be displayed**. Only when set to `ulog_lvl 255` (syslog mode) or `ulog_lvl 7` (non-syslog mode) can you see the hexdump output.

## Using in ISR (Interrupt)

ulog supports log output in interrupt ISR, but it is not enabled by default. When using, enable `Enable ISR log` in menuconfig. The API usage is the same as in threads.

## References

- [RT-Thread ulog component documentation](https://www.rt-thread.org/document/programming/#ulog)
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)

## Change Log

| Version | Date | Release Notes |
|:---|:---|:---|
| 0.0.1 | 1/2025 | Initial version |
| 0.0.2 | 5/2025 | Improved dynamic filtering configuration description, corrected syslog mode level settings |
