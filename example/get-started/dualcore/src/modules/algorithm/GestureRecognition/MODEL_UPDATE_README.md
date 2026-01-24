# 手势识别模型动态更新指南

## 概述

本系统支持通过蓝牙从手机端动态更新手势识别模型,无需重新编译固件。系统会自动从 flash 文件系统加载模型,如果文件不存在则使用内置的默认模型。

## 架构说明

### 文件结构

```
GestureRecognition/
├── gesture_model_loader.h        # 模型加载器头文件
├── gesture_model_loader.c        # 模型加载器实现
├── gesture_model_config.h        # 模型配置(文件路径等)
├── gesture_detect_model_data.cc  # 默认内置模型数据
├── gesture_detect_model_data.h   # 默认模型头文件
└── main_functions.cc             # 已修改以支持动态模型加载
```

### 模型文件路径

- **Tap 检测模型**: `/model/gesture_tap.tflite`
- **Release 检测模型**: `/model/gesture_release.tflite`

### 工作流程

1. **系统启动**: 初始化模型加载器
2. **模型加载**:
   - 首先尝试从文件系统加载模型文件
   - 如果文件不存在或加载失败,则使用内置的默认模型
3. **模型运行**: TensorFlow Lite 使用加载的模型进行推理

## 使用方法

### 1. 从手机端更新模型

#### 步骤:

1. **准备模型文件**
   - 确保新模型是 TensorFlow Lite 格式 (.tflite)
   - 模型大小不超过 10KB (可在 `gesture_model_loader.h` 中调整 `MAX_MODEL_SIZE`)

2. **通过蓝牙传输**
   ```c
   // 在你的蓝牙通信代码中,接收到新模型文件后保存到指定路径

   // 示例: 保存 tap 模型
   int fd = open("/model/gesture_tap.tflite", O_WRONLY | O_CREAT | O_TRUNC | O_BINARY);
   if (fd >= 0) {
       write(fd, model_data, model_size);
       close(fd);
   }

   // 示例: 保存 release 模型
   int fd = open("/model/gesture_release.tflite", O_WRONLY | O_CREAT | O_TRUNC | O_BINARY);
   if (fd >= 0) {
       write(fd, model_data, model_size);
       close(fd);
   }
   ```

3. **重新加载模型**
   ```c
   #include "gesture_model_loader.h"

   // 重新加载所有模型
   reload_gesture_models();

   // 之后需要重新初始化 TensorFlow Lite 解释器
   init_gesture_recognition_model();
   init_gesture_recognition_release_model();
   ```

### 2. 使用文件系统同步功能

利用现有的 `bloc_filesystem` 模块:

```c
#include "bloc_filesystem.h"
#include "gesture_model_loader.h"

// 接收到新模型后,保存到 flash
void update_gesture_model(uint8_t *model_data, uint32_t model_size, bool is_tap_model)
{
    const char *path = is_tap_model ?
        "/model/gesture_tap.tflite" :
        "/model/gesture_release.tflite";

    // 确保目录存在
    mkdir("/model", 0777);

    // 保存模型文件
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY);
    if (fd >= 0) {
        ssize_t written = write(fd, model_data, model_size);
        close(fd);

        if (written == model_size) {
            LOG_I("Model saved successfully: %s", path);

            // 重新加载模型
            reload_gesture_models();

            // 重新初始化相关解释器
            if (is_tap_model) {
                init_gesture_recognition_model();
            } else {
                init_gesture_recognition_release_model();
            }
        }
    }
}
```

### 3. 验证模型是否成功加载

查看日志输出:

```
[MODEL.LOADER] Successfully loaded model from /model/gesture_tap.tflite (4152 bytes)
[MODEL.LOADER] Using dynamic tap model from file system
```

或

```
[MODEL.LOADER] Model file not found: /model/gesture_tap.tflite
[MODEL.LOADER] Using default embedded tap model
```

## API 参考

### 主要函数

#### 初始化
```c
int gesture_model_loader_init(void);
```
初始化模型加载器,在系统启动时调用一次。

#### 加载模型
```c
int load_tap_model(gesture_model_t *model);
int load_release_model(gesture_model_t *model);
```
加载指定的模型,自动尝试从文件系统加载,失败则使用默认模型。

#### 获取模型数据
```c
const unsigned char *get_tap_model_data(void);
uint32_t get_tap_model_size(void);
const unsigned char *get_release_model_data(void);
uint32_t get_release_model_size(void);
```
获取当前使用的模型数据指针和大小。

#### 重新加载模型
```c
int reload_gesture_models(void);
```
卸载当前模型并重新从文件系统加载。

#### 卸载模型
```c
int unload_model(gesture_model_t *model);
```
卸载动态加载的模型并释放内存。

## 注意事项

### 内存管理
- 动态加载的模型会占用堆内存
- 默认最大模型大小为 10KB,可根据需要调整
- 使用完毕后应及时卸载不需要的模型

### 文件系统
- 确保 `/model` 目录存在
- 模型文件必须是有效的 TensorFlow Lite 格式
- 建议在传输前后校验文件完整性(如使用 CRC)

### 错误处理
- 模型加载失败时会自动回退到默认模型
- 检查日志以确认模型来源(动态或默认)
- 如果新模型导致问题,删除文件后重启即可恢复默认模型

### 性能考虑
- 模型加载发生在初始化阶段,不影响运行时性能
- 重新加载模型需要重新初始化 TensorFlow Lite 解释器
- 建议在系统空闲时进行模型更新

## 手机端集成示例

### Android 示例

```java
// 发送模型文件到手表
public void updateGestureModel(File tfliteFile, boolean isTapModel) {
    try {
        byte[] modelData = Files.readAllBytes(tfliteFile.toPath());

        // 通过蓝牙发送模型数据
        // 协议格式: [命令字节] [模型类型] [数据长度] [模型数据]
        ByteBuffer buffer = ByteBuffer.allocate(modelData.length + 6);
        buffer.put((byte) 0xA1);  // 更新模型命令
        buffer.put(isTapModel ? (byte) 0x01 : (byte) 0x02);  // 模型类型
        buffer.putInt(modelData.length);  // 数据长度
        buffer.put(modelData);  // 模型数据

        sendDataToBluetooth(buffer.array());

    } catch (IOException e) {
        Log.e("ModelUpdate", "Failed to read model file", e);
    }
}
```

### iOS 示例

```swift
// 发送模型文件到手表
func updateGestureModel(tfliteURL: URL, isTapModel: Bool) {
    do {
        let modelData = try Data(contentsOf: tfliteURL)

        // 通过蓝牙发送模型数据
        var buffer = Data()
        buffer.append(0xA1)  // 更新模型命令
        buffer.append(isTapModel ? 0x01 : 0x02)  // 模型类型

        var length = UInt32(modelData.count).bigEndian
        buffer.append(Data(bytes: &length, count: 4))  // 数据长度
        buffer.append(modelData)  // 模型数据

        sendDataToBluetooth(buffer)

    } catch {
        print("Failed to read model file: \\(error)")
    }
}
```

## 调试技巧

### 1. 启用详细日志
在 `gesture_model_loader.c` 中设置:
```c
#define DBG_LVL DBG_LOG  // 改为 DBG_LOG 以查看详细日志
```

### 2. 检查模型文件
```c
// 在串口命令行或代码中检查
struct stat st;
if (stat("/model/gesture_tap.tflite", &st) == 0) {
    rt_kprintf("Model exists, size: %d bytes\\n", st.st_size);
} else {
    rt_kprintf("Model file not found\\n");
}
```

### 3. 测试模型加载
```c
// 测试代码
void test_model_loading(void) {
    gesture_model_loader_init();

    const unsigned char *tap_data = get_tap_model_data();
    uint32_t tap_size = get_tap_model_size();

    rt_kprintf("Tap model: %p, size: %d\\n", tap_data, tap_size);

    const unsigned char *release_data = get_release_model_data();
    uint32_t release_size = get_release_model_size();

    rt_kprintf("Release model: %p, size: %d\\n", release_data, release_size);
}
```

## 常见问题

### Q: 模型文件上传后没有生效?
A: 确保调用了 `reload_gesture_models()` 并重新初始化了解释器。

### Q: 如何确认使用的是新模型还是默认模型?
A: 查看启动日志,会显示 "Using dynamic model from file system" 或 "Using default embedded model"。

### Q: 新模型导致系统异常怎么办?
A: 删除 `/model` 目录下的模型文件并重启,系统会自动使用默认模型。

### Q: 可以同时更新两个模型吗?
A: 可以,但建议分别更新并测试,确保每个模型都正常工作。

## 版本历史

- **v1.0** (2025-01-29): 初始版本,支持动态模型加载
  - 支持 tap 和 release 两种模型
  - 自动回退到默认模型
  - 提供完整的 API

## 联系方式

如有问题或建议,请联系 Skaiwalk 软件开发团队。
