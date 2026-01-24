# 文件同步协议说明

## 概述

本系统使用三个 KEY 来实现文件传输功能:
- `KEY_START_SYNC_FILE` (0x52): 开始文件传输
- `KEY_SYNC_FILE` (0x53): 传输文件数据
- `KEY_END_SYNC_FILE` (0x54): 结束文件传输

## 协议格式

### 1. KEY_START_SYNC_FILE (0x52)

开始文件传输,包含文件路径和总大小。

**数据格式:**
```
[路径长度(1字节)][文件路径(N字节)][文件总大小(4字节,大端)]
```

**示例:**
```
路径: "/model/gesture_tap.tflite"
大小: 4152 bytes

数据包:
[0x1B] [/model/gesture_tap.tflite] [0x00] [0x00] [0x10] [0x38]
 ^^^^   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^  ^^^^^^^^^^^^^^^^^^^^^^
路径长度         文件路径                    文件大小(4152)
```

### 2. KEY_SYNC_FILE (0x53)

传输文件数据块。

**数据格式:**
```
[文件数据...]
```

**说明:**
- 每个数据包最多可包含 BLE MTU - 协议头 的数据
- 建议每包 200-400 字节
- 多次调用直到传输完所有数据

### 3. KEY_END_SYNC_FILE (0x54)

结束文件传输并验证。

**数据格式:**
```
无数据
```

**说明:**
- 系统会验证接收的数据大小是否等于声明的总大小
- 如果是手势模型文件,会自动重新加载模型

## 特殊处理: 手势模型文件

如果接收的文件路径包含以下字符串,系统会自动识别为手势模型文件:
- `gesture_tap.tflite`: Tap 检测模型
- `gesture_release.tflite`: Release 检测模型

接收完成后,系统会:
1. 重新加载模型
2. 重新初始化 TensorFlow Lite 解释器
3. 新模型立即生效

## Flutter 示例代码

### 文件传输管理器

```dart
import 'dart:typed_data';
import 'dart:async';
import 'package:flutter/services.dart';

class FileTransferManager {
  static const int KEY_START_SYNC_FILE = 0x52;
  static const int KEY_SYNC_FILE = 0x53;
  static const int KEY_END_SYNC_FILE = 0x54;
  static const int CHUNK_SIZE = 200; // 每包大小

  // BLE 通信服务 (需要根据你的实际 BLE 实现修改)
  final BleService _bleService;

  // 进度回调
  final Function(int progress)? onProgress;

  FileTransferManager(this._bleService, {this.onProgress});

  /// 发送文件到手表
  /// [filePath] 手表上的目标路径 (如 "/model/gesture_tap.tflite")
  /// [fileData] 文件数据
  Future<bool> sendFileToWatch(String filePath, Uint8List fileData) async {
    try {
      print('[FileTransfer] Starting file transfer: $filePath (${fileData.length} bytes)');

      // 1. 发送开始包
      await _sendStartPacket(filePath, fileData.length);
      await Future.delayed(Duration(milliseconds: 50));

      // 2. 分块发送文件数据
      int offset = 0;
      while (offset < fileData.length) {
        final chunkSize = (fileData.length - offset) < CHUNK_SIZE
            ? (fileData.length - offset)
            : CHUNK_SIZE;

        final chunk = fileData.sublist(offset, offset + chunkSize);
        await _sendDataChunk(chunk);

        offset += chunkSize;

        // 更新进度
        final progress = (offset * 100 / fileData.length).round();
        onProgress?.call(progress);
        print('[FileTransfer] Progress: $progress%');

        // 短暂延时避免数据溢出
        await Future.delayed(Duration(milliseconds: 20));
      }

      // 3. 发送结束包
      await _sendEndPacket();
      await Future.delayed(Duration(milliseconds: 100));

      print('[FileTransfer] File sent successfully: $filePath');
      return true;

    } catch (e) {
      print('[FileTransfer] Failed to send file: $e');
      return false;
    }
  }

  /// 发送开始包
  Future<void> _sendStartPacket(String filePath, int fileSize) async {
    final pathBytes = Uint8List.fromList(filePath.codeUnits);
    final pathLen = pathBytes.length;

    final buffer = ByteData(1 + pathLen + 4);

    // 路径长度
    buffer.setUint8(0, pathLen);

    // 文件路径
    for (int i = 0; i < pathLen; i++) {
      buffer.setUint8(1 + i, pathBytes[i]);
    }

    // 文件大小 (大端)
    buffer.setUint32(1 + pathLen, fileSize, Endian.big);

    await _sendCommand(KEY_START_SYNC_FILE, buffer.buffer.asUint8List());
  }

  /// 发送数据块
  Future<void> _sendDataChunk(Uint8List chunk) async {
    await _sendCommand(KEY_SYNC_FILE, chunk);
  }

  /// 发送结束包
  Future<void> _sendEndPacket() async {
    await _sendCommand(KEY_END_SYNC_FILE, Uint8List(0));
  }

  /// 发送命令到手表
  Future<void> _sendCommand(int key, Uint8List data) async {
    // 通过 BLE 发送数据
    // 具体实现取决于你的 BLE 通信层
    await _bleService.sendData(key, data);
  }
}
```

### BLE 服务接口 (示例)

```dart
/// BLE 通信服务接口
/// 你需要根据实际使用的 BLE 库实现这个接口
abstract class BleService {
  Future<void> sendData(int key, Uint8List data);
}

/// 使用 flutter_blue_plus 的实现示例
class BleServiceImpl implements BleService {
  final BluetoothCharacteristic _characteristic;

  BleServiceImpl(this._characteristic);

  @override
  Future<void> sendData(int key, Uint8List data) async {
    // 构建完整的数据包
    // 格式: [协议头...][key][data...]
    // 根据你的协议格式调整
    final packet = Uint8List.fromList([
      // 你的协议头
      0x01, 0x02, // 示例: 协议头
      key,        // KEY
      ...data,    // 数据
    ]);

    await _characteristic.write(packet);
  }
}
```

### 使用示例 - 发送模型文件

```dart
import 'package:flutter/material.dart';
import 'package:flutter/services.dart' show rootBundle;

class ModelUpdatePage extends StatefulWidget {
  @override
  _ModelUpdatePageState createState() => _ModelUpdatePageState();
}

class _ModelUpdatePageState extends State<ModelUpdatePage> {
  double _progress = 0.0;
  bool _isTransferring = false;
  String _statusMessage = '准备就绪';

  late FileTransferManager _fileTransferManager;

  @override
  void initState() {
    super.initState();

    // 初始化文件传输管理器
    _fileTransferManager = FileTransferManager(
      yourBleService, // 你的 BLE 服务实例
      onProgress: (progress) {
        setState(() {
          _progress = progress / 100.0;
          _statusMessage = '传输中: $progress%';
        });
      },
    );
  }

  /// 从 assets 加载模型并发送
  Future<void> _sendModelFromAssets(String assetPath, String watchPath) async {
    setState(() {
      _isTransferring = true;
      _progress = 0.0;
      _statusMessage = '正在加载模型文件...';
    });

    try {
      // 从 assets 读取模型文件
      final ByteData data = await rootBundle.load(assetPath);
      final Uint8List fileData = data.buffer.asUint8List();

      setState(() {
        _statusMessage = '开始传输...';
      });

      // 发送到手表
      final success = await _fileTransferManager.sendFileToWatch(
        watchPath,
        fileData,
      );

      setState(() {
        _isTransferring = false;
        _statusMessage = success ? '传输成功!' : '传输失败';
        if (success) {
          _progress = 1.0;
        }
      });

      // 显示结果
      _showResultDialog(success);

    } catch (e) {
      setState(() {
        _isTransferring = false;
        _statusMessage = '错误: $e';
      });

      _showResultDialog(false);
    }
  }

  /// 从文件选择器选择文件并发送
  Future<void> _sendModelFromFile() async {
    // 使用 file_picker 包选择文件
    // final result = await FilePicker.platform.pickFiles(
    //   type: FileType.custom,
    //   allowedExtensions: ['tflite'],
    // );

    // if (result != null && result.files.isNotEmpty) {
    //   final file = File(result.files.single.path!);
    //   final fileData = await file.readAsBytes();
    //
    //   await _fileTransferManager.sendFileToWatch(
    //     '/model/gesture_tap.tflite',
    //     fileData,
    //   );
    // }
  }

  void _showResultDialog(bool success) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text(success ? '成功' : '失败'),
        content: Text(
          success
              ? '模型已成功上传到手表'
              : '模型上传失败,请重试',
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: Text('确定'),
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('模型更新'),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            // 进度显示
            Card(
              child: Padding(
                padding: const EdgeInsets.all(16.0),
                child: Column(
                  children: [
                    Text(
                      _statusMessage,
                      style: TextStyle(fontSize: 16),
                      textAlign: TextAlign.center,
                    ),
                    SizedBox(height: 16),
                    LinearProgressIndicator(
                      value: _progress,
                      backgroundColor: Colors.grey[300],
                      valueColor: AlwaysStoppedAnimation<Color>(
                        _isTransferring ? Colors.blue : Colors.green,
                      ),
                    ),
                    SizedBox(height: 8),
                    Text(
                      '${(_progress * 100).toStringAsFixed(0)}%',
                      style: TextStyle(
                        fontSize: 24,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                  ],
                ),
              ),
            ),

            SizedBox(height: 32),

            // 更新 Tap 模型按钮
            ElevatedButton.icon(
              onPressed: _isTransferring
                  ? null
                  : () => _sendModelFromAssets(
                        'assets/models/gesture_tap.tflite',
                        '/model/gesture_tap.tflite',
                      ),
              icon: Icon(Icons.touch_app),
              label: Text('更新 Tap 模型'),
              style: ElevatedButton.styleFrom(
                padding: EdgeInsets.all(16),
              ),
            ),

            SizedBox(height: 16),

            // 更新 Release 模型按钮
            ElevatedButton.icon(
              onPressed: _isTransferring
                  ? null
                  : () => _sendModelFromAssets(
                        'assets/models/gesture_release.tflite',
                        '/model/gesture_release.tflite',
                      ),
              icon: Icon(Icons.pan_tool),
              label: Text('更新 Release 模型'),
              style: ElevatedButton.styleFrom(
                padding: EdgeInsets.all(16),
              ),
            ),

            SizedBox(height: 16),

            // 从文件选择器选择按钮
            OutlinedButton.icon(
              onPressed: _isTransferring ? null : _sendModelFromFile,
              icon: Icon(Icons.folder_open),
              label: Text('从文件选择...'),
              style: OutlinedButton.styleFrom(
                padding: EdgeInsets.all(16),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
```

### pubspec.yaml 依赖

```yaml
dependencies:
  flutter:
    sdk: flutter

  # BLE 通信 (选择一个)
  flutter_blue_plus: ^1.30.0  # 推荐
  # 或
  # flutter_reactive_ble: ^5.0.3

  # 文件选择器 (可选)
  file_picker: ^6.1.1

flutter:
  assets:
    - assets/models/gesture_tap.tflite
    - assets/models/gesture_release.tflite
```

## 日志输出

### 正常流程

```
[FileTransfer] Starting file transfer: /model/gesture_tap.tflite (4152 bytes)
[FileTransfer] Progress: 4%
[FileTransfer] Progress: 9%
...
[FileTransfer] Progress: 100%
[FileTransfer] File sent successfully: /model/gesture_tap.tflite

手表端:
[commu.parse.notify] Start receiving file: /model/gesture_tap.tflite (4152 bytes)
[bloc.filesystem] Started receiving file: /model/gesture_tap.tflite (4152 bytes)
[bloc.filesystem] Received 4152/4152 bytes (100%)
[bloc.filesystem] File receive completed: /model/gesture_tap.tflite (4152 bytes)
[commu.parse.notify] Gesture model file received, reloading models
[MODEL.LOADER] Successfully loaded model from /model/gesture_tap.tflite (4152 bytes)
```

## 注意事项

1. **文件路径**: 必须使用绝对路径,如 `/model/gesture_tap.tflite`
2. **分块大小**: 建议 200-400 字节,根据 BLE MTU 调整
3. **传输速度**: 每个数据包之间建议延时 20ms,避免缓冲区溢出
4. **错误处理**: 如果传输失败,手表会保留原有的模型文件
5. **模型验证**: 确保传输的 .tflite 文件是有效的 TensorFlow Lite 模型
6. **BLE 连接**: 确保在传输前已建立稳定的 BLE 连接

## 测试建议

1. **小文件测试**: 先传输一个小文本文件测试流程
2. **分块测试**: 验证不同分块大小的传输
3. **中断测试**: 测试传输中断后的恢复机制
4. **模型验证**: 传输后验证模型文件的完整性
5. **性能测试**: 测量传输速度和成功率

## 故障排除

### 问题: 文件传输失败

**解决方案:**
1. 检查文件路径是否正确
2. 确保目录存在或系统能自动创建
3. 验证文件大小是否正确
4. 检查 BLE 连接稳定性
5. 增加数据包之间的延时

### 问题: 模型未生效

**解决方案:**
1. 检查文件路径是否包含 `gesture_tap.tflite` 或 `gesture_release.tflite`
2. 验证模型文件是否有效
3. 查看日志确认模型是否重新加载
4. 重启手势识别功能

### 问题: 传输速度慢

**解决方案:**
1. 增大 CHUNK_SIZE (不超过 BLE MTU)
2. 减少数据包之间的延时
3. 优化 BLE 连接参数

## 版本历史

- **v1.0** (2025-01-29): 初始版本
  - 支持通用文件传输
  - 自动识别和重新加载手势模型
  - 提供 Flutter 实现示例
