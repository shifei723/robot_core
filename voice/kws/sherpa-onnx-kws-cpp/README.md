# Sherpa-ONNX KWS + VAD (C++ 版本)

> **目标平台**: Ubuntu 22.04 ARM64 (aarch64)
> **SDK 版本**: sherpa-onnx v1.13.3 (linux-aarch64-shared-cpu)

基于 sherpa-onnx C API 的实时唤醒词检测 (KWS) + VAD 语音命令录音。

## 工作流程

1. 持续监听麦克风，用 KWS 模型检测唤醒词（默认："你好小飞"）
2. 检测到唤醒词后，切换为 VAD 模式，捕获后续语音命令
3. 静音后自动保存命令为 WAV 文件，状态归零
4. 在 `commands/kws_status` 写入状态（`0`=未唤醒，`1`=已唤醒）

## 目录结构

```
sherpa-onnx-kws-cpp/
├── CMakeLists.txt                          # CMake 构建配置
├── build.sh                                # 一键编译脚本
├── keywords_xiaofei.txt                    # 唤醒词文件
├── src/
│   ├── main.cc                             # 主程序：KWS + VAD 状态机
│   ├── audio_capture.cc                    # PortAudio 麦克风采集
│   └── wav_writer.cc                       # WAV 文件写入
├── include/
│   ├── audio_capture.h
│   └── wav_writer.h
├── sherpa-onnx-v1.13.3-linux-aarch64-shared-cpu/  # 预编译 SDK
│   ├── include/sherpa-onnx/c-api/          # C API 头文件
│   ├── lib/                                # .so 动态库
│   └── bin/                                # 预编译工具
├── model/                                  # KWS 模型文件
│   ├── encoder-*.onnx
│   ├── decoder-*.onnx
│   ├── joiner-*.onnx
│   └── tokens.txt
├── vad_model/                              # VAD 模型
│   └── silero_vad.onnx
├── build/                                  # 编译输出（自动生成）
└── commands/                               # 运行时输出（自动生成）
```

## 部署步骤

### 1. 安装系统依赖

```bash
sudo apt update
sudo apt install -y portaudio19-dev libportaudio2 cmake build-essential
```

### 2. 编译

```bash
cd sherpa-onnx-kws-cpp
chmod +x build.sh
./build.sh
```

`build.sh` 会自动检测项目内的 SDK 目录并编译。如果 SDK 在其他位置：

```bash
SHERPA_ONNX_SDK_PATH=/path/to/sdk ./build.sh
```

编译产物在 `build/kws_vad_demo`。

### 3. 运行

```bash
cd sherpa-onnx-kws-cpp

# 麦克风实时模式
./build/kws_vad_demo --keywords-file keywords_xiaofei.txt --model-dir model

# WAV 文件离线测试
./build/kws_vad_demo \
    --keywords-file model/test_wavs/test_keywords.txt \
    --wav model/test_wavs/3.wav \
    --model-dir model
```

### 4. 后台运行（可选）

```bash
nohup ./build/kws_vad_demo \
    --keywords-file keywords_xiaofei.txt \
    --model-dir model \
    > /tmp/kws.log 2>&1 &
```

## 运行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--keywords-file` | `keywords_xiaofei.txt` | 唤醒词文件 |
| `--model-dir` | `model` | 模型目录 |
| `--wav` | _(空)_ | 指定 WAV 文件则离线处理 |
| `--vad-model` | `vad_model/silero_vad.onnx` | VAD 模型路径 |
| `--output-dir` | `commands` | 输出目录 |
| `--keywords-threshold` | `0.25` | KWS 检测阈值 |
| `--keywords-score` | `3.0` | KWS 关键词得分 |
| `--vad-threshold` | `0.5` | VAD 语音阈值 |
| `--vad-min-silence-duration` | `0.5` | 最短静音时长(秒) |
| `--vad-min-speech-duration` | `0.25` | 最短语音时长(秒) |
| `--num-threads` | `2` | 推理线程数 |
| `--sample-rate` | `16000` | 采样率 |
| `--max-command-duration` | `30.0` | 命令最大时长(秒) |

## 输出文件

- `commands/latest_command.wav` — 最新一条语音命令（16-bit PCM, 16kHz, 单声道）
- `commands/kws_status` — 当前状态（`0`=未唤醒，`1`=已唤醒监听中）

## 自定义唤醒词

编辑 `keywords_xiaofei.txt`，格式为拼音 + 中文标注：

```
n ǐ h ǎo x iǎo f ēi @你好小飞
```

如需添加更多唤醒词，每行一个，格式相同。

## 常见问题

**Q: 编译报错找不到 PortAudio**
```bash
sudo apt install -y portaudio19-dev
```

**Q: 运行时报错 `libonnxruntime.so: cannot open shared object file`**
确保从项目根目录运行，或设置 `LD_LIBRARY_PATH`：
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/sherpa-onnx-v1.13.3-linux-aarch64-shared-cpu/lib
```

**Q: 报错 `No default input device found`**
检查麦克风是否已连接并被系统识别：
```bash
arecord -l   # 列出录音设备
```
# Sherpa-ONNX KWS + VAD (C++ 版本)

基于 sherpa-onnx C++ API 的实时唤醒词检测 + VAD 命令录音。

## 前置条件

### macOS (ARM64)
```bash
brew install portaudio cmake
```

### Ubuntu 22.04 (ARM64)
```bash
sudo apt install -y portaudio19-dev libportaudio2 cmake build-essential
```

## 构建步骤

```bash
cd sherpa-onnx-kws-cpp
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

CMake 会自动下载 sherpa-onnx 预编译 SDK（根据平台自动选择 macOS ARM64 / Linux ARM64 / Linux x64）。

## 运行

```bash
# 使用麦克风
./kws_vad_demo --keywords-file ../keywords_xiaofei.txt --model-dir ../model

# 使用 WAV 文件测试
./kws_vad_demo --keywords-file ../model/test_wavs/test_keywords.txt --wav ../model/test_wavs/3.wav --model-dir ../model
```

## 输出

- `commands/latest_command.wav`：最新一条命令音频
- `commands/kws_status`：状态文件（0=未唤醒，1=已唤醒）

## 目录结构

```
sherpa-onnx-kws-cpp/
├── CMakeLists.txt
├── README.md
├── src/
│   ├── main.cc           # 主程序：KWS + VAD + 状态机
│   ├── audio_capture.h   # 音频采集接口
│   ├── audio_capture.cc  # PortAudio 实现
│   ├── wav_writer.h      # WAV 写入接口
│   └── wav_writer.cc     # WAV 实现
├── build/                # 构建输出（git 忽略）
├── commands/             # 运行时输出（git 忽略）
├── model/                # KWS 模型（从 Python 版本复制）
└── vad_model/            # VAD 模型（从 Python 版本复制）
```
