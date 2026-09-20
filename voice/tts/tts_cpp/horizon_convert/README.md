# Matcha-TTS ONNX → 地平线 BPU 模型转换

将 Matcha-TTS 中文语音合成的两个 ONNX 模型转换为地平线 RDK X5 (S100P) BPU 的 .hbm 格式。

## 模型信息

| 模型 | 文件 | 大小 | 输入 | 输出 | 功能 |
|------|------|------|------|------|------|
| 声学模型 | model-steps-3.onnx | 72MB | token IDs (1,50) int64 | mel (1,80,T) float32 | 文本→梅尔频谱 |
| 声码器 | vocos-22khz-univ.onnx | 51MB | mel (1,80,500) float32 | ISTFT 分量 3×(1,513,T) | 梅尔→波形 |

声学模型输入还包含 3 个标量参数: `x_length`(int64)、`noise_scale`(float32=0.667)、`length_scale`(float32=1.0)。

## 目录结构

```
horizon_convert/
├── gen_calibration_data.py    # 校准数据生成脚本
├── acoustic_config.yaml       # 声学模型编译配置
├── vocoder_config.yaml        # 声码器编译配置
├── compile.sh                 # Docker 内编译脚本
├── verify.py                  # 精度验证脚本
├── setup_workspace.sh         # Workspace 初始化脚本
├── calibration_data/          # 校准数据 (已生成)
│   ├── acoustic_cal/
│   │   ├── x/                 # (1,50) int64 × 10
│   │   ├── x_length/          # (1,) int64 × 10
│   │   ├── noise_scale/       # (1,) float32 × 10
│   │   └── length_scale/      # (1,) float32 × 10
│   └── vocoder_cal/           # (1,80,500) float32 × 10
└── output/                    # 编译输出 (HBM 文件)
```

## 操作步骤

### 1. 准备 Docker 容器 (已完成)

```bash
# 加载工具链镜像
sudo docker load -i ai_toolchain_ubuntu_22_s100_s600_cpu_v3.7.0.tar

# 启动容器，挂载本目录为 /workspace
sudo docker run -d --network host --name xgs_OE_v3.7.0 \
  -v /path/to/oe-package:/open_explorer \
  -v $(pwd):/workspace -w /workspace \
  ai_toolchain_ubuntu_22_s100_s600_cpu:v3.7.0 sleep infinity
```

### 2. 初始化 Workspace

```bash
# 将 ONNX 模型链接到 output/onnx/
bash setup_workspace.sh /Users/shifei/Documents/code/tts_cpp/matcha-icefall-zh-baker
```

### 3. 编译模型

```bash
# 编译全部（声码器约 1-5 分钟，声学模型约 30-90 分钟）
bash compile.sh

# 或单独编译
bash compile.sh vocoder     # 先编译声码器（快）
bash compile.sh acoustic    # 再编译声学模型（慢）
bash compile.sh check       # 只检查兼容性
```

### 4. 验证

```bash
# 查看编译结果
hrt_model_exec model_info --model_file=output/acoustic_quantized_nash_m/matcha_acoustic.hbm
hrt_model_exec model_info --model_file=output/vocoder_quantized_nash_m/vocos_vocoder.hbm
```

## 已知挑战与解决方案

### 1. int64 输入 (`x` token IDs)

BPU 通常要求 float32 输入。如果编译报错，需要:
- 方案 A: 修改 ONNX 将 int64 输入转为 float32（在模型前端加 Cast 节点）
- 方案 B: 将 embedding lookup 移到 CPU，只将 embedding 后的 float 部分放到 BPU

### 2. ODE Solver 循环

Matcha-TTS 使用 3 步 ODE solver（Euler 法），在 ONNX 中可能表现为 Loop 节点或展开的网络。
- 如果是展开的: hb_compile 应该能正常处理
- 如果是 Loop 节点: 可能需要先展开循环再导出 ONNX

### 3. 声码器多输出

Vocos 输出 3 个张量 (mag, x, y)，是 ISTFT 的分量。需要在 CPU 端做 ISTFT 重建波形:
```
waveform = ISTFT(mag * x + mag * y * j)
```
其中 x=cos(phase), y=sin(phase), n_fft=1024, hop=256。

### 4. 动态形状

声学模型输出的 mel 帧数随文本长度变化（50 token → 约 416~491 帧）。
当前方案: 固定输入 50 token，mel 帧数 pad/truncate 到 500。

## 部署架构

```
文本 → [CPU: 分词+G2P+FST] → token IDs → [BPU: 声学模型] → mel → [BPU: 声码器] → ISTFT分量 → [CPU: ISTFT] → WAV
```

## 参数调整

- `noise_scale`: 控制语音随机性 (0.0=确定, 1.0=最随机, 默认 0.667)
- `length_scale`: 控制语速 (>1 慢, <1 快, 默认 1.0)
- `max_len=50`: 最大 token 数 ≈ 一句中文 (~5 秒音频)
- `MAX_MEL_LEN=500`: 固定 mel 帧数

修改后需重新生成校准数据并重新编译。
