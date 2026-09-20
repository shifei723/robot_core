# Matcha TTS C++ 版

基于 sherpa-onnx C API 的中文语音合成命令行工具。

## 快速开始

```bash
bash build.sh          # 自动下载 SDK + 编译
./build/tts "你好世界"  # 生成 tts_output.wav
```

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-o, --output` | tts_output.wav | 输出文件 |
| `-s, --speed` | 1.0 | 语速 |
| `-m, --model-dir` | ./matcha-icefall-zh-baker | 模型目录 |
| `-v, --vocoder` | 自动 | 声码器路径 |
| `-t, --num-threads` | 2 | 线程数 |
| `-d, --debug` | 关闭 | 调试模式 |

## 部署到 Ubuntu ARM64

```bash
scp -r tts_cpp sunrise@192.168.0.108:/data/sf_code/tts_cpp
ssh sunrise@192.168.0.108 "cd /data/sf_code/tts_cpp && bash build.sh"
# Linux 运行: ./build/run.sh "文本"
```

## 目录结构

```
tts_cpp/
├── tts.cpp             # C++ 源码
├── CMakeLists.txt      # 构建配置
├── build.sh            # 编译脚本
├── README.md
├── matcha-icefall-zh-baker/  # 模型文件
│   ├── model-steps-3.onnx    (72MB)
│   ├── vocos-22khz-univ.onnx (51MB)
│   ├── lexicon.txt, tokens.txt
│   ├── phone.fst, date.fst, number.fst
│   └── dict/
├── build/              # 编译输出 (build.sh 创建)
│   ├── tts             # 可执行文件
│   └── run.sh          # Linux 运行脚本
└── sherpa-onnx-sdk/    # SDK (build.sh 下载)
```
