# Matcha TTS - 中文语音合成

基于 sherpa-onnx + Matcha 声学模型 + Vocos 声码器的中文 TTS 系统。

## 快速开始

```bash
# 1. 初始化环境（仅首次）
bash setup.sh

# 2. 日常使用
source venv/bin/activate
python3 tts.py "你好，欢迎使用语音合成"

# 或使用快捷脚本
./tts.sh "你好世界" hello.wav
```

## 目录结构

```
TTS/
├── setup.sh              # 环境初始化脚本
├── tts.py                # TTS 主脚本
├── tts.sh                # 快捷调用脚本
├── tts_module.py         # Python 模块（供其他脚本导入）
├── DEPLOY.md             # 详细部署文档
├── README.md             # 本文件
├── venv/                 # Python 虚拟环境（setup.sh 创建）
└── matcha-icefall-zh-baker/  # 模型文件
    ├── model-steps-3.onnx     # 声学模型
    ├── vocos-22khz-univ.onnx  # 声码器
    ├── lexicon.txt, tokens.txt
    ├── phone.fst, date.fst, number.fst
    └── dict/                   # 分词词典
```

## 部署到 Ubuntu

将整个 TTS 目录复制到目标服务器，运行 `bash setup.sh` 即可。

```bash
scp -r /Users/shifei/Documents/code/TTS sunrise@192.168.0.108:/data/sf_code/tts
ssh sunrise@192.168.0.108 "cd /data/sf_code/tts && bash setup.sh"
```

## 数字朗读

模型通过 FST 规则自动处理数字：
- 110 → "幺幺零"（电话号码）
- 2024年12月31号 → "二零二四年十二月三十一号"（日期）
- 123456块钱 → "十二万三千四百五十六块钱"（金额）
