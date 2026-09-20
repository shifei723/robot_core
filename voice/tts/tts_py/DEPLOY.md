# Matcha TTS 部署文档

目标环境：Ubuntu 22.04 ARM64
部署路径：/data/sf_code/tts

## 1. 环境准备

```bash
# 安装系统依赖
sudo apt update && sudo apt install -y python3 python3-pip python3-venv

# 创建虚拟环境
cd /data/sf_code/tts
python3 -m venv venv
source venv/bin/activate

# 安装 Python 依赖
pip install sherpa-onnx soundfile
```

## 2. 模型文件

模型文件需放在 /data/sf_code/tts/ 下，目录结构如下：

```
/data/sf_code/tts/
├── venv/                          # Python 虚拟环境
├── tts.py                         # TTS 脚本
├── tts.sh                         # 快捷脚本
├── matcha-icefall-zh-baker/       # TTS 模型目录
│   ├── model-steps-3.onnx         # 声学模型 (~72MB)
│   ├── vocos-22khz-univ.onnx      # 声码器 (~51MB)
│   ├── lexicon.txt
│   ├── tokens.txt
│   ├── phone.fst
│   ├── date.fst
│   ├── number.fst
│   └── dict/                      # jieba 词典目录
│       ├── hmm_model.utf8
│       ├── idf.utf8
│       ├── jieba.dict.utf8
│       ├── pos_dict/
│       ├── stop_words.utf8
│       └── user.dict.utf8
└── DEPLOY.md                      # 本文档
```

### 模型获取

如果服务器上还没有模型文件，可通过以下方式下载：

```bash
cd /data/sf_code/tts

# 方法一：从 macOS 开发机 scp 传输（推荐，已在本地准备好）
scp -r /Users/shifei/Documents/code/sherpa-onnx/matcha-icefall-zh-baker sunrise@192.168.0.108:/data/sf_code/tts/

# 方法二：在服务器上直接下载
# 声学模型（从 HuggingFace 镜像）
git clone https://hf-mirror.com/csukuangfj/matcha-icefall-zh-baker
cd matcha-icefall-zh-baker
# LFS 大文件需单独下载
GIT_LFS_SKIP_SMUDGE=1 git restore --source=HEAD :/
curl -L -o model-steps-3.onnx "https://hf-mirror.com/csukuangfj/matcha-icefall-zh-baker/resolve/main/model-steps-3.onnx"
cd ..

# 声码器（从 GitHub，可能较慢）
curl -L -o matcha-icefall-zh-baker/vocos-22khz-univ.onnx \
  "https://github.com/k2-fsa/sherpa-onnx/releases/download/vocoder-models/vocos-22khz-univ.onnx"
```

## 3. 运行 TTS

### 激活环境

```bash
cd /data/sf_code/tts
source venv/bin/activate
```

### 基本用法

```bash
# 合成语音（默认输出到 tts_output.wav）
python3 tts.py "你好，欢迎使用语音合成系统"

# 指定输出文件和语速
python3 tts.py --output=hello.wav --speed=1.0 "100人参加活动"

# 使用快捷脚本
./tts.sh "今天天气不错" output.wav
```

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| text | (必填) | 要合成的文本 |
| --output / -o | tts_output.wav | 输出 WAV 路径 |
| --speed | 1.0 | 语速，>1 更快，<1 更慢 |
| --num-threads | 2 | 推理线程数 |
| --model-dir | ./matcha-icefall-zh-baker | 模型目录 |
| --vocoder | ./matcha-icefall-zh-baker/vocos-22khz-univ.onnx | 声码器路径 |
| --debug | false | 显示调试信息 |

### 数字朗读规则

模型通过 number.fst 规则自动处理数字读法：

- 电话号码：110 → "幺幺零"，18920240511 → "幺八九二零二四零五一一"
- 日期：2024年12月31号 → "二零二四年十二月三十一号"
- 金额：123456块钱 → "十二万三千四百五十六块钱"
- 普通数字：根据上下文自动选择读法

## 4. 集成到唤醒词流水线

在你的 KWS 唤醒词检测到唤醒词后，可调用 TTS 生成回复语音：

```python
import subprocess

def tts_speak(text: str, output_path: str = "/tmp/tts_reply.wav"):
    """调用 TTS 合成语音"""
    subprocess.run([
        "python3", "/data/sf_code/tts/tts.py",
        "--output", output_path,
        "--speed", "1.0",
        text
    ], check=True)
    return output_path

# 示例：唤醒后回复
tts_speak("你好，请问有什么可以帮您？")
```

## 5. 故障排除

### sherpa-onnx 安装失败

```bash
# 确认 Python 版本
python3 --version  # 需要 >= 3.8

# 升级 pip
pip install --upgrade pip

# 重新安装
pip install --force-reinstall sherpa-onnx soundfile
```

### 模型加载失败

```bash
# 检查模型文件完整性
ls -lh matcha-icefall-zh-baker/model-steps-3.onnx  # 应约 72MB
ls -lh matcha-icefall-zh-baker/vocos-22khz-univ.onnx  # 应约 51MB
file matcha-icefall-zh-baker/model-steps-3.onnx  # 应为 "data"

# 如果 model-steps-3.onnx 只有几百字节，说明是 LFS 指针文件
# 需要重新下载实际文件（见第 2 节方法二）
```

### 生成速度慢

```bash
# 增加线程数
python3 tts.py --num-threads=4 "你的文本"

# ARM64 上 72MB 模型首次加载约 2-3 秒，后续生成 RTF 约 0.3-0.5
```
