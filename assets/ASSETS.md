# ASSETS —— 运行资产清单（Git LFS 携带 / 未进库）

本仓库源码可直接在**新机器**上编译。资产分两类：

- **已随仓库携带（Git LFS）**：语音模型与运行库（`*.onnx/*.hbm/*.gguf/*.bin`、`voice/` 下预编译库、
  sherpa SDK 命令行工具），见 `.gitattributes`。clone 后须 `git lfs install && git lfs pull`
  才能取到真实文件，否则只是指针文件。
- **仍未进库**（按下表补齐）：地图 `*.db/*.ply/*.pcd`、演示媒体、实机建图产物、仓库外的多模态模型。

用 `bash assets/verify_assets.sh` 一键体检。

## 1. 模型资产（放在仓库外，默认 `/data/models`，可用 `ROBOT_MODELS` 覆盖）

| 用途 | 目录 | 关键文件 | 获取方式 |
|---|---|---|---|
| 多模态 omni（voice/agent） | `$ROBOT_MODELS/Qwen2.5-Omni-3B/` | `*.hbm` ×3、`embed_tokens.bin`、`config/` | 从旧机 `/data/models` 拷贝 |
| ASR（可选） | `$ROBOT_MODELS/{Hobot_ASR,Whisper_ASR,Paraformer}/` | `*.hbm` | 同上 |
| 视觉语言（可选） | `$ROBOT_MODELS/{InternVL2_5_1B,InternVL3_2B_Instruct}/` | `*.gguf/*.hbm` | 同上 |
| TTS（可选） | `$ROBOT_MODELS/TTS/`、`voice/tts/tts_py/matcha-icefall-zh-baker/` | 模型/词典 | 同上 |

> `voice/agent/.../config/omni_params.yaml`、`llm_sdk/.../omni_offline_config.json`
> 内以 `/data/models/...` 绝对路径引用。若模型不放 `/data/models`，
> 改这两个配置文件里的前缀（C++/SDK 直读，不支持环境变量展开）。

## 2. KWS 唤醒模型（在 `voice/kws/` 内，已随仓库 LFS 入库）

- `voice/kws/sherpa-onnx-kws/model/`、`voice/kws/sherpa-onnx-kws-cpp/model/`：
  `encoder/decoder/joiner.onnx`（含 int8）、`tokens.txt`、`keywords.txt`
- `*/vad_model/silero_vad.onnx`、sherpa SDK 运行库
  `lib/{libonnxruntime,libsherpa-onnx-c-api,libsherpa-onnx-cxx-api}.so`
- 均随仓库携带，`git lfs pull` 后即可运行；缺失时可从旧机备份拷贝，
  或从 sherpa-onnx 官方下载 `zipformer-wenetspeech-3.3M` 重新生成关键词。

## 3. 地图 / 定位资产（运行时产物，需在新环境**重新建图**或从旧机拷贝）

| 文件 | 说明 |
|---|---|
| `data/rtabmap_maps/*.db`（`rtabmap_vins.db` 等） | rtabmap 视觉/RGB-D 地图数据库 |
| `data/rtabmap_maps/*_cloud.ply` | 点云地图 |
| `localization/.../FAST_LIO_LOCALIZATION_HUMANOID/data/map.pcd` | FAST-LIO 重定位先验地图 |
| `data/vins_output/` | VINS 轨迹/外参输出（`vio.csv` 由建图生成） |

## 4. 构建阻塞的第三方预编译库（⚠️ 被 `*.so`/`*.a` 排除，缺失则对应包无法编译）

以下库不是纯运行时资产，**编译阶段就要用到**（新机器需从旧机或 SDK 发行包恢复，之后 `verify_assets.sh` 会校验）：

| 目标包 | 需要的库（组）| 源位置 |
|---|---|---|
| `omni_node` (agent_ws) | `voice/llm_sdk/.../oellm_runtime/lib/*.so`（含 `libxlm.so`）| 地平线 LLM SDK 发行包 |
| `unitree_lidar_*` (ros_ws) | `sensors/lidar/unitree_lidar/unitree_lidar_sdk/lib/{aarch64,x86_64}/libunilidar_sdk2.a` | unilidar_sdk2 发布 |
| `hobot_tts` (tts) | `voice/tts/hobot_tts/wetts/lib/{libtts.so,libonnxruntime.so*}` | wetts/TTS 依赖 |
| `tts_cpp` (tts) | `voice/tts/tts_cpp/sherpa-onnx-sdk/lib/{libsherpa-onnx-c-api.so,libonnxruntime.so}` | sherpa-onnx 预编译 |

> 建议：若这些库不大且固定，可在 `.gitignore` 里为它们加**白名单例外**（`!path/*.so`）直接随仓库携带；否则提供一键拷贝脚本从 SDK 包恢复。

## 5. 已随源码保留（标为 OK 仅供参考）

- Hesai 角度/火时校正 `csv`（`sensors/lidar/HesaiLidar_ROS_2.0/.../correction/`）
- `oellm_runtime/include`（LLM SDK 头文件）
- `unitree_lidar/unitree_lidar_sdk/bin/*`（⚠️ **aarch64 预编译 ELF**，内含旧机绝对路径字符串；换架构需重编）

## 6. 系统级依赖（非本仓库，需在新机安装）

见 `assets/PREREQUISITES.md`。
