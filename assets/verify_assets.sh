#!/usr/bin/env bash
# ============================================================
# assets/verify_assets.sh —— 校验新机器上运行所需的“未进版本库”资产
#
# 用法:  bash assets/verify_assets.sh
#        (可选)  ROBOT_MODELS=/path/to/models bash assets/verify_assets.sh
#
# 大二进制(模型 *.hbm/*.gguf、地图 *.db/*.pcd、kws *.onnx、tts 模型、
# 预编译 *.so)被 .gitignore 排除，本脚本逐项检查并给出缺失清单。
#   OK   = 已存在
#   MISS = 运行相关功能会受影响，需从旧机/官方获取
#   OPT  = 可选项，缺失只影响部分功能
# ============================================================
set -u
RC="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
export ROBOT_CORE="${ROBOT_CORE:-$RC}"
: "${ROBOT_MODELS:=$ROBOT_CORE/../models}"
[ -d "$ROBOT_MODELS" ] || ROBOT_MODELS="/data/models"

miss=0; warn=0
ck() {  # ck <OK|MISS|OPT> <说明> <路径>
  local level="$1" desc="$2" path="$3"
  if [ -e "$path" ]; then
    printf '  [OK]   %-42s %s\n' "$desc" "$path"
  else
    if [ "$level" = MISS ]; then
      printf '  [MISS] %-42s %s\n' "$desc" "$path"; miss=$((miss+1))
    else
      printf '  [OPT]  %-42s %s\n' "$desc" "$path"; warn=$((warn+1))
    fi
  fi
}

echo "ROBOT_CORE  = $ROBOT_CORE"
echo "ROBOT_MODELS= $ROBOT_MODELS"
echo

echo "## 端侧大模型 / 多模态 (voice/agent, llm_sdk)"
ck MISS "Qwen2.5-Omni Visual.hbm" "$ROBOT_MODELS/Qwen2.5-Omni-3B/Qwen2.5_Omni_3B_Visual.hbm"
ck MISS "Qwen2.5-Omni Audio.hbm"   "$ROBOT_MODELS/Qwen2.5-Omni-3B/Qwen2.5_Omni_3B_Audio.hbm"
ck MISS "Qwen2.5-Omni Text.hbm"    "$ROBOT_MODELS/Qwen2.5-Omni-3B/Qwen2.5_Omni_3B_Text.hbm"
ck MISS "embed_tokens.bin"         "$ROBOT_MODELS/Qwen2.5-Omni-3B/embed_tokens.bin"
ck MISS "tokenizer/config 目录"    "$ROBOT_MODELS/Qwen2.5-Omni-3B/config"
ck OPT  "oellm_runtime 预编译库"   "$ROBOT_CORE/voice/llm_sdk/D-Robotics_LLM_S100_1.0.0_SDK/oellm_runtime/lib"

echo
echo "## 唤醒 KWS (voice/kws，模型走 Git LFS，需 git lfs pull)"
ck MISS "KWS 模型目录"             "$ROBOT_CORE/voice/kws/sherpa-onnx-kws/model"
ck OPT  "sherpa-onnx 运行库(cpp)"  "$ROBOT_CORE/voice/kws/sherpa-onnx-kws-cpp/sherpa-onnx-v1.13.3-linux-aarch64-shared-cpu"

echo
echo "## 语音合成 TTS (voice/tts，模型走 Git LFS，需 git lfs pull)"
ck OPT  "matcha 中文声学模型"      "$ROBOT_CORE/voice/tts/tts_py/matcha-icefall-zh-baker"

echo
case "$(uname -m)" in
  aarch64|arm64) _O3D="$ROBOT_CORE/third/open3d141" ;;
  *)             _O3D="$ROBOT_CORE/third/open3d141_x86" ;;
esac
echo "## Open3D 预编译版（不入库，$(uname -m)，bash assets/fetch_open3d.sh）"
ck OPT "Open3D 1.4.1" "$_O3D"

echo
echo "## 地图 / 定位资产 (*.db/*.ply/*.pcd 被排除，需实机建图生成)"
ck OPT  "FAST_LIO 重定位地图 pcd"  "$ROBOT_CORE/localization/lidar_localization/FAST_LIO_LOCALIZATION_HUMANOID/data/map.pcd"
ck OPT  "rtabmap 视觉地图 db"      "$ROBOT_CORE/data/rtabmap_maps/rtabmap_vins.db"

echo
echo "## 传感器标定/校正文件（通常随源码在库内）"
ck OK   "Hesai 角度校正目录"       "$ROBOT_CORE/sensors/lidar/HesaiLidar_ROS_2.0/src/driver/HesaiLidar_SDK_2.0/correction"
ck OK   "VINS 外参 csv"            "$ROBOT_CORE/data/vins_output/extrinsic_parameter.csv"

echo
echo "## 构建必需的第三方预编译库 (*.so/*.a 被排除, 缺失则对应包编译失败)"
ck MISS "omni_node: oellm libxlm.so"        "$ROBOT_CORE/voice/llm_sdk/D-Robotics_LLM_S100_1.0.0_SDK/oellm_runtime/lib/libxlm.so"
ck MISS "unitree: libunilidar_sdk2.a"       "$ROBOT_CORE/sensors/lidar/unitree_lidar/unitree_lidar_sdk/lib/aarch64/libunilidar_sdk2.a"
ck MISS "hobot_tts: libtts.so"              "$ROBOT_CORE/voice/tts/hobot_tts/wetts/lib/libtts.so"
ck MISS "tts_cpp: libsherpa-onnx-c-api.so"  "$ROBOT_CORE/voice/tts/tts_cpp/sherpa-onnx-sdk/lib/libsherpa-onnx-c-api.so"

echo
echo "## 系统级前置（非本仓库，需 apt/手动装；缺失则对应包无法编译）"
if ls /usr/lib/aarch64-linux-gnu/librealsense2.so* /usr/lib/librealsense2.so* >/dev/null 2>&1; then
  printf '  [OK]   %-42s librealsense2 已装\n' "realsense2_camera 前置"
else
  printf '  [MISS] %-42s 未装 -> realsense2_camera 无法编译(见 PREREQUISITES.md)\n' "librealsense2"; miss=$((miss+1))
fi

echo
if [ "$miss" -gt 0 ]; then
  echo "结论: 缺失 $miss 项关键资产、$warn 项可选资产。请按 assets/ASSETS.md 从旧机 /data/models 或官方获取后再运行相关功能。"
  exit 2
else
  echo "结论: 关键资产齐全（$warn 项可选项缺失，视需要补充）。"
fi
