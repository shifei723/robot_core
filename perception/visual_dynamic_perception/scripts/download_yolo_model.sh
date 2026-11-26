#!/usr/bin/env bash
set -eo pipefail
sense_yolo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$sense_yolo_root/models"
sense_yolo_tmp="$(mktemp "$sense_yolo_root/models/yolox_download.XXXXXX")"
trap 'rm -f "$sense_yolo_tmp"' EXIT
curl -fL --connect-timeout 15 --max-time 180 https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_tiny.onnx -o "$sense_yolo_tmp"
printf '%s  %s\n' 427cc366d34e27ff7a03e2899b5e3671425c262ea2291f88bb942bc1cc70b0f7 "$sense_yolo_tmp" | sha256sum -c -
mv "$sense_yolo_tmp" "$sense_yolo_root/models/yolox_tiny.onnx"
