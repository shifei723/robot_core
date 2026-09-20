#!/bin/bash

# 自动获取脚本所在目录的上级目录（即项目根目录）
# 这样无论你在哪里启动，它都能找到正确的 FAST-LIVO2 文件夹
SOURCE_DIR=$(cd "$(dirname "$0")/.."; pwd)

TARGET_DIRS=(
    "$SOURCE_DIR/Log/Colmap/images"
    "$SOURCE_DIR/Log/Colmap/sparse/0"
)

# 第一步：清理旧目录
for dir in "${TARGET_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        rm -rf "$dir"
        echo "Removed: $dir"
    fi
done

# 第二步：重建新目录
for dir in "${TARGET_DIRS[@]}"; do
    mkdir -p "$dir"
    if [ $? -eq 0 ]; then
        echo "Successfully Created: $dir"
    else
        echo "Failed to Create: $dir (Check permissions!)"
    fi
done