#!/usr/bin/env bash
# ============================================================
# pack_open3d.sh —— 把 third/open3d141[_x86] 打包成 tarball，
#                   用于上传 GitHub Release / 内网分发，供 fetch_open3d.sh 下载。
#
# 用法:
#   bash assets/pack_open3d.sh            # 打当前架构
#   bash assets/pack_open3d.sh arm|x86|all
#
# 输出目录默认 $ROBOT_CORE/../open3d_dist，可用 OPEN3D_PACK_DIR 覆盖。
# 上传后把地址填进 assets/open3d_sources.env：
#   OPEN3D_ARM_URL=https://github.com/<owner>/<repo>/releases/download/<tag>/open3d141-aarch64.tar.gz
#   OPEN3D_X86_URL=https://.../open3d141-x86_64.tar.gz
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${OPEN3D_PACK_DIR:-$ROOT/../open3d_dist}"
mkdir -p "$OUT"

case "$(uname -m)" in
  aarch64|arm64) host=arm ;;
  *)             host=x86 ;;
esac
case "${1:-auto}" in
  auto) kinds="$host" ;;
  arm|x86) kinds="$1" ;;
  all) kinds="arm x86" ;;
  *) echo "用法: $0 [auto|arm|x86|all]" >&2; exit 1 ;;
esac

pack_one() {
  local kind="$1" base name src
  if [ "$kind" = arm ]; then
    base="open3d141"; name="aarch64"
  else
    base="open3d141_x86"; name="x86_64"
  fi
  src="$ROOT/third/$base"
  if [ ! -d "$src" ]; then
    echo "  [跳过] $src 不存在（先 bash assets/fetch_open3d.sh $kind）"
    return 0
  fi
  local out="$OUT/open3d141-$name.tar.gz"
  echo "== 打包 $base  ->  $out"
  tar -czf "$out" -C "$ROOT/third" "$base"
  echo "  [OK]   $(du -h "$out" | cut -f1)"
}

for k in $kinds; do pack_one "$k"; done

echo
echo "上传 $OUT 下的 tarball 到 GitHub Release / 内网，"
echo "再把 URL 填进 assets/open3d_sources.env 即可在新机器上一键获取。"
