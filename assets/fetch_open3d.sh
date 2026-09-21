#!/usr/bin/env bash
# ============================================================
# fetch_open3d.sh —— 把 Open3D 1.4.1 预编译版放到 third/
#
# 背景：Open3D 两版都是静态库版，aarch64 约 576M、x86_64 约 1.6G，
#       既不进版本库也不走 Git LFS（免费额度仅 1GB），改为按需获取。
#       放置后 open3d_loc / rtabmap 的 find_package(Open3D) 会自动找到它
#       （CMakeLists 按 uname -m 选择目录）。
#
# 用法:
#   bash assets/fetch_open3d.sh            # 只拉当前架构（推荐）
#   bash assets/fetch_open3d.sh arm        # 只拉 aarch64 版
#   bash assets/fetch_open3d.sh x86        # 只拉 x86_64 版
#   bash assets/fetch_open3d.sh all        # 两版都拉（约 2.2G）
#
# 来源优先级（取到即停）:
#   1) 目标目录已含 lib/cmake/Open3D/Open3DConfig.cmake -> 跳过
#   2) 本地副本目录：环境变量 OPEN3D_LOCAL_ARM_DIR / OPEN3D_LOCAL_X86_DIR，
#      默认探测仓库同级的 third/open3d141[_x86] -> 复制（cp -aL，解软链接）
#   3) 本地压缩包：环境变量 OPEN3D_LOCAL_TARBALL_ARM / _X86，
#      默认探测同级 third/open3d141-aarch64.tar.gz / open3d141-x86_64.tar.gz
#      （百度网盘下载后放这里即可，见 README「Open3D 预编译版」）
#   4) 直链下载：环境变量 OPEN3D_ARM_URL / OPEN3D_X86_URL，
#      或 assets/open3d_sources.env 中配置的 Release / 内网地址
#
# 打包上传（生成供第 3、4 步使用的 tarball）: bash assets/pack_open3d.sh
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$ROOT/assets/open3d_sources.env"
[ -f "$ENV_FILE" ] && . "$ENV_FILE"

# 百度网盘分享（非直链，需浏览器手动下载，下载后放到同级 third/ 即可）
PAN_URL="https://pan.baidu.com/s/1gz-cIzgdKCIciffBTP4ejQ?pwd=1234"

ok()   { echo "  [OK]   $1"; }
info() { echo "  [INFO] $1"; }
die()  { echo "  [ERR]  $1" >&2; exit 1; }

want="${1:-auto}"
case "$(uname -m)" in
  aarch64|arm64) host=arm ;;
  x86_64|amd64)  host=x86 ;;
  *)             host=arm ;;
esac
case "$want" in
  auto) targets="$host" ;;
  arm)  targets="arm" ;;
  x86)  targets="x86" ;;
  all)  targets="arm x86" ;;
  *)    die "用法: $0 [auto|arm|x86|all]" ;;
esac

fetch_one() {
  local kind="$1" dst src tarball url label
  if [ "$kind" = arm ]; then
    dst="$ROOT/third/open3d141"
    src="${OPEN3D_LOCAL_ARM_DIR:-$ROOT/../third/open3d141}"
    tarball="${OPEN3D_LOCAL_TARBALL_ARM:-$ROOT/../third/open3d141-aarch64.tar.gz}"
    url="${OPEN3D_ARM_URL:-}"
    label="aarch64"
  else
    dst="$ROOT/third/open3d141_x86"
    src="${OPEN3D_LOCAL_X86_DIR:-$ROOT/../third/open3d141_x86}"
    tarball="${OPEN3D_LOCAL_TARBALL_X86:-$ROOT/../third/open3d141-x86_64.tar.gz}"
    url="${OPEN3D_X86_URL:-}"
    label="x86_64"
  fi

  echo "== Open3D $label  ->  ${dst#$ROOT/}/"
  if [ -f "$dst/lib/cmake/Open3D/Open3DConfig.cmake" ]; then
    ok "已就位，跳过"
    return 0
  fi

  if [ -d "$src" ]; then
    info "本地副本: $src  体积 $(du -shL "$src" 2>/dev/null | cut -f1)"
    mkdir -p "$dst"
    cp -aL "$src/." "$dst/"
  elif [ -f "$tarball" ]; then
    info "本地压缩包: $tarball"
    mkdir -p "$dst"
    tar -xzf "$tarball" -C "$dst" --strip-components=1 || die "解压失败: $tarball"
  elif [ -n "$url" ]; then
    info "下载: $url"
    local tmp
    tmp="$(mktemp "${TMPDIR:-/tmp}/open3d.XXXXXX.tar.gz")"
    curl -fL --retry 2 -o "$tmp" "$url" || die "下载失败: $url"
    mkdir -p "$dst"
    tar -xzf "$tmp" -C "$dst" --strip-components=1 || die "解压失败: $tmp"
    rm -f "$tmp"
  else
    die "未找到来源，任选其一：
      1) 放置解压好的目录到  $src
      2) 百度网盘下载后把压缩包放到  $tarball
         网盘：$PAN_URL （提取码 1234）
      3) 设置直链 $( [ "$kind" = arm ] && echo OPEN3D_ARM_URL || echo OPEN3D_X86_URL )=https://.../open3d141-<arch>.tar.gz
         或填写 $ENV_FILE"
  fi

  [ -f "$dst/lib/cmake/Open3D/Open3DConfig.cmake" ] ||
    die "校验失败：$dst 内缺少 lib/cmake/Open3D/Open3DConfig.cmake"
  ok "就位（Open3D_DIR=$dst/lib/cmake/Open3D）"
}

for t in $targets; do fetch_one "$t"; done

echo
echo "完成。构建 open3d_loc / rtabmap 时会按 uname -m 自动使用上述目录。"
echo "如需打包分发: bash assets/pack_open3d.sh"
