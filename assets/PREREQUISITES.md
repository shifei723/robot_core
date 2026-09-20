# PREREQUISITES —— 新机器系统级依赖与构建前置

目标环境：**Ubuntu 22.04 + ROS2 Humble**（`/opt/ros/humble`）。

## 0. 一键装 ROS 包依赖（推荐，覆盖绝大部分）

```bash
source /opt/ros/humble/setup.bash
cd "$ROBOT_CORE"
# 让 rosdep 扫描本仓库所有 package.xml 并安装缺失依赖（-r 跳过无法解析的项）
rosdep install --from-paths localization navigation sensors robot voice --ignore-src -r -y
```

`rosdep` 能自动解决：`PCL、OpenCV、Eigen、Boost、cv_bridge、pcl_conversions、
image_transport、laser_geometry、grid_map、GTSAM、libpointmatcher、libnabo、assimp` 等。

## 1. 需手动安装的通用库（apt）

```bash
sudo apt update && sudo apt install -y \
  build-essential cmake git \
  libpcl-dev libopencv-dev libeigen3-dev libboost-all-dev libusb-1.0-0-dev \
  libpcap-dev libserial-dev libglew-dev libglfw3-dev libglm-dev \
  libassimp-dev libqhull-dev libvtk9-dev csvviz libdlib-dev \
  python3-colcon-common-extensions python3-rosdep
```

> 说明：PCL/OpenCV/VTK 的具体 dev 包名以你系统的 ROS humble 依赖为准；
> 一般先跑 `rosdep`（第 0 步）即可带出正确版本，避免手装版本冲突。

## 2. 各工作区特殊前置（非 rosdep 能覆盖）

| 工作区/包 | 前置 |
|---|---|
| `rtabmap`（组）| rtabmap 库源码在本仓 `localization/visual_slam/rtabmap`，随组编译；其依赖 VTK/PCL/ceres/g2o 等，第 0/1 步基本覆盖。 |
| `realsense_ros`（组内）| 依赖 **librealsense2** 已安装。仓库含 `sensors/camera/librealsense` 源码（已 `COLCON_IGNORE`），按其自身 CMake 单独构建安装，或 `apt install librealsense2-dkms ros-humble-realsense2-*`。 |
| `livox_ros2_driver` / `rslidar_sdk` / `HesaiLidar_ROS_2.0` / `unitree_lidar` | 各自 vendor 的 SDK 随包编译；Livox/rslidar 的 `*.so` 由包的 vendor 步骤拉取或已在库内。`rslidar_msg/ros1|ros2` 已 `COLCON_IGNORE`，只用根 `rslidar_msg`。 |
| `voice/agent`（omni_node）| 依赖地平线 `oellm_runtime/{lib,include}`（已在库内）。构建需先 `source setup.sh` 令 `ROBOT_CORE` 生效，或 `-DOELLM_SDK_ROOT=...` 指定。 |
| `voice/kws`（C++ demo）| 非 ROS 包（无 `package.xml`），用其自带 CMake + `sherpa-onnx-*-aarch64` 运行库单独构建。 |
| `voice/llm_sdk`（SDK 例程）| 非 ROS 包；`resolve_model_*.txt` 内含官方 `wget` 链接，用于下载 `*.hbm` 模型。 |
| `sensors/camera/mvs_ros2_pkg` | 海康 MV 相机 SDK（`.so` 被排除），需装厂商 MVS SDK。 |

## 3. 架构提醒

原系统在 **aarch64**（地平线 S100 / 树莓派类）上运行，库内含少量 aarch64 预编译 ELF
（`unitree_lidar/bin/*`、`sherpa-onnx-...-aarch64`、`oellm_runtime/lib/*.so`）。若新机器是
**x86_64**，这些需替换为对应架构版本或从源码重编；ROS 包本身可正常跨架构重编。

## 4. 构建顺序

```bash
cd "$ROBOT_CORE"
./build.sh            # 按 ros_ws→rtabmap→ws_nav→agent_ws→tts 顺序构建到 install/<组>
source setup.sh       # 之后开新终端运行 scripts 前都要 source 一次
bash assets/verify_assets.sh   # 体检运行资产
```
