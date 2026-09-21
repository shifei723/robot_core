# ws — 机器人代码库（按类别重组）

本目录是 `/data/sf_code` 的**源码重组版**：按功能类别整理，剥离了各自的 git 仓、构建产物（`build/ install/ log/`）、虚拟环境与大二进制（模型 `*.hbm/*.gguf`、地图 `*.db/*.ply/*.pcd`、库 `*.so/*.whl/*.tar.*`、演示媒体 `*.gif/*.mp4` 等），作为**纯代码**统一管理。

> ℹ️ 现状
> - 原 `/data/sf_code` 仍保留；本目录在其基础上做了**可移植化改造**，可作为新机器上的编译/运行源。
> - **绝对路径已适配**：硬编码 `/data/sf_code/...` 改为基于环境变量 `ROBOT_CORE`（clone 到任意目录即可用）；
>   少数由 C++ 直读、不支持环境变量展开的 yaml（Hesai 校正、VINS 输出）默认指向 `/data/robot_core`，换目录时需同步改前缀。
> - **构建/运行入口**：`build.sh`（分多工作区构建）、`setup.sh`（加载环境）、`assets/`（依赖与资产清单）。
> - 大二进制（模型 `*.hbm/*.gguf/*.onnx`、地图 `*.db/*.ply/*.pcd`、预编译库）不进库，见 `assets/ASSETS.md`；
>   其中**编译必需的闭源预编译库**已在 `.gitignore` 中单独放行并随仓库携带（见下方「内置二进制依赖」）。
> - 规模：约 620M，70 个 ROS2 源码包。
> - **当前版本已验证可编译**：aarch64 / Ubuntu 22.04 / ROS 2 Humble 下 `./build.sh all` 五组全部通过（详见「构建状态」）。

## 新机器：构建与运行

```bash
# 0) 装系统依赖（详见 assets/PREREQUISITES.md）
source /opt/ros/humble/setup.bash

# 1) 构建（按 ros_ws→rtabmap→ws_nav→agent_ws→tts 分组建到 install/<组>）
./build.sh            # 或单组: ./build.sh ros_ws

# 2) 加载环境（自动定位仓库根、导出 ROBOT_CORE、source 各组 install）
source setup.sh

# 3) 运行资产体检（模型/地图/KWS 等，详见 assets/ASSETS.md）
bash assets/verify_assets.sh

# 4) 启动功能脚本（已改为基于 ROBOT_CORE，无需再改路径）
bash scripts/run/run_mapping.sh l2
bash scripts/run/run_pipeline.sh
```

- 每次**新开终端**都要先 `source setup.sh`（或在 `~/.bashrc` 里 source 它）。
- `build.sh` / `setup.sh` 均按脚本自身位置推导 `ROBOT_CORE`，因此整仓可放在任意路径。
- `scripts/run/`、`scripts/test/` 顶部已内置 `ROBOT_CORE` 自定位并 source `setup.sh`。

## 构建状态与依赖

### 已验证环境

| 项 | 值 |
|---|---|
| 架构 / 系统 | aarch64 · Ubuntu 22.04.5 |
| ROS | Humble |
| 机器资源 | 4 核 / 8G（小内存机器请用默认并行度，见下） |

### 构建结果（`./build.sh all`）

| 工作区 | 包数 | 状态 |
|---|---|---|
| `ros_ws` | 27 | 通过（含 `open3d_loc`、`fastlio2`、点云定位等） |
| `rtabmap` | 22 | 通过（rtabmap 0.23.9 核心库、`rtabmap_ros`、VINS-Fusion、RealSense） |
| `ws_nav` | 14 | 通过 |
| `agent_ws` | 0 | 组内唯一包 `omni_node` 暂跳过（见下） |
| `tts` | 2 | 通过（`hobot_tts`、`tts_cpp`） |

### 系统依赖（apt，详见 `assets/PREREQUISITES.md`）

ROS 2 Humble 基础环境外，主要还需要：

```bash
sudo apt install -y \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-cv-bridge \
  libpcl-dev libopencv-dev libeigen3-dev libboost-filesystem-dev \
  libyaml-cpp-dev libcereal-dev libceres-dev \
  libzmq3-dev libasound2-dev libusb-1.0-0-dev \
  librealsense2-dev liblapacke-dev
```

说明：
- **Ceres**：VINS-Fusion 的 `loop_fusion` / `global_fusion` 已按 Ceres **2.x** API 适配（旧代码用 Ceres 1.x 的 `LocalParameterization` 等接口会编译失败）。
- **liblapacke-dev**：`open3d_loc` 链接需要。系统没装也能编——`open3d_loc/CMakeLists.txt` 会自动回落到仓库自带的 `third/lapacke/lib/liblapacke.so`（免 root 方案）。

### 内置二进制依赖（随仓库携带）

这些是厂商/上游只提供二进制的闭源依赖，无法现场编译，已在 `.gitignore` 中单独放行：

| 路径 | 内容 | 用途 |
|---|---|---|
| `voice/tts/hobot_tts/wetts/lib/` | `libtts.so`、`libonnxruntime.so.1.11.1` | 地平线 TTS 节点链接 |
| `voice/tts/tts_cpp/sherpa-onnx-sdk/lib/` | sherpa-onnx 1.13.3（aarch64）`libsherpa-onnx-c-api.so` 等 | `tts_cpp` 链接 |
| `third/lapacke/lib/` | LAPACKE 3.10.0 | `open3d_loc` 链接（系统无 liblapacke 时用） |

### 需自备 / 暂跳过的部分

| 项 | 说明 | 恢复方式 |
|---|---|---|
| `sensors/camera/mvs_ros2_pkg` | 需海康 MVS SDK（`/opt/MVS/include/MvCameraControl.h` + `/opt/MVS/lib/aarch64/libMvCameraControl.so`） | 装好 SDK 后删除该目录下的 `COLCON_IGNORE` |
| `voice/agent/src/omni_node` | 需地平线 `voice/llm_sdk/D-Robotics_LLM_S100_1.0.0_SDK/oellm_runtime/lib`（`libxlm.so` 等，仓库里只有 config/include/model） | 补齐 `lib/` 后删除该目录下的 `COLCON_IGNORE` |
| Open3D 1.4.1 | 体积 576M，不入库。`open3d_loc` 需要它 | 放到 `third/open3d141/`（本机为指向外部备份的软链接），或用 `OPEN3D_DIR` / `-DOpen3D_DIR=...` 指定 |
| TTS 运行模型 | `tts_cpp/matcha-icefall-zh-baker/`、`tts_cpp/horizon_convert/`（编译不需要，运行 TTS 才用） | 从原机器备份拷贝，或按 `voice/tts/tts_cpp/README.md` 获取 |

> `COLCON_IGNORE` 是本仓库的既有惯例（`sensors/camera/librealsense`、`rslidar_msg/ros1|ros2` 同样处理）：加了这个文件的包会被 colcon 跳过，不影响其它包构建。

### 并行度控制（`build.sh`）

小内存/嵌入式机器默认就是"一次一个包 + 包内 2 线程"，避免拉满 CPU 或 OOM。可用环境变量覆盖：

```bash
./build.sh all                      # 默认: make -j2, 包级并发 1, 包间串行
MAKE_JOBS=1 ./build.sh rtabmap      # 最稳（单线程编译单个包）
MAKE_JOBS=4 PKG_SERIAL=0 ./build.sh all   # 大内存机器: 包内 4 线程 + 包间并行
```

## 目录分类

| 类别 | 说明 | 主要内容 |
|---|---|---|
| `localization/` | 定位与 SLAM | `lidar_slam/` 激光SLAM · `lidar_localization/` 激光重定位 · `visual_slam/` 视觉/RGB-D |
| `navigation/` | 导航与规划 | `scan_planner/` 自研局部规划 · nav2 任务管理 · 区域插件 · 强度代价地图 |
| `sensors/` | 传感器驱动 | `camera/` 相机 · `lidar/` 激光雷达 · `imu_serial/` IMU与串口 |
| `robot/` | 机器人本体 | 轮足底盘控制包 |
| `voice/` | 语音交互 | `kws/` 唤醒 · `tts/` 合成 · `agent/` 多模态 · `llm_sdk/` 端侧大模型 |
| `tools/` | 工具脚本 | TF/里程计转换、地面标定、RealSense 探针等 |
| `scripts/` | 顶层脚本 | `run/` 启动脚本 · `test/` 测试脚本 |
| `data/` | 配置/输出数据 | 地图可视化配置、VINS 输出、第三方清单（大二进制已排除） |

## 原路径映射（ws ← sf_code）

### localization
| ws 路径 | 原路径 |
|---|---|
| `localization/lidar_slam/fastlio2` | `ros_ws/mapping/fastlio2` |
| `localization/lidar_slam/point_lio_ros2` | `ros_ws/mapping/point_lio_ros2` |
| `localization/lidar_slam/FAST-LIVO2` | `ros_ws/mapping/FAST-LIVO2` |
| `localization/lidar_slam/SC_PGO` | `ros_ws/mapping/SC_PGO` |
| `localization/lidar_slam/FAST-Calib-ROS2` | `ros_ws/mapping/FAST-Calib-ROS2` |
| `localization/lidar_slam/rpg_vikit` | `ros_ws/mapping/rpg_vikit` |
| `localization/lidar_localization/FAST_LIO_LOCALIZATION_HUMANOID` | `ros_ws/src/FAST_LIO_LOCALIZATION_HUMANOID` |
| `localization/visual_slam/rtabmap` | `rtabmap/src/rtabmap` |
| `localization/visual_slam/rtabmap_ros` | `rtabmap/src/rtabmap_ros` |
| `localization/visual_slam/vins_fusion` | `rtabmap/src/VINS-Fusion-ROS2-main` |
| `localization/visual_slam/realsense_ros` | `rtabmap/src/realsense-ros` |

### navigation
| ws 路径 | 原路径 |
|---|---|
| `navigation/scan_planner` | `ws_nav/src/Local_Planner` |
| `navigation/zone_interfaces` | `ros_ws/nav/zone_interfaces` |
| `navigation/nav2_task_manager` | `ros_ws/nav/nav2_task_manager` |
| `navigation/rviz2_zone_plugin` | `ros_ws/nav/rviz2_zone_plugin` |
| `navigation/l2_nav_bringup` | `ros_ws/nav/l2_nav_bringup` |
| `navigation/costmap_intensity` | `ros_ws/nav/costmap_intensity` |

### sensors
| ws 路径 | 原路径 |
|---|---|
| `sensors/camera/mvs_ros2_pkg` | `ros_ws/sensor/mvs_ros2_pkg` |
| `sensors/camera/librealsense` | `tools/librealsense` |
| `sensors/lidar/livox_ros2_driver` | `ros_ws/src/livox_ros2_driver` |
| `sensors/lidar/rslidar_sdk` · `rslidar_msg` | `ros_ws/src/rslidar_sdk` · `rslidar_msg` |
| `sensors/lidar/HesaiLidar_ROS_2.0` | `ros_ws/src/HesaiLidar_ROS_2.0` |
| `sensors/lidar/unitree_lidar` | `unilidar_sdk2` |
| `sensors/imu_serial/h30` | `ros_ws/sensor/h30` |

### robot / voice / tools / scripts / data
| ws 路径 | 原路径 |
|---|---|
| `robot/wheeled_legged_pkg` | `ros_ws/robot/wheeled_legged_pkg` |
| `voice/kws` | `kws` |
| `voice/tts` | `tts` |
| `voice/agent` | `agent_ws` |
| `voice/llm_sdk` | `agent_pipeline`（SDK 的 `.whl/.so/.tar.*` 已排除） |
| `tools/` | `tools/`（`librealsense` 归入 `sensors/camera/`） |
| `scripts/run/` | 顶层 `run_*.sh`、`setup_l2_lidar.sh`、`export_2d_map.sh` 等 |
| `scripts/test/` | 顶层 `test_*.sh` |
| `data/rtabmap_maps` · `vins_output` | 同名目录（`.db/.ply/.pcd` 已排除） |
| `data/third_party/manifest.yaml` | `third_party/manifest.yaml` |
