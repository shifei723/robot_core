# ws — 机器人代码库（按类别重组）

本目录是 `/data/sf_code` 的**源码重组版**：按功能类别整理，剥离了各自的 git 仓、构建产物（`build/ install/ log/`）、虚拟环境与大二进制（模型 `*.hbm/*.gguf`、地图 `*.db/*.ply/*.pcd`、库 `*.so/*.whl/*.tar.*`、演示媒体 `*.gif/*.mp4` 等），作为**纯代码**统一管理。

> ℹ️ 现状
> - 原 `/data/sf_code` 仍保留；本目录在其基础上做了**可移植化改造**，可作为新机器上的编译/运行源。
> - **绝对路径已适配**：硬编码 `/data/sf_code/...` 改为基于环境变量 `ROBOT_CORE`（clone 到任意目录即可用）；
>   少数由 C++ 直读、不支持环境变量展开的 yaml（Hesai 校正、VINS 输出）默认指向 `/data/robot_core`，换目录时需同步改前缀。
> - **构建/运行入口**：`build.sh`（分多工作区构建）、`setup.sh`（加载环境）、`assets/`（依赖与资产清单）。
> - 大二进制（模型 `*.hbm/*.gguf/*.onnx`、地图 `*.db/*.ply/*.pcd`、预编译库）不进库，见 `assets/ASSETS.md`。
> - 规模：约 620M，70 个 ROS2 源码包。

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
