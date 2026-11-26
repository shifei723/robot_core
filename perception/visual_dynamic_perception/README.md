# 视觉惯性定位与动态障碍感知

本模块由 `robot_sense` 的已验证链路移植而来，使用专用 Gemini 336L、双目/IMU 定位、深度几何跟踪和 YOLO 语义辅助，输出供感知验证使用的位姿、点云与动态框。默认 VINS-Fusion，可显式选择 OpenVINS。当前不包含预建静态地图差分、RTAB-Map 重定位、FRS 或导航接线；这些仍由导航工程相应模块负责。

## 目录与命名

| 位置 | 用途 |
| --- | --- |
| `perception/visual_dynamic_perception/` | 集成配置、运行脚本、离线测试与模型下载 |
| `perception/onboard_detector/` | 几何聚类、关联、运动补偿和语义辅助；保留上游 ROS 包名 `onboard_detector` |
| `localization/visual_slam/vins_fusion/` | 复用原有唯一 VINS 源码，合入失效检测、重置死锁、可靠输入及 Jazzy 兼容修复 |
| `localization/visual_slam/open_vins/` | OpenVINS 的核心、初始化和 MSCKF 包，不复制其数据集 |
| `sensors/camera/orbbec_camera/` | Orbbec ROS 2 驱动及所需 SDK、描述资源 |
| `scripts/run/run_visual_dynamic_perception.sh` | 与仓库其他功能一致的顶层启动入口 |
| `build/ws_perception/`、`install/ws_perception/`、`log/ws_perception/` | 独立编译产物，忽略入库 |

自有目录与脚本均采用小写下划线，上游包名及既有输出话题保留，避免破坏调用接口。新增可选目录使用 `COLCON_IGNORE` 隔离；专用构建显式指定包路径。`build.sh all` 和 `setup.sh` 不自动加入此 Jazzy 配置，原导航安装与遥操作环境不被覆盖。正常使用不依赖原 `robot_sense` 的源码、安装目录或环境。

## 构建

当前验证平台为 Ubuntu 24.04 x86_64、ROS 2 Jazzy、系统 Python 3.12、OpenCV 4.6。ARM64 SDK 随驱动保留，但 ARM64/Humble 没有完成本模块验证。

基础依赖包括 ROS Jazzy 的 cv_bridge、image_transport、PCL、tf2、消息过滤与驱动所需组件，以及 Eigen、Boost、glog、gflags、SuiteSparse、OpenCV、Python numpy/scipy/yaml。系统 ROS 和这些开发依赖需预先就绪。小内存设备建议保持 `MAKE_JOBS=2` 或设为 1。

本机已为新目录准备独立的本地依赖和 YOLO 模型。全新克隆后先执行：

```bash
cd /path/to/robot_core
bash perception/visual_dynamic_perception/scripts/bootstrap_dependencies.sh
bash build.sh ws_perception
```

本地引导使用 `apt-get download`/解包准备 Foxglove、vision_msgs、rapidjson，编译兼容旧优化接口的 Ceres 2.1，并下载带 SHA256 校验的 YOLOX-tiny 模型；不执行 sudo 或全局软件安装。依赖缓存 `.deps/` 和模型不随普通 Git 提交，下载脚本与来源信息入库。旧版 VINS 的优化接口需要 Ceres 2.1，不能直接换成系统 Ceres 2.2。

## 启动

先停止原 `robot_sense` 专用检测服务，再运行下列命令；两份服务不能同时占用同一相机和端口。脚本发现 9876 端口被占用时会拒绝重复启动。

```bash
cd /path/to/robot_core
# VINS-Fusion，默认开启 YOLO 辅助
LOCALIZATION_MODE=vins bash scripts/run/run_visual_dynamic_perception.sh

# 或 OpenVINS，二选一
LOCALIZATION_MODE=openvins bash scripts/run/run_visual_dynamic_perception.sh

# 关闭语义辅助进行对照
YOLO_ASSIST=0 LOCALIZATION_MODE=vins bash scripts/run/run_visual_dynamic_perception.sh
```

默认只绑定序列号 `CPC87630003A`，在 ROS 域 36 运行，不继承遥操作域 42。设备缺失直接退出；不会自动选择另一台相机。可用 `CAMERA_SERIAL`、`DYNAMIC_ROS_DOMAIN_ID`、`FOXGLOVE_PORT` 显式覆盖。启动时读取实际设备的 CameraInfo/TF 生成 `runtime/336l/` 配置，测试夹具不是实机标定的替代品。

Foxglove WebSocket：本机 `ws://localhost:9876`，远端使用运行电脑的 IP。导入本目录 `config/336l_yolo_foxglove.json` 同时显示语义图、动态深度图与三维点云。默认不启动 RViz；需要桌面显示时另开终端：

```bash
bash scripts/run/view_visual_dynamic_perception.sh
```

## 数据与判别逻辑

双目和 IMU 经 VINS-Fusion 或 OpenVINS 得到惯性单元位姿；`body_odometry.py` 利用工厂外参转换为 `camera_link` 位姿，输出 `/detection/odometry` 与 `map -> camera_link` TF。这里的 map 为本次定位的局部参考系，不是已完成全局地图重定位。

深度图按采集时间配对位姿，经点云投影、体素过滤、DBSCAN、跨帧关联和卡尔曼跟踪形成目标历史。几何运动判据结合可见边界运动、滤波速度、方向一致性和多帧投票。定位失效/超时会暂停分类并清除过期显示。

YOLOX-tiny COCO 使用彩色去畸变图、OpenCV DNN CPU 双线程，默认 5 Hz。语义框按历史相机位姿与目标点云投影关联：置信度至少 0.45、时间差不超过 40 ms、至少 5 个点且覆盖至少 35%；结果有效期 0.35 s。语义支持只保留正常运动门槛，未匹配目标要求更强运动证据，过期结果回退几何判别；不会把 person 类别直接当动态。该策略可能漏检低速未知物体，不是完整的一对一三维语义跟踪。

主要输出保持兼容：

- `/detection/odometry`：相机机身位姿。
- `/detection/semantic_boxes`、`/detection/semantic_image`、`/detection/semantic_status`：语义框、叠加图与推理状态。
- `/onboard_detector/tracked_bboxes`：全部几何跟踪框，不等于动态框。
- `/onboard_detector/dynamic_bboxes`、`/onboard_detector/dynamic_point_cloud`：最终动态结果。
- `/onboard_detector/filtered_depth_cloud`、`/onboard_detector/detected_depth_map`：处理点云与动态叠加图。

OpenVINS 使用工厂内外参、200 Hz IMU 噪声工程初值和零时间偏移，关闭在线标定；启用视差触发的条件零速更新。移动相机精度和零速阈值仍需实机标定与验证。VINS 的当前下游输入为优化位姿；其高频 IMU 传播输出用于检查，不被混淆为已接入的检测输入。

## 验证

```bash
# 无需相机；独立域 37，串行运行
bash perception/visual_dynamic_perception/scripts/test.sh

# 服务启动后，采样实机输入输出（不自动启动相机）
source perception/visual_dynamic_perception/scripts/env.sh
source perception/visual_dynamic_perception/scripts/dds_env.sh
ROS_DOMAIN_ID=36 /usr/bin/python3 perception/visual_dynamic_perception/scripts/check_pipeline.py --backend vins --seconds 30 --output perception/visual_dynamic_perception/runtime/live_check.json
```

合成测试覆盖坐标转换、OpenVINS 静止—横移—静止响应、语义支持、未知运动保留、语义过期回退与定位失效清除，不作为实机精确率或导航安全性结论。历史采集的配置仅保存在 `tests/fixtures/gemini336l` 用作离线测试。来源清单见 `source_manifest.json`，当前迁移构建/验证结果另见 `validation.json`。
