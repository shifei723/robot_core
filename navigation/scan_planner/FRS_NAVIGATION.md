# Slash 导航中的前向可达集（FRS）

FRS 已接入本工程的实际 B 样条执行链路：

```
障碍物跟踪位置、速度与时间戳
  → IntentFrs 在线估计加速度输入范围
  → 未来位置占据矩形（3 秒预测时域）
  → 检查 B 样条未来路径
  → 选择推进倍率 1 / 0.5 / 0
  → 原闭环控制器发送速度与高度命令
```

核心文件：

- `src/planner/plan_manage/include/plan_manage/intent_frs.hpp`：意图输入集、FRS 传播、候选时序检查。
- `src/planner/plan_manage/src/closed_loop_controller.cpp`：订阅跟踪数据，选择推进倍率，记录决策。
- `src/planner/plan_manage/launch/run.launch.py`：`dynamic_prediction` 启动参数。
- `tools/validate_dynamic_navigation.py`：含行人轨迹发布、导航目标下发、碰撞和预测评价的完整仿真实验。

这是固定几何路径上的动态通行时序调节。当前没有把 FRS 代价加入
`bspline_optimizer.cpp`，不会因为动态障碍生成新的横向绕行路径，也不是完整的位置—高度联合优化。
现在支持多个固定身份目标，各自维护历史与 FRS，所有目标共同参与候选检查。
原脚本仿真仍使用固定身份的 `dynamic_navigation/pedestrian` 接口。
另已加入不依赖语义检测的深度几何感知接口，支持多目标关联、动静分类和目标生命周期；使用方法见下文。

## 编译

从 `robot_core` 根目录执行。现存旧构建目录含迁移前的绝对路径，使用独立目录：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths navigation/scan_planner/src \
  --build-base build/ws_nav_frs --install-base install/ws_nav_frs \
  --parallel-workers 3 --cmake-args -DBUILD_TESTING=ON \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
source install/ws_nav_frs/setup.bash
```

## 运行完整仿真实验

以下命令同时启动导航、Slash 运动学仿真和带噪行人跟踪输入，自动下发导航目标。
输出目录须尚不存在：

```bash
source install/ws_nav_frs/setup.bash
/usr/bin/python3 navigation/scan_planner/tools/validate_dynamic_navigation.py \
  --mode frs --case crossing --seed 100 --domain 151 \
  --output /tmp/slash_frs_crossing_run1
```

`--case combined` 测试低横梁与行人横穿组合；`--mode cv` 和 `--mode physical`
分别运行匀速点预测和固定物理加速度上限预测，用于公平对照。
这里的行人由理想跟踪接口表示，不是点云检测结果；仿真为运动学模型，不含轮地接触和平衡动力学。

## 单独启动导航并接入跟踪器

```bash
bash navigation/scan_planner/tools/run_frs_navigation.sh
```

该入口已启用 FRS，但不会自动发布行人或导航目标。发布目标至 `/move_base_simple/goal`，
跟踪器需要持续提供以下接口：

| 项目 | 要求 |
| --- | --- |
| 话题 | `/dynamic_navigation/pedestrian` |
| 消息 | `nav_msgs/msg/Odometry` |
| `header.frame_id` | `world` |
| `child_frame_id` | `pedestrian_0`、`pedestrian_1` 等，后缀必须为非负整数 |
| `header.stamp` | 观测时刻，与 ROS 当前时间使用同一时钟 |
| `pose.pose.position.x/y` | 世界坐标系位置，单位 m |
| `twist.twist.linear.x/y` | 世界坐标系速度，单位 m/s；这是本接口的明确约定 |

输入应先由上游完成坐标变换，不能直接把相机坐标或机体系速度填入此接口。
`dynamic_min_tracks` 设置启动所需目标数（默认 1，三行人实验为 3）。
目标数不足或任一已跟踪目标超过 0.5 秒未更新时，控制器进入 `WAIT_TRACK`；乱序或无效数据不更新历史。
启动参数目前在 Slash 仿真配置中接线，尚未启用真机导航的 FRS 链路。

诊断话题 `/dynamic_navigation/decision` 为 JSON 字符串，包含预测矩形、名义与选择的未来路径、
推进倍率、预测安全间隔、可行性、保护触发和决策耗时。
可用 `ros2 topic echo /dynamic_navigation/decision` 查看；`/height_control/state` 显示控制状态。

`dynamic_prediction` 可取 `off`、`cv`、`frs`、`physical`，普通 `run.launch.py` 默认 `off`。
过去输入的范围不保证覆盖未来所有突变；无可行候选时停车也不构成一般安全保证。

## 已有验证

FRS 模块现有 6 项 C++ 单元测试，新增目标历史隔离、乱序到达、单目标失联、所有目标共同约束与顺序不变性检查。
原高度/航向模块有 9 项测试。
普通横穿和低横梁组合共 18 次正式 ROS 对照运行全部到达，未记录碰撞或紧急保护触发。
原始记录、对照表及动画位于兄弟工程 `../mythesis/output/ch5_frs_evaluation`（相对 robot_core 根目录）。
这些结果验证的是当前单目标运动学场景，不代表多行人、遮挡或真机性能。

## 多目标对照与无头渲染

三名行人分别在 0、8、16 秒启动横穿，其中第二名反向行走。普通横穿与低横梁组合采用
不同的横穿纵坐标；所有横穿线避开中间实体墙，目标运动独立于机器人方法。
每帧交替反转消息到达顺序，目标历史按 ID 维护。目标均为半径 0.30 m 的脚本化障碍，
没有行人间社交交互或物理碰撞响应；离开局部画面后仍沿脚本运动并持续提供理想跟踪输入。

```bash
source install/ws_nav_frs/setup.bash
/usr/bin/python3 navigation/scan_planner/tools/run_multi_frs_experiments.py \
  --output output/multi_frs_evaluation_new --seeds 200 201 202 \
  --cases crossing combined --pedestrians 3 --domain 181
```

本轮正式运行保存于 `output/multi_frs_evaluation`，与开发种子 0 的 `output/multi_frs_pilot` 分离。
每次实验统一上限 100 秒，保留失败和超时；所有候选均不满足预测约束时停车并记录不可行状态。

使用已有结果生成图表和无头三维帧（`PY` 指向已安装 MuJoCo、Pillow、Matplotlib 的环境）：

```bash
PY=/home/robotx-2/ws_tianji_sharpa/temp/UniLab/.venv/bin/python
$PY navigation/scan_planner/tools/plot_multi_frs_results.py output/multi_frs_evaluation
MUJOCO_GL=egl $PY navigation/scan_planner/tools/render_frs_headless.py \
  output/multi_frs_evaluation/combined_frs_200 \
  --output output/multi_frs_headless_new --times 4 16 28 40 --animation
```

渲染使用 Slash 原模型网格、ROS 同款五连杆高度逆解及记录的位姿；右前膝关节按 MJCF 轴方向转换，
每帧检查闭链约束误差。渲染器只调用 `mj_forward`，不调用 `mj_step`，所以这些图像是
**MuJoCo EGL 生成的 ROS 运动学日志回放，不是动力学导航结果**。
固定展示首个正式种子 200 的 4/16/28/40 秒，三种方法采用同一相机与场景；不根据效果挑选运行。
论文用拼图、图注与原始高分辨率 PNG 位于结果目录的 `paper_assets/` 和 `headless_frs_200/`。
原始日志、渲染清单中的 SHA-256 与完整方法对照表一起保留，可核对每幅图的来源。


### 已完成的正式结果

`output/multi_frs_evaluation/` 保存种子 200、201、202 的 18 次正式运行，全部到达，无机器人与行人接触、静态碰撞或反应式保护触发。低横梁组合场景中，CV / Intent-FRS / Physical-FRS 的平均用时分别为 39.87 / 46.85 / 72.91 秒，平均最小表面间距为 0.116 / 0.620 / 0.620 米。三种方法均无碰撞，因此不能从本组实验推断一般碰撞率改善。

完整指标与标准差见 `summary.json`、`comparison.csv`、`REPORT.md`；原始轨迹和协议用于复核。`headless_frs_200/replay.gif` 为连续无头回放，`paper_assets/multi_frs_headless.pdf` 为论文四帧图。渲染只调用正向运动学，不执行动力学积分。

## D435i + VINS：几何动态障碍物感知

新增 `src/perception/geometric_obstacles`，不需要行人检测、RGB 网络或类别标签。
这是一版可接入真实深度话题的工程实现；已做合成数据和 ROS 消息联调，尚未用真机或真实 rosbag 验证识别率、漏检率与计算时延。

```text
对齐彩色相机的深度图 + CameraInfo + 采样时刻 world←光学相机 TF
  → 必要时对深度像素网格去畸变（最近邻，避免混合前后景深度）
  → 下采样反投影、补偿机器人自运动、地面高度过滤
  → 体素聚类、跨帧几何变化、Kalman 速度估计与匈牙利关联
  ├─ 静态：保留在静态地图
  ├─ 待确认：仍保留在静态地图，同时送入保守 FRS 检查
  └─ 已确认动态：独立跟踪，从本帧静态建图射线中排除
       → 目标尺寸 + 位置/速度协方差 + 每个目标独立的 FRS
       → 原有 B 样条执行速度调整：正常 / 减速 / 等待
```

这里补齐的是感知及控制接口，**没有新增绕行路径优化器**。FRS 仍用于调整既有路径的通行时序；遇到持续占路目标可能等待，不能把本实现表述成已经实现了任意动态场景的主动绕行。

### 编译与启动

在真机既有 Humble 工作区内重新编译（新增消息、感知包及依赖库均需编译）：

```bash
cd /home/robotx-2/ws_tianji_sharpa/temp/robot_core
./build.sh ws_nav
source /opt/ros/humble/setup.bash
source install/ws_nav/setup.bash
```

新增 Python 依赖为 NumPy、SciPy、OpenCV，以及 ROS `cv_bridge`、`tf2_ros`。
先用已运行的定位和相机链路独立查看感知；这条命令不启动底盘控制器：

```bash
ros2 run geometric_obstacles geometric_obstacles_node --ros-args \
  -p ground_z:=0.0
```

确认 TF、地面高度、点云投影和运动目标正常后，停止这个独立节点，再通过导航启动脚本开启整条链路，避免两个感知发布者并存：

```bash
GEOMETRIC_PERCEPTION=true GEOMETRIC_GROUND_Z=0.0 \
  bash scripts/run/run_vins_navigation.sh --planner-only
```

`--planner-only` 要求定位链路已运行；去掉该选项沿用原脚本的自动定位启动流程。
`GEOMETRIC_PERCEPTION` 默认 `false`，保留原导航行为。开启后要求深度模式、闭环控制器，自动启用 `dynamic_input=array` 和 FRS。
等价的规划器入口为：

```bash
ros2 launch scan_planner run.launch.py robot_model:=slash \
  is_real_world:=true sensor_type:=depth controller_mode:=closed_loop \
  geometric_perception:=true geometric_ground_z:=0.0
```

若使用 `dynamic_prediction:=cv` 或 `physical`，可以在相同几何跟踪输入上做预测算法对照。
仿真或 rosbag 更换输入时，感知节点参数为 `depth_topic`、`camera_info_topic`、`world_frame`；
集成 launch 的 CameraInfo 参数为 `geometric_camera_info`。仿真必须提供对应光学相机的 TF 和 CameraInfo，原脚本的真值目标不会自动转换为深度观测。

### 观察输出

RViz 的固定坐标系设为 `world`：

| 话题 | 用途 |
| --- | --- |
| `/dynamic_obstacles/cloud_dynamic` | 已确认动态区域点云 |
| `/dynamic_obstacles/cloud_static` | 非动态、非待确认的可见点云，包含地面等背景 |
| `/dynamic_obstacles/cloud_unknown` | 待确认障碍簇 |
| `/dynamic_obstacles/markers` | 目标包络；红色动态/外推，橙色待确认 |
| `/dynamic_obstacles/tracks` | 带 ID、速度、半径、协方差、最近观测时刻的完整目标快照 |
| `/dynamic_obstacles/depth_static` | 带跳过射线掩码的建图深度图 |
| `/dynamic_obstacles/camera_pose` | 深度曝光时刻的光学相机世界位姿 |
| `/dynamic_navigation/decision` | 控制器预测与速度倍率日志 |

```bash
ros2 topic echo /dynamic_obstacles/tracks
ros2 topic hz /dynamic_obstacles/tracks
ros2 topic echo /dynamic_navigation/decision
```

心跳也会发布 `tracks`，所以话题频率不能直接视为有效感知帧率。应同时看 `healthy`、`status`、曝光时间戳及延时。

### 默认参数和必要标定

- 深度单位：`16UC1` 为毫米，`32FC1` 为米；只接受这两种编码。
- `stride=4`、`voxel=0.08 m`、`cluster_distance=0.20 m`、`min_points=6`（体素数）。
- `min_depth=0.3 m`、`max_depth=3.0 m`；点预算 `max_points=15000`，默认适配 640×360。更高分辨率需要调整步长或预算并测量耗时。
- `ground_z=0.0 m` 是 **world 系地面高度**，不是相机或机体高度。去除地面上方 `ground_band=0.12 m` 以内的点，仅用于障碍簇检测；这些点仍留在静态建图深度中。
- 速度大于 `motion_on=0.18 m/s`，并有足够的几何变化，连续 `motion_frames=3` 次后确认动态。变化检测使用约 0.5 s 的参考帧，避免 30 Hz 相邻帧位移过小。
- 速度低于 `motion_off=0.08 m/s` 连续 `static_frames=8` 次后归为静态。中间速度或信息不足的目标保持待确认/原状态。
- `max_frame_age=0.4 s`；控制器快照超时为 0.5 s。
- 单帧位姿跳变超过 `pose_jump_distance=0.3 m` 或 `pose_jump_angle=0.35 rad`，或处理帧时间间隔超过 0.5 s，锁定异常。定位恢复后需重启感知并重新规划，不能继续使用跳变前的轨迹。

应先检查 `world→base_link→camera_color_optical_frame` 外参是否正确。分类使用连续 `world`，不使用会因重定位跳变的 `map`。新模式中静态建图也使用相同 TF 位姿和运行时 CameraInfo，不再叠加旧的手工相机外参。
原始深度若含机器人自身可见结构，应先核对这些点的位置并按实际几何增加自身过滤；当前未使用机器人网格做自遮挡剔除。

### 生命周期与故障处理

`DynamicObstacleArray` 是完整快照：`healthy=true` 且列表为空表示当前没有需要独立 FRS 管理的目标；静态地图仍然参与规划。异常或超时不能解释为空场景，控制器保持等待。

新目标和已确认动态目标送入 FRS；确认静态且当前可见的目标退出独立 FRS 列表，仍保留在静态地图。
已确认静态且丢失的簇交回占据地图管理；其余暂时丢失的目标进行 Kalman 外推；控制器保留最后真实观测的 FRS 历史，不把预测结果当作新观测学习加速度。
缺失目标只有在其不确定性包络采样位置被有效深度证实为空闲，或超出局部管理距离 `retire_distance=6 m` 时才删除。
此前版本在 `lost_timeout=1.0 s` 后将未解决遮挡标为整帧异常并等待。
当前版本不再将该旧参数作为全局停车阈值，改为保留逐目标预测与协方差并进行局部风险检查；
参见“FRS 连续跟踪与遮挡恢复”。这不意味着前向相机具有全向可观测性。

动态像素用 **65535** 表示“跳过这条射线”，并在新模式跳过原始无效深度 0。
不能把动态像素简单清零：旧建图逻辑会把 0 当远距离空闲射线。
已有地图中的动态残影仅由后续真实可见的空闲射线清除，不直接清空遮挡后面的区域。

FRS 碰撞半径使用 `0.43 m` 的机器人包络、跟踪得到的目标半径及 `0.10 m` 余量。
另外将位置/速度协方差的逐轴 3σ 加入可达包络。它是工程概率裕量，**不是严格覆盖保证**；VINS 漂移、错误关联、部分可见表面导致的中心漂移仍需真实数据评估。
`max_cluster_extent=2 m` 以上的大结构、与墙连成一个簇的运动物体目前留在静态建图中，不能保证获得独立运动估计；斜坡、台阶、多层地面也需要扩展地面模型。

### 自动验证

本次在本机 ROS Jazzy 的隔离工作区 `build/ws_nav_frs` / `install/ws_nav_frs` 编译验证，未替换真机 Humble 安装目录。

```bash
source /opt/ros/jazzy/setup.bash
source install/ws_nav_frs/setup.bash
colcon test --build-base build/ws_nav_frs --install-base install/ws_nav_frs \
  --packages-select geometric_obstacles scan_planner
colcon test-result --test-result-base build/ws_nav_frs --verbose
```

新增测试包括纯几何/跟踪测试、独立 ROS 域中的合成深度→TF→目标/掩码测试、
以及实际闭环控制器二进制的快照输入测试。控制器测试将执行器输出重映射到测试话题，验证健康空列表放行、异常/缺失/超时停车、近距离目标停车及目标退出后恢复。
FRS 的 C++ 单元测试验证完整快照的原子校验、目标退出、外推不污染历史和不确定性包络。
这些是软件与合成场景验证，不能替代 D435i 实测或用作真实场景成功率结果。

## 从深度到导航的无头闭环验证（2026-10-21）

新增以下工具：

- `tools/validate_geometric_pipeline.py`：深度射线仿真、理想位姿/TF、平面运动学及腿高响应；启动实际感知、建图规划和闭环控制器进程。
- `tools/report_geometric_validation.py`：依据真值净距、目标距离和各模块消息生成验收报告。
- `tools/render_geometric_validation.py`：从实际日志绘制中文四联图、PDF 和 GIF。

模拟器不向感知或 FRS 注入目标 ID、类别、位置和速度，也不预先灌入静态地图。
障碍物真值只用于生成深度和离线评估。仿真不运行 VINS 估计器、不模拟轮地接触、深度噪声和位姿漂移，属于软件闭环集成验证。

本次结果保存在 `robot_core/data/geometric_validation/validation_report.txt` 和同名 JSON 中。
**静态场景、深度中断停车通过；启停横穿和持续横穿均未通过最终到达目标验收。**
不可把感知/控制器接口测试通过写成动态导航系统整体验收通过。

| 最终配置试验 | 结果 |
| --- | --- |
| `static03` | 29.43 s 到达 3 m 目标的 0.16 m 容差范围；无包络碰撞，最小净距 0.220 m |
| `crossing03` | 检测到动态目标、生成掩码和 FRS 等待；65 s 内未到达，结束时处于 WAIT_MAP；无碰撞 |
| `continuous_crossing02` | 检测与 FRS 链路工作，但 50 s 内未到达；无碰撞 |
| `dropout03` | 在已有约 0.118 m/s 运动时中断深度，约 0.394 s 后停车并保持；无碰撞 |

最终四个场景均已补齐里程计速度字段，使用同一仿真程序和感知代码版本（SHA-256 保存在 summary.json）；较早版本只保留作调试记录。
这些均为单次可复现配置的运行记录，不是统计成功率。到达时间从仿真启动算起，包含初始化；消息延迟是该机器该次运行的测量结果。
早期 `probe01`、`probe02`、`probe03` 及 `crossing01` 也保留。
其中 `crossing01` 曾出现包络碰撞，不能作为成功案例；这促使集成模式同时启用静态高度/碰撞保护和地图、规划状态检查。

验证期间修正了：

1. 稀疏斜视侧面碎簇：仅对细小碎片使用重叠包络及相邻深度像素约束进行合并；默认 `surface_bridge_distance=0.50 m`。不再因两个完整障碍簇的包络接近就合并，防止运动物体带走墙面。
2. 已确认静态簇离开视野后交回占据地图管理，不继续生成丢失的动态目标。静态占据仍按原可见射线规则清除。
3. `geometric_perception=true` 同时启用控制器的 `height_safety_enabled`，并使用 `/dynamic_obstacles/cloud_static` 作为感知更新心跳；静态地图过期或 NO_PATH 时保持停车。

剩余问题已由真实闭环日志暴露：视角变化下的墙面/局部表面跟踪仍会出现伪运动；部分可见表面的中心变化会影响速度及 FRS 输入集；复杂重规划期间出现长时间地图更新间隔，后续会进入 WAIT_MAP/NO_PATH。
还需要改善这些问题和规划失败后的恢复，之后再做动态场景验收，不能直接据此宣称真机动态避障已经完成。

复现一个试验（输出目录必须不存在，各并行运行使用不同 ROS_DOMAIN_ID）：

```bash
cd /home/robotx-2/ws_tianji_sharpa/temp/robot_core
source /opt/ros/jazzy/setup.bash
source install/ws_nav_frs/setup.bash

/usr/bin/python3 navigation/scan_planner/tools/validate_geometric_pipeline.py \
  --case crossing --duration 65 --domain 191 \
  --output data/geometric_validation/new_crossing

/usr/bin/python3 navigation/scan_planner/tools/render_geometric_validation.py \
  data/geometric_validation/new_crossing

/usr/bin/python3 navigation/scan_planner/tools/report_geometric_validation.py \
  data/geometric_validation --runs new_crossing
```

`--case` 支持 `static`、`crossing`（静止 3 s 后横穿）、`continuous_crossing`（从开始就横穿）、`dropout`（7–8.5 s 中断深度）。
所有执行器命令使用 `/sim/*`，在独立 ROS 域中运行，不连接真实底盘。
评估必须读取 `evaluation.json` 的 `pass` 或汇总的 `all_pass`；脚本正常退出只表示运行及日志写入结束，不代表导航到达目标。

## 2026-10-21：几何感知模式采用 FRS 软代价与近场硬约束

原先 `chooseTiming` 把全部 3 s FRS 当作硬禁入区；它没有可调的“FRS 代价权重”。点云可见面变化引起的逐帧加速度差分，以及初始速度协方差，会把远期框显著放大。旧 `crossing03` 日志中 3 s 位置框的中位全宽约为 7.65 × 4.87 m，且不包含物理外形膨胀。另有长时间碰撞检查和重规划阻塞单线程地图回调的问题。

现在 `geometric_perception:=true` 自动启用以下实现，旧理想轨迹实验默认仍保留原先硬 FRS 模式以便对照：

- **路径软代价**：轮足网格搜索的每条边代价为 `边长 × (1 + manager.dynamic_frs_weight × 风险场)`，默认权重 1.5。风险场取 0、0.5、1、1.5 s 预测切片的最大值。使用启发式系数 1.8 的加权 A*，不声称求得严格最小代价路径；这是空间风险偏好，尚不是完整的时空轨迹优化。
- **速度软代价**：在速度比例 `1、0.75、0.5、0.25、0` 中，最小化 `J(s) = (1-s)^2 + dynamic_frs_weight × 平均FRS风险`，默认权重 2.0、软预测时域 3 s。先排除违反近场硬约束的候选，再比较软代价。FRS 再大也不会仅因远期重叠就把所有候选判为非法。
- **风险定义**：`exp(-t/1.5) × exp(-max(0, clearance)/0.35)`，多目标取最大值。`clearance` 为候选位置到 FRS 矩形的距离减合成外形半径。它是有界的工程风险指标，不是经校准的碰撞概率；框内风险饱和，避免巨大不确定框产生无界惩罚。
- **近场硬约束**：保留 `0.43 m 机器人半径 + 目标实测半径 + 0.10 m 裕量`、3σ 位置/速度误差，以及 CV 运动外推。检查时域为 `0.5 s + 当前候选缩放前速度 / 0.3 m/s²`。0.3 m/s² 是这套低速仿真的制动假设，实机应依据实测延迟与制动能力配置实现；当前代码不是任意突变运动下的形式化安全证明。当前姿态的独立防碰撞层和传感器失效停车继续生效。
- **加速度估计**：仅对点云类型化目标使用至少 0.25 s、约 0.35 s 的速度窗口差分，减少逐帧差分放大噪声。初始协方差和真实运动变化仍进入 FRS，没有为了图好看截掉预测框。

主要开关位于 `closed_loop_controller` 的 `dynamic_soft_constraint`、`dynamic_frs_weight`、`dynamic_horizon`，空间权重位于规划节点的 `manager.dynamic_frs_weight`。前者决定速度偏好，后者决定绕行偏好，不能只调其中一个就认为已经完成全部动态规划。

同时修复了三类独立问题：

1. 近场碰撞检查限制为 3 s、50 ms 采样；安全定时器只提出重规划请求，不在其中重复运行规划。网格搜索预算 0.25 s、规划尝试间隔 0.4 s，并缓存搜索代价，为地图回调留出处理机会。
2. 局部目标采样恰好达到全局轨迹终点时，初始化目标为终点并处理相等分支，避免未赋值坐标进入 A*。
3. 待确认目标不写入持久静态地图；动态/待确认目标的掩码延伸到其接地区域，避免去地面聚类后漏掉低处点，积累成假墙。掩码仍为 65535 的“跳过射线”，不表示遮挡区已知空闲；待确认目标继续由跟踪与硬约束管理。

### 本轮闭环结果与保留问题

最终验收要求同时满足：距离目标小于 0.16 m、速度小于 0.01 m/s、控制器状态为 `GOAL_REACHED`。不能仅凭轨迹线经过目标或进入目标邻域就算完成。

| 日志目录 | 结果 | 从启动到停车 | 最小外形净距 | 最大地图消息间隔 |
|---|---|---:|---:|---:|
| `soft_crossing_verified` | 到达并停车、无碰撞 | 32.47 s | 0.300 m | 0.067 s |
| `soft_turn_verified` | 突然反向后到达并停车、无碰撞 | 35.52 s | 0.300 m | 0.067 s |
| `soft_continuous05` | **未到达**，转向后 `WAIT_TRACK` | — | 0.615 m | 0.294 s |
| `soft_dropout_verified` | 深度中断保护通过 | 中断后约 0.411 s 停车 | 0.632 m | 0.062 s |

`soft_continuous05` 已经能够规划并运动，但可见墙面截取范围变化会产生虚假的沿墙速度，另有目标表面分裂后的身份丢失，随后触发未解决遮挡保护。下一步应改进几何配准/可观测性、截断表面处理与跟踪关联，不能靠降低硬约束或跳过不健康状态来伪造成功。所有 `soft01`～`soft_continuous04` 等中间失败探针仍保留；报告不是全场景通过或统计成功率声明。

本轮使用真实 ROS 感知、静态建图、规划和控制节点，输入是射线生成的深度与理想里程计/光学 TF，没有向感知或 FRS 注入目标真值。`scan_planner` 的现有运动学模拟器并不等价于 IMU/VINS 定位仿真；本轮不包括 D435i 噪声、IMU 偏置、VINS 估计误差或接触动力学。目标真值仅用于传感器生成、碰撞评估和图中实体叠加。

### 三张中文点云交互图

输出目录：`data/geometric_validation/soft_crossing_verified/paper_figures/`。

- `process_1.png/pdf`：感知、静态点云地图与动态目标识别。
- `process_2.png/pdf`：在线位置 FRS（0.5/1/1.5 s 切片）、近场当前合成半径、交互减速与运动拖影。
- `process_3.png/pdf`：实际运动历史与最终停车到达。

图中机器人采用 `slash_robot_control/assets/mjcf/scene_flat.xml` 的真实 Slash 网格和闭链腿部运动学；MuJoCo 仅负责 EGL 渲染，没有进行动力学重演。地面点来自累计深度观测，障碍物地图来自当时静态占据点云。灰色绘图底板不是被测地图；橙色实体是真值叠加，FRS 是在线估计。PDF 为高分辨率栅格封装，非矢量网格图。图内不画未来真值轨迹；拖影均来自过去实际记录。

```bash
cd /home/robotx-2/ws_tianji_sharpa/temp/robot_core
source /opt/ros/jazzy/setup.bash
source install/ws_nav_frs/setup.bash
# 输出目录必须不存在，避免覆盖实验。
/usr/bin/python3 navigation/scan_planner/tools/validate_geometric_pipeline.py \
  --output data/geometric_validation/my_soft_crossing \
  --case crossing --duration 65 --domain 206

/home/robotx-2/ws_tianji_sharpa/temp/UniLab/.venv/bin/python \
  navigation/scan_planner/tools/render_pointcloud_interaction.py \
  data/geometric_validation/my_soft_crossing \
  --output data/geometric_validation/my_soft_crossing/paper_figures
```

针对性测试包括：大范围外层 FRS 不再强制停车、硬外形碰撞不能用权重交换、权重确实改变速度偏好、窗口差分抑制抖动且保留真实加速度、待确认目标与接地带不进入静态图，以及已有跟踪/失联/高度安全测试。

## 预建静态底图 + 实时新增/动态目标：部署入口

本节取代上一节“尚未接入预建地图”的描述。实现不依赖学习权重，不向跟踪器输入动态目标真值。实机与仿真使用相同的 `geometric_obstacles`、`GridMap`、轮足路径搜索和闭环控制器；底图通过 ROS `PointCloud2` 输入，PCD 由现有 `map_generator/map_pub` 加载。

### 数据链路与保护

1. `static_prior.py` 对预建地图做 5 cm 体素降采样并建立一次 KD-tree。导航期间冻结首份有效地图，避免在线动态点污染背景；替换底图需重启感知及重新规划。
2. 每帧深度使用曝光时刻的 `world ← optical` 投影，再使用同一时刻的 `map ← world` 与底图匹配，默认匹配距离 0.10 m。只把未匹配点送入聚类、关联和速度估计，已知墙面不再估速。地图外新点仍需经过静止/运动确认。
3. 原始实时深度仍参与近场静态避障，背景匹配不会把近墙的新障碍直接清为空闲。待确认与动态目标由独立轨迹管理，掩码仍代表“省略射线”。新增静止目标进入在线占据层，既有底图始终独立保存。
4. 感知节点按 5 Hz 发布半径 8 m 的局部底图到 `/dynamic_obstacles/static_prior`，转换到连续 `world` 坐标。规划器把这层与实时占据层联合查询；在线空闲射线、衰减和局部地图重置不能清除底图。
5. 规划器更新完底图后发布 `/grid_map/static_prior_applied` 确认。控制器要求该确认在 0.8 s 内有效；没有底图、精确时刻 TF 不可用、感知不健康时不放行。明显 `map/world` 跳变沿用故障锁存，需重启感知并重新规划，不能把重定位修正当作目标速度。

`map` 是底图坐标，`world` 是现有 VINS 连续里程计坐标，二者不要求相同。PCD 中坐标必须使用米，`static_prior_frame` 必须填写文件真实坐标系，不可只改名称来冒充对齐。`geometric_ground_z` 仍对应连续 `world` 中地面高度。预建点云只有占据表面，不意味着其余空间已被观测为空闲；底图模式不启用未经观测的横梁净空放行。

### 实机启动

目标机沿用项目现有 ROS 2 Humble、D435i 与 VINS/RTAB-Map 环境。新增依赖已经写入包清单：`sensor_msgs_py`、NumPy、SciPy、OpenCV 等。部署前在目标机重新编译：

```bash
cd /home/robotx-2/ws_tianji_sharpa/temp/robot_core
./build.sh ws_nav
# 默认读取定位模式发布的 /cloud_obstacles，自动开启几何感知与底图模式。
./scripts/run/run_static_frs_navigation.sh
# 已经单独启动定位时，可用：
./scripts/run/run_static_frs_navigation.sh --planner-only
# 停止导航，保留定位：
./scripts/run/run_static_frs_navigation.sh stop --keep-loc
```

包装脚本固定 `GEOMETRIC_PERCEPTION=true`、`STATIC_PRIOR=true`，其余接口继承 `run_vins_navigation.sh`。若由包装脚本启动定位，RTAB-Map 会启用 `Mem/InitWMWithAllNodes=true` 和 `GridGlobal/MaxNodes=0` 加载完整已保存背景。若复用已运行的定位进程，则复用它当前发布的底图；该话题应为预建全局障碍地图，不能替换成实时局部深度云。

直接使用导出的 PCD 时，在目标机设置真实路径：

```bash
STATIC_PRIOR_PCD=/绝对路径/静态地图.pcd \
STATIC_PRIOR_FRAME=map \
./scripts/run/run_static_frs_navigation.sh
```

PCD 由 `map_pub` 发布到 `/navigation_static_map`，支持 PCL 可读取的 PCD 格式。也可通过 `STATIC_PRIOR_TOPIC=/你的静态地图话题` 指定其它预建地图源。默认 `STATIC_PRIOR_MATCH_DISTANCE=0.10`，它是匹配容差，不是障碍物硬安全半径。底图质量、分辨率和定位误差不匹配时应检查坐标/标定，而不是无限增大过滤距离。

底层 launch 同样可直接使用：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true robot_model:=slash sensor_type:=depth controller_mode:=closed_loop \
  geometric_perception:=true static_prior:=true static_prior_topic:=/cloud_obstacles
```

**部署边界：**本机已完成 Jazzy 环境编译、ROS 通信/控制测试、实机 launch 的离线构造检查和无头仿真；未操作真实底盘，没有验证目标机的 Humble/ARM 构建与实际 D435i/VINS 误差。当前本机 `data/rtabmap_maps/` 只有配置，未提供真实地图数据库或 PCD。目标机需要已有的 RTAB-Map 数据库（默认 `rtabmap_vins.db` 或 `DB=` 指定）与 `map → world` 定位链路。“无需训练”不等于无需地图、TF、标定或制动能力验证。未进行实机零碰撞保证。

### 可复现验证

仿真先生成静态墙面 PCD，通过同一个 `map_pub` 和底图话题加载；动态箱体只出现在模拟深度里。新增静态箱体故意不加入 PCD。`--shifted-map-frame` 给底图添加平移和旋转，通过 TF 恢复匹配，不使用坐标系名称相同的特例。

```bash
source /opt/ros/jazzy/setup.bash
source install/ws_nav_frs/setup.bash
/usr/bin/python3 navigation/scan_planner/tools/validate_geometric_pipeline.py \
  --output data/geometric_validation/my_prior_run --case continuous_crossing \
  --duration 80 --domain 214 --static-prior --shifted-map-frame
```

本轮最终配置使用保守的未知空间策略，验收同时要求进入 0.16 m 目标范围、速度低于 0.01 m/s、控制器报告 `GOAL_REACHED`：

| 日志 | 结果 | 到达并停车时间 | 最小外形净距 |
|---|---|---:|---:|
| `prior_continuous_final` | 启动即运动，成功、无碰撞 | 42.48 s | 0.268 m |
| `prior_turn_final` | 突然反向，成功、无碰撞 | 34.41 s | 0.300 m |
| `prior_new_static_final` | 底图外新增静态箱体，成功、无碰撞 | 30.48 s | 0.220 m |
| `prior_dropout_final` | 深度中断，保护停车通过 | 约 0.412 s 停车延迟 | 0.632 m |

这三次导航记录中，观测为动态目标的墙面条目为 0；这不是任意地图/定位误差下的保证。`prior_continuous_final` 中约 39.8% 的有效深度采样点被识别为背景并排除出聚类输入，背景查询加跟踪耗时 P95 约 3.25 ms（本机、该场景，不代表实机性能）。未匹配点中仍有随后由地面过滤排除的地面点。早期 `prior_continuous01` 曾发生遮挡等待后恢复，原始日志保留，不能把这些有限运行解释为统计成功率。

报告：`data/geometric_validation/prior_validation_report.json/txt`。

三张中文渲染：`data/geometric_validation/prior_continuous_final/paper_figures/process_1/2/3.png` 及同名 PDF。图中使用预建底图＋实时点云、真实 Slash 网格、在线 FRS 和实际运动拖影，最终帧速度为零。图示来自软件运动学闭环，MuJoCo 仅用于 EGL 渲染，不是实机录像或接触动力学验证。

新增测试覆盖已知墙面过滤、新目标保留、不同坐标系匹配、缺失底图等待、底图不被空闲证据/重置清掉、错误坐标系拒收，以及控制器不得绕过规划底图确认；已有 FRS、跟踪和高度安全测试亦通过。

## 2026-10-21：FAPP 几何判据与可见面漂移修复

修改前检查点：`4f7ceb0`（只提交源码、配置与工具，原始深度与仿真输出留在本地）。
参考本机 `temp/nav/FAPP/src/mot_mapping/src/mapping_manger.cpp` 的最近邻距离均值、
归一化方差和已有运动轨迹保持思路，独立实现于 `geometric_obstacles/core.py`。
没有复制 FAPP 的 ROS1 节点、固定 0.02 s 差分或仿真目标真值输入。

- 保留预建静态底图、曝光时刻 TF、实际时间间隔 KF 与 Hungarian 一对一关联。
- 每个簇计算与 0.5 s 历史点云的最近邻距离均值和归一化方差，作为几何运动证据。
- 深度相机的平面表面可能在真实平移时大面积重叠，因此上述证据不作为必需门槛；
  另外用最近 0.6 s 可见面补偿后的位移（跨度至少 0.3 s）与滤波速度一致性确认运动。
- 每条轨迹保留曾经观测到的 XY 最大支撑尺寸。可见宽度缩小超过 0.08 m 时，
  用仍可见的边界和历史尺寸构造候选中心，按预测状态选择相容边界；关联和更新使用同一观测。
  不再直接对缩小的包围盒中心求运动。尺寸随轨迹退休清除，不能靠短时超时强制遗忘，
  否则长期侧面遮挡又会产生一次虚假速度。
- 部分可见时增大观测协方差；边界解释接近时进一步增加不确定性，不把补偿当作精确测量。
  半径按补偿中心到当前观测边界的距离重算，避免平移中心后缩小实体保护范围。
- `/dynamic_obstacles/perception_stats` 增加 `visibility_corrected_tracks` 供部署排查。
- FRS 预测器、3-sigma 传播、代价权重和近场硬约束本次未调整。

新增 ROS 参数（由 Config 自动声明）：`geometry_mean_distance=0.025` m、
`geometry_relative_variance=0.8`、`visibility_extent_tolerance=0.08` m。
它们针对几何一致性，不是已标定的概率阈值。

### 相同输入的对照

`tools/compare_perception_replay.py` 从 Git 检查点导入旧跟踪器，基于原运行记录的相机
轨迹与传感器时间重建同一射线深度，分别交给新旧跟踪器。地图、像素和姿态输入完全相同；
真值仅参与射线生成及误差评估，不送入跟踪器。
`tools/evaluate_frs_replay.cpp` 直接调用生产 `IntentFrs` 头文件，用相同传播算法比较
估计差异；离线 FRS 以观测时刻为起点，未加在线消息时延，不能与在线画面逐数等同。

```bash
g++ -std=c++17 -O2 -I/usr/include/eigen3 \
  -Inavigation/scan_planner/src/planner/plan_manage/include \
  navigation/scan_planner/tools/evaluate_frs_replay.cpp -o /tmp/evaluate_frs_replay
/usr/bin/python3 navigation/scan_planner/tools/compare_perception_replay.py \
  data/geometric_validation/prior_turn_final \
  --frs-evaluator /tmp/evaluate_frs_replay \
  --output data/geometric_validation/fapp_replay_final.json
```

4.0–7.9 s（折返前）纵向速度 RMSE：0.08336 → 0.01192 m/s；横穿速度 RMSE：
0.01542 → 0.01276 m/s。该段 1.5 s 位置 FRS 平均面积：3.480 → 2.906 m²，缩小约 16.5%。
6.519 s 的纵向速度：−0.1854 → +0.00643 m/s；1.5 s 预测中心 x：1.074 → 1.796 m，
真实障碍物 x 为 1.8 m。相同时刻预测矩形由 2.480×1.712 m 变为 1.914×1.607 m。
这说明主要方向漂移显著改善，但并未解决所有保守性；折返瞬间范围仍可能扩大。
不要将上述选定匀速段的改善推广成全时域都更小或安全概率保证。

### 验证和边界

新增长期单面可见、静止物体可见面丢失、面重新出现、真实纵向运动、变帧率折返、
几何抖动、重叠表面运动等回归测试。`colcon test` 的几何单元测试与 ROS 管线测试均通过。
真实 ROS 节点的软件闭环另行记录于 `data/geometric_validation/fapp_*`，汇总见
`results/perception_repair_20261021.json`。所有失败尝试仍在本地保留：
`fapp_turn01` 因把几何阈值设为硬门槛而漏判动态目标，已被否决，不计为修复成功。
`fapp_continuous_final` 虽到达，但第一条轨迹约 10 s 才产生，没有活动目标参与控制器 FRS，
因此没有通过该场景的动态交互验收，仍需进一步调查规划启动等待。

支撑尺寸补偿是刚体近似。目标大幅旋转、变形、合并/分裂、初见时仅有一个面，仍可能造成
中心偏差或尺寸偏大；不能恢复从未观测过的真实物体中心。本次未用 D435i/VINS 实机数据、
接触动力学或统计种子组验证，属于有回归证据的实验性修复，不能宣称零样本部署已经通过。

## 2026-10-21：三层背景管理与动态轨迹保护

在 `bdb306f` 的可见面/速度修复基础上，新增 `geometric_obstacles/online_background.py`：

1. 固定底图仍由 `StaticPrior` 冻结保存，导航中不修改它。
2. 新物体先聚类跟踪；只有 observed、已判 static、速度小于 0.06 m/s，且在 0.06 m
   位置范围内连续稳定至少 2 s 才转交在线静态增量层。确认阶段仍保持原有地图/轨迹保护。
   只收录当前实际看见的点，不填充未观测物体体积。转交后停止对该静态簇重复跟踪。
3. 当前点同时对固定底图、在线增量层做差分。已有动态/待确认轨迹附近的在线背景点
   重新开放给跟踪；对固定底图只放行“不精确贴合墙面”的疑似前景点，不能把整个墙面 ROI
   放回来制造动态墙面。高置信匹配阈值为 min(底图距离容差/2, 底图体素边长×0.75)。

在线点投影到曝光时刻的相机，只有 3×3 邻域所有深度有效、在传感器量程内、且比该点
远至少 0.15 m，才累计一次可见空闲证据；连续 3 帧满足后清除该在线点。遮挡、无效深度、
视野外、量程外与中断均不作为空闲，且会打断连续证据。发现空闲后，原物体的剩余表面在
1.5 s 内重新开放检查，从而检测重新运动，而不是永久当成背景。

`/dynamic_obstacles/static_prior` 发布的是“固定底图局部裁剪＋当前在线增量”的完整快照；
GridMap 的原有替换接口同步接收增删。在线清除绝不直接擦除固定底图，实时深度占据层仍由
实际自由射线更新。没有自由空间证据的旧在线点可能保守留存，这是遮挡安全策略，不是 TTL。

默认启用 `online_background_enabled=true`。节点 ROS 参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| online_voxel | 0.05 m | 在线体素分辨率 |
| online_match_distance | 0.07 m | 在线背景匹配半径 |
| online_confirm_seconds | 2 s | 已判静态后的持续确认时间 |
| online_stable_distance | 0.06 m | 静态确认期间允许的位移 |
| online_stable_speed | 0.06 m/s | 静态确认速度上限 |
| online_free_margin | 0.15 m | 可见自由证据的深度余量 |
| online_free_frames | 3 | 连续自由观测数 |
| online_audit_seconds | 1.5 s | 局部变化后的重检时长 |
| online_max_points | 20000 | 在线点数上限 |

容量达到上限时拒绝新增转交，继续保留跟踪，不淘汰未确认空闲的旧障碍。
诊断话题增加 `online_background_points`、`online_promotions`、`online_cleared_total`、
`online_cleared_this_frame`、`online_matched_points`、`protected_background_points`。

验证包含：静止确认与中断重置，遮挡/NaN/Inf/量程外不删除，重复时间戳不累计，自由证据
清除与剩余表面重检，贴墙前景保留且真实墙面仍过滤，相机位姿变换、容量限制；ROS 测试
实际发送深度并检查增量入图、恢复动态、清除出图以及底图不变。地图层既有 C++ 回归也通过。
`validate_geometric_pipeline.py --case stop_restart` 新增先静止到 6 s 再横穿的射线深度场景，
用以核验真实节点的静态转交与动态恢复。结果见 `results/background_layers_20261021.json`。

边界：不能仅由地图差分发现与既有墙面在测量精度内完全重合的目标，也不能从遮挡中推断
物体已离开。动态目标的初次检测仍需要几何或连续运动证据；贴墙阈值依赖定位和深度误差。
没有将仿真真值送入感知节点，也未宣称 D435i/VINS 实机验证完成。

这里的“背景差分膨胀”已经由最近邻匹配容差实现（固定底图默认 0.10 m，在线表面 0.07 m），
与导航的机器人实体半径膨胀不同，不能互相代替。容差内新采样点被背景吸收；超出容差但靠近
休眠静态物体的新表面会重新激活原 ID 和支撑尺寸，重新静态确认后扩充在线表面。重检活动
期间持续开放整个物体，不因短时审查窗口结束又只剩一个新露出的窄条。真正的动态轨迹仍有
背景匹配保护；未观测区域不会仅凭大幅包络膨胀被宣布为静态。

## 2026-10-21：行进中横穿冲突与冻结时间对照

实验参数见 `experiments/encounter_soft_frs.json`，配套命令：

```bash
cd /home/robotx-2/ws_tianji_sharpa/temp/robot_core
bash navigation/scan_planner/tools/run_encounter_comparison.sh encounter_reproduce 201
```

该脚本顺序运行软风险与硬 FRS 两种策略，并输出比较报告；输出目录已存在时拒绝覆盖。
机器人先行进到 x=0.6 m，障碍物才从 (1.8,-1.1) m 以 0.22 m/s 横穿，最后停于 y=0.8 m。
障碍物半宽 0.22 m、高 0.9 m。触发时机器人速度约 0.117 m/s；若继续保持该速度与航向，
开环计算的包络净距约为 -0.18 m，说明场景存在实际通行冲突，而非互不阻碍的展示。
真值只用于射线生成、运动学与评价，不输入感知或预测。

匹配设置：软时序权重 2.0，预测时域 3 s；本组空间风险权重置 0，以单独评价速度时序。
深度图 320×180、fx=fy=170 px（水平视场约 86.5°）；位置观测噪声尺度 0.06 m、过程加速度
尺度 0.2 m/s²。机器人半径 0.43 m、硬检查裕度 0.10 m、独立保护与失效停车均保留。
这些是理想深度实验的设置，不写回部署默认值，实机视场由 CameraInfo 与标定决定。

每种策略重复两次，均无碰撞到达，最小实体包络净距 0.15 m。交互触发后累计停车
（|v|<0.01 m/s，剔除到达状态）均值：硬 10.07 s、软 7.20 s，减少 28.5%；最长连续停车
均值：硬 9.34 s、软 5.43 s，减少 41.9%。交互到达用时仅减少 9.5%，不能将停车减少比例
当成整体效率提升。部分运行触发独立保护，因此无碰撞结果属于整个控制链路。

所有开发尝试保留。仅降低风险权重几乎无效；较窄视场下，软策略提前推进后会出现约 21 s
的遮挡等待，使总停车更长。扩大仿真视场并让目标在可观察侧方停止后，两种策略才无此额外
等待。该结果不意味着解决了任意遮挡下的冻结。包括突然启停在内，y 速度的经验 3σ 覆盖率
约 92%–95%，不支持概率安全保证。完整对照、估计误差与失败探索记录见
`results/encounter_comparison_20261021.json`；这是重复软件集成验证，不是实机统计安全证明。


## 2026-10-21：三动态障碍物闭环验证

复现命令（需已构建 `install/ws_nav_frs`）：

```bash
bash navigation/scan_planner/tools/run_multi_encounter_comparison.sh multi_reproduce 201
```

场景和参数见 `experiments/multi_encounter_soft_frs.json`。机器人从 (0,0) 到 (4.9,0) m，
三个障碍分别在 x=1.8、2.5、3.7 m 横穿，速度均为 0.22 m/s。第一目标在机器人到达
x=0.6 m 后启动，第二目标在其后 6 s 反向横穿，第三目标在机器人到达 x=2.5 m 后启动。
三个物体从场景初始化即存在；最多两个真实运动目标被同时观测。真值仅用于深度射线、
运动学回放和离线评价，不送入感知或 FRS。在线链路使用实际 ROS 感知、地图、预测及控制节点。

固定底图保护区增加局部平面一致性判定：稀疏墙面采样导致最近邻点距偏大时，只有局部
邻域充分、平面性成立且法向残差不超过 2 cm 的已匹配点才保留为背景，避免墙面虚假动态
目标阻断通行。不扩大整个背景匹配半径；回归检查保留离墙 8 cm 的前景，拒绝非平面邻域。
`geometric_obstacles` 构建及测试通过（测试汇总 46 项，0 失败）。

最终匹配实验每种策略运行一次：

| 指标 | 软 FRS | 硬 FRS |
|---|---:|---:|
| 到达 / 碰撞 | 是 / 无 | 是 / 无 |
| 首次交互至到达 / s | 54.01 | 72.14 |
| 交互后累计停车 / s | 15.76 | 27.83 |
| 最长连续停车 / s | 5.90 | 13.35 |
| 最小实体包络净距 / m | 0.15 | 0.15 |

软策略累计停车减少 43.4%，最长连续停车减少 55.8%。三个目标均在真实运动期间被识别
为动态目标，并分别成为减速或停车决策的限制目标。空间风险权重仍为 0，本实验只比较
FRS 时序软代价与硬排除；实体硬检查和独立保护保留。

软策略第三个目标发生两次 ID 重获，独立保护触发两次，停止后的动态分类滞后仍存在。
因此不能把无碰撞归因于软代价单独提供安全保证，也不能以这一组开发场景证明普遍性能。
首轮墙面误跟踪导致未到达；第二轮虽到达，但第二目标未独立影响软策略，均保留为未通过
的开发记录。结果及统计口径见 `results/multi_encounter_20261021.json`，原始日志见
`data/geometric_validation/multi_encounter_*_03`。这是理想位姿及射线深度的软件闭环验证，
尚非 D435i/VINS 实机验证。

回放命令（需要含 pyrender 的 Python 环境）：

```bash
python navigation/scan_planner/tools/render_frs_animation.py \
  data/geometric_validation/multi_encounter_soft_03 \
  --output output/multi_encounter_frs_animation --step .75 --fps 4
```

GIF 显示仿真实体与在线感知产生的 FRS 位置截面，播放为 3 倍速，渲染不重新生成预测。

## 室内转弯、低横梁与多目标感知组合验证

入口为 `tools/run_indoor_turn_height.sh [输出前缀] [ROS域]`，参数摘要见
`experiments/indoor_turn_height.json`。场景参考现有 `simulator.yaml` 的室内横梁设置
（净空 0.62 m、纵深 0.6 m），以及 `validate_dynamic_navigation.py` 中门洞前后
交错横穿的布置。动态实体采用地面二维移动的方箱；没有沿用旧实验直接注入理想目标轨迹
的接口，也没有声称复现原随机十目标场景。

A=(0,0)，走廊转角路由点=(4,0)，B=(4,4.7) m。路由点是明确配置的导航任务输入；
每段轨迹由真实规划器在线生成，局部规划接收静态底图、深度感知和在线 FRS。此试验验证
带路由点的室内导航集成，不等同于无路由提示的全局拓扑规划。三个障碍物分布在转弯前、
转弯后以及低横梁之后，通过机器人进度触发运动，避免仅有动态物体却不发生交互。

场景暴露了原直走测试未覆盖的搜索问题：0.25 s 搜索预算不足；搜索安全余量小于后续
轨迹复核时会反复拒绝已找到的路线。本场景使用 1 s 搜索预算、0.50 m 搜索包络和
0.05 m 天花板余量，保留控制器原安全检查。实体评价仍为半径 0.43 m，横梁净空要求
不低于 0.03 m。空间 FRS 权重 0.5、时序权重 2，均为软代价；实体硬检查仍保留。
高度由控制器根据轨迹和感知净空主动调整，运动学执行器速率限制为 0.08 m/s。

`report_indoor_navigation.py` 对实际日志检查：无碰撞到达；经过横梁且净空达标；
腿高下降到 0.215 m 以下并在终点恢复；航向变化超过 1 rad；三个目标均在实际运动期间
被识别，至少两个目标实际限制减速/让行决策。顺序交互场景不强制三个目标同时运动；
同时检测数、ID 重获、感知等待及独立保护触发次数单独报告。统计不使用渲染插值轨迹。

探索记录保留：01 搜索超时；02 轨迹复核失败；03 目标停在切角路线附近导致持续阻塞；
04 转向后目标脱离视野导致失效停车；05 新增布尔字段的 JSON 序列化失败、未产生完整
原始记录，不能用于统计。所有实验均为理想位姿、射线深度和受限高度运动学的软件闭环，
不包含腿部接触动力学、D435i 深度误差或 VINS 漂移。

完成记录 `indoor_turn_height_soft_06` 通过组合验收：118.18 s 到达，无实体碰撞，
最小实体净距 0.15 m；腿高 0.25→0.15→0.25 m，过梁最小顶部净空 0.1098 m。
三个目标均被在线识别，目标 0 和 2 在运动期间分别限制 59、61 次决策；目标 1 虽被识别，
但未成为 FRS 限速决策的限制目标。本组合是顺序交互，最多同时观测 1 个真实运动目标，
不能替代前述双目标同时运动的压力测试。仍有 10.02 s 感知等待、目标 2 一次 ID 重获；
名义直角路线累计航向调整为 181.52°，不能声称已消除转向修正和冻结。
完整结果见 `results/indoor_turn_height_20261021.json`，回放位于
`output/indoor_turn_height_frs_animation/frs_perception_navigation.gif`。

## 转弯时虚假运动与感知等待的修正

基线 `indoor_turn_height_soft_06` 在 56.07–66.09 s 进入 `WAIT_TRACK`（10.02 s）。
深度结果仍约 40–55 ms 新鲜，故不是推理耗时十秒。转弯时目标 2 尚未运动，远处部分
表面逐渐进入量程和视场，包围盒中心从约 x=3.12 移至 2.84 m，产生最高 0.539 m/s
的虚假速度。目标随后失去观测，预测漂移及协方差扩大使自由空间消退条件无法满足，
该历史实验版本将整个目标数组标记为 `unresolved occlusion`，控制器停车。
当前版本已改为逐目标保守预测，具体规则见“FRS 连续跟踪与遮挡恢复”。

跟踪器现在区分两种变化：既有的表面缩小补偿，以及新增的单侧表面扩张补偿。
扩张时使用与预测支撑面一致的边界，显式计算支撑中心的坐标变化；同步平移位置状态
与短时观测历史，保留速度状态，再由保留面的残差估计真实运动。对称扩张不会额外平移。
这仍是刚性支撑近似，不声称能够消除任意旋转、形变与遮挡歧义。没有缩短遮挡等待阈值、
丢弃未知目标、放宽健康检查或扩大相机视场。

提前恢复通行后还暴露了斜视接地掩膜的缺口：原代码从所有采样列的最大行号向下扩展，
会遗漏其他列较高的底部轮廓，在静态地图留下 z≈0.075 m 的动态物体条带，导致重规划
反复失败。现在从可见包络覆盖至投影地面，包括采样间隙；像素编码仍为 65535，即
忽略该射线，不清空已有占据，也不把遮挡区域当作自由空间。

新增测试覆盖首次局部可见后单侧扩张、扩张同时具有真实双轴运动、对称扩张，以及
斜视接地轮廓漏掩膜；既有真实遮挡保留/可见自由空间消退测试仍通过。ROS 包构建及
完整测试汇总为 50 项通过。`report_perception_wait.py` 将 `WAIT_*` 与主动 FRS 让行
`DYNAMIC_WAIT` 分开统计，防止仅替换状态名称就宣称冻结减少。

同场景固定参数复测 `indoor_turn_height_growth_02/03` 均通过组合验收。首次交互后
感知等待由 10.020 s 降为 0 / 0.061 s，初始静止目标 2 的错误动态观测由 27 帧降为
0 / 0 帧。两次都无碰撞、实体最小净距 0.15 m，腿高仍为 0.25→0.15→0.25 m，
横梁顶部净空约 0.1098 m；三个目标均实际限制 FRS 决策，独立保护触发均为 0 次。
到达时间 111.52 / 124.22 s（基线 118.18 s），累计停车 20.90 / 21.59 s（基线
25.68 s），因此只确认误等待减少，不宣称整体效率稳定提升。第三目标仍有 1 / 6 次
离线关联 ID 重获；第二次仍有短暂的 tentative 速度尖峰，但未确认成虚假动态目标。
仅做扩张修正的 `growth_01` 因上述接地掩膜问题未到达，保留为失败开发记录。

结果见 `results/perception_wait_fix_20261021.json` 与
`results/indoor_turn_height_wait_fixed_20261021.json`；新回放位于
`output/indoor_turn_height_wait_fixed/frs_perception_navigation.gif`。复现沿用
`tools/run_indoor_turn_height.sh`，需先构建更新后的 `geometric_obstacles` 包。

四阶段论文过程图可由 `tools/render_indoor_storyboard.py` 从修复后的同一运行生成：

```bash
python navigation/scan_planner/tools/render_indoor_storyboard.py \
  data/geometric_validation/indoor_turn_height_growth_02 \
  --output output/pdf/indoor_navigation_process
```

依赖 MuJoCo/EGL、Pillow、NumPy 和 ReportLab。输出独立四图、2×2 与横向四联图的
300 dpi PNG/PDF、中文图注及来源清单。四个关键时刻为 12、48、80、111.52 s，分别
表示首次交互、转弯后交互、降高过梁与让行、恢复高度到达。A/P/B 标记任务起点、
配置的转角路由点及终点。横梁点云在展示时剖隐、轮廓透明化以显示低姿态；感知、轨迹
和 FRS 数据均未改动。最终帧不展示未来预测，所有拖影均来自该运行的历史位置。

## 静态平面任务的连续速度跟踪

静态差速任务可使用 `static_spatial_tracking`。该分支仅在动态预测关闭、
高度安全控制关闭、横向速度上限为零且轨迹高度基本恒定时工作。
FRS 平面轨迹由 `frs_spatial_tracking` 接入空间跟踪；高度变化片段继续使用
原时序控制。两种分支都保留原有高度、感知时效和动态安全检查。

- `manager.shortcut_wheel_path`：对栅格路径做逐段碰撞检查后捷径化，减少细碎转折；
  平滑曲线检查失败时回退原栅格路径。存在动态 FRS 目标时保留风险搜索路径；仅新鲜、健康、空目标快照允许静态捷径。
- `static_spatial_tracking`：按空间进度跟踪，替换路径时保持命令速度，并按曲率、
  曲率变化率和剩余距离进行预减速；线速度与转向同时计算。
- `static_cruise_speed`：静态巡航目标，默认 0.5 m/s，同时受 `max_vx` 限制。
- `static_acceleration_limit`：正常跟踪时的线速度变化率上限，默认 0.5 m/s²；角速度仍受
  `max_vyaw` 限制，角速度变化率限制为 2 rad/s²。
  紧急停车及感知数据异常停车可覆盖上述变化率限制。
- `manager.static_tracking_margin`：静态轮式捷径模式的额外跟踪裕量。
  搜索圆盘至少覆盖执行包络的外接圆及此裕量，实体执行包络保持原尺寸。
  此裕量会使窄通道规划更保守；关闭静态捷径及空间跟踪可恢复原分支。

控制器通过 `planning/execution_phase` 回传轨迹 ID、空间进度对应的样条时间、
速度及加速度上界。规划器拒绝过期轨迹 ID，并按当前执行速度估计接近障碍的时间；
机身已进入地图安全包络的占据区域时停车。里程计超时或非法时停止，恢复后重新限加速度。
该机制用于修正执行时标，并不构成对任意环境的安全保证。

`planning/tracking_metrics` 记录速度、转向、跟踪误差和控制计算耗时；
`WHEEL_PLAN_MS` 日志记录单次轮式规划的墙钟耗时。
数值测试为 `test_spatial_tracker`、`test_wheel_timing`，覆盖路径切换速度保持、
弯道跟踪、终点停车、运动限制、障碍捷径检查和到达障碍时间估计。

空间跟踪同时订阅 `grid_map/occupancy`。该点云必须与 `body_pose` 使用相同坐标系，
分辨率通过 `static_map_resolution` 指定，默认 0.05 m；地面高度通过
`static_ground_height` 指定。静态执行包络参数为 `static_body_radius` 和
`static_body_offset`，Slash 默认沿用 0.30 m 半径、0.18 m 前后偏置。

跟踪层对候选速度进行包含反应时间和减速过程的近场轨迹检查；正常候选不可行时，
缩短前视距离或减速，仍不可行时停车并请求重新规划。大方向误差采用对齐滞回，
防止在很近的前视点周围低速绕圈；`static_progress_timeout` 默认 2.5 s，
实际位置长期不推进时请求刷新路径。此处不删除机身碰撞检查。

新增数值测试还覆盖近场制动轨迹和大方向误差恢复。可在完成带测试的编译后执行：

```bash
source install/ws_nav_frs/setup.bash
ctest --test-dir build/ws_nav_frs/scan_planner \
  -R 'test_(spatial_tracker|wheel_timing)$' --output-on-failure
```

常规 Slash 仿真和几何 FRS 入口默认使用高度安全控制；这些入口保留原有几何
走廊配置。高度变化片段保留时序控制，平面 FRS 轨迹使用空间跟踪。静态空间跟踪的 BARN 验证入口位于论文工程的
`tmp/barn_ros2_benchmark/run_spatial_release.sh`，只验证固定高度静态条件。

2026-11-12 最终复测：BARN 0、6、12 各运行三次，空间跟踪与同批 MPPI
均为 9/9 到达、零接触碰撞，平均仿真导航时间分别为 26.57 s 和 26.74 s。
此前静态走廊复用版本在三个场景各一次的均值为 42.23 s；本次观测降幅为
37.10%，不代表跨场景统计优势。最终版本采样最小实体圆盘净距为 0.075 m。
两者统一速度上限 0.5 m/s、角速度上限 1 rad/s、目标容差 1 m；
MPPI 保留自身代价与运动配置。BARN 的 0.334 m 实体圆盘仅用于基准，
生产 Slash 保留自身几何。原始记录与图表位于
`tmp/barn_ros2_benchmark/results/spatial_release`、`spatial_mppi` 和
`spatial_comparison`（均相对于论文工程）。

实际 `robot_core` 工作空间已完成原生编译，两项 CTest 通过；已安装控制器的
ROS 接口测试通过路径切换连续性、过期里程计停车与进度保持、非法里程计恢复及
非平面轨迹回退检查。构建记录见基准目录的 `production_release_build.log`。


## FRS 连续跟踪与遮挡恢复

`frs_spatial_tracking=true` 默认启用平面 FRS 轨迹的空间进度跟踪。
轨迹换接不重置命令速度；线速度、转向、曲率与制动检查使用同一执行状态。
FRS 的五档候选按差速模型、速度变化与 0.1 s 反应时间进行前向模拟；
未来检查位置与决策日志中的候选轨迹一致。候选比例作为巡航速度上限，
不反复乘到上一帧已经缩小的速度上，避免非零比例最终变成近乎停车。
比例零仍停止前进和转向。

只有新鲜、健康且完整的空目标快照允许无高度控制的平面任务回到静态空间跟踪。
此时规划器也可复用静态走廊；出现目标、快照异常或超时后立即撤销这一条件。
不能用不健康快照或“未检测到新目标”代替健康空目标快照。高度控制任务仍在
同一控制回调中执行高度限制，空间进度不会绕过降身等待。

遮挡是逐目标的预测状态，不再直接将正常处理的整帧标记为感知失效。
跟踪器在可信失观窗口内保留目标 ID、预测状态和增长的协方差，并标记 `COASTING`；
超过 `lost_timeout` 后转为固定 `UNKNOWN` 区域，不再外推运动，见文末生命周期规则。
接收端将其状态对齐到快照时刻，不二次传播整段失观年龄，也不从预测值学习加速度。
健康完整快照中的新预测 ID 可按其状态与协方差接管，但不创建虚假的加速度观测历史；
已有 ID 的异常跳变、非法协方差、错误坐标系仍使输入无效。
目标只有在观测到自由空间或离开局部管理范围后才退出，遮挡不会生成健康空场景。
新鲜深度、位姿或地图缺失仍按原规则停车。

`dynamic_safety_margin` 默认 0.20 m，作用于近场实体检查，不把外层 FRS 改成硬约束。
这是本次连续运动回归中补足的近场跟踪/感知裕量，独立于静态路径的 0.125 m 搜索裕量。
`odom_twist_in_world` 默认 false，遵循 ROS 里程计机体系速度约定；若上游发布世界系
线速度，必须显式设为 true。内置几何仿真发布世界系速度，验证入口已明确设置该项。

回归覆盖 FRS 换轨速度保持、健康空目标切换、感知/静态地图超时停车、远处遮挡继续
运动、近处遮挡保留硬安全约束、预测跳变拒绝、输入协方差与非零速度比例收敛。
实验脚本与原始记录在论文工程 `tmp/barn_ros2_benchmark/frs_continuity_release`。


本次回归结果：FRS 开启的 BARN 0/6/12 各一次，24.1/38.6/23.2 s 全部到达且无
接触碰撞，最后构建补充检查 BARN 12 为 22.5 s。三目标闭环中，修复后的软风险从
下发目标到达用时 41.03 s，动态让行 14.70 s、感知等待 0 s；同配置硬约束为
47.50 s、28.60 s、0 s，均无碰撞且最小实体净距 0.15 m。旧软风险组为 64.77 s，
其中感知等待 19.01 s。每种配置仅一次，这些数值是整体修复的实测而非单因素消融。
室内转弯/降身/三目标综合回归 `combined_pass=true`，首次下发目标后约 69.57 s
到达，无碰撞，最小实体净距 0.15 m，过梁竖向裕量至少 0.111 m，最终腿高恢复至
0.25 m。上述动态仿真使用真实 ROS 感知与规划节点、理想里程计和运动学机器人。
五项 CTest 和 48 项感知测试通过，原始数据、构建记录、候选失败记录及配置说明均保留。

## 最新静态与动态基准入口

两类基准分别整理在 [`../benchmarks/static_barn/`](../benchmarks/static_barn/) 和
[`../benchmarks/dynamic_dynabarn/`](../benchmarks/dynamic_dynabarn/)。构建与运行方法见
[`../benchmarks/README.txt`](../benchmarks/README.txt)。试验工作空间直接链接本目录源码，
包括空间连续跟踪、局部轮式时间分配、FRS连续跟踪与遮挡恢复，无独立算法副本。
执行包络通过 `manager.execution_radius`、`manager.execution_offset` 配置，
Slash 保持 `0.30/0.18 m`，Jackal 参数仅在基准 launch 中覆盖。
动态激光适配器属于 DynaBARN 圆柱基准，实机仍使用现有 D435i 深度几何感知链。
本次基准仍暴露停车后被动态障碍接近的失败，不表示主动脱离策略已经完成。

## 失观与重新观测时的包络连续性（2026-11-13）

原实现会在失观超过 0.5 s 后将已学习的加速度半宽强制增至至少 0.2，并清零
加速度中心。位置 FRS 的半宽包含 `0.5 * horizon² * acceleration_half`，
因此从 0.05 跳到 0.2 会令 3 s 预测的单轴完整宽度额外突增 1.35 m。
另外，连续发布的预测快照刷新了状态时间戳，不能据此判断真实观测连续；
重新观测时若沿用旧速度窗口，滤波修正可能被误学为目标加速度。

现在失观只接收当前时刻的预测状态和协方差，保持原意图区间；重新观测时重启
加速度学习窗口，积累真实连续观测后再更新输入集，热身期间保持上次输入集。
不重复传播失观时长，不按固定尺寸截断 KF 协方差，也不因固定 TTL 删除附近威胁。
近场硬安全检查、输入失效停车和自由空间证据退场逻辑均保留。

这修正的是模式切换导致的非连续膨胀，不意味着长期遮挡下的预测应保持固定大小。
短时失观的协方差仍会增长；长期失观由下文的固定未知区域管理，主动脱离策略仍需单独处理。
新增回归覆盖：失观跨阈值、重新观测速度修正、协方差增长及近场目标保留。

初始/重新跟踪还增加了学习可信度门限：连续观测至少 0.6 s，且两个速度分量
的估计标准差均不超过 0.15 m/s，才允许积累加速度学习样本；随后仍需原有
连续速度窗口。该门限只控制“是否把速度变化当作意图样本”，不会缩小输入
协方差、删除目标或放宽近场实体检查。门限是当前工程配置，不是覆盖概率保证。


## 短时失观与固定未知区域生命周期（2026-11-13）

当前规则替代过去“持续 COASTING 直到可见自由空间”的长期传播：

1. 实际观测：`DYNAMIC/TENTATIVE`，正常更新位置、速度和意图。
2. 已确认动态目标短时失观：`COASTING`，最多 `lost_timeout`（默认 1.0 s）内进行 KF 预测。
   待确认残差失观则立即转为 UNKNOWN，固定最后观测的位置与位置协方差，不传播初始化速度先验。
3. 超过窗口：`UNKNOWN=3`，固定在可信窗口终点的位置与位置协方差，停止运动外推。
   物理未知区半径取 `radius + 3*sqrt(max(Pxx,Pyy))`，只计算一次。
4. 重新观测并关联：保留目标 ID，退出 UNKNOWN，重新初始化速度估计和意图学习窗口。
5. 获得有效自由空间证据后移除；超出原有局部管理范围的缓存仅做本地退场，
   不意味着该位置已被证明自由。传感器中断保持不健康并停车，不清空未知区域。

UNKNOWN 不是静止目标确认。其零速度/零速度协方差表示停止使用运动模型，
不能解释为精确测得静止。位置与区域范围仍保留，且不写入永久静态背景。

规划器将 UNKNOWN 作为可撤销局部硬碰撞覆盖层：栅格搜索不穿越、最终轨迹再次
检查；控制器同时检查固定区域，只有它远离候选路径时才继续运行。该机制独立于
运动 FRS 软风险权重，不能通过将空间风险权重设为零来绕过未知区域。
日志 `tracks/boxes` 仅包含运动 FRS；固定区域单独记录在 `unknown_regions`，
并提供 `moving_track_count`、`unknown_region_count`，避免继续画出“消失目标的运动 FRS”。
深度节点的 MarkerArray 中 UNKNOWN 使用 `unknown_regions` 命名空间。

自由证据：深度检查覆盖区域投影包围框内的全部有效像素；激光检查与未知圆盘
相交的全部有效射线，并要求量程足以覆盖后边界。遮挡、零值、NaN、视野不完整
或量程不足都不清除；激光无返回只在传感器有效量程内作为自由证据。

深度节点与动态激光基准均通过 `lost_timeout` 节点参数配置可信时长。
动态基准 launch 还支持 `lost_timeout:=1.0`。接口增加 UNKNOWN 常量，部署前需
重建 `scan_planner_msgs` 和依赖它的规划/感知包，再重启相应节点。
本次运行结果与旧正式9轮分开保存于 `dynamic_dynabarn/results/unknown_lifecycle/`。


## 室内失观可信度分层修复（2026-11-16）

复现发现，单帧墙角残差携带 0.4 m/s 的初始化速度标准差，经过 1 s 的
失观预测后，可能形成约 2 m 的机器人中心禁入区并堵塞路线。该速度先验
是滤波初始化条件，不是观测到的运动证据。

`Tracker.step` 保留待确认目标预测前的位置后验；关联成功仍使用正常 KF
预测/更新以估计速度，关联失败则恢复该观测支撑并立即进入固定 UNKNOWN。
已确认动态目标保持原来的短时预测与窗口终点冻结。两者均保留位置误差、
实际实体半径和自由空间证据清除要求，不通过缩小机器人包络或清空遮挡目标解锁。

修复前实验在 `output/paper_navigation_20261113`，诊断与重测记录在
`output/indoor_repair_20261116`；探查运行与冻结版本后的正式对照分开保存。

静止目标的速度一旦离开 `motion_off` 静止带，即退回 TENTATIVE 并遮罩其深度，
不再等到 DYNAMIC 多帧确认后才停止写入静态地图。

高度安全模式在仅 UNKNOWN 造成停车时，可进入 `OBSERVING_UNKNOWN`：
保持平移零速，按最近未知区方向原地转向；输入必须新鲜，且转向的完整
刹停机身扫掠必须通过静态几何检查。存在已观测运动目标、近场保护触发、
数据过期或转向碰撞时不执行。未知区仍由实际观测证据解除，轨迹进度保持冻结。
等待统计应包含该状态，避免将状态改名误当作等待改善。

第一阶段探查发现单项修复仍不足，相关3次已完成记录全部保留；
增加静止退出和安全重观测后，冻结验证使用独立目录
`output/indoor_repair_20261116_v2`，不混入第一阶段统计。


## 与 BARN MPPI 对齐的室内仿真速度档（2026-11-16）

此前室内工具固定线速度0.3m/s、角速度0.5rad/s；BARN MPPI配置则为
0.5m/s、1rad/s。现增加显式运动限制参数，并提供统一仿真入口：

```bash
bash navigation/scan_planner/tools/run_indoor_benchmark_speed.sh frs_soft 0.22 output/indoor_benchmark_speed/soft_022
```

该入口统一规划速度/加速度、控制巡航限速/加速度和动态制动预测：
线速度0.5m/s、角速度1rad/s、加速和制动0.5m/s²。高度、障碍运动、
安全包络与风险权重不变。制动能力仅为运动学仿真的配置，部署时必须
按真实平台能力设置`dynamic_braking_deceleration`，不能直接假定相同。

底层验证工具仍保留原来的默认速度档；新的`--robot-speed`、
`--robot-yaw-rate`、`--robot-acceleration`、`--robot-braking`参数及
实际跟踪加速度均写入summary.json，避免只改执行限速而遗漏规划限速。
同速度上限并不意味着不同地图、机器人几何和交互任务的绝对通行时间可直接比较。

该速度入口还开启 `height_spatial_tracking`：带高度变化的样条同样采用
XY几何进度追踪，并从对应样条相位提取高度参考，避免低横梁轨迹退回
纯时间跟踪。保留测得高度的碰撞检查、提前降身、恢复高度、刹停扫掠检查
及FRS时序决策。该功能在控制器默认关闭，入口显式开启，旧运行默认不变。


## 室内成对条件验证工具（2026-11-16）

验证器新增`dual_crossing`场景：转弯处两个目标在不同横断面相向横穿，
目标采用有界加减速，路径互不碰撞且高目标不会穿越低横梁。正常机身
顶部0.6102m高于本场景0.60m横梁，低姿态通行具有实际几何必要性。

`--minimum-leg-height .13`将轮足搜索与轨迹复检的最低基座高度统一为
0.1622m，并将搜索高度格固定于该下界。否则以起终点中点为格原点的
5cm高度采样可能漏掉可行低高度层。默认不启用，原先运行默认不变。
对应回归通过真实GridMap和AStar构造窄高度通道验证。

`--search-budget .1`限制单次搜索，避免超过控制器0.8s数据时效门限；
失败搜索不会发布未验证轨迹。`--goal-height-tolerance .008`要求终点
停车且姿态恢复后才计完成；`--record-period .05`保留20Hz常规记录及
全部状态/停车阈值变化事件。碰撞及最小净距仍在每个仿真积分步评价。
新增点云分区快照带原始曝光时间戳，用于展示真实在线感知输出。

本次试验设计、源码快照和结果位于`output/navigation_study_20261116`。
预建点云底图与局部占据更新不等同于验证视觉SLAM；此工具仍为理想
位姿与深度射线输入的运动学闭环。

成对方法对照时，验证器可用`--search-heuristic 1.8`显式统一加权A*
启发式。原来的自动模式会按是否复用静态走廊选择1.0或1.8，不宜在
严格计算预算下用作空间风险开关的唯一差别。默认自动模式不变。
正式室内研究采用共同0.1s搜索预算、1.8启发式、0.1622m高度格原点，
设计与记录见`output/navigation_study_20261116_v2`；早期协议检查数据
保留在`output/navigation_study_20261116`，不混入正式统计。

### 室内全流程修复与复测（2026-11-16，v3）

`tools/run_indoor_full_speed.sh full 0.22 <新输出目录>` 运行单次完整方法；
`python3 tools/run_indoor_comparison.py --output-root <新输出目录>` 按固定顺序运行
四方法×三档障碍速度×两次重复，共24轮。需先source ROS Jazzy及`install/ws_nav_frs/setup.bash`。
场景为dual_crossing，横梁底高0.60m、最低腿高0.13m、线速度0.5m/s；到达还要求通过横梁并恢复高度。
新脚本保留失败，拒绝覆盖已有研究目录，逐轮检查源码哈希；trace以gzip无损保存。

实现修复：先生成带静态/未知区硬检查的几何种子，剩余搜索时间用于FRS空间代价优化，
优化超时保留种子；平滑高度轨迹未通过碰撞检查时，使用经过逐段及整曲线验证的保角点回退，
必要时仅延长时间以满足速度/加速度界，不降低碰撞阈值；局部段结束不再直接清除尚未到达的全局目标。
静态底图保护区采用局部多支持面匹配，避免把横梁/墙角误分为新目标，同时保持前景法向距离判据。

失观目标保留原冻结后验用于关联。对有接地观测支持的目标，将位置不确定性离散为固定中心单元；
仅当该单元按物体支持半径扩展后的整个接触平面区域有有效自由深度证据时清除它。
剩余单元以固定ID、零速度和包含物体半径/单元半对角线的包络发布为UNKNOWN，绝不生成运动FRS。
此证据使用现有局部地面高度及连通接地假设；悬空目标、无效深度、遮挡和视野外部分保留原保守管理。
超过256个单元时退回整体未知区，保证计算有界。旧v2的正式记录和全部预试验独立保留。

### 可选主动避险恢复模块（实验功能，默认关闭）

`plan_manage/active_escape.hpp` 为独立、无 ROS 依赖的差速候选评估模块。
`closed_loop_controller` 的 `active_escape_enabled` 控制启用，默认 false；
`active_escape_horizon` 默认 2.5 s，`active_escape_max_speed` 默认 0.5 m/s。
启用需要原有动态跟踪、空间跟踪及新鲜的地图/里程计输入；输入过期或不健康继续停车。

模块以“当前位置在未来 FRS 中存在冲突”为触发依据，在进入保护停车前预留转向时间。
候选为前进加转向（0、左右约37/66/90度的目标航向及两个速度），不输出横向平移，
也不启用无法确认后方安全的倒车。滚动预测从实测速度开始，包含反应时间和制动尾段。
静态占据、固定 UNKNOWN 全程硬检查；运动目标保留近场 CV/误差包络硬检查；
远期 FRS 进入软代价，同时排除远期 CV 中心与当前位置误差包络的名义实体拦截。
未来速度不确定性不会被再次全部硬化为禁行区。没有可行候选时，继续原有保护停车，
不会删除 UNKNOWN、缩小实体或关闭近场保护以换取通过。

执行中保持航向偏好，按控制周期连续增加命令，避免里程计频率较低时转向增长过慢。
避险期间暂停原参考推进并请求重规划；风险消除且正常跟踪可行后，通过保持时间退出。
`ESCAPING_DYNAMIC` 表示正在执行；决策日志的 `escape_enabled/active/triggered/feasible/reason`、
`escape_candidates/parked_clearance/chosen_clearance/path` 用于单独评价。
日志 `feasible/scale` 仍表示原路径的时序决策，不能以其单独判断避险候选。
`chosen_path` 在避险期间改为所选避险预测路径，实际发布命令仍有最终静态制动检查。

DynaBARN 调用示例（须重新 build；使用新分组防止覆盖）：
```
DYNA_GROUP=escape_on_new DYNA_ACTIVE_ESCAPE=true navigation/benchmarks/dynamic_dynabarn/run.sh frs_soft 0 0
DYNA_GROUP=escape_off_new DYNA_ACTIVE_ESCAPE=false navigation/benchmarks/dynamic_dynabarn/run.sh frs_soft 0 0
```
评估应使用同一二进制、相同场景/初相位/传感器，仅切换模块开关。
这是受限候选的局部恢复，不保证任何动态环境均可脱离；遮挡、估计误差、超出预测的
加速度及平台跟踪误差仍可能导致失败。高度通行继续由原高度安全层约束，未把
Jackal 平面避险试验等同于 Slash 全高度/实机验证。

避险对照沿用原始轮速/IMU定位，不更换理想位姿。评价同时区分原碰撞路段的局部交互
及完整导航到达。为了与Nav2动作返回的处理一致，本文控制器报告终点到达、规划器
进入WAIT_TARGET且估计位置距原目标小于0.1m，但真值尚未进入0.3m门限时，评价器
记`estimated_goal_only`并结束；这是精度失败，不是成功。真值与估计误差单独报告，
不把定位偏差当作避险无效，也不把局部无碰撞通过当作整场导航成功。旧版延长停车
后发生碰撞的开发记录保留，不覆盖或反改。

### 局部规划层的动态路径与速度联合选择（默认关闭）

`manager.dynamic_local_planning` 在 `planWheelTrajectory` 内启用
`dynamic_local_path.hpp`。对同一静态几何种子生成相对横向偏移
0、±0.25、±0.5、±0.9 m 的平滑候选，偏移在不超过3 m的行进距离内重新接回
原参考；比较0.5、0.375、0.25 m/s速度上限及等待。实际速度上限按manager.max_vel
同比例缩放。候选均通过同一空间跟踪器模型传播，评分包含时序FRS风险、速度损失、
相对全局引导的偏离及转向，保留静态/UNKNOWN硬检查与近场动态硬检查。远期FRS
仍是软代价，另排除CV名义实体拦截。该离散候选方法不等同于完整连续最优控制器。

选择发生在最终样条发布之前，控制器不再通过旧ActiveEscape旁路接管。
规划器发布`planning/local_speed_limit=[traj_id, stamp, cap]`；控制器设置
`planned_local_speed_required=true`，只采用匹配当前轨迹编号的有效速度上限。
缓存相邻编号避免新轨迹/速度消息不同步引起停车；缺失、无效或零速度计划保持停车。
等待时仍按重规划周期重新评估，不能因轨迹时钟冻结永久阻塞。曲线经静态认证后
才发布，候选最终样条与用于评估的曲线相同。非平面高度轨迹保留原高度参考及
既有FRS/高度安全层，本功能未宣称完成高度变化下的全状态联合优化验证。

两个开关必须成对配置；DynaBARN启动入口已完成配对：
```
DYNA_GROUP=local_path_on_new DYNA_ACTIVE_ESCAPE=false DYNA_LOCAL_PLANNER=true navigation/benchmarks/dynamic_dynabarn/run.sh frs_soft 0 0
DYNA_GROUP=local_path_off_new DYNA_ACTIVE_ESCAPE=false DYNA_LOCAL_PLANNER=false navigation/benchmarks/dynamic_dynabarn/run.sh frs_soft 0 0
```
禁止同时启用旧控制器ActiveEscape与新局部选择。对照双方共用0.2 m动态裕度、
原传感器与轮速/IMU定位；没有真值输入规划。`planning/dynamic_local_decision`记录
选中偏移、速度、候选数、风险及选择耗时，评价器保存为local_plans.json，实际命令
保存commands.csv。默认关闭，须根据固定场景对照的完整到达及碰撞结果判断效果。

### 局部规划层联合速度与转向滚动选择（2026-11-17，实验开关）

`manager.joint_rollout_planning` 配对控制器 `joint_rollout_required`，默认均为false。
DynaBARN入口使用 `DYNA_JOINT_ROLLOUT=true` 自动配对；不可叠加旧ActiveEscape或
旧DynamicLocalPath候选。模块位于规划层，控制器只执行经时间戳及轨迹ID匹配的前缀，
不再用旧路径的速度缩放消去选中的转向。该入口目前验证平面差速任务；启用高度安全
控制时拒绝执行活动序列，非平面高度参考显式委托原高度规划与跟踪。

`joint_rollout.hpp`从实测速度、角速度开始，以0.05s积分、0.4s控制段、3.2s预测窗口
联合搜索4档非负速度及7档角速度；考虑线加减速和角加速度限制。有限宽度搜索保留
不同转向状态、恒定控制种子及上次序列的安全延续，避免只按短期推进剪掉提前转弯。
每次只执行前缀，并约0.4s重新规划；无导航目标不自行移动。该方法是有限候选滚动
选择，不是连续非线性MPC，也不具备全局可行性或递归安全保证。

保持远期FRS为软风险；静态地图、固定UNKNOWN、近场CV误差包络为硬检查，远期另
排除名义实体拦截。保留静态/UNKNOWN制动尾段与执行时最新观测的0.6s前缀检查。
缺失、过期、不可行序列及异常感知停车；不扩大指令有效期来掩盖计算超时。
静态查询使用每轮独立的保守空闲证书缓存：外包圆覆盖两圆柱、整个位置格及体素误差，
证书失败则回到原精确包络查询，UNKNOWN仍逐步独立检查。

`planning/joint_rollout`为可靠、瞬态本地Float64MultiArray：
`[version=1, traj_id, input_stamp, active, feasible, segment_dt, count, v0,w0,...]`。
控制器缓存相邻ID以处理消息先后差异，只接受当前轨迹的有效序列。无障碍及非平面
参考通过显式inactive消息委托既有跟踪器；过期inactive也不能解锁运动。
`planning/dynamic_local_decision`保存输入时间、活动标志、可行性、停车威胁、候选数、
控制序列、预测路径及耗时。`dynamic_navigation/decision`保存执行前缀检查与实际命令。

示例（先重新build；新分组避免覆盖）：
```
DYNA_GROUP=joint_on_new DYNA_ACTIVE_ESCAPE=false DYNA_LOCAL_PLANNER=false DYNA_JOINT_ROLLOUT=true navigation/benchmarks/dynamic_dynabarn/run.sh frs_soft 0 0
DYNA_GROUP=joint_off_new DYNA_ACTIVE_ESCAPE=false DYNA_LOCAL_PLANNER=false DYNA_JOINT_ROLLOUT=false navigation/benchmarks/dynamic_dynabarn/run.sh frs_soft 0 0
```
本轮开发与正式对照分别保存在`output/joint_rollout_20261117/development`和`formal_v2`，
原有实验与源码备份保留，不用新预试验覆盖旧失败，不自动改写论文或推送Git。

联合模式下，状态机的前视静态/未知区检查改为实际选中序列的预测路径；当前机身
碰撞检查保留，避免未执行的引导样条触发无关急停。目标代价还包含连续净距软代价，
用于提前远离名义硬边界，未改变硬半径或将FRS整体改成禁行区。前缀、轨迹消息及
新鲜度仍独立检查；不可行时没有强行穿越硬约束的恢复旁路。

联合控制到达分支发布终点执行进度，允许新鲜里程计下的终点捕获完成，避免旧序列过期掩盖完成状态。旧formal批次因该衔接缺陷单独保留，最终对照在formal_v2。
