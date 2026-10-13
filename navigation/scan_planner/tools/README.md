# Keypoint recorder (ROS 2)

Build and source the workspace, then run:

```bash
ros2 run scan_planner keypoint_recorder.py --odom /LIO/odom_vehicle --output keypoints.yaml
```

The generated file is a ROS 2 parameter file containing the flat
`fsm.waypoints: [x0, y0, z0, ...]` array. Pass it to `run.launch.py` with
`keypoints_file:=/absolute/path/to/keypoints.yaml` when `navi_mode:=2`.

# 轮足变腿高导航仿真实验

本实验验证 **主动降低腿高穿过低横梁，车尾离开后恢复高度**，以及绕开落地障碍物。
它不验证抬轮跨台阶、跳跃或接触动力学。当前 `slash` 模拟器是运动学模型；
不能据此声称真实机器人已具备跨越地面障碍的能力。

在 `robot_core` 目录执行（已在 ROS 2 Jazzy / Ubuntu 24.04 验证）：

```bash
source /opt/ros/jazzy/setup.bash
ROS_DISTRO=jazzy bash build.sh ws_nav
source install/ws_nav/setup.bash
ros2 launch scan_planner height_experiment.launch.py
```

启动后自动等待里程计、雷达和地图就绪，再发送 `(0,-2) → (0,2)` 的目标。
横梁底面默认高 `0.62 m`，开口宽 `2 m`、梁厚 `0.6 m`。
仿真通过局部雷达点云建图，规划器不读取验收脚本的场景真值。
演示默认打开 RViz，可用 `rviz:=false` 无界面运行。
用 `beam_clearance:=0.48` 查看不可通过时的停车；
用 `leg_height_rate:=0.02` 查看慢速执行器下等待降身的行为。

```bash
# 六个端到端场景：正常、低梁、不可通行、绕桌、慢执行器、梁下雷达中断
/usr/bin/python3 navigation/scan_planner/tools/validate_height_navigation.py \
  --output outputs/height_experiment --repeat 1

# 单个场景；独立 ROS_DOMAIN_ID，结束后清理本次启动的进程
/usr/bin/python3 navigation/scan_planner/tools/validate_height_navigation.py \
  --case low --output outputs/height_single --domain 150

# 实测轨迹图与回放 GIF，需要系统 numpy、matplotlib、Pillow
/usr/bin/python3 navigation/scan_planner/tools/plot_height_experiment.py \
  outputs/height_experiment/run_1

colcon --log-base log/ws_nav test --base-paths navigation/scan_planner \
  --build-base build/ws_nav --install-base install/ws_nav --packages-select scan_planner
colcon test-result --test-result-base build/ws_nav/scan_planner --verbose
```

自动验收输出 CSV（实际位姿、腿高、指令、状态）、规划轨迹、节点日志和 JSON 结论。
任一断言失败时脚本返回非零。验收按双圆柱车身（半径 `0.25 m`，前后偏移 `0.18 m`）
独立检查场景几何，并对相邻里程计采样间的位置、转向和高度插值检查，
防止仅凭“到达目标”把穿墙误判为成功。通行要求净空至少 `3 cm`；
低梁必须实际降身、穿梁并恢复；不可通行必须报告 `NO_PATH` 且原地停车；
传感器故障在梁下暂停本次启动的雷达进程，检查停车且不升高，随后恢复进程并清理。

仅 `slash` 仿真默认启用新增轮式栅格轨迹和闭环高度保护。
规划使用包含转弯余量的包络；平滑后再次检查实际包络。
控制器使用实测高度等待降身完成，限制升降速率，检查前方扫掠体，
在雷达、地图或里程计超时后停车并保持高度。
`/height_control/state` 输出 `LOWERING`、`TRACKING`、`BLOCKED`、`WAIT_MAP`、
`NO_PATH`、`GOAL_REACHED` 等状态。腿长范围 `0.14–0.36 m`，
基座高度为腿长加 `0.0322 m`，车身顶部高于基座 `0.328 m`。
腿部可视化跟随实际基座高度，不另行假设执行器已经完成指令。

实验默认关闭动态障碍物。当前验收范围是平地、静态场景和上述几种异常工况，
尚未验证动态避障、轮地接触、摩擦、力矩、倾覆或实机稳定性。
已有实机控制分支保留；雷达表面采样、地图遮挡保持和 A* 修复属于通用代码改动，
用于实机前仍需针对实际传感器和机体尺寸回归验证。

CPU 雷达的 `surface_render: true` 模式按角度栅格保留最近的真实表面采样，
避免原来的边缘平面外推生成横梁下方的虚假点。当前配置量程 `6 m`、角分辨率
`0.4°`、垂直视场 `90°`；这是理想化采样，并未复现 Mid-360 扫描时序或噪声。
该模式不使用原有的扫描花纹和点插值选项，深度相机分支仍走原来的渲染流程。

## 首版验证记录（修复原地转向前，2026-10-13）

ROS 2 Jazzy，Release 构建：14 个包完成；colcon 报告 14 tests、0 failures。
六个端到端场景各运行一次，均通过。普通及低梁通行目标为 `(0,2)`；绕桌为 `(5.6,-4.5)`。

| 场景 | 梁底高度 | 结果 | 到达误差 | 最小梁下净空 |
| --- | --- | --- | --- | --- |
| 正常通行 | 0.80 m | 到达 | 0.38 cm | 19.20 cm |
| 降身穿梁 | 0.62 m | 到达并恢复腿高 | 0.18 cm | 7.27 cm |
| 不可通过 | 0.48 m | NO_PATH，原地停车 | — | 未进入 |
| 绕桌 | 0.80 m | 绕开落地障碍后到达 | 1.72 cm | 未经过梁 |
| 慢执行器 0.02 m/s | 0.62 m | 到达并恢复腿高 | 0.13 cm | 7.03 cm |
| 梁下雷达中断 | 0.62 m | WAIT_MAP，停车且不升高 | — | 7.26 cm |

独立扫掠几何检查的实体碰撞数均为零。标准低梁场景实测最低腿高 `0.1678 m`，
到达后恢复到 `0.2486 m`，用时约 `63.7 s`。耗时会随规划重启、调度及建图变化，
本记录不是实时性或统计成功率保证。

[验收 JSON](../../../outputs/height_experiment_final/summary.json)、
[轨迹和高度曲线](../../../outputs/height_experiment_final/run_1/overview.png)、
[实测回放 GIF（约 2 倍速）](../../../outputs/height_experiment_final/run_1/low_replay.gif)。
同目录保留 CSV、规划轨迹和节点日志，上级目录保存构建日志、测试日志及源码 SHA-256 清单。

## 论文用执行接口对照实验

后续修正了低水平速度时由位置超前误差引起的反向航向反馈，并增加降身滞回和终点位置捕获。
`stable_heading:=false` 可复现修正前的执行接口，仅供对照；默认启用修正。
演示初始航向现在沿通道方向，必要的起始对准仍可通过 `run.launch.py init_yaw:=0.0` 验证。

论文工程中的 `code/ch5_height_navigation/run_experiments.py` 运行三组交替顺序的新旧对照，
随后运行六工况回归及 0.60 m 横梁的补充试验。`plot_results.py` 读取全部原始结果，生成矢量 PDF、可编辑文本 SVG、
300 dpi PNG、三线表和实测回放。指标新增实际运动的原地转向时长、降身累计转角、路径长度等。

```bash
source /opt/ros/jazzy/setup.bash
source install/ws_nav/setup.bash
/usr/bin/python3 ../mythesis/code/ch5_height_navigation/run_experiments.py
/usr/bin/python3 ../mythesis/code/ch5_height_navigation/plot_results.py
```

图形风格参考 [IEEE 图表规范](https://proceedingsoftheieee.ieee.org/resources/guidelines-for-figures-and-tables/)
和 [Nature 图形指南](https://research-figure-guide.nature.com/figures/preparing-figures-our-specifications/)，
采用统一字号、单位、可区分线型与逐次数据点；原始曲线不作平滑。
论文实验仅评价静态场景的高度导航执行层，并不运行完整的动态风险联合梯度优化器。
