导航对比试验（ROS 2 Jazzy / Gazebo Harmonic）

两个独立试验目录：
  static_barn/       静态 BARN，MPPI 与最新 SCAN/HAVEN 导航对比。
  dynamic_dynabarn/  动态 DynaBARN，MPPI、FRS软风险、全时域FRS硬约束对比。

算法唯一来源是 robot_core/navigation/scan_planner/src。
prepare_workspace.py 在被忽略的 .workspaces/jazzy 中创建相对软链接，构建时直接
使用机器人工程的规划器、控制器和几何跟踪源码；不维护另一份试验算法副本。
静态和动态的入口、评价器、场景与参数各自放在对应目录。
本目录带 COLCON_IGNORE，不参与 robot_core 全仓库默认扫描；下面的专用构建入口
显式发现试验工作空间，不影响现有 install/ws_nav_frs 的实机导航安装。

从 robot_core 根目录执行：
  ./navigation/benchmarks/build.sh
  ./navigation/benchmarks/static_barn/run.sh slash 0 0
  ./navigation/benchmarks/static_barn/run.sh mppi 0 0
  ./navigation/benchmarks/dynamic_dynabarn/run.sh frs_soft 0 0
  ./navigation/benchmarks/dynamic_dynabarn/run.sh frs_hard 0 0
  ./navigation/benchmarks/dynamic_dynabarn/run.sh mppi 0 0

默认使用已存在的 barn-jazzy:local Docker 镜像。新机器可先构建：
  docker build -t barn-jazzy:local navigation/benchmarks
可通过 BENCH_IMAGE 指定等价镜像。Dockerfile 为新环境依赖入口，本机验证使用已有镜像。
主机仅需 Docker、Python3；每次运行启动独立容器，默认无头，不要求安装ROS1或Classic。
实验会使用CPU软件渲染，三种方法请串行执行，避免仿真资源争用。

全套试验：
  ./navigation/benchmarks/static_barn/run_suite.sh 0
  ./navigation/benchmarks/dynamic_dynabarn/run_suite.sh 0
静态为3地图×3次×2方法；动态为1地图×3初相位×3方法。
数字为起始试验编号；运行脚本拒绝覆盖已有结果。
静态 BARN_RUN_SET、动态 DYNA_GROUP 可指定新的结果分组。
结果和日志保存在各自目录的 results/、logs/，默认不进入Git。
reference_results/ 是整理前已经完成试验的只读历史汇总，不冒充本次重构后的新试验。
原始完整数据保留在 source_manifest.json 指明的论文tmp目录中，没有删除或移动。

最新算法与同步范围：
- 空间弧长进度、换轨迹速度连续性、横向反馈和曲率前馈。
- 局部轮式时间分配、碰撞检查后的路径简化、静态走廊复用。
- FRS空间跟踪衔接、遮挡目标的预测状态保留与近场检查。
- 执行包络半径/偏置已参数化。Slash 默认仍为0.30/0.18 m；
  Jackal基准显式覆盖为0.334/0 m，不将基准车尺寸写成实机默认值。
- 动态基准包含激光曝光时刻TF等待缓存、背景差分、圆弧中心补偿与KF跟踪。
  其中激光圆拟合是DynaBARN圆柱场景适配，不替代Slash的D435i深度几何感知链。

历史正式结果：静态双方均9/9到达，平均26.57 s（本文）、26.74 s（MPPI）；
动态软风险2/3到达，成功均时45.45 s，MPPI三轮碰撞，硬约束两轮碰撞一轮超时。
软风险仍有停车后被运动障碍接近导致的碰撞，尚未修复主动脱离/后退策略。
这是Jackal固定高度导航对比，不是Slash接触动力学或主动变高度验证；
不同试验的目标容差和场景不同，不直接混合统计。

同步前的导航源码保存在 robot_core/output/navigation_sync_backup_20261112/。
版本记录以 git log 为准。验证记录见 validation.txt。
