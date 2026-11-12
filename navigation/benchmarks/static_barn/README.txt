静态 BARN 基准

从 robot_core 根目录先执行 navigation/benchmarks/build.sh。
单轮：navigation/benchmarks/static_barn/run.sh 方法 地图编号 试验编号
方法为 slash 或 mppi；随代码提供地图0、6、12。
全套：navigation/benchmarks/static_barn/run_suite.sh 起始试验编号
默认结果组为 spatial_release（slash）及 spatial_mppi（mppi）。
使用 BARN_RUN_SET=新分组 可保留额外验证，已有结果拒绝覆盖。

使用同一Jackal原生物理、前向激光和轮速/IMU融合里程计，动态预测关闭，
不向规划器提供障碍真值地图。0.5 m/s线速度、1 rad/s角速度上限。
起点(-2.25,3,pi/2)，目标(-2.25,13)，目标容差1 m、超时120模拟秒。
碰撞由Gazebo接触传感器判断，记录路径和控制时序。
该协议不同于旧BARN排行榜，不能将其分数直接当作官方排名。

重新汇总新试验：
  python3 navigation/benchmarks/static_barn/summarize_spatial.py
  python3 navigation/benchmarks/static_barn/plot_spatial_report.py
需宿主机numpy/scipy/matplotlib，或在相同Docker环境内执行。
分析器读取默认结果组；早期开发组若不存在会自动略过。未跑试验时请查看
reference_results/历史汇总，不要将它与新 results/ 混合。

upstream/ 保留所用Jackal辅助包的源代码、传感器/导航参数和三张静态地图，
来源及版本见上级source_manifest.json；原作者仓库README保留供追溯。
算法来自robot_core，Jackal差异全部由benchmark.launch.py覆盖。
