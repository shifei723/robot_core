动态 DynaBARN 基准

从 robot_core 根目录先执行 navigation/benchmarks/build.sh。
单轮：navigation/benchmarks/dynamic_dynabarn/run.sh 方法 初相位 试验编号
方法为 mppi / frs_soft / frs_hard。原始场景world_2有8个运动圆柱，
保留原关键帧、周期和周期末尾回跳，不依赖ROS1或Gazebo Classic。
全套：navigation/benchmarks/dynamic_dynabarn/run_suite.sh 试验编号
固定初相位为0、7、14 s，共9轮；不是3个独立地图或同条件统计重复。
使用 DYNA_GROUP=新分组 可额外验证，默认final，禁止覆盖已有结果。

传感器链：5 m真实模拟激光、原生里程计、同一静态墙背景 -> 背景差分 ->
残差聚类 -> 可靠圆弧拟合 -> KF -> FRS。拟合不可靠的点纳入近场占据；
动态真值只进入评价器，不输入感知。延迟TF采用最新扫描缓存，真实超时仍停车。
圆拟合是基准圆柱的形状适配，不等价于任意形状恢复，也不是Slash实机深度输入。

共同sim=8 s启动，起点(12,0,pi)，目标(-9,0)，容差0.3 m、超时120模拟秒。
双方线速度上限0.5 m/s、角速度上限1 rad/s，MPPI保留倒车，本文控制器前向。
软/硬FRS仅风险处理开关不同。详细协议见PROTOCOL.txt（其中旧目录名为历史记录）。

新结果汇总：python3 navigation/benchmarks/dynamic_dynabarn/report.py
           python3 navigation/benchmarks/dynamic_dynabarn/plot.py
report默认读取results/final和logs/final；图像使用Tinos数字/英文、Noto宋体中文，
需要numpy/scipy/matplotlib/fonttools以及fonts-croscore、fonts-noto-cjk。
reference_results/保留此前9轮汇总及中文图，原始完整记录位置见上级source_manifest.json。
既往软风险到达2/3，MPPI三轮碰撞，硬约束两轮碰撞一轮超时，失败均被保留。
目前仍有保护停车后被运动目标接近的失败，代码整理不表示该策略缺口已经解决。

包内测试：
  python3 dynabarn_ros2/test/test_laser_geometry.py
  python3 dynabarn_ros2/test/test_conversion.py
延迟TF真实ROS2回归在安装并source工作空间后执行：
  python3 dynabarn_ros2/test/test_delayed_laser_tf.py
原始官方插件往返转换测试需要可选原数据，其余测试使用包内独立夹具。
