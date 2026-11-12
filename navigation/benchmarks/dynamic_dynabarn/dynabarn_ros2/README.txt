DynaBARN ROS 2 Jazzy / Gazebo Harmonic 包

本包将官方world_2的8个运动圆柱移植为gz::sim系统插件，保留关键帧、周期、
0.5 m半径与1 m高度。原轨迹通过只读ELF常量提取，不加载执行旧Classic插件。
其他地图需提供原world和插件另行转换，本包未包含完整60场景。

本包由 robot_core/navigation/benchmarks/build.sh 构建，运行和对比请使用
上一级目录run.sh、run_suite.sh，详见上级README.txt。
算法只链接到robot_core/navigation/scan_planner，避免私有算法副本。
传感器输出与目标跟踪来自真实模拟激光，真值只供独立评价器。
暂停和时间复位服从模拟时钟，原周期末尾回跳保持原样。

主要入口：
launch/comparison.launch.py：MPPI / frs_soft / frs_hard 公共场景与配置。
scripts/laser_perception.py：背景差分、观测圆拟合、KF及TF帧缓存。
scripts/compare_evaluate.py：固定起步、真实接触、路径和跟踪离线评价。
src/motion.cpp：Harmonic原生运动与真值发布系统。
test/：轨迹提取、几何背景分离、延迟TF回归。

迁移依据：
https://gazebosim.org/api/sim/8/migrationplugins.html
https://gazebosim.org/api/sim/8/createsystemplugins.html
原始DynaBARN数据和轨迹版权归原作者；生成world旁JSON保留原文件哈希和来源。
此处结果不是旧物理引擎/旧排行榜的逐时刻复现。
