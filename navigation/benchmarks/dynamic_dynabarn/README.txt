动态 DynaBARN：ROS2统一条件适配对照

从robot_core根目录先运行 navigation/benchmarks/build.sh。
默认使用challenge协议，按BARN Challenge固定run.py的任务/终止规则评分：
(11,0,3.14)->(-9,0)，真值距终点<=1m，移动0.1m后开始100s计时。
控制器报告完成不提前结束；估计到达与定位偏差另外记录。
定位仍为原生轮速/IMU EKF；未把真值送入导航、感知或目标跟踪。
共同速度0.5m/s、1rad/s，有效感知5m，Jazzy/Harmonic现代Jackal；
这是适配对照，不等同于官方完整样本/物理条件或排行榜成绩。详见PROTOCOL.txt。

单轮（新分组，禁止覆盖）：
DYNA_GROUP=challenge_new navigation/benchmarks/dynamic_dynabarn/run.sh mppi 0 0
DYNA_GROUP=challenge_new navigation/benchmarks/dynamic_dynabarn/run.sh frs_soft 0 0
DYNA_GROUP=challenge_new navigation/benchmarks/dynamic_dynabarn/run.sh frs_joint 0 0
frs_joint自动配对规划器与控制器开关；不要叠加旧ActiveEscape或旧局部曲线候选。
其他方法frs_hard、mpc_d_cbf仍支持；后者需设置BENCH_IMAGE=barn-jazzy-mpc-d-cbf:local。
DYNA_GROUP=challenge_suite navigation/benchmarks/dynamic_dynabarn/run_suite.sh 0
便利套件为初相位0/7/14×MPPI/FRS-Soft/frs_joint共9轮，不能视为3张独立地图。

复现此前内部条件必须显式设置DYNA_PROTOCOL=legacy：起点12m、真值0.3m、
发目标计时120s及estimated_goal_only提前终止。历史数据保持原结果，不与challenge混算。

汇总（所有重复与启动异常都保留，不按方法/初相位静默去重）：
python3 navigation/benchmarks/dynamic_dynabarn/report.py --group challenge_new --output output/challenge_review
python3 navigation/benchmarks/dynamic_dynabarn/plot.py --report output/challenge_review/report.json
报告拒绝混合challenge和legacy结果，保留成功/碰撞/超时、路径、等待、净距和定位误差。
时间平均仅用成功样本；MPPI没有同名FRS状态，不虚写求解器耗时。

场景world_2保留原8圆柱轨迹及周期回跳，尚未包含完整60场景。
原生模拟激光->静态墙背景差分->聚类->可靠圆拟合->KF->FRS；
该圆柱适配不等同于任意物体形状恢复，也不等同于Slash实机深度相机输入。
长期失观目标超过可信时长转固定UNKNOWN，取得自由空间证据后清除。

测试：dynabarn_ros2/scripts/test_benchmark_protocol.py、test_evaluation_ros.py，
以及dynabarn_ros2/test/中的轨迹转换、几何与延迟TF测试。
新协议启动规则与未启动看门狗、计时采样以及原仓库版本等全部记录在PROTOCOL.txt。

MPPI控制器显式使用/platform/odom/filtered速度反馈；challenge评测器在发目标前
检查controller_server实际Odometry订阅，避免默认/odom重映射到原始轮速。
legacy只恢复评测条件；精确重现旧算法/接线还需使用对应实验的源码与镜像快照。

2026-11-17 FRS动态导航修复：
run.sh无参数默认frs_joint，执行联合线速度/转向滚动选择，FRS为软风险、近场CV/实体包络为硬检查。
DYNA_LOCALIZATION=velocity_imu为新默认：编码器只融合机体系vx/vy，原生IMU融合yaw rate，
不将同源轮速积分的XY/yaw再次作为绝对观测融合。不是全局定位，也没有使用机器人真值。
新配置使用隔离的临时setup目录，原robot.yaml与旧实验快照保留；require_imu检查实时输入。
DYNA_LOCALIZATION=legacy可以复现原融合配置；DYNA_PROTOCOL=legacy只选择旧评分规则，两者独立。
MPPI平滑器正负角速度均限制在±1rad/s，线速度[-0.35,0.5]m/s。
结果保存localization_profile、runtime_localization.yaml和传感器新鲜度；不同定位配置不得混合统计。
建议：BENCH_IMAGE=barn-jazzy-mpc-d-cbf:local DYNA_GROUP=frs_new navigation/benchmarks/dynamic_dynabarn/run.sh frs_joint 14 0
