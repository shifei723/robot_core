MPC-D-CBF ROS2共同输入适配（实验基线）

来源：https://github.com/jianzhuozhuTHU/MPC-D-CBF
论文：https://arxiv.org/abs/2209.08539
原版本与哈希见provenance.json；upstream/local_planner.py原样保留，不执行ROS1导入。
core.py提取作者choose_goal_state及MPC_ellip的AST，保留运动学、屏障函数、exceed_ob
筛选、目标函数、N25/T0.1/gamma0.3。仅匹配速度上限0.5m/s和1rad/s、增加0.25秒
IPOPT CPU预算及诊断引用；测试逆变换后要求与原AST完全一致。Jackal safe_dist原值0.5m。

输入使用现有DynaBARN模拟激光、圆拟合/KF及轮速/IMU定位；不使用动态真值或理想定位。
KF状态映射为未来0.1至2.5秒的圆椭圆序列，半轴=观测半径+0.95倍预测位置标准差。
UNKNOWN保持固定，不继续运动外推。静态OccupancyGrid按方向分割墙面再作外包椭圆，
不把连接U形墙整体当作实心障碍。作者原PCL/MBE/9维椭圆KF没有整体移植，因此结果
应标为“作者优化器＋共同输入适配”，不可当作原论文整套系统的原样复现。

控制输出50Hz，求解请求间隔至少0.05秒；数值求解失败、输入过期或解过期时发布零命令，修复原版首次失败数组未初始化及
过期控制回放问题。该停车属于适配保护，不修改D-CBF约束，也不附加FRS安全层。
保留原v_min0.1（距目标超过1m），近目标可-0.1；没有新增加速度约束。
exceed_ob原样保留，active_mask记录参与CBF约束的对象；跳过对象不应误说成已被CBF保护。

隔离镜像（不修改原barn-jazzy:local或宿主机Python）：
docker build --network host -t barn-jazzy-mpc-d-cbf:local -f navigation/benchmarks/dynamic_dynabarn/mpc_d_cbf/Dockerfile .
BENCH_IMAGE=barn-jazzy-mpc-d-cbf:local navigation/benchmarks/build.sh
单轮（新分组，禁止覆盖）：
BENCH_IMAGE=barn-jazzy-mpc-d-cbf:local DYNA_GROUP=my_mpc_run navigation/benchmarks/dynamic_dynabarn/run.sh mpc_d_cbf 0 0
相同镜像基线：
BENCH_IMAGE=barn-jazzy-mpc-d-cbf:local DYNA_GROUP=my_frs_run DYNA_ACTIVE_ESCAPE=false DYNA_LOCAL_PLANNER=false navigation/benchmarks/dynamic_dynabarn/run.sh frs_soft 0 0
不能同时启动其他规划器，旧ActiveEscape及新局部候选开关须关闭。

输出mpc_decisions.json包含求解状态、接受状态、求解总耗时、active_mask、约束数量、
预测椭圆及计划路径。commands.csv是实际发布的命令；collision为Gazebo接触。
只有真值目标距离<0.3m计到达，估计到达但真值未达标仍是失败。
静态/动态输入、速度边界、保守性、算法内部下限及求解器预算差异均须在对照中说明。
不要只凭求解成功或预测约束满足宣称真实闭环碰撞安全。
