# DynaBARN FRS联合滚动导航验证（2026-11-17）

本目录保存与当前代码配套的实验摘要和版本依据。完整原始数据、开发预试验、报告及动图保存在仓库根目录的 `output/dynabarn-frs-navigation-fix-20261117.zip`，其SHA256见 `archive.json`。该压缩包不随Git上传，需另行保管。

## 条件与结果

world_2，8个动态圆柱，初相位0/7/14 s，每方法每条件两次，共18次。三种方法共用原生激光、静态墙底图和velocity_imu融合配置；前进速度上限0.5 m/s、角速度上限1 rad/s。真值仅用于评价。

| 方法 | 到达 | 碰撞 | 超时 |
|---|---:|---:|---:|
| 联合滚动FRS | 6 | 0 | 0 |
| 基础FRS | 4 | 2 | 0 |
| MPPI | 0 | 6 | 0 |

联合滚动FRS通行时间39.77～47.85 s，成功平均43.79 s。不同方法成功样本构成不同，不能直接用总体成功时间均值宣称效率优势；同条件均成功的时间比较见 `matched_success_comparison.json`。

25项Python/真实ROS测试和6项C++规划测试通过；185项运行数据核验通过。评测规则参考版本与核查见 `reference_conformance.json`。这是一张地图的三种交互时序验证，不代表完整官方基准成绩或Slash实机验证。

## 实现范围

- 局部规划联合选择线速度与转向速度，执行与轨迹编号匹配的新鲜计划前缀。
- FRS作为软风险，近场CV误差包络、实体尺寸、静态地图及UNKNOWN保留硬检查。
- 编码器只融合机体系vx/vy，原生IMU融合yaw rate，避免轮速积分XY/yaw与同源速度重复融合；显式声明IMU话题并检查其新鲜度。
- 对齐起终点、移动0.1 m后开始100 s计时、真值距目标不大于1 m判到达；控制器完成仅作为诊断。
- MPPI显式使用融合里程计，速度平滑器的正负方向限幅配对。
- 保存旧定位与旧评测选项，以及此前相关的避险候选模块、MPC-D-CBF适配器及其来源记录。最终组关闭旧避险候选模块。

## 复测

在robot_core根目录执行，分组名必须尚未使用：

```bash
BENCH_IMAGE=barn-jazzy-mpc-d-cbf:local navigation/benchmarks/build.sh
BENCH_IMAGE=barn-jazzy-mpc-d-cbf:local DYNA_GROUP=frs_repeat_new navigation/benchmarks/dynamic_dynabarn/run_suite.sh 0
BENCH_IMAGE=barn-jazzy-mpc-d-cbf:local DYNA_GROUP=frs_repeat_new navigation/benchmarks/dynamic_dynabarn/run_suite.sh 1
```

默认 `DYNA_PROTOCOL=challenge`、`DYNA_LOCALIZATION=velocity_imu`。指定 `DYNA_LOCALIZATION=legacy` 恢复原融合，指定 `DYNA_PROTOCOL=legacy` 恢复旧评分；两者含义独立，不能与本组混算。

`manifest.json` 对应试验开始时冻结的源码和安装文件，新增本结果目录不参与其源码遍历。JSON内保留采集机器上的绝对路径；迁移原始数据后应重新指定结果组生成报告。
