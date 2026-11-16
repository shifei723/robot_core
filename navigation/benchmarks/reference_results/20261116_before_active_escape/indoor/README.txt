室内全流程实验与历史BARN汇总（先审阅，未改论文）
2026-11-16

先看 report/navigation-study-review-20261116.pdf。室内主实验24轮，旧版静态BARN12轮、动态DynaBARN9轮，共45条正式记录；分组阅读，不能汇成一个总成功率。

一、室内正式结果
CV-Hard：5/6到达，0碰撞，1超时，成功运行平均用时 66.40s。
FRS-Hard：4/6到达，0碰撞，2超时，成功运行平均用时 54.02s。
Soft-Time：5/6到达，1碰撞，0超时，成功运行平均用时 49.04s。
完整方法：6/6到达，0碰撞，0超时，成功运行平均用时 48.11s。

方案详见PROTOCOL.txt及报告第1节。三档障碍速度×四方法×每条件两次，统一0.5m/s机器人速度、感知/高度控制和A*后端。失败全部保留。代表运行事先指定Full .22 r0，其成功或失败均如实展示。补充过程图与GIF另记录数据来源和选择规则，不替换任何正式记录。

二、建议阅读顺序
1. report/navigation-study-review-20261116.pdf：中文方案、结果、失败诊断与论文组织建议。
2. figures/indoor_process_horizontal.png：预指定Full .22 r0的四阶段过程，成功/失败如实展示。独立stage PDF供LaTeX拼版。
3. figures/indoor_completed_replay.gif：成功过程回放（优先预指定运行），来源和加速倍率见animation_metadata.json。
4. figures/indoor_clearance_stops.png：逐工况净距与最长连续风险停车。
5. 室内汇总.csv、室内逐次结果.csv、BARN汇总.csv、BARN逐次结果.csv、全部实验逐次记录.csv：UTF-8 BOM，可用表格软件打开。
6. failure_diagnostics.json：未完成任务的控制状态、未知区与搜索日志诊断。

三、版本与证据
室内为本次冻结源码，旧BARN为20261113_v2，仅归档汇总，没有宣称重测最新版。室内是理想位姿/深度、运动学执行和真实生产节点闭环；BARN是Jackal接触仿真、激光/滤波里程计。机器人评价半径及计时不同，不跨组直接比较绝对用时。没有室内MPPI组。旧动态BARN仍有碰撞，报告完整列出。

四、复现
本机先进入robot_core并source /opt/ros/jazzy/setup.bash及install/ws_nav_frs/setup.bash。
新目录重跑（不覆盖本结果）：
/usr/bin/python3 navigation/scan_planner/tools/run_indoor_comparison.py --output-root output/navigation_study_repeat
该脚本依赖导航工程tools/run_indoor_comparison.py，源快照包含其完整实现；历史基准仅汇总，未重跑。脚本不会重建已安装二进制，复现前应按source_snapshot.tar.gz恢复对应导航源码并构建工作空间；不要在用户工作区直接覆盖源码。源快照只包含导航相关目录，不是整个robot_core仓库。
本目录已含code/fonts及历史数据；新输出目录若需后处理，先复制本目录code与fonts，再运行code/collect_barn.py生成历史汇总。后处理脚本按自身路径定位输出根目录。后处理：python3 code/analyze_study.py；python3 code/diagnose_failures.py；python3 code/plot_study.py；python3 code/build_report.py（均在本结果目录执行）。
字体：图中文字FandolSong，英文数字TeX Gyre Termes；报告使用相应XeLaTeX字体。PDF/PNG同时提供。

五、原始数据与分享包
本机indoor/<条件>/包含完整trace.json.gz（无损压缩）、snapshots.npz、静态PCD、summary.json、配置和节点日志。分享ZIP保留24轮summary及配置、结果CSV、图像、报告、协议、代码与源码快照；体积较大的trace/npz/PCD/逐节点日志留在本机，ZIP不能独立重画所有数据图。原始文件绝对路径由逐次CSV记录。
source_snapshot.tar.gz、manifest.json及verification.json用于源码与条件一致性核验。旧v2正式组和本次修复预试验分别保存在同级navigation_study_20261116_v2与navigation_repair_20261116目录。历史BARN原始数据仍位于其source_result指向的位置。预试验与首版协议检查在同级navigation_study_20261116单独保留，不混入45条正式统计；见protocol_development.json。
