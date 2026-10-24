# 轮式腿机器人 Mac 上位机

通过串口实时显示下位机 (`LeggedRobot` 固件) 回传的机器人信息, 并支持一键全量录制。

## 运行

**推荐用 Web 版** (浏览器显示, 渲染稳定, 4 张实时曲线图):

```bash
cd /Users/shifei/Documents/robot/slash_pc_test

./monitor.sh start               # 启动后台服务并自动打开浏览器
./monitor.sh start --no-browser  # 只启动后台服务, 不开浏览器
./monitor.sh stop                # 停止服务 (释放串口)
./monitor.sh restart             # 重启
./monitor.sh status              # 查看状态 / 帧率 / CRC 错误 / 是否在录制
./monitor.sh log                 # 实时日志 (Ctrl+C 退出)
./monitor.sh open                # 打开页面 http://127.0.0.1:8765/
```

也可以前台运行 (Ctrl+C 停止):

```bash
python3 web_monitor.py            # 默认 8765 端口, 自动连串口 + 开浏览器
python3 web_monitor.py --port 9000 --no-browser --no-auto-connect
```

- 仅依赖 **Python 标准库**, 无需 `pip install` 任何包
- 启动后**自动连接** 串口 (优先 `usbmodem` 设备, 波特率默认 `2000000`),
  页面顶部可随时切换串口/波特率、手动连接或断开
- 服务运行时**独占串口**; 要让其他程序 (Arduino 串口监视器、ROS 节点) 访问机器人,
  先 `./monitor.sh stop`

> 旧版 Tk 界面 (`pc_monitor.py`) 在部分 macOS 上会白屏, 已不再推荐使用。

## 界面说明

| 区域 | 内容 |
|------|------|
| 串口连接 | 串口选择 / 刷新 / 波特率 / 帧类型过滤 / 连接 / 断开 |
| 状态栏 | RX 频率、总帧数、变体帧数、CRC 错误、帧长错误、丢弃字节、帧类型、运行时长 |
| 里程计 | 左轮/右轮/前进速度 (m/s)、角速度 (rad/s) |
| IMU | 加速度、陀螺仪、四元数、欧拉角 (Roll/Pitch/Yaw) |
| 电机反馈 | 6 个电机 (左前/左后/右前/右后/左轮/右轮) 的位置/速度/力矩/温度 |
| SBUS | 遥控器 10 通道实时数值条 |
| 机器人状态 | 开关机、工作模式 (电机控制/底盘控制)、帧计数 |
| 实时曲线 | 轮式速度、姿态角、关节位置 |

## 录制 (全量录制)

1. 在「全量录制」区输入 **名称** (留空则按 `rec_年月日_时分秒` 自动命名;
   重名会自动加 `_1`、`_2` 后缀)
2. 选择保存目录 (默认 `slash_pc_test/logs/`)
3. 勾选录制内容: `解码CSV` + `原始bin` (默认全选 = 全量录制)
4. 点 **● 开始记录** → 实验 → 点 **■ 结束记录**

每次录制生成 `logs/<名称>/` 目录:

```
logs/<名称>/
├── data.csv     解码后的全量数据 (含时间戳, 58 列)
├── raw.bin      串口原始字节流 (完整留存, 可用 robot_protocol.py 重新解码)
└── meta.json    录制信息 (端口/波特率/帧统计/字段说明)
```

`data.csv` 列说明见 `meta.json` 的 `csv_field_desc`。时间戳两列:
`t_s` (录制起始起的秒数) 和 `wall_time` (墙钟时间)。

## 协议说明

与固件 `LeggedRobot.ino` / `LeggedRobot.h` 及参考代码
`robot_core/robot/wheeled_legged_pkg/src/robot_functions.cpp` 一致:

- 帧格式: `0x7B 0x2D | len:u16 | payload | crc:u16` (小端)
- CRC16: Modbus 风格 (初值 `0xFFFF`, 多项式 `0xA001`), 覆盖除 CRC 字段外全部字节
- 帧类型 (字段偏移已用 clang `static_assert` 依据 `robot_structs.h` 验证):
  - `ROS_body_t` 182B 全量帧: 里程计@4, IMU@20, 电机@60, SBUS@156, 状态@176
  - `ROS_body_t1` 124B 精简帧: 电机@4, SBUS@100
  - 181B 变体帧: 实测固件每轮会多发一帧「去掉 4 字节帧头的全量帧载荷」,
    解析器自动识别 (CRC 重建校验), 因此帧率约为固件主循环的 2 倍

## 文件说明

| 文件 | 作用 |
|------|------|
| `web_monitor.py` | **Web 上位机主程序** (HTTP 服务 + 串口读取 + 录制, 纯标准库) |
| `web/index.html` | 前端页面 (仪表盘 + 4 张 Canvas 实时曲线 + 录制面板) |
| `monitor.sh` | 后台服务控制脚本 (start/stop/restart/status/log/open) |
| `robot_protocol.py` | CRC16 / 帧解析 / 结构体解码 (`python3 robot_protocol.py` 可自测) |
| `mac_serial.py` | 纯标准库串口 (termios + IOSSIOSPEED, 支持 2Mbps; 打开时清除 DTR/RTS 防止误触 ESP32 下载模式) |
| `recorder.py` | 录制器 (CSV + 原始 bin + meta) |
| `pc_monitor.py` | 旧版 Tk 界面 (部分 macOS 白屏, 不推荐) |

## 常见问题

- **窗口白屏/花屏**: 旧版 Tk 界面的已知问题, 请改用 Web 版 `./monitor.sh start`
- **打不开串口**: 确认机器人已开机、USB 已连接; 若被其他程序 (Arduino IDE
  串口监视器等) 或上一个未停止的上位机服务占用, 请先关闭 / `./monitor.sh stop`
- **端口 8765 被占用**: 换端口 `./monitor.sh` 前先 `PORT=9000 ./monitor.sh start`
- **无数据**: 检查状态栏 RX 频率; 若为 0, 确认波特率是 `2000000`
  (固件 `RosSerial.begin(2000000, SERIAL_8N1, RXD1, TXD1)`)
