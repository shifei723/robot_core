#!/usr/bin/env python3
"""
Tool-Call 执行节点：把 Omni 模型输出的工具调用 JSON 拆解成任务队列并顺序执行

工作流程：
1. 订阅 /omni/output_text，按 sid 聚合流式分段（模型的 JSON 会分片到达）
2. 收到 /omni/node_state 的 "Infer finished" 后，从聚合文本中提取 JSON
3. 解析出任务序列，压入队列，由工作线程**顺序**执行（前一个做完再做下一个）
4. 每步执行结果通过 /robot/speech 播报（由 tts_bridge 转给当前 TTS 后端）

支持的动作：
- navigate: 调用 /go_to_zone 服务导航到命名地点（阻塞直到 Nav2 完成）
- nod:      点头 —— pitch 俯仰轨迹
- shake:    摇头 —— roll 横滚轨迹

期望的模型输出格式（单行 JSON）：
    {"tasks":[{"action":"navigate","target":"kitchen"},{"action":"nod"}]}

姿态控制接口（对接 wheeled_legged_pkg 的 wl_base_node）：
    话题: /cmd_posture      类型: sensor_msgs/JointState
    name     = [joint_height, joint_roll, joint_pitching]   三个必须同时存在
    position = [高度(米),  roll(度),   pitch(度)]
    下位机限幅: roll ±15°, pitch ±35°, 高度 0.14~0.36m (默认 0.25m)

    两个关键约束（来自 wl_base_node.cpp 源码）：
    1. 三个关节名缺任意一个，整条消息会被直接忽略，所以 height 必须一起发
    2. 超过 1 秒未收到新姿态，下位机自动回到默认高度且 roll/pitch 归零，
       因此动作期间必须持续以 20Hz 下发，停发即自动回中位（天然的安全保护）

    注意: 该话题与 Nav2 的 /cmd_vel 是独立两路指令，互不干扰。

用法：
    python3 tool_agent.py
    python3 tool_agent.py --ros-args -p locations_yaml:=/path/to/locations.yaml
"""

import json
import math
import re
import threading
import time
from collections import deque

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

import yaml

from zone_interfaces.srv import GoToZone

# omni_node 输出带 "[sid:123] " 前缀
SID_RE = re.compile(r"^\[sid:(\d+)\]\s*(.*)", re.S)

# 与 wl_base_node.cpp 保持一致的硬限幅（超出会被下位机 clamp，这里提前守住）
JOINT_NAMES = ["joint_height", "joint_roll", "joint_pitching"]
ROLL_LIMIT_DEG = 15.0
PITCH_LIMIT_DEG = 35.0
HEIGHT_MIN_M = 0.14
HEIGHT_MAX_M = 0.36


def extract_json(text: str):
    """从可能混有自然语言的文本中提取第一个含 tasks 的 JSON 对象。

    做括号配对扫描而非整串 json.loads，因为模型常在 JSON 前后带多余的话。

    容错重点：3B 小模型的输出经常被截断，实机观测到过
    {"tasks":[{"action":"nod"}]  这种少一个右花括号的形态。
    若扫到末尾仍未配对完，就按残余的括号栈自动补全再试一次，
    避免因一个字符就丢掉整条指令（那样既不执行也不播报，体验最差）。
    """
    def try_load(s: str):
        try:
            obj = json.loads(s)
        except json.JSONDecodeError:
            return None
        return obj if isinstance(obj, dict) and "tasks" in obj else None

    start = text.find("{")
    while start != -1:
        stack = []          # 记录未闭合的括号，用于截断时补全
        in_str = False
        escaped = False
        for i in range(start, len(text)):
            c = text[i]
            # 字符串内的括号不参与配对
            if in_str:
                if escaped:
                    escaped = False
                elif c == "\\":
                    escaped = True
                elif c == '"':
                    in_str = False
                continue
            if c == '"':
                in_str = True
            elif c in "{[":
                stack.append(c)
            elif c in "}]":
                if stack:
                    stack.pop()
                if not stack:
                    obj = try_load(text[start:i + 1])
                    if obj is not None:
                        return obj
                    break
        else:
            # 扫到文本末尾仍有未闭合括号 → 被截断了，尝试补全
            if stack:
                tail = text[start:]
                if in_str:
                    tail += '"'          # 字符串也断在中间
                closing = "".join("}" if b == "{" else "]" for b in reversed(stack))
                obj = try_load(tail + closing)
                if obj is not None:
                    return obj
        start = text.find("{", start + 1)
    return None


class ToolAgentNode(Node):
    """解析工具调用并顺序执行导航与本体动作。"""

    def __init__(self):
        super().__init__("tool_agent_node")

        # ── 参数 ──────────────────────────────────────────────────
        self.declare_parameter("text_topic", "/omni/output_text")
        self.declare_parameter("state_topic", "/omni/node_state")
        self.declare_parameter("speech_topic", "/robot/speech")
        self.declare_parameter("interrupt_topic", "/robot/interrupt")
        self.declare_parameter("posture_topic", "/cmd_posture")
        self.declare_parameter("zone_service", "/go_to_zone")
        self.declare_parameter(
            "locations_yaml",
            "/data/sf_code/ros_ws/nav/rviz2_zone_plugin/locations.yaml")
        # 动作幅度与节奏（单位：度，与 /cmd_posture 一致）
        self.declare_parameter("nod_pitch_deg", 15.0)    # 点头俯仰幅度，限幅 ±35
        self.declare_parameter("shake_roll_deg", 8.0)    # 摇头横滚幅度，限幅 ±15
        self.declare_parameter("body_height", 0.25)      # 动作期间保持的腿长
        self.declare_parameter("action_cycles", 2)       # 往复次数
        self.declare_parameter("action_period", 1.0)     # 单次往复周期 (秒)
        self.declare_parameter("action_rate", 20.0)      # 轨迹发布频率 (Hz)，必须 >1Hz 避免超时回中
        self.declare_parameter("nav_timeout", 300.0)     # 单次导航最长等待 (秒)

        self.text_topic = self.get_parameter("text_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.speech_topic = self.get_parameter("speech_topic").value
        self.interrupt_topic = self.get_parameter("interrupt_topic").value
        self.posture_topic = self.get_parameter("posture_topic").value
        self.zone_service = self.get_parameter("zone_service").value
        self.locations_yaml = self.get_parameter("locations_yaml").value
        self.nod_pitch = self.get_parameter("nod_pitch_deg").value
        self.shake_roll = self.get_parameter("shake_roll_deg").value
        self.body_height = self.get_parameter("body_height").value
        self.cycles = self.get_parameter("action_cycles").value
        self.period = self.get_parameter("action_period").value
        self.rate = self.get_parameter("action_rate").value
        self.nav_timeout = self.get_parameter("nav_timeout").value

        # 就地收敛到下位机合法范围，避免发出会被默默 clamp 的无效幅度
        self.nod_pitch = max(-PITCH_LIMIT_DEG, min(PITCH_LIMIT_DEG, self.nod_pitch))
        self.shake_roll = max(-ROLL_LIMIT_DEG, min(ROLL_LIMIT_DEG, self.shake_roll))
        self.body_height = max(HEIGHT_MIN_M, min(HEIGHT_MAX_M, self.body_height))

        # ── ROS 接口 ─────────────────────────────────────────────
        self.cb_group = ReentrantCallbackGroup()

        self.speech_pub = self.create_publisher(String, self.speech_topic, 10)
        self.posture_pub = self.create_publisher(JointState, self.posture_topic, 10)

        self.nav_client = self.create_client(
            GoToZone, self.zone_service, callback_group=self.cb_group)

        self.create_subscription(
            String, self.text_topic, self._on_text, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            String, self.state_topic, self._on_state, 10,
            callback_group=self.cb_group)
        # 唤醒打断: 清空待执行队列 + 中止当前动作
        self.create_subscription(
            String, self.interrupt_topic, self._on_interrupt, 10,
            callback_group=self.cb_group)

        # ── 文本聚合状态 ─────────────────────────────────────────
        self._cur_sid = None
        self._buffer = []

        # ── 任务队列与工作线程 ───────────────────────────────────
        self._queue = deque()
        self._queue_lock = threading.Lock()
        self._queue_cv = threading.Condition(self._queue_lock)
        self._stop = False
        # 打断标志: 动作循环每帧检查，能在 50ms 内停下来
        self._abort = threading.Event()
        self._worker = threading.Thread(target=self._work_loop, daemon=True)
        self._worker.start()

        zones = self._load_zones()
        self.get_logger().info(
            f"ToolAgent ready | 监听 {self.text_topic} | 姿态话题 {self.posture_topic}")
        self.get_logger().info(
            f"动作幅度: 点头 pitch={self.nod_pitch}° 摇头 roll={self.shake_roll}° "
            f"高度={self.body_height}m")
        self.get_logger().info(
            f"可用地点: {', '.join(f'{k}({v})' for k, v in zones.items()) or '(空)'}")

    # ══════════════════════════════════════════════════════════
    # 地点表：id -> 中文名
    # ══════════════════════════════════════════════════════════
    def _load_zones(self) -> dict:
        """每次用时重新读取，便于在 RViz 里改完地点无需重启本节点。"""
        try:
            with open(self.locations_yaml, "r") as f:
                data = yaml.safe_load(f) or {}
            zones = data.get("zones", {}) or {}
            return {zid: (info.get("name") or zid) for zid, info in zones.items()}
        except (FileNotFoundError, OSError, yaml.YAMLError) as e:
            self.get_logger().warn(f"读取地点文件失败: {e}")
            return {}

    def _resolve_zone(self, target: str):
        """把模型给的 target 归一化成 zone id；支持英文 id 与中文名两种写法。"""
        if not target:
            return None
        target = target.strip()
        zones = self._load_zones()
        if target in zones:                       # 直接是 id
            return target
        for zid, cname in zones.items():          # 中文名精确匹配
            if target == cname:
                return zid
        for zid, cname in zones.items():          # 中文名包含匹配（如"去厨房"）
            if cname and cname in target:
                return zid
        return None

    # ══════════════════════════════════════════════════════════
    # 文本聚合：模型的 JSON 会被流式分片
    # ══════════════════════════════════════════════════════════
    def _on_text(self, msg: String):
        m = SID_RE.match(msg.data)
        sid, piece = (m.group(1), m.group(2)) if m else ("?", msg.data)

        if sid != self._cur_sid:
            self._flush()          # 上一轮没被 state 触发过，这里补一次
            self._cur_sid = sid
            self._buffer = []
        self._buffer.append(piece)

    def _on_state(self, msg: String):
        # 推理结束是最可靠的聚合完成信号
        if "Infer finished" in msg.data:
            self._flush()

    def _flush(self):
        """聚合完成，尝试提取工具调用并入队。"""
        if not self._buffer:
            return
        full = "".join(self._buffer)
        self._buffer = []
        self._cur_sid = None

        obj = extract_json(full)
        if obj is None:
            # 文本里有 JSON 痕迹却解不出来，说明模型想下指令但格式坏得太厉害。
            # 必须给反馈：tts_bridge 会把这种文本当工具调用跳过不朗读，
            # 若这里也默不作声，用户就面对完全的沉默（既不动作也不回应）。
            if '"tasks"' in full or '"action"' in full:
                self.get_logger().warn(f"检测到工具调用意图但解析失败: {full[:120]}")
                self._speak("指令没听清，能再说一遍吗")
            # 否则是普通聊天，交给 tts_bridge 正常播报，本节点不干预
            return

        tasks = obj.get("tasks")
        if not isinstance(tasks, list) or not tasks:
            self.get_logger().warn(f"JSON 里没有有效的 tasks: {full[:120]}")
            self._speak("我没听明白要做什么")
            return

        self.get_logger().info(f"解析出 {len(tasks)} 个任务: {tasks}")
        self._speak(f"好的，我来完成{len(tasks)}个任务")
        # 新一轮指令到达，清除上一轮的打断标志
        self._abort.clear()
        with self._queue_cv:
            for t in tasks:
                self._queue.append(t)
            self._queue_cv.notify()

    # ══════════════════════════════════════════════════════════
    # 工作线程：顺序执行，前一个完成才做下一个
    # ══════════════════════════════════════════════════════════
    def _on_interrupt(self, msg: String):
        """唤醒打断: 清空待执行队列，并让正在跑的动作尽快收手。

        不在这里播报“已停止”：tts_bridge 此时已关闭闸门，发也不会出声，
        而且用户此刻正在说新指令，不应再插话。
        """
        with self._queue_cv:
            dropped = len(self._queue)
            self._queue.clear()
        self._abort.set()
        # 丢弃被打断那轮的文本残片，避免与下一轮粘连
        self._buffer = []
        self._cur_sid = None
        self.get_logger().info(
            f"[打断] 来源={msg.data} → 丢弃 {dropped} 个待执行任务，中止当前动作")

    def _work_loop(self):
        while not self._stop:
            with self._queue_cv:
                while not self._queue and not self._stop:
                    self._queue_cv.wait(timeout=0.5)
                if self._stop:
                    return
                task = self._queue.popleft()
                remaining = len(self._queue)
            # 打断后可能还有刚取出的任务，直接丢弃
            if self._abort.is_set():
                continue
            try:
                self._run_task(task)
            except Exception as e:                      # 单个任务失败不拖垮队列
                self.get_logger().error(f"任务执行异常: {e}")
                self._speak("这个动作没能完成")
            if remaining == 0 and not self._abort.is_set():
                self._speak("全部任务已完成")

    def _run_task(self, task: dict):
        action = (task.get("action") or "").strip().lower()
        self.get_logger().info(f"▶ 执行: {task}")

        if action in ("navigate", "goto", "go_to", "nav"):
            self._do_navigate(task.get("target") or task.get("zone"))
        elif action in ("nod", "点头"):
            self._do_attitude("pitch", self.nod_pitch, "点头")
        elif action in ("shake", "shake_head", "摇头"):
            self._do_attitude("roll", self.shake_roll, "摇头")
        else:
            self.get_logger().warn(f"未知动作: {action}")
            self._speak(f"我还不会{action}这个动作")

    # ── 导航 ─────────────────────────────────────────────────
    def _do_navigate(self, target):
        zone_id = self._resolve_zone(target)
        if zone_id is None:
            known = "、".join(self._load_zones().values()) or "（地点库为空）"
            self.get_logger().warn(f"未知地点: {target}")
            self._speak(f"我不知道{target}在哪里，我认识的地方有{known}")
            return

        if not self.nav_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"导航服务 {self.zone_service} 未就绪")
            self._speak("导航系统还没准备好")
            return

        cname = self._load_zones().get(zone_id, zone_id)
        self._speak(f"正在前往{cname}")

        req = GoToZone.Request()
        req.zone_name = zone_id
        future = self.nav_client.call_async(req)

        # 服务端会阻塞到 Nav2 走完，这里轮询等待（MultiThreadedExecutor 下安全）
        deadline = time.time() + self.nav_timeout
        while not future.done():
            if self._abort.is_set():
                # 打断不能真正取消已发出的导航（/go_to_zone 无 cancel 接口），
                # 但不再阻塞在这里，后续任务已被清空，因此不会继续执行
                self.get_logger().warn(
                    f"导航到 {cname} 被打断（Nav2 仍会走完本次目标）")
                return
            if time.time() > deadline:
                self.get_logger().error(f"导航到 {zone_id} 超时")
                self._speak(f"去{cname}花的时间太久了，我先停下")
                return
            time.sleep(0.2)

        resp = future.result()
        if resp is not None and resp.success:
            self.get_logger().info(f"导航成功: {resp.message}")
            self._speak(f"已经到{cname}了")
        else:
            detail = resp.message if resp is not None else "服务无响应"
            self.get_logger().warn(f"导航失败: {detail}")
            self._speak(f"没能到达{cname}")

    # ── 本体姿态动作（点头 / 摇头）────────────────────────────
    def _publish_posture(self, roll_deg: float, pitch_deg: float):
        """向 /cmd_posture 发一帧姿态。

        三个关节名必须同时带上，缺一个 wl_base_node 会整条丢弃；
        高度固定为 body_height，避免动作时腿长被意外改变。
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(self.body_height), float(roll_deg), float(pitch_deg)]
        self.posture_pub.publish(msg)

    def _do_attitude(self, axis: str, amplitude: float, label: str):
        """按正弦轨迹往复摆动指定姿态轴，结束后回中位。

        axis: "pitch" 俯仰（点头）; "roll" 横滚（摇头）。幅度单位为度。
        以 self.rate 持续下发，避开下位机“1 秒无更新就回中位”的保护。
        """
        self._speak(label)

        dt = 1.0 / self.rate
        steps_per_cycle = max(1, int(self.period * self.rate))
        total = steps_per_cycle * max(1, int(self.cycles))

        for i in range(total):
            if self._abort.is_set():        # 每帧检查，打断后 50ms 内退出
                self.get_logger().info(f"{label} 被打断，提前回中位")
                break
            phase = 2.0 * math.pi * (i % steps_per_cycle) / steps_per_cycle
            value = amplitude * math.sin(phase)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(JOINT_NAMES)
            # 三个关节必须同时带上，缺一个 wl_base_node 会整条丢弃
            if axis == "pitch":
                msg.position = [float(self.body_height), 0.0, float(value)]
            else:
                msg.position = [float(self.body_height), float(value), 0.0]
            self.posture_pub.publish(msg)
            time.sleep(dt)

        # 回到中位姿态，并多发几帧确保下位机能接住
        # （wl_base_node 有 cmd_posture_updated_sign_ 握手，上一帧未被消费时会跳过新值）
        for _ in range(3):
            self._publish_posture(0.0, 0.0)
            time.sleep(dt)
        if self._abort.is_set():
            self.get_logger().info(f"{label} 已中断并回中位")
        else:
            self.get_logger().info(f"{label} 完成（{axis} 幅度 {amplitude}°）")

    # ── 语音反馈 ─────────────────────────────────────────────
    def _speak(self, text: str):
        self.speech_pub.publish(String(data=text))
        self.get_logger().info(f"[播报] {text}")

    def shutdown(self):
        with self._queue_cv:
            self._stop = True
            self._queue_cv.notify_all()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ToolAgentNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("ToolAgent 退出")
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
