#!/usr/bin/env python3
"""fake_nav_server.py — 模拟 /go_to_zone 服务，用于在没有真实 Nav2 时联调 tool_agent

行为与真实 nav2_task_manager 一致：阻塞若干秒模拟走路过程，再返回成功。
模拟耗时用环境变量 FAKE_NAV_SEC 控制（默认 3 秒）。
坑提醒：rclpy 的 Node 已有 handle 属性，子类切勿定义名为 handle 的方法，
否则 Node.__init__ 内部的 `with self.handle` 会拿到方法对象而报 AttributeError: __enter__。
"""
import os
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from zone_interfaces.srv import GoToZone

TRAVEL_SEC = float(os.environ.get("FAKE_NAV_SEC", "3"))


class FakeNav(Node):
    def __init__(self):
        super().__init__("fake_nav_server")
        self.cb = ReentrantCallbackGroup()
        self.srv = self.create_service(
            GoToZone, "go_to_zone", self.on_request, callback_group=self.cb)
        self.get_logger().info(f"假导航服务就绪 (每次耗时 {TRAVEL_SEC}s)")

    def on_request(self, req, resp):
        self.get_logger().info(f"==> 收到导航请求: {req.zone_name}")
        time.sleep(TRAVEL_SEC)          # 模拟阻塞式导航
        resp.success = True
        resp.message = f"成功：机器人已到达 [{req.zone_name}]"
        self.get_logger().info(resp.message)
        return resp


def main():
    rclpy.init()
    node = FakeNav()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
