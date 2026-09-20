from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取功能包安装后的共享目录路径，自动定位参数文件
    pkg_share = get_package_share_directory("omni_node")
    params_file = os.path.join(pkg_share, "config", "omni_params.yaml")

    omni_node = Node(
        package="omni_node",          # 功能包名
        executable="omni_node",       # 可执行文件名
        name="omni_multimodal_node",  # 节点名，必须与 yaml 顶层名称一致
        output="screen",              # 日志输出到终端
        parameters=[params_file]      # 加载参数文件
    )

    return LaunchDescription([omni_node])