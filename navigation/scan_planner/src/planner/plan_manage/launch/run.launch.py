"""Main ROS 2 launch entry point for simulation and real-robot remapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


# Robot model config: name → (description_pkg, urdf_path, child_frame_id)
def _robot_urdf_source(cfg):
    """Return a Command substitution for generating robot_description."""
    if cfg["urdf"]:
        return Command(cfg["urdf"])
    return Command(["cat ", os.path.join(
        get_package_share_directory(cfg["pkg"]), "urdf", "slash.urdf")])


ROBOT_CONFIGS = {
    "go2": {
        "pkg": "go2_description",
        "urdf": ["xacro ", os.path.join(
            get_package_share_directory("go2_description"), "xacro", "robot.xacro"),
            " use_gazebo:=false"],
        "child_frame_id": "base",
        "kinematic_sim": "go2_kinematic_sim",
        "gait_publisher": "go2_gait_publisher",
    },
    "slash": {
        "pkg": "slash_description",
        "urdf": None,  # using cat to read static URDF
        "child_frame_id": "base_link",
        "kinematic_sim": "slash_kinematic_sim",
        "gait_publisher": "slash_gait_publisher",
    },
}


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    robot_model = LaunchConfiguration("robot_model").perform(context)
    cfg = ROBOT_CONFIGS.get(robot_model)
    if cfg is None:
        raise RuntimeError(f"robot_model must be one of {list(ROBOT_CONFIGS.keys())}")

    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    is_real = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    # 不显式指定时按场景取默认: 真机走 D435i 深度图, 仿真走模拟雷达
    if not sensor_type:
        sensor_type = "depth" if is_real else "lidar"
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if controller_mode not in ("open_loop", "closed_loop"):
        raise RuntimeError("controller_mode must be 'open_loop' or 'closed_loop'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )

    if is_real:
        # VINS(局部, 连续) + rtabmap(全局, 会跳变) 定位链路:
        # 规划全部跑在 VINS 的 world 系, 重定位修正不会传到轨迹上。
        # odom_to_tf.py 把 VINS 位姿转成 base_link @ world 后发到 /odometry_base。
        body_pose = "/odometry_base"
        # depth 模式下 need_extrinsic=True, grid_map 会拿机体位姿乘外参
        # 自行算出相机位姿, 所以这里和 body_pose 共用同一个话题
        sensor_pose = "/odometry_base"
        cloud = "/LIO/clouds_lidar"
        depth = "/camera/camera/aligned_depth_to_color/image_raw"
        cloud_is_world = False
        need_extrinsic = True
        # D435i 640x360 Color 实测值 (rs-enumerate-devices -c),
        # aligned_depth_to_color 的内参等于 color 内参。
        # 换分辨率或换相机必须重新实测, 内参不对会整体投影错位。
        intrinsics = {
            "grid_map.cx": 320.937377929688,
            "grid_map.cy": 190.828170776367,
            "grid_map.fx": 455.744598388672,
            "grid_map.fy": 455.591918945312,
        }
    else:
        body_pose = "/quad_0/body_pose"
        sensor_pose = "/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose"
        cloud = "/quad_0/cloud"
        depth = "/quad_0/depth"
        cloud_is_world = True
        need_extrinsic = False
        intrinsics = {}

    common = {"use_sim_time": use_sim_time}
    planner_overrides = {
        **common,
        **intrinsics,
        "fsm.navi_mode": navi_mode,
        "grid_map.sensor_type": sensor_type,
        "grid_map.cloud_is_world": cloud_is_world,
        "grid_map.need_extrinsic": need_extrinsic,
    }
    if not is_real:
        # 仿真世界边界与仿真地图尺寸保持一致, 边界外不可达
        planner_overrides.update({
            "grid_map.world_bound_enable": True,
            "grid_map.world_size_x": float(LaunchConfiguration("map_size_x").perform(context)),
            "grid_map.world_size_y": float(LaunchConfiguration("map_size_y").perform(context)),
            # 仿真地图是完整静态场景：允许用已识别的横梁净空规划下蹲；
            # 真机保持 planner.yaml 中的保守未知区策略。
            "grid_map.allow_unverified_ceiling_clearance": True,
        })
    else:
        planner_overrides["grid_map.world_bound_enable"] = False
        # 真机首跑降速: 深度相机有效视距只有 3m 左右, 视野外全是 unknown,
        # 速度上限必须配合视距压低, 否则来不及对新出现的障碍反应。
        real_max_vel = float(LaunchConfiguration("real_max_vel").perform(context))
        real_max_acc = float(LaunchConfiguration("real_max_acc").perform(context))
        planner_overrides.update({
            "manager.max_vel": real_max_vel,
            "manager.max_acc": real_max_acc,
            "optimization.max_vel": real_max_vel,
            "optimization.max_acc": real_max_acc,
        })

    # RViz2 的 SetGoal 在当前 vins_map.rviz 中发 /goal_pose，旧版 slash.rviz
    # 使用 /move_base_simple/goal。规划器不看 frame_id，真机模式下两者都先经
    # goal_frame_bridge 转到 world 再喂给规划器，中间话题避免自环。
    planner_goal = "/planner/goal_world" if is_real else "/move_base_simple/goal"
    actions = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml] + ([keypoints_file] if keypoints_file else []) + [planner_overrides],
            remappings=[
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", planner_goal),
                ("initial_path", "/initial_path"),
            ],
        )
    ]
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name=f"{robot_model}_robot_state_publisher",
            output="screen",
            parameters=[
                common,
                {"robot_description": ParameterValue(_robot_urdf_source(cfg), value_type=str)},
            ],
        )
    )

    # 真机首跑跟踪器限速, 与规划器上限保持一致。
    # open_loop 自己积分出 body_pose, 只用于仿真, 不参与真机限速。
    controller_overrides = [common]
    if is_real:
        controller_overrides = [common, {
            "max_vx": float(LaunchConfiguration("real_max_vel").perform(context)),
            "max_vyaw": float(LaunchConfiguration("real_max_vyaw").perform(context)),
        }]

    if controller_mode == "open_loop":
        actions.append(
            Node(
                package="scan_planner",
                executable="open_loop_controller",
                name="open_loop_controller",
                output="screen",
                parameters=[controllers_yaml, common],
                remappings=[
                    ("planning/bspline", "/planning/bspline"),
                    ("body_pose", body_pose),
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="scan_planner",
                executable="closed_loop_controller",
                name="closed_loop_controller",
                output="screen",
                parameters=[controllers_yaml] + controller_overrides,
                remappings=[
                    ("body_pose", body_pose),
                    ("cmd_vel", "/cmd_vel" if is_real else "/quad_0/cmd_vel"),
                ],
            )
        )
        if not is_real:
            actions.append(
                Node(
                    package="scan_planner",
                    executable=cfg["kinematic_sim"],
                    name=f"{robot_model}_kinematic_sim",
                    output="screen",
                    parameters=[
                        controllers_yaml,
                        common,
                        {
                            "init_x": float(LaunchConfiguration("init_x").perform(context)),
                            "init_y": float(LaunchConfiguration("init_y").perform(context)),
                            "init_z": float(LaunchConfiguration("init_z").perform(context)),
                            "publish_tf": True,
                            "child_frame_id": cfg["child_frame_id"],
                        },
                    ],
                    remappings=[
                        ("body_pose", "/quad_0/body_pose"),
                        ("cmd_vel", "/quad_0/cmd_vel"),
                    ],
                )
            )

    if is_real:
        # 真机下 slash_leg_controller 不启动, leg_height_cmd 没有消费者,
        # 由本节点转成底盘认的 /cmd_posture (JointState)。
        actions.append(
            Node(
                package="scan_planner",
                executable="leg_height_to_posture.py",
                name="leg_height_to_posture",
                output="screen",
                parameters=[common],
                remappings=[("leg_height_cmd", "/leg_height_cmd")],
            )
        )
        # 目标点 map → world 转换, 并带里程计发散看门狗
        actions.append(
            Node(
                package="scan_planner",
                executable="goal_frame_bridge.py",
                name="goal_frame_bridge",
                output="screen",
                parameters=[common, {"target_frame": "world"}],
                remappings=[
                    ("goal_in", "/goal_pose"),
                    ("goal_in_legacy", "/move_base_simple/goal"),
                    ("goal_out", planner_goal),
                    ("body_pose", body_pose),
                    ("cmd_vel", "/cmd_vel"),
                ],
            )
        )

    if not is_real:
        # Slash uses Python leg height controller; go2 uses C++ gait publisher
        if robot_model == "slash":
            actions.append(
                Node(
                    package="scan_planner",
                    executable="slash_leg_controller.py",
                    name="slash_leg_controller",
                    output="screen",
                    parameters=[controllers_yaml, common],
                    remappings=[
                        ("cmd_vel", "/quad_0/cmd_vel"),
                        ("body_pose", body_pose),
                    ],
                )
            )
        else:
            actions.append(
                Node(
                    package="scan_planner",
                    executable=cfg["gait_publisher"],
                    name=f"{robot_model}_gait_publisher",
                    output="screen",
                    parameters=[controllers_yaml, common],
                    remappings=[("body_pose", body_pose)],
                )
            )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(scan_share, "launch", "simulator.launch.py")
                ),
                launch_arguments={
                    **{
                        name: LaunchConfiguration(name)
                        for name in (
                            "is_real_world",
                            "use_gpu",
                            "use_pcd_map",
                            "pcd_map_file",
                            "map_size_x",
                            "map_size_y",
                            "map_size_z",
                            "beam_clearance",
                            "use_sim_time",
                        )
                    },
                    # 传解析后的值, 避免把 sensor_type 的置空默认漏到仿真器
                    "sensor_type": sensor_type,
                }.items(),
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_model", default_value="go2"),
            DeclareLaunchArgument("is_real_world", default_value="false"),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            # 置空时自动选: 真机 depth, 仿真 lidar
            DeclareLaunchArgument("sensor_type", default_value=""),
            DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_pcd_map", default_value="false"),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument("map_size_x", default_value="20.0"),
            DeclareLaunchArgument("map_size_y", default_value="20.0"),
            DeclareLaunchArgument("map_size_z", default_value="3.0"),
            DeclareLaunchArgument("beam_clearance", default_value=""),
            DeclareLaunchArgument("init_x", default_value="-8.0"),
            DeclareLaunchArgument("init_y", default_value="-8.0"),
            DeclareLaunchArgument("init_z", default_value="0.28"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # 真机首跑限速 (仅 is_real_world:=true 时生效, 不影响仿真默认值)
            DeclareLaunchArgument("real_max_vel", default_value="0.3"),
            DeclareLaunchArgument("real_max_acc", default_value="0.3"),
            DeclareLaunchArgument("real_max_vyaw", default_value="0.5"),
            OpaqueFunction(function=_setup),
        ]
    )