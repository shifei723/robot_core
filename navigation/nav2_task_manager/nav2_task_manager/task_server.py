import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from zone_interfaces.srv import GoToZone
import yaml
import os

class ZoneTaskServer(Node):
    def __init__(self):
        super().__init__('zone_task_server')
        
        # 核心：使用多线程回调组！
        # 因为 Nav2 的等待是阻塞的，如果不用多线程，会导致 ROS 2 服务死锁
        self.cb_group = ReentrantCallbackGroup()
        
        # 1. 创建服务
        self.srv = self.create_service(
            GoToZone, 
            'go_to_zone', 
            self.execute_nav_task, 
            callback_group=self.cb_group
        )
        
        # 2. 初始化 Nav2 客户端
        self.navigator = BasicNavigator()
        
        # 3. 指定你之前插件保存的 YAML 绝对路径
        # 注意：RViz 的 ZonePanel 是用 __FILE__ 把 locations.yaml 写到源码目录的，
        # 所以这里必须读同一个源码路径，否则会找不到文件。
        # 可通过 ROS 参数 yaml_path 覆盖默认值。
        default_yaml = '/data/sf_code/ros_ws/nav/rviz2_zone_plugin/locations.yaml'
        self.declare_parameter('yaml_path', default_yaml)
        self.yaml_path = self.get_parameter('yaml_path').get_parameter_value().string_value
        
        # 等待导航就绪
        self.get_logger().info(f"使用地点数据文件: {self.yaml_path}")
        self.get_logger().info("等待 Nav2 激活...")
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')
        self.get_logger().info("=== 区域导航服务已就绪！可以发送指令 ===")

    def load_locations(self):
        if not os.path.exists(self.yaml_path):
            self.get_logger().error(f"找不到 YAML 文件: {self.yaml_path}")
            return {}
        with open(self.yaml_path, 'r') as f:
            data = yaml.safe_load(f)
            return data.get('zones', {}) if data else {}

    def execute_nav_task(self, request, response):
        zone_name = request.zone_name
        self.get_logger().info(f"==> 收到任务：前往 [{zone_name}]")
        
        # 每次接到任务都重新读一次 YAML，这样你在 RViz 里改了位置不需要重启节点
        locations = self.load_locations()
        
        # 检查名字是否存在
        if zone_name not in locations:
            response.success = False
            response.message = f"错误：地图数据库中不存在区域 '{zone_name}'"
            self.get_logger().warn(response.message)
            return response

        # 构建目标点
        pose_data = locations[zone_name]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose_data['x']
        goal_pose.pose.position.y = pose_data['y']
        goal_pose.pose.orientation.z = pose_data['z']
        goal_pose.pose.orientation.w = pose_data['w']

        # 发送导航目标
        self.navigator.goToPose(goal_pose)

        # 阻塞轮询，等待任务完成
        while not self.navigator.isTaskComplete():
            # 可以在这里打印剩余距离，或执行其他检查
            pass

        # 解析最终结果
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            response.success = True
            response.message = f"成功：机器人已到达 [{zone_name}]"
        elif result == TaskResult.CANCELED:
            response.success = False
            response.message = f"警告：前往 [{zone_name}] 的任务被强行取消"
        elif result == TaskResult.FAILED:
            response.success = False
            response.message = f"失败：无法到达 [{zone_name}] (可能遇到死胡同或被卡死)"
        else:
            response.success = False
            response.message = "未知错误"

        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    
    task_server = ZoneTaskServer()
    
    # 必须使用多线程执行器，配合 ReentrantCallbackGroup 避免死锁
    executor = MultiThreadedExecutor()
    executor.add_node(task_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        task_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()