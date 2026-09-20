import open3d as o3d
import numpy as np  # 导入 NumPy 库并赋予别名 'np'
 
def visualize_point_cloud(pcd_file_path):
    # 读取点云文件
    print("Loading point cloud...")
    pcd = o3d.io.read_point_cloud(pcd_file_path)
 
    # 检查点云是否为空
    if pcd.is_empty():
        print("Point cloud is empty!")
        return
 
    # 可视化点云
    print("Visualizing point cloud...")
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
 
    # 设置相机参数
    vis.get_render_option().background_color = np.asarray([0, 0, 0])  # 使用 'np' 别名
    vis.get_render_option().point_size = 5.0  # 设置点的大小
    vis.run()  # 启动可视化窗口
    vis.destroy_window()
 
if __name__ == "__main__":
    # 替换为你的点云文件路径
    ply_file_path = "/home/sunrise/Code/Localization_project/src/FAST-Calib-ROS2/output/aligned_cloud.ply"
    visualize_point_cloud(ply_file_path)