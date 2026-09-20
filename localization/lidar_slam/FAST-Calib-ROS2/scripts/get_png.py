import os
import cv2
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from cv_bridge import CvBridge
import argparse

def main():
    parser = argparse.ArgumentParser(description='从静态 bag 中提取稳态图像')
    parser.add_argument('bag_path', help='db3 文件路径')
    parser.add_argument('topic_name', help='话题名称')
    parser.add_argument('output_dir', help='保存目录')
    parser.add_argument('--skip', type=int, default=10, help='跳过开头的帧数以确保稳态')
    
    args = parser.parse_args()

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    reader = rosbag2_py.SequentialReader()
    # 强制指定为 sqlite3，因为你确认了是 .db3 文件
    storage_options = rosbag2_py.StorageOptions(uri=args.bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )
    
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    msg_type = get_message(type_map[args.topic_name])
    bridge = CvBridge()
    
    count = 0
    extracted = False

    print(f"正在寻找稳态图像...")

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == args.topic_name:
            count += 1
            # 跳过前 N 帧，取第 N+1 帧作为稳态图
            if count <= args.skip:
                continue
            
            msg = deserialize_message(data, msg_type)
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 保存为 single_steady_frame.png
            file_path = os.path.join(args.output_dir, "steady_frame.png")
            cv2.imwrite(file_path, cv_img)
            print(f"成功提取稳态图像（第 {count} 帧）: {file_path}")
            extracted = True
            break # 提取完一张直接退出

    if not extracted:
        print("未能提取到图像，请检查话题名称是否正确或 bag 是否过短。")

if __name__ == '__main__':
    main()