/*
 * ROS2 IMU数据发布节点
 * 该节点创建一个定时器，以50Hz频率发布模拟的IMU数据
 * 消息类型：sensor_msgs/msg/Imu
 * 话题名称：/imu/data
 * 坐标系：imu_link
 * 新增：串口任务执行线程
 * 新增：机器人运动控制订阅
 * 新增：关节状态订阅
 * 新增：里程计数据发布
 */

// ROS2核心库
#include <rclcpp/rclcpp.hpp>
// IMU消息类型
#include <sensor_msgs/msg/imu.hpp>
// 运动控制消息类型
#include <geometry_msgs/msg/twist.hpp>
// 关节状态消息类型
#include <sensor_msgs/msg/joint_state.hpp>
// 里程计消息类型
#include <nav_msgs/msg/odometry.hpp>
// 数学函数库（sin, cos等）
#include <cmath>
// 时间处理库
#include <chrono>
// 线程支持
#include <thread>
// 互斥锁
#include <mutex>
// 包含关节轨迹消息的头文件，用于控制机器人关节运动
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <algorithm> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <termios.h>  // 用于B1000000波特率定义
#include <limits>
#include <unistd.h>   // 新增：用于close()函数
#include <fcntl.h>    // 新增：用于文件控制操作
#include <errno.h>    // 新增：用于错误处理
#include "robot_functions.h"
#include "std_msgs/msg/int16_multi_array.hpp"

const double Default_Height = 0.25f; 
const double LEG_MIN_LOW = 0.14f;   //腿最低
const double LEG_MAX_HIGH = 0.36f;   //腿最高
const double  Pitch_limits  = 35.0f; 
const double  Roll_limits  = 15.0f; 

const float  Timeout_threshold  = 1.0f; //订阅话题未更新超时

typedef struct {
    rclcpp::Time current_time;
    rclcpp::Time  last_time;
    float dt;
  } Time_dt_t;



// 使用时间字面量（如20ms）
using namespace std::chrono_literals;

//腿轮类，继承自rclcpp::Node
class Legwheel : public rclcpp::Node
{
public:
  // 构造函数
  Legwheel()
  : Node("wl_base_node"),  // 节点名称
    cmd_updated_vel_(false),         // 控制指令更新标志（先声明）
    cmd_posture_updated_sign_(false),
    Robot_Mode_(0),
    cmd_joint_updated_sign_(false),
    stop_serial_task_(false),    // 初始化停止标志（后声明）
    serial_fd_(-1),              // 初始化串口文件描述符
    serial_selected_(false),      // 串口选择标志
    last_odom_time_(this->get_clock()->now())  // 初始化里程计时间
  {
    // 第一步：串口选择
    selectSerialPort();
    
    // 如果串口选择成功，初始化ROS2组件
    if (serial_selected_) 
    {
      initROSComponents();
    } 
    else 
    {
      RCLCPP_ERROR(this->get_logger(), "串口选择失败，节点无法正常启动");
      // 可以在这里添加适当的错误处理逻辑
    }
  }

  // 析构函数
  ~Legwheel()
  {
    // ===================== 安全停止串口线程 =====================
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      stop_serial_task_ = true;
    }
    
    if (serial_thread_.joinable()) {
      serial_thread_.join();
      RCLCPP_INFO(this->get_logger(), "Serial task thread stopped");
    }
    
    // 关闭串口
    if (serial_fd_ != -1) {
      close(serial_fd_);
      RCLCPP_INFO(this->get_logger(), "Serial port closed");
    }
  }

private:
  // 串口选择函数
  void selectSerialPort()
  {
    std::string fixed_port = "/dev/wheel_leg_base";
    RCLCPP_INFO(this->get_logger(), "强制使用固定串口: %s", fixed_port.c_str());
    
    serial_fd_ = openSerialPort(fixed_port, B2000000);
    if (serial_fd_ != -1) {
        RCLCPP_INFO(this->get_logger(), "✅ 固定串口打开成功！");
        serial_selected_ = true;
        return;  // 直接成功，不搜索、不手动选
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ 固定串口打开失败");
        // 失败才进入手动选择
    }




    
    std::vector<std::string> ports = listSerialPorts();



   

    // 2. 自动搜索失败，进入手动选择循环
    int selectedPortIndex = -1;
    char choice;
    bool quitProgram = false;
    bool restartSelection = true;

    while (!quitProgram && restartSelection) 
    {
        restartSelection = false;
        ports = listSerialPorts(); // 循环内重新扫描，方便插拔后重试

        if (ports.empty()) 
        {
            std::cout << "\n未找到可用串口!" << std::endl;
            std::cout << "按 [R] 重新扫描或 [Q] 退出: ";
            std::cin >> choice;
            choice = std::toupper(choice);
            
            if (choice == 'R') { restartSelection = true; continue; }
            else if (choice == 'Q') { quitProgram = true; break; }
        }
        
        std::cout << "\n--- 手动选择串口 ---" << std::endl;
        for (size_t i = 0; i < ports.size(); i++) {
            std::cout << "[" << i+1 << "] " << ports[i] << std::endl;
        }
        std::cout << "[R] 重新扫描" << std::endl;
        std::cout << "[Q] 退出" << std::endl;
        std::cout << "请选择串口索引: ";
        std::cin >> choice;
        choice = std::toupper(choice);
        
        if (choice == 'Q') {
            quitProgram = true;
        } 
        else if (choice == 'R') {
            restartSelection = true;
        } 
        else if (std::isdigit(choice)) {
            int index = choice - '1';
            if (index >= 0 && static_cast<size_t>(index) < ports.size()) {
                serial_fd_ = openSerialPort(ports[index], B2000000);
                if (serial_fd_ != -1) {
                    std::cout << "手动打开成功: " << ports[index] << std::endl;
                    serial_selected_ = true;
                    restartSelection = false;
                } else {
                    std::cout << "打开失败，请重试。" << std::endl;
                    restartSelection = true;
                }
            } else {
                std::cout << "无效索引!" << std::endl;
                restartSelection = true;
            }
        } 
        
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    
    if (quitProgram) {
        RCLCPP_ERROR(this->get_logger(), "用户终止了程序");
        rclcpp::shutdown();
        exit(0);
    }
  }
  
  // 初始化ROS2组件
  void initROSComponents()
  {
    // 创建发布者，话题名为/imu/data，队列大小为10
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    
    // ===================== 新增：里程计发布者 =====================
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    
    // 创建发布者，发布到"sbus_channels"话题，队列大小为10
    sbus_publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/sbus_channels", 10);    //遥控器10通道原始数据


    // 初始化关节名称
    g_joint_names = {
        "left_front_joint_link",
        "left_back_joint_link",
        "right_front_joint_link",
        "right_back_joint_link",
        "left_wheel_joint_link",
        "right_wheel_joint_link"
    };


    // 验证关节数量
    if (g_joint_names.size() != MotorID::TOTAL_COUNT) {
        RCLCPP_INFO(this->get_logger(), "关节名称数量与电机总数不匹配！");
    }
    // 创建发布者
    g_publisher = this->create_publisher<sensor_msgs::msg::JointState>("/robot_joint_states", 10);


    // ===================== 新增：TF变换广播器 =====================
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);//


    // 创建订阅者，订阅"joint_commands"话题     /cmd_robot_joint
    cmd_joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/cmd_robot_joint",10,std::bind(&Legwheel::joint_control_callback, this, std::placeholders::_1)//订阅关节控制
    );


    // ===================== 新增：运动控制订阅 =====================
    // 创建订阅者，话题名为/cmd_vel，队列大小为10
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&Legwheel::cmd_vel_callback, this, std::placeholders::_1));//控制机器人 X线速度（米每秒）  Z角速度（弧度每秒）
    
    // ===================== 新增：关节状态订阅 =====================
    // 创建订阅者，话题名为/cmd_posture，队列大小为10
    body_posture_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/cmd_posture", 10, std::bind(&Legwheel::joint_state_callback, this, std::placeholders::_1));//控制机器人俯仰（度） 横滚（度） 高度(米)
    
    // 创建定时器
    timer_ = this->create_wall_timer(
      20ms, std::bind(&Legwheel::timer_callback, this));//定时发布IMU 和里程计数据

    timer2_ = this->create_wall_timer(
      10ms,std::bind(&Legwheel::timer2_callback, this)//给下位机定时发送数据 定时发布IMU 和里程计数据
    );
      

    
    // ===================== 串口任务初始化 =====================
    // 启动串口任务线程
    serial_thread_ = std::thread(&Legwheel::serial_task, this);
    
    // 初始化关节数据
    joint_positions_ = {Default_Height, 0.0, 0.0};
    joint_velocities_ = {0.0, 0.0, 0.0};
    joint_efforts_ = {0.0, 0.0, 0.0};
    cmd_posture_updated_sign_ = false;



    cmd_joint_updated_sign_ = false;
    
    //初始里程计信息
    odom.angle_speed = 0;
    odom.linear_speed = 0;
    odom.angle = 0;  
    odom.x = 0;
    odom.y = 0;
    cmd_updated_vel_ = false;

    COD_vel_dt.current_time = this->get_clock()->now(); 
    COD_vel_dt.last_time = COD_vel_dt.current_time;

    COD_posture_dt.current_time = this->get_clock()->now();
    COD_posture_dt.last_time = COD_posture_dt.current_time;
    
    TxdMitData_init();

    // 输出启动信息
    RCLCPP_INFO(this->get_logger(), "IMU发布话题: /imu/data");
    RCLCPP_INFO(this->get_logger(), "里程计发布话题: /odom");
    RCLCPP_INFO(this->get_logger(), "遥控器发布话题: /sbus_channels");
    RCLCPP_INFO(this->get_logger(), "机器人关节状态发布话题: /robot_joint_states");


    RCLCPP_INFO(this->get_logger(), "移动底盘速度控制订阅话题：/cmd_vel");
    RCLCPP_INFO(this->get_logger(), "移动底盘姿态控制订阅话题：/cmd_posture");
    RCLCPP_INFO(this->get_logger(), "机器人关节控制订阅话题：/cmd_robot_joint");
    
  }


  void TXD_MobileChassis(void)//发送移动底盘控制数据
  {
    const size_t dataSize = sizeof(ComWheelLegged_t); 
    
    get_ComWheelLegged.header1 = 123;         // 帧头1 (123)
    get_ComWheelLegged.header2 = 45;          // 帧头2 (45)
    get_ComWheelLegged.length = dataSize;     // 结构体长度

    ComWheelLegged.header1 = 123;             // 帧头1 (123)
    ComWheelLegged.header2 = 45;              // 帧头2 (45)
    ComWheelLegged.length = dataSize;         // 结构体长度

    if((cmd_posture_updated_sign_ == true)&&(cmd_updated_vel_ == true))
    {
      ComWheelLegged = get_ComWheelLegged;
      get_ComWheelLegged.count++;
      cmd_posture_updated_sign_ = false;
      cmd_updated_vel_ = false;

    }
    else if((cmd_posture_updated_sign_ == true)&&(cmd_updated_vel_ == false))
    {

      ComWheelLegged.leg_length = get_ComWheelLegged.leg_length;
      ComWheelLegged.roll_angle = get_ComWheelLegged.roll_angle;
      ComWheelLegged.pitch_angle = get_ComWheelLegged.pitch_angle; 
      ComWheelLegged.count = get_ComWheelLegged.count; 
      get_ComWheelLegged.count++;
      cmd_posture_updated_sign_ = false;

    }
    else if((cmd_posture_updated_sign_ == false)&&(cmd_updated_vel_ == true))
    {
      ComWheelLegged.linear_velocity = get_ComWheelLegged.linear_velocity;
      ComWheelLegged.angle_velocity = get_ComWheelLegged.angle_velocity;
      ComWheelLegged.count = get_ComWheelLegged.count; 
      get_ComWheelLegged.count++;
      cmd_updated_vel_ = false;
    }

    // 计算实际时间间隔
    COD_vel_dt.current_time = this->get_clock()->now();
    COD_vel_dt.dt = (COD_vel_dt.current_time - COD_vel_dt.last_time).seconds();
    

    COD_posture_dt.current_time = this->get_clock()->now();
    COD_posture_dt.dt = (COD_posture_dt.current_time - COD_posture_dt.last_time).seconds();

      
    if((COD_vel_dt.dt>Timeout_threshold)&&(Timeout_threshold!=0)) 
    {
      ComWheelLegged.linear_velocity = 0;
      ComWheelLegged.angle_velocity = 0;
      //std::cout << " 速度更新超时\n";
    }
  
    
    if((COD_posture_dt.dt>Timeout_threshold)&&(Timeout_threshold!=0)) 
    { 
      ComWheelLegged.leg_length = Default_Height;
      ComWheelLegged.roll_angle = 0;
      ComWheelLegged.pitch_angle = 0;
      //std::cout << " 姿态更新超时\n";
    }
    
    
    if(get_ComWheelLegged.count==255)
      get_ComWheelLegged.count = 0; 




    uint8_t dataArray[dataSize];  // 用于存储转换后的字节数据

    size_t copied = TXD_structToArr(ComWheelLegged, dataArray, dataSize);//将结构体数据复制到字节数组
    if (copied > 0) 
    {
      ComWheelLegged.crc = crc16(dataArray, dataSize-2);
      
      copied = TXD_structToArr(ComWheelLegged, dataArray, dataSize);//将结构体数据复制到字节数组

      if (copied > 0) 
      {
        ssize_t bytesWritten = write(serial_fd_, dataArray, dataSize);

        // 检查写入结果
        if (bytesWritten < 0) {
            // 处理写入错误
            perror("写入串口失败");
        } else if (bytesWritten < dataSize) {
            // 只写入了部分数据，可能需要重试
            printf("警告：只写入了 %zd/%zu 字节\n", bytesWritten, dataSize);
        } else {
            // 成功写入所有数据
            //printf("成功写入 %zd 字节\n", bytesWritten);
/*
            std::cout << std::fixed << std::setprecision(5);
            std::cout << "joint:\n";
            std::cout << "  leg_length: " << ComWheelLegged.leg_length << " m\n";
            std::cout << "  roll_angle: " << ComWheelLegged.roll_angle << " rad\n";
            std::cout << "  pitch_angle: " << ComWheelLegged.pitch_angle << " rad\n";
        
*/

        } 

      }
      else
      {
        perror("结构体转数组失败！");

      }
    } 
    else 
    {
      perror("结构体转数组失败！");

    }

  }

  
  void TXD_JointData(void)//发送移动底盘控制数据
  {
    const size_t dataSize = sizeof(TX_MIT_Data_t); 

    TX_MIT_Data.header1 = 123;             // 帧头1 (123)
    TX_MIT_Data.header2 = 45;              // 帧头2 (45)
    TX_MIT_Data.length = dataSize;         // 结构体长度

    TX_MIT_Data.count++;
    
    if(TX_MIT_Data.count==255)
      TX_MIT_Data.count = 0; 

    uint8_t dataArray[dataSize];  // 用于存储转换后的字节数据

    size_t copied = TXD_joint_structToArr(TX_MIT_Data, dataArray, dataSize);//将结构体数据复制到字节数组
    if (copied > 0) 
    {
      TX_MIT_Data.crc = crc16(dataArray, dataSize-2);
      
      copied = TXD_joint_structToArr(TX_MIT_Data, dataArray, dataSize);//将结构体数据复制到字节数组

      if (copied > 0) 
      {
        ssize_t bytesWritten = write(serial_fd_, dataArray, dataSize);

        // 检查写入结果
        if (bytesWritten < 0) {
            // 处理写入错误
            perror("写入串口失败");
        } else if (bytesWritten < dataSize) {
            // 只写入了部分数据，可能需要重试
            printf("警告：只写入了 %zd/%zu 字节\n", bytesWritten, dataSize);
        } else {
            // 成功写入所有数据
            //printf("成功写入 %zd 字节\n", bytesWritten);
        } 

      }
      else
      {
        perror("结构体转数组失败！");

      }
    } 
    else 
    {
      perror("结构体转数组失败！");

    }
    

  }




  void publish_IMU_Data(void)
  {
    // 创建IMU消息对象
    auto imu_msg = sensor_msgs::msg::Imu();

    //printMilemeterData(ROS_body.milemeter);
    //printImuData(ROS_body.ImuData);

    // ===================== 设置消息头 =====================
    imu_msg.header.stamp = this->now();    // 当前时间戳
    imu_msg.header.frame_id = "imu_link";  // 坐标系名称
    
    // ===================== 方向数据（四元数） =====================
    // 模拟随时间旋转的角度（每帧增加0.05弧度）

    // 设置四元数（绕Z轴旋转）
    // 注意：实际应用中应从真实传感器获取这些数据
    imu_msg.orientation.x = ROS_body.ImuData.q0;
    imu_msg.orientation.y = ROS_body.ImuData.q1;
    imu_msg.orientation.z = ROS_body.ImuData.q2;  // Z分量
    imu_msg.orientation.w = ROS_body.ImuData.q3;  // W分量
    
    // ===================== 角速度数据 =====================
    // 模拟角速度值（单位：弧度/秒）
    imu_msg.angular_velocity.x = ROS_body.ImuData.gyro[0];  // X轴角速度
    imu_msg.angular_velocity.y = ROS_body.ImuData.gyro[1];  // Y轴角速度
    imu_msg.angular_velocity.z = ROS_body.ImuData.gyro[2];              // Z轴角速度（恒定）
    
    
    // ===================== 线加速度数据 =====================
    // 模拟加速度值（单位：米/秒²）
    // 注意：实际IMU测量值包含重力加速度
    imu_msg.linear_acceleration.x = ROS_body.ImuData.accel[0];   // X轴加速度
    imu_msg.linear_acceleration.y = ROS_body.ImuData.accel[1];   // Y轴加速度
    imu_msg.linear_acceleration.z = ROS_body.ImuData.accel[2];  // Z轴加速度（重力）
    
  
    // 发布IMU消息
    imu_publisher_->publish(imu_msg);


  }

  // 定时器回调函数，每次触发时发布IMU数据
  void timer_callback()
  {
    if(Serial_update_flag==1)
    {
      //printMilemeterData(ROS_body.milemeter);
      publish_odom();     // 发布里程计信息
      publish_IMU_Data();
      publish_sbus_data();
      publish_JointState();
      Serial_update_flag = 0;
    }
   
  }


  void timer2_callback()
  {
    if(Robot_Mode_==0)
    {
      TXD_MobileChassis();
      //printMITFeedbackData(ROS_body);
    }
    else
    {
      if(cmd_joint_updated_sign_==true)
      {
        TXD_JointData();  
        //printMitTxdData(TX_MIT_Data);
        cmd_joint_updated_sign_ = false;
      }

    }
  }



  // 回调函数处理接收到的关节控制消息（
  void joint_control_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
  {
      // 检查消息数据的一致性（保留原有逻辑）
      if (msg->name.size() != msg->position.size() || 
          msg->name.size() != msg->velocity.size() ||
          msg->name.size() != msg->effort.size())
      {
          RCLCPP_WARN(this->get_logger(), "不匹配的关节状态数据长度");
          return;
      }
      else
      {

        if(cmd_joint_updated_sign_ == false)
        {
          for (size_t i = 0; i < msg->name.size(); ++i)
          {
              const std::string& joint_name = msg->name[i];  // 关节名称
              TX_MIT_Data.MIT_Command_Data[i].position = radians_to_degrees(msg->position[i]);            // 关节位置（度）
              TX_MIT_Data.MIT_Command_Data[i].velocity = radians_to_degrees(msg->velocity[i]);            // 关节速度（度/秒）
              TX_MIT_Data.MIT_Command_Data[i].effort = msg->effort[i];                                    // 关节力/力矩（牛·米）
  
          }

          cmd_joint_updated_sign_ = true;

        }

      }

  }





  
  // ===================== 新增：运动控制回调函数 =====================
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 获取互斥锁保护控制指令数据
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    // 存储接收到的控制指令
    current_cmd_ = *msg;
    COD_vel_dt.last_time = COD_vel_dt.current_time; 

    if(cmd_updated_vel_ == false)
    {
      get_ComWheelLegged.linear_velocity = msg->linear.x;
      get_ComWheelLegged.angle_velocity = msg->angular.z;
      cmd_updated_vel_ = true;
    }
/*
    // 打印接收到的控制指令
    RCLCPP_INFO(this->get_logger(), 
                "Received control command: "
                "Linear: x=%.2f, y=%.2f, z=%.2f | "
                "Angular: x=%.2f, y=%.2f, z=%.2f",
                msg->linear.x, msg->linear.y, msg->linear.z,
                msg->angular.x, msg->angular.y, msg->angular.z);
*/
        
        
  }
  
  // ===================== 新增：关节状态回调函数 =====================
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // 获取互斥锁保护关节状态数据
    std::lock_guard<std::mutex> lock(joint_mutex_);
    
    // 假设消息包含至少3个关节的数据
    if (msg->name.size() >= 3 && msg->position.size() >= 3) 
    {
      // 查找三个关节的索引
      int joint_height_idx = -1, joint_roll_idx = -1, joint_pitching_idx = -1;
      
      for (size_t i = 0; i < msg->name.size(); i++) 
      {
        if (msg->name[i] == "joint_height") joint_height_idx = i;
        if (msg->name[i] == "joint_roll") joint_roll_idx = i;
        if (msg->name[i] == "joint_pitching") joint_pitching_idx = i;
      }
      
      // 更新关节数据
      if (joint_height_idx != -1 && joint_roll_idx != -1 && joint_pitching_idx != -1) 
      {
        joint_positions_[0] = msg->position[joint_height_idx];
        joint_positions_[1] = msg->position[joint_roll_idx];
        joint_positions_[2] = msg->position[joint_pitching_idx];
        COD_posture_dt.last_time = COD_posture_dt.current_time; 
        
        if(cmd_posture_updated_sign_ == false)
        {
          get_ComWheelLegged.leg_length = std::clamp(joint_positions_[0], LEG_MIN_LOW, LEG_MAX_HIGH);
          get_ComWheelLegged.roll_angle = std::clamp(joint_positions_[1], -Roll_limits, Roll_limits);
          get_ComWheelLegged.pitch_angle = std::clamp(joint_positions_[2], -Pitch_limits, Pitch_limits); 
          cmd_posture_updated_sign_ = true;
        }

        // 如果有速度和力数据，也更新它们
        if (msg->velocity.size() >= 3) 
        {
          joint_velocities_[0] = msg->velocity[joint_height_idx];
          joint_velocities_[1] = msg->velocity[joint_roll_idx];
          joint_velocities_[2] = msg->velocity[joint_pitching_idx];
        }
        
        if (msg->effort.size() >= 3) 
        {
          joint_efforts_[0] = msg->effort[joint_height_idx];
          joint_efforts_[1] = msg->effort[joint_roll_idx];
          joint_efforts_[2] = msg->effort[joint_pitching_idx];
        }
      }
    } else if (msg->position.size() >= 3) {
      // 如果没有关节名称但有至少3个位置数据，假设前三个数据对应三个关节
      joint_positions_[0] = msg->position[0];
      joint_positions_[1] = msg->position[1];
      joint_positions_[2] = msg->position[2];
      
      if (msg->velocity.size() >= 3) {
        joint_velocities_[0] = msg->velocity[0];
        joint_velocities_[1] = msg->velocity[1];
        joint_velocities_[2] = msg->velocity[2];
      }
      
      if (msg->effort.size() >= 3) {
        joint_efforts_[0] = msg->effort[0];
        joint_efforts_[1] = msg->effort[1];
        joint_efforts_[2] = msg->effort[2];
      }
    }
  }
  
  // ===================== 串口任务函数 =====================
  void serial_task()
  {
    // 主通信循环：持续接收并解析串口数据
    while (rclcpp::ok()) 
    {
        // 检查停止标志
        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            if (stop_serial_task_) break;
        }
        
        
        // 读取并解析串口数据
        if (serial_fd_ != -1) 
        {
            try {
                SERIAL_RXT(serial_fd_);  // 读取并解析串口数据
                if(Serial_update_flag == 1)
                {
                  // 计算实际时间间隔
                  rclcpp::Time current_time = this->get_clock()->now();
                  double dt = (current_time - last_odom_time_).seconds();
                  last_odom_time_ = current_time;
                  
                  update_odom(dt);    // 更新里程计数据
                  //printMilemeterData(ROS_body.milemeter);
                  //printImuData(ROS_body.ImuData);

                  if(ROS_body.Status.status.model==0)
                  {
                    Robot_Mode_ = 0;
                  }
                  else 
                  {
                    Robot_Mode_ = 1;
                  }
                  
                }
            } 
            catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Serial communication error: %s", e.what());
                std::this_thread::sleep_for(1s);  // 异常后延时1秒
            }
        } else {
            // 如果没有串口，等待一段时间再重试
            std::this_thread::sleep_for(1s);
        }
        
        // 短暂睡眠以避免CPU占用过高
        std::this_thread::sleep_for(10ms);
    }

    // 关闭串口
    if (serial_fd_ != -1) {
        close(serial_fd_);
        serial_fd_ = -1;
        RCLCPP_INFO(this->get_logger(), "串口已关闭");
    }
  }


  void publish_sbus_data()
  {
    auto msg = std_msgs::msg::Int16MultiArray();
    
    // 确保数组大小为10
    msg.data.resize(10);
    
    // 将ros_body中的SBUS通道数据复制到消息中
    for (int i = 0; i < 10; ++i) {
        msg.data[i] = ROS_body.SBUS_Channels_Data[i];
    }
    
    // 发布消息
    sbus_publisher_->publish(msg);
    
    // 打印调试信息（仅在DEBUG级别可见）
    // 打印全部10通道数据，便于完整调试
    //RCLCPP_INFO(this->get_logger(), 
    //            "Publishing SBUS data: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
    //            msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],
    //            msg.data[5], msg.data[6], msg.data[7], msg.data[8], msg.data[9]);
  }




  // ===================== 发布里程计数据函数 =====================
  void publish_odom()
  {
    // 检查里程计数据是否有显著变化
    static double last_x = 0, last_y = 0, last_angle = 0;
    const double pos_threshold = 0.001;  // 1mm
    const double angle_threshold = 0.001; // 约0.057度
    
    double dx = std::abs(odom.x - last_x);
    double dy = std::abs(odom.y - last_y);
    double dangle = std::abs(odom.angle - last_angle);
    
    // 如果变化太小，跳过发布
    //if (dx < pos_threshold && dy < pos_threshold && dangle < angle_threshold) {
    //  return;
    //}
    
    // 更新上次值
    last_x = odom.x;
    last_y = odom.y;
    last_angle = odom.angle;
    
    // 创建里程计消息
    auto odom_msg = nav_msgs::msg::Odometry();
    
    // 设置消息头
    odom_msg.header.stamp = this->now(); // 或者使用硬件时间戳
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    // 设置位置
    odom_msg.pose.pose.position.x = odom.x;
    odom_msg.pose.pose.position.y = odom.y;
    odom_msg.pose.pose.position.z = 0.0;
    
    // 将欧拉角转换为四元数
    tf2::Quaternion quat;
    quat.setRPY(0, 0, odom.angle);
    odom_msg.pose.pose.orientation.x = quat.x();
    odom_msg.pose.pose.orientation.y = quat.y();
    odom_msg.pose.pose.orientation.z = quat.z();
    odom_msg.pose.pose.orientation.w = quat.w();
    
    // 设置速度
    odom_msg.twist.twist.linear.x = (double)odom.linear_speed;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.z = (double)odom.angle_speed;
    
    // 优化后的协方差设置
    odom_msg.pose.covariance[0] = 0.05;   // x 方差
    odom_msg.pose.covariance[7] = 0.05;   // y 方差
    odom_msg.pose.covariance[14] = 0.1;   // z 方差
    odom_msg.pose.covariance[21] = 0.1;   // roll 方差
    odom_msg.pose.covariance[28] = 0.1;   // pitch 方差
    odom_msg.pose.covariance[35] = 0.03;  // yaw 方差

    // 速度协方差
    odom_msg.twist.covariance[0] = 0.1;   // vx 方差
    odom_msg.twist.covariance[7] = 0.1;   // vy 方差
    odom_msg.twist.covariance[14] = 0.1;  // vz 方差
    odom_msg.twist.covariance[21] = 0.1;  // vroll 方差
    odom_msg.twist.covariance[28] = 0.1;  // vpitch 方差
    odom_msg.twist.covariance[35] = 0.05; // vyaw 方差
    
    // 发布里程计消息
    odom_publisher_->publish(odom_msg);

    //RCLCPP_INFO(this->get_logger(), "odom_msg sx: %.2f, px: %.2f, px1: %.2f, angle: %.2f", 
    //            odom_msg.twist.twist.linear.x, odom_msg.pose.pose.position.x,  odom.x, odom.angle);

    

    // 发布TF变换
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odom_msg.header.stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    // 位置信息（平移）
    transform.transform.translation.x = odom.x;
    transform.transform.translation.y = odom.y;
    transform.transform.translation.z = 0.0;
    
    // 姿态信息（旋转）   
    transform.transform.rotation = odom_msg.pose.pose.orientation;
    
    // 发送变换
    // tf_broadcaster_->sendTransform(transform);
  }

  // Publish joint status
  void publish_JointState()
  {
      auto msg = sensor_msgs::msg::JointState();      
      // 设置消息头
      msg.header.stamp = this->now(); // 或者使用硬件时间戳
      msg.header.frame_id = "base_link";

      // 关节名称
      msg.name = g_joint_names;

      // 位置数据（弧度）
      msg.position.resize(MotorID::TOTAL_COUNT);
      for (int i = 0; i < MotorID::TOTAL_COUNT; ++i)
      {
        msg.position[i] = degrees_to_radians(ROS_body.MIT_Feedback_Data[i].position);
      }
      // 速度数据（弧度/秒）
      msg.velocity.resize(MotorID::TOTAL_COUNT);
      for (int i = 0; i < MotorID::TOTAL_COUNT; ++i)
      {
        msg.velocity[i] = degrees_to_radians(ROS_body.MIT_Feedback_Data[i].velocity);
      }
      // 扭矩数据（N·m）
      msg.effort.resize(MotorID::TOTAL_COUNT);
      for (int i = 0; i < MotorID::TOTAL_COUNT; ++i)
      {
        msg.effort[i] = ROS_body.MIT_Feedback_Data[i].torque;
      }
      // 发布消息
      g_publisher->publish(msg);
  }

  


    
    // 修改update_odom函数，使用double类型的时间间隔
  void update_odom(double dt)
  {
    odom.angle_speed = ROS_body.milemeter.AngularVelocity;
    odom.linear_speed = ROS_body.milemeter.Speed;

    // 角速度积分
    odom.angle += odom.angle_speed * dt;  // 
    TransAngleInPI(odom.angle, odom.angle);

    // 小车前进的距离
    double dx = odom.linear_speed * dt;

    odom.x += dx * cos(odom.angle);
    odom.y += dx * sin(odom.angle);
    
    /*
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "里程计数据:\n";
    std::cout << "  左轮速度: " << ROS_body.milemeter.LeftWheelSpeed << " m/s\n";
    std::cout << "  右轮速度: " << ROS_body.milemeter.RightWheelSpeed << " m/s\n";
    std::cout << "  前进速度: " << odom.linear_speed << " m/s\n";
    std::cout << "  角速度: " << odom.angle_speed << " rad/s\n";
    std::cout << "  angle: " << odom.angle << " rad\n";
    std::cout << "  odom.x: " << odom.x << " m\n";
    std::cout << "  odom.y: " << odom.y << " m\n";
    std::cout << "  dt: " << dt << " s";  
    */
  }


  void TransAngleInPI(float angle,float& out_angle)
  {
    if(angle > M_PI)
    {
      out_angle -= 2*M_PI;
    }
    else if(angle < -M_PI)
    {
      out_angle += 2*M_PI;
    }
      
  }

  
  // ===================== 成员变量 =====================
  rclcpp::TimerBase::SharedPtr timer_;        // 定时器
  rclcpp::TimerBase::SharedPtr timer2_;        // 定时器
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr sbus_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;  // IMU发布者
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;  // 里程计发布者


  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr g_publisher;
  std::vector<std::string> g_joint_names;


  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // TF变换广播器
  
 
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_joint_subscriber_;
  bool cmd_joint_updated_sign_;                       // 指令更新标志（先声明）

  // ===== 新增：运动控制相关成员 =====
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; // 运动控制订阅
  geometry_msgs::msg::Twist current_cmd_;      // 当前控制指令
  std::mutex cmd_mutex_;                       // 控制指令互斥锁
  bool cmd_updated_vel_;                       // 指令更新标志（先声明）
  
  // ===== 新增：关节状态相关成员 =====
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr body_posture_sub_; //身体身体姿式订阅
  std::vector<double> joint_positions_;        // 关节位置
  std::vector<double> joint_velocities_;       // 关节速度
  std::vector<double> joint_efforts_;          // 关节力/力矩
  std::mutex joint_mutex_;                     // 关节数据互斥锁
  bool cmd_posture_updated_sign_;                       // 指令更新标志（先声明）
  
  // ===== 串口任务相关成员 =====
  std::thread serial_thread_;          // 串口任务线程
  std::mutex serial_mutex_;            // 保护串口资源的互斥锁
  bool stop_serial_task_;              // 线程停止标志（后声明）
  int serial_fd_;                      // 串口文件描述符
  bool serial_selected_;               // 串口选择成功标志
  
  odom_t odom;    //里程计信息
  rclcpp::Time last_odom_time_;  // 存储上一次里程计更新时间

  Time_dt_t COD_vel_dt;
  Time_dt_t COD_posture_dt;

  ComWheelLegged_t ComWheelLegged;
  ComWheelLegged_t get_ComWheelLegged;

  int Robot_Mode_;//0:控制移动底盘  1：控制电机

};

// 主函数
int main(int argc, char * argv[])
{

  // 初始化ROS2
  rclcpp::init(argc, argv);
  
  // 创建节点并开始循环
  auto node = std::make_shared<Legwheel>();
  
  // 只有当串口选择成功后才开始spin
  rclcpp::spin(node);
  
  // 关闭ROS2
  rclcpp::shutdown();
  return 0;
}