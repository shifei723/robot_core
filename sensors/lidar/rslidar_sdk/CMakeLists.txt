cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(rslidar_sdk)

#=======================================
# Custom Point Type (XYZI,XYZIRT, XYZIF, XYZIRTF)
#=======================================
set(POINT_TYPE XYZI)

option(ENABLE_TRANSFORM "Enable transform functions" OFF)
if(${ENABLE_TRANSFORM})
  add_definitions("-DENABLE_TRANSFORM")
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif(${ENABLE_TRANSFORM})

option(ENABLE_EPOLL_RECEIVE "Receive packets with epoll() instead of select()" OFF)
if(${ENABLE_EPOLL_RECEIVE})
  add_definitions("-DENABLE_EPOLL_RECEIVE")
endif(${ENABLE_EPOLL_RECEIVE})

option(ENABLE_MODIFY_RECVBUF "Enable modify size of RECVBUF" ON)
if(${ENABLE_MODIFY_RECVBUF})
  add_definitions("-DENABLE_MODIFY_RECVBUF")
endif(${ENABLE_MODIFY_RECVBUF})

option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)
if(${ENABLE_WAIT_IF_QUEUE_EMPTY})
  add_definitions("-DENABLE_WAIT_IF_QUEUE_EMPTY")
endif(${ENABLE_WAIT_IF_QUEUE_EMPTY})

option(ENABLE_DIFOP_PARSE "Enable parsing DIFOP Packet" ON)
if(${ENABLE_DIFOP_PARSE})
  add_definitions("-DENABLE_DIFOP_PARSE")
endif(${ENABLE_DIFOP_PARSE})

option(ENABLE_STAMP_WITH_LOCAL "Enable stamp point cloud with local time" OFF)
if(${ENABLE_STAMP_WITH_LOCAL})
  add_definitions("-DENABLE_STAMP_WITH_LOCAL")
endif(${ENABLE_STAMP_WITH_LOCAL})

option(ENABLE_SOURCE_PACKET_LEGACY "Enable ROS Source of MSOP/DIFOP Packet v1.3.x" OFF)
if(${ENABLE_SOURCE_PACKET_LEGACY})
  add_definitions("-DENABLE_SOURCE_PACKET_LEGACY")
endif(${ENABLE_SOURCE_PACKET_LEGACY})

option(ENABLE_IMU_DATA_PARSE           "Enable imu data parse" OFF)
if(${ENABLE_IMU_DATA_PARSE})
  message(=============================================================)
  message("-- Enable imu data parse")
  message(=============================================================)
  add_definitions("-DENABLE_IMU_DATA_PARSE")
endif(${ENABLE_IMU_DATA_PARSE})

#========================
# Project details / setup
#========================
set(PROJECT_NAME rslidar_sdk)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-O3)
endif()

add_definitions(-std=c++17)
add_compile_options(-Wall)

#========================
# Point Type Definition
#========================
if(${POINT_TYPE} STREQUAL "XYZI")
  add_definitions(-DPOINT_TYPE_XYZI)
elseif(${POINT_TYPE} STREQUAL "XYZIRT")
  add_definitions(-DPOINT_TYPE_XYZIRT)
elseif(${POINT_TYPE} STREQUAL "XYZIF")
  add_definitions(-DPOINT_TYPE_XYZIF)
elseif(${POINT_TYPE} STREQUAL "XYZIRTF")
  add_definitions(-DPOINT_TYPE_XYZIRTF)
endif()

message(=============================================================)
message("-- POINT_TYPE is ${POINT_TYPE}")
message(=============================================================)

#========================
# Dependencies Setup
#========================

# ========== 屏蔽 ROS1 全部代码 ==========
# ROS1 分支注释掉，避免误进入 catkin
# find_package(roscpp 1.12 QUIET)
# if(roscpp_FOUND)
#   ... 此处全部注释，不再使用 ROS1
# endif(roscpp_FOUND)

# ---------------- ROS2 单独启用 ----------------
find_package(rclcpp REQUIRED)
message(=============================================================)
message("-- ROS2 Found. ROS2 Support is turned On.")
message(=============================================================)

add_definitions(-DROS2_FOUND)
include_directories(${rclcpp_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rslidar_msg REQUIRED)
find_package(std_msgs REQUIRED)

# 其他依赖
find_package(yaml-cpp REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

# 驱动核心
add_subdirectory(src/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})

#========================
# Build Setup
#========================
add_executable(rslidar_sdk_node
               node/rslidar_sdk_node.cpp
               src/manager/node_manager.cpp)

target_link_libraries(rslidar_sdk_node
                      ${YAML_CPP_LIBRARIES}
                      ${rs_driver_LIBRARIES})

# ROS2 依赖与安装
ament_target_dependencies(rslidar_sdk_node
  rclcpp
  std_msgs
  sensor_msgs
  rslidar_msg)

install(TARGETS
  rslidar_sdk_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()