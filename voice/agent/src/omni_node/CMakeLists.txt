cmake_minimum_required(VERSION 3.8)
project(omni_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 基础依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 地平线 LLM SDK 路径
set(OELLM_SDK_LIB "/data/sf_code/agent_pipeline/D-Robotics_LLM_S100_1.0.0_SDK/oellm_runtime/lib")
set(OELLM_SDK_INC "/data/sf_code/agent_pipeline/D-Robotics_LLM_S100_1.0.0_SDK/oellm_runtime/include")

add_executable(${PROJECT_NAME}
  src/omni_node.cpp
)

# 头文件目录
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${OELLM_SDK_INC}
)

# 库搜索路径：指向SDK库目录
target_link_directories(${PROJECT_NAME} PRIVATE
  ${OELLM_SDK_LIB}
)

# ========== 核心修改：显式链接直接依赖的库 ==========
# 修复 sws_scale 符号缺失、DSO missing 报错
target_link_libraries(${PROJECT_NAME}
  xlm
  swscale   # 图像格式转换/缩放：sws_scale / sws_getContext
  avutil    # FFmpeg 基础工具：av_image_fill_arrays / av_image_get_buffer_size
)
# =====================================================

# RPATH 保证编译/运行时找到SDK内所有so
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${OELLM_SDK_LIB}:$ORIGIN/.."
  INSTALL_RPATH "${OELLM_SDK_LIB}:$ORIGIN/.."
)
target_link_options(${PROJECT_NAME} PRIVATE
  -Wl,--disable-new-dtags
)

# ROS ament 依赖绑定
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
)

# 安装规则（完全保留你原有配置）
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mock_omni_inputs.py
  scripts/transformVL.py
  scripts/camera_pub.py
  scripts/tts_bridge.py
  scripts/watch_omni.py
  scripts/tool_agent.py
  # 以下为联调工具，不参与正式链路运行
  scripts/fake_nav_server.py
  scripts/probe_toolcall.py
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()