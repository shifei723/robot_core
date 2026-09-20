cmake_minimum_required(VERSION 3.5)
project(rviz2_zone_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)             # 导航依赖
find_package(zone_interfaces REQUIRED)      # 导航依赖接口文件


# 使用 qt5_wrap_cpp 宏生成MOC_FILES，用于处理Qt元对象编译
qt5_wrap_cpp(MOC_FILES
        include/rviz2_zone_plugin/zone_panel.hpp
        include/rviz2_nav_plugin/nav_panel.hpp
)

# 设置源代码文件列表
set(SRC_FILES
        src/nav_panel.cpp
        src/zone_panel.cpp
)

# Include directories
include_directories(include)

# Create library
add_library(${PROJECT_NAME} SHARED ${MOC_FILES} ${SRC_FILES})

# 链接所需的库
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  yaml-cpp
  ament_index_cpp::ament_index_cpp
)

# Target dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_rendering
  geometry_msgs
  yaml-cpp
  visualization_msgs
  std_msgs             
  zone_interfaces      # 导航接口
)

# Export the plugin
ament_export_dependencies(rclcpp)
ament_export_dependencies(rviz_common)
ament_export_dependencies(zone_interfaces)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include
  DESTINATION include
)

# 使用宏 pluginlib_export_plugin_description_file 导出插件描述文件
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()