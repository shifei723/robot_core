cmake_minimum_required(VERSION 3.8)
project(open3d_loc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_BUILD_TYPE "Release")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(urdf REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

# 仓库根：优先 ROBOT_CORE 环境变量，其次按源码位置推导（不再硬编码某台机器的路径）
if(DEFINED ENV{ROBOT_CORE} AND NOT "$ENV{ROBOT_CORE}" STREQUAL "")
  set(_RC_ROOT "$ENV{ROBOT_CORE}")
else()
  get_filename_component(_RC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
endif()

# Open3D 路径：优先 -DOpen3D_DIR=... / 环境变量，其次仓库内 third/open3d141
if(NOT Open3D_DIR AND DEFINED ENV{OPEN3D_DIR})
  set(Open3D_DIR "$ENV{OPEN3D_DIR}")
endif()
if(NOT Open3D_DIR)
  set(Open3D_DIR "${_RC_ROOT}/third/open3d141/lib/cmake/Open3D")
endif()
find_package(Open3D REQUIRED)

# LAPACKE：Open3D 的 CMake 配置里写死了 /usr/lib/<arch>/liblapacke.so。
# 若系统未装 liblapacke-dev（例如没有 sudo 的环境），改用仓库自带的
# third/lapacke/lib/liblapacke.so，保证免 root 也能链接通过。
set(_LAPACKE_FALLBACK "")
if(NOT EXISTS "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/liblapacke.so")
  if(EXISTS "${_RC_ROOT}/third/lapacke/lib/liblapacke.so")
    set(_LAPACKE_FALLBACK "${_RC_ROOT}/third/lapacke/lib/liblapacke.so")
    message(STATUS "lapacke: 系统库缺失，使用仓库自带 ${_LAPACKE_FALLBACK}")
  endif()
endif()

# Fix Open3D libraries - remove problematic .so files
# In Ubuntu 22.04+, some libraries (librt, libc++) have different paths or are merged
# Open3D's CMake config files may have hardcoded paths that don't exist

# After finding Open3D, patch ALL its imported targets
# We need to patch multiple targets that may have these dependencies
foreach(target Open3D::Open3D Open3D::3rdparty_filament Open3D::3rdparty_glfw Open3D::3rdparty_rply Open3D::3rdparty_glew)
        if(TARGET ${target})
                get_target_property(LINK_LIBS ${target} INTERFACE_LINK_LIBRARIES)

                if(LINK_LIBS)
                        # Create a new list with problematic libraries removed
                        set(FILTERED_LIBS "")

                        foreach(lib ${LINK_LIBS})
                                # 系统缺 liblapacke 时替换成仓库自带副本
                                if(_LAPACKE_FALLBACK AND "${lib}" MATCHES "liblapacke\\.so")
                                        list(APPEND FILTERED_LIBS "${_LAPACKE_FALLBACK}")
                                # Skip problematic libraries
                                elseif(NOT "${lib}" MATCHES ".*librt\\.so.*" AND
                                        NOT "${lib}" MATCHES ".*libc\\+\\+\\.so.*" AND
                                        NOT "${lib}" MATCHES ".*libc\\+\\+abi\\.so.*")
                                        list(APPEND FILTERED_LIBS ${lib})
                                endif()
                        endforeach()

                        # Set the filtered list back
                        set_target_properties(${target} PROPERTIES INTERFACE_LINK_LIBRARIES "${FILTERED_LIBS}")
                endif()
        endif()
endforeach()

# Also filter the Open3D_LIBRARIES variable
if(Open3D_LIBRARIES)
        set(FILTERED_O3D_LIBS "")

        foreach(lib ${Open3D_LIBRARIES})
                if(_LAPACKE_FALLBACK AND "${lib}" MATCHES "liblapacke\\.so")
                        list(APPEND FILTERED_O3D_LIBS "${_LAPACKE_FALLBACK}")
                elseif(NOT "${lib}" MATCHES ".*librt\\.so.*" AND
                        NOT "${lib}" MATCHES ".*libc\\+\\+\\.so.*" AND
                        NOT "${lib}" MATCHES ".*libc\\+\\+abi\\.so.*")
                        list(APPEND FILTERED_O3D_LIBS ${lib})
                endif()
        endforeach()

        set(Open3D_LIBRARIES ${FILTERED_O3D_LIBS})
endif()

include_directories(
        include
        ${Open3D_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${YAMLCPP_INCLUDE_DIRS}
)

# Build open3d_registration library
add_library(open3d_registration src/open3d_registration/open3d_registration.cpp)
target_link_libraries(open3d_registration ${Open3D_LIBRARIES})
ament_target_dependencies(open3d_registration
        rclcpp
        sensor_msgs
        geometry_msgs
)

# Build open3d_conversions library
add_library(open3d_conversions src/open3d_conversions/open3d_conversions.cpp)
target_link_libraries(open3d_conversions ${Open3D_LIBRARIES})
ament_target_dependencies(open3d_conversions
        rclcpp
        sensor_msgs
        geometry_msgs
        std_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        tf2_eigen
        urdf
)

# Build global_localization_node
add_executable(global_localization_node src/global_localization.cpp)
# 使用仓库自带 lapacke 时，把它的目录写进 RPATH，保证运行时能加载
if(_LAPACKE_FALLBACK)
  get_filename_component(_LAPACKE_DIR "${_LAPACKE_FALLBACK}" DIRECTORY)
  set_target_properties(global_localization_node PROPERTIES
    BUILD_RPATH "${_LAPACKE_DIR}"
    INSTALL_RPATH "${_LAPACKE_DIR}"
  )
endif()
target_link_libraries(global_localization_node
        ${Open3D_LIBRARIES}
        open3d_registration
        open3d_conversions
)
ament_target_dependencies(global_localization_node
        rclcpp
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        tf2_eigen
        pcl_ros
        pcl_conversions
)

# Install targets
install(TARGETS
        open3d_registration
        open3d_conversions
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS
        global_localization_node

        DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
        launch
        config
        rviz_cfg
        DESTINATION share/${PROJECT_NAME}/
)

# Install include directory
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()