cmake_minimum_required(VERSION 3.8)
project(slash_description)

find_package(ament_cmake REQUIRED)

foreach(directory meshes urdf)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${directory}")
    install(DIRECTORY ${directory} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()