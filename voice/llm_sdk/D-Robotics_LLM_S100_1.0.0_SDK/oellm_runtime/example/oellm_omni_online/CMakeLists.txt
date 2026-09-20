cmake_minimum_required(VERSION 3.15)

project(omni_online_demo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Werror -Wl,-unresolved-symbols=ignore-in-shared-libs")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-function -Wno-unused-parameter -Wno-unused-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

set(FFMPEG_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/ffmpeg/include")
set(FFMPEG_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/ffmpeg/lib")

link_directories(${FFMPEG_LIB_DIR})
include_directories(${FFMPEG_INCLUDE_DIR})

set(FFMPEG_AARCH64_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/ffmpeg")
set(CMAKE_FIND_ROOT_PATH ${FFMPEG_AARCH64_PREFIX})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(CMAKE_BUILD_TYPE ${build_type})

# debug
set(CMAKE_CXX_FLAGS_DEBUG " -g -O0 -ggdb -pipe -fPIC ")
set(CMAKE_C_FLAGS_DEBUG " -g -O0 -ggdb -pipe -fPIC ")
# release
set(CMAKE_CXX_FLAGS_RELEASE " -O3 -pipe -fPIC ")
set(CMAKE_C_FLAGS_RELEASE " -O3 -pipe -fPIC ")

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${FFMPEG_AARCH64_PREFIX}/include)

#link_directories(${FFMPEG_AARCH64_PREFIX}/lib)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../lib)

set(LINK_LIBS xlm avformat avcodec avutil
    swscale swresample)

add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/omni_online_demo.cc)

target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
