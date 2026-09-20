cmake_minimum_required(VERSION 3.10)
project(tts_cpp C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# sherpa-onnx SDK 路径
if(DEFINED ENV{SHERPA_ONNX_SDK})
    set(SDK "$ENV{SHERPA_ONNX_SDK}")
elseif(EXISTS "${CMAKE_SOURCE_DIR}/sherpa-onnx-sdk")
    set(SDK "${CMAKE_SOURCE_DIR}/sherpa-onnx-sdk")
else()
    message(FATAL_ERROR "未找到 sherpa-onnx SDK\n"
        "请运行: bash build.sh (自动下载)\n"
        "或设置: export SHERPA_ONNX_SDK=/path/to/sdk")
endif()

message(STATUS "SDK: ${SDK}")
include_directories(${SDK}/include)
link_directories(${SDK}/lib)

# 项目根目录 (用于 third_party/)
include_directories(${CMAKE_SOURCE_DIR})

# ── tts (简单版，保存到文件) ──
add_executable(tts tts.cpp)
target_link_libraries(tts sherpa-onnx-c-api onnxruntime pthread dl)

# ── tts_stream (流水线版，ZMQ + 实时播放) ──
find_library(ZMQ_LIB NAMES zmq HINTS /opt/homebrew/lib /usr/local/lib)
find_path(ZMQ_INC NAMES zmq.h HINTS /opt/homebrew/include /usr/local/include)
if(NOT ZMQ_LIB OR NOT ZMQ_INC)
    message(FATAL_ERROR "未找到 ZeroMQ\n"
        "  macOS: brew install zeromq\n"
        "  Linux: sudo apt install libzmq3-dev")
endif()
message(STATUS "ZeroMQ: ${ZMQ_LIB}")

add_executable(tts_stream tts_stream.cpp)
target_include_directories(tts_stream PRIVATE ${ZMQ_INC})
target_link_libraries(tts_stream
    sherpa-onnx-c-api onnxruntime
    ${ZMQ_LIB}
    pthread)

# macOS 需要额外链接音频框架
if(APPLE)
    target_link_libraries(tts_stream
        "-framework CoreFoundation"
        "-framework CoreAudio"
        "-framework AudioToolbox")
endif()
