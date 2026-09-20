cmake_minimum_required(VERSION 3.16)
project(sherpa_onnx_kws_cpp LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- sherpa-onnx SDK ---
# 默认使用项目内自带的 SDK 目录
set(SHERPA_ONNX_SDK_PATH "" CACHE PATH "Path to sherpa-onnx SDK directory (default: auto-detect in project)")

# 自动检测：优先使用用户指定路径，其次使用项目内 SDK
if(SHERPA_ONNX_SDK_PATH)
    message(STATUS "Using user-specified SDK: ${SHERPA_ONNX_SDK_PATH}")
elseif(EXISTS "${CMAKE_SOURCE_DIR}/sherpa-onnx-v1.13.3-linux-aarch64-shared-cpu/lib")
    set(SHERPA_ONNX_SDK_PATH "${CMAKE_SOURCE_DIR}/sherpa-onnx-v1.13.3-linux-aarch64-shared-cpu")
    message(STATUS "Using project-local SDK: ${SHERPA_ONNX_SDK_PATH}")
else()
    message(FATAL_ERROR
        "sherpa-onnx SDK not found!\n"
        "Please ensure the SDK directory exists at:\n"
        "  ${CMAKE_SOURCE_DIR}/sherpa-onnx-v1.13.3-linux-aarch64-shared-cpu/\n"
        "Or specify manually: cmake -DSHERPA_ONNX_SDK_PATH=/path/to/sdk ..")
endif()

set(SHERPA_ONNX_LIB_DIR "${SHERPA_ONNX_SDK_PATH}/lib")
set(SHERPA_ONNX_INCLUDE_DIR "${SHERPA_ONNX_SDK_PATH}/include")

# 验证 SDK 完整性
if(NOT EXISTS "${SHERPA_ONNX_INCLUDE_DIR}/sherpa-onnx/c-api/c-api.h")
    message(FATAL_ERROR "SDK invalid: c-api.h not found in ${SHERPA_ONNX_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${SHERPA_ONNX_LIB_DIR}/libsherpa-onnx-c-api.so")
    message(FATAL_ERROR "SDK invalid: libsherpa-onnx-c-api.so not found in ${SHERPA_ONNX_LIB_DIR}")
endif()

message(STATUS "SDK include: ${SHERPA_ONNX_INCLUDE_DIR}")
message(STATUS "SDK lib:     ${SHERPA_ONNX_LIB_DIR}")

# --- PortAudio ---
find_library(PORTAUDIO_LIBRARY NAMES portaudio)
find_path(PORTAUDIO_INCLUDE_DIR NAMES portaudio.h)
if(NOT PORTAUDIO_LIBRARY OR NOT PORTAUDIO_INCLUDE_DIR)
    message(FATAL_ERROR "PortAudio not found. Install with: sudo apt install portaudio19-dev")
endif()
message(STATUS "PortAudio: ${PORTAUDIO_LIBRARY}")

# --- 主程序 ---
add_executable(kws_vad_demo
    src/main.cc
    src/audio_capture.cc
    src/wav_writer.cc
)

target_include_directories(kws_vad_demo PRIVATE
    ${SHERPA_ONNX_INCLUDE_DIR}
    ${PORTAUDIO_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(kws_vad_demo PRIVATE
    ${SHERPA_ONNX_LIB_DIR}/libsherpa-onnx-c-api.so
    ${SHERPA_ONNX_LIB_DIR}/libonnxruntime.so
    ${PORTAUDIO_LIBRARY}
)

target_link_directories(kws_vad_demo PRIVATE ${SHERPA_ONNX_LIB_DIR})

set_target_properties(kws_vad_demo PROPERTIES
    BUILD_RPATH "${SHERPA_ONNX_LIB_DIR}"
    INSTALL_RPATH "$ORIGIN/../lib"
)

# 安装目标
install(TARGETS kws_vad_demo DESTINATION bin)
install(DIRECTORY ${SHERPA_ONNX_LIB_DIR}/ DESTINATION lib
        FILES_MATCHING PATTERN "libsherpa-onnx*" PATTERN "libonnxruntime*")
