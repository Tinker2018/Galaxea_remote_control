cmake_minimum_required(VERSION 3.16)
project(rtele VERSION 0.1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

# 平台检测
option(USE_ARM64 "Build for ARM64/aarch64 platform" OFF)
if(NOT USE_ARM64)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        set(USE_ARM64 ON)
        message(STATUS "Detected ARM64 platform")
    else()
        message(STATUS "Detected x86_64 platform")
    endif()
endif()

include(FetchContent)

# ================= 1. 查找依赖 =================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hdas_msg REQUIRED)
find_package(system_manager_msg REQUIRED)
find_package(teleoperation_msg_ros2 REQUIRED)

# !!! 新增：图像转换依赖 !!!
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# 系统库
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenSSL REQUIRED)

# ================= 2. Abseil (源码编译) =================
message(STATUS "Fetching Abseil C++...")
FetchContent_Declare(
    absl
    GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
    GIT_TAG        20240116.2
    GIT_SHALLOW    TRUE
)
set(ABSL_PROPAGATE_CXX_STD ON)
set(ABSL_BUILD_TESTING OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
FetchContent_MakeAvailable(absl)

# ================= 3. VolcEngineRTC SDK =================
if(USE_ARM64)
    set(VOLCENGINE_RTC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/VolcEngineRTC_Linux_3.58.1.500_aarch64)
else()
    set(VOLCENGINE_RTC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/VolcEngineRTC_Linux_x86_64)
endif()

set(VOLCENGINE_RTC_INCLUDE_DIRS ${VOLCENGINE_RTC_DIR}/include)
set(VOLCENGINE_RTC_LIBRARY_DIRS ${VOLCENGINE_RTC_DIR}/lib)

find_library(VOLCENGINE_RTC_LIBRARY 
    NAMES VolcEngineRTC bytertc_sdk rtc_sdk
    PATHS ${VOLCENGINE_RTC_LIBRARY_DIRS}
    NO_DEFAULT_PATH
)

if(NOT VOLCENGINE_RTC_LIBRARY)
    message(FATAL_ERROR "VolcEngineRTC library not found in ${VOLCENGINE_RTC_LIBRARY_DIRS}")
else()
    message(STATUS "Found VolcEngineRTC library: ${VOLCENGINE_RTC_LIBRARY}")
endif()

# ================= 4. 构建节点 =================

file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.cc")

add_executable(rtele_node ${SOURCES})

# 包含路径
target_include_directories(rtele_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${VOLCENGINE_RTC_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mpmc
    ${absl_SOURCE_DIR}
    ${OPENSSL_INCLUDE_DIR}
    ${cv_bridge_INCLUDE_DIRS}
    ${image_transport_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(rtele_node
    ${OpenCV_LIBS}
    ${VOLCENGINE_RTC_LIBRARY}
    Threads::Threads
    spdlog::spdlog
    fmt::fmt
    OpenSSL::SSL
    OpenSSL::Crypto
    cv_bridge::cv_bridge
    image_transport::image_transport
    
    # Abseil
    absl::strings
    absl::flat_hash_map
    absl::time
    absl::flags
    absl::flags_parse
)

# ROS2 依赖链接
ament_target_dependencies(rtele_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    hdas_msg
    system_manager_msg
    teleoperation_msg_ros2
)

# ================= 5. 安装规则 =================
install(TARGETS rtele_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resources/
    DESTINATION share/${PROJECT_NAME}/resources
    PATTERN "*.yuv"
    PATTERN "*.pcm"
    PATTERN "*.jpg"
)

ament_package()