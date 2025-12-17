cmake_minimum_required(VERSION 3.16)
project(rtele VERSION 0.1.0 LANGUAGES CXX C)

# ================== 编译选项 ==================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

# 自动检测是电脑(x86)还是机器人(ARM)
option(USE_ARM64 "Build for ARM64/aarch64 platform" OFF)
if(NOT USE_ARM64)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        set(USE_ARM64 ON)
        message(STATUS "Detected ARM64 platform (Robot)")
    else()
        message(STATUS "Detected x86_64 platform (PC)")
    endif()
endif()

include(FetchContent)

# ================== 1. 查找依赖包 ==================
find_package(ament_cmake REQUIRED)

# ROS2 基础包
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)         

# 自定义消息包
find_package(hdas_msg REQUIRED)
find_package(system_manager_msg REQUIRED)
find_package(teleoperation_msg_ros2 REQUIRED)

# !!! 关键：机器人业务包 !!!
find_package(galaxea_robot_tele REQUIRED) 

# 获取 galaxea 包的头文件路径 (修复 robot_msg_generated.h 找不到的问题)
# 通常 galaxea_robot_tele_DIR 指向 lib/cmake/galaxea_robot_tele
# 我们向上找3层得到 install/include
get_filename_component(GALAXEA_INCLUDE_ROOT "${galaxea_robot_tele_DIR}/../../../include" ABSOLUTE)
message(STATUS "Galaxea Include Root: ${GALAXEA_INCLUDE_ROOT}")

# 图像处理包
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# 系统库
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenSSL REQUIRED)

# ================== 2. Abseil 库 (在线下载编译) ==================
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

# ================== 3. 配置火山引擎 RTC SDK ==================
if(USE_ARM64)
    # ARM 路径 (机器人)
    set(VOLCENGINE_RTC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/VolcEngineRTC_Linux_3.58.1.500_aarch64)
else()
    # x86 路径 (电脑)
    set(VOLCENGINE_RTC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/VolcEngineRTC_Linux_x86_64)
endif()

set(VOLCENGINE_RTC_INCLUDE_DIRS ${VOLCENGINE_RTC_DIR}/include)
set(VOLCENGINE_RTC_LIBRARY_DIRS ${VOLCENGINE_RTC_DIR}/lib)

# 查找库文件
find_library(VOLCENGINE_RTC_LIBRARY 
    NAMES VolcEngineRTC 
    PATHS ${VOLCENGINE_RTC_LIBRARY_DIRS}
    NO_DEFAULT_PATH
)

# 找到所有的 .so 文件以便安装
file(GLOB VOLCENGINE_RTC_EXT_LIBS "${VOLCENGINE_RTC_LIBRARY_DIRS}/*.so")

if(NOT VOLCENGINE_RTC_LIBRARY)
    message(FATAL_ERROR "VolcEngineRTC library not found in ${VOLCENGINE_RTC_LIBRARY_DIRS}")
else()
    message(STATUS "Found VolcEngineRTC library: ${VOLCENGINE_RTC_LIBRARY}")
endif()

# ================== 4. 编译主程序 ==================

# 源文件列表
set(SOURCE_FILES
    "src/rtc_tele_node.cc"        # 我们的新主程序
    "src/config/rtc_config.cc"    # 配置类
    "src/rtc/rtc_engine.cc"       # RTC 引擎封装
    "src/rtc/message_consumer.cc" # 消息回调
    "src/util/token_generator.cc" # Token 工具
    "src/util/video_file_reader.cc" 
)

add_executable(rtele_node ${SOURCE_FILES})

# 设置头文件搜索路径
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
    
    # 修复头文件路径 (两个都加上以防万一)
    ${GALAXEA_INCLUDE_ROOT}
    ${GALAXEA_INCLUDE_ROOT}/galaxea_robot_tele 
)

# 链接所有的库
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
    
    # 链接机器人代码里的库 (flatbuffer_utils)
    galaxea_robot_tele::flatbuffer_utils

    # 链接 Abseil
    absl::strings
    absl::flat_hash_map
    absl::time
    absl::flags
    absl::flags_parse
)

# 链接 ROS2 的库
ament_target_dependencies(rtele_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    std_srvs
    hdas_msg
    system_manager_msg
    teleoperation_msg_ros2
    galaxea_robot_tele
)

# ================== 5. 安装与打包 ==================

# 安装可执行文件
install(TARGETS rtele_node
    DESTINATION lib/${PROJECT_NAME}
)

# 安装 RTC 的动态库 (.so) 到 lib 目录
install(FILES ${VOLCENGINE_RTC_EXT_LIBS}
    DESTINATION lib/${PROJECT_NAME}
)

# 安装资源文件
install(DIRECTORY resources/
    DESTINATION share/${PROJECT_NAME}/resources
    OPTIONAL
)

ament_package()
