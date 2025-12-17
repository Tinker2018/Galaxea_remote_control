cmake_minimum_required(VERSION 3.16)
project(rtele VERSION 0.1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 忽略 SDK 的未使用参数警告
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# ================== 0. 架构与 SDK 路径判定 ==================
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(USE_ARM64 ON)
    message(STATUS "[RTele] Mode: Robot (ARM64 / Orin)")
    
    # 指向 3.58.1.500 版本 SDK
    set(RTC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/VolcEngineRTC_Linux_3.58.1.500_aarch64)
    
    # 定义宏，用于代码中处理 API 差异
    add_definitions(-DVOLC_RTC_ARM64) 
else()
    set(USE_ARM64 OFF)
    message(STATUS "[RTele] Mode: PC (x86_64)")
    
    # 指向 x86 版本 SDK
    set(RTC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/VolcEngineRTC_Linux_x86_64)
endif()

# ================== 1. 解决 Orin OpenCV 冲突 (关键!) ==================
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

if(USE_ARM64)
    # [Orin 独占逻辑]
    # 强制不搜索系统路径 (/usr/include/opencv4)，只在 ROS 路径下找
    # 这样能避免链接到 JetPack 的 OpenCV 导致 cv_bridge 崩溃
    set(OpenCV_DIR "/opt/ros/${ROS_DISTRO}/share/opencv4")
    find_package(OpenCV REQUIRED PATHS "/opt/ros/${ROS_DISTRO}/share/opencv4" NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
    message(STATUS "[RTele] Orin: Forced ROS OpenCV at ${OpenCV_DIR}")
else()
    # x86 PC 正常查找
    find_package(OpenCV REQUIRED)
endif()

# ================== 2. 其他 ROS 依赖 ==================
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)         
find_package(hdas_msg REQUIRED)
find_package(system_manager_msg REQUIRED)
find_package(teleoperation_msg_ros2 REQUIRED)
find_package(galaxea_robot_tele REQUIRED) 

# 获取 galaxea 头文件路径
get_filename_component(GALAXEA_INSTALL_PREFIX "${galaxea_robot_tele_DIR}/../../.." ABSOLUTE)
set(GALAXEA_INCLUDE_DIR "${GALAXEA_INSTALL_PREFIX}/include")
set(GALAXEA_INNER_INCLUDE_DIR "${GALAXEA_INSTALL_PREFIX}/include/galaxea_robot_tele")

# ================== 3. 系统库 ==================
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(OpenSSL REQUIRED)

include(FetchContent)
find_package(absl QUIET)
if(NOT absl_FOUND)
    FetchContent_Declare(
        absl
        GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
        GIT_TAG        20240116.2
        GIT_SHALLOW    TRUE
    )
    set(ABSL_PROPAGATE_CXX_STD ON)
    set(ABSL_BUILD_TESTING OFF)
    FetchContent_MakeAvailable(absl)
endif()

# ================== 4. 配置 RTC 库 ==================
set(RTC_INCLUDE_DIRS ${RTC_ROOT}/include)
set(RTC_LIB_FILE ${RTC_ROOT}/lib/libVolcEngineRTC.so)

if(NOT EXISTS ${RTC_LIB_FILE})
    message(FATAL_ERROR "RTC Lib not found: ${RTC_LIB_FILE}")
endif()

# ================== 5. 编译 ==================
add_executable(rtele_node 
    "src/main.cc"
    "src/config/rtc_config.cc"
    "src/rtc/rtc_engine.cc"
    "src/rtc/message_consumer.cc"
    "src/util/token_generator.cc"
    "src/util/video_file_reader.cc" 
)

target_include_directories(rtele_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    ${RTC_INCLUDE_DIRS}
    ${GALAXEA_INCLUDE_DIR}
    ${GALAXEA_INNER_INCLUDE_DIR}
)

target_link_libraries(rtele_node
    ${OpenCV_LIBS}
    ${RTC_LIB_FILE}
    cv_bridge::cv_bridge
    image_transport::image_transport
    galaxea_robot_tele::flatbuffer_utils
    spdlog::spdlog
    OpenSSL::SSL OpenSSL::Crypto
    absl::strings absl::flags absl::flags_parse
)

ament_target_dependencies(rtele_node
    rclcpp
    std_msgs sensor_msgs geometry_msgs std_srvs
    hdas_msg system_manager_msg
)

# ================== 6. 安装 ==================
install(TARGETS rtele_node DESTINATION lib/${PROJECT_NAME})

# 安装 RTC 动态库
file(GLOB RTC_SO_FILES "${RTC_ROOT}/lib/*.so")
install(FILES ${RTC_SO_FILES} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY resources/ DESTINATION share/${PROJECT_NAME}/resources OPTIONAL)

ament_package()