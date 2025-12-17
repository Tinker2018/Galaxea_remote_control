cmake_minimum_required(VERSION 3.16)
project(rtele VERSION 0.1.0 LANGUAGES CXX C)

# ================== 编译选项 ==================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 添加 -Wno-unused-parameter 以屏蔽 SDK 的大量警告
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# 自动检测是电脑(x86)还是机器人(ARM)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(USE_ARM64 ON)
    message(STATUS "[RTele] Detected ARM64 platform (Robot)")
else()
    set(USE_ARM64 OFF)
    message(STATUS "[RTele] Detected x86_64 platform (PC)")
endif()

include(FetchContent)

# ================== 1. 查找依赖包 ==================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)         
find_package(hdas_msg REQUIRED)
find_package(system_manager_msg REQUIRED)
find_package(teleoperation_msg_ros2 REQUIRED)

# !!! 关键依赖 !!!
find_package(galaxea_robot_tele REQUIRED) 

# ----------------- 核心修复开始 -----------------
# 自动推导 galaxea 的 include 路径
# galaxea_robot_tele_DIR 通常在 install/galaxea_robot_tele/share/galaxea_robot_tele/cmake
# 我们需要指向 install/galaxea_robot_tele/include
get_filename_component(GALAXEA_INSTALL_PREFIX "${galaxea_robot_tele_DIR}/../../.." ABSOLUTE)
set(GALAXEA_INCLUDE_DIR "${GALAXEA_INSTALL_PREFIX}/include")
set(GALAXEA_INNER_INCLUDE_DIR "${GALAXEA_INSTALL_PREFIX}/include/galaxea_robot_tele")

message(STATUS "[RTele] Galaxea Include Dir: ${GALAXEA_INCLUDE_DIR}")
message(STATUS "[RTele] Galaxea Inner Dir:   ${GALAXEA_INNER_INCLUDE_DIR}")

# 检查路径是否存在，辅助调试
if(NOT EXISTS "${GALAXEA_INNER_INCLUDE_DIR}/robot_msg_generated.h")
    message(WARNING "[RTele] !!! Cannot find robot_msg_generated.h in expected path !!!")
    message(WARNING "Please check if 'galaxea_robot_tele' is built and sourced correctly.")
endif()
# ----------------- 核心修复结束 -----------------

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(OpenSSL REQUIRED)

# ================== 2. Abseil 库 ==================
find_package(absl QUIET)
if(NOT absl_FOUND)
    message(STATUS "[RTele] Abseil not found, fetching...")
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

# ================== 3. 配置火山引擎 RTC SDK ==================
if(USE_ARM64)
    set(VOLCENGINE_RTC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/VolcEngineRTC_Linux_3.58.1.500_aarch64)
else()
    set(VOLCENGINE_RTC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/VolcEngineRTC_Linux_x86_64)
endif()

set(VOLCENGINE_RTC_INCLUDE_DIRS ${VOLCENGINE_RTC_DIR}/include)
set(VOLCENGINE_RTC_LIB ${VOLCENGINE_RTC_DIR}/lib/libVolcEngineRTC.so)

if(NOT EXISTS ${VOLCENGINE_RTC_LIB})
    message(FATAL_ERROR "VolcEngineRTC library not found at: ${VOLCENGINE_RTC_LIB}")
endif()

# ================== 4. 编译主程序 ==================
# 注意：这里使用的是 main.cc，与你的文件树一致
set(SOURCE_FILES
    "src/main.cc"
    "src/config/rtc_config.cc"
    "src/rtc/rtc_engine.cc"
    "src/rtc/message_consumer.cc"
    "src/util/token_generator.cc"
    "src/util/video_file_reader.cc" 
)

add_executable(rtele_node ${SOURCE_FILES})

# 设置头文件路径
target_include_directories(rtele_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${VOLCENGINE_RTC_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mpmc
    
    # 显式添加 galaxea 的两个层级路径，解决嵌套引用问题
    ${GALAXEA_INCLUDE_DIR}
    ${GALAXEA_INNER_INCLUDE_DIR}
)

# 链接库
target_link_libraries(rtele_node
    ${OpenCV_LIBS}
    ${VOLCENGINE_RTC_LIB}
    Threads::Threads
    spdlog::spdlog
    OpenSSL::SSL
    OpenSSL::Crypto
    cv_bridge::cv_bridge
    image_transport::image_transport
    
    # 链接机器人包的 FlatBuffer 库
    galaxea_robot_tele::flatbuffer_utils

    absl::strings absl::flags absl::flags_parse
)

ament_target_dependencies(rtele_node
    rclcpp
    std_msgs sensor_msgs geometry_msgs std_srvs
    hdas_msg system_manager_msg teleoperation_msg_ros2
    galaxea_robot_tele
)

# ================== 5. 安装与打包 ==================
install(TARGETS rtele_node DESTINATION lib/${PROJECT_NAME})

# 安装 RTC 依赖库
file(GLOB VOLCENGINE_RTC_LIBS "${VOLCENGINE_RTC_DIR}/lib/*.so")
install(FILES ${VOLCENGINE_RTC_LIBS} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY resources/ DESTINATION share/${PROJECT_NAME}/resources OPTIONAL)

ament_package()