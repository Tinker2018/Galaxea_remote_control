cmake_minimum_required(VERSION 3.8)
project(galaxea_robot_tele)

# 设置 C++ 标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(flatbuffers REQUIRED)
find_package(yaml-cpp REQUIRED)
# ========== 核心修改1：添加 hdas_msg 依赖查找 ==========
find_package(hdas_msg REQUIRED)

# 包含头文件目录（一级 include + 生成的 FlatBuffer 头文件目录）
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}  # 新增：包含include下的FlatBuffer头文件
  ${flatbuffers_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# -------------------------- 1. 编译 FlatBuffer 协议文件 --------------------------
set(FLATBUFFERS_SCHEMA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/schemas/robot_msg.fbs)
set(FLATBUFFERS_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)  # 构建目录（编译用）
set(FLATBUFFERS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})  # 源码include目录

# 创建输出目录（确保目录存在）
file(MAKE_DIRECTORY ${FLATBUFFERS_GENERATED_DIR})
file(MAKE_DIRECTORY ${FLATBUFFERS_INCLUDE_DIR})  # 创建include下的专属目录

# 生成 FlatBuffer 头文件（双目录输出：构建目录 + include目录）
add_custom_command(
  OUTPUT 
    ${FLATBUFFERS_GENERATED_DIR}/robot_msg_generated.h  # 构建目录（原有编译逻辑）
    ${FLATBUFFERS_INCLUDE_DIR}/robot_msg_generated.h     # include目录（新增）
  COMMAND 
    # 第一步：在构建目录生成.h文件
    flatc --cpp -o ${FLATBUFFERS_GENERATED_DIR} ${FLATBUFFERS_SCHEMA_FILE}
  COMMAND 
    # 第二步：复制生成的.h文件到include目录
    ${CMAKE_COMMAND} -E copy 
    ${FLATBUFFERS_GENERATED_DIR}/robot_msg_generated.h 
    ${FLATBUFFERS_INCLUDE_DIR}/robot_msg_generated.h
  DEPENDS ${FLATBUFFERS_SCHEMA_FILE}  # 依赖.fbs文件，修改后自动重新生成
  COMMENT "Compiling FlatBuffer schema (build + include directory)"
)

# 生成 FlatBuffer 依赖目标（依赖两个目录的.h文件）
add_custom_target(generate_flatbuffers
  DEPENDS 
    ${FLATBUFFERS_GENERATED_DIR}/robot_msg_generated.h
    ${FLATBUFFERS_INCLUDE_DIR}/robot_msg_generated.h
)

# -------------------------- 2. 编译 UDP Socket 工具库 --------------------------
add_library(udp_socket SHARED
  src/udp_socket.cpp
  include/udp_socket.hpp  # 一级 include 路径
)
ament_target_dependencies(udp_socket
  rclcpp
  ament_index_cpp
  yaml-cpp
)
target_link_libraries(udp_socket
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(udp_socket generate_flatbuffers)
target_include_directories(udp_socket PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>  # 新增：包含include目录
  $<INSTALL_INTERFACE:include>
)

# -------------------------- 3. 编译 FlatBuffer 编解码工具库 --------------------------
add_library(flatbuffer_utils SHARED
  src/flatbuffer_utils.cpp
  include/flatbuffer_utils.hpp  # 一级 include 路径
)
ament_target_dependencies(flatbuffer_utils
  rclcpp
  sensor_msgs
  geometry_msgs
  flatbuffers
  # ========== 核心修改2：flatbuffer_utils 依赖 hdas_msg ==========
  hdas_msg
)
target_link_libraries(flatbuffer_utils
  udp_socket
  ${flatbuffers_LIBRARIES}
)
add_dependencies(flatbuffer_utils generate_flatbuffers)
target_include_directories(flatbuffer_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>  # 新增：包含include目录
  $<INSTALL_INTERFACE:include>
)

# -------------------------- 4. 编译机器人端节点 --------------------------
add_executable(robot_tele_node
  src/robot_tele_node.cpp
  include/robot_tele_node.hpp  # 一级 include 路径
)
ament_target_dependencies(robot_tele_node
  rclcpp
  sensor_msgs
  geometry_msgs
  ament_index_cpp
  # ========== 核心修改3：robot_tele_node 依赖 hdas_msg（若用到 MotorControl） ==========
  hdas_msg
)
target_link_libraries(robot_tele_node
  udp_socket
  flatbuffer_utils
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(robot_tele_node generate_flatbuffers)

# -------------------------- 5. 编译 PC 端节点 --------------------------
add_executable(pc_tele_node
  src/pc_tele_node.cpp
  include/pc_tele_node.hpp  # 一级 include 路径
)
ament_target_dependencies(pc_tele_node
  rclcpp
  sensor_msgs
  geometry_msgs
  hdas_msg
  ament_index_cpp
)
target_link_libraries(pc_tele_node
  udp_socket
  flatbuffer_utils
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(pc_tele_node generate_flatbuffers)

# -------------------------- 安装目标 --------------------------
# 安装库文件
install(TARGETS
  udp_socket
  flatbuffer_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装可执行文件
install(TARGETS
  robot_tele_node
  pc_tele_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（一级 include + FlatBuffer 生成的头文件）
install(DIRECTORY include/
  DESTINATION include
)

# 安装 FlatBuffer 生成的头文件（冗余保障，实际已通过include/目录安装）
install(DIRECTORY ${FLATBUFFERS_GENERATED_DIR}/
  DESTINATION include/${PROJECT_NAME}
)

# 安装配置文件和启动文件
install(DIRECTORY config launch schemas
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------- ament 配置 --------------------------
ament_export_include_directories(include)
ament_export_libraries(
  udp_socket
  flatbuffer_utils
)
ament_export_targets(export_${PROJECT_NAME})
# ========== 核心修改4：导出 hdas_msg 依赖 ==========
ament_export_dependencies(hdas_msg)

ament_package()
