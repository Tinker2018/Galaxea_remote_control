cmake_minimum_required(VERSION 3.8)
project(udp_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖（新增 ament_index_cpp）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)   # 关节消息依赖
find_package(geometry_msgs REQUIRED) # 位姿消息依赖
find_package(ament_index_cpp REQUIRED) # 新增：获取包路径依赖
find_program(FLATC flatc REQUIRED)
if(NOT FLATC)
  message(FATAL_ERROR "未找到flatc编译器，请安装：sudo apt install flatbuffers-compiler")
endif()

# FlatBuffers配置（保留你的原有逻辑）
set(FBS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/msg/udp_messages.fbs)
set(FLATC_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# 自动生成FB头文件（保留你的原有逻辑）
add_custom_command(
  OUTPUT ${FLATC_OUTPUT_DIR}/udp_messages_generated.h
  COMMAND ${FLATC} --cpp -o ${FLATC_OUTPUT_DIR} ${FBS_FILE}
  DEPENDS ${FBS_FILE}
  COMMENT "生成FlatBuffers头文件：${FBS_FILE}"
)

# 自定义目标（保留你的原有逻辑）
add_custom_target(generate_flatbuffers ALL
  DEPENDS ${FLATC_OUTPUT_DIR}/udp_messages_generated.h
)

# 包含头文件目录（保留你的原有逻辑）
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${YAML_CPP_INCLUDE_DIR}
)

# 编译UDP工具库（新增 ament_index_cpp 依赖）
add_library(udp_socket SHARED
  src/udp_socket.hpp
  src/udp_socket.cpp
)
ament_target_dependencies(udp_socket 
  rclcpp 
  sensor_msgs   # 关节消息
  geometry_msgs # 位姿消息
  ament_index_cpp # 新增：包路径依赖
)
target_link_libraries(udp_socket ${YAML_CPP_LIBRARIES})

# 编译节点（新增 ament_index_cpp 依赖）
add_executable(udp_comm_node
  src/udp_node.cpp
  ${FLATC_OUTPUT_DIR}/udp_messages_generated.h
)
add_dependencies(udp_comm_node generate_flatbuffers)
ament_target_dependencies(udp_comm_node 
  rclcpp 
  sensor_msgs   # 关节消息
  geometry_msgs # 位姿消息
  ament_index_cpp # 新增：包路径依赖
)
target_link_libraries(udp_comm_node 
  udp_socket 
  ${YAML_CPP_LIBRARIES}
)

# 安装（保留你的原有逻辑）
install(TARGETS
  udp_comm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  udp_socket
  LIBRARY DESTINATION lib
)

# 安装配置文件（yaml）
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
