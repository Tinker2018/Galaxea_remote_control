cmake_minimum_required(VERSION 3.8)
project(galaxea_robot_tele)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)         # 已添加
find_package(system_manager_msg REQUIRED) # 已添加
find_package(ament_index_cpp REQUIRED)
find_package(flatbuffers REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(hdas_msg REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${flatbuffers_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# 1. FlatBuffers
set(FLATBUFFERS_SCHEMA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/schemas/robot_msg.fbs)
set(FLATBUFFERS_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(FLATBUFFERS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

file(MAKE_DIRECTORY ${FLATBUFFERS_GENERATED_DIR})
file(MAKE_DIRECTORY ${FLATBUFFERS_INCLUDE_DIR})

add_custom_command(
  OUTPUT 
    ${FLATBUFFERS_GENERATED_DIR}/robot_msg_generated.h
    ${FLATBUFFERS_INCLUDE_DIR}/robot_msg_generated.h
  COMMAND 
    flatc --cpp -o ${FLATBUFFERS_GENERATED_DIR} ${FLATBUFFERS_SCHEMA_FILE}
  COMMAND 
    ${CMAKE_COMMAND} -E copy 
    ${FLATBUFFERS_GENERATED_DIR}/robot_msg_generated.h 
    ${FLATBUFFERS_INCLUDE_DIR}/robot_msg_generated.h
  DEPENDS ${FLATBUFFERS_SCHEMA_FILE}
  COMMENT "Compiling FlatBuffer schema"
)

add_custom_target(generate_flatbuffers
  DEPENDS 
    ${FLATBUFFERS_GENERATED_DIR}/robot_msg_generated.h
    ${FLATBUFFERS_INCLUDE_DIR}/robot_msg_generated.h
)

# 2. UDP Socket Lib
add_library(udp_socket SHARED src/udp_socket.cpp include/udp_socket.hpp)
ament_target_dependencies(udp_socket rclcpp ament_index_cpp yaml-cpp)
target_link_libraries(udp_socket ${YAML_CPP_LIBRARIES})
add_dependencies(udp_socket generate_flatbuffers)
target_include_directories(udp_socket PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
)

# 3. Flatbuffer Utils Lib
add_library(flatbuffer_utils SHARED src/flatbuffer_utils.cpp include/flatbuffer_utils.hpp)
ament_target_dependencies(flatbuffer_utils
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs              # 已添加
  system_manager_msg    # 已添加
  flatbuffers
  hdas_msg
)
target_link_libraries(flatbuffer_utils udp_socket ${flatbuffers_LIBRARIES})
add_dependencies(flatbuffer_utils generate_flatbuffers)
target_include_directories(flatbuffer_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
)

# 4. Robot Node
add_executable(robot_tele_node src/robot_tele_node.cpp include/robot_tele_node.hpp)
ament_target_dependencies(robot_tele_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs              # 已添加
  system_manager_msg    # 已添加
  ament_index_cpp
  hdas_msg
)
target_link_libraries(robot_tele_node udp_socket flatbuffer_utils ${YAML_CPP_LIBRARIES})
add_dependencies(robot_tele_node generate_flatbuffers)

# 5. PC Node
add_executable(pc_tele_node src/pc_tele_node.cpp include/pc_tele_node.hpp)
ament_target_dependencies(pc_tele_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs              # 已添加
  system_manager_msg    # 已添加
  hdas_msg
  ament_index_cpp
)
target_link_libraries(pc_tele_node udp_socket flatbuffer_utils ${YAML_CPP_LIBRARIES})
add_dependencies(pc_tele_node generate_flatbuffers)

# Install
install(TARGETS udp_socket flatbuffer_utils EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS robot_tele_node pc_tele_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${FLATBUFFERS_GENERATED_DIR}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch schemas DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(udp_socket flatbuffer_utils)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(hdas_msg std_srvs system_manager_msg) # 已添加导出

ament_package()