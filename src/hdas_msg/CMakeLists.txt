cmake_minimum_required(VERSION 3.8)
project(hdas_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 声明消息文件（路径：~/tinker_ws/src/hdas_msg/msg/MotorControl.msg）
set(msg_files
  "msg/MotorControl.msg"
)

# 生成消息接口
rosidl_generate_interfaces(\${PROJECT_NAME}
  \${msg_files}
  DEPENDENCIES std_msgs
)

# 导出运行时依赖
ament_export_dependencies(rosidl_default_runtime)

ament_package()
