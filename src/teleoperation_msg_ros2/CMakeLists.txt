cmake_minimum_required(VERSION 3.8)
project(system_manager_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 声明 srv 文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TeleopFrame.srv"
  DEPENDENCIES  # 无额外依赖，留空
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
