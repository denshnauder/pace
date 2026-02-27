cmake_minimum_required(VERSION 3.8)
project(pace_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# 添加可执行文件
add_executable(cmd_vel_to_serial_node src/cmd_vel_to_serial_node.cpp)

# 包含头文件目录
include_directories(
  include
)

# 链接依赖
ament_target_dependencies(cmd_vel_to_serial_node 
  rclcpp
  geometry_msgs
)

# 安装可执行文件
install(TARGET
S
  cmd_vel_to_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件（如果有）
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()cmake_minimum_required(VERSION 3.8)
project(pace_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# 编译节点可执行文件
add_executable(cmd_vel_to_serial_node src/cmd_vel_to_serial_node.cpp) # 根据你的实际相对路径调整

# 链接依赖
ament_target_dependencies(cmd_vel_to_serial_node
  rclcpp
  geometry_msgs
)

# 安装规则：将可执行文件安装到 ROS 2 的 lib 目录下
install(TARGETS cmd_vel_to_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

# 如果你后续添加了 launch 文件或 config 参数文件，还需要加上：
# install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()