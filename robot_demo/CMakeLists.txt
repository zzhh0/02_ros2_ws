cmake_minimum_required(VERSION 3.8)
project(robot_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(my_interface REQUIRED)


add_executable(cmd_publisher src/cmd_pub.cpp)
ament_target_dependencies(cmd_publisher rclcpp std_msgs geometry_msgs )

add_executable(robot_controller src/robot_controller.cpp)
ament_target_dependencies(robot_controller rclcpp std_msgs geometry_msgs std_srvs my_interface)

add_executable(pose_monitor src/pose_monitor.cpp)
ament_target_dependencies(pose_monitor rclcpp std_msgs geometry_msgs my_interface)


#安装目标文件
install(TARGETS
  cmd_publisher
  robot_controller
  pose_monitor
  DESTINATION lib/${PROJECT_NAME}
)
# 安装启动文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
