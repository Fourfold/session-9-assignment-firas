cmake_minimum_required(VERSION 3.8)
project(turtlebot_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(navigator_node src/navigator.cpp)
ament_target_dependencies(navigator_node rclcpp geometry_msgs sensor_msgs custom_interfaces)

add_executable(find_closest_wall_server src/closest_wall_server.cpp)
ament_target_dependencies(find_closest_wall_server rclcpp geometry_msgs sensor_msgs custom_interfaces)

add_executable(measure_lap_time_server src/lap_time_server.cpp)
ament_target_dependencies(measure_lap_time_server rclcpp custom_interfaces rclcpp_action nav_msgs)

add_executable(measure_lap_time_client src/lap_time_client.cpp)
ament_target_dependencies(measure_lap_time_client rclcpp custom_interfaces rclcpp_action)

install(TARGETS
  navigator_node
  find_closest_wall_server
  measure_lap_time_server
  measure_lap_time_client
  DESTINATION lib/${PROJECT_NAME})

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
