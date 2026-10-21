cmake_minimum_required(VERSION 3.8)
project(fastlio_px4_bridge)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(fastlio_to_px4_visual_odometry src/fastlio_to_px4_visual_odometry.cpp)

ament_target_dependencies(fastlio_to_px4_visual_odometry rclcpp nav_msgs px4_msgs)
add_executable(px4_imu_to_ros src/px4_imu_to_ros.cpp)
ament_target_dependencies(px4_imu_to_ros rclcpp sensor_msgs px4_msgs)

install(
  TARGETS fastlio_to_px4_visual_odometry px4_imu_to_ros
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(bridge_behavior test/test_bridge_behavior.py TIMEOUT 90)
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
