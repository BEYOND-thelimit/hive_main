cmake_minimum_required(VERSION 3.8)
project(hive_main_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pose_estimator_node src/pose_estimation.cpp)
ament_target_dependencies(pose_estimator_node
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(grid_map_odom_node src/odom_in_grid_map.cpp)
ament_target_dependencies(grid_map_odom_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

add_executable(print_grid_odom_node src/print_grid.cpp)
ament_target_dependencies(print_grid_odom_node
  rclcpp
  std_msgs
)

# node
install(TARGETS grid_map_odom_node pose_estimator_node print_grid_odom_node
  DESTINATION lib/${PROJECT_NAME}
)
# launch
install(DIRECTORY
  launch
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
