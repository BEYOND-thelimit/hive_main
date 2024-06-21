cmake_minimum_required(VERSION 3.8)
project(hive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED) 
find_package(tf2_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

add_executable(dwa_controller src/dwa_controller.cpp)
ament_target_dependencies(dwa_controller rclcpp sensor_msgs nav_msgs geometry_msgs Eigen3)

add_executable(admittance_controller src/admittance_controller.cpp)
ament_target_dependencies(admittance_controller rclcpp sensor_msgs nav_msgs geometry_msgs Eigen3)

add_executable(p_controller src/p_controller.cpp)
ament_target_dependencies(p_controller rclcpp sensor_msgs nav_msgs geometry_msgs Eigen3)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp sensor_msgs nav_msgs geometry_msgs Eigen3 tf2_ros)

add_executable(test_trajectory src/test_trajectory.cpp)
ament_target_dependencies(test_trajectory rclcpp std_msgs nav_msgs)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS 
  dwa_controller
  admittance_controller
  p_controller
  test_trajectory
  controller
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
