cmake_minimum_required(VERSION 3.8)
project(hive_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED) 
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)



include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(image_visualize_code src/image_visualize_code.cpp)
ament_target_dependencies(image_visualize_code Eigen3)
target_link_libraries(image_visualize_code ${OpenCV_LIBS})

add_executable(map_publisher src/map_publisher.cpp)
ament_target_dependencies(map_publisher rclcpp std_msgs nav_msgs tf2_ros geometry_msgs Eigen3)

add_executable(map_transformer_node src/map_transformer_node.cpp)
ament_target_dependencies(map_transformer_node rclcpp nav_msgs Eigen3 tf2_ros geometry_msgs tf2_eigen OpenCV)
target_include_directories(map_transformer_node PRIVATE ${Eigen3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(hive_master_node src/hive_master.cpp)
ament_target_dependencies(hive_master_node rclcpp std_msgs Eigen3 nav_msgs geometry_msgs)


install(TARGETS 
  image_visualize_code
  map_publisher
  map_transformer_node
  hive_master_node


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
