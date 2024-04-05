# One robot P-Control
## Init Gazebo at hive_simualtion
`ros2 launch hive_gazebo spawn_one_robot.launch.py`

## Start Controller
`ros2 run hive_control control_publisher`

## pub x, y, yaw
`ros2 topic pub /target_position std_msgs/msg/String "data: '3.0, 2.0, 3.14'"`