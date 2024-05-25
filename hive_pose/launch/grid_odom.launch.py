from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    start_robot1_grid_odom_cmd = Node(
        package='hive_pose',
        executable='grid_map_odom_node',
        name='robot1_grid_map_odom_node',
        output='screen',
        parameters=[
                    {'use_sim_time': True},
                    {'robot_num': 1}
                    ]
    )
    start_robot2_grid_odom_cmd = Node(
        package='hive_pose',
        executable='grid_map_odom_node',
        name='robot2_grid_map_odom_node',
        output='screen',
        parameters=[
                    {'use_sim_time': True},
                    {'robot_num': 2}
                    ]
    )
    start_robot3_grid_odom_cmd = Node(
        package='hive_pose',
        executable='grid_map_odom_node',
        name='robot3_grid_map_odom_node',
        output='screen',
        parameters=[
                    {'use_sim_time': True},
                    {'robot_num': 3}
                    ]
    )

    return LaunchDescription([
        start_robot1_grid_odom_cmd,
        start_robot2_grid_odom_cmd,
        start_robot3_grid_odom_cmd
    ])