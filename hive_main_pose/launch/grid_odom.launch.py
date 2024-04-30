from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    start_robot1_grid_odom_cmd = Node(
        package='hive_main_pose',
        executable='grid_map_odom_node',
        name='robot1_grid_map_odom_node',
        output='screen',
        parameters=[
                    {'use_sim_time': True},
                    ]
    )
    start_robot2_grid_odom_cmd = Node(
        package='hive_main_pose',
        executable='grid_map_odom_node',
        name='robot2_grid_map_odom_node',
        output='screen',
        parameters=[
                    {'use_sim_time': True},
                    ],
        remappings=[
            ('/robot1/final_pose', '/robot2/final_pose'),
            ('/robot1/grid_odom', '/robot2/grid_odom'),
        ]
    )
    start_robot3_grid_odom_cmd = Node(
        package='hive_main_pose',
        executable='grid_map_odom_node',
        name='robot3_grid_map_odom_node',
        output='screen',
        parameters=[
                    {'use_sim_time': True},
                    ],
        remappings=[
            ('/robot1/final_pose', '/robot3/final_pose'),
            ('/robot1/grid_odom', '/robot3/grid_odom'),
        ]
    )

    return LaunchDescription([
        start_robot1_grid_odom_cmd,
        start_robot2_grid_odom_cmd,
        start_robot3_grid_odom_cmd
    ])