#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():
    hive_robot1_control_node = Node(
        package='hive_controller',
        executable='controller',
        namespace='robot1',  # 이렇게 네임스페이스를 추가합니다.
        parameters=[{'robot_num': 1} ],
        arguments=[],
        output="screen",
    )
    hive_robot2_control_node = Node(
        package='hive_controller',
        executable='controller',
        namespace='robot2',  # 이렇게 네임스페이스를 추가합니다.
        parameters=[{'robot_num': 1} ],
        arguments=[],
        output="screen",
    )
    hive_robot3_control_node = Node(
        package='hive_controller',
        executable='controller',
        namespace='robot3',  # 이렇게 네임스페이스를 추가합니다.
        parameters=[{'robot_num': 1} ],
        arguments=[],
        output="screen",
    )

    

    # create and return launch description object
    return LaunchDescription(
        [
            hive_robot1_control_node,
            hive_robot2_control_node,
            hive_robot3_control_node,
        ]
    )