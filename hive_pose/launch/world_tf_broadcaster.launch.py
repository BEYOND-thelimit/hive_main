#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def generate_launch_description():
    st_pub1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        emulate_tty=True,
        arguments=["0", "0", "0", "0", "0", "1", "0", "virtual_world", "world"]
    )

    st_pub2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        emulate_tty=True,
        arguments=["1.44", "1.12", "2.5", "-1.570796", "1.570796", "1.570796", "world", "camera_link"]
    )

    return LaunchDescription([
        st_pub1,
        st_pub2,
        ])
