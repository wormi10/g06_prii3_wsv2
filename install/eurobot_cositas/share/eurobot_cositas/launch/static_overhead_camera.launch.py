#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='overhead_camera_static_tf',
            arguments=[
                '0', '0', '2.5',        # x y z (meters)
                '0', '0', '0',          # roll pitch yaw (radians)
                'world',
                'overhead_camera_link'
            ],
            parameters=[{'use_sim_time': True}]
        )
    ])
