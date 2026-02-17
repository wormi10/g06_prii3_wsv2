#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    vision_navigation_node = Node(
        package='eurobot_cositas',
        executable='vision_navigation',
        name='movimiento_JetBot',
        output='screen',
        parameters=[{'use_sim_time': False}]  # Tiempo real, NO simulación
    )

    return LaunchDescription([
        vision_navigation_node,
    ])






