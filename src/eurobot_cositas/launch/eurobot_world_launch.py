#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(
        get_package_share_directory('eurobot_cositas'),
        'world',
        'Tablero2.world'
    )

    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch'
    )

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ---------- ArUco TABLE detector (PBI 4.2 – table) ----------
    aruco_table_node = Node(
        package='aruco_detector_pkg',
        executable='aruco_detector',
        name='aruco_table_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ---------- ArUco ROBOT detector (PBI 4.2 – robot) ----------
    aruco_robot_node = Node(
        package='aruco_detector_pkg',
        executable='aruco_robot_detector',
        name='aruco_robot_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([

        # ---------- Gazebo server ----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        # ---------- Gazebo client ----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # ---------- Robot State Publisher ----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # ---------- Static overhead camera TF ----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('eurobot_cositas'),
                    'launch',
                    'static_overhead_camera.launch.py'
                )
            ),
        ),

        # ---------- ArUco table detector ----------
        aruco_table_node,

        # ---------- ArUco robot detector ----------
        aruco_robot_node,
    ])
