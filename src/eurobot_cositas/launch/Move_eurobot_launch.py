#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # =========================================================================
    # Paths
    # =========================================================================
    eurobot_pkg = get_package_share_directory('eurobot_cositas')
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world = os.path.join(eurobot_pkg, 'world', 'Tablero2.world')
    rviz_config = os.path.join(eurobot_pkg, 'config', 'eurobot_view.rviz')
    launch_file_dir = os.path.join(turtlebot3_gazebo_pkg, 'launch')

    # =========================================================================
    # 0. STATIC CAMERA: TF from world -> overhead_camera_link
    # =========================================================================
    static_camera_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='overhead_camera_static_tf',
        arguments=['0', '0', '2.5', '0', '0', '0', 'world', 'overhead_camera_link']
    )

    # =========================================================================
    # 1. GAZEBO: Server
    # =========================================================================
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    # =========================================================================
    # 2. GAZEBO: Client
    # =========================================================================
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # =========================================================================
    # 3. Robot State Publisher
    # =========================================================================
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/robot_state_publisher.launch.py']
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # =========================================================================
    # 4. ArUco Detector
    # =========================================================================
    aruco_detector_node = Node(
        package='eurobot_cositas',
        executable='aruco_detector_cositas.py',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'camera_topic': '/overhead_camera/image_raw',
            'camera_info_topic': '/overhead_camera/camera_info',
            'camera_frame': 'overhead_camera_link',
            'aruco_dict': 'DICT_4X4_50',
            'marker_size': 0.1,
            'use_sim_time': use_sim_time,
        }]
    )

    # =========================================================================
    # 5. Vision-guided navigation
    # =========================================================================
    vision_navigation_node = Node(
        package='eurobot_cositas',
        executable='vision_navigation.py',
        name='vision_guided_navigation',
        output='screen',
        parameters=[{
            'linear_speed': 0.12,
            'angular_speed': 0.5,
            'distance_tolerance': 0.15,
            'angle_tolerance': 0.08,
            'wait_time': 5.0,
            'start_delay': 10.0,
            'use_sim_time': use_sim_time,
        }]
    )

    # =========================================================================
    # 6. RViz2
    # =========================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # =========================================================================
    # Launch description
    # =========================================================================
    return LaunchDescription([
        static_camera_node,         # t=0s: static camera TF
        gzserver_cmd,               # Gazebo server
        gzclient_cmd,               # Gazebo client
        robot_state_publisher_cmd,  # Robot state publisher
        aruco_detector_node,        # ArUco detector
        vision_navigation_node,     # Vision-guided navigation
        rviz_node,                  # RViz2
    ])
