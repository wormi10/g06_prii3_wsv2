#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Paths
    eurobot_pkg = get_package_share_directory('eurobot_cositas')
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world = os.path.join(eurobot_pkg, 'world', 'Tablero2.world')
    rviz_config = os.path.join(eurobot_pkg, 'config', 'eurobot_view.rviz')
    launch_file_dir = os.path.join(turtlebot3_gazebo_pkg, 'launch')

    # =========================================================================
    # 1. GAZEBO: Servidor (se inicia primero)
    # =========================================================================
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),     
    )

    # =========================================================================
    # 2. GAZEBO: Cliente (se inicia 2 segundos después)
    # =========================================================================
    gzclient_cmd = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                ),
            )
        ]
    )

    # =========================================================================
    # 3. ROBOT: State Publisher (se inicia 3 segundos después)
    # IMPORTANTE: Solo si el robot waffle ya tiene un modelo URDF/SDF en Gazebo
    # =========================================================================
    robot_state_publisher_cmd = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [launch_file_dir, '/robot_state_publisher.launch.py']
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            )
        ]
    )

    # =========================================================================
    # 4. ARUCO: Detector (se inicia 5 segundos después)
    # =========================================================================
    aruco_detector_node = TimerAction(
        period=5.0,
        actions=[
            Node(
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
        ]
    )

    # =========================================================================
    # 5. NAVEGACION: Vision guided (se inicia 6 segundos después)
    # =========================================================================
    vision_navigation_node = TimerAction(
        period=6.0,
        actions=[
            Node(
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
                    'start_delay': 10.0,  # Esperar 10 segundos para RViz
                    'use_sim_time': use_sim_time,
                }]
            )
        ]
    )

    # =========================================================================
    # 6. RVIZ2: Visualización (se inicia 8 segundos después)
    # =========================================================================
    rviz_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================
    return LaunchDescription([
        gzserver_cmd,              # t=0s: Gazebo Server (con robot waffle incluido)
        gzclient_cmd,              # t=2s: Gazebo Client
        robot_state_publisher_cmd, # t=3s: Robot State Publisher (opcional)
        aruco_detector_node,       # t=5s: ArUco Detector
        vision_navigation_node,    # t=6s: Vision Navigation
        rviz_node,                 # t=8s: RViz2
    ])