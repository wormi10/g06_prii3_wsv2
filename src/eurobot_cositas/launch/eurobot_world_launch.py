#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
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

    # Ruta del modelo del robot
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_' + TURTLEBOT3_MODEL,
        'model.sdf'
    )

    # Nodo detector de ArUco
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

    return LaunchDescription([
        # Servidor de Gazebo con el mundo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),     
        ),

        # Cliente de Gazebo (interfaz gráfica)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # Robot State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_dir, '/robot_state_publisher.launch.py']
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Nodo detector de ArUco
        aruco_detector_node,
    ])