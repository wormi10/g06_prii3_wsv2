from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file para iniciar el sistema de navegación por ArUco.
    Inicia tanto el listener como el navegador.
    """
    
    # Argumentos de configuración
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='aruco_nav_params.yaml',
        description='Archivo de parámetros YAML'
    )
    
    robot_aruco_id_arg = DeclareLaunchArgument(
        'robot_aruco_id',
        default_value='3',
        description='ID del ArUco montado en el robot (3 u 8)'
    )
    
    # Nodo listener (monitor de tópicos)
    listener_node = Node(
        package='your_package_name',  # Cambiar al nombre de tu paquete
        executable='subcriptor.py',
        name='robot_listener',
        output='screen',
        emulate_tty=True,
    )
    
    # Nodo navegador
    navigator_node = Node(
        package='your_package_name',  # Cambiar al nombre de tu paquete
        executable='aruco_navigator.py',
        name='aruco_navigator',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
    )
    
    return LaunchDescription([
        params_file_arg,
        robot_aruco_id_arg,
        listener_node,
        navigator_node,
    ])
