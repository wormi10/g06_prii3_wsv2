import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Ruta al paquete
    pkg_dir = get_package_share_directory('eurobot_cositas')

    # Ruta a la carpeta del rosbag (dentro del paquete instalado)
    rosbag_dir = os.path.join(pkg_dir, 'rosbag2')

    # Nodo subcriptor
    subcriptor_node = Node(
        package='eurobot_cositas',
        executable='subcriptor',
        name='subcriptor',
        output='screen',
    )

    # Reproducción del rosbag en bucle
    '''rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', rosbag_dir, '--loop'],
        output='screen',
    )
'''
    return LaunchDescription([
      #  rosbag_play,
        subcriptor_node,
    ])
