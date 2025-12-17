from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'eurobot_cositas'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ROS 2 package indexing
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # Gazebo worlds
        (os.path.join('share', package_name, 'world'),
            glob('world/*.world')),

        # RViz configs
        (os.path.join('share', package_name, 'config'),
            glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilias',
    maintainer_email='ilias@todo.todo',
    description='Eurobot vision-guided navigation using overhead ArUco markers (PBI 4.3)',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            # ArUco detection from overhead camera
            'aruco_detector = eurobot_cositas.aruco_detector_cositas:main',
            'interactive_vision_navigation = eurobot_cositas.interactive_vision_navigation:main',
            # PURE vision-based navigation (NO Nav2)
            'vision_guided_navigation = eurobot_cositas.vision_guided_navigation:main',
            'goto_xy = eurobot_cositas.goto_xy:main',
            'aruco_map_viewer = eurobot_cositas.aruco_map_viewer:main',
            'board_frame_publisher = eurobot_cositas.board_frame_publisher',
            'waypoint_navigation = eurobot_cositas.waypoint_navigation:main',
        ],
    },
)
