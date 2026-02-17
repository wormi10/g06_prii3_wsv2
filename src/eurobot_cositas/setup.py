from setuptools import setup
import os
from glob import glob

package_name = 'eurobot_cositas'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/eurobot_cositas']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # ↓ Esto copia los archivos del rosbag al install
        ('share/' + package_name + '/rosbag2', glob('rosbag2/*')),
                # Instalar mundo de Gazebo
        (os.path.join('share', package_name, 'world'),
            glob('world/*')),

        # Instalar configuracion de RViz
        (os.path.join('share', package_name, 'config'),
            glob('config/*.rviz')),

            #Templates
        (os.path.join('share', package_name, 'templates'), glob('templates/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'subcriptor = eurobot_cositas.subcriptor:main',
            'aruco_detector_cositas = eurobot_cositas.aruco_detector_cositas:main',
            'vision_navigation = eurobot_cositas.vision_navigation:main'
        ],
    },
)
