from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'eurobot_cositas'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        ('share/' + package_name, ['package.xml']),
        
        # Instalar todos los archivos launch
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        
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
    maintainer='wormi',
    maintainer_email='wormi@todo.todo',
    description='Eurobot 2026 navigation with ArUco vision guidance',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'aruco_detector_cositas = eurobot_cositas.aruco_detector_cositas:main',
            'vision_navigation = eurobot_cositas.vision_navigation:main',
        ],
    },
)