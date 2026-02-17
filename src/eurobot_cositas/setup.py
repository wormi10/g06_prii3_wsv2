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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            # ↓ Esto registra el ejecutable subcriptor
            'subcriptor = eurobot_cositas.subcriptor:main',
        ],
    },
)