from setuptools import setup
from glob import glob
import os

package_name = 'aruco_detector_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilias',
    maintainer_email='iliasselkamilili@gmail.com',
    description='ArUco detection nodes for Eurobot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Table detector (PBI 4.2 – first bullet)
            'aruco_detector = aruco_detector_pkg.aruco_detector:main',

            # Robot detector (PBI 4.2 – second bullet)
            'aruco_robot_detector = aruco_detector_pkg.aruco_robot_detector:main',
        ],
    },
)
