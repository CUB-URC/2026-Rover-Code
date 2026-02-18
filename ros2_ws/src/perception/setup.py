from setuptools import setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CU Rover Team',
    maintainer_email='curoverteam@colorado.edu',
    description='Perception package for YOLO and ArUco detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = perception.aruco_node:main',
            'debug_cam_node = perception.debug_cam_node:main',
            'yolo_node = perception.yolo_node:main',
        ],
    },
)