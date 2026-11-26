#!/usr/bin/env python3
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'parking_inspector_bot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            [f for f in glob(os.path.join('launch', '*')) if os.path.isfile(f)],
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            [f for f in glob(os.path.join('urdf', '*')) if os.path.isfile(f)],
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            [f for f in glob(os.path.join('worlds', '*')) if os.path.isfile(f)],
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            [f for f in glob(os.path.join('rviz', '*')) if os.path.isfile(f)],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Equipe de Engenharia de Robotica Autonoma',
    maintainer_email='engenharia@robotica.edu.br',
    description=(
        'Simulacao completa em ROS2, Gazebo e RViz2 de um robo movel autonomo '
        'para monitoramento de vagas de estacionamento via sensor ultrassonico.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_navigator = parking_inspector_bot.patrol_navigator_node:main',
            'ultrasonic_detector = parking_inspector_bot.ultrasonic_detector_node:main',
            'central_monitor = parking_inspector_bot.central_monitor_node:main',
            'hardware_bridge = parking_inspector_bot.hardware_bridge_node:main',
        ],
    },
)
