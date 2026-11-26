#!/usr/bin/env python3
"""Launch file para operacao em tempo real com o Prototipo Fisico ESP32-WROOM + RViz2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description() -> LaunchDescription:
    """Inicia robot_state_publisher, hardware_bridge, central_monitor e RViz2 para o robo real."""
    pkg_share = get_package_share_directory('parking_inspector_bot')
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'parking_bot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'parking_monitor.rviz')

    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Porta serial USB onde o ESP32-WROOM-32 esta conectado',
    )

    declare_baud_rate = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Taxa de comunicacao serial (baud rate) do ESP32-WROOM-32',
    )

    robot_description_xml = xacro.process_file(xacro_file_path).toxml()

    # 1. Publica a geometria URDF e os frames fixos dos sensores ultrassonicos
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_xml,
                'use_sim_time': False,
            }
        ],
    )

    # 2. Ponte Serial ESP32-WROOM <-> Topicos ROS2 (/odom, /ultrasonic/*, /spot_status_update, /cmd_vel)
    hardware_bridge_node = Node(
        package='parking_inspector_bot',
        executable='hardware_bridge',
        name='hardware_bridge_node',
        output='screen',
        parameters=[
            {
                'serial_port': serial_port,
                'baud_rate': ParameterValue(baud_rate, value_type=int),
                'use_sim_time': False,
            }
        ],
    )

    # 3. Sistema Central de Monitoramento calibrado inicialmente para as vagas fisicas (1.60m x 0.68m)
    central_monitor_node = Node(
        package='parking_inspector_bot',
        executable='central_monitor',
        name='central_monitor_node',
        output='screen',
        parameters=[
            {
                'spots_x': [1.60, 3.20, 4.80, 6.40, 8.00],
                'spots_center_y': -0.49,
                'spot_width_m': 1.50,
                'spot_length_m': 0.68,
                'use_sim_time': False,
            }
        ],
    )

    # 4. Visualizacao 3D ao vivo no RViz2 sincronizada com o prototipo fisico
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_physical_monitor',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription(
        [
            declare_serial_port,
            declare_baud_rate,
            robot_state_publisher_node,
            hardware_bridge_node,
            central_monitor_node,
            rviz2_node,
        ]
    )
