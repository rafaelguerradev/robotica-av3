#!/usr/bin/env python3
"""Launch file completo para simulacao integrada ROS2 + Gazebo + RViz2 do parking_inspector_bot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description() -> LaunchDescription:
    """Inicia Gazebo, robot_state_publisher, spawn_entity, RViz2 e os 3 nos autonomos."""
    pkg_share = get_package_share_directory('parking_inspector_bot')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_file_path = os.path.join(pkg_share, 'worlds', 'parking_lot.world')
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'parking_bot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'parking_monitor.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usa o relogio simulado /clock publicado pelo Gazebo',
    )

    # Processa o modelo Xacro gerando o XML URDF completo
    robot_description_xml = xacro.process_file(xacro_file_path).toxml()

    # 1. Servidor e cliente grafico do Gazebo carregando parking_lot.world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file_path,
            'verbose': 'false',
        }.items(),
    )

    # 2. Publicador da arvore cinematica TF do robo (robot_state_publisher)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_xml,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # 3. Insercao (spawn) do robo na Estacao de Inicio e Retorno (0.0, 0.0, 0.05)
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_parking_inspector_bot',
        output='screen',
        arguments=[
            '-entity',
            'parking_inspector_bot',
            '-topic',
            'robot_description',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.05',
            '-Y',
            '0.0',
        ],
    )

    # 4. Visualizacao 3D em tempo real no RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_parking_monitor',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 5. No de navegacao autonoma por Maquina de Estados Finitos
    patrol_navigator_node = Node(
        package='parking_inspector_bot',
        executable='patrol_navigator',
        name='patrol_navigator_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 6. No de deteccao e classificacao de vagas por ultrassom (Opcao C)
    ultrasonic_detector_node = Node(
        package='parking_inspector_bot',
        executable='ultrasonic_detector',
        name='ultrasonic_detector_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 7. No do Sistema Central de Monitoramento (Painel CLI + JSON + RViz2 Markers)
    central_monitor_node = Node(
        package='parking_inspector_bot',
        executable='central_monitor',
        name='central_monitor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            gazebo_launch,
            robot_state_publisher_node,
            spawn_robot_node,
            rviz2_node,
            patrol_navigator_node,
            ultrasonic_detector_node,
            central_monitor_node,
        ]
    )
