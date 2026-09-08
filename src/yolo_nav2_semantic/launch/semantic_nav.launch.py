#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('yolo_nav2_semantic')
    gazebo_share = get_package_share_directory('gazebo_ros')
    nav2_share = get_package_share_directory('nav2_bringup')
    world = os.path.join(share, 'worlds', 'semantic_warehouse.world')
    urdf = os.path.join(share, 'urdf', 'semantic_rover.urdf')
    params = os.path.join(share, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(share, 'maps', 'warehouse.yaml')
    rviz = os.path.join(share, 'rviz', 'semantic_nav.rviz')
    with open(urdf, 'r', encoding='utf-8') as stream:
        robot_description = stream.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'gazebo_gui', default_value='true',
            description='Open the Gazebo graphical client; false is faster in a VM'),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(share, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': world, 'verbose': 'false',
                'gui': LaunchConfiguration('gazebo_gui')
            }.items(),
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen'
        ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'semantic_rover', '-topic', 'robot_description',
                       '-x', '-4.0', '-y', '-3.0', '-z', '0.25'],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml, 'params_file': params, 'use_sim_time': 'true',
                'autostart': 'true', 'use_composition': 'False'
            }.items(),
        ),
        TimerAction(period=4.0, actions=[Node(
            package='yolo_nav2_semantic', executable='semantic_obstacle_node',
            parameters=[{'use_sim_time': True}], output='screen'
        )]),
        Node(
            package='rviz2', executable='rviz2', arguments=['-d', rviz],
            parameters=[{'use_sim_time': True}], output='screen'
        ),
        TimerAction(period=12.0, actions=[Node(
            package='yolo_nav2_semantic', executable='patrol_manager',
            parameters=[{'use_sim_time': True}], output='screen'
        )]),
    ])
