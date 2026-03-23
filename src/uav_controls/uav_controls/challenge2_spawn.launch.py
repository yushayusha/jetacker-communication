#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    
    # Get package directories
    uav_ugv_pkg = get_package_share_directory('uav_ugv_sim')
    jetacker_pkg = get_package_share_directory('jetacker_description')
    
    # World file path
    world_file = os.path.join(uav_ugv_pkg, 'worlds', 'challenge2.world')
    
    # Robot URDF path
    urdf_path = os.path.join(jetacker_pkg, 'urdf', 'jetacker.xacro')
    robot_description = Command(['xacro ', urdf_path])
    
    # Launch Gazebo with Challenge 2 world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': world_file}.items()
    )
    
    # Robot state publisher - publishes robot TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    # Spawn JetAcker robot in Gazebo at origin
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'jetacker',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
