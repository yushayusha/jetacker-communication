#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get package directory path
    pkg_dir = get_package_share_directory('uav_ugv_sim')
    
    # Get world file path
    world_file = os.path.join(pkg_dir, 'worlds', 'field.world')
    
    # Launch Gazebo with the world file
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
    
    # TODO: Spawn UGV robot here
    # TODO: Spawn UAV robot here
    
    # Return all launch actions
    return LaunchDescription([
        gazebo,
    ]) 
