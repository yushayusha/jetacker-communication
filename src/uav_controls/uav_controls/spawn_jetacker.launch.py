#!/usr/bin/env python3
# ================================================================
# SPAWN JETACKER WITH UAV LAUNCH FILE
# ================================================================
# This launch file spawns the complete Hiwonder JetAcker UGV
# with landing pad and UAV quadcopter on top
# ================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    
    # ============================================
    # SET LIDAR TYPE ENVIRONMENT VARIABLE
    # ============================================
    # This tells the URDF which LiDAR model to use (A1, A2, S2L, LD14P, or G4)
    set_lidar_type = SetEnvironmentVariable('LIDAR_TYPE', 'A1')
    
    # ============================================
    # GET PACKAGE DIRECTORY
    # ============================================
    jetacker_pkg = get_package_share_directory('jetacker_description')
    
    # ============================================
    # LOAD ROBOT DESCRIPTION
    # ============================================
    # Use xacro to process the main jetacker.xacro file
    # This includes the UGV, landing pad, and UAV
    urdf_path = os.path.join(jetacker_pkg, 'urdf', 'jetacker.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    
    # ============================================
    # ROBOT STATE PUBLISHER NODE
    # ============================================
    # Publishes the robot's URDF and TF transforms to ROS2
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True  # Use Gazebo's simulation time
        }]
    )
    
    # ============================================
    # SPAWN ENTITY IN GAZEBO
    # ============================================
    # Spawns the JetAcker (with landing pad and UAV) at origin
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'jetacker',           # Name of the entity in Gazebo
            '-topic', 'robot_description',   # Topic containing URDF
            '-x', '0',                       # Spawn at x=0
            '-y', '0',                       # Spawn at y=0
            '-z', '0.1'                      # Spawn 10cm above ground
        ],
        output='screen'
    )
    
    # ============================================
    # RETURN LAUNCH DESCRIPTION
    # ============================================
    return LaunchDescription([
        set_lidar_type,           # Set environment variable first
        robot_state_publisher,    # Start robot state publisher
        spawn_robot,              # Spawn robot in Gazebo
    ])
