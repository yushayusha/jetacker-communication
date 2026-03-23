#!/usr/bin/env python3
"""
bringup_nodes.launch.py

Starts the core ROS2 nodes in this package:
- aruco_detector_improved
- uav_integrated_controller (PX4 OFFBOARD via MAVROS)
- optional ugv_controller (only if you are NOT using JetAcker's own UGV controller)

This launch does NOT start:
- Gazebo world
- PX4 SITL
- MAVROS
because those vary by your setup. Start those first, then run this launch.

Usage example:
  ros2 launch uavugv_control bringup_nodes.launch.py challenge:=2 scan:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    challenge = LaunchConfiguration("challenge")
    aruco_id = LaunchConfiguration("aruco_id")
    scan = LaunchConfiguration("scan")

    # ArUco detector parameters
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    fallback_to_gps = LaunchConfiguration("fallback_to_gps")
    ugv_odom_topic = LaunchConfiguration("ugv_odom_topic")
    uav_pose_topic = LaunchConfiguration("uav_pose_topic")

    aruco_node = Node(
        package="uavugv_control",
        executable="aruco_detector_improved",
        output="screen",
        parameters=[{
            "marker_id": 0,
            "image_topic": image_topic,
            "camera_info_topic": camera_info_topic,
            "fallback_to_gps": fallback_to_gps,
            "ugv_odom_topic": ugv_odom_topic,
            "uav_pose_topic": uav_pose_topic,
        }],
    )

    # NOTE: uav_integrated_controller parses CLI args (remove_ros_args(sys.argv)),
    # so we pass mission settings as "arguments" here.
    uav_node = Node(
        package="uavugv_control",
        executable="uav_integrated_controller",
        output="screen",
        arguments=[
            "--challenge", challenge,
            "--aruco_id", aruco_id,
        ],
    )

    ugv_node = Node(
        package="uavugv_control",
        executable="ugv_controller",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("challenge", default_value="1"),
        DeclareLaunchArgument("aruco_id", default_value="0"),
        DeclareLaunchArgument("scan", default_value="false"),

        DeclareLaunchArgument("image_topic", default_value="auto"),
        DeclareLaunchArgument("camera_info_topic", default_value="auto"),
        DeclareLaunchArgument("fallback_to_gps", default_value="true"),
        DeclareLaunchArgument("ugv_odom_topic", default_value="/ugv/odom"),
        DeclareLaunchArgument("uav_pose_topic", default_value="/mavros/local_position/pose"),

        aruco_node,
        uav_node,
        ugv_node,
    ])
