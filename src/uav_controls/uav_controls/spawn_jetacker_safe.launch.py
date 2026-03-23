#!/usr/bin/env python3
"""
spawn_jetacker_safe.launch.py

Spawns ONLY the UGV ("jetacker") into an already-running Gazebo Classic instance via gazebo_ros_factory,
and starts the UGV controller. This avoids the common integration failure where a "fake UAV" model is
spawned as part of the JetAcker URDF (which is NOT connected to PX4 SITL), leading to "UAV won't fly".

Use this in place of spawn_jetacker.launch.py when you are controlling the UAV via PX4 + MAVROS.

Assumptions:
- You have a 'jetacker_description' package that contains a minimal xacro without the UAV, e.g.:
    urdf/jetacker_minimal.xacro
- Gazebo Classic is running with gazebo_ros factory plugin (provides /spawn_entity service).
- You have a ROS2 node ugv_controller.py that publishes /ugv/odom.

If your xacro has a different name/path, change XACRO_REL_PATH below.

Typical flow (one Gazebo instance):
  Terminal A: start Gazebo (challenge world) with gazebo_ros
  Terminal B: ros2 launch <your_pkg> spawn_jetacker_safe.launch.py
  Terminal C: start PX4 SITL + MAVROS + touchdown_controller (UAV)

"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

XACRO_REL_PATH = os.path.join("urdf", "jetacker_minimal.xacro")


def generate_launch_description():
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    entity = LaunchConfiguration("entity")
    controller_pkg = LaunchConfiguration("controller_pkg")
    controller_exec = LaunchConfiguration("controller_exec")

    desc_pkg = get_package_share_directory("jetacker_description")
    xacro_path = os.path.join(desc_pkg, XACRO_REL_PATH)

    # Generate robot_description at launch time using xacro
    robot_description = {"robot_description": ["xacro ", xacro_path]}

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", entity,
            "-topic", "robot_description",
            "-x", x, "-y", y, "-z", z,
            "-Y", yaw,
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    controller = Node(
        package=controller_pkg,
        executable=controller_exec,
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.10"),  # slightly above ground to avoid collision pop
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("entity", default_value="jetacker"),

        # Update these if your controller package/executable names differ
        DeclareLaunchArgument("controller_pkg", default_value="uavugv_control"),
        DeclareLaunchArgument("controller_exec", default_value="ugv_controller"),

        # Publish robot_description so spawn_entity.py can read it
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[robot_description]),

        # Give Gazebo a moment to finish starting before we call /spawn_entity
        TimerAction(period=2.0, actions=[spawn, controller]),
    ])
