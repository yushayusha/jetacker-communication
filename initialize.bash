#!/bin/bash

source install/setup.bash
export ROS_DOMAIN_ID=0
echo $ROS_DOMAIN_ID
ros2 run telemetry_package main_gui