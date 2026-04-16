echo 'Press ENTER for UAV Trigger' && read && \
ros2 topic pub --once /uav_to_ugv/command std_msgs/msg/String \
"data: '{\"command\": \"START_MOVING\"}'"