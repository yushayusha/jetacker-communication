from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    send_csv = Node(
        package='uav_package', 
        executable='send_csv',
        name='send_csv',
        output='screen'
    )
        
    ssh_ugv = Node(
        package='uav_package',
        executable='start_challenge1',
        name='start_challenge1',
        output='screen'
    )

    start_nav2 = Node(
        package='uav_package',
        executable='start_nav2',
        name='start_nav2',
        output='screen'
    )

    uav_trigger = Node(
        package='uav_package',
        executable='uav_trigger_c1',
        name='uav_trigger_c1',
        output='screen'
    )
    return LaunchDescription([
            send_csv,
            ssh_ugv,
            uav_trigger
    ])