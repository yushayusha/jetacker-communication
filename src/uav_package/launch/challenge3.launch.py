from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    send_csv = Node(
        package='uav_package', 
        executable='send_csv',
        name='send_csv',
        output='screen'
    )
        
    ssh_ugv = Node(
        package='uav_package',
        executable='start_challenge3',
        name='start_challenge3',
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
        executable='uav_trigger_c2c3',
        name='uav_trigger_c2c3',
        output='screen'
    )

    delayed_nav2 = TimerAction(
        period=45.0,   # wait 45 seconds
        actions=[start_nav2]
    )

    return LaunchDescription([
            send_csv,
            ssh_ugv,
            uav_trigger,
            delayed_nav2
    ])