from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='remote',
            executable='remote_main',
            name='remote_main',
            output='screen'
        ),
        Node(
            package='holoprojector',
            executable='holoprojector_node',
            name='holoprojector_node',
            output='screen'
        )
    ])