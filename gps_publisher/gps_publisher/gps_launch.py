from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_publisher',
            executable='gps_publisher',
            name='gps_publisher',
            output='screen'
        ),
        Node(
            package='gps_publisher',
            executable='gps2_publisher',
            name='gps2_publisher',
            output='screen'
        ),
        Node(
            package='gps_publisher',
            executable='gps_fusion_node',
            name='gps_fusion_node',
            output='screen'
        ),
    ])
