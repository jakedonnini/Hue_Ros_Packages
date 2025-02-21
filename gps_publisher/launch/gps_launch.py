from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_publisher',
            executable='gps_node',
            name='gps_publisher',
            output='screen'
        ),
        Node(
            package='gps_publisher',
            executable='gps2_node',
            name='gps2_publisher',
            output='screen'
        ),
        Node(
            package='gps_publisher',
            executable='gps3_node',
            name='gps_fusion_node',
            output='screen'
        ),
    ])
