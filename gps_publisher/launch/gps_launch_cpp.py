from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_publisher_cpp',
            executable='gps_pub1',
            name='gps1_publisher',
            output='screen'
        ),
        Node(
            package='gps_publisher_cpp',
            executable='gps_pub2',
            name='gps2_publisher',
            output='screen'
        ),
        Node(
            package='gps_publisher_cpp',
            executable='gps_fus',
            name='gps_fusion_node',
            output='screen'
        )
        # Node(
        #     package='gps_publisher',
        #     executable='gps3_node',
        #     name='gps_fusion_node',
        #     output='screen'
        # )
    ])
