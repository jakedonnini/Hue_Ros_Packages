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
        Node(
            package='arduino_reader',
            executable='arduino_serial_node',
            name='arduino_reader',
            output='log'
        ),
        Node(
            package='navigator',
            executable='DeadReck',
            name='dead_reckoning',
            output='log'
        ),
        Node(
            package='calm_man_cpp',
            executable='kalman_node',
            name='kalman_filter',
            output='screen'
        ),
        Node(
            package='navigator',
            executable='NavLite',
            name='navigation_node',
            output='screen'
        )
    ])
