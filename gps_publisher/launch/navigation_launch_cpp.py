from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # idk if we need this cause of the gps launch...
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
        ),
        Node(
            package='arduino_reader_cpp',
            executable='ard_read_exe',
            name='arduino_reader',
            output='log'
        ),
        Node(
            package='deadreck_cpp',
            executable='dead_reckoning_node',
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
