from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('altitude', default_value='3.0',
                              description='Takeoff altitude in metres'),
        DeclareLaunchArgument('display', default_value='false',
                              description='Show ArUco detection window'),
        DeclareLaunchArgument('mavsdk_server_host', default_value='sim',
                              description='Hostname of the mavsdk_server (default: sim container)'),

        Node(
            package='camera_feed',
            executable='aruco_node',
            name='aruco_node',
            output='screen',
            parameters=[{'display': LaunchConfiguration('display')}],
        ),
        Node(
            package='camera_feed',
            executable='landing_node',
            name='landing_node',
            output='screen',
            parameters=[{
                'altitude': LaunchConfiguration('altitude'),
                'mavsdk_server_host': LaunchConfiguration('mavsdk_server_host'),
            }],
        ),
    ])
