from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration('world')
    vehicle = LaunchConfiguration('vehicle')

    camera_topic = PythonExpression([
        '"/world/" + "', world,
        '" + "/model/" + "', vehicle,
        '" + "_0/link/camera_link/sensor/camera/image"'
        '@sensor_msgs/msg/Image[gz.msgs.Image'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='default'),
        DeclareLaunchArgument('vehicle', default_value='x500_mono_cam_down'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[camera_topic],
            output='screen',
        ),
    ])
