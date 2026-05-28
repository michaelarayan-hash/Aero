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
        '" + "_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"'
    ])

    lidar_topic = PythonExpression([
        '"/world/" + "', world,
        '" + "/model/" + "', vehicle,
        '" + "_0/link/lidar_sensor_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan]"'
    ])
    lidar_points_topic = PythonExpression([
        '"/world/" + "', world,
        '" + "/model/" + "', vehicle,
        '" + "_0/link/lidar_sensor_link/sensor/lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked]"'
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
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[lidar_topic],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[lidar_points_topic],
            output='screen',
        ),
    ])
