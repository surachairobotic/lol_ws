import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lslidar_c16.yaml')

    frame_id = LaunchConfiguration('frame_id', default='laser_link')

    return LaunchDescription([
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar. Default frame_id is \'laser_link\''
        ),
        Node(
            package='lslidar_driver',
            namespace='c16',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[driver_dir],
            remappings=[
                ('/c16/scan', '/scan'),  # Remap /c16/scan to /scan
            ],
        ),
    ])

