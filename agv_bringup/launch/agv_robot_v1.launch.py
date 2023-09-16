#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():

    agv_param_dir = LaunchConfiguration(
        'agv_param_dir',
        default=os.path.join(
            get_package_share_directory('agv_bringup'),
            'param',
            'agv5.yaml'))
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'agv_param_dir',
            default_value=agv_param_dir,
            description='Full path to agv parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/agv_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a3_launch.py')
            ),
            launch_arguments={
                'serial_port': '/dev/ttyUSB1',  # Set the correct USB port
                'frame_id': 'laser'
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('agv_bringup'), 'launch', 'laser.launch.py')
            ),
            launch_arguments={'device_ip': '192.168.1.200', 'frame_id': 'laser_link'}.items(),
        ),

        Node(
            package='hospital_agv',
            executable='imu_publisher_s1',
            name='imu_publisher_s1',
            output='screen',
        ),

        Node(
            package='hospital_agv',
            executable='odom_publisher_v1',
            name='odom_publisher_v1',
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        )

        # Add other nodes below as needed

    ])

