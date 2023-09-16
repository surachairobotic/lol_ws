import launch
import launch_ros.actions

def generate_launch_description():
    ekf_config = launch.substitutions.LaunchConfiguration('ekf_config', default='$(find my_robot)/config/ekf.yaml')

    ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('odom', '/agv/odom'), ('imu/data', '/imu/imu'), ('imu/mag', '/imu/mag')],
        arguments=['load', ekf_config])

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('ekf_config', default_value=ekf_config.default_value),

        ekf_node
    ])

