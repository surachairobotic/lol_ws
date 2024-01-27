from setuptools import setup

package_name = 'hospital_agv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yok',
    maintainer_email='yok@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = hospital_agv.imu_publisher_v2:main',
            'odom_publisher = hospital_agv.odom_publisher_v2:main',
            'motor_publisher = hospital_agv.motor_driver:main',
            'robot_state_publisher = hospital_agv.robot_state_publisher:main',
            'server = hospital_agv.server:main',
            'send_goal = hospital_agv.send_goal:main'
        ],
    },
)
