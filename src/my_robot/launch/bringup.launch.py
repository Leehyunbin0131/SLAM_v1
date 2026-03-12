import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    my_robot_dir = get_package_share_directory('my_robot')

    motor_config = os.path.join(my_robot_dir, 'config', 'motor_params.yaml')
    urdf_file = os.path.join(my_robot_dir, 'urdf', 'my_robot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    lidar_serial_port = LaunchConfiguration('lidar_serial_port', default='/dev/lidar')
    motor_serial_port = LaunchConfiguration('motor_serial_port', default='/dev/dynamixel')

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_serial_port',
            default_value='/dev/lidar',
            description='Serial port for LiDAR'),
        DeclareLaunchArgument(
            'motor_serial_port',
            default_value='/dev/dynamixel',
            description='Serial port for Dynamixel motors'),

        # Robot State Publisher (URDF → TF: base_footprint → base_link → laser, wheels, etc.)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Motor controller (cmd_vel → motors, publishes /odom, TF off — EKF handles it)
        Node(
            package='my_robot',
            executable='motor_controller',
            name='motor_controller',
            parameters=[
                motor_config,
                {'device_port': motor_serial_port,
                 'publish_odom_tf': False},
            ],
            output='screen',
        ),

        # IMU (BNO085 via I2C)
        Node(
            package='my_robot',
            executable='imu_node',
            name='imu_node',
            parameters=[{
                'frame_id': 'imu_link',
                'i2c_address': 0x4B,
                'publish_rate': 50.0,
                'reset_gpio': 18,
            }],
            output='screen',
        ),

        # LiDAR (SLLidar C1)
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_serial_port,
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            output='screen',
        ),
    ])
