import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot'),
        'config',
        'motor_params.yaml'
    )

    motor_controller_node = Node(
        package='my_robot',
        executable='motor_controller',
        name='motor_controller',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([
        motor_controller_node,
    ])
