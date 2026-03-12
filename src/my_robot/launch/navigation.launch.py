import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    my_robot_dir = get_package_share_directory('my_robot')
    nav2_params = os.path.join(my_robot_dir, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]

    nav2_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),

            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[nav2_params],
                remappings=[('cmd_vel', 'cmd_vel_raw')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                output='screen',
                parameters=[nav2_params],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                output='screen',
                parameters=[nav2_params],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                output='screen',
                parameters=[nav2_params],
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                output='screen',
                parameters=[nav2_params],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                output='screen',
                parameters=[nav2_params],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': lifecycle_nodes,
                }],
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        nav2_nodes,
    ])
