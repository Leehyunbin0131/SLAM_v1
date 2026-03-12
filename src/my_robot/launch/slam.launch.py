import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (EmitEvent, LogInfo, RegisterEventHandler,
                            TimerAction)
from launch.events import matches_action
from launch.event_handlers import OnProcessStart
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    my_robot_dir = get_package_share_directory('my_robot')

    ekf_config = os.path.join(my_robot_dir, 'config', 'ekf.yaml')
    slam_config = os.path.join(my_robot_dir, 'config', 'slam_params.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        output='screen',
    )

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config, {'use_sim_time': False}],
        output='screen',
        namespace='',
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    trigger_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_node,
            on_start=[
                TimerAction(period=2.0, actions=[configure_event]),
            ],
        )
    )

    trigger_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] slam_toolbox is activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    return LaunchDescription([
        ekf_node,
        slam_node,
        trigger_configure,
        trigger_activate,
    ])
