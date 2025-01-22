from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.events import matches_action
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg
import os

def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'calculator_file',
            default_value=PathJoinSubstitution([FindPackageShare('codes'),
                                                'params', 'calculator.yaml']),
            description='Full path to the file with all parameters.'
        )
    )

    # Initialize arguments
    calculator_file = LaunchConfiguration('calculator_file')

    # Define the lifecycle node (MainNode)
    calculator_lifecycle_node = LifecycleNode(
    package='codes',  # Nome do pacote
    executable='codes_main',  # Nome correto do executável
    name='calculator',
    namespace='',
    output='screen',
    parameters=[calculator_file],
    remappings=[]
)

    event_handlers = []

    # Right after the node starts, make it take the 'configure' transition
    event_handlers.append(
        RegisterEventHandler(
            OnProcessStart(
                target_action=calculator_lifecycle_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(calculator_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
    )

    # Transition from 'configuring' to 'inactive' then to 'active'
    event_handlers.append(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=calculator_lifecycle_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(calculator_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        ),
    )

    ld = LaunchDescription()

    # Declare the arguments
    for argument in declared_arguments:
        ld.add_action(argument)

    # Add client node
    ld.add_action(calculator_lifecycle_node)

    # Add event handlers
    for event_handler in event_handlers:
        ld.add_action(event_handler)

    return ld
