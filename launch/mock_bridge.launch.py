"""Launch the mock bridge GUI and the two managed lifecycle nodes.

    ros2 launch mock_bridge mock_bridge.launch.py

Five processes: the GUI, and `control`, `planner`, `mock1` and `mock2` -- four
lifecycle nodes that do nothing but follow their state machine, so the
Orchestrator has something to drive and the GUI has something to show. Their
node names are arguments, because the Orchestrator's config.yml expects names
of its own (`control_node`, `skidpad_planner_node`, ...) that are not settled
yet:

    ros2 launch mock_bridge mock_bridge.launch.py \\
        control_node_name:=control_node planner_node_name:=skidpad_planner_node

The four topic names default to the real bridge's, which is what makes the
mock a drop-in replacement. Override them to run the mock somewhere harmless:

    ros2 launch mock_bridge mock_bridge.launch.py \\
        as_state_topic:=/scratch/as_state

!! Never run this against a graph where the real fs_uart_bridge is also
publishing. The node warns a few seconds after startup if it finds another
publisher on its topics, but it cannot stop you.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Kept in step with mock_bridge/ros_interface.py, which explains where each
# name came from. Spelled out here rather than imported: a launch file is read
# by people deciding what to run, and importing the package to find out what
# it will publish helps nobody.
_DEFAULTS = {
    'as_state_topic': '/detached_sensor/as_state',
    'mission_topic': '/detached_sensor/mission',
    'go_topic': '/detached_sensor/go',
    'override_topic': '/vehicle/as_override',
}

# The managed nodes. Kept in step with mock_bridge/lifecycle.py's
# DEFAULT_CONTROL_NODE / DEFAULT_PLANNER_NODE, and spelled out here for the
# same reason as the topics above.
_MANAGED = {
    'control_node_name': 'control',
    'planner_node_name': 'planner',
    'mock1_node_name': 'mock1',
    'mock2_node_name': 'mock2',
}


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(
            name,
            default_value=default,
            description=f'Topic for {name.removesuffix("_topic")}.',
        )
        for name, default in _DEFAULTS.items()
    ]
    arguments += [
        DeclareLaunchArgument(
            name,
            default_value=default,
            description=(
                f'ROS node name for the managed '
                f'{name.removesuffix("_node_name")} node.'
            ),
        )
        for name, default in _MANAGED.items()
    ]

    gui = Node(
        package='mock_bridge',
        executable='mock_bridge_gui',
        name='mock_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            name: LaunchConfiguration(name)
            for name in (*_DEFAULTS, *_MANAGED)
        }],
    )

    # Separate processes, as they would be in the real stack -- the GUI drives
    # them over their lifecycle services, not by calling into them.
    managed = [
        Node(
            package='mock_bridge',
            executable=executable,
            name=LaunchConfiguration(argument),
            output='screen',
            emulate_tty=True,
        )
        for executable, argument in (
            ('mock_bridge_control', 'control_node_name'),
            ('mock_bridge_planner', 'planner_node_name'),
            ('mock_bridge_mock1', 'mock1_node_name'),
            ('mock_bridge_mock2', 'mock2_node_name'),
        )
    ]

    return LaunchDescription([*arguments, *managed, gui])
