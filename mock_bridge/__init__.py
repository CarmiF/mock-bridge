"""ROS 2 Mock Bridge for testing the real Orchestrator without the vehicle.

PLAN.md steps 1 to 5: the input state model (`channels`, `state`), the ROS
layer (`ros_interface`, `node`, `runner`), the PySide6 GUI (`gui`) and the
Docker packaging. `headless` is the same node driven from the keyboard, for a
terminal-only host or a scripted check.

Beyond PLAN.md: four managed nodes that do nothing but follow their ROS 2
lifecycle (`lifecycle_nodes`), the model of their states and transitions
(`lifecycle`) and the client that watches and drives them
(`lifecycle_client`), so the Orchestrator's lifecycle management has
something to act on and the GUI shows what state each node is in.

This module deliberately re-exports only the ROS-free half, so the state model
and its tests keep working without rclpy on the path. Import `mock_bridge.node`
or `mock_bridge.runner` directly for the ROS side.
"""

from .channels import (
    AS_STATE_SPEC,
    CHANNEL_SPECS,
    GO_SPEC,
    MISSION_SPEC,
    SPEC_BY_ID,
    ChannelId,
    ChannelSpec,
    ChannelValue,
)
from .lifecycle import (
    COMMANDS,
    DEFAULT_CONTROL_NODE,
    DEFAULT_MOCK1_NODE,
    DEFAULT_MOCK2_NODE,
    DEFAULT_PLANNER_NODE,
    LifecycleCommand,
    LifecycleState,
    ManagedNode,
    TransitionId,
    default_nodes,
)
from .state import BridgeState, Channel, ClickAction, ClickResult

__all__ = [
    'AS_STATE_SPEC',
    'BridgeState',
    'CHANNEL_SPECS',
    'COMMANDS',
    'Channel',
    'ChannelId',
    'ChannelSpec',
    'ChannelValue',
    'ClickAction',
    'ClickResult',
    'DEFAULT_CONTROL_NODE',
    'DEFAULT_MOCK1_NODE',
    'DEFAULT_MOCK2_NODE',
    'DEFAULT_PLANNER_NODE',
    'GO_SPEC',
    'LifecycleCommand',
    'LifecycleState',
    'MISSION_SPEC',
    'ManagedNode',
    'SPEC_BY_ID',
    'TransitionId',
    'default_nodes',
]
