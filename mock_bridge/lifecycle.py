"""The managed-node model: what state each lifecycle node is in, and what
transitions are offered from there.

Pure Python, like `state.py`: no rclpy, no Qt, no lifecycle_msgs. The numbers
and labels below mirror `lifecycle_msgs/msg/State` and
`lifecycle_msgs/msg/Transition`, and `lifecycle_client.verify_numbering()`
fails at startup if they ever drift apart -- the same arrangement as
`channels.py` and `ros_interface.verify_as_state_numbering()`.

Four nodes are managed: a controller, a planner, and two spares. They do
nothing; they exist so the Orchestrator has something to drive, and so a
person can drive them by hand from the GUI. The real Orchestrator's
`config.yml` names the nodes it will activate (`control_node`,
`skidpad_planner_node`, `acceleration_planner_node`, `lidar_reader_node`,
`camera_reader_node`) and notes that none of them exist in the workspace yet
-- which is why the names here are ROS parameters rather than constants, and
why there are two spares to point at whichever of them a test needs.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import IntEnum
from typing import Mapping, Optional

# The node names the GUI starts with. Overridable per run -- see
# `node.MockBridgeNode`'s `control_node_name` / `planner_node_name`
# parameters and the launch arguments of the same name.
DEFAULT_CONTROL_NODE = 'control'
DEFAULT_PLANNER_NODE = 'planner'
DEFAULT_MOCK1_NODE = 'mock1'
DEFAULT_MOCK2_NODE = 'mock2'


class LifecycleState(IntEnum):
    """`lifecycle_msgs/msg/State`'s ids, with its labels.

    The four primary states plus the six transition states. UNKNOWN is also
    what this package uses for "we have not been able to ask" -- a node that
    is not running has no state to report, and saying `unknown` is honest
    where showing `unconfigured` would be a guess.
    """

    UNKNOWN = 0
    UNCONFIGURED = 1
    INACTIVE = 2
    ACTIVE = 3
    FINALIZED = 4
    CONFIGURING = 10
    CLEANINGUP = 11
    SHUTTINGDOWN = 12
    ACTIVATING = 13
    DEACTIVATING = 14
    ERRORPROCESSING = 15

    @property
    def label(self) -> str:
        """The lower-case label ROS itself prints, e.g. `ros2 lifecycle get`."""
        return self.name.lower()

    @property
    def is_primary(self) -> bool:
        return self in _PRIMARY_STATES


_PRIMARY_STATES = frozenset({
    LifecycleState.UNCONFIGURED,
    LifecycleState.INACTIVE,
    LifecycleState.ACTIVE,
    LifecycleState.FINALIZED,
})


def state_from_id(state_id: int) -> LifecycleState:
    """A `State.id` off the wire, or UNKNOWN if it is not one we know."""
    try:
        return LifecycleState(int(state_id))
    except ValueError:
        return LifecycleState.UNKNOWN


class TransitionId(IntEnum):
    """`lifecycle_msgs/msg/Transition`'s externally requestable ids.

    Only these are listed: the `TRANSITION_ON_*` ids above 9 are what a node
    reports about its own callbacks, never something a client asks for.
    """

    CREATE = 0
    CONFIGURE = 1
    CLEANUP = 2
    ACTIVATE = 3
    DEACTIVATE = 4
    UNCONFIGURED_SHUTDOWN = 5
    INACTIVE_SHUTDOWN = 6
    ACTIVE_SHUTDOWN = 7
    DESTROY = 8


@dataclass(frozen=True)
class LifecycleCommand:
    """One button: a label, and which transition it means from which state.

    `transitions` is keyed by state because the lifecycle has no single
    "shutdown" transition -- there is one per primary state, and asking for
    the wrong one is refused. A state that is not a key means the command is
    not available there, which is also what greys the button out.
    """

    label: str
    transitions: Mapping[LifecycleState, TransitionId]

    def transition_from(self, state: LifecycleState) -> Optional[TransitionId]:
        return self.transitions.get(state)

    def is_available(self, state: LifecycleState) -> bool:
        return state in self.transitions


CONFIGURE = LifecycleCommand('Configure', {
    LifecycleState.UNCONFIGURED: TransitionId.CONFIGURE,
})
ACTIVATE = LifecycleCommand('Activate', {
    LifecycleState.INACTIVE: TransitionId.ACTIVATE,
})
DEACTIVATE = LifecycleCommand('Deactivate', {
    LifecycleState.ACTIVE: TransitionId.DEACTIVATE,
})
CLEANUP = LifecycleCommand('Cleanup', {
    LifecycleState.INACTIVE: TransitionId.CLEANUP,
})
SHUTDOWN = LifecycleCommand('Shutdown', {
    LifecycleState.UNCONFIGURED: TransitionId.UNCONFIGURED_SHUTDOWN,
    LifecycleState.INACTIVE: TransitionId.INACTIVE_SHUTDOWN,
    LifecycleState.ACTIVE: TransitionId.ACTIVE_SHUTDOWN,
})

#: In GUI order. Shutdown is last because it is the one that cannot be undone
#: -- a finalized node has to be restarted as a process.
COMMANDS: tuple[LifecycleCommand, ...] = (
    CONFIGURE, ACTIVATE, DEACTIVATE, CLEANUP, SHUTDOWN)

COMMANDS_BY_LABEL = {command.label: command for command in COMMANDS}


@dataclass(frozen=True)
class ManagedNode:
    """What is known about one lifecycle node. Frozen: updates replace it.

    Frozen because this crosses threads -- the ROS executor learns the new
    state and the GUI thread draws it -- and swapping one immutable value for
    another under the client's lock means the GUI can never read a half-written
    one.
    """

    name: str
    title: str
    state: LifecycleState = LifecycleState.UNKNOWN
    detail: str = 'not seen yet'

    @property
    def status_text(self) -> str:
        if self.state is LifecycleState.UNKNOWN:
            return f'unknown -- {self.detail}' if self.detail else 'unknown'
        return f'{self.state.label.upper()} [{int(self.state)}]'

    @property
    def is_active(self) -> bool:
        return self.state is LifecycleState.ACTIVE

    @property
    def is_reachable(self) -> bool:
        """Whether the last thing we heard was an actual state."""
        return self.state is not LifecycleState.UNKNOWN

    def allows(self, command: LifecycleCommand) -> bool:
        return command.is_available(self.state)

    def commands(self) -> tuple[LifecycleCommand, ...]:
        """The commands available from this state, in GUI order."""
        return tuple(c for c in COMMANDS if c.is_available(self.state))

    def with_state(self, state: LifecycleState, detail: str = '') -> 'ManagedNode':
        return replace(self, state=state, detail=detail)


def default_nodes(
    control_name: str = DEFAULT_CONTROL_NODE,
    planner_name: str = DEFAULT_PLANNER_NODE,
    mock1_name: str = DEFAULT_MOCK1_NODE,
    mock2_name: str = DEFAULT_MOCK2_NODE,
) -> tuple[ManagedNode, ...]:
    """The four managed nodes, in the order the GUI shows them.

    Control and planner first, because those are the two the Orchestrator's
    state machine activates by name; the two spares after them.
    """
    return (
        ManagedNode(name=control_name, title='Control'),
        ManagedNode(name=planner_name, title='Planner'),
        ManagedNode(name=mock1_name, title='Mock 1'),
        ManagedNode(name=mock2_name, title='Mock 2'),
    )


def describe_change(node: ManagedNode, previous: Optional[ManagedNode]) -> str:
    """One event-log line for a state change."""
    before = previous.status_text if previous is not None else 'nothing'
    return f'lifecycle {node.name}: {before} -> {node.status_text}'


def service_names(node_name: str) -> tuple[str, str, str]:
    """The three interfaces a managed node exposes, as rcl names them.

    `/<node>/get_state`, `/<node>/change_state` and `/<node>/transition_event`
    -- the same construction the Orchestrator's `LifecycleManager` uses, so
    the mock and the real manager talk to exactly the same endpoints.
    """
    target = node_name if node_name.startswith('/') else '/' + node_name
    return (
        f'{target}/get_state',
        f'{target}/change_state',
        f'{target}/transition_event',
    )


__all__ = [
    'ACTIVATE',
    'DEFAULT_MOCK1_NODE',
    'DEFAULT_MOCK2_NODE',
    'CLEANUP',
    'COMMANDS',
    'COMMANDS_BY_LABEL',
    'CONFIGURE',
    'DEACTIVATE',
    'DEFAULT_CONTROL_NODE',
    'DEFAULT_PLANNER_NODE',
    'LifecycleCommand',
    'LifecycleState',
    'ManagedNode',
    'SHUTDOWN',
    'TransitionId',
    'default_nodes',
    'describe_change',
    'service_names',
    'state_from_id',
]
