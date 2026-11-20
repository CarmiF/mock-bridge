"""The managed nodes and the lifecycle client, against a real DDS graph.

What these cover:

  * the four do-nothing nodes really are ROS 2 lifecycle nodes -- the standard
    services, the standard state machine, and surviving the invalid request
    that kills a stock rclpy lifecycle node in Jazzy;
  * the client in the mock bridge node reports what they are in, picks up a
    transition made by anyone (not just by us), and drives one node without
    touching the other three.

Skipped where rclpy and fs_msgs are not on the path. Isolated on its own
ROS_DOMAIN_ID, and every test gets node names of its own: a name that outlives
its test on the DDS graph would have the next test's requests answered by a
node that is already being torn down.
"""

from __future__ import annotations

import itertools
import os
import threading
import time

import pytest

os.environ.setdefault('ROS_DOMAIN_ID', '77')

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('fs_msgs')

from lifecycle_msgs.msg import Transition as TransitionMsg  # noqa: E402
from lifecycle_msgs.srv import ChangeState, GetState  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

from mock_bridge.lifecycle import (  # noqa: E402
    LifecycleState,
    TransitionId,
    default_nodes,
)
from mock_bridge.lifecycle_nodes import (  # noqa: E402
    DoNothingLifecycleNode,
    spin_resiliently,
)
from mock_bridge.node import MockBridgeNode  # noqa: E402

#: The four slots, in the order `default_nodes` returns them.
SLOTS = ('control', 'planner', 'mock1', 'mock2')

# Long enough for DDS discovery plus the client's 1 s poll, short enough that
# a real failure does not look like a hang.
WAIT_S = 20.0

_RUN = itertools.count(1)


def wait_until(predicate, timeout: float = WAIT_S):
    """Spin the clock, not the executor -- that runs on its own thread."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.05)
    return predicate()


class Stack:
    """The four managed nodes and a mock bridge node, on one executor thread.

    One process and one context, because what is being tested is the ROS
    interface between them, not process boundaries: the client reaches the
    managed nodes through their services either way.

    `absent` names slots to leave without a process, which is how a node that
    is not running gets tested.
    """

    def __init__(self, absent: tuple[str, ...] = ()) -> None:
        self.events: list[str] = []
        self.changes: list = []

        run = next(_RUN)
        self.names = {slot: f'lc{run}_{slot}' for slot in SLOTS}
        self.control, self.planner, self.mock1, self.mock2 = (
            self.names[slot] for slot in SLOTS)

        self.context = rclpy.Context()
        self.context.init(args=[
            '--ros-args',
            # Kept off the real bridge's topic names, as in test_node.py.
            '-p', 'as_state_topic:=/mock_test/lc_as_state',
            '-p', 'mission_topic:=/mock_test/lc_mission',
            '-p', 'go_topic:=/mock_test/lc_go',
            '-p', 'override_topic:=/mock_test/lc_override',
            *[arg
              for slot in SLOTS
              for arg in ('-p', f'{slot}_node_name:={self.names[slot]}')],
        ])
        self.executor = SingleThreadedExecutor(context=self.context)

        self.managed = [
            DoNothingLifecycleNode(self.names[slot], context=self.context)
            for slot in SLOTS if slot not in absent
        ]
        for node in self.managed:
            self.executor.add_node(node)

        self.node = MockBridgeNode(
            context=self.context,
            on_event=self.events.append,
            on_lifecycle=self.changes.append,
        )
        self.executor.add_node(self.node)

        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self) -> None:
        try:
            spin_resiliently(self.executor, self.node.get_logger())
        except Exception:  # noqa: BLE001 - the context going away is normal
            pass

    def state(self, name: str) -> LifecycleState:
        return self.node.lifecycle.node(name).state

    def wait_for_state(self, name: str, state: LifecycleState) -> bool:
        return bool(wait_until(lambda: self.state(name) is state))

    def wait_for_all(self, state: LifecycleState) -> None:
        for name in self.names.values():
            assert self.wait_for_state(name, state), (
                f'{name} is {self.state(name).label}, not {state.label}')

    def warnings(self) -> list[str]:
        return [line for line in self.events if line.startswith('WARNING')]

    def close(self) -> None:
        self.node.shutdown()
        self.executor.shutdown()
        self.thread.join(timeout=3.0)
        for node in (self.node, *self.managed):
            node.destroy_node()
        if self.context.ok():
            self.context.try_shutdown()


@pytest.fixture
def stack():
    instance = Stack()
    try:
        yield instance
    finally:
        instance.close()


def test_every_node_is_discovered_unconfigured(stack):
    # Nothing configures them at startup -- that is the Orchestrator's job or
    # the operator's.
    stack.wait_for_all(LifecycleState.UNCONFIGURED)
    assert [m.name for m in stack.node.lifecycle.nodes()] == [
        stack.control, stack.planner, stack.mock1, stack.mock2]


def test_the_whole_walk_and_only_the_node_asked_for(stack):
    stack.wait_for_all(LifecycleState.UNCONFIGURED)

    for command, expected in (
        ('Configure', LifecycleState.INACTIVE),
        ('Activate', LifecycleState.ACTIVE),
        ('Deactivate', LifecycleState.INACTIVE),
        ('Cleanup', LifecycleState.UNCONFIGURED),
    ):
        stack.node.lifecycle.request(stack.control, command)
        assert stack.wait_for_state(stack.control, expected), (
            f'{command} did not reach {expected.label}; '
            f'now {stack.state(stack.control).label}')

    # The other three were never asked for anything and never moved.
    for name in (stack.planner, stack.mock1, stack.mock2):
        assert stack.state(name) is LifecycleState.UNCONFIGURED, name
    assert not stack.warnings()

    # Every change was announced to the GUI, with the node it belongs to.
    assert {change.name for change in stack.changes} == set(stack.names.values())
    assert any(
        change.name == stack.control and change.state is LifecycleState.ACTIVE
        for change in stack.changes)


def test_the_spares_are_driven_like_the_others(stack):
    """mock1 and mock2 are ordinary managed nodes, not a special case."""
    stack.wait_for_all(LifecycleState.UNCONFIGURED)

    stack.node.lifecycle.request(stack.mock1, 'Configure')
    assert stack.wait_for_state(stack.mock1, LifecycleState.INACTIVE)
    stack.node.lifecycle.request(stack.mock1, 'Activate')
    assert stack.wait_for_state(stack.mock1, LifecycleState.ACTIVE)

    assert stack.state(stack.mock2) is LifecycleState.UNCONFIGURED
    assert stack.state(stack.control) is LifecycleState.UNCONFIGURED
    assert not stack.warnings()


def test_shutdown_finalizes_and_then_nothing_is_offered(stack):
    assert stack.wait_for_state(stack.control, LifecycleState.UNCONFIGURED)

    stack.node.lifecycle.request(stack.control, 'Shutdown')
    assert stack.wait_for_state(stack.control, LifecycleState.FINALIZED)
    assert stack.node.lifecycle.node(stack.control).commands() == ()
    # The process is still there; it is the state machine that is finished.
    assert stack.state(stack.planner) is LifecycleState.UNCONFIGURED


def test_a_command_the_state_does_not_allow_is_not_sent(stack):
    assert stack.wait_for_state(stack.control, LifecycleState.UNCONFIGURED)

    # The GUI greys these out; asking anyway must not move the node.
    stack.node.lifecycle.request(stack.control, 'Activate')
    assert wait_until(lambda: stack.warnings())
    assert stack.state(stack.control) is LifecycleState.UNCONFIGURED
    assert 'Activate is not available' in stack.warnings()[0]


def test_an_unknown_name_is_refused_outright(stack):
    with pytest.raises(KeyError):
        stack.node.lifecycle.request('no_such_node', 'Configure')
    with pytest.raises(KeyError):
        stack.node.lifecycle.request(stack.control, 'Teleport')


def test_a_name_with_no_node_behind_it_reads_unknown():
    stack = Stack(absent=('planner',))
    try:
        assert stack.wait_for_state(stack.control, LifecycleState.UNCONFIGURED)
        assert stack.wait_for_state(stack.mock1, LifecycleState.UNCONFIGURED)
        absent = stack.node.lifecycle.node(stack.planner)
        assert absent.state is LifecycleState.UNKNOWN
        # Said plainly, rather than shown as some plausible state.
        assert 'no lifecycle services' in absent.status_text
    finally:
        stack.close()


def test_a_transition_made_by_someone_else_shows_up(stack):
    """The Orchestrator drives these nodes too -- the GUI must follow it."""
    assert stack.wait_for_state(stack.control, LifecycleState.UNCONFIGURED)

    other = Node('lc_other_manager', context=stack.context)
    stack.executor.add_node(other)
    try:
        client = other.create_client(
            ChangeState, f'/{stack.control}/change_state')
        assert client.wait_for_service(timeout_sec=WAIT_S)
        request = ChangeState.Request()
        request.transition = TransitionMsg(id=int(TransitionId.CONFIGURE))
        client.call_async(request)

        assert stack.wait_for_state(stack.control, LifecycleState.INACTIVE)
    finally:
        stack.executor.remove_node(other)
        other.destroy_node()


def test_an_illegal_request_does_not_kill_the_managed_node():
    """The rclpy Jazzy bug, and the one thing that can be done about it.

    An invalid `change_state` raises inside rclpy's own service handler, so
    the request goes unanswered -- but the node must still be there
    afterwards, and the state machine must not have moved. No mock bridge
    node here: this is about the managed node alone.
    """
    name = f'lc{next(_RUN)}_alone'
    context = rclpy.Context()
    context.init()
    executor = SingleThreadedExecutor(context=context)
    managed = DoNothingLifecycleNode(name, context=context)
    client_node = Node(f'{name}_client', context=context)
    executor.add_node(managed)
    executor.add_node(client_node)
    thread = threading.Thread(
        target=spin_resiliently,
        args=(executor, managed.get_logger()),
        daemon=True,
    )
    thread.start()
    try:
        change = client_node.create_client(
            ChangeState, f'/{name}/change_state')
        get = client_node.create_client(GetState, f'/{name}/get_state')
        assert change.wait_for_service(timeout_sec=WAIT_S)
        assert get.wait_for_service(timeout_sec=WAIT_S)

        def call(client, request, timeout=WAIT_S):
            future = client.call_async(request)
            wait_until(lambda: future.done(), timeout)
            return future.result() if future.done() else None

        def transition(transition_id):
            request = ChangeState.Request()
            request.transition = TransitionMsg(id=int(transition_id))
            return request

        assert call(get, GetState.Request()).current_state.id == int(
            LifecycleState.UNCONFIGURED)

        # Activate is not registered in unconfigured. rclpy raises instead
        # of answering, so there is no reply at all.
        assert call(change, transition(TransitionId.ACTIVATE),
                    timeout=3.0) is None

        # The node is still up, still unconfigured, and still works.
        assert thread.is_alive()
        assert call(get, GetState.Request()).current_state.id == int(
            LifecycleState.UNCONFIGURED)
        assert call(change, transition(TransitionId.CONFIGURE)).success
        assert call(get, GetState.Request()).current_state.id == int(
            LifecycleState.INACTIVE)
    finally:
        executor.shutdown()
        thread.join(timeout=3.0)
        managed.destroy_node()
        client_node.destroy_node()
        if context.ok():
            context.try_shutdown()


def test_default_names_are_what_the_launch_file_starts():
    # Every test renames them for isolation; these are the real defaults.
    assert [m.name for m in default_nodes()] == [
        'control', 'planner', 'mock1', 'mock2']
