"""Four managed nodes that do nothing: `control`, `planner`, `mock1`, `mock2`.

They exist so there is something with a real ROS 2 lifecycle on the graph --
for the Orchestrator's `LifecycleManager` to drive, and for a person to drive
by hand from the mock bridge GUI. Each transition callback logs and returns
SUCCESS; no timers, no publishers, no subscriptions, no work of any kind.

That emptiness is the point. The real controller and planners do not exist in
this workspace yet (the Orchestrator's `config.yml` says as much next to the
names it expects), so anything these did would be invention -- which PLAN.md
forbids. What they faithfully reproduce is the *interface*: `get_state`,
`change_state` and `transition_event` under the node's own name, with the
standard state machine behind them, enforced by rcl rather than by us.

    ros2 run mock_bridge mock_bridge_control
    ros2 run mock_bridge mock_bridge_planner
    ros2 run mock_bridge mock_bridge_mock1
    ros2 run mock_bridge mock_bridge_mock2

`mock1` and `mock2` are the same node under different names -- spares, for
standing in for whatever the Orchestrator is made to manage next without
editing anything here.

Both are started by `mock_bridge.launch.py` alongside the GUI, which is the
normal way to get them.

**They are hardened against one rclpy bug**, because a mock that dies is
worse than useless. In Jazzy, `change_state` for a transition that is not
registered in the current state raises `RCLError` inside rclpy's own service
handler; the exception comes out of `executor.spin()` and takes the process
with it. (The Orchestrator's README records the same bug, and its driver
guards against it by checking client-side before sending -- as
`lifecycle_client` does.) `spin_resiliently` below catches it and keeps
spinning, so a stale button press costs the requester a timeout instead of
costing us the node.
"""

from __future__ import annotations

import sys
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn

try:  # The only place rclpy exposes it; RuntimeError is its base class.
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:  # pragma: no cover - if the private module ever moves
    RCLError = RuntimeError

from .lifecycle import (
    DEFAULT_CONTROL_NODE,
    DEFAULT_MOCK1_NODE,
    DEFAULT_MOCK2_NODE,
    DEFAULT_PLANNER_NODE,
)

# Where every node in the real stack says it is alive, and what fs_watchdog
# reads. Taken from camera_reader/lidar_reader and fs_uart_bridge, which all
# use this name.
MONITOR_TOPIC = '/monitor'
# Not a parameter, in the real nodes or here: the watchdog's timeouts and the
# Orchestrator's watchdog settings are all written for 1 Hz.
HEARTBEAT_HZ = 1.0


class DoNothingLifecycleNode(LifecycleNode):
    """A lifecycle node with no behaviour, and one heartbeat.

    Every callback returns SUCCESS, so the node always follows the transition
    it was asked for. It never fails or errors on purpose: a mock that refuses
    transitions would be testing the Orchestrator against a fault, and that is
    a separate exercise from testing it against a working stack.

    The one thing it does do is beat on `/monitor` at 1 Hz -- a
    `diagnostic_msgs/DiagnosticArray` carrying its own node name, exactly as
    the real readers and the real bridge do. Without it, `fs_watchdog` reports
    this node missing forever, and an Orchestrator that checks the watchdog
    before leaving AS_READY goes to AS_EMERGENCY instead.

    **The publisher is created in `configure` and the timer runs only while
    ACTIVE**, copied from camera_reader and lidar_reader. That is what makes
    "beating" mean "ACTIVE": a node that beat from startup would let the
    Orchestrator past a readiness check before anything had been activated,
    which would make this mock worse than the real thing rather than a stand-in
    for it.
    """

    def __init__(self, node_name: str, **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        # The heartbeat: publisher from `configure`, timer only while ACTIVE.
        # See the class docstring for why that split matters.
        self._heartbeat_pub = None
        self._heartbeat_timer = None
        # The node starts UNCONFIGURED, as the lifecycle requires. Nothing
        # here configures or activates it: that is the Orchestrator's job, or
        # the operator's from the GUI.
        self.get_logger().info(
            f"'{self.get_name()}' is up and unconfigured -- a placeholder "
            f'managed node that does nothing but follow its lifecycle'
        )

    # ------------------------------------------------------------------
    # Transitions. rcl has already checked that each is legal from the
    # current state, so there is nothing left for these to verify.
    # ------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # A plain publisher, not a lifecycle one: the heartbeat is this node's
        # own liveness, and the readers make the same choice for the same
        # reason. Its QoS is theirs too (depth 10, volatile) -- a retained
        # `alive` from a node that has since died is the wrong answer.
        self._heartbeat_pub = self.create_publisher(
            DiagnosticArray, MONITOR_TOPIC, 10)
        return self._done('configure', state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._heartbeat_timer = self.create_timer(
            1.0 / HEARTBEAT_HZ, self._beat)
        return self._done('activate', state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._stop_beating()
        return self._done('deactivate', state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._teardown_heartbeat()
        return self._done('cleanup', state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        # Reachable from any state, so it tears down whatever exists.
        self._teardown_heartbeat()
        return self._done('shutdown', state)

    def on_error(self, state: State) -> TransitionCallbackReturn:
        # Unreachable while the callbacks above all succeed, but a managed
        # entity is expected to have it, and silence here would be confusing
        # if it ever did run.
        return self._done('error', state)

    def _done(self, transition: str, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'{transition}: from {state.label} -- nothing to do')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # The heartbeat
    # ------------------------------------------------------------------

    def _beat(self) -> None:
        """One DiagnosticArray naming this node, as fs_watchdog expects.

        `DiagnosticStatus.name` is the node's RUNTIME name, so a node launched
        under another name (`name:=lidar_reader_node`) beats under that name
        without anything here knowing about it -- which is how this stands in
        for whichever nodes the Orchestrator is made to manage.
        """
        if self._heartbeat_pub is None:
            return

        status = DiagnosticStatus()
        status.name = self.get_name()
        status.level = DiagnosticStatus.OK
        status.message = 'alive'

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._heartbeat_pub.publish(message)

    def _stop_beating(self) -> None:
        if self._heartbeat_timer is not None:
            self.destroy_timer(self._heartbeat_timer)
            self._heartbeat_timer = None

    def _teardown_heartbeat(self) -> None:
        self._stop_beating()
        if self._heartbeat_pub is not None:
            self.destroy_publisher(self._heartbeat_pub)
            self._heartbeat_pub = None


def spin_resiliently(executor, logger) -> None:
    """Spin until shutdown, surviving rclpy's invalid-transition crash.

    See the module docstring: an invalid `change_state` request raises out of
    rclpy's service handler and through the executor. The state machine has
    not moved when that happens -- the transition was refused before any
    callback ran -- so there is nothing to repair; the request simply goes
    unanswered, and its sender sees a timeout.

    Any other exception is left alone: this is for the one bug, not a blanket
    `except Exception` around the whole node.
    """
    while executor.context.ok():
        try:
            executor.spin()
            return
        except (KeyboardInterrupt, ExternalShutdownException):
            return
        except RCLError as exc:
            logger.warning(
                f'refused a lifecycle transition request ({exc}) -- still up'
            )


def run(node_name: str, args: Optional[list[str]] = None) -> int:
    """Spin one do-nothing managed node until it is shut down."""
    rclpy.init(args=args)
    node = DoNothingLifecycleNode(node_name)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        spin_resiliently(executor, node.get_logger())
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
    return 0


def control_main(args: Optional[list[str]] = None) -> int:
    return run(DEFAULT_CONTROL_NODE, args)


def planner_main(args: Optional[list[str]] = None) -> int:
    return run(DEFAULT_PLANNER_NODE, args)


def mock1_main(args: Optional[list[str]] = None) -> int:
    return run(DEFAULT_MOCK1_NODE, args)


def mock2_main(args: Optional[list[str]] = None) -> int:
    return run(DEFAULT_MOCK2_NODE, args)


if __name__ == '__main__':
    sys.exit(control_main())
