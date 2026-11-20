"""The mock bridge ROS node (PLAN.md implementation step 2).

Three publishers driven by one 100 ms timer that sends only the channels the
state model has selected, plus the single read-only feedback subscriber.

It also owns the lifecycle client (`lifecycle_client.LifecycleClient`), which
watches the four managed nodes -- `control`, `planner`, `mock1` and `mock2` --
and transitions them when the GUI asks. That is a second, unrelated job in one node on
purpose: it is one more set of endpoints on the graph, and the GUI then has a
single collaborator to talk to and a single thread boundary to cross.

**Manual only.** Nothing here reacts to feedback, emulates a VCU, validates a
combination or changes a channel on its own. The only thing that changes an
input is a `click()` from the GUI.

Threading: the GUI thread calls `click()` / `snapshot()` / `stop_all()` while
the ROS executor thread runs the timer and the subscription callback. One lock
guards the state model and the latest feedback, held only for the few lines
that touch them -- never across a `publish()`, and never while a listener
runs.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from rclpy.node import Node

from .channels import ChannelId, ChannelValue
from .lifecycle import (
    DEFAULT_CONTROL_NODE,
    DEFAULT_MOCK1_NODE,
    DEFAULT_MOCK2_NODE,
    DEFAULT_PLANNER_NODE,
    ManagedNode,
    default_nodes,
)
from .lifecycle_client import LifecycleClient
from .ros_interface import (
    DEFAULT_AS_STATE_TOPIC,
    DEFAULT_GO_TOPIC,
    DEFAULT_MISSION_TOPIC,
    DEFAULT_OVERRIDE_TOPIC,
    FEEDBACK_TYPE,
    MESSAGE_TYPES,
    PUBLISH_PERIOD_S,
    PUBLISH_RATE_HZ,
    describe_as_state,
    feedback_qos,
    state_qos,
    verify_as_state_numbering,
)
from .state import BridgeState, ClickResult

NODE_NAME = 'mock_bridge'

# How long to let DDS discovery settle before looking for another publisher
# on our own topics.
RIVAL_CHECK_DELAY_S = 3.0


@dataclass(frozen=True)
class Feedback:
    """One received Orchestrator State / Override message, for display only.

    `received_at` is wall-clock seconds, because this is shown to a person
    rather than used for a timeout.
    """

    state: int
    label: str
    received_at: float
    count: int

    @property
    def timestamp_text(self) -> str:
        return time.strftime('%H:%M:%S', time.localtime(self.received_at))

    def describe(self) -> str:
        return f'{self.label} ({self.state}) at {self.timestamp_text}'


class MockBridgeNode(Node):
    """Publishes the selected channels at 10 Hz; displays Orchestrator feedback.

    `on_event` and `on_feedback`, if given, are called **from the ROS executor
    thread**: a GUI must marshal them onto its own thread (stage 3).
    """

    def __init__(
        self,
        state: Optional[BridgeState] = None,
        on_event: Optional[Callable[[str], None]] = None,
        on_feedback: Optional[Callable[[Feedback], None]] = None,
        on_lifecycle: Optional[Callable[[ManagedNode], None]] = None,
        context=None,
    ) -> None:
        # context is passed through so a caller can run the node on its own
        # rclpy context instead of the global one (see runner.RosRunner).
        super().__init__(NODE_NAME, context=context)

        # A renumbering between channels.py and fs_msgs is a startup failure,
        # not a wrong value on the wire.
        verify_as_state_numbering()

        self._state = state if state is not None else BridgeState()
        self._lock = threading.Lock()
        self._on_event = on_event
        self._on_feedback = on_feedback
        self._feedback: Optional[Feedback] = None
        self._feedback_count = 0

        topics = {
            ChannelId.AS_STATE: self.declare_parameter(
                'as_state_topic', DEFAULT_AS_STATE_TOPIC).value,
            ChannelId.MISSION: self.declare_parameter(
                'mission_topic', DEFAULT_MISSION_TOPIC).value,
            ChannelId.GO: self.declare_parameter(
                'go_topic', DEFAULT_GO_TOPIC).value,
        }
        override_topic = self.declare_parameter(
            'override_topic', DEFAULT_OVERRIDE_TOPIC).value

        # All three publishers exist for the node's lifetime; what changes with
        # a click is only whether the timer sends anything on them. Creating
        # and destroying them per selection would churn discovery and, with
        # TRANSIENT_LOCAL, change what a late subscriber is given.
        self._publishers_by_channel = {
            channel_id: self.create_publisher(
                MESSAGE_TYPES[channel_id], topic, state_qos())
            for channel_id, topic in topics.items()
        }

        # Display only. Never acted on -- see the module docstring.
        self._feedback_sub = self.create_subscription(
            FEEDBACK_TYPE, override_topic, self._on_override, feedback_qos())

        self._timer = self.create_timer(PUBLISH_PERIOD_S, self._publish_tick)

        # PLAN.md: never run the real bridge and the mock as competing
        # publishers. Nothing here can enforce that -- it would mean refusing
        # to start on a graph this node cannot judge -- so it says so loudly,
        # once, after discovery has had a moment. Checked on a one-shot timer
        # because at construction no remote endpoint is known yet.
        self._topics_by_channel = topics
        self._rival_timer = self.create_timer(
            RIVAL_CHECK_DELAY_S, self._warn_about_rival_publishers)

        # The managed nodes. Names are parameters because the Orchestrator's
        # config.yml expects its own (`control_node`,
        # `skidpad_planner_node`, ...) and those are not settled yet; see
        # lifecycle.py.
        self.lifecycle = LifecycleClient(
            self,
            managed=default_nodes(
                self.declare_parameter(
                    'control_node_name', DEFAULT_CONTROL_NODE).value,
                self.declare_parameter(
                    'planner_node_name', DEFAULT_PLANNER_NODE).value,
                self.declare_parameter(
                    'mock1_node_name', DEFAULT_MOCK1_NODE).value,
                self.declare_parameter(
                    'mock2_node_name', DEFAULT_MOCK2_NODE).value,
            ),
            on_event=self._log_event,
            on_change=on_lifecycle,
        )

        self.get_logger().info(
            f'manual mock bridge: out at {PUBLISH_RATE_HZ:g} Hz on '
            f'{topics[ChannelId.AS_STATE]}, {topics[ChannelId.MISSION]}, '
            f'{topics[ChannelId.GO]} while selected; '
            f'in (display only): {override_topic}'
        )
        self.get_logger().info('no channel selected -- publishing nothing')

    # ------------------------------------------------------------------
    # Inputs: called from the GUI thread
    # ------------------------------------------------------------------

    def click(self, channel_id: ChannelId, label: str) -> ClickResult:
        """Apply a button click and log what it did.

        The 10 Hz timer picks the change up on its next tick, so a switch
        never sends the old value again.
        """
        with self._lock:
            result = self._state.click(channel_id, label)
        self._log_event(result.describe())
        return result

    def snapshot(self) -> dict[ChannelId, ChannelValue]:
        """What is being published right now, for the GUI to render."""
        with self._lock:
            return self._state.publishing()

    @property
    def state(self) -> BridgeState:
        """The state model itself, for reading channel specs and selections.

        Mutate it only through `click()` / `stop_all()`, which hold the lock.
        """
        return self._state

    def stop_all(self) -> list[ClickResult]:
        """Stop every publishing channel, as on window close."""
        with self._lock:
            results = self._state.stop_all()
        for result in results:
            self._log_event(result.describe())
        return results

    # ------------------------------------------------------------------
    # Feedback: read-only
    # ------------------------------------------------------------------

    def latest_feedback(self) -> Optional[Feedback]:
        with self._lock:
            return self._feedback

    def _on_override(self, msg) -> None:
        """Record the latest Orchestrator State / Override. Changes no input."""
        with self._lock:
            self._feedback_count += 1
            previous = self._feedback
            # std_msgs/UInt8 calls it `data`; the old fs_msgs/ASOverride
            # called it `state`. See ros_interface for why this changed.
            feedback = Feedback(
                state=int(msg.data),
                label=describe_as_state(int(msg.data)),
                received_at=time.time(),
                count=self._feedback_count,
            )
            self._feedback = feedback

        # Log changed values only -- the Orchestrator publishes every tick.
        if previous is None or previous.state != feedback.state:
            self._log_event(f'orchestrator state/override: {feedback.describe()}')

        if self._on_feedback is not None:
            self._on_feedback(feedback)

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------

    def _publish_tick(self) -> None:
        """Send one message per selected channel. Unselected channels: nothing.

        Not even an empty message, an OFF, a NONE or a False -- a stopped
        channel is absent from `publishing()`, so there is no message to send.
        """
        with self._lock:
            publishing = self._state.publishing()

        for channel_id, value in publishing.items():
            message = MESSAGE_TYPES[channel_id](data=value.payload)
            self._publishers_by_channel[channel_id].publish(message)

    def _warn_about_rival_publishers(self) -> None:
        """Say so if something else publishes our topics. Runs once."""
        self.destroy_timer(self._rival_timer)
        self._rival_timer = None

        for channel_id, topic in self._topics_by_channel.items():
            others = self.count_publishers(topic) - 1  # minus our own
            if others > 0:
                self._log_event(
                    f'WARNING: {others} other publisher(s) on {topic} -- the '
                    f'real bridge and the mock must not both publish; the '
                    f'Orchestrator will see interleaved values'
                )

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        """Stop the timer and the publishers cleanly; safe to call twice."""
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None
        if self._rival_timer is not None:
            self.destroy_timer(self._rival_timer)
            self._rival_timer = None
        # Stops polling. The managed nodes are left in whatever state they
        # are in -- see LifecycleClient.shutdown.
        self.lifecycle.shutdown()
        self.stop_all()

    def _log_event(self, line: str) -> None:
        if line.startswith('WARNING'):
            self.get_logger().warning(line)
        else:
            self.get_logger().info(line)
        if self._on_event is not None:
            self._on_event(line)
