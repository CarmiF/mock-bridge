"""The ROS layer, against a real DDS graph (PLAN.md test scenarios 1, 2, 4, 6, 7).

Skipped where rclpy and fs_msgs are not on the path. Run them with the
workspace sourced:

    source /opt/ros/jazzy/setup.bash
    source <pedal_node workspace>/install/setup.bash
    python3 -m pytest tests -q

Isolated from any running stack by its own ROS_DOMAIN_ID and by topic names
under /mock_test, passed as the node's own parameters -- which is also what
proves those parameters are wired.
"""

from __future__ import annotations

import os
import threading
import time

import pytest

# A domain of its own, so these tests never see or feed a running stack.
# Read when the context is initialised, so it must be set before that.
os.environ.setdefault('ROS_DOMAIN_ID', '77')

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('fs_msgs')

from fs_msgs.msg import ASOverride           # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node                  # noqa: E402
from std_msgs.msg import Bool, UInt8         # noqa: E402

from mock_bridge import ChannelId            # noqa: E402
from mock_bridge.ros_interface import (      # noqa: E402
    PUBLISH_PERIOD_S,
    feedback_qos,
    state_qos,
)
from mock_bridge.node import RIVAL_CHECK_DELAY_S  # noqa: E402
from mock_bridge.runner import RosRunner     # noqa: E402

AS_STATE_TOPIC = '/mock_test/as_state'
MISSION_TOPIC = '/mock_test/mission'
GO_TOPIC = '/mock_test/go'
OVERRIDE_TOPIC = '/mock_test/as_override'

NODE_ARGS = [
    '--ros-args',
    '-p', f'as_state_topic:={AS_STATE_TOPIC}',
    '-p', f'mission_topic:={MISSION_TOPIC}',
    '-p', f'go_topic:={GO_TOPIC}',
    '-p', f'override_topic:={OVERRIDE_TOPIC}',
]

# Several publish ticks, with room for DDS delivery.
SETTLE_S = 10 * PUBLISH_PERIOD_S


class Listener:
    """Stands in for the Orchestrator: counts what the mock publishes.

    It subscribes with the same QoS the Orchestrator uses, and it also
    publishes the feedback topic the Orchestrator owns.
    """

    def __init__(self) -> None:
        self.context = rclpy.Context()
        self.context.init()
        self.node = Node('mock_test_listener', context=self.context)
        self.received: dict[ChannelId, list] = {
            ChannelId.AS_STATE: [],
            ChannelId.MISSION: [],
            ChannelId.GO: [],
        }
        self._lock = threading.Lock()
        subscriptions = (
            (ChannelId.AS_STATE, UInt8, AS_STATE_TOPIC),
            (ChannelId.MISSION, UInt8, MISSION_TOPIC),
            (ChannelId.GO, Bool, GO_TOPIC),
        )
        for channel_id, msg_type, topic in subscriptions:
            self.node.create_subscription(
                msg_type, topic,
                self._recorder(channel_id), state_qos())
        self.override_pub = self.node.create_publisher(
            UInt8, OVERRIDE_TOPIC, feedback_qos())

        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _recorder(self, channel_id: ChannelId):
        def callback(msg) -> None:
            with self._lock:
                self.received[channel_id].append(msg.data)
        return callback

    def counts(self) -> dict[ChannelId, int]:
        with self._lock:
            return {key: len(values) for key, values in self.received.items()}

    def values(self, channel_id: ChannelId) -> list:
        with self._lock:
            return list(self.received[channel_id])

    def stop(self) -> None:
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        self.context.try_shutdown()


@pytest.fixture
def listener():
    listener = Listener()
    # Subscribe before the publishers exist, so nothing is missed and no
    # retained TRANSIENT_LOCAL sample is counted as a live publication.
    time.sleep(0.2)
    yield listener
    listener.stop()


@pytest.fixture
def node(listener):
    with RosRunner(args=NODE_ARGS) as runner:
        time.sleep(0.3)  # discovery
        yield runner.node


def wait_for(predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


# Scenario 1: zero input messages until the first click.
def test_silent_until_first_click(node, listener):
    time.sleep(SETTLE_S)
    assert listener.counts() == {
        ChannelId.AS_STATE: 0,
        ChannelId.MISSION: 0,
        ChannelId.GO: 0,
    }


# Scenario 2: continuous AS_OFF, then no further AS state messages.
def test_select_publishes_continuously_then_stops(node, listener):
    node.click(ChannelId.AS_STATE, 'AS_OFF')
    assert wait_for(lambda: listener.counts()[ChannelId.AS_STATE] >= 3)
    assert set(listener.values(ChannelId.AS_STATE)) == {0}

    node.click(ChannelId.AS_STATE, 'AS_OFF')
    time.sleep(SETTLE_S)
    settled = listener.counts()[ChannelId.AS_STATE]
    time.sleep(SETTLE_S)
    assert listener.counts()[ChannelId.AS_STATE] == settled

    # And nothing was sent on the other two channels along the way.
    assert listener.counts()[ChannelId.MISSION] == 0
    assert listener.counts()[ChannelId.GO] == 0


# Publishing is 10 Hz: roughly SETTLE_S / PUBLISH_PERIOD_S messages.
def test_publishes_at_about_ten_hz(node, listener):
    node.click(ChannelId.MISSION, 'SKIDPAD')
    assert wait_for(lambda: listener.counts()[ChannelId.MISSION] >= 1)

    start = listener.counts()[ChannelId.MISSION]
    time.sleep(1.0)
    sent = listener.counts()[ChannelId.MISSION] - start
    assert 7 <= sent <= 13, f'{sent} messages in 1 s, expected about 10'


# Scenario 3 / rule 4: a switch never sends the old value again.
def test_switch_stops_the_old_value(node, listener):
    node.click(ChannelId.AS_STATE, 'AS_READY')
    assert wait_for(lambda: listener.counts()[ChannelId.AS_STATE] >= 2)
    node.click(ChannelId.AS_STATE, 'AS_DRIVING')
    assert wait_for(lambda: 2 in listener.values(ChannelId.AS_STATE))

    after_switch = len(listener.values(ChannelId.AS_STATE))
    time.sleep(SETTLE_S)
    # Everything from the switch onwards is AS_DRIVING (2); AS_READY (1) is gone.
    assert set(listener.values(ChannelId.AS_STATE)[after_switch:]) == {2}


# Scenario 4: True, then False, then silence.
def test_go_true_false_then_silence(node, listener):
    node.click(ChannelId.GO, 'True')
    assert wait_for(lambda: True in listener.values(ChannelId.GO))

    node.click(ChannelId.GO, 'False')
    assert wait_for(lambda: False in listener.values(ChannelId.GO))

    node.click(ChannelId.GO, 'False')
    time.sleep(SETTLE_S)
    settled = listener.counts()[ChannelId.GO]
    time.sleep(SETTLE_S)
    assert listener.counts()[ChannelId.GO] == settled


# Scenario 6: deselecting one channel leaves the others publishing.
def test_other_channels_keep_publishing(node, listener):
    node.click(ChannelId.AS_STATE, 'AS_DRIVING')
    node.click(ChannelId.MISSION, 'TRACKDRIVE')
    node.click(ChannelId.GO, 'True')
    assert wait_for(lambda: all(count >= 2 for count in listener.counts().values()))

    node.click(ChannelId.MISSION, 'TRACKDRIVE')
    time.sleep(SETTLE_S)
    stopped = listener.counts()[ChannelId.MISSION]
    before = listener.counts()

    time.sleep(SETTLE_S)
    after = listener.counts()
    assert after[ChannelId.MISSION] == stopped
    assert after[ChannelId.AS_STATE] > before[ChannelId.AS_STATE]
    assert after[ChannelId.GO] > before[ChannelId.GO]


# Scenario 7: feedback is displayed and never modifies an input.
def test_feedback_is_displayed_and_changes_nothing(node, listener):
    node.click(ChannelId.AS_STATE, 'AS_READY')
    assert wait_for(lambda: listener.counts()[ChannelId.AS_STATE] >= 2)
    before = node.snapshot()

    listener.override_pub.publish(UInt8(data=ASOverride.AS_STATE_EMERGENCY))

    assert wait_for(lambda: node.latest_feedback() is not None)
    feedback = node.latest_feedback()
    assert feedback.state == 4
    assert feedback.label == 'AS_EMERGENCY'
    assert feedback.timestamp_text

    time.sleep(SETTLE_S)
    assert node.snapshot() == before
    assert set(listener.values(ChannelId.AS_STATE)) == {1}


def test_feedback_logs_only_changed_values(node, listener):
    lines: list[str] = []
    node._on_event = lines.append   # the GUI's event-log hook

    for _ in range(3):
        listener.override_pub.publish(UInt8(data=ASOverride.AS_STATE_DRIVING))
        time.sleep(0.05)
    assert wait_for(lambda: any('DRIVING' in line for line in lines))

    listener.override_pub.publish(UInt8(data=ASOverride.AS_STATE_FINISHED))
    assert wait_for(lambda: any('FINISHED' in line for line in lines))

    feedback_lines = [line for line in lines if 'override' in line]
    assert len(feedback_lines) == 2, feedback_lines


def test_shutdown_stops_publishing(listener):
    with RosRunner(args=NODE_ARGS) as runner:
        time.sleep(0.3)
        runner.node.click(ChannelId.GO, 'True')
        assert wait_for(lambda: listener.counts()[ChannelId.GO] >= 2)
        runner.node.shutdown()
        time.sleep(SETTLE_S)
        settled = listener.counts()[ChannelId.GO]
        time.sleep(SETTLE_S)
        assert listener.counts()[ChannelId.GO] == settled


def test_warns_about_a_competing_publisher(listener):
    """PLAN.md: the real bridge and the mock must never both publish."""
    rival = Node('mock_test_rival', context=listener.context)
    rival.create_publisher(UInt8, AS_STATE_TOPIC, state_qos())
    listener.executor.add_node(rival)

    lines: list[str] = []
    try:
        with RosRunner(args=NODE_ARGS, on_event=lines.append):
            assert wait_for(
                lambda: any('other publisher' in line for line in lines),
                timeout=RIVAL_CHECK_DELAY_S + 5.0,
            ), lines
        warning = next(line for line in lines if 'other publisher' in line)
        assert AS_STATE_TOPIC in warning
        assert warning.startswith('WARNING')
    finally:
        listener.executor.remove_node(rival)
        rival.destroy_node()


def test_no_warning_when_nothing_else_publishes(listener):
    lines: list[str] = []
    with RosRunner(args=NODE_ARGS, on_event=lines.append):
        time.sleep(RIVAL_CHECK_DELAY_S + 1.5)
    assert not any('other publisher' in line for line in lines), lines
