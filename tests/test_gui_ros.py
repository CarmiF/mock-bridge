"""GUI click -> real ROS publication, the two halves wired as they ship.

The other two files each test one side against a stub; this one is the seam:
a Qt button click on the window, the node spinning on its own thread, and a
subscriber on the graph counting what came out. Needs both PySide6 and rclpy,
and skips otherwise.
"""

from __future__ import annotations

import os
import threading
import time

import pytest

os.environ.setdefault('ROS_DOMAIN_ID', '77')
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

pytest.importorskip('PySide6')
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('fs_msgs')

from PySide6.QtWidgets import QApplication            # noqa: E402
from fs_msgs.msg import ASOverride                    # noqa: E402
from rclpy.executors import SingleThreadedExecutor    # noqa: E402
from rclpy.node import Node                           # noqa: E402
from std_msgs.msg import UInt8                        # noqa: E402

from mock_bridge import ChannelId                     # noqa: E402
from mock_bridge.gui import MockBridgeWindow, _Relay  # noqa: E402
from mock_bridge.ros_interface import feedback_qos, state_qos  # noqa: E402
from mock_bridge.runner import RosRunner              # noqa: E402

AS_STATE_TOPIC = '/mock_gui/as_state'
OVERRIDE_TOPIC = '/mock_gui/as_override'
NODE_ARGS = [
    '--ros-args',
    '-p', f'as_state_topic:={AS_STATE_TOPIC}',
    '-p', 'mission_topic:=/mock_gui/mission',
    '-p', 'go_topic:=/mock_gui/go',
    '-p', f'override_topic:={OVERRIDE_TOPIC}',
]


class Peer:
    """Subscribes to what the GUI publishes, and publishes the feedback."""

    def __init__(self) -> None:
        self.context = rclpy.Context()
        self.context.init()
        self.node = Node('mock_gui_peer', context=self.context)
        self.as_states: list[int] = []
        self._lock = threading.Lock()
        self.node.create_subscription(
            UInt8, AS_STATE_TOPIC, self._record, state_qos())
        self.override_pub = self.node.create_publisher(
            UInt8, OVERRIDE_TOPIC, feedback_qos())
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _record(self, msg) -> None:
        with self._lock:
            self.as_states.append(msg.data)

    def count(self) -> int:
        with self._lock:
            return len(self.as_states)

    def stop(self) -> None:
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        self.context.try_shutdown()


def pump(seconds: float) -> None:
    """Run the Qt event loop for a while, as a user staring at the window would."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        QApplication.processEvents()
        time.sleep(0.01)


def wait_for(predicate, timeout: float = 3.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        QApplication.processEvents()
        if predicate():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def peer():
    peer = Peer()
    time.sleep(0.2)
    yield peer
    peer.stop()


@pytest.fixture
def window(peer):
    app = QApplication.instance() or QApplication([])
    relay = _Relay()
    runner = RosRunner(
        args=NODE_ARGS,
        on_event=relay.event.emit,
        on_feedback=relay.feedback.emit,
    )
    node = runner.start()
    time.sleep(0.3)  # discovery
    window = MockBridgeWindow(node, relay)
    window.show()
    yield window
    window.close()
    runner.stop()
    app.processEvents()


def test_button_click_publishes_and_second_click_stops(window, peer):
    pump(0.3)
    assert peer.count() == 0, 'published before any click'

    window.group(ChannelId.AS_STATE).button('AS_READY').click()
    assert wait_for(lambda: peer.count() >= 3)
    assert set(peer.as_states) == {1}
    assert window.group(ChannelId.AS_STATE).status_text == 'PUBLISHING 10 Hz'

    window.group(ChannelId.AS_STATE).button('AS_READY').click()
    pump(0.5)
    settled = peer.count()
    pump(0.5)
    assert peer.count() == settled
    assert window.group(ChannelId.AS_STATE).status_text == 'STOPPED'


def test_feedback_from_ros_reaches_the_window(window, peer):
    peer.override_pub.publish(UInt8(data=ASOverride.AS_STATE_FINISHED))

    assert wait_for(lambda: 'AS_FINISHED' in window.feedback_text)
    assert 'AS_FINISHED (3)' in window.feedback_text
    # Display only: no input became selected because feedback arrived.
    assert window._controller.snapshot() == {}
    assert all(
        not group.button(label).isChecked()
        for group in window._groups.values()
        for label in group._buttons
    )


def test_closing_the_window_stops_publishing(window, peer):
    window.group(ChannelId.AS_STATE).button('AS_DRIVING').click()
    assert wait_for(lambda: peer.count() >= 2)

    window.close()
    pump(0.5)
    settled = peer.count()
    pump(0.5)
    assert peer.count() == settled
