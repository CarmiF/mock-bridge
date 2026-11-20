"""The GUI, driven by real Qt button clicks (PLAN.md GUI requirements).

Runs offscreen and against a stub controller, so these tests need no display
and no ROS: what they check is the GUI's own contract -- buttons reflect the
state model, the status lines are right, feedback is display-only, the
lifecycle rows show what the managed nodes report and offer only the
transitions that are legal there, and closing stops the publishers.

Skipped where PySide6 is not installed (it is not in the Ubuntu 24.04 archive;
the Docker image pip-installs it).
"""

from __future__ import annotations

import os

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

pytest.importorskip('PySide6')

from PySide6.QtCore import Qt  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from mock_bridge import BridgeState, ChannelId  # noqa: E402
from mock_bridge.lifecycle import (  # noqa: E402
    COMMANDS,
    LifecycleState,
    default_nodes,
)
from mock_bridge.gui import MockBridgeWindow, _Relay, _split_args  # noqa: E402


class StubLifecycle:
    """What the window needs of the lifecycle client, with no ROS behind it.

    `report` is this stub's version of a managed node answering: the real
    client only ever learns a state from the node itself, which is why
    nothing in the GUI sets one.
    """

    def __init__(self, relay: _Relay) -> None:
        self._nodes = {m.name: m for m in default_nodes()}
        self._relay = relay
        self.requests: list[tuple[str, str]] = []

    def nodes(self):
        return tuple(self._nodes.values())

    def node(self, name):
        return self._nodes[name]

    def request(self, name: str, command: str) -> None:
        self.requests.append((name, command))

    def report(self, name: str, state: LifecycleState) -> None:
        """Pretend the node published a transition to `state`."""
        self._nodes[name] = self._nodes[name].with_state(state)
        self._relay.lifecycle.emit(self._nodes[name])


class StubController:
    """What the window needs of the node, with the state model behind it.

    The real node logs through the same `on_event` hook, so routing it into
    the relay here is the same wiring `gui.main` sets up.
    """

    def __init__(self, relay: _Relay) -> None:
        self.state = BridgeState()
        self._relay = relay
        self.stop_all_calls = 0
        self.lifecycle = StubLifecycle(relay)

    def click(self, channel_id: ChannelId, label: str):
        result = self.state.click(channel_id, label)
        self._relay.event.emit(result.describe())
        return result

    def stop_all(self):
        self.stop_all_calls += 1
        results = self.state.stop_all()
        for result in results:
            self._relay.event.emit(result.describe())
        return results

    def latest_feedback(self):
        return None


class StubFeedback:
    """Shaped like node.Feedback, without needing fs_msgs."""

    def __init__(self, label: str, state: int) -> None:
        self.label = label
        self.state = state

    def describe(self) -> str:
        return f'{self.label} ({self.state}) at 12:00:00'


@pytest.fixture(scope='session')
def app():
    application = QApplication.instance() or QApplication([])
    yield application


@pytest.fixture
def window(app):
    relay = _Relay()
    controller = StubController(relay)
    window = MockBridgeWindow(controller, relay)
    window.controller = controller  # for the tests' convenience
    yield window
    window.close()


def press_lifecycle(window, name: str, label: str) -> None:
    """A real Qt click on a transition button."""
    window.lifecycle_row(name).button(label).click()
    QApplication.processEvents()


def report(window, name: str, state: LifecycleState) -> None:
    """A managed node reports a state; deliver the queued signal."""
    window.controller.lifecycle.report(name, state)
    QApplication.processEvents()


def press(window, channel_id: ChannelId, label: str) -> None:
    """A real Qt click, not a direct call into the handler."""
    window.group(channel_id).button(label).click()
    QApplication.processEvents()  # deliver the queued log signal


def test_starts_with_everything_stopped(window):
    for channel_id in ChannelId:
        group = window.group(channel_id)
        assert group.status_text == 'STOPPED'
        for label in window.controller.state.channel(channel_id).spec.labels:
            assert not group.button(label).isChecked()
    assert window.controller.state.publishing() == {}
    assert 'nothing is published' in window.log_text


def test_click_highlights_and_publishes(window):
    press(window, ChannelId.AS_STATE, 'AS_READY')

    group = window.group(ChannelId.AS_STATE)
    assert group.button('AS_READY').isChecked()
    assert group.status_text == 'PUBLISHING 10 Hz'
    assert window.controller.state.publishing()[ChannelId.AS_STATE].label == 'AS_READY'
    assert 'AS_READY selected' in window.log_text


def test_second_click_unhighlights_and_stops(window):
    press(window, ChannelId.AS_STATE, 'AS_READY')
    press(window, ChannelId.AS_STATE, 'AS_READY')

    group = window.group(ChannelId.AS_STATE)
    assert not group.button('AS_READY').isChecked()
    assert group.status_text == 'STOPPED'
    assert ChannelId.AS_STATE not in window.controller.state.publishing()
    assert 'deselected' in window.log_text


def test_switch_moves_the_highlight(window):
    press(window, ChannelId.AS_STATE, 'AS_READY')
    press(window, ChannelId.AS_STATE, 'AS_DRIVING')

    group = window.group(ChannelId.AS_STATE)
    assert not group.button('AS_READY').isChecked()
    assert group.button('AS_DRIVING').isChecked()
    assert group.status_text == 'PUBLISHING 10 Hz'
    assert 'AS_READY -> AS_DRIVING' in window.log_text


def test_channels_are_independent(window):
    press(window, ChannelId.GO, 'True')
    press(window, ChannelId.MISSION, 'SKIDPAD')
    press(window, ChannelId.AS_STATE, 'AS_DRIVING')
    press(window, ChannelId.MISSION, 'SKIDPAD')  # stop mission only

    assert window.group(ChannelId.MISSION).status_text == 'STOPPED'
    assert window.group(ChannelId.GO).status_text == 'PUBLISHING 10 Hz'
    assert window.group(ChannelId.AS_STATE).status_text == 'PUBLISHING 10 Hz'
    assert window.group(ChannelId.GO).button('True').isChecked()


def test_only_one_button_per_group_is_checked(window):
    group = window.group(ChannelId.MISSION)
    for label in ('NONE', 'ACCELERATION', 'TRACKDRIVE'):
        press(window, ChannelId.MISSION, label)
        checked = [
            name for name in group._buttons
            if group.button(name).isChecked()
        ]
        assert checked == [label]


# Scenario 7: feedback shows up and modifies nothing.
def test_feedback_is_displayed_only(window):
    press(window, ChannelId.AS_STATE, 'AS_READY')
    before = window.controller.state.publishing()

    window._relay.feedback.emit(StubFeedback('AS_EMERGENCY', 4))
    QApplication.processEvents()

    assert window.feedback_text == 'AS_EMERGENCY (4) at 12:00:00'
    assert window.controller.state.publishing() == before
    assert window.group(ChannelId.AS_STATE).button('AS_READY').isChecked()


def test_feedback_field_is_read_only(window):
    assert window._feedback_field.isReadOnly()
    assert window._log.isReadOnly()


def test_log_lines_arrive_from_the_ros_thread_signal(window):
    window._relay.event.emit('orchestrator state/override: AS_DRIVING (2) at 12:00:00')
    QApplication.processEvents()
    assert 'AS_DRIVING (2)' in window.log_text


def test_log_is_capped(window):
    for index in range(600):
        window.append_log(f'line {index}')
    lines = window.log_text.splitlines()
    assert len(lines) <= 500
    assert lines[-1] == 'line 599'


def test_closing_stops_the_publishers(window):
    press(window, ChannelId.GO, 'True')
    press(window, ChannelId.AS_STATE, 'AS_FINISHED')

    window.close()
    QApplication.processEvents()

    assert window.controller.stop_all_calls == 1
    assert window.controller.state.publishing() == {}
    assert window.group(ChannelId.GO).status_text == 'STOPPED'
    assert not window.group(ChannelId.GO).button('True').isChecked()


def test_window_has_all_three_groups_in_order(window):
    titles = [window.group(channel_id).title() for channel_id in ChannelId]
    assert titles == ['AS State', 'Mission', 'GO']


def test_ros_arguments_are_kept_away_from_qt():
    # `ros2 launch` appends --ros-args; Qt must not see them, and rclpy must.
    argv = ['mock_bridge_gui', '--ros-args', '-p', 'go_topic:=/x/go']
    qt_argv, ros_argv = _split_args(argv)
    assert qt_argv == ['mock_bridge_gui']
    assert ros_argv == ['mock_bridge_gui', '--ros-args', '-p', 'go_topic:=/x/go']


def test_plain_arguments_are_left_alone():
    assert _split_args(['mock_bridge_gui']) == (['mock_bridge_gui'], [])


# ----------------------------------------------------------------------
# The managed nodes' lifecycle
# ----------------------------------------------------------------------


def test_lifecycle_rows_start_unknown_with_nothing_enabled(window):
    for name in ('control', 'planner', 'mock1', 'mock2'):
        row = window.lifecycle_row(name)
        assert 'unknown' in row.state_text
        # Nothing can be asked of a node we have not reached.
        for command in COMMANDS:
            assert not row.button(command.label).isEnabled()


def test_unconfigured_offers_configure_and_shutdown_only(window):
    report(window, 'control', LifecycleState.UNCONFIGURED)

    row = window.lifecycle_row('control')
    assert row.state_text == 'UNCONFIGURED [1]'
    enabled = {c.label for c in COMMANDS if row.button(c.label).isEnabled()}
    assert enabled == {'Configure', 'Shutdown'}


def test_inactive_and_active_offer_their_own_transitions(window):
    report(window, 'control', LifecycleState.INACTIVE)
    row = window.lifecycle_row('control')
    assert {c.label for c in COMMANDS if row.button(c.label).isEnabled()} == {
        'Activate', 'Cleanup', 'Shutdown'}

    report(window, 'control', LifecycleState.ACTIVE)
    assert row.state_text == 'ACTIVE [3]'
    assert {c.label for c in COMMANDS if row.button(c.label).isEnabled()} == {
        'Deactivate', 'Shutdown'}


def test_finalized_offers_nothing(window):
    report(window, 'control', LifecycleState.FINALIZED)

    row = window.lifecycle_row('control')
    assert row.state_text == 'FINALIZED [4]'
    for command in COMMANDS:
        assert not row.button(command.label).isEnabled()


def test_a_transition_button_only_asks(window):
    report(window, 'control', LifecycleState.UNCONFIGURED)
    press_lifecycle(window, 'control', 'Configure')

    assert window.controller.lifecycle.requests == [('control', 'Configure')]
    # The GUI does not move the row itself: what is shown is what the node
    # has reported, and it has reported nothing new yet.
    assert window.lifecycle_row('control').state_text == 'UNCONFIGURED [1]'


def test_a_disabled_transition_cannot_be_asked_for(window):
    report(window, 'control', LifecycleState.UNCONFIGURED)
    press_lifecycle(window, 'control', 'Activate')

    assert window.controller.lifecycle.requests == []


def test_the_managed_nodes_are_independent(window):
    report(window, 'control', LifecycleState.ACTIVE)
    report(window, 'planner', LifecycleState.UNCONFIGURED)
    report(window, 'mock1', LifecycleState.INACTIVE)
    report(window, 'mock2', LifecycleState.FINALIZED)

    assert window.lifecycle_row('control').state_text == 'ACTIVE [3]'
    assert window.lifecycle_row('planner').state_text == 'UNCONFIGURED [1]'
    assert window.lifecycle_row('mock1').state_text == 'INACTIVE [2]'
    assert window.lifecycle_row('mock2').state_text == 'FINALIZED [4]'

    press_lifecycle(window, 'control', 'Deactivate')
    press_lifecycle(window, 'mock1', 'Activate')
    # mock2 is finalized: its buttons are dead, so a click asks for nothing.
    press_lifecycle(window, 'mock2', 'Activate')
    assert window.controller.lifecycle.requests == [
        ('control', 'Deactivate'), ('mock1', 'Activate')]


def test_every_managed_node_gets_a_row(window):
    assert [row for row in ('control', 'planner', 'mock1', 'mock2')
            if window.lifecycle_row(row) is not None] == [
        'control', 'planner', 'mock1', 'mock2']
