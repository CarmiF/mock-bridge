"""The manual control GUI (PLAN.md implementation step 3).

Three compact button groups, a per-channel status line, one row per managed
lifecycle node, one read-only field for the last Orchestrator State /
Override, and an event log.

**The state model is the only source of truth for what is selected.** A click
is handed to the node, and the buttons are then redrawn from the model, so the
GUI cannot drift into showing a selection that is not being published.

Threading: the ROS executor runs on its own thread (`runner.RosRunner`), and
its callbacks reach Qt only through `_Relay`'s signals. A signal emitted from
the ROS thread to a receiver that lives in the GUI thread is queued by Qt and
delivered on the GUI thread, which is what makes it safe; no widget is touched
from a ROS callback.
"""

from __future__ import annotations

import signal
import sys
from typing import Optional

from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from .channels import ChannelId
from .lifecycle import COMMANDS

WINDOW_TITLE = 'Mock Bridge -- manual'
MAX_LOG_LINES = 500

_PUBLISHING_STYLE = 'color: #0a7d28; font-weight: bold;'
_STOPPED_STYLE = 'color: #777777;'

# Lifecycle state: active stands out, and a node we cannot reach is visibly
# not just "some other state".
_ACTIVE_STYLE = 'color: #0a7d28; font-weight: bold;'
_KNOWN_STYLE = 'font-weight: bold;'
_UNKNOWN_STYLE = 'color: #999999; font-style: italic;'

# The active button has to be obvious across the three groups, so it is
# coloured rather than left to the theme's subtle "checked" shade.
_BUTTON_STYLE = """
QPushButton {
    padding: 6px 10px;
}
QPushButton:checked {
    background-color: #1f6feb;
    color: white;
    font-weight: bold;
    border: 1px solid #10459e;
}
"""


class _Relay(QObject):
    """Carries ROS-thread callbacks into the GUI thread as queued signals."""

    event = Signal(str)
    feedback = Signal(object)
    lifecycle = Signal(object)


class _ChannelGroup(QGroupBox):
    """One channel's buttons and its PUBLISHING / STOPPED line."""

    def __init__(self, channel, on_click, parent=None) -> None:
        super().__init__(channel.spec.title, parent)
        self._channel = channel
        self._buttons: dict[str, QPushButton] = {}

        layout = QVBoxLayout(self)
        buttons_row = QHBoxLayout()

        # Exclusive=False: the model decides. An exclusive QButtonGroup would
        # refuse to let the user un-check the active button, and un-checking
        # is how a channel is stopped.
        self._group = QButtonGroup(self)
        self._group.setExclusive(False)

        for value in channel.spec.values:
            button = QPushButton(value.label)
            button.setCheckable(True)
            button.setStyleSheet(_BUTTON_STYLE)
            button.clicked.connect(
                lambda _checked=False, label=value.label: on_click(
                    channel.channel_id, label)
            )
            self._group.addButton(button)
            buttons_row.addWidget(button)
            self._buttons[value.label] = button

        layout.addLayout(buttons_row)

        self._status = QLabel()
        layout.addWidget(self._status)
        self.refresh()

    def refresh(self) -> None:
        """Redraw from the state model."""
        for label, button in self._buttons.items():
            selected = self._channel.is_selected(label)
            # Qt has already toggled the clicked button; setChecked here is
            # what corrects it when the model says otherwise.
            if button.isChecked() != selected:
                button.setChecked(selected)

        publishing = self._channel.is_publishing
        # The model renders its own status text, so the GUI cannot word it
        # differently from the event log.
        self._status.setText(self._channel.status_text)
        self._status.setStyleSheet(
            _PUBLISHING_STYLE if publishing else _STOPPED_STYLE)

    def button(self, label: str) -> QPushButton:
        """A button by label, for tests."""
        return self._buttons[label]

    @property
    def status_text(self) -> str:
        return self._status.text()


class _LifecycleRow(QWidget):
    """One managed node: its name, its current lifecycle state, its buttons.

    The buttons are plain (not checkable): a transition is a request, not a
    selection, and what the node ends up in is whatever it reports back. Only
    the transitions legal from the current state are enabled, which is also
    the quickest way to read the lifecycle off the screen.
    """

    def __init__(self, managed, on_command, parent=None) -> None:
        super().__init__(parent)
        self._name = managed.name

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self._title = QLabel(f'{managed.title}  /{managed.name}')
        self._title.setMinimumWidth(150)
        layout.addWidget(self._title)

        self._state = QLabel()
        self._state.setMinimumWidth(170)
        layout.addWidget(self._state)

        self._buttons: dict[str, QPushButton] = {}
        for command in COMMANDS:
            button = QPushButton(command.label)
            button.clicked.connect(
                lambda _checked=False, label=command.label: on_command(
                    managed.name, label)
            )
            layout.addWidget(button)
            self._buttons[command.label] = button

        self.update_from(managed)

    def update_from(self, managed) -> None:
        """Redraw from a `lifecycle.ManagedNode` snapshot."""
        self._state.setText(managed.status_text)
        if managed.is_active:
            style = _ACTIVE_STYLE
        elif managed.is_reachable:
            style = _KNOWN_STYLE
        else:
            style = _UNKNOWN_STYLE
        self._state.setStyleSheet(style)

        for command in COMMANDS:
            self._buttons[command.label].setEnabled(managed.allows(command))

    def button(self, label: str) -> QPushButton:
        """A transition button by label, for tests."""
        return self._buttons[label]

    @property
    def state_text(self) -> str:
        return self._state.text()


class MockBridgeWindow(QMainWindow):
    """The window. `controller` is a `node.MockBridgeNode`.

    Only five things are asked of it -- `state`, `click`, `latest_feedback`,
    `stop_all` and `lifecycle` -- so the window can be tested against a stub
    and needs no ROS of its own.
    """

    def __init__(self, controller, relay: Optional[_Relay] = None, parent=None) -> None:
        super().__init__(parent)
        self._controller = controller
        self.setWindowTitle(WINDOW_TITLE)

        central = QWidget()
        layout = QVBoxLayout(central)

        self._groups: dict[ChannelId, _ChannelGroup] = {}
        for channel in controller.state:
            group = _ChannelGroup(channel, self._on_click)
            layout.addWidget(group)
            self._groups[channel.channel_id] = group

        lifecycle_box = QGroupBox('Managed nodes -- ROS 2 lifecycle')
        lifecycle_layout = QVBoxLayout(lifecycle_box)
        self._lifecycle_rows: dict[str, _LifecycleRow] = {}
        for managed in controller.lifecycle.nodes():
            row = _LifecycleRow(managed, self._on_lifecycle_command)
            lifecycle_layout.addWidget(row)
            self._lifecycle_rows[managed.name] = row
        lifecycle_layout.addWidget(QLabel(
            'These nodes do nothing; only their lifecycle state is real. '
            'The Orchestrator may drive them too -- what is shown is what '
            'they report.'))
        layout.addWidget(lifecycle_box)

        feedback_box = QGroupBox('Orchestrator State / Override (received)')
        feedback_layout = QVBoxLayout(feedback_box)
        self._feedback_field = QLineEdit()
        self._feedback_field.setReadOnly(True)
        self._feedback_field.setPlaceholderText('nothing received yet')
        feedback_layout.addWidget(self._feedback_field)
        feedback_layout.addWidget(
            QLabel('Display only -- never acts on any input.'))
        layout.addWidget(feedback_box)

        log_box = QGroupBox('Event log')
        log_layout = QVBoxLayout(log_box)
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(MAX_LOG_LINES)
        self._log.setFont(QFont('monospace'))
        log_layout.addWidget(self._log)
        layout.addWidget(log_box)

        self.setCentralWidget(central)
        self.resize(760, 830)

        self._relay = relay
        if relay is not None:
            # Queued across threads: the ROS thread emits, the GUI thread runs
            # the slot. Qt.QueuedConnection is the default for that case; it is
            # named here because it is the whole point.
            relay.event.connect(self.append_log, Qt.QueuedConnection)
            relay.feedback.connect(self._on_feedback, Qt.QueuedConnection)
            relay.lifecycle.connect(self._on_lifecycle, Qt.QueuedConnection)

        self.append_log(
            'ready -- nothing is published until a value is selected')

    # ------------------------------------------------------------------
    # GUI thread only
    # ------------------------------------------------------------------

    def _on_click(self, channel_id: ChannelId, label: str) -> None:
        """A button was pressed: tell the node, then redraw from the model."""
        self._controller.click(channel_id, label)
        # Only this channel's buttons can have changed, but the status lines
        # are cheap and redrawing all three keeps one code path.
        self.refresh()

    def refresh(self) -> None:
        for group in self._groups.values():
            group.refresh()
        self.refresh_lifecycle()

    def refresh_lifecycle(self) -> None:
        """Redraw every managed node from the client's snapshot."""
        for managed in self._controller.lifecycle.nodes():
            self._lifecycle_rows[managed.name].update_from(managed)

    def _on_lifecycle_command(self, name: str, label: str) -> None:
        """A transition button was pressed: queue the request.

        Nothing is drawn here. The node reports the state it reaches, which
        arrives as a `lifecycle` signal -- so the row cannot show a state the
        node is not actually in.
        """
        self._controller.lifecycle.request(name, label)

    def _on_lifecycle(self, managed) -> None:
        self._lifecycle_rows[managed.name].update_from(managed)

    def append_log(self, line: str) -> None:
        self._log.appendPlainText(line)

    def _on_feedback(self, feedback) -> None:
        """Show the latest feedback. Touches no input -- scenario 7."""
        self._feedback_field.setText(feedback.describe())

    def group(self, channel_id: ChannelId) -> _ChannelGroup:
        """A channel's group, for tests."""
        return self._groups[channel_id]

    def lifecycle_row(self, name: str) -> _LifecycleRow:
        """A managed node's row, for tests."""
        return self._lifecycle_rows[name]

    @property
    def feedback_text(self) -> str:
        return self._feedback_field.text()

    @property
    def log_text(self) -> str:
        return self._log.toPlainText()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:
        """Stop the publishers before the window goes away."""
        self._controller.stop_all()
        self.refresh()
        super().closeEvent(event)


def _split_args(argv: list[str]) -> tuple[list[str], list[str]]:
    """Separate Qt's arguments from ROS's.

    `ros2 launch` appends `--ros-args -p name:=value ...` to argv. Qt would
    either ignore or misread them, so each side is given only its own.
    """
    if '--ros-args' in argv:
        index = argv.index('--ros-args')
        return argv[:index], [argv[0], *argv[index:]]
    return argv, []


def main(args: Optional[list[str]] = None) -> int:
    """Run the GUI with the ROS node spinning on its own thread."""
    # Imported here so `mock_bridge.gui` can be imported, and the window
    # tested, without rclpy on the path.
    from .runner import RosRunner

    argv = sys.argv if args is None else [sys.argv[0], *args]
    qt_argv, ros_argv = _split_args(argv)

    app = QApplication(qt_argv)
    app.setStyleSheet(_BUTTON_STYLE)

    relay = _Relay()
    runner = RosRunner(
        args=ros_argv or None,
        on_event=relay.event.emit,
        on_feedback=relay.feedback.emit,
        on_lifecycle=relay.lifecycle.emit,
    )
    node = runner.start()
    window = MockBridgeWindow(node, relay)
    window.show()

    # Ctrl-C in the launching terminal should close the window, and Python
    # signal handlers only run between bytecodes -- hence the idle timer.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    keep_alive = QTimer()
    keep_alive.start(200)
    keep_alive.timeout.connect(lambda: None)

    try:
        return app.exec()
    finally:
        # Stops the publishers, shuts the node down and joins the ROS thread.
        runner.stop()


if __name__ == '__main__':
    sys.exit(main())
