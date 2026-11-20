"""The mock bridge's input state model (PLAN.md implementation step 1).

Pure Python: no rclpy, no Qt.  The ROS publishers (stage 2) and the GUI
(stage 3) both read this model and nothing else, so the toggle rules live in
exactly one place and can be tested without a ROS graph or a display.

Per channel the whole state is `selected: Optional[ChannelValue]`, where
`None` means **this channel publishes nothing at all** -- not OFF, not NONE,
not False, not an empty message.  The three channels never touch each other.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterator, Optional

from .channels import CHANNEL_SPECS, ChannelId, ChannelSpec, ChannelValue, SPEC_BY_ID

# PLAN.md: 10 Hz only, while a channel is selected. Here rather than in
# `ros_interface` so the GUI and the state model can read it without rclpy or
# fs_msgs on the path; `ros_interface` re-exports it for the node.
PUBLISH_PERIOD_S = 0.1
PUBLISH_RATE_HZ = 10.0


class ClickAction(Enum):
    """What a click did to a channel -- what the GUI event log reports."""

    SELECTED = 'selected'      # was publishing nothing, now publishes `value`
    SWITCHED = 'switched'      # was publishing `previous`, now publishes `value`
    DESELECTED = 'deselected'  # was publishing `previous`, now publishes nothing


@dataclass(frozen=True)
class ClickResult:
    """The outcome of one button click, for logging and GUI refresh."""

    channel_id: ChannelId
    action: ClickAction
    value: Optional[ChannelValue]      # None after a deselect
    previous: Optional[ChannelValue]   # None after a select from stopped

    @property
    def is_publishing(self) -> bool:
        return self.value is not None

    def describe(self) -> str:
        """One line for the event log."""
        name = self.channel_id.value
        if self.action is ClickAction.SELECTED:
            return (
                f'{name}: {self.value.label} selected '
                f'-- publishing {PUBLISH_RATE_HZ:g} Hz'
            )
        if self.action is ClickAction.SWITCHED:
            return (
                f'{name}: {self.previous.label} -> {self.value.label} '
                f'-- publishing {PUBLISH_RATE_HZ:g} Hz'
            )
        return f'{name}: {self.previous.label} deselected -- stopped'


class Channel:
    """One input channel: at most one selected value, or none at all."""

    def __init__(self, spec: ChannelSpec) -> None:
        self._spec = spec
        self._selected: Optional[ChannelValue] = None

    @property
    def spec(self) -> ChannelSpec:
        return self._spec

    @property
    def channel_id(self) -> ChannelId:
        return self._spec.channel_id

    @property
    def selected(self) -> Optional[ChannelValue]:
        """The value being published, or None while this channel is silent."""
        return self._selected

    @property
    def is_publishing(self) -> bool:
        return self._selected is not None

    @property
    def status_text(self) -> str:
        """What the GUI shows next to the channel's group."""
        if self.is_publishing:
            return f'PUBLISHING {PUBLISH_RATE_HZ:g} Hz'
        return 'STOPPED'

    def is_selected(self, label: str) -> bool:
        """True if `label` is this channel's active value (for highlighting)."""
        return self._selected is not None and self._selected.label == label

    def click(self, label: str) -> ClickResult:
        """Apply a click on the button `label` and return what it did.

        Inactive value -> select it.  The selected value -> deselect it and
        stop publishing.  A different value -> switch immediately; the old
        value is never published again.  Raises KeyError for an unknown label.
        """
        value = self._spec.value(label)
        previous = self._selected

        if previous is not None and previous.label == label:
            self._selected = None
            return ClickResult(self.channel_id, ClickAction.DESELECTED, None, previous)

        self._selected = value
        action = ClickAction.SWITCHED if previous is not None else ClickAction.SELECTED
        return ClickResult(self.channel_id, action, value, previous)

    def stop(self) -> Optional[ClickResult]:
        """Stop publishing this channel, as on shutdown.

        Returns the result if it was publishing, None if it already was not.
        Sends no final value: stopping is silence, not a reset.
        """
        previous = self._selected
        if previous is None:
            return None
        self._selected = None
        return ClickResult(self.channel_id, ClickAction.DESELECTED, None, previous)


class BridgeState:
    """The three channels together.

    At startup every channel is unselected, so the mock publishes nothing
    until the first click.
    """

    def __init__(self, specs: tuple[ChannelSpec, ...] = CHANNEL_SPECS) -> None:
        self._channels = {spec.channel_id: Channel(spec) for spec in specs}

    def __iter__(self) -> Iterator[Channel]:
        """The channels in GUI order."""
        return iter(self._channels.values())

    def channel(self, channel_id: ChannelId) -> Channel:
        return self._channels[channel_id]

    def click(self, channel_id: ChannelId, label: str) -> ClickResult:
        """Click `label` in `channel_id`; the other two channels are untouched."""
        return self._channels[channel_id].click(label)

    @property
    def is_publishing_anything(self) -> bool:
        return any(channel.is_publishing for channel in self._channels.values())

    def publishing(self) -> dict[ChannelId, ChannelValue]:
        """The channels to publish this tick, and the value each sends.

        Channels with no selection are absent, which is how the 100 ms timer
        knows to send nothing for them.
        """
        return {
            channel_id: channel.selected
            for channel_id, channel in self._channels.items()
            if channel.selected is not None
        }

    def stop_all(self) -> list[ClickResult]:
        """Stop every publishing channel; returns one result per channel stopped."""
        results = [channel.stop() for channel in self._channels.values()]
        return [result for result in results if result is not None]


__all__ = [
    'PUBLISH_PERIOD_S',
    'PUBLISH_RATE_HZ',
    'BridgeState',
    'Channel',
    'ChannelId',
    'ChannelSpec',
    'ChannelValue',
    'ClickAction',
    'ClickResult',
    'SPEC_BY_ID',
]
