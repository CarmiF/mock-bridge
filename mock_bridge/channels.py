"""Value catalogues for the three mock input channels.

The channels are the three things the real bridge reports from the STM32 and
that the mock replaces: AS state, mission and the GO flag.  This module only
describes *what values exist*; it says nothing about ROS topics, message types
or QoS, which stage 2 wires up (see PLAN.md, "Deferred").

`payload` is the wire value the eventual publisher will put in the message.
For AS state that mapping is settled -- it mirrors `ASState` in the real
bridge's `fs_uart_protocol_v2.py`.  For mission it is **provisional**: the
real protocol records the mission as a raw id with the mapping "TBD by
firmware", so these ids are placeholders to be confirmed before integration,
and nothing in this package depends on their particular numbers.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ChannelId(Enum):
    """The three independent input channels."""

    AS_STATE = 'as_state'
    MISSION = 'mission'
    GO = 'go'


@dataclass(frozen=True)
class ChannelValue:
    """One selectable value of a channel.

    label:   what the GUI button shows, and the key the state model is driven by.
    payload: the value a publisher will send (int for AS state / mission,
             bool for GO).
    """

    label: str
    payload: object


@dataclass(frozen=True)
class ChannelSpec:
    """A channel's identity and its ordered list of selectable values."""

    channel_id: ChannelId
    title: str
    values: tuple[ChannelValue, ...]

    def value(self, label: str) -> ChannelValue:
        """Return the value with `label`, or raise KeyError."""
        for value in self.values:
            if value.label == label:
                return value
        raise KeyError(
            f'{self.channel_id.value} has no value {label!r}; '
            f'expected one of {[v.label for v in self.values]}'
        )

    @property
    def labels(self) -> tuple[str, ...]:
        return tuple(value.label for value in self.values)


# Settled: same numbering as ASState in fs_uart_protocol_v2.py.
AS_STATE_SPEC = ChannelSpec(
    channel_id=ChannelId.AS_STATE,
    title='AS State',
    values=(
        ChannelValue('AS_OFF', 0),
        ChannelValue('AS_READY', 1),
        ChannelValue('AS_DRIVING', 2),
        ChannelValue('AS_FINISHED', 3),
        ChannelValue('AS_EMERGENCY', 4),
    ),
)

# PROVISIONAL payloads -- the firmware's mission ids are not resolved yet.
MISSION_SPEC = ChannelSpec(
    channel_id=ChannelId.MISSION,
    title='Mission',
    values=(
        ChannelValue('NONE', 0),
        ChannelValue('ACCELERATION', 1),
        ChannelValue('SKIDPAD', 2),
        ChannelValue('AUTOCROSS', 3),
        ChannelValue('TRACKDRIVE', 4),
    ),
)

GO_SPEC = ChannelSpec(
    channel_id=ChannelId.GO,
    title='GO',
    values=(
        ChannelValue('True', True),
        ChannelValue('False', False),
    ),
)

CHANNEL_SPECS: tuple[ChannelSpec, ...] = (AS_STATE_SPEC, MISSION_SPEC, GO_SPEC)

SPEC_BY_ID = {spec.channel_id: spec for spec in CHANNEL_SPECS}
