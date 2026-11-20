"""The ROS contract the mock must match, in one place.

Every name, message type and QoS profile here was read off the real sources,
not proposed -- PLAN.md's "Deferred" section forbids inventing them:

  Publishers (what the Orchestrator subscribes to)
    /detached_sensor/as_state  std_msgs/UInt8  bridge_node_v2._setup_topics
    /detached_sensor/mission   std_msgs/UInt8  (same, SENSOR_NS = 'detached_sensor')
    /detached_sensor/go        std_msgs/Bool
    on the bridge's STATE_QOS: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST depth 1.
    The Orchestrator subscribes to exactly these names and that QoS
    (orchestrator/config.py DEFAULT_*_TOPIC, orchestrator_node._latched).

  Subscriber (the single Orchestrator feedback interface, display only)
    /bridge/override           std_msgs/UInt8
    on the Orchestrator's own LATCHED_QOS: RELIABLE, TRANSIENT_LOCAL,
    KEEP_LAST depth 1 (orchestrator_node.OVERRIDE_TOPIC and LATCHED_QOS,
    which publish it on every 10 Hz tick).

    NOT the bridge's `vehicle/as_override` (fs_msgs/ASOverride): that is the
    PC -> STM32 request the real bridge subscribes to, it accepts only
    DRIVING/FINISHED/EMERGENCY, and the Orchestrator creates no publisher on
    it. `/bridge/override` is the name its PLAN.md gives its own output, and
    it carries all five AS states as a bare number.

"Orchestrator State" and "Override" are the same information, as PLAN.md says:
the Orchestrator's state *is* the AS state it asks for, published on this one
topic (orchestrator/interface.py, `OrchestratorState = ASState`).

`SENSOR_NS` is marked temporary in the real bridge, so all four names are ROS
parameters with these defaults -- the same reason the Orchestrator made them
configurable.
"""

from __future__ import annotations

from fs_msgs.msg import ASOverride
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt8

from .channels import AS_STATE_SPEC, ChannelId
# The rate lives with the state model, which also renders it in the GUI's
# status line. Not a parameter -- it is part of what the mock is, not
# something a test should retune.
from .state import PUBLISH_PERIOD_S, PUBLISH_RATE_HZ  # noqa: F401  (re-export)

DEFAULT_AS_STATE_TOPIC = '/detached_sensor/as_state'
DEFAULT_MISSION_TOPIC = '/detached_sensor/mission'
DEFAULT_GO_TOPIC = '/detached_sensor/go'
DEFAULT_OVERRIDE_TOPIC = '/bridge/override'

# Which message type carries each channel, and the field the value goes in.
# std_msgs/UInt8 and std_msgs/Bool both call it `data`.
MESSAGE_TYPES = {
    ChannelId.AS_STATE: UInt8,
    ChannelId.MISSION: UInt8,
    ChannelId.GO: Bool,
}

FEEDBACK_TYPE = UInt8


def state_qos() -> QoSProfile:
    """The bridge's STATE_QOS: current state, latched for late subscribers.

    TRANSIENT_LOCAL is required, not a preference: the Orchestrator subscribes
    TRANSIENT_LOCAL and creates its GO subscription late (only while the VCU
    reports READY), relying on the retained sample being delivered then.

    Consequence worth knowing when reading a test: a sample already published
    stays retained after a channel is deselected, so an Orchestrator that
    starts later still receives that last value once. Deselecting stops this
    node from publishing -- it cannot un-send what DDS kept, and the real
    bridge behaves the same way.
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def feedback_qos() -> QoSProfile:
    """The Orchestrator's LATCHED_QOS, which it publishes the override with.

    TRANSIENT_LOCAL matters on this side too: the Orchestrator publishes its
    state from construction, so a mock started afterwards is shown the current
    value at once instead of waiting for the next tick.
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def verify_as_state_numbering() -> None:
    """Fail loudly if our AS state numbers and fs_msgs/ASOverride disagree.

    The same five values live in fs_uart_protocol_v2.ASState, in
    ASOverride.msg's constants and in channels.AS_STATE_SPEC. ASOverride
    defines constants for the three states the bridge accepts as an override,
    so those three are what can be cross-checked; a renumbering on either side
    should be a startup failure, not a wrong value on the wire.
    """
    expected = {
        'AS_DRIVING': ASOverride.AS_STATE_DRIVING,
        'AS_FINISHED': ASOverride.AS_STATE_FINISHED,
        'AS_EMERGENCY': ASOverride.AS_STATE_EMERGENCY,
    }
    for label, constant in expected.items():
        ours = AS_STATE_SPEC.value(label).payload
        if ours != constant:
            raise RuntimeError(
                f'AS state numbering disagrees: {label} is {ours} in '
                f'channels.py and {constant} in fs_msgs/ASOverride'
            )


def describe_as_state(value: int) -> str:
    """Name an AS state number for display, or mark it unknown."""
    for channel_value in AS_STATE_SPEC.values:
        if channel_value.payload == value:
            return channel_value.label
    return f'UNKNOWN({value})'
