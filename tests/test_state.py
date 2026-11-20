"""Toggle rules of the input state model (PLAN.md "Common toggle behavior").

Each test names the rule or test scenario it covers.
"""

import pytest

from mock_bridge import BridgeState, ChannelId, ClickAction
from mock_bridge.channels import AS_STATE_SPEC, GO_SPEC, MISSION_SPEC


@pytest.fixture
def state():
    return BridgeState()


# Rule 1 / scenario 1: nothing publishes until the first click.
def test_startup_publishes_nothing(state):
    assert state.publishing() == {}
    assert not state.is_publishing_anything
    for channel in state:
        assert channel.selected is None
        assert channel.status_text == 'STOPPED'


# Rule 2: clicking an inactive value selects it and starts publishing it.
def test_click_selects_and_publishes(state):
    result = state.click(ChannelId.AS_STATE, 'AS_OFF')

    assert result.action is ClickAction.SELECTED
    assert result.previous is None
    assert result.value.label == 'AS_OFF'
    assert state.publishing() == {ChannelId.AS_STATE: AS_STATE_SPEC.value('AS_OFF')}
    assert state.channel(ChannelId.AS_STATE).status_text == 'PUBLISHING 10 Hz'
    assert state.channel(ChannelId.AS_STATE).is_selected('AS_OFF')


# Rule 3 / scenario 2: clicking the selected value stops the channel entirely.
def test_second_click_stops_the_channel(state):
    state.click(ChannelId.AS_STATE, 'AS_OFF')
    result = state.click(ChannelId.AS_STATE, 'AS_OFF')

    assert result.action is ClickAction.DESELECTED
    assert result.previous.label == 'AS_OFF'
    assert result.value is None
    assert not result.is_publishing
    assert ChannelId.AS_STATE not in state.publishing()
    assert state.channel(ChannelId.AS_STATE).status_text == 'STOPPED'


# Rule 5: stopping sends no OFF / NONE / False / reset value.
def test_stopping_leaves_no_value_to_publish(state):
    state.click(ChannelId.GO, 'True')
    state.click(ChannelId.GO, 'True')

    assert state.publishing() == {}
    assert state.channel(ChannelId.GO).selected is None


# Rule 4: a different value in the same channel switches immediately.
def test_click_other_value_switches(state):
    state.click(ChannelId.AS_STATE, 'AS_READY')
    result = state.click(ChannelId.AS_STATE, 'AS_DRIVING')

    assert result.action is ClickAction.SWITCHED
    assert result.previous.label == 'AS_READY'
    assert result.value.label == 'AS_DRIVING'
    assert state.publishing() == {
        ChannelId.AS_STATE: AS_STATE_SPEC.value('AS_DRIVING')
    }
    assert not state.channel(ChannelId.AS_STATE).is_selected('AS_READY')


# Rule 6 / scenario 3: switching AS state leaves Mission and GO alone.
def test_switching_one_channel_leaves_the_others(state):
    state.click(ChannelId.GO, 'True')
    state.click(ChannelId.MISSION, 'SKIDPAD')
    state.click(ChannelId.AS_STATE, 'AS_READY')
    state.click(ChannelId.AS_STATE, 'AS_DRIVING')

    assert state.publishing() == {
        ChannelId.AS_STATE: AS_STATE_SPEC.value('AS_DRIVING'),
        ChannelId.MISSION: MISSION_SPEC.value('SKIDPAD'),
        ChannelId.GO: GO_SPEC.value('True'),
    }


# Scenario 6: deselecting one channel leaves the other active ones publishing.
def test_deselecting_one_channel_leaves_the_others(state):
    state.click(ChannelId.AS_STATE, 'AS_DRIVING')
    state.click(ChannelId.MISSION, 'TRACKDRIVE')
    state.click(ChannelId.GO, 'True')

    state.click(ChannelId.MISSION, 'TRACKDRIVE')

    publishing = state.publishing()
    assert set(publishing) == {ChannelId.AS_STATE, ChannelId.GO}
    assert publishing[ChannelId.GO].payload is True


# Scenario 4: True, then False, then silence.
def test_go_true_false_then_silence(state):
    first = state.click(ChannelId.GO, 'True')
    assert first.value.payload is True

    second = state.click(ChannelId.GO, 'False')
    assert second.action is ClickAction.SWITCHED
    assert second.value.payload is False
    assert state.publishing()[ChannelId.GO].payload is False

    third = state.click(ChannelId.GO, 'False')
    assert third.action is ClickAction.DESELECTED
    assert state.publishing() == {}


def test_as_state_payloads_match_the_real_protocol():
    # fs_uart_protocol_v2.ASState: OFF=0, READY=1, DRIVING=2, FINISHED=3, EMERGENCY=4.
    assert [(v.label, v.payload) for v in AS_STATE_SPEC.values] == [
        ('AS_OFF', 0),
        ('AS_READY', 1),
        ('AS_DRIVING', 2),
        ('AS_FINISHED', 3),
        ('AS_EMERGENCY', 4),
    ]


def test_unknown_label_is_rejected(state):
    with pytest.raises(KeyError):
        state.click(ChannelId.AS_STATE, 'AS_FLYING')
    assert state.publishing() == {}


def test_stop_all_reports_only_the_channels_that_were_publishing(state):
    state.click(ChannelId.AS_STATE, 'AS_FINISHED')
    state.click(ChannelId.GO, 'False')

    results = state.stop_all()

    assert {r.channel_id for r in results} == {ChannelId.AS_STATE, ChannelId.GO}
    assert all(r.action is ClickAction.DESELECTED for r in results)
    assert state.publishing() == {}
    assert state.stop_all() == []


def test_describe_lines_for_the_event_log(state):
    assert state.click(ChannelId.AS_STATE, 'AS_READY').describe() == (
        'as_state: AS_READY selected -- publishing 10 Hz'
    )
    assert state.click(ChannelId.AS_STATE, 'AS_DRIVING').describe() == (
        'as_state: AS_READY -> AS_DRIVING -- publishing 10 Hz'
    )
    assert state.click(ChannelId.AS_STATE, 'AS_DRIVING').describe() == (
        'as_state: AS_DRIVING deselected -- stopped'
    )


def test_channels_iterate_in_gui_order(state):
    assert [c.channel_id for c in state] == [
        ChannelId.AS_STATE,
        ChannelId.MISSION,
        ChannelId.GO,
    ]
    assert [c.spec.title for c in state] == ['AS State', 'Mission', 'GO']
