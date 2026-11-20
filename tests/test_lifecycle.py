"""The managed-node model: states, what each state offers, and its text.

Pure Python, like `test_state.py`: no ROS, no Qt, no display. What this pins
down is the lifecycle's rules as the GUI relies on them -- which buttons are
live in which state, and that 'Shutdown' means a different transition from
each one.
"""

from __future__ import annotations

import pytest

from mock_bridge.lifecycle import (
    ACTIVATE,
    CLEANUP,
    COMMANDS,
    CONFIGURE,
    DEACTIVATE,
    SHUTDOWN,
    LifecycleState,
    ManagedNode,
    TransitionId,
    default_nodes,
    describe_change,
    service_names,
    state_from_id,
)


def test_default_nodes_are_the_four_in_gui_order():
    # Control and planner first: those are the two the Orchestrator's state
    # machine activates by name. mock1 and mock2 are spares.
    assert [(m.name, m.title) for m in default_nodes()] == [
        ('control', 'Control'),
        ('planner', 'Planner'),
        ('mock1', 'Mock 1'),
        ('mock2', 'Mock 2'),
    ]


def test_default_nodes_start_unknown_not_unconfigured():
    # Nothing has been asked yet, and "unconfigured" would be a guess: a node
    # that is not running has no state at all.
    for managed in default_nodes():
        assert managed.state is LifecycleState.UNKNOWN
        assert not managed.is_reachable
        assert 'unknown' in managed.status_text


def test_names_are_overridable():
    # The Orchestrator's config.yml expects its own names, and the two spares
    # are there to be pointed at whichever of them a test needs.
    managed = default_nodes(
        'control_node', 'skidpad_planner_node',
        'lidar_reader_node', 'camera_reader_node')
    assert [m.name for m in managed] == [
        'control_node', 'skidpad_planner_node',
        'lidar_reader_node', 'camera_reader_node']
    # Titles stay put: they say which slot a node is filling.
    assert [m.title for m in managed] == [
        'Control', 'Planner', 'Mock 1', 'Mock 2']


def test_status_text_names_the_state_and_its_id():
    managed = ManagedNode('control', 'Control').with_state(
        LifecycleState.ACTIVE)
    assert managed.status_text == 'ACTIVE [3]'
    assert managed.is_active


def test_unknown_status_text_says_why():
    managed = ManagedNode('control', 'Control').with_state(
        LifecycleState.UNKNOWN, 'no lifecycle services on /control')
    assert managed.status_text == 'unknown -- no lifecycle services on /control'


def test_with_state_leaves_the_original_alone():
    before = ManagedNode('control', 'Control')
    after = before.with_state(LifecycleState.INACTIVE)
    assert before.state is LifecycleState.UNKNOWN
    assert after.state is LifecycleState.INACTIVE


@pytest.mark.parametrize('state, expected', [
    (LifecycleState.UNCONFIGURED, [CONFIGURE, SHUTDOWN]),
    (LifecycleState.INACTIVE, [ACTIVATE, CLEANUP, SHUTDOWN]),
    (LifecycleState.ACTIVE, [DEACTIVATE, SHUTDOWN]),
    # Nothing comes back from finalized, and nothing can be asked of a node
    # we have not reached or one that is mid-transition.
    (LifecycleState.FINALIZED, []),
    (LifecycleState.UNKNOWN, []),
    (LifecycleState.CONFIGURING, []),
])
def test_available_commands_per_state(state, expected):
    managed = ManagedNode('control', 'Control').with_state(state)
    assert list(managed.commands()) == expected
    for command in COMMANDS:
        assert managed.allows(command) == (command in expected)


@pytest.mark.parametrize('state, transition', [
    (LifecycleState.UNCONFIGURED, TransitionId.UNCONFIGURED_SHUTDOWN),
    (LifecycleState.INACTIVE, TransitionId.INACTIVE_SHUTDOWN),
    (LifecycleState.ACTIVE, TransitionId.ACTIVE_SHUTDOWN),
])
def test_shutdown_is_a_different_transition_from_each_state(state, transition):
    # The reason a command maps state -> transition rather than holding one id.
    assert SHUTDOWN.transition_from(state) is transition


def test_shutdown_is_not_offered_from_finalized():
    assert SHUTDOWN.transition_from(LifecycleState.FINALIZED) is None


def test_the_simple_commands_have_one_transition_each():
    assert CONFIGURE.transition_from(
        LifecycleState.UNCONFIGURED) is TransitionId.CONFIGURE
    assert ACTIVATE.transition_from(
        LifecycleState.INACTIVE) is TransitionId.ACTIVATE
    assert DEACTIVATE.transition_from(
        LifecycleState.ACTIVE) is TransitionId.DEACTIVATE
    assert CLEANUP.transition_from(
        LifecycleState.INACTIVE) is TransitionId.CLEANUP


def test_state_from_id_knows_the_real_ids():
    assert state_from_id(1) is LifecycleState.UNCONFIGURED
    assert state_from_id(3) is LifecycleState.ACTIVE
    assert state_from_id(13) is LifecycleState.ACTIVATING


def test_state_from_id_does_not_guess():
    assert state_from_id(99) is LifecycleState.UNKNOWN


def test_labels_match_what_ros_prints():
    # `ros2 lifecycle get` prints these, so the GUI should too.
    assert LifecycleState.UNCONFIGURED.label == 'unconfigured'
    assert LifecycleState.CLEANINGUP.label == 'cleaningup'
    assert LifecycleState.ERRORPROCESSING.label == 'errorprocessing'


def test_only_primary_states_are_primary():
    assert LifecycleState.ACTIVE.is_primary
    assert not LifecycleState.ACTIVATING.is_primary
    assert not LifecycleState.UNKNOWN.is_primary


def test_service_names_are_the_ones_the_orchestrator_calls():
    assert service_names('control') == (
        '/control/get_state', '/control/change_state',
        '/control/transition_event')
    # An already-absolute name is not doubled up.
    assert service_names('/control')[0] == '/control/get_state'


def test_describe_change_shows_both_ends():
    before = ManagedNode('control', 'Control').with_state(
        LifecycleState.INACTIVE)
    after = before.with_state(LifecycleState.ACTIVE)
    assert describe_change(after, before) == (
        'lifecycle control: INACTIVE [2] -> ACTIVE [3]')
