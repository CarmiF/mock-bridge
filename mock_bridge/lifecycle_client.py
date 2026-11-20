"""Watch the managed nodes' lifecycle state, and switch it on request.

This is the client half of the lifecycle: the mock bridge node owns one of
these, the GUI draws what it reports and asks it for transitions. It talks to
exactly the interfaces the Orchestrator's own `LifecycleManager` talks to --
`/<node>/get_state`, `/<node>/change_state` and `/<node>/transition_event`
(`lifecycle.service_names`) -- so driving a node from the GUI and the
Orchestrator driving it are the same thing as far as the node is concerned.

How the state shown stays current, both halves being necessary:

  * **`transition_event`** is the push. A node publishes one per transition,
    so a change made by anyone -- this GUI, the Orchestrator, `ros2 lifecycle
    set` -- shows up immediately.
  * **`get_state`** is the poll, once a second. It is what gives the right
    answer for a node that was already running before the GUI started (no
    event to have missed), and what notices a node that has gone away.

Threading: everything that touches a client, a future or a timer runs on the
ROS executor thread. The GUI thread only ever appends to a deque
(`request()`) and reads a snapshot under the lock -- it never calls a service
itself, because rclpy's clients are not safe to call from a second thread
while the executor spins.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable, Optional, Sequence

from lifecycle_msgs.msg import State as StateMsg
from lifecycle_msgs.msg import Transition as TransitionMsg
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import ChangeState, GetState

from .lifecycle import (
    COMMANDS_BY_LABEL,
    LifecycleCommand,
    LifecycleState,
    ManagedNode,
    TransitionId,
    default_nodes,
    describe_change,
    service_names,
    state_from_id,
)

#: How often to ask every managed node what state it is in.
POLL_PERIOD_S = 1.0
#: How often the queue of GUI requests is drained onto the executor thread.
#: Short enough that a button press feels immediate.
PUMP_PERIOD_S = 0.05
#: How long one get_state / change_state call may take before it is given up
#: on. The Orchestrator allows 5 s for the same calls (config.yml
#: `lifecycle.timeout_s`); these nodes do nothing, so a reply that slow means
#: the node is gone rather than busy.
CALL_TIMEOUT_S = 3.0


def verify_numbering() -> None:
    """Fail loudly if our mirrored ids disagree with lifecycle_msgs.

    `lifecycle.py` keeps its own copy of the state and transition ids so that
    the model and the GUI need no ROS packages. That copy is only safe if it
    is checked, and a startup failure is the right place to find out -- the
    same arrangement as `ros_interface.verify_as_state_numbering()`.
    """
    expected_states = {
        LifecycleState.UNKNOWN: StateMsg.PRIMARY_STATE_UNKNOWN,
        LifecycleState.UNCONFIGURED: StateMsg.PRIMARY_STATE_UNCONFIGURED,
        LifecycleState.INACTIVE: StateMsg.PRIMARY_STATE_INACTIVE,
        LifecycleState.ACTIVE: StateMsg.PRIMARY_STATE_ACTIVE,
        LifecycleState.FINALIZED: StateMsg.PRIMARY_STATE_FINALIZED,
        LifecycleState.CONFIGURING: StateMsg.TRANSITION_STATE_CONFIGURING,
        LifecycleState.CLEANINGUP: StateMsg.TRANSITION_STATE_CLEANINGUP,
        LifecycleState.SHUTTINGDOWN: StateMsg.TRANSITION_STATE_SHUTTINGDOWN,
        LifecycleState.ACTIVATING: StateMsg.TRANSITION_STATE_ACTIVATING,
        LifecycleState.DEACTIVATING: StateMsg.TRANSITION_STATE_DEACTIVATING,
        LifecycleState.ERRORPROCESSING: StateMsg.TRANSITION_STATE_ERRORPROCESSING,
    }
    expected_transitions = {
        TransitionId.CREATE: TransitionMsg.TRANSITION_CREATE,
        TransitionId.CONFIGURE: TransitionMsg.TRANSITION_CONFIGURE,
        TransitionId.CLEANUP: TransitionMsg.TRANSITION_CLEANUP,
        TransitionId.ACTIVATE: TransitionMsg.TRANSITION_ACTIVATE,
        TransitionId.DEACTIVATE: TransitionMsg.TRANSITION_DEACTIVATE,
        TransitionId.UNCONFIGURED_SHUTDOWN:
            TransitionMsg.TRANSITION_UNCONFIGURED_SHUTDOWN,
        TransitionId.INACTIVE_SHUTDOWN:
            TransitionMsg.TRANSITION_INACTIVE_SHUTDOWN,
        TransitionId.ACTIVE_SHUTDOWN: TransitionMsg.TRANSITION_ACTIVE_SHUTDOWN,
        TransitionId.DESTROY: TransitionMsg.TRANSITION_DESTROY,
    }
    for ours, theirs in (*expected_states.items(), *expected_transitions.items()):
        if int(ours) != int(theirs):
            raise RuntimeError(
                f'lifecycle numbering disagrees: {ours.name} is {int(ours)} in '
                f'lifecycle.py and {int(theirs)} in lifecycle_msgs'
            )


class _Call:
    """One service call in flight, with the time it has to answer in."""

    __slots__ = ('future', 'deadline', 'label')

    def __init__(self, future, label: str = '') -> None:
        self.future = future
        self.deadline = time.monotonic() + CALL_TIMEOUT_S
        self.label = label

    @property
    def expired(self) -> bool:
        return time.monotonic() > self.deadline


class LifecycleClient:
    """Tracks the managed nodes' states on `node`, and transitions them.

    `on_event` (a log line) and `on_change` (the new `ManagedNode`) are called
    **from the ROS executor thread**; a GUI must marshal them onto its own,
    which `gui.main` does with a queued signal.
    """

    def __init__(
        self,
        node,
        managed: Optional[Sequence[ManagedNode]] = None,
        on_event: Optional[Callable[[str], None]] = None,
        on_change: Optional[Callable[[ManagedNode], None]] = None,
    ) -> None:
        verify_numbering()

        self._node = node
        self._on_event = on_event
        self._on_change = on_change
        self._lock = threading.Lock()

        managed = tuple(managed if managed is not None else default_nodes())
        self._order = tuple(m.name for m in managed)
        self._managed = {m.name: m for m in managed}

        self._get_clients: dict[str, object] = {}
        self._change_clients: dict[str, object] = {}
        self._subscriptions: list[object] = []
        self._pending_get: dict[str, _Call] = {}
        self._pending_change: dict[str, _Call] = {}

        # Appended to by the GUI thread, drained by `_pump` on the executor
        # thread. A deque's append/popleft are atomic, which is all that is
        # needed here.
        self._requests: deque = deque()

        for name in self._order:
            get_name, change_name, event_name = service_names(name)
            self._get_clients[name] = node.create_client(GetState, get_name)
            self._change_clients[name] = node.create_client(
                ChangeState, change_name)
            self._subscriptions.append(node.create_subscription(
                TransitionEvent,
                event_name,
                # Default-bound so each subscription knows its own node.
                lambda msg, name=name: self._on_transition_event(name, msg),
                10,
            ))

        self._pump_timer = node.create_timer(PUMP_PERIOD_S, self._pump)
        self._poll_timer = node.create_timer(POLL_PERIOD_S, self._poll)

        self._log(
            'managing lifecycle nodes: ' + ', '.join(
                f'/{name}' for name in self._order))

    # ------------------------------------------------------------------
    # Read: safe from any thread
    # ------------------------------------------------------------------

    @property
    def order(self) -> tuple[str, ...]:
        """The managed node names, in the order the GUI shows them."""
        return self._order

    def nodes(self) -> tuple[ManagedNode, ...]:
        """A consistent snapshot of every managed node, in GUI order."""
        with self._lock:
            return tuple(self._managed[name] for name in self._order)

    def node(self, name: str) -> ManagedNode:
        with self._lock:
            return self._managed[name]

    def snapshot(self) -> dict[str, ManagedNode]:
        with self._lock:
            return dict(self._managed)

    # ------------------------------------------------------------------
    # Write: called from the GUI thread
    # ------------------------------------------------------------------

    def request(self, name: str, command: str) -> None:
        """Ask for a transition. Queued; sent by the executor thread.

        `command` is a `lifecycle.LifecycleCommand` label ('Configure',
        'Activate', ...). The transition id is resolved when the request is
        sent, from the state then current, because 'Shutdown' means a
        different transition from each state.
        """
        if name not in self._managed:
            raise KeyError(f'{name!r} is not managed; have {list(self._order)}')
        if command not in COMMANDS_BY_LABEL:
            raise KeyError(
                f'{command!r} is not a lifecycle command; have '
                f'{list(COMMANDS_BY_LABEL)}')
        self._requests.append((name, command))

    # ------------------------------------------------------------------
    # Executor thread only
    # ------------------------------------------------------------------

    def _pump(self) -> None:
        """Send the transitions the GUI has asked for."""
        while True:
            try:
                name, label = self._requests.popleft()
            except IndexError:
                return
            self._send(name, COMMANDS_BY_LABEL[label])

    def _send(self, name: str, command: LifecycleCommand) -> None:
        state = self.node(name).state
        transition = command.transition_from(state)
        if transition is None:
            self._log(
                f'WARNING: {command.label} is not available on /{name} from '
                f'{state.label} -- ignored')
            return

        client = self._change_clients[name]
        if not client.service_is_ready():
            self._log(
                f'WARNING: cannot {command.label.lower()} /{name} -- its '
                f'change_state service is not there')
            return
        if name in self._pending_change:
            self._log(
                f'WARNING: /{name} already has a transition in flight -- '
                f'{command.label} ignored')
            return

        request = ChangeState.Request()
        request.transition = TransitionMsg(id=int(transition))
        future = client.call_async(request)
        self._pending_change[name] = _Call(future, command.label)
        future.add_done_callback(
            lambda fut, name=name: self._on_change_done(name, fut))
        self._log(f'lifecycle {name}: requested {command.label.lower()}')

    def _on_change_done(self, name: str, future) -> None:
        call = self._pending_change.pop(name, None)
        label = call.label.lower() if call is not None else 'transition'
        response = None if future.cancelled() else future.result()
        if response is None:
            self._log(f'WARNING: lifecycle {name}: {label} got no answer')
        elif not response.success:
            # rcl refuses a transition that is not legal from the current
            # state. Worth saying out loud: the GUI greys those buttons out,
            # so seeing this means the state moved under us.
            self._log(f'WARNING: lifecycle {name}: {label} was refused')
        # The resulting state arrives as a transition_event, and the poll
        # would catch it anyway, so nothing is recorded from the reply.

    def _poll(self) -> None:
        """Ask every managed node for its state; notice the ones that are gone."""
        self._expire_pending()

        for name in self._order:
            client = self._get_clients[name]
            if not client.service_is_ready():
                self._set_state(
                    name, LifecycleState.UNKNOWN,
                    f'no lifecycle services on /{name}')
                continue
            if name in self._pending_get or name in self._pending_change:
                # One call at a time per node; a transition in flight will
                # publish its own event.
                continue
            future = client.call_async(GetState.Request())
            self._pending_get[name] = _Call(future)
            future.add_done_callback(
                lambda fut, name=name: self._on_get_done(name, fut))

    def _expire_pending(self) -> None:
        """Give up on calls that were never answered."""
        for pending, what in (
            (self._pending_get, 'get_state'),
            (self._pending_change, 'change_state'),
        ):
            for name in [n for n, call in pending.items() if call.expired]:
                call = pending.pop(name)
                call.future.cancel()
                client = (self._get_clients if what == 'get_state'
                          else self._change_clients)[name]
                # Stops a late reply arriving on a future nobody awaits.
                client.remove_pending_request(call.future)
                self._log(
                    f'WARNING: lifecycle {name}: {what} timed out after '
                    f'{CALL_TIMEOUT_S:g} s')

    def _on_get_done(self, name: str, future) -> None:
        self._pending_get.pop(name, None)
        response = None if future.cancelled() else future.result()
        if response is None:
            return
        self._set_state(name, state_from_id(response.current_state.id))

    def _on_transition_event(self, name: str, msg) -> None:
        """A node reported a transition of its own, whoever asked for it."""
        self._set_state(name, state_from_id(msg.goal_state.id))

    def _set_state(
        self, name: str, state: LifecycleState, detail: str = '',
    ) -> None:
        """Record a state, and announce it if it changed."""
        with self._lock:
            previous = self._managed[name]
            if previous.state is state and previous.detail == detail:
                return
            updated = previous.with_state(state, detail)
            self._managed[name] = updated

        self._log(describe_change(updated, previous))
        if self._on_change is not None:
            self._on_change(updated)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        """Stop the timers. Safe to call twice.

        The managed nodes are deliberately left as they are: they are separate
        processes, and closing the GUI is not a reason to deactivate the stack
        underneath it.
        """
        for attr in ('_pump_timer', '_poll_timer'):
            timer = getattr(self, attr, None)
            if timer is not None:
                timer.cancel()
                self._node.destroy_timer(timer)
                setattr(self, attr, None)

    def _log(self, line: str) -> None:
        if self._on_event is not None:
            self._on_event(line)
