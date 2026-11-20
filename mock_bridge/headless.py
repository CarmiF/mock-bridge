"""A keyboard console for the mock bridge -- stage 2's way to drive it.

The GUI is stage 3; until it exists this is how the PLAN.md test scenarios get
run against the real Orchestrator. It is a thin shell over the same node and
state model the GUI will use, so what it proves carries over.

    ros2 run mock_bridge mock_bridge_console

Commands (the channel name may be abbreviated a/m/g):

    as_state AS_READY     select or switch
    as_state              click the selected value again, i.e. stop the channel
    lc                    the managed nodes' lifecycle states
    lc control activate   ask a managed node for a transition
    status                what is publishing, and the last feedback
    help / quit
"""

from __future__ import annotations

import sys
from typing import Optional

from .channels import CHANNEL_SPECS, ChannelId
from .lifecycle import COMMANDS, COMMANDS_BY_LABEL
from .ros_interface import PUBLISH_RATE_HZ
from .runner import RosRunner

_ALIASES = {
    'a': ChannelId.AS_STATE, 'as_state': ChannelId.AS_STATE, 'as': ChannelId.AS_STATE,
    'm': ChannelId.MISSION, 'mission': ChannelId.MISSION,
    'g': ChannelId.GO, 'go': ChannelId.GO,
}


def _usage() -> str:
    lines = [f'Publishing is {PUBLISH_RATE_HZ:g} Hz while a value is selected.']
    for spec in CHANNEL_SPECS:
        lines.append(f'  {spec.channel_id.value:<9} {" | ".join(spec.labels)}')
    lines.append('  a channel name with no value deselects it (stops that channel)')
    lines.append('  lc [<node> <'
                 + '|'.join(c.label.lower() for c in COMMANDS) + '>]')
    lines.append('  status | help | quit')
    return '\n'.join(lines)


def _status(node) -> str:
    lines = []
    for channel in node.state:
        selected = channel.selected.label if channel.selected else '-'
        lines.append(
            f'  {channel.spec.title:<9} {selected:<13} {channel.status_text}')
    feedback = node.latest_feedback()
    lines.append(
        '  orchestrator state/override: '
        + (feedback.describe() if feedback else 'nothing received yet'))
    lines.append(_lifecycle(node))
    return '\n'.join(lines)


def _lifecycle(node) -> str:
    lines = []
    for managed in node.lifecycle.nodes():
        available = ', '.join(c.label.lower() for c in managed.commands())
        lines.append(
            f'  {managed.title:<9} /{managed.name:<22} '
            f'{managed.status_text:<28} {available or "-"}')
    return '\n'.join(lines)


def _handle_lifecycle(node, parts: list[str]) -> None:
    """`lc` on its own lists the states; `lc <node> <command>` transitions one."""
    if len(parts) == 1:
        print(_lifecycle(node))
        return
    if len(parts) != 3:
        print('usage: lc [<node> <'
              + '|'.join(c.label.lower() for c in COMMANDS) + '>]')
        return

    name, wanted = parts[1], parts[2].lower()
    labels = {label.lower(): label for label in COMMANDS_BY_LABEL}
    if wanted not in labels:
        print(f'unknown transition {wanted!r}; one of '
              + ', '.join(sorted(labels)))
        return
    try:
        node.lifecycle.request(name.lstrip('/'), labels[wanted])
    except KeyError as exc:
        print(exc.args[0])
        return
    print(f'  requested {wanted} on /{name.lstrip("/")}')


def _handle(node, line: str) -> bool:
    """Run one console line. Returns False to quit."""
    parts = line.split()
    if not parts:
        return True

    command = parts[0].lower()
    if command in ('quit', 'exit', 'q'):
        return False
    if command in ('help', '?'):
        print(_usage())
        return True
    if command == 'status':
        print(_status(node))
        return True
    if command == 'lc':
        _handle_lifecycle(node, parts)
        return True

    channel_id = _ALIASES.get(command)
    if channel_id is None:
        print(f'unknown command {command!r}; try help')
        return True

    channel = node.state.channel(channel_id)
    if len(parts) == 1:
        # No value given: deselect, which is a click on the selected value.
        if channel.selected is None:
            print(f'{channel_id.value} is already stopped')
            return True
        label: Optional[str] = channel.selected.label
    else:
        label = parts[1]

    try:
        result = node.click(channel_id, label)
    except KeyError as exc:
        print(exc.args[0])
        return True
    print('  ' + result.describe())
    return True


def main(args: Optional[list[str]] = None) -> int:
    with RosRunner(args=args) as runner:
        node = runner.node
        print(_usage())
        print('nothing is being published until the first selection.')
        try:
            for line in sys.stdin:
                if not _handle(node, line.strip()):
                    break
        except KeyboardInterrupt:
            pass
        print('stopping publishers and shutting down')
    return 0


if __name__ == '__main__':
    sys.exit(main())
