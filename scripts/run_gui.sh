#!/bin/bash
# Start the mock bridge GUI in its container, with X access granted for as
# long as it runs.
#
#   ./scripts/run_gui.sh [--build]
set -euo pipefail

cd "$(dirname "$0")/.."

if [[ -z "${DISPLAY:-}" ]]; then
    echo "DISPLAY is not set -- is this a graphical session?" >&2
    echo "For a terminal-only host use ./scripts/run_console.sh instead." >&2
    exit 1
fi

# The container runs as root, so it needs to be let in. local: covers only
# connections over the local socket, which is all the container uses --
# `xhost +` would open the display to the network.
revoke() { xhost -local:root >/dev/null 2>&1 || true; }
trap revoke EXIT
xhost +local:root >/dev/null

docker compose up "$@"
