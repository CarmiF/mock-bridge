#!/bin/bash
# The keyboard console instead of the GUI: same node, no display needed.
# Useful over ssh, and for a scripted check.
#
#   ./scripts/run_console.sh
#   printf 'g True\nstatus\nquit\n' | ./scripts/run_console.sh
set -euo pipefail

cd "$(dirname "$0")/.."

# --rm: a throwaway container. --service-ports is not wanted; host networking
# is already in the service definition.
exec docker compose run --rm mock_bridge ros2 run mock_bridge mock_bridge_console
