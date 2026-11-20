#!/bin/bash
# Run the test suite inside the image, where ROS, fs_msgs and PySide6 all
# exist. Qt runs offscreen, and the tests use their own ROS_DOMAIN_ID, so this
# is safe to run while something else is on the graph.
#
#   ./scripts/run_tests.sh [extra pytest args]
set -euo pipefail

cd "$(dirname "$0")/.."

exec docker compose run --rm \
    -e QT_QPA_PLATFORM=offscreen \
    -e ROS_DOMAIN_ID=77 \
    mock_bridge \
    bash -c "cd /mock_bridge_ws/src/mock_bridge && python3 -m pytest tests -q $*"
