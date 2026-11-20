#!/bin/bash
# Source ROS and this workspace, then run whatever the container was given.
# `exec` keeps the command as PID 1's direct child so the stop signal reaches
# it (docker-compose.yml sends SIGINT, not SIGTERM).
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${ROS_WS}/install/setup.bash"
exec "$@"
