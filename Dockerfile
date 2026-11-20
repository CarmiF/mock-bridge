###############################################################################
# Mock Bridge: the manual mock of fs_uart_bridge, plus its GUI, in one image.
#
# Its own container, separate from the vehicle stack (PLAN.md "Deployment").
# Nothing in here talks to a VCU, an STM32 or a UART -- there is no serial
# dependency at all, which is the point.
#
# Needs a second build context for fs_msgs, which lives in the pedal_node
# workspace and is not copied into this folder:
#
#   docker compose build              # compose passes it (docker-compose.yml)
#   docker build -t bgr_mock_bridge \
#       --build-context fs_msgs=../pedal_node/src/fs_msgs .
###############################################################################

# ros-base, not ros-core: std_msgs, rclpy and rosidl_default_generators come
# with it at versions consistent with its own Fast-CDR. The same reasoning as
# pedal_node's Dockerfile, and the same reason not to apt-install message
# packages on top of a ros-core base.
FROM ros:jazzy-ros-base

ENV ROS_DISTRO=jazzy
ENV ROS_WS=/mock_bridge_ws
ENV COLCON_LOG_PATH=/tmp/colcon_logs
ENV PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-c"]

# colcon to build, pip for PySide6, and the X libraries Qt's xcb platform
# plugin dlopens at runtime. Qt fails with "could not load the Qt platform
# plugin xcb" and no further detail when one of these is missing, so they are
# listed explicitly rather than pulled in by a desktop metapackage.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    libgl1 \
    libegl1 \
    libdbus-1-3 \
    libfontconfig1 \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xinerama0 \
    && rm -rf /var/lib/apt/lists/*

# PySide6 is not in the Ubuntu 24.04 archive, so it comes from pip -- see the
# comment in package.xml about why it is not a rosdep key. -Essentials is
# QtCore/QtGui/QtWidgets and friends, ~85 MB, without the WebEngine and
# multimedia parts this GUI has no use for. Pinned: a Qt major upgrade should
# be a deliberate change, not whatever the image happens to build on.
RUN pip install --no-cache-dir --break-system-packages PySide6-Essentials==6.12.0

WORKDIR ${ROS_WS}

# fs_msgs first, from its own build context: it changes far less often than
# this package, so the layer that builds it stays cached.
COPY --from=fs_msgs . ${ROS_WS}/src/fs_msgs
COPY . ${ROS_WS}/src/mock_bridge

# No rosdep: every dependency is either in ros-base (rclpy, std_msgs), built
# here (fs_msgs) or pip-installed above (PySide6). `rosdep install` would add
# a network round trip to every build and resolve nothing.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && mkdir -p "${COLCON_LOG_PATH}" \
    && colcon build --symlink-install --packages-select fs_msgs mock_bridge

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "mock_bridge", "mock_bridge.launch.py"]
