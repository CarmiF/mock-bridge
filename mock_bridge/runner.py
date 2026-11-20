"""Spin the node on its own thread, so callbacks stay off the caller's thread.

PLAN.md step 3 asks for ROS callbacks off the GUI thread; this is the piece
that does it, and the headless console (stage 2) and the GUI (stage 3) both
use it rather than each rolling their own executor.
"""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from .node import MockBridgeNode


class RosRunner:
    """Owns the rclpy context, the node and the thread that spins it.

    Use it as a context manager:

        with RosRunner() as runner:
            runner.node.click(ChannelId.GO, 'True')
    """

    def __init__(self, args: Optional[list[str]] = None, **node_kwargs) -> None:
        self._args = args
        self._node_kwargs = node_kwargs
        self._context: Optional[rclpy.Context] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self.node: Optional[MockBridgeNode] = None

    def start(self) -> MockBridgeNode:
        self._context = rclpy.Context()
        self._context.init(args=self._args)
        self._executor = SingleThreadedExecutor(context=self._context)

        # The node takes the context explicitly, so nothing here depends on
        # rclpy's global state and a test can run a second runner.
        self.node = MockBridgeNode(context=self._context, **self._node_kwargs)
        self._executor.add_node(self.node)

        self._thread = threading.Thread(
            target=self._spin, name='mock_bridge_ros', daemon=True)
        self._thread.start()
        return self.node

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

    def stop(self) -> None:
        """Stop publishing, shut the node down and join the thread."""
        if self.node is not None:
            self.node.shutdown()
        if self._executor is not None:
            self._executor.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self._context is not None and self._context.ok():
            self._context.try_shutdown()
        self._context = None
        self._executor = None

    def __enter__(self) -> 'RosRunner':
        self.start()
        return self

    def __exit__(self, *exc_info) -> None:
        self.stop()
