#!/usr/bin/env python3
"""
GUI node for the LeBrick n' Place pick-and-place system.

Architecture:
  - ROS2 node (BrickGuiNode) runs in a background daemon thread
  - PyQt5 MainWindow runs on the main thread (Qt requirement)
  - A pyqtSignal bridges status updates from the ROS thread to the Qt thread
    safely — never update Qt widgets directly from a non-main thread

Topics:
  Publishes:   /brick_command   (std_msgs/String) -> brick_interaction
  Subscribes:  /system_status   (std_msgs/String) <- brick_interaction

Qt Designer layout file: brick_gui/ui/main_window.ui
  Required widget objectNames:
    startButton        (QPushButton)
    pauseButton        (QPushButton)
    stopButton         (QPushButton)
    statusLabel        (QLabel)       — state badge
    lastUpdateLabel    (QLabel)       — last update time
    currentTaskLabel   (QLabel)       — human-readable task
    logTextEdit        (QTextEdit)    — execution log
  You can redesign the layout freely in Qt Designer — just keep those names.
"""

import os
import sys
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5 import uic


# ------------------------------------------------------------------ #
# ROS2 node                                                            #
# ------------------------------------------------------------------ #

class BrickGuiNode(Node):
    """
    Thin ROS2 node.  Only handles pub/sub — no Qt imports.

    After construction, set on_status_update to a callable that accepts
    a status string. MainWindow sets this to emit its Qt signal.
    """

    def __init__(self) -> None:
        super().__init__('brick_gui')

        self._cmd_pub = self.create_publisher(String, '/brick_command', 10)

        self.create_subscription(
            String, '/system_status', self._status_callback, 10
        )

        # Set by MainWindow after construction to emit the Qt signal
        self.on_status_update = None  # type: callable | None

        self.get_logger().info('BrickGuiNode started.')

    def send_command(self, command: str) -> None:
        """Publish a command string to /brick_command."""
        msg = String()
        msg.data = command
        self._cmd_pub.publish(msg)
        self.get_logger().info(f'Sent command: {command}')

    def _status_callback(self, msg: String) -> None:
        """
        Called in the ROS spin thread.
        Forwards to on_status_update (a Qt signal emit) which is thread-safe.
        """
        if self.on_status_update is not None:
            self.on_status_update(msg.data)


# ------------------------------------------------------------------ #
# Qt MainWindow                                                        #
# ------------------------------------------------------------------ #

class _SignalBridge(QObject):
    """Minimal QObject subclass that owns the cross-thread signal."""
    status_received = pyqtSignal(str)


# Colour scheme for each system state
_STATE_STYLES = {
    'idle':      ('color: #666666;', 'background: #ebebeb;'),
    'running':   ('color: white;',   'background: #5cb85c;'),
    'paused':    ('color: white;',   'background: #f0ad4e;'),
    'completed': ('color: white;',   'background: #5bc0de;'),
    'error':     ('color: white;',   'background: #d9534f;'),
}

_TASK_NAMES = {
    'idle':      'Waiting',
    'running':   'Pick and Place',
    'paused':    'Paused',
    'completed': 'Done',
    'error':     'Error — check terminal',
}


class MainWindow(QMainWindow):
    """
    Loads the Qt Designer .ui file and wires buttons to the ROS node.

    The _bridge.status_received signal is emitted from the ROS subscriber
    thread (via ros_node.on_status_update) and connected to _update_status,
    which runs on the Qt main thread.  This is the correct Qt way to cross
    thread boundaries safely.
    """

    def __init__(self, ros_node: BrickGuiNode) -> None:
        super().__init__()

        # Load the Qt Designer layout — populates self.<widgetName> for every
        # named widget in the .ui file
        ui_path = os.path.join(os.path.dirname(__file__), 'ui', 'main_window.ui')
        uic.loadUi(ui_path, self)

        # Wire buttons -> ROS commands
        self.startButton.clicked.connect(lambda: ros_node.send_command('start'))
        self.pauseButton.clicked.connect(lambda: ros_node.send_command('pause'))
        self.stopButton.clicked.connect(lambda: ros_node.send_command('stop'))

        # Thread-safe bridge: ROS thread emits, Qt main thread receives
        self._bridge = _SignalBridge()
        self._bridge.status_received.connect(self._update_status)

        # Give the ROS node a way to trigger the signal from its spin thread
        ros_node.on_status_update = self._bridge.status_received.emit

        # Initial log entry and status bar
        self._append_log('System ready. Waiting for start command.')
        self.statusBar.showMessage(
            'ROS 2 Node: brick_gui  |  /brick_command  /system_status'
        )

    def _update_status(self, status: str) -> None:
        """
        Update all status widgets from a /system_status message.
        Always called on the Qt main thread via the signal bridge.
        """
        key = status.lower()
        text_style, bg_style = _STATE_STYLES.get(key, ('color: #666;', 'background: #ebebeb;'))

        # State badge
        self.statusLabel.setText(f'\u25cf  {status.upper()}')
        self.statusLabel.setStyleSheet(
            f'font-size: 13px; font-weight: bold; {text_style} {bg_style} '
            f'padding: 6px; border-radius: 6px;'
        )

        # Info labels
        now = datetime.now().strftime('%H:%M:%S')
        self.lastUpdateLabel.setText(f'Last Update:    {now}')
        self.currentTaskLabel.setText(
            f'Current Task:    {_TASK_NAMES.get(key, status)}'
        )

        # Log entry
        self._append_log(f'Status \u2192 {status.upper()}')

    def _append_log(self, message: str) -> None:
        """Append a timestamped line to the execution log. Main thread only."""
        now = datetime.now().strftime('%H:%M:%S')
        self.logTextEdit.append(f'[{now}]  {message}')


# ------------------------------------------------------------------ #
# Entry point                                                          #
# ------------------------------------------------------------------ #

def main(args=None) -> None:
    rclpy.init(args=args)
    ros_node = BrickGuiNode()

    # Spin ROS in a daemon thread so it exits automatically when Qt closes
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    exit_code = app.exec_()  # blocks on the main thread until the window closes

    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
