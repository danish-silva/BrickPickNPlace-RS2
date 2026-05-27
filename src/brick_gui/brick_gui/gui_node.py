#!/usr/bin/env python3
"""
GUI node for the LeBrick n' Place pick-and-place system.

Flow on startup:
  1. ModeSelectionDialog asks the user to choose a pick_filter:
       nearest / red / green
  2. The chosen filter is sent to /brick_command as 'filter:<mode>'.
  3. MainWindow opens with start/pause/stop AND a "Use microphone"
     toggle. The microphone toggle publishes a std_msgs/Bool on
     /voice_enabled so the voice_interface nodes can be gated by it.
     GUI buttons and the microphone work simultaneously — the toggle
     only controls whether voice commands are passed through.

Topics:
  Publishes:   /brick_command   (std_msgs/String) -> brick_interaction
  Publishes:   /voice_enabled   (std_msgs/Bool)   -> voice_interface
  Subscribes:  /system_status   (std_msgs/String) <- brick_interaction

Qt Designer layout: brick_gui/ui/main_window.ui
"""

import os
import sys
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDialog,
    QDialogButtonBox,
    QLabel,
    QMainWindow,
    QRadioButton,
    QVBoxLayout,
)
from PyQt5 import uic


# ------------------------------------------------------------------ #
# ROS2 node                                                            #
# ------------------------------------------------------------------ #

class BrickGuiNode(Node):
    """Thin ROS2 node — pub/sub only, no Qt imports."""

    def __init__(self) -> None:
        super().__init__('brick_gui')

        self._cmd_pub   = self.create_publisher(String, '/brick_command', 10)
        self._voice_pub = self.create_publisher(Bool,   '/voice_enabled', 10)

        self.create_subscription(
            String, '/system_status', self._status_callback, 10
        )

        # Set by MainWindow after construction to emit the Qt signal
        self.on_status_update = None  # type: callable | None

        self.get_logger().info('BrickGuiNode started.')

    def send_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self._cmd_pub.publish(msg)
        self.get_logger().info(f'Sent command: {command}')

    def send_voice_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self._voice_pub.publish(msg)
        self.get_logger().info(f'Voice enabled = {enabled}')

    def _status_callback(self, msg: String) -> None:
        if self.on_status_update is not None:
            self.on_status_update(msg.data)


# ------------------------------------------------------------------ #
# Mode-selection dialog (shown first)                                  #
# ------------------------------------------------------------------ #

class ModeSelectionDialog(QDialog):
    """First screen: user picks the pick_filter mode."""

    MODES = [
        ('regular', 'Closest large bricks (4×2)',
         'Only picks 4×2 / "regular" bricks. Ties broken by distance.'),
        ('small',   'Closest small bricks (3×2)',
         'Only picks 3×2 / "small" bricks. Ties broken by distance.'),
        ('red',     'Red bricks only',
         'Only picks red bricks (any size). Ties broken by distance.'),
        ('green',   'Green bricks only',
         'Only picks green bricks (any size). Ties broken by distance.'),
    ]

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("LeBrick n' Place — Choose pick mode")
        self.setMinimumWidth(420)

        layout = QVBoxLayout(self)

        header = QLabel('Which bricks should the robot pick?')
        header.setStyleSheet('font-size: 14px; font-weight: bold;')
        layout.addWidget(header)

        helper = QLabel('You can change this later from the terminal '
                        'with:  ros2 param set /brick_interaction '
                        'pick_filter <mode>')
        helper.setStyleSheet('color: #666; font-size: 11px;')
        helper.setWordWrap(True)
        layout.addWidget(helper)

        layout.addSpacing(8)

        self._radios = {}
        for key, label, desc in self.MODES:
            radio = QRadioButton(label)
            radio.setStyleSheet('font-size: 13px;')
            radio.setToolTip(desc)
            layout.addWidget(radio)
            sub = QLabel(f'    {desc}')
            sub.setStyleSheet('color: #888; font-size: 11px; margin-bottom: 4px;')
            layout.addWidget(sub)
            self._radios[key] = radio

        # Default selection — first mode in MODES.
        self._radios[self.MODES[0][0]].setChecked(True)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        buttons.button(QDialogButtonBox.Ok).setText('Continue →')
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def chosen_mode(self) -> str:
        for key, radio in self._radios.items():
            if radio.isChecked():
                return key
        return self.MODES[0][0]


# ------------------------------------------------------------------ #
# Qt MainWindow                                                        #
# ------------------------------------------------------------------ #

class _SignalBridge(QObject):
    """QObject that owns the cross-thread Qt signal."""
    status_received = pyqtSignal(str)


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
    """Loads main_window.ui and wires buttons + mic toggle to the ROS node."""

    def __init__(self, ros_node: BrickGuiNode, pick_mode: str) -> None:
        super().__init__()

        ui_path = os.path.join(os.path.dirname(__file__), 'ui', 'main_window.ui')
        uic.loadUi(ui_path, self)

        # Wire start/pause/stop buttons → ROS commands
        self.startButton.clicked.connect(lambda: ros_node.send_command('start'))
        self.pauseButton.clicked.connect(lambda: ros_node.send_command('pause'))
        self.stopButton.clicked.connect(lambda: ros_node.send_command('stop'))

        # Microphone toggle — added programmatically into the left panel.
        # GUI buttons remain fully usable while the mic is on.
        self.micCheckBox = QCheckBox('Use microphone')
        self.micCheckBox.setToolTip(
            'When enabled, voice commands from voice_interface are '
            'passed through. GUI buttons still work simultaneously.'
        )
        self.micCheckBox.setStyleSheet('font-size: 12px; margin-top: 8px;')
        self.micCheckBox.stateChanged.connect(
            lambda state: ros_node.send_voice_enabled(state == Qt.Checked)
        )
        # The leftLayout (QVBoxLayout) is created by uic.loadUi from
        # main_window.ui; insert the toggle below the existing buttons.
        if hasattr(self, 'leftLayout'):
            self.leftLayout.addWidget(self.micCheckBox)

        # Thread-safe bridge: ROS thread → Qt main thread
        self._bridge = _SignalBridge()
        self._bridge.status_received.connect(self._update_status)
        ros_node.on_status_update = self._bridge.status_received.emit

        # Push the user's choice from the dialog to /brick_command.
        # The interaction node now understands 'filter:<mode>'.
        ros_node.send_command(f'filter:{pick_mode}')

        # Send the initial voice-enabled state (False on startup).
        ros_node.send_voice_enabled(False)

        self._append_log(f"Pick filter set to '{pick_mode}'.")
        self._append_log('System ready. Waiting for start command.')
        self.statusBar.showMessage(
            'ROS 2 Node: brick_gui  |  /brick_command  /system_status  /voice_enabled'
        )

    def _update_status(self, status: str) -> None:
        key = status.lower()
        text_style, bg_style = _STATE_STYLES.get(key, ('color: #666;', 'background: #ebebeb;'))

        self.statusLabel.setText(f'●  {status.upper()}')
        self.statusLabel.setStyleSheet(
            f'font-size: 13px; font-weight: bold; {text_style} {bg_style} '
            f'padding: 6px; border-radius: 6px;'
        )

        now = datetime.now().strftime('%H:%M:%S')
        self.lastUpdateLabel.setText(f'Last Update:    {now}')
        self.currentTaskLabel.setText(
            f'Current Task:    {_TASK_NAMES.get(key, status)}'
        )

        self._append_log(f'Status → {status.upper()}')

    def _append_log(self, message: str) -> None:
        now = datetime.now().strftime('%H:%M:%S')
        self.logTextEdit.append(f'[{now}]  {message}')


# ------------------------------------------------------------------ #
# Entry point                                                          #
# ------------------------------------------------------------------ #

def main(args=None) -> None:
    rclpy.init(args=args)
    ros_node = BrickGuiNode()

    # Spin ROS in a daemon thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)

    # 1) Mode-selection dialog
    selector = ModeSelectionDialog()
    if selector.exec_() != QDialog.Accepted:
        rclpy.shutdown()
        sys.exit(0)
    chosen = selector.chosen_mode()

    # 2) Main control window
    window = MainWindow(ros_node, pick_mode=chosen)
    window.show()

    exit_code = app.exec_()

    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
