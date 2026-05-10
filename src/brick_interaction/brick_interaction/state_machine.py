#!/usr/bin/env python3
"""
State machine for the brick pick-and-place system.

Pure Python — no ROS imports. This makes it independently testable and easy
to swap out for a py_trees behaviour tree later by replacing only this file
and the call site in interaction_node.py.

Valid state transitions:
    IDLE      + "start"          -> RUNNING
    RUNNING   + "pause"          -> PAUSED
    RUNNING   + "stop"           -> IDLE
    RUNNING   + motion success   -> COMPLETED
    RUNNING   + motion failure   -> ERROR
    PAUSED    + "start"          -> RUNNING
    PAUSED    + "stop"           -> IDLE
    COMPLETED + "start"          -> RUNNING (start next cycle)
    COMPLETED + "stop"           -> IDLE
    ERROR     + "start"          -> RUNNING (retry after error)
    ERROR     + "stop"           -> IDLE
    ANY       + "stop"           -> IDLE   (always safe to stop)
"""

import enum
from typing import Callable


class SystemState(enum.Enum):
    IDLE = 'idle'
    RUNNING = 'running'
    PAUSED = 'paused'
    COMPLETED = 'completed'
    ERROR = 'error'


# Maps (current_state, command_string) -> next_state
# Commands from the GUI: "start", "pause", "stop"
_COMMAND_TRANSITIONS: dict[tuple[SystemState, str], SystemState] = {
    (SystemState.IDLE,      'start'): SystemState.RUNNING,
    (SystemState.RUNNING,   'pause'): SystemState.PAUSED,
    (SystemState.RUNNING,   'stop'):  SystemState.IDLE,
    (SystemState.PAUSED,    'start'): SystemState.RUNNING,
    (SystemState.PAUSED,    'stop'):  SystemState.IDLE,
    (SystemState.COMPLETED, 'start'): SystemState.RUNNING,
    (SystemState.COMPLETED, 'stop'):  SystemState.IDLE,
    (SystemState.ERROR,     'start'): SystemState.RUNNING,
    (SystemState.ERROR,     'stop'):  SystemState.IDLE,
}


class StateMachine:
    """
    Deterministic state machine for the pick-and-place execution cycle.

    Usage:
        sm = StateMachine(on_state_change=lambda s: print(s.value))
        sm.handle_command('start')   # IDLE -> RUNNING
        sm.set_completed()           # RUNNING -> COMPLETED
    """

    def __init__(self, on_state_change: Callable[[SystemState], None]) -> None:
        """
        Args:
            on_state_change: called with the new SystemState whenever the
                             state actually changes. Used by interaction_node
                             to publish /system_status.
        """
        self._state = SystemState.IDLE
        self._on_state_change = on_state_change

    @property
    def state(self) -> SystemState:
        return self._state

    def handle_command(self, command: str) -> bool:
        """
        Process a GUI command string ('start', 'pause', 'stop').

        Returns:
            True  if the command caused a valid state transition.
            False if the command is not valid in the current state
                  (caller should log a warning).
        """
        next_state = _COMMAND_TRANSITIONS.get((self._state, command.strip().lower()))
        if next_state is None:
            return False
        self._transition(next_state)
        return True

    def set_completed(self) -> None:
        """Called by MotionClient when the MoveGroup action succeeds."""
        if self._state == SystemState.RUNNING:
            self._transition(SystemState.COMPLETED)

    def set_error(self) -> None:
        """Called by MotionClient when the MoveGroup action fails or is rejected."""
        if self._state == SystemState.RUNNING:
            self._transition(SystemState.ERROR)

    def _transition(self, next_state: SystemState) -> None:
        if next_state != self._state:
            self._state = next_state
            self._on_state_change(self._state)
