"""Shared mock sensors/actuators for simulation and tests.

These mocks aim to be feature-compatible with the classes used across tests
and the simulator script.
"""
from __future__ import annotations

import time
from typing import Any, Dict
from bus_door_controller import SensorType, ActuatorType


class MockSensor:
    def __init__(self, id: str, type: SensorType, initial_value: bool = False, healthy: bool = True):
        self.id = id
        self.type = type
        self._value = initial_value
        self.healthy = healthy
        self._last_read = time.time()
        self._error = None

    def read(self) -> Any:
        if not self.healthy:
            raise RuntimeError(f"Sensor {self.id} failed")
        self._last_read = time.time()
        return self._value

    def is_triggered(self) -> bool:
        if not self.healthy:
            raise RuntimeError(f"Sensor {self.id} failed")
        return bool(self._value)

    def set(self, v: bool) -> None:
        self._value = v
        self._last_read = time.time()

    def simulate_failure(self) -> None:
        self.healthy = False

    def self_check(self) -> bool:
        """Perform self-diagnostic check."""
        return self.healthy

    def report_error(self, error_msg: str) -> None:
        """Report an error condition."""
        self._error = error_msg
        self.healthy = False

    def update_value(self, new_value: Any) -> None:
        """Update sensor value."""
        self._value = new_value
        self._last_read = time.time()


class MockSpeedSensor(MockSensor):
    """Mock speed sensor that returns a numeric speed via read().

    is_triggered() returns True when speed > 0.0.
    """
    def __init__(self, id: str, initial_speed: float = 0.0, healthy: bool = True):
        super().__init__(id, SensorType.SPEED, initial_value=False, healthy=healthy)
        # store numeric speed separately
        self._speed = float(initial_speed)

    def read(self) -> float:
        if not self.healthy:
            raise RuntimeError(f"Sensor {self.id} failed")
        # return numeric speed
        return self._speed

    def is_triggered(self) -> bool:
        if not self.healthy:
            raise RuntimeError(f"Sensor {self.id} failed")
        return self._speed > 0.0

    def set_speed(self, speed: float) -> None:
        self._speed = float(speed)
        # keep _value for compatibility (bool triggered)
        self._value = self._speed > 0.0


class MockActuator:
    def __init__(self, id: str, type: ActuatorType):
        self.id = id
        self.type = type
        self.last_command: Dict[str, Any] | None = None
        self.stopped = True
        self._functional = True
        self._stuck = False

    def command(self, cmd: Dict[str, Any]) -> bool:
        if not self._functional:
            return False
        self.last_command = cmd
        # manage stopped flag
        action = cmd.get("action")
        if action in ("open", "close"):
            self.stopped = False
        elif action == "stop":
            self.stopped = True

        # if stuck, accept but won't lead to completion
        return True

    def stop(self) -> None:
        self.stopped = True
        self.last_command = None

    def get_status(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "type": self.type.value if hasattr(self.type, "value") else str(self.type),
            "stopped": self.stopped,
            "functional": self._functional,
            "stuck": self._stuck,
            "last_command": self.last_command,
        }

    def simulate_failure(self) -> None:
        self._functional = False
        self.stopped = True

    def simulate_stuck(self) -> None:
        self._stuck = True


class MockDriverInterface:
    def __init__(self) -> None:
        self.connected = True
        self.protocol = "mock"
        self.override_requested = False
        self.last_command: Dict[str, Any] | None = None
        self._should_accept = True
        # Physical/UI elements
        self.openButton: bool = False
        self.closeButton: bool = False
        self.emergencyStopButton: bool = False
        self.statusLED: bool = False
        self.last_error: str | None = None

    def send_command(self, cmd: Dict[str, Any]) -> bool:
        self.last_command = cmd
        return self._should_accept

    def receive_ack(self, timeout_s: float = 1.0):
        return {"status": "ok"} if self._should_accept else None

    def request_override(self, reason: str) -> bool:
        self.override_requested = True
        return self._should_accept

    def simulate_disconnected(self) -> None:
        self.connected = False
        self._should_accept = False

    # New UI methods
    def showStatus(self) -> None:
        """Update the driver UI status LED (simple mock implementation)."""
        # Toggle status LED to indicate activity
        self.statusLED = True

    def showError(self, msg: str) -> None:
        """Display an error on the driver UI (mock stores last_error)."""
        self.last_error = msg
        self._last_error = msg  # For test verification
        # When error occurs, clear status LED
        self.statusLED = False
