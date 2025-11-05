"""Bus door controller module

Implements five classes:
- DoorController
- Sensor
- Actuator
- DriverInterface
- SafetySystem

Designed for Python 3.11+. All hardware interfaces are abstract / mockable.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Protocol, Any, Dict
import time


class DoorState(Enum):
    CLOSED = 0
    OPENING = 1
    OPEN = 2
    CLOSING = 3
    STOPPED = 4
    FAULT = 5


class SensorType(Enum):
    POSITION = 0
    EDGE = 1  # not sure about this one
    OBSTACLE = 2
    LIMIT_SWITCH = 3


class ActuatorType(Enum):
    MOTOR = 0
    LOCK = 1


class DoorOperationError(RuntimeError):
    pass


class Sensor(Protocol):
    """Abstract sensor interface."""

    id: str
    type: SensorType
    healthy: bool

    def read(self) -> Any:
        """Read the raw sensor value."""

    def is_triggered(self) -> bool:
        """Return True when the sensor condition is triggered (e.g. obstacle present).
        For position sensors, this should reflect the door open/closed condition depending on the concrete sensor semantics.
        """


class Actuator(Protocol):
    """Abstract actuator interface."""

    id: str
    type: ActuatorType

    def command(self, cmd: Dict[str, Any]) -> bool:
        """Send a command to the actuator. Return True on acceptance/ack."""

    def stop(self) -> None:
        """Stop the actuator immediately."""

    def get_status(self) -> Dict[str, Any]:
        """Return a status dict for the actuator."""


class DriverInterface(Protocol):
    """Abstract driver/vehicle interface for sending commands and receiving ack/overrides."""

    connected: bool
    protocol: str

    def send_command(self, cmd: Dict[str, Any]) -> bool:
        """Send a command to the vehicle driver display/logic."""

    def receive_ack(self, timeout_s: float = 1.0) -> Optional[Dict[str, Any]]:
        """Wait for an ack or command from driver UI."""

    def request_override(self, reason: str) -> bool:
        """Request an override (e.g., to allow door to keep moving)."""


@dataclass
class SafetySystem:
    """Encapsulate safety checks and emergency stop state."""

    emergency_stop_engaged: bool = False
    obstacle_detected: bool = False
    interlock_engaged: bool = False
    last_safety_check: float = field(default_factory=time.time)

    def check_safety(self, sensors: List[Sensor]) -> bool:
        """Check sensors and update safety flags. Returns True if safe to operate."""
        self.last_safety_check = time.time()
        self.obstacle_detected = any(
            (s.type == SensorType.OBSTACLE and getattr(s, "is_triggered", lambda: False)()) # lambda to avoid error if method missing
            for s in sensors
        )
        # interlock example: if limit switch shows door fully open/closed unexpectedly
        self.interlock_engaged = any(
            (s.type == SensorType.LIMIT_SWITCH and getattr(s, "is_triggered", lambda: False)())
            for s in sensors
        )
        safe = not self.emergency_stop_engaged and not self.obstacle_detected and not self.interlock_engaged
        return safe

    def engage_emergency_stop(self, reason: str) -> None:
        self.emergency_stop_engaged = True

    def reset(self) -> None:
        self.emergency_stop_engaged = False
        self.obstacle_detected = False
        self.interlock_engaged = False


@dataclass
class DoorController:
    id: str
    sensors: List[Sensor]
    actuators: List[Actuator]
    driver_interface: Optional[DriverInterface] = None
    safety_system: SafetySystem = field(default_factory=SafetySystem)
    state: DoorState = DoorState.CLOSED
    command_timeout_s: float = 5.0

    def _find_sensor(self, t: SensorType) -> Optional[Sensor]:
        for s in self.sensors:
            if s.type == t:
                return s
        return None

    def _find_actuator(self, t: ActuatorType) -> Optional[Actuator]:
        for a in self.actuators:
            if a.type == t:
                return a
        return None

    def _check_preconditions(self) -> bool:
        # quick health check for sensors and actuators
        if any(not getattr(s, "healthy", True) for s in self.sensors):
            return False
        # actuators presence
        if not self.actuators:
            return False
        return True

    def open_door(self, timeout_s: Optional[float] = None) -> bool:
        """Blocking: attempt to open the door and return True on success.
        If operation fails or safety triggers, returns False.
        """
        timeout_s = timeout_s if timeout_s is not None else self.command_timeout_s
        if not self._check_preconditions():
            self.state = DoorState.FAULT
            raise DoorOperationError("Preconditions failed: sensors/actuators unhealthy or missing")

        if not self.safety_system.check_safety(self.sensors):
            self.state = DoorState.FAULT
            return False

        motor = self._find_actuator(ActuatorType.MOTOR)
        if motor is None:
            raise DoorOperationError("No motor actuator available")

        self.state = DoorState.OPENING
        accepted = motor.command({"action": "open"})
        if not accepted:
            self.state = DoorState.FAULT
            return False

        start = time.time()
        pos_sensor = self._find_sensor(SensorType.POSITION)
        # poll until position indicates open or timeout or safety trigger
        while time.time() - start < timeout_s:
            if not self.safety_system.check_safety(self.sensors):
                motor.stop()
                self.state = DoorState.STOPPED
                return False
            if pos_sensor is not None:
                val = pos_sensor.read()
                # assume position sensor returns True when open
                if val:
                    motor.stop()
                    self.state = DoorState.OPEN
                    return True
            time.sleep(0.01)

        motor.stop()
        self.state = DoorState.FAULT
        return False

    def close_door(self, timeout_s: Optional[float] = None) -> bool:
        timeout_s = timeout_s if timeout_s is not None else self.command_timeout_s
        if not self._check_preconditions():
            self.state = DoorState.FAULT
            raise DoorOperationError("Preconditions failed: sensors/actuators unhealthy or missing")

        if not self.safety_system.check_safety(self.sensors):
            self.state = DoorState.FAULT
            return False

        motor = self._find_actuator(ActuatorType.MOTOR)
        if motor is None:
            raise DoorOperationError("No motor actuator available")

        self.state = DoorState.CLOSING
        accepted = motor.command({"action": "close"})
        if not accepted:
            self.state = DoorState.FAULT
            return False

        start = time.time()
        pos_sensor = self._find_sensor(SensorType.POSITION)
        # assume position sensor returns False when closed
        while time.time() - start < timeout_s:
            if not self.safety_system.check_safety(self.sensors):
                motor.stop()
                # if obstacle detected while closing, attempt to reopen once
                if self.safety_system.obstacle_detected:
                    # try to open a bit to release
                    motor.command({"action": "open"})
                    self.state = DoorState.OPENING
                    time.sleep(0.05)
                self.state = DoorState.STOPPED
                return False
            if pos_sensor is not None:
                val = pos_sensor.read()
                # assume False when closed
                if not val:
                    motor.stop()
                    self.state = DoorState.CLOSED
                    return True
            time.sleep(0.01)

        motor.stop()
        self.state = DoorState.FAULT
        return False

    def stop(self) -> None:
        for a in self.actuators:
            try:
                a.stop()
            except Exception:
                pass
        self.state = DoorState.STOPPED

    def emergency_unlock(self) -> None:
        # attempt to stop actuators and transition to STOPPED
        self.safety_system.engage_emergency_stop("emergency_unlock")
        self.stop()

    def status_report(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "state": self.state.value,
            "safety": {
                "emergency_stop": self.safety_system.emergency_stop_engaged,
                "obstacle": self.safety_system.obstacle_detected,
                "interlock": self.safety_system.interlock_engaged,
            },
            "actuators": [a.get_status() for a in self.actuators],
        }
