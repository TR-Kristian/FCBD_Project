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
    OUT_OF_SERVICE = 6


@dataclass
class ExternalButton:
    """External door open request button typically mounted outside the bus at each door."""
    id: str
    door_id: str
    pressed: bool = False
    enabled: bool = True
    last_press_time: float = field(default_factory=time.time)
    
    def press(self) -> bool:
        """Simulate a button press from outside the bus.
        Returns True if the press was registered, False if button is disabled."""
        if not self.enabled:
            return False
        self.pressed = True
        self.last_press_time = time.time()
        return True
    
    def reset(self) -> None:
        """Reset button state after processing."""
        self.pressed = False
        
    def disable(self) -> None:
        """Disable the button (e.g. when bus is moving)."""
        self.enabled = False
        
    def enable(self) -> None:
        """Enable the button (e.g. when bus is at a stop)."""
        self.enabled = True
        self.pressed = False


class SensorType(Enum):
    POSITION = 0
    OBSTACLE = 1
    SPEED = 2
    LIMIT_SWITCH = 3


class ActuatorType(Enum):
    MOTOR = 0
    LOCK = 1
    EXTERNAL_BUTTON = 2


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

    def self_check(self) -> bool:
        """Perform a self-diagnostic test on the sensor.
        Returns True if the sensor is functioning correctly, False otherwise.
        """
        
    def report_error(self, error_msg: str) -> None:
        """Report an error condition with the sensor.
        Args:
            error_msg: Description of the error condition
        """
        
    def update_value(self, new_value: Any) -> None:
        """Update the sensor's current value.
        Args:
            new_value: The new value to set for the sensor
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

    # Physical / UI indicators and buttons on the driver interface
    openButton: bool
    closeButton: bool
    emergencyStopButton: bool
    statusLED: bool

    def send_command(self, cmd: Dict[str, Any]) -> bool:
        """Send a command to the vehicle driver display/logic."""

    def receive_ack(self, timeout_s: float = 1.0) -> Optional[Dict[str, Any]]:
        """Wait for an ack or command from driver UI."""

    def request_override(self, reason: str) -> bool:
        """Request an override (e.g., to allow door to keep moving)."""

    def showStatus(self) -> None:
        """Update the driver UI status (e.g., light up LEDs)."""

    def showError(self, msg: str) -> None:
        """Display or log an error message on the driver UI."""


@dataclass
class SafetySystem:
    """Encapsulate safety checks and emergency stop state."""

    emergency_stop_engaged: bool = False
    obstacle_detected: bool = False
    moving: bool = False
    interlock_engaged: bool = False
    last_safety_check: float = field(default_factory=time.time)

    def check_safety(self, sensors: List[Sensor]) -> bool:
        """Check sensors and update safety flags. Returns True if safe to operate."""
        self.last_safety_check = time.time()
        self.obstacle_detected = any(
            (s.type == SensorType.OBSTACLE and getattr(s, "is_triggered", lambda: False)()) # lambda to avoid error if method missing
            for s in sensors
        )
        # detect if any speed sensors report movement
        self.moving = any(
            (s.type == SensorType.SPEED and getattr(s, "is_triggered", lambda: False)())
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
    external_button: Optional[ExternalButton] = None
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
        """Check all preconditions for door operation. Returns True if all checks pass."""
        errors = []
        
        # Check sensor health
        unhealthy_sensors = []
        for sensor in self.sensors:
            if not getattr(sensor, "healthy", True):
                unhealthy_sensors.append(sensor.id)
        if unhealthy_sensors:
            errors.append(f"Unhealthy sensors detected: {', '.join(unhealthy_sensors)}")
            
        # Check actuator presence and types
        if not self.actuators:
            errors.append("No actuators available")
        else:
            motor = self._find_actuator(ActuatorType.MOTOR)
            if not motor:
                errors.append("Required motor actuator not found")
        
        # Report all errors to driver interface
        if errors and self.driver_interface is not None:
            try:
                for error in errors:
                    self.driver_interface.showError(error)
            except Exception as e:
                # Log communication failure but continue with safety checks
                print(f"Failed to communicate with driver interface: {e}")
        
        return len(errors) == 0

    def _check_critical_sensors(self) -> bool:
        """Check if all critical sensors are functioning properly.
        Returns True if all present critical sensors are healthy, False if any fails.
        Only checks sensors that are actually present in the configuration.
        """
        critical_sensors = [
            SensorType.POSITION,
            SensorType.OBSTACLE,
            SensorType.SPEED,
        ]
        
        for sensor_type in critical_sensors:
            sensor = self._find_sensor(sensor_type)
            # Skip if sensor is not configured
            if sensor is None:
                continue
            # Check if sensor self-check passes
            try:
                if not sensor.self_check():
                    return False
            except Exception:
                return False
        
        return True

    def set_out_of_service(self, reason: str = "Technical issue - maintenance required") -> None:
        """Set the door system out of service due to technical issues.
        This prevents further door operations and notifies the driver interface.
        
        Args:
            reason: Description of why the system is out of service
        """
        # Stop all actuators immediately
        try:
            for a in self.actuators:
                try:
                    a.stop()
                except Exception:
                    pass
        except Exception as e:
            print(f"Error stopping actuators during out-of-service: {e}")
        
        # Set state AFTER stopping actuators to preserve OUT_OF_SERVICE state
        self.state = DoorState.OUT_OF_SERVICE
        
        # Disable external button
        if self.external_button:
            self.external_button.disable()
        
        # Engage emergency stop for safety
        self.safety_system.engage_emergency_stop(reason)
        
        # Notify driver interface
        if self.driver_interface:
            try:
                self.driver_interface.showError(
                    f"BUS OUT OF SERVICE: {reason}. Please contact maintenance."
                )
            except Exception as e:
                print(f"Failed to communicate with driver interface: {e}")

    def open_door(self, timeout_s: Optional[float] = None) -> bool:
        """Blocking: attempt to open the door and return True on success.
        If operation fails or safety triggers, returns False.
        
        Raises:
            DoorOperationError: If preconditions fail or critical errors occur
        """
        # Check if system is out of service
        if self.state == DoorState.OUT_OF_SERVICE:
            error_msg = "Door system is out of service - maintenance required"
            if self.driver_interface:
                try:
                    self.driver_interface.showError(error_msg)
                except Exception as e:
                    print(f"Failed to communicate with driver interface: {e}")
            return False
        
        timeout_s = timeout_s if timeout_s is not None else self.command_timeout_s
        
        # Check critical sensors before attempting operation
        if not self._check_critical_sensors():
            error_msg = "Critical sensor failure detected - system going out of service"
            self.set_out_of_service(error_msg)
            return False
        
        # Check preconditions
        if not self._check_preconditions():
            self.state = DoorState.FAULT
            raise DoorOperationError("Preconditions failed: sensors/actuators unhealthy or missing")

        # Check safety system
        if not self.safety_system.check_safety(self.sensors):
            self.state = DoorState.FAULT
            safety_status = {
                "emergency_stop": self.safety_system.emergency_stop_engaged,
                "obstacle": self.safety_system.obstacle_detected,
                "interlock": self.safety_system.interlock_engaged
            }
            error_reasons = [k for k, v in safety_status.items() if v]
            error_msg = f"Safety check failed: {', '.join(error_reasons)}"
            if self.driver_interface:
                try:
                    self.driver_interface.showError(error_msg)
                except Exception as e:
                    print(f"Failed to communicate with driver interface: {e}")
            return False

        # Check vehicle motion
        if getattr(self.safety_system, "moving", False):
            error_msg = "Cannot open doors while vehicle is moving"
            if self.driver_interface:
                try:
                    self.driver_interface.showError(error_msg)
                except Exception as e:
                    print(f"Failed to communicate with driver interface: {e}")
            return False

        # Get motor actuator
        motor = self._find_actuator(ActuatorType.MOTOR)
        if motor is None:
            error_msg = "No motor actuator available"
            if self.driver_interface:
                try:
                    self.driver_interface.showError(error_msg)
                except Exception as e:
                    print(f"Failed to communicate with driver interface: {e}")
            raise DoorOperationError(error_msg)

        self.state = DoorState.OPENING
        # notify driver UI that operation started
        if self.driver_interface is not None:
            try:
                self.driver_interface.showStatus()
            except Exception as e:
                print(f"Failed to update driver interface status: {e}")
        
        # Send open command to motor
        try:
            accepted = motor.command({"action": "open"})
            if not accepted:
                error_msg = "Motor failed to accept open command"
                if self.driver_interface:
                    try:
                        self.driver_interface.showError(error_msg)
                    except Exception as e:
                        print(f"Failed to communicate with driver interface: {e}")
                self.state = DoorState.FAULT
                return False
        except Exception as e:
            error_msg = f"Motor command failed: {str(e)}"
            if self.driver_interface:
                try:
                    self.driver_interface.showError(error_msg)
                except Exception as comm_e:
                    print(f"Failed to communicate with driver interface: {comm_e}")
            self.state = DoorState.FAULT
            return False

        start = time.time()
        pos_sensor = self._find_sensor(SensorType.POSITION)
        if pos_sensor is None:
            error_msg = "Position sensor not found"
            if self.driver_interface:
                try:
                    self.driver_interface.showError(error_msg)
                except Exception as e:
                    print(f"Failed to communicate with driver interface: {e}")
            self.state = DoorState.FAULT
            motor.stop()
            return False

        # poll until position indicates open or timeout or safety trigger
        while time.time() - start < timeout_s:
            # Check safety conditions
            if not self.safety_system.check_safety(self.sensors):
                motor.stop()
                self.state = DoorState.STOPPED
                
                # Determine and report the safety issue
                safety_reasons = []
                if self.safety_system.emergency_stop_engaged:
                    safety_reasons.append("emergency stop engaged")
                if self.safety_system.obstacle_detected:
                    safety_reasons.append("obstacle detected")
                if self.safety_system.interlock_engaged:
                    safety_reasons.append("interlock engaged")
                
                error_msg = f"Operation stopped: {', '.join(safety_reasons)}"
                if self.driver_interface:
                    try:
                        self.driver_interface.showError(error_msg)
                    except Exception as e:
                        print(f"Failed to communicate with driver interface: {e}")
                return False

            # Check position sensor
            try:
                val = pos_sensor.read()
                # assume position sensor returns True when open
                if val:
                    motor.stop()
                    self.state = DoorState.OPEN
                    if self.driver_interface:
                        try:
                            self.driver_interface.showStatus()
                        except Exception as e:
                            print(f"Failed to update driver interface status: {e}")
                    return True
            except Exception as e:
                motor.stop()
                self.state = DoorState.FAULT
                error_msg = f"Position sensor error: {str(e)}"
                if self.driver_interface:
                    try:
                        self.driver_interface.showError(error_msg)
                    except Exception as comm_e:
                        print(f"Failed to communicate with driver interface: {comm_e}")
                raise DoorOperationError(error_msg)
            
            time.sleep(0.01)

        # Operation timed out
        motor.stop()
        self.state = DoorState.FAULT
        error_msg = f"Door operation timed out after {timeout_s} seconds"
        if self.driver_interface:
            try:
                self.driver_interface.showError(error_msg)
            except Exception as e:
                print(f"Failed to communicate with driver interface: {e}")
        return False

    def close_door(self, timeout_s: Optional[float] = None) -> bool:
        """Blocking: attempt to close the door and return True on success.
        If operation fails or safety triggers, returns False.
        
        Raises:
            DoorOperationError: If preconditions fail or critical errors occur
        """
        # Check if system is out of service
        if self.state == DoorState.OUT_OF_SERVICE:
            error_msg = "Door system is out of service - maintenance required"
            if self.driver_interface:
                try:
                    self.driver_interface.showError(error_msg)
                except Exception as e:
                    print(f"Failed to communicate with driver interface: {e}")
            return False
        
        timeout_s = timeout_s if timeout_s is not None else self.command_timeout_s
        
        # Check critical sensors before attempting operation
        if not self._check_critical_sensors():
            error_msg = "Critical sensor failure detected - system going out of service"
            self.set_out_of_service(error_msg)
            return False
        
        if not self._check_preconditions():
            self.state = DoorState.FAULT
            raise DoorOperationError("Preconditions failed: sensors/actuators unhealthy or missing")

        if not self.safety_system.check_safety(self.sensors):
            if self.safety_system.obstacle_detected:
                self.state = DoorState.STOPPED
            else:
                self.state = DoorState.FAULT
            return False

        motor = self._find_actuator(ActuatorType.MOTOR)
        if motor is None:
            raise DoorOperationError("No motor actuator available")

        self.state = DoorState.CLOSING
        # notify driver UI that closing started
        try:
            if self.driver_interface is not None:
                self.driver_interface.showStatus()
        except Exception:
            pass
        accepted = motor.command({"action": "close"})
        if not accepted:
            self.state = DoorState.FAULT
            return False

        start = time.time()
        pos_sensor = self._find_sensor(SensorType.POSITION)
        # assume position sensor returns False when closed
        while time.time() - start < timeout_s:
            if not self.safety_system.check_safety(self.sensors):
                # if obstacle detected while closing, attempt to reopen once
                if self.safety_system.obstacle_detected:
                    # Stop first
                    motor.stop()
                    # try to open a bit to release
                    motor.command({"action": "open"})
                    self.state = DoorState.OPENING
                    time.sleep(0.05)
                    # Stop again after opening
                    motor.command({"action": "stop"})
                    motor.stop()
                else:
                    motor.stop()
                self.state = DoorState.STOPPED
                # inform driver UI about safety/fault
                try:
                    if self.driver_interface is not None:
                        reason = ""
                        if self.safety_system.emergency_stop_engaged:
                            reason = "emergency_stop"
                        elif self.safety_system.obstacle_detected:
                            reason = "obstacle_detected"
                        elif self.safety_system.interlock_engaged:
                            reason = "interlock_engaged"
                        self.driver_interface.showError(f"Operation stopped: {reason}")
                except Exception:
                    pass
                return False
            if pos_sensor is not None:
                try:
                    val = pos_sensor.read()
                    # assume False when closed
                    if not val:
                        motor.stop()
                        self.state = DoorState.CLOSED
                        try:
                            if self.driver_interface is not None:
                                self.driver_interface.showStatus()
                        except Exception:
                            pass
                        return True
                except RuntimeError as e:
                    motor.stop()
                    self.state = DoorState.FAULT
                    try:
                        if self.driver_interface is not None:
                            self.driver_interface.showError(str(e))
                    except Exception:
                        pass
                    raise DoorOperationError(str(e))
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

    def check_external_button(self) -> bool:
        """Check if external button is pressed and conditions allow door opening.
        Returns True if door should be opened.
        
        Returns False if system is out of service, bus is moving, or conditions don't allow opening.
        """
        # Prevent operations if system is out of service
        if self.state == DoorState.OUT_OF_SERVICE:
            error_msg = "Door system is out of service - external button requests cannot be processed"
            if self.driver_interface:
                try:
                    self.driver_interface.showError(error_msg)
                except Exception as e:
                    print(f"Failed to communicate with driver interface: {e}")
            return False
        
        if self.external_button is None or not self.external_button.enabled:
            return False
            
        if self.external_button.pressed:
            # Check if any speed sensors indicate movement
            speed_sensors = [s for s in self.sensors if s.type == SensorType.SPEED]
            is_moving = any(s.is_triggered() for s in speed_sensors)
            
            # Only allow external button when not moving
            if not is_moving:
                self.external_button.reset()  # Clear the press state
                return True
            else:
                try:
                    if self.driver_interface is not None:
                        self.driver_interface.showError("External button pressed while moving - ignored")
                except Exception:
                    pass
                self.external_button.reset()
        return False

    def status_report(self) -> Dict[str, Any]:
        """Generate a comprehensive status report including system state and health information."""
        sensors_health = {
            "position_sensor": None,
            "obstacle_sensor": None,
            "speed_sensor": None,
            "limit_switch": None,
        }
        
        # Check health of all sensors
        for sensor in self.sensors:
            if sensor.type == SensorType.POSITION:
                sensors_health["position_sensor"] = getattr(sensor, "healthy", True)
            elif sensor.type == SensorType.OBSTACLE:
                sensors_health["obstacle_sensor"] = getattr(sensor, "healthy", True)
            elif sensor.type == SensorType.SPEED:
                sensors_health["speed_sensor"] = getattr(sensor, "healthy", True)
            elif sensor.type == SensorType.LIMIT_SWITCH:
                sensors_health["limit_switch"] = getattr(sensor, "healthy", True)
        
        return {
            "id": self.id,
            "state": self.state.value,
            "state_name": self.state.name,
            "out_of_service": self.state == DoorState.OUT_OF_SERVICE,
            "sensors_health": sensors_health,
            "safety": {
                "emergency_stop": self.safety_system.emergency_stop_engaged,
                "obstacle": self.safety_system.obstacle_detected,
                "interlock": self.safety_system.interlock_engaged,
            },
            "actuators": [a.get_status() for a in self.actuators],
            "external_button": {
                "enabled": self.external_button.enabled if self.external_button else False,
                "pressed": self.external_button.pressed if self.external_button else False
            }
        }
