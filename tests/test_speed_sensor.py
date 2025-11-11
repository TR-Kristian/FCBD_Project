import threading
import time

from bus_door_controller import (
    DoorController,
    SensorType,
    ActuatorType,
    SafetySystem,
    DoorOperationError,
)
from sim.mocks import MockSensor, MockActuator, MockDriverInterface, MockSpeedSensor


def test_open_blocked_when_moving():
    pos = MockSensor("pos", SensorType.POSITION, initial_value=False)
    speed = MockSpeedSensor("speed", initial_speed=12.5)
    motor = MockActuator("motor", ActuatorType.MOTOR)
    driver = MockDriverInterface()

    controller = DoorController(
        "door1",
        sensors=[pos, speed],
        actuators=[motor],
        driver_interface=driver,
        safety_system=SafetySystem(),
    )

    # Attempt to open while speed > 0 should be blocked
    ok = controller.open_door(timeout_s=0.2)
    assert ok is False
    assert driver.last_error is not None
    assert "Cannot open doors" in driver.last_error


def test_open_allowed_when_stationary():
    pos = MockSensor("pos", SensorType.POSITION, initial_value=False)
    speed = MockSpeedSensor("speed", initial_speed=0.0)
    motor = MockActuator("motor", ActuatorType.MOTOR)
    driver = MockDriverInterface()

    controller = DoorController(
        "door1",
        sensors=[pos, speed],
        actuators=[motor],
        driver_interface=driver,
        safety_system=SafetySystem(),
    )

    # simulate motor command causing position sensor to become True shortly after
    orig_cmd = motor.command

    def cmd_and_flip(c):
        r = orig_cmd(c)
        if c.get("action") == "open":
            threading.Timer(0.05, lambda: pos.set(True)).start()
        return r

    motor.command = cmd_and_flip

    ok = controller.open_door(timeout_s=1.0)
    assert ok is True
    assert driver.statusLED is True
