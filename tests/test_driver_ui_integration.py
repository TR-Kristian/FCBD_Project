import time
import threading

from bus_door_controller import (
    DoorController,
    SensorType,
    ActuatorType,
    SafetySystem,
    DoorOperationError,
)
from sim.mocks import MockSensor, MockActuator, MockDriverInterface


def test_driver_status_updated_on_open_close():
    pos = MockSensor("pos", SensorType.POSITION, initial_value=False)
    motor = MockActuator("motor", ActuatorType.MOTOR)
    driver = MockDriverInterface()
    controller = DoorController("door1", sensors=[pos], actuators=[motor], driver_interface=driver, safety_system=SafetySystem())

    # make motor command flip sensor after a short delay
    orig_cmd = motor.command

    def cmd_and_flip(c):
        r = orig_cmd(c)
        if c.get("action") == "open":
            # simulate sensor becoming True shortly
            threading.Timer(0.05, lambda: pos.set(True)).start()
        elif c.get("action") == "close":
            threading.Timer(0.05, lambda: pos.set(False)).start()
        return r

    motor.command = cmd_and_flip

    ok = controller.open_door(timeout_s=1.0)
    assert ok is True
    # driver.showStatus should have been called -> statusLED True
    assert driver.statusLED is True

    ok = controller.close_door(timeout_s=1.0)
    assert ok is True
    assert driver.statusLED is True


def test_driver_shows_error_on_sensor_failure():
    pos = MockSensor("pos", SensorType.POSITION, initial_value=False)
    motor = MockActuator("motor", ActuatorType.MOTOR)
    driver = MockDriverInterface()
    controller = DoorController("door1", sensors=[pos], actuators=[motor], driver_interface=driver, safety_system=SafetySystem())

    # simulate sensor failure shortly after start
    def fail_sensor():
        time.sleep(0.05)
        pos.simulate_failure()

    threading.Thread(target=fail_sensor).start()

    try:
        controller.open_door(timeout_s=1.0)
    except DoorOperationError:
        pass

    # driver.showError should have been called and last_error set
    assert driver.last_error is not None
