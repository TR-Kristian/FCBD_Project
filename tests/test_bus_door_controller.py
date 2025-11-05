import time
import pytest

from bus_door_controller import (
    DoorController,
    SensorType,
    ActuatorType,
    SafetySystem,
    DoorOperationError,
)


class MockSensor:
    def __init__(self, id, type, initial_value=False, healthy=True):
        self.id = id
        self.type = type
        self._value = initial_value
        self.healthy = healthy

    def read(self):
        return self._value

    def is_triggered(self):
        return bool(self._value)

    def set(self, v):
        self._value = v


class MockActuator:
    def __init__(self, id, type):
        self.id = id
        self.type = type
        self.last_command = None
        self.stopped = True

    def command(self, cmd):
        self.last_command = cmd
        self.stopped = False
        # simulate immediate acceptance
        return True

    def stop(self):
        self.stopped = True

    def get_status(self):
        return {"id": self.id, "type": self.type.value if hasattr(self.type, 'value') else str(self.type), "stopped": self.stopped}


def test_happy_path_open_close():
    pos = MockSensor("pos1", SensorType.POSITION, initial_value=False)
    motor = MockActuator("m1", ActuatorType.MOTOR)
    controller = DoorController("door1", sensors=[pos], actuators=[motor], safety_system=SafetySystem())

    # simulate actuator causing sensor to change
    def do_open():
        # actuator command will be called; simulate sensor becomes True shortly
        pos.set(True)

    # monkey-patch motor.command to flip sensor
    orig_cmd = motor.command

    def cmd_and_flip(c):
        r = orig_cmd(c)
        if c.get('action') == 'open':
            do_open()
        elif c.get('action') == 'close':
            pos.set(False)
        return r

    motor.command = cmd_and_flip

    assert controller.open_door(timeout_s=1.0) is True
    assert controller.state.name == 'OPEN'

    assert controller.close_door(timeout_s=1.0) is True
    assert controller.state.name == 'CLOSED'


def test_sensor_failure_raises_fault():
    pos = MockSensor("pos1", SensorType.POSITION, initial_value=False, healthy=False)
    motor = MockActuator("m1", ActuatorType.MOTOR)
    controller = DoorController("door1", sensors=[pos], actuators=[motor], safety_system=SafetySystem())

    with pytest.raises(DoorOperationError):
        controller.open_door(timeout_s=0.5)


def test_obstacle_while_closing_triggers_safety_and_stop():
    pos = MockSensor("pos1", SensorType.POSITION, initial_value=True)
    obstacle = MockSensor("obs1", SensorType.OBSTACLE, initial_value=False)
    motor = MockActuator("m1", ActuatorType.MOTOR)
    safety = SafetySystem()
    controller = DoorController("door1", sensors=[pos, obstacle], actuators=[motor], safety_system=safety)

    # When close command is issued, simulate obstacle detection shortly after
    def cmd_and_trigger(c):
        motor.last_command = c
        motor.stopped = False
        if c.get('action') == 'close':
            # simulate obstacle shortly
            obstacle.set(True)
        return True

    motor.command = cmd_and_trigger

    result = controller.close_door(timeout_s=0.5)
    assert result is False
    assert controller.state in (controller.state.STOPPED, controller.state.FAULT, controller.state.CLOSING, controller.state.STOPPED)
    # safety should have registered obstacle
    assert safety.obstacle_detected is True
