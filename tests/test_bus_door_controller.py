import time
import pytest

from bus_door_controller import (
    DoorController,
    SensorType,
    ActuatorType,
    SafetySystem,
    DoorOperationError,
    ExternalButton,
)
from sim.mocks import MockSensor, MockActuator, MockDriverInterface

# This test file uses simple mock classes to simulate sensors and actuators
# The test is written to simulate a bus moving from one stop to another,
# with door operations in between.

# Use shared mocks from sim.mocks


def test_happy_path_open_close():
    pos = MockSensor("pos1", SensorType.POSITION, initial_value=False)
    motor = MockActuator("m1", ActuatorType.MOTOR)
    controller = DoorController("door1", sensors=[pos], actuators=[motor], safety_system=SafetySystem())

    # simulate actuator causing sensor to change
    def do_open():
        # actuator command will be called; simulate sensor becomes True shortly
        pos.set(True)

    # motor.command to flip sensor
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

    # With the new out-of-service functionality, unhealthy sensors cause the system
    # to transition to OUT_OF_SERVICE state and return False instead of raising
    result = controller.open_door(timeout_s=0.5)
    assert result is False
    assert controller.state.name == 'OUT_OF_SERVICE'


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


def test_sensor_self_check_and_report():
    pos = MockSensor("pos1", SensorType.POSITION, initial_value=False)
    assert pos.self_check() is True
    
    pos.report_error("Test error")
    assert pos.healthy is False
    assert not pos.self_check()


def test_external_button_operations():
    pos = MockSensor("pos1", SensorType.POSITION, initial_value=False)
    speed = MockSensor("speed1", SensorType.SPEED, initial_value=False)
    motor = MockActuator("m1", ActuatorType.MOTOR)
    driver_ui = MockDriverInterface()
    external_button = ExternalButton("ext1", "door1")
    
    controller = DoorController(
        id="door1",
        sensors=[pos, speed],
        actuators=[motor],
        driver_interface=driver_ui,
        external_button=external_button
    )
    
    # Test when bus is stopped
    assert external_button.press() is True
    assert controller.check_external_button() is True
    
    # Test when bus is moving
    speed.set(True)  # Bus is moving
    external_button.press()
    assert controller.check_external_button() is False
    assert driver_ui._last_error == "External button pressed while moving - ignored"

    # Test button disable/enable
    external_button.disable()
    assert external_button.press() is False
    external_button.enable()
    assert external_button.press() is True
