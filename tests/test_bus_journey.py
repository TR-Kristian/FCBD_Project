"""Test suite simulating a complete bus journey with door operations.

Scenarios covered:
1. Normal operations at bus stops (open/close)
2. Obstacle detection (passengers, objects)
3. Sensor failures
4. Emergency situations
5. Driver interface interactions
6. Safety system triggers
7. Timing/timeout scenarios
"""

import time
import threading
import pytest
from typing import List, Optional

from bus_door_controller import (
    DoorController,
    SensorType,
    ActuatorType,
    SafetySystem,
    DoorOperationError,
    DoorState,
    DriverInterface,
)
from sim.mocks import MockSensor, MockActuator, MockDriverInterface


@pytest.fixture
def setup_door_system():
    """Create a complete door system with all sensors."""
    position = MockSensor("pos1", SensorType.POSITION, initial_value=False)
    obstacle = MockSensor("obs1", SensorType.OBSTACLE, initial_value=False)
    edge = MockSensor("edge1", SensorType.EDGE, initial_value=False)
    limit = MockSensor("lim1", SensorType.LIMIT_SWITCH, initial_value=False)
    
    motor = MockActuator("motor1", ActuatorType.MOTOR)
    lock = MockActuator("lock1", ActuatorType.LOCK)
    
    driver = MockDriverInterface()
    safety = SafetySystem()
    
    controller = DoorController(
        "door1",
        sensors=[position, obstacle, edge, limit],
        actuators=[motor, lock],
        driver_interface=driver,
        safety_system=safety
    )
    
    return {
        "controller": controller,
        "position": position,
        "obstacle": obstacle,
        "edge": edge,
        "limit": limit,
        "motor": motor,
        "lock": lock,
        "driver": driver,
        "safety": safety
    }


class TestObstacleDetection:
    """Test obstacle detection scenarios."""

    def test_obstacle_while_closing(self, setup_door_system):
        sys = setup_door_system
        
        # Start with door open
        sys["position"].set(True)  # Door open
        sys["controller"].state = DoorState.OPEN
        
        # Simulate obstacle (passenger) when closing
        def trigger_obstacle():
            time.sleep(0.05)  # Wait briefly
            sys["obstacle"].set(True)  # Passenger detected
        
        # Start closing
        t = threading.Thread(target=trigger_obstacle)
        t.start()
        
        result = sys["controller"].close_door(timeout_s=1.0)
        assert result is False  # Door should not complete closing
        assert sys["controller"].state == DoorState.STOPPED  # Should stop when obstacle detected
        assert sys["motor"].stopped  # Motor should have stopped
        assert sys["safety"].obstacle_detected  # Safety system should register obstacle

    def test_multiple_obstacle_attempts(self, setup_door_system):
        sys = setup_door_system
        
        # Try closing with persistent obstacle
        sys["position"].set(True)  # Door open
        sys["controller"].state = DoorState.OPEN
        sys["obstacle"].set(True)  # Persistent obstacle
        
        # Multiple close attempts should fail
        for _ in range(3):
            assert sys["controller"].close_door(timeout_s=0.5) is False
            assert sys["safety"].obstacle_detected is True
            assert sys["controller"].state == DoorState.STOPPED
        
        # Remove obstacle and try again
        sys["obstacle"].set(False)
        
        def simulate_door_close():
            time.sleep(0.1)
            sys["position"].set(False)
        
        t = threading.Thread(target=simulate_door_close)
        t.start()
        
        assert sys["controller"].close_door(timeout_s=1.0) is True
        assert sys["controller"].state == DoorState.CLOSED


class TestEmergencyScenarios:
    """Test emergency and failure scenarios."""

    def test_emergency_stop_while_moving(self, setup_door_system):
        sys = setup_door_system
        
        # Start opening
        def trigger_emergency():
            time.sleep(0.05)
            sys["controller"].emergency_unlock()
        
        import threading
        t = threading.Thread(target=trigger_emergency)
        t.start()
        
        result = sys["controller"].open_door(timeout_s=1.0)
        assert result is False
        assert sys["controller"].state == DoorState.STOPPED
        assert sys["safety"].emergency_stop_engaged is True

    def test_sensor_failure_during_operation(self, setup_door_system):
        sys = setup_door_system
        
        # Start operation then simulate sensor failure
        def fail_sensor():
            time.sleep(0.05)
            sys["position"].simulate_failure()
        
        import threading
        t = threading.Thread(target=fail_sensor)
        t.start()
        
        with pytest.raises(DoorOperationError):
            sys["controller"].open_door(timeout_s=1.0)
        
        assert sys["controller"].state == DoorState.FAULT

    def test_actuator_stuck(self, setup_door_system):
        sys = setup_door_system
        
        # Simulate actuator getting stuck
        sys["motor"].simulate_stuck()
        
        # Try to open - should timeout
        assert sys["controller"].open_door(timeout_s=0.5) is False
        assert sys["controller"].state == DoorState.FAULT


class TestDriverInterface:
    """Test driver interface interactions."""

    def test_driver_override_request(self, setup_door_system):
        sys = setup_door_system
        driver = sys["driver"]
        
        # Simulate obstacle but with driver override
        sys["position"].set(True)  # Door open
        sys["controller"].state = DoorState.OPEN
        
        def trigger_obstacle_then_override():
            time.sleep(0.05)
            sys["obstacle"].set(True)  # Obstacle appears
            time.sleep(0.05)
            # Simulate driver accepting override
            driver._should_accept = True
            driver.override_requested = True
            
        t = threading.Thread(target=trigger_obstacle_then_override)
        t.start()
        
        result = sys["controller"].close_door(timeout_s=1.0)
        assert result is False  # Should still fail despite override (safety first)
        assert driver.override_requested  # Override should have been requested
        assert sys["controller"].state == DoorState.STOPPED

    def test_disconnected_driver_interface(self, setup_door_system):
        sys = setup_door_system
        
        # Simulate disconnected interface
        sys["driver"].simulate_disconnected()
        
        # Operations should still work (failsafe)
        def simulate_door_open():
            time.sleep(0.1)
            sys["position"].set(True)
            
        t = threading.Thread(target=simulate_door_open)
        t.start()
        assert sys["controller"].open_door(timeout_s=1.0) is True
        assert sys["controller"].state == DoorState.OPEN
        
        def simulate_door_close():
            time.sleep(0.1)
            sys["position"].set(False)
            
        t = threading.Thread(target=simulate_door_close)
        t.start()
        assert sys["controller"].close_door(timeout_s=1.0) is True
        assert sys["controller"].state == DoorState.CLOSED


class TestEdgeCases:
    """Test various edge cases and timing scenarios."""

    def test_rapid_open_close_commands(self, setup_door_system):
        sys = setup_door_system
        
        def simulate_door_motion(to_open: bool):
            time.sleep(0.1)
            sys["position"].set(to_open)
        
        # Rapidly toggle commands
        for _ in range(2):  # Two full cycles
            t = threading.Thread(target=lambda: simulate_door_motion(True))
            t.start()
            assert sys["controller"].open_door(timeout_s=0.5) is True
            assert sys["controller"].state == DoorState.OPEN
            
            t = threading.Thread(target=lambda: simulate_door_motion(False))
            t.start()
            assert sys["controller"].close_door(timeout_s=0.5) is True
            assert sys["controller"].state == DoorState.CLOSED

    def test_timeout_scenarios(self, setup_door_system):
        sys = setup_door_system
        
        # Make position sensor never indicate open
        sys["position"].set(False)
        
        # Should timeout
        assert sys["controller"].open_door(timeout_s=0.1) is False
        assert sys["controller"].state == DoorState.FAULT

    @pytest.mark.parametrize("scenario", [
        "obstacle_during_open",
        "obstacle_during_close",
        "emergency_during_open",
        "emergency_during_close"
    ])
    def test_interruption_scenarios(self, setup_door_system, scenario):
        sys = setup_door_system
        
        # Set initial state based on operation
        if "close" in scenario:
            sys["position"].set(True)  # Start open for close operations
            sys["controller"].state = DoorState.OPEN
        else:
            sys["position"].set(False)  # Start closed for open operations
            sys["controller"].state = DoorState.CLOSED
            
        def interrupt():
            time.sleep(0.05)  # Brief delay to let operation start
            if "obstacle" in scenario:
                sys["obstacle"].set(True)
            elif "emergency" in scenario:
                sys["controller"].emergency_unlock()
        
        t = threading.Thread(target=interrupt)
        t.start()
        
        def simulate_door_motion(to_open: bool):
            time.sleep(0.1)
            # Only change position if not interrupted
            if not sys["safety"].obstacle_detected and not sys["safety"].emergency_stop_engaged:
                sys["position"].set(to_open)
        
        motion = threading.Thread(target=lambda: simulate_door_motion("open" in scenario))
        motion.start()
        
        if "open" in scenario:
            result = sys["controller"].open_door(timeout_s=1.0)
        else:
            result = sys["controller"].close_door(timeout_s=1.0)
            
        # For emergency scenarios, operation should fail
        if "emergency" in scenario:
            assert result is False
            assert sys["safety"].emergency_stop_engaged is True
            assert sys["controller"].state == DoorState.STOPPED
        # For obstacle scenarios during close, operation should fail
        elif "obstacle" in scenario and "close" in scenario:
            assert result is False
            assert sys["safety"].obstacle_detected is True
            assert sys["controller"].state == DoorState.STOPPED