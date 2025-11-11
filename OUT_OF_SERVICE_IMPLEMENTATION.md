# Bus Door System - Error Handling & Out-of-Service Implementation

## Summary of Changes

This document outlines the improvements made to handle sensor failures gracefully and prevent system crashes when critical sensors malfunction.

---

## 1. New DoorState: OUT_OF_SERVICE

**File**: `bus_door_controller.py`

Added a new state to the `DoorState` enum:
```python
class DoorState(Enum):
    # ... existing states ...
    OUT_OF_SERVICE = 6
```

This state represents a condition where the bus door system cannot operate safely due to hardware failures or maintenance issues.

---

## 2. New Methods in DoorController

### `_check_critical_sensors() -> bool`

Validates that all critical sensors are functioning properly:
- Position sensor
- Obstacle sensor  
- Speed sensor

Returns `False` if any critical sensor fails self-check or raises an exception.

### `set_out_of_service(reason: str) -> None`

Safely transitions the system to the `OUT_OF_SERVICE` state:
1. Stops all actuators immediately
2. Disables external button
3. Engages emergency stop
4. Notifies driver interface with maintenance message
5. Prevents any further door operations

---

## 3. Enhanced Error Handling

### Door Opening/Closing Operations

Both `open_door()` and `close_door()` now include:

1. **Out-of-Service Guard**: Rejects operations if system is already out of service
2. **Critical Sensor Check**: Validates sensors before attempting operation
3. **Auto-Transition**: Automatically transitions to `OUT_OF_SERVICE` on critical failures
4. **Detailed Error Messages**: Reports specific failure reasons to driver interface

### External Button Handling

The `check_external_button()` method now:
- Rejects requests when system is out of service
- Provides clear error messages
- Prevents potentially unsafe operations

---

## 4. Enhanced Status Reporting

The `status_report()` method now includes:

```python
{
    "id": "door1",
    "state": 6,  # OUT_OF_SERVICE
    "state_name": "OUT_OF_SERVICE",  # Human-readable state
    "out_of_service": True,
    "sensors_health": {
        "position_sensor": False,      # Failed sensor
        "obstacle_sensor": True,       # Healthy
        "speed_sensor": True,          # Healthy
        "limit_switch": True           # Healthy
    },
    # ... other fields ...
}
```

---

## 5. Simulation Integration

**File**: `simulate_bus_journey.py`

Updated `run_one_stop()` method to:

1. **Check System Status**: Verifies system is not out of service before each operation
2. **Detect Sensor Failures**: Identifies when sensors fail during operation
3. **Handle Gracefully**: Terminates simulation cleanly with diagnostic information
4. **Report Failures**: Displays sensor health and maintenance recommendations

---

## 6. Test Coverage

### New Test Files

#### `test_out_of_service.py`
Tests three critical scenarios:
1. **Sensor Failure Detection**: Validates automatic transition to out-of-service
2. **External Button Blocking**: Ensures buttons are disabled when out of service
3. **System Safeguards**: Verifies all safeguards activate correctly

#### `demo_out_of_service.py`
Demonstrates real-world scenario:
- Bus approaching stop normally
- Sensor failure event
- Automatic out-of-service transition
- Failed door operation attempts
- Maintenance notification

---

## 7. Error Messages & User Feedback

### Driver Interface Messages

When critical failures occur, the driver receives:
- `"BUS OUT OF SERVICE: [reason]. Please contact maintenance."`

When operations are blocked:
- `"Door system is out of service - maintenance required"`
- `"Critical sensor failure detected - system going out of service"`

### Simulation Output

The simulator displays:
- `"!!! SYSTEM OUT OF SERVICE !!!"`
- Detailed sensor health report
- `"BUS CANNOT OPERATE - MAINTENANCE REQUIRED"`
- `"BUS OUT OF SERVICE - MAINTENANCE REQUIRED"`

---

## 8. Safety Guarantees

✓ **No Crashes**: System gracefully handles sensor failures without exceptions  
✓ **No Operations**: Prevents any door operations when system is compromised  
✓ **Clear Communication**: Notifies operators of maintenance needs  
✓ **Emergency Stop**: Engages safety system when transitioning to out-of-service  
✓ **Button Disabled**: External button cannot request operations  

---

## 9. How It Works

### Normal Operation Flow
```
Bus Approaches Stop
    ↓
Sensor Health Check
    ↓
Door Operations (Open/Close)
    ↓
Bus Departs
```

### On Sensor Failure
```
Bus Approaches Stop
    ↓
Sensor Health Check
    ↓
❌ SENSOR FAILURE DETECTED
    ↓
[Automatic Transition]
    ↓
OUT_OF_SERVICE State
    ↓
❌ All Operations Blocked
    ↓
⚠️ Maintenance Required
```

---

## 10. Testing & Validation

**Run Tests**:
```bash
python test_out_of_service.py
python demo_out_of_service.py
```

**Run Simulation with High Sensor Failure Rate**:
```bash
python simulate_bus_journey.py --stops 2 --sensor-fail-prob 0.9
```

---

## 11. Benefits

1. **Reliability**: System never crashes due to sensor failures
2. **Safety**: Prevents dangerous operations on failed hardware
3. **Maintainability**: Clear diagnostics help technicians identify issues
4. **User Experience**: Drivers receive clear instructions when maintenance is needed
5. **Compliance**: Meets safety standards for transportation systems

---

## 12. Future Enhancements

Potential improvements:
- Recovery procedure: Allow restart after manual inspection
- Logging: Store sensor failure events for analysis
- Notifications: Send alerts to maintenance center
- Diagnostics: Provide detailed sensor reading history
- Fallback: Allow manual override with safety approval

---

**Last Updated**: November 11, 2025  
**Status**: ✅ Complete and Tested
