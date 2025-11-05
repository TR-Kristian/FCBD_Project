# FCBD Project

FCBD — the "FCBD" (fucking cool bus door) system — is a small, testable Python library
that models bus door control logic. It provides five main components as mockable
interfaces so you can integrate real hardware later:

- `DoorController` — orchestrates sensors, actuators, driver interface and safety checks
- `Sensor` — abstract sensor interface (position, obstacle, limit switch, etc.)
- `Actuator` — abstract actuator interface (motor, lock)
- `DriverInterface` — bridge for driver/vehicle commands and acknowledgements
- `SafetySystem` — encapsulates safety checks and emergency stop logic

This repository contains a reference implementation in `bus_door_controller.py` and
unit tests under `tests/` that show how to use mock sensors/actuators.

## Quick start (Windows / PowerShell)

1. Create a virtual environment and activate it (PowerShell):

```powershell
python -m venv .venv
& .\.venv\Scripts\Activate.ps1
```

2. Install the project test/runtime dependencies:

```powershell
python -m pip install -r requirements.txt
```

3. Run the test suite:

```powershell
python -m pytest -q
```

The repo includes `requirements.txt` (currently lists `pytest`) so tests can run
without hardware.

## Usage example (in code)

Below is a minimal example showing how to create mock sensors and actuators and
use the `DoorController` to open/close a door. See `tests/test_bus_door_controller.py`
for a working example.

```python
from bus_door_controller import (
	DoorController, SensorType, ActuatorType, SafetySystem
)

# Create simple mock objects (the tests show a concrete MockSensor/MockActuator)
class SimplePosSensor:
	def __init__(self, id, initial=False):
		self.id = id
		self.type = SensorType.POSITION
		self._value = initial
		self.healthy = True
	def read(self):
		return self._value
	def is_triggered(self):
		return bool(self._value)

class SimpleMotor:
	def __init__(self, id):
		self.id = id
		self.type = ActuatorType.MOTOR
		self.stopped = True
	def command(self, cmd):
		self.stopped = False
		# simulate action accepted
		return True
	def stop(self):
		self.stopped = True
	def get_status(self):
		return {"id": self.id, "type": self.type.value, "stopped": self.stopped}

# Compose controller
pos = SimplePosSensor("pos1", initial=False)
motor = SimpleMotor("m1")
controller = DoorController("door1", sensors=[pos], actuators=[motor], safety_system=SafetySystem())

# Example: open the door (in production you'd drive the sensor value change via hardware)
try:
	success = controller.open_door(timeout_s=2.0)
	print("Opened?", success)
except Exception as e:
	print("Error operating door:", e)

```

Notes:
- `open_door` and `close_door` are synchronous/blocking and accept an optional
  `timeout_s` parameter. They return `True` on success and `False` when the
  operation failed or safety conditions prevented completion.
- All hardware interfaces are expressed as abstract protocols so they can be
  replaced with real drivers later.

## API summary

- `DoorController(id, sensors, actuators, driver_interface=None, safety_system=None, command_timeout_s=5.0)`
  - key methods: `open_door(timeout_s=None)`, `close_door(timeout_s=None)`, `stop()`, `emergency_unlock()`, `status_report()`

- `Sensor` (protocol): `read()`, `is_triggered()`, attributes: `id`, `type`, `healthy`

- `Actuator` (protocol): `command(cmd) -> bool`, `stop()`, `get_status()`

- `DriverInterface` (protocol): `send_command(cmd) -> bool`, `receive_ack(timeout_s)`, `request_override(reason)`

- `SafetySystem`: `check_safety(sensors) -> bool`, `engage_emergency_stop(reason)`, `reset()`

## Contributing

Contributions are welcome. If you'd like to add hardware drivers or integration
tests, please open an issue to discuss the design. For small changes, send a
pull request with tests.

## License

Pick an appropriate license for your project. This repository does not include
an explicit license file by default.
