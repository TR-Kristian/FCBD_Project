class SensorSystem:
    def __init__(self):
        self.obstacle_detected = False
        self.door_open = False
        self.bus_stopped = True

    def update(self, obstacle, door_state, stopped):
        self.obstacle_detected = obstacle
        self.door_open = door_state
        self.bus_stopped = stopped

class DoorActuator:
    def __init__(self):
        self.state = "closed"

    def open(self):
        self.state = "open"

    def close(self):
        self.state = "closed"

class BusDoorController:
    def __init__(self, sensors, actuator):
        self.sensors = sensors
        self.actuator = actuator

    def request_open(self):
        if self.sensors.bus_stopped and not self.sensors.obstacle_detected:
            self.actuator.open()
            print("Door opened")
        else:
            print("Door cannot be opened safely!")

    def request_close(self):
        if not self.sensors.obstacle_detected:
            self.actuator.close()
            print("Door closed")
        else:
            print("Obstacle detected, cannot close the door!")

# Instantiate and connect systems
sensors = SensorSystem()
actuator = DoorActuator()
controller = BusDoorController(sensors, actuator)

