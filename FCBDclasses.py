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
            print("Az ajtó nyitva")
        else:
            print("Az ajtót nem lehet biztonságosan kinyitni!")

    def request_close(self):
        if not self.sensors.obstacle_detected:
            self.actuator.close()
            print("Az ajtó zárva")
        else:
            print("Akadály észlelve, az ajtó nem zárható be!")

# Példányosítás és összekapcsolás
sensors = SensorSystem()
actuator = DoorActuator()
controller = BusDoorController(sensors, actuator)
