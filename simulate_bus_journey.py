"""Simple bus journey simulator that prints step-by-step messages.

This standalone script imitates the scenarios used in the tests and prints
human-readable messages with timestamps. It is independent of the test mocks
and is intended for demonstration and manual inspection.

Usage:
    python simulate_bus_journey.py

The script will:
 - approach a stop
 - stop
 - open doors
 - simulate passenger boarding
 - (optionally) simulate an obstacle during closing
 - remove obstacle
 - close doors and depart

"""
from __future__ import annotations

import time
import threading
import argparse
import random
import sys
try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    TK_AVAILABLE = True
except Exception:
    TK_AVAILABLE = False
from typing import Callable
from bus_door_controller import (
    DoorController,
    SensorType,
    ActuatorType,
    SafetySystem,
    DoorOperationError,
    DoorState,
    ExternalButton,
)
from sim.mocks import MockSensor, MockActuator, MockDriverInterface


def ts() -> str:
    return time.strftime("%H:%M:%S")


def print_step(msg: str) -> None:
    print(f"[{ts()}] {msg}")


def make_printer(verbose: bool):
    if verbose:
        return print_step
    else:
        return lambda *_args, **_kwargs: None


class JourneySimulator:
    def __init__(self, controller: DoorController, position_sensor, obstacle_sensor, speed_sensor, motor_actuator, 
                 external_button=None, obstacle_during_close: bool = True, 
                 obstacle_probability: float = 1.0, persist_obstacle: bool = False,
                 sensor_fail_prob: float = 0.2,
                 printer: Callable[[str], None] = print_step):
        self.controller = controller
        self.position = position_sensor
        self.obstacle_sensor = obstacle_sensor
        self.speed_sensor = speed_sensor
        self.motor = motor_actuator
        self.external_button = external_button
        self.obstacle = False
        self.obstacle_during_close = obstacle_during_close
        self.obstacle_probability = float(obstacle_probability)
        self.persist_obstacle = bool(persist_obstacle)
        self.print = printer
        self._lock = threading.Lock()
        self.bus_moving = False
        self.controller._sensor_fail_prob = sensor_fail_prob  # Store sensor failure probability

    def approach_stop(self, dwell_s: float = 0.5):
        self.print("Bus approaching stop")
        # Set bus in motion and disable external button
        self.speed_sensor.update_value(True)
        if self.external_button:
            self.external_button.disable()
        self.bus_moving = True
        
        time.sleep(dwell_s)
        
        # Bus stops
        self.speed_sensor.update_value(False)
        self.bus_moving = False
        self.print("Bus stopped")
        
        # Enable external button at stop
        if self.external_button:
            self.external_button.enable()
            self.print("External door buttons enabled")
            
    def simulate_external_button_press(self):
        """Simulate a passenger pressing the external button at a stop."""
        if not self.external_button:
            return False
        
        if self.external_button.press():
            self.print("External button pressed")
            if self.controller.check_external_button():
                self.print("External button request accepted")
                # Check if any sensors are unhealthy before proceeding
                unhealthy_sensors = [s for s in self.controller.sensors if not s.self_check()]
                if unhealthy_sensors:
                    self.print(f"Warning: {len(unhealthy_sensors)} sensors are unhealthy")
                    for s in unhealthy_sensors:
                        self.print(f"- Sensor {s.id} failed self-check")
                return True
            else:
                self.print("External button request denied")
        return False

    def open_doors(self, open_time_s: float = 0.8):
        self.print("Opening doors (controller)")

        # Simulate actuator causing position change after a delay
        def cause_open():
            # wait a little to simulate motor action
            time.sleep(open_time_s)
            with self._lock:
                self.position.set(True)
            self.print("(sensor) Doors reported open")

        t = threading.Thread(target=cause_open)
        t.start()

        try:
            ok = self.controller.open_door(timeout_s=open_time_s + 1.0)
            self.print(f"Controller open_door returned: {ok} (state={self.controller.state})")
        except DoorOperationError as e:
            self.print(f"Controller raised DoorOperationError during open: {e}")

    def passengers_boarding(self, boarding_s: float = 1.0):
        self.print("Passengers boarding")
        time.sleep(boarding_s)
        self.print("Boarding complete")

    def _trigger_obstacle(self, delay: float = 0.2, persist: bool = False):
        time.sleep(delay)
        with self._lock:
            self.obstacle = True
            self.obstacle_sensor.set(True)
        self.print("Obstacle encountered")
        if not persist:
            # remove after a short time
            time.sleep(0.4)
            with self._lock:
                self.obstacle = False
                self.obstacle_sensor.set(False)
            self.print("Obstacle removed")

    def close_doors(self, close_time_s: float = 0.8):
        self.print("Closing doors (controller)")

        # Optionally spawn an obstacle during closing based on probability
        if self.obstacle_during_close and random.random() < self.obstacle_probability:
            t_obs = threading.Thread(target=self._trigger_obstacle, args=(0.15, self.persist_obstacle))
            t_obs.start()

        # Simulate actuator causing position change after a delay if no obstacle
        def cause_close():
            start = time.time()
            while time.time() - start < close_time_s:
                with self._lock:
                    if self.obstacle:
                        # abort causing closed state
                        return
                time.sleep(0.02)
            with self._lock:
                self.position.set(False)
            self.print("(sensor) Doors reported closed")

        t = threading.Thread(target=cause_close)
        t.start()

        try:
            ok = self.controller.close_door(timeout_s=close_time_s + 1.0)
            self.print(f"Controller close_door returned: {ok} (state={self.controller.state})")
            return ok
        except DoorOperationError as e:
            self.print(f"Controller raised DoorOperationError during close: {e}")
            return False

    def depart(self):
        self.print("\n=== Departing bus stop ===")
        # Set speed sensor to indicate movement
        self.speed_sensor.update_value(True)
        self.bus_moving = True
        
        # Disable external button while moving
        if self.external_button:
            self.external_button.disable()
            self.print("- External door buttons disabled")
        
        # Verify door state before departure
        status = self.controller.status_report()
        if status['state'] != DoorState.CLOSED.value:
            self.print("WARNING: Doors not fully closed before departure!")
        else:
            self.print("- All doors secured")
            self.print("- Bus in motion")

    def run_one_stop(self):
        self.print("\n=== Approaching bus stop ===")
        self.approach_stop()
        
        # Perform sensor self-diagnostics
        self.print("\nPerforming sensor diagnostics:")
        for sensor in [self.position, self.obstacle_sensor, self.speed_sensor]:
            if sensor.self_check():
                self.print(f"- {sensor.id}: OK")
            else:
                self.print(f"- {sensor.id}: FAILED")
        
        # Check if system is already out of service
        status = self.controller.status_report()
        if status.get('out_of_service', False):
            self.print("\n!!! SYSTEM OUT OF SERVICE !!!")
            self.print(f"Current state: {status.get('state_name', 'UNKNOWN')}")
            self.print(f"Sensor health:")
            for sensor_name, healthy in status.get('sensors_health', {}).items():
                health_str = "OK" if healthy else "FAILED"
                self.print(f"  - {sensor_name}: {health_str}")
            self.print("\nBUS CANNOT OPERATE - MAINTENANCE REQUIRED")
            self.print("Preventing any further door operations")
            return
        
        # Randomly simulate a sensor error (20% chance)
        if random.random() < self.controller._sensor_fail_prob:  # Use configured probability
            problem_sensor = random.choice([self.position, self.obstacle_sensor, self.speed_sensor])
            problem_sensor.report_error(f"{problem_sensor.id} malfunction")
            self.print(f"\nWARNING: {problem_sensor.id} reported an error")
            
            # Re-check if system should go out of service
            if not self.controller._check_critical_sensors():
                self.print("\n!!! CRITICAL SENSOR FAILURE - TAKING SYSTEM OUT OF SERVICE !!!")
                self.controller.set_out_of_service("Critical sensor failure during operation")
                status = self.controller.status_report()
                self.print(f"Current state: {status.get('state_name', 'UNKNOWN')}")
                self.print("BUS OUT OF SERVICE - MAINTENANCE REQUIRED")
                return
        
        # Simulate external button press with 50% probability when at stop
        self.print("\nPassengers at stop:")
        if self.external_button and random.random() < 0.5:
            self.print("- Passenger requesting door open via external button")
            if self.simulate_external_button_press():
                self.print("- Door opening in response to external button")
                self.passengers_boarding()
            else:
                self.print("- External button request denied, using normal door cycle")
                try:
                    self.open_doors()
                except DoorOperationError as e:
                    self.print(f"Door operation failed: {e}")
                    return
        else:
            self.print("- Normal door cycle initiated")
            try:
                self.open_doors()
            except DoorOperationError as e:
                self.print(f"Door operation failed: {e}")
                return
            self.passengers_boarding()
        
        self.print("\nPreparing for departure:")
        closed = self.close_doors()
        if not closed:
            self.print("- Door closure interrupted")
            if self.obstacle:
                self.print("- Obstacle detected, waiting for clearance")
            # wait for obstacle to be removed if persists
            time.sleep(0.6)
            if self.persist_obstacle:
                self.print("- Obstacle persists, requesting assistance")
            else:
                self.print("- Obstacle cleared, retrying door close")
                closed = self.close_doors(close_time_s=0.6)
        
        if closed:
            self.print("- Doors secured, ready for departure")
            self.depart()
        else:
            self.print("=== WARNING: Door fault detected ===")
            status = self.controller.status_report()
            
            # Check if system is now out of service
            if status.get('out_of_service', False):
                self.print("\n!!! SYSTEM OUT OF SERVICE !!!")
                self.print(f"Current state: {status.get('state_name', 'UNKNOWN')}")
                self.print(f"Sensor health:")
                for sensor_name, healthy in status.get('sensors_health', {}).items():
                    health_str = "OK" if healthy else "FAILED"
                    self.print(f"  - {sensor_name}: {health_str}")
                self.print("\nBUS CANNOT OPERATE - MAINTENANCE REQUIRED")
            else:
                self.print("- System requires maintenance")
                self.print("- Status:")
                self.print(f"  * Door state: {DoorState(status.get('state', DoorState.FAULT.value)).name}")
                self.print(f"  * Safety system: {status.get('safety', {})}")
                if 'external_button' in status:
                    self.print(f"  * External button: {status['external_button']}")


def main():
    parser = argparse.ArgumentParser(description="Simulate a bus journey with door operations and optional obstacles")
    parser.add_argument("--stops", type=int, default=2, help="Number of stops to simulate (default: 2)")
    parser.add_argument("--obstacle-prob", type=float, default=1.0, help="Probability (0.0-1.0) that an obstacle appears during closing (default: 1.0)")
    parser.add_argument("--persist-obstacle", action="store_true", help="If set, obstacles persist until cleared manually (default: False)")
    parser.add_argument("--sensor-fail-prob", type=float, default=0.2, help="Probability (0.0-1.0) of sensor failures during operation (default: 0.2)")
    parser.add_argument("--quiet", action="store_true", help="Suppress printed steps (except errors)")
    parser.add_argument("--interactive", action="store_true", help="Prompt for CLI options interactively (useful when running from VSCode)")
    parser.add_argument("--gui", action="store_true", help="Open a small GUI dialog to enter options (requires tkinter)")
    args = parser.parse_args()

    # Interactive input (useful when running from VSCode Run) - overrides parsed args
    if args.gui:
        if not TK_AVAILABLE:
            print_step("tkinter not available; falling back to interactive prompts")
        else:
            # show GUI dialog and collect options
            def show_gui_dialog(defaults: dict):
                root = tk.Tk()
                root.title("Bus Journey Simulator")

                # Variables that need to be accessed by nested functions
                result = {}
                buttons = {}

                # Configure root window for resizing
                root.resizable(True, True)
                root.minsize(800, 500)
                
                # Configure grid weights for resizing
                root.grid_rowconfigure(0, weight=1)
                root.grid_columnconfigure(0, weight=1)
                
                # Create main container with two frames
                container = ttk.Frame(root, padding=15)
                container.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
                
                # Configure container grid weights
                container.grid_rowconfigure(0, weight=1)
                container.grid_columnconfigure(1, weight=1)  # Log frame will expand
                
                # Settings frame (left side)
                frm = ttk.Frame(container)
                frm.grid(row=0, column=0, sticky="n", padx=(0, 10))
                
                # Log frame (right side)
                log_frame = ttk.Frame(container)
                log_frame.grid(row=0, column=1, sticky="nsew")
                
                # Configure log frame grid weights
                log_frame.grid_rowconfigure(0, weight=1)
                log_frame.grid_columnconfigure(0, weight=1)
                
                # Configure text widget for logging with expanded size
                log_text = tk.Text(log_frame, wrap=tk.WORD)
                log_text.grid(row=0, column=0, sticky="nsew")
                
                # Add scrollbar
                scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=log_text.yview)
                scrollbar.grid(row=0, column=1, sticky="ns")
                log_text.configure(yscrollcommand=scrollbar.set)

                # Stops configuration
                ttk.Label(frm, text="Number of stops:", width=30).grid(column=0, row=0, sticky="w", pady=5)
                stops_var = tk.StringVar(value=str(defaults.get("stops", 2)))
                stops_entry = ttk.Entry(frm, width=10, textvariable=stops_var)
                stops_entry.grid(column=1, row=0, padx=(10, 0), pady=5)

                # Obstacle configuration
                ttk.Label(frm, text="Obstacle probability (0.0-1.0):", width=30).grid(column=0, row=1, sticky="w", pady=5)
                prob_var = tk.StringVar(value=str(defaults.get("obstacle_prob", 1.0)))
                prob_entry = ttk.Entry(frm, width=10, textvariable=prob_var)
                prob_entry.grid(column=1, row=1, padx=(10, 0), pady=5)

                # Add separator for visual grouping
                ttk.Separator(frm, orient='horizontal').grid(row=2, columnspan=2, sticky='ew', pady=10)

                # Persist obstacle checkbox
                persist_var = tk.BooleanVar(value=bool(defaults.get("persist_obstacle", False)))
                persist_cb = ttk.Checkbutton(frm, text="Persist obstacle", variable=persist_var)
                persist_cb.grid(column=0, row=3, columnspan=2, sticky="w", pady=5)

                # Sensor issues configuration
                ttk.Label(frm, text="Sensor Issue Settings:", font=('TkDefaultFont', 9, 'bold')).grid(
                    column=0, row=4, columnspan=2, sticky="w", pady=(15, 5))
                
                ttk.Label(frm, text="Sensor failure probability (0.0-1.0):", width=30).grid(
                    column=0, row=5, sticky="w", pady=5)
                sensor_fail_var = tk.StringVar(value=str(defaults.get("sensor_fail_prob", 0.2)))
                sensor_fail_entry = ttk.Entry(frm, width=10, textvariable=sensor_fail_var)
                sensor_fail_entry.grid(column=1, row=5, padx=(10, 0), pady=5)

                # Add separator before output configuration
                ttk.Separator(frm, orient='horizontal').grid(row=6, columnspan=2, sticky='ew', pady=10)

                # Output configuration
                quiet_var = tk.BooleanVar(value=bool(defaults.get("quiet", False)))
                quiet_cb = ttk.Checkbutton(frm, text="Quiet output", variable=quiet_var)
                quiet_cb.grid(column=0, row=7, columnspan=2, sticky="w", pady=5)

                result = {}
                
                def log_message(msg: str):
                    log_text.insert(tk.END, f"[{ts()}] {msg}\n")
                    log_text.see(tk.END)
                    log_text.update()

                def gui_printer(msg: str):
                    log_message(msg)
                    root.update()

                def run_simulation():
                    nonlocal result
                    # Build controller with shared mock sensors/actuators
                    pos = MockSensor("pos", SensorType.POSITION, initial_value=False)
                    obs = MockSensor("obs", SensorType.OBSTACLE, initial_value=False)
                    speed = MockSensor("speed", SensorType.SPEED, initial_value=False)
                    lim = MockSensor("lim", SensorType.LIMIT_SWITCH, initial_value=False)

                    # Test initial sensor health
                    for sensor in [pos, obs, speed, lim]:
                        if not sensor.self_check():
                            gui_printer(f"Warning: Sensor {sensor.id} failed self-check")

                    motor = MockActuator("motor", ActuatorType.MOTOR)
                    lock = MockActuator("lock", ActuatorType.LOCK)

                    driver = MockDriverInterface()
                    safety = SafetySystem()
                    external_button = ExternalButton("ext1", "door1")

                    controller = DoorController(
                        id="door1",
                        sensors=[pos, obs, speed, lim],
                        actuators=[motor, lock],
                        driver_interface=driver,
                        external_button=external_button,
                        safety_system=safety,
                    )

                    sim = JourneySimulator(
                        controller=controller,
                        position_sensor=pos,
                        obstacle_sensor=obs,
                        speed_sensor=speed,
                        motor_actuator=motor,
                        external_button=external_button,
                        obstacle_during_close=True,
                        obstacle_probability=result["obstacle_prob"],
                        persist_obstacle=result["persist_obstacle"],
                        sensor_fail_prob=result["sensor_fail_prob"],
                        printer=gui_printer,
                    )

                    for stop in range(1, result["stops"] + 1):
                        gui_printer(f"\n--- Stop {stop} ---")
                        sim.run_one_stop()
                        # If controller went out of service during the stop, halt the remaining stops
                        status = controller.status_report()
                        if status.get('out_of_service', False):
                            gui_printer("\nJourney halted: BUS OUT OF SERVICE. Remaining stops aborted.")
                            break
                        time.sleep(0.6)

                    gui_printer("\nJourney complete!")
                    
                    # Re-enable the start button
                    buttons['start'].configure(state="normal")

                def on_start():
                    try:
                        # Clear previous log and disable start button
                        log_text.delete(1.0, tk.END)
                        buttons['start'].configure(state="disabled")
                        log_message("Starting simulation...")
                        
                        # Validate and collect all parameters
                        stops = max(1, int(stops_var.get()))
                        prob = float(prob_var.get())
                        if not (0.0 <= prob <= 1.0):
                            raise ValueError("Invalid obstacle probability")
                        
                        sensor_prob = float(sensor_fail_var.get())
                        if not (0.0 <= sensor_prob <= 1.0):
                            raise ValueError("Invalid sensor probability")
                        
                        # Store all parameters
                        result["stops"] = stops
                        result["obstacle_prob"] = prob
                        result["sensor_fail_prob"] = sensor_prob
                        result["persist_obstacle"] = bool(persist_var.get())
                        result["quiet"] = bool(quiet_var.get())
                        
                        # Log configuration
                        log_message(f"Number of stops: {stops}")
                        log_message(f"Obstacle probability: {prob}")
                        log_message(f"Sensor failure probability: {sensor_prob}")
                        log_message(f"Persist obstacle: {'Yes' if result['persist_obstacle'] else 'No'}")
                        log_message(f"Quiet mode: {'Yes' if result['quiet'] else 'No'}")
                        log_message("\nStarting simulation...\n")
                        
                        # Start simulation in a separate thread
                        threading.Thread(target=run_simulation, daemon=True).start()
                        
                    except ValueError as e:
                        messagebox.showerror("Invalid input", str(e))
                        buttons['start'].configure(state="normal")
                    except Exception as e:
                        messagebox.showerror("Error", f"An error occurred: {str(e)}")
                        buttons['start'].configure(state="normal")

                def on_cancel():
                    log_message("Simulation cancelled.")
                    root.destroy()
                    sys.exit(0)

                # Button frame with increased padding
                btn_frame = ttk.Frame(frm)
                btn_frame.grid(column=0, row=8, columnspan=2, pady=(15, 5))
                
                # Create and store buttons in the dictionary
                buttons['start'] = ttk.Button(btn_frame, text="Start Simulation", command=on_start, width=15)
                buttons['start'].grid(column=0, row=0, padx=(0, 10))
                buttons['exit'] = ttk.Button(btn_frame, text="Exit", command=on_cancel, width=15)
                buttons['exit'].grid(column=1, row=0)
                
                # Create a button frame for log controls
                log_btn_frame = ttk.Frame(log_frame)
                log_btn_frame.grid(row=1, column=0, columnspan=2, pady=(5, 0), sticky="ew")
                log_btn_frame.grid_columnconfigure(0, weight=1)  # Makes buttons right-aligned
                
                # Add clear log button
                ttk.Button(log_btn_frame, text="Clear Log", 
                          command=lambda: log_text.delete(1.0, tk.END)).grid(
                    row=0, column=1, sticky="e")

                root.mainloop()
                return result

            gui_defaults = {
                "stops": args.stops, 
                "obstacle_prob": args.obstacle_prob, 
                "persist_obstacle": args.persist_obstacle, 
                "quiet": args.quiet,
                "sensor_fail_prob": 0.2  # Default sensor failure probability
            }
            gui_result = show_gui_dialog(gui_defaults)
            # Override args with GUI results if provided
            if gui_result:
                args.stops = gui_result.get("stops", args.stops)
                args.obstacle_prob = gui_result.get("obstacle_prob", args.obstacle_prob)
                args.persist_obstacle = gui_result.get("persist_obstacle", args.persist_obstacle)
                args.quiet = gui_result.get("quiet", args.quiet)
                args.sensor_fail_prob = gui_result.get("sensor_fail_prob", args.sensor_fail_prob)
    elif args.interactive:
        def ask(prompt: str, default: str) -> str:
            try:
                return input(f"{prompt} [{default}]: ") or default
            except EOFError:
                # Not interactive (e.g. running in a non-tty); fallback to default
                return default

        stops_raw = ask("Number of stops", str(args.stops))
        try:
            args.stops = max(1, int(stops_raw))
        except Exception:
            args.stops = 2

        prob_raw = ask("Obstacle probability (0.0-1.0)", str(args.obstacle_prob))
        try:
            args.obstacle_prob = min(1.0, max(0.0, float(prob_raw)))
        except Exception:
            args.obstacle_prob = 1.0

        persist_raw = ask("Persist obstacle? (y/n)", "n").lower()
        args.persist_obstacle = persist_raw.startswith("y")

        quiet_raw = ask("Quiet output? (y/n)", "n").lower()
        args.quiet = quiet_raw.startswith("y")

    printer = make_printer(not args.quiet)
    printer("Starting bus journey simulation")

    # Build a DoorController with shared mock sensors/actuators

    # Instantiate sensors/actuators
    pos = MockSensor("pos", SensorType.POSITION, initial_value=False)
    obs = MockSensor("obs", SensorType.OBSTACLE, initial_value=False)
    speed = MockSensor("speed", SensorType.SPEED, initial_value=False)
    lim = MockSensor("lim", SensorType.LIMIT_SWITCH, initial_value=False)

    # Test initial sensor health
    for sensor in [pos, obs, speed, lim]:
        if not sensor.self_check():
            print_step(f"Warning: Sensor {sensor.id} failed self-check")

    motor = MockActuator("motor", ActuatorType.MOTOR)
    lock = MockActuator("lock", ActuatorType.LOCK)

    driver = MockDriverInterface()
    safety = SafetySystem()
    external_button = ExternalButton("ext1", "door1")

    controller = DoorController(
        id="door1",
        sensors=[pos, obs, speed, lim],
        actuators=[motor, lock],
        driver_interface=driver,
        external_button=external_button,
        safety_system=safety,
    )

    sim = JourneySimulator(
        controller=controller,
        position_sensor=pos,
        obstacle_sensor=obs,
        speed_sensor=speed,
        motor_actuator=motor,
        external_button=external_button,
        obstacle_during_close=True,
        obstacle_probability=args.obstacle_prob,
        persist_obstacle=args.persist_obstacle,
        sensor_fail_prob=args.sensor_fail_prob,
        printer=printer,
    )    # Simulate a few stops
    for stop in range(1, args.stops + 1):
        printer(f"--- Stop {stop} ---")
        sim.run_one_stop()
        # If controller went out of service during the stop, halt the remaining stops
        status = controller.status_report()
        if status.get('out_of_service', False):
            printer("Journey halted: BUS OUT OF SERVICE. Remaining stops aborted.")
            break
        # short travel time
        time.sleep(0.6)

    printer("Journey complete")


if __name__ == "__main__":
    main()
