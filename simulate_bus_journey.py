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
    def __init__(self, controller: DoorController, position_sensor, obstacle_sensor, motor_actuator, obstacle_during_close: bool = True, obstacle_probability: float = 1.0, persist_obstacle: bool = False, printer: Callable[[str], None] = print_step):
        self.controller = controller
        self.position = position_sensor
        self.obstacle_sensor = obstacle_sensor
        self.motor = motor_actuator
        self.obstacle = False
        self.obstacle_during_close = obstacle_during_close
        self.obstacle_probability = float(obstacle_probability)
        self.persist_obstacle = bool(persist_obstacle)
        self.print = printer
        self._lock = threading.Lock()

    def approach_stop(self, dwell_s: float = 0.5):
        self.print("Bus approaching stop")
        time.sleep(dwell_s)
        self.print("Bus stopped")

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
        self.print("Bus departing")

    def run_one_stop(self):
        self.approach_stop()
        self.open_doors()
        self.passengers_boarding()
        closed = self.close_doors()
        if not closed:
            # wait for obstacle to be removed if persists
            self.print("Waiting for obstacle to be cleared")
            # in this demo we assume obstacle is removed shortly by the trigger thread
            time.sleep(0.6)
            # attempt to close again
            self.print("Attempting to close doors again")
            closed = self.close_doors(close_time_s=0.6)
        if closed:
            self.depart()
        else:
            self.print("Unable to close doors - signalling fault and awaiting assistance")


def main():
    parser = argparse.ArgumentParser(description="Simulate a bus journey with door operations and optional obstacles")
    parser.add_argument("--stops", type=int, default=2, help="Number of stops to simulate (default: 2)")
    parser.add_argument("--obstacle-prob", type=float, default=1.0, help="Probability (0.0-1.0) that an obstacle appears during closing (default: 1.0)")
    parser.add_argument("--persist-obstacle", action="store_true", help="If set, obstacles persist until cleared manually (default: False)")
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
                root.resizable(False, False)

                frm = ttk.Frame(root, padding=12)
                frm.grid()

                ttk.Label(frm, text="Number of stops:").grid(column=0, row=0, sticky="w")
                stops_var = tk.StringVar(value=str(defaults.get("stops", 2)))
                stops_entry = ttk.Entry(frm, width=10, textvariable=stops_var)
                stops_entry.grid(column=1, row=0)

                ttk.Label(frm, text="Obstacle probability (0.0-1.0):").grid(column=0, row=1, sticky="w")
                prob_var = tk.StringVar(value=str(defaults.get("obstacle_prob", 1.0)))
                prob_entry = ttk.Entry(frm, width=10, textvariable=prob_var)
                prob_entry.grid(column=1, row=1)

                persist_var = tk.BooleanVar(value=bool(defaults.get("persist_obstacle", False)))
                persist_cb = ttk.Checkbutton(frm, text="Persist obstacle", variable=persist_var)
                persist_cb.grid(column=0, row=2, columnspan=2, sticky="w")

                quiet_var = tk.BooleanVar(value=bool(defaults.get("quiet", False)))
                quiet_cb = ttk.Checkbutton(frm, text="Quiet output", variable=quiet_var)
                quiet_cb.grid(column=0, row=3, columnspan=2, sticky="w")

                result = {}

                def on_start():
                    try:
                        result["stops"] = max(1, int(stops_var.get()))
                    except Exception:
                        messagebox.showerror("Invalid input", "Number of stops must be an integer >= 1")
                        return
                    try:
                        prob = float(prob_var.get())
                        if not (0.0 <= prob <= 1.0):
                            raise ValueError()
                        result["obstacle_prob"] = prob
                    except Exception:
                        messagebox.showerror("Invalid input", "Obstacle probability must be a number between 0.0 and 1.0")
                        return
                    result["persist_obstacle"] = bool(persist_var.get())
                    result["quiet"] = bool(quiet_var.get())
                    root.destroy()

                def on_cancel():
                    root.destroy()
                    sys.exit(0)

                btn_frame = ttk.Frame(frm)
                btn_frame.grid(column=0, row=4, columnspan=2, pady=(8, 0))
                ttk.Button(btn_frame, text="Start", command=on_start).grid(column=0, row=0, padx=(0, 6))
                ttk.Button(btn_frame, text="Cancel", command=on_cancel).grid(column=1, row=0)

                root.mainloop()
                return result

            gui_defaults = {"stops": args.stops, "obstacle_prob": args.obstacle_prob, "persist_obstacle": args.persist_obstacle, "quiet": args.quiet}
            gui_result = show_gui_dialog(gui_defaults)
            # Override args with GUI results if provided
            if gui_result:
                args.stops = gui_result.get("stops", args.stops)
                args.obstacle_prob = gui_result.get("obstacle_prob", args.obstacle_prob)
                args.persist_obstacle = gui_result.get("persist_obstacle", args.persist_obstacle)
                args.quiet = gui_result.get("quiet", args.quiet)
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
    lim = MockSensor("lim", SensorType.LIMIT_SWITCH, initial_value=False)

    motor = MockActuator("motor", ActuatorType.MOTOR)
    lock = MockActuator("lock", ActuatorType.LOCK)

    driver = MockDriverInterface()
    safety = SafetySystem()

    controller = DoorController(
        id="door1",
        sensors=[pos, obs, lim],
        actuators=[motor, lock],
        driver_interface=driver,
        safety_system=safety,
    )

    sim = JourneySimulator(
        controller,
        pos,
        obs,
        motor,
        obstacle_during_close=True,
        obstacle_probability=args.obstacle_prob,
        persist_obstacle=args.persist_obstacle,
        printer=printer,
    )

    # Simulate a few stops
    for stop in range(1, args.stops + 1):
        printer(f"--- Stop {stop} ---")
        sim.run_one_stop()
        # short travel time
        time.sleep(0.6)

    printer("Journey complete")


if __name__ == "__main__":
    main()
