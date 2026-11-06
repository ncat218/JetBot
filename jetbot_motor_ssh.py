"""Run a bounded motor test from the JetBot project directory."""
import argparse
import signal
import time

from jetbot_hri.configuration import load_config
from jetbot_hri.control import JetBotMotorBackend, SafeMotorController, SafetySupervisor
from jetbot_hri.platform import BatteryMonitor, INA219Backend
from jetbot_hri.types import Motion, Observation


def run_test(direction):
    speed = 0.12
    # Turn duration selected for the physical JetBot test.
    turn_duration = 0.90
    pulses = {
        "forward": (speed, speed, 1.0),
        "backward": (-speed, -speed, 1.0),
        "left": (-speed, speed, turn_duration),
        "right": (speed, -speed, turn_duration),
    }
    left, right, duration = pulses[direction]
    config = load_config("config/calibration.yaml")
    pin = config["battery"]
    safety = dict(config["safety"], max_forward_speed=speed, max_turn_speed=speed)
    battery = BatteryMonitor(INA219Backend(pin), pin)
    supervisor = SafetySupervisor(safety, battery)
    motor = None
    try:
        for index in range(pin["filter_window"]):
            reading = battery.read()
            if reading.status != "NORMAL":
                raise SystemExit("Battery: " + reading.status)
            if index + 1 < pin["filter_window"]:
                time.sleep(pin["sample_interval_sec"])
        if not battery.healthy():
            raise SystemExit("Battery sample window is invalid")

        motor = SafeMotorController(JetBotMotorBackend(), supervisor, safety)
        reading = battery.read()
        print("Fresh battery:", reading, flush=True)
        if reading.status != "NORMAL" or not battery.healthy():
            raise SystemExit("Battery check failed after driver startup")

        motion = Motion(left, right, direction)
        deadline = time.monotonic() + duration
        refresh = min(0.05, safety["motor_command_ttl_sec"] / 2)
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            actual = motor.set(motion, Observation(now, now), "TEST_MOTOR")
            if actual.left == 0 and actual.right == 0:
                raise SystemExit("STOP reason: " + str(supervisor.reason))
            time.sleep(min(refresh, max(0.0, deadline - time.monotonic())))
        motor.stop()
        print("Pulse sent: {} for {:.2f}s; STOP sent".format(direction, duration), flush=True)
    finally:
        try:
            if motor is not None:
                motor.close()
        finally:
            battery.close()


def stop_on_signal(_signum, _frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("direction", choices=("forward", "backward", "left", "right"))
    setup = parser.add_mutually_exclusive_group(required=True)
    setup.add_argument("--wheels-lifted", action="store_true", help="both wheels are off the floor")
    setup.add_argument("--clear-floor", action="store_true", help="test a turn on a clear, flat floor")
    args = parser.parse_args()
    if args.clear_floor and args.direction not in ("left", "right"):
        parser.error("--clear-floor is only for left and right turns")
    for name in ("SIGTERM", "SIGHUP"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), stop_on_signal)
    run_test(args.direction)
