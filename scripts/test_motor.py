"""A short motor pulse through the safety controller; never raw Robot calls."""
import argparse
import time
from jetbot_hri.configuration import load_config
from jetbot_hri.control import JetBotMotorBackend, SafeMotorController, SafetySupervisor, commit_forward
from jetbot_hri.platform import BatteryMonitor, INA219Backend
from jetbot_hri.types import Observation

parser = argparse.ArgumentParser()
parser.add_argument("--config", default="config/calibration.yaml")
parser.add_argument("--wheels-lifted", action="store_true")
args = parser.parse_args()
if not args.wheels_lifted:
    parser.error("Lift wheels or secure robot, then pass --wheels-lifted")
cfg = load_config(args.config)
for key in ("i2c_address",):
    if not isinstance(cfg["battery"][key], int):
        raise RuntimeError("Calibrate battery." + key + " first")
if not isinstance(cfg["safety"]["max_forward_speed"], (int, float)):
    raise RuntimeError("Calibrate safety.max_forward_speed first")
battery = BatteryMonitor(INA219Backend(cfg["battery"]), cfg["battery"])
motor = None
try:
    for _ in range(cfg["battery"]["filter_window"]):
        battery.read()
        time.sleep(cfg["battery"]["sample_interval_sec"])
    if not battery.healthy():
        raise RuntimeError("Battery sensor not healthy")
    supervisor = SafetySupervisor(cfg["safety"], battery)
    motor = SafeMotorController(JetBotMotorBackend(), supervisor, cfg["safety"])
    deadline = time.monotonic()+0.2
    while time.monotonic() < deadline:
        now = time.monotonic()
        obs = Observation(now, now)
        motor.set(commit_forward(min(0.08, cfg["safety"]["max_forward_speed"])), obs,
                  "TEST_MOTOR")
        time.sleep(0.03)
finally:
    if motor:
        motor.close()
    battery.close()
