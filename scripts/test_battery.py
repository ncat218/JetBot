import argparse
from jetbot_hri.configuration import load_config
from jetbot_hri.platform import BatteryMonitor, INA219Backend, MockBatteryBackend

parser = argparse.ArgumentParser()
parser.add_argument("--mock", action="store_true")
parser.add_argument("--config", default="config/calibration.yaml")
args = parser.parse_args()
config = load_config(args.config)["battery"]
backend = MockBatteryBackend() if args.mock else INA219Backend(config)
monitor = BatteryMonitor(backend, config)
try:
    for _ in range(config["filter_window"]):
        print(monitor.read())
finally:
    monitor.close()

