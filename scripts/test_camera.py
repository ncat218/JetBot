import argparse
import cv2
from jetbot_hri.configuration import load_config

parser = argparse.ArgumentParser()
parser.add_argument("--config", default="config/calibration.yaml")
args = parser.parse_args()
cfg = load_config(args.config)["camera"]
capture = cv2.VideoCapture(cfg["source"])
try:
    ok, frame = capture.read()
    if not ok:
        raise RuntimeError("Camera read failed; check source/GStreamer permissions")
    print("Camera frame:", frame.shape)
finally:
    capture.release()

