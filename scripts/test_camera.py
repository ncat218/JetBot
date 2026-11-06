"""Motor-free camera test: read frames and save a preview for Jupyter."""
import argparse
from pathlib import Path
import time

from jetbot_hri.camera import open_camera
from jetbot_hri.configuration import load_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/calibration.yaml")
    parser.add_argument("--backend", choices=("opencv", "csi_gstreamer"))
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--output", default="logs/camera_check.jpg")
    args = parser.parse_args()
    if args.frames <= 0:
        parser.error("--frames must be positive")
    cfg = dict(load_config(args.config)["camera"])
    if args.backend:
        cfg["backend"] = args.backend
    import cv2
    camera = open_camera(cfg)
    started = time.monotonic()
    frame = None
    try:
        for index in range(args.frames):
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Camera read failed at frame " + str(index + 1))
    finally:
        camera.release()
    elapsed = max(time.monotonic() - started, 1e-6)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), frame):
        raise RuntimeError("Could not save camera preview")
    print("Backend: {} | Frames: {} | Size: {} | FPS: {:.1f} | Preview: {}".format(
        cfg.get("backend", "opencv"), args.frames, frame.shape,
        args.frames / elapsed, output))


if __name__ == "__main__":
    main()

