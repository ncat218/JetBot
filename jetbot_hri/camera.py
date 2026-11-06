"""Camera opening boundary. The CSI pipeline must be checked on the actual JetBot."""


def csi_gstreamer_pipeline(config):
    sensor = int(config.get("source", 0))
    width = int(config.get("width", 640))
    height = int(config.get("height", 480))
    fps = int(config.get("fps", 30))
    if sensor < 0 or width <= 0 or height <= 0 or fps <= 0:
        raise ValueError("Invalid CSI camera source, dimensions, or FPS")
    return ("nvarguscamerasrc sensor-id={sensor} ! "
            "video/x-raw(memory:NVMM), width={width}, height={height}, "
            "format=(string)NV12, framerate=(fraction){fps}/1 ! "
            "nvvidconv ! video/x-raw, width=(int){width}, "
            "height=(int){height}, format=(string)BGRx ! "
            "videoconvert ! appsink drop=true max-buffers=1").format(
                sensor=sensor, width=width, height=height, fps=fps)


def open_camera(config):
    import cv2
    backend = config.get("backend", "opencv")
    if backend == "csi_gstreamer":
        capture = cv2.VideoCapture(csi_gstreamer_pipeline(config), cv2.CAP_GSTREAMER)
    elif backend == "opencv":
        capture = cv2.VideoCapture(config.get("source", 0))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(config.get("width", 640)))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(config.get("height", 480)))
    else:
        raise ValueError("Unknown camera backend: " + str(backend))
    if not capture.isOpened():
        capture.release()
        raise RuntimeError("Camera could not open with backend " + backend)
    return capture
