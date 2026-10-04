import argparse
from dataclasses import asdict
import json
from pathlib import Path
import signal
import sys
import time

from .configuration import load_config, validate_robot_config
from .control import (MockMotorBackend, JetBotMotorBackend, SafeMotorController,
                      SafetySupervisor, stop)
from .interaction import StateMachine
from .platform import BatteryMonitor, INA219Backend, MockBatteryBackend, get_wifi_status
from .types import Observation, Person


def log_record(stream, fsm, obs, desired, actual, battery, wifi, supervisor):
    record = {"timestamp": obs.timestamp, "state": fsm.state, "transition": fsm.transition,
              "fps": obs.fps, "frame_age": max(0, time.monotonic()-obs.frame_timestamp),
              "wifi": asdict(wifi), "battery": asdict(battery), "motion_allowed": supervisor.reason is None,
              "collision": obs.blocked, "owner_visible": obs.owner_visible,
              "owner_bbox": obs.owner_bbox, "gesture": obs.gesture, "point_side": obs.point_side,
              "candidate_bbox": obs.candidate_bbox, "receiver_bbox": obs.receiver_bbox,
              "desired_motion": asdict(desired), "actual_motor": asdict(actual),
              "safety_reason": supervisor.reason, "abort_reason": fsm.abort_reason}
    stream.write(json.dumps(record, ensure_ascii=False) + "\n")
    stream.flush()


def mock_observation(fsm, now):
    """Deterministic full-protocol scenario. Timestamps are synthetic for fast dry-run."""
    owner_gesture = "NONE"
    owner_box = (0.35, 0.2, 0.65, 0.75)
    if fsm.state == "SEARCH_OWNER" or (fsm.state == "APPROACH_OWNER" and
                                       now-fsm.entered < 0.4):
        owner_box = (0.42, 0.2, 0.58, 0.43)
    candidate_box = (0.42, 0.2, 0.58, 0.55)
    receiver_box = (0.38, 0.1, 0.62, 0.75)
    if fsm.state == "DELIVERY" and now-fsm.entered < 0.4:
        receiver_box = candidate_box
    if fsm.state == "SEARCH_OWNER":
        owner_gesture = "WAVE"
    if fsm.state in ("AIMING", "PULSE_SCAN"):
        owner_gesture = "POINT_RIGHT"
    if fsm.state == "WAIT_OWNER_DECISION":
        owner_gesture = "CONFIRM"
    owner = Person(owner_box, 1, gesture=owner_gesture,
                   point_side="RIGHT" if owner_gesture == "POINT_RIGHT" else "NONE")
    people = [owner]
    if fsm.state in ("PULSE_SCAN", "WAIT_OWNER_DECISION", "LOCK_RECEIVER", "DELIVERY"):
        people.append(Person(receiver_box if fsm.state == "DELIVERY" else candidate_box, 2))
    return Observation(timestamp=now, frame_timestamp=time.monotonic(), people=people,
                       owner_visible_raw=True, owner_visible=True, owner_bbox=owner_box,
                       owner_pointing=owner_gesture == "POINT_RIGHT",
                       point_side=owner.point_side, gesture=owner_gesture,
                       candidate_visible=len(people)>1 and fsm.state != "DELIVERY",
                       candidate_bbox=people[-1].bbox if len(people)>1 and fsm.state != "DELIVERY" else None,
                       receiver_visible=fsm.state == "DELIVERY",
                       receiver_bbox=receiver_box if fsm.state == "DELIVERY" else None,
                       wave_count=1 if owner_gesture == "WAVE" else 0, fps=10.0)


def run_dry(config, log):
    fsm = StateMachine(config)
    clock = [0.0]
    timer = lambda: clock[0]
    battery = BatteryMonitor(MockBatteryBackend(), config["battery"], clock=timer)
    for _ in range(config["battery"]["filter_window"]):
        battery.read()
    safety_cfg = dict(config["safety"])
    safety_cfg["stop_on_wifi_loss"] = config["wifi"]["stop_on_wifi_loss"]
    safety = SafetySupervisor(safety_cfg, battery=battery)
    motor = SafeMotorController(MockMotorBackend(), safety, config["safety"])
    wifi = get_wifi_status()
    try:
        for _ in range(300):
            obs = mock_observation(fsm, clock[0])
            desired = fsm.step(obs)
            actual = motor.set(desired, obs, fsm.state, wifi)
            log_record(log, fsm, obs, desired, actual, battery.last, wifi, safety)
            if fsm.transition:
                print("{:.1f}s {} -> {}".format(clock[0], fsm.transition["from"], fsm.state))
            if fsm.state == "COMPLETE":
                print("Dry-run COMPLETE")
                return 0
            if fsm.state == "ABORT":
                break
            clock[0] += 0.1
            battery.read()
        print("Dry-run did not complete; final state=" + fsm.state, file=sys.stderr)
        return 1
    finally:
        motor.close()
        battery.close()


def draw_overlay(frame, fsm, obs, battery, wifi, safety):
    import cv2
    lines = ["STATE " + fsm.state, "FPS {:.1f}".format(obs.fps),
             "WiFi {} {}".format(wifi.ssid or "OFF", wifi.ip_address or ""),
             "Battery {:.2f}V {}".format(battery.voltage, battery.status),
             "Collision {}".format("BLOCKED" if obs.blocked else "FREE"),
             "Gesture {} point {}".format(obs.gesture, obs.point_side),
             "E-STOP {} (press E)".format(safety.emergency_latched)]
    for i, line in enumerate(lines):
        cv2.putText(frame, line, (8, 22+24*i), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255), 2)
    h, w = frame.shape[:2]
    for label, box, color in (("Owner", obs.owner_bbox, (255, 0, 0)),
                              ("Candidate", obs.candidate_bbox, (0, 255, 255)),
                              ("Receiver", obs.receiver_bbox, (0, 255, 0))):
        if box:
            x1, y1, x2, y2 = box
            cv2.rectangle(frame, (int(x1*w), int(y1*h)), (int(x2*w), int(y2*h)), color, 2)
            cv2.putText(frame, label, (int(x1*w), int(y1*h)-4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    for person in obs.people:
        points = person.keypoints
        for joint_a, joint_b in (("shoulder_l", "elbow_l"), ("elbow_l", "wrist_l"),
                                 ("shoulder_r", "elbow_r"), ("elbow_r", "wrist_r"),
                                 ("shoulder_l", "shoulder_r")):
            if joint_a in points and joint_b in points:
                a, b = points[joint_a], points[joint_b]
                cv2.line(frame, (int(a[0]*w), int(a[1]*h)),
                         (int(b[0]*w), int(b[1]*h)), (255, 255, 0), 2)
    return frame


def run_camera(config, mode, log, show, dashboard_port):
    import cv2
    from .perception import TRTPOSEBackend, CollisionModelBackend, ObservationBuilder
    pose = TRTPOSEBackend(config["pose"])
    collision = CollisionModelBackend(config["collision"])
    capture = cv2.VideoCapture(config["camera"]["source"])
    if not capture.isOpened():
        raise RuntimeError("Camera could not open; check IMX219 / GStreamer source")
    battery = None
    motor = None
    dashboard = None
    try:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, config["camera"]["width"])
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config["camera"]["height"])
        if mode == "robot":
            battery = BatteryMonitor(INA219Backend(config["battery"]), config["battery"])
            for _ in range(config["battery"]["filter_window"]):
                battery.read()
                if battery.last.status == "SENSOR_ERROR":
                    raise RuntimeError("Battery sensor error; robot motion disabled")
                time.sleep(config["battery"]["sample_interval_sec"])
            motor_backend = JetBotMotorBackend()
        else:
            battery = BatteryMonitor(MockBatteryBackend(), config["battery"])
            for _ in range(config["battery"]["filter_window"]):
                battery.read()
            motor_backend = MockMotorBackend()
        safety_cfg = dict(config["safety"])
        safety_cfg["stop_on_wifi_loss"] = config["wifi"]["stop_on_wifi_loss"]
        safety = SafetySupervisor(safety_cfg, battery=battery)
        motor = SafeMotorController(motor_backend, safety, config["safety"])
        if dashboard_port:
            from .dashboard import Dashboard
            dashboard = Dashboard(safety, dashboard_port)
        builder = ObservationBuilder(config["gesture"])
        fsm = StateMachine(config)
        last = time.monotonic()
        wifi = get_wifi_status()
        wifi_at = last
        battery_at = last
        while True:
            ok, frame = capture.read()
            now = time.monotonic()
            if not ok:
                motor.stop()
                raise RuntimeError("Camera frame read failed")
            if now-battery_at >= config["battery"]["sample_interval_sec"]:
                battery.read()
                battery_at = now
            if now-wifi_at >= 3:
                wifi = get_wifi_status()
                wifi_at = now
            people = pose.infer(frame)
            blocked, confidence = collision.infer(frame)
            obs = builder.build(people, blocked, now, now, 1/max(1e-6, now-last))
            obs.collision_confidence = confidence
            builder.owner_anchor = fsm.owner_anchor
            builder.receiver_anchor = fsm.receiver_anchor
            desired = fsm.step(obs) if mode == "robot" else stop()
            actual = motor.set(desired, obs, fsm.state, wifi) if mode == "robot" else stop()
            log_record(log, fsm, obs, desired, actual, battery.last, wifi, safety)
            if dashboard:
                dashboard.update(state=fsm.state, fps=obs.fps, battery=asdict(battery.last),
                                 wifi=asdict(wifi), blocked=obs.blocked,
                                 emergency_stop=safety.emergency_latched,
                                 safety_reason=safety.reason, actual_motor=asdict(actual))
            last = now
            if show:
                cv2.imshow("JetBot HRI", draw_overlay(frame, fsm, obs, battery.last, wifi, safety))
                key = cv2.waitKey(1) & 0xff
                if key == ord("e"):
                    safety.emergency_stop()
                    motor.stop()
                if key == ord("q"):
                    break
            if fsm.state in ("COMPLETE", "ABORT"):
                break
    finally:
        if dashboard:
            dashboard.close()
        if motor:
            motor.close()
        if battery:
            battery.close()
        capture.release()
        if show:
            cv2.destroyAllWindows()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("dry-run", "perception", "robot"), default="dry-run")
    parser.add_argument("--config")
    parser.add_argument("--log", default="logs/run.jsonl")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--dashboard-port", type=int, default=8765,
                        help="loopback-only dashboard; 0 disables it")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    if args.mode == "robot":
        if not args.config:
            parser.error("Robot mode requires --config config/calibration.yaml")
        validate_robot_config(config)
    path = Path(args.log)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as log:
        return run_dry(config, log) if args.mode == "dry-run" else run_camera(
            config, args.mode, log, not args.no_display, args.dashboard_port)


if __name__ == "__main__":
    def _shutdown_signal(_signum, _frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _shutdown_signal)
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except (RuntimeError, ValueError, ImportError) as exc:
        print("JetBot HRI startup/runtime error: " + str(exc), file=sys.stderr)
        sys.exit(2)
