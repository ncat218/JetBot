"""Only SafeMotorController owns a raw motor backend."""
import threading
import time
from .types import Motion


class MockMotorBackend:
    def __init__(self):
        self.last = (0.0, 0.0)
        self.stops = 0

    def set_motors(self, left, right):
        self.last = (left, right)

    def stop(self):
        self.last = (0.0, 0.0)
        self.stops += 1


class JetBotMotorBackend:
    def __init__(self):
        from jetbot import Robot
        self.robot = Robot()
        self.robot.stop()

    def set_motors(self, left, right):
        self.robot.set_motors(left, right)

    def stop(self):
        self.robot.stop()


class SafetySupervisor:
    def __init__(self, config, battery=None, clock=time.monotonic):
        self.config, self.battery, self.clock = config, battery, clock
        self.emergency_latched = False
        self.reason = None

    def emergency_stop(self):
        self.emergency_latched = True
        self.reason = "EMERGENCY_STOP"

    def manual_reset(self):
        self.emergency_latched = False
        self.reason = None

    def allows(self, observation, state, wifi=None, motion=None):
        allowed_states = {"SEARCH_OWNER", "APPROACH_OWNER", "WAIT_LOAD", "POSE_STABLE",
                          "AIMING", "PULSE_SCAN", "WAIT_OWNER_DECISION", "LOCK_RECEIVER",
                          "DELIVERY", "AVOID", "WAIT_RECEIVER_WAVE", "AMBIGUOUS_CALLER",
                          "OWNER_LOST", "OWNER_AT_FOV_LIMIT", "SCAN_BLOCKED",
                          "AMBIGUOUS_RECEIVER", "ARRIVED", "COMPLETE", "TEST_MOTOR"}
        if self.emergency_latched or state not in allowed_states:
            self.reason = "EMERGENCY_STOP" if self.emergency_latched else "INVALID_STATE"
        elif self.battery is not None and not self.battery.healthy():
            self.reason = "BATTERY_OR_SENSOR"
        elif observation is None or self.clock() - observation.frame_timestamp > self.config["max_frame_age_sec"]:
            self.reason = "STALE_FRAME"
        elif observation.blocked and not (state == "AVOID" and motion is not None and
                                          motion.left * motion.right < 0):
            self.reason = "COLLISION"
        elif self.config.get("stop_on_wifi_loss", False) and wifi is not None and not wifi.connected:
            self.reason = "WIFI_LOSS"
        else:
            self.reason = None
        return self.reason is None


class SafeMotorController:
    def __init__(self, backend, supervisor, config, clock=time.monotonic):
        self._backend, self._supervisor, self._config, self._clock = backend, supervisor, config, clock
        self._lock = threading.RLock()
        self._deadline = 0.0
        self._closed = threading.Event()
        self.actual = Motion()
        self._backend.stop()
        self._thread = threading.Thread(target=self._watch, daemon=True)
        self._thread.start()

    def set(self, motion, observation, state, wifi=None):
        with self._lock:
            if not self._supervisor.allows(observation, state, wifi, motion):
                self.stop()
                return self.actual
            forward = self._config["max_forward_speed"]
            turn = self._config["max_turn_speed"]
            left = max(-turn, min(forward, motion.left))
            right = max(-turn, min(forward, motion.right))
            self._backend.set_motors(left, right)
            self.actual = Motion(left, right, motion.name)
            self._deadline = self._clock() + self._config["motor_command_ttl_sec"]
            return self.actual

    def stop(self):
        with self._lock:
            self._backend.stop()
            self.actual = Motion()
            self._deadline = 0.0

    def _watch(self):
        while not self._closed.wait(0.02):
            with self._lock:
                if self._deadline and (self._clock() >= self._deadline or
                                       self._supervisor.emergency_latched or
                                       (self._supervisor.battery is not None and
                                        not self._supervisor.battery.healthy())):
                    self.stop()

    def close(self):
        self._closed.set()
        self._thread.join(timeout=1)
        self.stop()


def stop():
    return Motion()


def visual_servo(bbox, speed, gain, stop_ratio):
    if bbox is None or bbox[3] - bbox[1] >= stop_ratio:
        return stop()
    error = (bbox[0] + bbox[2]) / 2 - 0.5
    turn = max(-speed, min(speed, gain * error))
    return Motion(speed + turn, speed - turn, "VISUAL_SERVO")


def pulse_turn(side, speed):
    return Motion(-speed, speed, "PULSE_TURN") if side == "LEFT" else Motion(speed, -speed, "PULSE_TURN")


def commit_forward(speed):
    return Motion(speed, speed, "COMMIT_FORWARD")


def avoid_step(side, speed):
    return pulse_turn(side, speed)
