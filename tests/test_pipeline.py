import time
import unittest
from http.client import HTTPConnection

from jetbot_hri.configuration import load_config, validate_robot_config
from jetbot_hri.control import (MockMotorBackend, SafeMotorController, SafetySupervisor,
                                commit_forward, pulse_turn)
from jetbot_hri.dashboard import Dashboard
from jetbot_hri.interaction import StateMachine
from jetbot_hri.perception import classify_arm, GestureHistory
from jetbot_hri.platform import BatteryMonitor, MockBatteryBackend
from jetbot_hri.types import Observation, Person


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, value=1.0):
        self.now += value


def person(box, gesture="NONE", identity=1):
    return Person(box, identity, gesture=gesture,
                  point_side=gesture[6:] if gesture.startswith("POINT_") else "NONE")


OWNER = (0.3, 0.1, 0.7, 0.7)
CANDIDATE = (0.43, 0.2, 0.58, 0.55)


def observation(clock, owner=None, candidate=None, blocked=False, waves=0):
    people = [p for p in (owner, candidate) if p is not None]
    return Observation(clock(), clock(), people=people, owner_visible=owner is not None,
                       owner_bbox=owner.bbox if owner else None, blocked=blocked,
                       wave_count=waves, candidate_bbox=candidate.bbox if candidate else None)


class BatteryTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.backend = MockBatteryBackend()
        self.monitor = BatteryMonitor(self.backend, load_config()["battery"], self.clock)

    def sample(self, voltage, count):
        self.backend.voltage = voltage
        for _ in range(count):
            self.monitor.read()
            self.clock.advance()

    def test_sag_then_sustained_critical_and_latch(self):
        self.sample(11.0, 5)
        self.sample(9.8, 1)
        self.assertFalse(self.monitor.is_critical_latched())
        self.sample(11.0, 5)
        self.sample(9.8, 8)
        self.assertTrue(self.monitor.is_critical_latched())
        self.assertEqual(self.monitor.last.status, "CRITICAL")
        self.sample(11.4, 5)
        self.assertTrue(self.monitor.is_critical_latched())
        self.monitor.reset_latch()
        self.assertFalse(self.monitor.is_critical_latched())

    def test_sensor_error_latches(self):
        self.backend.voltage = -1
        self.assertEqual(self.monitor.read().status, "SENSOR_ERROR")
        self.assertFalse(self.monitor.healthy())


class SafetyTests(unittest.TestCase):
    def setUp(self):
        cfg = load_config()
        self.clock = FakeClock()
        self.safety = SafetySupervisor(cfg["safety"], clock=self.clock)
        self.backend = MockMotorBackend()
        self.motor = SafeMotorController(self.backend, self.safety, cfg["safety"], self.clock)

    def tearDown(self):
        self.motor.close()

    def test_stale_and_emergency(self):
        obs = observation(self.clock)
        self.motor.set(commit_forward(0.1), obs, "DELIVERY")
        self.clock.advance(1)
        self.motor.set(commit_forward(0.1), obs, "DELIVERY")
        self.assertEqual(self.backend.last, (0, 0))
        self.safety.emergency_stop()
        fresh = observation(self.clock)
        self.motor.set(commit_forward(0.1), fresh, "DELIVERY")
        self.assertEqual(self.backend.last, (0, 0))

    def test_collision_turn_exception(self):
        obs = observation(self.clock, blocked=True)
        self.motor.set(commit_forward(0.1), obs, "AVOID")
        self.assertEqual(self.backend.last, (0, 0))
        self.motor.set(pulse_turn("LEFT", 0.1), obs, "AVOID")
        self.assertNotEqual(self.backend.last, (0, 0))

    def test_ttl(self):
        obs = observation(self.clock)
        self.motor.set(commit_forward(0.1), obs, "DELIVERY")
        self.clock.advance(1)
        time.sleep(0.06)
        self.assertEqual(self.backend.last, (0, 0))

    def test_dashboard_emergency_stop(self):
        dashboard = Dashboard(self.safety, 0)
        port = dashboard.server.server_address[1]
        try:
            connection = HTTPConnection("127.0.0.1", port, timeout=2)
            connection.request("POST", "/estop")
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            response.read()
            self.assertTrue(self.safety.emergency_latched)
            connection.close()
        finally:
            dashboard.close()


class FSMTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.fsm = StateMachine(load_config())
        self.owner = person(OWNER)
        self.candidate = person(CANDIDATE, identity=2)

    def step(self, owner=None, candidate=None, blocked=False, waves=0):
        return self.fsm.step(observation(self.clock, owner, candidate, blocked, waves))

    def test_call_ambiguity_and_approach(self):
        self.step(self.owner, self.candidate, waves=2)
        self.assertEqual(self.fsm.state, "AMBIGUOUS_CALLER")
        self.step(self.owner, waves=0)
        self.assertEqual(self.fsm.state, "SEARCH_OWNER")
        self.owner.gesture = "WAVE"
        self.step(self.owner, waves=1)
        self.assertEqual(self.fsm.state, "APPROACH_OWNER")

    def test_approach_and_delivery_blocked(self):
        for initial in ("APPROACH_OWNER", "DELIVERY"):
            self.fsm.state = initial
            self.fsm.owner_anchor = OWNER
            self.fsm.receiver_anchor = CANDIDATE
            self.step(self.owner, self.candidate, blocked=True)
            self.assertEqual(self.fsm.state, "AVOID")

    def test_scan_blocked_does_not_avoid(self):
        self.fsm.state = "PULSE_SCAN"
        self.fsm.owner_anchor = OWNER
        self.step(self.owner, blocked=True)
        self.assertEqual(self.fsm.state, "SCAN_BLOCKED")

    def test_scan_ambiguous_candidates_waits(self):
        self.fsm.state = "PULSE_SCAN"
        self.fsm.owner_anchor = OWNER
        self.fsm.scan_side = "RIGHT"
        self.fsm.scan_phase = "SETTLE"
        self.clock.advance(1)
        owner = person(OWNER, "POINT_RIGHT")
        second = person((0.35, 0.2, 0.55, 0.6), identity=3)
        obs = observation(self.clock, owner, self.candidate)
        obs.people.append(second)
        self.fsm.step(obs)
        self.assertEqual(self.fsm.state, "PULSE_SCAN")

    def test_decision_owner_only_confirm_and_reject(self):
        self.fsm.state = "WAIT_OWNER_DECISION"
        self.fsm.decision_owner_anchor = OWNER
        self.fsm.candidate_anchor = CANDIDATE
        self.fsm.scan_side = "RIGHT"
        self.candidate.gesture = "CONFIRM"
        self.step(self.owner, self.candidate)
        self.assertEqual(self.fsm.state, "WAIT_OWNER_DECISION")
        self.owner.gesture = "CONFIRM"
        self.step(self.owner, self.candidate)
        self.assertEqual(self.fsm.state, "LOCK_RECEIVER")
        self.fsm.state = "WAIT_OWNER_DECISION"
        self.owner.gesture = "POINT_RIGHT"
        self.owner.point_side = "RIGHT"
        self.clock.advance(1)
        self.step(self.owner, self.candidate)
        self.assertEqual(self.fsm.state, "PULSE_SCAN")

    def test_owner_lost_receiver_lost_avoid_timeout_abort_reset(self):
        self.fsm.state = "AIMING"
        self.fsm.owner_anchor = OWNER
        self.step()
        self.assertEqual(self.fsm.state, "OWNER_LOST")
        self.clock.advance(4)
        self.step()
        self.assertEqual(self.fsm.state, "SEARCH_OWNER")
        self.fsm.state = "DELIVERY"
        self.fsm.receiver_anchor = CANDIDATE
        self.step(self.owner)
        self.assertEqual(self.fsm.state, "WAIT_RECEIVER_WAVE")
        self.fsm.state = "AVOID"
        self.fsm.avoid_started = self.clock()
        self.clock.advance(7)
        self.step(blocked=True)
        self.assertEqual(self.fsm.state, "AVOID_TIMEOUT")
        self.step()
        self.assertEqual(self.fsm.state, "ABORT")
        self.step(self.owner, waves=1)
        self.assertEqual(self.fsm.state, "ABORT")
        self.fsm.manual_reset(self.clock())
        self.assertEqual(self.fsm.state, "SEARCH_OWNER")

    def test_owner_wave_cannot_reacquire_receiver(self):
        self.fsm.state = "WAIT_RECEIVER_WAVE"
        self.fsm.owner_anchor = OWNER
        self.owner.gesture = "WAVE"
        self.step(self.owner, waves=1)
        self.assertEqual(self.fsm.state, "WAIT_RECEIVER_WAVE")
        self.candidate.gesture = "WAVE"
        self.step(self.owner, self.candidate, waves=2)
        self.assertEqual(self.fsm.state, "DELIVERY")


class GestureTests(unittest.TestCase):
    def test_point_confirm_geometry_separate(self):
        cfg = load_config()["gesture"]
        point = {"shoulder_l": (0.4, 0.5), "shoulder_r": (0.6, 0.5),
                 "elbow_r": (0.7, 0.5), "wrist_r": (0.85, 0.5)}
        confirm = {"shoulder_l": (0.4, 0.5), "shoulder_r": (0.6, 0.5),
                   "elbow_r": (0.6, 0.35), "wrist_r": (0.6, 0.2)}
        self.assertEqual(classify_arm(point, cfg), "POINT_RIGHT")
        self.assertEqual(classify_arm(confirm, cfg), "CONFIRM_GEOMETRY")

    def test_uncalibrated_robot_rejected(self):
        from pathlib import Path
        path = Path(__file__).resolve().parents[1] / "config" / "calibration.yaml"
        with self.assertRaisesRegex(ValueError, "calibration"):
            validate_robot_config(load_config(path))

    def test_point_and_wave_require_elapsed_time(self):
        cfg = load_config()["gesture"]
        history = GestureHistory(cfg)
        pointed = Person(OWNER, 1, keypoints={"shoulder_l": (0.4, 0.5),
                                              "shoulder_r": (0.6, 0.5),
                                              "elbow_r": (0.7, 0.5),
                                              "wrist_r": (0.85, 0.5)})
        self.assertEqual(history.update(pointed, 0.0), "NONE")
        self.assertEqual(history.update(pointed, 0.2), "NONE")
        self.assertEqual(history.update(pointed, 0.4), "POINT_RIGHT")
        wave = Person(OWNER, 2, keypoints={"shoulder_l": (0.4, 0.5),
                                           "shoulder_r": (0.6, 0.5),
                                           "wrist_l": (0.4, 0.3)})
        result = "NONE"
        for index, x in enumerate((0.4, 0.6, 0.3, 0.65, 0.25)):
            wave.keypoints["wrist_l"] = (x, 0.3)
            result = history.update(wave, index*0.2)
        self.assertEqual(result, "WAVE")


if __name__ == "__main__":
    unittest.main()
