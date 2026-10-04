"""Frozen HRI protocol. Transitions produce desired motion, never raw motor calls."""
from .control import stop, visual_servo, pulse_turn, commit_forward, avoid_step
from .perception import center, match_person, iou


class StateMachine:
    def __init__(self, config):
        self.cfg = config
        self.state = "SEARCH_OWNER"
        self.entered = 0.0
        self.transition = None
        self.abort_reason = None
        self.owner_anchor = None
        self.owner_id = None
        self.decision_owner_anchor = None
        self.decision_owner_id = None
        self.candidate_anchor = None
        self.candidate_id = None
        self.receiver_anchor = None
        self.receiver_id = None
        self.scan_side = "NONE"
        self.scan_phase = "TURN"
        self.avoid_phase = "TURN"
        self.avoid_attempts = 0
        self.avoid_started = 0.0
        self.resume_state = None
        self.last_point_at = None

    def _go(self, state, now, reason=""):
        old = self.state
        self.state, self.entered = state, now
        self.transition = {"from": old, "to": state, "reason": reason}
        if state == "PULSE_SCAN":
            self.scan_phase = "TURN"
        if state == "AVOID":
            self.avoid_phase = "TURN"
            self.avoid_attempts = 0
            self.avoid_started = now
        if state == "ABORT":
            self.abort_reason = reason

    def manual_reset(self, now):
        self.__init__(self.cfg)
        self.entered = now

    def _person(self, identity, anchor, obs, excluded=None):
        if identity is not None:
            return next((p for p in obs.people if p.person_id == identity and p is not excluded), None)
        return match_person(anchor, obs.people, excluded=excluded) if anchor else None

    def _owner(self, obs):
        return self._person(self.owner_id, self.owner_anchor, obs)

    def _candidate(self, obs):
        return self._person(self.candidate_id, self.candidate_anchor, obs,
                            excluded=self._owner(obs))

    def step(self, obs):
        self.transition = None
        now = obs.timestamp
        elapsed = now-self.entered
        s = self.state
        speed = self.cfg["safety"]["max_forward_speed"]
        turn = self.cfg["safety"]["max_turn_speed"]
        if s in ("ABORT", "COMPLETE", "ARRIVED", "AVOID_TIMEOUT"):
            if s == "ARRIVED":
                self._go("COMPLETE", now)
            elif s == "AVOID_TIMEOUT":
                self._go("ABORT", now, "AVOID_TIMEOUT")
            return stop()
        if s == "SEARCH_OWNER":
            if obs.wave_count > 1:
                self._go("AMBIGUOUS_CALLER", now)
            elif obs.wave_count == 1:
                caller = next((p for p in obs.people if p.gesture == "WAVE"), None)
                self.owner_anchor = caller.bbox if caller else obs.owner_bbox
                self.owner_id = caller.person_id if caller else None
                if self.owner_anchor:
                    self._go("APPROACH_OWNER", now)
            return stop()
        if s == "AMBIGUOUS_CALLER":
            if obs.wave_count <= 1:
                self._go("SEARCH_OWNER", now)
            return stop()
        if s == "APPROACH_OWNER":
            if obs.blocked:
                self.resume_state = s
                self._go("AVOID", now, "BLOCKED")
                return stop()
            owner = self._owner(obs)
            if owner:
                self.owner_anchor = owner.bbox
            box = owner.bbox if owner else obs.owner_bbox
            if box is None:
                if not obs.owner_visible:
                    self._go("OWNER_LOST", now)
                return stop()
            if owner is None:
                return stop()
            if box[3]-box[1] >= self.cfg["approach"]["owner_stop_bbox_ratio"]:
                self._go("WAIT_LOAD", now)
                return stop()
            return visual_servo(box, speed, self.cfg["approach"]["servo_gain"],
                                self.cfg["approach"]["owner_stop_bbox_ratio"])
        if s == "WAIT_LOAD":
            owner = self._owner(obs)
            if owner and owner.gesture == "NONE" and elapsed >= 0.5:
                self.owner_anchor = owner.bbox
                self._go("POSE_STABLE", now)
            return stop()
        if s == "POSE_STABLE":
            if elapsed >= self.cfg["gesture"]["pointing_window_sec"]:
                self._go("AIMING", now)
            return stop()
        if s == "AIMING":
            owner = self._owner(obs)
            if owner is None:
                if not obs.owner_visible:
                    self._go("OWNER_LOST", now)
            elif owner.point_side in ("LEFT", "RIGHT"):
                x = center(owner.bbox)[0]
                limits = self.cfg["scan"]
                if not limits["owner_fov_min_x"] <= x <= limits["owner_fov_max_x"]:
                    self._go("OWNER_AT_FOV_LIMIT", now)
                else:
                    self.owner_anchor = owner.bbox
                    self.scan_side = owner.point_side
                    self._go("PULSE_SCAN", now)
            return stop()
        if s == "PULSE_SCAN":
            if obs.blocked:
                self._go("SCAN_BLOCKED", now)
                return stop()
            owner = self._owner(obs)
            if owner is None:
                if not obs.owner_visible:
                    self._go("OWNER_LOST", now)
                return stop()
            if owner.point_side != self.scan_side:
                self._go("AIMING", now)
                return stop()
            if not (self.cfg["scan"]["owner_fov_min_x"] <= center(owner.bbox)[0] <=
                    self.cfg["scan"]["owner_fov_max_x"]):
                self._go("OWNER_AT_FOV_LIMIT", now)
                return stop()
            if self.scan_phase == "TURN":
                if elapsed < self.cfg["scan"]["pulse_duration_sec"]:
                    return pulse_turn(self.scan_side, turn)
                self.scan_phase, self.entered = "SETTLE", now
                return stop()
            if elapsed < self.cfg["scan"]["settle_sec"]:
                return stop()
            candidates = [p for p in obs.people if p is not owner and
                          self.cfg["scan"]["candidate_min_x"] <= center(p.bbox)[0] <=
                          self.cfg["scan"]["candidate_max_x"]]
            if len(candidates) > 1:
                return stop()
            if candidates:
                self.decision_owner_anchor = owner.bbox
                self.decision_owner_id = owner.person_id
                self.candidate_anchor = candidates[0].bbox
                self.candidate_id = candidates[0].person_id
                self._go("WAIT_OWNER_DECISION", now)
            else:
                self.scan_phase, self.entered = "TURN", now
            return stop()
        if s == "WAIT_OWNER_DECISION":
            owner = self._person(self.decision_owner_id, self.decision_owner_anchor, obs)
            candidate = self._candidate(obs)
            if owner is candidate:
                owner = None
            if owner and owner.gesture == "CONFIRM":
                self.receiver_anchor = self.candidate_anchor
                self.receiver_id = self.candidate_id
                self._go("LOCK_RECEIVER", now)
                return stop()
            if elapsed >= self.cfg["decision"]["timeout_sec"]:
                self._go("AIMING", now, "DECISION_TIMEOUT")
                return stop()
            if owner and owner.point_side == self.scan_side:
                self.last_point_at = now
            if (elapsed >= self.cfg["decision"]["min_dwell_sec"] and
                    self.last_point_at is not None and
                    now-self.last_point_at <= self.cfg["decision"]["gesture_gap_grace_sec"]):
                self._go("PULSE_SCAN", now, "CANDIDATE_REJECTED")
            return stop()
        if s == "LOCK_RECEIVER":
            self._go("DELIVERY", now)
            return stop()
        if s == "DELIVERY":
            if obs.blocked:
                self.resume_state = s
                self._go("AVOID", now, "BLOCKED")
                return stop()
            receiver = self._person(self.receiver_id, self.receiver_anchor, obs,
                                    excluded=self._owner(obs))
            box = receiver.bbox if receiver else obs.receiver_bbox
            if box is None:
                if not obs.receiver_visible:
                    self._go("WAIT_RECEIVER_WAVE", now)
                return stop()
            if receiver is None:
                return stop()
            self.receiver_anchor = box
            if box[3]-box[1] >= self.cfg["delivery"]["receiver_stop_bbox_ratio"]:
                self._go("ARRIVED", now)
                return stop()
            return visual_servo(box, speed, self.cfg["delivery"]["servo_gain"],
                                self.cfg["delivery"]["receiver_stop_bbox_ratio"])
        if s == "WAIT_RECEIVER_WAVE":
            owner = self._owner(obs)
            waves = [p for p in obs.people if p is not owner and p.gesture == "WAVE"]
            if len(waves) > 1:
                self._go("AMBIGUOUS_RECEIVER", now)
            elif len(waves) == 1:
                self.receiver_anchor = waves[0].bbox
                self.receiver_id = waves[0].person_id
                self._go("DELIVERY", now)
            return stop()
        if s == "AMBIGUOUS_RECEIVER":
            owner = self._owner(obs)
            waves = [p for p in obs.people if p is not owner and p.gesture == "WAVE"]
            if len(waves) <= 1:
                self._go("WAIT_RECEIVER_WAVE", now)
            return stop()
        if s == "AVOID":
            cfg = self.cfg["avoid"]
            if now-self.avoid_started >= cfg["timeout_sec"] or self.avoid_attempts >= cfg["max_attempts"]:
                self._go("AVOID_TIMEOUT", now)
                return stop()
            if self.avoid_phase == "TURN":
                if elapsed < cfg["turn_pulse_sec"]:
                    return avoid_step(cfg["turn_side"], turn)
                self.avoid_phase, self.entered = "LOOK", now
                return stop()
            if self.avoid_phase == "LOOK":
                if elapsed < cfg["settle_sec"]:
                    return stop()
                if obs.blocked:
                    self.avoid_attempts += 1
                    self.avoid_phase, self.entered = "TURN", now
                    return stop()
                self.avoid_phase, self.entered = "COMMIT", now
            if self.avoid_phase == "COMMIT":
                if obs.blocked:
                    self.avoid_attempts += 1
                    self.avoid_phase, self.entered = "TURN", now
                    return stop()
                if elapsed < cfg["commit_forward_sec"]:
                    return commit_forward(speed)
                self._go(self.resume_state or "AIMING", now, "AVOID_COMPLETE")
                return stop()
        if s == "OWNER_LOST":
            owner = self._owner(obs)
            if owner:
                self.owner_anchor = owner.bbox
                self._go("AIMING", now)
            elif elapsed >= self.cfg["recovery"]["owner_lost_sec"]:
                self._go("SEARCH_OWNER", now)
            return stop()
        if s == "OWNER_AT_FOV_LIMIT":
            owner = self._owner(obs)
            if owner and owner.point_side in ("LEFT", "RIGHT") and (
                    self.cfg["scan"]["owner_fov_min_x"] <= center(owner.bbox)[0] <=
                    self.cfg["scan"]["owner_fov_max_x"]):
                self._go("AIMING", now)
            elif elapsed >= self.cfg["recovery"]["fov_limit_sec"]:
                self._go("ABORT", now, "OWNER_AT_FOV_LIMIT")
            return stop()
        if s == "SCAN_BLOCKED":
            if not obs.blocked or elapsed >= self.cfg["recovery"]["scan_blocked_sec"]:
                self._go("AIMING", now)
            return stop()
        self._go("ABORT", now, "UNKNOWN_STATE")
        return stop()
