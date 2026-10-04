"""Pose geometry, short-term association, temporal gestures, and model adapters."""
from collections import deque
import json
import math
from pathlib import Path
from .types import Person, Observation


def center(box):
    return ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    area_a, area_b = (a[2]-a[0])*(a[3]-a[1]), (b[2]-b[0])*(b[3]-b[1])
    return intersection / max(1e-9, area_a + area_b - intersection)


def match_person(anchor, people, excluded=None, max_distance=0.25):
    if anchor is None:
        return None
    best, score = None, -1e9
    for person in people:
        if person is excluded:
            continue
        distance = math.hypot(*(a-b for a, b in zip(center(anchor), center(person.bbox))))
        if distance > max_distance and iou(anchor, person.bbox) == 0:
            continue
        candidate_score = 2*iou(anchor, person.bbox) - distance
        if candidate_score > score:
            best, score = person, candidate_score
    return best


def _angle(a, b, c):
    first = (a[0]-b[0], a[1]-b[1])
    second = (c[0]-b[0], c[1]-b[1])
    cosine = sum(x*y for x, y in zip(first, second)) / max(1e-9, math.hypot(*first)*math.hypot(*second))
    return math.degrees(math.acos(max(-1, min(1, cosine))))


def classify_arm(keypoints, cfg):
    """Return POINT_LEFT/RIGHT, CONFIRM_GEOMETRY, or NONE."""
    if "shoulder_l" not in keypoints or "shoulder_r" not in keypoints:
        return "NONE"
    sw = math.hypot(*(a-b for a, b in zip(keypoints["shoulder_l"], keypoints["shoulder_r"])))
    if sw < 1e-5:
        return "NONE"
    for suffix in ("l", "r"):
        try:
            shoulder, elbow, wrist = (keypoints[name + suffix] for name in
                                      ("shoulder_", "elbow_", "wrist_"))
        except KeyError:
            continue
        dx = (wrist[0]-shoulder[0])/sw
        level = abs(wrist[1]-shoulder[1])/sw
        up = (shoulder[1]-wrist[1])/sw
        reach = math.hypot(*(a-b for a, b in zip(shoulder, wrist)))/sw
        straight = _angle(shoulder, elbow, wrist) >= cfg["elbow_straight_thr"]
        if straight and up >= cfg["up_thr"] and abs(dx) <= cfg["vertical_thr"]:
            return "CONFIRM_GEOMETRY"
        if (straight and reach >= cfg["reach_thr"] and level <= cfg["level_thr"]
                and abs(dx) >= cfg["direction_thr"]):
            return "POINT_LEFT" if dx < 0 else "POINT_RIGHT"
    return "NONE"


class GestureHistory:
    def __init__(self, cfg):
        self.cfg = cfg
        self.wrists = {}
        self.confirm_since = {}
        self.confirm_origin = {}
        self.point_since = {}
        self.point_direction = {}

    def update(self, person, now):
        identity = person.person_id
        points = person.keypoints
        geometry = classify_arm(points, self.cfg)
        if geometry == "CONFIRM_GEOMETRY":
            if identity not in self.confirm_since:
                self.confirm_since[identity] = now
                self.confirm_origin[identity] = [points.get("wrist_l"), points.get("wrist_r")]
            origins = self.confirm_origin[identity]
            current = [points.get("wrist_l"), points.get("wrist_r")]
            displacement = [math.hypot(*(x-y for x, y in zip(a, b)))
                            for a, b in zip(origins, current) if a and b]
            if displacement and max(displacement) > self.cfg.get("confirm_static_thr", 0.04):
                self.confirm_since[identity] = now
                self.confirm_origin[identity] = current
            if now-self.confirm_since[identity] >= self.cfg["confirm_hold_sec"]:
                return "CONFIRM"
        else:
            self.confirm_since.pop(identity, None)
            self.confirm_origin.pop(identity, None)
        if geometry.startswith("POINT_"):
            if self.point_direction.get(identity) != geometry:
                self.point_direction[identity] = geometry
                self.point_since[identity] = now
            if now-self.point_since[identity] >= self.cfg["pointing_window_sec"]:
                return geometry
            return "NONE"
        self.point_since.pop(identity, None)
        self.point_direction.pop(identity, None)
        if "shoulder_l" not in points or "shoulder_r" not in points:
            return "NONE"
        sw = math.hypot(*(a-b for a, b in zip(points["shoulder_l"], points["shoulder_r"])))
        for suffix in ("l", "r"):
            wrist = points.get("wrist_" + suffix)
            shoulder = points.get("shoulder_" + suffix)
            if wrist is None or shoulder is None or wrist[1] >= shoulder[1] or sw < 1e-5:
                continue
            key = (identity, suffix)
            history = self.wrists.setdefault(key, deque())
            history.append((now, wrist[0]/sw))
            while history and now-history[0][0] > self.cfg["wave_window_sec"]:
                history.popleft()
            if len(history) < 4:
                continue
            delta = [history[i][1]-history[i-1][1] for i in range(1, len(history))]
            signs = [1 if d > 0 else -1 for d in delta if abs(d) >= self.cfg["wave_jitter_thr"]]
            changes = sum(a != b for a, b in zip(signs, signs[1:]))
            xs = [item[1] for item in history]
            if (max(xs)-min(xs) >= self.cfg["wave_amplitude_thr"] and
                    changes >= self.cfg["wave_min_changes"] and
                    history[-1][0]-history[0][0] >= self.cfg["wave_min_duration_sec"]):
                return "WAVE"
        return "NONE"


class PersonAssociator:
    def __init__(self):
        self.previous = []
        self.previous_at = None
        self.next_id = 1

    def assign(self, people, now):
        if self.previous_at is not None and now-self.previous_at > 0.3:
            self.previous = []
        used = set()
        for person in people:
            candidates = [(iou(person.bbox, old.bbox), old) for old in self.previous
                          if old.person_id not in used]
            candidates.sort(key=lambda item: item[0], reverse=True)
            old = candidates[0][1] if candidates and candidates[0][0] > 0.1 else None
            if old is None:
                old = match_person(person.bbox, [p for p in self.previous if p.person_id not in used],
                                   max_distance=0.12)
            if old is None:
                person.person_id = self.next_id
                self.next_id += 1
            else:
                person.person_id = old.person_id
                used.add(old.person_id)
        if people:
            self.previous = people
            self.previous_at = now
        return people


class ObservationBuilder:
    def __init__(self, cfg):
        self.association = PersonAssociator()
        self.gestures = GestureHistory(cfg)
        self.owner_anchor = None
        self.receiver_anchor = None
        self.previous = None

    def build(self, people, blocked, now, frame_timestamp, fps=0.0):
        people = self.association.assign(people, now)
        for person in people:
            person.gesture = self.gestures.update(person, now)
            person.point_side = person.gesture[6:] if person.gesture.startswith("POINT_") else "NONE"
        waves = [p for p in people if p.gesture == "WAVE"]
        owner = match_person(self.owner_anchor, people) if self.owner_anchor else None
        if owner is None and len(waves) == 1 and self.owner_anchor is None:
            owner = waves[0]
        receiver = match_person(self.receiver_anchor, people) if self.receiver_anchor else None
        candidate = next((p for p in people if p is not owner and p is not receiver), None)
        obs = Observation(now, frame_timestamp, people=people, blocked=blocked, fps=fps,
                          wave_count=len(waves), owner_visible_raw=owner is not None,
                          owner_bbox=owner.bbox if owner else None,
                          owner_gesture=owner.gesture if owner else "NONE",
                          gesture_raw=owner.gesture if owner else "NONE",
                          point_side=owner.point_side if owner else "NONE",
                          owner_pointing_raw=bool(owner and owner.point_side != "NONE"),
                          candidate_visible=candidate is not None,
                          candidate_bbox=candidate.bbox if candidate else None,
                          receiver_visible=receiver is not None,
                          receiver_bbox=receiver.bbox if receiver else None)
        # One-frame pose misses retain visibility; control still stops on stale camera frames.
        obs.owner_visible = obs.owner_visible_raw or bool(self.previous and self.previous.owner_visible
                                                         and now-self.previous.timestamp < 0.3)
        obs.receiver_visible = obs.receiver_visible or bool(self.previous and self.previous.receiver_visible
                                                            and now-self.previous.timestamp < 0.3)
        obs.owner_pointing = obs.owner_pointing_raw or bool(self.previous and self.previous.owner_pointing
                                                           and now-self.previous.timestamp < 0.3)
        obs.gesture = obs.gesture_raw if obs.gesture_raw != "NONE" else (
            self.previous.gesture if self.previous and now-self.previous.timestamp < 0.3 else "NONE")
        self.previous = obs
        return obs


class TRTPOSEBackend:
    """TRTModule trt_pose adapter; validate tensor shapes on the actual Jetson."""
    def __init__(self, config):
        if not config.get("model_path") or not config.get("human_pose_json"):
            raise RuntimeError("Pose weights and local human_pose.json are required")
        path = Path(config["model_path"])
        topology = Path(config["human_pose_json"])
        if not path.is_file() or not topology.is_file():
            raise RuntimeError("trt_pose weights and local human_pose.json are required")
        try:
            import torch
            from torch2trt import TRTModule
            from trt_pose.coco import coco_category_to_topology
            from trt_pose.parse_objects import ParseObjects
        except ImportError as exc:
            raise RuntimeError("Install trt_pose/torch2trt matching JetPack/TensorRT/PyTorch") from exc
        with topology.open("r", encoding="utf-8") as stream:
            descriptor = json.load(stream)
        self.names = descriptor["keypoints"]
        self.topology = coco_category_to_topology(descriptor)
        self.parser = ParseObjects(self.topology)
        self.torch = torch
        self.input_width = int(config["input_width"])
        self.input_height = int(config["input_height"])
        self.model = TRTModule()
        self.model.load_state_dict(torch.load(str(path), map_location="cpu"))
        self.model.eval().cuda()

    def infer(self, frame):
        import cv2
        rgb = cv2.cvtColor(cv2.resize(frame, (self.input_width, self.input_height)), cv2.COLOR_BGR2RGB)
        image = self.torch.from_numpy(rgb).float().permute(2, 0, 1).unsqueeze(0).cuda() / 255.0
        mean = self.torch.tensor([0.485, 0.456, 0.406], device="cuda").view(1, 3, 1, 1)
        std = self.torch.tensor([0.229, 0.224, 0.225], device="cuda").view(1, 3, 1, 1)
        with self.torch.no_grad():
            cmap, paf = self.model((image-mean)/std)
        counts, objects, peaks = self.parser(cmap, paf)
        people = []
        for person_index in range(int(counts[0])):
            points = {}
            for key_index, name in enumerate(self.names):
                peak_index = int(objects[0, person_index, key_index])
                if peak_index >= 0:
                    y, x = peaks[0, key_index, peak_index]
                    canonical = name
                    for joint in ("shoulder", "elbow", "wrist"):
                        canonical = canonical.replace("left_" + joint, joint + "_l")
                        canonical = canonical.replace("right_" + joint, joint + "_r")
                    points[canonical] = (float(x), float(y))
            if len(points) >= 4:
                xs, ys = zip(*points.values())
                people.append(Person((max(0, min(xs)), max(0, min(ys)),
                                      min(1, max(xs)), min(1, max(ys))), keypoints=points))
        return people


class CollisionModelBackend:
    def __init__(self, config):
        if not config.get("model_path"):
            raise RuntimeError("Collision model weights are required")
        path = Path(config["model_path"])
        if not path.is_file():
            raise RuntimeError("Collision model missing; collect/retrain and supply weights")
        import torch
        self.torch = torch
        self.model = torch.jit.load(str(path), map_location="cuda").eval()
        self.roi = config["roi"]
        self.threshold = config["threshold"]

    def infer(self, frame):
        import cv2
        height, width = frame.shape[:2]
        x1, y1, x2, y2 = self.roi
        crop = frame[int(y1*height):int(y2*height), int(x1*width):int(x2*width)]
        if crop.size == 0:
            raise RuntimeError("Empty collision ROI")
        rgb = cv2.cvtColor(cv2.resize(crop, (224, 224)), cv2.COLOR_BGR2RGB)
        tensor = self.torch.from_numpy(rgb).float().permute(2, 0, 1).unsqueeze(0).cuda()/255.0
        with self.torch.no_grad():
            output = self.model(tensor)
        score = float(self.torch.sigmoid(output.reshape(-1)[0]).item())
        return score >= self.threshold, score
