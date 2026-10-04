from dataclasses import dataclass, field
from typing import List, Optional, Tuple

BBox = Tuple[float, float, float, float]  # normalized x1,y1,x2,y2


@dataclass
class Person:
    bbox: BBox
    person_id: Optional[int] = None
    # Named normalized keypoints: shoulder_l, elbow_l, wrist_l, etc.
    keypoints: dict = field(default_factory=dict)
    gesture: str = "NONE"
    point_side: str = "NONE"


@dataclass
class Observation:
    timestamp: float
    frame_timestamp: float
    people: List[Person] = field(default_factory=list)
    owner_visible_raw: bool = False
    owner_visible: bool = False
    owner_bbox: Optional[BBox] = None
    owner_pointing_raw: bool = False
    owner_pointing: bool = False
    point_side: str = "NONE"
    gesture_raw: str = "NONE"
    gesture: str = "NONE"
    candidate_visible: bool = False
    candidate_bbox: Optional[BBox] = None
    receiver_visible: bool = False
    receiver_bbox: Optional[BBox] = None
    blocked: bool = False
    collision_confidence: float = 0.0
    fps: float = 0.0
    wave_count: int = 0
    owner_gesture: str = "NONE"


@dataclass
class Motion:
    left: float = 0.0
    right: float = 0.0
    name: str = "STOP"

