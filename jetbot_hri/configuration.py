from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parent.parent
REQUIRED_ROBOT = {
    "battery.i2c_address", "safety.max_forward_speed", "safety.max_turn_speed",
    "gesture.wave_window_sec", "gesture.pointing_window_sec",
    "gesture.confirm_hold_sec", "gesture.elbow_straight_thr",
    "gesture.reach_thr", "gesture.level_thr", "gesture.direction_thr",
    "gesture.up_thr", "gesture.vertical_thr", "scan.pulse_duration_sec",
    "scan.settle_sec", "scan.owner_fov_min_x", "scan.owner_fov_max_x",
    "scan.candidate_min_x", "scan.candidate_max_x",
    "approach.owner_stop_bbox_ratio", "delivery.receiver_stop_bbox_ratio",
    "avoid.max_attempts", "avoid.turn_pulse_sec", "avoid.commit_forward_sec",
    "collision.roi", "collision.model_path", "pose.model_path",
}


def load_config(path=None):
    source = Path(path) if path else ROOT / "config" / "default.yaml"
    with source.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Config must be a YAML mapping")
    return config


def validate_robot_config(config):
    missing = []
    for key in sorted(REQUIRED_ROBOT):
        value = config
        for component in key.split("."):
            value = value.get(component) if isinstance(value, dict) else None
        if value is None or value == "TBD" or value == "":
            missing.append(key)
    def find_tbd(value, prefix=""):
        if isinstance(value, dict):
            for name, item in value.items():
                find_tbd(item, prefix + "." + name if prefix else name)
        elif value == "TBD":
            missing.append(prefix)
    find_tbd(config)
    missing = sorted(set(missing))
    if missing:
        raise ValueError("Robot calibration required: " + ", ".join(missing))
