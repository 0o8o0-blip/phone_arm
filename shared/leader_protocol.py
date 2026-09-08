"""Wire protocol shared by the leader controller and follower gateway."""
from __future__ import annotations

import math
from typing import Any


LEADER_MESSAGE_TYPE = "leader_joints_v1"
LEADER_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


def parse_leader_positions(data: dict[str, Any]) -> dict[str, float] | None:
    """Return a complete, finite leader position map or reject the message."""
    if data.get("type") != LEADER_MESSAGE_TYPE:
        return None
    raw = data.get("joints")
    if not isinstance(raw, dict):
        return None
    out: dict[str, float] = {}
    for joint in LEADER_JOINTS:
        try:
            value = float(raw[joint])
        except (KeyError, TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        out[joint] = value
    return out
