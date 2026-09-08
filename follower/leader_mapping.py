"""Follower-side relative mapping for network leader-arm control."""
from __future__ import annotations

class RelativeLeaderMapper:
    """Map leader movement relative to the follower's engage-time pose.

    Releasing, losing a fresh stream, or changing source resets the anchor. The
    next good enabled sample therefore starts from the follower's actual pose
    instead of replaying movement accumulated while the network was absent.
    """

    def __init__(self, joint_names: list[str] | tuple[str, ...]) -> None:
        self.joint_names = tuple(joint_names)
        self._leader_previous: dict[str, float] | None = None
        self._follower_start: dict[str, float] | None = None
        self._displacement = {joint: 0.0 for joint in self.joint_names}
        self._source_id: str | None = None

    def reset(self) -> None:
        self._leader_previous = None
        self._follower_start = None
        self._displacement = {joint: 0.0 for joint in self.joint_names}

    def targets(
        self,
        leader_now: dict[str, float],
        follower_now: dict[str, float],
        *,
        source_id: str,
    ) -> dict[str, float]:
        if source_id != self._source_id:
            self.reset()
            self._source_id = source_id
        if self._leader_previous is None:
            self._leader_previous = dict(leader_now)
            self._follower_start = dict(follower_now)

        assert self._follower_start is not None
        targets: dict[str, float] = {}
        for joint in self.joint_names:
            step = leader_now[joint] - self._leader_previous[joint]
            if joint != "gripper":
                step = (step + 180.0) % 360.0 - 180.0
            self._displacement[joint] += step
            targets[joint] = self._follower_start[joint] + self._displacement[joint]

        self._leader_previous = dict(leader_now)
        return targets
