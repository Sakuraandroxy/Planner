"""Collision recovery actions for AirSim closed-loop navigation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


@dataclass
class CollisionRecoveryResult:
    attempted: bool
    pose: Optional[Sequence[float]] = None
    yaw: Optional[float] = None
    collided: bool = False


class CollisionRecovery:
    """Recover from collision by moving straight backward in the current body frame."""

    def __init__(self, enabled: bool = True, back_distance_m: float = 2.0, velocity: float = 1.0):
        self.enabled = bool(enabled)
        self.back_distance_m = max(0.0, float(back_distance_m))
        self.velocity = max(0.1, float(velocity))

    @classmethod
    def from_config(cls, cfg) -> "CollisionRecovery":
        rcfg = cfg.get("COLLISION_RECOVERY", {}) if isinstance(cfg, dict) else {}
        return cls(
            enabled=rcfg.get("ENABLED", True),
            back_distance_m=rcfg.get("BACK_DISTANCE_M", 2.0),
            velocity=rcfg.get("VELOCITY", 1.0),
        )

    def recover(self, client) -> CollisionRecoveryResult:
        if not self.enabled or self.back_distance_m <= 0:
            return CollisionRecoveryResult(attempted=False)

        print(f"  [CollisionRecovery] collision detected; back {self.back_distance_m:.1f}m")
        try:
            pose, yaw, collided = client.execute_waypoints(
                [[-self.back_distance_m, 0.0, 0.0]],
                velocity=self.velocity,
                use_forward_only=False,
            )
            print(
                f"  [CollisionRecovery] after back pose=({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}) "
                f"yaw={yaw:.1f}° collided={collided}"
            )
            return CollisionRecoveryResult(attempted=True, pose=pose, yaw=yaw, collided=collided)
        except Exception as exc:
            print(f"  [CollisionRecovery] back failed: {exc}")
            pose, yaw = client.get_pose()
            return CollisionRecoveryResult(attempted=True, pose=pose, yaw=yaw, collided=True)
