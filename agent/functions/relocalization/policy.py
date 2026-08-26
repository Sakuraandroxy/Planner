"""State and hard budgets for target-loss recovery.

The policy deliberately separates large structures from compact targets.
Buildings continue from locked geometry (and, for ``above``, down-view roof
acquisition); only compact targets are allowed to rotate in place.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence


def _horizontal_distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


@dataclass
class RelocalizationSession:
    session_id: str
    stage_key: tuple
    instance_id: str
    start_position: list[float]
    start_yaw_deg: float
    preferred_yaw_deg: float
    started_at: float = field(default_factory=time.perf_counter)
    searched_yaws_deg: list[float] = field(default_factory=list)
    total_rotation_deg: float = 0.0
    global_search: bool = False
    result: str = "active"
    cooldown_until: float = 0.0


@dataclass
class LargeTargetBlindRecord:
    stage_key: tuple
    instance_id: str
    start_position: list[float]
    started_at: float = field(default_factory=time.perf_counter)
    last_reason: str = ""


class TargetLostRecoveryState:
    """Keep recovery attempts bounded across asynchronous scheduler loops."""

    def __init__(self, config: Optional[dict] = None):
        self.config = dict(config or {})
        self.sessions: dict[tuple, RelocalizationSession] = {}
        self.large_blind_records: dict[tuple, LargeTargetBlindRecord] = {}
        self._session_counter = 0

    def reset(self, stage_key: Optional[tuple] = None) -> None:
        if stage_key is None:
            self.sessions.clear()
            self.large_blind_records.clear()
            return
        key = tuple(stage_key)
        self.sessions.pop(key, None)
        self.large_blind_records.pop(key, None)

    def mark_observed(self, stage_key: tuple) -> None:
        """A validated observation ends both blind-flight and search state."""
        key = tuple(stage_key)
        self.large_blind_records.pop(key, None)
        session = self.sessions.get(key)
        if session is not None:
            session.result = "observed"
            session.cooldown_until = time.perf_counter() + self.cooldown_s

    @property
    def cooldown_s(self) -> float:
        return max(0.0, float(self.config.get("RELOCALIZATION_COOLDOWN_S", 4.0)))

    def large_loss_budget(
        self,
        *,
        stage_key: tuple,
        instance_id: str,
        current_position: Sequence[float],
        reason: str,
        expected_offscreen: bool,
    ) -> dict:
        """Return a no-rotation decision for one large-structure RGB miss."""
        key = tuple(stage_key)
        if expected_offscreen:
            # Expected facade disappearance is a phase change, not a recovery
            # failure. Do not accumulate a blind budget while down depth owns
            # roof acquisition or the vehicle is already over the footprint.
            self.large_blind_records.pop(key, None)
            return {
                "continue": True,
                "expected_offscreen": True,
                "elapsed_s": 0.0,
                "distance_m": 0.0,
            }

        now = time.perf_counter()
        record = self.large_blind_records.get(key)
        if record is None or record.instance_id != str(instance_id or ""):
            record = LargeTargetBlindRecord(
                stage_key=key,
                instance_id=str(instance_id or ""),
                start_position=[float(v) for v in current_position[:3]],
                started_at=now,
                last_reason=str(reason or ""),
            )
            self.large_blind_records[key] = record
        record.last_reason = str(reason or "")
        elapsed = max(0.0, now - record.started_at)
        distance = _horizontal_distance(record.start_position, current_position)
        max_time = max(1.0, float(self.config.get("LARGE_BLIND_MAX_TIME_S", 30.0)))
        max_distance = max(1.0, float(self.config.get("LARGE_BLIND_MAX_DISTANCE_M", 45.0)))
        return {
            "continue": elapsed <= max_time and distance <= max_distance,
            "expected_offscreen": False,
            "elapsed_s": elapsed,
            "distance_m": distance,
            "max_time_s": max_time,
            "max_distance_m": max_distance,
        }

    def begin_small_session(
        self,
        *,
        stage_key: tuple,
        instance_id: str,
        current_position: Sequence[float],
        current_yaw_deg: float,
        preferred_yaw_deg: Optional[float],
        reliable_memory: bool,
    ) -> tuple[Optional[RelocalizationSession], str]:
        key = tuple(stage_key)
        now = time.perf_counter()
        previous = self.sessions.get(key)
        min_move = max(0.0, float(self.config.get("SMALL_RETRY_MIN_MOVE_M", 3.0)))
        if previous is not None:
            moved = _horizontal_distance(previous.start_position, current_position)
            if previous.result in {"success", "observed"} and now < previous.cooldown_until:
                return None, "relocalization cooldown active"
            if previous.result == "failed" and moved < min_move:
                return None, f"search already exhausted at this position; move {min_move:.1f}m before retry"

        self._session_counter += 1
        preferred = (
            float(preferred_yaw_deg)
            if preferred_yaw_deg is not None
            else float(current_yaw_deg)
        )
        session = RelocalizationSession(
            session_id=f"relocalize-{self._session_counter}",
            stage_key=key,
            instance_id=str(instance_id or ""),
            start_position=[float(v) for v in current_position[:3]],
            start_yaw_deg=float(current_yaw_deg),
            preferred_yaw_deg=preferred,
            global_search=not bool(reliable_memory),
        )
        self.sessions[key] = session
        return session, "session started"

    def search_offsets(self, session: RelocalizationSession) -> list[float]:
        local = self.config.get("SMALL_LOCAL_OFFSETS_DEG", [0.0, -15.0, -30.0, 15.0, 30.0])
        offsets = [float(value) for value in list(local or [])]
        if session.global_search:
            global_offsets = self.config.get("SMALL_GLOBAL_OFFSETS_DEG", [90.0, 180.0, -90.0])
            offsets.extend(float(value) for value in list(global_offsets or []))
        # Preserve configured order while removing aliases such as -180/180.
        unique: list[float] = []
        normalized_seen: set[float] = set()
        for offset in offsets:
            normalized = round((offset + 180.0) % 360.0 - 180.0, 3)
            if normalized in normalized_seen:
                continue
            normalized_seen.add(normalized)
            unique.append(offset)
        return unique

    def finish_small_session(
        self,
        session: RelocalizationSession,
        *,
        found: bool,
        searched_yaws_deg: Sequence[float],
        total_rotation_deg: float,
    ) -> None:
        session.searched_yaws_deg = [float(v) for v in searched_yaws_deg]
        session.total_rotation_deg = float(total_rotation_deg)
        session.result = "success" if found else "failed"
        if found:
            session.cooldown_until = time.perf_counter() + self.cooldown_s
