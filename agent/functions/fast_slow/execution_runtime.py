"""Queue execution and path synchronization helpers for the fast-slow runtime.

The main runtime owns stage orchestration and completion decisions.  This
module only handles the mechanics of executing direct actions, consuming the
world waypoint queue, and keeping ``ContinuousPathStream`` synchronized with
the live AirSim pose.  Runtime-specific collision and target-reorientation
policies are supplied as callbacks so this module does not depend on
``runtime.py``.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, TYPE_CHECKING

from config import cfg

from agent.functions.common.config_access import function_section
from agent.functions.fast_slow.above_runtime import (
    _minimum_airsim_climb_command_m,
    _minimum_airsim_vertical_command_m,
    _normalize_direct_vertical_action_m,
)
from agent.functions.fast_slow.completion_pipeline import CompletionPipeline
from agent.functions.fast_slow.planning_runtime import _format_waypoints

if TYPE_CHECKING:
    from agent.functions.fast_slow.runtime_context import RuntimeObjects
else:
    RuntimeObjects = Any


def _execute_action_stage(client, stage) -> None:
    """Execute a parser-produced direct action without touching the planner."""
    act = stage.action
    val = stage.value or 0
    if act in ("up", "down"):
        requested = float(val or 0.0)
        vertical_config = function_section(cfg, "MEMORY")
        val = _normalize_direct_vertical_action_m(
            requested,
            vertical_config,
            climb=(act == "up"),
        )
        if abs(val - abs(requested)) > 1e-6:
            minimum = (
                _minimum_airsim_climb_command_m(vertical_config)
                if act == "up"
                else _minimum_airsim_vertical_command_m(vertical_config)
            )
            print(
                "  [Action] AirSim vertical displacement quantized "
                f"requested={requested:.2f}m executed={val:.2f}m "
                f"minimum={minimum:.2f}m direction={act}"
            )
    print(f"  [Action] direct execution: {act} {val}")
    pos_before, yaw_before = client.get_pose()
    try:
        if act in ("left", "right"):
            sign = 1 if act == "right" else -1
            client.rotate_to_yaw(yaw_before + sign * val)
        elif act == "forward":
            rad = math.radians(yaw_before)
            client.move_to_position(
                pos_before[0] + val * math.cos(rad),
                pos_before[1] + val * math.sin(rad),
                pos_before[2],
            )
        elif act == "backward":
            rad = math.radians(yaw_before)
            client.move_to_position(
                pos_before[0] - val * math.cos(rad),
                pos_before[1] - val * math.sin(rad),
                pos_before[2],
            )
        elif act == "up":
            client.move_to_position(pos_before[0], pos_before[1], pos_before[2] - val)
        elif act == "down":
            client.move_to_position(pos_before[0], pos_before[1], pos_before[2] + val)
        elif act == "land":
            # Landing is a physical action and cannot be replaced by memory.
            client.land()
    except Exception as exc:
        print(f"  [Action] error: {exc}")
    pos_after, yaw_after = client.get_pose()
    print(
        f"  [Action] from: ({pos_before[0]:.1f}, {pos_before[1]:.1f}, {pos_before[2]:.1f}) "
        f"yaw={yaw_before:.1f}deg"
    )
    print(
        f"           to:   ({pos_after[0]:.1f}, {pos_after[1]:.1f}, {pos_after[2]:.1f}) "
        f"yaw={yaw_after:.1f}deg"
    )


def _execute_one_from_queue(objects: RuntimeObjects, client, state, count: int | None = None) -> bool:
    """Execute one fast batch from the world queue (legacy synchronous path)."""
    queue_len = len(objects.controller.queue.world_waypoints)
    requested = objects.controller.execute_count if count is None else int(count)
    n = min(max(1, requested), queue_len)
    n = max(1, n)
    exec_waypoints = [list(wp) for wp in objects.controller.queue.world_waypoints[:n]]
    if not exec_waypoints:
        return False
    if all(all(abs(v) < 1e-6 for v in wp) for wp in exec_waypoints):
        objects.controller.mark_executed(len(exec_waypoints))
        return False

    print(
        f"  [FastExec] execute={len(exec_waypoints)} "
        f"queue_before={len(objects.controller.queue.world_waypoints)} "
        f"world_wp={exec_waypoints}"
    )
    state.update(status="executing")
    t0 = time.perf_counter()
    pos_final, yaw_final, collided = client.execute_waypoints(exec_waypoints)
    elapsed = time.perf_counter() - t0
    state.update(pose=pos_final, yaw=yaw_final, collided=collided)
    if collided:
        objects.controller.queue.clear()
        recovery = objects.collision_recovery.recover(client)
        print(f"  [Collision] collided=True recovery_attempted={recovery.attempted}")
    else:
        objects.controller.mark_executed(len(exec_waypoints))
        settle = float(function_section(cfg, "FAST_SLOW").get("EXECUTION_SETTLE_S", 0.0))
        if settle > 0:
            time.sleep(settle)
    print(
        f"  [FastExec] done time={elapsed:.2f}s "
        f"queue_after={len(objects.controller.queue.world_waypoints)}"
    )
    return True


def _needs_planning_snapshot(objects: RuntimeObjects) -> bool:
    return (
        len(objects.controller.queue.world_waypoints) == 0
        and objects.controller.can_submit_plan()
    )


def _clear_stopped_queue(objects, path_stream, state) -> None:
    """Stop active AirSim motion and clear planner state exposed to the UI."""
    emergency_stop = getattr(path_stream, "emergency_stop", None)
    if callable(emergency_stop):
        emergency_stop()
    else:
        path_stream.stop()
    objects.controller.clear()
    if objects.completion_pipeline is not None:
        objects.completion_pipeline.clear()
    state.update(
        trajectory_queue=[],
        qwen_waypoints=[],
        trajectory_candidates=[],
        selected_trajectory={},
    )


def _drop_path_points_behind_vehicle(objects, current_pos, current_yaw_deg) -> int:
    """Remove horizontally passed queue-head points behind the vehicle.

    Pure vertical commands intentionally keep nearly the same XY position as
    the vehicle.  Tiny pose/planning round-off must not discard a climb before
    it reaches AirSim.
    """
    waypoints = objects.controller.queue.world_waypoints
    if not waypoints:
        return 0

    fast_slow_cfg = {
        **(cfg.get("FAST_SLOW", {}) or {}),
        **function_section(cfg, "FAST_SLOW"),
    }
    vertical_xy_tolerance_m = max(
        0.01,
        float(fast_slow_cfg.get("PATH_VERTICAL_XY_TOLERANCE_M", 0.15)),
    )
    yaw = math.radians(float(current_yaw_deg))
    forward_x = math.cos(yaw)
    forward_y = math.sin(yaw)
    dropped = 0
    while waypoints:
        point = waypoints[0]
        dx = float(point[0]) - float(current_pos[0])
        dy = float(point[1]) - float(current_pos[1])
        if math.hypot(dx, dy) <= vertical_xy_tolerance_m:
            break
        forward_projection = dx * forward_x + dy * forward_y
        if forward_projection >= 0.0:
            break
        waypoints.pop(0)
        dropped += 1
    if dropped:
        print(
            f"  [PathProgress] dropped_behind={dropped} "
            f"pose=({float(current_pos[0]):.2f},{float(current_pos[1]):.2f},{float(current_pos[2]):.2f}) "
            f"yaw={float(current_yaw_deg):.1f}"
        )
    return dropped


def _continuous_path_velocity(objects, current_pos) -> float:
    """Avoid near-hover speed while preserving planner response time."""
    nominal = float(cfg.get("SIM", {}).get("AIRSIM_VELOCITY", 2.0))
    if not objects.controller.planning:
        return nominal
    waypoints = objects.controller.queue.world_waypoints
    if not waypoints:
        return nominal
    previous = [float(current_pos[0]), float(current_pos[1]), float(current_pos[2])]
    path_length = 0.0
    for waypoint in waypoints:
        point = [float(waypoint[0]), float(waypoint[1]), float(waypoint[2])]
        path_length += math.sqrt(sum((point[i] - previous[i]) ** 2 for i in range(3)))
        previous = point
    reserve_fn = getattr(objects.controller, "required_planning_reserve_s", None)
    reserve_time = max(
        float(reserve_fn()) if callable(reserve_fn) else float(objects.controller.reserve_time_s),
        1e-6,
    )
    fast_slow_cfg = function_section(cfg, "FAST_SLOW")
    minimum = max(0.0, float(fast_slow_cfg.get("CONTINUOUS_MIN_SPEED_MPS", 1.0)))
    minimum = min(minimum, nominal)
    return max(minimum, min(nominal, path_length / reserve_time))


def _sync_path_if_ready(
    objects,
    path_stream,
    client,
    state,
    stage=None,
    *,
    record_collision: Callable | None = None,
    reorient_if_behind: Callable | None = None,
    debug_print: Callable | None = None,
    drop_behind: Callable | None = None,
    velocity_fn: Callable | None = None,
    format_fn: Callable | None = None,
) -> int:
    """Reconcile queue progress and issue/extend the active world path.

    ``record_collision`` and ``reorient_if_behind`` are runtime policies. They
    are callbacks to keep target identity and completion logic out of this
    execution module.
    """
    current_pos, current_yaw = client.get_pose()
    progress = path_stream.poll(objects.controller.queue.world_waypoints, current_pos)
    if progress.consumed > 0:
        objects.controller.mark_executed(progress.consumed)
        print(
            f"  [FlightPose] world=({current_pos[0]:.2f},{current_pos[1]:.2f},{current_pos[2]:.2f}) "
            f"yaw={current_yaw:.1f} consumed={progress.consumed} "
            f"remaining={len(objects.controller.queue.world_waypoints)}"
        )
        state.update(
            pose=current_pos,
            yaw=current_yaw,
            trajectory_queue=[list(wp) for wp in objects.controller.queue.world_waypoints],
        )
        if debug_print is not None:
            debug_print(
                f"  [PathProgress] consumed={progress.consumed} before_reissue "
                f"remaining={len(objects.controller.queue.world_waypoints)}"
            )
    if progress.collided:
        contact = (
            record_collision(objects, stage, current_pos, current_yaw)
            if record_collision is not None and stage is not None
            else None
        )
        objects.controller.clear()
        recovery = objects.collision_recovery.recover(client)
        print(f"  [Collision] before_path_reissue=True recovery_attempted={recovery.attempted}")
        if contact is not None:
            print(
                "  [TargetSurfaceContact] collision matched locked facade "
                f"range={contact['contact_range_m']:.2f}m memory_distance={contact['distance_m']:.2f}m"
            )
        state.update(collided=True, trajectory_queue=[])
        return progress.consumed

    if reorient_if_behind is not None and reorient_if_behind(
        objects,
        client,
        path_stream,
        state,
        stage,
        allow_queued=True,
    ):
        current_pos, current_yaw = client.get_pose()
    drop_fn = drop_behind or _drop_path_points_behind_vehicle
    velocity_callback = velocity_fn or _continuous_path_velocity
    format_callback = format_fn or _format_waypoints
    dropped_behind = drop_fn(objects, current_pos, current_yaw)
    waypoints = objects.controller.queue.world_waypoints
    if not waypoints:
        return progress.consumed + dropped_behind
    was_active = path_stream.active
    velocity = velocity_callback(objects, current_pos)
    issued = path_stream.sync(waypoints, current_pos, velocity)
    if issued:
        state.update(status="executing")
        mode = "extended" if was_active else "started"
        print(
            f"  [WorldPath] {mode} velocity={velocity:.2f}m/s "
            f"world_wp={format_callback(waypoints, limit=20)}"
        )
        if not was_active:
            print(f"  [Flight] started remaining={len(waypoints)} velocity={velocity:.1f}m/s")
    return progress.consumed + dropped_behind


class CompletionRadiusWatchdog:
    """Keep the completion circle monitored while RPC/model calls block."""

    def __init__(
        self,
        objects,
        client,
        path_stream,
        stage,
        trigger_radius_m: float,
        interval_s: float,
        *,
        stage_key_fn: Callable | None = None,
        distance_fn: Callable | None = None,
        trigger_fn: Callable | None = None,
    ):
        self.objects = objects
        self.client = client
        self.path_stream = path_stream
        self.stage = stage
        self._stage_key_fn = stage_key_fn or CompletionPipeline.stage_key
        self._distance_fn = distance_fn
        self._trigger_fn = trigger_fn
        self.stage_key = self._stage_key_fn(stage)
        self.trigger_radius_m = float(trigger_radius_m)
        self.interval_s = max(0.02, float(interval_s))
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="completion-radius-watchdog",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=max(0.1, self.interval_s * 3.0))

    def _run(self) -> None:
        while not self._stop_event.wait(self.interval_s):
            try:
                current_stage = self.objects.task_manager.current_stage()
                if self._stage_key_fn(current_stage) != self.stage_key:
                    return
                controller = getattr(self.objects, "controller", None)
                queue = getattr(controller, "queue", None)
                if not (
                    self.path_stream.active
                    or bool(getattr(queue, "world_waypoints", []) or [])
                    or bool(getattr(controller, "planning", False))
                    or bool(getattr(controller, "has_plan_job", False))
                ):
                    continue
                current_world, _yaw = self.client.get_pose()
                if self._distance_fn is None or self._trigger_fn is None:
                    continue
                estimated = self._distance_fn(self.objects, self.stage, current_world)
                if not self._trigger_fn(
                    self.objects,
                    self.stage,
                    estimated,
                    self.trigger_radius_m,
                ):
                    continue
                emergency_stop = getattr(self.path_stream, "emergency_stop", None)
                if callable(emergency_stop):
                    emergency_stop()
                else:
                    self.path_stream.stop()
                if controller is not None:
                    controller.clear()
                print("  [CompletionWatchdog] live pose entered target radius; path stopped")
                return
            except Exception:
                # A concurrent AirSim pose RPC may fail during image capture;
                # retry on the next tick without changing navigation state.
                continue


__all__ = [
    "CompletionRadiusWatchdog",
    "_clear_stopped_queue",
    "_continuous_path_velocity",
    "_drop_path_points_behind_vehicle",
    "_execute_action_stage",
    "_execute_one_from_queue",
    "_needs_planning_snapshot",
    "_sync_path_if_ready",
]
