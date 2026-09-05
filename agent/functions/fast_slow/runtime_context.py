"""Runtime dependency container and construction for the fast-slow loop.

This module owns object wiring only. Flight decisions and state transitions remain in
``runtime.py`` so moving this code does not change navigation behavior.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Any

from config import cfg

from agent.functions.common.config_access import function_section
from agent.functions.common.task_manager import TaskManager
from agent.functions.completion import (
    NavigationMetricsTracker,
    build_distance_arrival_completion,
    build_task_completion_checker,
)
from agent.functions.debug import TargetSnapshotRecorder
from agent.functions.direction import build_direction_estimator
from agent.functions.distance_estimation import build_distance_estimator
from agent.functions.fast_slow.completion_pipeline import CompletionPipeline
from agent.functions.fast_slow.controller import FastSlowController
from agent.functions.memory import MissionMemory, build_mission_memory
from agent.functions.obstacle_avoidance import DepthObstacleAvoider, build_depth_obstacle_avoider
from agent.functions.perception import TargetBearingTracker
from agent.functions.planning.sliding_window_planning import SlidingWindowPlanningFunction
from agent.functions.recovery.collision_recovery import CollisionRecovery
from agent.functions.relocalization import TargetLostRecoveryState, TargetRelocalizer
from agent.models.detection import build_detector
from agent.models.world_model import build_world_model


@dataclass
class RuntimeObjects:
    """Long-lived services and mutable run state shared by the main loop."""

    detector: Any
    direction_estimator: Any
    completion_checker: Any
    planner: SlidingWindowPlanningFunction
    controller: FastSlowController
    task_manager: TaskManager
    relocalizer: TargetRelocalizer
    collision_recovery: CollisionRecovery
    detect_executor: ThreadPoolExecutor
    slow_executor: ThreadPoolExecutor
    future_detect_executor: ThreadPoolExecutor
    future_scan_executor: ThreadPoolExecutor
    distance_estimator: Any
    arrival_completion: Any
    navigation_metrics: NavigationMetricsTracker
    mission_memory: MissionMemory
    obstacle_avoider: DepthObstacleAvoider
    target_bearing_tracker: TargetBearingTracker
    target_lost_recovery: TargetLostRecoveryState | None = None
    target_snapshot_recorder: TargetSnapshotRecorder | None = None
    world_model: Any = None
    completion_pipeline: CompletionPipeline | None = None
    display_step: int = 0
    display_max_steps: int = 0
    task_failed: bool = False
    completion_attempts: dict[tuple, int] = field(default_factory=dict)
    completion_retry_after: dict[tuple, float] = field(default_factory=dict)
    future_scan_index: int = 0
    last_future_scan_s: float = 0.0
    future_scan_job: Any = None
    above_stage_states: dict[tuple, Any] = field(default_factory=dict)
    stage_generations: dict[tuple, int] = field(default_factory=dict)
    lock_generations: dict[tuple, int] = field(default_factory=dict)


def build_runtime_objects(
    target_snapshot_recorder: TargetSnapshotRecorder | None = None,
) -> RuntimeObjects:
    """Build the dependency graph used by one fast-slow runtime invocation."""
    detector = build_detector()
    direction_estimator = build_direction_estimator()
    completion_checker = build_task_completion_checker(
        detector=detector,
        direction_estimator=direction_estimator,
    )
    planner_cfg = {
        **(cfg.get("SLIDING_WINDOW", {}) or {}),
        **function_section(cfg, "PLANNING"),
    }
    fast_slow_cfg = {
        **(cfg.get("FAST_SLOW", {}) or {}),
        **function_section(cfg, "FAST_SLOW"),
    }
    relocalization_cfg = {
        **cfg,
        "RELOCALIZATION": {
            **(cfg.get("RELOCALIZATION", {}) or {}),
            **function_section(cfg, "RELOCALIZATION"),
        },
    }
    collision_cfg = {
        **cfg,
        "COLLISION_RECOVERY": {
            **(cfg.get("COLLISION_RECOVERY", {}) or {}),
            **function_section(cfg, "COLLISION_RECOVERY"),
        },
    }
    planner = SlidingWindowPlanningFunction(planner_cfg)
    controller = FastSlowController(fast_slow_cfg)
    detect_executor = ThreadPoolExecutor(
        max_workers=int(fast_slow_cfg.get("DETECT_WORKERS", 2)),
        thread_name_prefix="fast_slow_detect",
    )
    slow_executor = ThreadPoolExecutor(
        max_workers=int(fast_slow_cfg.get("SLOW_WORKERS", 2)),
        thread_name_prefix="fast_slow_slow",
    )
    # Keep future-target work isolated from current-stage detector workers.
    future_detect_executor = ThreadPoolExecutor(
        max_workers=max(1, int(fast_slow_cfg.get("FUTURE_DETECT_WORKERS", 1))),
        thread_name_prefix="future_memory_detect",
    )
    future_scan_executor = ThreadPoolExecutor(
        max_workers=1,
        thread_name_prefix="future_memory_scan",
    )
    arrival_completion = build_distance_arrival_completion(cfg)
    mission_memory = build_mission_memory()
    return RuntimeObjects(
        detector=detector,
        direction_estimator=direction_estimator,
        completion_checker=completion_checker,
        planner=planner,
        controller=controller,
        task_manager=TaskManager(enabled=True),
        relocalizer=TargetRelocalizer(relocalization_cfg, detector=detector),
        collision_recovery=CollisionRecovery.from_config(collision_cfg),
        detect_executor=detect_executor,
        slow_executor=slow_executor,
        future_detect_executor=future_detect_executor,
        future_scan_executor=future_scan_executor,
        distance_estimator=build_distance_estimator(),
        arrival_completion=arrival_completion,
        navigation_metrics=NavigationMetricsTracker(arrival_completion.arrival_radius_m),
        mission_memory=mission_memory,
        obstacle_avoider=build_depth_obstacle_avoider(),
        target_bearing_tracker=TargetBearingTracker(
            mission_memory.config,
            cfg.get("SIM", {}) or {},
        ),
        target_lost_recovery=TargetLostRecoveryState(
            relocalization_cfg.get("RELOCALIZATION", {})
        ),
        target_snapshot_recorder=target_snapshot_recorder,
        world_model=build_world_model(),
    )
