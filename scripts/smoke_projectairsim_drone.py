#!/usr/bin/env python3
"""M4 acceptance: live CityEnviron drone workflows through the adapter.

Start the simulator first (the dual scene also spawns the rover, which stays
parked), e.g.:
    scripts/launch_projectairsim_xpra.sh cityenviron :104 10010 8991 8992
Then:
    conda activate carla-vln
    python scripts/smoke_projectairsim_drone.py --bbox 0.47,0.55,0.53,0.62

The script attaches ``ProjectAirSimDrone`` to Drone1, runs the reference
readiness sequence (tolerant native takeoff + position hold), captures a
synchronized RGB/depth frame, flies a forward ``BasicMotionWorkflow`` segment,
optionally runs the visual-goal closed loop with the deterministic manual
detector and geometric planner, and lands before closing the connection.

The visual-goal run uses a local always-approve completion checker: M4
exercises the drone control loop, while the semantic model checker is wired
through ``config.task_completion`` in the production session (M5).
"""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from planner.adapters.projectairsim.connection import (  # noqa: E402
    ProjectAirSimConnection,
    ProjectAirSimConnectionConfig,
)
from planner.adapters.projectairsim.drone import (  # noqa: E402
    ProjectAirSimDrone,
)
from planner.adapters.projectairsim.moving_source import (  # noqa: E402
    BorrowedMovingSource,
)
from planner.application.factories import (  # noqa: E402
    build_detector,
    build_geometry,
    build_planner,
)
from planner.capabilities.approach import TargetApproach  # noqa: E402
from planner.capabilities.completion.visual_goal import (  # noqa: E402
    VisualGoalCompletion,
)
from planner.capabilities.targeting.bbox_goal_observation import (  # noqa: E402
    BBoxGoalObserver,
)
from planner.config import load_config  # noqa: E402
from planner.domain.basic_motion import BasicMotionParameters  # noqa: E402
from planner.domain.depth_visualization import render_depth  # noqa: E402
from planner.domain.mission import MissionStage, TaskKind  # noqa: E402
from planner.domain.visual_goal import (  # noqa: E402
    VisualGoalNavigationParameters,
)
from planner.services.execution import ExecutionService  # noqa: E402
from planner.services.motion import SynchronizedMotionPlanner  # noqa: E402
from planner.services.trajectory_validation import (  # noqa: E402
    TrajectoryValidator,
)
from planner.workflows.basic_motion import BasicMotionWorkflow  # noqa: E402
from planner.workflows.visual_goal_navigation import (  # noqa: E402
    VisualGoalNavigationWorkflow,
)
from projectairsim import Drone  # noqa: E402


class AlwaysApproveCompletion:
    """Smoke-only stand-in for the semantic completion model.

    The local trigger still decides when a check is due; M4 only needs the
    drone control loop and target approach, not another network dependency.
    """

    def check(self, request, *, timeout_s: float) -> bool:
        return True


def _floats(text: str, count: int, name: str) -> tuple[float, ...]:
    values = tuple(float(part) for part in text.split(","))
    if len(values) != count:
        raise SystemExit(f"{name} requires {count} comma-separated numbers")
    return values


def _save_observation(observation, folder: Path) -> None:
    folder.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(
        str(folder / "rgb.png"),
        cv2.cvtColor(observation.rgb, cv2.COLOR_RGB2BGR),
    )
    cv2.imwrite(
        str(folder / "depth.png"),
        cv2.cvtColor(
            render_depth(observation.depth_m, 0.5, 60.0), cv2.COLOR_RGB2BGR
        ),
    )
    np.save(folder / "depth_m.npy", observation.depth_m)


def _show_pose(label, pose) -> None:
    print(
        f"{label}: x={pose.x:.2f} y={pose.y:.2f} z={pose.z:.2f} "
        f"yaw={pose.yaw_deg:.1f}"
    )


def _build_visual_goal(config, drone, execution, motion, detector, planner):
    settings = config.visual_goal_navigation
    limits = settings.limits
    goal_observer = BBoxGoalObserver(
        detector,
        min(config.target_detection.timeout_s, limits.model_request_timeout_s),
    )
    validator = TrajectoryValidator(
        config.trajectory_planner.point_count,
        config.trajectory_planner.max_step_m,
        config.trajectory_planner.max_yaw_step_deg,
    )
    return VisualGoalNavigationWorkflow(
        drone,
        goal_observer,
        planner,
        validator,
        motion,
        execution,
        build_geometry(config),
        VisualGoalCompletion(limits, TargetApproach()),
        limits,
        completion_checker=AlwaysApproveCompletion(),
        trigger_rules=settings.completion_rules,
        sliding_window=config.sliding_window,
        motion_limits=drone.tracking_limits,
        moving_observation_source=(
            BorrowedMovingSource(drone)
            if config.sliding_window.enabled
            else None
        ),
        stream_speed_mps=config.motion.max_speed_mps,
        stream_waypoint_timeout_s=60.0,
        continuous_tracking=config.continuous,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(ROOT / "config" / "base.yaml"))
    parser.add_argument("--forward-m", type=float, default=3.0)
    parser.add_argument(
        "--bbox",
        default=None,
        help="normalized xmin,ymin,xmax,ymax; enables the visual-goal run",
    )
    parser.add_argument("--target", default="the surface inside the box")
    parser.add_argument("--instruction", default="飞向画面中的目标")
    parser.add_argument(
        "--out", default=str(ROOT / "output" / "drone_smoke")
    )
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s %(name)s: %(message)s"
    )

    config = load_config(args.config)
    pas = config.projectairsim
    connection = ProjectAirSimConnection(
        ProjectAirSimConnectionConfig(
            address=pas.address,
            port_topics=pas.port_topics,
            port_services=pas.port_services,
            timeout_s=pas.timeout_s,
            scene_file=pas.scene_file,
            sim_config_path=pas.sim_config_path,
            delay_after_load_s=pas.delay_after_load_s,
        )
    )
    print(f"connecting to {pas.address}:{pas.port_topics}/{pas.port_services}")
    connection.connect(wait_s=max(10.0, pas.timeout_s))
    drone = None
    workflow = None
    try:
        world = connection.load_scene()
        robot = Drone(connection.client, world, pas.drone_name)
        drone = ProjectAirSimDrone(
            connection,
            robot,
            camera=config.drone.camera(config.camera),
            depth=config.depth,
            camera_id=pas.front_camera,
            speed_mps=config.drone.speed_mps,
            motion_timeout_s=config.drone.move_timeout_s,
            takeoff_timeout_s=config.drone.takeoff_timeout_s,
            land_timeout_s=config.drone.land_timeout_s,
            readiness_stable_s=config.drone.readiness_stable_s,
            readiness_poll_interval_s=config.drone.readiness_poll_interval_s,
            motion=config.motion,
            tracking=config.continuous,
        )
        drone.start()
        _show_pose("armed", drone.current_pose())
        drone.prepare(takeoff_if_landed=True)
        _show_pose("airborne hold", drone.current_pose())

        observation = drone.capture(timeout_s=8.0)
        _save_observation(observation, Path(args.out))
        valid = observation.depth_m[
            np.isfinite(observation.depth_m) & (observation.depth_m < 60000.0)
        ]
        if valid.size:
            print(
                f"capture: {observation.rgb.shape} depth_m "
                f"min={float(valid.min()):.2f} "
                f"median={float(np.median(valid)):.2f} "
                f"max={float(valid.max()):.2f}"
            )
        _show_pose("capture pose", observation.pose)
        print(f"frame written to {args.out}")

        execution = ExecutionService(drone)
        motion = SynchronizedMotionPlanner(
            config.motion.max_speed_mps, drone.tracking_limits
        )
        ok = True
        if args.forward_m:
            basic = BasicMotionWorkflow(
                drone, motion, execution, limits=drone.tracking_limits
            )
            began = time.monotonic()
            result = basic.run(
                MissionStage(
                    "stage_1",
                    TaskKind.BASIC_MOTION,
                    BasicMotionParameters(
                        f"drone forward {args.forward_m:g} m",
                        forward_m=args.forward_m,
                    ),
                )
            )
            value = result.value
            print(
                f"basic forward ({time.monotonic() - began:.1f}s): "
                f"success={result.success} status={value.status.value} "
                f"reason={value.reason!r} "
                f"actual={value.actual} stop_confirmed={value.stop.confirmed}"
            )
            _show_pose("after basic", drone.current_pose())
            ok &= result.success

        if args.bbox:
            detector = build_detector(
                config, "manual", bbox=_floats(args.bbox, 4, "--bbox")
            )
            planner = build_planner(config, "geometric")
            workflow = _build_visual_goal(
                config, drone, execution, motion, detector, planner
            )
            began = time.monotonic()
            result = workflow.run(
                MissionStage(
                    "stage_2",
                    TaskKind.VISUAL_GOAL_NAVIGATION,
                    VisualGoalNavigationParameters(
                        instruction=args.instruction,
                        target_description=args.target,
                    ),
                )
            ).value
            print(
                json.dumps(
                    {
                        "status": result.status.value,
                        "reason": result.reason,
                        "planning_calls": result.planning_calls,
                        "completion_checks": result.completion_checks,
                        "final_distance_m": result.final_distance_m,
                        "path_length_m": result.path_length_m,
                        "elapsed_s": round(time.monotonic() - began, 2),
                    },
                    indent=2,
                )
            )
            _show_pose("after visual goal", drone.current_pose())
            ok &= result.status.value == "reached"

        print(f"drone smoke {'PASSED' if ok else 'FAILED'}")
        return 0 if ok else 1
    finally:
        if workflow is not None:
            try:
                workflow.close()
            except Exception as error:
                print(f"warning: workflow close failed: {error}", file=sys.stderr)
        if drone is not None:
            try:
                landed = drone.land()
                print(f"landed={landed}")
            except Exception as error:
                print(f"warning: land failed: {error}", file=sys.stderr)
            drone.close()
        connection.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"error: {error}", file=sys.stderr)
        print(
            "is CityEnviron running with -topicsport=8991 -servicesport=8992?",
            file=sys.stderr,
        )
        raise SystemExit(2)