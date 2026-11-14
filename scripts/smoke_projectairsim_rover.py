#!/usr/bin/env python3
"""M3 acceptance: live CityEnviron rover workflows through the adapter.

Start the simulator first, e.g.:
    scripts/launch_projectairsim_xpra.sh cityenviron :104 10010 8991 8992
Then:
    conda activate carla-vln
    python scripts/smoke_projectairsim_rover.py

The script loads the dual scene, attaches a ``ProjectAirSimRover`` to Rover1,
runs the shared ``BasicMotionWorkflow`` (forward and reverse segments) through
the calibrated drive model, optionally runs continuous ``free_navigation`` for
a fixed wall-clock window (then emulates ``POST /api/stop``) and restores the
spawn pose.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from planner.adapters.projectairsim.connection import (  # noqa: E402
    ProjectAirSimConnection,
    ProjectAirSimConnectionConfig,
)
from planner.adapters.projectairsim.rover import ProjectAirSimRover  # noqa: E402
from planner.application.factories import (  # noqa: E402
    build_free_navigation_planner,
)
from planner.config import load_config  # noqa: E402
from planner.domain.basic_motion import BasicMotionParameters  # noqa: E402
from planner.domain.free_navigation import FreeNavigationParameters  # noqa: E402
from planner.domain.mission import MissionStage, TaskKind  # noqa: E402
from planner.services.execution import ExecutionService  # noqa: E402
from planner.services.motion import SynchronizedMotionPlanner  # noqa: E402
from planner.services.trajectory_validation import TrajectoryValidator  # noqa: E402
from planner.workflows.basic_motion import BasicMotionWorkflow  # noqa: E402
from planner.workflows.free_navigation import FreeNavigationWorkflow  # noqa: E402
from projectairsim import Rover  # noqa: E402


def print_basic(label, result, rover):
    value = result.value
    pose = rover.current_pose()

    def show(item):
        return (
            "None"
            if item is None
            else f"({item.x:.2f},{item.y:.2f},{item.yaw_deg:.1f})"
        )

    print(
        f"{label}: success={result.success} status={value.status.value} "
        f"reason={value.reason!r}"
    )
    print(
        f"  target={show(value.target)} actual={show(value.actual)} "
        f"pose=({pose.x:.2f},{pose.y:.2f},{pose.yaw_deg:.1f}) "
        f"stop_confirmed={value.stop.confirmed}"
    )


def run_basic(workflow, rover, parameters, label):
    stage = MissionStage("stage_1", TaskKind.BASIC_MOTION, parameters)
    began = time.monotonic()
    result = workflow.run(stage)
    print_basic(f"{label} ({time.monotonic() - began:.1f}s)", result, rover)
    return result.success


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(ROOT / "config" / "base.yaml"))
    parser.add_argument("--forward-m", type=float, default=5.0)
    parser.add_argument("--reverse-m", type=float, default=2.0)
    parser.add_argument("--free-navigation-seconds", type=float, default=0.0)
    parser.add_argument(
        "--instruction",
        default="沿着道路一直前进，不要碰到障碍物",
    )
    parser.add_argument("--no-reset", action="store_true")
    args = parser.parse_args()

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
    rover = None
    free_workflow = None
    try:
        world = connection.load_scene()
        robot = Rover(connection.client, world, pas.rover_name)
        rover = ProjectAirSimRover(
            connection,
            robot,
            camera=config.camera,
            depth=config.depth,
            camera_id=pas.front_camera,
        )
        rover.start()
        print(f"rover armed at {rover.current_pose()}")
        execution = ExecutionService(rover)
        motion = SynchronizedMotionPlanner(
            config.motion.max_speed_mps, rover.tracking_limits
        )
        basic = BasicMotionWorkflow(
            rover, motion, execution, limits=rover.tracking_limits
        )
        ok = True
        if args.forward_m:
            ok &= run_basic(
                basic,
                rover,
                BasicMotionParameters(
                    f"forward {args.forward_m:g} m",
                    forward_m=args.forward_m,
                ),
                "basic forward",
            )
        if args.reverse_m:
            ok &= run_basic(
                basic,
                rover,
                BasicMotionParameters(
                    f"reverse {args.reverse_m:g} m",
                    forward_m=-args.reverse_m,
                ),
                "basic reverse",
            )
        print(f"basic motion chain {'PASSED' if ok else 'FAILED'}")

        if args.free_navigation_seconds > 0:
            planner = build_free_navigation_planner(config)
            if planner is None:
                raise RuntimeError(
                    "trajectory_planner.url/model are not configured"
                )
            validator = TrajectoryValidator(
                config.trajectory_planner.point_count,
                config.trajectory_planner.max_step_m,
                config.trajectory_planner.max_yaw_step_deg,
            )
            free_workflow = FreeNavigationWorkflow(
                rover,
                planner,
                validator,
                motion,
                execution,
                model_request_timeout_s=config.trajectory_planner.timeout_s,
                sliding_window=config.sliding_window,
                motion_limits=rover.tracking_limits,
                moving_observation_source=rover,
                stream_speed_mps=config.motion.max_speed_mps,
                stream_waypoint_timeout_s=60.0,
                collision_backoff_m=config.free_navigation.collision_backoff_m,
                maximum_collision_recoveries=(
                    config.free_navigation.maximum_collision_recoveries
                ),
                collision_recovery_reset_m=(
                    config.free_navigation.collision_recovery_reset_m
                ),
                continuous_tracking=config.continuous,
                lane_reference_provider=None,
            )
            free_workflow.prepare()
            stop_event = threading.Event()

            def hold():
                if stop_event.is_set():
                    raise KeyboardInterrupt

            execution.bind_hold_checkpoint(hold)
            timer = threading.Timer(
                args.free_navigation_seconds, stop_event.set
            )
            timer.daemon = True
            timer.start()
            stage = MissionStage(
                "stage_1",
                TaskKind.FREE_NAVIGATION,
                FreeNavigationParameters(args.instruction),
            )
            print(
                f"free navigation for ~{args.free_navigation_seconds:g}s "
                f"(simulating POST /api/stop)"
            )
            began = time.monotonic()
            try:
                result = free_workflow.run(stage)
            except KeyboardInterrupt:
                print(
                    f"cancelled: task was cancelled "
                    f"(after {time.monotonic() - began:.1f}s)"
                )
            else:
                value = result.value
                print(
                    f"free navigation ended: status={value.status.value} "
                    f"reason={value.reason!r}"
                )
            finally:
                timer.cancel()
                execution.bind_hold_checkpoint(None)
            print(
                f"  planning_calls={free_workflow.continuous.planning_calls} "
                f"path_length={free_workflow.continuous.path_length_m:.1f} m "
                f"final_pose={free_workflow.continuous.final_pose}"
            )
            print(
                f"  collision_recoveries="
                f"{free_workflow.continuous.collision_recoveries} "
                f"last_collision={rover.last_collision}"
            )
        final_pose = rover.current_pose()
        print(
            f"final pose x={final_pose.x:.2f} y={final_pose.y:.2f} "
            f"z={final_pose.z:.2f} yaw={final_pose.yaw_deg:.1f}"
        )
        if not args.no_reset:
            rover.return_home()
            print(f"return_home -> {rover.current_pose()}")
        return 0
    finally:
        if free_workflow is not None:
            try:
                free_workflow.close()
            except Exception:
                pass
        if rover is not None:
            rover.close()
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