from __future__ import annotations
import time
import logging
import math
import threading
from dataclasses import replace
from typing import Callable

from planner.domain.execution import ExecutionBudget, ExecutionReceipt, ExecutionRecord, StopResult

from planner.domain.trajectory import MotionTrajectory, RelativeTrajectory
from planner.domain.pose import WorldPose, RelativePoseDelta, resolve_yaw_delta_deg, wrap_yaw_deg
from planner.errors import BudgetExceeded, ExecutionError, MotionPaused
from planner.ports.vehicle import Vehicle
from planner.ports.collision_check import CollisionCheck


class ExecutionService:
    def __init__(self, vehicle: Vehicle, clock=time.monotonic):
        self.vehicle = vehicle
        self.clock = clock
        self._owner = threading.get_ident()
        self._flight_owner = None
        self._pending_cleanup = None
        self._deadline = None
        self._stop_requested = threading.Event()
        self._observation_checkpoint = self._cancel_observations = None
        self._executing = False
        self._collision_check: CollisionCheck | None = None
        self._collision_seen = False

    def _assert_owner(self):
        if threading.get_ident() != self._owner:
            raise ExecutionError("flight execution may only run on its owner thread")

    def acquire_flight(self, owner, *, collision_check: CollisionCheck | None = None):
        self._assert_owner()
        if self._flight_owner is not None:
            raise ExecutionError("another flight session is already active")
        if self._pending_cleanup is not None and self._pending_cleanup():
            raise ExecutionError("a previous request is still running; flight handover refused")
        if collision_check is not None and not callable(collision_check):
            raise ValueError("collision_check must be callable or None")
        self._pending_cleanup, self._flight_owner = None, owner
        self._collision_check, self._collision_seen = collision_check, False

    def release_flight(self, owner, pending_cleanup=None):
        self._assert_owner()
        if self._flight_owner is not owner:
            raise ExecutionError("cannot release another flight session")
        self.clear_deadline()
        self._pending_cleanup = pending_cleanup if pending_cleanup is not None and pending_cleanup() else None
        self._flight_owner = None
        self._collision_check, self._collision_seen = None, False

    def stop_if_owner(self, owner):
        self._assert_owner()
        return self.stop() if self._flight_owner is owner else StopResult(False)

    def set_deadline(self, deadline):
        self._assert_owner()
        if deadline is not None and (isinstance(deadline, bool) or not isinstance(deadline, (int, float)) or not math.isfinite(deadline)):
            raise ValueError("deadline must be finite")
        self._deadline = deadline
        self._stop_requested.clear()

    def clear_deadline(self):
        self._assert_owner()
        self._deadline = None
        if self._cancel_observations is not None:
            self._cancel_observations()

    def request_stop(self):
        """Thread-safe signal; only the owner sends flight commands."""
        self._stop_requested.set()

    def check_deadline(self):
        self._assert_owner()
        if self._stop_requested.is_set():
            raise ExecutionError("flight stop requested")
        if self._deadline is not None and self.clock() >= self._deadline:
            raise BudgetExceeded("stage time budget exceeded")

    def bind_observation_checkpoint(self, checkpoint, cancel):
        self._observation_checkpoint, self._cancel_observations = checkpoint, cancel

    def check_safety(self):
        """Owner-only, non-reentrant check usable during observation and motion."""
        if self.collision_state():
            raise ExecutionError("collision detected by enabled workflow collision check")

    def checkpoint(self):
        self.check_safety()
        if self._observation_checkpoint is not None:
            self._observation_checkpoint()
        self.check_safety()

    def execute(self, trajectory: MotionTrajectory) -> ExecutionRecord:
        self.checkpoint()
        start = self.vehicle.current_pose()
        receipt = self.execute_with_receipt(trajectory)
        return ExecutionRecord(start, receipt.final_pose, RelativeTrajectory(receipt.measured_actions), receipt.paused)

    def execute_with_receipt(self, trajectory: MotionTrajectory, *, budget: ExecutionBudget | None = None,
                             pause_when: Callable[[WorldPose], bool] | None = None) -> ExecutionReceipt:
        self._assert_owner()
        if self._executing:
            raise ExecutionError("nested motion execution is not allowed")
        if not trajectory.segments:
            raise ExecutionError("cannot execute an empty motion trajectory")
        self._executing = True
        began = self.clock()
        completed = 0
        measured_chords = 0.0
        actions = []
        active = None
        previous = None
        observed_yaw = None
        measurement_start_yaw = None
        try:
            self.checkpoint()
            previous = self.vehicle.current_pose()
            for index, segment in enumerate(trajectory.segments):
                self.checkpoint()
                if budget is not None and budget.remaining(self.clock()) <= 0:
                    raise BudgetExceeded("deadline before segment")
                # Tracker yaw is continuous in the original segment's angle
                # branch, which can differ by 360 degrees from measured pose.
                measurement_start_yaw = segment.start.yaw_deg + resolve_yaw_delta_deg(
                    segment.start.yaw_deg, previous.yaw_deg)
                observed_yaw = measurement_start_yaw
                active = segment

                def checkpoint(pose, unwrapped_yaw):
                    nonlocal observed_yaw
                    observed_yaw = unwrapped_yaw
                    self.check_safety()
                    return pause_when(pose) if pause_when is not None else False

                try:
                    self.vehicle.execute_segment(segment, budget=budget, pose_checkpoint=checkpoint)
                except MotionPaused as paused:
                    stop = self.stop()
                    if stop.errors:
                        raise ExecutionError("failed to cancel/hover after execution pause")
                    current = self.vehicle.current_pose()
                    if self.collision_state():
                        raise ExecutionError("collision while pausing execution")
                    actual_yaw = paused.unwrapped_yaw_deg + wrap_yaw_deg(current.yaw_deg-paused.pose.yaw_deg)
                    actions.append(self._measured_action(previous, current, actual_yaw-measurement_start_yaw))
                    measured_chords += math.dist((previous.x, previous.y, previous.z), (current.x, current.y, current.z))
                    # Preserve the ORIGINAL world endpoint and signed yaw goal.
                    remaining = self._remaining(current, actual_yaw, segment, trajectory.segments[index+1:])
                    return ExecutionReceipt(completed, current, self.clock()-began, measured_chords,
                                            True, remaining, tuple(actions))
                completed += 1
                current = self.vehicle.current_pose()
                yaw = observed_yaw + wrap_yaw_deg(current.yaw_deg-observed_yaw) - measurement_start_yaw
                actions.append(self._measured_action(previous, current, yaw))
                measured_chords += math.dist(
                    (previous.x, previous.y, previous.z), (current.x, current.y, current.z)
                )
                previous = current
                active = None
                self.checkpoint()
                if self.collision_state():
                    raise ExecutionError("AirSim reported a collision")
                if budget is not None and budget.remaining(self.clock()) <= 0:
                    raise BudgetExceeded("deadline after segment")
            return ExecutionReceipt(completed, self.vehicle.current_pose(), self.clock()-began, measured_chords,
                                    measured_actions=tuple(actions))
        except (Exception, KeyboardInterrupt) as exc:
            stop = self.stop()
            try:
                current = self.vehicle.current_pose()
                if active is not None and previous is not None:
                    actual_yaw = observed_yaw + wrap_yaw_deg(current.yaw_deg-observed_yaw)
                    actions.append(self._measured_action(previous, current, actual_yaw-measurement_start_yaw))
                    measured_chords += math.dist((previous.x, previous.y, previous.z), (current.x, current.y, current.z))
                    remaining = self._remaining(current, actual_yaw, active, trajectory.segments[completed+1:])
                else:
                    remaining = MotionTrajectory(trajectory.segments[completed:])
                receipt = ExecutionReceipt(completed, current, self.clock()-began, measured_chords,
                                           remaining=remaining, measured_actions=tuple(actions))
            except Exception:
                receipt = None
            if isinstance(exc, (ExecutionError, KeyboardInterrupt)):
                exc.receipt, exc.stop_result = receipt, stop
                raise
            wrapped = ExecutionError(str(exc))
            wrapped.receipt, wrapped.stop_result = receipt, stop
            raise wrapped from exc
        finally:
            self._executing = False

    @staticmethod
    def _remaining(current, actual_yaw, segment, tail):
        original_yaw = segment.start.yaw_deg + resolve_yaw_delta_deg(
            segment.start.yaw_deg, segment.end.yaw_deg, segment.yaw_delta_deg)
        remaining_segment = replace(segment, start=current, yaw_delta_deg=original_yaw-actual_yaw)
        return MotionTrajectory((remaining_segment, *tail))

    @staticmethod
    def _measured_action(start, end, yaw):
        angle = math.radians(start.yaw_deg)
        x, y = end.x-start.x, end.y-start.y
        return RelativePoseDelta(math.cos(angle)*x+math.sin(angle)*y,
                                 -math.sin(angle)*x+math.cos(angle)*y, end.z-start.z, yaw)

    def hold(self) -> None:
        self._assert_owner()
        self.vehicle.hover()

    def current_pose(self):
        self.checkpoint()
        return self.vehicle.current_pose()

    def collision_state(self):
        """Whether the active optional check blocks flight, not proof of no contact.

        Disabled sessions perform no read. Enabled sessions latch the first event
        until release, including when the simulator consumes events on read.
        """
        self.check_deadline()
        if self._collision_check is None:
            return False
        if not self._collision_seen:
            detected = self._collision_check()
            if not isinstance(detected, bool):
                raise ExecutionError("collision check must return a boolean")
            self._collision_seen = detected
        return self._collision_seen

    def stop(self) -> StopResult:
        self._assert_owner()
        errors = []
        for action in (self.vehicle.cancel, self.vehicle.hover):
            try:
                action()
            except Exception as exc:
                errors.append(f"{action.__name__}: {exc}")
                logging.getLogger(__name__).error("Stop action failed: %s", exc)
        # Successful API calls alone do not verify physical stabilization.
        return StopResult(False, tuple(errors))

