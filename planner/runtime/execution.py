from __future__ import annotations

import logging
import math
import threading
import time

from planner.domain import StopResult
from planner.errors import BudgetExceeded, ExecutionError
from planner.ports import Vehicle


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
        self._hold_checkpoint = None

    def bind_hold_checkpoint(self, checkpoint):
        self._hold_checkpoint = checkpoint

    def wait(self, seconds: float, *, sleep=time.sleep) -> None:
        """Keep owner-side control and task deadlines active during local waits."""
        deadline = self.clock()+seconds
        while self.clock() < deadline:
            self.checkpoint()
            sleep(min(.05, max(0.,deadline-self.clock())))

    def assert_owner(self):
        if threading.get_ident() != self._owner:
            raise ExecutionError("flight execution may only run on its owner thread")

    def acquire_flight(self, owner):
        self.assert_owner()
        if self._flight_owner is not None:
            raise ExecutionError("another flight session is already active")
        if self._pending_cleanup is not None and self._pending_cleanup():
            raise ExecutionError("a previous request is still running; flight handover refused")
        self._pending_cleanup, self._flight_owner = None, owner

    def release_flight(self, owner, pending_cleanup=None):
        self.assert_owner()
        if self._flight_owner is not owner:
            raise ExecutionError("cannot release another flight session")
        self.clear_deadline()
        self._pending_cleanup = pending_cleanup if pending_cleanup is not None and pending_cleanup() else None
        self._flight_owner = None

    def stop_if_owner(self, owner):
        self.assert_owner()
        return self.stop() if self._flight_owner is owner else StopResult(False)

    def set_deadline(self, deadline):
        self.assert_owner()
        if deadline is not None and (isinstance(deadline, bool) or not isinstance(deadline, (int, float)) or not math.isfinite(deadline)):
            raise ValueError("deadline must be finite")
        self._deadline = deadline
        self._stop_requested.clear()

    def clear_deadline(self):
        self.assert_owner()
        self._deadline = None
        if self._cancel_observations is not None:
            self._cancel_observations()

    def request_stop(self):
        """Thread-safe signal; only the owner sends flight commands."""
        self._stop_requested.set()

    def check_deadline(self):
        self.assert_owner()
        if self._stop_requested.is_set():
            raise ExecutionError("flight stop requested")
        if self._deadline is not None and self.clock() >= self._deadline:
            raise BudgetExceeded("stage time budget exceeded")

    def bind_observation_checkpoint(self, checkpoint, cancel):
        self._observation_checkpoint, self._cancel_observations = checkpoint, cancel

    def checkpoint(self):
        """Owner-only stop/deadline check plus the bound hold and observation checks.

        碰撞不在这里判：接触状态由 ``ControlLoop._guard`` 经 ``CollisionRecovery.poll``
        单独取，引擎的 ``collision_state()`` 是消费式的，多一个读取者就会把它抢走。
        """
        self.check_deadline()
        if self._hold_checkpoint is not None:
            self._hold_checkpoint()
        if self._observation_checkpoint is not None:
            self._observation_checkpoint()
        self.check_deadline()

    def current_pose(self):
        self.checkpoint()
        return self.vehicle.current_pose()

    def stop(self) -> StopResult:
        self.assert_owner()
        errors = []
        for action in (self.vehicle.cancel, self.vehicle.hold):
            try:
                action()
            except Exception as exc:
                errors.append(f"{action.__name__}: {exc}")
                logging.getLogger(__name__).error("Stop action failed: %s", exc)
        # Successful API calls alone do not verify physical stabilization.
        return StopResult(False, tuple(errors))