"""暂停轨迹执行并保持固定位置；正式采集期间仅发送保持指令。"""
from contextlib import contextmanager
from concurrent.futures import Future
from collections import deque
import math
import threading
import time

from planner.domain.pose import wrap_yaw_deg
from planner.domain.execution import ExecutionBudget
from planner.errors import ExecutionError, BudgetExceeded


class ObservationGate:
    def __init__(self, vehicle, limits, timeout_s, clock=time.monotonic, sleep=time.sleep,
                 *, owner_thread_id=None, check_safety=None):
        self.vehicle = vehicle
        self.limits = limits
        self.timeout_s = timeout_s
        self.clock = clock
        self.sleep = sleep
        self._condition = threading.Condition(threading.RLock())
        self._generation = 0
        self._failed = False
        self._owner = owner_thread_id
        self._check_safety = check_safety
        self._requests = deque()
        self._request_lock = threading.Lock()

    def _assert_owner(self):
        if self._owner is not None and threading.get_ident() != self._owner:
            raise ExecutionError("formal observation must run on the control owner; use request_capture()")

    def request_capture(self, operation, timeout_s=None):
        """Enqueue only. The caller must not wait while holding any flight lock."""
        future = Future()
        timeout = self.timeout_s if timeout_s is None else timeout_s
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("observation request timeout must be finite and positive")
        with self._request_lock:
            if len(self._requests) >= 1:
                raise ExecutionError("an observation request is already pending")
            self._requests.append((future, operation, self.clock() + timeout))
        if self._owner is not None and threading.get_ident() == self._owner:
            self.service_requests()
        return future

    def cancel_requests(self):
        with self._request_lock:
            while self._requests:
                self._requests.popleft()[0].cancel()

    def service_requests(self):
        self._assert_owner()
        with self._request_lock:
            request = self._requests.popleft() if self._requests else None
        if request is None:
            return
        future, operation, deadline = request
        if not future.set_running_or_notify_cancel():
            return
        if self.clock() >= deadline:
            future.set_exception(ExecutionError("observation request expired before its checkpoint"))
            return
        try:
            with self.capture(budget=ExecutionBudget(deadline)):
                result = operation()
                if self.clock() >= deadline:
                    raise ExecutionError("observation request timed out during capture")
            future.set_result(result)
        except BaseException as exc:
            future.set_exception(exc)
            raise

    @contextmanager
    def capture(self, *, budget=None):
        self._assert_owner()
        with self._condition:
            try:
                if self._failed:
                    raise ExecutionError("observation failed; restart runtime before retrying")
                if self._check_safety:
                    self._check_safety()
                begin_hold = getattr(self.vehicle, "begin_observation_hold", None)
                if callable(begin_hold):
                    begin_hold()
                else:
                    self.vehicle.hover()
                self.wait_for_observation(budget)
                yield
            except BaseException:
                self._failed = True
                raise
            finally:
                self._generation += 1
                self._condition.notify_all()

    @property
    def generation(self):
        return self._generation

    @contextmanager
    def step(self):
        self._assert_owner()
        self.service_requests()
        # Legacy unowned gates also support lock-serialized callers.
        time.sleep(0)
        with self._condition:
            if self._failed:
                raise ExecutionError("observation failed; motion remains stopped (restart runtime to retry)")
            yield self._generation

    def wait_for_observation(self, budget=None):
        self._assert_owner()
        wait_observation = getattr(self.vehicle, "wait_for_observation", None)
        if callable(wait_observation):
            wait_observation(budget=budget, checkpoint=self._check_safety)
            return
        started = previous_time = self.clock()
        previous = self.vehicle.current_pose()
        stable_since = None
        while self.clock() - started < self.timeout_s:
            if budget is not None and budget.remaining(self.clock()) <= 0:
                raise BudgetExceeded("deadline while waiting for stopped observation")
            if self._check_safety:
                self._check_safety()
            self.sleep(1 / self.limits.control_hz)
            now = self.clock()
            current = self.vehicle.current_pose()
            if self._check_safety:
                self._check_safety()
            dt = now - previous_time
            speed = math.dist((previous.x, previous.y, previous.z),
                              (current.x, current.y, current.z)) / dt if dt > 0 else math.inf
            rate = abs(wrap_yaw_deg(current.yaw_deg - previous.yaw_deg)) / dt if dt > 0 else math.inf
            if speed <= self.limits.stopped_speed_mps and rate <= self.limits.stopped_yaw_rate_deg_s:
                stable_since = now if stable_since is None else stable_since
                if now - stable_since >= self.limits.settle_time_s:
                    return
            else:
                stable_since = None
            previous, previous_time = current, now
        raise ExecutionError("timed out waiting for stable observation")

    def keep_holding(self):
        self._assert_owner()
        if self._check_safety:
            self._check_safety()
        tick = getattr(self.vehicle, 'hold_tick', None)
        if callable(tick):
            tick()
