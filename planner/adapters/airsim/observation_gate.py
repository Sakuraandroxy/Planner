"""协调观测与飞行，保证正式采集期间不继续发送运动指令"""
from contextlib import contextmanager
import math
import threading
import time

from planner.domain.pose import wrap_yaw_deg
from planner.errors import ExecutionError


class ObservationGate:
    def __init__(self, vehicle, limits, timeout_s, clock=time.monotonic, sleep=time.sleep):
        self.vehicle = vehicle
        self.limits = limits
        self.timeout_s = timeout_s
        self.clock = clock
        self.sleep = sleep
        self._condition = threading.Condition(threading.RLock())
        self._generation = 0
        self._failed = False

    @contextmanager
    def capture(self):
        with self._condition:
            try:
                self.vehicle.hover()
                self._wait_stable()
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
        # Yield outside the lock so waiting capture threads can acquire it.
        time.sleep(0)
        with self._condition:
            if self._failed:
                raise ExecutionError("observation failed; motion remains stopped (restart runtime to retry)")
            yield self._generation

    def _wait_stable(self):
        started = previous_time = self.clock()
        previous = self.vehicle.current_pose()
        stable_since = None
        while self.clock() - started < self.timeout_s:
            self.sleep(1 / self.limits.control_hz)
            now = self.clock()
            current = self.vehicle.current_pose()
            if self.vehicle.collision_state():
                raise ExecutionError("collision while stopping for observation")
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
