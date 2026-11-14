"""Bounded read-only capture pump. It never receives a vehicle controller."""
from functools import partial

from planner.services.background_operation import BackgroundOperation
from planner.errors import ObservationError
from planner.ports.observation_source import MovingObservationSource


class MovingObservations:
    def __init__(self, source: MovingObservationSource, clock, timeout_s, maximum_failures=3):
        self.source, self.clock, self.timeout_s = source, clock, timeout_s
        self.job = BackgroundOperation(clock)
        self.latest = None
        self.failures = 0
        self.retry_at = 0.0
        self.maximum_failures = maximum_failures

    def begin(self):
        if not self.job.busy:
            self.job.discard()
        self.job.require_available()
        self.latest, self.failures, self.retry_at = None, 0, 0.0

    def poll(self):
        try:
            frame = self.job.poll()
        except Exception as exc:
            self.failures += 1
            self.retry_at = self.clock()+1.0
            if self.failures >= self.maximum_failures or self.job.busy:
                raise ObservationError(f"moving observation unavailable: {exc}") from exc
            return None
        if frame is not None:
            self.latest, self.failures = frame, 0
        return frame

    def request(self, budget):
        if not self.job.busy and self.job.future is None and self.clock() >= self.retry_at:
            self.job.start(partial(self.source.capture_moving, budget=budget),
                           min(self.timeout_s, budget.remaining(self.clock())))

    def discard(self):
        self.latest = None
        self.job.discard()

    def close(self):
        self.job.close(self.source.close)
