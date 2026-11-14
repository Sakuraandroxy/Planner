"""Pure refill policy: waypoint count plus estimated smooth execution time."""


class RefillScheduler:
    def __init__(self, pending_threshold: int, margin_s: float):
        self.pending_threshold = pending_threshold
        self.margin_s = margin_s
        self.latency_s = 5.5  # Conservative experimental initial estimate, not a guaranteed bound.

    def should_refill(self, count: int, duration_s: float) -> bool:
        return 1 <= count <= 5 and (
            count <= self.pending_threshold or duration_s <= self.latency_s + self.margin_s
        )

    def record_latency(self, latency_s: float) -> None:
        self.latency_s = 0.75 * self.latency_s + 0.25 * latency_s
