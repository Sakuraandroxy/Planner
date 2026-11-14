"""Owner-recorded simulator-time queue history for delayed image observations."""
from collections import deque

from planner.domain.streaming import QueueObservationAnchor
from planner.errors import ProtocolError


class StreamTimeline:
    def __init__(self, maximum_entries=4096):
        self.entries = deque(maxlen=maximum_entries)

    def clear(self):
        self.entries.clear()

    def record(self, timestamp_ns, stage_token, queue, history=None):
        if self.entries and timestamp_ns < self.entries[-1].timestamp_ns:
            raise ProtocolError("simulator clock moved backwards during streaming")
        item = QueueObservationAnchor(stage_token, queue.revision, timestamp_ns, queue.items, history)
        if self.entries and timestamp_ns == self.entries[-1].timestamp_ns:
            self.entries[-1] = item
        else:
            self.entries.append(item)

    def at(self, timestamp_ns, stage_token, maximum_gap_ns=250_000_000):
        if not self.entries or timestamp_ns < self.entries[0].timestamp_ns:
            raise ProtocolError("image predates retained execution history")
        for entry in reversed(self.entries):
            if entry.timestamp_ns <= timestamp_ns:
                if entry.stage_token != stage_token or timestamp_ns-entry.timestamp_ns > maximum_gap_ns:
                    raise ProtocolError("image has no matching simulator-time queue anchor")
                return entry
        raise ProtocolError("image queue anchor unavailable")

