"""Only read command data in the input thread; flight stays on the worker owner."""
import json
import queue
import threading

from planner.domain.server import TaskSubmission


class CommandInbox:
    def __init__(self, stream):
        self.items = queue.Queue()
        self.thread = threading.Thread(target=self._read, args=(stream,), daemon=True, name="planner-input")
        self.thread.start()

    def _read(self, stream):
        try:
            for line in stream:
                command = json.loads(line)
                if command['kind'] == 'shutdown':
                    return
                self.items.put(TaskSubmission(command['task_id'], command['instruction']))
        finally:
            self.items.put(None)

    def receive(self, timeout_s=None):
        return self.items.get(timeout=timeout_s)
