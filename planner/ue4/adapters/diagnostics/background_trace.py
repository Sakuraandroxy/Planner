"""One bounded file writer; it owns diagnostics, never vehicle control."""
from datetime import datetime
import logging
from queue import Empty, Full, Queue
import threading

logger = logging.getLogger(__name__)


class BackgroundTrace:
    def __init__(self, trace):
        self.trace, self.path = trace, trace.path
        self.jobs = Queue(maxsize=16)
        self.closed = threading.Event()
        self.dropped = 0
        self.thread = threading.Thread(target=self._run,name="planner-diagnostic-writer",daemon=True)
        self.thread.start()

    def _submit(self, callback, *args, **kwargs):
        if self.closed.is_set():
            return
        try:
            self.jobs.put_nowait((callback,args,kwargs))
        except Full:
            self.dropped += 1  # Diagnostics cannot delay or reject a flight action.

    def _run(self):
        while not self.closed.is_set() or not self.jobs.empty():
            try:
                callback,args,kwargs = self.jobs.get(timeout=.1)
            except Empty:
                continue
            try:
                callback(*args,**kwargs)
            except Exception as exc:
                logger.warning("Diagnostic write failed (%s)",type(exc).__name__)
            finally:
                self.jobs.task_done()
        if self.dropped:
            logger.warning("Diagnostic writer skipped %s jobs while storage was busy",self.dropped)

    def event(self, event, **data):
        data.setdefault("logged_wall_time",datetime.now().isoformat())
        self._submit(self.trace.event,event,**data)

    def observation(self, observation, detection):
        self._submit(self.trace.observation,observation,detection)

    def planning_observation(self, request_id, observation):
        self._submit(self.trace.planning_observation,request_id,observation)

    def completion_observation(self, request, verdict):
        self._submit(self.trace.completion_observation,request,verdict)

    def control_samples(self, samples, *, dropped=0):
        self._submit(self.trace.control_samples,samples,dropped=dropped)

    def close(self, *, wait=False):
        """Drain existing jobs after stage end; the owner need not wait on disk."""
        self.closed.set()
        if wait:
            self.thread.join()
