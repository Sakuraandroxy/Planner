"""Run existing missions on one owner thread, and emit data-only lifecycle events."""
from typing import Callable
import queue
import time

from planner.application.reporting import mission_report
from planner.domain.server import TaskSubmission, WorkerEvent, WorkerEventKind, WorkerMode
from planner.ports.backend_runtime import BackendSession


def serve_worker(session: BackendSession, mode: WorkerMode,
                 receive: Callable[[], TaskSubmission | None], emit: Callable[[WorkerEvent], None], *, idle_poll=None):
    try:
        emit(WorkerEvent(WorkerEventKind.LOG, message="Connecting to AirSim..."))
        session.prepare(mode)
        if mode is WorkerMode.CLEANUP:
            emit(WorkerEvent(WorkerEventKind.CLOSED, message="Vehicle stop completed."))
            return
        emit(WorkerEvent(WorkerEventKind.LOG, message="AirSim ready. Waiting for a task."))
        emit(WorkerEvent(WorkerEventKind.READY))
        while True:
            if idle_poll is None:
                task = receive()
            else:
                tick_started = time.monotonic()
                session.idle_tick()
                try:
                    task = idle_poll(max(.001,.05-(time.monotonic()-tick_started)))
                except queue.Empty:
                    continue
            if task is None:
                return
            success = False
            lines = ()
            try:
                result = session.execute(task.instruction)
                lines, code = mission_report(result)
                success = code == 0
            except Exception as exc:
                emit(WorkerEvent(WorkerEventKind.LOG, task.task_id,
                                 f"Task rejected: {exc}", "ERROR"))
            if session.connection_lost:
                emit(WorkerEvent(WorkerEventKind.DISCONNECTED, task.task_id,
                                 "AirSim connection lost during task execution.", "ERROR"))
                return
            try:
                session.hold()
            except Exception:
                emit(WorkerEvent(WorkerEventKind.LOG, task.task_id,
                    "Task execution ended; final position hold was not confirmed.", "ERROR"))
                raise
            for line in lines:
                emit(WorkerEvent(WorkerEventKind.LOG, task.task_id, line))
            emit(WorkerEvent(WorkerEventKind.FINISHED, task.task_id, success=success))
            if not success:
                return  # A failed runtime can retain quarantined I/O. Replace it.
    except Exception as exc:
        kind = WorkerEventKind.DISCONNECTED if session.connection_lost else WorkerEventKind.ERROR
        emit(WorkerEvent(kind, message=f"Planner worker failed: {exc}", level="ERROR"))
    finally:
        try:
            session.close()
        except Exception as exc:
            emit(WorkerEvent(WorkerEventKind.LOG, message=f"Worker cleanup failed: {exc}", level="ERROR"))
