"""Bounded subprocess ownership; each generation has separate stdin/stdout pipes."""
from dataclasses import asdict
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading

from planner.domain.server import WorkerEvent, WorkerEventKind


class SubprocessPlannerWorker:
    def __init__(self, config_path, mode, *, command=None):
        root = Path(__file__).resolve().parents[3]
        args = command or [sys.executable, "-u", "-m", "planner.server_worker",
                           "--config", str(config_path), "--mode", mode.value]
        self.events = queue.Queue()
        environment = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")
        self.process = subprocess.Popen(args, cwd=root, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace", bufsize=1,
            env=environment, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        self._reader = threading.Thread(target=self._read, name="planner-worker-output", daemon=True)
        self._reader.start()

    def _read(self):
        try:
            for line in self.process.stdout:
                try:
                    value = json.loads(line)
                    event = WorkerEvent(WorkerEventKind(value["kind"]), value.get("task_id"),
                        value.get("message", ""), value.get("level", "INFO"), value.get("success"))
                except (ValueError, KeyError, TypeError):
                    event = WorkerEvent(WorkerEventKind.LOG, message=line.rstrip())
                self.events.put(event)
        finally:
            self.events.put(WorkerEvent(WorkerEventKind.EXITED, message="Planner process exited."))

    def _send(self, command):
        self.process.stdin.write(json.dumps(command, ensure_ascii=False) + "\n")
        self.process.stdin.flush()

    def submit(self, task):
        self._send({"kind": "task", **asdict(task)})

    def poll(self):
        try:
            return self.events.get_nowait()
        except queue.Empty:
            return None

    def terminate(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self._release_pipes()

    def shutdown(self):
        if self.process.poll() is None:
            try:
                self._send({"kind": "shutdown"})
                self.process.wait(timeout=3)
            except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
                self.terminate()
                return
        self._release_pipes()

    def _release_pipes(self):
        # Wait for process death before closing pipes. No old writer survives.
        self._reader.join(timeout=2)
        for stream in (self.process.stdin, self.process.stdout):
            if stream is not None:
                stream.close()
