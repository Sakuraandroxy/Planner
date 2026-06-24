"""Thread-safe shared state for real-time dashboard updates."""
import threading


class SharedState:
    """Central state object shared between main loop, frame capturer, and Flask routes."""

    def __init__(self):
        self.lock = threading.Lock()
        self.version = 0
        self.status = "starting"
        self.model_name = ""
        self.task = ""
        self.step = 0
        self.max_steps = 20
        self.pose = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.action_mode = "atomic"
        self.collided = False
        self.reasoning = ""
        self.scene_analysis = ""
        self.candidates = []
        self.selected_actions = []
        self.error = ""
        self.reasoning_summary = ""
        self.task_done = False
        self._frame_png = b""
        self._depth_png = b""
        self.frame_version = 0
        self.depth_version = 0

    def update(self, **kwargs):
        with self.lock:
            for k, v in kwargs.items():
                if hasattr(self, k):
                    setattr(self, k, v)
            self.version += 1

    def set_frame(self, png_bytes: bytes):
        with self.lock:
            self._frame_png = png_bytes
            self.frame_version += 1
            self.version += 1

    def set_depth_frame(self, png_bytes: bytes):
        with self.lock:
            self._depth_png = png_bytes
            self.depth_version += 1
            self.version += 1

    def get_frame(self) -> bytes:
        with self.lock:
            return self._frame_png

    def get_depth_frame(self) -> bytes:
        with self.lock:
            return self._depth_png

    def get_state(self) -> dict:
        with self.lock:
            return {
                "version": self.version,
                "frame_version": self.frame_version,
                "depth_version": self.depth_version,
                "depth_bytes": len(self._depth_png),
                "status": self.status,
                "task": self.task,
                "action_mode": self.action_mode,
                "step": self.step,
                "max_steps": self.max_steps,
                "pose": self.pose,
                "yaw": self.yaw,
                "collided": self.collided,
                "reasoning": self.reasoning,
                "scene_analysis": self.scene_analysis,
                "candidates": self.candidates,
                "selected": self.selected_actions,
                "reasoning_summary": self.reasoning_summary,
                "task_done": self.task_done,
                "model_name": self.model_name,
                "error": self.error,
            }
