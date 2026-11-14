"""Save once per stage; all temporary views restore the same initial mount."""
import logging
from planner.ports.camera_view import CameraView


class TemporaryView:
    def __init__(self, camera: CameraView):
        self.camera = camera
        self.original = None
        self.last_visible = None

    def begin(self):
        if self.original is not None:
            raise RuntimeError("temporary camera stage is already active")
        self.original = self.camera.read_mount()
        self.last_visible = None

    def remember_visible(self):
        self.last_visible = self.camera.read_mount()

    def recover_visible(self):
        if self.last_visible is not None:
            self.camera.restore(self.last_visible)

    def look_at(self, point):
        self.camera.point_at_world(point)

    def look_direction(self, direction):
        self.camera.point_direction_world(direction)

    def down(self):
        self.camera.point_down()

    def restore(self):
        if self.original is not None:
            self.camera.restore(self.original)
            self.original = None
            self.last_visible = None
            logging.getLogger(__name__).info("[AboveView] restored the original camera mount")
