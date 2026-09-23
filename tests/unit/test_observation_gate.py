import threading
from types import SimpleNamespace

from planner.adapters.airsim.observation_gate import ObservationGate
from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose


def test_motion_thread_waits_until_capture_finishes():
    vehicle = SimpleNamespace(hover=lambda: None,
        current_pose=lambda: WorldPose(0, 0, -2, 0), collision_state=lambda: False)
    gate = ObservationGate(vehicle, MotionLimits(control_hz=50, settle_time_s=0.01), 2)
    attempted, moved = threading.Event(), threading.Event()

    def motion():
        attempted.set()
        with gate.step():
            moved.set()

    thread = threading.Thread(target=motion)
    with gate.capture():
        thread.start()
        assert attempted.wait(1)
        assert not moved.wait(0.05)
    thread.join(1)
    assert not thread.is_alive()
    assert moved.is_set()
