from types import SimpleNamespace

from planner import bootstrap
from planner.adapters.airsim.observation_source import AirSimObservationSource


def test_recording_runtime_uses_independent_source_without_motion_gate(monkeypatch, tmp_path):
    class FakeConnection:
        def __init__(self, host, port, timeout_s):
            self.args = (host, port, timeout_s)

    class FakeRecorder:
        def __init__(self, source, output_root, fps):
            self.source = source
            self.output_root = output_root
            self.fps = fps

    monkeypatch.setattr(bootstrap, "AirSimConnection", FakeConnection)
    monkeypatch.setattr(bootstrap, "CameraApiRecorder", FakeRecorder)

    config = SimpleNamespace(
        airsim=SimpleNamespace(
            host="localhost",
            port=41451,
            connect_timeout_s=5.0,
            move_timeout_s=60.0,
            speed_mps=2.0,
            camera_id="front_center",
        ),
        depth=SimpleNamespace(min_m=0.5, max_m=200.0),
        recording=SimpleNamespace(output_root=str(tmp_path), fps=5.0),
    )

    runtime = bootstrap.build_recording_runtime(config)

    assert isinstance(runtime.recorder.source, AirSimObservationSource)
    assert runtime.recorder.source.connection is runtime.connection
    assert runtime.recorder.source.observation_gate is None
