import json

from PIL import Image

from planner.adapters.airsim.recording import CameraApiRecorder
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import WorldPose


class ObservationSource:
    def capture(self):
        image = Image.new("RGB", (4, 3), color=(10, 20, 30))
        return Observation(
            rgb=image,
            depth_meters=None,
            depth_image=image,
            vehicle_pose=WorldPose(1, 2, -3, 15),
            camera_id="front_center",
            intrinsics=CameraIntrinsics(4, 3, 2, 2, 2, 1.5, 90),
            camera_position_world=(1.1, 2.2, -3.3),
            rotation_camera_to_world=((1, 0, 0), (0, 1, 0), (0, 0, 1)),
            timestamp_ns=123,
        )


def test_recorder_saves_aligned_frames_and_metadata(tmp_path):
    recorder = CameraApiRecorder(ObservationSource(), tmp_path, fps=5)
    run_dir = recorder.start()
    recorder.stop()
    assert (run_dir / "rgb" / "000000.jpg").exists()
    assert (run_dir / "depth" / "000000.png").exists()
    metadata = json.loads((run_dir / "metadata" / "000000.json").read_text(encoding="utf-8"))
    assert metadata["camera_id"] == "front_center"
    assert metadata["vehicle_pose"]["yaw_deg"] == 15
    assert metadata["intrinsics"]["horizontal_fov_deg"] == 90
