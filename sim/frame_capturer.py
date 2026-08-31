"""Background capture of timestamped, pose-aware AirSim Camera API frames."""

from __future__ import annotations

import math
import threading
import time as _time
import uuid
from io import BytesIO
from typing import Optional, Tuple

from sim.camera_frame_hub import CameraFrameHub
from sim.camera_frames import CameraFrame, CameraIntrinsics, MultiCameraSnapshot


def _quaternion_values(value) -> list[float]:
    if value is None:
        return [0.0, 0.0, 0.0, 1.0]
    return [
        float(getattr(value, "x_val", 0.0)),
        float(getattr(value, "y_val", 0.0)),
        float(getattr(value, "z_val", 0.0)),
        float(getattr(value, "w_val", 1.0)),
    ]


def _rotation_from_quaternion(quaternion) -> list[list[float]]:
    x, y, z, w = _quaternion_values(quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12 or not math.isfinite(norm):
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def _yaw_from_quaternion(quaternion) -> float:
    x, y, z, w = _quaternion_values(quaternion)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny, cosy))


def _position_values(position) -> list[float]:
    return [
        float(getattr(position, "x_val", 0.0)),
        float(getattr(position, "y_val", 0.0)),
        float(getattr(position, "z_val", 0.0)),
    ]


class FrameCapturer:
    """Continuously capture configured cameras through one background client.

    The original ``get_latest_frame`` interface remains mapped to the primary
    camera so existing code remains usable while consumers migrate to full
    ``MultiCameraSnapshot`` objects.
    """

    def __init__(self, state, interval=0.1, frame_hub=None, camera_ids=None):
        import airsim
        from config import cfg

        sim_cfg = dict(cfg.get("SIM", {}) or {})
        port = int(sim_cfg.get("AIRSIM_PORT", 41451))
        ip = str(sim_cfg.get("AIRSIM_IP", "") or "").strip()
        self.client = airsim.MultirotorClient(ip=ip, port=port) if ip else airsim.MultirotorClient(port=port)
        self.state = state
        self.interval = float(interval)
        self.frame_hub = frame_hub or CameraFrameHub()
        self._camera_specs = self._resolve_camera_specs(sim_cfg, camera_ids)
        self.camera_ids = tuple(self._camera_specs)
        self.primary_camera_id = self._resolve_primary_camera_id(self._camera_specs)
        self._stop = threading.Event()
        self._paused = threading.Event()
        self._busy = threading.Event()
        self._thread = None
        self._lock = threading.Lock()

        self._latest_rgb: Optional[bytes] = None
        self._latest_depth: Optional["numpy.ndarray"] = None
        self._latest_rgb_png: Optional[bytes] = None
        self._latest_captured_at = 0.0
        self._latest_observer_world = (0.0, 0.0, 0.0)
        self._latest_observer_yaw_deg = 0.0
        self._latest_snapshot: Optional[MultiCameraSnapshot] = None
        self._latest_camera_frames: dict[str, CameraFrame] = {}
        self._capture_count = 0
        self._capture_period_ema_s = 0.0
        self._last_capture_duration_s = 0.0
        self._error_count = 0
        front_offset = sim_cfg.get("FRONT_CAMERA_OFFSET", [1.0, 0.0, 0.0])
        self._front_camera_offset = tuple(float(value) for value in list(front_offset)[:3])
        if len(self._front_camera_offset) < 3:
            self._front_camera_offset = (1.0, 0.0, 0.0)
        self._fov_cache: dict[str, float] = {}

    @staticmethod
    def _resolve_camera_specs(sim_cfg: dict, camera_ids=None) -> dict[str, dict]:
        configured = sim_cfg.get("CAMERAS")
        specs: dict[str, dict] = {}
        if isinstance(configured, dict):
            for camera_id, value in configured.items():
                specs[str(camera_id)] = dict(value or {})
        elif isinstance(configured, list):
            for value in configured:
                if isinstance(value, str):
                    specs[value] = {}
                elif isinstance(value, dict):
                    camera_id = value.get("ID") or value.get("id") or value.get("CAMERA_ID")
                    if camera_id:
                        specs[str(camera_id)] = dict(value)
        requested = tuple(dict.fromkeys(str(value).strip() for value in (camera_ids or ()) if str(value).strip()))
        if requested:
            specs = {camera_id: specs.get(camera_id, {}) for camera_id in requested}
        if not specs:
            specs = {"front_center": {}}
        return specs

    @staticmethod
    def _resolve_primary_camera_id(specs: dict[str, dict]) -> str:
        for camera_id, spec in specs.items():
            roles = spec.get("ROLES", spec.get("roles", spec.get("ROLE", spec.get("role", []))))
            if isinstance(roles, str):
                roles = [roles]
            if "primary" in {str(role).strip().lower() for role in list(roles or [])}:
                return camera_id
        return next(iter(specs), "front_center")

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="camera-frame-capturer", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    def pause(self, wait: bool = True, timeout: float = 2.0):
        self._paused.set()
        if not wait:
            return
        deadline = _time.perf_counter() + max(0.0, timeout)
        while self._busy.is_set() and _time.perf_counter() < deadline:
            _time.sleep(0.01)

    def resume(self):
        self._paused.clear()

    def get_latest_frame(self) -> Tuple[Optional[bytes], Optional["numpy.ndarray"]]:
        with self._lock:
            return self._latest_rgb, self._latest_depth

    def get_latest_frame_with_timestamp(self):
        with self._lock:
            return self._latest_rgb, self._latest_depth, self._latest_captured_at

    def get_latest_observation(self) -> dict:
        with self._lock:
            return {
                "rgb": self._latest_rgb,
                "depth": self._latest_depth,
                "rgb_png": self._latest_rgb_png,
                "captured_at": self._latest_captured_at,
                "observer_world": self._latest_observer_world,
                "observer_yaw_deg": self._latest_observer_yaw_deg,
                "camera_id": getattr(self, "primary_camera_id", "front_center"),
                "snapshot": getattr(self, "_latest_snapshot", None),
            }

    def get_capture_status(self) -> dict:
        with self._lock:
            return {
                "capture_count": self._capture_count,
                "capture_period_ema_s": self._capture_period_ema_s,
                "last_capture_duration_s": self._last_capture_duration_s,
                "captured_at": self._latest_captured_at,
                "camera_ids": list(getattr(self, "camera_ids", ("front_center",))),
                "busy": bool(getattr(self, "_busy", threading.Event()).is_set()),
                "paused": bool(getattr(self, "_paused", threading.Event()).is_set()),
                "error_count": int(getattr(self, "_error_count", 0)),
            }

    def get_latest_rgb_png(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_rgb_png

    def get_latest_snapshot(self) -> Optional[MultiCameraSnapshot]:
        with self._lock:
            return self._latest_snapshot

    def get_latest_camera_frame(self, camera_id: str) -> Optional[CameraFrame]:
        with self._lock:
            return self._latest_camera_frames.get(str(camera_id))

    def _observer_pose_from_response(self, response):
        position = getattr(response, "camera_position", None)
        orientation = getattr(response, "camera_orientation", None)
        if position is None or orientation is None:
            return (0.0, 0.0, 0.0), 0.0
        rotation = _rotation_from_quaternion(orientation)
        camera_world = _position_values(position)
        offset = list(getattr(self, "_front_camera_offset", (1.0, 0.0, 0.0)))
        body_world = [
            camera_world[row] - sum(rotation[row][column] * offset[column] for column in range(3))
            for row in range(3)
        ]
        return tuple(body_world), _yaw_from_quaternion(orientation)

    def _camera_fov(self, camera_id: str, fallback: float) -> float:
        if camera_id in self._fov_cache:
            return self._fov_cache[camera_id]
        fov = float(fallback)
        try:
            info = self.client.simGetCameraInfo(camera_id)
            candidate = float(getattr(info, "fov", fov))
            if math.isfinite(candidate) and 0.0 < candidate < 180.0:
                fov = candidate
        except Exception:
            pass
        self._fov_cache[camera_id] = fov
        return fov

    @staticmethod
    def _response_timestamp_ns(response) -> int:
        value = int(getattr(response, "time_stamp", 0) or 0)
        return value if value > 0 else _time.time_ns()

    @staticmethod
    def _vehicle_pose(client):
        try:
            pose = client.simGetVehiclePose()
            position = _position_values(pose.position)
            rotation = _rotation_from_quaternion(pose.orientation)
            yaw = _yaw_from_quaternion(pose.orientation)
            return position, rotation, yaw
        except Exception:
            return [0.0, 0.0, 0.0], [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], 0.0

    def _request_specs(self, airsim):
        requests = []
        response_specs = []
        for camera_id, spec in self._camera_specs.items():
            if bool(spec.get("RGB", spec.get("rgb", True))):
                requests.append(airsim.ImageRequest(camera_id, airsim.ImageType.Scene, False, True))
                response_specs.append((camera_id, "rgb"))
            if bool(spec.get("DEPTH", spec.get("depth", True))):
                requests.append(airsim.ImageRequest(camera_id, airsim.ImageType.DepthPerspective, True, False))
                response_specs.append((camera_id, "depth"))
        return requests, response_specs

    def _loop(self):
        import airsim
        import numpy as np
        from PIL import Image

        requests, response_specs = self._request_specs(airsim)
        previous_capture_at = 0.0
        while not self._stop.is_set():
            if self._paused.is_set():
                _time.sleep(min(self.interval, 0.05))
                continue
            started_at = _time.perf_counter()
            try:
                self._busy.set()
                responses = self.client.simGetImages(requests)
                vehicle_position, body_rotation, navigation_yaw = self._vehicle_pose(self.client)
                decoded: dict[str, dict] = {camera_id: {} for camera_id in self.camera_ids}
                for index, (camera_id, kind) in enumerate(response_specs):
                    if not responses or index >= len(responses):
                        continue
                    response = responses[index]
                    decoded[camera_id][kind + "_response"] = response
                    if kind == "rgb" and getattr(response, "image_data_uint8", None):
                        with Image.open(BytesIO(bytes(response.image_data_uint8))) as source:
                            image = source.convert("RGB").copy()
                        jpeg_buffer = BytesIO()
                        image.save(jpeg_buffer, format="JPEG", quality=85)
                        png_buffer = BytesIO()
                        image.save(png_buffer, format="PNG")
                        decoded[camera_id].update(
                            rgb=image,
                            rgb_jpeg=jpeg_buffer.getvalue(),
                            rgb_png=png_buffer.getvalue(),
                        )
                    elif kind == "depth" and getattr(response, "image_data_float", None):
                        decoded[camera_id]["depth"] = np.asarray(
                            response.image_data_float, dtype=np.float32
                        ).reshape(int(response.height), int(response.width))

                capture_id = f"{_time.time_ns()}-{uuid.uuid4().hex[:8]}"
                frames: dict[str, CameraFrame] = {}
                for camera_id, values in decoded.items():
                    rgb_response = values.get("rgb_response")
                    depth_response = values.get("depth_response")
                    pose_response = rgb_response if rgb_response is not None else depth_response
                    if pose_response is None:
                        continue
                    spec = self._camera_specs.get(camera_id, {})
                    default_fov = float(spec.get("FOV", spec.get("fov", 90.0)))
                    rgb_fov = self._camera_fov(camera_id, default_fov)
                    depth_fov = float(spec.get("DEPTH_FOV", spec.get("depth_fov", rgb_fov)))
                    camera_position = _position_values(getattr(pose_response, "camera_position", None))
                    camera_orientation = getattr(pose_response, "camera_orientation", None)
                    rgb_intrinsics = None
                    if rgb_response is not None and int(getattr(rgb_response, "width", 0) or 0) > 0:
                        rgb_intrinsics = CameraIntrinsics.from_horizontal_fov(
                            rgb_response.width, rgb_response.height, rgb_fov, depth_mode="radial"
                        )
                    depth_intrinsics = None
                    if depth_response is not None and int(getattr(depth_response, "width", 0) or 0) > 0:
                        depth_intrinsics = CameraIntrinsics.from_horizontal_fov(
                            depth_response.width, depth_response.height, depth_fov, depth_mode="radial"
                        )
                    depth_position = None
                    depth_quaternion = None
                    depth_rotation = None
                    if depth_response is not None:
                        depth_position = _position_values(getattr(depth_response, "camera_position", None))
                        depth_orientation = getattr(depth_response, "camera_orientation", None)
                        depth_quaternion = _quaternion_values(depth_orientation)
                        depth_rotation = _rotation_from_quaternion(depth_orientation)
                    frames[camera_id] = CameraFrame(
                        camera_id=camera_id,
                        capture_id=capture_id,
                        timestamp_ns=self._response_timestamp_ns(pose_response),
                        rgb=values.get("rgb"),
                        depth=values.get("depth"),
                        rgb_png=values.get("rgb_png"),
                        rgb_jpeg=values.get("rgb_jpeg"),
                        rgb_intrinsics=rgb_intrinsics,
                        depth_intrinsics=depth_intrinsics,
                        camera_position_world=camera_position,
                        camera_quaternion_world=_quaternion_values(camera_orientation),
                        rotation_camera_to_world=_rotation_from_quaternion(camera_orientation),
                        depth_timestamp_ns=self._response_timestamp_ns(depth_response) if depth_response is not None else 0,
                        depth_camera_position_world=depth_position,
                        depth_camera_quaternion_world=depth_quaternion,
                        depth_rotation_camera_to_world=depth_rotation,
                        vehicle_position_world=vehicle_position,
                        rotation_body_to_world=body_rotation,
                        navigation_yaw_deg=navigation_yaw,
                        captured_at=started_at,
                    )
                    if values.get("rgb") is not None:
                        try:
                            values["rgb"].camera_frame = frames[camera_id]
                            values["rgb"].camera_id = camera_id
                            values["rgb"].capture_id = capture_id
                        except Exception:
                            pass
                if not frames:
                    raise RuntimeError("simGetImages returned no decodable camera frames")
                snapshot = MultiCameraSnapshot(
                    capture_id=capture_id,
                    frames=frames,
                    vehicle_position_world=vehicle_position,
                    rotation_body_to_world=body_rotation,
                    navigation_yaw_deg=navigation_yaw,
                    captured_at=started_at,
                )
                primary = frames.get(self.primary_camera_id) or next(iter(frames.values()))
                # Vehicle pose comes from simGetVehiclePose for this snapshot;
                # it must not be reconstructed from a camera with arbitrary
                # Pitch/Yaw/Roll and a guessed front-camera offset.
                observer_world, observer_yaw = tuple(vehicle_position), float(navigation_yaw)
                completed_at = _time.perf_counter()
                period = completed_at - previous_capture_at if previous_capture_at else 0.0
                with self._lock:
                    self._latest_rgb = primary.rgb_jpeg
                    self._latest_depth = primary.depth
                    self._latest_rgb_png = primary.rgb_png
                    self._latest_captured_at = started_at
                    self._latest_observer_world = observer_world
                    self._latest_observer_yaw_deg = observer_yaw
                    self._latest_snapshot = snapshot
                    self._latest_camera_frames.update(frames)
                    self._capture_count += 1
                    self._last_capture_duration_s = completed_at - started_at
                    if period > 0:
                        self._capture_period_ema_s = (
                            period if self._capture_period_ema_s <= 0 else 0.85 * self._capture_period_ema_s + 0.15 * period
                        )
                previous_capture_at = completed_at
                if self.state is not None and hasattr(self.state, "set_camera_frame"):
                    for camera_id, frame in frames.items():
                        if frame.rgb_png:
                            self.state.set_camera_frame(camera_id, frame.rgb_png, frame.to_metadata_dict())
                self.frame_hub.publish(snapshot)
            except Exception as exc:
                self._error_count += 1
                if self._error_count <= 3:
                    print(f"  [FrameCapturer] error #{self._error_count}: {exc}")
                elif self._error_count == 4:
                    print("  [FrameCapturer] suppressing further errors...")
            finally:
                self._busy.clear()
            _time.sleep(self.interval)
