"""Wraps all AirSim API calls into a clean interface."""
import contextlib
import math, io, time
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R
import airsim
from PIL import Image, ImageDraw, ImageFont


class _SynchronizedAirSimRpcClient:
    """Serialize calls made through one msgpackrpc client.

    AirSim's Python client uses a mutable Tornado write buffer and is not
    thread-safe.  The fast/slow runtime intentionally captures images in a
    worker while the main thread polls vehicle pose, so all calls that share
    this connection must cross one lock.
    """

    def __init__(self, client):
        self._client = client
        self._lock = threading.RLock()

    def __getattr__(self, name):
        attribute = getattr(self._client, name)
        if not callable(attribute):
            return attribute

        def synchronized_call(*args, **kwargs):
            with self._lock:
                return attribute(*args, **kwargs)

        return synchronized_call


class AirSimClient:
    """Singleton-style wrapper around the AirSim MultirotorClient."""

    def __init__(self, ip: str = "", port: int = 41451, use_config_ip: bool = True, timeout_value: float | None = None):
        from config import cfg
        sim = cfg.get("SIM", {})
        if port == 41451:
            port = int(sim.get("AIRSIM_PORT", 41451))
        if not ip and use_config_ip:
            ip = sim.get("AIRSIM_IP", "")
        self._ip = ip
        self._port = port
        self._timeout_value = timeout_value
        if timeout_value is None:
            raw_client = airsim.MultirotorClient(ip=ip, port=port) if ip else airsim.MultirotorClient(port=port)
        else:
            raw_client = (
                airsim.MultirotorClient(ip=ip, port=port, timeout_value=timeout_value)
                if ip else airsim.MultirotorClient(port=port, timeout_value=timeout_value)
            )
        self.client = _SynchronizedAirSimRpcClient(raw_client)
        self._connected = False

    def connect(self, quiet: bool = False):
        # Do not redirect stdout here. `confirmConnection()` may be called from a
        # worker thread with a timeout wrapper; `redirect_stdout()` mutates the
        # process-global `sys.stdout`, which can hide main-thread retry logs if the
        # RPC handshake blocks.
        self.client.confirmConnection()
        self._connected = True

    def enable_api_control(self, enabled: bool = True):
        self.client.enableApiControl(enabled)

    def arm(self, armed: bool = True):
        self.client.armDisarm(armed)

    def takeoff(self):
        self.client.takeoffAsync().join()

    def land(self):
        self.client.landAsync().join()

    def get_multirotor_state(self):
        return self.client.getMultirotorState()

    def get_pose(self):
        """Returns (pos: [x,y,z], yaw_deg: float)."""
        p = self.client.simGetVehiclePose()
        pos = [p.position.x_val, p.position.y_val, p.position.z_val]
        q = p.orientation
        siny = 2.0 * (q.w_val * q.z_val + q.x_val * q.y_val)
        cosy = 1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val)
        yaw = math.degrees(math.atan2(siny, cosy))
        return pos, yaw

    def get_pose_full(self):
        """Returns (pos, yaw_deg, rotation_matrix body->world)."""
        p = self.client.simGetVehiclePose()
        pos = [p.position.x_val, p.position.y_val, p.position.z_val]
        q = p.orientation
        siny = 2.0 * (q.w_val * q.z_val + q.x_val * q.y_val)
        cosy = 1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val)
        yaw = math.degrees(math.atan2(siny, cosy))
        rot = R.from_quat([q.x_val, q.y_val, q.z_val, q.w_val]).as_matrix().tolist()
        return pos, yaw, rot

    def move_to_position(self, x, y, z, velocity=None, timeout=10.0):
        if velocity is None:
            from config import cfg
            velocity = float(cfg.get("SIM", {}).get("AIRSIM_VELOCITY", 2.0))
        self.client.moveToPositionAsync(x, y, z, velocity, timeout_sec=timeout).join()

    def rotate_to_yaw(self, yaw_deg, timeout=5.0):
        self.client.rotateToYawAsync(yaw_deg, timeout_sec=timeout).join()

    def rotate_yaw(self, delta_deg, timeout=5.0):
        """Rotate relative to the current yaw angle."""
        _pos, yaw = self.get_pose()
        self.rotate_to_yaw(yaw + float(delta_deg), timeout=timeout)

    def set_pose_xyz_yaw(self, x, y, z, yaw_deg: float, ignore_collision: bool = True):
        """Teleport vehicle to an exact position/yaw pose for dataset initialization."""
        yaw_rad = math.radians(float(yaw_deg))
        pose = airsim.Pose(
            airsim.Vector3r(float(x), float(y), float(z)),
            airsim.to_quaternion(0.0, 0.0, yaw_rad),
        )
        self.client.simSetVehiclePose(pose, ignore_collision=ignore_collision)

    def get_image(self):
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, True)
        ])
        if not responses or not responses[0].image_data_uint8:
            return None
        return self._decode_rgb_image_bytes(responses[0].image_data_uint8)

    def get_scene_and_depth_meters(self):
        """Return RGB frame and raw DepthPerspective meters from one AirSim RPC."""
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, True),
            airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False),
        ])
        frame = None
        depth = None
        if responses and len(responses) > 0 and responses[0].image_data_uint8:
            frame = self._decode_rgb_image_bytes(responses[0].image_data_uint8)
        if responses and len(responses) > 1 and responses[1].image_data_float:
            r = responses[1]
            depth = np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)
        return frame, depth

    def resolve_capture_profile(self, profile: str | None = None) -> str:
        """解析抓图内容配置。

        支持：
        - front_depth
        - front_down
        - front_depth_only
        - front_down_depth_only
        - front_down_front_depth
        - front_down_both_depth
        - auto
        """
        from config import cfg

        if profile and profile != "auto":
            return profile

        sim_cfg = cfg.get("SIM", {})
        configured = str(sim_cfg.get("CAPTURE_PROFILE", "auto") or "auto").strip().lower()
        if configured and configured != "auto":
            return configured

        return "front_down"

    def resolve_capture_mode(self, mode: str | None = None) -> str:
        """解析抓图模式配置。"""
        from config import cfg

        if mode and mode != "auto":
            return mode
        sim_cfg = cfg.get("SIM", {})
        configured = str(sim_cfg.get("CAPTURE_MODE", "batch") or "batch").strip().lower()
        return configured or "batch"

    def get_configured_views(self, profile: str | None = None, mode: str | None = None, verbose: bool = False):
        """按配置抓取前/下视 RGB 与深度。

        返回:
            (front_rgb, down_rgb, front_depth, down_depth)
        其中未请求的项为 None。
        """
        resolved_profile = self.resolve_capture_profile(profile)
        resolved_mode = self.resolve_capture_mode(mode)
        front_rgb, down_rgb, front_depth, down_depth, _timing = self.capture_views(
            profile=resolved_profile,
            mode=resolved_mode,
            verbose=verbose,
        )
        return front_rgb, down_rgb, front_depth, down_depth

    def depth_meters_to_image(self, depth_meters, preview_width: int = 256):
        """Convert raw meter depth matrix to an 8-bit preview image."""
        if depth_meters is None:
            return None
        MAX_DEPTH = 100.0
        depth = np.clip(depth_meters, 0, MAX_DEPTH)
        depth_8bit = (depth / MAX_DEPTH * 255).astype(np.uint8)
        img = Image.fromarray(depth_8bit, mode="L")
        if preview_width and img.width > preview_width:
            preview_height = max(1, int(img.height * preview_width / img.width))
            img = img.resize((preview_width, preview_height), Image.Resampling.BILINEAR)
        return img

    def depth_meters_to_stats(self, depth_meters):
        """Compute scene-wide and center depth statistics from one depth matrix."""
        if depth_meters is None:
            return None
        all_valid = depth_meters[(depth_meters > 0.1) & (depth_meters < 1000.0)]
        if len(all_valid) == 0:
            return None
        scene_min = float(np.min(all_valid))
        scene_max = float(np.max(all_valid))
        h, w = depth_meters.shape
        cy, cx = h // 2, w // 2
        half_h, half_w = h // 10, w // 10
        region = depth_meters[cy-half_h:cy+half_h, cx-half_w:cx+half_w]
        valid = region[(region > 0.1) & (region < 1000.0)]
        center_min = float(np.min(valid)) if len(valid) > 0 else None
        center_avg = float(np.mean(valid)) if len(valid) > 0 else None
        return {
            "scene_min": scene_min,
            "scene_max": scene_max,
            "center_min": center_min,
            "center_avg": center_avg,
        }

    def get_depth_image(self, preview_width: int = 256):
        """Return depth as 8-bit grayscale PNG.
        DepthPerspective returns actual meters (float32).
        Normalized to 0-100m: pixel_value * 100 / 255 = distance in meters.
        Examples: pixel 13 = 5m, pixel 128 = 50m, pixel 204 = 80m.
        16-bit PNGs are not supported by VLM APIs (crushed to black),
        so we normalize to 8-bit with a fixed 100m range.
        """
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)
        ])
        if not responses or not responses[0].image_data_float:
            return None
        r = responses[0]
        depth = np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)
        return self.depth_meters_to_image(depth, preview_width=preview_width)

    def get_depth_meters(self):
        """Return raw DepthPerspective meters as a float32 array."""
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)
        ])
        if not responses or not responses[0].image_data_float:
            return None
        r = responses[0]
        return np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)



    def get_labeled_depth_heatmap(self):
        """VLM depth: heatmap with white text labels showing exact depth.
        Red=close(0-10m), yellow=mid(10-30), green=far(30+), black=sky.
        White labels every 200px show exact distance in meters.
        VLM reads the text label at the target location for precise depth.
        """
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)
        ])
        if not responses or not responses[0].image_data_float:
            return None
        r = responses[0]
        dm = np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)
        h, w = dm.shape
        # Heatmap colors
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        # Red: 0-10m (close)
        rgb[:, :, 0] = (np.clip(10.0 - np.clip(dm, 0, 10), 0, 10) / 10 * 255).astype(np.uint8)
        # Green: 5-30m (mid range)
        rgb[:, :, 1] = (np.clip(30.0 - np.abs(np.clip(dm, 5, 30) - 17.5), 0, 12.5) / 12.5 * 255).astype(np.uint8)
        # Blue: 20m+ (far)
        rgb[:, :, 2] = (np.clip(np.clip(dm, 20, 100) - 20, 0, 80) / 80 * 255).astype(np.uint8)
        sky = (dm < 0.01) | (dm > 1e4)
        rgb[sky] = [0, 0, 0]
        img = Image.fromarray(rgb, mode="RGB")
        draw = ImageDraw.Draw(img)
        try:
            font = ImageFont.truetype("arial.ttf", 22)
        except Exception:
            try:
                font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 22)
            except Exception:
                font = ImageFont.load_default()
        step = 200
        for y in range(step//2, h, step):
            for x in range(step//2, w, step):
                val = dm[y, x]
                if val > 0.01 and val < 1e4:
                    label = f"{val:.1f}m"
                    bbox = draw.textbbox((0, 0), label, font=font)
                    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
                    draw.rectangle([x-tw//2-3, y-th//2-3, x+tw//2+3, y+th//2+3], fill=(0, 0, 0))
                    draw.text((x-tw//2, y-th//2), label, fill=(255, 255, 255), font=font)
        return img

    def get_depth_image_vlm(self):
        """Backward-compatible alias for get_labeled_depth_heatmap."""
        return self.get_labeled_depth_heatmap()

    def get_center_depth_meters(self):
        """Get scene-wide depth statistics.
        Returns dict with scene_min, scene_max, center_min, center_avg,
        or None on failure.
        """
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)
        ])
        if not responses or not responses[0].image_data_float:
            return None
        r = responses[0]
        depth = np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)
        return self.depth_meters_to_stats(depth)

    def check_collision(self):
        return self.client.simGetCollisionInfo().has_collided

    def collision_marker(self):
        info = self.client.simGetCollisionInfo()
        return (
            bool(getattr(info, "has_collided", False)),
            int(getattr(info, "time_stamp", 0) or 0),
            str(getattr(info, "object_name", "") or ""),
        )

    def has_collision_since(self, marker) -> bool:
        current = self.collision_marker()
        if not current[0]:
            return False
        if marker is None:
            return True
        previous = tuple(marker)
        if not previous[0]:
            return True
        return current[1:] != previous[1:]

    def get_speed_mps(self) -> float:
        state = self.client.getMultirotorState()
        velocity = state.kinematics_estimated.linear_velocity
        return math.sqrt(
            float(velocity.x_val) ** 2
            + float(velocity.y_val) ** 2
            + float(velocity.z_val) ** 2
        )

    def start_waypoint_path(self, waypoints, velocity=None):
        """Start or replace the active AirSim path without waiting for it to finish."""
        from config import cfg

        if velocity is None:
            velocity = float(cfg.get("SIM", {}).get("AIRSIM_VELOCITY", 2.0))
        active_wps = [[float(v) for v in waypoint[:3]] for waypoint in waypoints]
        if not active_wps:
            return None

        current_pos, _yaw = self.get_pose()
        previous = np.array(current_pos, dtype=float)
        path_len = 0.0
        for waypoint in active_wps:
            target = np.array(waypoint, dtype=float)
            path_len += float(np.linalg.norm(target - previous))
            previous = target

        sim_cfg = cfg.get("SIM", {})
        configured_lookahead = float(sim_cfg.get("AIRSIM_PATH_LOOKAHEAD", 3.0))
        minimum_lookahead = float(sim_cfg.get("AIRSIM_MIN_PATH_LOOKAHEAD", 0.2))
        effective_lookahead = max(
            minimum_lookahead,
            min(configured_lookahead, max(path_len * 0.5, minimum_lookahead)),
        )
        adaptive_lookahead = float(sim_cfg.get("AIRSIM_ADAPTIVE_LOOKAHEAD", 1.0))
        move_timeout = int(sim_cfg.get("AIRSIM_MOVE_TIMEOUT", 60))
        path = [airsim.Vector3r(*waypoint) for waypoint in active_wps]
        functions_cfg = cfg.get("FUNCTIONS", {}) or {}
        fast_slow_cfg = {**(cfg.get("FAST_SLOW", {}) or {}), **(functions_cfg.get("FAST_SLOW", {}) or {})}
        if bool(fast_slow_cfg.get("DEBUG_LOGS", False)):
            print(
                f"  [PathCommand] points={len(active_wps)} path_m={path_len:.2f} "
                f"lookahead={effective_lookahead:.2f} adaptive={adaptive_lookahead:g}"
            )
        return self.client.moveOnPathAsync(
            path=path,
            velocity=float(velocity),
            timeout_sec=move_timeout,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(is_rate=False),
            lookahead=effective_lookahead,
            adaptive_lookahead=adaptive_lookahead,
        )

    def stop_waypoint_path(self):
        try:
            self.client.cancelLastTask()
        except Exception:
            pass
        self.client.hoverAsync().join()

    def cleanup(self):
        try:
            self.client.armDisarm(False)
            self.client.enableApiControl(False)
        except Exception:
            pass


    def _profile_requests(self, profile: str):
        profile = (profile or "").strip().lower()
        mapping = {
            "front_depth": [
                ("front_rgb", airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, True)),
                ("front_depth", airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)),
            ],
            "front_depth_only": [
                ("front_depth", airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)),
            ],
            "front_down": [
                ("front_rgb", airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, True)),
                ("down_rgb", airsim.ImageRequest("down_center", airsim.ImageType.Scene, False, True)),
            ],
            "front_down_depth_only": [
                ("front_depth", airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)),
                ("down_depth", airsim.ImageRequest("down_center", airsim.ImageType.DepthPerspective, True, False)),
            ],
            "front_down_front_depth": [
                ("front_rgb", airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, True)),
                ("down_rgb", airsim.ImageRequest("down_center", airsim.ImageType.Scene, False, True)),
                ("front_depth", airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)),
            ],
            "front_down_both_depth": [
                ("front_rgb", airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, True)),
                ("down_rgb", airsim.ImageRequest("down_center", airsim.ImageType.Scene, False, True)),
                ("front_depth", airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)),
                ("down_depth", airsim.ImageRequest("down_center", airsim.ImageType.DepthPerspective, True, False)),
            ],
        }
        if profile not in mapping:
            raise ValueError(f"Unsupported capture profile: {profile}")
        return mapping[profile]

    def _decode_rgb_image_bytes(self, image_data_uint8):
        """Fully decode AirSim RGB bytes before returning a PIL image.

        PIL's Image.open is lazy; forcing load+copy here avoids later decode
        failures when images are passed across threads/executors.
        """
        if not image_data_uint8:
            return None
        with Image.open(io.BytesIO(bytes(image_data_uint8))) as img:
            return img.convert("RGB").copy()

    def _decode_capture_response(self, request_name: str, response):
        if request_name.endswith("_rgb") and response.image_data_uint8:
            return self._decode_rgb_image_bytes(response.image_data_uint8)
        if request_name.endswith("_depth") and response.image_data_float:
            return np.array(response.image_data_float, dtype=np.float32).reshape(response.height, response.width)
        return None

    def _print_capture_timing(self, timing: dict, front_rgb, front_depth, down_depth):
        rpc_s = timing.get("rpc_s", 0.0)
        decode_s = timing.get("decode_s", 0.0)
        total_s = timing.get("total_s", rpc_s + decode_s)
        if front_rgb is not None and front_depth is not None:
            msg = (
                f"[TIMING] simGetImages={rpc_s:.3f}s  decode={decode_s:.3f}s  total={total_s:.3f}s  "
                f"rgb={front_rgb.size}  depth_front={front_depth.shape}"
            )
            if down_depth is not None:
                msg += f" depth_down={down_depth.shape}"
            print(msg)
        elif front_rgb is not None:
            print(f"[TIMING] simGetImages={rpc_s:.3f}s  decode={decode_s:.3f}s  total={total_s:.3f}s  rgb={front_rgb.size}")
        else:
            print(f"[TIMING] simGetImages={rpc_s:.3f}s  decode={decode_s:.3f}s  total={total_s:.3f}s")

    def _capture_views_batch(self, profile: str):
        named_requests = self._profile_requests(profile)
        t0 = time.perf_counter()
        responses = self.client.simGetImages([req for _, req in named_requests])
        t1 = time.perf_counter()

        values = {
            "front_rgb": None,
            "down_rgb": None,
            "front_depth": None,
            "down_depth": None,
        }
        for idx, (name, _req) in enumerate(named_requests):
            if responses and len(responses) > idx:
                values[name] = self._decode_capture_response(name, responses[idx])
        t2 = time.perf_counter()

        timing = {
            "profile": profile,
            "mode": "batch",
            "rpc_s": t1 - t0,
            "decode_s": t2 - t1,
            "total_s": t2 - t0,
        }
        return values["front_rgb"], values["down_rgb"], values["front_depth"], values["down_depth"], timing

    @staticmethod
    def _body_pose_from_camera_response(response, camera_offset):
        """Recover the vehicle pose paired with an AirSim image response."""
        if response is None:
            return None
        position = getattr(response, "camera_position", None)
        orientation = getattr(response, "camera_orientation", None)
        if position is None or orientation is None:
            return None
        q = orientation
        quat = [float(q.x_val), float(q.y_val), float(q.z_val), float(q.w_val)]
        if not all(math.isfinite(value) for value in quat):
            return None
        rot = R.from_quat(quat).as_matrix()
        camera_world = np.array(
            [float(position.x_val), float(position.y_val), float(position.z_val)],
            dtype=float,
        )
        offset = np.array([float(value) for value in camera_offset[:3]], dtype=float)
        body_world = camera_world - rot @ offset
        siny = 2.0 * (q.w_val * q.z_val + q.x_val * q.y_val)
        cosy = 1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val)
        yaw_deg = math.degrees(math.atan2(siny, cosy))
        return body_world.tolist(), yaw_deg, rot.tolist()

    def capture_planning_views_with_pose(self, camera_offset=(1.0, 0.0, 0.0)):
        """Capture paired front/down RGB and their exact vehicle-frame pose."""
        named_requests = self._profile_requests("front_down")
        t0 = time.perf_counter()
        responses = self.client.simGetImages([request for _, request in named_requests])
        t1 = time.perf_counter()

        front = down = None
        front_response = None
        for index, (name, _request) in enumerate(named_requests):
            if not responses or len(responses) <= index:
                continue
            response = responses[index]
            if name == "front_rgb":
                front_response = response
                front = self._decode_capture_response(name, response)
            elif name == "down_rgb":
                down = self._decode_capture_response(name, response)

        pose = self._body_pose_from_camera_response(front_response, camera_offset)
        if pose is None:
            pose = self.get_pose_full()
        t2 = time.perf_counter()
        timing = {
            "profile": "front_down",
            "mode": "batch_with_pose",
            "rpc_s": t1 - t0,
            "decode_s": t2 - t1,
            "total_s": t2 - t0,
        }
        return front, down, pose[0], pose[1], pose[2], timing

    def _new_aux_client(self):
        """创建一个额外 RPC client，供实验性并发抓图使用。"""
        aux = airsim.MultirotorClient(ip=self._ip, port=self._port) if self._ip else airsim.MultirotorClient(port=self._port)
        return aux

    def _capture_views_parallel(self, profile: str, verbose: bool = False):
        """多 client 并发抓图。"""
        from concurrent.futures import ThreadPoolExecutor

        named_requests = self._profile_requests(profile)

        clients = {}
        for name, _req in named_requests:
            clients[name] = self._new_aux_client()

        def _fetch_one(name, req):
            client = clients[name]
            t0 = time.perf_counter()
            responses = client.simGetImages([req])
            t1 = time.perf_counter()

            value = None
            if responses:
                resp = responses[0]
                value = self._decode_capture_response(name, resp)
            t2 = time.perf_counter()
            return {
                "name": name,
                "value": value,
                "rpc_s": t1 - t0,
                "decode_s": t2 - t1,
                "total_s": t2 - t0,
            }

        t_all0 = time.perf_counter()
        results = {}
        with ThreadPoolExecutor(max_workers=len(named_requests)) as executor:
            futures = {
                name: executor.submit(_fetch_one, name, req)
                for name, req in named_requests
            }
            for name, future in futures.items():
                results[name] = future.result()
        t_all1 = time.perf_counter()

        front_rgb = results.get("front_rgb", {}).get("value")
        down_rgb = results.get("down_rgb", {}).get("value")
        front_depth = results.get("front_depth", {}).get("value")
        down_depth = results.get("down_depth", {}).get("value")

        timing = {
            "wall_s": t_all1 - t_all0,
            "max_rpc_s": max(item["rpc_s"] for item in results.values()),
            "sum_rpc_s": sum(item["rpc_s"] for item in results.values()),
            "sum_decode_s": sum(item["decode_s"] for item in results.values()),
            "profile": profile,
            "mode": "parallel",
            "rpc_s": t_all1 - t_all0,
            "decode_s": sum(item["decode_s"] for item in results.values()),
            "total_s": t_all1 - t_all0,
            "per_request": {
                name: {
                    "rpc_s": round(item["rpc_s"], 3),
                    "decode_s": round(item["decode_s"], 3),
                    "total_s": round(item["total_s"], 3),
                }
                for name, item in results.items()
            },
        }

        if verbose:
            label = f"[TIMING Parallel:{profile}]"
            print(f"{label} wall={timing['wall_s']:.3f}s  max_rpc={timing['max_rpc_s']:.3f}s  sum_rpc={timing['sum_rpc_s']:.3f}s  sum_decode={timing['sum_decode_s']:.3f}s")
            for name, item in timing["per_request"].items():
                print(
                    f"  [Parallel:{name}] rpc={item['rpc_s']:.3f}s  "
                    f"decode={item['decode_s']:.3f}s  total={item['total_s']:.3f}s"
                )
        return front_rgb, down_rgb, front_depth, down_depth, timing

    def capture_views(self, profile: str = "front_down_both_depth", mode: str = "batch", verbose: bool = False):
        """按指定 profile/mode 抓图。"""
        resolved_profile = self.resolve_capture_profile(profile)
        resolved_mode = self.resolve_capture_mode(mode)
        if resolved_mode == "parallel":
            front_rgb, down_rgb, front_depth, down_depth, timing = self._capture_views_parallel(resolved_profile, verbose=verbose)
        else:
            front_rgb, down_rgb, front_depth, down_depth, timing = self._capture_views_batch(resolved_profile)
            if verbose:
                self._print_capture_timing(timing, front_rgb, front_depth, down_depth)
        return front_rgb, down_rgb, front_depth, down_depth, timing

    def get_dual_view(self):
        """兼容旧接口：批量抓取前视+下视 RGB + 前/下视深度。"""
        front_rgb, down_rgb, front_depth, down_depth, _timing = self.capture_views(
            profile="front_down_both_depth",
            mode="batch",
            verbose=True,
        )
        return front_rgb, down_rgb, front_depth, down_depth

    def get_dual_view_parallel_experimental(self):
        """实验接口：4 个独立 client 并发抓取前视/下视 RGB + 深度。"""
        return self.capture_views(
            profile="front_down_both_depth",
            mode="parallel",
            verbose=True,
        )




    def get_dual_view_separate(self):
        """分两次 RPC 获取前视+下视 RGB + 前视深度（每次只请求一个相机）。
        用于排查 simGetImages 多相机请求慢的问题。
        """
        import time as _time
        # 第一次：前视 RGB + 深度
        _t0 = _time.perf_counter()
        resp1 = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, True),
            airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False),
        ])
        _t1 = _time.perf_counter()
        print(f"[TIMING] front RPC={_t1-_t0:.3f}s")
        front_rgb = None
        front_depth = None
        if resp1 and len(resp1) > 0 and resp1[0].image_data_uint8:
            front_rgb = self._decode_rgb_image_bytes(resp1[0].image_data_uint8)
        if resp1 and len(resp1) > 1 and resp1[1].image_data_float:
            r = resp1[1]
            front_depth = np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)

        # 第二次：下视 RGB
        _t2 = _time.perf_counter()
        resp2 = self.client.simGetImages([
            airsim.ImageRequest("down_center", airsim.ImageType.Scene, False, True),
        ])
        _t3 = _time.perf_counter()
        print(f"[TIMING] down RPC={_t3-_t2:.3f}s")
        down_rgb = None
        if resp2 and len(resp2) > 0 and resp2[0].image_data_uint8:
            down_rgb = self._decode_rgb_image_bytes(resp2[0].image_data_uint8)
        print(f"[TIMING] total={_t3-_t0:.3f}s")
        return front_rgb, down_rgb, front_depth


    def warmup_capture(self):
        """AirSim 冷启动预热：第一次 simGetImages 通常很慢（10-20s），
        预抓 RGB + Depth 让 AirSim 初始化渲染管线。
        """
        _t0 = time.perf_counter()
        try:
            self.get_configured_views()
            _elapsed = time.perf_counter() - _t0
            print(f"[AirSim] warmup capture done in {_elapsed:.2f}s")
        except Exception as e:
            print(f"[AirSim] warmup skipped: {e}")

    def execute_waypoints(self, waypoints, velocity=None, use_forward_only=True):
        """执行世界坐标系 waypoints。

        所有 waypoints 都是 [x, y, z] 世界坐标。

        use_forward_only=True（默认）:
            一次 moveOnPathAsync 执行整批世界坐标航点。
            ForwardOnly 让机头沿路径切线连续转向，lookahead 负责平滑过点。

        use_forward_only=False（旧行为，碰撞后退用）:
            逐个 moveToPositionAsync(MaxDegreeOfFreedom)
            waypoints 为 [dx, dy, dz] 机体位移

        Returns (pos_final, yaw_final, collided).
        """
        from config import cfg

        if velocity is None:
            velocity = float(cfg.get("SIM", {}).get("AIRSIM_VELOCITY", 2.0))

        pos, yaw = self.get_pose()
        pos = list(pos)
        collided = False

        move_timeout = int(cfg.get("SIM", {}).get("AIRSIM_MOVE_TIMEOUT", 60))

        if use_forward_only:
            # Ignore waypoints that are effectively the current position.
            active_wps = []
            prev_target = np.array(pos, dtype=float)
            for wp in waypoints:
                cur_target = np.array(wp, dtype=float)
                if float(np.linalg.norm(cur_target - prev_target)) > 0.01:
                    active_wps.append(wp)
                    prev_target = cur_target
            if not active_wps:
                return self.get_pose() + (False,)

            path_world = [tuple(round(v, 1) for v in target) for target in active_wps]
            path_len = 0.0
            prev = np.array(pos, dtype=float)
            for target in active_wps:
                cur = np.array(target, dtype=float)
                path_len += float(np.linalg.norm(cur - prev))
                prev = cur

            configured_lookahead = float(cfg.get("SIM", {}).get("AIRSIM_PATH_LOOKAHEAD", 3.0))
            minimum_lookahead = float(cfg.get("SIM", {}).get("AIRSIM_MIN_PATH_LOOKAHEAD", 0.2))
            lookahead = max(minimum_lookahead, min(configured_lookahead, max(path_len * 0.5, minimum_lookahead)))
            adaptive_lookahead = float(cfg.get("SIM", {}).get("AIRSIM_ADAPTIVE_LOOKAHEAD", 1.0))
            print(
                f"  [Path] {len(active_wps)} world waypoints, moveOnPath "
                f"ForwardOnly lookahead={lookahead:g} adaptive={adaptive_lookahead:g}"
            )
            print(f"    world coords: {path_world}")
            before_pos, before_yaw = self.get_pose()
            collision_before = self.collision_marker()
            path = [airsim.Vector3r(float(wp[0]), float(wp[1]), float(wp[2])) for wp in active_wps]
            used_fallback = False
            try:
                self.client.moveOnPathAsync(
                    path=path,
                    velocity=velocity,
                    timeout_sec=move_timeout,
                    drivetrain=airsim.DrivetrainType.ForwardOnly,
                    yaw_mode=airsim.YawMode(is_rate=False),
                    lookahead=lookahead,
                    adaptive_lookahead=adaptive_lookahead,
                ).join()
            except Exception as e:
                print(f"  [Path] moveOnPath error: {e}; falling back to moveToPosition")
                used_fallback = True

            after_pos, after_yaw = self.get_pose()
            moved = float(np.linalg.norm(np.array(after_pos, dtype=float) - np.array(before_pos, dtype=float)))
            if not used_fallback and path_len > 0.5 and moved < 0.05:
                print("  [Path] moveOnPath produced no displacement; falling back to moveToPosition")
                try:
                    self.client.hoverAsync().join()
                except Exception:
                    pass
                used_fallback = True

            if used_fallback:
                for target in active_wps:
                    self.client.moveToPositionAsync(
                        float(target[0]),
                        float(target[1]),
                        float(target[2]),
                        velocity,
                        timeout_sec=move_timeout,
                        drivetrain=airsim.DrivetrainType.ForwardOnly,
                        yaw_mode=airsim.YawMode(is_rate=False),
                    ).join()
                after_pos, after_yaw = self.get_pose()
                moved = float(np.linalg.norm(np.array(after_pos, dtype=float) - np.array(before_pos, dtype=float)))

            collided = self.has_collision_since(collision_before)
            final_target = active_wps[-1]
            target_error = float(
                np.linalg.norm(np.array(after_pos, dtype=float) - np.array(final_target, dtype=float))
            )
            print(
                f"  [PathResult] points={len(active_wps)} moved={moved:.2f}m "
                f"target_error={target_error:.2f}m "
                f"yaw={before_yaw:.1f}->{after_yaw:.1f} fallback={used_fallback} collided={collided}"
            )

        else:
            # ═══ 旧行为: 逐点 moveToPositionAsync ═══
            for dx, dy, dz in waypoints:
                local_dir = np.array([dx, dy])
                yaw_rad = math.radians(yaw)
                world_dir = np.array([
                    local_dir[0] * math.cos(yaw_rad) - local_dir[1] * math.sin(yaw_rad),
                    local_dir[0] * math.sin(yaw_rad) + local_dir[1] * math.cos(yaw_rad),
                ])
                target = [
                    pos[0] + float(world_dir[0]),
                    pos[1] + float(world_dir[1]),
                    pos[2] + dz,
                ]
                try:
                    print(f"  [Move] ({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}) + ({dx},{dy},{dz}) -> ({target[0]:.1f},{target[1]:.1f},{target[2]:.1f})")
                    result = self.client.moveToPositionAsync(
                        target[0], target[1], target[2], velocity,
                        timeout_sec=move_timeout, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                        yaw_mode=airsim.YawMode(False, 0)
                    ).join()
                    if result:
                        collided = True
                except Exception as e:
                    print(f"  [Move] ⚠️ error: {e}")
                    collided = True
                pos = target

        final_pos, final_yaw = self.get_pose()
        return final_pos, final_yaw, collided

    def execute_actions(self, actions, velocity=2.0):
        """执行原子动作序列（含 yaw 变化）。旧项目风格：rotate_to_yaw + move_to_position。

        actions 格式：["forward 4", "left 30", "forward 5"]
        Returns (pos_final, yaw_final, collided).
        """
        import time as _time

        pos, yaw = self.get_pose()
        collided = False

        for action_str in actions:
            parts = action_str.strip().split()
            if len(parts) < 2:
                continue
            name = parts[0].lower()
            try:
                value = float(parts[1])
            except ValueError:
                continue

            if name in ("left", "right"):
                sign = -1 if name == "left" else 1
                yaw = (yaw + sign * value) % 360.0
                if yaw > 180:
                    yaw -= 360
                print(f"  [Rotate] {name} {value}° → yaw={yaw:.1f}°")
                try:
                    self.rotate_to_yaw(yaw, timeout=5.0)
                except Exception as e:
                    print(f"  [Rotate] ⚠️ error: {e}")
                    collided = True
                _time.sleep(0.2)

            elif name == "forward":
                rad = math.radians(yaw)
                pos[0] += value * math.cos(rad)
                pos[1] += value * math.sin(rad)
                print(f"  [Move] forward {value}m → ({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})")
                try:
                    self.move_to_position(pos[0], pos[1], pos[2], velocity=velocity, timeout=15.0)
                except Exception as e:
                    print(f"  [Move] ⚠️ error: {e}")
                    collided = True

            elif name == "backward":
                rad = math.radians(yaw)
                pos[0] -= value * math.cos(rad)
                pos[1] -= value * math.sin(rad)
                print(f"  [Move] backward {value}m → ({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})")
                try:
                    self.move_to_position(pos[0], pos[1], pos[2], velocity=velocity, timeout=15.0)
                except Exception as e:
                    print(f"  [Move] ⚠️ error: {e}")
                    collided = True

            elif name == "up":
                pos[2] -= value
                print(f"  [Move] up {value}m → ({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})")
                try:
                    self.move_to_position(pos[0], pos[1], pos[2], velocity=velocity, timeout=15.0)
                except Exception as e:
                    print(f"  [Move] ⚠️ error: {e}")
                    collided = True

            elif name == "down":
                pos[2] += value
                print(f"  [Move] down {value}m → ({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})")
                try:
                    self.move_to_position(pos[0], pos[1], pos[2], velocity=velocity, timeout=15.0)
                except Exception as e:
                    print(f"  [Move] ⚠️ error: {e}")
                    collided = True

            _time.sleep(0.2)

        final_pos, final_yaw = self.get_pose()
        return final_pos, final_yaw, collided
