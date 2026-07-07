"""Wraps all AirSim API calls into a clean interface."""
import math, io
import numpy as np
from scipy.spatial.transform import Rotation as R
import airsim
from PIL import Image, ImageDraw, ImageFont


class AirSimClient:
    """Singleton-style wrapper around the AirSim MultirotorClient."""

    def __init__(self, ip: str = "", port: int = 41451):
        if not ip:
            from config import cfg
            sim = cfg.get("SIM", {})
            ip = sim.get("AIRSIM_IP", "")
            port = int(sim.get("AIRSIM_PORT", 41451))
        self.client = airsim.MultirotorClient(ip=ip, port=port) if ip else airsim.MultirotorClient(port=port)
        self._connected = False

    def connect(self):
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

    def move_to_position(self, x, y, z, velocity=0.5, timeout=10.0):
        self.client.moveToPositionAsync(x, y, z, velocity, timeout_sec=timeout).join()

    def rotate_to_yaw(self, yaw_deg, timeout=5.0):
        self.client.rotateToYawAsync(yaw_deg, timeout_sec=timeout).join()

    def get_image(self):
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, True)
        ])
        if not responses or not responses[0].image_data_uint8:
            return None
        return Image.open(io.BytesIO(bytes(responses[0].image_data_uint8)))

    def get_scene_and_depth_meters(self):
        """Return RGB frame and raw DepthPerspective meters from one AirSim RPC."""
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, True),
            airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False),
        ])
        frame = None
        depth = None
        if responses and len(responses) > 0 and responses[0].image_data_uint8:
            frame = Image.open(io.BytesIO(bytes(responses[0].image_data_uint8)))
        if responses and len(responses) > 1 and responses[1].image_data_float:
            r = responses[1]
            depth = np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)
        return frame, depth

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

    def cleanup(self):
        try:
            self.client.armDisarm(False)
            self.client.enableApiControl(False)
        except Exception:
            pass


    def get_dual_view(self):
        """一次 RPC 获取前视+下视 RGB + 前视深度。
        Returns (front_rgb, down_rgb, front_depth_meters) 或 (None, None, None)。
        """
        import time as _time
        _t0 = _time.perf_counter()
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, True),
            airsim.ImageRequest("down_center", airsim.ImageType.Scene, False, True),
            airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False),
        ])
        _t1 = _time.perf_counter()
        front_rgb = None
        down_rgb = None
        front_depth = None

        if responses and len(responses) > 0 and responses[0].image_data_uint8:
            front_rgb = Image.open(io.BytesIO(bytes(responses[0].image_data_uint8)))
        if responses and len(responses) > 1 and responses[1].image_data_uint8:
            down_rgb = Image.open(io.BytesIO(bytes(responses[1].image_data_uint8)))
        if responses and len(responses) > 2 and responses[2].image_data_float:
            r = responses[2]
            front_depth = np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)

        if front_rgb is not None and front_depth is not None:
            _t2 = _time.perf_counter()
            print(f"[TIMING] simGetImages={_t1-_t0:.3f}s  decode={_t2-_t1:.3f}s  total={_t2-_t0:.3f}s  "
                  f"rgb={front_rgb.size}  depth={front_depth.shape}")
        elif front_rgb is not None:
            print(f"[TIMING] simGetImages={_t1-_t0:.3f}s  decode w/o depth")
        return front_rgb, down_rgb, front_depth




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
            front_rgb = Image.open(io.BytesIO(bytes(resp1[0].image_data_uint8)))
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
            down_rgb = Image.open(io.BytesIO(bytes(resp2[0].image_data_uint8)))
        print(f"[TIMING] total={_t3-_t0:.3f}s")
        return front_rgb, down_rgb, front_depth


    def warmup_capture(self):
        """AirSim 冷启动预热：第一次 simGetImages 通常很慢（10-20s），
        预抓 RGB + Depth 让 AirSim 初始化渲染管线。
        """
        import time as _time
        _t0 = _time.perf_counter()
        try:
            self.client.simGetImages([
                airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, True),
                airsim.ImageRequest("down_center", airsim.ImageType.Scene, False, True),
                airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False),
            ])
            _elapsed = _time.perf_counter() - _t0
            print(f"[AirSim] warmup capture done in {_elapsed:.2f}s")
        except Exception as e:
            print(f"[AirSim] warmup skipped: {e}")

    def execute_waypoints(self, waypoints, velocity=2.0, use_forward_only=True):
        """执行机体坐标系 waypoints。

        use_forward_only=True（默认，匹配 3DG-VLN）:
            转世界坐标路径 → moveOnPathAsync(ForwardOnly)
            → 无人机自动面朝每个航点方向

        use_forward_only=False（旧行为）:
            逐个 moveToPositionAsync(MaxDegreeOfFreedom)
            → 无人机保持初始朝向

        Returns (pos_final, yaw_final, collided).
        """
        from scipy.spatial.transform import Rotation as R

        start_pos, start_yaw = self.get_pose()
        yaw_rad = math.radians(start_yaw)
        rot_3d = R.from_euler('z', yaw_rad).as_matrix()
        rot_2d = rot_3d[:2, :2]

        pos = list(start_pos)
        collided = False

        if use_forward_only:
            # ═══ 3DG-VLN 风格: moveOnPathAsync(ForwardOnly) ═══
            # 先构建世界坐标路径
            path = []
            for wp in waypoints:
                # 机体坐标 → 世界坐标（waypoints 可能是 [dx,dy,dz] 或 [dx,dy,dz,dyaw], 只取前3维）
                local = np.array(wp[:3], dtype=float)
                world = rot_3d @ local
                target = [pos[0] + float(world[0]),
                          pos[1] + float(world[1]),
                          pos[2] + float(world[2])]
                path.append(airsim.Vector3r(*target))

            if not path:
                return self.get_pose() + (False,)

            print(f"  [Path] {len(path)} waypoints, ForwardOnly")
            try:
                result = self.client.moveOnPathAsync(
                    path=path,
                    velocity=velocity,
                    timeout_sec=max(15, len(path) * 3),
                    drivetrain=airsim.DrivetrainType.ForwardOnly,
                    yaw_mode=airsim.YawMode(is_rate=False),
                    lookahead=3,
                    adaptive_lookahead=1,
                ).join()
                if result:
                    collided = True
            except Exception as e:
                print(f"  [Path] ⚠️ error: {e}")
                collided = True

        else:
            # ═══ 旧行为: 逐点 moveToPositionAsync ═══
            for dx, dy, dz in waypoints:
                local_dir = np.array([dx, dy])
                world_dir = rot_2d @ local_dir
                target = [
                    pos[0] + float(world_dir[0]),
                    pos[1] + float(world_dir[1]),
                    pos[2] + dz,
                ]
                try:
                    print(f"  [Move] ({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}) + ({dx},{dy},{dz}) -> ({target[0]:.1f},{target[1]:.1f},{target[2]:.1f})")
                    result = self.client.moveToPositionAsync(
                        target[0], target[1], target[2], velocity,
                        timeout_sec=15, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
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
