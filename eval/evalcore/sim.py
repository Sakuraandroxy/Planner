"""AirSim-UE4 场景适配层（TravelUAV_env 数据集场景）。

只依赖 airsim 客户端（vln-ue4 环境，msgpack 0.5.6）。一个进程顺序跑，
一次拉起一张图：xvfb-run 无头 + GPU 渲染，等 ApiServerPort 就绪后连接。
与主 Planner 的 projectairsim 完全隔离。
"""
from __future__ import annotations

import copy
import json
import math
import os
import signal
import socket
import subprocess
import tempfile
import time
from contextlib import contextmanager

import airsim
import numpy as np


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _port_open(port: int, host: str = "127.0.0.1") -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        try:
            s.connect((host, port))
            return True
        except OSError:
            return False


def build_settings(api_port: int, cfg) -> dict:
    cw, ch, fov = cfg.capture.width, cfg.capture.height, cfg.capture.fov
    cam = {
        "Pitch": 0, "Roll": 0, "Yaw": 0,
        "CaptureSettings": [
            {"ImageType": 0, "Width": cw, "Height": ch, "FOV_Degrees": fov},
            {"ImageType": 2, "Width": cw, "Height": ch, "FOV_Degrees": fov},
        ],
    }
    front = {"X": 1, "Y": 0, "Z": 0, **copy.deepcopy(cam)}
    down = {"X": 0, "Y": 0, "Z": 0, **copy.deepcopy(cam)}
    down["Pitch"] = -90
    return {
        "SeeDocsAt": "https://microsoft.github.io/AirSim/settings/",
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "ClockSpeed": cfg.clock_speed,
        "LockStep": getattr(cfg, "lockstep", True),
        "ViewMode": "NoDisplay",
        "ApiServerPort": api_port,
        "Vehicles": {
            "Drone_1": {
                "VehicleType": "SimpleFlight",
                "AutoCreate": True,
                "LockStep": False,
                "X": 0, "Y": 0, "Z": 0,
                "Cameras": {"FrontCamera": front, "DownCamera": down},
            }
        },
    }


class Scene:
    """一张已拉起的图 + 一个 AirSim 客户端。"""

    def __init__(self, scene_id: str, sh_path: str, gpu: int, cfg):
        self.scene_id = scene_id
        self.sh_path = sh_path
        self.gpu = gpu
        self.cfg = cfg
        self.proc: subprocess.Popen | None = None
        self.client: airsim.MultirotorClient | None = None
        self.port: int = 0
        self._tmpdir: str | None = None
        self._spawn_cnt = 0
        self._spawned: list[str] = []

    # ---- 生命周期 ----
    def start(self, launch_timeout_s: float = 150.0):
        self.port = _free_port()
        self._tmpdir = tempfile.mkdtemp(prefix="eval_scene_")
        settings_path = os.path.join(self._tmpdir, "settings.json")
        with open(settings_path, "w", encoding="utf-8") as f:
            json.dump(build_settings(self.port, self.cfg), f)
        out_path = os.path.join(self._tmpdir, "ue4.out")

        cmd = [
            "xvfb-run", "-a", "-s", "-screen 0 1280x720x24",
            "bash", self.sh_path,
            f"-settings={settings_path}",
            "-RenderOffscreen", "-NoSound", "-NoVSync",
            f"-GraphicsAdapter={self.gpu}",
            "-ResX=1280", "-ResY=720", "-log",
        ]
        print(f"[sim] launching {self.scene_id} port={self.port} gpu={self.gpu}")
        self._out = open(out_path, "w")
        self.proc = subprocess.Popen(
            cmd, stdout=self._out, stderr=subprocess.STDOUT,
            start_new_session=True,  # 独立进程组，便于整组回收
        )
        t0 = time.time()
        while time.time() - t0 < launch_timeout_s:
            if self.proc.poll() is not None:
                self._dump_log_tail()
                raise RuntimeError(f"场景 {self.scene_id} 进程提前退出")
            if _port_open(self.port):
                break
            time.sleep(1.0)
        else:
            self._dump_log_tail()
            self.stop()
            raise TimeoutError(f"场景 {self.scene_id} 端口 {self.port} 未在 {launch_timeout_s}s 内就绪")

        self.client = airsim.MultirotorClient(ip="127.0.0.1", port=self.port, timeout_value=30)
        for _ in range(30):
            try:
                self.client.confirmConnection()
                break
            except Exception:
                time.sleep(1.0)
        else:
            raise RuntimeError("AirSim confirmConnection 失败")
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        # 关键：先起飞进入 flight 模式，否则 SimpleFlight 会自由落体（本 build 在 ClockSpeed=1 下稳定）
        try:
            self.client.takeoffAsync(timeout_sec=20).join()
            self.client.hoverAsync()
        except Exception as exc:
            print(f"[sim] takeoff 失败（继续）: {exc}")
        print(f"[sim] {self.scene_id} ready ({time.time()-t0:.1f}s)")

    def _dump_log_tail(self, n=25):
        try:
            path = os.path.join(self._tmpdir or "", "ue4.out")
            with open(path, "r", encoding="utf-8", errors="ignore") as f:
                lines = f.readlines()
            print("[sim] --- ue4.out tail ---")
            print("".join(lines[-n:]))
        except Exception:
            pass

    def stop(self):
        try:
            if self.client is not None:
                try:
                    self.client.enableApiControl(False)
                except Exception:
                    pass
                try:
                    self.client.client.close()  # msgpackrpc socket
                except Exception:
                    pass
        except Exception:
            pass
        self.client = None
        if self.proc is not None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except Exception:
                pass
            try:
                self.proc.wait(timeout=10)
            except Exception:
                pass
            self.proc = None
        try:
            self._out.close()
        except Exception:
            pass
        print(f"[sim] {self.scene_id} stopped")

    # ---- 基本操作 ----
    def set_pose(self, position, orientation_xyzw):
        ks = airsim.KinematicsState()
        ks.position = airsim.Vector3r(*position)
        ks.orientation = airsim.Quaternionr(*orientation_xyzw)  # (x,y,z,w)
        ks.linear_velocity = airsim.Vector3r(0.0, 0.0, 0.0)
        ks.angular_velocity = airsim.Vector3r(0.0, 0.0, 0.0)
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.client.simSetKinematics(ks, ignore_collision=True)
        self.client.simContinueForFrames(1)
        # 锁定当前位置为悬停 setpoint，避免漂回上一高度
        self.client.hoverAsync()

    def spawn_object(self, asset_name, position, quat_xyzw, scale):
        name = f"eval_obj_{self._spawn_cnt}"
        pose = airsim.Pose(
            position_val=airsim.Vector3r(*position),
            orientation_val=airsim.Quaternionr(*quat_xyzw),
        )
        vec_scale = airsim.Vector3r(scale, scale, scale)
        try:
            self.client.simSpawnObject(name, asset_name, pose, vec_scale,
                                       physics_enabled=False, is_blueprint=False)
            self._spawned.append(name)
        except Exception as exc:
            print(f"[sim] simSpawnObject 失败 asset={asset_name}: {exc}")
        self.client.simContinueForFrames(1)
        self._spawn_cnt += 1

    def clear_objects(self):
        for name in self._spawned:
            try:
                self.client.simDestroyObject(name)
            except Exception:
                pass
        self._spawned = []

    def get_state(self) -> dict:
        s = self.client.getMultirotorState()
        k = s.kinematics_estimated
        q = k.orientation
        return {
            "position": [q0 for q0 in (k.position.x_val, k.position.y_val, k.position.z_val)],
            "orientation": [q.x_val, q.y_val, q.z_val, q.w_val],
            "linear_velocity": [k.linear_velocity.x_val, k.linear_velocity.y_val, k.linear_velocity.z_val],
        }

    def get_collision(self) -> bool:
        try:
            return bool(self.client.simGetCollisionInfo().has_collided)
        except Exception:
            return False

    def get_rgb(self, cameras=("FrontCamera", "DownCamera")) -> list[np.ndarray]:
        reqs = [airsim.ImageRequest(c, airsim.ImageType.Scene, pixels_as_float=False, compress=False)
                for c in cameras]
        resp = self.client.simGetImages(reqs)
        imgs = []
        for r in resp:
            imgs.append(np.frombuffer(r.image_data_uint8, dtype=np.uint8)
                        .reshape(r.height, r.width, 3))
        return imgs

    def get_rgb_depth(self, cameras=("FrontCamera", "DownCamera")):
        reqs = []
        for c in cameras:
            reqs.append(airsim.ImageRequest(c, airsim.ImageType.Scene, False, False))
            reqs.append(airsim.ImageRequest(c, airsim.ImageType.DepthPerspective, True, False))
        resp = self.client.simGetImages(reqs)
        rgbs, depths = [], []
        for i in range(len(cameras)):
            r = resp[2 * i]
            d = resp[2 * i + 1]
            rgbs.append(np.frombuffer(r.image_data_uint8, dtype=np.uint8).reshape(r.height, r.width, 3))
            depths.append(airsim.list_to_2d_float_array(d.image_data_float, d.width, d.height))
        return rgbs, depths

    def fly(self, world_pts, sample_hz: float = 10.0):
        """逐航点飞过 world_pts（NED 世界点），边飞边采样位置。

        本 build 的 moveOnPathAsync 路径跟踪不稳定，改用逐点 moveToPositionAsync（可靠）。
        返回 (visited_positions, final_state, collided)。
        """
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.client.simPause(False)  # set_pose/spawn 的 simContinueForFrames 会暂停仿真，这里恢复
        m = self.cfg.move
        visited = []
        collided = False
        dt = 1.0 / sample_hz
        wp_timeout = max(5.0, m.chunk_timeout_s / max(1, len(world_pts)))
        for wp in world_pts:
            wp = np.asarray(wp, dtype=float)
            self.client.moveToPositionAsync(
                float(wp[0]), float(wp[1]), float(wp[2]),
                velocity=float(m.velocity),
                drivetrain=airsim.DrivetrainType.ForwardOnly,
                yaw_mode=airsim.YawMode(is_rate=False),
            )
            t0 = time.time()
            while time.time() - t0 < wp_timeout:
                st = self.get_state()
                pos = np.asarray(st["position"], dtype=float)
                visited.append(list(pos))
                if self.get_collision():
                    collided = True
                    break
                if np.linalg.norm(pos - wp) <= m.reach_tol_m:
                    break
                time.sleep(dt)
            if collided:
                break
        self.client.hoverAsync()
        return visited, self.get_state(), collided


@contextmanager
def open_scene(scene_id: str, cfg, gpu: int | None = None):
    """上下文管理器：拉起一张图，退出时关闭。"""
    launchers = cfg.resolve_scenes()
    sh = launchers.get(scene_id)
    if sh is None:
        raise KeyError(f"未知场景 {scene_id}，无启动脚本映射")
    sh_path = os.path.join(cfg.root_path, sh)
    if not os.path.isfile(sh_path):
        raise FileNotFoundError(f"启动脚本不存在: {sh_path}")
    gpu = cfg.gpus[0] if gpu is None else gpu
    scene = Scene(scene_id, sh_path, gpu, cfg)
    scene.start()
    try:
        yield scene
    finally:
        scene.stop()
