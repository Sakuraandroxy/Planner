"""真实规划器接入：在 vln-ue4 里直接复用 Planner 的 visual-goal 工作流。

这与主 Planner 仓库对 UE4 的接法一致（``config/simulators.yaml`` 的 ``airsim`` 段本来就是
把 mission session 跑在 vln-ue4 子进程里，因为 msgpack 0.5.6 与 1.2.4 冲突）。区别只是：
- eval 负责拉起 TravelUAV 数据集、spawn 目标物、teleport 起点（见 :mod:`evalcore.sim`）；
- 本模块把手里的活着的 AirSim（``scene.port``）交给 Planner 自己的
  ``AirSimMissionSession`` + ``assemble_missions``，装配出**完整** visual-goal 工作流
  （真检测 + 真VLM规划 + 完成判定 + 滑窗续规划），跑数据集的自然语言指令；
- 一个后台线程按 ~5Hz 记录无人机真实位姿写 metric 帧；成功率仍由 ``metric.py``
  按真值距离判定（不采信模型自判"到了"，但会把模型判定的结果记进 result.json）。

运行环境：vln-ue4（自带 airsim 0.5.6 + numpy/httpx/PIL）。Planner 仓库加入 ``sys.path``。
需要环境变量 ``VLN_API_KEY``（deepseek 端点）。
"""
from __future__ import annotations

import os
import sys
import threading
import time

import numpy as np

from .config import REPO_ROOT

# UE4 环境直接从当前检出目录导入规划器。
_PLANNER_ROOT = os.path.abspath(os.path.expanduser(
    os.environ.get("AVLN_PLANNER_ROOT") or REPO_ROOT
))
if _PLANNER_ROOT not in sys.path:
    sys.path.insert(0, _PLANNER_ROOT)

from planner.application.factories import (  # noqa: E402
    build_detector,
    build_geometry,
    build_planner,
)
from planner.application.mission_assembly import (  # noqa: E402
    MissionResources,
    assemble_missions,
)
from planner.config import load_config  # noqa: E402
from planner.domain.runtime import SimulatorFamily  # noqa: E402
from planner.domain.server import TaskSubmission  # noqa: E402
from planner.simulators.airsim.session import AirSimMissionSession  # noqa: E402

from .writer import TrajectoryWriter  # noqa: E402

# 与 evalcore.sim.build_settings 注册的载具 / 相机保持一致。
VEHICLE_NAME = "Drone_1"
CAMERA_ID = "FrontCamera"


def _log(message, level="INFO"):
    print(f"[planner {level}] {message}")


class _NullTelemetry:
    """工作流的遥测汇（剩余轨迹面板）。

    评测不读这个面板——我们用自己的位姿采样线程写 metric 帧——所以这里全部空实现。
    用 ``__getattr__`` 兜底，工作流调用任何遥测方法都不会因为它多了一个方法而崩。
    """

    def publish_queue(self, **kwargs):
        return None

    def publish_no_queue(self, **kwargs):
        return None

    def __getattr__(self, name):
        def _noop(*args, **kwargs):
            return None

        return _noop


def build_family(scene) -> SimulatorFamily:
    """把 eval 已拉起的 TravelUAV 数据集，包成 Planner 认的 SimulatorFamily。"""
    return SimulatorFamily(
        id="airsim-ue4",
        label="AirSim (UE4, TravelUAV eval)",
        host="127.0.0.1",
        ports={"rpc": int(scene.port)},
        vehicle_name=VEHICLE_NAME,
        camera_id=CAMERA_ID,
        timeout_s=15.0,
        # 我们不通过这个 family 启动仿真（数据集由 evalcore.sim 拉起，这里只连端口），
        # 但 SimulatorFamily 的校验要求 argv/stop_argv 非空——给占位即可，永不渲染。
        argv=("true",),
        stop_argv=("true",),
    )


def build_planner_config(cfg):
    """加载主界面当前 profile 的 core 配置，按需覆盖成 eval 的口径。

    profile 是主界面「世界模型 / 普通 VLM」的只读镜像（default / vlm），起评测时由
    dashboard 读取主服务 /api/profile 后经 ``--planner-profile`` 透传；读不到则用 default。
    eval 的相机 rig（FrontCamera 偏移 (1,0,0)、rpy (0,0,0)、hfov 90、AirSim 深度是
    ray_range）恰好与 profile 的无人机默认值一致，所以这里基本是零覆盖；只把 eval 侧可调
    的滑窗开关透传下去（滑窗关闭时工作流走 stopped-segments 模式，更简单，便于对照）。
    """
    profile = getattr(cfg.planner, "profile", "") or "default"
    base = os.path.join(_PLANNER_ROOT, "config", "core", profile, "base.yaml")
    if not os.path.isfile(base):
        _log(f"找不到 profile '{profile}' 的配置 ({base})，回退 default", "WARNING")
        profile = "default"
        base = os.path.join(_PLANNER_ROOT, "config", "core", "default", "base.yaml")
    app = load_config(base)
    if getattr(cfg.planner, "disable_sliding_window", False):
        # 数据类多为 frozen，用 object.__setattr__ 覆盖单个字段。
        try:
            object.__setattr__(app.sliding_window, "enabled", False)
        except Exception as exc:  # 覆盖失败不应阻断，记录后沿用默认
            _log(f"无法关闭滑窗，沿用默认: {exc}", "WARNING")
    return app


class _RealStack:
    """一张图上的完整规划器栈：会话 + 装配好的工作流。"""

    def __init__(self, scene, cfg):
        self.scene = scene
        self.family = build_family(scene)
        self.config = build_planner_config(cfg)
        self.session = AirSimMissionSession(self.config, self.family, log=_log)
        resources = MissionResources(
            detector=build_detector(self.config, self.config.server.detector),
            planner=build_planner(self.config, self.config.server.planner),
            geometry=build_geometry(self.config),
            trajectory=_NullTelemetry(),
        )
        arms = assemble_missions(
            self.config,
            inputs=self.session.mission_inputs(),
            resources=resources,
            log=_log,
        )
        if "drone" not in arms:
            raise RuntimeError("mission assembly returned no drone arm")
        # 关键：把装配好的臂交给会话，否则 session._arm 不存在，run() 一调用就崩。
        # 与主服务 registry.build_session 的 session.attach_arms(assemble_missions(...)) 一致。
        self.session.attach_arms(arms)
        self.arm = arms["drone"]

    def run(self, instruction):
        task = TaskSubmission(task_id="eval", instruction=instruction, vehicle="drone")
        return self.session.run(task, log=lambda m, level="INFO": _log(m, level))


class _PoseSampler:
    """后台按固定频率记录无人机真实位姿，写 metric 能吃的帧。"""

    def __init__(self, scene, writer: TrajectoryWriter, hz: float = 5.0,
                 thumb_every: int = 5):
        self.scene = scene
        self.writer = writer
        self.period = 1.0 / max(1e-3, float(hz))
        self.thumb_every = max(1, int(thumb_every))
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._count = 0
        self.collided = False
        self.last_state: dict | None = None

    def start(self):
        self._thread = threading.Thread(target=self._loop, name="eval-pose-sampler",
                                        daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            try:
                state = self.scene.get_state()
                coll = self.scene.get_collision()
                self.collided = self.collided or coll
                self.last_state = state
                self.writer.write_frame(state, collision=coll)
                if self._count % self.thumb_every == 0:
                    self._save_thumbs()
                self._count += 1
            except Exception as exc:  # 单帧读取失败不致命
                _log(f"采样帧失败: {exc}", "WARNING")
            self._stop.wait(self.period)

    def _save_thumbs(self):
        try:
            import cv2
            rgbs = self.scene.get_rgb()
        except Exception:
            return
        for name, idx in (("frame_front.png", 0), ("frame_down.png", 1)):
            if idx < len(rgbs) and rgbs[idx] is not None:
                try:
                    cv2.imwrite(os.path.join(self.writer.dir, name),
                                cv2.cvtColor(rgbs[idx], cv2.COLOR_RGB2BGR))
                except Exception:
                    pass

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)


# 缓存：一张图只装配一次栈（会话持有连接，跨 episode 复用；进程结束时销毁）。
_STACKS: dict[int, _RealStack] = {}


def _get_stack(scene, cfg) -> _RealStack:
    key = id(scene)
    stack = _STACKS.get(key)
    if stack is None:
        stack = _RealStack(scene, cfg)
        _STACKS[key] = stack
    return stack


def close_scene(scene) -> None:
    """关闭并移除某张图的栈（在 kill 仿真进程之前调用最安全）。"""
    stack = _STACKS.pop(id(scene), None)
    if stack is None:
        return
    try:
        stack.session.close()
    except Exception as exc:
        _log(f"关闭规划器会话失败: {exc}", "WARNING")


def _ne(pos, target) -> float:
    return float(np.linalg.norm(np.asarray(pos, dtype=float) - np.asarray(target, dtype=float)))


def run_episode_real(scene, episode, cfg, out_root: str, on_step=None) -> dict:
    """用真实规划器在 pygame 已拉起的 scene 上跑一个 episode。"""
    stack = _get_stack(scene, cfg)

    # episode 布置：清场 → 回起点 → 生成目标物。
    scene.clear_objects()
    scene.set_pose(episode.start_position, episode.start_orientation)
    diag = {"after_set_pose": scene.collision_detail()}
    scene.spawn_object(episode.asset_name, episode.object_position,
                       episode.object_quat_xyzw, episode.object_scale)
    diag["after_spawn"] = scene.collision_detail()
    # set_pose/spawn 用 simContinueForFrames 会暂停仿真；交给规划器前必须恢复。
    try:
        scene.client.simPause(False)
    except Exception:
        pass
    # 瞬移会让飞控失去控制，恢复运行后无人机会直接下落撞地；须重新起飞进入悬停。
    try:
        scene.client.enableApiControl(True)
        scene.client.armDisarm(True)
        scene.client.takeoffAsync(timeout_sec=20).join()
        scene.client.hoverAsync()
    except Exception as exc:
        _log(f"重新起飞失败（继续）: {exc}", "WARNING")

    writer = TrajectoryWriter(out_root, episode.uuid, episode.ori_traj_dir)
    sampler = _PoseSampler(scene, writer, hz=getattr(cfg.planner, "sampler_hz", 5.0))
    sampler.start()

    mission = None
    error = None
    t0 = time.time()
    try:
        mission = stack.run(episode.instruction)
    except Exception as exc:  # 单集失败不影响整批
        error = str(exc)
        _log(f"episode {episode.uuid} 规划器执行失败: {exc}", "ERROR")
    finally:
        sampler.stop()

    final_state = sampler.last_state or scene.get_state()
    collided = bool(sampler.collided or scene.get_collision())
    ne = _ne(final_state["position"], episode.object_position)
    success = (ne <= cfg.success_radius_m) and not collided
    mission_success = bool(getattr(mission, "success", False)) if mission else False

    summary = {
        "uuid": episode.uuid,
        "scene": episode.scene,
        "instruction": episode.instruction,
        "success": success,
        "collided": collided,
        "final_ne": ne,
        "elapsed_s": time.time() - t0,
        "target_position": episode.object_position,
        "planner_profile": cfg.planner.profile,
        "mission_success": mission_success,
        "mission_error": error,
        "collision_diag": diag,
        "expert_positions": episode.gt_positions,
    }
    out_dir = writer.finalize(success, summary)
    summary["dir"] = out_dir
    if on_step is not None:
        on_step(step=0, ne=ne, collided=collided)
    return summary
