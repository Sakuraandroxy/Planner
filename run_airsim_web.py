#!/usr/bin/env python3
"""run_airsim_web.py — 入口。只负责组装模块、启动服务、运行主循环。

基础设施都在各自模块里：
  - API 预热     → agent/common/warmup.py
  - 图像编码缓存  → agent/common/image_encoder.py
  - 抓帧         → sim/capture.py
  - 后台取帧     → sim/frame_capturer.py
  - 步级指标     → 本文件的 StepTracker（含 SR/NE/SPL/OSR/TL 论文指标）
"""

import contextlib
import io
import os, sys, math, time, threading, socket
from concurrent.futures import ThreadPoolExecutor

_script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _script_dir)

from config import cfg, get_cfg
get_cfg(os.path.join(_script_dir, "config", "default.yaml"))

from agent.detector import build_detector
from agent.planner import build_planner
from agent.direction import build_direction_estimator
from agent.task_parser import build_task_parser
from agent.world_model import build_world_model
from agent.candidate import prepare_candidates_for_world_model
from agent.completion import build_task_completion_checker, run_completion_and_planning
from agent.common.trajectory import actions_to_cumulative_body_waypoints
from agent.common.task_manager import TaskManager
from agent.common.warmup import warmup_from_config
from agent.common.image_encoder import ImageEncoder
from agent.relocalization import TargetRelocalizer
from agent.trajectory_refiner import TrajectoryRefiner
from agent.collision_recovery import CollisionRecovery
from sim.airsim_client import AirSimClient
from sim.frame_capturer import FrameCapturer
from web.shared_state import SharedState
from web.app import create_app


# ═══════════════════════════════════════════════════
#  步级指标（含 SR / NE / SPL / OSR / TL 论文指标）
# ═══════════════════════════════════════════════════

class StepTracker:
    """追踪每步和全局指标，含 3DG-VLN 论文指标。"""

    def __init__(self, success_radius: float = 12.0):
        self.success_radius = success_radius
        self.reset()

    def reset(self):
        self.steps = 0
        self.cumulative_distance = 0.0
        self.cumulative_vlm_time = 0.0
        self.cumulative_exec_time = 0.0
        self.total_start = time.time()
        self.last_position = None
        self.start_position = None
        self.step_times = []
        self.detection_confs = []
        self.target_depths = []

        # ── 论文指标 ──
        self.target_world_pos = None       # [x, y, z] 估计目标世界坐标
        self._ever_in_radius = False       # OSR
        self.positions = []                # 每步位置，用于 TL

    def set_target_from_detection(self, drone_pos: list, yaw_deg: float,
                                   detection, image_size=None) -> bool:
        """Estimate the target world coordinate from the accepted visual target.

        Returns True if the target was set for the first time.
        """
        first_set = self.target_world_pos is None
        if not detection or not detection.visible or not detection.depth_median:
            return False
        if not detection.bbox:
            return False
        from config import cfg
        depth = float(detection.depth_median)
        camera = getattr(detection, "camera", "front")
        if camera == "front":
            # bbox 中心偏移角（粗略：bbox 中心相对图像中心的水平偏角）
            width = float(image_size[0]) if image_size else float(cfg.get("SIM", {}).get("FRONT_WIDTH", 1920))
            fov_deg = float(cfg.get("SIM", {}).get("FRONT_FOV", 90))
            cx_img = width / 2.0
            fx = cx_img / max(math.tan(math.radians(fov_deg / 2.0)), 1e-6)
            bbox_cx = (detection.bbox[0] + detection.bbox[2]) / 2.0
            bearing_deg = math.degrees(math.atan2(bbox_cx - cx_img, fx))

            target_yaw = math.radians(yaw_deg + bearing_deg)
            self.target_world_pos = [
                drone_pos[0] + depth * math.cos(target_yaw),
                drone_pos[1] + depth * math.sin(target_yaw),
                drone_pos[2],
            ]
            return first_set

        if camera == "down":
            width = float(image_size[0]) if image_size else float(cfg.get("SIM", {}).get("DOWN_WIDTH", 1024))
            height = float(image_size[1]) if image_size else float(cfg.get("SIM", {}).get("DOWN_HEIGHT", 1024))
            fov_deg = float(cfg.get("SIM", {}).get("DOWN_FOV", 90))
            half_w = width / 2.0
            half_h = height / 2.0
            focal = half_w / max(math.tan(math.radians(fov_deg / 2.0)), 1e-6)
            bbox_cx = (detection.bbox[0] + detection.bbox[2]) / 2.0
            bbox_cy = (detection.bbox[1] + detection.bbox[3]) / 2.0
            right_m = (bbox_cx - half_w) * depth / max(focal, 1e-6)
            forward_m = (bbox_cy - half_h) * depth / max(focal, 1e-6)
            yaw = math.radians(yaw_deg)
            world_dx = forward_m * math.cos(yaw) - right_m * math.sin(yaw)
            world_dy = forward_m * math.sin(yaw) + right_m * math.cos(yaw)
            self.target_world_pos = [
                drone_pos[0] + world_dx,
                drone_pos[1] + world_dy,
                drone_pos[2] + depth,
            ]
            return first_set

        return False

    def ensure_start_position(self, pos: list):
        """Record the trajectory start before the first movement."""
        if pos and self.start_position is None:
            self.start_position = list(pos)
            self.last_position = list(pos)
            self.positions.append(list(pos))

    def record(self, timing: dict, pos: list, detection=None):
        self.steps += 1
        t = timing.get("step_total", 0.0)
        self.step_times.append(t)
        self.cumulative_vlm_time += timing.get("vlm_total", 0.0)
        self.cumulative_exec_time += timing.get("execute", 0.0)

        if detection and detection.visible:
            self.detection_confs.append(detection.score)
            if detection.depth_median:
                self.target_depths.append(detection.depth_median)

        if pos:
            self.positions.append(list(pos))
            if self.start_position is None:
                self.start_position = list(pos)
            if self.last_position:
                d = math.sqrt((pos[0]-self.last_position[0])**2 +
                              (pos[1]-self.last_position[1])**2 +
                              (pos[2]-self.last_position[2])**2)
                self.cumulative_distance += d
            self.last_position = list(pos)

        # OSR
        if self.target_world_pos and pos:
            dist = self._dist_to_target(pos)
            if dist <= self.success_radius:
                self._ever_in_radius = True

    def _dist_to_target(self, pos: list) -> float:
        if self.target_world_pos is None:
            return float("inf")
        return math.sqrt((pos[0]-self.target_world_pos[0])**2 +
                         (pos[1]-self.target_world_pos[1])**2 +
                         (pos[2]-self.target_world_pos[2])**2)

    # ── 论文指标 ──
    @property
    def NE(self) -> float:
        """Navigation Error: 终点到目标的距离 (m)。"""
        if not self.positions or self.target_world_pos is None:
            return float("inf")
        return self._dist_to_target(self.positions[-1])

    @property
    def SR(self) -> bool:
        """Success Rate: NE ≤ success_radius。"""
        return self.NE <= self.success_radius

    @property
    def OSR(self) -> bool:
        """Oracle Success Rate: 过程中是否曾进入成功半径。"""
        return self._ever_in_radius

    @property
    def TL(self) -> float:
        """Trajectory Length: 总路径长度 (m)。"""
        return self.cumulative_distance

    @property
    def SL(self) -> float:
        """Straight-Line distance: 起点到目标直线距离 (m)。"""
        if self.start_position is None or self.target_world_pos is None:
            return 0.0
        return math.sqrt(
            (self.start_position[0]-self.target_world_pos[0])**2 +
            (self.start_position[1]-self.target_world_pos[1])**2 +
            (self.start_position[2]-self.target_world_pos[2])**2)

    @property
    def SPL(self) -> float:
        """Success weighted by Path Length。"""
        if not self.SR:
            return 0.0
        ref = self.SL
        if ref <= 0:
            return 0.0
        return ref / max(self.TL, ref)

    # ── 轻量指标 ──
    @property
    def elapsed(self): return time.time() - self.total_start
    @property
    def avg_step_time(self):
        return sum(self.step_times)/len(self.step_times) if self.step_times else 0.0
    @property
    def avg_speed(self):
        return self.cumulative_distance/self.elapsed if self.elapsed > 0 else 0.0
    @property
    def vlm_ratio(self):
        s = sum(self.step_times)
        return self.cumulative_vlm_time/s if s > 0 else 0.0
    @property
    def avg_conf(self):
        return sum(self.detection_confs)/len(self.detection_confs) if self.detection_confs else 0.0

    def print_step(self, step, max_steps, timing):
        t = timing.get("step_total", 0.0)
        parts = [f"total={t:.2f}s", f"cap={timing.get('capture_rpc',0):.2f}s"]
        if timing.get("detect_api"): parts.append(f"det={timing['detect_api']:.2f}s")
        if timing.get("planning_api"): parts.append(f"plan={timing['planning_api']:.2f}s")
        if timing.get("vlm_total", 0) > 0: parts.append(f"vlm={timing['vlm_total']:.2f}s")
        if timing.get("execute"): parts.append(f"exec={timing['execute']:.2f}s")
        print(f"  [TIMING Step {step}/{max_steps}] " + "  ".join(parts))

        m = [f"dist={self.cumulative_distance:.1f}m", f"spd={self.avg_speed:.1f}m/s",
             f"vlm%={self.vlm_ratio*100:.0f}%"]
        if self.target_world_pos:
            ne = self._dist_to_target(self.positions[-1]) if self.positions else float("inf")
            m.append(f"NE={ne:.1f}m")
        if self.detection_confs: m.append(f"conf={self.avg_conf:.2f}")
        if self.target_depths:
            recent = self.target_depths[-3:]
            m.append(f"z={'→'.join(f'{d:.1f}' for d in recent)}m")
        print(f"  [METRICS Step {step}] " + "  ".join(m))

    def print_final(self):
        print("\n" + "=" * 55)
        print("  📊 导航全程汇总")
        print("=" * 55)
        print(f"  总步数:             {self.steps}")
        print(f"  总耗时:             {self.elapsed:.1f}s")
        print(f"  累计飞行距离 (TL):  {self.TL:.1f}m")
        print(f"  直线距离 (SL):      {self.SL:.1f}m")
        print(f"  平均速度:           {self.avg_speed:.1f}m/s")
        print(f"  平均每步:           {self.avg_step_time:.2f}s")
        print(f"  VLM 总耗时:         {self.cumulative_vlm_time:.1f}s ({self.vlm_ratio*100:.0f}%)")
        print(f"  执行总耗时:         {self.cumulative_exec_time:.1f}s")
        if self.detection_confs:
            print(f"  检测置信度:         avg={self.avg_conf:.2f} "
                  f"(min={min(self.detection_confs):.2f} max={max(self.detection_confs):.2f})")
        if self.target_depths:
            print(f"  目标深度变化:       {self.target_depths[0]:.1f}m → {self.target_depths[-1]:.1f}m")
        print("─" * 55)
        print(f"  📐 论文指标 (success_radius={self.success_radius}m)")
        print(f"  NE  (导航误差):     {self.NE:.2f}m")
        print(f"  SR  (成功率):       {'✅ 成功' if self.SR else '❌ 失败'}  (NE ≤ {self.success_radius}m)")
        print(f"  OSR (宽松成功率):   {'✅ 曾经进入' if self.OSR else '❌ 从未进入'}")
        if self.target_world_pos:
            print(f"  SPL (路径效率):     {self.SPL*100:.1f}%  (SL={self.SL:.1f}m / max(TL,SL))")
        else:
            print(f"  SPL (路径效率):     N/A  (未锁定目标世界坐标)")
        print("=" * 55 + "\n")


# ═══════════════════════════════════════════════════
#  主循环
# ═══════════════════════════════════════════════════

def _shape_text(value):
    if value is None:
        return "N/A"
    if hasattr(value, "shape"):
        return str(value.shape)
    if hasattr(value, "size") and not isinstance(value, (list, tuple)):
        return str(value.size)
    return str(value)


def _target_depth_text(name, detection, image, depth_meters) -> str:
    if detection is None or not getattr(detection, "visible", False) or not getattr(detection, "bbox", None):
        return f"{name}:not_visible"
    if image is None or depth_meters is None:
        return (
            f"{name}:bbox={getattr(detection, 'bbox', None)} "
            f"score={float(getattr(detection, 'score', 0.0) or 0.0):.2f} depth=N/A"
        )
    try:
        import numpy as _np

        h, w = depth_meters.shape
        sw = w / float(image.width)
        sh = h / float(image.height)
        bbox = list(getattr(detection, "bbox", []) or [])
        db = [
            max(0, min(w - 1, int(bbox[0] * sw))),
            max(0, min(h - 1, int(bbox[1] * sh))),
            max(0, min(w - 1, int(bbox[2] * sw))),
            max(0, min(h - 1, int(bbox[3] * sh))),
        ]
        if db[2] < db[0]:
            db[0], db[2] = db[2], db[0]
        if db[3] < db[1]:
            db[1], db[3] = db[3], db[1]
        cx = max(0, min(w - 1, (db[0] + db[2]) // 2))
        cy = max(0, min(h - 1, (db[1] + db[3]) // 2))
        center_depth = float(depth_meters[cy, cx])
        region = depth_meters[db[1]:db[3] + 1, db[0]:db[2] + 1]
        valid = region[_np.isfinite(region)]
        valid = valid[valid > 0]
        median_depth = float(_np.median(valid)) if valid.size else float("nan")
        detection.depth_median = center_depth
        detection.depth_bbox = db
        median_text = f"{median_depth:.1f}m" if _np.isfinite(median_depth) else "N/A"
        return (
            f"{name}:bbox={bbox} score={float(getattr(detection, 'score', 0.0) or 0.0):.2f} "
            f"center={center_depth:.1f}m median={median_text} depth_bbox={db}"
        )
    except Exception as exc:
        return (
            f"{name}:bbox={getattr(detection, 'bbox', None)} "
            f"score={float(getattr(detection, 'score', 0.0) or 0.0):.2f} depth=ERR({exc})"
        )


def _push_pil_png_to_frontend(state, frame, *, view: str = "front"):
    """Publish the freshly captured PIL frame used by the control loop."""
    if state is None or frame is None:
        return
    from io import BytesIO as _BytesIO

    buf = _BytesIO()
    frame.save(buf, format="PNG")
    if view == "down":
        state.set_down_frame(buf.getvalue())
    else:
        state.set_frame(buf.getvalue())


def _depth_only_profile(profile: str) -> str:
    normalized = (profile or "").strip().lower()
    mapping = {
        "front_depth": "front_depth_only",
        "front_down_front_depth": "front_depth_only",
        "front_down_both_depth": "front_down_depth_only",
    }
    return mapping.get(normalized, normalized)


def _capture_profile_isolated(client: AirSimClient, profile: str):
    """Capture one profile through an isolated client to avoid nested parallel RPC noise."""
    aux = AirSimClient(ip=getattr(client, "_ip", ""), port=getattr(client, "_port", 41451), use_config_ip=False)
    return aux.capture_views(profile=profile, mode="batch", verbose=False)


@contextlib.contextmanager
def _pause_background_capture(capturer):
    enabled = bool(cfg.get("SIM", {}).get("PAUSE_BACKGROUND_CAPTURE_DURING_STEP", True))
    if not enabled or capturer is None or not hasattr(capturer, "pause"):
        yield
        return

    capturer.pause(wait=True, timeout=2.0)
    try:
        yield
    finally:
        capturer.resume()


def _landed_state_text(value) -> str:
    if value is None:
        return "unknown"
    return str(value)


def _should_takeoff(client: AirSimClient) -> tuple[bool, list, float, str]:
    pos, yaw = client.get_pose()
    landed_text = "unknown"
    should_takeoff = pos[2] > -0.5
    try:
        state = client.get_multirotor_state()
        landed_state = getattr(state, "landed_state", None)
        landed_text = _landed_state_text(landed_state)
        try:
            import airsim  # type: ignore
            if landed_state == airsim.LandedState.Landed:
                should_takeoff = True
            elif landed_state == airsim.LandedState.Flying:
                should_takeoff = False
        except Exception:
            normalized = landed_text.strip().lower()
            if normalized in {"0", "landed", "landedstate.landed"}:
                should_takeoff = True
            elif normalized in {"1", "flying", "landedstate.flying"}:
                should_takeoff = False
    except Exception:
        pass
    return should_takeoff, pos, yaw, landed_text


def _format_waypoints(waypoints, max_items=5):
    compact = []
    for wp in waypoints[:max_items]:
        compact.append([round(float(v), 2) for v in wp[:3]])
    suffix = " ..." if len(waypoints) > max_items else ""
    return f"{compact}{suffix}"


def _print_candidate_ranking(candidates, topk=5):
    if not candidates:
        return
    print(f"  [TopK Trajectory] showing {min(len(candidates), max(1, topk))}/{len(candidates)}")
    for idx, cand in enumerate(candidates[:max(1, topk)]):
        marker = "√" if cand.get("selected_by_world_model") or cand.get("selected_by_prescore") else " "
        pre_score = float(cand.get("pre_score", 0.0) or 0.0)
        confidence = float(cand.get("confidence", 0.0) or 0.0)
        source = cand.get("source", "")
        body_wp = _format_waypoints(cand.get("waypoints", []), max_items=3)
        print(
            f"    [{marker}] #{idx + 1} score={pre_score:.4f} conf={confidence:.2f} "
            f"source={source} wp={body_wp}"
        )


def _shutdown_executor(executor, wait: bool):
    if executor is None:
        return
    try:
        executor.shutdown(wait=wait, cancel_futures=not wait)
    except TypeError:
        executor.shutdown(wait=wait)


def _connect_web_airsim_client():
    import logging

    def _error_text(exc: Exception) -> str:
        msg = str(exc).strip()
        return msg if msg else exc.__class__.__name__

    sim_cfg = cfg.get("SIM", {})
    wait_enabled = bool(sim_cfg.get("WAIT_FOR_AIRSIM_ON_WEB_START", True))
    retry_interval = float(sim_cfg.get("WAIT_FOR_AIRSIM_INTERVAL", 2.0))
    rpc_timeout = float(sim_cfg.get("AIRSIM_CONNECT_TIMEOUT", 3.0))
    host = str(sim_cfg.get("AIRSIM_IP", "") or "127.0.0.1").strip() or "127.0.0.1"
    port = int(sim_cfg.get("AIRSIM_PORT", 41451))

    logging.getLogger("tornado.general").setLevel(logging.CRITICAL)
    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.settimeout(1.0)
            if sock.connect_ex((host, port)) != 0:
                raise ConnectionError(f"tcp {host}:{port} not ready")
            client_ip = "" if host in {"127.0.0.1", "localhost"} else host
            probe_client = AirSimClient(
                ip=client_ip,
                port=port,
                use_config_ip=False,
                timeout_value=rpc_timeout,
            )
            try:
                probe_client.connect(False)
            except Exception as exc:
                raise TimeoutError(
                    f"AirSim RPC handshake timeout/error after {rpc_timeout:.1f}s: {_error_text(exc)}"
                ) from exc
            return AirSimClient(ip=client_ip, port=port, use_config_ip=False)
        except Exception as exc:
            if not wait_enabled:
                raise
            print(f"[AirSim] waiting for {host}:{port} ... ({_error_text(exc)})", flush=True)
            time.sleep(max(0.5, retry_interval))
        finally:
            try:
                sock.close()
            except Exception:
                pass

def main_loop(state, initial_task="", max_steps=999, client=None, capturer=None):
    import time as _time

    detector = build_detector()
    planner = build_planner()
    world_model = build_world_model()
    direction_est = build_direction_estimator()
    completion_checker = build_task_completion_checker(detector=detector, direction_estimator=direction_est)
    relocalizer = TargetRelocalizer(cfg, detector=detector)
    task_manager = TaskManager(enabled=True)
    tracker = StepTracker(success_radius=cfg["AGENT"]["STOP_DEPTH_THRESHOLD"])
    tracker_stage_key = None

    stop_threshold = cfg["AGENT"]["STOP_DEPTH_THRESHOLD"]
    relocalization_enabled = bool(cfg.get("RELOCALIZATION", {}).get("ENABLED", True))
    trajectory_refiner_enabled = bool(cfg.get("TRAJECTORY_REFINER", {}).get("ENABLED", False))
    trajectory_refiner = (
        TrajectoryRefiner(stop_threshold=stop_threshold)
        if trajectory_refiner_enabled
        else None
    )
    collision_recovery = CollisionRecovery.from_config(cfg)
    detector_min_confidence = float(cfg.get("AGENT", {}).get(
        "DETECTOR_MIN_CONFIDENCE",
        cfg.get("AGENT", {}).get("DETECTOR_BOX_THRESHOLD", 0.0),
    ))
    completion_call_mode = str(cfg.get("TASK_COMPLETION", {}).get("CALL_MODE", "sync_preplan"))
    split_rgb_depth = bool(cfg.get("TASK_COMPLETION", {}).get("SPLIT_RGB_DEPTH_CAPTURE", True))
    capture_mode = client.resolve_capture_mode()
    print(f"[CompletionConfig] name={getattr(completion_checker, 'name', type(completion_checker).__name__)} "
          f"enabled={completion_checker.enabled} call_mode={completion_call_mode}")
    print(f"[CaptureConfig] mode={capture_mode} rgb_profile={completion_checker.rgb_profile} "
          f"completion_profile={completion_checker.depth_profile}")
    if getattr(completion_checker, "uses_detector", False) and not completion_checker.is_detector_enabled():
        print("[Completion] detector disabled: target/detect stages will not auto-complete from visual evidence")

    try:
        cur_task = initial_task.strip()
        if cur_task:
            print("[TASK PARSER] parsing task...")
            t_parse_start = _time.perf_counter()
            task_parser = build_task_parser()
            parsed = task_parser.parse(cur_task)
            t_parse_elapsed = _time.perf_counter() - t_parse_start
            if parsed:
                task_manager.start_with_stages(cur_task, parsed)
                stage = task_manager.current_stage()
                print(f"[TASK PARSER] parsed {len(parsed)} stages in {t_parse_elapsed:.2f}s")
                print(f"[TASK] {task_manager.summary()}")
                if stage:
                    print(f"[TASK] current: {stage.instruction} (target={stage.target_query}, mode={stage.mode})")
            else:
                print(f"[TASK PARSER] no stages returned in {t_parse_elapsed:.2f}s")
                task_manager.start(cur_task)

        step = 0
        while step < max_steps:
            step += 1
            step_started = _time.perf_counter()
            step_timing = {}
            print(f"\n[Step {step}/{max_steps}]")

            with _pause_background_capture(capturer):
                current_stage = task_manager.current_stage()
                # 用任务解析器的英文输出作为 planner 的 instruction
                stage_instruction = (current_stage.instruction
                                     if current_stage else cur_task)
                if current_stage:
                    print(f"  stage {current_stage.index+1}/{len(task_manager.stages)} "
                          f"mode={current_stage.mode}: {stage_instruction}")
                    if current_stage.mode == "target":
                        stage_key = (current_stage.index, current_stage.instruction)
                        if tracker_stage_key != stage_key:
                            tracker.reset()
                            tracker_stage_key = stage_key

                state.update(step=step, status="capturing", error="")

                # === 0.9 action 阶段直接执行，不调用 VLM ===
                if current_stage and current_stage.mode == "action":
                    act = current_stage.action
                    val = current_stage.value or 0
                    print(f"  [Action] direct execution: {act} {val}")
                    pos_before, yaw_before = client.get_pose()
                    try:
                        if act in ("left", "right"):
                            sign = 1 if act == "right" else -1
                            client.rotate_to_yaw(yaw_before + sign * val)
                        elif act == "forward":
                            rad = math.radians(yaw_before)
                            client.move_to_position(
                                pos_before[0] + val * math.cos(rad),
                                pos_before[1] + val * math.sin(rad),
                                pos_before[2])
                        elif act == "backward":
                            rad = math.radians(yaw_before)
                            client.move_to_position(
                                pos_before[0] - val * math.cos(rad),
                                pos_before[1] - val * math.sin(rad),
                                pos_before[2])
                        elif act == "up":
                            client.move_to_position(pos_before[0], pos_before[1], pos_before[2] - val)
                        elif act == "down":
                            client.move_to_position(pos_before[0], pos_before[1], pos_before[2] + val)
                    except Exception as e:
                        print(f"  [Action] error: {e}")
                    pos_after, yaw_after = client.get_pose()
                    print(f"  [Action] from: ({pos_before[0]:.1f}, {pos_before[1]:.1f}, {pos_before[2]:.1f}) yaw={yaw_before:.1f}°")
                    print(f"           to:   ({pos_after[0]:.1f}, {pos_after[1]:.1f}, {pos_after[2]:.1f}) yaw={yaw_after:.1f}°")
                    task_manager.complete_current("action executed")
                    print(f"  [TASK] {task_manager.summary()}")
                    if task_manager.is_done():
                        print("  [TASK] all stages complete")
                        state.update(status="done", task_done=True, step=0)
                        break
                    continue

                # === 1. 抓帧：先同步拿 RGB，再后台单独补 depth 供完成判定 ===
                _ct0 = _time.perf_counter()
                step_capture_profile = completion_checker.capture_profile_for_stage(current_stage)
                if current_stage and current_stage.mode == "detect":
                    step_capture_profile = completion_checker.rgb_profile
                depth_meters = None
                down_depth_meters = None
                capture_executor = None
                need_depth = (
                    split_rgb_depth
                    and getattr(completion_checker, "name", "") in {"depth_detector", "api_completion"}
                    and completion_checker.should_check_stage(current_stage)
                    and getattr(current_stage, "mode", "") == "target"
                    and step_capture_profile != completion_checker.rgb_profile
                )

                if need_depth:
                    frame, down_frame, _unused_front_depth, _unused_down_depth, rgb_timing = client.capture_views(
                        profile=completion_checker.rgb_profile,
                        mode=capture_mode,
                        verbose=False,
                    )
                    rgb_elapsed = _time.perf_counter() - _ct0
                    depth_profile = _depth_only_profile(step_capture_profile)
                    capture_executor = ThreadPoolExecutor(max_workers=1)
                    depth_future = capture_executor.submit(
                        _capture_profile_isolated,
                        client,
                        depth_profile,
                    )
                    print(
                        f"  [Pipeline] rgb ready in {rgb_elapsed:.2f}s; "
                        f"detect+plan start now, depth={depth_profile} in background"
                    )
                else:
                    depth_future = None
                    frame, down_frame, depth_meters, down_depth_meters, rgb_timing = client.capture_views(
                        profile=step_capture_profile,
                        mode=capture_mode,
                        verbose=False,
                    )
                    rgb_elapsed = _time.perf_counter() - _ct0

                if frame is None:
                    if capture_executor is not None:
                        capture_executor.shutdown(wait=False)
                    print("  [Capture] no frame, retrying...")
                    _time.sleep(0.1)
                    continue

                # 预编码缓存（detector/planner 可选复用）
                ImageEncoder.encode_front(frame)
                if down_frame is not None:
                    ImageEncoder.encode_down(down_frame)

                # 推送本轮真实控制帧到前端。后台预览线程在 step 期间会暂停，
                # 所以不能只依赖 FrameCapturer，否则 /frame 容易停在首帧。
                _push_pil_png_to_frontend(state, frame, view="front")
                if down_frame is not None:
                    _push_pil_png_to_frontend(state, down_frame, view="down")
                else:
                    state.set_down_frame(b"")

                # 推送前端深度
                if not need_depth and depth_meters is not None:
                    depth_preview = client.depth_meters_to_image(depth_meters)
                    if depth_preview:
                        from io import BytesIO as _B2
                        bd = _B2()
                        depth_preview.save(bd, format="PNG")
                        state.set_depth_frame(bd.getvalue())
                else:
                    state.set_depth_frame(b"")

                def _validate_relocalized_target(stage_obj, front_img, down_img, front_det, down_det, best_det):
                    relocalized_completion = completion_checker.evaluate_with_detection(
                        stage_obj,
                        cur_task,
                        front_img,
                        down_img,
                        best_det,
                        front_detection=front_det,
                        down_detection=down_det,
                        front_depth_meters=None,
                        down_depth_meters=None,
                    )
                    return relocalized_completion.detection if relocalized_completion.target_detected else None

                if current_stage and current_stage.mode == "detect":
                    if not relocalization_enabled:
                        print(
                            f"  [Capture] profile={completion_checker.rgb_profile} time={rgb_elapsed:.2f}s  "
                            f"front={_shape_text(frame)}  down={_shape_text(down_frame)}"
                        )
                        print("  [Relocalize] disabled; detect stage skipped")
                        state.update(status="target_not_found", step=step)
                        continue

                    _rd0 = _time.perf_counter()
                    relocalized = relocalizer.search(
                        client,
                        current_stage,
                        front_image=frame,
                        down_image=down_frame,
                        capture_mode=capture_mode,
                        skip_initial_frame=True,
                        validator=_validate_relocalized_target,
                    )
                    step_timing["capture_rpc"] = rgb_elapsed
                    step_timing["detect"] = relocalized.elapsed or (_time.perf_counter() - _rd0)
                    step_timing["detect_api"] = step_timing["detect"]
                    print(
                        f"  [Capture] profile={completion_checker.rgb_profile} time={rgb_elapsed:.2f}s  "
                        f"front={_shape_text(frame)}  down={_shape_text(down_frame)}"
                    )
                    print(
                        f"  [Relocalize] found={relocalized.found} time={step_timing['detect']:.2f}s  "
                        f"reason={relocalized.reason}"
                    )
                    if relocalized.found:
                        task_manager.complete_current(relocalized.reason)
                        print(f"  [TASK] {task_manager.summary()}")
                        if task_manager.is_done():
                            print("  [TASK] all stages complete")
                            state.update(status="done", task_done=True, step=0)
                            break
                    continue

                # ── 2. target 阶段：RGB 到手后立刻并行检测 + 规划；深度回来后只做完成判定 ──
                def _planning_fn():
                    state.update(status="planning")
                    return planner.plan(
                        frame, down_frame,
                        instruction=stage_instruction,
                        relation=getattr(current_stage, "relation", "") if current_stage else "",
                        target=getattr(current_stage, "target", "") if current_stage else "",
                    )

                use_rgb_detector_pipeline = (
                    current_stage
                    and current_stage.mode == "target"
                    and getattr(completion_checker, "uses_detector", False)
                    and completion_checker.is_detector_enabled()
                )

                if use_rgb_detector_pipeline:
                    detect_caption = (
                        getattr(current_stage, "target_query", None)
                        or getattr(current_stage, "target", None)
                        or stage_instruction
                    )

                    def _detect_one(image, camera_name):
                        if image is None:
                            from agent.detector.base import DetectionResult
                            return DetectionResult(visible=False, camera=camera_name)
                        return detector.detect(
                            image,
                            detect_caption,
                            depth_meters=None,
                            camera_name=camera_name,
                        )

                    def _describe_det(name, det):
                        if det and det.visible:
                            return f"{name}:bbox={det.bbox} score={det.score:.2f}"
                        if det and det.score:
                            return f"{name}:not_visible score={det.score:.2f}"
                        return f"{name}:not_visible"

                    step_executor = ThreadPoolExecutor(max_workers=3)
                    _td0 = _time.perf_counter()
                    front_det_future = step_executor.submit(_detect_one, frame, "front")
                    down_det_future = step_executor.submit(_detect_one, down_frame, "down")
                    plan_started_at = _time.perf_counter()
                    plan_future = step_executor.submit(_planning_fn)
                    front_det = front_det_future.result()
                    down_det = down_det_future.result()
                    detect_elapsed = _time.perf_counter() - _td0

                    visible_dets = [d for d in (front_det, down_det) if d and d.visible]
                    if visible_dets:
                        best_detection = max(visible_dets, key=lambda d: float(d.score or 0.0))
                    else:
                        best_detection = None

                    low_confidence = (
                        best_detection is None
                        or float(best_detection.score or 0.0) < detector_min_confidence
                    )

                    print(
                        f"  [DetectRGB] {_describe_det('front', front_det)}  "
                        f"{_describe_det('down', down_det)}  "
                        f"best={(best_detection.camera if best_detection else 'none')} "
                        f"threshold={detector_min_confidence:.2f}  time={detect_elapsed:.2f}s"
                    )
                    if low_confidence and not relocalization_enabled:
                        print("  [Relocalize] disabled; continue with planner output")

                    if low_confidence and relocalization_enabled:
                        if capture_executor is not None:
                            _shutdown_executor(capture_executor, wait=False)
                        _shutdown_executor(step_executor, wait=False)
                        _rd0 = _time.perf_counter()
                        relocalized = relocalizer.search(
                            client,
                            current_stage,
                            front_image=frame,
                            down_image=down_frame,
                            capture_mode=capture_mode,
                            skip_initial_frame=True,
                            validator=_validate_relocalized_target,
                        )
                        step_timing["capture_rpc"] = rgb_elapsed
                        step_timing["detect"] = detect_elapsed
                        step_timing["detect_api"] = detect_elapsed
                        step_timing["relocalize"] = _time.perf_counter() - _rd0
                        print(
                            f"  [Capture] profile={completion_checker.rgb_profile} time={rgb_elapsed:.2f}s  "
                            f"front={_shape_text(frame)}  down={_shape_text(down_frame)}"
                        )
                        print(
                            f"  [Relocalize] found={relocalized.found} time={step_timing['relocalize']:.2f}s  "
                            f"reason=low confidence; {relocalized.reason}"
                        )
                        state.update(status="relocalized" if relocalized.found else "target_not_found", step=step)
                        continue

                    local_depth = depth_meters
                    local_down_depth = down_depth_meters
                    if need_depth and depth_future is not None:
                        if not depth_future.done():
                            print("  [Depth] waiting for completion depth frames...")
                        _front, _down, local_depth, local_down_depth, depth_timing = depth_future.result()
                        depth_meters = local_depth
                        down_depth_meters = local_down_depth
                        step_timing["depth_capture_rpc"] = depth_timing.get("total_s", 0.0)
                        if local_depth is not None:
                            depth_preview = client.depth_meters_to_image(local_depth)
                            if depth_preview:
                                from io import BytesIO as _B2
                                bd = _B2()
                                depth_preview.save(bd, format="PNG")
                                state.set_depth_frame(bd.getvalue())
                    if capture_executor is not None:
                        _shutdown_executor(capture_executor, wait=False)

                    if local_depth is not None or local_down_depth is not None:
                        print(
                            "  [TargetDepth] "
                            + _target_depth_text("front", front_det, frame, local_depth)
                            + "  "
                            + _target_depth_text("down", down_det, down_frame, local_down_depth)
                        )

                    _tc0 = _time.perf_counter()
                    completion = completion_checker.evaluate_with_detection(
                        current_stage,
                        cur_task,
                        frame,
                        down_frame,
                        best_detection,
                        front_detection=front_det,
                        down_detection=down_det,
                        front_depth_meters=local_depth,
                        down_depth_meters=local_down_depth,
                    )
                    completion.elapsed = _time.perf_counter() - _tc0
                    detection = completion.detection
                    direction = completion.direction

                    if completion.done:
                        result = None
                        planning_elapsed = 0.0
                        with contextlib.redirect_stdout(io.StringIO()):
                            try:
                                plan_future.result()
                            except Exception:
                                pass
                        _shutdown_executor(step_executor, wait=False)
                    else:
                        result = plan_future.result()
                        planning_elapsed = _time.perf_counter() - plan_started_at
                        _shutdown_executor(step_executor, wait=False)

                    step_timing["capture_rpc"] = max(
                        rgb_elapsed,
                        step_timing.get("depth_capture_rpc", 0.0) if need_depth else rgb_elapsed,
                    )
                    step_timing["detect"] = detect_elapsed + completion.elapsed
                    step_timing["detect_api"] = detect_elapsed
                    step_timing["planning"] = planning_elapsed
                    step_timing["planning_api"] = planning_elapsed
                    step_timing["vlm_total"] = step_timing.get("detect_api", 0) + step_timing.get("planning_api", 0)
                else:
                    def _completion_fn():
                        state.update(status="checking_completion")
                        analysis = None
                        if getattr(completion_checker, "name", "") == "api_completion" and hasattr(completion_checker, "analyze_rgb"):
                            analysis = completion_checker.analyze_rgb(
                                current_stage,
                                cur_task,
                                frame,
                                down_frame,
                            )
                        local_depth = depth_meters
                        local_down_depth = down_depth_meters
                        if need_depth and depth_future is not None:
                            if not depth_future.done():
                                print("  [Depth] waiting for completion depth frames...")
                            _front, _down, local_depth, local_down_depth, depth_timing = depth_future.result()
                            step_timing["depth_capture_rpc"] = depth_timing.get("total_s", 0.0)
                            if local_depth is not None:
                                depth_preview = client.depth_meters_to_image(local_depth)
                                if depth_preview:
                                    from io import BytesIO as _B2
                                    bd = _B2()
                                    depth_preview.save(bd, format="PNG")
                                    state.set_depth_frame(bd.getvalue())
                        if getattr(completion_checker, "name", "") == "api_completion" and hasattr(completion_checker, "finalize_analysis"):
                            return completion_checker.finalize_analysis(
                                current_stage,
                                analysis,
                                frame,
                                down_frame,
                                front_depth_meters=local_depth,
                                down_depth_meters=local_down_depth,
                            )
                        return completion_checker.evaluate(
                            current_stage,
                            cur_task,
                            frame,
                            down_frame,
                            front_depth_meters=local_depth,
                            down_depth_meters=local_down_depth,
                        )

                    scheduled = run_completion_and_planning(
                        completion_call_mode,
                        _completion_fn,
                        _planning_fn,
                    )
                    if need_depth and depth_future is not None:
                        _front, _down, depth_meters, down_depth_meters, depth_timing = depth_future.result()
                        if capture_executor is not None:
                            _shutdown_executor(capture_executor, wait=False)
                    step_timing["capture_rpc"] = max(
                        rgb_elapsed,
                        step_timing.get("depth_capture_rpc", 0.0) if need_depth else rgb_elapsed,
                    )
                    completion = scheduled.completion
                    result = scheduled.plan_result
                    detection = completion.detection
                    direction = completion.direction
                    step_timing["detect"] = scheduled.completion_elapsed
                    step_timing["detect_api"] = step_timing["detect"] if completion.checked else 0.0
                    if scheduled.planning_started:
                        step_timing["planning"] = scheduled.planning_elapsed
                        step_timing["planning_api"] = scheduled.planning_elapsed
                    else:
                        step_timing["planning"] = 0.0
                        step_timing["planning_api"] = 0.0
                    step_timing["vlm_total"] = (step_timing.get("detect_api", 0) +
                                                 step_timing.get("planning_api", 0))

                if detection and detection.visible:
                    # 首次检测到目标 → 估计世界坐标（供论文指标用）
                    pos_now, yaw_now = client.get_pose()
                    det_image = down_frame if getattr(detection, "camera", "front") == "down" else frame
                    tracker.set_target_from_detection(
                        pos_now,
                        yaw_now,
                        detection,
                        image_size=det_image.size if det_image is not None else None,
                    )

                print(
                    f"  [Capture] profile={step_capture_profile} time={step_timing['capture_rpc']:.2f}s  "
                    f"front={_shape_text(frame)}  down={_shape_text(down_frame)}  "
                    f"front_depth={_shape_text(depth_meters)}  down_depth={_shape_text(down_depth_meters)}"
                )
                print(
                    f"  [Completion] detected={completion.target_detected} accepted={completion.accepted_view} "
                    f"done={completion.done} time={step_timing['detect']:.2f}s"
                )

                if (
                    completion.checked
                    and completion.target_detected is False
                    and current_stage is not None
                    and getattr(current_stage, "allow_relocalize", False)
                    and relocalization_enabled
                ):
                    _rd0 = _time.perf_counter()
                    relocalized = relocalizer.search(
                        client,
                        current_stage,
                        front_image=frame,
                        down_image=down_frame,
                        capture_mode=capture_mode,
                        skip_initial_frame=True,
                        validator=_validate_relocalized_target,
                    )
                    step_timing["relocalize"] = _time.perf_counter() - _rd0
                    print(
                        f"  [Relocalize] found={relocalized.found} time={step_timing['relocalize']:.2f}s  "
                        f"yaw={relocalized.yaw_delta_deg:.1f}deg  "
                        f"reason=vlm rejected detector proposals; {relocalized.reason}"
                    )
                    state.update(status="relocalized" if relocalized.found else "target_not_found", step=step)
                    continue

                # ── 3. 若任务已完成，直接进入下一阶段 ──
                if completion.done:
                    if current_stage and current_stage.mode == "target":
                        step_timing["step_total"] = _time.perf_counter() - step_started
                        tracker.record(step_timing, client.get_pose()[0], detection)
                        tracker.print_step(step, max_steps, step_timing)
                        tracker.print_final()

                    task_manager.complete_current(completion.reason)
                    print(f"  [TASK] {task_manager.summary()}")
                    if task_manager.is_done():
                        print("  [TASK] all stages complete")
                        state.update(status="done", task_done=True, step=0)
                        break
                    state.update(status="advancing", step=step)
                    continue

                if result is None:
                    continue

                if current_stage and current_stage.mode == "target" and trajectory_refiner is not None:
                    refinement = trajectory_refiner.refine_result(
                        result,
                        current_stage,
                        detection,
                        frame,
                        down_frame,
                    )
                    if refinement.changed:
                        print(f"  [TrajectoryRefiner] {refinement.summary()}")

                use_candidate_pipeline = bool(cfg.get("CANDIDATE", {}).get("ENABLED", False))
                if use_candidate_pipeline:
                    candidate_prep = prepare_candidates_for_world_model(
                        result,
                        detection=detection,
                        direction=direction,
                        stop_threshold=stop_threshold,
                    )
                    if candidate_prep.all_candidates:
                        result.candidates = [c.to_dict() for c in candidate_prep.all_candidates]
                        # Detailed candidate logging is intentionally omitted in the closed-loop console.

                    if world_model and candidate_prep.wm_candidates:
                        _tw0 = _time.perf_counter()
                        with contextlib.redirect_stdout(io.StringIO()):
                            wm_result = world_model.score_from_pil(
                                frame,
                                down_frame,
                                instruction=stage_instruction,
                                candidates=[c.to_world_model_dict() for c in candidate_prep.wm_candidates],
                            )
                        step_timing["world_model"] = _time.perf_counter() - _tw0
                        if 0 <= wm_result.best_index < len(candidate_prep.wm_candidates):
                            chosen = candidate_prep.wm_candidates[wm_result.best_index]
                            result.actions = list(chosen.actions)
                            result.waypoints = [list(wp) for wp in chosen.waypoints]
                            result.reasoning = (
                                (result.reasoning + " | " if result.reasoning else "")
                                + f"WM chose idx={wm_result.best_index} conf={chosen.confidence:.2f}"
                            )
                            for idx, cand in enumerate(result.candidates):
                                cand["selected_by_world_model"] = idx == wm_result.best_index
                    elif candidate_prep.all_candidates:
                        chosen = candidate_prep.all_candidates[0]
                        result.actions = list(chosen.actions)
                        result.waypoints = [list(wp) for wp in chosen.waypoints]
                        result.reasoning = (
                            (result.reasoning + " | " if result.reasoning else "")
                            + f"PreScore chose source={chosen.source} conf={chosen.confidence:.2f}"
                        )
                        for idx, cand in enumerate(result.candidates):
                            cand["selected_by_prescore"] = idx == 0
                else:
                    result.candidates = []

                if result.candidates:
                    _print_candidate_ranking(
                        result.candidates,
                        topk=int(cfg.get("CANDIDATE", {}).get("TOPK_FOR_WORLD_MODEL", 1)),
                    )

                wp_count = len(result.waypoints)
                non_zero = sum(1 for wp in result.waypoints if not all(v == 0.0 for v in wp))
                wm_part = f" wm={step_timing['world_model']:.2f}s" if step_timing.get("world_model") else ""
                print(
                    f"  [Trajectory] wp={wp_count} non_zero={non_zero} "
                    f"time={step_timing['planning']:.2f}s{wm_part}  body_wp={_format_waypoints(result.waypoints)}"
                )

                # ── 6. 执行（统一：所有规划器都走 waypoints → execute_waypoints）──
                #
                # api_atomic_planner: actions → common trajectory conversion → body waypoints
                # qwen_planner:       直接输出 body waypoints
                # 两者最终都在 execute_waypoints 中：body→世界坐标 → moveOnPathAsync(ForwardOnly)
                #
                exec_waypoints = list(result.waypoints) if result.waypoints else []
                # 兜底：如果 waypoints 全零但 actions 非空（解析异常等边界情况），从 actions 重新转
                if (not exec_waypoints or all(all(abs(v) < 1e-6 for v in wp) for wp in exec_waypoints)) \
                        and result.actions:
                    exec_waypoints = actions_to_cumulative_body_waypoints(result.actions)
                    pass

                non_zero_wp = sum(1 for wp in exec_waypoints if not all(abs(v) < 1e-6 for v in wp))
                can_execute = non_zero_wp > 0

                if can_execute:
                    state.update(status="executing")
                    if capturer is not None and hasattr(capturer, "resume"):
                        # Planning/capture RPCs are done; let the dashboard update while the UAV is moving.
                        capturer.resume()
                    _te0 = _time.perf_counter()
                    pos_before, yaw_before = client.get_pose()
                    tracker.ensure_start_position(pos_before)

                    print(
                        f"  [Execute] start wp={non_zero_wp} "
                        f"from=({pos_before[0]:.1f}, {pos_before[1]:.1f}, {pos_before[2]:.1f}) "
                        f"yaw={yaw_before:.1f}°"
                    )

                    with contextlib.redirect_stdout(io.StringIO()):
                        pos_final, yaw_final, col = client.execute_waypoints(exec_waypoints)

                    step_timing["execute"] = _time.perf_counter() - _te0
                    step_timing["collided"] = col
                    state.update(pose=pos_final, yaw=yaw_final, collided=col)
                    if col:
                        recovery = collision_recovery.recover(client)
                        if recovery.attempted and recovery.pose is not None:
                            pos_final = list(recovery.pose)
                            yaw_final = float(recovery.yaw if recovery.yaw is not None else yaw_final)
                            state.update(pose=pos_final, yaw=yaw_final, collided=recovery.collided)

                else:
                    step_timing["execute"] = 0.0
                    pos_final, _ = client.get_pose()
                    step_timing["collided"] = False

                step_timing["step_total"] = _time.perf_counter() - step_started
                tracker.record(step_timing, pos_final, detection)
                tracker.print_step(step, max_steps, step_timing)

    except KeyboardInterrupt:
        print("\n[Exit] interrupted")
        tracker.print_final()
        raise  # 抛给外层循环处理 cleanup


# ═══════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════

if __name__ == "__main__":
    state = SharedState()
    app = create_app(state)
    web_port = cfg.get("WEB", {}).get("PORT", 5000)

    def run_web():
        import logging
        logging.getLogger('werkzeug').setLevel(logging.ERROR)
        cli = sys.modules.get('flask.cli')
        if cli:
            cli.show_server_banner = lambda *_, **__: None
        app.run(host="0.0.0.0", port=web_port, debug=False, use_reloader=False)

    threading.Thread(target=run_web, daemon=True).start()
    print("=" * 50)
    print(f"  Dashboard: http://localhost:{web_port}")
    print(f"  Config: config/default.yaml")
    print("=" * 50)

    if bool(cfg.get("SIM", {}).get("APPLY_SETTINGS_ON_WEB_START", True)):
        from sim.airsim_settings import write_local_airsim_settings

        settings_path = write_local_airsim_settings(make_backup=True)
        print(f"[AirSimSettings] wrote: {settings_path}")
        print("[AirSimSettings] start/restart AirSim now so the camera settings take effect")

    print("[AirSim] connecting...", flush=True)
    client = _connect_web_airsim_client()
    client.warmup_capture()
    client.enable_api_control(True)
    client.arm(True)
    should_takeoff, start_pos, start_yaw, landed_text = _should_takeoff(client)
    print(
        f"[AirSim] startup pose=({start_pos[0]:.1f}, {start_pos[1]:.1f}, {start_pos[2]:.1f}) "
        f"yaw={start_yaw:.1f}° landed_state={landed_text}"
    )
    if should_takeoff:
        print("[AirSim] takeoff...")
        client.takeoff()
        pos_after_takeoff, yaw_after_takeoff = client.get_pose()
        print(
            f"[AirSim] post-takeoff pose=({pos_after_takeoff[0]:.1f}, {pos_after_takeoff[1]:.1f}, {pos_after_takeoff[2]:.1f}) "
            f"yaw={yaw_after_takeoff:.1f}°"
        )
    else:
        print("[AirSim] skip takeoff: vehicle already airborne")

    capturer = FrameCapturer(state, interval=0.1)
    capturer.start()
    print("[FrameCapturer] background capture started")

    warmup_from_config()

    print("[AirSim] waiting for first frame...")
    while True:
        rgb, depth = capturer.get_latest_frame()
        if rgb is not None and depth is not None:
            print(f"[Ready] first frame ready (depth shape={depth.shape})")
            break
        time.sleep(0.1)

    initial_task = ""
    while not initial_task.strip():
        time.sleep(1)
        st = state.get_state()
        initial_task = st.get("task", "").strip()

    # ── 任务循环：完成后等待 Web 输入新任务 ──
    cur_task = initial_task
    while True:
        print(f"\n{'='*50}")
        print(f"  新任务: {cur_task}")
        print(f"{'='*50}")
        state.update(status="running", task_done=False)

        try:
            main_loop(state, initial_task=cur_task,
                      client=client, capturer=capturer)
        except KeyboardInterrupt:
            print("\n[Exit] shutting down...")
            break

        # 当前任务完成，清空等待下一个
        state.update(status="waiting_task", task="", task_done=True)
        print("\n[TASK] 任务完成，等待新任务...")
        next_task = ""
        while not next_task.strip():
            time.sleep(1)
            st = state.get_state()
            next_task = st.get("task", "").strip()
        cur_task = next_task

    # 程序退出前清理
    try:
        client.cleanup()
    except Exception:
        pass
