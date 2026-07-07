#!/usr/bin/env python3
"""run_airsim_web.py — 入口。只负责组装模块、启动服务、运行主循环。

基础设施都在各自模块里：
  - API 预热     → agent/common/warmup.py
  - 图像编码缓存  → agent/common/image_encoder.py
  - 抓帧         → sim/capture.py
  - 后台取帧     → sim/frame_capturer.py
  - 步级指标     → 本文件的 StepTracker（含 SR/NE/SPL/OSR/TL 论文指标）
"""

import os, sys, math, time, threading

_script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _script_dir)

from config import cfg, get_cfg
get_cfg(os.path.join(_script_dir, "config", "default.yaml"))

from agent.detector import build_detector
from agent.planner import build_planner
from agent.direction import build_direction_estimator
from agent.task_parser import build_task_parser
from agent.common.task_manager import TaskManager
from agent.common.warmup import warmup_from_config
from agent.common.image_encoder import ImageEncoder
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
                                   detection) -> bool:
        """首次检测到目标时，估计其世界坐标。

        Returns True if target was set for the first time.
        """
        if self.target_world_pos is not None:
            return False
        if not detection or not detection.visible or not detection.depth_median:
            return False
        if not detection.bbox:
            return False

        # bbox 中心偏移角（粗略：bbox 中心相对图像中心的水平偏角）
        # 使用方向估计器的同类逻辑
        depth = detection.depth_median
        # 用 bbox 中心在图像中的水平位置估算 bearing
        fx = 960.0  # 近似焦距 (1920/2/tan(45°))
        cx_img = 960.0
        bbox_cx = (detection.bbox[0] + detection.bbox[2]) / 2.0
        bearing_deg = math.degrees(math.atan2(bbox_cx - cx_img, fx))

        target_yaw = math.radians(yaw_deg + bearing_deg)
        self.target_world_pos = [
            drone_pos[0] + depth * math.cos(target_yaw),
            drone_pos[1] + depth * math.sin(target_yaw),
            drone_pos[2],
        ]
        print(f"  [Metrics] target world pos estimated: "
              f"({self.target_world_pos[0]:.1f}, {self.target_world_pos[1]:.1f}, "
              f"{self.target_world_pos[2]:.1f})  depth={depth:.1f}m  bearing={bearing_deg:.1f}°")
        return True

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

def main_loop(state, initial_task="", max_steps=999, client=None, capturer=None):
    import time as _time

    detector = build_detector()
    planner = build_planner()
    direction_est = build_direction_estimator()
    task_manager = TaskManager(enabled=True)
    tracker = StepTracker(success_radius=cfg["AGENT"]["STOP_DEPTH_THRESHOLD"])

    stop_threshold = cfg["AGENT"]["STOP_DEPTH_THRESHOLD"]

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

            current_stage = task_manager.current_stage()
            # 用任务解析器的英文输出作为 planner 的 instruction
            stage_instruction = (current_stage.instruction
                                 if current_stage else cur_task)
            if current_stage:
                print(f"  stage {current_stage.index+1}/{len(task_manager.stages)} "
                      f"mode={current_stage.mode}: {stage_instruction}")

            state.update(step=step, status="capturing", error="")

            # ── 1. 抓帧（前视 + 下视 + 深度，一次 RPC）──
            _ct0 = _time.perf_counter()
            frame, down_frame, depth_meters = client.get_dual_view()
            step_timing["capture_rpc"] = _time.perf_counter() - _ct0

            if frame is None:
                print("  [Capture] no frame, retrying...")
                _time.sleep(0.1)
                continue

            # 预编码缓存（detector/planner 可选复用）
            ImageEncoder.encode_front(frame)
            if down_frame is not None:
                ImageEncoder.encode_down(down_frame)

            # 推送前端深度 + 下视图
            if depth_meters is not None:
                depth_preview = client.depth_meters_to_image(depth_meters)
                if depth_preview:
                    from io import BytesIO as _B2
                    bd = _B2()
                    depth_preview.save(bd, format="PNG")
                    state.set_depth_frame(bd.getvalue())
            if down_frame is not None:
                from io import BytesIO as _B3
                bd2 = _B3()
                down_frame.save(bd2, format="PNG")
                state.set_down_frame(bd2.getvalue())

            print(f"  [Capture] rpc={step_timing['capture_rpc']:.2f}s  size={frame.size}  "
                  f"depth_shape={depth_meters.shape if depth_meters is not None else 'N/A'}")

            # ── 2. 目标检测 ──
            state.update(status="detecting")
            _td0 = _time.perf_counter()
            detect_caption = (current_stage.target_query
                              if current_stage and current_stage.target_query
                              else cur_task)
            detection = detector.detect(frame, detect_caption, depth_meters)
            step_timing["detect"] = _time.perf_counter() - _td0
            step_timing["detect_api"] = step_timing["detect"]

            if detection.visible:
                d = detection.depth_median
                ds = f"depth={d:.2f}m" if d else "depth=N/A"
                si = f" STOP(<{stop_threshold}m)" if (d and d < stop_threshold) else ""
                print(f"  [Detect] {detection.label} bbox={detection.bbox} "
                      f"conf={detection.score:.2f} {ds}{si} ({step_timing['detect']:.2f}s)")

                # 首次检测到目标 → 估计世界坐标（供论文指标用）
                pos_now, yaw_now = client.get_pose()
                tracker.set_target_from_detection(pos_now, yaw_now, detection)
            else:
                print(f"  [Detect] '{detect_caption}' not found ({step_timing['detect']:.2f}s)")

            # ── 3. 方向估计 ──
            direction = ""
            if detection.visible and detection.bbox:
                direction = direction_est.estimate(
                    detection.bbox, 0, (frame.width, frame.height))
            if direction:
                print(f"  [Direction] {direction}")

            # ── 4. 轨迹规划（用英文 stage_instruction）──
            state.update(status="planning")
            _tp0 = _time.perf_counter()
            result = planner.plan(
                frame, down_frame,
                instruction=stage_instruction,
                direction=direction,
                detected_bbox=detection.bbox if detection.visible else None,
                depth_meters=depth_meters,
            )
            step_timing["planning"] = _time.perf_counter() - _tp0
            step_timing["planning_api"] = step_timing["planning"]
            step_timing["vlm_total"] = (step_timing.get("detect_api", 0) +
                                         step_timing.get("planning_api", 0))

            wp_count = len(result.waypoints)
            non_zero = sum(1 for wp in result.waypoints if not all(v == 0.0 for v in wp))
            print(f"  [Plan] {wp_count} wp ({non_zero} non-zero), "
                  f"stop={result.done}, {len(result.candidates)} candidates "
                  f"({step_timing['planning']:.2f}s)")

            # ── 5. 停止判断 ──
            should_stop = result.done or (
                detection.visible and detection.depth_median is not None
                and detection.depth_median < stop_threshold
            )
            if should_stop:
                reason = (f"depth={detection.depth_median:.1f}m < {stop_threshold}m"
                          if (detection.visible and detection.depth_median is not None
                              and detection.depth_median < stop_threshold)
                          else "planner reported done")
                print(f"  [Stop] {reason}")
                step_timing["step_total"] = _time.perf_counter() - step_started
                tracker.record(step_timing, client.get_pose()[0], detection)
                tracker.print_step(step, max_steps, step_timing)
                tracker.print_final()
                state.update(status="done", task_done=True, step=0)
                break

            # ── 6. 执行（统一：所有规划器都走 waypoints → execute_waypoints）──
            #
            # api_atomic_planner: actions → _actions_to_body_waypoints() → body waypoints
            # qwen_planner:       直接输出 body waypoints
            # 两者最终都在 execute_waypoints 中：body→世界坐标 → moveOnPathAsync(ForwardOnly)
            #
            exec_waypoints = list(result.waypoints) if result.waypoints else []
            # 兜底：如果 waypoints 全零但 actions 非空（解析异常等边界情况），从 actions 重新转
            if (not exec_waypoints or all(all(abs(v) < 1e-6 for v in wp) for wp in exec_waypoints)) \
                    and result.actions:
                from agent.planner.api_atomic_planner import _actions_to_body_waypoints
                exec_waypoints = _actions_to_body_waypoints(result.actions)
                print(f"  [Execute] actions→waypoints: {len(result.actions)} actions → {len(exec_waypoints)} wp")

            non_zero_wp = sum(1 for wp in exec_waypoints if not all(abs(v) < 1e-6 for v in wp))
            can_execute = non_zero_wp > 0

            if can_execute:
                state.update(status="executing")
                _te0 = _time.perf_counter()
                pos_before, yaw_before = client.get_pose()

                pos_final, yaw_final, col = client.execute_waypoints(exec_waypoints)

                step_timing["execute"] = _time.perf_counter() - _te0
                step_timing["collided"] = col
                state.update(pose=pos_final, yaw=yaw_final, collided=col)

                if detection.visible and detection.depth_median:
                    print(f"  [Execute] {step_timing['execute']:.2f}s, "
                          f"dist_to_target={detection.depth_median:.1f}m")
                else:
                    moved = ((pos_final[0]-pos_before[0])**2 +
                             (pos_final[1]-pos_before[1])**2)**0.5
                    print(f"  [Execute] {step_timing['execute']:.2f}s, moved={moved:.1f}m")

                from agent.planner.base import compute_per_step_deltas
                step_deltas = compute_per_step_deltas(result, start_yaw_deg=yaw_before)
                if step_deltas:
                    cum = [sum(d[i] for d in step_deltas) for i in range(4)]
                    print(f"  [Delta] {len(step_deltas)} steps, "
                          f"net=[dx={cum[0]:.1f} dy={cum[1]:.1f} dz={cum[2]:.1f} dphi={cum[3]:.1f}°]")
            else:
                print(f"  [Execute] no valid actions, skipping")
                step_timing["execute"] = 0.0
                pos_final, _ = client.get_pose()
                step_timing["collided"] = False

            step_timing["step_total"] = _time.perf_counter() - step_started
            tracker.record(step_timing, pos_final, detection)
            tracker.print_step(step, max_steps, step_timing)

    except KeyboardInterrupt:
        print("\n[Exit] interrupted")
        tracker.print_final()
    finally:
        try:
            client.cleanup()
        except Exception:
            pass


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

    print("[AirSim] connecting...")
    client = AirSimClient()
    client.connect()
    client.warmup_capture()
    client.enable_api_control(True)
    client.arm(True)
    if client.get_multirotor_state().landed_state != "Flying":
        client.takeoff()

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

    main_loop(state, initial_task=initial_task,
              client=client, capturer=capturer)
