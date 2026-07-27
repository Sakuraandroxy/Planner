"""
eval/run_eval.py — 通用离线评估器（闭环，需 AirSim）。

用法:
    # 单场景手动模式 (不变)
    python eval/run_eval.py --dataset ./data/test --scene ModularEuropean

    # 多场景自动切换模式 (新增, 需 TravelUAV 环境)
    python eval/run_eval.py --dataset ./data/test --env_root /data/sakura/data/TravelUAV_env [--gpu 0]

数据集格式（兼容 3DG-VLN UAV-VLN-FOV）:
    dataset/
    └── scene_name/
        └── episode_name/
            ├── mark.json      (start, target.position)
            └── obj_des.json   (instruction)

流程:
    1. 遍历所有 episode
    2. 对于每个: 连接 AirSim → 放到起点 → 运行 Agent → 执行 → 记录轨迹
    3. 全部跑完后出指标 SR / OSR / NE / SPL
"""

import os, json, sys, time, math
from pathlib import Path
from types import SimpleNamespace
from typing import List, Dict, Optional
import numpy as np

# 项目路径
_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_root))

from config import cfg
from agent.models.detection import build_detector
from agent.functions.direction import build_direction_estimator
from agent.functions.task_parser import build_task_parser
from agent.models.world_model import build_world_model
from agent.functions.candidate import prepare_candidates_for_world_model, select_best_candidate
from agent.functions.completion import build_task_completion_checker
from agent.functions.common.config_access import function_section
from agent.functions.common.task_manager import TaskManager
from agent.functions.planning.sliding_window_planning import SlidingWindowPlanningFunction
from agent.functions.relocalization import TargetRelocalizer
from agent.models.planner.sliding_window_planner import incremental_to_cumulative
from eval.metrics import MetricsTracker
from sim.airsim_client import AirSimClient


SUCCESS_RADIUS = float(cfg.get("EVAL", {}).get("SUCCESS_RADIUS", 10.0))
MAX_STEPS = int(cfg.get("EVAL", {}).get("MAX_STEPS", 100))
INIT_MOVE_VELOCITY = float(cfg.get("EVAL", {}).get("INIT_MOVE_VELOCITY", 5.0))
INIT_MOVE_TIMEOUT = float(cfg.get("EVAL", {}).get("INIT_MOVE_TIMEOUT", 30.0))
INIT_SETTLE_SECONDS = float(cfg.get("EVAL", {}).get("INIT_SETTLE_SECONDS", 1.0))
SCENE_SWITCH_RECONNECT_WAIT = float(cfg.get("EVAL", {}).get("SCENE_SWITCH_RECONNECT_WAIT", 2.0))
PLANNER_STOP_THRESHOLD = float(cfg.get("AGENT", {}).get("STOP_DEPTH_THRESHOLD", 8.0))


def _shape_text(value):
    if value is None:
        return "N/A"
    if hasattr(value, "shape"):
        return str(value.shape)
    if hasattr(value, "size") and not isinstance(value, (list, tuple)):
        return str(value.size)
    return str(value)


def _distance(a: List[float], b: List[float]) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


def _yaw_from_quaternion_xyzw(q: List[float]) -> Optional[float]:
    """Return yaw degrees from AirSim-style quaternion [x, y, z, w]."""
    if not isinstance(q, (list, tuple)) or len(q) < 4:
        return None
    x, y, z, w = [float(v) for v in q[:4]]
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny, cosy))


def _load_first_logged_pose(ep_dir: Path) -> tuple[Optional[List[float]], Optional[List[float]], str]:
    """Load the first frame pose used by the released UAV-VLN-FOV images/logs.

    mark.json can contain a coarse start field, but the images and training
    samples are aligned to the first available frame under FrontCamera/log.
    """
    front_dir = ep_dir / "FrontCamera"
    if not front_dir.exists():
        return None, None, ""
    front_imgs = sorted(p for p in front_dir.iterdir() if p.suffix.lower() == ".png")
    if not front_imgs:
        return None, None, ""
    frame_stem = front_imgs[0].stem
    log_file = ep_dir / "log" / f"{frame_stem}.json"
    if not log_file.exists():
        return None, None, frame_stem
    try:
        with open(log_file, "r") as f:
            log = json.load(f)
    except Exception:
        return None, None, frame_stem
    state = log.get("sensors", {}).get("state", {})
    imu = log.get("sensors", {}).get("imu", {})
    pos = state.get("position")
    ori = state.get("orientation") or imu.get("orientation")
    return pos, ori, frame_stem


def _execute_direct_action(client: AirSimClient, action: str, value: float):
    """执行 action 阶段固定动作，逻辑与在线闭环保持一致。"""
    pos_before, yaw_before = client.get_pose()
    if action in ("left", "right"):
        sign = 1 if action == "right" else -1
        client.rotate_to_yaw(yaw_before + sign * value)
    elif action == "forward":
        rad = math.radians(yaw_before)
        client.move_to_position(
            pos_before[0] + value * math.cos(rad),
            pos_before[1] + value * math.sin(rad),
            pos_before[2],
        )
    elif action == "backward":
        rad = math.radians(yaw_before)
        client.move_to_position(
            pos_before[0] - value * math.cos(rad),
            pos_before[1] - value * math.sin(rad),
            pos_before[2],
        )
    elif action == "up":
        client.move_to_position(pos_before[0], pos_before[1], pos_before[2] - value)
    elif action == "down":
        client.move_to_position(pos_before[0], pos_before[1], pos_before[2] + value)
    pos_after, yaw_after = client.get_pose()
    return pos_before, yaw_before, pos_after, yaw_after


def _build_eval_task_manager(task_parser, instruction: str) -> TaskManager:
    """为评测 episode 构造 TaskManager；解析失败时回退到单阶段 target。"""
    tm = TaskManager(enabled=True)
    raw_instruction = (instruction or "").strip()
    if not raw_instruction:
        raw_instruction = "Navigate to the target"

    try:
        t0 = time.perf_counter()
        parsed = task_parser.parse(raw_instruction)
        elapsed = time.perf_counter() - t0
        tm.start_with_stages(raw_instruction, parsed)
        print(f"  [TASK PARSER] parsed {len(tm.stages)} stages in {elapsed:.2f}s")
        print(f"  [TASK] {tm.summary()}")
        return tm
    except Exception as exc:
        print(f"  [TASK PARSER] fallback to single target stage: {exc}")
        tm.start_with_stages(raw_instruction, [{
            "instruction": raw_instruction,
            "mode": "target",
            "target": raw_instruction,
        }])
        print(f"  [TASK] {tm.summary()}")
        return tm


def discover_episodes(dataset_path: str) -> List[Dict]:
    """扫描数据集目录，返回所有 episode 的元信息。

    兼容 3DG-VLN UAV-VLN-FOV 格式:
        Scene/traj/mark.json  +  obj_des.json
    """
    episodes = []
    dataset_path = Path(dataset_path)
    if not dataset_path.exists():
        raise FileNotFoundError(f"数据集不存在: {dataset_path}")

    for scene_dir in sorted(dataset_path.iterdir()):
        if not scene_dir.is_dir():
            continue
        scene_name = scene_dir.name
        for ep_dir in sorted(scene_dir.iterdir()):
            if not ep_dir.is_dir():
                continue
            mark_file = ep_dir / "mark.json"
            if not mark_file.exists():
                continue
            with open(mark_file, "r") as f:
                meta = json.load(f)

            # 3DG-VLN 格式: mark.json 含 start/end/target。
            # 但实际图像/训练样本与 FrontCamera 第一帧对应的 log 对齐，
            # 因此闭环评测也优先使用第一帧 log 中的完整位姿。
            mark_start = meta.get("start", [0, 0, 0])
            start_pos = mark_start
            start_orientation = None
            start_frame = ""
            logged_pos, logged_ori, start_frame = _load_first_logged_pose(ep_dir)
            if logged_pos is not None:
                start_pos = logged_pos
            if logged_ori is not None:
                start_orientation = logged_ori
            target_info = meta.get("target", {})
            target_pos = target_info.get("position", [0, 0, 0]) if isinstance(target_info, dict) else target_info

            # 指令从 obj_des.json 读取
            obj_des_file = ep_dir / "obj_des.json"
            instruction = ""
            target_obj = ""
            if obj_des_file.exists():
                with open(obj_des_file, "r") as f:
                    obj_des = json.load(f)
                    if isinstance(obj_des, list) and len(obj_des) > 0:
                        instruction = obj_des[0]
                    elif isinstance(obj_des, str):
                        instruction = obj_des
                    target_obj = meta.get("object_name", "")

            episodes.append({
                "name": ep_dir.name,
                "scene": scene_name,
                "start": start_pos,
                "mark_start": mark_start,
                "start_orientation": start_orientation,
                "start_frame": start_frame,
                "target": target_pos,
                "instruction": instruction,
                "target_obj": target_obj,
            })
    return episodes


def evaluate(episodes: List[Dict], scene_filter: str = None,
             scene_manager=None, current_scene: str = ""):
    """批量评估。

    Args:
        episodes: discover_episodes() 的输出
        scene_filter: 只跑特定场景（如 "Town01"）
        scene_manager: SceneManager 实例（可选，用于多场景自动切换）
        current_scene: 当前已启动的场景名
    """
    if scene_filter:
        episodes = [ep for ep in episodes if ep["scene"] == scene_filter]

    print(f"评估 {len(episodes)} 条 episode...")
    if not episodes:
        print("没有要跑的 episode")
        return

    # 初始化各模块（由 config 决定实现）
    detector = build_detector()
    planner = SlidingWindowPlanningFunction(function_section(cfg, "PLANNING"))
    world_model = build_world_model()
    direction_est = build_direction_estimator()
    task_parser = build_task_parser()
    completion_checker = build_task_completion_checker(detector=detector, direction_estimator=direction_est)
    relocalizer = TargetRelocalizer(cfg, detector=detector)
    detector_min_confidence = float(cfg.get("AGENT", {}).get(
        "DETECTOR_MIN_CONFIDENCE",
        cfg.get("AGENT", {}).get("DETECTOR_BOX_THRESHOLD", 0.0),
    ))

    # 连接 AirSim
    client = AirSimClient()
    client.connect()
    client.warmup_capture()
    capture_mode = client.resolve_capture_mode()
    print(f"[CaptureConfig] mode={capture_mode} rgb_profile={completion_checker.rgb_profile} "
          f"completion_profile={completion_checker.depth_profile}")

    all_metrics = {
        "sr": 0, "osr": 0,
        "ne_list": [], "spl_list": [],
        "trajectory_lengths": [],
    }

    for idx, ep in enumerate(episodes):
        print(f"\n[{idx+1}/{len(episodes)}] {ep['scene']}/{ep['name']}")
        print(f"  指令: {ep['instruction']}")
        print(f"  目标: {ep['target']}")

        # ── 场景切换检测 ──
        if scene_manager is not None and ep["scene"] != current_scene:
            current_scene = ep["scene"]
            if not scene_manager.start(current_scene):
                print(f"  [SKIP] 场景 '{current_scene}' 不可用")
                continue
            # 重新连接 AirSim (旧连接已失效)
            try:
                client.cleanup()
            except Exception:
                pass
            time.sleep(SCENE_SWITCH_RECONNECT_WAIT)
            client = AirSimClient()
            client.connect()
            client.warmup_capture()
            capture_mode = client.resolve_capture_mode()
            print(f"  [CaptureConfig] mode={capture_mode} rgb_profile={completion_checker.rgb_profile} "
                  f"completion_profile={completion_checker.depth_profile}")
            print(f"  [SceneManager] 已切换到 {current_scene}")

        # 创建指标跟踪器
        metrics = MetricsTracker(ep["target"], success_radius=SUCCESS_RADIUS)
        task_finished = False
        task_manager = _build_eval_task_manager(task_parser, ep["instruction"])

        # ── 初始化无人机到轨迹起始位姿 ──
        start = ep["start"]
        start_yaw = _yaw_from_quaternion_xyzw(ep.get("start_orientation"))
        if start_yaw is None:
            print(f"  起始位姿: {start}")
        else:
            print(f"  起始位姿: {start} yaw={start_yaw:.1f}° frame={ep.get('start_frame') or 'N/A'}")
            if ep.get("mark_start") and _distance(start, ep["mark_start"]) > 1.0:
                print(f"  [Init] 使用首帧 log 位姿, mark.start={ep['mark_start']}")
        try:
            client.enable_api_control(True)
            client.arm(True)
            if start_yaw is not None:
                client.set_pose_xyz_yaw(start[0], start[1], start[2], start_yaw)
            else:
                client.move_to_position(
                    start[0], start[1], start[2],
                    velocity=INIT_MOVE_VELOCITY,
                    timeout=INIT_MOVE_TIMEOUT,
                )
            time.sleep(INIT_SETTLE_SECONDS)
            metrics.record_step(client.get_pose()[0])
        except Exception as e:
            print(f"  [WARN] 初始化位姿失败: {e}")
            continue

        # 主循环
        for step in range(MAX_STEPS):
            try:
                pos = client.get_pose()[0]
            except Exception:
                break

            current_stage = task_manager.current_stage()
            stage_instruction = current_stage.instruction if current_stage else ep["instruction"]
            if current_stage:
                print(f"  [Stage] {current_stage.index + 1}/{len(task_manager.stages)} "
                      f"mode={current_stage.mode}: {stage_instruction}")

            # 固定动作阶段：直接执行，不调用 VLM
            if current_stage and current_stage.mode == "action":
                action = current_stage.action
                value = float(current_stage.value or 0.0)
                print(f"  [ACTION] direct execution: {action} {value}")
                try:
                    pos_before, yaw_before, pos_after, yaw_after = _execute_direct_action(
                        client, action, value
                    )
                    metrics.record_step(pos_after)
                    print(f"  [ACTION] from: ({pos_before[0]:.1f}, {pos_before[1]:.1f}, {pos_before[2]:.1f}) "
                          f"yaw={yaw_before:.1f}°")
                    print(f"           to:   ({pos_after[0]:.1f}, {pos_after[1]:.1f}, {pos_after[2]:.1f}) "
                          f"yaw={yaw_after:.1f}°")
                except Exception as exc:
                    print(f"  [ACTION] error: {exc}")
                    break
                task_manager.complete_current("action executed")
                print(f"  [TASK] {task_manager.summary()}")
                if task_manager.is_done():
                    task_finished = True
                    break
                continue

            # ═══ 单次抓图：和 run_airsim_web.py 一样，一次 capture_views 搞定 ═══
            _cap0 = time.perf_counter()
            step_capture_profile = completion_checker.capture_profile_for_stage(current_stage)
            if current_stage and current_stage.mode == "detect":
                step_capture_profile = completion_checker.rgb_profile
            front_rgb, down_rgb, front_depth, down_depth, _rgb_timing = client.capture_views(
                profile=step_capture_profile,
                mode=capture_mode,
                verbose=False,
            )
            cap_rgb_elapsed = time.perf_counter() - _cap0
            if front_rgb is None:
                time.sleep(0.1)
                continue

            print(
                f"  [Capture] profile={step_capture_profile} rgb_time={cap_rgb_elapsed:.2f}s  "
                f"front={_shape_text(front_rgb)} down={_shape_text(down_rgb)}"
            )

            # detect 阶段：四向环视重定位，不进入 planner
            if current_stage and current_stage.mode == "detect":
                _rd0 = time.perf_counter()
                relocalized = relocalizer.search(
                    client, current_stage, front_image=front_rgb, down_image=down_rgb,
                    capture_mode=capture_mode,
                )
                print(f"  [Relocalize] found={relocalized.found} "
                      f"time={time.perf_counter() - _rd0:.2f}s reason={relocalized.reason}")
                if relocalized.found:
                    task_manager.complete_current(relocalized.reason)
                    print(f"  [TASK] {task_manager.summary()}")
                    if task_manager.is_done():
                        task_finished = True
                        break
                continue

            # 停止判断：GT 世界坐标距离
            pos = client.get_pose()[0]
            dist_to_gt = _distance(pos, ep["target"])
            if dist_to_gt < SUCCESS_RADIUS:
                print(f"  [Done] world_dist={dist_to_gt:.1f}m < {SUCCESS_RADIUS}m")
                if current_stage:
                    task_manager.complete_current("reached target radius")
                    print(f"  [TASK] {task_manager.summary()}")
                    task_finished = task_manager.is_done()
                else:
                    task_finished = True
                break

            detect_caption = (
                current_stage.target_query if current_stage and current_stage.target_query
                else stage_instruction
            )

            # ═══ 顺序检测：先前视后下视，不创建额外 AirSim 连接 ═══
            _td0 = time.perf_counter()
            if front_rgb is not None:
                front_det = detector.detect(front_rgb, detect_caption, depth_meters=None, camera_name="front")
            else:
                from agent.models.detection.base import DetectionResult
                front_det = DetectionResult(visible=False, camera="front")
            if down_rgb is not None:
                down_det = detector.detect(down_rgb, detect_caption, depth_meters=None, camera_name="down")
            else:
                from agent.models.detection.base import DetectionResult
                down_det = DetectionResult(visible=False, camera="down")
            _td = time.perf_counter() - _td0

            visible_dets = [d for d in (front_det, down_det) if d and d.visible]
            detection = max(visible_dets, key=lambda d: float(d.score or 0.0)) if visible_dets else None
            low_confidence = detection is None or float(detection.score or 0.0) < detector_min_confidence

            det_front_str = f"front:bbox={front_det.bbox} score={front_det.score:.2f}" if front_det and front_det.visible else "front:not_visible"
            det_down_str = f"down:bbox={down_det.bbox} score={down_det.score:.2f}" if down_det and down_det.visible else "down:not_visible"
            print(f"  [DetectRGB] {det_front_str}  {det_down_str}  "
                  f"best={(detection.camera if detection else 'none')} "
                  f"threshold={detector_min_confidence:.2f} ({_td:.2f}s)")

            if low_confidence:
                _rd0 = time.perf_counter()
                relocalized = relocalizer.search(
                    client, current_stage, front_image=front_rgb, down_image=down_rgb,
                    capture_mode=capture_mode,
                )
                print(f"  [Relocalize] found={relocalized.found} "
                      f"time={time.perf_counter() - _rd0:.2f}s reason=low confidence; {relocalized.reason}")
                continue

            # ═══ 完成判定 + 规划（顺序执行）═══
            completion = completion_checker.evaluate_with_detection(
                current_stage, ep["instruction"], front_rgb, down_rgb, detection,
                front_detection=front_det,
                down_detection=down_det,
                front_depth_meters=front_depth, down_depth_meters=down_depth,
            )
            detection = completion.detection
            direction = completion.direction
            if detection and detection.visible:
                ds = f"depth={detection.depth_median:.1f}m" if detection.depth_median else "depth=N/A"
                print(f"  [Completion] done={completion.done} reason={completion.reason} "
                      f"{detection.camera}:{detection.label} score={detection.score:.2f} {ds}")
            else:
                print(f"  [Completion] done={completion.done} reason={completion.reason}")

            if completion.done:
                if current_stage:
                    task_manager.complete_current(completion.reason)
                    print(f"  [TASK] {task_manager.summary()}")
                    task_finished = task_manager.is_done()
                else:
                    task_finished = True
                if task_finished:
                    break
                continue

            # ═══ Planner（顺序执行）═══
            _tp0 = time.perf_counter()
            result = planner.plan(
                front_rgb, down_rgb,
                instruction=stage_instruction,
                pending_waypoints=[],
                relation=getattr(current_stage, "relation", "") if current_stage else "",
                target=getattr(current_stage, "target", "") if current_stage else "",
            )
            _tp = time.perf_counter() - _tp0

            # 滑窗 Qwen 输出相邻增量点；候选扰动和 AirSim 执行使用累计机体系点。
            qwen_cumulative = incremental_to_cumulative(result.waypoints)
            prepared_result = SimpleNamespace(
                waypoints=qwen_cumulative,
                candidates=getattr(result, "candidates", []),
                reasoning=getattr(result, "reasoning", ""),
            )
            candidate_prep = prepare_candidates_for_world_model(
                prepared_result, detection=detection, direction=direction,
                stop_threshold=PLANNER_STOP_THRESHOLD,
            )
            selection = select_best_candidate(
                candidate_prep,
                world_model=world_model,
                front_image=front_rgb,
                down_image=down_rgb,
                instruction=stage_instruction,
            )
            for candidate_index, candidate in enumerate(candidate_prep.all_candidates):
                wm_text = ""
                if candidate_index < len(selection.world_model_scores):
                    wm_score = selection.world_model_scores[candidate_index]
                    if math.isfinite(wm_score):
                        wm_text = f" wm={wm_score:.4f}"
                points = " ".join(
                    f"[{wp[0]:.2f},{wp[1]:.2f},{wp[2]:.2f}]"
                    for wp in candidate.waypoints
                )
                print(
                    f"  [Candidate {candidate_index}] pre={candidate.pre_score:.4f}{wm_text} "
                    f"source={candidate.source} waypoints={points}"
                )
            if selection.chosen is not None:
                print(
                    f"  [Candidate] selected={selection.selected_index} "
                    f"by={'world_model' if selection.used_world_model else 'pre_score'} "
                    f"reason={selection.reasoning}"
                )

            # ═══ 执行 ═══
            exec_waypoints = (
                [list(wp) for wp in selection.chosen.waypoints]
                if selection.chosen is not None
                else qwen_cumulative
            )

            if exec_waypoints and not all(all(abs(v) < 1e-6 for v in wp) for wp in exec_waypoints):
                _te0 = time.perf_counter()
                pos_final, _, collided = client.execute_waypoints(exec_waypoints)
                _te = time.perf_counter() - _te0
                metrics.record_step(pos_final)
                wp_count = sum(1 for wp in exec_waypoints if not all(abs(v) < 1e-6 for v in wp))
                print(f"  [Step {step:3d}] plan={_tp:.1f}s  exec={_te:.1f}s  "
                      f"wp={wp_count}  GT_dist={dist_to_gt:.1f}m"
                      f"{' ✅' if dist_to_gt < SUCCESS_RADIUS else ''}")
            else:
                print(f"  [Step {step:3d}] no valid waypoints, stopping")
                break

            if result.done:
                if current_stage:
                    task_manager.complete_current("planner reported done")
                    print(f"  [TASK] {task_manager.summary()}")
                    if task_manager.is_done():
                        task_finished = True
                        break

        # 记录指标
        ne = metrics.ne
        sr = ne is not None and ne < SUCCESS_RADIUS
        osr_flag = metrics.osr
        spl = metrics.spl

        all_metrics["sr"] += 1 if sr else 0
        all_metrics["osr"] += 1 if osr_flag else 0
        if ne is not None:
            all_metrics["ne_list"].append(ne)
        all_metrics["spl_list"].append(spl)
        all_metrics["trajectory_lengths"].append(metrics.trajectory_length)

        print(f"  NE={ne:.1f}m SR={'✅' if sr else '❌'} OSR={osr_flag} SPL={spl:.2f} "
              f"finished={task_finished}")

    # 汇总
    n = len(episodes)
    if n > 0:
        print("\n" + "=" * 50)
        print(f"评估完成: {n} 条")
        print(f"  SR  (Success Rate):          {all_metrics['sr']/n*100:.2f}%")
        print(f"  OSR (Oracle Success Rate):   {all_metrics['osr']/n*100:.2f}%")
        if all_metrics['ne_list']:
            print(f"  NE  (Navigation Error):      {np.mean(all_metrics['ne_list']):.2f}m")
        print(f"  SPL (Path Length Weighted):  {np.mean(all_metrics['spl_list']):.4f}")
        print(f"  平均路径长度:                {np.mean(all_metrics['trajectory_lengths']):.1f}m")
        print("=" * 50)

    client.cleanup()



def _group_by_scene(episodes: List[Dict]) -> Dict[str, List[Dict]]:
    """按场景分组，保持数据集原始顺序。"""
    groups: Dict[str, List[Dict]] = {}
    for ep in episodes:
        groups.setdefault(ep["scene"], []).append(ep)
    return groups


def evaluate_multi_scene(
    episodes: List[Dict],
    env_root: str,
    gpu_id: int = 0,
    remote_host: str | None = None,
    remote_user: str | None = None,
    remote_port: int = 22,
):
    """多场景自动切换评估。

    按场景分组 → 逐场景启动 UE4 → 跑该场景所有轨迹 → 切换下一场景。
    """
    from eval.scene_manager import SceneManager
    eval_cfg = cfg.get("EVAL", {})
    if remote_host is None:
        remote_host = eval_cfg.get("REMOTE_HOST")
    if remote_user is None:
        remote_user = eval_cfg.get("REMOTE_USER")
    if remote_port == 22:
        remote_port = int(eval_cfg.get("REMOTE_PORT", 22))

    scene_groups = _group_by_scene(episodes)
    scene_order = sorted(scene_groups.keys())
    print(f"\n数据集含 {len(scene_order)} 个场景: {scene_order}")

    manager = SceneManager(
        env_root,
        gpu_id=gpu_id,
        remote_host=remote_host,
        remote_user=remote_user,
        remote_port=remote_port,
    )
    available = manager.available_scenes
    print(f"可用 UE4 环境 {len(available)} 个场景: {available[:5]}...")

    all_scene_metrics = {}

    for scene_name in scene_order:
        eps = scene_groups[scene_name]
        print(f"\n{'='*50}")
        print(f"  场景: {scene_name} ({len(eps)} 条)")
        print(f"{'='*50}")

        # 按场景排序 → 同场景的 episode 已连续排列
        # evaluate() 内部检测到新场景会自动调用 manager.start()
        sorted_eps = sorted(eps, key=lambda e: e["name"])

        if not manager.start(scene_name):
            print(f"  [SKIP] 场景 '{scene_name}' 的 UE4 环境不可用, 跳过 {len(eps)} 条")
            continue

        evaluate(sorted_eps, scene_filter=None,
                 scene_manager=manager, current_scene=scene_name)

    manager.stop()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="闭环评测器（需 AirSim）")
    parser.add_argument("--dataset", required=True, help="数据集根目录 (如 UAV-VLN-FOV/test)")
    parser.add_argument("--scene", default=None, help="只跑指定场景（单场景模式）")
    parser.add_argument("--env_root", default=None,
                        help="TravelUAV 环境根目录（启用多场景自动切换）")
    parser.add_argument("--gpu", type=int, default=0,
                        help="GPU ID (多场景模式, 默认 0)")
    parser.add_argument("--remote_host", default=None,
                        help="远程场景服务器地址；设置后通过 ssh/scp 在服务器上切换场景")
    parser.add_argument("--remote_user", default=None,
                        help="远程场景服务器用户名")
    parser.add_argument("--remote_port", type=int, default=22,
                        help="SSH 端口，默认 22")
    args = parser.parse_args()

    episodes = discover_episodes(args.dataset)

    if args.env_root:
        # 多场景自动切换模式
        evaluate_multi_scene(
            episodes,
            args.env_root,
            gpu_id=args.gpu,
            remote_host=args.remote_host,
            remote_user=args.remote_user,
            remote_port=args.remote_port,
        )
    else:
        # 单场景/手动模式（保持原有行为）
        evaluate(episodes, scene_filter=args.scene)
