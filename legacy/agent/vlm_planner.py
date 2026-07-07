"""Planner - orchestrates VLMClient + PromptBuilder + ResponseParser."""
import time

from .vlm_client import VLMClient
from .prompt_builder import PromptBuilder
from .response_parser import ResponseParser
from .context_manager import ContextManager
from .target_identity import TargetIdentityManager
from .target_depth import (choose_target_estimate,
                           TargetDepthEstimate, estimate_depth_in_bbox, normalize_bbox)


class Planner:
    """High-level planner: encode frames, call VLM, parse response."""

    def __init__(self, config):
        self.config = config
        self.vlm = VLMClient(config)
        self.prompter = PromptBuilder(config)
        self.action_mode = getattr(config, "action_mode", "atomic")
        self.parser = ResponseParser()
        self.context = ContextManager(config, max_steps=getattr(config, "context_max_steps", 5))
        self.identity = TargetIdentityManager(config)
        self.last_timing = {}
        self.last_target_missing = False
        self.last_target_depth = None

    def generate_candidates(self, current_frame, task_description: str,
                            k=None, conversation_history=None, depth_frame=None,
                            center_depth=None, depth_meters=None,
                            step_num: int = 0, pose=None, yaw_deg=None,
                            task_key: str = None,
                            target_depth_enabled: bool = None,
                            allow_relocalize: bool = None,
                            encoded_rgb: str = None,
                            detect_only: bool = False,
                            down_frame=None):
        """Main entry point: returns
        (trajectories, scene_analysis, raw_json, reasoning_content,
         reasoning_summary, task_done, selected_index).
        center_depth: (center_min_meters, center_avg_meters) or a single float.
        """
        if k is None:
            k = self.config.candidate_count
        task_key = task_key or task_description
        effective_target_depth_enabled = (
            getattr(self.config, "target_depth_enabled", True)
            if target_depth_enabled is None
            else bool(target_depth_enabled)
        )
        effective_allow_relocalize = (
            getattr(self.config, "relocalizer_enabled", True)
            if allow_relocalize is None
            else bool(allow_relocalize)
        )
        self.context.ensure_task(task_key)

        step_started = time.perf_counter()
        self.last_timing = {
            "target_depth_total": 0.0,
            "target_bbox_api": 0.0,
            "target_bbox_retry_api": 0.0,
            "target_depth_compute": 0.0,
            "target_depth_encode": 0.0,
            "prompt_build": 0.0,
            "planning_api": 0.0,
            "parse": 0.0,
            "planner_total": 0.0,
            "vlm_calls": [],
        }
        # 存储下视图像供 Qwen 使用
        if down_frame is not None:
            self._last_down_frame = down_frame

        target_depth, encoded_rgb = self.estimate_target_depth(
            current_frame, task_description, depth_meters,
            step_num=step_num, pose=pose, yaw_deg=yaw_deg,
            task_key=task_key,
            target_depth_enabled=effective_target_depth_enabled,
            encoded_rgb=encoded_rgb,
        )
        self.last_target_depth = target_depth
        image_size = current_frame.size if current_frame is not None else None
        self.last_target_missing = (
            (isinstance(target_depth, dict) and target_depth.get("target_visible") is False)
            or (
                target_depth is None
                and getattr(self.config, "target_depth_enabled", True)
                and effective_target_depth_enabled
                and current_frame is not None
                and depth_meters is not None
            )
        )
        target_bbox = target_depth.get("target_bbox") if isinstance(target_depth, dict) else None
        self.context.update_obstacle_memory(depth_meters, image_size=image_size, target_bbox=target_bbox)
        depth_context = self._merge_depth_context(center_depth, target_depth)
        if isinstance(depth_context, dict):
            last_decision = getattr(self.context, "last_decision", None)
            if last_decision is not None:
                status = getattr(last_decision, "status", None)
                depth_context["context_status"] = getattr(status, "value", str(status))
                depth_context["context_reason"] = getattr(last_decision, "reason", "")
                depth_context["arrival_depth"] = float(getattr(self.config, "context_arrival_depth", 5.0))

        if detect_only:
            target_visible = bool(
                isinstance(target_depth, dict)
                and target_depth.get("target_visible")
                and not target_depth.get("target_identity_rejected")
            )
            self.last_timing["planning_api"] = 0.0
            self.last_timing["planner_total"] = time.perf_counter() - step_started
            if target_visible:
                target_name = target_depth.get("target_name") or target_depth.get("target") or "target"
                target_depth_m = target_depth.get("target_depth_median")
                depth_text = f"，depth={target_depth_m}" if target_depth_m is not None else ""
                print(f"[DETECT] target locked; skip planner API target={target_name}{depth_text}")
                return (
                    [],
                    f"检测阶段已发现并锁定目标：{target_name}{depth_text}。",
                    "",
                    "",
                    "检测阶段只调用bbox/场景目标识别，不调用planner API；目标已写入身份锁和上下文，供后续阶段复用。",
                    True,
                    0,
                    [],
                )
            print("[DETECT] target not reliably visible; skip planner API")
            return (
                [],
                "检测阶段未可靠发现任务目标。",
                "",
                "",
                "检测阶段只调用bbox/场景目标识别，不调用planner API；当前未锁定目标，主循环可进入重定位或等待下一帧。",
                False,
                0,
                [],
            )

        if self.last_target_missing and effective_allow_relocalize:
            self.last_timing["planning_api"] = 0.0
            self.last_timing["planner_total"] = time.perf_counter() - step_started
            return (
                [],
                "当前帧未可靠发现任务目标。",
                "",
                "",
                "当前帧未检测到任务目标，跳过规划API，交给四向环视重定位模块。",
                False,
                0,
                [],
            )

        local_result = self._try_local_context_action(target_depth, k, step_started, yaw_deg, pose)
        if local_result is not None:
            return local_result

        prompt_started = time.perf_counter()
        # Reuse the already-encoded base64 image from estimate_target_depth
        if encoded_rgb:
            messages = self.prompter.build_messages_encoded(
                encoded_rgb, task_description, k,
                conversation_history, depth_frame,
                center_depth=depth_context,
            )
        else:
            messages = self.prompter.build_messages(
                current_frame, task_description, k,
                conversation_history, depth_frame,
                center_depth=depth_context,
            )

        use_qwen = bool(getattr(self.config, "use_qwen", False))

        if use_qwen:
            parse_started = time.perf_counter()
            # ═══ Qwen2.5-VL 轨迹规划（服务器 8004） ═══
            from agent.planner.qwen_planner import QwenPlanner
            qwen_plan = QwenPlanner().plan
            from agent.target_depth import detect_with_groundingdino
            from planner.trajectory import Trajectory

            planning_started = time.perf_counter()

            # 获取下视图像（从 planner 传入参数中找或复用当前帧）
            down_img = getattr(self, "_last_down_frame", current_frame)

            # ═══ 方向更新（下视优先，论文 Sec.IV-D） ═══
            from agent.direction.three_dg_estimator import estimate_direction
            from agent.target_depth import detect_with_groundingdino

            front_bbox = None
            down_bbox = None

            # 1. 前视检测
            if isinstance(depth_context, dict):
                front_bbox = depth_context.get("target_bbox")
                if not front_bbox and depth_context.get("scene_objects"):
                    for obj in depth_context["scene_objects"]:
                        if obj.get("bbox") and obj.get("role") == "target":
                            front_bbox = obj["bbox"]
                            break

            # 2. 下视检测（优先！）
            down_img = getattr(self, "_last_down_frame", None)
            if down_img is not None:
                down_result = detect_with_groundingdino(down_img, task_description)
                if down_result.visible and down_result.bbox:
                    down_bbox = down_result.bbox

            # 3. 下视优先方向判定（论文 Eq.14-15）
            direction = ""
            if down_bbox is not None:
                # 下视检测到 → 目标在下方 → 优先
                direction = "It's down below you"
            elif front_bbox is not None:
                # 前视检测到 → 计算 yaw 方向
                direction = estimate_direction(
                    front_bbox, camera_id=0,
                    image_size=(current_frame.width, current_frame.height),
                )
            # 都没检测到 → 保留空字符串（不更新方向）

            # ═══ 停止策略（论文：检测目标 + bbox 中心深度 < 阈值 → stop） ═══
            import os as _os
            _stop_threshold = float(_os.environ.get("STOP_DEPTH_THRESHOLD", "8.0"))
            task_done = False
            if isinstance(depth_context, dict) and depth_context.get("target_visible"):
                depth_m = depth_context.get("target_depth_median")
                if depth_m is not None and depth_m < _stop_threshold:
                    task_done = True
                    print(f"  [STOP] target depth {depth_m:.1f}m < {_stop_threshold:.0f}m → reached!")

            # 调用 Qwen 服务
            # 提取简洁指令（去掉 TaskManager 的多余包装）
            clean_instruction = task_description
            if "原始任务：" in task_description:
                import re as _re
                m = _re.search(r"原始任务：(.+?)(?:\n|$)", task_description)
                if m:
                    clean_instruction = m.group(1).strip()
            raw_waypoints = qwen_plan(
                current_frame, down_img,
                instruction=clean_instruction,
                direction=direction,
            )
            self.last_timing["planning_api"] = time.perf_counter() - planning_started
            self.last_timing["vlm_calls"].append({
                "name": "qwen_planning",
                "elapsed": self.last_timing["planning_api"],
            })

            # 构造单条轨迹
            trajectories = []
            if raw_waypoints:
                traj = Trajectory(
                    actions=[f"waypoint {i+1}" for i in range(len(raw_waypoints))],
                    name="qwen_trajectory",
                    waypoints=raw_waypoints,
                )
                trajectories.append(traj)

            scene_analysis = f"Qwen规划: {len(raw_waypoints)} waypoints" + (" [到达!]" if task_done else "")
            reasoning_summary = f"Qwen2.5-VL 输出 {len(raw_waypoints)} 个轨迹点: {raw_waypoints}"
            selected_index = 0 if trajectories else -1
            raw_json = ""
            reasoning_content = ""
            raw_candidates = [{"waypoints": raw_waypoints}]

        else:
            # ═══ 原有 VLM 规划路径 ═══
            # Merge conversation history from context manager if none explicitly given
            if conversation_history is None:
                conv_msgs = self.context.get_messages()
                if conv_msgs:
                    messages[1:1] = conv_msgs
            self.last_timing["prompt_build"] = time.perf_counter() - prompt_started

            response_text, reasoning_content = self.vlm.call(
                messages,
                max_tokens=getattr(self.config, "planner_max_tokens", 8192),
            )
            planning_call = dict(self.vlm.last_call_info)
            planning_call["name"] = "planning"
            self.last_timing["planning_api"] = planning_call.get("elapsed", 0.0)
            self.last_timing["vlm_calls"].append(planning_call)
            parse_text = response_text or reasoning_content
            raw_json = ""

            parse_started = time.perf_counter()
            target_visible = depth_context.get("target_visible") if isinstance(depth_context, dict) else None
            trajectories, scene_analysis, reasoning_summary, task_done, selected_index, raw_candidates = \
                self.parser.parse(parse_text, self.config, k, target_visible=target_visible)
        self.last_timing["parse"] = time.perf_counter() - parse_started
        self.last_timing["planner_total"] = time.perf_counter() - step_started

        return (trajectories, scene_analysis, raw_json, reasoning_content,
                reasoning_summary, task_done, selected_index, raw_candidates)

    def estimate_target_depth(self, current_frame, task_description: str,
                              depth_meters=None, step_num: int = 0,
                              pose=None, yaw_deg=None,
                              task_key: str = None,
                              target_depth_enabled: bool = None,
                              encoded_rgb: str = None):
        """Ask the MLLM for target bbox, then compute depth from raw meters.

        Returns (target_depth_dict, encoded_base64_image).
        The base64 image is reused by generate_candidates for the planning API.
        """
        base64_image = encoded_rgb
        effective_target_depth_enabled = (
            getattr(self.config, "target_depth_enabled", True)
            if target_depth_enabled is None
            else bool(target_depth_enabled)
        )
        if not effective_target_depth_enabled:
            return None, base64_image
        if current_frame is None or depth_meters is None:
            return None, base64_image
        bbox_response = ""
        target_started = time.perf_counter()
        try:
            reused = self._try_reuse_target_depth(current_frame, depth_meters, step_num, pose, yaw_deg)
            if reused is not None:
                return reused, base64_image

            if base64_image:
                self.last_timing["target_depth_encode"] = 0.0
            else:
                encode_started = time.perf_counter()
                base64_image = self.prompter.encode_image(current_frame)
                self.last_timing["target_depth_encode"] = time.perf_counter() - encode_started
            image_size = current_frame.size
            # ═══ GroundingDINO 检测（替换 VLM） ═══
            use_scene_objects = bool(getattr(self.config, "scene_obstacle_planning_enabled", True))
            bbox_started = time.perf_counter()
            if use_scene_objects:
                # 多物体检测（目标 + 障碍物）
                from agent.target_depth import detect_scene_with_groundingdino
                estimates = detect_scene_with_groundingdino(
                    current_frame, task_description,
                    depth_meters=depth_meters,
                    bbox_image_size=image_size,
                )
            else:
                # 单目标检测
                from agent.target_depth import detect_with_groundingdino
                chosen = detect_with_groundingdino(
                    current_frame, task_description,
                    depth_meters=depth_meters,
                    bbox_image_size=image_size,
                )
                estimates = [chosen] if chosen.visible else [chosen]
            self.last_timing["target_bbox_api"] = time.perf_counter() - bbox_started
            bbox_response = ""  # GroundingDINO 无原始文本
            compute_started = time.perf_counter()
            for estimate in estimates:
                estimate_depth_in_bbox(depth_meters, estimate, bbox_image_size=image_size)
            chosen = choose_target_estimate(estimates, task_description)
            scene_objects = [
                estimate.scene_dict(index)
                for index, estimate in enumerate(estimates, start=1)
                if estimate.visible and estimate.bbox
            ]
            self.last_timing["target_depth_compute"] = time.perf_counter() - compute_started
            depth_shape = getattr(depth_meters, "shape", None)
            print(f"[TARGET DEPTH] image_size={image_size} depth_shape={depth_shape}")
            print(f"[TARGET DEPTH] target={chosen.target or 'unknown'} "
                  f"rgb_bbox={chosen.bbox} depth_bbox={chosen.depth_bbox} "
                  f"median={chosen.depth_median}")
            if use_scene_objects and scene_objects:
                object_summary = "; ".join(
                    f"{obj['id']}:{obj['role']}:{obj['name']}:{obj['position']} depth={obj['depth_median']}"
                    for obj in scene_objects[:8]
                )
                print(f"[SCENE OBJECTS] {object_summary}")
            chosen_dict = chosen.as_dict()
            chosen_dict["scene_objects"] = scene_objects
            self.context.annotate_estimate_geometry(chosen_dict, image_size, yaw_deg, pose)
            identity_decision = self.identity.evaluate(chosen_dict, step_num, task_key or task_description)
            if not identity_decision.accepted:
                print(f"[IDENTITY] rejected target: {identity_decision.reason} "
                      f"distance={identity_decision.distance_m} tolerance={identity_decision.tolerance_m}")
                chosen_dict = self.identity.reject_estimate(chosen_dict, identity_decision)
            else:
                print(f"[IDENTITY] accepted target: {identity_decision.reason}")
            decision = self.context.evaluate_observation(
                chosen_dict, step_num, pose, yaw_deg, task_description
            )
            print(f"[CONTEXT] status={decision.status.value} reason={decision.reason}")
            return chosen_dict, base64_image
        except Exception as exc:
            print(f"[TARGET DEPTH] skipped: {exc}")
            if bbox_response:
                print(f"[TARGET DEPTH] raw bbox response: {bbox_response[:500]}")
            self.context.update_target_from_estimate(None, step_num, pose, yaw_deg, source="bbox_api_error")
            return None, base64_image
        finally:
            self.last_timing["target_depth_total"] = time.perf_counter() - target_started

    def _merge_depth_context(self, center_depth, target_depth):
        """Merge scene depth stats and target bbox depth stats for prompts."""
        if isinstance(center_depth, dict):
            merged = dict(center_depth)
        elif center_depth is None:
            merged = {}
        else:
            merged = {"center_depth": center_depth}
        if target_depth:
            merged.update(target_depth)
        return merged or None

    def record_step(self, step_num: int, actions: list,
                    old_pos, new_pos, yaw_deg: float, collided: bool):
        """Record a completed step into the context manager for future VLM calls."""
        self.context.add_step(step_num, actions, old_pos, new_pos, yaw_deg, collided)

    def clear_context(self, clear_identity: bool = True):
        """Reset conversation history (e.g. on new task)."""
        self.context.clear()
        if clear_identity:
            self.identity.clear()

    def _try_reuse_target_depth(self, current_frame, depth_meters, step_num: int, pose, yaw_deg):
        """Reuse the previous target bbox and recompute depth on the current depth map."""
        if not self.context.should_try_reuse_target_bbox(step_num):
            return None
        target = self.context.target
        estimate = TargetDepthEstimate(
            visible=True,
            bbox=list(target.bbox),
            confidence=target.confidence,
            target=target.target_class,
        )
        estimate_depth_in_bbox(depth_meters, estimate, bbox_image_size=current_frame.size)
        if not self.context.accept_reused_depth(estimate.depth_median, estimate.valid_pixel_count):
            print("[TARGET DEPTH] bbox reuse rejected; falling back to bbox API")
            return None
        depth_shape = getattr(depth_meters, "shape", None)
        print(f"[TARGET DEPTH] image_size={current_frame.size} depth_shape={depth_shape}")
        print(f"[TARGET DEPTH] reused target={estimate.target or 'unknown'} "
              f"rgb_bbox={estimate.bbox} depth_bbox={estimate.depth_bbox} "
              f"median={estimate.depth_median}")
        estimate_dict = estimate.as_dict()
        self.context.annotate_estimate_geometry(estimate_dict, current_frame.size, yaw_deg, pose)
        decision = self.context.evaluate_observation(
            estimate_dict,
            step_num,
            pose,
            yaw_deg,
            self.context.task.instruction,
        )
        print(f"[CONTEXT] status={decision.status.value} reason={decision.reason}")
        return estimate_dict

    def _target_memory_as_depth_dict(self):
        target = self.context.target
        if not target.has_target:
            return None
        return {
            "target_visible": True,
            "target_bbox": target.bbox,
            "target_depth_bbox": None,
            "target_confidence": target.confidence,
            "target_name": target.target_class,
            "target_depth_median": target.depth_median,
            "target_depth_min": None,
            "target_depth_mean": None,
            "target_depth_center_median": None,
            "target_depth_valid_pixels": 0,
        }

    def _try_local_context_action(self, target_depth, k: int, step_started: float, yaw_deg, pose=None):
        """Use deterministic context recovery when the locked target is lost.

        The VLM is good at planning when the target is visible. When the target is
        missing or a different class is detected, letting the VLM freely plan often
        turns a temporary loss into a target switch. In that state the program must
        take control and only rotate/relocalize until the locked target is visible
        again.
        """
        if getattr(self.config, "relocalizer_enabled", True):
            return None
        decision = getattr(self.context, "last_decision", None)
        if not decision:
            return None
        if not getattr(self.context, "context_enabled", False):
            return None
        if not getattr(self.context, "recovery_enabled", False):
            return None
        status = getattr(decision, "status", None)
        status_value = getattr(status, "value", status)
        explicit_missing_target = (
            status_value == "REDETECT"
            and isinstance(target_depth, dict)
            and target_depth.get("target_visible") is False
            and bool(target_depth.get("target_name"))
        )
        if not status or (status_value != "LOST_OR_OCCLUDED" and not explicit_missing_target):
            return None

        self.last_timing["planning_api"] = 0.0
        self.last_timing["planner_total"] = time.perf_counter() - step_started
        scene_analysis = "当前帧未可靠发现任务目标，等待独立环视重定位。"
        reasoning_summary = (
            f"目标不可见：{getattr(decision, 'reason', '')}。"
            "Planner 不输出动作，主循环将调用独立四向环视重定位模块。"
        )
        return ([], scene_analysis, "", "", reasoning_summary, False, 0, [])
