"""Planner - orchestrates VLMClient + PromptBuilder + ResponseParser."""
import time

from .vlm_client import VLMClient
from .prompt_builder import PromptBuilder
from .response_parser import ResponseParser
from .context_manager import ContextManager
from .target_depth import (build_target_bbox_messages, choose_target_estimate,
                           TargetDepthEstimate, estimate_depth_in_bbox,
                           parse_target_candidates)
from planner.trajectory import Trajectory, add_noise, compute_delta


class Planner:
    """High-level planner: encode frames, call VLM, parse response."""

    def __init__(self, config):
        self.config = config
        self.vlm = VLMClient(config)
        self.prompter = PromptBuilder(config)
        self.action_mode = getattr(config, "action_mode", "atomic")
        self.parser = ResponseParser()
        self.context = ContextManager(config, max_steps=getattr(config, "context_max_steps", 5))
        self.last_timing = {}

    def generate_candidates(self, current_frame, task_description: str,
                            k=None, conversation_history=None, depth_frame=None,
                            center_depth=None, depth_meters=None,
                            step_num: int = 0, pose=None, yaw_deg=None):
        """Main entry point: returns
        (trajectories, scene_analysis, raw_json, reasoning_content,
         reasoning_summary, task_done, selected_index).
        center_depth: (center_min_meters, center_avg_meters) or a single float.
        """
        if k is None:
            k = self.config.candidate_count
        self.context.ensure_task(task_description)

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

        target_depth = self.estimate_target_depth(
            current_frame, task_description, depth_meters,
            step_num=step_num, pose=pose, yaw_deg=yaw_deg,
        )
        image_size = current_frame.size if current_frame is not None else None
        target_bbox = target_depth.get("target_bbox") if isinstance(target_depth, dict) else None
        self.context.update_obstacle_memory(depth_meters, image_size=image_size, target_bbox=target_bbox)
        depth_context = self._merge_depth_context(center_depth, target_depth)

        local_result = self._try_local_context_action(target_depth, k, step_started, yaw_deg)
        if local_result is not None:
            return local_result

        prompt_started = time.perf_counter()
        messages = self.prompter.build_messages(
            current_frame, task_description, k,
            conversation_history, depth_frame,
            center_depth=depth_context,
        )

        # Merge conversation history from context manager if none explicitly given
        if conversation_history is None:
            conv_msgs = self.context.get_messages()
            if conv_msgs:
                # Place history before the current user message
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
                              pose=None, yaw_deg=None):
        """Ask the MLLM for target bbox, then compute depth from raw meters."""
        if not getattr(self.config, "target_depth_enabled", True):
            return None
        if current_frame is None or depth_meters is None:
            return None
        bbox_response = ""
        target_started = time.perf_counter()
        try:
            reused = self._try_reuse_target_depth(current_frame, depth_meters, step_num, pose, yaw_deg)
            if reused is not None:
                return reused

            encode_started = time.perf_counter()
            base64_image = self.prompter.encode_image(current_frame)
            image_size = current_frame.size
            messages = build_target_bbox_messages(base64_image, task_description, image_size)
            self.last_timing["target_depth_encode"] = time.perf_counter() - encode_started
            response_text, reasoning_content = self.vlm.call(messages, max_tokens=2048)
            bbox_call = dict(self.vlm.last_call_info)
            bbox_call["name"] = "target_bbox"
            self.last_timing["target_bbox_api"] = bbox_call.get("elapsed", 0.0)
            self.last_timing["vlm_calls"].append(bbox_call)
            bbox_response = response_text or reasoning_content
            if not bbox_response.strip():
                print("[TARGET DEPTH] empty bbox response from MLLM")
                return None
            try:
                estimates = parse_target_candidates(bbox_response, image_size)
            except Exception as first_exc:
                retry_messages = messages + [
                    {"role": "assistant", "content": bbox_response[:1200]},
                    {"role": "user", "content": (
                        "上一条没有给出可解析JSON。不要分析，不要解释，不要复述规则。"
                        "请基于同一张图片，只输出一行JSON，字段只允许visible、bbox_norm、confidence、target。"
                        "禁止输出candidates。bbox_norm必须是4个0到1之间的数字。"
                    )},
                ]
                response_text, reasoning_content = self.vlm.call(retry_messages, max_tokens=2048)
                retry_call = dict(self.vlm.last_call_info)
                retry_call["name"] = "target_bbox_retry"
                self.last_timing["target_bbox_retry_api"] = retry_call.get("elapsed", 0.0)
                self.last_timing["vlm_calls"].append(retry_call)
                bbox_response = response_text or reasoning_content
                estimates = parse_target_candidates(bbox_response, image_size)
            compute_started = time.perf_counter()
            for estimate in estimates:
                estimate_depth_in_bbox(depth_meters, estimate, bbox_image_size=image_size)
            chosen = choose_target_estimate(estimates, task_description)
            self.last_timing["target_depth_compute"] = time.perf_counter() - compute_started
            depth_shape = getattr(depth_meters, "shape", None)
            print(f"[TARGET DEPTH] image_size={image_size} depth_shape={depth_shape}")
            print(f"[TARGET DEPTH] target={chosen.target or 'unknown'} "
                  f"rgb_bbox={chosen.bbox} depth_bbox={chosen.depth_bbox} "
                  f"median={chosen.depth_median}")
            chosen_dict = chosen.as_dict()
            self.context.annotate_estimate_geometry(chosen_dict, image_size, yaw_deg)
            decision = self.context.evaluate_observation(
                chosen_dict, step_num, pose, yaw_deg, task_description
            )
            print(f"[CONTEXT] status={decision.status.value} reason={decision.reason}")
            if not decision.accepted:
                return {"target_visible": False, "target_recovery": True}
            return chosen_dict
        except Exception as exc:
            print(f"[TARGET DEPTH] skipped: {exc}")
            if bbox_response:
                print(f"[TARGET DEPTH] raw bbox response: {bbox_response[:500]}")
            self.context.update_target_from_estimate(None, step_num, pose, yaw_deg, source="bbox_api_error")
            return None
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

    def clear_context(self):
        """Reset conversation history (e.g. on new task)."""
        self.context.clear()

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
        self.context.annotate_estimate_geometry(estimate_dict, current_frame.size, yaw_deg)
        decision = self.context.evaluate_observation(
            estimate_dict,
            step_num,
            pose,
            yaw_deg,
            self.context.task.instruction,
        )
        print(f"[CONTEXT] status={decision.status.value} reason={decision.reason}")
        return estimate_dict if decision.accepted else {"target_visible": False, "target_recovery": True}

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

    def _try_local_context_action(self, target_depth, k: int, step_started: float, yaw_deg):
        """Skip planning API when context can safely produce a local action."""
        target_value = None
        if isinstance(target_depth, dict) and target_depth.get("target_visible"):
            target_value = target_depth.get("target_depth_median")
        candidate_actions = []
        reason = ""
        scene = ""
        summary = ""
        decision = self.context.last_decision
        if decision.allow_auto_done:
            self.last_timing["planner_total"] = time.perf_counter() - step_started
            scene = "目标已经在近距离范围内。"
            depth_text = "未知" if decision.expected_depth is None else f"{float(decision.expected_depth):.2f}m"
            summary = f"状态机判定已到达目标附近，预期目标深度约{depth_text}，停止继续前进。"
            return ([], scene, "", "", summary, True, 0, [])
        if target_value is not None and self.context.should_use_local_forward(target_value):
            scales = [1.0, 0.8, 0.6, 0.4, 0.25]
            seen = set()
            for scale in scales[:max(1, k)]:
                actions = self.context.build_local_forward_actions(float(target_value), distance_scale=scale)
                key = tuple(actions)
                if actions and key not in seen:
                    candidate_actions.append(actions)
                    seen.add(key)
            if candidate_actions:
                reason = "目标较远且目标方向深度通道安全，使用上下文本地连续前进，跳过规划API。"
                scene = "目标已锁定，前方/目标方向深度通道安全。"
                summary = f"目标深度约{float(target_value):.2f}m；根据上下文和深度通道生成本地前进轨迹。"
        if not candidate_actions and decision.recovery_action:
            actions = decision.recovery_action
            if actions:
                candidate_actions.append(actions)
                reason = "目标短暂丢失，依据搜索记忆转回历史目标方向，跳过规划API。"
                scene = "当前帧未可靠锁定目标，使用历史目标方向恢复。"
                summary = "目标短暂丢失；不切换目标，优先回到上次看到目标的方向。"
        if not candidate_actions:
            return None

        trajectories = []
        raw_candidates = []
        for idx, actions in enumerate(candidate_actions[:max(1, k)]):
            trajectory = Trajectory(
                actions=actions,
                clean_delta=compute_delta(actions, self.config),
                name=f"context_local_{idx + 1}",
                scale=1.0,
            )
            trajectory.noisy_delta = add_noise(trajectory.clean_delta, self.config)
            trajectories.append(trajectory)
            raw_candidates.append({"actions": actions, "reason": reason, "scale": 1.0})
        self.last_timing["planner_total"] = time.perf_counter() - step_started
        return (trajectories, scene, "", "", summary, False, 0, raw_candidates)
