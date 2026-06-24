"""Planner - orchestrates VLMClient + PromptBuilder + ResponseParser."""
from .vlm_client import VLMClient
from .prompt_builder import PromptBuilder
from .response_parser import ResponseParser
from .context_manager import ContextManager
from .target_depth import (build_target_bbox_messages, choose_target_estimate,
                           estimate_depth_in_bbox, parse_target_candidates)


class Planner:
    """High-level planner: encode frames, call VLM, parse response."""

    def __init__(self, config):
        self.config = config
        self.vlm = VLMClient(config)
        self.prompter = PromptBuilder(config)
        self.action_mode = getattr(config, "action_mode", "atomic")
        self.parser = ResponseParser()
        self.context = ContextManager(max_steps=getattr(config, "context_max_steps", 5))

    def generate_candidates(self, current_frame, task_description: str,
                            k=None, conversation_history=None, depth_frame=None,
                            center_depth=None, depth_meters=None):
        """Main entry point: returns
        (trajectories, scene_analysis, raw_json, reasoning_content,
         reasoning_summary, task_done, selected_index).
        center_depth: (center_min_meters, center_avg_meters) or a single float.
        """
        if k is None:
            k = self.config.candidate_count

        target_depth = self.estimate_target_depth(
            current_frame, task_description, depth_meters
        )
        depth_context = self._merge_depth_context(center_depth, target_depth)

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

        response_text, reasoning_content = self.vlm.call(messages)
        raw_json = ""

        target_visible = depth_context.get("target_visible") if isinstance(depth_context, dict) else None
        trajectories, scene_analysis, reasoning_summary, task_done, selected_index, raw_candidates = \
            self.parser.parse(response_text, self.config, k, target_visible=target_visible)

        return (trajectories, scene_analysis, raw_json, reasoning_content,
                reasoning_summary, task_done, selected_index, raw_candidates)

    def estimate_target_depth(self, current_frame, task_description: str,
                              depth_meters=None):
        """Ask the MLLM for target bbox, then compute depth from raw meters."""
        if not getattr(self.config, "target_depth_enabled", True):
            return None
        if current_frame is None or depth_meters is None:
            return None
        bbox_response = ""
        try:
            base64_image = self.prompter.encode_image(current_frame)
            image_size = current_frame.size
            messages = build_target_bbox_messages(base64_image, task_description, image_size)
            response_text, reasoning_content = self.vlm.call(messages, max_tokens=2048)
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
                        "请基于同一张图片，立即只输出一行JSON，字段为visible、bbox_norm、confidence、target、candidates；"
                        "bbox_norm必须是4个0到1之间的数字。"
                    )},
                ]
                response_text, reasoning_content = self.vlm.call(retry_messages, max_tokens=2048)
                bbox_response = response_text or reasoning_content
                estimates = parse_target_candidates(bbox_response, image_size)
            for estimate in estimates:
                estimate_depth_in_bbox(depth_meters, estimate, bbox_image_size=image_size)
            chosen = choose_target_estimate(estimates, task_description)
            depth_shape = getattr(depth_meters, "shape", None)
            debug_items = [
                f"target={estimate.target or 'unknown'} raw_bbox={estimate.raw_bbox} "
                f"rgb_bbox={estimate.bbox} depth_bbox={estimate.depth_bbox} "
                f"conf={estimate.confidence:.2f} median={estimate.depth_median} "
                f"min={estimate.depth_min} mean={estimate.depth_mean} pixels={estimate.valid_pixel_count}"
                for estimate in estimates if estimate.visible
            ]
            print(f"[TARGET DEPTH] image_size={image_size} depth_shape={depth_shape}")
            print(f"[TARGET DEPTH] candidates: {'; '.join(debug_items) or 'none'}")
            print(f"[TARGET DEPTH] chosen: target={chosen.target or 'unknown'} "
                  f"rgb_bbox={chosen.bbox} depth_bbox={chosen.depth_bbox} "
                  f"median={chosen.depth_median} min={chosen.depth_min}")
            return chosen.as_dict()
        except Exception as exc:
            print(f"[TARGET DEPTH] skipped: {exc}")
            if bbox_response:
                print(f"[TARGET DEPTH] raw bbox response: {bbox_response[:500]}")
            return None

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
