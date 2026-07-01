"""Builds system + user messages for the VLM using prompt templates."""
import base64, io
from PIL import Image
import numpy as np
from planner.prompt_templates import (PLANNER_SYSTEM_PROMPT, TASK_INSTRUCTION_PROMPT,
    PLANNER_SYSTEM_PROMPT_VALUE_MODE, VALUE_TASK_INSTRUCTION_PROMPT)


class PromptBuilder:
    """Formats prompt templates with config values and encodes images."""

    def __init__(self, config):
        self.config = config
        self.action_mode = getattr(config, "action_mode", "atomic")

    def encode_image(self, image) -> str:
        """Convert PIL Image or ndarray to base64 PNG string."""
        if isinstance(image, np.ndarray):
            image = Image.fromarray(image)
        elif isinstance(image, str):
            image = Image.open(image)
        if image is None:
            return ""
        if image.mode == "I;16":
            buffered = io.BytesIO()
            image.save(buffered, format="PNG")
        else:
            if image.mode != "RGB":
                image = image.convert("RGB")
            buffered = io.BytesIO()
            image.save(buffered, format="PNG")
        return base64.b64encode(buffered.getvalue()).decode("utf-8")

    def _format_depth_info(self, depth_data) -> str:
        """Format numeric depth hints captured from AirSim."""
        if not depth_data:
            return "深度提示：未获取到有效目标深度统计，请保守行动并优先小步观察。"
        if isinstance(depth_data, dict):
            parts = []
            context_status = depth_data.get("context_status")
            if context_status:
                parts.append(
                    f"上下文状态={context_status}，到达半径={float(getattr(self.config, 'context_arrival_depth', 5.0)):.2f}m"
                )
            target_visible = depth_data.get("target_visible")
            if target_visible:
                bbox = depth_data.get("target_bbox")
                target_name = depth_data.get("target_name") or "目标"
                confidence = depth_data.get("target_confidence")
                target_depth = depth_data.get("target_depth_median")
                target_min = depth_data.get("target_depth_min")
                target_mean = depth_data.get("target_depth_mean")
                target_center = depth_data.get("target_depth_center_median")
                target_parts = [f"{target_name} bbox={bbox}"]
                if confidence is not None:
                    target_parts.append(f"置信度={float(confidence):.2f}")
                if target_center is not None:
                    target_parts.append(f"目标中心中位深度={float(target_center):.2f}m")
                if target_depth is not None:
                    target_parts.append(f"目标中位深度={float(target_depth):.2f}m")
                if target_min is not None:
                    target_parts.append(f"目标最近深度={float(target_min):.2f}m")
                if target_mean is not None:
                    target_parts.append(f"目标平均深度={float(target_mean):.2f}m")
                parts.append("程序已从目标bbox内原始深度矩阵计算：" + "，".join(target_parts) + "；规划必须优先使用这些目标深度数值")
            elif target_visible is False:
                parts.append("目标检测：当前RGB图中未可靠发现目标")
            labels = {
                "center_min": "画面中心最近深度",
                "center_avg": "画面中心平均深度",
                "scene_min": "全画面最近深度",
                "scene_max": "全画面最远深度",
            }
            for key, label in labels.items():
                value = depth_data.get(key)
                if value is not None:
                    parts.append(f"{label}={float(value):.2f}m")
            scene_objects = depth_data.get("scene_objects") or []
            if scene_objects:
                object_parts = []
                for obj in scene_objects[:8]:
                    depth = obj.get("depth_median")
                    depth_text = "未知" if depth is None else f"{float(depth):.2f}m"
                    object_parts.append(
                        f"{obj.get('id')} role={obj.get('role')} name={obj.get('name')} "
                        f"位置={obj.get('position')} bbox={obj.get('bbox')} 深度={depth_text} "
                        f"说明={obj.get('description')}"
                    )
                parts.append(
                    "场景物体与障碍物深度列表：" + "；".join(object_parts)
                    + "。规划时必须根据这些物体的位置和深度避开障碍物，同时完成任务目标"
                )
            if parts:
                return "深度提示：" + "，".join(parts) + "。目标精确深度以程序计算的目标深度为准。"
        try:
            return f"深度提示：画面中心深度={float(depth_data):.2f}m。未获取到目标bbox深度，请保守行动并优先小步观察。"
        except (TypeError, ValueError):
            return "深度提示：未获取到有效目标深度统计，请保守行动并优先小步观察。"

    def build_system_prompt(self, task_description: str, k: int,
                             depth_info_str: str = "未知") -> str:
        template = PLANNER_SYSTEM_PROMPT_VALUE_MODE if self.action_mode == "value" else PLANNER_SYSTEM_PROMPT
        prompt = template.format(
            horizontal_step=self.config.horizontal_step,
            vertical_step=self.config.vertical_step,
            yaw_step_deg=self.config.yaw_step_deg,
            max_forward_step=self.config.max_forward_step,
            max_tracking_yaw_step_deg=self.config.max_tracking_yaw_step_deg,
            max_trajectory_length=self.config.max_trajectory_length,
            arrival_depth=getattr(self.config, "context_arrival_depth", 5.0),
            approach_stop_margin=getattr(self.config, "approach_stop_margin", 1.0),
            task_description=task_description,
            k=k,
            depth_info=depth_info_str,
        )
        return (
            prompt
            + "\n补充限制：只输出JSON，不要Markdown；轨迹必须使用深度提示中的目标/障碍物位置和深度；"
              f"任务是旁边/附近/接近/靠近时，可用 {getattr(self.config, 'context_arrival_depth', 5.0)}m 到达半径判断完成；"
              "到达半径只用于判断done=true/false，禁止把到达半径从目标深度中减去来生成forward距离；"
              "如果当前目标深度大于到达半径，说明任务尚未完成，应规划尽可能少步数接近目标；"
              f"动作距离应根据目标深度、障碍物深度和接近停止余量{getattr(self.config, 'approach_stop_margin', 1.0)}m决定，而不是target_depth-arrival_radius；"
              f"例如target_depth=41m、arrival_radius=5m、approach_stop_margin={getattr(self.config, 'approach_stop_margin', 1.0)}m时，不能因为半径为5m就输出forward 36；若路径安全，应接近输出forward 40；"
              "任务是上方/上面/顶部/侧方/后方/绕行/穿过时，必须到对应相对位置后才 done=true；"
              "未完成时在安全可通行、不越过目标、不切换目标实例的前提下，用尽可能少的动作完成任务，安全时优先单个较大的forward；"
              "当前帧有目标bbox和深度时必须基于当前目标规划；"
              "只有深度提示明确写着当前RGB未可靠发现目标时才只允许原地旋转搜索；"
              "相邻动作必须不同类型，连续同类动作必须合并。\n"
        )

    def build_user_content(self, base64_image: str, depth_base64: str,
                            task_description: str, k: int,
                            depth_info_str: str = "未知") -> list:
        content = [
            {"type": "image_url",
             "image_url": {"url": f"data:image/png;base64,{base64_image}"}},
        ]
        if depth_base64:
            content.append({
                "type": "image_url",
                "image_url": {"url": f"data:image/png;base64,{depth_base64}"}
            })
        inst_prompt = VALUE_TASK_INSTRUCTION_PROMPT if self.action_mode == "value" else TASK_INSTRUCTION_PROMPT
        content.append({
            "type": "text",
            "text": inst_prompt.format(
                task_description=task_description, k=k,
                max_trajectory_length=self.config.max_trajectory_length,
                depth_info=depth_info_str)
        })
        return content

    def build_messages(self, current_frame, task_description: str, k: int,
                       conversation_history=None, depth_frame=None,
                       center_depth=None) -> list:
        """Build the full messages list for the VLM API call."""
        base64_image = self.encode_image(current_frame)
        depth_base64 = self.encode_image(depth_frame) if depth_frame is not None else None
        depth_info_str = self._format_depth_info(center_depth)

        sys_prompt = self.build_system_prompt(task_description, k, depth_info_str)
        messages = [{"role": "system", "content": sys_prompt}]

        if conversation_history:
            messages.extend(conversation_history)

        user_content = self.build_user_content(base64_image, depth_base64,
                                                task_description, k, depth_info_str)
        messages.append({"role": "user", "content": user_content})
        return messages
    def build_messages_encoded(self, base64_image: str, task_description: str, k: int,
                                conversation_history=None, depth_frame=None,
                                center_depth=None) -> list:
        """Same as build_messages but reuses an already-encoded RGB image.

        This avoids encoding the same frame twice when the caller has already
        encoded it (e.g. in estimate_target_depth).
        """
        depth_base64 = self.encode_image(depth_frame) if depth_frame is not None else None
        depth_info_str = self._format_depth_info(center_depth)

        sys_prompt = self.build_system_prompt(task_description, k, depth_info_str)
        messages = [{"role": "system", "content": sys_prompt}]

        if conversation_history:
            messages.extend(conversation_history)

        user_content = self.build_user_content(base64_image, depth_base64,
                                                task_description, k, depth_info_str)
        messages.append({"role": "user", "content": user_content})
        return messages
