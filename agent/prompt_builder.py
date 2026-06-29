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
            task_description=task_description,
            k=k,
            depth_info=depth_info_str,
        )
        return (
            prompt
            + "\n输出限制：只输出一个合法 JSON 对象，不要 Markdown，不要额外解释；"
              "scene_analysis、reasoning_summary 和 reason 必须简短。"
              "如果程序给出的目标深度明显大于单次最大前进距离，优先在同一候选轨迹中使用多个连续 forward 分段接近，"
              f"而不是只输出一个 forward；但每条轨迹最多 {self.config.max_trajectory_length} 个动作，"
              "每个 forward 仍必须满足单次最大前进距离限制。\n"
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
