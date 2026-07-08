"""VLM 检测器 —— 调用 VLM API 输出目标 bbox，替代 GroundingDINO。

与旧 Planner 项目的 estimate_target_depth() 使用相同的 prompt 格式。
"""

import base64
import io
import json
import re
from typing import Optional

import numpy as np
from PIL import Image
from openai import OpenAI

from agent.detector.base import BaseDetector, DetectionResult
from agent.detector import register_detector

# ─── 复用旧项目的 bbox prompt ───
TARGET_BBOX_SYSTEM_PROMPT = """你是目标检测器。根据图片输出真实目标框JSON。
只输出一行JSON对象；禁止思考过程、解释、Markdown、动作规划、占位符。"""

TARGET_BBOX_PROMPT = """请观察这张RGB图，定位导航任务目标。

任务目标：{task_description}
当前图片尺寸：宽 {image_width} 像素，高 {image_height} 像素。

必须直接返回一行JSON对象，只包含这些字段：visible、bbox_norm、confidence、target。

规则：
1. visible 是 true 或 false。
2. bbox_norm 是4个数字组成的数组，范围 0.0 到 1.0，顺序是左、上、右、下。
3. confidence 是0.0到1.0之间的数字。
4. target 是目标名字符串，例如 car。
5. 如果画面中有多个同类目标，必须根据任务目标直接选择最终要导航的那个目标框，不要输出候选框列表。
6. 如果任务包含"较远/远处/远的/far"，bbox_norm 直接选择视觉上更远的目标，不要选择最近目标。
7. 如果目标不可见，输出 visible=false，bbox_norm=null，confidence=0.0，target=""。
8. 禁止输出 candidates 字段，禁止复述本提示词，禁止输出 x1、y1、x2、y2，禁止输出解释、Markdown、分析过程或动作规划。
9. 如果无法精确计算坐标，也必须基于视觉估计给出数字 bbox_norm，不要解释原因。"""


@register_detector("vlm_detector")
class VLMDetector(BaseDetector):
    """基于 VLM API 的目标检测器，输出归一化 bbox + 深度。"""

    def __init__(self):
        from config import cfg
        ag = cfg["AGENT"]
        # 优先 DETECTOR_URL，回退到 PLANNER_URL
        base_url = ag.get("DETECTOR_URL") or ag["PLANNER_URL"]
        api_key = ag.get("DETECTOR_API_KEY") or ag.get("PLANNER_API_KEY", "no-key")
        model = ag.get("DETECTOR_MODEL") or ag.get("PLANNER_MODEL", "")
        self.client = OpenAI(
            base_url=base_url,
            api_key=api_key,
        )
        self.model = model
        self.max_tokens = int(ag.get("DETECTOR_MAX_TOKENS", 512))

    def detect(self, image: Image.Image, caption: str,
               depth_meters=None, camera_name: str = "front") -> DetectionResult:
        """调用 VLM API 获取目标 bbox，然后从 depth_meters 计算深度。"""
        if image.mode == "RGBA":
            image = image.convert("RGB")
        w, h = image.size

        # 编码图片（优先复用 ImageEncoder 缓存）
        from agent.common.image_encoder import get_cached_down_b64, get_cached_front_b64
        cached = get_cached_front_b64() if camera_name == "front" else get_cached_down_b64()
        if cached:
            b64 = cached
        else:
            buf = io.BytesIO()
            image.save(buf, format="JPEG", quality=85)
            b64 = base64.b64encode(buf.getvalue()).decode()

        # 构建 prompt（复用旧项目格式）
        messages = [
            {"role": "system", "content": TARGET_BBOX_SYSTEM_PROMPT},
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{b64}"},
                    },
                    {
                        "type": "text",
                        "text": TARGET_BBOX_PROMPT.format(
                            task_description=caption,
                            image_width=w,
                            image_height=h,
                        ),
                    },
                ],
            },
        ]

        try:
            resp = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=self.max_tokens,
                temperature=0.0,
                extra_body={"thinking": {"type": "disabled"}},
            )
            raw = resp.choices[0].message.content or ""
            # Debug: if response is empty, log details
            if not raw.strip():
                finish = getattr(resp.choices[0], 'finish_reason', 'unknown')
                print(f"  [VLMDetector] empty response! finish_reason={finish}, "
                      f"model={self.model}, image_size={w}x{h}, b64_len={len(b64)}")
                # Save debug frame
                try:
                    image.save("debug_detector_frame.jpg", format="JPEG", quality=85)
                    print(f"  [VLMDetector] saved debug frame to debug_detector_frame.jpg")
                except Exception:
                    pass
        except Exception as exc:
            print(f"  [VLMDetector] API error: {exc}")
            return DetectionResult(visible=False, camera=camera_name)

        result = self._parse_response(raw, image, depth_meters, caption)
        result.camera = camera_name if result.visible else camera_name
        return result

    def _parse_response(self, raw: str, image: Image.Image,
                        depth_meters, caption: str = "") -> DetectionResult:
        """解析 VLM 返回的 bbox JSON。"""
        text = re.sub(r"```json\s*", "", raw)
        text = re.sub(r"```\s*", "", text)
        match = re.search(r"\{.*\}", text, re.DOTALL)
        if not match:
            print(f"  [VLMDetector] no JSON in response: {raw[:200]}")
            return DetectionResult(visible=False, camera="none")

        try:
            data = json.loads(match.group(), strict=False)
        except json.JSONDecodeError:
            data = _relaxed_json_parse(match.group())
            if data is None:
                print(f"  [VLMDetector] parse failed: {match.group()[:200]}")
                return DetectionResult(visible=False, camera="none")

        visible = bool(data.get("visible", False))
        if not visible:
            return DetectionResult(visible=False, camera="none")

        bbox_norm = data.get("bbox_norm") or data.get("bbox")
        if not bbox_norm or len(bbox_norm) != 4:
            return DetectionResult(visible=False, camera="none")

        w, h = image.size
        try:
            values = [float(v) for v in bbox_norm]
            if all(0.0 <= v <= 1.0 for v in values):
                x1, y1, x2, y2 = values
                bbox = [
                    int(round(x1 * w)), int(round(y1 * h)),
                    int(round(x2 * w)), int(round(y2 * h)),
                ]
            else:
                # 像素坐标
                bbox = [int(round(v)) for v in values]
        except (ValueError, TypeError):
            return DetectionResult(visible=False, camera="none")

        bbox[0] = max(0, min(bbox[0], w - 1))
        bbox[1] = max(0, min(bbox[1], h - 1))
        bbox[2] = max(bbox[0] + 1, min(bbox[2], w))
        bbox[3] = max(bbox[1] + 1, min(bbox[3], h))

        score = float(data.get("confidence", 0.0) or 0.0)
        label = str(data.get("target", caption) or caption)

        # 深度计算
        depth_median = None
        depth_bbox_list = None
        if depth_meters is not None:
            import numpy as np
            dh, dw = depth_meters.shape
            sw = dw / w
            sh = dh / h
            db = [
                int(bbox[0] * sw), int(bbox[1] * sh),
                int(bbox[2] * sw), int(bbox[3] * sh),
            ]
            depth_bbox_list = db
            cx = (db[0] + db[2]) // 2
            cy = (db[1] + db[3]) // 2
            if 0 <= cy < dh and 0 <= cx < dw:
                depth_median = float(depth_meters[cy, cx])

        print(f"  [VLMDetector] ✅ {label} bbox={bbox} score={score:.2f}"
              f" depth={depth_median:.1f}m" if depth_median else
              f"  [VLMDetector] ✅ {label} bbox={bbox} score={score:.2f}")

        return DetectionResult(
            visible=True, bbox=bbox, score=score,
            label=label, depth_median=depth_median,
            depth_bbox=depth_bbox_list,
            camera="front",
        )


def _relaxed_json_parse(raw: str) -> Optional[dict]:
    """容错 JSON 解析。"""
    text = raw.strip()
    text = text.replace("None", "null").replace("True", "true").replace("False", "false")
    text = text.replace("'", '"')
    text = re.sub(r",\s*([}\]])", r"\1", text)
    text = re.sub(r"([{,])\s*([A-Za-z_][\w]*)\s*:", r'\1"\2":', text)
    try:
        return json.loads(text, strict=False)
    except json.JSONDecodeError:
        return None
