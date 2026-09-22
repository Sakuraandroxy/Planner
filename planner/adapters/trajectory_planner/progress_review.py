"""Visual progress review, separate from the waypoint generation protocol."""
import logging

import requests

from planner.adapters.task_parser.response_schema import parse_json_object
from planner.adapters.trajectory_planner.qwen_vl_planner import _image_url
from planner.adapters.trajectory_planner.prompts import NavigationProgressPrompt, ProgressPromptBuilder
from planner.domain.progress import ProgressDecision, ProgressStatus
from planner.errors import ProtocolError

logger = logging.getLogger(__name__)


class VisualProgressReviewer:
    def __init__(self, planner, prompt_builder: ProgressPromptBuilder | None = None):
        self.planner = planner
        self.prompt_builder = prompt_builder or NavigationProgressPrompt()

    def review(self, instruction, initial, current, round_index):
        prompt = self.prompt_builder.build(instruction, initial, current, round_index)
        images = [initial.rgb, initial.depth_image, current.rgb, current.depth_image]
        content = [{"type": "image_url", "image_url": {"url": _image_url(im, "PNG")}} for im in images]
        content.append({"type": "text", "text": prompt})
        backend = self.planner
        response = requests.post(backend.url, headers={"Authorization": f"Bearer {backend.api_key}"},
            json={"model": backend.model, "messages": [{"role": "user", "content": content}],
                  "temperature": 0, "max_tokens": 512,
                  **({"thinking": {"type": backend.thinking}} if backend.thinking else {})},
            timeout=backend.timeout_s)
        response.raise_for_status()
        raw = response.json()["choices"][0]["message"]["content"]
        logger.info("[ProgressReview] %s", raw)
        data = parse_json_object(raw)
        try:
            status = ProgressStatus(data.get("status"))
        except ValueError as exc:
            raise ProtocolError("invalid progress review status") from exc
        reason = data.get("reason")
        remaining = data.get("next_instruction", "")
        if not isinstance(reason, str) or not reason.strip():
            raise ProtocolError("invalid progress review status/evidence")
        if not isinstance(remaining, str) or (status is ProgressStatus.CONTINUE and not remaining.strip()):
            raise ProtocolError("progress review requires remaining instruction")
        return ProgressDecision(status, reason, remaining)
