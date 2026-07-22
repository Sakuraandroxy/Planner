"""VLM-backed task parser for multi-stage UAV navigation instructions."""

from __future__ import annotations

import json
import re
import time
from typing import Any, Dict, List

from agent.functions.task_parser import register_task_parser
from agent.functions.task_parser.base import BaseTaskParser, TaskStage

_ACTION_VALUE_DEFAULTS = {
    "forward": 10.0,
    "backward": 5.0,
    "left": 90.0,
    "right": 90.0,
    "up": 5.0,
    "down": 5.0,
}

TASK_PARSER_SYSTEM_PROMPT = """You are a UAV navigation task parser.
Return only one valid JSON object. Do not include markdown or explanations.

All output fields must be English, even when the user instruction is Chinese.

Split the user instruction into ordered executable stages. Use:
- mode="action" only for fixed ego-motion commands: forward/backward/left/right/up/down.
- mode="detect" for find/search/locate target without flying to it.
- mode="target" for navigation to a physical target or spatial relation such as beside/near/above/on top.

When the instruction contains "然后/then/and then/first...then", split into multiple stages.
  Example: "右转，然后飞到红色车旁" -> task_type="multi" with:
    stage 1: {"instruction": "Turn right", "mode": "action", "action": "right", "value": 90, "unit": "degree"}
    stage 2: {"instruction": "Fly to the red car", "mode": "target", "target": "red car", "relation": "beside"}
  Example: "先左转45度再飞到房子附近" -> task_type="multi" with:
    stage 1: {"instruction": "Turn left 45 degrees", "mode": "action", "action": "left", "value": 45, "unit": "degree"}
    stage 2: {"instruction": "Fly to the house", "mode": "target", "target": "house", "relation": "near"}

The "instruction" field is a simple English command. Examples:
  "飞到红色车旁" -> {"instruction": "Fly to the red car", "target": "red car", "relation": "beside"}
  "飞到房子附近" -> {"instruction": "Fly to the house", "target": "house", "relation": "near"}
  "飞到房子上方" -> {"instruction": "Fly above the house", "target": "house", "relation": "above"}

JSON schema:
{
  "task_type": "single|multi",
  "stages": [
    {
      "index": 1,
      "instruction": "English task description",
      "mode": "target|detect|action",
      "target": "English target name",
      "relation": "beside|near|above|on top|",
      "action": "forward|backward|left|right|up|down|",
      "value": null,
      "unit": "",
      "requires_target": false,
      "allow_relocalize": false,
      "completion_condition": "English completion condition"
    }
  ]
}
"""

TASK_PARSER_USER_PROMPT = """User task:
{task}

Return only the task parsing JSON."""


@register_task_parser("vlm_task_parser")
@register_task_parser("vlm_parser")
class TaskParser(BaseTaskParser):
    """Parse a natural-language task into TaskStage objects."""

    def __init__(self, vlm=None, max_tokens: int = 2048):
        self.vlm = vlm
        self.max_tokens = max_tokens
        self.last_timing: Dict[str, Any] = {}

    def parse(self, instruction: str) -> List[TaskStage]:
        started = time.perf_counter()
        messages = [
            {"role": "system", "content": TASK_PARSER_SYSTEM_PROMPT},
            {"role": "user", "content": TASK_PARSER_USER_PROMPT.format(task=(instruction or "").strip())},
        ]

        if self.vlm is not None:
            response_text, _reasoning = self.vlm.call(messages, max_tokens=self.max_tokens)
            elapsed = time.perf_counter() - started
            self.last_timing = dict(getattr(self.vlm, "last_call_info", {}) or {})
            self.last_timing["name"] = "task_parse"
            self.last_timing["elapsed"] = self.last_timing.get("elapsed", elapsed)
        else:
            response_text = self._call_api_directly(messages)
            elapsed = time.perf_counter() - started
            self.last_timing = {"name": "task_parse", "elapsed": elapsed}

        return parse_task_parser_to_stages(response_text or "", original_instruction=instruction)

    def _call_api_directly(self, messages) -> str:
        from config import cfg
        from openai import OpenAI
        from agent.functions.common.config_access import first_value, function_section

        fc = function_section(cfg, "TASK_PARSER")
        ag_cfg = cfg.get("AGENT", {}) or {}
        client = OpenAI(
            base_url=first_value(fc.get("URL"), ag_cfg.get("TASK_PARSER_URL"), default=""),
            api_key=first_value(fc.get("API_KEY"), ag_cfg.get("TASK_API_KEY"), default="no-key"),
        )
        resp = client.chat.completions.create(
            model=first_value(fc.get("MODEL_NAME"), ag_cfg.get("TASK_PARSER_MODEL"), default="Qwen3-VL-4B-Instruct"),
            messages=messages,
            max_tokens=int(first_value(fc.get("MAX_TOKENS"), ag_cfg.get("TASK_PARSER_MAX_TOKENS"), default=1024)),
            temperature=0.0,
            extra_body={"thinking": {"type": "disabled"}},
        )
        msg = resp.choices[0].message
        raw = msg.content or ""
        if not raw.strip():
            raw = getattr(msg, "reasoning_content", None) or ""
        return raw


def parse_task_parser_to_stages(response_text: str, original_instruction: str = "") -> List[TaskStage]:
    data = _load_json_object(response_text)
    raw_stages = data.get("stages") if isinstance(data, dict) else None
    if not isinstance(raw_stages, list):
        raise ValueError("task parser response does not contain stages list")

    stages: List[TaskStage] = []
    single_stage_original = original_instruction if len(raw_stages) == 1 else ""
    for item in raw_stages:
        if not isinstance(item, dict):
            continue
        instruction = str(item.get("instruction", "") or "").strip()
        mode = str(item.get("mode", "") or "").strip().lower()
        if not instruction or not mode:
            continue

        action = _normalize_action(str(item.get("action", "") or "").strip().lower())
        target = str(item.get("target", "") or "").strip()
        relation = str(item.get("relation", "") or "").strip().lower()
        mode, target, relation = _coerce_spatial_action_stage(
            mode=mode,
            action=action,
            target=target,
            relation=relation,
            instruction=instruction,
            original_instruction=single_stage_original,
        )

        value = item.get("value")
        if mode == "action":
            if action not in _ACTION_VALUE_DEFAULTS:
                action = _infer_action_from_instruction(instruction)
            value = _coerce_action_value(value, action)
            target = ""
            relation = ""
        else:
            action = ""
            value = None
            if not target:
                target = instruction

        stages.append(
            TaskStage(
                index=len(stages),
                instruction=_repair_instruction(instruction, target, relation, mode),
                mode=mode,
                target=target if mode in {"target", "detect"} else "",
                relation=relation if mode == "target" else "",
                action=action if mode == "action" else "",
                value=value if mode == "action" else None,
                unit=str(item.get("unit", "") or "").strip(),
                requires_target=bool(item.get("requires_target", mode in {"target", "detect"})),
                allow_relocalize=bool(item.get("allow_relocalize", mode in {"target", "detect"})),
                completion_condition=str(item.get("completion_condition", "") or "").strip(),
            )
        )

    if not stages:
        raise ValueError("task parser returned no valid stages")
    return stages


def parse_task_parser_response(response_text: str, original_instruction: str = "") -> List[TaskStage]:
    return parse_task_parser_to_stages(response_text, original_instruction=original_instruction)


def _load_json_object(text: str) -> Dict[str, Any]:
    cleaned = (text or "").strip()
    if cleaned.startswith("```"):
        cleaned = re.sub(r"^```(?:json)?\s*", "", cleaned)
        cleaned = re.sub(r"\s*```$", "", cleaned)
    try:
        return json.loads(cleaned)
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", cleaned, flags=re.S)
        if not match:
            raise
        return json.loads(match.group(0))


def _normalize_action(action: str) -> str:
    aliases = {
        "back": "backward",
        "move_forward": "forward",
        "move_backward": "backward",
        "turn_left": "left",
        "turn_right": "right",
        "ascend": "up",
        "descend": "down",
    }
    return aliases.get(action, action)


def _infer_action_from_instruction(instruction: str) -> str:
    text = (instruction or "").lower()
    for name in ("forward", "backward", "left", "right", "up", "down"):
        if name in text:
            return name
    if "turn left" in text:
        return "left"
    if "turn right" in text:
        return "right"
    if "ascend" in text:
        return "up"
    if "descend" in text:
        return "down"
    return ""


def _coerce_action_value(value, action: str) -> float | None:
    if action not in _ACTION_VALUE_DEFAULTS:
        return None
    try:
        return float(value) if value is not None else _ACTION_VALUE_DEFAULTS[action]
    except (TypeError, ValueError):
        return _ACTION_VALUE_DEFAULTS[action]


def _coerce_spatial_action_stage(
    *,
    mode: str,
    action: str,
    target: str,
    relation: str,
    instruction: str,
    original_instruction: str,
):
    source = f"{original_instruction} {instruction}".lower()
    if _has_above_relation(source) and target:
        return "target", target, "above"
    if mode not in {"action", "detect", "target"}:
        mode = "target"
    if mode == "action" and action not in _ACTION_VALUE_DEFAULTS:
        mode = "target"
    return mode, target, relation


def _has_above_relation(text: str) -> bool:
    lower = (text or "").lower()
    return any(token in lower for token in ("above", "over", "on top", "top of", "上方", "上面", "顶部"))


def _repair_instruction(instruction: str, target: str, relation: str, mode: str) -> str:
    if mode == "target" and target and relation in {"above", "over", "on top", "on top of"}:
        low = instruction.lower()
        if not any(token in low for token in ("above", "over", "on top")):
            article = "" if target.lower().startswith(("the ", "a ", "an ")) else "the "
            return f"Fly above {article}{target}"
    return instruction
